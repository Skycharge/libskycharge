/*
 * Copyright 2021 Skycharge GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met: 1. Redistributions of source code must retain the above
 * copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <endian.h>
#include <limits.h>
#include <string.h>
#include <uuid/uuid.h>

#include <czmq.h>
#include <mongoc.h>

#include "skybroker-cmd.h"
#include "daemon.h"
#include "version.h"
#include "skyproto.h"
#include "libskycharge.h"
#include "libskycharge-pri.h"

struct server_peer {
	zmsg_t *idents_msg;
	zframe_t *data_frame;
	uuid_t devuuid;
	const void *rsp_data;
	struct sky_dev_desc *devdesc;
	size_t rsp_len;
	size_t num_devs;
	size_t info_off;
	bool dyn_info;
	union {
		unsigned long long expires_at;
		struct server_peer *next;
	};
};

struct db {
	mongoc_uri_t         *uri;
	mongoc_client_pool_t *pool;
};

struct db_client {
	mongoc_client_pool_t *pool;
	mongoc_client_t      *client;
	mongoc_database_t    *db;
	mongoc_collection_t  *coll_chargers;
	mongoc_collection_t  *coll_user_uuids;
	mongoc_collection_t  *coll_charging_sessions;

};

static pthread_mutex_t hash_lock = PTHREAD_MUTEX_INITIALIZER;

static int db_init(struct cli *cli, struct db *db)
{
	bson_error_t error;

	/*
	 * Init mongodb
	 */
	mongoc_init ();
	db->uri = mongoc_uri_new_with_error(cli->dburi, &error);
	if (!db->uri) {
		sky_err("failed to parse URI: %s\n"
			"error message:       %s\n",
			cli->dburi,
			error.message);
		return -1;
	}

	/*
	 * Create a new pool and client instance
	 */
	db->pool = mongoc_client_pool_new(db->uri);
	if (!db->pool) {
		sky_err("mongoc_client_pool_new(): failed\n");
		mongoc_uri_destroy(db->uri);
		return -1;
	}
	mongoc_client_pool_set_error_api(db->pool, 2);
	mongoc_client_pool_set_appname(db->pool, "skybroker");

	return 0;
}

static void db_deinit(struct db *db)
{
	mongoc_client_pool_destroy(db->pool);
	mongoc_uri_destroy(db->uri);
	mongoc_cleanup();
}

static int
db_client_connect(struct db *db, struct db_client *db_client)
{
	db_client->client = mongoc_client_pool_pop(db->pool);
	if (!db_client->client) {
		sky_err("mongoc_client_pool_pop(): failed\n");
		return -1;
	}
	db_client->db = mongoc_client_get_database(db_client->client,
						   "skycharge_db");
	if (!db_client->db) {
		sky_err("mongoc_client_get_database(): failed\n");
		goto err_push_pool;
	}
	db_client->coll_chargers =
		mongoc_database_get_collection(db_client->db, "chargers");
	if (!db_client->coll_chargers) {
		sky_err("mongoc_database_get_collection(): failed\n");
		goto err_database_destroy;
	}
	db_client->coll_user_uuids =
		mongoc_database_get_collection(db_client->db, "userUUIDS");
	if (!db_client->coll_user_uuids) {
		sky_err("mongoc_database_get_collection(): failed\n");
		goto err_coll_chargers_destroy;
	}
	db_client->coll_charging_sessions =
		mongoc_database_get_collection(db_client->db, "chargingSessions");
	if (!db_client->coll_charging_sessions) {
		sky_err("mongoc_database_get_collection(): failed\n");
		goto err_coll_user_uuids_destroy;
	}

	db_client->pool = db->pool;

	return 0;

err_coll_user_uuids_destroy:
	mongoc_collection_destroy(db_client->coll_charging_sessions);
err_coll_chargers_destroy:
	mongoc_collection_destroy(db_client->coll_chargers);
err_database_destroy:
	mongoc_database_destroy(db_client->db);
err_push_pool:
	mongoc_client_pool_push(db->pool, db_client->client);

	return -1;
}

static void db_client_disconnect(struct db_client *db_client)
{
	mongoc_collection_destroy(db_client->coll_charging_sessions);
	mongoc_collection_destroy(db_client->coll_chargers);
	mongoc_collection_destroy(db_client->coll_user_uuids);
	mongoc_database_destroy(db_client->db);
	mongoc_client_pool_push(db_client->pool, db_client->client);
}

static bool
db_lookup_and_update_device_by_devuuid(struct db_client *db_client,
				       const struct sky_dev_desc *devdesc,
				       bool only_ts)
{
	mongoc_find_and_modify_opts_t *opts;
	bson_iter_t iter, citer;
	bson_t *query;
	bson_t *update;
	bson_t reply;

	char devuuid_str[37];
	bool success;

	uuid_unparse(devdesc->dev_uuid, devuuid_str);
	query = BCON_NEW("deviceId", BCON_UTF8(devuuid_str));

	if (!only_ts) {
		char plc_proto_ver_str[16];
		char proto_ver_str[16];
		char fw_ver_str[16];
		char hw_ver_str[16];
		char uid_str[32];

		snprintf(proto_ver_str, sizeof(plc_proto_ver_str), "%u.%u",
			 version_minor(devdesc->proto_version),
			 version_revis(devdesc->proto_version));
		snprintf(plc_proto_ver_str, sizeof(plc_proto_ver_str), "%u.%u.%u",
			 version_major(devdesc->hw_info.plc_proto_version),
			 version_minor(devdesc->hw_info.plc_proto_version),
			 version_revis(devdesc->hw_info.plc_proto_version));
		snprintf(fw_ver_str, sizeof(fw_ver_str), "%u.%u.%u",
			 version_major(devdesc->hw_info.fw_version),
			 version_minor(devdesc->hw_info.fw_version),
			 version_revis(devdesc->hw_info.fw_version));
		snprintf(hw_ver_str, sizeof(hw_ver_str), "%u.%u.%u",
			 version_major(devdesc->hw_info.hw_version),
			 version_minor(devdesc->hw_info.hw_version),
			 version_revis(devdesc->hw_info.hw_version));
		snprintf(uid_str, sizeof(uid_str), "%x%x%x",
			 devdesc->hw_info.uid.part1,
			 devdesc->hw_info.uid.part2,
			 devdesc->hw_info.uid.part3);

		update = BCON_NEW("$currentDate", "{",
				      "loggedinAt", BCON_BOOL(true),
				      "updatedAt",  BCON_BOOL(true),
				  "}",
				  "$set", "{",
				      "type", BCON_UTF8(sky_devtype_to_str(devdesc->dev_type)),
				      "name", BCON_UTF8(devdesc->dev_name),
				      "firmwareVersion", BCON_UTF8(fw_ver_str),
				      "hardwareVersion", BCON_UTF8(hw_ver_str),
				      "protocolVersion", BCON_UTF8(proto_ver_str),
				      "plcProtocolVersion", BCON_UTF8(plc_proto_ver_str),
				      "hw_uid", BCON_UTF8(uid_str),
				  "}");
	} else {
		update = BCON_NEW("$currentDate", "{",
				      "updatedAt",  BCON_BOOL(true),
				  "}");
	}

	opts = mongoc_find_and_modify_opts_new();
	mongoc_find_and_modify_opts_set_update(opts, update);

	success = mongoc_collection_find_and_modify_with_opts(
		db_client->coll_chargers, query, opts, &reply, NULL);

	if (success)
		success = bson_iter_init_find(&iter, &reply, "lastErrorObject") &&
			BSON_ITER_HOLDS_DOCUMENT(&iter) &&
			bson_iter_recurse(&iter, &citer) &&
			bson_iter_find (&citer, "updatedExisting") &&
			BSON_ITER_HOLDS_BOOL(&citer) &&
			bson_iter_bool(&citer);

	bson_destroy(update);
	bson_destroy(query);
	bson_destroy(&reply);
	mongoc_find_and_modify_opts_destroy(opts);

	return success;
}

static bool
db_lookup_online_devices_by_usruuid(struct db_client *db_client, uuid_t usruuid,
				    uuid_t **devuuids, size_t *nr_devuuids)
{
	mongoc_cursor_t *cursor;
	bson_t *pipeline;
	char usruuid_str[37];
	const bson_t *doc;

	const char *chargers_name;
	unsigned found;
	uuid_t *uuids;

	uuid_unparse(usruuid, usruuid_str);

	chargers_name = mongoc_collection_get_name(db_client->coll_chargers);
	pipeline = BCON_NEW(
		"pipeline", "[",

		/* Lookup user UUID with valid userId */
		  "{",
		      "$match", "{",
		          "userUUID", BCON_UTF8(usruuid_str),
		          "userId", "{", "$exists", BCON_BOOL(true), "}",
		      "}",
		  "}",

		/* Lookup online chargers belong to that user */
		  "{",
		      "$lookup", "{",
		          "from", BCON_UTF8(chargers_name),
		          "as", "online_charger",
		          "let", "{", "userId", "$userId",
		                      "updatedAt", "$updatedAt",
			  "}",
		          "pipeline", "[",
		              "{",
		                  "$match", "{",
		                      "$expr", "{",
		                          "$and", "[",
		                               "{", "$eq", "[", "$userId", "$$userId", "]", "}",
		                               "{", "$gte", "[", "$updatedAt", "{", "$subtract", "[", "$$NOW", BCON_INT32(SKY_HEARTBEAT_IVL_MS), "]", "}", "]", "}",
					  "]",
				      "}",
				  "}",
			      "}",
			  "]",
		      "}",
		  "}",

		/* Unwind array of online chargers */
		  "{", "$unwind", "$online_charger", "}",

		/* Get flatten deviceId fields only of online chargers */
		  "{",
		      "$project", "{", "_id", "$online_charger._id",
				       "deviceId", "$online_charger.deviceId", "}",
		      "}",
		"]");

	cursor = mongoc_collection_aggregate(db_client->coll_user_uuids,
					     MONGOC_QUERY_NONE, pipeline,
					     NULL, NULL);
	uuids = NULL;
	found = 0;
	while (mongoc_cursor_next(cursor, &doc)) {
		uuid_t uuid, *ptr;
		bson_iter_t iter;
		bool parsed;

		parsed = bson_iter_init_find(&iter, doc, "deviceId") &&
			 BSON_ITER_HOLDS_UTF8(&iter) &&
			 uuid_parse(bson_iter_utf8(&iter, NULL), uuid) == 0;
		found += !!parsed;

		if (parsed) {
			ptr = reallocarray(uuids, found, 37);
			if (!ptr) {
				sky_err("No memory!\n");
				free(uuids);
				uuids = NULL;
				found = 0;
				break;
			}
			uuids = ptr;
			memcpy(&uuids[found - 1], uuid, sizeof(uuid));
		}
	}

	mongoc_cursor_destroy(cursor);
	bson_destroy(pipeline);

	*nr_devuuids = found;
	*devuuids = uuids;

	return !!found;
}

static bool
db_lookup_device_by_usruuid_devuuid(struct db_client *db_client,
				    uuid_t usruuid, uuid_t devuuid)
{
	mongoc_cursor_t *cursor;
	bson_t *pipeline;
	char usruuid_str[37];
	char devuuid_str[37];
	const bson_t *doc;

	const char *chargers_name;
	bool found;

	uuid_unparse(usruuid, usruuid_str);
	uuid_unparse(devuuid, devuuid_str);

	chargers_name = mongoc_collection_get_name(db_client->coll_chargers);
	pipeline = BCON_NEW(
		"pipeline", "[",

		/* Lookup user UUID with valid userId */
		  "{",
		      "$match", "{",
		          "userUUID", BCON_UTF8(usruuid_str),
		          "userId", "{", "$exists", BCON_BOOL(true), "}",
		      "}",
		  "}",

		/* Lookup charger belong to that user */
		  "{",
		      "$lookup", "{",
		          "from", BCON_UTF8(chargers_name),
		          "as", "charger",
		          "let", "{",
		              "userId", "$userId",
		              "deviceId", "$deviceId",
		              "updatedAt", "$updatedAt",
			  "}",
		          "pipeline", "[",
		              "{",
		                  "$match", "{",
		                      "$expr", "{",
		                          "$and", "[",
		                               "{", "$eq", "[", "$userId", "$$userId", "]", "}",
		                               "{", "$eq", "[", "$deviceId", BCON_UTF8(devuuid_str), "]", "}",
					  "]",
				      "}",
				  "}",
			      "}",
			  "]",
		      "}",
		  "}",

		/* Unwind array of chargers */
		  "{", "$unwind", "$charger", "}",

		/* Get flatten deviceId fields only of chargers */
		  "{",
		      "$project", "{", "_id", "$charger._id",
				       "deviceId", "$charger.deviceId", "}",
		      "}",
		"]");

	cursor = mongoc_collection_aggregate(db_client->coll_user_uuids,
					     MONGOC_QUERY_NONE, pipeline,
					     NULL, NULL);
	found = mongoc_cursor_next(cursor, &doc);
	mongoc_cursor_destroy(cursor);
	bson_destroy(pipeline);

	return found;
}

static bool
db_lookup_user_by_usruuid(struct db_client *db_client, uuid_t usruuid)
{
	mongoc_cursor_t *cursor;
	bson_t *pipeline;
	char usruuid_str[37];
	const bson_t *doc;

	bool found;

	uuid_unparse(usruuid, usruuid_str);
	pipeline = BCON_NEW(
		"pipeline", "[",

		/* Lookup user UUID with valid userId */
		  "{",
		      "$match", "{",
		          "userUUID", BCON_UTF8(usruuid_str),
		          "userId", "{", "$exists", BCON_BOOL(true), "}",
		      "}",
		  "}",

		/* Get few fields only */
		  "{",
		      "$project", "{", "_id", BCON_BOOL(true),
		                       "userUUID", BCON_BOOL(true),
		                       "userId", BCON_BOOL(true),
		      "}",
		  "}",
		"]");

	cursor = mongoc_collection_aggregate(db_client->coll_user_uuids,
					     MONGOC_QUERY_NONE, pipeline,
					     NULL, NULL);
	found = mongoc_cursor_next(cursor, &doc);
	mongoc_cursor_destroy(cursor);
	bson_destroy(pipeline);

	return found;
}

static void db_update_charging_session(struct db_client *db_client, uuid_t devuuid,
				       const struct sky_charging_state *state)
{
	bson_error_t error;
	bson_t *query;
	bson_t *update;
	bson_t *opts;

	char devuuid_str[37];
	bool success;

	uuid_unparse(devuuid, devuuid_str);

	query = BCON_NEW("deviceId", BCON_UTF8(devuuid_str),
			 "chargingSessionId", BCON_INT32(state->charging_session_id));

	/* Pipeline update */
        update = BCON_NEW(
		/*
		 * If document exists don't modify 'createdAt' and
		 * 'startTime' fields.  If document does not exist set
		 * fields mentioned above and set 'values' array to
		 * initial '[]' value.
		 */

		"0", "{",
                        "$set", "{",
                            "createdAt", "{",
                                "$ifNull", "[", "$createdAt", "$$NOW", "]",
                            "}",
                            "startTime", "{",
                                "$ifNull", "[",
                                    "$startTime", "{",
                                        "$subtract", "[", "$$NOW", BCON_INT32(state->charging_secs), "]",
                                    "}",
                                "]",
                            "}",
                            "updatedAt", "$$NOW",
                            "totalAhCharged", BCON_DOUBLE(state->charge_mAh / 1000.0),
                            "totalWhCharged", BCON_DOUBLE(state->energy_mWh / 1000.0),
			    "values", "{",
                                "$ifNull", "[",
                                    "$values", "[", "]",
                                "]",
                            "}",
                        "}",
		"}",

		/*
		 * Second step in update pipeline: document is
		 * prepared, append the measurements from the charging
		 * state to the 'values' array.
		 */
		"1", "{",
                        "$set", "{",
                            "values", "{",
                                "$concatArrays", "[", "$values", "[", "{",
		                    "timestamp", "$$NOW",
		                    "state", BCON_UTF8(sky_devstate_to_str(SKY_MUX_HW2,
									   state->dev_hw_state)),
		                    "reason", BCON_UTF8(sky_devreason_to_str(state->dev_hw_reason)),
                                    "current", BCON_DOUBLE(state->current_mA / 1000.0),
                                    "voltage", BCON_DOUBLE(state->voltage_mV / 1000.0),
		                    "power", BCON_DOUBLE((double)state->voltage_mV * state->current_mA  / 1000000.0),
                                    "soc", BCON_INT32(state->state_of_charge),
                                    "untilFullSecs", BCON_INT32(state->until_full_secs),
                                    "energy", BCON_DOUBLE(state->energy_mWh / 1000.0),
                                    "charge", BCON_DOUBLE(state->charge_mAh / 1000.0),
                                    "muxHumidityPerc", BCON_INT32(state->mux_humidity_perc),
                                    "muxTemperature", BCON_INT32(state->mux_temperature_C),
                                    "sinkTemperature", BCON_INT32(state->sink_temperature_C),
                                    "linkQualityFactor", BCON_INT32(state->link_quality_factor),
                                    "tx", "{",
                                        "bytes", BCON_INT32(state->tx.bytes),
                                        "packets", BCON_INT32(state->tx.packets),
                                        "errBytes", BCON_INT32(state->tx.err_bytes),
                                        "errPackets", BCON_INT32(state->tx.err_packets),
                                    "}",
                                    "rx", "{",
                                        "bytes", BCON_INT32(state->rx.bytes),
                                        "packets", BCON_INT32(state->rx.packets),
                                        "errBytes", BCON_INT32(state->rx.err_bytes),
                                        "errPackets", BCON_INT32(state->rx.err_packets),
                                    "}",
		                "}", "]", "]",
		            "}",
		        "}",
		    "}"
		);

	opts = BCON_NEW("upsert", BCON_BOOL(true));

	success = mongoc_collection_update_one(
		db_client->coll_charging_sessions, query, update,
		opts, NULL, &error);

	if (!success)
		sky_err("mongoc_collection_update_one() error: \"%s\"\n",
			error.message);

	bson_destroy(update);
	bson_destroy(query);
	bson_destroy(opts);
}

static void db_update_charger_device(struct db_client *db_client, uuid_t devuuid,
				     const struct sky_charging_state *state)
{
	bson_error_t error;
	bson_t *query;
	bson_t *update;

	char devuuid_str[37];
	bool success;

	uuid_unparse(devuuid, devuuid_str);

	query = BCON_NEW("deviceId", BCON_UTF8(devuuid_str));
	/* Pipeline update */
        update = BCON_NEW(
		"0", "{",
		    "$set", "{",
		        "updatedAt", "$$NOW",
		        "state", BCON_UTF8(sky_devstate_to_str(SKY_MUX_HW2,
							       state->dev_hw_state)),
		        "reason", BCON_UTF8(sky_devreason_to_str(state->dev_hw_reason)),
		        "muxHumidityPerc", BCON_INT32(state->mux_humidity_perc),
		        "muxTemperature", BCON_INT32(state->mux_temperature_C),
		    "}",
		"}");

	success = mongoc_collection_update_one(
		db_client->coll_chargers, query, update,
		NULL, NULL, &error);

	if (!success)
		sky_err("mongoc_collection_update_one() error: \"%s\"\n",
			error.message);

	bson_destroy(update);
	bson_destroy(query);
}

/**
 * sky_find_data_frame() - finds first data frame, skips all
 *                         IDENT and 0 frames.
 */
static inline zframe_t *sky_find_data_frame(zmsg_t *msg)
{
	zframe_t *data, *next;

	data = zmsg_first(msg);
	while (data) {
		next = zmsg_next(msg);
		if (!next || zframe_size(next) != 0) {
			zframe_t *pos;
			/*
			 * Replay from first till data, in order to
			 * set internal cursor correctly. Stupid zmsg.
			 */
			for (pos = zmsg_first(msg); pos != data;
			     pos = zmsg_next(msg))
				;
			/* Found data frame */
			break;
		}
		data = zmsg_next(msg);
	}

	return data;
}

static zframe_t *sky_zmsg_prev(zmsg_t *msg, zframe_t *frame)
{
	zframe_t *prev = NULL, *next;

	next = zmsg_first(msg);
	while (next) {
		if (next == frame)
			return prev;
		prev = next;
		next = zmsg_next(msg);
	}

	return NULL;
}

static void server_peer_free(struct server_peer *peer)
{
	zmsg_destroy(&peer->idents_msg);
	zframe_destroy(&peer->data_frame);
	sky_devsfree(peer->devdesc);
	free(peer);
}

static bool is_uuid_frame_valid(zframe_t *uuid)
{
	return (uuid && zframe_size(uuid) == 16);
}

static void peer_destructor_fn(void **item)
{
	struct server_peer *peer = *item;

	server_peer_free(peer);
}

static int uuid_comparator_fn(const void *uuid1, const void *uuid2)
{
	return memcmp(uuid1, uuid2, sizeof(uuid_t));
}

/*
 * This is the "One-at-a-Time" algorithm by Bob Jenkins
 * from requirements by Colin Plumb.
 * (http://burtleburtle.net/bob/hash/doobs.html)
 */
static inline unsigned int jhash(const char *key, size_t len)
{
	unsigned int hash, i;

	for (hash = 0, i = 0; i < len; ++i) {
		hash += key[i];
		hash += (hash << 10);
		hash ^= (hash >> 6);
	}
	hash += (hash << 3);
	hash ^= (hash >> 11);
	hash += (hash << 15);

	return hash;
}

static size_t uuid_hasher_fn(const void *uuid)
{
	return jhash(uuid, sizeof(uuid_t));
}

static int sky_reap_dead_servers(zhashx_t *hash)
{
	struct server_peer *head = NULL, *next;
	struct server_peer *peer;
	unsigned long long ms, nearest = ~0ull;

	ms = zclock_time();

	pthread_mutex_lock(&hash_lock);
	for (peer = zhashx_first(hash); peer; peer = zhashx_next(hash)) {
		if (ms >= peer->expires_at) {
			peer->next = head;
			head = peer;
		} else if (peer->expires_at < nearest)
			nearest = peer->expires_at;

	}

	/*
	 * Ugly crap, but we are not able to delete hash item while
	 * traversing hash itself.
	 */
	while (head) {
		next = head->next;
		/*
		 * Zmq guys do lookup on each delete, even we have item
		 * in our hands, what a wonderfull design.
		 */
		zhashx_delete(hash, head->devuuid);
		head = next;
	}
	pthread_mutex_unlock(&hash_lock);

	return nearest == ~0ull ? -1 : (nearest - ms) * ZMQ_POLL_MSEC;
}

static bool parse_devs_list_rsp(struct db_client *db_client,
				zframe_t *data, struct server_peer *peer)
{
	struct sky_dev_desc *list = NULL;
	struct sky_dev_desc *devdesc;
	struct sky_parse_devs_list_rsp_result result = {
		.list = &list
	};
	int rc;

	rc = sky_parse_devs_list_rsp(zframe_data(data), zframe_size(data),
				     NULL, &result);
	if (rc || result.error)
		return false;

	if (result.num_devs != 1) {
		/* We don't support many devices from the same host */
		sky_devsfree(list);
		return false;
	}

	/* Even this is a loop, we expect only 1 device from the same host */
	foreach_devdesc(devdesc, list) {
		bool authed;

		authed = db_lookup_and_update_device_by_devuuid(db_client,
								devdesc,
								false);
		if (!authed) {
			char devuuid_str[37];

			uuid_unparse(devdesc->dev_uuid, devuuid_str);
			sky_err("Unauthorized attempt to register device with UUID '%s'\n",
				devuuid_str);
			sky_devsfree(list);
			return false;
		}

	}

	peer->devdesc  = list;
	peer->rsp_len  = result.rsp_len;
	peer->num_devs = result.num_devs;
	peer->info_off = result.info_off;
	peer->dyn_info = result.dyn_info;

	return true;
}

static void sky_handle_server_msg(void *servers, void *clients,
				  zhashx_t *srvs_hash,
				  struct db_client *db_client)
{
	zframe_t *data, *ident;
	struct server_peer *peer;
	uuid_t devuuid;
	zmsg_t *msg;
	bool authed;
	int i, rc;

	msg = zmsg_recv(servers);
	ident = msg ? zmsg_first(msg) : NULL;
	if (!ident) {
		zmsg_destroy(&msg);
		/* WTF? */
		return;
	}
	if (!is_uuid_frame_valid(ident)) {
		/* Support uuids as idents only */
		zmsg_destroy(&msg);
		return;
	}
	memcpy(devuuid, zframe_data(ident), sizeof(devuuid));

	pthread_mutex_lock(&hash_lock);
	peer = zhashx_lookup(srvs_hash, devuuid);
	pthread_mutex_unlock(&hash_lock);
	if (!peer) {
		/*
		 * New connection from server
		 */

		data = sky_find_data_frame(msg);
		if (!data) {
			/* Malformed or unauthorized message? */
			zmsg_destroy(&msg);
			return;
		}

		peer = calloc(1, sizeof(*peer));
		if (!peer) {
			sky_err("No memory\n");
			zmsg_destroy(&msg);
			return;
		}

		peer->expires_at = zclock_time() +
			SKY_HEARTBEAT_IVL_MS * SKY_HEARTBEAT_CNT;
		peer->idents_msg = msg;
		memcpy(peer->devuuid, devuuid, sizeof(devuuid));

		if (!parse_devs_list_rsp(db_client, data, peer)) {
			server_peer_free(peer);
			return;
		}

		/* Take ownership on data frame */
		peer->data_frame = data;
		peer->rsp_data = (void *)zframe_data(data);
		zmsg_remove(msg, data);

		/* Remove all possible frames below IDENT+0 frames */
		for (i = 0, data = zmsg_first(msg); data;
		     data = zmsg_next(msg), i++) {
			/* Skip two ident frames: IDENT+0 */
			if (i >= 2) {
				zmsg_remove(msg, data);
				zframe_destroy(&data);
			}
		}

		pthread_mutex_lock(&hash_lock);
		rc = zhashx_insert(srvs_hash, peer->devuuid, peer);
		pthread_mutex_unlock(&hash_lock);
		if (rc) {
			sky_err("zhashx_insert(): %d\n", rc);
			server_peer_free(peer);
			return;
		}
	} else {
		/*
		 * Response from known server, forward to client.  Server keeps
		 * all idents on stack, so simply forward the whole message.
		 */

		authed = db_lookup_and_update_device_by_devuuid(db_client,
								peer->devdesc,
								true);
		if (!authed) {
			/*
			 * Charging device is not authorized any more.
			 * Do nothing, soon the peer will be expired
			 * and reaped.
			 */
			zmsg_destroy(&msg);
			return;
		}

		/* Refresh expiration */
		peer->expires_at = zclock_time() +
			SKY_HEARTBEAT_IVL_MS * SKY_HEARTBEAT_CNT;

		data = sky_find_data_frame(msg);
		if (!data) {
			/* Heartbeat message is empty, pong it back */
			rc = zmsg_send(&msg, servers);
			if (rc) {
				sky_err("zmsg_send(): %s\n", strerror(errno));
				zmsg_destroy(&msg);
				return;
			}
		} else {
			/* Remove IDENT+0 frame from stack */
			ident = zmsg_first(msg);
			zmsg_remove(msg, ident);
			zframe_destroy(&ident);
			ident = zmsg_next(msg);
			zmsg_remove(msg, ident);
			zframe_destroy(&ident);

			rc = zmsg_send(&msg, clients);
			if (rc) {
				sky_err("zmsg_send(): %s\n", strerror(errno));
				zmsg_destroy(&msg);
				return;
			}
		}
	}
}

static int sky_devs_list_rsp(zhashx_t *srvs_hash,
			     uuid_t *devuuids,
			     size_t nr_devuuids,
			     uint16_t proto_version,
			     struct sky_devs_list_rsp **rsp_,
			     size_t *rsp_len)
{
	struct sky_devs_list_rsp *rsp;
	struct server_peer *peer;
	void *rsp_data;
	bool dyn_info;

	size_t i, num, len;

	*rsp_ = NULL;
	*rsp_len = 0;

	/* Compatibility with protocols below 0x0400 */
	dyn_info = (proto_version >= 0x0400);

	len = dyn_info ?
		sizeof(*rsp) :
		offsetof_end(typeof(*rsp), info_off);
	rsp = rsp_data = calloc(1, len);
	if (!rsp)
		return -ENOMEM;

	rsp->hdr.type  = htole16(SKY_DEVS_LIST_RSP);
	rsp->hdr.error = 0;

	for (num = 0, i = 0; i < nr_devuuids; i++) {
		size_t info_len;
		void *tmp;

		pthread_mutex_lock(&hash_lock);
		peer = zhashx_lookup(srvs_hash, devuuids[i]);
		pthread_mutex_unlock(&hash_lock);
		if (!peer)
			/* Rarely can happen */
			continue;

		/* Skybroker does not try to peer old clients with new servers */
		if (!dyn_info && peer->dyn_info) {
			sky_err("The old client can't peer with the new server, so server will not be added to the result list!\n");
			continue;
		}

		info_len = peer->rsp_len - peer->info_off;
		tmp = realloc(rsp, len + info_len);
		if (!tmp) {
			free(rsp);
			return -ENOMEM;
		}
		rsp = rsp_data = tmp;

		memcpy(rsp_data + len, peer->rsp_data + peer->info_off, info_len);
		num += peer->num_devs;
		len += info_len;
	}
	rsp->num_devs = htole16(num);
	if (dyn_info) {
		/* Protocol version >= 0x0400 */
		rsp->info_off = htole16(offsetof_end(typeof(*rsp), info_off));
	}

	*rsp_ = rsp;
	*rsp_len = len;

	return 0;
}

static void sky_handle_client_msg(void *servers, void *clients,
				  zhashx_t *srvs_hash,
				  struct db_client *db_client)
{
	enum sky_proto_type req_type;
	struct sky_req_hdr *req_hdr;
	struct server_peer *peer;
	zframe_t *usruuid_frame;
	zframe_t *data, *ident;
	zmsg_t *msg, *srv_msg;
	size_t len;

	uint16_t proto_version;
	uuid_t devuuid, usruuid;
	bool authed;
	int rc;

	msg = zmsg_recv(clients);
	ident = msg ? zmsg_first(msg) : NULL;
	if (!ident) {
		zmsg_destroy(&msg);
		/* WTF? */
		return;
	}
	data = sky_find_data_frame(msg);
	if (!data) {
		/* Malformed message? */
		zmsg_destroy(&msg);
		return;
	}
	req_hdr = (void *)zframe_data(data);

	len = zframe_size(data);
	if (len < offsetof_end(typeof(*req_hdr), type)) {
		sky_err("malformed request header\n");
		/* Malformed request? */
		zmsg_destroy(&msg);
		return;
	} else if (len == offsetof_end(typeof(*req_hdr), type)) {
		/* Protocols below 0x0400 version */
		proto_version = 0;
	} else {
		/* Protocols below 0x0400 version has 0 in this field */
		proto_version = le16toh(req_hdr->proto_version);
	}

	req_type = le16toh(req_hdr->type);

	switch (req_type) {
	case SKY_DEVS_LIST_REQ: {
		/* TODO: cooperate with skyserver, make single function */
		struct sky_devs_list_rsp *rsp;
		uuid_t *devuuids = NULL;
		size_t len, nr_devuuids;

		usruuid_frame = zmsg_last(msg);
		if (!is_uuid_frame_valid(usruuid_frame)) {
			/* Malformed message? */
			sky_err("Malformed message: invalid USRUUID frame\n");
			zmsg_destroy(&msg);
			return;
		}

		memcpy(usruuid, zframe_data(usruuid_frame), sizeof(usruuid));
		(void)db_lookup_online_devices_by_usruuid(db_client, usruuid,
							  &devuuids, &nr_devuuids);

		rc = sky_devs_list_rsp(srvs_hash, devuuids, nr_devuuids,
				       proto_version, &rsp, &len);
		free(devuuids);
		if (rc) {
			sky_err("sky_devs_list_rsp(): %s\n", strerror(-rc));
			zmsg_destroy(&msg);
			return;
		}
		/* Replace frame with rsp (does memcpy, thus free rsp) */
		zframe_reset(data, rsp, len);
		free(rsp);
		rc = zmsg_send(&msg, clients);
		if (rc) {
			sky_err("zmsg_send(): %s\n", strerror(errno));
			zmsg_destroy(&msg);
		}
		break;
	}
	case SKY_PEERINFO_REQ: {
		/* TODO: cooperate with skyserver, make a single function */
		struct sky_peerinfo_rsp rsp;

		usruuid_frame = zmsg_last(msg);
		if (!is_uuid_frame_valid(usruuid_frame)) {
			/* Malformed message? */
			sky_err("Malformed message: invalid USRUUID frame\n");
			zmsg_destroy(&msg);
			return;
		}
		memcpy(usruuid, zframe_data(usruuid_frame), sizeof(usruuid));
		authed = db_lookup_user_by_usruuid(db_client, usruuid);

		if (authed) {
			rsp = (struct sky_peerinfo_rsp) {
				.hdr.type  = htole16(SKY_PEERINFO_RSP),
				.hdr.error = htole16(0),
				.proto_version  = htole16(SKY_PROTO_VERSION),
				.server_version = htole32(SKY_VERSION)
			};
		} else {
			rsp = (struct sky_peerinfo_rsp) {
				.hdr.type  = htole16(SKY_PEERINFO_RSP),
				.hdr.error = htole16(EPERM),
			};
		}

		zframe_reset(data, &rsp, sizeof(rsp));
		rc = zmsg_send(&msg, clients);
		if (rc) {
			sky_err("zmsg_send(): %s\n", strerror(errno));
			zmsg_destroy(&msg);
		}
		break;
	}
	default:
		/* Destination ident frame is always the last in the message */
		ident = zmsg_last(msg);
		if (!is_uuid_frame_valid(ident)) {
			sky_err("Malformed ident\n");
			zmsg_destroy(&msg);
			return;
		}
		usruuid_frame = sky_zmsg_prev(msg, ident);
		if (!is_uuid_frame_valid(usruuid_frame)) {
			/* Malformed message? */
			sky_err("Malformed message: invalid USRUUID frame\n");
			zmsg_destroy(&msg);
			return;
		}
		memcpy(devuuid, zframe_data(ident), sizeof(devuuid));
		memcpy(usruuid, zframe_data(usruuid_frame), sizeof(usruuid));

		authed = db_lookup_device_by_usruuid_devuuid(db_client, usruuid,
							     devuuid);
		if (!authed) {
			char devuuid_str[37];

			uuid_unparse(devuuid, devuuid_str);
			sky_err("Unauthorized attempt to access device with UUID '%s'\n",
				devuuid_str);
			zmsg_destroy(&msg);
			return;
		}

		pthread_mutex_lock(&hash_lock);
		peer = zhashx_lookup(srvs_hash, devuuid);
		pthread_mutex_unlock(&hash_lock);
		if (!peer) {
			sky_err("zhashx_lookup(): Unknown peer\n");
			zmsg_destroy(&msg);
			return;
		}
		zmsg_remove(msg, ident);
		zframe_destroy(&ident);

		srv_msg = zmsg_dup(peer->idents_msg);
		if (!srv_msg) {
			sky_err("zmsg_dup(): No memory\n");
			zmsg_destroy(&msg);
			return;
		}

		/* Copy all frames from client message to server message */
		data = zmsg_first(msg);
		while (data) {
			zmsg_remove(msg, data);
			rc = zmsg_append(srv_msg, &data);
			if (rc) {
				sky_err("zmsg_prepend(): No memory\n");
				zframe_destroy(&data);
				zmsg_destroy(&msg);
				zmsg_destroy(&srv_msg);
				return;
			}
			data = zmsg_next(msg);
		}

		zmsg_destroy(&msg);
		rc = zmsg_send(&srv_msg, servers);
		if (rc) {
			sky_err("zmsg_send(): %s\n", strerror(errno));
			zmsg_destroy(&srv_msg);
		}
		break;
	}
}

/* TODO: same function exists in skyserver.c */
static int sky_kill_pthread(pthread_t thread)
{
	int rc;

	do {
		struct timespec ts;

		/*
		 * Repeat killing to cover the race if first signal
		 * comes when we are not in kernel.
		 */
		clock_gettime(CLOCK_REALTIME, &ts);
		ts.tv_sec += 1;
		pthread_kill(thread, SIGTERM);
		rc = pthread_timedjoin_np(thread, NULL, &ts);
	} while (rc == ETIMEDOUT);

	return rc;
}

struct pub_proxy {
	void *pub;
	void *pull;
	zhashx_t *srvs_hash;
	struct db_client db_client;

	pthread_t thr;
};

static int rsp_to_charging_state(zframe_t *rsp_frame,
				 struct sky_charging_state *state)
{
	struct sky_charging_state_rsp *untrusty_rsp;
	struct sky_charging_state_rsp rsp;

	untrusty_rsp = (void *)zframe_data(rsp_frame);

	if (zframe_size(rsp_frame) < sizeof(untrusty_rsp->hdr))
		/* Malformed response */
		return -EPROTO;

	if (le16toh(untrusty_rsp->hdr.type) != SKY_CHARGING_STATE_EV ||
	    untrusty_rsp->hdr.error != 0)
		/* Malformed response */
		return -EPROTO;

	memset(&rsp, 0, sizeof(rsp));
	memcpy(&rsp, untrusty_rsp, min(zframe_size(rsp_frame), sizeof(rsp)));

	state->dev_hw_state = rsp.dev_hw_state;
	state->dev_hw_reason = rsp.dev_hw_reason;
	state->voltage_mV = le16toh(rsp.voltage_mV);
	state->current_mA = le16toh(rsp.current_mA);
	state->state_of_charge = le16toh(rsp.state_of_charge);
	state->until_full_secs = le16toh(rsp.until_full_secs);
	state->charging_secs = le16toh(rsp.charging_secs);
	state->mux_temperature_C = le16toh(rsp.mux_temperature_C);
	state->sink_temperature_C = le16toh(rsp.sink_temperature_C);
	state->energy_mWh = le32toh(rsp.energy_mWh);
	state->charge_mAh = le32toh(rsp.charge_mAh);
	state->mux_humidity_perc = rsp.mux_humidity_perc;
	state->link_quality_factor = rsp.link_quality_factor;

	state->tx.bytes       = le32toh(rsp.tx.bytes);
	state->tx.packets     = le32toh(rsp.tx.packets);
	state->tx.err_bytes   = le32toh(rsp.tx.err_bytes);
	state->tx.err_packets = le32toh(rsp.tx.err_packets);

	state->rx.bytes       = le32toh(rsp.rx.bytes);
	state->rx.packets     = le32toh(rsp.rx.packets);
	state->rx.err_bytes   = le32toh(rsp.rx.err_bytes);
	state->rx.err_packets = le32toh(rsp.rx.err_packets);

	state->charging_session_id = le32toh(rsp.charging_session_id);

	return 0;
}

static int sky_handle_charging_state(struct pub_proxy *proxy, zmsg_t *msg)
{
	struct sky_charging_state charging_state;
	struct server_peer *peer;
	zframe_t *ident, *data;
	uuid_t devuuid;
	int rc;

	/* Skip subscription topic */
	zmsg_first(msg);
	data = zmsg_next(msg);
	ident = zmsg_next(msg);
	if (!ident || !data) {
		return -EPROTO;
	}
	if (zframe_size(ident) < sizeof(devuuid)) {
		sky_err("Malformed IDENT\n");
		return -EPROTO;
	}
	memcpy(&devuuid, zframe_data(ident), sizeof(devuuid));

	rc = rsp_to_charging_state(data, &charging_state);
	if (rc) {
		sky_err("Malformed charging state response\n");
		return rc;
	}

	pthread_mutex_lock(&hash_lock);
	peer = zhashx_lookup(proxy->srvs_hash, devuuid);
	pthread_mutex_unlock(&hash_lock);
	if (!peer) {
		char devuuid_str[37];

		uuid_unparse(devuuid, devuuid_str);
		return -EPERM;
	}

	if (!sky_hw_is_idle(SKY_MUX_HW2, charging_state.dev_hw_state))
		/* Insert only if charge state is "interesting" */
		db_update_charging_session(&proxy->db_client, devuuid,
					   &charging_state);

	db_update_charger_device(&proxy->db_client, devuuid,
				 &charging_state);


	return 0;
}

static int sky_zmq_proxy(struct pub_proxy *proxy)
{
	int rc;

	while (true) {
		zmq_pollitem_t items [] = {
			{ proxy->pull, 0, ZMQ_POLLIN, 0 },
		};
		zmsg_t *msg;

		rc = zmq_poll(items, ARRAY_SIZE(items), -1);
		if (rc == -1)
			/* Interrupted */
			return -EINTR;

		msg = zmsg_recv(proxy->pull);
		if (!msg) {
			sky_err("Received NULL message\n");
			return -ECONNRESET;
		}

		rc = sky_handle_charging_state(proxy, msg);
		if (rc) {
			/* Ignore message and don't publish */
			zmsg_destroy(&msg);
			continue;
		}

		rc = zmsg_send(&msg, proxy->pub);
		if (rc) {
			sky_err("Failed to send a message\n");
			zmsg_destroy(&msg);
			return -ECONNRESET;
		}
	}

	return 0;
}

static void *thread_do_proxy(void *arg)
{
	struct pub_proxy *proxy = arg;

	(void)sky_zmq_proxy(proxy);

	return NULL;
}

static int sky_setup_and_proxy_pub(void *ctx, struct sky_discovery *discovery,
				   struct db *db, const char *ip,
				   struct pub_proxy *proxy)
{
	void *pub = NULL;
	void *pull = NULL;
	char zaddr[128];
	int rc, opt;

	pub = zmq_socket(ctx, ZMQ_PUB);
	if (!pub) {
		sky_err("zmq_socket(ZMQ_PUB): No memory\n");
		rc = -ENOMEM;
		goto err;
	}
	opt = 0;
	rc = zmq_setsockopt(pub, ZMQ_LINGER, &opt, sizeof(opt));
	if (rc != 0) {
		rc = -errno;
		sky_err("zmq_setsockopt(ZMQ_LINGER)\n");
		goto err;
	}
	snprintf(zaddr, sizeof(zaddr), "tcp://%s:%d",
		 ip, le16toh(discovery->pub_port));
        rc = zmq_bind(pub, zaddr);
	if (rc) {
		rc = -errno;
		sky_err("zmq_bind(%s): %s\n", zaddr, strerror(-rc));
		goto err;
	}
	pull = zmq_socket(ctx, ZMQ_PULL);
	if (!pull) {
		sky_err("zmq_socket(ZMQ_PULL): No memory\n");
		rc = -ENOMEM;
		goto err;
	}
	opt = 0;
	rc = zmq_setsockopt(pull, ZMQ_LINGER, &opt, sizeof(opt));
	if (rc != 0) {
		rc = -errno;
		sky_err("zmq_setsockopt(ZMQ_LINGER)\n");
		goto err;
	}
	snprintf(zaddr, sizeof(zaddr), "tcp://%s:%d",
		 ip, le16toh(discovery->sub_port));
	rc = zmq_bind(pull, zaddr);
	if (rc) {
		rc = -errno;
		sky_err("zmq_bind(%s): %s\n", zaddr, strerror(-rc));
		goto err;
	}

	rc = db_client_connect(db, &proxy->db_client);
	if (rc)
		goto err;

	proxy->pub = pub;
	proxy->pull = pull;

	rc = pthread_create(&proxy->thr, NULL, thread_do_proxy, proxy);
	if (rc) {
		rc = -rc;
		sky_err("pthread_create(): %s\n", strerror(-rc));
		goto err_disconnect_client;
	}

	return 0;

err_disconnect_client:
	db_client_disconnect(&proxy->db_client);
err:
	if (pub)
		zmq_close(pub);
	if (pull)
		zmq_close(pull);

	return rc;
}

static void sky_destroy_pub(struct pub_proxy *proxy)
{
	(void)sky_kill_pthread(proxy->thr);
	db_client_disconnect(&proxy->db_client);
	zmq_close(proxy->pub);
	zmq_close(proxy->pull);
}

/**
 * setup_tcp_keepalive() - enable kernel TCP keepalive in order
 * to handle dead connections and properly free resources on
 * ZMQ side.
 */
static int setup_tcp_keepalive(void *sock)
{
	int rc, val;

	val = 1;
	rc = zmq_setsockopt(sock, ZMQ_TCP_KEEPALIVE, &val,
			    sizeof(val));
	if (rc) {
		sky_err("zmq_setsockopt(ZMQ_TCP_KEEPALIVE): %s\n",
			strerror(errno));
		return rc;
	}
	val = 5;
	rc = zmq_setsockopt(sock, ZMQ_TCP_KEEPALIVE_IDLE, &val,
			    sizeof(val));
	if (rc) {
		sky_err("zmq_setsockopt(ZMQ_TCP_KEEPALIVE_IDLE): %s\n",
			strerror(errno));
		return rc;
	}
	val = SKY_HEARTBEAT_CNT;
	rc = zmq_setsockopt(sock, ZMQ_TCP_KEEPALIVE_CNT, &val,
			    sizeof(val));
	if (rc) {
		sky_err("zmq_setsockopt(ZMQ_TCP_KEEPALIVE_CNT): %s\n",
			strerror(errno));
		return rc;
	}
	val = SKY_HEARTBEAT_IVL_MS / 1000;
	rc = zmq_setsockopt(sock, ZMQ_TCP_KEEPALIVE_INTVL, &val,
			    sizeof(val));
	if (rc) {
		sky_err("zmq_setsockopt(ZMQ_TCP_KEEPALIVE_INTVL): %s\n",
			strerror(errno));
		return rc;
	}

	return 0;
}

static void wait_for_ifaces_up(void)
{
	unsigned ITERS = 10;

	while (ITERS--) {
		ziflist_t *iflist;
		const char *name;

		iflist = ziflist_new();
		if (!iflist)
			goto rest_and_repeat;

		name = ziflist_first(iflist);
		while (name) {
			if (!strncmp("eth", name, 3))
				break;
			name = ziflist_next(iflist);
		}
		ziflist_destroy(&iflist);
		if (name)
			/* Found something, leave the loop */
			break;
rest_and_repeat:
		sleep(1);
	}
}

int main(int argc, char *argv[])
{
	struct sky_discovery discovery = {
		.magic = SKY_DISCOVERY_MAGIC,
		.proto_version = htole16(SKY_PROTO_VERSION),
	};
	struct db_client db_client;
	struct db db;

	void *ctx, *servers, *clients;
	struct pub_proxy pub_proxy;
	zhashx_t *srvs_hash;
	zactor_t *speaker;
	struct cli cli;
	char zaddr[32];
	int timeout;
	int rc, opt;

	rc = cli_parse(argc, argv, &cli);
	if (rc) {
		fprintf(stderr, "%s\n", cli_usage);
		return -1;
	}

	srvs_hash = zhashx_new();
	if (!srvs_hash) {
		sky_err("zhashx_new(): Failed\n");
		return -1;
	}

	zhashx_set_key_comparator(srvs_hash, uuid_comparator_fn);
	zhashx_set_key_duplicator(srvs_hash, NULL);
	zhashx_set_key_destructor(srvs_hash, NULL);
	zhashx_set_destructor(srvs_hash, peer_destructor_fn);
	zhashx_set_duplicator(srvs_hash, NULL);
	zhashx_set_key_hasher(srvs_hash, uuid_hasher_fn);

	pub_proxy.srvs_hash = srvs_hash;

	discovery.servers_port = atoi(cli.srvport);
	discovery.sub_port     = discovery.servers_port + 1;
	discovery.clients_port = atoi(cli.cliport);
	discovery.pub_port     = discovery.clients_port + 1;

	/*
	 * Daemonize before creating zmq socket, ZMQ is written by
	 * people who are not able to deal with fork() gracefully.
	 */
	if (cli.daemon)
		sky_daemonize(cli.pidf);

	ctx = zmq_ctx_new();
	if (!ctx) {
		sky_err("zmq_ctx_new(): Failed\n");
		return -1;
	}

	/*
	 * That is needed if daemon is started on boot before
	 * network is up.
	 *
	 * And yes, I know that systemd service should have
	 *
	 *   After=network-online.target
	 *   Wants=network-online.target
	 *
	 * and actually it has, but it does not work.
	 *
	 * I am tired fighting with systemd and all this admin
	 * stuff, so just wait for reasonable time and continue.
	 */
	wait_for_ifaces_up();

	rc = db_init(&cli, &db);
	if (rc != 0)
		return -1;

	rc = db_client_connect(&db, &db_client);
	if (rc != 0)
		return -1;

	servers = zmq_socket(ctx, ZMQ_ROUTER);
	clients = zmq_socket(ctx, ZMQ_ROUTER);
	if (!servers || !clients) {
		sky_err("zmq_socket(): Failed\n");
		return -1;
	}
	opt = 0;
	rc = zmq_setsockopt(servers, ZMQ_LINGER, &opt, sizeof(opt));
	if (rc != 0) {
		sky_err("zmq_setsockopt(ZMQ_LINGER)\n");
		return -1;
	}
	opt = 0;
	rc = zmq_setsockopt(clients, ZMQ_LINGER, &opt, sizeof(opt));
	if (rc != 0) {
		sky_err("zmq_setsockopt(ZMQ_LINGER)\n");
		return -1;
	}

	rc = setup_tcp_keepalive(servers);
	if (rc)
		return -1;

	rc = setup_tcp_keepalive(clients);
	if (rc)
		return -1;

	opt = 1;
	rc = zmq_setsockopt(servers, ZMQ_ROUTER_HANDOVER, &opt,
			    sizeof(opt));
	if (rc) {
		sky_err("zmq_setsockopt(ZMQ_ROUTER_HANDOVER): %s\n",
			strerror(errno));
		return -1;
	}

	snprintf(zaddr, sizeof(zaddr), "tcp://%s:%d", cli.addr,
		 discovery.servers_port);
	rc = zmq_bind(servers, zaddr);
	if (rc)	{
		sky_err("zmq_bind(%s): %s\n", zaddr, strerror(errno));
		return -1;
	}
	snprintf(zaddr, sizeof(zaddr), "tcp://%s:%d", cli.addr,
		 discovery.clients_port);
	rc = zmq_bind(clients, zaddr);
	if (rc)	{
		sky_err("zmq_bind(%s): %s\n", zaddr, strerror(errno));
		return -1;
	}

	speaker = zactor_new(zbeacon, NULL);
	if (!speaker) {
		sky_err("zactor_new(): Failed\n");
		return -1;
	}
	rc = zsock_send(speaker, "si", "CONFIGURE", SKY_DISCOVERY_PORT);
	if (rc) {
		sky_err("zsock_send(): %s\n", strerror(errno));
		return -1;
	}
	rc = zsock_send(speaker, "sb", "PUBLISH",
			&discovery, sizeof(discovery));
	if (rc) {
		sky_err("zsock_send(): %s\n", strerror(errno));
		return -1;
	}
	rc = sky_setup_and_proxy_pub(ctx, &discovery, &db, cli.addr,
				     &pub_proxy);
	if (rc) {
		sky_err("sky_setup_and_proxy_pub(): %d\n", rc);
		return -1;
	}

	timeout = -1;
	while (true) {
		zmq_pollitem_t items [] = {
			{ servers, 0, ZMQ_POLLIN, 0 },
			{ clients, 0, ZMQ_POLLIN, 0 }
		};

		rc = zmq_poll(items, 2, timeout);
		if (rc == -1)
			break; /* Interrupted */

		if (items[0].revents & ZMQ_POLLIN) {
			sky_handle_server_msg(servers, clients, srvs_hash,
					      &db_client);
		}
		if (items[1].revents & ZMQ_POLLIN) {
			sky_handle_client_msg(servers, clients, srvs_hash,
					      &db_client);
		}

		timeout = sky_reap_dead_servers(srvs_hash);
	}

	sky_destroy_pub(&pub_proxy);
	zhashx_destroy(&srvs_hash);
	zmq_close(servers);
	zmq_close(clients);
	zactor_destroy(&speaker);
	zmq_ctx_destroy(&ctx);
	db_client_disconnect(&db_client);
	db_deinit(&db);
	cli_free(&cli);

	return 0;
}

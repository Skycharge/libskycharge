#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <endian.h>
#include <limits.h>
#include <string.h>

#include <czmq.h>

#include "skybroker-cmd.h"
#include "daemon.h"
#include "version.h"
#include "skyproto.h"

struct server_peer {
	zmsg_t *idents_msg;
	zframe_t *ident_frame;
	zframe_t *data_frame;
	zframe_t *usruuid_frame;
	const void *rsp_data;
	size_t rsp_len;
	size_t num_devs;
	size_t info_off;
	bool dyn_info;
	union {
		unsigned long long expires_at;
		struct server_peer *next;
	};
};

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

static void server_peer_free(struct server_peer *peer)
{
	zmsg_destroy(&peer->idents_msg);
	zframe_destroy(&peer->data_frame);
	zframe_destroy(&peer->usruuid_frame);
	free(peer);
}

static void peer_destructor_fn(void **item)
{
	struct server_peer *peer = *item;

	server_peer_free(peer);
}

static int zframe_comparator_fn(const void *item1, const void *item2)
{
	zframe_t *frame1 = (void *)item1;
	zframe_t *frame2 = (void *)item2;
	size_t sz1 = zframe_size(frame1);
	size_t sz2 = zframe_size(frame2);

	if (sz1 < sz2)
		return -1;
	else if (sz1 > sz2)
		return 1;

	return memcmp(zframe_data(frame1), zframe_data(frame2), sz1);
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

static size_t zframe_hasher_fn(const void *key)
{
	zframe_t *frame = (void *)key;

	return jhash((char *)zframe_data(frame), zframe_size(frame));
}

static int sky_reap_dead_servers(zhashx_t *hash)
{
	struct server_peer *head = NULL, *next;
	struct server_peer *peer;
	unsigned long long ms, nearest = ~0ull;

	ms = zclock_time();

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
		zhashx_delete(hash, head->ident_frame);
		head = next;
	}

	return nearest == ~0ull ? -1 : (nearest - ms) * ZMQ_POLL_MSEC;
}

static bool sky_is_valid_devs_list_rsp(zframe_t *data, struct server_peer *peer)
{
	struct sky_devs_list_rsp *rsp;

	size_t num_devs, rsp_len, off, info_off;
	void *rsp_data;
	bool dyn_info;
	int i;

	rsp_data = (void *)zframe_data(data);
	rsp = rsp_data;
	rsp_len = zframe_size(data);
	if (rsp_len < offsetof_end(typeof(*rsp), info_off))
		return false;
	if (rsp->hdr.error)
		return false;

	num_devs = le16toh(rsp->num_devs);
	if (!num_devs)
		return false;

	info_off = le16toh(rsp->info_off);
	/* Dynamic info is supported starting from protocol version >= 0x0400 */
	dyn_info = !!info_off;
	info_off = off = (info_off ?: offsetof_end(typeof(*rsp), info_off));
	if (rsp_len < info_off)
		return false;

	for (i = 0; i < num_devs; i++) {
		struct sky_dev_info *notrust_info = rsp_data + info_off;
		size_t info_len;

		/* Sanity checks */
		if (rsp_len < info_off + offsetof_end(typeof(*notrust_info), info_len))
			return false;

		/* Compatibility with protocols below 0x0400 */
		info_len = dyn_info ?
			le16toh(notrust_info->info_len) :
			offsetof_end(typeof(*notrust_info), portname);
		if (!info_len)
			return false;

		info_off += info_len;
	}

	peer->rsp_len = rsp_len;
	peer->num_devs = num_devs;
	peer->info_off = off;
	peer->dyn_info = dyn_info;

	return true;
}

static bool is_usruuid_frame_valid(zframe_t *usruuid)
{
	return (usruuid && zframe_size(usruuid) == 16);
}

static void sky_handle_server_msg(void *servers, void *clients,
				  zhashx_t *srvs_hash)
{
	zframe_t *data, *ident, *usruuid;
	struct server_peer *peer;
	zmsg_t *msg;
	int i, rc;

	msg = zmsg_recv(servers);
	ident = msg ? zmsg_first(msg) : NULL;
	if (!ident) {
		zmsg_destroy(&msg);
		/* WTF? */
		return;
	}
	if (zframe_size(ident) !=
	    sizeof(((struct sky_dev_info *)0)->dev_uuid)) {
		/* Support uuids as idents only */
		zmsg_destroy(&msg);
		return;
	}

	peer = zhashx_lookup(srvs_hash, ident);
	if (!peer) {
		/*
		 * New connection from server
		 */

		data = sky_find_data_frame(msg);
		if (!data) {
			/* Malformed message? */
			sky_err("Malformed message\n");
			zmsg_destroy(&msg);
			return;
		}
		/* USRUUID frame is the last one */
		usruuid = zmsg_last(msg);
		if (!is_usruuid_frame_valid(usruuid)) {
			/* Malformed message? */
			sky_err("Malformed message: invalid USRUUID frame\n");
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
		peer->ident_frame = ident;

		if (!sky_is_valid_devs_list_rsp(data, peer)) {
			sky_err("Malformed devs list\n");
			server_peer_free(peer);
			return;
		}

		/* Take ownership on data frame */
		peer->data_frame = data;
		peer->rsp_data = (void *)zframe_data(data);
		zmsg_remove(msg, data);

		/* Take ownership on USRUUID frame */
		peer->usruuid_frame = usruuid;
		zmsg_remove(msg, usruuid);

		/* Remove all possible frames below IDENT+0 frames */
		for (i = 0, data = zmsg_first(msg); data;
		     data = zmsg_next(msg), i++) {
			/* Skip two ident frames: IDENT+0 */
			if (i >= 2)
				zmsg_remove(msg, data);
		}

		rc = zhashx_insert(srvs_hash, peer->ident_frame, peer);
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
			     zframe_t *usruuid,
			     uint16_t proto_version,
			     struct sky_devs_list_rsp **rsp_,
			     size_t *rsp_len)
{
	struct sky_devs_list_rsp *rsp;
	struct server_peer *peer;
	void *rsp_data;
	bool dyn_info;

	size_t num, len;

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

	/*
	 * TODO: O(n) loop, makes sense to make something better?
	 */
	for (num = 0, peer = zhashx_first(srvs_hash); peer;
	     peer = zhashx_next(srvs_hash)) {
		size_t info_len;
		void *tmp;

		/* Find corresponding user devices */
		if (!zframe_eq(peer->usruuid_frame, usruuid))
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
				  zhashx_t *srvs_hash)
{
	enum sky_proto_type req_type;
	struct sky_req_hdr *req_hdr;
	struct server_peer *peer;
	zframe_t *data, *ident;
	zmsg_t *msg, *srv_msg;
	size_t len;

	uint16_t proto_version;
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
		zframe_t *usruuid;
		size_t len;

		usruuid = zmsg_last(msg);
		if (!is_usruuid_frame_valid(usruuid)) {
			/* Malformed message? */
			sky_err("Malformed message: invalid USRUUID frame\n");
			zmsg_destroy(&msg);
			return;
		}
		rc = sky_devs_list_rsp(srvs_hash, usruuid, proto_version, &rsp, &len);
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
		/* TODO: cooperate with skyserver, make single function */
		struct sky_peerinfo_rsp *rsp;

		rsp = calloc(1, sizeof(*rsp));
		if (!rsp) {
			sky_err("No memory\n");
			zmsg_destroy(&msg);
			return;
		}

		rsp->hdr.type  = htole16(SKY_PEERINFO_RSP);
		rsp->hdr.error = htole16(0);
		rsp->proto_version  = htole16(SKY_PROTO_VERSION);
		rsp->server_version = htole32(SKY_VERSION);

		zframe_reset(data, rsp, sizeof(*rsp));
		free(rsp);
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
		assert(ident);

		peer = zhashx_lookup(srvs_hash, ident);
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
	pthread_t thr;
};

static void *thread_do_proxy(void *arg)
{
	struct pub_proxy *proxy = arg;

	(void)zmq_proxy(proxy->pub, proxy->pull, NULL);

	return NULL;
}

static int sky_setup_and_proxy_pub(void *ctx, struct sky_discovery *discovery,
				   const char *ip, struct pub_proxy *proxy)
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
	proxy->pub = pub;
	proxy->pull = pull;

	rc = pthread_create(&proxy->thr, NULL, thread_do_proxy, proxy);
	if (rc) {
		rc = -rc;
		sky_err("pthread_create(): %s\n", strerror(-rc));
		goto err;
	}

	return 0;

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
	rc = sky_setup_and_proxy_pub(ctx, &discovery, cli.addr, &pub_proxy);
	if (rc) {
		sky_err("sky_setup_and_proxy_pub(): %d\n", rc);
		return -1;
	}

	srvs_hash = zhashx_new();
	if (!srvs_hash) {
		sky_err("zhashx_new(): Failed\n");
		return -1;
	}

	zhashx_set_key_comparator(srvs_hash, zframe_comparator_fn);
	zhashx_set_key_duplicator(srvs_hash, NULL);
	zhashx_set_key_destructor(srvs_hash, NULL);
	zhashx_set_destructor(srvs_hash, peer_destructor_fn);
	zhashx_set_duplicator(srvs_hash, NULL);
	zhashx_set_key_hasher(srvs_hash, zframe_hasher_fn);

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
			sky_handle_server_msg(servers, clients, srvs_hash);
		}
		if (items[1].revents & ZMQ_POLLIN) {
			sky_handle_client_msg(servers, clients, srvs_hash);
		}

		timeout = sky_reap_dead_servers(srvs_hash);
	}

	sky_destroy_pub(&pub_proxy);
	zhashx_destroy(&srvs_hash);
	zmq_close(servers);
	zmq_close(clients);
	zactor_destroy(&speaker);
	zmq_ctx_destroy(&ctx);
	cli_free(&cli);

	return 0;
}

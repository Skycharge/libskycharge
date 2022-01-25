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
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <poll.h>

#include <zmq.h>
#include <czmq.h>

#include "libskycharge-pri.h"
#include "types.h"
#include "skyproto.h"
#include "rbtree.h"

struct zocket {
	void *ctx;
	void *sub;
};

struct skyrem_dev {
	struct sky_dev dev;
	struct zocket  zock;
};

struct skyrem_async {
	struct sky_async   async;
	struct rb_root     reqs_root; /* requests awaiting for responses */
	unsigned int       reqs_seq;
	void               *zctx;
	void               *zock;
	unsigned long long completions;
};

struct sky_async_req_node {
	struct rb_node       entry;  /* entry in skyrem_async->reqs_root */
	struct sky_async_req *req;
	unsigned int         tag;
};

static inline int cmp_tags(const struct rb_node *a_,
			   const struct rb_node *b_)
{
	struct sky_async_req_node *a;
	struct sky_async_req_node *b;

	a = container_of(a_, typeof(*a), entry);
	b = container_of(b_, typeof(*b), entry);

	return (a->tag < b->tag ? -1 :
		a->tag > b->tag ? 1 : 0);
}

static int skyrem_asyncreq_register(struct skyrem_async *async,
				    struct sky_async_req *req,
				    unsigned int tag)
{
	struct rb_node **this = &async->reqs_root.rb_node;
	struct rb_node *parent = NULL;
	struct rb_node *new;
	int cmp;

	struct sky_async_req_node *req_node;

	req_node = malloc(sizeof(*req_node));
	if (!req_node)
		return -ENOMEM;

	req->tag = tag;
	req_node->req = req;
	req_node->tag = tag;
	new = &req_node->entry;

	while (*this) {
		parent = *this;
		cmp = cmp_tags(new, *this);

		/*
		 * Equal elements go to the right, i.e. FIFO order, thus '<',
		 * if LIFO is needed then use '<='
		 */
		if (cmp < 0)
			this = &(*this)->rb_left;
		else
			this = &(*this)->rb_right;
	}
	/* Add new node to the tree and rebalance it */
	rb_link_node(new, parent, this);
	rb_insert_color(new, &async->reqs_root);

	return 0;
}

static struct sky_async_req *
skyrem_asyncreq_lookup_and_remove(struct skyrem_async *async,
				  unsigned int tag)
{
	struct rb_node *this = async->reqs_root.rb_node;
	struct sky_async_req_node *req_node;
	struct sky_async_req *req;

	while (this) {
		req_node = container_of(this, typeof(*req_node), entry);
		if (tag < req_node->tag) {
			this = this->rb_left;
		} else if (tag > req_node->tag) {
			this = this->rb_right;
		} else {
			req = req_node->req;
			rb_erase(&req_node->entry, &async->reqs_root);
			free(req_node);
			req->tag = 0;

			return req;
		}
	}

	return NULL;
}

static bool __skyrem_asyncreq_cancel(struct skyrem_async *async,
				     struct sky_async_req *req)
{
	bool res = false;

	if (req->next != req) {
		/* Request still in the submit queue */
		assert(!req->tag);
		sky_asyncreq_del(req);
		res = true;
	} else if (req->tag) {
		/* Request waits for response */
		res = (req == skyrem_asyncreq_lookup_and_remove(async, req->tag));
	}

	return res;
}

static bool skyrem_asyncreq_cancel(struct sky_async *async_,
				   struct sky_async_req *req)
{
	struct skyrem_async *async;

	async = container_of(async_, typeof(*async), async);
	return __skyrem_asyncreq_cancel(async, req);
}

static bool skyrem_asyncreq_complete(struct skyrem_async *async,
				     struct sky_async_req *req,
				     int rc)
{
	if (req->out.completed)
		return false;

	__skyrem_asyncreq_cancel(async, req);
	return sky_asyncreq_complete(&async->async, req, rc);
}

static const uint8_t *skyrem_asyncreq_useruuid(const struct skyrem_async *async,
					       const struct sky_async_req *req)
{
	const struct sky_conf *conf = async->async.conf;
	return req->usruuid ? req->usruuid : conf->usruuid;
}

static int skyrem_connect(struct skyrem_async *async)
{
	const struct sky_conf *conf = async->async.conf;
	void *zctx, *zock;
	char zaddr[128];
	int rc, opt;

	rc = snprintf(zaddr, sizeof(zaddr), "tcp://%s:%d",
		      conf->hostname,
		      conf->cliport);
	if (rc < 0 || rc >= sizeof(zaddr))
		return -EINVAL;

	zctx = zmq_ctx_new();
	if (zctx == NULL)
		return -ENOMEM;

	zock = zmq_socket(zctx, ZMQ_DEALER);
	if (zock == NULL) {
		rc = -ENOMEM;
		goto err_free_ctx;
	}

	opt = DEFAULT_TIMEOUT;
	rc = zmq_setsockopt(zock, ZMQ_RCVTIMEO, &opt, sizeof(opt));
	if (rc != 0) {
		rc = -errno;
		goto err;
	}
	opt = DEFAULT_TIMEOUT;
	rc = zmq_setsockopt(zock, ZMQ_SNDTIMEO, &opt, sizeof(opt));
	if (rc != 0) {
		rc = -errno;
		goto err;
	}
	opt = 0;
	rc = zmq_setsockopt(zock, ZMQ_LINGER, &opt, sizeof(opt));
	if (rc != 0) {
		rc = -errno;
		goto err;
	}
	rc = zmq_connect(zock, zaddr);
	if (rc != 0) {
		rc = -errno;
		goto err;
	}
	async->zctx = zctx;
	async->zock = zock;

	return 0;

err:
	zmq_close(zock);
err_free_ctx:
	zmq_ctx_term(zctx);

	return rc;
}

static int skyrem_sendreq(struct skyrem_async *async,
			  struct sky_async_req *req,
			  struct sky_req_hdr *hdr,
			  size_t len)
{
	const uint8_t *usruuid = skyrem_asyncreq_useruuid(async, req);
	unsigned int tag;
	le32 le32tag;
	zmsg_t *msg;
	int rc;

	/*
	 * USRUUID should be a valid pointer, but can be zero-uuid for
	 * a connection between client and server in LAN.
	 */
	if (!usruuid)
		return -EINVAL;

	msg = zmsg_new();
	if (!msg)
		return -ENOMEM;

	/* Set protocol version for each request */
	hdr->proto_version = htole16(SKY_PROTO_VERSION);

	tag = async->reqs_seq + 1;
	le32tag = htole32(tag);

	/* Prepend zero frame for DEALER to emulate REQ */
	rc = zmsg_addmem(msg, NULL, 0);
	rc |= zmsg_addmem(msg, hdr, len);

	if (req->dev) {
		const struct sky_dev_desc *devdesc = &req->dev->devdesc;

		/* DEVPORT frame follows request frame */
		rc |= zmsg_addstr(msg, devdesc->portname);
		/* TAG frame for the response identification */
		rc |= zmsg_addmem(msg, &le32tag, sizeof(le32tag));
		/* USRUUID frame is the penultimate */
		rc |= zmsg_addmem(msg, usruuid, 16);
		/* DEVUUID frame is the last one */
		rc |= zmsg_addmem(msg, devdesc->dev_uuid,
				  sizeof(devdesc->dev_uuid));
	} else {
		/*
		 * DEVPORT frame follows request frame. But for this kind
		 * of request DEVPORT should not be sent at all, but we
		 * send TAG frame, which will be treated by the server as
		 * a DEVPORT frame and removed from the response. In order
		 * to get the TAG frame back we have to send something not
		 * zero, otherwise server side won't find data frame and
		 * sky_find_data_frame() returns incorrect result.
		 *
		 * Why we send TAG two times? Because skybroker is always
		 * transparrent and modifies only the data frame replacing
		 * with a response, thus other frames are not touched or
		 * removed. Since the behaviour of skyserver and skybroker
		 * differs (skyserver removes devport frame, skybroker
		 * does not) we duplicate the TAG frame in order for
		 * different connections (to the skyserver or to the
		 * skybroker) get correct result.  Arghh.
		 */
		rc |= zmsg_addmem(msg, &le32tag, sizeof(le32tag));
		/* TAG frame for the response identification */
		rc |= zmsg_addmem(msg, &le32tag, sizeof(le32tag));
		/* USRUUID frame is the last one */
		rc |= zmsg_addmem(msg, usruuid, 16);
	}
	if (rc != 0) {
		rc = -ENOMEM;
		goto out;
	}
	/* Send request */
	rc = zmsg_send(&msg, async->zock);
	if (rc != 0) {
		rc = -errno;
		goto out;
	}
	rc = skyrem_asyncreq_register(async, req, tag);
	if (rc)
		goto out;

	async->reqs_seq = tag;

out:
	zmsg_destroy(&msg);
	return rc;
}

static int skyrem_recvresp(struct skyrem_async *async,
			   struct sky_async_req **req_,
			   void **rsp_, size_t *len_)
{
	zframe_t *data_frame, *tag_frame;
	struct sky_async_req *req;
	struct sky_rsp_hdr *hdr;
	unsigned int tag;
	zmsg_t *msg;
	void *rsp;
	size_t len;
	int rc;

	/* Recv response */
	msg = zmsg_recv(async->zock);
	if (!msg)
		goto ignored_malfored_response;

	/* Skip heading zero frame */
	zmsg_first(msg);
	data_frame = zmsg_next(msg);
	tag_frame = zmsg_next(msg);

	if (!data_frame || !tag_frame)
		goto ignored_malfored_response;
	if (zframe_size(tag_frame) != 4)
		goto ignored_malfored_response;

	tag = le32toh(*(le32*)zframe_data(tag_frame));
	req = skyrem_asyncreq_lookup_and_remove(async, tag);
	if (!req) {
		sky_err("Unknown response with tag: %d\n", tag);
		goto ignored_malfored_response;
	}
	len = zframe_size(data_frame);
	if (len < sizeof(*hdr))
		goto complete_with_EPROTO;

	hdr = (void *)zframe_data(data_frame);
	/* Response type always follows request type */
	if (le16toh(hdr->type) != req->type + 1)
		goto complete_with_EPROTO;

	rsp = malloc(len);
	if (!rsp) {
		rc = -ENOMEM;
		goto out;
	}
	memcpy(rsp, zframe_data(data_frame), len);
	rc = 0;
	*rsp_ = rsp;
	*req_ = req;
	*len_ = len;
out:
	zmsg_destroy(&msg);
	return rc;

complete_with_EPROTO:
	skyrem_asyncreq_complete(async, req, -EPROTO);
ignored_malfored_response:
	rc = 0;
	*rsp_ = NULL;
	*req_ = NULL;
	*len_ = 0;
	goto out;
}

static int skyrem_genericreq_send(struct skyrem_async *async,
				  struct sky_async_req *req)
{
	struct sky_req_hdr hdr = {
		.type = htole16(req->type)
	};
	return skyrem_sendreq(async, req, &hdr, sizeof(hdr));
}

static int skyrem_genericresp_parse(struct skyrem_async *async,
				    struct sky_async_req *req,
				    void *rsp_data, size_t rsp_len)
{
	struct sky_rsp_hdr *rsp = rsp_data;
	int rc;

	if (rsp_len < sizeof(*rsp))
		rc = -EPROTO;
	else
		rc = -le16toh(rsp->error);

	skyrem_asyncreq_complete(async, req, rc);
	return 0;
}

static int skyrem_paramsget_send(struct skyrem_async *async,
				 struct sky_async_req *req)
{
	const struct sky_dev_params *params = req->in.ptr;
	struct sky_get_dev_params_req remreq = {
		.hdr.type        = htole16(req->type),
		.dev_params_bits = htole32(params->dev_params_bits),
	};

	return skyrem_sendreq(async, req, &remreq.hdr, sizeof(remreq));
}

static int skyrem_paramsget_parse(struct skyrem_async *async,
				  struct sky_async_req *req,
				  void *rsp_data, size_t rsp_len)
{
	struct sky_dev_params *params = req->out.ptr;
	struct {
		struct sky_get_dev_params_rsp rsp;
		typeof(((struct sky_get_dev_params_rsp *)NULL)->dev_params[0])
				dev_params[ARRAY_SIZE(params->dev_params)];
	} *rsp_uni = rsp_data;

	int rc, i, ind;

	if (rsp_len < sizeof(rsp_uni->rsp))
		goto complete_with_EPROTO;

	rc = -le16toh(rsp_uni->rsp.hdr.error);
	if (rc)
		goto complete;

	for (i = 0, ind = 0; i < ARRAY_SIZE(params->dev_params); i++) {
		if (params->dev_params_bits & (1<<i)) {
			if (rsp_len < (sizeof(rsp_uni->rsp) +
				  sizeof(rsp_uni->dev_params[0]) * (ind + 1))) {
				/* Malformed response */
				goto complete_with_EPROTO;
			}
			params->dev_params[i] = le32toh(rsp_uni->dev_params[ind]);
			ind++;
		} else {
			params->dev_params[i] = 0;
		}
	}

complete:
	skyrem_asyncreq_complete(async, req, rc);
	return 0;

complete_with_EPROTO:
	rc = -EPROTO;
	goto complete;
}

static int skyrem_paramsset_send(struct skyrem_async *async,
				 struct sky_async_req *req)
{
	const struct sky_dev_params *params = req->in.ptr;
	struct {
		struct sky_set_dev_params_req req;
		typeof(((struct sky_set_dev_params_req *)NULL)->dev_params[0])
				dev_params[ARRAY_SIZE(params->dev_params)];
	} req_uni = {
		.req.hdr.type        = htole16(req->type),
		.req.dev_params_bits = htole32(params->dev_params_bits),
	};

	int sz, i, ind;

	assert(params->dev_params_bits);

	for (i = 0, ind = 0; i < ARRAY_SIZE(params->dev_params); i++) {
		if (params->dev_params_bits & (1<<i))
			req_uni.dev_params[ind++] =
				htole32(params->dev_params[i]);
	}
	sz = sizeof(req_uni.req) + sizeof(req_uni.dev_params[0]) * ind;

	return skyrem_sendreq(async, req, &req_uni.req.hdr, sz);
}

static int skyrem_sink_passthru_msgsend_send(struct skyrem_async *async,
					     struct sky_async_req *req)
{
	const struct sky_passthru_msg *msg = req->in.ptr;
	struct {
		struct sky_sink_passthru_msg_send_req req;
		uint8_t                               buf[sizeof(msg->buf)];
	} req_uni = {
		.req.hdr.type = htole16(req->type),
		.req.len      = msg->len,
	};
	size_t sz;

	sz = min_t(size_t, msg->len, sizeof(req_uni.buf));
	memcpy(req_uni.req.buf, &msg->buf, sz);

	sz = sizeof(req_uni.req) + msg->len;

	return skyrem_sendreq(async, req, &req_uni.req.hdr, sz);
}

static int skyrem_sink_passthru_msgrecv_parse(struct skyrem_async *async,
					      struct sky_async_req *req,
					      void *rsp_data, size_t rsp_len)
{
	struct sky_passthru_msg *msg = req->out.ptr;
	struct sky_sink_passthru_msg_recv_rsp *rsp = rsp_data;
	int rc;

	if (rsp_len < sizeof(*rsp))
		/* Malformed response */
		goto complete_with_EPROTO;

	if (rsp_len < sizeof(*rsp) + rsp->len)
		/* Malformed response */
		goto complete_with_EPROTO;

	if (rsp->len > sizeof(msg->buf))
		/* Malformed response */
		goto complete_with_EPROTO;

	rc = -le16toh(rsp->hdr.error);
	if (rc)
		goto complete;

	msg->len = rsp->len;
	memcpy(msg->buf, rsp->buf, rsp->len);

complete:
	skyrem_asyncreq_complete(async, req, rc);
	return 0;

complete_with_EPROTO:
	rc = -EPROTO;
	goto complete;
}


static int skyrem_chargingstate_parse(struct skyrem_async *async,
				      struct sky_async_req *req,
				      void *rsp_data, size_t rsp_len)
{
	struct sky_charging_state *state = req->out.ptr;
	struct sky_charging_state_rsp *untrusty_rsp = rsp_data;
	struct sky_charging_state_rsp rsp;
	int rc;

	if (rsp_len < sizeof(untrusty_rsp->hdr))
		/* Malformed response */
		goto complete_with_EPROTO;

	rc = -le16toh(untrusty_rsp->hdr.error);
	if (rc)
		goto complete;

	memset(&rsp, 0, sizeof(rsp));
	memcpy(&rsp, untrusty_rsp, min(rsp_len, sizeof(rsp)));

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

complete:
	skyrem_asyncreq_complete(async, req, rc);
	return 0;

complete_with_EPROTO:
	rc = -EPROTO;
	goto complete;
}

static int skyrem_devslist_parse(struct skyrem_async *async,
				 struct sky_async_req *req,
				 void *rsp_data, size_t rsp_len)
{
	struct sky_parse_devs_list_rsp_result result = {
		.list = req->out.ptr
	};
	int rc;

	rc = sky_parse_devs_list_rsp(rsp_data, rsp_len, &async->async,
				     &result);
	if (rc)
		return rc;

	skyrem_asyncreq_complete(async, req, result.error);
	return 0;
}

static int skyrem_peerinfo_parse(struct skyrem_async *async,
				 struct sky_async_req *req,
				 void *rsp_data, size_t rsp_len)
{
	struct sky_peerinfo *peerinfo = req->out.ptr;
	struct sky_peerinfo_rsp *rsp = rsp_data;
	int rc;

	if (rsp_len < sizeof(*rsp))
		/* Malformed response */
		goto complete_with_EPROTO;

	rc = -le16toh(rsp->hdr.error);
	if (rc)
		goto complete;

	peerinfo->server_version = le32toh(rsp->server_version);
	peerinfo->proto_version  = le16toh(rsp->proto_version);

complete:
	skyrem_asyncreq_complete(async, req, rc);
	return 0;

complete_with_EPROTO:
	rc = -EPROTO;
	goto complete;
}

static int skyrem_droneport_state_parse(struct skyrem_async *async,
					struct sky_async_req *req,
					void *rsp_data, size_t rsp_len)
{
	struct sky_droneport_state *state = req->out.ptr;
	struct sky_droneport_state_rsp *rsp = rsp_data;
	int rc;

	if (rsp_len < sizeof(*rsp))
		/* Malformed response */
		goto complete_with_EPROTO;

	rc = -le16toh(rsp->hdr.error);
	if (rc)
		goto complete;

	state->status = le32toh(rsp->status);

complete:
	skyrem_asyncreq_complete(async, req, rc);
	return 0;

complete_with_EPROTO:
	rc = -EPROTO;
	goto complete;
}

static int skyrem_gpsdata_parse(struct skyrem_async *async,
				struct sky_async_req *req,
				void *rsp_data, size_t rsp_len)
{
	struct sky_gpsdata *gpsdata = req->out.ptr;
	struct sky_gpsdata_rsp *rsp = rsp_data;
	int rc;

	if (rsp_len < sizeof(*rsp))
		/* Malformed response */
		goto complete_with_EPROTO;

	rc = -le16toh(rsp->hdr.error);
	if (rc)
		goto complete;

	if (le32toh(rsp->status) > SKY_GPS_STATUS_DGPS_FIX)
		goto complete_with_EPROTO;

	if (le32toh(rsp->fix.mode) > SKY_GPS_MODE_3D)
		goto complete_with_EPROTO;

	memset(gpsdata, 0, sizeof(*gpsdata));
	gpsdata->status = le32toh(rsp->status);
	gpsdata->satellites_used = le32toh(rsp->satellites_used);

#define LLU2D(i)  ({ uint64_t llu = (i); double *dp = (double *)&llu; *dp; })

	gpsdata->dop.xdop = LLU2D(le64toh(rsp->dop.xdop));
	gpsdata->dop.ydop = LLU2D(le64toh(rsp->dop.ydop));
	gpsdata->dop.pdop = LLU2D(le64toh(rsp->dop.pdop));
	gpsdata->dop.hdop = LLU2D(le64toh(rsp->dop.hdop));
	gpsdata->dop.vdop = LLU2D(le64toh(rsp->dop.vdop));
	gpsdata->dop.tdop = LLU2D(le64toh(rsp->dop.tdop));
	gpsdata->dop.gdop = LLU2D(le64toh(rsp->dop.gdop));

	gpsdata->fix.mode = le32toh(rsp->fix.mode);
	gpsdata->fix.time = LLU2D(le64toh(rsp->fix.time));
	gpsdata->fix.latitude  = LLU2D(le64toh(rsp->fix.latitude));
	gpsdata->fix.longitude = LLU2D(le64toh(rsp->fix.longitude));
	gpsdata->fix.altitude  = LLU2D(le64toh(rsp->fix.altitude));

complete:
	skyrem_asyncreq_complete(async, req, rc);
	return 0;

complete_with_EPROTO:
	rc = -EPROTO;
	goto complete;
}

static int skyrem_dronedetect_parse(struct skyrem_async *async,
				    struct sky_async_req *req,
				    void *rsp_data, size_t rsp_len)
{
	enum sky_drone_status *status = req->out.ptr;
	struct sky_dronedetect_rsp *rsp = rsp_data;
	int rc;

	if (rsp_len < sizeof(*rsp))
		/* Malformed response */
		goto complete_with_EPROTO;

	rc = -le16toh(rsp->hdr.error);
	if (rc)
		goto complete;

	*status = le16toh(rsp->status);

complete:
	skyrem_asyncreq_complete(async, req, rc);
	return 0;

complete_with_EPROTO:
	rc = -EPROTO;
	goto complete;
}

static int skyrem_sinkinfo_parse(struct skyrem_async *async,
				 struct sky_async_req *req,
				 void *rsp_data, size_t rsp_len)
{
	struct sky_sink_info *info = req->out.ptr;
	struct sky_sink_get_info_rsp *rsp = rsp_data;
	int rc;

	if (rsp_len < sizeof(*rsp))
		/* Malformed response */
		goto complete_with_EPROTO;

	rc = -le16toh(rsp->hdr.error);
	if (rc)
		goto complete;

	info->hw_info.fw_version =
		le32toh(rsp->fw_version);
	info->hw_info.hw_version =
		le32toh(rsp->hw_version);
	info->hw_info.plc_proto_version =
		le32toh(rsp->plc_proto_version);
	info->hw_info.uid.part1 =
		le32toh(rsp->hw_uid.part1);
	info->hw_info.uid.part2 =
		le32toh(rsp->hw_uid.part2);
	info->hw_info.uid.part2 =
		le32toh(rsp->hw_uid.part3);

complete:
	skyrem_asyncreq_complete(async, req, rc);
	return 0;

complete_with_EPROTO:
	rc = -EPROTO;
	goto complete;
}

static int skyrem_subscription_token_parse(struct skyrem_async *async,
					   struct sky_async_req *req,
					   void *rsp_data, size_t rsp_len)
{
	struct sky_subscription_token *token = req->out.ptr;
	struct sky_subscription_token_rsp *rsp = rsp_data;
	size_t len;
	int rc;

	if (rsp_len <= sizeof(*rsp))
		goto complete_with_EPROTO;

	len = le16toh(rsp->len);
	if (!len)
		goto complete_with_EPROTO;

	if (rsp_len < sizeof(*rsp) + len)
		goto complete_with_EPROTO;

	if (len > sizeof(token->buf))
		goto complete_with_EPROTO;

	rc = -le16toh(rsp->hdr.error);
	if (rc)
		goto complete;

	token->len = len;
	memcpy(token->buf, rsp->buf, len);

complete:
	skyrem_asyncreq_complete(async, req, rc);
	return 0;

complete_with_EPROTO:
	rc = -EPROTO;
	goto complete;
}

static int skyrem_asyncreq_send(struct skyrem_async *async)
{
	struct sky_async_req *req;
	int rc;

	req = sky_asyncreq_pop(&async->async);

	switch (req->type) {
	case SKY_SINK_GET_DEV_PARAMS_REQ:
	case SKY_GET_DEV_PARAMS_REQ:
		rc = skyrem_paramsget_send(async, req);
		break;
	case SKY_SINK_SET_DEV_PARAMS_REQ:
	case SKY_SET_DEV_PARAMS_REQ:
		rc = skyrem_paramsset_send(async, req);
		break;
	case SKY_SINK_PASSTHRU_MSG_SEND_REQ:
		rc = skyrem_sink_passthru_msgsend_send(async, req);
		break;
	case SKY_RESUME_SCAN_REQ:
	case SKY_STOP_SCAN_REQ:
	case SKY_OPEN_DRONEPORT_REQ:
	case SKY_CLOSE_DRONEPORT_REQ:
	case SKY_DRONEPORT_STATE_REQ:
	case SKY_CHARGING_STATE_REQ:
	case SKY_RESET_DEV_REQ:
	case SKY_DEVS_LIST_REQ:
	case SKY_PEERINFO_REQ:
	case SKY_GPSDATA_REQ:
	case SKY_DRONEDETECT_REQ:
	case SKY_SINK_GET_INFO_REQ:
	case SKY_SINK_START_CHARGE_REQ:
	case SKY_SINK_STOP_CHARGE_REQ:
	case SKY_GET_SUBSCRIPTION_TOKEN_REQ:
	case SKY_SINK_PASSTHRU_MSG_RECV_REQ:
		rc = skyrem_genericreq_send(async, req);
		break;
	default:
		/* Consider fatal */
		sky_err("Unknown request: %d\n", req->type);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int skyrem_asyncreq_recv(struct skyrem_async *async)
{
	struct sky_async_req *req;
	size_t rsp_len;
	void *rsp;
	int rc;

	rc = skyrem_recvresp(async, &req, &rsp, &rsp_len);
	if (rc)
		return rc;

	if (!req) {
		/*
		 * Non-fatal error for the whole async context,
		 * response is malformed or unrecognized.
		 */
		return 0;
	}

	switch (req->type) {
	case SKY_SINK_GET_DEV_PARAMS_REQ:
	case SKY_GET_DEV_PARAMS_REQ:
		rc = skyrem_paramsget_parse(async, req, rsp, rsp_len);
		break;
	case SKY_SINK_SET_DEV_PARAMS_REQ:
	case SKY_SET_DEV_PARAMS_REQ:
	case SKY_RESUME_SCAN_REQ:
	case SKY_STOP_SCAN_REQ:
	case SKY_OPEN_DRONEPORT_REQ:
	case SKY_CLOSE_DRONEPORT_REQ:
	case SKY_RESET_DEV_REQ:
	case SKY_SINK_START_CHARGE_REQ:
	case SKY_SINK_STOP_CHARGE_REQ:
	case SKY_SINK_PASSTHRU_MSG_SEND_REQ:
		rc = skyrem_genericresp_parse(async, req, rsp, rsp_len);
		break;
	case SKY_CHARGING_STATE_REQ:
		rc = skyrem_chargingstate_parse(async, req, rsp, rsp_len);
		break;
	case SKY_DEVS_LIST_REQ:
		rc = skyrem_devslist_parse(async, req, rsp, rsp_len);
		break;
	case SKY_PEERINFO_REQ:
		rc = skyrem_peerinfo_parse(async, req, rsp, rsp_len);
		break;
	case SKY_DRONEPORT_STATE_REQ:
		rc = skyrem_droneport_state_parse(async, req, rsp, rsp_len);
		break;
	case SKY_GPSDATA_REQ:
		rc = skyrem_gpsdata_parse(async, req, rsp, rsp_len);
		break;
	case SKY_DRONEDETECT_REQ:
		rc = skyrem_dronedetect_parse(async, req, rsp, rsp_len);
		break;
	case SKY_SINK_GET_INFO_REQ:
		rc = skyrem_sinkinfo_parse(async, req, rsp, rsp_len);
		break;
	case SKY_GET_SUBSCRIPTION_TOKEN_REQ:
		rc = skyrem_subscription_token_parse(async, req, rsp, rsp_len);
		break;
	case SKY_SINK_PASSTHRU_MSG_RECV_REQ:
		rc = skyrem_sink_passthru_msgrecv_parse(async, req, rsp, rsp_len);
		break;
	default:
		/* Consider fatal */
		sky_err("Unknown request: %d\n", req->type);
		rc = -EINVAL;
		break;
	}
	free(rsp);

	return rc;
}

static int skyrem_asyncopen(const struct sky_conf *conf,
			    const struct sky_dev_ops *ops,
			    struct sky_async **async_)
{
	struct skyrem_async *async;
	int rc;

	async = calloc(1, sizeof(*async));
	if (!async)
		return -ENOMEM;

	sky_async_init(conf, ops, &async->async);
	rc = skyrem_connect(async);
	if (rc)
		return rc;

	async->reqs_root = RB_ROOT;
	*async_ = &async->async;

	return 0;
}

static void skyrem_asyncclose(struct sky_async *async_)
{
	struct skyrem_async *async;
	struct rb_node *node;

	async = container_of(async_, typeof(*async), async);
	while ((node = rb_first(&async->reqs_root))) {
		struct sky_async_req_node *req_node;
		struct sky_async_req *req;

		rb_erase(node, &async->reqs_root);
		req_node = container_of(node, typeof(*req_node), entry);
		req = req_node->req;
		free(req_node);
		skyrem_asyncreq_complete(async, req, -EIO);
	}
	while (!sky_async_empty(async_)) {
		struct sky_async_req *req;

		req = sky_asyncreq_pop(async_);
		skyrem_asyncreq_complete(async, req, -EIO);
	}

	zmq_close(async->zock);
	zmq_ctx_term(async->zctx);
	free(async);
}

static int skyrem_asyncfd(struct sky_async *async_)
{
	struct skyrem_async *async;
	int rc, fd;
	size_t len;

	async = container_of(async_, typeof(*async), async);
	len = sizeof(fd);
	rc = zmq_getsockopt(async->zock, ZMQ_FD, &fd, &len);
	if (rc)
		return -errno;

	return fd;
}

static bool async_want_send(struct skyrem_async *async)
{
	return !sky_async_empty(&async->async);
}

static bool async_want_recv(struct skyrem_async *async)
{
	return !RB_EMPTY_ROOT(&async->reqs_root);
}

static bool async_has_work(struct skyrem_async *async)
{
	return async_want_recv(async) || async_want_send(async);
}

static bool async_can_send(unsigned int events)
{
	return events & ZMQ_POLLOUT;
}

static bool async_can_recv(unsigned int events)
{
	return events & ZMQ_POLLIN;
}

static int async_events(struct skyrem_async *async, unsigned int *events)
{
	size_t len = sizeof(*events);
	int rc;

	rc = zmq_getsockopt(async->zock, ZMQ_EVENTS, events, &len);
	if (rc)
		return -errno;

	return 0;
}

static unsigned int async_completions_diff(struct skyrem_async *async,
					   unsigned long long completions)
{
	return (async->completions - completions);
}

static int skyrem_asyncexecute(struct sky_async *async_, bool wait)
{
	struct skyrem_async *async;

	unsigned long long completions;
	unsigned int events;
	int rc;

	async = container_of(async_, typeof(*async), async);
	completions = async->completions;
	do {
		rc = async_events(async, &events);
		if (rc < 0)
			return rc;

		if (async_want_send(async) && async_can_send(events)) {
			rc = skyrem_asyncreq_send(async);
			if (rc)
				return rc;

			continue;
		}
		if (async_want_recv(async) && async_can_recv(events)) {
			rc = skyrem_asyncreq_recv(async);
			if (rc)
				return rc;

			continue;
		}
		if (wait) {
			struct pollfd pfd;

			pfd.fd = skyrem_asyncfd(async_);
			if (pfd.fd < 0)
				return pfd.fd;

			pfd.events = POLLIN;
			rc = poll(&pfd, 1, -1);
			if (rc < 0 && errno != EINTR)
				return -errno;
		} else {
			break;
		}
	} while (async_has_work(async));

	return async_completions_diff(async, completions);
}

static int skyrem_discoverbroker(const struct sky_dev_ops *ops,
				 struct sky_brokerinfo *brokerinfo,
				 unsigned int timeout_ms)
{
	struct sky_discovery *discovery;
	zframe_t *frame = NULL;
	zactor_t *beacon;
	char *hostname;
	char *ip = NULL;
	int rc;

	beacon = zactor_new(zbeacon, NULL);
	if (beacon == NULL) {
		rc = -ENOMEM;
		goto err;
	}
	rc = zsock_send(beacon, "si", "CONFIGURE", SKY_DISCOVERY_PORT);
	if (rc) {
		sky_err("zsock_send(CONFIGURE)\n");
		rc = -ENONET;
		goto err;
	}
	hostname = zstr_recv(beacon);
	if (!hostname || !hostname[0]) {
		free(hostname);
		sky_err("hostname = zstr_recv()\n");
		rc = -ENONET;
		goto err;
	}
	free(hostname);
	zsock_set_rcvtimeo(beacon, timeout_ms);

	rc = zsock_send(beacon, "ss", "SUBSCRIBE", SKY_DISCOVERY_MAGIC);
	if (rc) {
		sky_err("zsock_send(SUBSCRIBE)\n");
		rc = -ENONET;
		goto err;
	}
	ip = zstr_recv(beacon);
	if (!ip) {
		rc = -ENONET;
		goto err;
	}
	frame = zframe_recv(beacon);
	if (!frame) {
		rc = -ENONET;
		goto err;
	}
	if (zframe_size(frame) != sizeof(*discovery)) {
		sky_err("Malformed discovery frame, %zd got %zd\n",
			sizeof(*discovery), zframe_size(frame));
		rc = -ENONET;
		goto err;
	}
	discovery = (struct sky_discovery *)zframe_data(frame);

	brokerinfo->af_family = AF_INET; /* currently IPv4 only */
	strncpy(brokerinfo->addr, ip, sizeof(brokerinfo->addr)-1);
	brokerinfo->proto_version = le16toh(discovery->proto_version);
	brokerinfo->servers_port = le16toh(discovery->servers_port);
	brokerinfo->sub_port = le16toh(discovery->sub_port);
	brokerinfo->clients_port = le16toh(discovery->clients_port);
	brokerinfo->pub_port = le16toh(discovery->pub_port);

	rc = 0;

err:
	zstr_free(&ip);
	zframe_destroy(&frame);
	zactor_destroy(&beacon);

	return rc;
}

static int skyrem_devopen(const struct sky_dev_desc *devdesc,
			  struct sky_dev **dev_)
{
	struct skyrem_dev *dev;

	dev = calloc(1, sizeof(*dev));
	if (!dev)
		return -ENOMEM;

	dev->zock.ctx = zmq_ctx_new();
	if (!dev->zock.ctx) {
		free(dev);
		return -ENOMEM;
	}
	*dev_ = &dev->dev;

	return 0;
}

static void skyrem_devclose(struct sky_dev *dev_)
{
	struct skyrem_dev *dev;

	dev = container_of(dev_, struct skyrem_dev, dev);
	zmq_ctx_term(dev->zock.ctx);
	free(dev);
}

static int
skyrem_subscribe(struct sky_dev *dev_, struct sky_subscription_token *token)
{
	struct sky_dev_desc *devdesc;
	struct skyrem_dev *dev;
	char zaddr[128];
	void *sub;
	int rc, opt;

	if (!token)
		return -EINVAL;
	if (!token->len)
		return -EINVAL;

	devdesc = &dev_->devdesc;
	dev = container_of(dev_, struct skyrem_dev, dev);

	rc = snprintf(zaddr, sizeof(zaddr), "tcp://%s:%d",
		      devdesc->conf.hostname,
		      devdesc->conf.subport);
	if (rc < 0 || rc >= sizeof(zaddr))
		return -EINVAL;

	sub = zmq_socket(dev->zock.ctx, ZMQ_SUB);
	if (!sub)
		return -ENOMEM;

	opt = DEFAULT_TIMEOUT;
	rc = zmq_setsockopt(sub, ZMQ_RCVTIMEO, &opt, sizeof(opt));
	if (rc != 0) {
		zmq_close(sub);
		return rc;
	}
	opt = DEFAULT_TIMEOUT;
	rc = zmq_setsockopt(sub, ZMQ_SNDTIMEO, &opt, sizeof(opt));
	if (rc != 0) {
		zmq_close(sub);
		return rc;
	}
	opt = 0;
	rc = zmq_setsockopt(sub, ZMQ_LINGER, &opt, sizeof(opt));
	if (rc != 0) {
		zmq_close(sub);
		return rc;
	}
	rc = zmq_setsockopt(sub, ZMQ_SUBSCRIBE, token->buf, token->len);
	if (rc != 0) {
		zmq_close(sub);
		return rc;
	}
	rc = zmq_connect(sub, zaddr);
	if (rc) {
		zmq_close(sub);
		return rc;
	}

	dev->zock.sub = sub;

	return 0;
}

static void skyrem_unsubscribe(struct sky_dev *dev_)
{
	struct skyrem_dev *dev;

	dev = container_of(dev_, struct skyrem_dev, dev);
	zmq_close(dev->zock.sub);
	dev->zock.sub = NULL;
}

static int skyrem_subscription_work(struct sky_dev *dev_,
				    struct sky_charging_state *state)
{
	struct sky_charging_state_rsp *untrusty_rsp;
	struct sky_charging_state_rsp rsp;
	struct skyrem_dev *dev;
	zframe_t *frame;
	zmsg_t *msg;
	int rc;

	dev = container_of(dev_, struct skyrem_dev, dev);

	/* TODO: we can sleep here forever */
	msg = zmsg_recv(dev->zock.sub);
	if (!msg)
		return -ECONNRESET;

	zmsg_first(msg);
	frame = zmsg_next(msg);

	if (zframe_size(frame) < sizeof(untrusty_rsp->hdr)) {
		/* Malformed response */
		zmsg_destroy(&msg);
		return -ECONNRESET;
	}

	untrusty_rsp = (void *)zframe_data(frame);
	rc = -le16toh(untrusty_rsp->hdr.error);
	if (rc) {
		zmsg_destroy(&msg);
		return rc;
	}

	memset(&rsp, 0, sizeof(rsp));
	memcpy(&rsp, untrusty_rsp, min(zframe_size(frame), sizeof(rsp)));
	zmsg_destroy(&msg);

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

	/* Indicate we sleep here */
	return 1;
}

static struct sky_dev_ops sky_remote_devops = {
	.contype           = SKY_REMOTE,
	.asyncopen         = skyrem_asyncopen,
	.asyncclose        = skyrem_asyncclose,
	.asyncexecute      = skyrem_asyncexecute,
	.asyncfd           = skyrem_asyncfd,
	.asyncreq_cancel   = skyrem_asyncreq_cancel,
	.discoverbroker    = skyrem_discoverbroker,
	.devopen           = skyrem_devopen,
	.devclose          = skyrem_devclose,
	.subscribe         = skyrem_subscribe,
	.unsubscribe       = skyrem_unsubscribe,
	.subscription_work = skyrem_subscription_work,
};
sky_register_devops(&sky_remote_devops);

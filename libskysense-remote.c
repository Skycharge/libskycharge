#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <assert.h>

#include <zmq.h>

#include "libskysense-pri.h"
#include "types.h"
#include "proto.h"

struct zocket {
	void *ctx;
	void *sub;
	void *pub;
};

struct skyrem_lib {
	struct sky_lib lib;
	struct zocket zock;
};

static int skyrem_send_recv(struct skyrem_lib *lib,
			    void *req, size_t req_len,
			    void **rsp)
{
	struct zocket *z = &lib->zock;
	void *zock, *buf;
	char zaddr[128];
	uint32_t timeo;
	size_t nb, sz;
	zmq_msg_t msg;
	int rc;

	rc = snprintf(zaddr, sizeof(zaddr), "tcp://%s:%d",
		      lib->lib.conf.remote.hostname,
		      lib->lib.conf.remote.cmdport);
	if (rc < 0 || rc >= sizeof(zaddr))
		return -EINVAL;

	zock = zmq_socket(z->ctx, ZMQ_REQ);
	if (zock == NULL)
		return -ENOMEM;

	timeo = DEFAULT_TIMEOUT;
	rc = zmq_setsockopt(zock, ZMQ_RCVTIMEO, &timeo, sizeof(timeo));
	if (rc != 0) {
		rc = -errno;
		goto out;
	}
	timeo = DEFAULT_TIMEOUT;
	rc = zmq_setsockopt(zock, ZMQ_SNDTIMEO, &timeo, sizeof(timeo));
	if (rc != 0) {
		rc = -errno;
		goto out;
	}
	rc = zmq_connect(zock, zaddr);
	if (rc != 0) {
		rc = -errno;
		goto out;
	}
	/* Send request */
	rc = zmq_send(zock, req, req_len, 0);
	if (rc < 0) {
		rc = -errno;
		goto out;
	}
	/* Recv response */
	zmq_msg_init(&msg);
	sz = nb = 0;
	buf = NULL;
	do {
		rc = zmq_msg_recv(&msg, zock, 0);
		if (rc < 0) {
			rc = -errno;
			zmq_msg_close(&msg);
			free(buf);
			goto out;
		}
		sz = zmq_msg_size(&msg);
		if (sz == 0)
			continue;
		buf = realloc(buf, sz);
		if (buf == NULL) {
			zmq_msg_close(&msg);
			rc = -ENOMEM;
			goto out;
		}
		memcpy(buf + nb, zmq_msg_data(&msg), sz);
		nb += sz;
	} while (zmq_msg_more(&msg));

	zmq_msg_close(&msg);
	*rsp = buf;
	rc = nb;

out:
	zmq_close(zock);

	return rc;
}

static int skyrem_complex_req_rsp(struct skyrem_lib *lib,
				  enum sky_proto_type req_type,
				  struct sky_req_hdr *req_, size_t req_len,
				  struct sky_rsp_hdr *rsp_, size_t rsp_len)
{
	struct sky_rsp_hdr *rsp = NULL;
	enum sky_proto_type rsp_type;
	int rc;

	if (!(req_type & 1)) {
		assert(0);
		return -EINVAL;
	}
	if (req_type == SKY_UNKNOWN_REQRSP || req_type >= SKY_LAST_REQRSP) {
		assert(0);
		return -EINVAL;
	}
	if (rsp_len < sizeof(*rsp_)) {
		assert(0);
		return -EINVAL;
	}

	/* Responses always follow requests */
	rsp_type = req_type + 1;
	req_->type = htole16(req_type);

	rc = skyrem_send_recv(lib, req_, req_len, (void **)&rsp);
	if (rc < 0)
		return rc;
	if (rc < sizeof(*rsp)) {
		/* Malformed response, too small */
		rc = -ECONNRESET;
		goto out;
	}
	if (rc > rsp_len) {
		/* Malformed response, too big */
		rc = -ECONNRESET;
		goto out;
	}
	if (le16toh(rsp->type) != rsp_type) {
		/* Malformed response, incorrect response type */
		rc = -ECONNRESET;
		goto out;
	}
	memcpy(rsp_, rsp, rc);

out:
	free(rsp);

	return rc;
}

static int skyrem_simple_req_rsp(struct skyrem_lib *lib,
				 enum sky_proto_type req_type,
				 struct sky_req_hdr *req,
				 size_t req_len)
{
	struct sky_rsp_hdr rsp;
	int rc;

	rc = skyrem_complex_req_rsp(lib, req_type, req, req_len,
				    &rsp, sizeof(rsp));
	if (rc < 0)
		return rc;

	return le16toh(rsp.error);
}

static int skyrem_libopen(const struct sky_lib_conf *conf,
			  struct sky_lib **lib_)
{
	struct skyrem_lib *lib;

	lib = calloc(1, sizeof(*lib));
	if (!lib)
		return -ENOMEM;

	lib->zock.ctx = zmq_ctx_new();
	if (!lib->zock.ctx) {
		free(lib);
		return -ENOMEM;
	}
	*lib_ = &lib->lib;

	return 0;
}

static void skyrem_libclose(struct sky_lib *lib_)
{
	struct skyrem_lib *lib;

	lib = container_of(lib_, struct skyrem_lib, lib);
	zmq_ctx_term(lib->zock.ctx);
	free(lib);
}

static int skyrem_devinfo(struct sky_lib *lib_, struct sky_dev *dev)
{
	struct sky_dev_info_rsp rsp;
	struct sky_req_hdr req;
	struct skyrem_lib *lib;
	int rc;

	lib = container_of(lib_, struct skyrem_lib, lib);

	rc = skyrem_complex_req_rsp(lib, SKY_DEV_INFO_REQ, &req, sizeof(req),
				    &rsp.hdr, sizeof(rsp));
	if (rc < 0)
		return rc;

	if (rc != sizeof(rsp)) {
		/* Malformed response */
		return -ECONNRESET;
	}
	rc = le16toh(rsp.hdr.error);
	if (rc)
		return rc;

	dev->dev_type = le16toh(rsp.dev_type);
	memcpy(dev->portname, rsp.portname, sizeof(rsp.portname));

	return 0;
}

static int skyrem_confget(struct sky_lib *lib_, struct sky_dev_conf *conf)
{
	struct {
		struct sky_get_dev_params_rsp rsp;
		typeof(((struct sky_get_dev_params_rsp *)NULL)->dev_params[0])
				dev_params[SKY_NUM_DEVPARAM];
	} rsp_uni;

	struct sky_get_dev_params_req req;
	struct skyrem_lib *lib;
	int rc, sz, i, ind;

	if (conf->dev_params_bits == 0)
		return -EINVAL;

	lib = container_of(lib_, struct skyrem_lib, lib);
	req.dev_params_bits = htole32(conf->dev_params_bits);

	sz = skyrem_complex_req_rsp(lib, SKY_GET_DEV_PARAMS_REQ,
				    &req.hdr, sizeof(req),
				    &rsp_uni.rsp.hdr, sizeof(rsp_uni));
	if (sz < 0)
		return sz;

	rc = le16toh(rsp_uni.rsp.hdr.error);
	if (rc)
		return rc;

	BUILD_BUG_ON(sizeof(conf->dev_params_bits) * 8 <
		     SKY_NUM_DEVPARAM);

	for (i = 0, ind = 0; i < SKY_NUM_DEVPARAM; i++) {
		if (conf->dev_params_bits & (1<<i)) {
			if (sz < (sizeof(rsp_uni.rsp) +
				  sizeof(rsp_uni.dev_params[0]) * (ind + 1)))
				/* Malformed response */
				return -ECONNRESET;
			conf->dev_params[i] = le32toh(rsp_uni.dev_params[ind++]);
		} else
			conf->dev_params[i] = 0;
	}

	return 0;
}

static int skyrem_confset(struct sky_lib *lib_, struct sky_dev_conf *conf)
{
	struct {
		struct sky_set_dev_params_req req;
		typeof(((struct sky_set_dev_params_req *)NULL)->dev_params[0])
				dev_params[SKY_NUM_DEVPARAM];
	} req_uni;

	struct skyrem_lib *lib;
	int sz, i, ind;

	if (conf->dev_params_bits == 0)
		return -EINVAL;

	lib = container_of(lib_, struct skyrem_lib, lib);
	req_uni.req.dev_params_bits = htole32(conf->dev_params_bits);

	BUILD_BUG_ON(sizeof(conf->dev_params_bits) * 8 <
		     SKY_NUM_DEVPARAM);

	for (i = 0, ind = 0; i < SKY_NUM_DEVPARAM; i++) {
		if (conf->dev_params_bits & (1<<i))
			req_uni.dev_params[ind++] =
				htole32(conf->dev_params[i]);
	}
	sz = sizeof(req_uni.req) + sizeof(req_uni.dev_params[0]) * ind;

	return skyrem_simple_req_rsp(lib, SKY_SET_DEV_PARAMS_REQ,
				     &req_uni.req.hdr, sz);
}

static int skyrem_chargingstate(struct sky_lib *lib_,
				struct sky_charging_state *state)
{
	struct sky_charging_state_rsp rsp;
	struct sky_req_hdr req;
	struct skyrem_lib *lib;
	int rc;

	lib = container_of(lib_, struct skyrem_lib, lib);

	rc = skyrem_complex_req_rsp(lib, SKY_CHARGING_STATE_REQ,
				    &req, sizeof(req),
				    &rsp.hdr, sizeof(rsp));
	if (rc < 0)
		return rc;

	if (rc != sizeof(rsp)) {
		/* Malformed response */
		return -ECONNRESET;
	}
	rc = le16toh(rsp.hdr.error);
	if (rc)
		return rc;

	state->dev_hw_state = le16toh(rsp.dev_hw_state);
	state->voltage = le16toh(rsp.voltage);
	state->current = le16toh(rsp.current);

	return 0;
}

static int skyrem_subscribe(struct sky_lib *lib_)
{
	struct skyrem_lib *lib;
	char zaddr[128];
	uint32_t timeo;
	void *sub;
	int rc;

	lib = container_of(lib_, struct skyrem_lib, lib);

	rc = snprintf(zaddr, sizeof(zaddr), "tcp://%s:%d",
		      lib->lib.conf.remote.hostname,
		      lib->lib.conf.remote.subport);
	if (rc < 0 || rc >= sizeof(zaddr))
		return -EINVAL;

	sub = zmq_socket(lib->zock.ctx, ZMQ_SUB);
	if (!sub)
		return -ENOMEM;

	timeo = DEFAULT_TIMEOUT;
	rc = zmq_setsockopt(sub, ZMQ_RCVTIMEO, &timeo, sizeof(timeo));
	if (rc != 0) {
		zmq_close(sub);
		return rc;
	}
	timeo = DEFAULT_TIMEOUT;
	rc = zmq_setsockopt(sub, ZMQ_SNDTIMEO, &timeo, sizeof(timeo));
	if (rc != 0) {
		zmq_close(sub);
		return rc;
	}
	rc = zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "", 0);
	if (rc != 0) {
		zmq_close(sub);
		return rc;
	}
	rc = zmq_connect(sub, zaddr);
	if (rc) {
		zmq_close(sub);
		return rc;
	}

	lib->zock.sub = sub;

	return 0;
}

static void skyrem_unsubscribe(struct sky_lib *lib_)
{
	struct skyrem_lib *lib;

	lib = container_of(lib_, struct skyrem_lib, lib);
	zmq_close(lib->zock.sub);
	lib->zock.sub = NULL;
}

static int skyrem_subscription_work(struct sky_lib *lib_,
				    struct sky_charging_state *state)
{
	struct sky_charging_state_rsp rsp;
	struct skyrem_lib *lib;
	int rc;

	lib = container_of(lib_, struct skyrem_lib, lib);

	/* TODO: we can sleep here forever */
	rc = zmq_recv(lib->zock.sub, &rsp, sizeof(rsp), 0);
	if (rc < 0)
		return -errno;
	if (rc != sizeof(rsp)) {
		/* Malformed response */
		return -ECONNRESET;
	}
	rc = le16toh(rsp.hdr.error);
	if (rc)
		return rc;

	state->dev_hw_state = le16toh(rsp.dev_hw_state);
	state->voltage = le16toh(rsp.voltage);
	state->current = le16toh(rsp.current);

	return 0;
}

static int skyrem_reset(struct sky_lib *lib_)
{
	struct sky_req_hdr req;
	struct skyrem_lib *lib;

	lib = container_of(lib_, struct skyrem_lib, lib);

	return skyrem_simple_req_rsp(lib, SKY_RESET_DEV_REQ,
				     &req, sizeof(req));
}

static int skyrem_autoscan(struct sky_lib *lib_, unsigned autoscan)
{
	struct sky_set_autoscan_req req;
	struct skyrem_lib *lib;

	lib = container_of(lib_, struct skyrem_lib, lib);
	req.autoscan = htole16(autoscan);

	return skyrem_simple_req_rsp(lib, SKY_SET_AUTOSCAN_REQ,
				     &req.hdr, sizeof(req));
}

static int skyrem_chargestart(struct sky_lib *lib_)
{
	struct sky_req_hdr req;
	struct skyrem_lib *lib;

	lib = container_of(lib_, struct skyrem_lib, lib);

	return skyrem_simple_req_rsp(lib, SKY_START_CHARGE_REQ,
				     &req, sizeof(req));
}

static int skyrem_chargestop(struct sky_lib *lib_)
{
	struct sky_req_hdr req;
	struct skyrem_lib *lib;

	lib = container_of(lib_, struct skyrem_lib, lib);

	return skyrem_simple_req_rsp(lib, SKY_STOP_CHARGE_REQ,
				     &req, sizeof(req));
}

static int skyrem_coveropen(struct sky_lib *lib_)
{
	struct sky_req_hdr req;
	struct skyrem_lib *lib;

	lib = container_of(lib_, struct skyrem_lib, lib);

	return skyrem_simple_req_rsp(lib, SKY_OPEN_COVER_REQ,
				     &req, sizeof(req));
}

static int skyrem_coverclose(struct sky_lib *lib_)
{
	struct sky_req_hdr req;
	struct skyrem_lib *lib;

	lib = container_of(lib_, struct skyrem_lib, lib);

	return skyrem_simple_req_rsp(lib, SKY_CLOSE_COVER_REQ,
				     &req, sizeof(req));
}

struct sky_lib_ops sky_remote_lib_ops = {
	.libopen = skyrem_libopen,
	.libclose = skyrem_libclose,
	.devinfo = skyrem_devinfo,
	.confget = skyrem_confget,
	.confset = skyrem_confset,
	.chargingstate = skyrem_chargingstate,
	.subscribe = skyrem_subscribe,
	.unsubscribe = skyrem_unsubscribe,
	.subscription_work = skyrem_subscription_work,
	.reset = skyrem_reset,
	.autoscan = skyrem_autoscan,
	.chargestart = skyrem_chargestart,
	.chargestop = skyrem_chargestop,
	.coveropen = skyrem_coveropen,
	.coverclose = skyrem_coverclose
};

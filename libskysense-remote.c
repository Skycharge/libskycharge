#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <assert.h>

#include <zmq.h>

#include "libskysense-pri.h"
#include "types.h"
#include "skyproto.h"

struct zocket {
	void *ctx;
	void *sub;
};

struct skyrem_dev {
	struct sky_dev dev;
	struct zocket zock;
};

static int skyrem_send_recv(void *zctx, const struct sky_dev_conf *conf,
			    void *req, size_t req_len, void **rsp)
{
	void *zock, *buf;
	char zaddr[128];
	uint32_t timeo;
	size_t nb, sz;
	zmq_msg_t msg;
	int rc;

	rc = snprintf(zaddr, sizeof(zaddr), "tcp://%s:%d",
		      conf->remote.hostname,
		      conf->remote.cmdport);
	if (rc < 0 || rc >= sizeof(zaddr))
		return -EINVAL;

	zock = zmq_socket(zctx, ZMQ_REQ);
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

static int __skyrem_complex_req_rsp(void *zctx, const struct sky_dev_conf *conf,
				    enum sky_proto_type req_type,
				    struct sky_req_hdr *req_, size_t req_len,
				    struct sky_rsp_hdr **rsp)
{
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

	/* Responses always follow requests */
	rsp_type = req_type + 1;
	req_->type = htole16(req_type);

	/* Make compiler happy */
	*rsp = NULL;
	rc = skyrem_send_recv(zctx, conf, req_, req_len, (void **)rsp);
	if (rc < 0)
		return rc;
	if (rc < sizeof(**rsp)) {
		/* Malformed response, too small */
		rc = -ECONNRESET;
		goto err;
	}
	if (le16toh((*rsp)->type) != rsp_type) {
		/* Malformed response, incorrect response type */
		rc = -ECONNRESET;
		goto err;
	}

	return rc;

err:
	free(*rsp);

	return rc;
}

static int skyrem_complex_req_rsp(struct skyrem_dev *dev,
				  enum sky_proto_type req_type,
				  struct sky_req_hdr *req_, size_t req_len,
				  struct sky_rsp_hdr *rsp_, size_t rsp_len)
{
	struct sky_rsp_hdr *rsp;
	int rc;

	if (rsp_len < sizeof(*rsp_)) {
		assert(0);
		return -EINVAL;
	}
	rc = __skyrem_complex_req_rsp(dev->zock.ctx, &dev->dev.devdesc.conf,
				      req_type, req_, req_len, &rsp);
	if (rc < 0)
		return rc;

	if (rc > rsp_len) {
		/* Malformed response, too big */
		rc = -ECONNRESET;
		goto out;
	}
	memcpy(rsp_, rsp, rc);

out:
	free(rsp);

	return rc;
}

static int skyrem_simple_req_rsp(struct skyrem_dev *dev,
				 enum sky_proto_type req_type,
				 struct sky_req_hdr *req,
				 size_t req_len)
{
	struct sky_rsp_hdr rsp;
	int rc;

	rc = skyrem_complex_req_rsp(dev, req_type, req, req_len,
				    &rsp, sizeof(rsp));
	if (rc < 0)
		return rc;

	return -le16toh(rsp.error);
}

static int skyrem_devslist(const struct sky_dev_ops *ops,
			   const struct sky_dev_conf *conf,
			   struct sky_dev_desc **out)
{
	struct sky_dev_desc *head = NULL, *tail = NULL;
	struct sky_devs_list_rsp *rsp;
	struct sky_rsp_hdr *rsp_hdr;
	struct sky_req_hdr req;
	size_t num_devs, len;
	void *zctx;
	int rc;

	zctx = zmq_ctx_new();
	if (!zctx)
		return -ENOMEM;

	rc = __skyrem_complex_req_rsp(zctx, conf, SKY_DEVS_LIST_REQ,
				      &req, sizeof(req), &rsp_hdr);
	if (rc < 0)
		goto out;

	len = rc;
	rc = -le16toh(rsp_hdr->error);
	if (rc)
		goto out;

	rsp = (struct sky_devs_list_rsp *)rsp_hdr;
	num_devs = le16toh(rsp->num_devs);

	/* Sanity checks */
	if (len != sizeof(*rsp) + sizeof(rsp->info[0]) * num_devs) {
		/* Malformed response */
		rc = -ECONNRESET;
		goto out;
	}
	if (num_devs) {
		struct sky_dev_desc *devdesc;
		int i;

		for (i = 0; i < num_devs; i++) {
			struct sky_dev_info *info = &rsp->info[i];

			devdesc = calloc(1, sizeof(*devdesc));
			if (!devdesc) {
				rc = -ENOMEM;
				sky_devsfree(head);
				goto out;
			}
			devdesc->dev_type = le16toh(info->dev_type);
			devdesc->conf = *conf;
			devdesc->opaque_ops = ops;
			memcpy(devdesc->portname, info->portname,
			       sizeof(info->portname));

			devdesc->next = head;
			head = devdesc;
			if (tail == NULL)
				tail = devdesc;
		}
	}
	rc = 0;
	if (head) {
		tail->next = *out;
		*out = head;
	}
out:
	zmq_ctx_term(zctx);

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

static int skyrem_paramsget(struct sky_dev *dev_, struct sky_dev_params *params)
{
	struct {
		struct sky_get_dev_params_rsp rsp;
		typeof(((struct sky_get_dev_params_rsp *)NULL)->dev_params[0])
				dev_params[SKY_NUM_DEVPARAM];
	} rsp_uni;

	struct sky_get_dev_params_req req;
	struct skyrem_dev *dev;
	int rc, sz, i, ind;

	if (params->dev_params_bits == 0)
		return -EINVAL;

	dev = container_of(dev_, struct skyrem_dev, dev);
	req.dev_params_bits = htole32(params->dev_params_bits);

	sz = skyrem_complex_req_rsp(dev, SKY_GET_DEV_PARAMS_REQ,
				    &req.hdr, sizeof(req),
				    &rsp_uni.rsp.hdr, sizeof(rsp_uni));
	if (sz < 0)
		return sz;

	rc = -le16toh(rsp_uni.rsp.hdr.error);
	if (rc)
		return rc;

	BUILD_BUG_ON(sizeof(params->dev_params_bits) * 8 <
		     SKY_NUM_DEVPARAM);

	for (i = 0, ind = 0; i < SKY_NUM_DEVPARAM; i++) {
		if (params->dev_params_bits & (1<<i)) {
			if (sz < (sizeof(rsp_uni.rsp) +
				  sizeof(rsp_uni.dev_params[0]) * (ind + 1)))
				/* Malformed response */
				return -ECONNRESET;
			params->dev_params[i] = le32toh(rsp_uni.dev_params[ind++]);
		} else
			params->dev_params[i] = 0;
	}

	return 0;
}

static int skyrem_paramsset(struct sky_dev *dev_, struct sky_dev_params *params)
{
	struct {
		struct sky_set_dev_params_req req;
		typeof(((struct sky_set_dev_params_req *)NULL)->dev_params[0])
				dev_params[SKY_NUM_DEVPARAM];
	} req_uni;

	struct skyrem_dev *dev;
	int sz, i, ind;

	if (params->dev_params_bits == 0)
		return -EINVAL;

	dev = container_of(dev_, struct skyrem_dev, dev);
	req_uni.req.dev_params_bits = htole32(params->dev_params_bits);

	BUILD_BUG_ON(sizeof(params->dev_params_bits) * 8 <
		     SKY_NUM_DEVPARAM);

	for (i = 0, ind = 0; i < SKY_NUM_DEVPARAM; i++) {
		if (params->dev_params_bits & (1<<i))
			req_uni.dev_params[ind++] =
				htole32(params->dev_params[i]);
	}
	sz = sizeof(req_uni.req) + sizeof(req_uni.dev_params[0]) * ind;

	return skyrem_simple_req_rsp(dev, SKY_SET_DEV_PARAMS_REQ,
				     &req_uni.req.hdr, sz);
}

static int skyrem_chargingstate(struct sky_dev *dev_,
				struct sky_charging_state *state)
{
	struct sky_charging_state_rsp rsp;
	struct sky_req_hdr req;
	struct skyrem_dev *dev;
	int rc;

	dev = container_of(dev_, struct skyrem_dev, dev);

	rc = skyrem_complex_req_rsp(dev, SKY_CHARGING_STATE_REQ,
				    &req, sizeof(req),
				    &rsp.hdr, sizeof(rsp));
	if (rc < 0)
		return rc;

	if (rc != sizeof(rsp)) {
		/* Malformed response */
		return -ECONNRESET;
	}
	rc = -le16toh(rsp.hdr.error);
	if (rc)
		return rc;

	state->dev_hw_state = le16toh(rsp.dev_hw_state);
	state->voltage = le16toh(rsp.voltage);
	state->current = le16toh(rsp.current);

	return 0;
}

static int skyrem_subscribe(struct sky_dev *dev_)
{
	struct skyrem_dev *dev;
	char zaddr[128];
	uint32_t timeo;
	void *sub;
	int rc;

	dev = container_of(dev_, struct skyrem_dev, dev);

	rc = snprintf(zaddr, sizeof(zaddr), "tcp://%s:%d",
		      dev->dev.devdesc.conf.remote.hostname,
		      dev->dev.devdesc.conf.remote.subport);
	if (rc < 0 || rc >= sizeof(zaddr))
		return -EINVAL;

	sub = zmq_socket(dev->zock.ctx, ZMQ_SUB);
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
	struct sky_charging_state_rsp rsp;
	struct skyrem_dev *dev;
	int rc;

	dev = container_of(dev_, struct skyrem_dev, dev);

	/* TODO: we can sleep here forever */
	rc = zmq_recv(dev->zock.sub, &rsp, sizeof(rsp), 0);
	if (rc < 0)
		return -errno;
	if (rc != sizeof(rsp)) {
		/* Malformed response */
		return -ECONNRESET;
	}
	rc = -le16toh(rsp.hdr.error);
	if (rc)
		return rc;

	state->dev_hw_state = le16toh(rsp.dev_hw_state);
	state->voltage = le16toh(rsp.voltage);
	state->current = le16toh(rsp.current);

	return 0;
}

static int skyrem_reset(struct sky_dev *dev_)
{
	struct sky_req_hdr req;
	struct skyrem_dev *dev;

	dev = container_of(dev_, struct skyrem_dev, dev);

	return skyrem_simple_req_rsp(dev, SKY_RESET_DEV_REQ,
				     &req, sizeof(req));
}

static int skyrem_chargestart(struct sky_dev *dev_)
{
	struct sky_req_hdr req;
	struct skyrem_dev *dev;

	dev = container_of(dev_, struct skyrem_dev, dev);

	return skyrem_simple_req_rsp(dev, SKY_START_CHARGE_REQ,
				     &req, sizeof(req));
}

static int skyrem_chargestop(struct sky_dev *dev_)
{
	struct sky_req_hdr req;
	struct skyrem_dev *dev;

	dev = container_of(dev_, struct skyrem_dev, dev);

	return skyrem_simple_req_rsp(dev, SKY_STOP_CHARGE_REQ,
				     &req, sizeof(req));
}

static int skyrem_coveropen(struct sky_dev *dev_)
{
	struct sky_req_hdr req;
	struct skyrem_dev *dev;

	dev = container_of(dev_, struct skyrem_dev, dev);

	return skyrem_simple_req_rsp(dev, SKY_OPEN_COVER_REQ,
				     &req, sizeof(req));
}

static int skyrem_coverclose(struct sky_dev *dev_)
{
	struct sky_req_hdr req;
	struct skyrem_dev *dev;

	dev = container_of(dev_, struct skyrem_dev, dev);

	return skyrem_simple_req_rsp(dev, SKY_CLOSE_COVER_REQ,
				     &req, sizeof(req));
}

static struct sky_dev_ops sky_remote_devops = {
	.contype = SKY_REMOTE,
	.devslist = skyrem_devslist,
	.devopen = skyrem_devopen,
	.devclose = skyrem_devclose,
	.paramsget = skyrem_paramsget,
	.paramsset = skyrem_paramsset,
	.chargingstate = skyrem_chargingstate,
	.subscribe = skyrem_subscribe,
	.unsubscribe = skyrem_unsubscribe,
	.subscription_work = skyrem_subscription_work,
	.reset = skyrem_reset,
	.chargestart = skyrem_chargestart,
	.chargestop = skyrem_chargestop,
	.coveropen = skyrem_coveropen,
	.coverclose = skyrem_coverclose
};
sky_register_devops(&sky_remote_devops);

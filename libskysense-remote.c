#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <assert.h>

#include <zmq.h>
#include <czmq.h>

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

static int skyrem_send_recv(void *zctx,
			    const struct sky_dev_conf *conf,
			    const struct sky_dev_desc *devdesc,
			    void *req, size_t req_len, void **rsp)
{
	void *zock;
	char zaddr[128];
	uint32_t timeo;
	zmsg_t *msg = NULL;
	zframe_t *frame;
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
	msg = zmsg_new();
	if (msg == NULL) {
		rc = -ENOMEM;
		goto out;
	}
	rc = zmsg_addmem(msg, req, req_len);
	if (!rc) {
		if (devdesc) {
			/* DEVPORT frame follows request frame */
			rc = zmsg_addstr(msg, devdesc->portname);
			/* DEVUUID frame is the last one */
			rc |= zmsg_addmem(msg, devdesc->dev_uuid,
					  sizeof(devdesc->dev_uuid));
		} else {
			/* USRUUID frame is the last one */
			rc = zmsg_addmem(msg, conf->remote.usruuid,
					 sizeof(conf->remote.usruuid));
		}
	}
	if (rc != 0) {
		rc = -ENOMEM;
		goto out;
	}
	/* Send request */
	rc = zmsg_send(&msg, zock);
	if (rc != 0) {
		rc = -errno;
		goto out;
	}
	/* Recv response */
	msg = zmsg_recv(zock);
	frame = msg ? zmsg_first(msg) : NULL;
	if (frame == NULL) {
		rc = -EIO;
		goto out;
	}
	rc = zframe_size(frame);
	*rsp = malloc(rc);
	if (*rsp == NULL) {
		rc = -ENOMEM;
		goto out;
	}
	memcpy(*rsp, zframe_data(frame), rc);

out:
	zmsg_destroy(&msg);
	zmq_close(zock);

	return rc;
}

static int __skyrem_complex_req_rsp(void *zctx,
				    const struct sky_dev_conf *conf,
				    const struct sky_dev_desc *devdesc,
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
	rc = skyrem_send_recv(zctx, conf, devdesc, req_, req_len, (void **)rsp);
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
	*rsp = NULL;

	return rc;
}

static int skyrem_complex_req_rsp(struct skyrem_dev *dev,
				  enum sky_proto_type req_type,
				  struct sky_req_hdr *req_, size_t req_len,
				  struct sky_rsp_hdr *rsp_, size_t rsp_len)
{
	struct sky_dev_desc *devdesc = &dev->dev.devdesc;
	struct sky_rsp_hdr *rsp;
	int rc;

	if (rsp_len < sizeof(*rsp_)) {
		assert(0);
		return -EINVAL;
	}
	rc = __skyrem_complex_req_rsp(dev->zock.ctx, &devdesc->conf,
				      devdesc, req_type, req_,
				      req_len, &rsp);
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

static int skyrem_peerinfo(const struct sky_dev_ops *ops,
			   const struct sky_dev_conf *conf,
			   struct sky_peerinfo *peerinfo)
{
	struct sky_peerinfo_rsp *rsp;
	struct sky_rsp_hdr *rsp_hdr;
	struct sky_req_hdr req;
	void *zctx;
	int rc;

	zctx = zmq_ctx_new();
	if (!zctx)
		return -ENOMEM;

	rc = __skyrem_complex_req_rsp(zctx, conf, NULL, SKY_PEERINFO_REQ,
				      &req, sizeof(req), &rsp_hdr);
	if (rc < 0)
		goto out;

	rc = -le16toh(rsp_hdr->error);
	if (rc)
		goto out;

	rsp = (struct sky_peerinfo_rsp *)rsp_hdr;
	peerinfo->server_version = le32toh(rsp->server_version);
	peerinfo->proto_version  = le16toh(rsp->proto_version);

out:
	zmq_ctx_term(zctx);

	return rc;
}

static int skyrem_devslist(const struct sky_dev_ops *ops,
			   const struct sky_dev_conf *conf,
			   struct sky_dev_desc **out)
{
	struct sky_dev_desc *head = NULL, *tail = NULL;
	struct sky_devs_list_rsp *rsp = NULL;
	struct sky_rsp_hdr *rsp_hdr;
	struct sky_req_hdr req;
	size_t num_devs, len;
	void *zctx;
	int rc;

	zctx = zmq_ctx_new();
	if (!zctx)
		return -ENOMEM;

	rc = __skyrem_complex_req_rsp(zctx, conf, NULL, SKY_DEVS_LIST_REQ,
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
			memcpy(devdesc->dev_name, info->dev_name,
			       sizeof(devdesc->dev_name));
			memcpy(devdesc->dev_uuid, info->dev_uuid,
			       sizeof(devdesc->dev_uuid));
			devdesc->firmware_version =
				le32toh(info->firmware_version);
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
	free(rsp);
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

static int skyrem_paramsset(struct sky_dev *dev_,
			    const struct sky_dev_params *params)
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
	state->bms.charge_perc = le16toh(rsp.bms.charge_perc);
	state->bms.charge_time = le16toh(rsp.bms.charge_time);

	return 0;
}

static int skyrem_subscribe(struct sky_dev *dev_)
{
	struct sky_dev_desc *devdesc;
	struct skyrem_dev *dev;
	char topic[128];
	char zaddr[128];
	uint32_t timeo;
	size_t len;
	void *sub;
	int rc;

	BUILD_BUG_ON(sizeof(devdesc->dev_uuid) + sizeof(devdesc->portname) >
		     sizeof(topic));

	devdesc = &dev_->devdesc;
	dev = container_of(dev_, struct skyrem_dev, dev);

	rc = snprintf(zaddr, sizeof(zaddr), "tcp://%s:%d",
		      devdesc->conf.remote.hostname,
		      devdesc->conf.remote.subport);
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
	memcpy(topic, devdesc->dev_uuid, sizeof(devdesc->dev_uuid));
	len = strlen(devdesc->portname);
	memcpy(topic + sizeof(devdesc->dev_uuid), devdesc->portname, len);
	len += sizeof(devdesc->dev_uuid);
	rc = zmq_setsockopt(sub, ZMQ_SUBSCRIBE, topic, len);
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
	struct sky_charging_state_rsp *rsp;
	struct skyrem_dev *dev;
	zframe_t *frame;
	zmsg_t *msg;
	int rc;

	dev = container_of(dev_, struct skyrem_dev, dev);

	/* TODO: we can sleep here forever */
	msg = zmsg_recv(dev->zock.sub);
	if (!msg)
		return -ECONNRESET;
	if (zmsg_size(msg) != 2)
		return -ECONNRESET;

	zmsg_first(msg);
	frame = zmsg_next(msg);

	if (zframe_size(frame) != sizeof(*rsp))
		/* Malformed response */
		return -ECONNRESET;

	rsp = (void *)zframe_data(frame);
	rc = -le16toh(rsp->hdr.error);
	if (rc)
		return rc;

	state->dev_hw_state = le16toh(rsp->dev_hw_state);
	state->voltage = le16toh(rsp->voltage);
	state->current = le16toh(rsp->current);
	state->bms.charge_perc = le16toh(rsp->bms.charge_perc);
	state->bms.charge_time = le16toh(rsp->bms.charge_time);

	/* Indicate we sleep here */
	return 1;
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

static int skyrem_gpsdata(struct sky_dev *dev_, struct sky_gpsdata *gpsdata)
{
	struct sky_gpsdata_rsp rsp;
	struct sky_req_hdr req;
	struct skyrem_dev *dev;
	int rc;

	dev = container_of(dev_, struct skyrem_dev, dev);

	rc = skyrem_complex_req_rsp(dev, SKY_GPSDATA_REQ,
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

	if (le32toh(rsp.status) > SKY_GPS_STATUS_DGPS_FIX)
		return -ECONNRESET;

	if (le32toh(rsp.fix.mode) > SKY_GPS_MODE_3D)
		return -ECONNRESET;

	memset(gpsdata, 0, sizeof(*gpsdata));
	gpsdata->status = le32toh(rsp.status);
	gpsdata->satellites_used = le32toh(rsp.satellites_used);

#define LLU2D(i)  ({ uint64_t llu = (i); double *dp = (double *)&llu; *dp; })

	gpsdata->dop.xdop = LLU2D(le64toh(rsp.dop.xdop));
	gpsdata->dop.ydop = LLU2D(le64toh(rsp.dop.ydop));
	gpsdata->dop.pdop = LLU2D(le64toh(rsp.dop.pdop));
	gpsdata->dop.hdop = LLU2D(le64toh(rsp.dop.hdop));
	gpsdata->dop.vdop = LLU2D(le64toh(rsp.dop.vdop));
	gpsdata->dop.tdop = LLU2D(le64toh(rsp.dop.tdop));
	gpsdata->dop.gdop = LLU2D(le64toh(rsp.dop.gdop));

	gpsdata->fix.mode = le32toh(rsp.fix.mode);
	gpsdata->fix.time = LLU2D(le64toh(rsp.fix.time));
	gpsdata->fix.latitude  = LLU2D(le64toh(rsp.fix.latitude));
	gpsdata->fix.longitude = LLU2D(le64toh(rsp.fix.longitude));
	gpsdata->fix.altitude  = LLU2D(le64toh(rsp.fix.altitude));

	return 0;
}

static int skyrem_dronedetect(struct sky_dev *dev_,
			      enum sky_drone_status *status)
{
	struct sky_dronedetect_rsp rsp;
	struct sky_req_hdr req;
	struct skyrem_dev *dev;
	int rc;

	dev = container_of(dev_, struct skyrem_dev, dev);

	rc = skyrem_complex_req_rsp(dev, SKY_DRONEDETECT_REQ,
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

	*status = le16toh(rsp.status);

	return 0;
}

static struct sky_dev_ops sky_remote_devops = {
	.contype = SKY_REMOTE,
	.discoverbroker = skyrem_discoverbroker,
	.peerinfo = skyrem_peerinfo,
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
	.coverclose = skyrem_coverclose,
	.gpsdata = skyrem_gpsdata,
	.dronedetect = skyrem_dronedetect,
};
sky_register_devops(&sky_remote_devops);

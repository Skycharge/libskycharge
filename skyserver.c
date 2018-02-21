#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <endian.h>
#include <limits.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <uuid/uuid.h>

#include <zmq.h>
#include <czmq.h>

#include "skyserver-cmd.h"
#include "libskysense.h"
#include "skyproto.h"
#include "version.h"
#include "types.h"

struct zocket {
	void *ctx;
	void *pub;
	void *router;
};

struct sky_server;

struct sky_server_dev {
	struct sky_server *serv;
	struct sky_dev_desc *devdesc;
	struct sky_dev *dev;
};

struct sky_server {
	bool exit;
	uuid_t uuid;
	struct cli cli;
	struct zocket zock;
	struct sky_server_dev *devs;
	struct sky_dev_desc *devhead;
};

static int sky_pidfile_create(const char *pidfile, int pid)
{
	int rc, fd, n;
	char buf[16];

	fd = open(pidfile, O_CREAT | O_EXCL | O_TRUNC | O_WRONLY, 0600);
	if (fd < 0) {
		sky_err("sky_pidfile_create: failed open(%s) errno=%d\n",
			pidfile, errno);

		return -1;
	}
	n = snprintf(buf, sizeof(buf), "%u\n", pid);
	rc = write(fd, buf, n);
	if (rc < 0) {
		sky_err("sky_pidfile_create: failed write(%s) errno=%d\n",
			pidfile, errno);
		close(fd);

		return -1;
	}
	close(fd);

	return 0;
}

/*
 * This routine is aimed to be used instead of standard call daemon(),
 * because we want to create pidfile from a parent process not to race
 * with a systemd PIDFile handler.
 */
static int sky_daemonize(const char *pidfile)
{
	int pid, rc;

	pid = fork();
	if (pid == -1) {
		sky_err("sky_daemonize: fork() failed, errno=%d\n", errno);
		return -1;
	}
	else if (pid != 0) {
		if (pidfile) {
			/*
			 * Yes, we write pid from a parent to avoid any races
			 * inside systemd PIDFile handlers.  In case of errors
			 * child is gracefully killed.
			 */

			rc = sky_pidfile_create(pidfile, pid);
			if (rc) {
				kill(pid, SIGTERM);

				return -1;
			}
		}
		/* Parent exits */
		exit(0);
	}
	if (setsid() == -1) {
		sky_err("sky_daemonize: setsid() failed, errno=%d\n", errno);
		return -1;
	}
	pid = chdir("/");
	(void)pid;

	return 0;
}

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

static pthread_mutex_t subsc_mutex = PTHREAD_MUTEX_INITIALIZER;

static void sky_on_charging_state(void *data, struct sky_charging_state *state)
{
	struct sky_server_dev *servdev = data;
	struct sky_server *serv = servdev->serv;
	struct sky_dev_desc *devdesc = servdev->devdesc;
	struct sky_charging_state_rsp rsp;
	char topic[128];
	zmsg_t *msg;
	size_t len;
	int rc;

	BUILD_BUG_ON(sizeof(serv->uuid) + sizeof(devdesc->portname) >
		     sizeof(topic));

	rsp.hdr.type = htole16(SKY_CHARGING_STATE_EV);
	rsp.hdr.error = 0;
	rsp.dev_hw_state = htole16(state->dev_hw_state);
	rsp.current = htole16(state->current);
	rsp.voltage = htole16(state->voltage);

	msg = zmsg_new();
	if (!msg) {
		sky_err("zmsg_new(): No memory\n");
		return;
	}
	/* Publisher topic */
	memcpy(topic, serv->uuid, sizeof(serv->uuid));
	len = strlen(devdesc->portname);
	memcpy(topic + sizeof(serv->uuid), devdesc->portname, len);
	len += sizeof(serv->uuid);
	rc = zmsg_addmem(msg, topic, len);
	if (!rc)
		rc = zmsg_addmem(msg, &rsp, sizeof(rsp));
	if (rc) {
		sky_err("zmsg_add(): No memory\n");
		zmsg_destroy(&msg);
		return;
	}
	pthread_mutex_lock(&subsc_mutex);
	rc = zmsg_send(&msg, serv->zock.pub);
	pthread_mutex_unlock(&subsc_mutex);
	if (rc) {
		sky_err("zmsg_send(): %s\n", strerror(errno));
		zmsg_destroy(&msg);
	}
}

static struct sky_rsp_hdr emergency_rsp;

static inline void sky_free(void *rsp)
{
	if (rsp != &emergency_rsp)
		free(rsp);
}

static inline const void *zframe_data_const(const zframe_t *frame)
{
	return zframe_data((zframe_t *)frame);
}

static inline size_t zframe_size_const(const zframe_t *frame)
{
	return zframe_size((zframe_t *)frame);
}

static inline struct sky_dev *sky_find_dev(struct sky_server *serv,
					   const zframe_t *devport_frame)
{
	struct sky_dev_desc *devdesc;
	const char *portname;
	size_t len, num;

	if (devport_frame == NULL)
		return NULL;

	portname = zframe_data_const(devport_frame);
	len = min(sizeof(devdesc->portname),
		  zframe_size((zframe_t *)devport_frame));

	/*
	 * For now do linear search.  Far from optimal, but simple.
	 */

	num = 0;
	foreach_devdesc(devdesc, serv->devhead) {
		struct sky_server_dev *servdev = &serv->devs[num++];

		if (!memcmp(devdesc->portname, portname, len))
			return servdev->dev;
	}

	return NULL;
}

static int sky_devs_list_rsp(struct sky_server *serv,
			     void **rsp_hdr, size_t *rsp_len)
{
	struct sky_dev_desc *dev, *head;
	struct sky_devs_list_rsp *rsp;
	struct sky_dev_conf local_conf = {
		.contype = SKY_LOCAL
	};
	void *rsp_void = NULL;
	size_t len;
	int rc;

	len = sizeof(*rsp);
	rsp = rsp_void = calloc(1, len);
	if (!rsp)
		return -ENOMEM;

	rc = sky_devslist(&local_conf, 1, &head);

	rsp->hdr.type  = htole16(SKY_DEVS_LIST_RSP);
	rsp->hdr.error = htole16(-rc);
	if (!rc) {
		int num;

		foreach_devdesc(dev, head)
			len += sizeof(rsp->info[0]);

		rsp = realloc(rsp, len);
		if (!rsp) {
			free(rsp_void);
			sky_devsfree(head);
			return -ENOMEM;
		}
		rsp_void = rsp;
		num = 0;
		foreach_devdesc(dev, head) {
			struct sky_dev_info *info = &rsp->info[num++];

			info->dev_type = htole16(dev->dev_type);
			info->firmware_version =
				htole32(dev->firmware_version);
			memcpy(info->portname, dev->portname,
			       sizeof(dev->portname));
			memcpy(info->dev_uuid, serv->uuid, sizeof(serv->uuid));
		}
		sky_devsfree(head);
		rsp->num_devs = htole16(num);
	}

	*rsp_hdr = rsp_void;
	*rsp_len = len;

	return rc;
}

static void sky_execute_cmd(struct sky_server *serv,
			    const zframe_t *devport_frame,
			    const zframe_t *data_frame,
			    struct sky_rsp_hdr **rsp_hdr, size_t *rsp_len)
{
	const struct sky_req_hdr *req_hdr = zframe_data_const(data_frame);
	enum sky_proto_type req_type = SKY_UNKNOWN_REQRSP;
	struct sky_dev *dev;

	void *rsp_void = NULL;
	size_t len;
	int rc, i;

	if (zframe_size_const(data_frame) < sizeof(*req_hdr)) {
		sky_err("malformed request header\n");
		rc = -EINVAL;
		goto emergency;
	}

	req_type = le16toh(req_hdr->type);

	switch (req_type) {
	case SKY_GET_DEV_PARAMS_REQ: {
		struct sky_get_dev_params_req *req = (void *)req_hdr;
		struct sky_get_dev_params_rsp *rsp;
		struct sky_dev_params params;
		int num;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		if (zframe_size_const(data_frame) < sizeof(*req)) {
			sky_err("malformed request\n");
			rc = -EINVAL;
			goto emergency;
		}

		BUILD_BUG_ON(sizeof(params.dev_params_bits) * 8 <
			     SKY_NUM_DEVPARAM);

		params.dev_params_bits = le32toh(req->dev_params_bits);
		for (i = 0, num = 0; i < SKY_NUM_DEVPARAM; i++) {
			if (params.dev_params_bits & (1<<i))
				num++;
		}
		len = num * sizeof(rsp->dev_params[0]) + sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}
		rc = sky_paramsget(dev, &params);

		rsp->hdr.type  = htole16(SKY_GET_DEV_PARAMS_RSP);
		rsp->hdr.error = htole16(-rc);
		if (!rc) {
			for (i = 0; i < SKY_NUM_DEVPARAM; i++)
				rsp->dev_params[i] = htole32(params.dev_params[i]);
		}

		break;
	}
	case SKY_SET_DEV_PARAMS_REQ: {
		struct sky_set_dev_params_req *req = (void *)req_hdr;
		struct sky_rsp_hdr *rsp;
		struct sky_dev_params params = {
			.dev_params_bits = 0
		};
		int ind;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		if (zframe_size_const(data_frame) < sizeof(*req)) {
			sky_err("malformed request\n");
			rc = -EINVAL;
			goto emergency;
		}

		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		BUILD_BUG_ON(sizeof(params.dev_params_bits) * 8 <
			     SKY_NUM_DEVPARAM);

		params.dev_params_bits = le32toh(req->dev_params_bits);

		for (i = 0, ind = 0; i < SKY_NUM_DEVPARAM; i++) {
			if (!(params.dev_params_bits & (1<<i)))
				continue;
			if (zframe_size_const(data_frame) <
			    (sizeof(*req) +
			     sizeof(req->dev_params[0]) * (ind + 1))) {
				sky_err("malformed request\n");
				free(rsp);
				rc = -EINVAL;
				goto emergency;
			}
			params.dev_params[i] = le32toh(req->dev_params[ind++]);
		}

		rc = sky_paramsset(dev, &params);

		rsp->type  = htole16(SKY_SET_DEV_PARAMS_RSP);
		rsp->error = htole16(-rc);

		break;
	}
	case SKY_START_CHARGE_REQ: {
		struct sky_rsp_hdr *rsp;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		rc = sky_chargestart(dev);

		rsp->type  = htole16(SKY_START_CHARGE_RSP);
		rsp->error = htole16(-rc);

		break;
	}
	case SKY_STOP_CHARGE_REQ: {
		struct sky_rsp_hdr *rsp;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		rc = sky_chargestop(dev);

		rsp->type  = htole16(SKY_STOP_CHARGE_RSP);
		rsp->error = htole16(-rc);

		break;
	}
	case SKY_OPEN_COVER_REQ: {
		struct sky_rsp_hdr *rsp;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		rc = sky_coveropen(dev);

		rsp->type  = htole16(SKY_OPEN_COVER_RSP);
		rsp->error = htole16(-rc);

		break;
	}
	case SKY_CLOSE_COVER_REQ: {
		struct sky_rsp_hdr *rsp;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		rc = sky_coverclose(dev);

		rsp->type  = htole16(SKY_CLOSE_COVER_RSP);
		rsp->error = htole16(-rc);

		break;
	}
	case SKY_CHARGING_STATE_REQ: {
		struct sky_charging_state_rsp *rsp;
		struct sky_charging_state state;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		rc = sky_chargingstate(dev, &state);

		rsp->hdr.type  = htole16(SKY_CHARGING_STATE_RSP);
		rsp->hdr.error = htole16(-rc);
		if (!rc) {
			rsp->dev_hw_state = htole32(state.dev_hw_state);
			rsp->current = htole16(state.current);
			rsp->voltage = htole16(state.voltage);
		}

		break;
	}
	case SKY_RESET_DEV_REQ: {
		struct sky_rsp_hdr *rsp;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		rc = sky_reset(dev);

		rsp->type  = htole16(SKY_RESET_DEV_RSP);
		rsp->error = htole16(-rc);

		break;
	}
	case SKY_DEVS_LIST_REQ: {
		rc = sky_devs_list_rsp(serv, &rsp_void, &len);
		if (rc)
			goto emergency;
		break;
	}
	case SKY_PEERINFO_REQ: {
		struct sky_peerinfo_rsp *rsp;

		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		rsp->hdr.type  = htole16(SKY_PEERINFO_RSP);
		rsp->hdr.error = htole16(0);
		rsp->proto_version  = htole16(SKY_PROTO_VERSION);
		rsp->server_version = htole32(SKY_VERSION);

		break;
	}

	default:
		sky_err("unknown request: %d\n", req_type);
		rc = -EINVAL;
		goto emergency;
	}

	*rsp_hdr = rsp_void;
	*rsp_len = len;

	return;

emergency:
	emergency_rsp.error = htole16(-rc);
	if (req_type == SKY_UNKNOWN_REQRSP || req_type >= SKY_LAST_REQRSP)
		emergency_rsp.type = SKY_UNKNOWN_REQRSP;
	else
		emergency_rsp.type = htole16(req_type + 1);

	*rsp_hdr = &emergency_rsp;
	*rsp_len = sizeof(emergency_rsp);

	return;
}

static void sky_zocket_destroy(struct sky_server *serv)
{
	struct zocket *z = &serv->zock;

	if (z->router) {
		(void)zmq_close(z->router);
		z->router = NULL;
	}
	if (z->pub) {
		(void)zmq_close(z->pub);
		z->pub = NULL;
	}
	if (z->ctx) {
		(void)zmq_ctx_term(z->ctx);
		z->ctx = NULL;
	}
}

static int sky_zocket_create(struct sky_server *serv, const char *addr, int port)
{
	struct zocket *z = &serv->zock;
	char zaddr1[128], zaddr2[128];
	uint32_t timeo;
	int rc;

	rc = snprintf(zaddr1, sizeof(zaddr1), "tcp://%s:%d", addr, port);
	if (rc < 0 || rc >= sizeof(zaddr1))
		return -EINVAL;

	rc = snprintf(zaddr2, sizeof(zaddr2), "tcp://%s:%d", addr, port+1);
	if (rc < 0 || rc >= sizeof(zaddr2))
		return -EINVAL;

	z->ctx = zmq_ctx_new();
	if (z->ctx == NULL)
		return -ENOMEM;

	z->router = zmq_socket(z->ctx, ZMQ_ROUTER);
	if (z->router == NULL) {
		rc = -ENOMEM;
		goto err;
	}
	z->pub = zmq_socket(z->ctx, ZMQ_PUB);
	if (z->pub == NULL) {
		rc = -ENOMEM;
		goto err;
	}

	timeo = DEFAULT_TIMEOUT;
	rc = zmq_setsockopt(z->pub, ZMQ_SNDTIMEO, &timeo, sizeof(timeo));
	if (rc != 0) {
		rc = -errno;
		sky_err("zmq_setsockopt(): %s\n", strerror(-rc));
		goto err;
	}
	timeo = DEFAULT_TIMEOUT;
	rc = zmq_setsockopt(z->router, ZMQ_SNDTIMEO, &timeo, sizeof(timeo));
	if (rc != 0) {
		rc = -errno;
		sky_err("zmq_setsockopt(): %s\n", strerror(-rc));
		goto err;
	}
	rc = zmq_bind(z->router, zaddr1);
	if (rc != 0) {
		rc = -errno;
		sky_err("zmq_bind(): %s\n", strerror(-rc));
		goto err;
	}
	rc = zmq_bind(z->pub, zaddr2);
	if (rc != 0) {
		rc = -errno;
		sky_err("zmq_bind(): %s\n", strerror(-rc));
		goto err;
	}

	return 0;

err:
	sky_zocket_destroy(serv);

	return rc;
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

static int sky_port_discovery(struct sky_discovery *discovery, char **ip_)
{
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
		rc = -ECONNRESET;
		goto err;
	}
	hostname = zstr_recv(beacon);
	if (!hostname || !hostname[0]) {
		free(hostname);
		sky_err("hostname = zstr_recv()\n");
		rc = -ECONNRESET;
		goto err;
	}
	free(hostname);
	zsock_set_rcvtimeo(beacon, SKY_DISCOVERY_MS);

	rc = zsock_send(beacon, "ss", "SUBSCRIBE", SKY_DISCOVERY_MAGIC);
	if (rc) {
		sky_err("zsock_send(SUBSCRIBE)\n");
		rc = -ECONNRESET;
		goto err;
	}
	ip = zstr_recv(beacon);
	if (!ip) {
		rc = -ECONNRESET;
		goto err;
	}
	frame = zframe_recv(beacon);
	if (!frame) {
		rc = -ECONNRESET;
		goto err;
	}
	if (zframe_size(frame) != sizeof(*discovery)) {
		sky_err("Malformed discovery frame, %zd got %zd\n",
			sizeof(*discovery), zframe_size(frame));
		rc = -ECONNRESET;
		goto err;
	}

	memcpy(discovery, zframe_data(frame), zframe_size(frame));
	*ip_ = ip;
	rc = 0;

err:
	if (rc)
		zstr_free(&ip);
	zframe_destroy(&frame);
	zactor_destroy(&beacon);

	return rc;
}

struct pub_proxy {
	void *sub;
	void *push;
	pthread_t thr;
};

static void *thread_do_proxy(void *arg)
{
	struct pub_proxy *proxy = arg;

	(void)zmq_proxy(proxy->sub, proxy->push, NULL);

	return NULL;
}

static int sky_setup_and_proxy_pub(struct sky_server *serv,
				   struct sky_discovery *discovery,
				   const char *ip, struct pub_proxy *proxy)
{
	void *ctx = serv->zock.ctx;
	void *sub = NULL;
	void *push = NULL;
	char zaddr[128];
	int rc;

	sub = zmq_socket(ctx, ZMQ_SUB);
	if (!sub) {
		sky_err("zmq_socket(ZMQ_SUB): No memory\n");
		rc = -ENOMEM;
		goto err;
	}
	rc = zmq_setsockopt(sub, ZMQ_SUBSCRIBE, NULL, 0);
	if (rc) {
		rc = -errno;
		sky_err("zmq_setsockopt(ZMQ_SUBSCRIBE): %s\n", strerror(-rc));
		goto err;
	}
	snprintf(zaddr, sizeof(zaddr), "tcp://%s:%d",
		 serv->cli.addr, atoi(serv->cli.port)+1);
        rc = zmq_connect(sub, zaddr);
	if (rc) {
		rc = -errno;
		sky_err("zmq_connect(%s): %s\n", zaddr, strerror(-rc));
		goto err;
	}
	push = zmq_socket(ctx, ZMQ_PUSH);
	if (!push) {
		sky_err("zmq_socket(ZMQ_PUSH): No memory\n");
		rc = -ENOMEM;
		goto err;
	}
	snprintf(zaddr, sizeof(zaddr), "tcp://%s:%d",
		 ip, le16toh(discovery->sub_port));
	rc = zmq_connect(push, zaddr);
	if (rc) {
		rc = -errno;
		sky_err("zmq_connect(%s): %s\n", zaddr, strerror(-rc));
		goto err;
	}
	proxy->sub = sub;
	proxy->push = push;

	rc = pthread_create(&proxy->thr, NULL, thread_do_proxy, proxy);
	if (rc) {
		rc = -rc;
		sky_err("pthread_create(): %s\n", strerror(-rc));
		goto err;
	}

	return 0;

err:
	if (sub)
		zmq_close(sub);
	if (push)
		zmq_close(push);

	return rc;
}

static void sky_destroy_pub(struct pub_proxy *proxy)
{
	(void)sky_kill_pthread(proxy->thr);
	zmq_close(proxy->sub);
	zmq_close(proxy->push);
}

struct req_proxy {
	void *to_server;
	void *to_broker;
	void *broker_monitor;
};


/**
 * sky_zock_proxy() - simple variant of zmq_proxy() which
 *                    detects disconnects from monitor socket.
 *
 * For some reason (Oh God, I do not want to debug zmq crap)
 * setting ZMQ_RECONNECT_IVL to -1 does not help and socket
 * continues reconnecting.  What we need instead is to repeat
 * discovery if broker goes does down.
 *
 * Also heartbeats here are supported.
 */
static int sky_zmq_proxy(struct req_proxy *p)
{
	int rc;

	while (true) {
		zmq_pollitem_t items [] = {
			{ p->broker_monitor, 0, ZMQ_POLLIN, 0 },
			{ p->to_broker,      0, ZMQ_POLLIN, 0 },
			{ p->to_server,      0, ZMQ_POLLIN, 0 },
		};
		void *dst, *src = NULL;
		zmsg_t *msg;

		rc = zmq_poll(items, 3,
			      SKY_HEARTBEAT_IVL_MS * ZMQ_POLL_MSEC);
		if (rc == -1)
			/* Interrupted */
			break;

		if (items[0].revents & ZMQ_POLLIN) {
			/* Connection is dead */
			break;
		} else if (items[1].revents & ZMQ_POLLIN) {
			src = p->to_broker;
			dst = p->to_server;
		} else if (items[2].revents & ZMQ_POLLIN) {
			src = p->to_server;
			dst = p->to_broker;
		} else {
			/* Heartbeat destination */
			dst = p->to_broker;
		}

		if (src) {
			msg = zmsg_recv(src);
			if (!msg)
				break;
			if (src == p->to_broker) {
				zframe_t *frame = zmsg_first(msg);
				/* DEALER prepends zero frame, remove it */
				if (frame) {
					zmsg_remove(msg, frame);
					zframe_destroy(&frame);
				}
			}
		} else {
			/* Create heartbeat */
			msg = zmsg_new();
			if (!msg)
				continue;
		}
		/* Prepend zero frame for DEALER */
		if (dst == p->to_broker)
			zmsg_pushmem(msg, NULL, 0);
		rc = zmsg_send(&msg, dst);
		if (rc) {
			zmsg_destroy(&msg);
			break;
		}
	}

	return rc;
}

static int sky_setup_req(struct sky_server *serv,
			 struct sky_discovery *discovery,
			 const char *ip, struct req_proxy *proxy)
{
	void *ctx = serv->zock.ctx;
	void *to_server = NULL;
	void *to_broker = NULL;
	void *monitor = NULL;
	char zaddr[128];
	void *rsp_void;
	size_t rsp_len;
	int rc;

	to_server = zmq_socket(ctx, ZMQ_REQ);
	if (!to_server) {
		sky_err("zmq_socket(): No memory\n");
		rc = -ENOMEM;
		goto err;
	}
	snprintf(zaddr, sizeof(zaddr), "tcp://%s:%d",
		 serv->cli.addr, atoi(serv->cli.port));
	rc = zmq_connect(to_server, zaddr);
	if (rc) {
		rc = -errno;
		sky_err("zmq_connect(): %s\n", strerror(-rc));
		goto err;
	}
	/* Use DEALER instead of REQ to have heartbeats */
	to_broker = zmq_socket(ctx, ZMQ_DEALER);
	if (!to_broker) {
		rc = -errno;
		sky_err("zmq_socket(): No memory\n");
		goto err;
	}
	rc = zmq_setsockopt(to_broker, ZMQ_IDENTITY,
			    serv->uuid, sizeof(serv->uuid));
	if (rc) {
		rc = -errno;
		sky_err("zmq_setsockopt(ZMQ_SUBSCRIBE): %s\n", strerror(-rc));
		goto err;
	}
	snprintf(zaddr, sizeof(zaddr), "tcp://%s:%d",
		 ip, le16toh(discovery->servers_port));
	rc = zmq_connect(to_broker, zaddr);
	if (rc) {
		rc = -errno;
		sky_err("zmq_connect(): %s\n", strerror(-rc));
		goto err;
	}
	/*
	 * Also imply random number to avoid races in zmq when
	 * zmq_socket_monitor() returns EADDRINUSE if previous
	 * monitor socket has been just closed.
	 */
	snprintf(zaddr, sizeof(zaddr), "inproc://monitor-broker-%d-%ld",
		 getpid(), random());
	/* Mintor disconnect event to repeat discovery */
	rc = zmq_socket_monitor(to_broker, zaddr,
				ZMQ_EVENT_DISCONNECTED);
	if (rc) {
		rc = -errno;
		sky_err("zmq_socket_monitor(): %s\n", strerror(-rc));
		goto err;
	}
	monitor = zmq_socket(ctx, ZMQ_PAIR);
	if (!monitor) {
		sky_err("zmq_socket(): No memory\n");
		rc = -ENOMEM;
		goto err;
	}
	rc = zmq_connect(monitor, zaddr);
	if (rc) {
		rc = -errno;
		sky_err("zmq_connect(): %s\n", strerror(-rc));
		goto err;
	}
	rc = sky_devs_list_rsp(serv, &rsp_void, &rsp_len);
	if (rc) {
		sky_err("sky_devs_list_rsp(): %s\n", strerror(-rc));
		goto err;
	}
	/* Prepend zero frame for DEALER to emulate REQ */
	rc = zmq_send(to_broker, NULL, 0, ZMQ_SNDMORE);
	if (!rc)
		rc = zmq_send(to_broker, rsp_void, rsp_len, 0);
	if (rc != rsp_len) {
		rc = -errno;
		sky_err("zmq_send(): %s\n", strerror(-rc));
		goto err;

	}

	proxy->to_broker = to_broker;
	proxy->to_server = to_server;
	proxy->broker_monitor = monitor;

	return 0;

err:
	if (to_server)
		zmq_close(to_server);
	if (to_broker)
		zmq_close(to_broker);
	if (monitor)
		zmq_close(monitor);

	return rc;
}

static void sky_destroy_req(struct req_proxy *proxy)
{
	zmq_close(proxy->to_server);
	zmq_close(proxy->to_broker);
	zmq_close(proxy->broker_monitor);
}

static void *sky_discover_broker(void *arg_)
{
	struct sky_server *serv = arg_;
	struct sky_discovery discovery;
	struct pub_proxy pub_proxy = {}; /* Make gcc happy */
	struct req_proxy req_proxy = {}; /* Make gcc happy */
	char *ip;
	int rc;

	/*
	 * Do discovery loop
	 */
	while (!serv->exit) {
		/* Receive UDP discovery */
		rc = sky_port_discovery(&discovery, &ip);
		if (rc) {
			sleep(1);
			continue;
		}
		rc = sky_setup_and_proxy_pub(serv, &discovery, ip, &pub_proxy);
		if (rc)
			goto free_ip;
		rc = sky_setup_req(serv, &discovery, ip, &req_proxy);
		if (rc)
			goto destroy_pub;

		/* Main request proxying */
		(void)sky_zmq_proxy(&req_proxy);

		sky_destroy_req(&req_proxy);
destroy_pub:
		sky_destroy_pub(&pub_proxy);
free_ip:
		zstr_free(&ip);
	}

	return NULL;
}

static int sky_server_loop(struct sky_server *serv)
{
	zframe_t *data_frame, *devport_frame;
	struct sky_rsp_hdr *rsp;
	pthread_t thread;
	size_t rsp_len;
	zmsg_t *msg;
	int rc;

	rc = pthread_create(&thread, NULL, sky_discover_broker, serv);
	if (rc) {
		sky_err("pthread_create(sky_discover_broker): %d\n", rc);
		thread = 0;
	}

	while (1) {
		msg = zmsg_recv(serv->zock.router);
		if (msg == NULL) {
			/* Interrupted */
			rc = 0;
			break;
		}
		data_frame = sky_find_data_frame(msg);
		if (data_frame == NULL) {
			sky_err("zmsg_recv(): no data frame\n");
			rc = -EIO;
			break;
		}
		devport_frame = zmsg_next(msg);
		sky_execute_cmd(serv, devport_frame, data_frame, &rsp, &rsp_len);
		if (devport_frame) {
			/* Remove and destroy devport frame */
			zmsg_remove(msg, devport_frame);
			zframe_destroy(&devport_frame);
		}
		/* Replace data frame with response */
		zframe_reset(data_frame, rsp, rsp_len);
		rc = zmsg_send(&msg, serv->zock.router);
		if (rc != 0) {
			rc = -errno;
			sky_err("zmsg_send(): %s\n", strerror(-rc));
		}
		sky_free(rsp);
	}
	if (thread) {
		/* Tear down discovery thread */
		serv->exit = true;

		(void)sky_kill_pthread(thread);
	}

	return rc;
}

int main(int argc, char *argv[])
{
	struct sky_dev_conf conf = {
		.contype = SKY_LOCAL,
	};
	struct sky_server serv = {
	};
	struct sky_dev_desc *devdesc;
	int num, rc;

	uuid_generate(serv.uuid);

	rc = cli_parse(argc, argv, &serv.cli);
	if (rc) {
		fprintf(stderr, "%s\n", cli_usage);
		return -1;
	}
	/*
	 * Daemonize before creating zmq socket, ZMQ is written by
	 * people who are not able to deal with fork() gracefully.
	 */
	if (serv.cli.daemon)
		sky_daemonize(serv.cli.pidf);

	rc = sky_zocket_create(&serv, serv.cli.addr, atoi(serv.cli.port));
	if (rc) {
		sky_err("Can't create server sockets: %s\n", strerror(-rc));
		goto free_cli;
	}
	rc = sky_devslist(&conf, 1, &serv.devhead);
	if (rc) {
		sky_err("sky_devslist(): %s\n", strerror(-rc));
		goto destroy_zocket;
	}
	num = 0;
	foreach_devdesc(devdesc, serv.devhead)
		num++;
	serv.devs = calloc(num, sizeof(*serv.devs));
	if (serv.devs == NULL) {
		rc = -ENOMEM;
		goto free_devs;
	}
	num = 0;
	foreach_devdesc(devdesc, serv.devhead) {
		struct sky_subscription subsc = {
			.on_state = sky_on_charging_state,
			.interval_msecs = 1000,
		};
		struct sky_server_dev *servdev;

		servdev = &serv.devs[num];
		servdev->serv = &serv;
		servdev->devdesc = devdesc;
		rc = sky_devopen(devdesc, &servdev->dev);
		if (rc) {
			sky_err("sky_devpopen(): %s\n", strerror(-rc));
			goto close_devs;
		}
		subsc.data = servdev;
		rc = sky_subscribe(servdev->dev, &subsc);
		if (rc) {
			sky_err("sky_subscribe(): %s\n", strerror(-rc));
			sky_devclose(servdev->dev);
			servdev->dev = NULL;
			goto close_devs;
		}
		num++;
	}
	rc = sky_server_loop(&serv);

close_devs:
	num = 0;
	foreach_devdesc(devdesc, serv.devhead) {
		struct sky_dev *dev = serv.devs[num++].dev;

		if (dev == NULL)
			continue;
		sky_unsubscribe(dev);
		sky_devclose(dev);
	}
free_devs:
	sky_devsfree(serv.devhead);
destroy_zocket:
	sky_zocket_destroy(&serv);
free_cli:
	cli_free(&serv.cli);

	return rc;
}

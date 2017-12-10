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

#include <zmq.h>

#include "skyserver-cmd.h"
#include "libskysense.h"
#include "skyproto.h"
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
	struct zocket zock;
	struct sky_server_dev *devs;
	struct sky_dev_desc *devhead;
};

#define sky_err(fmt, ...) \
	fprintf(stderr, __FILE__ ":%s():" stringify(__LINE__) ": " fmt, \
		__func__, ##__VA_ARGS__)

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

static pthread_mutex_t subsc_mutex = PTHREAD_MUTEX_INITIALIZER;

static void sky_on_charging_state(void *data, struct sky_charging_state *state)
{
	struct sky_server_dev *servdev = data;
	struct sky_server *serv = servdev->serv;
	struct sky_charging_state_rsp msg;
	size_t len;
	int rc;

	msg.hdr.type = htole16(SKY_CHARGING_STATE_EV);
	msg.hdr.error = 0;
	msg.dev_hw_state = htole16(state->dev_hw_state);
	msg.current = htole16(state->current);
	msg.voltage = htole16(state->voltage);

	pthread_mutex_lock(&subsc_mutex);
	/* Publisher topic (device portname) */
	len = sizeof(servdev->devdesc->portname);
	rc = zmq_send(serv->zock.pub, servdev->devdesc->portname,
		      len, ZMQ_SNDMORE);
	if (rc != len) {
		sky_err("zmq_send(): %s\n", strerror(-rc));
		goto unlock;
	}
	/* Actual message */
	rc = zmq_send(serv->zock.pub, &msg, sizeof(msg), 0);
	if (rc != sizeof(msg))
		sky_err("zmq_send(): %s\n", strerror(-rc));

unlock:
	pthread_mutex_unlock(&subsc_mutex);

	return;
}

static struct sky_rsp_hdr emergency_rsp;

static inline void sky_free(void *rsp)
{
	if (rsp != &emergency_rsp)
		free(rsp);
}

static inline struct sky_dev *sky_find_dev(struct sky_server *serv,
					   const char *portname,
					   size_t len)
{
	struct sky_dev_desc *devdesc;
	int num;

	/*
	 * For now do linear search.  Far from optimal, but simple.
	 */

	num = 0;
	foreach_devdesc(devdesc, serv->devhead) {
		struct sky_server_dev *servdev = &serv->devs[num++];

		len = min(sizeof(devdesc->portname), len);
		if (!memcmp(devdesc->portname, portname, len))
			return servdev->dev;
	}

	return NULL;
}

static void sky_execute_cmd(struct sky_server *serv, const void *ident,
			    size_t ident_len, void *req_, size_t req_len,
			    struct sky_rsp_hdr **rsp_hdr, size_t *rsp_len)
{
	enum sky_proto_type req_type = SKY_UNKNOWN_REQRSP;
	struct sky_req_hdr *req_hdr = req_;
	struct sky_dev *dev;

	void *rsp_void = NULL;
	size_t len;
	int rc, i;

	if (req_len < sizeof(*req_hdr)) {
		sky_err("malformed request header\n");
		rc = -EINVAL;
		goto emergency;
	}

	req_type = le16toh(req_hdr->type);

	switch (req_type) {
	case SKY_GET_DEV_PARAMS_REQ: {
		struct sky_get_dev_params_req *req = req_;
		struct sky_get_dev_params_rsp *rsp;
		struct sky_dev_params params;
		int num;

		dev = sky_find_dev(serv, ident, ident_len);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		if (req_len < sizeof(*req)) {
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
		struct sky_set_dev_params_req *req = req_;
		struct sky_rsp_hdr *rsp;
		struct sky_dev_params params = {
			.dev_params_bits = 0
		};
		int ind;

		dev = sky_find_dev(serv, ident, ident_len);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		if (req_len < sizeof(*req)) {
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
			if (req_len < (sizeof(*req) +
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

		dev = sky_find_dev(serv, ident, ident_len);
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

		dev = sky_find_dev(serv, ident, ident_len);
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

		dev = sky_find_dev(serv, ident, ident_len);
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

		dev = sky_find_dev(serv, ident, ident_len);
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

		dev = sky_find_dev(serv, ident, ident_len);
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

		dev = sky_find_dev(serv, ident, ident_len);
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
		struct sky_dev_desc *dev, *head;
		struct sky_devs_list_rsp *rsp;
		struct sky_dev_conf local_conf = {
			.contype = SKY_LOCAL
		};

		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

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
				rc = -ENOMEM;
				goto emergency;
			}
			rsp_void = rsp;
			num = 0;
			foreach_devdesc(dev, head) {
				struct sky_dev_info *info = &rsp->info[num++];

				info->dev_type = htole16(dev->dev_type);
				memcpy(info->portname, dev->portname,
				       sizeof(dev->portname));
			}
			sky_devsfree(head);
			rsp->num_devs = htole16(num);
		}
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

static int sky_zocket_recv(void *zock, void **ident,
			   int *ident_len, void **data)
{
	void *dbuf, *ibuf;
	zmq_msg_t msg;
	int rc, sz, nb;

	zmq_msg_init(&msg);

	/* Recv identity */
	rc = zmq_msg_recv(&msg, zock, 0);
	if (rc < 0)
		return -errno;
	sz = zmq_msg_size(&msg);
	ibuf = malloc(sz);
	if (ibuf == NULL) {
		zmq_msg_close(&msg);

		return -ENOMEM;
	}
	memcpy(ibuf, zmq_msg_data(&msg), sz);
	*ident_len = sz;

	/* Recv delimiter */
	rc = zmq_msg_recv(&msg, zock, 0);
	if (rc != 0) {
		zmq_msg_close(&msg);
		free(ibuf);

		return -EPIPE;
	}

	/* Recv message */
	nb = 0;
	dbuf = NULL;
	do {
		rc = zmq_msg_recv(&msg, zock, 0);
		if (rc < 0) {
			rc = -errno;
			zmq_msg_close(&msg);
			free(dbuf);
			free(ibuf);

			return rc;
		}
		if (zmq_msg_size(&msg) == 0)
			continue;
		sz = zmq_msg_size(&msg);
		dbuf = realloc(dbuf, sz);
		if (dbuf == NULL) {
			zmq_msg_close(&msg);
			free(ibuf);

			return -ENOMEM;
		}
		memcpy(dbuf + nb, zmq_msg_data(&msg), sz);
		nb += sz;
	} while (zmq_msg_more(&msg));

	zmq_msg_close(&msg);
	*data  = dbuf;
	*ident = ibuf;

	return nb;
}

static int sky_zocket_send(void *zock, const void *ident, int ident_len,
			   const void *data, int data_len, bool more)
{
	int rc;

	if (ident && ident_len) {
		rc = zmq_send(zock, ident, ident_len, ZMQ_SNDMORE);
		if (rc != ident_len)
			return -errno;
		/* Delimiter */
		rc = zmq_send(zock, NULL, 0, ZMQ_SNDMORE);
		if (rc != 0)
			return -errno;
	}
	rc = zmq_send(zock, data, data_len, more ? ZMQ_SNDMORE : 0);
	if (rc != data_len)
		rc = -errno;

	return rc;
}

static int sky_server_loop(struct sky_server *serv)
{
	void *ident = NULL, *req = NULL; /* make gcc happy */
	int ident_len = 0, req_len;
	struct sky_rsp_hdr *rsp;
	size_t rsp_len;
	int rc;

	while (1) {
		req_len = sky_zocket_recv(serv->zock.router, &ident,
					  &ident_len, &req);
		if (req_len < 0) {
			rc = req_len;
			sky_err("sky_zocket_recv(): %s\n", strerror(-rc));
			break;
		}
		sky_execute_cmd(serv, ident, ident_len, req, req_len,
				&rsp, &rsp_len);
		rc = sky_zocket_send(serv->zock.router, ident, ident_len,
				     rsp, rsp_len, false);
		if (rc < 0)
			sky_err("sky_zocket_send(): %s\n", strerror(-rc));

		sky_free(rsp);
		free(ident);
		free(req);
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
	struct cli cli;
	int num, rc;

	rc = cli_parse(argc, argv, &cli);
	if (rc) {
		fprintf(stderr, "%s\n", cli_usage);
		return -1;
	}
	/*
	 * Daemonize before creating zmq socket, ZMQ is written by
	 * people who are not able to deal with fork() gracefully.
	 */
	if (cli.daemon)
		sky_daemonize(cli.pidf);

	rc = sky_zocket_create(&serv, cli.addr, atoi(cli.port));
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
	cli_free(&cli);

	return rc;
}

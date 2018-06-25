#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <endian.h>
#include <limits.h>
#include <string.h>

#include <czmq.h>

#include "skybroker-cmd.h"
#include "version.h"
#include "skyproto.h"

struct server_peer {
	zmsg_t *idents_msg;
	zframe_t *ident_frame;
	zframe_t *data_frame;
	zframe_t *usruuid_frame;
	struct sky_devs_list_rsp *rsp;
	union {
		unsigned long long expires_at;
		struct server_peer *next;
	};
};

/* TODO: same function exists in skyserver.c */
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

/* TODO: same function exists in skyserver.c */
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

static bool sky_is_valid_devs_list_rsp(zframe_t *data)
{
	struct sky_devs_list_rsp *rsp;
	size_t num_devs, len;

	rsp = (void *)zframe_data(data);
	len = zframe_size(data);
	if (len < sizeof(*rsp))
		return false;
	if (rsp->hdr.error)
		return false;

	num_devs = le16toh(rsp->num_devs);
	if (len != sizeof(*rsp) + sizeof(rsp->info[0]) * num_devs)
		return false;

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

		if (!sky_is_valid_devs_list_rsp(data)) {
			sky_err("Malformed devs list\n");
			server_peer_free(peer);
			return;
		}

		/* Take ownership on data frame */
		peer->data_frame = data;
		peer->rsp = (void *)zframe_data(data);
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
			     struct sky_devs_list_rsp **rsp_,
			     size_t *rsp_len)
{
	struct sky_devs_list_rsp *rsp;
	struct server_peer *peer;
	size_t num, len;

	*rsp_ = NULL;
	*rsp_len = 0;

	len = sizeof(*rsp);
	rsp = calloc(1, len);
	if (!rsp)
		return -ENOMEM;

	rsp->hdr.type  = htole16(SKY_DEVS_LIST_RSP);
	rsp->hdr.error = 0;

	/*
	 * TODO: O(n) loop, makes sense to make something better?
	 */
	for (num = 0, peer = zhashx_first(srvs_hash); peer;
	     peer = zhashx_next(srvs_hash)) {
		void *tmp;

		/* Find corresponding user devices */
		if (!zframe_eq(peer->usruuid_frame, usruuid))
			continue;

		len += peer->rsp->num_devs * sizeof(peer->rsp->info[0]);
		tmp = realloc(rsp, len);
		if (!tmp) {
			free(rsp);
			return -ENOMEM;
		}
		rsp = tmp;

		memcpy(&rsp->info[num], &peer->rsp->info[0],
		       peer->rsp->num_devs * sizeof(peer->rsp->info[0]));
		num += peer->rsp->num_devs;
	}
	rsp->num_devs = htole16(num);

	*rsp_ = rsp;
	*rsp_len = len;

	return 0;
}

static void sky_handle_client_msg(void *servers, void *clients,
				  zhashx_t *srvs_hash)
{
	struct server_peer *peer;
	struct sky_req_hdr *req;
	zframe_t *data, *ident;
	zmsg_t *msg, *srv_msg;
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
	if (zframe_size(data) < sizeof(*req)) {
		/* Malformed request? */
		zmsg_destroy(&msg);
		return;
	}

	req = (void *)zframe_data(data);
	switch (le16toh(req->type)) {
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
		rc = sky_devs_list_rsp(srvs_hash, usruuid, &rsp, &len);
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
	int rc;

	pub = zmq_socket(ctx, ZMQ_PUB);
	if (!pub) {
		sky_err("zmq_socket(ZMQ_PUB): No memory\n");
		rc = -ENOMEM;
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
	int rc;

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
	servers = zmq_socket(ctx, ZMQ_ROUTER);
	clients = zmq_socket(ctx, ZMQ_ROUTER);
	if (!servers || !clients) {
		sky_err("zmq_socket(): Failed\n");
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

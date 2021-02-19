#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <limits.h>
#include <assert.h>
#include <uuid/uuid.h>
#include <microhttpd.h>

#include "skyhttpd-cmd.h"
#include "libskysense.h"
#include "libskysense-pri.h"
#include "types.h"
#include "rbtree.h"
#include "hash.h"
#include "version.h"
#include "daemon.h"
#include "skyproto.h"

/*
 * JSON API:
 *
 * GET
 * ---
 * "user-uuid" is mandatory for the broker:
 *     /devs-list       - sky_devslist()
 *
 * "user-uuid" is mandatory for the broker
 * "device-uuid" is mandatory
 *     /charging-params - sky_paramsget()
 *     /charging-state  - sky_chargingstate()
 *     /charge-start    - sky_chargestart()
 *     /charge-stop     - sky_chargestop()
 *     /cover-open      - sky_coveropen()
 *     /cover-close     - sky_coverclose()
 */

#define HELP								\
	"<html><head><title>Skycharge API</title></head>\n"		\
	"<body>Please contact <a href=\"mailto:info@skycharge.de\">info@skycharge.de</a> " \
	"for detailed API description.</body>\n</html>\n"

enum {
	HTTP_TOO_MANY_REQUESTS_STATUS = 429,

	MAX_CONNECTIONS       = 512,
	MAX_BAD_AUTH_ATTEMPTS = 3,
	CHILL_OUT_BAD_AUTH_MS = 10000, /* 10 secs should be enough */
	MAX_REQUESTS_RATE     = 3,     /* per 1000 ms */
	_1_SEC_IN_MS          = 1000,
};

#define SKY_FD_SET_T(name) \
	unsigned long name[MAX_CONNECTIONS / (8 * sizeof(long))]
#define SKY_FD_SET(fd, set) \
	set[(fd) / (8 * sizeof(long)] |= 1<<((fd) % (8 * sizeof(long)))
#define SKY_FD_ZERO(set) \
	memset(set, 0, sizeof(*set))

struct httpd_con_addr {
	struct hash_entry  hentry; /* httpd->addrs_hash */
	struct rb_node     tentry; /* httpd->addrs_tree */
	unsigned           bad_auth_attempts;
	struct in_addr     sin_addr;
	unsigned long long expire_ms;
	unsigned long long last_access_ms;
	int                refs;
};

struct httpd {
	struct cli         cli;
	struct MHD_Daemon  *mhd;
	struct hash_table  addrs_hash; /* addrs hashed by sin_addr */
	struct rb_root     addrs_root; /* addrs to expire */
};

typedef struct MHD_Response *(httpd_handler_t)(struct httpd *,
					       struct httpd_con_addr *,
					       const char *method,
					       const char *upload_data,
					       const char *user_uuid,
					       const char *device_uuid,
					       unsigned int *http_status);

static bool http_is_get(const char *method)
{
	return !strcmp(method, MHD_HTTP_METHOD_GET);
}

#define X(state) \
	case state: return #state

static inline const char *sky_devstate_to_str(enum sky_dev_hw_state state)
{
	switch(state) {
	X(SKY_UNKNOWN);
	X(SKY_SCANNING_INIT);
	X(SKY_SCANNING_RUN);
	X(SKY_SCANNING_CHECK_MATRIX);
	X(SKY_SCANNING_PRINT);
	X(SKY_SCANNING_CHECK_WATER);
	X(SKY_SCANNING_WET);
	X(SKY_SCANNING_DETECTING);
	X(SKY_PRE_CHARGING_INIT);
	X(SKY_PRE_CHARGING_RUN);
	X(SKY_PRE_CHARGING_CHECK_MATRIX);
	X(SKY_PRE_CHARGING_PRINT);
	X(SKY_PRE_CHARGING_CHECK_WATER);
	X(SKY_PRE_CHARGING_WET);
	X(SKY_PRE_CHARGING_FIND_CHARGERS);
	X(SKY_CHARGING_INIT);
	X(SKY_CHARGING_RUN);
	X(SKY_CHARGING_MONITOR_CURRENT);
	X(SKY_POST_CHARGING_INIT);
	X(SKY_POST_CHARGING_RUN);
	X(SKY_POST_CHARGING_CHECK_MATRIX);
	X(SKY_POST_CHARGING_PRINT);
	X(SKY_POST_CHARGING_CHECK_WATER);
	X(SKY_POST_CHARGING_WET);
	X(SKY_POST_CHARGING_FIND_CHARGERS);
	X(SKY_OVERLOAD);
	X(SKY_AUTOSCAN_DISABLED);
	default:
		sky_err("unknown state: %d\n", state);
		return "UNKNOWN_STATE";
	}
}

static inline const char *sky_devparam_to_str(enum sky_dev_param param)
{
	switch(param) {
	X(SKY_EEPROM_INITED);
	X(SKY_SCANNING_INTERVAL);
	X(SKY_PRECHARGING_INTERVAL);
	X(SKY_PRECHARGING_COUNTER);
	X(SKY_POSTCHARGING_INTERVAL);
	X(SKY_POSTCHARGING_DELAY);
	X(SKY_WET_DELAY);
	X(SKY_SHORTCIRC_DELAY);
	X(SKY_THRESH_FINISH_CHARGING);
	X(SKY_THRESH_NOCHARGER_PRESENT);
	X(SKY_THRESH_SHORTCIRC);
	X(SKY_CURRENT_MON_INTERVAL);
	X(SKY_WAIT_START_CHARGING_SEC);
	default:
		sky_err("unknown param: %d\n", param);
		return "UNKNOWN_PARAM";
	}
}

static void sky_prepare_conf(struct cli *cli, const char *user_uuid,
			     struct sky_conf *conf)
{
	int ret;

	sky_confinit(conf);

	conf->contype = SKY_REMOTE;
	/* Expect valid uuid */
	ret = uuid_parse(user_uuid, conf->usruuid);
	assert(!ret);
	conf->cliport = strtol(cli->skyport, NULL, 10);
	conf->subport = conf->cliport + 1;
	strncpy(conf->hostname, cli->skyaddr,
		sizeof(conf->hostname)-1);
}

static int sky_find_and_open_dev(struct httpd *httpd,
				 const char *user_uuid,
				 const char *device_uuid,
				 struct sky_dev **pdev)
{
	struct sky_dev_desc *devdesc, *devdescs;
	struct sky_conf conf;
	char dev_uuid[36];
	int rc;

	if (!device_uuid)
		return -ENODEV;

	sky_prepare_conf(&httpd->cli, user_uuid, &conf);
	rc = sky_devslist(&conf, &devdescs);
	if (rc)
		return rc;

	foreach_devdesc(devdesc, devdescs) {
		uuid_unparse(devdesc->dev_uuid, dev_uuid);
		if (strcmp(dev_uuid, device_uuid))
			continue;

		return sky_devopen(devdesc, pdev);
	}

	return -ENODEV;
}

static int snprintf_buffer(char **pbuffer, int *poff, int *psize,
			   const char *fmt, ...)
{
	int size = *psize, off = *poff;
	char *buffer = *pbuffer;
	va_list ap;
	int len;

	/* Determine required size */
	va_start(ap, fmt);
	len = vsnprintf(NULL, 0, fmt, ap);
	va_end(ap);

	if (len < 0)
		return -EINVAL;

	if (!buffer || (len + 1) > size - off) {
		size = round_up(off + len + 1, 128);
		buffer = realloc(buffer, size);
		if (!buffer)
			return -ENOMEM;
	}

	va_start(ap, fmt);
	vsnprintf(buffer + off, size - off, fmt, ap);
	va_end(ap);

	*pbuffer = buffer;
	*poff += len;
	*psize = size;

	return 0;
}

static struct MHD_Response *
sky_devs_list_handler(struct httpd *httpd,
		      struct httpd_con_addr *addr,
		      const char *method,
		      const char *upload_data,
		      const char *user_uuid,
		      const char *device_uuid,
		      unsigned int *http_status)
{
	struct sky_dev_desc *devdesc, *devdescs;
	struct sky_conf conf;

	char *buffer = NULL;
	int off = 0, size = 0;
	int rc, ret;

	/* Not needed for devs-list request */
	(void)device_uuid;

	if (!http_is_get(method)) {
		*http_status = MHD_HTTP_BAD_REQUEST;
		return MHD_create_response_from_buffer(0, NULL, MHD_RESPMEM_PERSISTENT);
	}

	sky_prepare_conf(&httpd->cli, user_uuid, &conf);
	rc = sky_devslist(&conf, &devdescs);
	ret = snprintf_buffer(&buffer, &off, &size,
			      "{\n\t\"errno\": %d,\n\t\"devices\": [\n",
			      -rc);
	if (ret)
		goto err;

	if (!rc) {
		foreach_devdesc(devdesc, devdescs) {
			char dev_uuid[36];

			uuid_unparse(devdesc->dev_uuid, dev_uuid);
			ret = snprintf_buffer(&buffer, &off, &size,
					      "\t\t{\n"
					      "\t\t\t\"device-uuid\": \"%s\",\n"
					      "\t\t\t\"device-name\": \"%s\",\n"
					      "\t\t\t\"port-name\": \"%s\"\n"
					      "\t\t}%s\n",
					      dev_uuid, devdesc->dev_name,
					      devdesc->portname,
					      devdesc->next ? "," : "");
			if (ret)
				goto err;
		}
	}
	ret = snprintf_buffer(&buffer, &off, &size, "\t]\n}\n");
	if (ret)
		goto err;

	*http_status = MHD_HTTP_OK;
	return MHD_create_response_from_buffer(off, buffer, MHD_RESPMEM_MUST_FREE);

err:
	free(buffer);
	return NULL;
}

static struct MHD_Response *
sky_charging_params_handler(struct httpd *httpd,
			    struct httpd_con_addr *addr,
			    const char *method,
			    const char *upload_data,
			    const char *user_uuid,
			    const char *device_uuid,
			    unsigned int *http_status)
{
	struct sky_dev *dev = NULL;
	struct sky_dev_params params = {
		/* Get all params */
		.dev_params_bits = (1 << SKY_NUM_DEVPARAM) - 1
	};

	char *buffer = NULL;
	int off = 0, size = 0;
	int i, rc, ret;

	if (!http_is_get(method)) {
		*http_status = MHD_HTTP_BAD_REQUEST;
		return MHD_create_response_from_buffer(0, NULL, MHD_RESPMEM_PERSISTENT);
	}

	rc = sky_find_and_open_dev(httpd, user_uuid, device_uuid, &dev);
	if (!rc)
		rc = sky_paramsget(dev, &params);

	ret = snprintf_buffer(&buffer, &off, &size,
			      "{\n\t\"errno\": %d,\n\t\"params\": {\n",
			      -rc);
	if (ret)
		goto err;

	if (!rc) {
		for (i = 0; i < SKY_NUM_DEVPARAM; i++) {
			ret = snprintf_buffer(&buffer, &off, &size,
					      "\t\t\t\"%s\": %d%s\n",
					      sky_devparam_to_str(i),
					      params.dev_params[i],
					      i + 1 < SKY_NUM_DEVPARAM ? "," : "");
			if (ret)
				goto err;
		}
	}
	ret = snprintf_buffer(&buffer, &off, &size, "\t}\n}\n");
	if (ret)
		goto err;

	sky_devclose(dev);
	*http_status = MHD_HTTP_OK;
	return MHD_create_response_from_buffer(off, buffer, MHD_RESPMEM_MUST_FREE);

err:
	free(buffer);
	sky_devclose(dev);
	return NULL;
}

static struct MHD_Response *
sky_charging_state_handler(struct httpd *httpd,
			   struct httpd_con_addr *addr,
			   const char *method,
			   const char *upload_data,
			   const char *user_uuid,
			   const char *device_uuid,
			   unsigned int *http_status)
{
	struct sky_dev *dev = NULL;
	struct sky_charging_state state;

	char *buffer = NULL;
	int off = 0, size = 0;
	int rc, ret;

	if (!http_is_get(method)) {
		*http_status = MHD_HTTP_BAD_REQUEST;
		return MHD_create_response_from_buffer(0, NULL, MHD_RESPMEM_PERSISTENT);
	}

	rc = sky_find_and_open_dev(httpd, user_uuid, device_uuid, &dev);
	if (!rc)
		rc = sky_chargingstate(dev, &state);

	ret = snprintf_buffer(&buffer, &off, &size,
			      "{\n\t\"errno\": %d,\n\t\"charging-state\": {\n",
			      -rc);
	if (ret)
		goto err;

	if (!rc) {
		ret = snprintf_buffer(&buffer, &off, &size,
				      "\t\t\t\"state\": \"%s\",\n"
				      "\t\t\t\"voltage_mv\": %d,\n"
				      "\t\t\t\"current_ma\": %d,\n"
				      "\t\t\t\"bms\": {\n"
				      "\t\t\t\t\"charge-percentage\": %d,\n"
				      "\t\t\t\t\"charge-time-sec\": %d\n"
				      "\t\t\t}\n",
				      sky_devstate_to_str(state.dev_hw_state),
				      state.voltage,
				      state.current,
				      state.bms.charge_perc,
				      state.bms.charge_time);
		if (ret)
			goto err;
	}
	ret = snprintf_buffer(&buffer, &off, &size, "\t}\n}\n");
	if (ret)
		goto err;

	sky_devclose(dev);
	*http_status = MHD_HTTP_OK;
	return MHD_create_response_from_buffer(off, buffer, MHD_RESPMEM_MUST_FREE);

err:
	free(buffer);
	sky_devclose(dev);
	return NULL;
}

static struct MHD_Response *
sky_dev_cmd_handler(int (*devcmd)(struct sky_dev *dev),
		    struct httpd *httpd,
		    struct httpd_con_addr *addr,
		    const char *method,
		    const char *upload_data,
		    const char *user_uuid,
		    const char *device_uuid,
		    unsigned int *http_status)
{
	struct sky_dev *dev = NULL;

	char *buffer = NULL;
	int off = 0, size = 0;
	int rc, ret;

	if (!http_is_get(method)) {
		*http_status = MHD_HTTP_BAD_REQUEST;
		return MHD_create_response_from_buffer(0, NULL, MHD_RESPMEM_PERSISTENT);
	}

	rc = sky_find_and_open_dev(httpd, user_uuid, device_uuid, &dev);
	if (!rc)
		rc = devcmd(dev);

	ret = snprintf_buffer(&buffer, &off, &size, "{\n\t\"errno\": %d\n}\n",
			      -rc);
	if (ret)
		goto err;

	sky_devclose(dev);
	*http_status = MHD_HTTP_OK;
	return MHD_create_response_from_buffer(off, buffer, MHD_RESPMEM_MUST_FREE);

err:
	free(buffer);
	sky_devclose(dev);
	return NULL;
}

static struct MHD_Response *
sky_charge_start_handler(struct httpd *httpd,
			 struct httpd_con_addr *addr,
			 const char *method,
			 const char *upload_data,
			 const char *user_uuid,
			 const char *device_uuid,
			 unsigned int *http_status)
{
	return sky_dev_cmd_handler(sky_chargestart,
				   httpd, addr, method, upload_data, user_uuid,
				   device_uuid, http_status);
}

static struct MHD_Response *
sky_charge_stop_handler(struct httpd *httpd,
			struct httpd_con_addr *addr,
			const char *method,
			const char *upload_data,
			const char *user_uuid,
			const char *device_uuid,
			unsigned int *http_status)
{
	return sky_dev_cmd_handler(sky_chargestop,
				   httpd, addr, method, upload_data, user_uuid,
				   device_uuid, http_status);
}

static struct MHD_Response *
sky_cover_open_handler(struct httpd *httpd,
		       struct httpd_con_addr *addr,
		       const char *method,
		       const char *upload_data,
		       const char *user_uuid,
		       const char *device_uuid,
		       unsigned int *http_status)
{
	return sky_dev_cmd_handler(sky_coveropen,
				   httpd, addr, method, upload_data, user_uuid,
				   device_uuid, http_status);
}

static struct MHD_Response *
sky_cover_close_handler(struct httpd *httpd,
			struct httpd_con_addr *addr,
			const char *method,
			const char *upload_data,
			const char *user_uuid,
			const char *device_uuid,
			unsigned int *http_status)
{
	return sky_dev_cmd_handler(sky_coverclose,
				   httpd, addr, method, upload_data, user_uuid,
				   device_uuid, http_status);
}

struct httpd_handler {
	const char      *url;
	httpd_handler_t *handler;
} handlers[] = {
	/* Status & state */
	{ "/devs-list",       sky_devs_list_handler       },
	{ "/charging-params", sky_charging_params_handler },
	{ "/charging-state",  sky_charging_state_handler  },

	/* Commands */
	{ "/charge-start",    sky_charge_start_handler },
	{ "/charge-stop",     sky_charge_stop_handler  },
	{ "/cover-open",      sky_cover_open_handler   },
	{ "/cover-close",     sky_cover_close_handler  },
};

static inline int cmp_timeouts(const struct rb_node *a_,
			       const struct rb_node *b_)
{
	struct httpd_con_addr *a;
	struct httpd_con_addr *b;

	a = container_of(a_, typeof(*a), tentry);
	b = container_of(b_, typeof(*b), tentry);

	return (a->expire_ms < b->expire_ms ? -1 :
		a->expire_ms > b->expire_ms ? 1 : 0);
}

static void httpd_addr_rbtree_insert(struct httpd *httpd,
				     struct httpd_con_addr *addr)
{
	struct rb_node **this = &httpd->addrs_root.rb_node;
	struct rb_node *parent = NULL;
	struct rb_node *new;
	int cmp;

	new = &addr->tentry;
	while (*this) {
		parent = *this;
		cmp = cmp_timeouts(new, *this);

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
	rb_insert_color(new, &httpd->addrs_root);
}

static struct httpd_con_addr *__httpd_find_con_addr(struct httpd *httpd,
						    const struct sockaddr *saddr,
						    unsigned int *lookup_hint)
{
	const struct sockaddr_in *saddr_in;
	struct hash_entry *he;

	if (saddr->sa_family != AF_INET) {
		/* Hm, we accept only IP4 */
		sky_err("%s: incorrect sa_family\n", __func__);
		return NULL;
	}
	saddr_in = (typeof(saddr_in))saddr;
	he = hash_lookup(&httpd->addrs_hash, &saddr_in->sin_addr,
			 sizeof(saddr_in->sin_addr), lookup_hint);
	if (!he)
		return NULL;

	return container_of(he, struct httpd_con_addr, hentry);
}

static struct httpd_con_addr *httpd_find_con_addr(struct httpd *httpd,
						  const struct sockaddr *saddr)
{
	return __httpd_find_con_addr(httpd, saddr, NULL);
}

static struct httpd_con_addr *
httpd_get_or_create_con_addr(struct httpd *httpd,
			     struct MHD_Connection *con)
{
	const union MHD_ConnectionInfo *info;
	struct httpd_con_addr *addr;

	unsigned int lookup_hint = 0;

	info = MHD_get_connection_info(con, MHD_CONNECTION_INFO_CLIENT_ADDRESS);
	assert(info);

	addr = __httpd_find_con_addr(httpd, info->client_addr, &lookup_hint);
	if (!addr) {
		addr = calloc(1, sizeof(*addr));
		if (!addr) {
			sky_err("%s: no memory\n", __func__);
			return NULL;
		}
		addr->sin_addr = ((struct sockaddr_in *)info->client_addr)->sin_addr;
		addr->bad_auth_attempts = 0;
		RB_CLEAR_NODE(&addr->tentry);
		hash_entry_init(&addr->hentry, &addr->sin_addr,
				sizeof(addr->sin_addr));
		hash_insert(&addr->hentry, &lookup_hint, &httpd->addrs_hash);
	}
	addr->refs++;

	return addr;
}

static void httpd_get_con_addr(struct httpd_con_addr *addr)
{
	assert(addr->refs);
	addr->refs++;
}

static void httpd_put_con_addr(struct httpd *httpd, struct httpd_con_addr *addr)
{
	if (!addr->refs) {
		sky_err("invalid refs!\n");
		assert(0);
		return;
	}
	if (--addr->refs)
		return;

	if (!RB_EMPTY_NODE(&addr->tentry))
		rb_erase(&addr->tentry, &httpd->addrs_root);
	hash_remove(&addr->hentry);
	free(addr);
}

static void httpd_mark_addr_as_suspicious(struct httpd *httpd,
					  struct httpd_con_addr *addr)
{
	bool account_refs = true;

	if (!addr->bad_auth_attempts && !RB_EMPTY_NODE(&addr->tentry)) {
		/*
		 * Mark addr as suspicious for the first time, but address
		 * can be already in the tree because of rate-limiting, so
		 * erase first
		 */
		rb_erase_init(&addr->tentry, &httpd->addrs_root);
		account_refs = false;
	}
	if (RB_EMPTY_NODE(&addr->tentry)) {
		addr->expire_ms = msecs_epoch() + CHILL_OUT_BAD_AUTH_MS;
		httpd_addr_rbtree_insert(httpd, addr);
		if (account_refs)
			httpd_get_con_addr(addr);
	}

	addr->bad_auth_attempts++;
}

static bool httpd_too_many_bad_auth_attempts(struct httpd_con_addr *addr)
{
	return addr->bad_auth_attempts >= MAX_BAD_AUTH_ATTEMPTS;
}

static bool httpd_authenticate(struct httpd *httpd, struct httpd_con_addr *addr,
			       const char *user_uuid)
{
	uuid_t uuid;
	if (user_uuid && !uuid_parse(user_uuid, uuid))
		/* TODO: need to add real authentication */
		return true;

	httpd_mark_addr_as_suspicious(httpd, addr);
	return false;
}

static bool httpd_limit_requests_rate(struct httpd *httpd,
				      struct httpd_con_addr *addr)
{
	unsigned long long period_ms, now = msecs_epoch();

	period_ms = _1_SEC_IN_MS / MAX_REQUESTS_RATE;
	if (now < addr->last_access_ms + period_ms)
		return true;

	addr->last_access_ms = now;

	if (RB_EMPTY_NODE(&addr->tentry)) {
		addr->expire_ms = now + _1_SEC_IN_MS;
		httpd_addr_rbtree_insert(httpd, addr);
		httpd_get_con_addr(addr);
	}

	return false;
}

static int
httpd_accept_policy_call(void *arg,
			 const struct sockaddr *saddr,
			 socklen_t addrlen)
{
	struct httpd *httpd = arg;
	struct httpd_con_addr *addr;

	addr = httpd_find_con_addr(httpd, saddr);
	if (!addr)
		/* No found address - no restrictions to accept */
		return MHD_YES;

	if (httpd_too_many_bad_auth_attempts(addr))
		/* Client has to chill out a bit, too many bad auth attempts */
		return MHD_NO;

	return MHD_YES;
}

static void
httpd_request_completed_call(void *arg,
			     struct MHD_Connection *connection,
			     void **ctx,
			     enum MHD_RequestTerminationCode toe)
{
	struct httpd *httpd = arg;
	struct httpd_con_addr *addr = *ctx;

	if (addr)
		httpd_put_con_addr(httpd, addr);
	*ctx = NULL;
}

static int
httpd_access_handler_call(void *arg,
			  struct MHD_Connection *con,
			  const char *url,
			  const char *method,
			  const char *version,
			  const char *upload_data,
			  size_t *upload_data_size,
			  void **ctx)
{
	struct httpd *httpd = arg;
	struct httpd_con_addr *addr;
	struct MHD_Response *resp;

	const char *user_uuid;
	const char *device_uuid;

	unsigned int http_status;
	int i, ret;

	addr = httpd_get_or_create_con_addr(httpd, con);
	if (!addr)
		return MHD_NO;

	*ctx = addr;
	user_uuid = MHD_lookup_connection_value(con, MHD_GET_ARGUMENT_KIND,
						"user-uuid");
	device_uuid = MHD_lookup_connection_value(con, MHD_GET_ARGUMENT_KIND,
						  "device-uuid");
	if (httpd_limit_requests_rate(httpd, addr)) {
		http_status = HTTP_TOO_MANY_REQUESTS_STATUS;
		resp = MHD_create_response_from_buffer(0, NULL, MHD_RESPMEM_PERSISTENT);
		goto out;
	}
	if (!httpd_authenticate(httpd, addr, user_uuid)) {
		http_status = MHD_HTTP_FORBIDDEN;
		resp = MHD_create_response_from_buffer(0, NULL, MHD_RESPMEM_PERSISTENT);
		goto out;
	}

	resp = NULL;
	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		struct httpd_handler *h = &handlers[i];

		if (strcmp(h->url, url))
			continue;

		resp = h->handler(httpd, addr, method, upload_data,
				  user_uuid, device_uuid, &http_status);
		break;
	}
	if (!resp) {
		/* No url found, return HELP page */
		resp = MHD_create_response_from_buffer(strlen(HELP), HELP,
						       MHD_RESPMEM_PERSISTENT);
		if (!resp)
			return MHD_NO;

		http_status = MHD_HTTP_OK;
		MHD_add_response_header(resp, MHD_HTTP_HEADER_CONTENT_TYPE,
					"text/html");
	}
out:
	ret = MHD_queue_response(con, http_status, resp);
	MHD_destroy_response(resp);

	return ret;
}

static void httpd_init(struct httpd *httpd)
{
	hash_table_init(&httpd->addrs_hash);
	httpd->addrs_root = RB_ROOT;
}

static struct timeval *
msecs_to_tv(struct timeval *tv, unsigned long long msecs)
{
	if (msecs == ULONG_MAX)
		return NULL;
	tv->tv_sec = msecs / 1000;
	tv->tv_usec = (msecs - (tv->tv_sec * 1000)) * 1000;
	return tv;
}

static struct timeval *
httpd_expire_suspicious_addrs(struct httpd *httpd, struct timeval *tv)
{
	struct httpd_con_addr *addr;
	struct rb_node *node;

	unsigned long long now, timeout;

	if (!MHD_get_timeout(httpd->mhd, &timeout))
		timeout = ULONG_MAX;

	if (RB_EMPTY_ROOT(&httpd->addrs_root))
		return msecs_to_tv(tv, timeout);

	now = msecs_epoch();
	while ((node = rb_first(&httpd->addrs_root))) {
		addr = container_of(node, typeof(*addr), tentry);
		if (now < addr->expire_ms) {
			timeout = min(timeout, addr->expire_ms - now);
			break;
		}
		rb_erase_init(&addr->tentry, &httpd->addrs_root);
		httpd_put_con_addr(httpd, addr);
	}

	return msecs_to_tv(tv, timeout);
}

/**
 * Call with the port number as the only argument.
 * Never terminates (other than by signals, such as CTRL-C).
 */
int main (int argc, char **argv)
{
	struct addrinfo hints, *addrinfo;
	struct httpd httpd;

	int rc;

	SKY_FD_SET_T(rs);
	SKY_FD_SET_T(ws);
	SKY_FD_SET_T(es);

	httpd_init(&httpd);

	rc = cli_parse(argc, argv, &httpd.cli);
	if (rc) {
		sky_err("%s\n", cli_usage);
		return 1;
	}

	/* initialize PRNG */
	srand(time(NULL));

	memset(&hints, 0, sizeof(struct addrinfo));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE;

	rc = getaddrinfo(httpd.cli.httpaddr, httpd.cli.httpport,
			 &hints, &addrinfo);
	if (rc) {
		sky_err("getaddrinfo: %s\n", gai_strerror(rc));
		return 1;
	}

	/*
	 * Daemonize before creating zmq socket, ZMQ is written by
	 * people who are not able to deal with fork() gracefully.
	 */
	if (httpd.cli.daemon)
		sky_daemonize(httpd.cli.pidf);

	httpd.mhd = MHD_start_daemon(MHD_USE_DEBUG, 0,
			&httpd_accept_policy_call, &httpd,
			&httpd_access_handler_call, &httpd,
			MHD_OPTION_CONNECTION_TIMEOUT, 10,
			MHD_OPTION_SOCK_ADDR, addrinfo->ai_addr,
			MHD_OPTION_NOTIFY_COMPLETED, &httpd_request_completed_call, &httpd,
			MHD_OPTION_END);
	freeaddrinfo(addrinfo);
	if (!httpd.mhd) {
		sky_err("Can't create HTTP server instance\n");
		return 1;
	}
	while (1) {
		struct timeval tv, *tvp;
		MHD_socket max = 0;

		SKY_FD_ZERO(&rs);
		SKY_FD_ZERO(&ws);
		SKY_FD_ZERO(&es);
		if (!MHD_get_fdset2(httpd.mhd, (fd_set*)&rs, (fd_set*)&ws,
				    (fd_set*)&es, &max,  MAX_CONNECTIONS)) {
			sky_err("MHD_get_fdset: returns err\n");
			break; /* fatal internal error */
		}
		tvp = httpd_expire_suspicious_addrs(&httpd, &tv);
		if (-1 == select(max + 1, (fd_set*)&rs, (fd_set*)&ws, (fd_set*)&es, tvp)) {
			if (errno != EINTR) {
				sky_err("select failed: %s\n", strerror(errno));
				break;
			}
		}
		MHD_run(httpd.mhd);
	}
	MHD_stop_daemon(httpd.mhd);

	return 0;
}

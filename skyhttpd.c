/*
 * Copyright (C) 2021-2022 Skycharge GmbH
 * Author: Roman Penyaev <r.peniaev@gmail.com>
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
#include "libskycharge.h"
#include "libskycharge-pri.h"
#include "types.h"
#include "rbtree.h"
#include "hash.h"
#include "version.h"
#include "daemon.h"
#include "skyproto.h"
#include "errnoname.h"

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
 *     /gps-data        - sky_gpsdata()
 *     /scan-resume     - sky_scanresume()
 *     /scan-stop       - sky_scanstop()
 *     /droneport-open  - sky_droneport_open()
 *     /droneport-close - sky_droneport_close()
 *     /droneport-state - sky_droneport_state()
 */

#define HELP								\
	"<html><head><title>Skycharge API</title></head>\n"		\
	"<body>Please contact <a href=\"mailto:support@skycharge.de\">support@skycharge.de</a> " \
	"for detailed API description.</body>\n</html>\n"

enum {
	HTTP_TOO_MANY_REQUESTS_STATUS = 429,

	MAX_CONNECTIONS       = 512,
	MAX_BAD_AUTH_ATTEMPTS = 3,
	CHILL_OUT_BAD_AUTH_MS = 10000, /* 10 secs should be enough */
	MAX_REQUESTS_RATE     = 3,     /* per 1000 ms */
	REQUEST_TIMEOUT_MS    = 10000,
	_1_SEC_IN_MS          = 1000,
};

#define SKY_FD_SET_T(name) \
	unsigned long name[MAX_CONNECTIONS / (8 * sizeof(long))]
#define SKY_FD_SET(fd, set) \
	set[(fd) / (8 * sizeof(long))] |= 1<<((fd) % (8 * sizeof(long)))
#define SKY_FD_ISSET(fd, set) \
	set[(fd) / (8 * sizeof(long))] & (1<<((fd) % (8 * sizeof(long))))
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
	struct cli           cli;
	struct sky_conf      conf;
	struct MHD_Daemon    *mhd;
	struct sky_async     *async;
	struct hash_table    addrs_hash; /* addrs hashed by sin_addr */
	struct rb_root       addrs_root; /* addrs to expire */
	struct rb_root       reqs_root;  /* requests to expire */
	struct MHD_Response  *emergency_resp;
	bool                 need_processing;
};

struct httpd_request;
typedef int (create_dev_request_t)(struct httpd_request *devslist_req);

struct httpd_request {
	struct httpd            *httpd;
	struct MHD_Connection   *httpcon;
	struct sky_async_req    skyreq;
	union sky_async_storage skystruct;
	uuid_t                  user_uuid;
	uuid_t                  device_uuid;
	create_dev_request_t    *create_dev_req;
	struct rb_node          tentry;    /* httpd->reqs_root */
	unsigned long long      expire_ms; /* when this request times out */
};

typedef int (httpd_handler_t)(struct httpd *,
			      struct MHD_Connection *con,
			      const uuid_t user_uuid,
			      const uuid_t device_uuid);

static bool http_is_get(const char *method)
{
	return !strcmp(method, MHD_HTTP_METHOD_GET);
}

static int sky_find_and_open_dev(const struct sky_dev_desc *devdescs,
				 const uuid_t device_uuid,
				 struct sky_dev **pdev)
{
	const struct sky_dev_desc *devdesc;

	if (!device_uuid)
		return -ENODEV;

	foreach_devdesc(devdesc, devdescs) {
		if (memcmp(devdesc->dev_uuid, device_uuid, sizeof(uuid_t)))
			continue;

		return sky_devopen(devdesc, pdev);
	}

	return -ENODEV;
}

__attribute__((format(printf, 4, 5)))
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

#define rbnode_compare(type, member, cmpmember, a_, b_) ({ \
	type *a;					\
	type *b;					\
							\
	a = container_of(a_, typeof(*a), tentry);	\
	b = container_of(b_, typeof(*b), tentry);	\
							\
	(a->cmpmember < b->cmpmember ? -1 :		\
	 a->cmpmember > b->cmpmember ? 1 : 0);		\
})

static void httpd_request_insert(struct httpd *httpd,
				 struct httpd_request *req)
{
	struct rb_node **this = &httpd->reqs_root.rb_node;
	struct rb_node *parent = NULL;
	struct rb_node *new;
	int cmp;

	req->expire_ms = msecs_epoch() + REQUEST_TIMEOUT_MS;

	new = &req->tentry;
	while (*this) {
		parent = *this;
		cmp = rbnode_compare(typeof(*req), tentry,
				     expire_ms, new, *this);

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
	rb_insert_color(new, &httpd->reqs_root);
}

static int
httpd_create_emergency_response(struct httpd *httpd)
{
	httpd->emergency_resp = MHD_create_response_from_buffer(0, NULL,
					MHD_RESPMEM_PERSISTENT);
	if (!httpd->emergency_resp)
		return -ENOMEM;

	return 0;
}

static void
httpd_destroy_emergency_response(struct httpd *httpd)
{
	MHD_destroy_response(httpd->emergency_resp);
}

static bool httpd_get_and_clear_need_processing(struct httpd *httpd)
{
	bool need_processing;

	need_processing = httpd->need_processing;
	httpd->need_processing = false;
	return need_processing;
}

static void httpd_set_need_processing(struct httpd *httpd)
{
	httpd->need_processing = true;
}

static int
httpd_queue_response(struct httpd *httpd, struct MHD_Connection *con,
		     int http_status, struct MHD_Response *resp)
{
	httpd_set_need_processing(httpd);
	return MHD_queue_response(con, http_status, resp);
}

static void
httpd_queue_emergency_response(struct httpd *httpd,
			       struct MHD_Connection *con)
{
	(void)httpd_queue_response(httpd, con, MHD_HTTP_SERVICE_UNAVAILABLE,
				   httpd->emergency_resp);
}

static void
httpd_queue_response_and_free_buffer(struct httpd *httpd,
				     struct MHD_Connection *con,
				     char *buffer, size_t len)
{
	struct MHD_Response *resp;

	resp = MHD_create_response_from_buffer(len, buffer, MHD_RESPMEM_MUST_FREE);
	if (!resp) {
		free(buffer);
		httpd_queue_emergency_response(httpd, con);
		return;
	}
	(void)httpd_queue_response(httpd, con, MHD_HTTP_OK, resp);
	MHD_destroy_response(resp);
}

static void
httpd_queue_json_response(struct httpd *httpd,
			  struct MHD_Connection *con,
			  int rc)
{
	char *buffer = NULL;
	int off = 0, size = 0;
	int ret;

	ret = snprintf_buffer(&buffer, &off, &size, "{\n\t\"errno\": \"%s\"\n}\n",
			      errnoname_unsafe(-rc));
	if (ret) {
		httpd_queue_emergency_response(httpd, con);
		return;
	}
	httpd_queue_response_and_free_buffer(httpd, con, buffer, off);
}

static int
devs_list_create_composite_request(struct httpd *httpd,
				   struct MHD_Connection *con,
				   const uuid_t user_uuid,
				   const uuid_t device_uuid,
				   create_dev_request_t *create_dev_req,
				   sky_async_completion_t *skycompletion)
{
	struct httpd_request *req;
	int rc;

	req = calloc(1, sizeof(*req));
	if (!req)
		return -ENOMEM;

	req->httpd   = httpd;
	req->httpcon = con;
	req->create_dev_req = create_dev_req;
	memcpy(req->user_uuid, user_uuid, sizeof(uuid_t));
	memcpy(req->device_uuid, device_uuid, sizeof(uuid_t));

	rc = sky_asyncreq_devslist(httpd->async, &req->skystruct.devdescs,
				   &req->skyreq);
	if (rc) {
		free(req);
		return rc;
	}

	sky_asyncreq_useruuidset(&req->skyreq, req->user_uuid);
	sky_asyncreq_completionset(&req->skyreq, skycompletion);
	httpd_request_insert(httpd, req);
	httpd_set_need_processing(httpd);

	return 0;
}

static void
devs_list_composite_skycompletion(struct sky_async_req *skyreq)
{
	struct httpd_request *req;
	int rc;

	req = container_of(skyreq, typeof(*req), skyreq);
	rc = skyreq->out.rc;

	if (!rc && req->create_dev_req) {
		rc = req->create_dev_req(req);
	}
	if (rc) {
		httpd_queue_json_response(req->httpd, req->httpcon, rc);
		MHD_resume_connection(req->httpcon);
	}
	sky_devsfree(req->skystruct.devdescs);
	rb_erase(&req->tentry, &req->httpd->reqs_root);
	free(req);
}

static void
dev_cmd_skycompletion(struct sky_async_req *skyreq)
{
	struct httpd_request *req;
	int rc;

	req = container_of(skyreq, typeof(*req), skyreq);
	rc = skyreq->out.rc;
	sky_devclose(skyreq->dev);

	/*
	 * Prior 1.3.0 version BMS, GPS or DP code returns -ENODEV
	 * if operation is not supported. This is a bad practice,
	 * because client can't destinguish real -ENODEV error
	 * "device was not found" from "device was found, but op
	 * is not supported". Change the return code here, which
	 * is safe.
	 */
	if (rc == -ENODEV)
		rc = -EOPNOTSUPP;

	httpd_queue_json_response(req->httpd, req->httpcon, rc);
	MHD_resume_connection(req->httpcon);
	rb_erase(&req->tentry, &req->httpd->reqs_root);
	free(req);
}

static int
dev_cmd_create_request(int (*devcmd)(struct sky_async *async,
				     struct sky_dev *dev,
				     struct sky_async_req *req),
		       struct httpd *httpd,
		       struct MHD_Connection *con,
		       const uuid_t user_uuid,
		       const uuid_t device_uuid,
		       struct sky_dev_desc *devdescs)
{
	struct httpd_request *req;
	struct sky_dev *dev;
	int rc;

	rc = sky_find_and_open_dev(devdescs, device_uuid, &dev);
	if (rc)
		return rc;

	req = calloc(1, sizeof(*req));
	if (!req) {
		sky_devclose(dev);
		return -ENOMEM;
	}

	req->httpd   = httpd;
	req->httpcon = con;
	memcpy(req->user_uuid, user_uuid, sizeof(uuid_t));
	memcpy(req->device_uuid, device_uuid, sizeof(uuid_t));

	rc = devcmd(httpd->async, dev, &req->skyreq);
	if (rc) {
		free(req);
		sky_devclose(dev);
		return rc;
	}

	sky_asyncreq_useruuidset(&req->skyreq, req->user_uuid);
	sky_asyncreq_completionset(&req->skyreq, dev_cmd_skycompletion);
	httpd_request_insert(httpd, req);
	httpd_set_need_processing(httpd);

	return 0;
}

static int
scan_resume_create_request(struct httpd_request *req)
{
	return dev_cmd_create_request(sky_asyncreq_scanresume,
				      req->httpd, req->httpcon,
				      req->user_uuid, req->device_uuid,
				      req->skystruct.devdescs);
}

static int
scan_stop_create_request(struct httpd_request *req)
{
	return dev_cmd_create_request(sky_asyncreq_scanstop,
				      req->httpd, req->httpcon,
				      req->user_uuid, req->device_uuid,
				      req->skystruct.devdescs);
}

static int
droneport_open_create_request(struct httpd_request *req)
{
	return dev_cmd_create_request(sky_asyncreq_droneport_open,
				      req->httpd, req->httpcon,
				      req->user_uuid, req->device_uuid,
				      req->skystruct.devdescs);
}

static int
cover_close_create_request(struct httpd_request *req)
{
	return dev_cmd_create_request(sky_asyncreq_droneport_close,
				      req->httpd, req->httpcon,
				      req->user_uuid, req->device_uuid,
				      req->skystruct.devdescs);
}

static void devs_list_skycompletion(struct sky_async_req *skyreq)
{
	struct httpd_request *req;

	char *buffer = NULL;
	int off = 0, size = 0;
	int ret;

	req = container_of(skyreq, typeof(*req), skyreq);
	ret = snprintf_buffer(&buffer, &off, &size,
			      "{\n\t\"errno\": \"%s\",\n\t\"devices\": [\n",
			      errnoname_unsafe(-skyreq->out.rc));
	if (ret)
		goto err;

	if (!skyreq->out.rc) {
		struct sky_dev_desc *devdesc;

		foreach_devdesc(devdesc, req->skystruct.devdescs) {
			char dev_uuid[37];

			uuid_unparse(devdesc->dev_uuid, dev_uuid);
			ret = snprintf_buffer(&buffer, &off, &size,
					      "\t\t{\n"
					      "\t\t\t\"device-uuid\": \"%s\",\n"
					      "\t\t\t\"device-name\": \"%s\"\n"
					      "\t\t}%s\n",
					      dev_uuid, devdesc->dev_name,
					      devdesc->next ? "," : "");
			if (ret)
				goto err;
		}
	}
	ret = snprintf_buffer(&buffer, &off, &size, "\t]\n}\n");
	if (ret)
		goto err;

	(void)httpd_queue_response_and_free_buffer(req->httpd, req->httpcon,
						   buffer, off);
out:
	sky_devsfree(req->skystruct.devdescs);
	MHD_resume_connection(req->httpcon);
	rb_erase(&req->tentry, &req->httpd->reqs_root);
	free(req);
	return;

err:
	free(buffer);
	goto out;
}

static int
devs_list_http_handler(struct httpd *httpd,
		       struct MHD_Connection *con,
		       const uuid_t user_uuid,
		       const uuid_t device_uuid)
{
	return devs_list_create_composite_request(httpd, con, user_uuid, device_uuid,
						  NULL, devs_list_skycompletion);
}

static void charging_params_skycompletion(struct sky_async_req *skyreq)
{
	struct sky_dev_params *params;
	struct sky_dev_desc devdesc;
	struct httpd_request *req;
	char *buffer = NULL;
	int off = 0, size = 0;
	int i, ret, rc;

	req = container_of(skyreq, typeof(*req), skyreq);
	rc = skyreq->out.rc;
	if (!rc)
		(void)sky_devinfo(skyreq->dev, &devdesc);
	sky_devclose(skyreq->dev);

	params = &req->skystruct.params;

	ret = snprintf_buffer(&buffer, &off, &size,
			      "{\n\t\"errno\": \"%s\",\n\t\"params\": {\n",
			      errnoname_unsafe(-rc));
	if (ret)
		goto err;

	if (!rc) {
		unsigned int nr_params;

		if (devdesc.dev_type == SKY_MUX_HW1)
			nr_params = SKY_HW1_NUM_DEVPARAM;
		else
			nr_params = SKY_HW2_NUM_DEVPARAM;

		for (i = 0; i < nr_params; i++) {
			ret = snprintf_buffer(&buffer, &off, &size,
					      "\t\t\t\"%s\": %d%s\n",
					      sky_devparam_to_str(devdesc.dev_type, i),
					      params->dev_params[i],
					      i + 1 < nr_params ? "," : "");
			if (ret)
				goto err;
		}
	}
	ret = snprintf_buffer(&buffer, &off, &size, "\t}\n}\n");
	if (ret)
		goto err;

	(void)httpd_queue_response_and_free_buffer(req->httpd, req->httpcon,
						   buffer, off);
out:
	MHD_resume_connection(req->httpcon);
	rb_erase(&req->tentry, &req->httpd->reqs_root);
	free(req);
	return;

err:
	free(buffer);
	goto out;
}

static void charging_state_skycompletion(struct sky_async_req *skyreq)
{
	struct sky_charging_state *state;
	struct sky_dev_desc devdesc;
	struct httpd_request *req;
	char *buffer = NULL;
	int off = 0, size = 0;
	int ret, rc;

	req = container_of(skyreq, typeof(*req), skyreq);
	rc = skyreq->out.rc;
	if (!rc)
		(void)sky_devinfo(skyreq->dev, &devdesc);
	sky_devclose(skyreq->dev);

	state = &req->skystruct.charging_state;

	ret = snprintf_buffer(&buffer, &off, &size,
			      "{\n\t\"errno\": \"%s\",\n\t\"charging-state\": {\n",
			      errnoname_unsafe(-rc));
	if (ret)
		goto err;

	if (!rc) {
		ret = snprintf_buffer(&buffer, &off, &size,
				      "\t\t\t\"state\": \"%s\",\n"
				      "\t\t\t\"reason\": \"%s\",\n"
				      "\t\t\t\"voltage-mV\": %d,\n"
				      "\t\t\t\"current-mA\": %d,\n"
				      "\t\t\t\"charging-time-sec\": %d,\n"
				      "\t\t\t\"state-of-charge-perc\": %d,\n"
				      "\t\t\t\"until-full-charge-time-sec\": %d,\n"
				      "\t\t\t\"source-temperature-C\": %d,\n"
				      "\t\t\t\"sink-temperature-C\": %d,\n"
				      "\t\t\t\"energy-mWh\": %d,\n"
				      "\t\t\t\"charge-mAh\": %d,\n"
				      "\t\t\t\"tx-bytes\": %d,\n"
				      "\t\t\t\"tx-packets\": %d,\n"
				      "\t\t\t\"tx-err-bytes\": %d,\n"
				      "\t\t\t\"tx-err-packets\": %d,\n"
				      "\t\t\t\"rx-bytes\": %d,\n"
				      "\t\t\t\"rx-packets\": %d,\n"
				      "\t\t\t\"rx-err-bytes\": %d,\n"
				      "\t\t\t\"rx-err-packets\": %d\n",
				      sky_devstate_to_str(devdesc.dev_type,
							  state->dev_hw_state),
				      sky_devreason_to_str(state->dev_hw_reason),
				      state->voltage_mV,
				      state->current_mA,
				      state->charging_secs,
				      state->state_of_charge,
				      state->until_full_secs,
				      state->mux_temperature_C,
				      state->sink_temperature_C,
				      state->energy_mWh,
				      state->charge_mAh,
				      state->tx.bytes,
				      state->tx.packets,
				      state->tx.err_bytes,
				      state->tx.err_packets,
				      state->rx.bytes,
				      state->rx.packets,
				      state->rx.err_bytes,
				      state->rx.err_packets);
		if (ret)
			goto err;
	}
	ret = snprintf_buffer(&buffer, &off, &size, "\t}\n}\n");
	if (ret)
		goto err;

	(void)httpd_queue_response_and_free_buffer(req->httpd, req->httpcon,
						   buffer, off);
out:
	MHD_resume_connection(req->httpcon);
	rb_erase(&req->tentry, &req->httpd->reqs_root);
	free(req);
	return;

err:
	free(buffer);
	goto out;
}

static void droneport_state_skycompletion(struct sky_async_req *skyreq)
{
	struct sky_droneport_state *state;
	struct httpd_request *req;
	char *buffer = NULL;
	int off = 0, size = 0;
	int ret, rc;

	req = container_of(skyreq, typeof(*req), skyreq);
	rc = skyreq->out.rc;
	sky_devclose(skyreq->dev);

	state = &req->skystruct.dp_state;

	ret = snprintf_buffer(&buffer, &off, &size,
			      "{\n\t\"errno\": \"%s\",\n\t\"droneport-state\": {\n",
			      errnoname_unsafe(-rc));
	if (ret)
		goto err;

	if (!rc) {
		ret = snprintf_buffer(&buffer, &off, &size,
				      "\t\t\t\"is-ready\": %s,\n"
				      "\t\t\t\"is-opened\": %s,\n"
				      "\t\t\t\"is-closed\": %s,\n"
				      "\t\t\t\"in-progress\": %s,\n"
				      "\t\t\t\"landing-err\": %s\n",
				      state->status & SKY_DP_IS_READY ? "true" : "false",
				      state->status & SKY_DP_IS_OPENED ? "true" : "false",
				      state->status & SKY_DP_IS_CLOSED ? "true" : "false",
				      state->status & SKY_DP_IN_PROGRESS ? "true" : "false",
				      state->status & SKY_DP_LANDING_ERROR ? "true" : "false");
		if (ret)
			goto err;
	}
	ret = snprintf_buffer(&buffer, &off, &size, "\t}\n}\n");
	if (ret)
		goto err;

	(void)httpd_queue_response_and_free_buffer(req->httpd, req->httpcon,
						   buffer, off);
out:
	MHD_resume_connection(req->httpcon);
	rb_erase(&req->tentry, &req->httpd->reqs_root);
	free(req);
	return;

err:
	free(buffer);
	goto out;
}

static void gps_data_skycompletion(struct sky_async_req *skyreq)
{
	struct sky_gpsdata *gpsdata;
	struct httpd_request *req;
	char *buffer = NULL;
	int off = 0, size = 0;
	int ret, rc;

	req = container_of(skyreq, typeof(*req), skyreq);
	rc = skyreq->out.rc;
	sky_devclose(skyreq->dev);

	gpsdata = &req->skystruct.gpsdata;

	ret = snprintf_buffer(&buffer, &off, &size,
			      "{\n\t\"errno\": \"%s\",\n\t\"gps-data\": {\n",
			      errnoname_unsafe(-rc));
	if (ret)
		goto err;

	if (!rc) {
		ret = snprintf_buffer(&buffer, &off, &size,
				      "\t\t\t\"status\": \"%s\",\n"
				      "\t\t\t\"satellites-used\": %d,\n"
				      "\t\t\t\"dop\": {\n"
				      "\t\t\t\t\"xdop\": \"%f\",\n"
				      "\t\t\t\t\"ydop\": \"%f\",\n"
				      "\t\t\t\t\"pdop\": \"%f\",\n"
				      "\t\t\t\t\"hdop\": \"%f\",\n"
				      "\t\t\t\t\"vdop\": \"%f\",\n"
				      "\t\t\t\t\"tdop\": \"%f\",\n"
				      "\t\t\t\t\"gdop\": \"%f\"\n"
				      "\t\t\t},\n"
				      "\t\t\t\"fix\": {\n"
				      "\t\t\t\t\"mode\": \"%s\",\n"
				      "\t\t\t\t\"time\": \"%f\",\n"
				      "\t\t\t\t\"latitude\": \"%f\",\n"
				      "\t\t\t\t\"longitude\": \"%f\",\n"
				      "\t\t\t\t\"altitude\": \"%f\"\n"
				      "\t\t\t}\n",
				      sky_gpsstatus_to_str(gpsdata->status),
				      gpsdata->satellites_used,
				      gpsdata->dop.xdop,
				      gpsdata->dop.ydop,
				      gpsdata->dop.pdop,
				      gpsdata->dop.hdop,
				      gpsdata->dop.vdop,
				      gpsdata->dop.tdop,
				      gpsdata->dop.gdop,
				      sky_gpsmode_to_str(gpsdata->fix.mode),
				      gpsdata->fix.time,
				      gpsdata->fix.latitude,
				      gpsdata->fix.longitude,
				      gpsdata->fix.altitude);
		if (ret)
			goto err;
	}
	ret = snprintf_buffer(&buffer, &off, &size, "\t}\n}\n");
	if (ret)
		goto err;

	(void)httpd_queue_response_and_free_buffer(req->httpd, req->httpcon,
						   buffer, off);
out:
	MHD_resume_connection(req->httpcon);
	rb_erase(&req->tentry, &req->httpd->reqs_root);
	free(req);
	return;

err:
	free(buffer);
	goto out;
}

static int
charging_params_create_request(struct httpd_request *devslist_req)
{
	struct sky_dev_desc *devdescs = devslist_req->skystruct.devdescs;
	struct httpd_request *req;
	struct sky_dev *dev;
	int rc;

	rc = sky_find_and_open_dev(devdescs, devslist_req->device_uuid, &dev);
	if (rc)
		return rc;

	req = calloc(1, sizeof(*req));
	if (!req) {
		sky_devclose(dev);
		return -ENOMEM;
	}

	req->httpd   = devslist_req->httpd;
	req->httpcon = devslist_req->httpcon;
	memcpy(req->user_uuid, devslist_req->user_uuid, sizeof(uuid_t));
	memcpy(req->device_uuid, devslist_req->device_uuid, sizeof(uuid_t));

	/* Get all params */
	req->skystruct.params.dev_params_bits = ~0;

	rc = sky_asyncreq_paramsget(req->httpd->async, dev,
				    &req->skystruct.params, &req->skyreq);
	if (rc) {
		free(req);
		sky_devclose(dev);
		return rc;
	}

	sky_asyncreq_useruuidset(&req->skyreq, req->user_uuid);
	sky_asyncreq_completionset(&req->skyreq, charging_params_skycompletion);
	httpd_request_insert(req->httpd, req);
	httpd_set_need_processing(req->httpd);

	return 0;
}

static int
charging_state_create_request(struct httpd_request *devslist_req)
{
	struct sky_dev_desc *devdescs = devslist_req->skystruct.devdescs;
	struct httpd_request *req;
	struct sky_dev *dev;
	int rc;

	rc = sky_find_and_open_dev(devdescs, devslist_req->device_uuid, &dev);
	if (rc)
		return rc;

	req = calloc(1, sizeof(*req));
	if (!req) {
		sky_devclose(dev);
		return -ENOMEM;
	}

	req->httpd   = devslist_req->httpd;
	req->httpcon = devslist_req->httpcon;
	memcpy(req->user_uuid, devslist_req->user_uuid, sizeof(uuid_t));
	memcpy(req->device_uuid, devslist_req->device_uuid, sizeof(uuid_t));

	rc = sky_asyncreq_chargingstate(devslist_req->httpd->async, dev,
					&req->skystruct.charging_state,
					&req->skyreq);
	if (rc) {
		free(req);
		sky_devclose(dev);
		return rc;
	}

	sky_asyncreq_useruuidset(&req->skyreq, req->user_uuid);
	sky_asyncreq_completionset(&req->skyreq, charging_state_skycompletion);
	httpd_request_insert(req->httpd, req);
	httpd_set_need_processing(req->httpd);

	return 0;
}

static int
droneport_state_create_request(struct httpd_request *devslist_req)
{
	struct sky_dev_desc *devdescs = devslist_req->skystruct.devdescs;
	struct httpd_request *req;
	struct sky_dev *dev;
	int rc;

	rc = sky_find_and_open_dev(devdescs, devslist_req->device_uuid, &dev);
	if (rc)
		return rc;

	req = calloc(1, sizeof(*req));
	if (!req) {
		sky_devclose(dev);
		return -ENOMEM;
	}

	req->httpd   = devslist_req->httpd;
	req->httpcon = devslist_req->httpcon;
	memcpy(req->user_uuid, devslist_req->user_uuid, sizeof(uuid_t));
	memcpy(req->device_uuid, devslist_req->device_uuid, sizeof(uuid_t));

	rc = sky_asyncreq_droneport_state(devslist_req->httpd->async, dev,
					  &req->skystruct.dp_state,
					  &req->skyreq);
	if (rc) {
		free(req);
		sky_devclose(dev);
		return rc;
	}

	sky_asyncreq_useruuidset(&req->skyreq, req->user_uuid);
	sky_asyncreq_completionset(&req->skyreq, droneport_state_skycompletion);
	httpd_request_insert(req->httpd, req);
	httpd_set_need_processing(req->httpd);

	return 0;
}

static int
gps_data_create_request(struct httpd_request *devslist_req)
{
	struct sky_dev_desc *devdescs = devslist_req->skystruct.devdescs;
	struct httpd_request *req;
	struct sky_dev *dev;
	int rc;

	rc = sky_find_and_open_dev(devdescs, devslist_req->device_uuid, &dev);
	if (rc)
		return rc;

	req = calloc(1, sizeof(*req));
	if (!req) {
		sky_devclose(dev);
		return -ENOMEM;
	}

	req->httpd   = devslist_req->httpd;
	req->httpcon = devslist_req->httpcon;
	memcpy(req->user_uuid, devslist_req->user_uuid, sizeof(uuid_t));
	memcpy(req->device_uuid, devslist_req->device_uuid, sizeof(uuid_t));

	rc = sky_asyncreq_gpsdata(devslist_req->httpd->async, dev,
				  &req->skystruct.gpsdata, &req->skyreq);
	if (rc) {
		free(req);
		sky_devclose(dev);
		return rc;
	}

	sky_asyncreq_useruuidset(&req->skyreq, req->user_uuid);
	sky_asyncreq_completionset(&req->skyreq, gps_data_skycompletion);
	httpd_request_insert(req->httpd, req);
	httpd_set_need_processing(req->httpd);

	return 0;
}

static int
charging_params_http_handler(struct httpd *httpd,
			     struct MHD_Connection *con,
			     const uuid_t user_uuid,
			     const uuid_t device_uuid)
{
	return devs_list_create_composite_request(httpd, con, user_uuid, device_uuid,
						  charging_params_create_request,
						  devs_list_composite_skycompletion);
}

static int
charging_state_http_handler(struct httpd *httpd,
			    struct MHD_Connection *con,
			    const uuid_t user_uuid,
			    const uuid_t device_uuid)
{
	return devs_list_create_composite_request(httpd, con, user_uuid, device_uuid,
						  charging_state_create_request,
						  devs_list_composite_skycompletion);
}

static int
droneport_state_http_handler(struct httpd *httpd,
			     struct MHD_Connection *con,
			     const uuid_t user_uuid,
			     const uuid_t device_uuid)
{
	return devs_list_create_composite_request(httpd, con, user_uuid, device_uuid,
						  droneport_state_create_request,
						  devs_list_composite_skycompletion);
}

static int
gps_data_http_handler(struct httpd *httpd,
		      struct MHD_Connection *con,
		      const uuid_t user_uuid,
		      const uuid_t device_uuid)
{
	return devs_list_create_composite_request(httpd, con, user_uuid, device_uuid,
						  gps_data_create_request,
						  devs_list_composite_skycompletion);
}

static int
scan_resume_http_handler(struct httpd *httpd,
			  struct MHD_Connection *con,
			  const uuid_t user_uuid,
			  const uuid_t device_uuid)
{
	return devs_list_create_composite_request(httpd, con, user_uuid, device_uuid,
						  scan_resume_create_request,
						  devs_list_composite_skycompletion);
}

static int
scan_stop_http_handler(struct httpd *httpd,
			  struct MHD_Connection *con,
			  const uuid_t user_uuid,
			  const uuid_t device_uuid)
{
	return devs_list_create_composite_request(httpd, con, user_uuid, device_uuid,
						  scan_stop_create_request,
						  devs_list_composite_skycompletion);
}

static int
droneport_open_http_handler(struct httpd *httpd,
			    struct MHD_Connection *con,
			    const uuid_t user_uuid,
			    const uuid_t device_uuid)
{
	return devs_list_create_composite_request(httpd, con, user_uuid, device_uuid,
						  droneport_open_create_request,
						  devs_list_composite_skycompletion);
}

static int
droneport_close_http_handler(struct httpd *httpd,
			     struct MHD_Connection *con,
			     const uuid_t user_uuid,
			     const uuid_t device_uuid)
{
	return devs_list_create_composite_request(httpd, con, user_uuid, device_uuid,
						  cover_close_create_request,
						  devs_list_composite_skycompletion);
}

struct httpd_handler {
	const char      *url;
	httpd_handler_t *handler;
} handlers[] = {
	/* Status & state */
	{ "/devs-list",       devs_list_http_handler       },
	{ "/charging-params", charging_params_http_handler },
	{ "/charging-state",  charging_state_http_handler  },
	{ "/droneport-state", droneport_state_http_handler },
	{ "/gps-data",        gps_data_http_handler        },

	/* Commands */
	{ "/scan-resume",     scan_resume_http_handler     },
	{ "/scan-stop",       scan_stop_http_handler       },
	{ "/droneport-open",  droneport_open_http_handler  },
	{ "/droneport-close", droneport_close_http_handler },
};

static void httpd_insert_addr(struct httpd *httpd,
			      struct httpd_con_addr *addr)
{
	struct rb_node **this = &httpd->addrs_root.rb_node;
	struct rb_node *parent = NULL;
	struct rb_node *new;
	int cmp;

	new = &addr->tentry;
	while (*this) {
		parent = *this;
		cmp = rbnode_compare(typeof(*addr), tentry,
				     expire_ms, new, *this);

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
		httpd_insert_addr(httpd, addr);
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
			       const char *user_uuid_str, uuid_t user_uuid)
{
	if (user_uuid_str && !uuid_parse(user_uuid_str, user_uuid))
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
		httpd_insert_addr(httpd, addr);
		httpd_get_con_addr(addr);
	}

	return false;
}

#if MHD_VERSION >= 0x00097101
static enum MHD_Result
#else
static int
#endif
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

#if MHD_VERSION >= 0x00097101
static enum MHD_Result
#else
static int
#endif
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

	const char *user_uuid_str;
	const char *device_uuid_str;

	uuid_t user_uuid = {}, device_uuid = {};
	unsigned int http_status;
	int i, ret;

	addr = httpd_get_or_create_con_addr(httpd, con);
	if (!addr)
		return MHD_NO;

	if (!http_is_get(method)) {
		http_status = MHD_HTTP_BAD_REQUEST;
		goto err;
	}

	*ctx = addr;
	user_uuid_str = MHD_lookup_connection_value(con, MHD_GET_ARGUMENT_KIND,
						    "user-uuid");
	device_uuid_str = MHD_lookup_connection_value(con, MHD_GET_ARGUMENT_KIND,
						      "device-uuid");

	if (httpd_limit_requests_rate(httpd, addr)) {
		http_status = HTTP_TOO_MANY_REQUESTS_STATUS;
		goto err;
	}
	if (!httpd_authenticate(httpd, addr, user_uuid_str, user_uuid)) {
		http_status = MHD_HTTP_FORBIDDEN;
		goto err;
	}
	if (device_uuid_str && uuid_parse(device_uuid_str, device_uuid)) {
		http_status = MHD_HTTP_BAD_REQUEST;
		goto err;
	}
	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		struct httpd_handler *h = &handlers[i];
		int rc;

		if (strcmp(h->url, url))
			continue;

		rc = h->handler(httpd, con, user_uuid, device_uuid);
		if (rc)
			httpd_queue_json_response(httpd, con, rc);
		else
			MHD_suspend_connection(con);

		return MHD_YES;
	}

	/* No url found, return HELP page */
	resp = MHD_create_response_from_buffer(strlen(HELP), HELP,
					       MHD_RESPMEM_PERSISTENT);
	if (!resp)
		return MHD_NO;

	http_status = MHD_HTTP_NOT_FOUND;
	MHD_add_response_header(resp, MHD_HTTP_HEADER_CONTENT_TYPE,
				"text/html");
out:
	ret = httpd_queue_response(httpd, con, http_status, resp);
	MHD_destroy_response(resp);

	return ret;
err:
	resp = MHD_create_response_from_buffer(0, NULL, MHD_RESPMEM_PERSISTENT);
	goto out;
}

static void httpd_init(struct httpd *httpd)
{
	hash_table_init(&httpd->addrs_hash);
	httpd->addrs_root = RB_ROOT;
	httpd->reqs_root  = RB_ROOT;
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
httpd_expire_addrs_and_requests(struct httpd *httpd, struct timeval *tv)
{
	struct rb_node *node;

	unsigned long long now, timeout = ULONG_MAX;

	now = msecs_epoch();

	/* Expire suspicious addresses */
	while ((node = rb_first(&httpd->addrs_root))) {
		struct httpd_con_addr *addr;

		addr = container_of(node, typeof(*addr), tentry);
		if (now < addr->expire_ms) {
			timeout = min(timeout, addr->expire_ms - now);
			break;
		}
		rb_erase_init(&addr->tentry, &httpd->addrs_root);
		httpd_put_con_addr(httpd, addr);
	}

	/* Expire timeouted requests */
	while ((node = rb_first(&httpd->reqs_root))) {
		struct httpd_request *req;

		req = container_of(node, typeof(*req), tentry);
		if (now < req->expire_ms) {
			timeout = min(timeout, req->expire_ms - now);
			break;
		}
		sky_asyncreq_cancel(req->httpd->async, &req->skyreq);
		sky_devclose(req->skyreq.dev);
		rb_erase_init(&req->tentry, &httpd->reqs_root);
		httpd_queue_json_response(req->httpd, req->httpcon, -ETIMEDOUT);
		MHD_resume_connection(req->httpcon);
		free(req);
	}

	return msecs_to_tv(tv, timeout);
}

static void httpd_prepare_sky_conf(struct httpd *httpd)
{
	struct sky_conf *conf = &httpd->conf;
	struct cli *cli = &httpd->cli;

	sky_confinit(conf);

	conf->contype = SKY_REMOTE;
	conf->cliport = strtol(cli->skyport, NULL, 10);
	conf->subport = conf->cliport + 1;
	strncpy(conf->hostname, cli->skyaddr,
		sizeof(conf->hostname)-1);
}

/**
 * Call with the port number as the only argument.
 * Never terminates (other than by signals, such as CTRL-C).
 */
int main (int argc, char **argv)
{
	struct addrinfo hints, *addrinfo;
	struct httpd httpd;

	int rc, asyncfd;

	memset(&httpd, 0, sizeof(httpd));

	SKY_FD_SET_T(rs);
	SKY_FD_SET_T(ws);
	SKY_FD_SET_T(es);

	httpd_init(&httpd);

	rc = cli_parse(argc, argv, &httpd.cli);
	if (rc) {
		sky_err("%s\n", cli_usage);
		return 1;
	}

	httpd_prepare_sky_conf(&httpd);

	/* initialize PRNG */
	srand(time(NULL));

	memset(&hints, 0, sizeof(struct addrinfo));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE;

	/*
	 * Daemonize before creating zmq socket, ZMQ is written by
	 * people who are not able to deal with fork() gracefully.
	 */
	if (httpd.cli.daemon)
		sky_daemonize(httpd.cli.pidf);

	rc = getaddrinfo(httpd.cli.httpaddr, httpd.cli.httpport,
			 &hints, &addrinfo);
	if (rc) {
		sky_err("getaddrinfo: %s\n", gai_strerror(rc));
		return 1;
	}

	rc = sky_asyncopen(&httpd.conf, &httpd.async);
	if (rc) {
		sky_err("sky_asyncopen(): %d\n", -rc);
		return 1;
	}

	asyncfd = sky_asyncfd(httpd.async);
	if (asyncfd < 0) {
		sky_err("sky_asyncfd(): %d\n", -asyncfd);
		return 1;
	}

	rc = httpd_create_emergency_response(&httpd);
	if (rc) {
		sky_err("Can't create emergency response\n");
		return 1;
	}

	httpd.mhd = MHD_start_daemon(MHD_USE_DEBUG | MHD_USE_SUSPEND_RESUME, 0,
			&httpd_accept_policy_call, &httpd,
			&httpd_access_handler_call, &httpd,
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
			break;
		}
		SKY_FD_SET(asyncfd, rs);
		max = max(max, asyncfd);
		tvp = httpd_expire_addrs_and_requests(&httpd, &tv);
		/*
		 * HTTP response can be queued while expiration, so processing
		 * can be required. Don't sleep in that case.
		 */
		if (!httpd.need_processing) {
			rc = select(max + 1, (fd_set*)&rs, (fd_set*)&ws, (fd_set*)&es, tvp);
			if (rc < 0 && errno != EINTR) {
				sky_err("select failed: %s\n", strerror(errno));
				break;
			}
		}
		do {
			MHD_run(httpd.mhd);
			rc = sky_asyncexecute(httpd.async, 0);
			if (rc < 0) {
				sky_err("sky_asyncexecute() failed: %d\n", -rc);
				goto out;
			}
		} while (httpd_get_and_clear_need_processing(&httpd));
	}
out:
	MHD_stop_daemon(httpd.mhd);
	httpd_destroy_emergency_response(&httpd);
	sky_asyncclose(httpd.async);

	return 0;
}

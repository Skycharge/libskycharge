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

#ifndef LIBSKYCHARGE_PRI_H
#define LIBSKYCHARGE_PRI_H

#include <search.h>
#include <assert.h>
#include <time.h>

#include "libskycharge.h"
#include "skyproto.h"
#include "types.h"

struct sky_dev_ops {
	enum sky_con_type contype;
	struct sky_dev_ops *next;

	int (*asyncopen)(const struct sky_conf *conf,
			 const struct sky_dev_ops *ops,
			 struct sky_async **async);
	void (*asyncclose)(struct sky_async *async);
	int (*asyncexecute)(struct sky_async *async, bool wait);
	int (*asyncfd)(struct sky_async *async);
	bool (*asyncreq_cancel)(struct sky_async *async,
				struct sky_async_req *req);

	int (*discoverbroker)(const struct sky_dev_ops *ops,
			      struct sky_brokerinfo *brokerinfo,
			      unsigned int timeout_ms);
	int (*devopen)(const struct sky_dev_desc *devdesc, struct sky_dev **dev);
	void (*devclose)(struct sky_dev *dev);
	int (*subscribe)(struct sky_dev *dev);
	void (*unsubscribe)(struct sky_dev *dev);
	int (*subscription_work)(struct sky_dev *dev,
				 struct sky_charging_state *state);
};

struct sky_dev {
	struct sky_dev_desc devdesc;
	struct sky_subscription subsc;
	bool unsubscribed;
	pthread_t thread;
};

struct sky_async {
	struct sky_async_req     *head;
	struct sky_async_req     *tail;
	const struct sky_dev_ops *ops;
	const struct sky_conf    *conf;
};

extern void __sky_register_devops(struct sky_dev_ops *ops);

static inline void
uint32_to_calib_point(uint32_t v, uint16_t *set, uint16_t *read)
{
	*set  = (v >> 16);
	*read = (v & 0xffff);
}

static inline uint32_t
calib_point_to_uint32(uint16_t set, uint16_t read)
{
	return (uint32_t)set << 16 | read;
}

static inline unsigned long long msecs_epoch(void)
{
	struct timespec ts;
	unsigned long long msecs;

	clock_gettime(CLOCK_REALTIME, &ts);
	msecs  = ts.tv_sec * 1000ull;
	msecs += ts.tv_nsec / 1000000ull;

	return msecs;
}

int hex_dump_to_buffer_oneline(const void *buf, size_t len, int rowsize,
			       int groupsize, char *linebuf,
			       size_t linebuflen, bool ascii);
int hex_dump_to_buffer(const void *buf, size_t len, int rowsize,
		       int groupsize, char *linebuf,
		       size_t linebuflen, bool ascii);

#define sky_register_devops(ops)                                       \
       __attribute__((constructor)) static void register_devops(void)  \
       {                                                               \
               __sky_register_devops(ops);			       \
       }                                                               \

#define foreach_devops(ops, devops)			\
       for (ops = devops; ops; ops = ops->next)

struct sky_parse_devs_list_rsp_result {
	struct sky_dev_desc **list;
	int                 error;
	size_t              rsp_len;
	size_t              num_devs;
	size_t              info_off;
	bool                dyn_info;
};

static inline int
sky_parse_devs_list_rsp(const void *rsp_data, size_t rsp_len,
			const struct sky_async *async,
			struct sky_parse_devs_list_rsp_result *result)
{
	struct sky_dev_desc *head = NULL, *tail = NULL;
	const struct sky_devs_list_rsp *rsp = rsp_data;
	size_t num_devs, info_off;
	bool dyn_info;
	int rc, i;

	/* Sanity checks */
	if (rsp_len < offsetof_end(typeof(*rsp), info_off))
		/* Malformed response */
		goto end_with_EPROTO;

	rc = -le16toh(rsp->hdr.error);
	if (rc)
		goto end_with_error;

	num_devs = le16toh(rsp->num_devs);
	if (!num_devs) {
		/* Empty list consider as success */
		rc = 0;
		goto end_with_error;
	}

	/*
	 * Protocol version before 0x0400 does not have `info_off`,
	 * so we use the fact that it is 0 if the protocol is below.
	 */
	dyn_info = !!rsp->info_off;
	info_off = dyn_info ?
		le16toh(rsp->info_off) :
		offsetof_end(typeof(*rsp), info_off);

	/* Result contains the start offset */
	result->info_off = info_off;

	for (i = 0; i < num_devs; i++) {
		const struct sky_dev_info *notrust_info = rsp_data + info_off;
		struct sky_dev_info info;
		struct sky_dev_desc *devdesc;
		size_t info_len;

		/* Sanity checks */
		if (rsp_len < info_off + offsetof_end(typeof(info), info_len))
			/* Malformed response */
			goto end_with_EPROTO;

		/*
		 * Protocol version before 0x0400 does not have `info_len`.
		 * But there is one corner case, when we are connected to
		 * the new skybroker, which sets `info_off` but forwards
		 * us the sky_dev_info from the old server, where the
		 * `info_len` is 0. So handle this carefully.
		 */
		info_len = le16toh(notrust_info->info_len) ?:
			offsetof_end(typeof(*notrust_info), portname);
		info_len = dyn_info ? info_len :
			offsetof_end(typeof(*notrust_info), portname);
		info_off += info_len;

		/* Copy to a local buffer */
		memset(&info, 0, sizeof(info));
		memcpy(&info, notrust_info, min(info_len, sizeof(info)));

		devdesc = calloc(1, sizeof(*devdesc));
		if (!devdesc) {
			sky_devsfree(head);
			return -ENOMEM;
		}
		devdesc->dev_type = le16toh(info.dev_type);
		memcpy(devdesc->dev_name, info.dev_name,
		       sizeof(devdesc->dev_name) - 1);
		memcpy(devdesc->dev_uuid, info.dev_uuid,
		       sizeof(devdesc->dev_uuid));
		devdesc->proto_version = le16toh(info.proto_version);
		devdesc->hw_info = (struct sky_hw_info) {
			.fw_version        = le32toh(info.fw_version),
			.hw_version        = le32toh(info.hw_version),
			.plc_proto_version = le32toh(info.plc_proto_version),
			.uid.part1         = le32toh(info.hw_uid.part1),
			.uid.part2         = le32toh(info.hw_uid.part2),
			.uid.part3         = le32toh(info.hw_uid.part3),
		};
		if (async) {
			devdesc->conf = *async->conf;
			devdesc->dev_ops = async->ops;
		}
		memcpy(devdesc->portname, info.portname,
		       sizeof(devdesc->portname) - 1);

		devdesc->next = head;
		head = devdesc;
		if (tail == NULL)
			tail = devdesc;
	}
	if (head)
		tail->next = *result->list;

	*result->list    = head;
	result->error    = 0;
	result->rsp_len  = rsp_len;
	result->num_devs = num_devs;
	result->dyn_info = dyn_info;

	return 0;

end_with_error:
	sky_devsfree(head);
	result->error = rc;
	return 0;

end_with_EPROTO:
	rc = -EPROTO;
	goto end_with_error;
}

static inline void sky_async_init(const struct sky_conf *conf,
				  const struct sky_dev_ops *ops,
				  struct sky_async *async)
{
	async->head = (typeof(async->head))async;
	async->tail = (typeof(async->tail))async;
	async->conf = conf;
	async->ops  = ops;
}

static inline bool sky_async_empty(struct sky_async *async)
{
	return async->head == (typeof(async->head))async;
}

static inline void sky_asyncreq_init(enum sky_proto_type type,
				     struct sky_dev *dev,
				     const void *in_ptr, void *out_ptr,
				     struct sky_async_req *req)
{
	assert(type & 1);
	assert(type != SKY_UNKNOWN_REQRSP && type < SKY_LAST_REQRSP);

	memset(req, 0, sizeof(*req));
	req->type    = type;
	req->dev     = dev;
	req->in.ptr  = in_ptr;
	req->out.ptr = out_ptr;
}

static inline bool sky_asyncreq_complete(struct sky_async *async,
					 struct sky_async_req *req,
					 int rc)
{
	if (req->out.completed)
		return false;

	req->out.completed = true;
	req->out.rc = rc;
	if (req->completion)
		req->completion(req);

	return true;
}

static inline void sky_asyncreq_add(struct sky_async *async,
				    struct sky_async_req *req)
{
	insque(req, async->tail);
}

static inline void sky_asyncreq_del(struct sky_async_req *req)
{
	remque(req);
	req->next = req;
	req->prev = req;
}

static inline struct sky_async_req *sky_asyncreq_pop(struct sky_async *async)
{
	struct sky_async_req *head;

	if (sky_async_empty(async))
		return NULL;

	head = async->head;
	sky_asyncreq_del(head);

	return head;
}

static inline const struct sky_dev_ops *get_devops(struct sky_dev *dev)
{
	return dev->devdesc.dev_ops;
}

static inline const struct sky_hw_ops *get_hwops(struct sky_dev *dev)
{
	return dev->devdesc.hw_ops;
}

#endif /* LIBSKYCHARGE_PRI_H */

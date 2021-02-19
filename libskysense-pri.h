#ifndef LIBSKYSENSE_PRI_H
#define LIBSKYSENSE_PRI_H

#include <search.h>
#include <assert.h>
#include <time.h>

#include "libskysense.h"
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

static inline unsigned long long msecs_epoch(void)
{
	struct timespec ts;
	unsigned long long msecs;

	clock_gettime(CLOCK_REALTIME, &ts);
	msecs  = ts.tv_sec * 1000ull;
	msecs += ts.tv_nsec / 1000000ull;

	return msecs;
}

#define sky_register_devops(ops)                                       \
       __attribute__((constructor)) static void register_devops(void)  \
       {                                                               \
               __sky_register_devops(ops);			       \
       }                                                               \

#define foreach_devops(ops, devops)			\
       for (ops = devops; ops; ops = ops->next)

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

#endif /* LIBSKYSENSE_PRI_H */

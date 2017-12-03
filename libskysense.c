#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>

#include "libskysense-pri.h"
#include "types.h"

static struct sky_dev_ops *devops;

void __sky_register_devops(struct sky_dev_ops *ops)
{
       ops->next = devops;
       devops = ops;
}

int sky_devslist(const struct sky_dev_conf *conf, size_t num,
		 struct sky_dev_desc **out)
{
	struct sky_dev_desc *head = NULL;
	struct sky_dev_ops *ops;
	int i, rc = -EINVAL;

	for (i = 0; i < num; i++) {
		foreach_devops(ops, devops) {
			if (ops->contype != conf->contype)
				continue;
			rc = ops->devslist(ops, conf, &head);
			if (rc)
				goto err;
		}
	}
	if (rc)
		/* No ops found */
		return rc;
	if (head == NULL)
		/* No devices found */
		return -ENODEV;

	*out = head;

	return 0;

err:
	sky_devsfree(head);

	return rc;
}

void sky_devsfree(struct sky_dev_desc *head)
{
	struct sky_dev_desc *next;

	while (head) {
		next = head->next;
		free(head);
		head = next;
	}
}

static inline const struct sky_dev_ops *get_devops(struct sky_dev *dev)
{
	return dev->devdesc.opaque_ops;
}

int sky_devopen(const struct sky_dev_desc *devdesc, struct sky_dev **dev_)
{
	const struct sky_dev_ops *ops = devdesc->opaque_ops;
	int rc;

	rc = ops->devopen(devdesc, dev_);
	if (!rc) {
		struct sky_dev *dev = *dev_;

		dev->devdesc = *devdesc;
	}

	return rc;
}

void sky_devclose(struct sky_dev *dev)
{
	(void)sky_unsubscribe(dev);
	get_devops(dev)->devclose(dev);
}

int sky_devinfo(struct sky_dev *dev, struct sky_dev_desc *devdesc)
{
	*devdesc = dev->devdesc;

	return 0;
}

int sky_paramsget(struct sky_dev *dev, struct sky_dev_params *params)
{
	return get_devops(dev)->paramsget(dev, params);
}

int sky_paramsset(struct sky_dev *dev, struct sky_dev_params *params)
{
	return get_devops(dev)->paramsset(dev, params);
}

int sky_chargingstate(struct sky_dev *dev, struct sky_charging_state *state)
{
	return get_devops(dev)->chargingstate(dev, state);
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

static void *subscription_work(void *data)
{
	struct sky_charging_state state;
	struct sky_dev *dev = data;
	unsigned long long ms;
	int rc;

	while (!dev->unsubscribed) {
		ms = msecs_epoch();
		rc = get_devops(dev)->subscription_work(dev, &state);
		ms = msecs_epoch() - ms;
		if (!rc)
			/* Notify subscribers only in case of success */
			dev->subsc.on_state(dev->subsc.data, &state);

		if (ms < dev->subsc.interval_msecs) {
			ms = dev->subsc.interval_msecs - ms;
			usleep(ms * 1000);
		}
	}

	return NULL;
}

int sky_subscribe(struct sky_dev *dev,
		  struct sky_subscription *subsc)
{
	int rc;

	if (dev->thread)
		return -EEXIST;
	if (subsc->on_state == NULL || !subsc->interval_msecs)
		return -EINVAL;

	memcpy(&dev->subsc, subsc, sizeof(*subsc));

	rc = get_devops(dev)->subscribe(dev);
	if (rc)
		return rc;
	rc = pthread_create(&dev->thread, NULL, subscription_work, dev);
	if (rc)
		get_devops(dev)->unsubscribe(dev);

	return -rc;
}

int sky_unsubscribe(struct sky_dev *dev)
{
	if (!dev->thread)
		return -ENOENT;

	dev->unsubscribed = true;
	pthread_join(dev->thread, NULL);

	dev->unsubscribed = false;
	dev->thread = 0;

	get_devops(dev)->unsubscribe(dev);

	return 0;
}

int sky_reset(struct sky_dev *dev)
{
	return get_devops(dev)->reset(dev);
}

int sky_chargestart(struct sky_dev *dev)
{
	return get_devops(dev)->chargestart(dev);
}

int sky_chargestop(struct sky_dev *dev)
{
	return get_devops(dev)->chargestop(dev);
}

int sky_coveropen(struct sky_dev *dev)
{
	return get_devops(dev)->coveropen(dev);
}

int sky_coverclose(struct sky_dev *dev)
{
	return get_devops(dev)->coverclose(dev);
}

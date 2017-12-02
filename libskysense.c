#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>

#include "libskysense-pri.h"
#include "types.h"

static const struct sky_dev_ops *local_ops =
	&sky_local_dev_ops;
	/* &sky_dummy_dev_ops; */

int sky_devslist(struct sky_dev_desc **head)
{
	*head = NULL;

	/* Forward to local implementation */
	return local_ops->devslist(head);
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

int sky_devopen(const struct sky_dev_conf *conf, struct sky_dev **dev_)
{
	const struct sky_dev_ops *ops;
	int rc;

	if (conf->contype == SKY_LOCAL)
		ops = local_ops;
	else if (conf->contype == SKY_REMOTE)
		ops = &sky_remote_dev_ops;
	else
		return -EINVAL;

	rc = ops->devopen(conf, dev_);
	if (!rc) {
		struct sky_dev *dev = *dev_;

		dev->ops  = *ops;
		dev->conf = *conf;
	}

	return rc;
}

void sky_devclose(struct sky_dev *dev)
{
	(void)sky_unsubscribe(dev);
	dev->ops.devclose(dev);
}

int sky_devinfo(struct sky_dev *dev, struct sky_dev_desc *devdesc)
{
	return dev->ops.devinfo(dev, devdesc);
}

int sky_paramsget(struct sky_dev *dev, struct sky_dev_params *params)
{
	return dev->ops.paramsget(dev, params);
}

int sky_paramsset(struct sky_dev *dev, struct sky_dev_params *params)
{
	return dev->ops.paramsset(dev, params);
}

int sky_chargingstate(struct sky_dev *dev, struct sky_charging_state *state)
{
	return dev->ops.chargingstate(dev, state);
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
		rc = dev->ops.subscription_work(dev, &state);
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

	rc = dev->ops.subscribe(dev);
	if (rc)
		return rc;
	rc = pthread_create(&dev->thread, NULL, subscription_work, dev);
	if (rc)
		dev->ops.unsubscribe(dev);

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

	dev->ops.unsubscribe(dev);

	return 0;
}

int sky_reset(struct sky_dev *dev)
{
	return dev->ops.reset(dev);
}

int sky_chargestart(struct sky_dev *dev)
{
	return dev->ops.chargestart(dev);
}

int sky_chargestop(struct sky_dev *dev)
{
	return dev->ops.chargestop(dev);
}

int sky_coveropen(struct sky_dev *dev)
{
	return dev->ops.coveropen(dev);
}

int sky_coverclose(struct sky_dev *dev)
{
	return dev->ops.coverclose(dev);
}

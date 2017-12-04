#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

#include "libskysense-pri.h"
#include "types.h"

struct skydum_dev {
	struct sky_dev dev;
	struct sky_charging_state state;
	struct sky_dev_params params;
	struct sky_dev_desc devdesc;
};

static int skydum_devslist(const struct sky_dev_ops *ops,
			   const struct sky_dev_conf *conf,
			   struct sky_dev_desc **head)
{
	struct sky_dev_desc *dev;

	dev = calloc(1, sizeof(*dev));
	if (!dev)
		return -ENOMEM;

	dev->dev_type = SKY_INDOOR;
	dev->conf = *conf;
	dev->opaque_ops = ops;
	strcpy(dev->portname, "dummy0");

	dev->next = *head;
	*head = dev;

	return 0;
}

static int skydum_devopen(const struct sky_dev_desc *devdesc,
			  struct sky_dev **dev_)
{
	struct skydum_dev *dev;

	dev = calloc(1, sizeof(*dev));
	if (!dev)
		return -ENOMEM;

	*dev_ = &dev->dev;

	return 0;
}

static void skydum_devclose(struct sky_dev *dev_)
{
	struct skydum_dev *dev;

	dev = container_of(dev_, struct skydum_dev, dev);
	free(dev);
}

static int skydum_paramsget(struct sky_dev *dev_, struct sky_dev_params *params)
{
	struct skydum_dev *dev;
	int i;

	if (params->dev_params_bits == 0)
		return -EINVAL;

	BUILD_BUG_ON(sizeof(params->dev_params_bits) * 8 <
		     SKY_NUM_DEVPARAM);

	dev = container_of(dev_, struct skydum_dev, dev);
	for (i = 0; i < SKY_NUM_DEVPARAM; i++) {
		if (!(params->dev_params_bits & (1<<i)))
				continue;
		params->dev_params[i] = dev->params.dev_params[i];
	}

	return 0;
}

static int skydum_paramsset(struct sky_dev *dev_,
			    const struct sky_dev_params *params)
{
	struct skydum_dev *dev;
	int i;

	if (params->dev_params_bits == 0)
		return -EINVAL;

	BUILD_BUG_ON(sizeof(params->dev_params_bits) * 8 <
		     SKY_NUM_DEVPARAM);

	dev = container_of(dev_, struct skydum_dev, dev);
	for (i = 0; i < SKY_NUM_DEVPARAM; i++) {
		if (!(params->dev_params_bits & (1<<i)))
				continue;
		dev->params.dev_params[i] = params->dev_params[i];
	}

	return 0;
}

static int skydum_chargingstate(struct sky_dev *dev_,
				struct sky_charging_state *state)
{
	struct skydum_dev *dev;

	dev = container_of(dev_, struct skydum_dev, dev);
	memcpy(state, &dev->state, sizeof(*state));

	return 0;
}

static int skydum_subscribe(struct sky_dev *dev_)
{
	/* Nop for now */
	return 0;
}

static void skydum_unsubscribe(struct sky_dev *dev_)
{
	/* Nop for now */
}

static int skydum_subscription_work(struct sky_dev *dev_,
				    struct sky_charging_state *state)
{
	struct skydum_dev *dev;

	dev = container_of(dev_, struct skydum_dev, dev);
	dev->state.current += 1;
	dev->state.voltage += 2;
	dev->state.dev_hw_state += 1;
	if (dev->state.dev_hw_state == 4)
		dev->state.dev_hw_state = 5;
	if (dev->state.dev_hw_state == 11)
		dev->state.dev_hw_state = 12;
	if (dev->state.dev_hw_state == 21)
		dev->state.dev_hw_state = 22;
	dev->state.dev_hw_state %= 26;

	*state = dev->state;

	return 0;
}

static int skydum_reset(struct sky_dev *dev_)
{
	return -EINVAL;
}

static int skydum_chargestart(struct sky_dev *dev_)
{
	return 0;
}

static int skydum_chargestop(struct sky_dev *dev_)
{
	return 0;
}

static int skydum_coveropen(struct sky_dev *dev_)
{
	return 0;
}

static int skydum_coverclose(struct sky_dev *dev_)
{
	return 0;
}

__attribute__((unused))
static struct sky_dev_ops sky_dummy_devops = {
	.contype = SKY_LOCAL,
	.devslist = skydum_devslist,
	.devopen = skydum_devopen,
	.devclose = skydum_devclose,
	.paramsget = skydum_paramsget,
	.paramsset = skydum_paramsset,
	.chargingstate = skydum_chargingstate,
	.subscribe = skydum_subscribe,
	.unsubscribe = skydum_unsubscribe,
	.subscription_work = skydum_subscription_work,
	.reset = skydum_reset,
	.chargestart = skydum_chargestart,
	.chargestop = skydum_chargestop,
	.coveropen = skydum_coveropen,
	.coverclose = skydum_coverclose
};
/* DEBUG sky_register_devops(&sky_dummy_devops); */

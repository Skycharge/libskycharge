#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

#include "libskysense-pri.h"
#include "types.h"

struct skydum_lib {
	struct sky_lib lib;
	struct sky_charging_state state;
	struct sky_dev_params params;
	struct sky_dev_desc dev;
};

static int skydum_devslist(struct sky_dev_desc **head)
{
	struct sky_dev_desc *dev;

	dev = calloc(1, sizeof(*dev));
	if (!dev)
		return -ENOMEM;

	dev->dev_type = SKY_INDOOR;
	strcpy(dev->portname, "port0");

	dev->next = *head;
	*head = dev;

	return 0;
}

static int skydum_libopen(const struct sky_lib_conf *conf,
			  struct sky_lib **lib_)
{
	struct skydum_lib *lib;

	lib = calloc(1, sizeof(*lib));
	if (!lib)
		return -ENOMEM;

	strcpy(lib->dev.portname, "port0");
	*lib_ = &lib->lib;

	return 0;
}

static void skydum_libclose(struct sky_lib *lib_)
{
	struct skydum_lib *lib;

	lib = container_of(lib_, struct skydum_lib, lib);
	free(lib);
}

static int skydum_devinfo(struct sky_lib *lib_, struct sky_dev_desc *dev)
{
	struct skydum_lib *lib;

	lib = container_of(lib_, struct skydum_lib, lib);
	memcpy(dev, &lib->dev, sizeof(*dev));

	return 0;
}

static int skydum_paramsget(struct sky_lib *lib_, struct sky_dev_params *params)
{
	struct skydum_lib *lib;
	int i;

	if (params->dev_params_bits == 0)
		return -EINVAL;

	BUILD_BUG_ON(sizeof(params->dev_params_bits) * 8 <
		     SKY_NUM_DEVPARAM);

	lib = container_of(lib_, struct skydum_lib, lib);
	for (i = 0; i < SKY_NUM_DEVPARAM; i++) {
		if (!(params->dev_params_bits & (1<<i)))
				continue;
		params->dev_params[i] = lib->params.dev_params[i];
	}

	return 0;
}

static int skydum_paramsset(struct sky_lib *lib_, struct sky_dev_params *params)
{
	struct skydum_lib *lib;
	int i;

	if (params->dev_params_bits == 0)
		return -EINVAL;

	BUILD_BUG_ON(sizeof(params->dev_params_bits) * 8 <
		     SKY_NUM_DEVPARAM);

	lib = container_of(lib_, struct skydum_lib, lib);
	for (i = 0; i < SKY_NUM_DEVPARAM; i++) {
		if (!(params->dev_params_bits & (1<<i)))
				continue;
		lib->params.dev_params[i] = params->dev_params[i];
	}

	return 0;
}

static int skydum_chargingstate(struct sky_lib *lib_,
				struct sky_charging_state *state)
{
	struct skydum_lib *lib;

	lib = container_of(lib_, struct skydum_lib, lib);
	memcpy(state, &lib->state, sizeof(*state));

	return 0;
}

static int skydum_subscribe(struct sky_lib *lib_)
{
	/* Nop for now */
	return 0;
}

static void skydum_unsubscribe(struct sky_lib *lib_)
{
	/* Nop for now */
}

static int skydum_subscription_work(struct sky_lib *lib_,
				    struct sky_charging_state *state)
{
	struct skydum_lib *lib;

	lib = container_of(lib_, struct skydum_lib, lib);
	lib->state.current += 1;
	lib->state.voltage += 2;
	lib->state.dev_hw_state += 1;
	if (lib->state.dev_hw_state == 4)
		lib->state.dev_hw_state = 5;
	if (lib->state.dev_hw_state == 11)
		lib->state.dev_hw_state = 12;
	if (lib->state.dev_hw_state == 21)
		lib->state.dev_hw_state = 22;
	lib->state.dev_hw_state %= 26;

	*state = lib->state;

	return 0;
}

static int skydum_reset(struct sky_lib *lib_)
{
	return -EINVAL;
}

static int skydum_chargestart(struct sky_lib *lib_)
{
	return 0;
}

static int skydum_chargestop(struct sky_lib *lib_)
{
	return 0;
}

static int skydum_coveropen(struct sky_lib *lib_)
{
	return 0;
}

static int skydum_coverclose(struct sky_lib *lib_)
{
	return 0;
}

struct sky_lib_ops sky_dummy_lib_ops = {
	.devslist = skydum_devslist,
	.libopen = skydum_libopen,
	.libclose = skydum_libclose,
	.devinfo = skydum_devinfo,
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

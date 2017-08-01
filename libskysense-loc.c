#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

#include "libskysense-pri.h"
#include "types.h"

struct skyloc_lib {
	struct sky_lib lib;
	struct sky_charging_state state;
	struct sky_dev_conf conf;
	struct sky_dev dev;
};

static int skyloc_libopen(const struct sky_lib_conf *conf,
			  struct sky_lib **lib_)
{
	struct skyloc_lib *lib;

	lib = calloc(1, sizeof(*lib));
	if (!lib)
		return -ENOMEM;

	strcpy(lib->dev.portname, "port0");
	*lib_ = &lib->lib;

	return 0;
}

static void skyloc_libclose(struct sky_lib *lib_)
{
	struct skyloc_lib *lib;

	lib = container_of(lib_, struct skyloc_lib, lib);
	free(lib);
}

static int skyloc_devinfo(struct sky_lib *lib_, struct sky_dev *dev)
{
	struct skyloc_lib *lib;

	lib = container_of(lib_, struct skyloc_lib, lib);
	memcpy(dev, &lib->dev, sizeof(*dev));

	return 0;
}

static int skyloc_confget(struct sky_lib *lib_, struct sky_dev_conf *conf)
{
	struct skyloc_lib *lib;
	int i;

	if (conf->dev_params_bits == 0)
		return -EINVAL;

	BUILD_BUG_ON(sizeof(conf->dev_params_bits) * 8 <
		     SKY_NUM_DEVPARAM);

	lib = container_of(lib_, struct skyloc_lib, lib);
	for (i = 0; i < SKY_NUM_DEVPARAM; i++) {
		if (!(conf->dev_params_bits & (1<<i)))
				continue;
		conf->dev_params[i] = lib->conf.dev_params[i];
	}

	return 0;
}

static int skyloc_confset(struct sky_lib *lib_, struct sky_dev_conf *conf)
{
	struct skyloc_lib *lib;
	int i;

	if (conf->dev_params_bits == 0)
		return -EINVAL;

	BUILD_BUG_ON(sizeof(conf->dev_params_bits) * 8 <
		     SKY_NUM_DEVPARAM);

	lib = container_of(lib_, struct skyloc_lib, lib);
	for (i = 0; i < SKY_NUM_DEVPARAM; i++) {
		if (!(conf->dev_params_bits & (1<<i)))
				continue;
		lib->conf.dev_params[i] = conf->dev_params[i];
	}

	return 0;
}

static int skyloc_chargingstate(struct sky_lib *lib_,
				struct sky_charging_state *state)
{
	struct skyloc_lib *lib;

	lib = container_of(lib_, struct skyloc_lib, lib);
	memcpy(state, &lib->state, sizeof(*state));

	return 0;
}

static int skyloc_subscribe(struct sky_lib *lib_)
{
	/* Nop for now */
	return 0;
}

static void skyloc_unsubscribe(struct sky_lib *lib_)
{
	/* Nop for now */
}

static int skyloc_subscription_work(struct sky_lib *lib_,
				    struct sky_charging_state *state)
{
	struct skyloc_lib *lib;

	lib = container_of(lib_, struct skyloc_lib, lib);
	lib->state.current += 0.1;
	lib->state.voltage += 0.2;
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

static int skyloc_reset(struct sky_lib *lib_)
{
	return -EINVAL;
}

static int skyloc_autoscan(struct sky_lib *lib_, unsigned autoscan)
{
	return 0;
}

static int skyloc_chargestart(struct sky_lib *lib_)
{
	return 0;
}

static int skyloc_chargestop(struct sky_lib *lib_)
{
	return 0;
}

static int skyloc_coveropen(struct sky_lib *lib_)
{
	return 0;
}

static int skyloc_coverclose(struct sky_lib *lib_)
{
	return 0;
}

struct sky_lib_ops sky_local_lib_ops = {
	.libopen = skyloc_libopen,
	.libclose = skyloc_libclose,
	.devinfo = skyloc_devinfo,
	.confget = skyloc_confget,
	.confset = skyloc_confset,
	.chargingstate = skyloc_chargingstate,
	.subscribe = skyloc_subscribe,
	.unsubscribe = skyloc_unsubscribe,
	.subscription_work = skyloc_subscription_work,
	.reset = skyloc_reset,
	.autoscan = skyloc_autoscan,
	.chargestart = skyloc_chargestart,
	.chargestop = skyloc_chargestop,
	.coveropen = skyloc_coveropen,
	.coverclose = skyloc_coverclose
};

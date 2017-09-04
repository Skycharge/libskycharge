#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <endian.h>
#include <limits.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>

#include "libskysense.h"
#include "skyclient-cmd.h"
#include "types.h"

#define sky_err(fmt, ...) \
	fprintf(stderr, __FILE__ ":%s():" stringify(__LINE__) ": " fmt, \
		__func__, ##__VA_ARGS__)

#define X(state) \
	case state: return #state

static inline const char *sky_devtype_to_str(enum sky_dev_type type)
{
	return type == SKY_INDOOR ? "indoor": "outdoor";
}

static inline const char *sky_devstate_to_str(enum sky_dev_hw_state state)
{
	switch(state) {
	X(SKY_UNKNOWN);
	X(SKY_SCANNING_INIT);
	X(SKY_SCANNING_RUN_STATE);
	X(SKY_SCANNING_CHECK_MATRIX);
	X(SKY_SCANNING_CHECK_WATER);
	X(SKY_SCANNING_WET);
	X(SKY_SCANNING_DETECTING);
	X(SKY_PRE_CHARGING_INIT);
	X(SKY_PRE_CHARGING_RUN);
	X(SKY_PRE_CHARGING_CHECK_MATRIX);
	X(SKY_PRE_CHARGING_CHECK_WATER);
	X(SKY_PRE_CHARGING_WET);
	X(SKY_PRE_CHARGING_FIND_CHARGERS);
	X(SKY_CHARGING_INIT);
	X(SKY_CHARGING_RUN);
	X(SKY_CHARGING_MONITOR_CURRENT);
	X(SKY_POST_CHARGING_INIT);
	X(SKY_POST_CHARGING_RUN);
	X(SKY_POST_CHARGING_CHECK_MATRIX);
	X(SKY_POST_CHARGING_CHECK_WATER);
	X(SKY_POST_CHARGING_WET);
	X(SKY_POST_CHARGING_FIND_CHARGERS);
	X(SKY_OVERLOAD);
	X(SKY_AUTOSCAN_DISABLED);
	default:
		sky_err("unknown state: %d\n", state);
		exit(-1);
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
		exit(-1);
	}
}

static void sky_prepare_lib(struct cli *cli, struct sky_lib **lib,
			    struct sky_dev **devs)
{
	struct sky_lib_conf conf;
	int rc;

	if (cli->addr && cli->port) {
		conf.conn_type = SKY_REMOTE;
		conf.remote.cmdport = strtol(cli->port, NULL, 10);
		conf.remote.subport = conf.remote.cmdport + 1;
		strncpy(conf.remote.hostname, cli->addr,
			sizeof(conf.remote.hostname));
	} else {
		conf.conn_type = SKY_LOCAL;
		rc = sky_devslist(devs);
		if (rc) {
			sky_err("sky_devslist(): %s\n", strerror(-rc));
			exit(-1);
		}
		/* Take first available port */
		if (*devs == NULL) {
			sky_err("sky_devslist(): No sky devices found\n");
			exit(-1);
		}
		strncpy(conf.local.portname, (*devs)->portname,
			sizeof(conf.local.portname));
	}
	rc = sky_libopen(&conf, lib);
	if (rc) {
		sky_err("sky_libopen(): %s\n", strerror(-rc));
		exit(-1);
	}
}

static void sky_print_charging_state(struct sky_charging_state *state)
{
	printf("Device in state: %s\n",
	       sky_devstate_to_str(state->dev_hw_state));
	printf("Voltage: %d mV\n", state->voltage);
	printf("Current: %d mA\n", state->current);
}

static void sky_on_charging_state(void *data, struct sky_charging_state *state)
{
	sky_print_charging_state(state);
	printf("\n");
}

int main(int argc, char *argv[])
{
	struct sky_dev *devs = NULL;
	struct sky_lib *lib = NULL;
	struct cli cli;
	int rc, i;

	rc = cli_parse(argc, argv, &cli);
	if (rc) {
		fprintf(stderr, "%s\n", cli_usage);
		return -1;
	}

	sky_prepare_lib(&cli, &lib, &devs);

	if (cli.listdevs) {
		printf("Found sky devices on ports:\n");
		while (devs) {
			printf("\t%s %s\n", sky_devtype_to_str(devs->dev_type),
			       devs->portname);
			devs = devs->next;
		}
	} else if (cli.monitor) {
		struct sky_subscription sub = {
			.on_state = sky_on_charging_state,
			.interval_msecs = strtol(cli.intervalms, NULL, 10),
		};

		rc = sky_subscribe(lib, &sub);
		if (rc) {
			sky_err("sky_subscribe(): %s\n", strerror(-rc));
			exit(-1);
		}
		while (1)
			/* Yeah, nothing to do here, simply die on Ctrl-C */
			sleep(1);
	} else if (cli.showdevparams) {
		struct sky_dev_conf conf;

		/* Get all params */
		conf.dev_params_bits = (1 << SKY_NUM_DEVPARAM) - 1;
		rc = sky_confget(lib, &conf);
		if (rc) {
			sky_err("sky_confget(): %s\n", strerror(-rc));
			exit(-1);
		}
		printf("Device has the following configuration:\n");
		for (i = 0; i < SKY_NUM_DEVPARAM; i++) {
			printf("\t%-30s %u\n", sky_devparam_to_str(i),
			       conf.dev_params[i]);
		}
	} else if (cli.setdevparam) {
		struct sky_dev_conf conf;

		for (i = 0; i < SKY_NUM_DEVPARAM; i++) {
			if (strcasestr(sky_devparam_to_str(i), cli.key))
				break;
		}
		if (i == SKY_NUM_DEVPARAM) {
			fprintf(stderr, "Incorrect parameter: %s\n", cli.key);
			fprintf(stderr, "Please, use one of the following:\n");
			for (i = 0; i < SKY_NUM_DEVPARAM; i++) {
				fprintf(stderr, "\t%s\n",
					sky_devparam_to_str(i));
			}
			exit(-1);
		}

		conf.dev_params_bits = (1 << i);
		conf.dev_params[i] = strtol(cli.value, NULL, 10);
		rc = sky_confset(lib, &conf);
		if (rc) {
			sky_err("sky_confset(): %s\n", strerror(-rc));
			exit(-1);
		}

	} else if (cli.startcharge) {
		rc = sky_chargestart(lib);
		if (rc) {
			sky_err("sky_chargestart(): %s\n", strerror(-rc));
			exit(-1);
		}
	} else if (cli.stopcharge) {
		rc = sky_chargestop(lib);
		if (rc) {
			sky_err("sky_stopcharge(): %s\n", strerror(-rc));
			exit(-1);
		}
	} else if (cli.opencover) {
		rc = sky_coveropen(lib);
		if (rc) {
			sky_err("sky_coveropen(): %s\n", strerror(-rc));
			exit(-1);
		}
	} else if (cli.closecover) {
		rc = sky_coverclose(lib);
		if (rc) {
			sky_err("sky_closecover(): %s\n", strerror(-rc));
			exit(-1);
		}
	} else if (cli.showchargingstate) {
		struct sky_charging_state state;

		rc = sky_chargingstate(lib, &state);
		if (rc) {
			sky_err("sky_closecover(): %s\n", strerror(-rc));
			exit(-1);
		}

		sky_print_charging_state(&state);
	} else if (cli.setautoscan) {
		rc = sky_autoscan(lib, strtol(cli.autoscan, NULL, 10));
		if (rc) {
			sky_err("sky_autoscan(): %s\n", strerror(-rc));
			exit(-1);
		}
	} else if (cli.reset) {
		rc = sky_reset(lib);
		if (rc) {
			sky_err("sky_reset(): %s\n", strerror(-rc));
			exit(-1);
		}
	} else if (cli.devinfo) {
		struct sky_dev dev;

		rc = sky_devinfo(lib, &dev);
		if (rc) {
			sky_err("sky_devinfo(): %s\n", strerror(-rc));
			exit(-1);
		}

		printf("Device portname: %s\n", dev.portname);
		printf("Device type:     %s\n", sky_devtype_to_str(dev.dev_type));

	} else
		assert(0);

	if (devs)
		sky_devsfree(devs);
	sky_libclose(lib);
	cli_free(&cli);

	return 0;
}

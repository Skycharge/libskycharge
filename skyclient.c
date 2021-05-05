#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <endian.h>
#include <limits.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <zlib.h>
#include <math.h>
#include <regex.h>

#include "libskysense.h"
#include "skyclient-cmd.h"
#include "version.h"
#include "types.h"

#define CONFFILE "/etc/skysense.conf"

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
		exit(-1);
	}
}

static inline unsigned int sky_dev_desc_crc32(struct sky_dev_desc *devdesc)
{
	uLong crc;

	crc = crc32(0L, Z_NULL, 0);
	crc = crc32(crc, (void *)devdesc->dev_uuid, sizeof(devdesc->dev_uuid));
	crc = crc32(crc, (void *)devdesc->portname, strlen(devdesc->portname));

	return crc;
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

/**
 * Unix UTC time to ISO8601, no timezone adjustment.
 * example: 2007-12-11T23:38:51.033Z
 *
 * Taken from gpsd/gpsutils.c
 */
static void unix_to_iso8601(double fixtime, char isotime[], size_t len)
{
	struct tm when;
	double integral, fractional;
	time_t intfixtime;
	char timestr[30];
	char fractstr[10];

	fractional = modf(fixtime, &integral);
	intfixtime = (time_t) integral;

	(void)gmtime_r(&intfixtime, &when);
	(void)strftime(timestr, sizeof(timestr), "%Y-%m-%dT%H:%M:%S", &when);
	/*
	 * Do not mess casually with the number of decimal digits in the
	 * format!  Most GPSes report over serial links at 0.01s or 0.001s
	 * precision.
	 */
	(void)snprintf(fractstr, sizeof(fractstr), "%.3f", fractional);
	/* add fractional part, ignore leading 0; "0.2" -> ".2" */
	(void)snprintf(isotime, len, "%s%sZ", timestr, strchr(fractstr,'.'));
}

static void sky_prepare_conf(struct cli *cli, struct sky_conf *conf)
{
	const char *conffile = cli->conff ?: CONFFILE;
	int rc;

	rc = sky_confparse(conffile, conf);

	if (cli->addr && cli->port) {
		/* Remote client, config should exist and parsed successfully */

		if (rc) {
			sky_err("sky_confparse(): %s\n", strerror(-rc));
			exit(-1);
		}
		conf->contype = SKY_REMOTE;
		/* TODO: we still get addr and port from command line */
		conf->cliport = strtol(cli->port, NULL, 10);
		conf->subport = conf->cliport + 1;
		strncpy(conf->hostname, cli->addr,
			sizeof(conf->hostname)-1);
	} else {
		/* Local client, config may be missing */

		if (rc && rc != -ENOENT) {
			sky_err("sky_confparse(): %s\n", strerror(-rc));
			exit(-1);
		}
		conf->contype = SKY_LOCAL;
	}
}

static struct sky_dev_desc *sky_find_devdesc_by_id(struct sky_dev_desc *head,
						   const char *devid)
{
	struct sky_dev_desc *devdesc;
	char id[16];

	foreach_devdesc(devdesc, head) {
		snprintf(id, sizeof(id), "%08X", sky_dev_desc_crc32(devdesc));
		if (!strcmp(devid, id))
			return devdesc;
	}

	return NULL;
}

static void sky_prepare_dev(struct cli *cli, struct sky_dev **dev,
			    struct sky_dev_desc **devdescs)
{
	struct sky_conf conf;
	int rc;

	sky_prepare_conf(cli, &conf);

	rc = sky_devslist(&conf, devdescs);
	if (rc) {
		if (rc == -ENODEV)
			fprintf(stderr, "No devices found.\n");
		else
			sky_err("sky_devslist(): %s\n", strerror(-rc));
		exit(-1);
	}
	if (!cli->listdevs) {
		struct sky_dev_desc *devdesc;

		if (!cli->devid) {
			if ((*devdescs)->next) {
				sky_err("Error: multiple devices found, please specify --id <dev-id>\n");
				exit(-1);
			}
			devdesc = *devdescs;
		} else {
			devdesc = sky_find_devdesc_by_id(*devdescs, cli->devid);
			if (!devdesc) {
				sky_err("Error: device by id '%s' was no found\n",
					cli->devid);
				exit(-1);
			}
		}
		rc = sky_devopen(devdesc, dev);
		if (rc) {
			sky_err("sky_devopen(): %s\n", strerror(-rc));
			exit(-1);
		}
	} else
		*dev = NULL;
}

static void sky_print_charging_state(struct cli *cli,
				     struct sky_charging_state *state)
{
	char timestr[64];
	struct timeval tv;
	struct tm *tm;
	size_t len;

	gettimeofday(&tv, NULL);
	tm = localtime(&tv.tv_sec);
	if (tm == NULL) {
		sky_err("localtime(): %s\n", strerror(errno));
		exit(-1);
	}
	len = strftime(timestr, sizeof(timestr), "%Y-%m-%d %H:%M:%S", tm);
	snprintf(timestr + len, sizeof(timestr) - len,
		 ".%03ld", tv.tv_usec/1000);

	if (!cli->pretty) {
		printf("%s\t%d\t%d\t%s\t%u\t%u\n",
		       timestr, state->voltage, state->current,
		       sky_devstate_to_str(state->dev_hw_state),
		       state->bms.charge_perc, state->bms.charge_time);
	} else {
		printf("Timestamp:   %s\n", timestr);
		printf("Dev state:   %s\n",
		       sky_devstate_to_str(state->dev_hw_state));
		printf("  Voltage:   %d mV\n", state->voltage);
		printf("  Current:   %d mA\n", state->current);
		printf("BMS:\n");

		if (state->bms.charge_perc)
			printf("  Charged:   %u %%\n", state->bms.charge_perc);
		else
			printf("  Charged:   N/A\n");

		if (state->bms.charge_time)
			printf("  Till full: %u sec\n", state->bms.charge_time);
		else
			printf("  Till full: N/A\n");
	}
}

static void sky_print_gpsdata(struct cli *cli, struct sky_gpsdata *gpsdata)
{
	char isotime[64];

	unix_to_iso8601(gpsdata->fix.time, isotime, sizeof(isotime));

	printf("Time:     %s\n", isotime);
	printf("Lat/Lon:  %f %f\n", gpsdata->fix.latitude,
	       gpsdata->fix.longitude);
        if (isnan(gpsdata->fix.altitude))
		printf("Altitude: ?\n");
	else
		printf("Altitude: %f\n", gpsdata->fix.altitude);

	printf("Status:   STATUS_%s\n",
	       gpsdata->status == SKY_GPS_STATUS_DGPS_FIX ? "DGPS_FIX" :
	       gpsdata->status == SKY_GPS_STATUS_FIX ? "FIX" :
	       "NO_FIX");
        printf("Mode:     MODE_%s\n",
	       gpsdata->fix.mode == SKY_GPS_MODE_3D ? "3D" :
	       gpsdata->fix.mode == SKY_GPS_MODE_2D ? "2D" :
	       gpsdata->fix.mode == SKY_GPS_MODE_NO_FIX ? "NO_FIX" :
	       "ZERO");
        printf("Quality:  %d p=%2.2f h=%2.2f v=%2.2f t=%2.2f g=%2.2f\n",
	       gpsdata->satellites_used, gpsdata->dop.pdop, gpsdata->dop.hdop,
	       gpsdata->dop.vdop, gpsdata->dop.tdop, gpsdata->dop.gdop);
}

static void sky_on_charging_state(void *data, struct sky_charging_state *state)
{
	struct cli *cli = data;

	sky_print_charging_state(cli, state);
}

int main(int argc, char *argv[])
{
	struct sky_dev_desc *devdescs = NULL;
	struct sky_dev *dev;
	struct cli cli;
	int rc, i;

	rc = cli_parse(argc, argv, &cli);
	if (rc) {
		fprintf(stderr, "%s\n", cli_usage);
		return -1;
	}
	if (cli.port) {
		const char *regexp = "[^[:digit:]]+";
		regmatch_t match;
		regex_t rgx;

		rc = regcomp(&rgx, regexp, REG_EXTENDED | REG_NEWLINE);
		assert(!rc);

		rc = regexec(&rgx, cli.port, 1, &match, 0);
		regfree(&rgx);
		if (rc != REG_NOMATCH) {
			fprintf(stderr, "Error: You've called a remote command invocation and specified the remote address '%s' and port '%s', but port is an invalid number.\n"
				"       If you attempted to call the 'monitor' command with the '1000 --raw' parameters, be aware that these parameters don't exist any more.\n"
				"       See help for details.\n", cli.addr, cli.port);
			return -1;
		}
	}

	if (cli.discoverbroker) {
		struct sky_brokerinfo brokerinfo;
		unsigned int timeout_ms;
		int rc;

		if (cli.timeoutsecs)
			timeout_ms = atoi(cli.timeoutsecs) * 1000;
		else
			timeout_ms = 5000;
		rc = sky_discoverbroker(&brokerinfo, timeout_ms);
		if (rc) {
			sky_err("sky_discoverbroker(): %s\n", strerror(-rc));
			exit(-1);
		}
		printf("Found skybroker:\n");
		printf("\t%s addr:        %s\n",
		       brokerinfo.af_family == AF_INET6 ? "IPv6" : "IPv4",
		       brokerinfo.addr);
		printf("\tServers port:     %u\n", brokerinfo.servers_port);
		printf("\tClients port:     %u\n", brokerinfo.clients_port);
		printf("\tProtocol version: %u.%u\n",
		       (brokerinfo.proto_version >> 8)  & 0xff,
		       brokerinfo.proto_version & 0xff);

		cli_free(&cli);

		return 0;

	} else if (cli.peerinfo) {
		struct sky_peerinfo peerinfo;
		struct sky_conf conf;
		int rc;

		sky_prepare_conf(&cli, &conf);

		rc = sky_peerinfo(&conf, &peerinfo);
		if (rc) {
			sky_err("sky_peerinfo(): %s\n", strerror(-rc));
			exit(-1);
		}
		printf("Remote peer %s:%d:\n", conf.hostname,
		       conf.cliport);
		printf("\tServer version:   %u.%u.%u\n",
		       version_major(peerinfo.server_version),
		       version_minor(peerinfo.server_version),
		       version_revis(peerinfo.server_version));
		printf("\tProtocol version: %u.%u\n",
		       version_minor(peerinfo.proto_version),
		       version_revis(peerinfo.proto_version));

		cli_free(&cli);

		return 0;
	}

	sky_prepare_dev(&cli, &dev, &devdescs);

	if (cli.listdevs) {
		size_t max_devname = strlen("DEV-NAME");
		struct sky_dev_desc *devdesc;

		foreach_devdesc(devdesc, devdescs) {
			max_devname = max(strnlen(devdesc->dev_name,
						  sizeof(devdesc->dev_name)),
					  max_devname);
		}

		printf("Found sky devices:\n");
		printf("\t  DEV-ID  %*s\n",
		       (int)max_devname, "DEV-NAME");
		foreach_devdesc(devdesc, devdescs) {
			printf("\t%08X  %*s\n",
			       sky_dev_desc_crc32(devdesc),
			       (int)max_devname,
			       devdesc->dev_name);
		}
	} else if (cli.monitor) {
		struct sky_subscription sub = {
			.data = &cli,
			.on_state = sky_on_charging_state,
			.interval_msecs = 1000,
		};

		rc = sky_subscribe(dev, &sub);
		if (rc) {
			sky_err("sky_subscribe(): %s\n", strerror(-rc));
			exit(-1);
		}
		while (1)
			/* Yeah, nothing to do here, simply die on Ctrl-C */
			sleep(1);
	} else if (cli.showdevparams) {
		struct sky_dev_params params;

		/* Get all params */
		params.dev_params_bits = (1 << SKY_NUM_DEVPARAM) - 1;
		rc = sky_paramsget(dev, &params);
		if (rc) {
			sky_err("sky_paramsget(): %s\n", strerror(-rc));
			exit(-1);
		}
		printf("Device has the following parameters:\n");
		for (i = 0; i < SKY_NUM_DEVPARAM; i++) {
			printf("\t%-30s %u\n", sky_devparam_to_str(i),
			       params.dev_params[i]);
		}
	} else if (cli.setdevparam) {
		struct sky_dev_params params;

		for (i = 0; i < SKY_NUM_DEVPARAM; i++) {
			if (!strcasecmp(sky_devparam_to_str(i), cli.key))
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

		params.dev_params_bits = (1 << i);
		params.dev_params[i] = strtol(cli.value, NULL, 10);
		rc = sky_paramsset(dev, &params);
		if (rc) {
			sky_err("sky_paramsset(): %s\n", strerror(-rc));
			exit(-1);
		}

	} else if (cli.startcharge) {
		rc = sky_chargestart(dev);
		if (rc) {
			sky_err("sky_chargestart(): %s\n", strerror(-rc));
			exit(-1);
		}
	} else if (cli.stopcharge) {
		rc = sky_chargestop(dev);
		if (rc) {
			sky_err("sky_stopcharge(): %s\n", strerror(-rc));
			exit(-1);
		}
	} else if (cli.opendroneport) {
		rc = sky_droneport_open(dev);
		if (rc) {
			sky_err("sky_droneport_open(): %s\n", strerror(-rc));
			exit(-1);
		}
	} else if (cli.closedroneport) {
		rc = sky_droneport_close(dev);
		if (rc) {
			sky_err("sky_droneport_close(): %s\n", strerror(-rc));
			exit(-1);
		}
	} else if (cli.showdroneportstate) {
		struct sky_droneport_state state;

		rc = sky_droneport_state(dev, &state);
		if (rc) {
			sky_err("sky_droneport_state(): %s\n", strerror(-rc));
			exit(-1);
		}

		printf("Skycharge Drone Port:\n"
		       "   IS READY      %s\n"
		       "   IS OPENED     %s\n"
		       "   IS CLOSED     %s\n"
		       "   IN PROGRESS   %s\n"
		       "   LANDING ERROR %s\n",
		       state.status & SKY_DP_IS_READY ? "yes": " no",
		       state.status & SKY_DP_IS_OPENED ? "yes": " no",
		       state.status & SKY_DP_IS_CLOSED ? "yes": " no",
		       state.status & SKY_DP_IN_PROGRESS ? "yes": " no",
		       state.status & SKY_DP_LANDING_ERROR ? "yes": " no");

	} else if (cli.dronedetect) {
		enum sky_drone_status status;

		rc = sky_dronedetect(dev, &status);
		if (rc) {
			sky_err("sky_dronedetect(): %s\n", strerror(-rc));
			exit(-1);
		}
		switch (status) {
		case SKY_DRONE_NOT_DETECTED:
			printf("Drone is not detected on the charging pad.\n");
			break;
		case SKY_DRONE_DETECTED:
			printf("Drone is detected on the charging pad.\n");
			break;
		default:
			printf("Unknown drone status: 0x%x\n", status);
		}
	} else if (cli.showchargingstate) {
		struct sky_charging_state state;

		rc = sky_chargingstate(dev, &state);
		if (rc) {
			sky_err("sky_chargingstate(): %s\n", strerror(-rc));
			exit(-1);
		}
		cli.pretty = !cli.nopretty;
		sky_print_charging_state(&cli, &state);
	} else if (cli.reset) {
		rc = sky_reset(dev);
		if (rc) {
			sky_err("sky_reset(): %s\n", strerror(-rc));
			exit(-1);
		}
	} else if (cli.devinfo) {
		struct sky_dev_desc devdesc;

		rc = sky_devinfo(dev, &devdesc);
		if (rc) {
			sky_err("sky_devinfo(): %s\n", strerror(-rc));
			exit(-1);
		}

		printf("Firmware version: %d.%d.%d\n",
		       version_major(devdesc.firmware_version),
		       version_minor(devdesc.firmware_version),
		       version_revis(devdesc.firmware_version));
	} else if (cli.gpsinfo) {
		struct sky_gpsdata gpsdata;

		rc = sky_gpsdata(dev, &gpsdata);
		if (rc) {
			sky_err("sky_gpsdata(): %s\n", strerror(-rc));
			exit(-1);
		}

		sky_print_gpsdata(&cli, &gpsdata);
	} else
		assert(0);

	sky_devsfree(devdescs);
	if (dev)
		sky_devclose(dev);
	cli_free(&cli);

	return 0;
}

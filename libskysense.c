#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <ctype.h>

#include <uuid/uuid.h>

#include "libskysense-pri.h"
#include "types.h"

static struct sky_dev_ops *devops;

void __sky_register_devops(struct sky_dev_ops *ops)
{
	struct sky_dev_ops *other_ops;

	foreach_devops(other_ops, devops) {
		if (ops->contype == other_ops->contype) {
			sky_err("Duplicate ops type: %d\n", ops->contype);
			exit(1);
		}
	}
	ops->next = devops;
	devops = ops;
}

static void trim(char *s)
{
	char *new = s;

	while (*s) {
		if (!isspace(*s))
			*new++ = *s;
		s++;
	}
	*new = *s;
}

static int parse_line(char *line, struct sky_conf *cfg)
{
	char *str;
	int rc;

	trim(line);
	*(strchrnul(line, '#')) = '\0';

	if ((str = strstr(line, "user-uuid="))) {
		rc = uuid_parse(str + 10, cfg->usruuid);
		if (rc)
			return -ENODATA;

	} else if ((str = strstr(line, "device-uuid="))) {
		rc = uuid_parse(str + 12, cfg->devuuid);
		if (rc)
			return -ENODATA;

	} else if ((str = strstr(line, "device-name="))) {
		strncpy(cfg->devname, str + 12, sizeof(cfg->devname) - 1);

	} else if ((str = strstr(line, "device-is-dummy="))) {
		unsigned is_dummy;
		rc = sscanf(str + 16, "%d", &is_dummy);
		if (rc != 1)
			return -ENODATA;

		if (is_dummy)
			cfg->contype = SKY_DUMMY;

	} else if ((str = strstr(line, "broker-url="))) {
		rc = sscanf(str + 11, "%64[^:]:%u,%u", cfg->hostname,
			    &cfg->srvport, &cfg->cliport);
		if (rc != 3)
			return -ENODATA;

		cfg->subport = cfg->srvport + 1;
		cfg->pubport = cfg->cliport + 1;

	} else if ((str = strstr(line, "mux-hw1-eeprom-inited="))) {
		unsigned p = SKY_EEPROM_INITED;
		rc = sscanf(str + 22, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-scanning-interval="))) {
		unsigned p = SKY_SCANNING_INTERVAL;
		rc = sscanf(str + 26, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-precharging-interval="))) {
		unsigned p = SKY_PRECHARGING_INTERVAL;
		rc = sscanf(str + 29, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-precharging-counter="))) {
		unsigned p = SKY_PRECHARGING_COUNTER;
		rc = sscanf(str + 28, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-postcharging-interval="))) {
		unsigned p = SKY_POSTCHARGING_INTERVAL;
		rc = sscanf(str + 30, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-postcharging-delay="))) {
		unsigned p = SKY_POSTCHARGING_DELAY;
		rc = sscanf(str + 27, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-wet-delay="))) {
		unsigned p = SKY_WET_DELAY;
		rc = sscanf(str + 18, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-shortcirc-delay="))) {
		unsigned p = SKY_SHORTCIRC_DELAY;
		rc = sscanf(str + 24, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-thresh-finish-charging="))) {
		unsigned p = SKY_THRESH_FINISH_CHARGING;
		rc = sscanf(str + 31, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-thresh-nocharger-present="))) {
		unsigned p = SKY_THRESH_NOCHARGER_PRESENT;
		rc = sscanf(str + 33, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-thresh-shortcirc="))) {
		unsigned p = SKY_THRESH_SHORTCIRC;
		rc = sscanf(str + 25, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-current-mon-interval="))) {
		unsigned p = SKY_CURRENT_MON_INTERVAL;
		rc = sscanf(str + 29, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-wait-start-charging-sec="))) {
		unsigned p = SKY_WAIT_START_CHARGING_SEC;
		rc = sscanf(str + 32, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "psu-type="))) {
		if (0 == strcasecmp(str + 9, "rsp-750-48") ||
		    0 == strcasecmp(str + 9, "rsp750-48"))
			cfg->psu.type = SKY_PSU_RSP_750_48;
		/* TODO not yet supported! */
		/* else if (0 == strcasecmp(str + 9, "rsp-1600-48") || */
		/* 	 0 == strcasecmp(str + 9, "rsp1600-48")) */
		/* 	cfg->psu.type = SKY_PSU_RSP_1600_48; */
		else
			return -ENODATA;

	} else if ((str = strstr(line, "psu-voltage="))) {
		rc = sscanf(str + 12, "%f", &cfg->psu.voltage);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "psu-current="))) {
		rc = sscanf(str + 12, "%f", &cfg->psu.current);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "psu-precharge-current="))) {
		rc = sscanf(str + 22, "%f", &cfg->psu.precharge_current);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "psu-precharge-current-coef="))) {
		rc = sscanf(str + 27, "%f", &cfg->psu.precharge_current_coef);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "psu-precharge-secs="))) {
		rc = sscanf(str + 19, "%u", &cfg->psu.precharge_secs);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "dp-hw-interface="))) {
		char buf[16];

		strncpy(buf, str + 16, sizeof(buf) - 1);
		if (!strcmp(buf, "gpio"))
			cfg->dp.hw_interface = SKY_DP_GPIO;
		else
			return -ENODATA;

	} else if ((str = strstr(line, "dp-open-pin="))) {
		rc = sscanf(str + 12, "%8s", cfg->dp.gpio.open_pin);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "dp-close-pin="))) {
		rc = sscanf(str + 13, "%8s", cfg->dp.gpio.close_pin);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "dp-is-opened-pin="))) {
		rc = sscanf(str + 17, "%8s", cfg->dp.gpio.is_opened_pin);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "dp-is-closed-pin="))) {
		rc = sscanf(str + 17, "%8s", cfg->dp.gpio.is_closed_pin);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "dp-in-progress-pin="))) {
		rc = sscanf(str + 19, "%8s", cfg->dp.gpio.in_progress_pin);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "dp-is-landing-err-pin="))) {
		rc = sscanf(str + 22, "%8s", cfg->dp.gpio.is_landing_err_pin);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "dp-is-ready-pin="))) {
		rc = sscanf(str + 16, "%8s", cfg->dp.gpio.is_ready_pin);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "dp-is-drone-detected-pin="))) {
		rc = sscanf(str + 25, "%8s", cfg->dp.gpio.is_drone_detected_pin);
		if (rc != 1)
			return -ENODATA;
	}

	/*
	 * No else: ignore unknown parameters
	 */

	return 0;
}

void sky_confinit(struct sky_conf *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
}

int sky_confparse(const char *path, struct sky_conf *cfg)
{
	FILE *fp;
	char *line = NULL;
	size_t len = 0;
	int rc = 0;

	sky_confinit(cfg);

	fp = fopen(path, "r");
	if (fp == NULL)
		return -ENOENT;

	while (getline(&line, &len, fp) != -1)
		if ((rc = parse_line(line, cfg)))
			break;

	free(line);
	fclose(fp);

	return rc;
}

int sky_discoverbroker(struct sky_brokerinfo *brokerinfo,
		       unsigned int timeout_ms)
{
	struct sky_dev_ops *ops;

	foreach_devops(ops, devops) {
		if (ops->contype != SKY_REMOTE)
			continue;
		if (!ops->discoverbroker)
			continue;
		return ops->discoverbroker(ops, brokerinfo, timeout_ms);
	}

	return -EOPNOTSUPP;
}

int sky_peerinfo(const struct sky_conf *conf,
		 struct sky_peerinfo *peerinfo)
{
	struct sky_dev_ops *ops;
	int rc = -EINVAL;

	foreach_devops(ops, devops) {
		if (ops->contype != conf->contype)
			continue;
		return ops->peerinfo(ops, conf, peerinfo);
	}

	return rc;
}

int sky_devslist(const struct sky_conf *conf,
		 struct sky_dev_desc **out)
{
	struct sky_dev_desc *head = NULL;
	struct sky_dev_ops *ops;
	int rc = -EINVAL;

	foreach_devops(ops, devops) {
		if (ops->contype != conf->contype)
			continue;
		rc = ops->devslist(ops, conf, &head);
		if (rc)
			goto err;
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

int sky_devopen(const struct sky_dev_desc *devdesc, struct sky_dev **dev_)
{
	const struct sky_dev_ops *ops = devdesc->dev_ops;
	int rc;

	rc = ops->devopen(devdesc, dev_);
	if (!rc) {
		struct sky_dev *dev = *dev_;

		dev->devdesc = *devdesc;
		dev->devdesc.next = NULL;
	}

	return rc;
}

void sky_devclose(struct sky_dev *dev)
{
	if (!dev)
		return;
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

int sky_paramsset(struct sky_dev *dev, const struct sky_dev_params *params)
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
		if (rc >= 0)
			/* Notify subscribers only in case of success */
			dev->subsc.on_state(dev->subsc.data, &state);

		/* We sleep only if subscription_work wants us to */
		if (rc == 0 && ms < dev->subsc.interval_msecs) {
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

int sky_gpsdata(struct sky_dev *dev, struct sky_gpsdata *gpsdata)
{
	return get_devops(dev)->gpsdata(dev, gpsdata);
}

int sky_dronedetect(struct sky_dev *dev, enum sky_drone_status *status)
{
	return get_devops(dev)->dronedetect(dev, status);
}

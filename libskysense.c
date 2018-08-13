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

	} else if ((str = strstr(line, "broker-url="))) {
		rc = sscanf(str + 11, "%64[^:]:%u,%u", cfg->hostname,
			    &cfg->srvport, &cfg->cliport);
		if (rc != 3)
			return -ENODATA;

		cfg->subport = cfg->srvport + 1;
		cfg->pubport = cfg->cliport + 1;
	}
	//XXX REMOVE ASAP
	else if ((str = strstr(line, "charge-current-ma="))) {
		rc = sscanf(str + 18, "%u", &cfg->XXX_charge_current_ma);
		if (rc != 1)
			return -ENODATA;
	}
	//XXX REMOVE ASAP
	else if ((str = strstr(line, "battery-capacity-mah="))) {
		rc = sscanf(str + 21, "%u", &cfg->XXX_battery_capacity_mah);
		if (rc != 1)
			return -ENODATA;
	}

	/*
	 * No else: ignore unknown parameters
	 */

	return 0;
}

int sky_confparse(const char *path, struct sky_conf *cfg)
{
	FILE *fp;
	char *line = NULL;
	size_t len = 0;
	int rc = 0;

	memset(cfg, 0, sizeof(*cfg));

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

int sky_peerinfo(const struct sky_dev_conf *conf, size_t num,
		 struct sky_peerinfo *peerinfo)
{
	struct sky_dev_ops *ops;
	int i, rc = -EINVAL;

	for (i = 0; i < num; i++) {
		foreach_devops(ops, devops) {
			if (ops->contype != conf->contype)
				continue;
			rc = ops->peerinfo(ops, conf, &peerinfo[i]);
			if (rc)
				return rc;
		}
	}

	return rc;
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
		dev->devdesc.next = NULL;
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

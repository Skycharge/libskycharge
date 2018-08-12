#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>

#include <gps.h>

#include "libskysense-pri.h"
#include "bms-btle.h"
#include "types.h"

struct skydum_dev {
	struct sky_dev dev;
	struct sky_charging_state state;
	struct sky_dev_params params;
	struct sky_dev_desc devdesc;
	struct gps_data_t gpsdata;
	struct bms *bms;
	bool gps_nodev;
	bool bms_nodev;

	unsigned cur, vol;
};

static int skydum_peerinfo(const struct sky_dev_ops *ops,
			   const struct sky_dev_conf *conf,
			   struct sky_peerinfo *peerinfo)
{
	return -EOPNOTSUPP;
}

static int skydum_devslist(const struct sky_dev_ops *ops,
			   const struct sky_dev_conf *conf,
			   struct sky_dev_desc **head)
{
	struct sky_dev_desc *dev;

	dev = calloc(1, sizeof(*dev));
	if (!dev)
		return -ENOMEM;

	dev->dev_type = SKY_INDOOR;
	dev->firmware_version = 0x00010203;
	dev->conf = *conf;
	dev->opaque_ops = ops;
	strcpy(dev->portname, "dummy0");

	dev->next = *head;
	*head = dev;

	return 0;
}

static unsigned rand_between(unsigned M, unsigned N)
{
	return M + rand() / (RAND_MAX / (N - M + 1) + 1);
}

static int skydum_devopen(const struct sky_dev_desc *devdesc,
			  struct sky_dev **dev_)
{
	struct skydum_dev *dev;
	struct timespec ts;
	int rc;

	clock_gettime(CLOCK_REALTIME, &ts);
	srand(ts.tv_nsec * ts.tv_sec);

	dev = calloc(1, sizeof(*dev));
	if (!dev)
		return -ENOMEM;

	rc = gps_open(GPSD_SHARED_MEMORY, NULL, &dev->gpsdata);
	if (rc)
		/* Do not make much noise if GPS does not exist */
		dev->gps_nodev = true;

	rc = bms_open(&dev->bms);
	if (rc)
		/* Do not make much noise if BMS-BTLE dongle does not exist */
		dev->bms_nodev = true;

	dev->cur = rand_between(100,  5000);
	dev->vol = rand_between(10000, 30000);

	*dev_ = &dev->dev;

	return 0;
}

static void skydum_devclose(struct sky_dev *dev_)
{
	struct skydum_dev *dev;

	dev = container_of(dev_, struct skydum_dev, dev);
	if (!dev->bms_nodev)
		bms_close(dev->bms);
	if (!dev->gps_nodev)
		gps_close(&dev->gpsdata);
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
	uint16_t charge_perc;
	int rc;

	dev = container_of(dev_, struct skydum_dev, dev);

	dev->state.current = dev->cur;
	dev->state.voltage = dev->vol;

	dev->state.dev_hw_state += 1;
	if (dev->state.dev_hw_state == 4)
		dev->state.dev_hw_state = 5;
	if (dev->state.dev_hw_state == 11)
		dev->state.dev_hw_state = 12;
	if (dev->state.dev_hw_state == 21)
		dev->state.dev_hw_state = 22;
	dev->state.dev_hw_state %= 26;

	*state = dev->state;

	/* First reset bms values to non valid */
	state->bms.charge_time = 0;
	state->bms.charge_perc = 0;

	if (!dev->bms_nodev) {
		rc = bms_request_nearest(dev->bms, &charge_perc, NULL);
		if (!rc) {
			/* XXX HOLY CRAP! */
			unsigned secs_in_h = 3600;
			unsigned capacity_mah = 32000; /* two batteries 16'000 mAh each */
			unsigned charged_mah = capacity_mah / 100 * charge_perc;
			unsigned charging_secs = charged_mah * secs_in_h / capacity_mah;
			unsigned left_secs = secs_in_h - charging_secs;

			state->bms.charge_time = left_secs;
			state->bms.charge_perc = charge_perc;
		}
	}

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
	return skydum_chargingstate(dev_, state);
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

static int skydum_gpsdata(struct sky_dev *dev_, struct sky_gpsdata *gpsdata)
{
	struct skydum_dev *dev;
	struct timespec ts;
	double msecs;
	int rc;

	dev = container_of(dev_, struct skydum_dev, dev);
	if (!dev->gps_nodev) {
		rc = gps_read(&dev->gpsdata);
		if (rc < 0)
			return -ENODEV;

		memset(gpsdata, 0, sizeof(*gpsdata));
		gpsdata->status = dev->gpsdata.status;
		gpsdata->satellites_used = dev->gpsdata.satellites_used;
		BUILD_BUG_ON(sizeof(gpsdata->dop) != sizeof(dev->gpsdata.dop));
		memcpy(&gpsdata->dop, &dev->gpsdata.dop, sizeof(gpsdata->dop));
		gpsdata->fix.mode = dev->gpsdata.fix.mode;
		gpsdata->fix.time = dev->gpsdata.fix.time;
		gpsdata->fix.latitude  = dev->gpsdata.fix.latitude;
		gpsdata->fix.longitude = dev->gpsdata.fix.longitude;
		gpsdata->fix.altitude  = dev->gpsdata.fix.altitude;
	} else {
		/*
		 * No real GPS, output dummy stuff
		 */
		memset(gpsdata, 0, sizeof(*gpsdata));

		clock_gettime(CLOCK_REALTIME, &ts);

		msecs = ts.tv_nsec / 1000000ull;
		/*
		 * integer to fractional, e.g. 123.0 -> 0.123,
		 * the equation is: x / 10 ^ ceil(log10(msecs))
		 */
		while (msecs > 1.0)
			msecs /= 10;

		gpsdata->fix.time = (double)ts.tv_sec + msecs;
		gpsdata->fix.latitude  = 12345.54321;
		gpsdata->fix.longitude = 54321.12345;
		gpsdata->fix.altitude  = NAN;

		gpsdata->status = SKY_GPS_STATUS_DGPS_FIX;
		gpsdata->fix.mode = SKY_GPS_MODE_3D;

		gpsdata->satellites_used = 42;
		gpsdata->dop.pdop = 12.21;
		gpsdata->dop.hdop = 21.12;
		gpsdata->dop.vdop = 23.32;
		gpsdata->dop.tdop = 34.43;
		gpsdata->dop.gdop = 54.43;
	}

	return 0;
}

static int skydum_dronedetect(struct sky_dev *dev_,
			      enum sky_drone_status *status)
{
	*status = SKY_DRONE_NOT_DETECTED;
	return 0;
}

static struct sky_dev_ops sky_dummy_devops = {
	.contype = SKY_LOCAL,
	.peerinfo = skydum_peerinfo,
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
	.coverclose = skydum_coverclose,
	.gpsdata = skydum_gpsdata,
	.dronedetect = skydum_dronedetect,
};
sky_register_devops(&sky_dummy_devops);

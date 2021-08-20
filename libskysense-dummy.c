#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>

#include <gps.h>

#include "libskysense-pri.h"
#include "libskybms.h"
#include "types.h"

struct skydum_dev {
	struct sky_dev dev;
	struct sky_charging_state state;
	struct sky_dev_params params;
	struct sky_dev_desc devdesc;
	struct gps_data_t gpsdata;
	struct bms_lib bms;
	bool gps_nodev;

	struct sky_droneport_state dp_state;

	unsigned capacity_mAh;
	unsigned cur, vol;

	unsigned long long ts_ms;
};

static int skydum_peerinfo(const struct sky_dev_ops *ops,
			   const struct sky_conf *conf,
			   struct sky_peerinfo *peerinfo)
{
	return -EOPNOTSUPP;
}

static int skydum_devslist(const struct sky_dev_ops *ops,
			   const struct sky_conf *conf,
			   struct sky_dev_desc **head)
{
	struct sky_dev_desc *dev;

	dev = calloc(1, sizeof(*dev));
	if (!dev)
		return -ENOMEM;

	dev->dev_type = SKY_MUX_HW1;
	dev->fw_version = 0x00010203;
	dev->hw_version = 0x00040302;
	dev->conf = *conf;
	dev->dev_ops = ops;
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

	dev->dp_state.status |= (SKY_DP_IS_READY | SKY_DP_IS_CLOSED);

	bms_init(&dev->bms);

	dev->capacity_mAh = 500;
	dev->cur = rand_between(5000,  15000);
	dev->vol = rand_between(16000, 51000);
	dev->ts_ms = msecs_epoch();

	*dev_ = &dev->dev;

	return 0;
}

static void skydum_devclose(struct sky_dev *dev_)
{
	struct skydum_dev *dev;

	dev = container_of(dev_, struct skydum_dev, dev);
	bms_deinit(&dev->bms);
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

	dev = container_of(dev_, struct skydum_dev, dev);
	for (i = 0; i < SKY_HW1_NUM_DEVPARAM; i++) {
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
		return 0;

	dev = container_of(dev_, struct skydum_dev, dev);
	for (i = 0; i < SKY_HW1_NUM_DEVPARAM; i++) {
		if (!(params->dev_params_bits & (1<<i)))
				continue;
		dev->params.dev_params[i] = params->dev_params[i];
	}

	return 0;
}

static int skydum_chargingstate(struct sky_dev *dev_,
				struct sky_charging_state *state)
{
	struct bms_data bms_data;
	struct skydum_dev *dev;
	float dt;
	int rc;

	dev = container_of(dev_, struct skydum_dev, dev);

	dt = (msecs_epoch() - dev->ts_ms) / 1000.0;

	dev->state.mux_temperature_C = rand_between(43, 56);
	dev->state.sink_temperature_C = rand_between(55, 64);
	dev->state.current_mA = dev->cur;
	dev->state.voltage_mV = dev->vol;
	dev->state.charging_secs = dt;
	dev->state.energy_mWh = dev->vol * dev->cur / 1000 * (dt / 3600.0);
	dev->state.charge_mAh = dev->cur * (dt / 3600.0);

	dev->state.dev_hw_state += 1;
	if (dev->state.dev_hw_state == 4)
		dev->state.dev_hw_state = 5;
	if (dev->state.dev_hw_state == 11)
		dev->state.dev_hw_state = 12;
	if (dev->state.dev_hw_state == 21)
		dev->state.dev_hw_state = 22;
	dev->state.dev_hw_state %= 26;

	*state = dev->state;

	rc = bms_request_data(&dev->bms, &bms_data);
	if (!rc) {
		state->until_full_secs = bms_data.charge_time;
		state->state_of_charge = bms_data.charge_perc;
	} else {
		unsigned int until_full_mAh;

		state->state_of_charge =
			(float)dev->state.charge_mAh / dev->capacity_mAh * 100;

		if (state->state_of_charge >= 100) {
			/* Start new charging */

			dev->ts_ms = msecs_epoch();
			dev->state.current_mA = dev->cur =
				rand_between(5000,  15000);
			dev->state.voltage_mV = dev->vol =
				rand_between(16000, 51000);
			state->state_of_charge = 0;
			dev->state.charging_secs = 0;
			dev->state.energy_mWh = 0;
			dev->state.charge_mAh = 0;
		}

		until_full_mAh = dev->capacity_mAh - dev->state.charge_mAh;
		state->until_full_secs =
			(float)until_full_mAh / dev->cur * 3600.0;
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

static int skydum_droneport_open(struct sky_dev *dev_)
{
	struct skydum_dev *dev;

	dev = container_of(dev_, struct skydum_dev, dev);
	if (dev->dp_state.status & SKY_DP_IS_OPENED)
		return -EALREADY;

	dev->dp_state.status |= SKY_DP_IS_OPENED;
	dev->dp_state.status &= ~SKY_DP_IS_CLOSED;

	return 0;
}

static int skydum_droneport_close(struct sky_dev *dev_)
{
	struct skydum_dev *dev;

	dev = container_of(dev_, struct skydum_dev, dev);
	if (dev->dp_state.status & SKY_DP_IS_CLOSED)
		return -EALREADY;

	dev->dp_state.status |= SKY_DP_IS_CLOSED;
	dev->dp_state.status &= ~SKY_DP_IS_OPENED;

	return 0;
}

static int
skydum_droneport_state(struct sky_dev *dev_, struct sky_droneport_state *state)
{
	struct skydum_dev *dev;

	dev = container_of(dev_, struct skydum_dev, dev);
	state->status = dev->dp_state.status;

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
#if GPSD_API_MAJOR_VERSION <= 6
		rc = gps_read(&dev->gpsdata);
#else
		rc = gps_read(&dev->gpsdata, NULL, 0);
#endif
		if (rc < 0)
			return -EOPNOTSUPP;

		memset(gpsdata, 0, sizeof(*gpsdata));
#if GPSD_API_MAJOR_VERSION < 10
		gpsdata->status = dev->gpsdata.status;
#else
		gpsdata->status = dev->gpsdata.fix.status;
#endif
		gpsdata->satellites_used = dev->gpsdata.satellites_used;
		BUILD_BUG_ON(sizeof(gpsdata->dop) != sizeof(dev->gpsdata.dop));
		memcpy(&gpsdata->dop, &dev->gpsdata.dop, sizeof(gpsdata->dop));
		gpsdata->fix.mode = dev->gpsdata.fix.mode;
#if GPSD_API_MAJOR_VERSION < 9
		gpsdata->fix.time = dev->gpsdata.fix.time;
#else
		{
			double tstamp = dev->gpsdata.fix.time.tv_sec;

			tstamp += dev->gpsdata.fix.time.tv_nsec / 1000000000.0;
			gpsdata->fix.time = tstamp;
		}
#endif
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

#if GPSD_API_MAJOR_VERSION < 9
		gpsdata->fix.time = (double)ts.tv_sec + msecs;
#else
		{
			double tstamp = ts.tv_sec;

			tstamp += ts.tv_nsec / 1000000000.0;
			gpsdata->fix.time = tstamp;
		}
#endif
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

static bool skydum_asyncreq_cancel(struct sky_async *async,
				   struct sky_async_req *req)
{
	bool res = false;

	if (req->next != req) {
		/* Request still in the submit queue */
		assert(!req->tag);
		sky_asyncreq_del(req);
		res = true;
	}

	return res;
}

static int skydum_asyncopen(const struct sky_conf *conf,
			    const struct sky_dev_ops *ops,
			    struct sky_async **async_)
{
	struct sky_async *async;

	async = calloc(1, sizeof(*async));
	if (!async)
		return -ENOMEM;

	sky_async_init(conf, ops, async);

	*async_ = async;

	return 0;
}

static void skydum_asyncclose(struct sky_async *async)
{
	while (!sky_async_empty(async)) {
		struct sky_async_req *req;

		req = sky_asyncreq_pop(async);
		sky_asyncreq_complete(async, req, -EIO);
	}
	free(async);
}

static int skydum_asyncfd(struct sky_async *async)
{
	/* Probably in future we support that */
	return -EOPNOTSUPP;
}

static int skydum_asyncreq_execute(struct sky_async *async,
				   struct sky_async_req *req)
{
	int rc;

	switch (req->type) {
	case SKY_GET_DEV_PARAMS_REQ:
		rc = skydum_paramsget(req->dev, req->out.ptr);
		break;
	case SKY_SET_DEV_PARAMS_REQ:
		rc = skydum_paramsset(req->dev, req->in.ptr);
		break;
	case SKY_START_CHARGE_REQ:
		rc = skydum_chargestart(req->dev);
		break;
	case SKY_STOP_CHARGE_REQ:
		rc = skydum_chargestop(req->dev);
		break;
	case SKY_PSU_SET_TYPE_REQ:
		rc = -EOPNOTSUPP;
		break;
	case SKY_PSU_SET_VOLTAGE_REQ:
		rc = -EOPNOTSUPP;
		break;
	case SKY_PSU_SET_CURRENT_REQ:
		rc = -EOPNOTSUPP;
		break;
	case SKY_OPEN_DRONEPORT_REQ:
		rc = skydum_droneport_open(req->dev);
		break;
	case SKY_CLOSE_DRONEPORT_REQ:
		rc = skydum_droneport_close(req->dev);
		break;
	case SKY_DRONEPORT_STATE_REQ:
		rc = skydum_droneport_state(req->dev, req->out.ptr);
		break;
	case SKY_RESET_DEV_REQ:
		rc = skydum_reset(req->dev);
		break;
	case SKY_CHARGING_STATE_REQ:
		rc = skydum_chargingstate(req->dev, req->out.ptr);
		break;
	case SKY_DEVS_LIST_REQ:
		rc = skydum_devslist(async->ops, async->conf, req->out.ptr);
		break;
	case SKY_PEERINFO_REQ:
		rc = skydum_peerinfo(async->ops, async->conf, req->out.ptr);
		break;
	case SKY_GPSDATA_REQ:
		rc = skydum_gpsdata(req->dev, req->out.ptr);
		break;
	case SKY_DRONEDETECT_REQ:
		rc = skydum_dronedetect(req->dev, req->out.ptr);
		break;
	default:
		/* Consider fatal */
		sky_err("Unknown request: %d\n", req->type);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int skydum_asyncexecute(struct sky_async *async, bool wait)
{
	int completed = 0;

	while (!sky_async_empty(async)) {
		struct sky_async_req *req;
		int rc;

		req = sky_asyncreq_pop(async);
		rc = skydum_asyncreq_execute(async, req);
		sky_asyncreq_complete(async, req, rc);
		completed++;
	}

	return completed;
}

static struct sky_dev_ops sky_dummy_devops = {
	.contype           = SKY_DUMMY,
	.asyncopen         = skydum_asyncopen,
	.asyncclose        = skydum_asyncclose,
	.asyncexecute      = skydum_asyncexecute,
	.asyncfd           = skydum_asyncfd,
	.asyncreq_cancel   = skydum_asyncreq_cancel,
	.devopen           = skydum_devopen,
	.devclose          = skydum_devclose,
	.subscribe         = skydum_subscribe,
	.unsubscribe       = skydum_unsubscribe,
	.subscription_work = skydum_subscription_work,
};
sky_register_devops(&sky_dummy_devops);

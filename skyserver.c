/*
 * Copyright (C) 2021-2022 Skycharge GmbH
 * Author: Roman Penyaev <r.peniaev@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met: 1. Redistributions of source code must retain the above
 * copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <endian.h>
#include <limits.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <uuid/uuid.h>
#include <systemd/sd-id128.h>

#include <zmq.h>
#include <czmq.h>

#include "skyserver-cmd.h"
#include "libskycharge.h"
#include "libskypsu.h"
#include "daemon.h"
#include "avahi.h"
#include "crc8.h"
#include "skyproto.h"
#include "version.h"
#include "types.h"

struct zocket {
	void *ctx;
	void *pub;
	void *router;
};

struct sky_server;

struct sky_server_dev {
	struct sky_server *serv;
	struct sky_dev_desc *devdesc;
	struct sky_dev *dev;
	struct {
		struct sky_charging_state prev_state;
		unsigned                  precharge_iter;
	} hw1; /* PSU is controlled by the BB for HW1 only */
};

struct sky_server {
	bool exit;
	sd_id128_t id;
	struct cli cli;
	struct sky_conf conf; /* Local device config */
	struct zocket zock;
	struct sky_server_dev *devs;
	struct sky_dev_desc *devhead;
	struct sky_psu hw1_psu;
	struct avahi *avahi;
};

static int sky_kill_pthread(pthread_t thread)
{
	int rc;

	do {
		struct timespec ts;

		/*
		 * Repeat killing to cover the race if first signal
		 * comes when we are not in kernel.
		 */
		clock_gettime(CLOCK_REALTIME, &ts);
		ts.tv_sec += 1;
		pthread_kill(thread, SIGTERM);
		rc = pthread_timedjoin_np(thread, NULL, &ts);
	} while (rc == ETIMEDOUT);

	return rc;
}

static float get_precharge_current(struct sky_server_dev *servdev)
{
	struct sky_server *serv = servdev->serv;
	struct sky_conf *conf = &serv->conf;

	float current_delta;

	/*
	 * TODO Here we assume this function is called exactly with
	 *      1 second rate
	 */
	if (servdev->hw1.precharge_iter >= conf->psu.precharge_secs)
		return conf->psu.current;

	servdev->hw1.precharge_iter++;

	current_delta =	conf->psu.current - conf->psu.precharge_current;
	current_delta /= conf->psu.precharge_secs;

	return conf->psu.precharge_current +
		current_delta * servdev->hw1.precharge_iter;
}

static void sky_on_charging_state(void *data, struct sky_charging_state *state)
{
	struct sky_server_dev *servdev = data;
	struct sky_server *serv = servdev->serv;
	struct sky_conf *conf = &serv->conf;
	struct sky_dev_desc *devdesc = servdev->devdesc;
	struct sky_charging_state_rsp rsp;
	zmsg_t *msg;
	int rc;

	if (serv->hw1_psu.type != SKY_PSU_UNKNOWN &&
	    sky_psu_is_precharge_set(&serv->hw1_psu)) {
		bool was_charging, is_charging;

		was_charging =
			sky_hw_is_charging(devdesc->dev_type,
					   servdev->hw1.prev_state.dev_hw_state);
		is_charging =
			sky_hw_is_charging(devdesc->dev_type,
					   state->dev_hw_state);

		if (was_charging && is_charging) {
			/*
			 * Increment precharge current
			 */
			float current;

			current = get_precharge_current(servdev);
			sky_psu_set_current(&serv->hw1_psu, current);

		} else if (was_charging ^ is_charging) {
			/*
			 * Set to precharge current if charging has been
			 * started or has been stopped.
			 */
			sky_psu_set_current(&serv->hw1_psu,
					    conf->psu.precharge_current);
			servdev->hw1.precharge_iter = 0;
		}

		servdev->hw1.prev_state = *state;
	}

	rsp.hdr.type            = htole16(SKY_CHARGING_STATE_EV);
	rsp.hdr.error           = 0;
	rsp.dev_hw_state        = state->dev_hw_state;
	rsp.dev_hw_reason       = state->dev_hw_reason;
	rsp.current_mA          = htole16(state->current_mA);
	rsp.voltage_mV          = htole16(state->voltage_mV);
	rsp.state_of_charge     = htole16(state->state_of_charge);
	rsp.until_full_secs     = htole16(state->until_full_secs);
	rsp.charging_secs       = htole16(state->charging_secs);
	rsp.mux_temperature_C   = htole16(state->mux_temperature_C);
	rsp.sink_temperature_C  = htole16(state->sink_temperature_C);
	rsp.energy_mWh          = htole32(state->energy_mWh);
	rsp.charge_mAh          = htole32(state->charge_mAh);
	rsp.mux_humidity_perc   = state->mux_humidity_perc;
	rsp.link_quality_factor = state->link_quality_factor;
	rsp.tx.bytes            = htole16(state->tx.bytes);
	rsp.tx.packets          = htole16(state->tx.packets);
	rsp.tx.err_bytes        = htole16(state->tx.err_bytes);
	rsp.tx.err_packets      = htole16(state->tx.err_packets);
	rsp.rx.bytes            = htole16(state->rx.bytes);
	rsp.rx.packets          = htole16(state->rx.packets);
	rsp.rx.err_bytes        = htole16(state->rx.err_bytes);
	rsp.rx.err_packets      = htole16(state->rx.err_packets);
	rsp.charging_session_id	= htole32(state->charging_session_id);

	msg = zmsg_new();
	if (!msg) {
		sky_err("zmsg_new(): No memory\n");
		return;
	}
	/* Publisher topic */
	rc = zmsg_addmem(msg, &serv->id, sizeof(serv->id));
	if (!rc)
		rc = zmsg_addmem(msg, &rsp, sizeof(rsp));
	if (!rc)
		rc = zmsg_addmem(msg, conf->devuuid, sizeof(conf->devuuid));
	if (rc) {
		sky_err("zmsg_add(): No memory\n");
		zmsg_destroy(&msg);
		return;
	}
	rc = zmsg_send(&msg, serv->zock.pub);
	if (rc) {
		sky_err("zmsg_send(): %s\n", strerror(errno));
		zmsg_destroy(&msg);
	}
}

static struct sky_rsp_hdr emergency_rsp;

static inline void sky_free(void *rsp)
{
	if (rsp != &emergency_rsp)
		free(rsp);
}

static inline const void *zframe_data_const(const zframe_t *frame)
{
	return zframe_data((zframe_t *)frame);
}

static inline size_t zframe_size_const(const zframe_t *frame)
{
	return zframe_size((zframe_t *)frame);
}

static inline struct sky_dev *sky_find_dev(struct sky_server *serv,
					   const zframe_t *devport_frame)
{
	struct sky_dev_desc *devdesc;
	const char *portname;
	size_t len, num;

	if (devport_frame == NULL)
		return NULL;

	portname = zframe_data_const(devport_frame);
	len = min(sizeof(devdesc->portname),
		  zframe_size((zframe_t *)devport_frame));

	/*
	 * For now do linear search.  Far from optimal, but simple.
	 */

	num = 0;
	foreach_devdesc(devdesc, serv->devhead) {
		struct sky_server_dev *servdev = &serv->devs[num++];

		if (!memcmp(devdesc->portname, portname, len))
			return servdev->dev;
	}

	return NULL;
}

static int sky_devs_list_rsp(struct sky_server *serv, const char *dev_name,
			     uint16_t proto_version, void **rsp_hdr, size_t *rsp_len)
{
	struct sky_conf *conf = &serv->conf;
	struct sky_dev_desc *dev, *head;
	struct sky_devs_list_rsp *rsp;
	void *rsp_void = NULL;
	size_t len, info_len;
	bool dyn_info;
	int rc;

	/* Compatibility with protocols below 0x0400 */
	dyn_info = (proto_version >= 0x0400);

	len = dyn_info ?
		sizeof(*rsp) :
		offsetof_end(typeof(*rsp), info_off);
	rsp = rsp_void = calloc(1, len);
	if (!rsp)
		return -ENOMEM;

	rc = sky_devslist(conf, &head);

	rsp->hdr.type  = htole16(SKY_DEVS_LIST_RSP);
	rsp->hdr.error = htole16(-rc);
	if (!rc) {
		int num;

		/* Compatibility with protocols below 0x0400 */
		info_len = dyn_info ?
			sizeof(rsp->info[0]) :
			offsetof_end(typeof(rsp->info[0]), portname);

		foreach_devdesc(dev, head)
			len += info_len;

		rsp = realloc(rsp, len);
		if (!rsp) {
			free(rsp_void);
			sky_devsfree(head);
			return -ENOMEM;
		}
		rsp_void = rsp;
		num = 0;
		foreach_devdesc(dev, head) {
			struct sky_dev_info *info =
				(void *)&rsp->info[0] + info_len * num;

			memset(info, 0, info_len);
			info->dev_type = htole16(dev->dev_type);
			info->fw_version = htole32(dev->hw_info.fw_version);
			memcpy(info->portname, dev->portname,
			       sizeof(dev->portname));
			memcpy(info->dev_uuid, conf->devuuid,
			       sizeof(conf->devuuid));
			strncpy(info->dev_name, dev_name, sizeof(info->dev_name));

			if (dyn_info) {
				/* Protocol version >= 0x0400 */
				info->info_len = htole16(sizeof(*info));
				info->proto_version =
					htole16(SKY_PROTO_VERSION);
				info->hw_version =
					htole32(dev->hw_info.hw_version);
				info->plc_proto_version =
					htole32(dev->hw_info.plc_proto_version);
				info->hw_uid.part1 =
					htole32(dev->hw_info.uid.part1);
				info->hw_uid.part2 =
					htole32(dev->hw_info.uid.part2);
				info->hw_uid.part3 =
					htole32(dev->hw_info.uid.part3);
			}
			num += 1;
		}
		sky_devsfree(head);
		rsp->num_devs = htole16(num);
		if (dyn_info) {
			/* Protocol version >= 0x0400 */
			rsp->info_off = htole16(offsetof_end(typeof(*rsp), info_off));
		}
	}

	*rsp_hdr = rsp_void;
	*rsp_len = len;

	return rc;
}

static void sky_execute_cmd(struct sky_server *serv,
			    const zframe_t *devport_frame,
			    const zframe_t *data_frame,
			    struct sky_rsp_hdr **rsp_hdr, size_t *rsp_len)
{
	const struct sky_req_hdr *req_hdr = zframe_data_const(data_frame);
	enum sky_proto_type req_type = SKY_UNKNOWN_REQRSP;
	struct sky_dev *dev;

	void *rsp_void = NULL;
	uint16_t proto_version;
	size_t len;
	int rc, i;

	len = zframe_size_const(data_frame);
	if (len < offsetof_end(typeof(*req_hdr), type)) {
		sky_err("malformed request header\n");
		rc = -EINVAL;
		goto emergency;
	} else if (len == offsetof_end(typeof(*req_hdr), type)) {
		/* Protocols below 0x0400 version */
		proto_version = 0;
	} else {
		/* Protocols below 0x0400 version has 0 in the field */
		proto_version = le16toh(req_hdr->proto_version);
	}

	req_type = le16toh(req_hdr->type);

	switch (req_type) {
	case SKY_SINK_GET_DEV_PARAMS_REQ:
	case SKY_GET_DEV_PARAMS_REQ: {
		struct sky_get_dev_params_req *req = (void *)req_hdr;
		struct sky_get_dev_params_rsp *rsp;
		struct sky_dev_params params;
		int num, ind;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		if (zframe_size_const(data_frame) < sizeof(*req)) {
			sky_err("malformed request\n");
			rc = -EINVAL;
			goto emergency;
		}

		params.dev_params_bits = le32toh(req->dev_params_bits);
		if (params.dev_params_bits == 0) {
			sky_err("malformed request: dev_params_bits == 0\n");
			rc = -EINVAL;
			goto emergency;
		}

		for (i = 0, num = 0; i < ARRAY_SIZE(params.dev_params); i++) {
			if (params.dev_params_bits & (1<<i))
				num++;
		}
		len = num * sizeof(rsp->dev_params[0]) + sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		if (req_type == SKY_GET_DEV_PARAMS_REQ)
			rc = sky_paramsget(dev, &params);
		else
			rc = sky_sink_paramsget(dev, &params);

		rsp->hdr.type  = htole16(req_type + 1);
		rsp->hdr.error = htole16(-rc);
		if (!rc) {
			for (i = 0, ind = 0; ind < num; i++) {
				if (!(params.dev_params_bits & (1<<i)))
					continue;
				rsp->dev_params[ind++] =
					htole32(params.dev_params[i]);
			}
		}

		break;
	}
	case SKY_SINK_SET_DEV_PARAMS_REQ:
	case SKY_SET_DEV_PARAMS_REQ: {
		struct sky_set_dev_params_req *req = (void *)req_hdr;
		struct sky_rsp_hdr *rsp;
		struct sky_dev_params params = {
			.dev_params_bits = 0
		};
		int ind;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		if (zframe_size_const(data_frame) < sizeof(*req)) {
			sky_err("malformed request\n");
			rc = -EINVAL;
			goto emergency;
		}

		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		params.dev_params_bits = le32toh(req->dev_params_bits);
		if (params.dev_params_bits == 0) {
			sky_err("malformed request: dev_params_bits == 0\n");
			rc = -EINVAL;
			goto emergency;
		}

		for (i = 0, ind = 0; i < ARRAY_SIZE(params.dev_params); i++) {
			if (!(params.dev_params_bits & (1<<i)))
				continue;
			if (zframe_size_const(data_frame) <
			    (sizeof(*req) +
			     sizeof(req->dev_params[0]) * (ind + 1))) {
				sky_err("malformed request\n");
				free(rsp);
				rc = -EINVAL;
				goto emergency;
			}
			params.dev_params[i] = le32toh(req->dev_params[ind++]);
		}

		if (req_type == SKY_SET_DEV_PARAMS_REQ)
			rc = sky_paramsset(dev, &params);
		else
			rc = sky_sink_paramsset(dev, &params);

		rsp->type  = htole16(req_type + 1);
		rsp->error = htole16(-rc);

		break;
	}
	case SKY_RESUME_SCAN_REQ: {
		struct sky_rsp_hdr *rsp;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		rc = sky_scanresume(dev);

		rsp->type  = htole16(SKY_RESUME_SCAN_RSP);
		rsp->error = htole16(-rc);

		break;
	}
	case SKY_STOP_SCAN_REQ: {
		struct sky_rsp_hdr *rsp;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		rc = sky_scanstop(dev);

		rsp->type  = htole16(SKY_STOP_SCAN_RSP);
		rsp->error = htole16(-rc);

		break;
	}
	case SKY_OPEN_DRONEPORT_REQ: {
		struct sky_rsp_hdr *rsp;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		rc = sky_droneport_open(dev);

		rsp->type  = htole16(SKY_OPEN_DRONEPORT_RSP);
		rsp->error = htole16(-rc);

		break;
	}
	case SKY_CLOSE_DRONEPORT_REQ: {
		struct sky_rsp_hdr *rsp;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		rc = sky_droneport_close(dev);

		rsp->type  = htole16(SKY_CLOSE_DRONEPORT_RSP);
		rsp->error = htole16(-rc);

		break;
	}
	case SKY_DRONEPORT_STATE_REQ: {
		struct sky_droneport_state_rsp *rsp;
		struct sky_droneport_state state;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		rc = sky_droneport_state(dev, &state);

		rsp->hdr.type  = htole16(SKY_DRONEPORT_STATE_RSP);
		rsp->hdr.error = htole16(-rc);
		if (!rc) {
			rsp->status = htole32(state.status);
		}

		break;
	}
	case SKY_CHARGING_STATE_REQ: {
		struct sky_charging_state_rsp *rsp;
		struct sky_charging_state state;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		rc = sky_chargingstate(dev, &state);

		rsp->hdr.type  = htole16(SKY_CHARGING_STATE_RSP);
		rsp->hdr.error = htole16(-rc);
		if (!rc) {
			rsp->dev_hw_state = state.dev_hw_state;
			rsp->dev_hw_reason = state.dev_hw_reason;
			rsp->current_mA = htole16(state.current_mA);
			rsp->voltage_mV = htole16(state.voltage_mV);
			rsp->state_of_charge =
				htole16(state.state_of_charge);
			rsp->until_full_secs =
				htole16(state.until_full_secs);
			rsp->charging_secs = htole16(state.charging_secs);
			rsp->mux_temperature_C
				= htole16(state.mux_temperature_C);
			rsp->sink_temperature_C
				= htole16(state.sink_temperature_C);
			rsp->energy_mWh = htole32(state.energy_mWh);
			rsp->charge_mAh = htole32(state.charge_mAh);
			rsp->mux_humidity_perc = state.mux_humidity_perc;
			rsp->link_quality_factor = state.link_quality_factor;
			rsp->tx.bytes       = htole16(state.tx.bytes);
			rsp->tx.packets     = htole16(state.tx.packets);
			rsp->tx.err_bytes   = htole16(state.tx.err_bytes);
			rsp->tx.err_packets = htole16(state.tx.err_packets);
			rsp->rx.bytes       = htole16(state.rx.bytes);
			rsp->rx.packets     = htole16(state.rx.packets);
			rsp->rx.err_bytes   = htole16(state.rx.err_bytes);
			rsp->rx.err_packets = htole16(state.rx.err_packets);
			rsp->charging_session_id =
				htole32(state.charging_session_id);
		}

		break;
	}
	case SKY_RESET_DEV_REQ: {
		struct sky_rsp_hdr *rsp;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		rc = sky_reset(dev);

		rsp->type  = htole16(SKY_RESET_DEV_RSP);
		rsp->error = htole16(-rc);

		break;
	}
	case SKY_DEVS_LIST_REQ: {
		rc = sky_devs_list_rsp(serv, serv->conf.devname, proto_version,
				       &rsp_void, &len);
		if (rc)
			goto emergency;
		break;
	}
	case SKY_PEERINFO_REQ: {
		struct sky_peerinfo_rsp *rsp;

		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		rsp->hdr.type  = htole16(SKY_PEERINFO_RSP);
		rsp->hdr.error = htole16(0);
		rsp->proto_version  = htole16(SKY_PROTO_VERSION);
		rsp->server_version = htole32(SKY_VERSION);

		break;
	}
	case SKY_GPSDATA_REQ: {
		struct sky_gpsdata_rsp *rsp;
		struct sky_gpsdata gpsdata;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		rc = sky_gpsdata(dev, &gpsdata);

		rsp->hdr.type  = htole16(SKY_GPSDATA_RSP);
		rsp->hdr.error = htole16(-rc);
		if (!rc) {
			rsp->status = htole32(gpsdata.status);
			rsp->satellites_used = htole32(gpsdata.satellites_used);

#define D2LLU(dp) ({ uint64_t *llu = (uint64_t *)(dp); *llu; })

			rsp->dop.xdop = htole64(D2LLU(&gpsdata.dop.xdop));
			rsp->dop.ydop = htole64(D2LLU(&gpsdata.dop.ydop));
			rsp->dop.pdop = htole64(D2LLU(&gpsdata.dop.pdop));
			rsp->dop.hdop = htole64(D2LLU(&gpsdata.dop.hdop));
			rsp->dop.vdop = htole64(D2LLU(&gpsdata.dop.vdop));
			rsp->dop.tdop = htole64(D2LLU(&gpsdata.dop.tdop));
			rsp->dop.gdop = htole64(D2LLU(&gpsdata.dop.gdop));

			rsp->fix.mode = htole32(gpsdata.fix.mode);
			rsp->fix.time = htole64(D2LLU(&gpsdata.fix.time));
			rsp->fix.latitude  =
				htole64(D2LLU(&gpsdata.fix.latitude));
			rsp->fix.longitude =
				htole64(D2LLU(&gpsdata.fix.longitude));
			rsp->fix.altitude  =
				htole64(D2LLU(&gpsdata.fix.altitude));
		}

		break;
	}
	case SKY_DRONEDETECT_REQ: {
		struct sky_dronedetect_rsp *rsp;
		enum sky_drone_status status;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		rc = sky_dronedetect(dev, &status);

		rsp->hdr.type  = htole16(SKY_DRONEDETECT_RSP);
		rsp->hdr.error = htole16(-rc);
		if (!rc)
			rsp->status = htole16(status);

		break;
	}
	case SKY_SINK_GET_INFO_REQ: {
		struct sky_sink_get_info_rsp *rsp;
		struct sky_sink_info info;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		rc = sky_sink_infoget(dev, &info);

		rsp->hdr.type  = htole16(SKY_SINK_GET_INFO_RSP);
		rsp->hdr.error = htole16(-rc);
		if (!rc) {
			rsp->fw_version =
				htole32(info.hw_info.fw_version);
			rsp->hw_version =
				htole32(info.hw_info.hw_version);
			rsp->plc_proto_version =
				htole32(info.hw_info.plc_proto_version);
			rsp->hw_uid.part1 =
				htole32(info.hw_info.uid.part1);
			rsp->hw_uid.part2 =
				htole32(info.hw_info.uid.part2);
			rsp->hw_uid.part3 =
				htole32(info.hw_info.uid.part3);
		}

		break;

	}
	case SKY_SINK_START_CHARGE_REQ:
	case SKY_SINK_STOP_CHARGE_REQ: {
		struct sky_rsp_hdr *rsp;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		if (req_type == SKY_SINK_START_CHARGE_REQ)
			rc = sky_sink_chargestart(dev);
		else
			rc = sky_sink_chargestop(dev);

		rsp->type  = htole16(req_type + 1);
		rsp->error = htole16(-rc);

		break;
	}
	case SKY_GET_SUBSCRIPTION_TOKEN_REQ: {
		struct sky_subscription_token_rsp *rsp;

		len = sizeof(*rsp) + sizeof(serv->id);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		rsp->hdr.type  = htole16(req_type + 1);
		rsp->hdr.error = htole16(0);
		rsp->len       = htole16(sizeof(serv->id.bytes));
		memcpy(rsp->buf, serv->id.bytes, sizeof(serv->id.bytes));

		break;
	}
	case SKY_SINK_PASSTHRU_MSG_SEND_REQ: {
		struct sky_sink_passthru_msg_send_req *req = (void *)req_hdr;
		struct sky_passthru_msg msg;
		struct sky_rsp_hdr *rsp;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}
		if (zframe_size_const(data_frame) < sizeof(*req)) {
			sky_err("malformed request\n");
			rc = -EINVAL;
			goto emergency;
		}
		if (zframe_size_const(data_frame) < sizeof(*req) + req->len) {
			sky_err("malformed request\n");
			rc = -EINVAL;
			goto emergency;
		}
		if (req->len > sizeof(msg.buf)) {
			rc = -EOVERFLOW;
			goto emergency;
		}

		len = sizeof(*rsp);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}

		msg.len = req->len;
		memcpy(msg.buf, req->buf, req->len);

		rc = sky_sink_passthru_msgsend(dev, &msg);

		rsp->type  = htole16(req_type + 1);
		rsp->error = htole16(-rc);

		break;
	}
	case SKY_SINK_PASSTHRU_MSG_RECV_REQ: {
		struct sky_sink_passthru_msg_recv_rsp *rsp;
		struct sky_passthru_msg msg;

		dev = sky_find_dev(serv, devport_frame);
		if (dev == NULL) {
			rc = -ENODEV;
			goto emergency;
		}

		rc = sky_sink_passthru_msgrecv(dev, &msg);

		len = sizeof(*rsp) + (rc ? 0 : msg.len);
		rsp = rsp_void = calloc(1, len);
		if (!rsp) {
			rc = -ENOMEM;
			goto emergency;
		}
		if (!rc) {
			rsp->len = msg.len;
			memcpy(rsp->buf, msg.buf, msg.len);
		}

		rsp->hdr.type  = htole16(req_type + 1);
		rsp->hdr.error = htole16(-rc);

		break;
	}
	default:
		sky_err("unknown request: %d\n", req_type);
		rc = -EINVAL;
		goto emergency;
	}

	*rsp_hdr = rsp_void;
	*rsp_len = len;

	return;

emergency:
	emergency_rsp.error = htole16(-rc);
	if (req_type == SKY_UNKNOWN_REQRSP || req_type >= SKY_LAST_REQRSP)
		emergency_rsp.type = SKY_UNKNOWN_REQRSP;
	else
		emergency_rsp.type = htole16(req_type + 1);

	*rsp_hdr = &emergency_rsp;
	*rsp_len = sizeof(emergency_rsp);

	return;
}

static void sky_zocket_destroy(struct sky_server *serv)
{
	struct zocket *z = &serv->zock;

	if (z->router) {
		(void)zmq_close(z->router);
		z->router = NULL;
	}
	if (z->pub) {
		(void)zmq_close(z->pub);
		z->pub = NULL;
	}
	if (z->ctx) {
		/*
		 * If one of the zmq sockets has not been successfully
		 * connected we hang on ctx_term forever.  So fuck it!
		 * I mean that!  I tired struggling with zmq.
		 *
		 * (void)zmq_ctx_term(z->ctx);
		 */
		z->ctx = NULL;
	}
}

static int sky_zocket_create(struct sky_server *serv, const char *addr, int port)
{
	struct zocket *z = &serv->zock;
	char zaddr1[128], zaddr2[128];
	uint32_t opt;
	int rc;

	rc = snprintf(zaddr1, sizeof(zaddr1), "tcp://%s:%d", addr, port);
	if (rc < 0 || rc >= sizeof(zaddr1))
		return -EINVAL;

	rc = snprintf(zaddr2, sizeof(zaddr2), "tcp://%s:%d", addr, port+1);
	if (rc < 0 || rc >= sizeof(zaddr2))
		return -EINVAL;

	z->ctx = zmq_ctx_new();
	if (z->ctx == NULL)
		return -ENOMEM;

	z->router = zmq_socket(z->ctx, ZMQ_ROUTER);
	if (z->router == NULL) {
		rc = -ENOMEM;
		goto err;
	}
	z->pub = zmq_socket(z->ctx, ZMQ_PUB);
	if (z->pub == NULL) {
		rc = -ENOMEM;
		goto err;
	}

	opt = DEFAULT_TIMEOUT;
	rc = zmq_setsockopt(z->pub, ZMQ_SNDTIMEO, &opt, sizeof(opt));
	if (rc != 0) {
		rc = -errno;
		sky_err("zmq_setsockopt(): %s\n", strerror(-rc));
		goto err;
	}
	opt = DEFAULT_TIMEOUT;
	rc = zmq_setsockopt(z->router, ZMQ_SNDTIMEO, &opt, sizeof(opt));
	if (rc != 0) {
		rc = -errno;
		sky_err("zmq_setsockopt(): %s\n", strerror(-rc));
		goto err;
	}
	opt = 0;
	rc = zmq_setsockopt(z->pub, ZMQ_LINGER, &opt, sizeof(opt));
	if (rc != 0) {
		rc = -errno;
		sky_err("zmq_setsockopt(): %s\n", strerror(-rc));
		goto err;
	}
	opt = 0;
	rc = zmq_setsockopt(z->router, ZMQ_LINGER, &opt, sizeof(opt));
	if (rc != 0) {
		rc = -errno;
		sky_err("zmq_setsockopt(): %s\n", strerror(-rc));
		goto err;
	}
	rc = zmq_bind(z->router, zaddr1);
	if (rc != 0) {
		rc = -errno;
		sky_err("zmq_bind(): %s\n", strerror(-rc));
		goto err;
	}
	rc = zmq_bind(z->pub, zaddr2);
	if (rc != 0) {
		rc = -errno;
		sky_err("zmq_bind(): %s\n", strerror(-rc));
		goto err;
	}

	return 0;

err:
	sky_zocket_destroy(serv);

	return rc;
}

/**
 * sky_find_data_frame() - finds first data frame, skips all
 *                         IDENT and 0 frames.
 */
static inline zframe_t *sky_find_data_frame(zmsg_t *msg)
{
	zframe_t *data, *next;

	data = zmsg_first(msg);
	while (data) {
		next = zmsg_next(msg);
		if (!next || zframe_size(next) != 0) {
			zframe_t *pos;
			/*
			 * Replay from first till data, in order to
			 * set internal cursor correctly. Stupid zmsg.
			 */
			for (pos = zmsg_first(msg); pos != data;
			     pos = zmsg_next(msg))
				;
			/* Found data frame */
			break;
		}
		data = zmsg_next(msg);
	}

	return data;
}

struct pub_proxy {
	void *sub;
	void *push;
	pthread_t thr;
};

static void *thread_do_proxy(void *arg)
{
	struct pub_proxy *proxy = arg;

	(void)zmq_proxy(proxy->sub, proxy->push, NULL);

	return NULL;
}

static int sky_setup_and_proxy_pub(struct sky_server *serv,
				   struct sky_conf *conf,
				   struct pub_proxy *proxy)
{
	void *ctx = serv->zock.ctx;
	void *sub = NULL;
	void *push = NULL;
	char zaddr[128];
	int rc, opt;

	sub = zmq_socket(ctx, ZMQ_SUB);
	if (!sub) {
		sky_err("zmq_socket(ZMQ_SUB): No memory\n");
		rc = -ENOMEM;
		goto err;
	}
	opt = 0;
	rc = zmq_setsockopt(sub, ZMQ_LINGER, &opt, sizeof(opt));
	if (rc != 0) {
		rc = -errno;
		sky_err("zmq_setsockopt(ZMQ_LINGER): %s\n", strerror(-rc));
		goto err;
	}
	rc = zmq_setsockopt(sub, ZMQ_SUBSCRIBE, NULL, 0);
	if (rc) {
		rc = -errno;
		sky_err("zmq_setsockopt(ZMQ_SUBSCRIBE): %s\n", strerror(-rc));
		goto err;
	}
	snprintf(zaddr, sizeof(zaddr), "tcp://%s:%d",
		 serv->cli.addr, atoi(serv->cli.port)+1);
        rc = zmq_connect(sub, zaddr);
	if (rc) {
		rc = -errno;
		sky_err("zmq_connect(%s): %s\n", zaddr, strerror(-rc));
		goto err;
	}
	push = zmq_socket(ctx, ZMQ_PUSH);
	if (!push) {
		sky_err("zmq_socket(ZMQ_PUSH): No memory\n");
		rc = -ENOMEM;
		goto err;
	}
	opt = 0;
	rc = zmq_setsockopt(push, ZMQ_LINGER, &opt, sizeof(opt));
	if (rc != 0) {
		rc = -errno;
		sky_err("zmq_setsockopt(ZMQ_LINGER): %s\n", strerror(-rc));
		goto err;
	}
	snprintf(zaddr, sizeof(zaddr), "tcp://%s:%d", conf->hostname,
		 conf->subport);
	rc = zmq_connect(push, zaddr);
	if (rc) {
		rc = -errno;
		sky_err("zmq_connect(%s): %s\n", zaddr, strerror(-rc));
		goto err;
	}
	proxy->sub = sub;
	proxy->push = push;

	rc = pthread_create(&proxy->thr, NULL, thread_do_proxy, proxy);
	if (rc) {
		rc = -rc;
		sky_err("pthread_create(): %s\n", strerror(-rc));
		goto err;
	}

	return 0;

err:
	if (sub)
		zmq_close(sub);
	if (push)
		zmq_close(push);

	return rc;
}

static void sky_destroy_pub(struct pub_proxy *proxy)
{
	(void)sky_kill_pthread(proxy->thr);
	zmq_close(proxy->sub);
	zmq_close(proxy->push);
}

struct req_proxy {
	void *to_server;
	void *to_broker;
	void *broker_monitor;
};


/**
 * sky_zock_proxy() - simple variant of zmq_proxy() which
 *                    detects disconnects from monitor socket.
 *
 * For some reason (Oh God, I do not want to debug zmq crap)
 * setting ZMQ_RECONNECT_IVL to -1 does not help and socket
 * continues reconnecting.  What we need instead is to repeat
 * connection procedure if broker goes down.
 *
 * Also heartbeats here are supported.
 */
static int sky_zmq_proxy(struct req_proxy *p)
{
	unsigned long long expires_at;
	int rc;

	expires_at = zclock_time() +
		SKY_HEARTBEAT_IVL_MS * SKY_HEARTBEAT_CNT;

	while (true) {
		zmq_pollitem_t items [] = {
			{ p->broker_monitor, 0, ZMQ_POLLIN, 0 },
			{ p->to_broker,      0, ZMQ_POLLIN, 0 },
			{ p->to_server,      0, ZMQ_POLLIN, 0 },
		};
		void *dst, *src = NULL;
		zmsg_t *msg;

		rc = zmq_poll(items, 3,
			      SKY_HEARTBEAT_IVL_MS * ZMQ_POLL_MSEC);
		if (rc == -1)
			/* Interrupted */
			return -EINTR;

		if (expires_at <= zclock_time()) {
			/* Connection is dead */
			return -ECONNRESET;
		}
		if (items[0].revents & ZMQ_POLLIN) {
			/* Connection is dead */
			return -ECONNRESET;
		} else if (items[1].revents & ZMQ_POLLIN) {
			src = p->to_broker;
			dst = p->to_server;
		} else if (items[2].revents & ZMQ_POLLIN) {
			src = p->to_server;
			dst = p->to_broker;
		} else {
			/* Heartbeat destination */
			dst = p->to_broker;
		}

		if (src) {
			msg = zmsg_recv(src);
			if (!msg) {
				sky_err("Received NULL message\n");
				return -ECONNRESET;
			}
			if (src == p->to_broker) {
				zframe_t *frame;

				/* Refresh expiration */
				expires_at = zclock_time() +
					SKY_HEARTBEAT_IVL_MS * SKY_HEARTBEAT_CNT;

				/* DEALER prepends zero frame, remove it */
				frame = zmsg_first(msg);
				if (frame) {
					zmsg_remove(msg, frame);
					zframe_destroy(&frame);
				}
				/* Heartbeat message is empty, ignore */
				if (!zmsg_first(msg)) {
					zmsg_destroy(&msg);
					continue;
				}
			}
		} else {
			/* Create heartbeat */
			msg = zmsg_new();
			if (!msg)
				continue;
		}
		/* Prepend zero frame for DEALER */
		if (dst == p->to_broker)
			zmsg_pushmem(msg, NULL, 0);
		rc = zmsg_send(&msg, dst);
		if (rc) {
			sky_err("Failed to send a message\n");
			zmsg_destroy(&msg);
			return -ECONNRESET;
		}
	}

	return 0;
}

static int sky_send_first_req(struct sky_server *serv,
			      struct sky_conf *conf,
			      struct req_proxy *proxy)
{
	void *ctx = serv->zock.ctx;
	void *to_server = NULL;
	void *to_broker = NULL;
	void *monitor = NULL;
	zmsg_t *msg = NULL;
	char zaddr[128];
	void *rsp_void;
	size_t rsp_len;
	int rc, opt;

	to_server = zmq_socket(ctx, ZMQ_REQ);
	if (!to_server) {
		sky_err("zmq_socket(): No memory\n");
		rc = -ENOMEM;
		goto err;
	}
	opt = 0;
	rc = zmq_setsockopt(to_server, ZMQ_LINGER, &opt, sizeof(opt));
	if (rc != 0) {
		rc = -errno;
		sky_err("zmq_setsockopt(): %s\n", strerror(-rc));
		goto err;
	}
	snprintf(zaddr, sizeof(zaddr), "tcp://%s:%d",
		 serv->cli.addr, atoi(serv->cli.port));
	rc = zmq_connect(to_server, zaddr);
	if (rc) {
		rc = -errno;
		sky_err("zmq_connect(): %s\n", strerror(-rc));
		goto err;
	}
	/* Use DEALER instead of REQ to have heartbeats */
	to_broker = zmq_socket(ctx, ZMQ_DEALER);
	if (!to_broker) {
		rc = -errno;
		sky_err("zmq_socket(): No memory\n");
		goto err;
	}
	opt = 0;
	rc = zmq_setsockopt(to_broker, ZMQ_LINGER, &opt, sizeof(opt));
	if (rc != 0) {
		rc = -errno;
		sky_err("zmq_setsockopt(): %s\n", strerror(-rc));
		goto err;
	}
	rc = zmq_setsockopt(to_broker, ZMQ_IDENTITY, serv->conf.devuuid,
			    sizeof(serv->conf.devuuid));
	if (rc) {
		rc = -errno;
		sky_err("zmq_setsockopt(ZMQ_SUBSCRIBE): %s\n", strerror(-rc));
		goto err;
	}
	snprintf(zaddr, sizeof(zaddr), "tcp://%s:%d", conf->hostname,
		 conf->srvport);
	rc = zmq_connect(to_broker, zaddr);
	if (rc) {
		rc = -errno;
		sky_err("zmq_connect(): %s\n", strerror(-rc));
		goto err;
	}
	/*
	 * Also imply random number to avoid races in zmq when
	 * zmq_socket_monitor() returns EADDRINUSE if previous
	 * monitor socket has been just closed.
	 */
	snprintf(zaddr, sizeof(zaddr), "inproc://monitor-broker-%d-%ld",
		 getpid(), random());
	/* Mintor disconnect event to repeat discovery */
	rc = zmq_socket_monitor(to_broker, zaddr,
				ZMQ_EVENT_DISCONNECTED);
	if (rc) {
		rc = -errno;
		sky_err("zmq_socket_monitor(): %s\n", strerror(-rc));
		goto err;
	}
	monitor = zmq_socket(ctx, ZMQ_PAIR);
	if (!monitor) {
		sky_err("zmq_socket(): No memory\n");
		rc = -ENOMEM;
		goto err;
	}
	opt = 0;
	rc = zmq_setsockopt(monitor, ZMQ_LINGER, &opt, sizeof(opt));
	if (rc != 0) {
		rc = -errno;
		sky_err("zmq_setsockopt(): %s\n", strerror(-rc));
		goto err;
	}
	rc = zmq_connect(monitor, zaddr);
	if (rc) {
		rc = -errno;
		sky_err("zmq_connect(): %s\n", strerror(-rc));
		goto err;
	}
	rc = sky_devs_list_rsp(serv, serv->conf.devname,
			       SKY_PROTO_VERSION,
			       &rsp_void, &rsp_len);
	if (rc) {
		sky_err("sky_devs_list_rsp(): %s\n", strerror(-rc));
		goto err;
	}
	msg = zmsg_new();
	if (msg) {
		/* Prepend zero frame for DEALER to emulate REQ */
		rc = zmsg_addmem(msg, NULL, 0);
		/* Frame with actual data */
		rc |= zmsg_addmem(msg, rsp_void, rsp_len);
	}
	free(rsp_void);
	if (!msg || rc) {
		rc = -ENOMEM;
		goto err;
	}
	rc = zmsg_send(&msg, to_broker);
	if (rc != 0) {
		rc = -errno;
		sky_err("zmsg_send(): %s\n", strerror(-rc));
		goto err;
	}

	proxy->to_broker = to_broker;
	proxy->to_server = to_server;
	proxy->broker_monitor = monitor;

	return 0;

err:
	zmsg_destroy(&msg);
	if (to_server)
		zmq_close(to_server);
	if (to_broker)
		zmq_close(to_broker);
	if (monitor)
		zmq_close(monitor);

	return rc;
}

static void sky_destroy_req(struct req_proxy *proxy)
{
	zmq_close(proxy->to_server);
	zmq_close(proxy->to_broker);
	zmq_close(proxy->broker_monitor);
}

struct thread_params {
	pthread_t thread;
	struct sky_server *serv;
	int (*fillin_conf)(struct sky_server *serv, struct sky_conf *conf);
};

static int discover_broker(struct sky_server *serv, struct sky_conf *conf)
{
	struct sky_brokerinfo brokerinfo;
	int rc;

	/* Find any broker on the network */
	rc = sky_discoverbroker(&brokerinfo, SKY_DISCOVERY_MS);
	if (rc)
		/* Not a fatal error, so return > 0 */
		return 1;

	memset(conf, 0, sizeof(*conf));
	strncpy(conf->hostname, brokerinfo.addr,
		min(sizeof(brokerinfo.addr), sizeof(conf->hostname)));
	conf->srvport = brokerinfo.servers_port;
	conf->subport = brokerinfo.sub_port;

	return 0;
}

static int resolve_hostname(struct sky_conf *conf)
{
	struct addrinfo hints, *res, *rp;
	bool resolved = false;
	int rc;

	memset(&hints, 0, sizeof (hints));
	hints.ai_family   = AF_INET; /* IPv4 only */
	hints.ai_socktype = SOCK_STREAM;

	rc = getaddrinfo(conf->hostname, NULL, &hints, &res);
	if (rc) {
		sky_err_lim("getaddrinfo(%s): %s\n", conf->hostname,
			    gai_strerror(rc));
		return -ENODATA;
	}
	for (rp = res; rp != NULL; rp = rp->ai_next) {
		const void *ptr;

		switch (rp->ai_family)	{
		case AF_INET:
			ptr = &((struct sockaddr_in *) rp->ai_addr)->sin_addr;
			break;
		case AF_INET6:
			ptr = &((struct sockaddr_in6 *) rp->ai_addr)->sin6_addr;
			break;
		default:
			freeaddrinfo(res);
			return -ENODATA;
		}
		ptr = inet_ntop(rp->ai_family, ptr, conf->hostname,
				sizeof(conf->hostname));
		if (ptr)
			resolved = true;

		/* TODO: do we need to test others? */
		break;
	}
	freeaddrinfo(res);

	if (!resolved)
		return -ENODATA;

	return 0;
}

static int parse_conf(struct sky_server *serv, struct sky_conf *conf)
{
	if (serv->conf.hostname[0] == '\0') {
		/*
		 * Silently skip connection to the external broker if
		 * broker-url is explicitly disabled.
		 */
		return -ENODATA;
	}
	if (serv->conf.devname[0] == '\0') {
		sky_err("Invalid device-name\n");
		return -ENODATA;
	}
	if (uuid_is_null(serv->conf.devuuid)) {
		sky_err("Invalid device-uuid\n");
		return -ENODATA;
	}
	memcpy(conf, &serv->conf, sizeof(*conf));

	/*
	 * That is utterly important to do dns lookup from here,
	 * because if zmq does that, than if dns server does not
	 * respond the whole zmq loop is stuck and no any other
	 * IO is performed! Total disaster!
	 */
	return resolve_hostname(conf);
}

static void *sky_connect_to_broker(void *arg_)
{
	struct thread_params *p = arg_;
	struct pub_proxy pub_proxy = {}; /* Make gcc happy */
	struct req_proxy req_proxy = {}; /* Make gcc happy */
	struct sky_conf conf;
	int rc;

	/*
	 * Do discovery loop
	 */
	while (!p->serv->exit) {
		rc = p->fillin_conf(p->serv, &conf);
		if (rc < 0)
			break;
		if (rc)
			goto sleep_and_continue;

		rc = sky_setup_and_proxy_pub(p->serv, &conf, &pub_proxy);
		if (rc)
			goto sleep_and_continue;

		rc = sky_send_first_req(p->serv, &conf, &req_proxy);
		if (rc)
			goto destroy_pub;

		/* Main request proxying */
		(void)sky_zmq_proxy(&req_proxy);

		sky_destroy_req(&req_proxy);
destroy_pub:
		sky_destroy_pub(&pub_proxy);
sleep_and_continue:
		sleep(1);
	}

	return NULL;
}

static int sky_server_loop(struct sky_server *serv)
{
	struct thread_params p1 = {
		.serv = serv,
		.fillin_conf = discover_broker
	};
	struct thread_params p2 = {
		.serv = serv,
		.fillin_conf = parse_conf
	};

	zframe_t *data_frame, *devport_frame;
	struct sky_rsp_hdr *rsp;
	size_t rsp_len;
	zmsg_t *msg;
	int rc;

	rc = pthread_create(&p1.thread, NULL, sky_connect_to_broker, &p1);
	if (rc) {
		sky_err("pthread_create(discover_broker): %d\n", rc);
		p1.thread = 0;
	}
	rc = pthread_create(&p2.thread, NULL, sky_connect_to_broker, &p2);
	if (rc) {
		sky_err("pthread_create(parse_conf): %d\n", rc);
		p2.thread = 0;
	}

	while (1) {
		msg = zmsg_recv(serv->zock.router);
		if (msg == NULL) {
			/* Interrupted */
			rc = 0;
			break;
		}
		data_frame = sky_find_data_frame(msg);
		if (data_frame == NULL) {
			sky_err("zmsg_recv(): no data frame\n");
			rc = -EIO;
			break;
		}
		devport_frame = zmsg_next(msg);
		sky_execute_cmd(serv, devport_frame, data_frame, &rsp, &rsp_len);
		if (devport_frame) {
			/* Remove and destroy devport frame */
			zmsg_remove(msg, devport_frame);
			zframe_destroy(&devport_frame);
		}
		/* Replace data frame with response */
		zframe_reset(data_frame, rsp, rsp_len);
		rc = zmsg_send(&msg, serv->zock.router);
		if (rc != 0) {
			rc = -errno;
			sky_err("zmsg_send(): %s\n", strerror(-rc));
		}
		sky_free(rsp);
	}
	if (p1.thread || p2.thread) {
		/* Tear down threads */
		serv->exit = true;

		if (p1.thread)
			(void)sky_kill_pthread(p1.thread);
		if (p2.thread)
			(void)sky_kill_pthread(p2.thread);
	}

	return rc;
}

static int sky_avahi_publish_device(struct sky_server *serv,
				    struct sky_server_dev *servdev)
{
	char name[128];
	int rc;

	if (!serv->avahi)
		/* Ignore */
		return 0;

	rc = snprintf(name, sizeof(name), "Skycharge Device ");
	uuid_unparse(serv->conf.devuuid, name + rc);

	rc = avahi_publish_service(serv->avahi, name, "_skydevice._tcp",
				   NULL, NULL, atoi(serv->cli.port), NULL);
	if (rc)
		return -EINVAL;

	return 0;
}

static void sky_wait_for_mux_hw1_dev(struct sky_server *serv)
{
	bool printed = false;
	int rc;

	if (serv->conf.mux_type != SKY_MUX_HW1 ||
	    serv->conf.contype == SKY_DUMMY)
		/* Only HW1 MUX should be awaited */
		return;

again:
	rc = access(serv->conf.mux_dev, F_OK);
	if (rc) {
		if (!printed) {
			sky_err("Device for HW1 does not exist, will wait\n");
			printed = true;
		}
		usleep(300000);
		goto again;
	}
}

static int sky_hw1_psu_init(struct sky_server *serv)
{
	struct sky_conf *conf = &serv->conf;
	int rc;

	if (conf->psu.type == SKY_PSU_UNKNOWN)
		return 0;

	rc = sky_psu_init(conf, &serv->hw1_psu);
	if (rc)
		return rc;

	/* Set voltage */
	rc = sky_psu_set_voltage(&serv->hw1_psu,
				 conf->psu.voltage);
	if (rc) {
		sky_err("psu: can't set voltage");
		goto deinit_psu;
	}

	/* Set precharge current if specified */
	if (sky_psu_is_precharge_set(&serv->hw1_psu))
		rc = sky_psu_set_current(&serv->hw1_psu,
					 conf->psu.precharge_current);
	else
		rc = sky_psu_set_current(&serv->hw1_psu,
					 conf->psu.current);
	if (rc) {
		sky_err("psu: can't set current: %s", strerror(-rc));
		goto deinit_psu;
	}

	return 0;

deinit_psu:
	sky_psu_deinit(&serv->hw1_psu);

	return rc;
}

static void sky_hw1_psu_deinit(struct sky_server *serv)
{
	if (serv->conf.psu.type != SKY_PSU_UNKNOWN)
		sky_psu_deinit(&serv->hw1_psu);
}

static bool sky_hw2_mux_detect_mode_is_plc(struct sky_dev_params *params)
{
	return params->dev_params[SKY_HW2_DETECT_MODE] == SKY_DETECT_PLC;
}

static int sky_get_machine_app_specific(const uuid_t uuid, sd_id128_t *id)
{
	uint8_t crc;
	int i, rc;

	rc = sd_id128_get_machine(id);
        if (rc < 0)
                return rc;

	/* Pfff, whatever, just not to leak dev-uuid */
	crc = crc8(uuid, sizeof(uuid_t), 0);
	BUILD_BUG_ON(sizeof(uuid_t) != sizeof(*id));
	for (i = 0; i < sizeof(*id); i++) {
		id->bytes[i] ^= uuid[i] ^ crc;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	struct sky_server serv = {
		.conf = {
			.contype = SKY_LOCAL,
		}
	};
	struct sky_conf *conf = &serv.conf;
	struct sky_dev_desc *devdesc;
	int num, rc;

	rc = cli_parse(argc, argv, &serv.cli);
	if (rc) {
		sky_err("%s\n", cli_usage);
		return -1;
	}
	rc = sky_confparse(serv.cli.conff, conf);
	if (rc) {
		sky_err("sky_confparse(): %s\n", strerror(-rc));
		goto free_cli;
	}

	/* Get machine app specific ID based on dev-uuid */
	rc = sky_get_machine_app_specific(serv.conf.devuuid, &serv.id);
	if (rc) {
		sky_err("sky_get_machine_app_specific(): %s\n", strerror(-rc));
		goto free_cli;
	}

	/* PSU is controlled by the BB, only HW1 */
	if (conf->mux_type == SKY_MUX_HW1) {
		rc = sky_hw1_psu_init(&serv);
		if (rc)
			goto free_cli;
	} else if (conf->mux_type == SKY_MUX_HW2) {
		/* TODO */
	}

	/*
	 * Daemonize before creating zmq socket, ZMQ is written by
	 * people who are not able to deal with fork() gracefully.
	 */
	if (serv.cli.daemon) {
		sky_daemonize(serv.cli.pidf);

		/* Only daemon waits for HW1 MUX, because HW1 is a USB device */
		sky_wait_for_mux_hw1_dev(&serv);
	}

	/*
	 * Setup ZMQ in order to catch SIGINT (Ctrl-C) or SIGTERM signals.
	 */
	zsys_catch_interrupts();

	rc = avahi_init(&serv.avahi);
	if (rc) {
		/* Not fatal */
		sky_err("Can't create avahi: %s\n", strerror(-rc));
	}
	rc = sky_zocket_create(&serv, serv.cli.addr, atoi(serv.cli.port));
	if (rc) {
		sky_err("Can't create server sockets: %s\n", strerror(-rc));
		goto deinit_avahi;
	}
	rc = sky_devslist(conf, &serv.devhead);
	if (rc) {
		sky_err("sky_devslist(): %s\n", strerror(-rc));
		goto destroy_zocket;
	}
	/* Count devs */
	num = 0;
	foreach_devdesc(devdesc, serv.devhead)
		num++;

	/*
	 * Many local devices? Meh, impossible! Keep the old code with lists
	 * for the sake of compatibility.
	 */
	if (num > 1) {
		sky_err("We actually never supported many local devices!");
		rc = -EINVAL;
		goto free_devs;
	}

	serv.devs = calloc(num, sizeof(*serv.devs));
	if (serv.devs == NULL) {
		rc = -ENOMEM;
		goto free_devs;
	}
	num = 0;
	foreach_devdesc(devdesc, serv.devhead) {
		struct sky_subscription subsc = {
			.on_state = sky_on_charging_state,
			/*
			 * Before changing 1 sec rate consider precharge
			 * current updates, see get_precharge_current()
			 */
			.interval_msecs = 1000,
		};
		struct sky_server_dev *servdev;
		struct sky_dev_params *mux_params = NULL;
		struct sky_dev_params *sink_params = NULL;

		servdev = &serv.devs[num];
		servdev->serv = &serv;
		servdev->devdesc = devdesc;

		rc = sky_devopen(devdesc, &servdev->dev);
		if (rc) {
			sky_err("sky_devpopen(): %s\n", strerror(-rc));
			goto close_devs;
		}

		if (conf->mux_type == SKY_MUX_HW1 &&
		    conf->mux_hw1_params.dev_params_bits) {
			mux_params = &conf->mux_hw1_params;
		} else if (conf->mux_type == SKY_MUX_HW2 &&
			   conf->mux_hw2_params.dev_params_bits) {
			mux_params = &conf->mux_hw2_params;
			sink_params = &conf->sink_params;
		}

		if (mux_params && mux_params->dev_params_bits) {
			/* Apply MUX params */
			rc = sky_paramsset(servdev->dev, mux_params);
			if (rc) {
				sky_err("sky_paramsset(): %s\n", strerror(-rc));
				sky_devclose(servdev->dev);
				servdev->dev = NULL;
				goto close_devs;
			}
		}
		if (sink_params && sink_params->dev_params_bits) {
			struct sky_dev_params params;

			/* Fetch all MUX params */
			params.dev_params_bits = ~0;
			rc = sky_paramsget(servdev->dev, &params);
			if (rc) {
				sky_err("sky_paramsget(): %s\n", strerror(-rc));
				sky_devclose(servdev->dev);
				servdev->dev = NULL;
				goto close_devs;
			}

			/* Apply sink params for detect modes other than PLC */
			if (!sky_hw2_mux_detect_mode_is_plc(&params)) {
				rc = sky_sink_paramsset(servdev->dev, sink_params);
				if (rc) {
					sky_err("sky_sink_paramsset(): %s\n", strerror(-rc));
					sky_devclose(servdev->dev);
					servdev->dev = NULL;
					goto close_devs;
				}
			}
		}
		subsc.data = servdev;
		rc = sky_subscribe(servdev->dev, NULL, &subsc);
		if (rc) {
			sky_err("sky_subscribe(): %s\n", strerror(-rc));
			sky_devclose(servdev->dev);
			servdev->dev = NULL;
			goto close_devs;
		}
		rc = sky_avahi_publish_device(&serv, servdev);
		if (rc) {
			sky_err("sky_avahi_publish_device(): %s\n", strerror(-rc));
			sky_devclose(servdev->dev);
			servdev->dev = NULL;
			goto close_devs;
		}
		num++;
	}
	rc = sky_server_loop(&serv);

close_devs:
	num = 0;
	foreach_devdesc(devdesc, serv.devhead) {
		struct sky_server_dev *servdev = &serv.devs[num++];

		if (servdev->dev == NULL)
			continue;
		sky_unsubscribe(servdev->dev);
		sky_devclose(servdev->dev);
	}
free_devs:
	sky_devsfree(serv.devhead);
	free(serv.devs);
destroy_zocket:
	sky_zocket_destroy(&serv);
deinit_avahi:
	avahi_deinit(serv.avahi);
	if (conf->mux_type == SKY_MUX_HW1)
		sky_hw1_psu_deinit(&serv);
free_cli:
	cli_free(&serv.cli);

	return rc;
}

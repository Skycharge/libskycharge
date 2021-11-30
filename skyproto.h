/*
 * Copyright 2021 Skycharge GmbH
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

#ifndef SKYPROTO_H
#define SKYPROTO_H

#include "types.h"

/**
 * enum sky_proto_type - skycharge protocol request/responses
 *
 * BEWARE:
 *    1. keep requests/responses sequential, one by one.
 *    2. requests are odd, responses are even.
 *    3. events start from 128.
 */
enum sky_proto_type {

	SKY_UNKNOWN_REQRSP     = 0,

	/*
	 * Device requests/responses.
	 * Format of requests is the following:
	 *          REQ
	 *          DEVPORT (taken from from @sky_dev_info.portname)
	 *          DEVUUID (taken from from @sky_dev_info.dev_uuid)
	 *
	 * That is ugly but that simplifies a lot skybroker.
	 */

	SKY_GET_DEV_PARAMS_REQ = 1,
	SKY_GET_DEV_PARAMS_RSP = 2,

	SKY_SET_DEV_PARAMS_REQ = 3,
	SKY_SET_DEV_PARAMS_RSP = 4,

	SKY_RESUME_SCAN_REQ    = 5,
	SKY_RESUME_SCAN_RSP    = 6,

	SKY_STOP_SCAN_REQ      = 7,
	SKY_STOP_SCAN_RSP      = 8,

	SKY_OPEN_DRONEPORT_REQ = 9,
	SKY_OPEN_DRONEPORT_RSP = 10,

	SKY_CLOSE_DRONEPORT_REQ = 11,
	SKY_CLOSE_DRONEPORT_RSP = 12,

	SKY_CHARGING_STATE_REQ = 13,
	SKY_CHARGING_STATE_RSP = 14,

	SKY_RESET_DEV_REQ      = 15,
	SKY_RESET_DEV_RSP      = 16,

	/*
	 * Format of the request is the following:
	 *          REQ
	 *          USRUUID (taken from from @sky_dev_conf.remote.usr_uuid)
	 */
	SKY_DEVS_LIST_REQ      = 17,
	SKY_DEVS_LIST_RSP      = 18,

	/*
	 * Format of the request is the following:
	 *          REQ
	 *          USRUUID (taken from from @sky_dev_conf.remote.usr_uuid)
	 */
	SKY_PEERINFO_REQ       = 19,
	SKY_PEERINFO_RSP       = 20,

	/*
	 * Normal device requests/responses.  See format above
	 */
	SKY_GPSDATA_REQ        = 21,
	SKY_GPSDATA_RSP        = 22,

	SKY_DRONEDETECT_REQ    = 23,
	SKY_DRONEDETECT_RSP    = 24,

	SKY_DRONEPORT_STATE_REQ = 25,
	SKY_DRONEPORT_STATE_RSP = 26,

	SKY_SINK_GET_INFO_REQ = 27,
	SKY_SINK_GET_INFO_RSP = 28,

	SKY_SINK_GET_DEV_PARAMS_REQ = 29,
	SKY_SINK_GET_DEV_PARAMS_RSP = 30,

	SKY_SINK_SET_DEV_PARAMS_REQ = 31,
	SKY_SINK_SET_DEV_PARAMS_RSP = 32,

	SKY_SINK_START_CHARGE_REQ = 33,
	SKY_SINK_START_CHARGE_RSP = 34,

	SKY_SINK_STOP_CHARGE_REQ = 35,
	SKY_SINK_STOP_CHARGE_RSP = 36,

	SKY_LAST_REQRSP,

	/* Events */

	SKY_CHARGING_STATE_EV  = 128
};

enum {
	DEFAULT_TIMEOUT = 30000,
	PORTNAME_LEN = 32,

	SKY_PROTO_VERSION = 0x0400,

	SKY_HEARTBEAT_IVL_MS = 5000,
	SKY_HEARTBEAT_CNT    = 3,

	SKY_DISCOVERY_PORT = 6666,
	SKY_DISCOVERY_MS   = 500, /* wait if no broadcasting */
};

#define	SKY_DISCOVERY_MAGIC "SKYBROCKER"

#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-Wpadded"

struct sky_discovery {
	char magic[16];
	le16 proto_version;
	le16 servers_port;
	le16 sub_port;
	le16 clients_port;
	le16 pub_port;
	char padding[18];
};

struct sky_req_hdr {
	le16 type;
	le16 proto_version; /*
			     * Protocol version before 0x0400 does not have
			     * `proto_version`, so for versions below this
			     * member is equal to 0.
			     */
};

struct sky_rsp_hdr {
	le16 type;
	le16 error;
};

struct sky_get_dev_params_req {
	struct sky_req_hdr hdr;
	le32 dev_params_bits;
};

struct sky_get_dev_params_rsp {
	struct sky_rsp_hdr hdr;
	le32 dev_params[];
};

struct sky_set_dev_params_req {
	struct sky_req_hdr hdr;
	le32 dev_params_bits;
	le32 dev_params[];
};

/*
 * Union of all possible requests for allocating the structure on a stack
 */
union sky_req {
	struct sky_req_hdr            hdr;
	struct sky_get_dev_params_req get_dev_params;
	struct sky_set_dev_params_req set_dev_params;
};

struct sky_dev_info {
	le16          dev_type;
	le16          info_len; /*
				 * Protocol version before 0x0400 does not have
				 * `info_len`, so for versions below this member
				 * is equal to 0.
				 */
	le32          fw_version;
	char          dev_name[16];
	unsigned char dev_uuid[16];
	char          portname[PORTNAME_LEN];

	/* The members below were added for the protocol version 0x0400 */

	le16         proto_version;
	le16         padding;
	le32         hw_version;
	le32         plc_proto_version;
	struct {
		le32 part1;
		le32 part2;
		le32 part3;
	}            hw_uid;
};

struct sky_sink_get_info_rsp {
	struct sky_rsp_hdr   hdr;
	le32         fw_version;
	le32         hw_version;
	le32         plc_proto_version;
	struct {
		le32 part1;
		le32 part2;
		le32 part3;
	}            hw_uid;
};

struct sky_devs_list_rsp {
	struct sky_rsp_hdr hdr;
	le16 num_devs;
	le16 info_off; /*
			* Protocol version before 0x0400 does not have `info_off`,
			* so for old protocol versions this member is equal to 0.
			* New members should be added strictly after `info_off`.
			*/

	struct sky_dev_info info[]; /*
				     * This can't be accessed as an array,
				     * in protocol version 0x400 `info->info_len`
				     * has to be taken into consideration.
				     */
};

struct sky_charging_state_rsp {
	struct sky_rsp_hdr hdr;
	le16 voltage_mV;
	le16 current_mA;
	le16 dev_hw_state;
	le16 state_of_charge; /* 0-100% */
	le16 until_full_secs;
	le16 charging_secs;
	le16 mux_temperature_C;
	le16 sink_temperature_C;
	le32 energy_mWh;
	le32 charge_mAh;
	uint8_t mux_humidity_perc; /* 0-100% */
	uint8_t link_quality_factor; /* 0-100% */
	le16  padding;
	struct {
		le32 bytes;
		le32 packets;
		le32 err_bytes;
		le32 err_packets;
	} tx;
	struct {
		le32 bytes;
		le32 packets;
		le32 err_bytes;
		le32 err_packets;
	} rx;

	le32 charging_session_id;  /* Random ID for each charging session */
};

struct sky_peerinfo_rsp {
	struct sky_rsp_hdr hdr;
	le16 proto_version;
	le16 padding;
	le32 server_version;
	char reserved[52];
};

struct sky_droneport_state_rsp {
	struct sky_rsp_hdr hdr;
	le32 status;
};

struct sky_gpsdata_rsp {
	struct sky_rsp_hdr hdr;
	le32 padding1;

	/* Dilution of precision factors */
	struct {
		le64 xdop, ydop, pdop, hdop, vdop, tdop, gdop;
	} dop;

	struct {
		le32 mode; /* Mode of fix */
		le32 padding1;

		le64 time; /* Time of update, unix time in sec with fractional part */
		le64 latitude;  /* Latitude in degrees (valid if mode >= 2) */
		le64 longitude; /* Longitude in degrees (valid if mode >= 2) */
		le64 altitude;  /* Altitude in meters (valid if mode == 3) */

		le32 padding2[8];
	} fix;

	le32 status;          /* GPS status -- always valid */
	le32 satellites_used; /* Number of satellites used in solution */
	le32 padding2[16];
};

struct sky_dronedetect_rsp {
	struct sky_rsp_hdr hdr;
	le16 status;
};

#pragma GCC diagnostic pop

#endif /* SKYPROTO_H */

#ifndef SKYPROTO_H
#define SKYPROTO_H

#include "types.h"

/**
 * enum sky_proto_type - skysense protocol request/responses
 *
 * BEWARE:
 *    1. keep requests/responses sequential, one by one.
 *    2. requests are odd, responses are even.
 *    3. events start from 128.
 */
enum sky_proto_type {

	SKY_UNKNOWN_REQRSP     = 0,

	/*
	 * Device requests/responses.  Format of requests
	 * is the following:
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

	SKY_START_CHARGE_REQ   = 5,
	SKY_START_CHARGE_RSP   = 6,

	SKY_STOP_CHARGE_REQ    = 7,
	SKY_STOP_CHARGE_RSP    = 8,

	SKY_OPEN_COVER_REQ     = 9,
	SKY_OPEN_COVER_RSP     = 10,

	SKY_CLOSE_COVER_REQ    = 11,
	SKY_CLOSE_COVER_RSP    = 12,

	SKY_CHARGING_STATE_REQ = 13,
	SKY_CHARGING_STATE_RSP = 14,

	SKY_RESET_DEV_REQ      = 15,
	SKY_RESET_DEV_RSP      = 16,

	/*
	 * Format of the request is the following:
	 *          REQ
	 */
	SKY_DEVS_LIST_REQ      = 17,
	SKY_DEVS_LIST_RSP      = 18,

	/*
	 * Format of the request is the following:
	 *          REQ
	 */
	SKY_PEERINFO_REQ       = 19,
	SKY_PEERINFO_RSP       = 20,

	/*
	 * Normal device requests/responses.  See format above
	 */
	SKY_GPSDATA_REQ        = 21,
	SKY_GPSDATA_RSP        = 22,

	SKY_LAST_REQRSP,

	/* Events */

	SKY_CHARGING_STATE_EV  = 128
};

enum {
	DEFAULT_TIMEOUT = 30000,
	PORTNAME_LEN = 32,

	SKY_PROTO_VERSION = 0x0200,

	SKY_HEARTBEAT_IVL_MS = 5000,

	SKY_DISCOVERY_PORT = 6666,
	SKY_DISCOVERY_MS   = 500, /* wait if no broadcasting */
};

#define	SKY_DISCOVERY_MAGIC "SKYSENSE-BROCKER"

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
};

struct sky_rsp_hdr {
	le16 type;
	le16 error;
};

struct sky_get_dev_params_req {
	struct sky_req_hdr hdr;
	le16 padding;
	le32 dev_params_bits;
};

struct sky_get_dev_params_rsp {
	struct sky_rsp_hdr hdr;
	le32 dev_params[];
};

struct sky_set_dev_params_req {
	struct sky_req_hdr hdr;
	le16 padding;
	le32 dev_params_bits;
	le32 dev_params[];
};

struct sky_dev_info {
	le16 dev_type;
	le16 padding;
	le32 firmware_version;
	char dev_name[16];
	unsigned char dev_uuid[16];
	char portname[PORTNAME_LEN];
};

struct sky_devs_list_rsp {
	struct sky_rsp_hdr hdr;
	le16 num_devs;
	le16 padding;
	struct sky_dev_info info[];
};

struct sky_charging_state_rsp {
	struct sky_rsp_hdr hdr;
	le16 voltage;  /* mV */
	le16 current;  /* mA */
	le16 dev_hw_state;
	le16 unused;
};

struct sky_peerinfo_rsp {
	struct sky_rsp_hdr hdr;
	le16 proto_version;
	le16 padding;
	le32 server_version;
	char reserved[52];
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

#pragma GCC diagnostic pop

#define sky_err(fmt, ...) \
	fprintf(stderr, __FILE__ ":%s():" stringify(__LINE__) ": " fmt, \
		__func__, ##__VA_ARGS__)

#endif /* SKYPROTO_H */

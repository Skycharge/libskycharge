#ifndef LIBSKYCHARGE_H
#define LIBSKYCHARGE_H

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <netinet/in.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * struct sky_dev - Opaque device context.
 */
struct sky_dev;

/**
 * struct sky_async - Opaque async context.
 */
struct sky_async;

/**
 * enum sky_con_type - Connection type.
 */
enum sky_con_type {
	SKY_LOCAL  = 0,
	SKY_REMOTE = 1,
	SKY_DUMMY  = 2,
};

/**
 * enum sky_dev_type - Device type.
 */
enum sky_dev_type {
	SKY_MUX_HW1 = 0,
	SKY_MUX_HW2 = 1,
};

/**
 * enum sky_batt_type - Battery type.
 */
enum sky_batt_type {
	SKY_BATT_LIPO = 0,
	SKY_BATT_LION,
};

/**
 * enum sky_param_value_format - Specifies how parameter value should
 *                               be reprensented.
 */
enum sky_param_value_format {
	SKY_PARAM_VALUE_TEXT = 0,
	SKY_PARAM_VALUE_TEXT_AND_NUMERIC,
};

/**
 * enum sky_detect_mode - Detect mode.
 */
enum sky_detect_mode {
	SKY_DETECT_PLC        = 0,
	SKY_DETECT_RESISTANCE,
	SKY_DETECT_CAPACITY,
};

/**
 * enum sky_dev_state - Hardware state of the device.
 */
enum sky_dev_state {
	/* HW1 */
	SKY_HW1_UNKNOWN				= 0,
	SKY_HW1_SCANNING_INIT			= 1,
	SKY_HW1_SCANNING_RUN			= 2,
	SKY_HW1_SCANNING_CHECK_MATRIX		= 3,
	SKY_HW1_SCANNING_PRINT			= 4,
	SKY_HW1_SCANNING_CHECK_WATER		= 5,
	SKY_HW1_SCANNING_WET		        = 6,
	SKY_HW1_SCANNING_DETECTING		= 7,
	SKY_HW1_PRE_CHARGING_INIT		= 8,
	SKY_HW1_PRE_CHARGING_RUN		= 9,
	SKY_HW1_PRE_CHARGING_CHECK_MATRIX	= 10,
	SKY_HW1_PRE_CHARGING_PRINT		= 11,
	SKY_HW1_PRE_CHARGING_CHECK_WATER	= 12,
	SKY_HW1_PRE_CHARGING_WET		= 13,
	SKY_HW1_PRE_CHARGING_FIND_CHARGERS	= 14,
	SKY_HW1_CHARGING_INIT			= 15,
	SKY_HW1_CHARGING_RUN			= 16,
	SKY_HW1_CHARGING_MONITOR_CURRENT	= 17,
	SKY_HW1_POST_CHARGING_INIT		= 18,
	SKY_HW1_POST_CHARGING_RUN		= 19,
	SKY_HW1_POST_CHARGING_CHECK_MATRIX	= 20,
	SKY_HW1_POST_CHARGING_PRINT		= 21,
	SKY_HW1_POST_CHARGING_CHECK_WATER	= 22,
	SKY_HW1_POST_CHARGING_WET		= 23,
	SKY_HW1_POST_CHARGING_FIND_CHARGERS	= 24,
	SKY_HW1_OVERLOAD			= 25,
	SKY_HW1_AUTOSCAN_DISABLED		= 250,

	/* HW2 */
	SKY_HW2_STOPPED				= 0x00,
	SKY_HW2_SCANNING			= 0x01,
	SKY_HW2_LINK_ESTABLISHED		= 0x02,
	SKY_HW2_PRECHARGE_DELAYED		= 0x03,
	SKY_HW2_PRECHARGING			= 0x04,
	SKY_HW2_CHARGING			= 0x05,
	SKY_HW2_CHARGING_FINISHED		= 0x06,

	/* HW2 error states */
	SKY_HW2_ERR_INVAL_CHARGING_SETTINGS	= 0x80,
	SKY_HW2_ERR_BAD_LINK			= 0x81,
	SKY_HW2_ERR_LOW_BATT_VOLTAGE		= 0x82,
	SKY_HW2_ERR_LOW_MUX_VOLTAGE		= 0x83,
	SKY_HW2_ERR_VOLTAGE_ON_OUTPUT		= 0x84,
};

/**
 * enum sky_dev_param - Configuration MUX device parameter.
 */
enum sky_dev_param {
	/* HW1 */
	SKY_HW1_EEPROM_INITED	         = 0,
	SKY_HW1_SCANNING_INTERVAL	 = 1,
	SKY_HW1_PRECHARGING_INTERVAL     = 2,
	SKY_HW1_PRECHARGING_COUNTER	 = 3,
	SKY_HW1_POSTCHARGING_INTERVAL    = 4,
	SKY_HW1_POSTCHARGING_DELAY	 = 5,
	SKY_HW1_WET_DELAY		 = 6,
	SKY_HW1_SHORTCIRC_DELAY	         = 7,
	SKY_HW1_THRESH_FINISH_CHARGING   = 8,
	SKY_HW1_THRESH_NOCHARGER_PRESENT = 9,
	SKY_HW1_THRESH_SHORTCIRC         = 10,
	SKY_HW1_CURRENT_MON_INTERVAL     = 11,
	SKY_HW1_WAIT_START_CHARGING_SEC  = 12,

	SKY_HW1_NUM_DEVPARAM, /* Should be the last for HW1 */

	/* HW2 */
	SKY_HW2_PSU_TYPE                         = 0,
	SKY_HW2_DETECT_MODE                      = 1,
	SKY_HW2_PSU_FIXED_VOLTAGE_MV             = 2,
	SKY_HW2_PSU_FIXED_CURRENT_MA             = 3,
	SKY_HW2_USE_FIXED_V_I                    = 4,
	SKY_HW2_NR_BAD_HEARTBEATS                = 5,
	SKY_HW2_IGNORE_INVAL_CHARGING_SETTINGS   = 6,
	SKY_HW2_IGNORE_LOW_BATT_VOLTAGE          = 7,
	SKY_HW2_ERROR_INDICATION_TIMEOUT_SECS    = 8,
	SKY_HW2_KEEP_SILENCE                     = 9,
	SKY_HW2_IGNORE_VOLTAGE_ON_OUTPUT         = 10,
	SKY_HW2_MIN_SENSE_CURRENT_MA             = 11,
	SKY_HW2_REPEAT_CHARGE_AFTER_MINS         = 12,
	SKY_HW2_SENSE_VOLTAGE_CALIB_POINT1_MV    = 13,
	SKY_HW2_SENSE_VOLTAGE_CALIB_POINT2_MV    = 14,
	SKY_HW2_SENSE_CURRENT_CALIB_POINT1_MA    = 15,
	SKY_HW2_SENSE_CURRENT_CALIB_POINT2_MA    = 16,
	SKY_HW2_PSU_VOLTAGE_CALIB_POINT1_MV      = 17,
	SKY_HW2_PSU_VOLTAGE_CALIB_POINT2_MV      = 18,
	SKY_HW2_PSU_CURRENT_CALIB_POINT1_MA      = 19,
	SKY_HW2_PSU_CURRENT_CALIB_POINT2_MA      = 20,

	SKY_HW2_NUM_DEVPARAM, /* Should be the last for HW2 */
};

/**
 * enum sky_sink_capabilities - Capabilties bits of the sink
 *     @SKY_SINK_CAPABILITIES param
 */
enum sky_sink_capabilities {
	SKY_CAP_PLC_WHILE_CHARGING   = 0,
};

/**
 * enum sky_sink_param - Configuration sink device parameter.
 */
enum sky_sink_param {
	SKY_SINK_CAPABILITIES                     = 0,
	SKY_SINK_BATT_TYPE                        = 1,
	SKY_SINK_BATT_CAPACITY_MAH                = 2,
	SKY_SINK_BATT_MIN_VOLTAGE_MV              = 3,
	SKY_SINK_BATT_MAX_VOLTAGE_MV              = 4,
	SKY_SINK_CHARGING_MAX_CURRENT_MA          = 5,
	SKY_SINK_CUTOFF_MIN_CURRENT_MA            = 6,
	SKY_SINK_CUTOFF_TIMEOUT_MS                = 7,
	SKY_SINK_PRECHARGE_CURRENT_COEF           = 8,
	SKY_SINK_PRECHARGE_DELAY_SECS             = 9,
	SKY_SINK_PRECHARGE_SECS                   = 10,
	SKY_SINK_TOTAL_CHARGE_SECS                = 11,
	SKY_SINK_USER_DATA1                       = 12,
	SKY_SINK_USER_DATA2                       = 13,
	SKY_SINK_USER_DATA3                       = 14,
	SKY_SINK_USER_DATA4                       = 15,

	SKY_SINK_NUM_DEVPARAM, /* Should be the last for sink */
};

/**
 * struct sky_dev_params - Device configuration parameters.
 */
struct sky_dev_params {
	uint32_t dev_params_bits;
	uint32_t dev_params[32];
};

/**
 * enum sky_drone_status - Drone status on charging pad.
 */
enum sky_drone_status {
	SKY_DRONE_NOT_DETECTED = 0,
	SKY_DRONE_DETECTED     = 1,
};

/**
 * enum sky_psu_type - Power supply unit type.
 */
enum sky_psu_type {
	SKY_PSU_UNKNOWN,
	SKY_PSU_RSP_750_48,
	SKY_PSU_RSP_1600_48
};

/**
 * enum sky_dp_hw_interface - HW interface between BBone and DP
 */
enum sky_dp_hw_interface {
	SKY_DP_UNKNOWN = 0,
	SKY_DP_GPIO,
};

/**
 * struct sky_conf - Generic configuration options.
 */
struct sky_conf {
	uint8_t  usruuid[16];
	uint8_t  devuuid[16];
	char     devname[16];
	char     hostname[64];
	unsigned srvport;      /**< Servers command port */
	unsigned subport;      /**< Servers subscribe port */
	unsigned cliport;      /**< Clients command port */
	unsigned pubport;      /**< Clients publish port */

	enum sky_con_type     contype;

	enum sky_dev_type     mux_type;
	char                  mux_dev[32];
	struct sky_dev_params mux_hw1_params;
	struct sky_dev_params mux_hw2_params;
	struct sky_dev_params sink_params;

	struct {
		enum sky_psu_type type;
		float    voltage;
		float    current;
		float    precharge_current;
		float    precharge_current_coef;
		unsigned precharge_secs;
	} psu;

	struct {
		enum sky_dp_hw_interface hw_interface;
		struct {
			/* Before changing the size check parse function */
			char open_pin[8];
			char close_pin[8];
			char is_opened_pin[8];
			char is_closed_pin[8];
			char in_progress_pin[8];
			char is_landing_err_pin[8];
			char is_ready_pin[8];
			char is_drone_detected_pin[8];
		} gpio;
	} dp;
};

struct sky_dev_ops;
struct sky_hw_ops;

/**
 * struct sky_hw_uid - HW unique id
 */
struct sky_hw_uid {
	uint32_t part1;
	uint32_t part2;
	uint32_t part3;
};

/**
 * struct sky_hw_info - HW information
 */
struct sky_hw_info {
	uint32_t          fw_version;
	uint32_t          hw_version;
	uint32_t          plc_proto_version;
	struct sky_hw_uid uid;
};

/**
 * struct sky_dev_desc - Device descriptor.
 */
struct sky_dev_desc {
	struct sky_dev_desc      *next;
	struct sky_conf          conf;
	enum sky_dev_type        dev_type;
	const struct sky_dev_ops *dev_ops;
	const struct sky_hw_ops  *hw_ops;
	char                     dev_name[16];
	unsigned char            dev_uuid[16];
	char                     portname[32];
	uint16_t                 proto_version;
	struct sky_hw_info       hw_info;
};

/**
 * struct sky_sink_info - Sink device information
 */
struct sky_sink_info {
	struct sky_hw_info hw_info;
};

#define make_version(min, maj, rev) (((maj) & 0xff) << 16 | ((min) & 0xff) << 8 | ((rev) & 0xff))
#define version_major(v) (((v) >> 16) & 0xff)
#define version_minor(v) (((v) >> 8) & 0xff)
#define version_revis(v) ((v) & 0xff)

#define foreach_devdesc(devdesc, head)		\
	for (devdesc = head; devdesc; devdesc = devdesc->next)

/**
 * struct sky_charging_state - Charging device state.
 */
struct sky_charging_state {
	uint16_t dev_hw_state; /* enum sky_dev_state */
	uint16_t voltage_mV;
	uint16_t current_mA;
	uint16_t state_of_charge; /* 0-100% */
	uint16_t until_full_secs;
	uint16_t charging_secs;
	int16_t  mux_temperature_C;
	int16_t  sink_temperature_C;
	uint32_t energy_mWh;
	uint32_t charge_mAh;
	uint8_t  mux_humidity_perc; /* 0-100% */
	uint8_t  link_quality_factor; /* 0-100% */
	uint16_t padding;
	struct {
		uint32_t bytes;
		uint32_t packets;
		uint32_t err_bytes;
		uint32_t err_packets;
	} tx;
	struct {
		uint32_t bytes;
		uint32_t packets;
		uint32_t err_bytes;
		uint32_t err_packets;
	} rx;
};

enum sky_droneport_status {
	SKY_DP_IS_READY      = 1<<0,
	SKY_DP_IS_OPENED     = 1<<1,
	SKY_DP_IS_CLOSED     = 1<<2,
	SKY_DP_IN_PROGRESS   = 1<<3,
	SKY_DP_LANDING_ERROR = 1<<4,
};

/**
 * struct sky_droneport_state - Droneport state.
 */
struct sky_droneport_state {
	unsigned int status;
};

/**
 * struct sky_subscription - Event subscription configuration.
 */
struct sky_subscription {
	void *data;
	void (*on_state)(void *data, struct sky_charging_state *state);
	unsigned interval_msecs;
};

/**
 * struct sky_peerinfo - Peer information.
 */
struct sky_peerinfo {
	uint32_t server_version;
	uint16_t proto_version;
};

/**
 * struct sky_brokerinfo - Broker information.
 */
struct sky_brokerinfo {
	uint16_t af_family;
	char addr[INET6_ADDRSTRLEN];
	uint16_t proto_version;
	uint16_t servers_port;
	uint16_t sub_port;
	uint16_t clients_port;
	uint16_t pub_port;
};

enum sky_gps_status {
	SKY_GPS_STATUS_NO_FIX   = 0,  /* no */
	SKY_GPS_STATUS_FIX      = 1,  /* yes, without DGPS */
	SKY_GPS_STATUS_DGPS_FIX = 2,  /* yes, with DGPS */
};

enum sky_gps_mode {
	SKY_GPS_MODE_NOT_SEEN = 0,  /* mode update not seen yet */
	SKY_GPS_MODE_NO_FIX   = 1,  /* none */
	SKY_GPS_MODE_2D       = 2,  /* good for latitude/longitude */
	SKY_GPS_MODE_3D       = 3   /* good for altitude/climb too */
};

/**
 * struct sky_gpsdata - GPS data
 */
struct sky_gpsdata {
	enum sky_gps_status status; /* GPS status -- always valid */
	unsigned satellites_used;   /* Number of satellites used in solution */

	/* Dilution of precision factors */
	struct {
		double xdop, ydop, pdop, hdop, vdop, tdop, gdop;
	} dop;

	struct {
		enum sky_gps_mode mode; /* Mode of fix */

		double time; /* Time of update, unix time in sec with fractional part */
		double latitude;  /* Latitude in degrees (valid if mode >= 2) */
		double longitude; /* Longitude in degrees (valid if mode >= 2) */
		double altitude;  /* Altitude in meters (valid if mode == 3) */
	} fix;
};

/**
 * union sky_async_storage - Storage for keeping all possible results structures
 *                           of asynchronous commands.
 */
union sky_async_storage {
	struct sky_peerinfo        peerinfo;
	struct sky_dev_desc        *devdescs;
	struct sky_dev_params      params;
	struct sky_charging_state  charging_state;
	struct sky_droneport_state dp_state;
	struct sky_gpsdata         gpsdata;
	enum sky_drone_status      drone_status;
	struct sky_sink_info       sink_info;
};

struct sky_async_req;

/**
 * Completion of a response of the asynchronous request.
 */
typedef void (sky_async_completion_t)(struct sky_async_req *req);

/**
 * struct sky_async_req - Request for asynchronous commands.
 */
struct sky_async_req {
	struct sky_async_req   *next;
	struct sky_async_req   *prev;
	sky_async_completion_t *completion;
	struct sky_dev         *dev;
	const uint8_t          *usruuid;
	unsigned int           type;
	unsigned int           tag;  /* response tag */
	struct {
		const void *ptr;
	}                    in;
	struct {
		void   *ptr;
		bool   completed;
		int    rc;
	}                    out;
};

/**
 * Returns string representation of the device type.
 */
static inline const char *sky_devtype_to_str(enum sky_dev_type type)
{
	return type == SKY_MUX_HW1 ? "MUX-HW1": "MUX-HW2";
}

/**
 * Returns string representation of the device state.
 */
const char *sky_devstate_to_str(enum sky_dev_type dev_type,
				enum sky_dev_state state);

/**
 * Returns string representation of the device param.
 */
const char *sky_devparam_to_str(enum sky_dev_type dev_type,
				enum sky_dev_param param);

/**
 * Returns device param if string is recognized. If string can't
 * be parsed number of device params is returned.
 */
enum sky_dev_param sky_devparam_from_str(enum sky_dev_type dev_type,
					 const char *str);

/**
 * Fill in the input buffer with a string representation of the device
 * param value and returns length of a string.
 */
int sky_devparam_value_to_str(enum sky_dev_type dev_type,
			      enum sky_dev_param param,
			      const struct sky_dev_params *params,
			      enum sky_param_value_format value_format,
			      char *buf, size_t size);

/**
 * Fill in the param with the value parsed from a string.
 */
int sky_devparam_value_from_str(const char *str,
				enum sky_dev_type dev_type,
				enum sky_dev_param param,
				struct sky_dev_params *params);

/**
 * Returns string representation of the sink param.
 */
const char *sky_sinkparam_to_str(enum sky_sink_param param);

/**
 * Returns sink param if string is recognized. If string can't
 * be parsed number of device params is returned.
 */
enum sky_sink_param sky_sinkparam_from_str(const char *str);

/**
 * Fill in the input buffer with a string representation of the sink
 * param value and returns length of a string.
 */
int sky_sinkparam_value_to_str(enum sky_sink_param param,
			       const struct sky_dev_params *params,
			       enum sky_param_value_format value_format,
			       char *buf, size_t size);

/**
 * Fill in the param with the value parsed from a string.
 */
int sky_sinkparam_value_from_str(const char *str,
				 enum sky_sink_param param,
				 struct sky_dev_params *params);

/**
 * Returns string representation of the GPS status.
 */
const char *sky_gpsstatus_to_str(enum sky_gps_status status);

/**
 * Returns string representation of the GPS mode.
 */
const char *sky_gpsmode_to_str(enum sky_gps_mode mode);

/**
 * sky_hw_is_charging() - returns true if hardware is in charge state
 */
static inline int sky_hw_is_charging(enum sky_dev_type mux_type,
				     enum sky_dev_state hw_state)
{
	if (mux_type == SKY_MUX_HW1)
		return  hw_state == SKY_HW1_CHARGING_RUN ||
			hw_state == SKY_HW1_CHARGING_MONITOR_CURRENT;

	return (hw_state == SKY_HW2_PRECHARGING ||
		hw_state == SKY_HW2_CHARGING);
}

/**
 * sky_confinit() - Inits generic configuration to zero.
 * @cfg:        Conf structure to be zeroed out.
 *
 * Sets all values of the @cfg structure to zero.
 */
void sky_confinit(struct sky_conf *cfg);

/**
 * sky_confparse() - Parse generic skycharge configuration.
 * @path:       Path to config.
 * @cfg:        Conf structure to be filled in.
 *
 * Parses configuration and fills in the @cfg structure.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise.
 */
int sky_confparse(const char *path, struct sky_conf *cfg);

/**
 * sky_asyncopen() - Opens asynchronous context.
 * @conf:       Generic device configuration.
 * @async:      Output of asynchronous context.
 *
 * Opens asynchronous context.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise.
 */
int sky_asyncopen(const struct sky_conf *conf,
		  struct sky_async **async);

/**
 * sky_asyncdeinit() - Closes asynchronous context.
 * @async:      Asynchronous context.
 *
 * Closes asynchronous context.
 */
void sky_asyncclose(struct sky_async *async);

/**
 * sky_asyncfd() - Returns fd of this asynchronous context.
 * @async:      Asynchronous context.
 *
 * Returns file descriptor of the asynchronous context.
 * Using this file descriptor usual polling on incomming events
 * can be done, so POLLIN events should be expected.
 * After POLLIN events are received sky_asyncexecute() should be
 * invoked with @false as @wait param.
 */
int sky_asyncfd(struct sky_async *async);

/**
 * sky_asyncexecute() - Executes all asynchronous requests and completes
 *                      corresponding responses.
 * @async:       Asynchronous context.
 * @wait:        Wait for submitted requests to complete
 *
 * Returns >=0 indicating number of completed requests and received
 * responses. Returns <0 in case of error. Error means mostly likely
 * unrecoverably error and the whole asynchronous context has to be
 * closed and all memory freed.
 */
int sky_asyncexecute(struct sky_async *async, bool wait);

/**
 * sky_asyncreq_completionset() - Sets completion handler.
 * @req:           Asynchronous request.
 * @completion:    Completion handler to be associated with the request.
 */
void sky_asyncreq_completionset(struct sky_async_req *req,
				sky_async_completion_t *completion);

/**
 * sky_asyncreq_useruuidset() - Sets user uuid for authentication.
 * @req:        Asynchronous request.
 * @uruuid:     User uuid, 16 bytes.
 *
 * NOTE: @usruuid is not copied, but is used as a pointer, so
 *       memory should be available during live time of the
 *       request.
 */
void sky_asyncreq_useruuidset(struct sky_async_req *req,
			      const uint8_t *usruuid);

/**
 * sky_asyncreq_cancel() - Cancels submitted request.
 * @async:      Asynchronous context.
 * @req:        Request to be cancelled.
 *
 * Cancel request which was previously submitted to the asynchronous
 * context queue.
 *
 * Returns true if request was successfully cancelled or false
 * in case if request was already completed.
 */
bool sky_asyncreq_cancel(struct sky_async *async,
			 struct sky_async_req *req);

/**
 * sky_subscribe() - Subscribe on @sky_charging_state update events.
 * @dev:	Device context.
 * @sub:	Subscription configuration.
 *
 * Subscribes on @sky_charging_state update events.
 * Beware:
 *    @state->on_event() callback is called from another thread.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EINVAL invalid parameters.
 * -EEXIST is already subscribed
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_subscribe(struct sky_dev *dev, struct sky_subscription *sub);

/**
 * sky_unsubscribe() - Unsubscribe from @sky_charging_state update events.
 * @dev:	Device context.
 *
 * Unsbscribes from @sky_charging_state update events.
 * Beware:
 *    function is synchronous and waits till subscription thread exits.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EPERM  if operation is not permitted.
 * -ENOENT was not subscribed
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_unsubscribe(struct sky_dev *dev);

/**
 * sky_discoverbroker() - Discover broker on the local network.
 * @brokerinfo: Broker infor to be filled in.
 * @timeout_ms: Timeout in ms.
 *
 * Function fills in @brokerinfo and returns 0 if skybroker was
 * disovered, otherwise -ENONET is returned.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -ENONET      skybroker is not on the network
 * -EOPNOTSUPP  library does not support any remote access
 */
int sky_discoverbroker(struct sky_brokerinfo *brokerinfo,
		       unsigned int timeout_ms);

/**
 * sky_peerinfo() - Fills in peer information.
 * @conf:      Device configuration.
 * @peerinfo:  Peerinfo structure.
 *
 * Function fills in @peerinfo for specified @conf.  @conf->contype
 * must be @SKY_REMOTE, otherwise -EOPNOTSUPP is returned.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EINVAL invalid parameters.
 */
int sky_peerinfo(const struct sky_conf *conf,
		 struct sky_peerinfo *peerinfo);

/**
 * sky_asyncreq_peerinfo() - Inits and submits an asynchronous request to
 *                           receive the peer information.
 *
 * See synchronous sky_peerinfo() variant for details.
 */
int sky_asyncreq_peerinfo(struct sky_async *async,
			  struct sky_peerinfo *peerinfo,
			  struct sky_async_req *req);

/**
 * sky_devslist() - Returns list of found devices.
 * @conf:     Device configuration.
 * @list:     Fills list of all found devices.
 *
 * Functions scans for all Sky devices and puts them to the
 * list provided as an argument.  Do not forget to call sky_devsfree().
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -ENODEV if no devices found, see sky_asyncreq_devslist() for API differences.
 * -EPERM  if operation is not permitted.
 * -ENOMEM if memory allocation failed.
 */
int sky_devslist(const struct sky_conf *conf,
		 struct sky_dev_desc **list);

/**
 * sky_asyncreq_devslist() - Inits and submits an asynchronous request to
 *                           fetch a list of devices.
 *
 * See synchronous sky_devslist() variant for details
 *
 * BEWARE: @list is set to NULL if no devices are found, but synchronous
 *         sky_devslist() variant returns -ENODEV.
 */
int sky_asyncreq_devslist(struct sky_async *async,
			   struct sky_dev_desc **list,
			   struct sky_async_req *req);

/**
 * sky_devsfree() - Frees allocated list.
 * @list:     Devices list.
 *
 * Frees a list which was allocated by sky_devslist().
 */
void sky_devsfree(struct sky_dev_desc *list);

/**
 * sky_devopen() - Opens a device and returns its context.
 * @devdesc:   Device descriptor.
 * @dev:       Output pointer for storing device context.
 *
 * Function opens a device, described by device descriptor.  In case
 * of success valid device context will be returned in @dev argument.
 * Do not forget to close the device with calling sky_devclose().
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EPERM  if operation is not permitted.
 * -ENOMEM if memory allocation failed.
 */
int sky_devopen(const struct sky_dev_desc *devdesc, struct sky_dev **dev);

/**
 * sky_devclose() - Closes a device context.
 * @dev:	Device context to be closed.
 *
 * Function disconnects from a device and frees all corresponding memory.
 */
void sky_devclose(struct sky_dev *dev);

/**
 * sky_devinfo() - Returns device information.
 * @dev:	Device context.
 * @devdesc:	Device structure to be filled in.
 *
 * Function fills in information about a device to which connection is
 * established (local or remote).
 *
 * E.g. that is needed to get a valid device type to be sure that functions
 * sky_droneport_open() and sky_droneport_close() will function.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_devinfo(struct sky_dev *dev, struct sky_dev_desc *devdesc);

/**
 * sky_paramsget() - Fetches device configuration parameters.
 * @dev:	Device context.
 * @params:	Device params to be filled in.
 *
 * Accesses the hardware and fetches current parameters of a device.
 * Required bits of a device parameter must be set in a bitfield
 * @params->dev_params_bits.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EINVAL invalid parameter, e.g. @params->dev_params_bits were not set.
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_paramsget(struct sky_dev *dev, struct sky_dev_params *params);

/**
 * sky_asyncreq_paramsget() - Inits and submits an asynchronous request to
 *                            fetch device configuration parameters.
 *
 * See synchronous sky_paramsget() variant for details.
 */
int sky_asyncreq_paramsget(struct sky_async *async,
			   struct sky_dev *dev,
			   struct sky_dev_params *params,
			   struct sky_async_req *req);

/**
 * sky_paramsset() - Saves device configuration parameters.
 * @dev:	Device context.
 * @params:	Device params to be saved.
 *
 * Accesses the hardware and saves parameters of a device.
 * Required bits of a device parameter must be set in a bitfield
 * @params->dev_params_bits.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EINVAL invalid parameter, e.g. @params->dev_params_bits were not set.
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_paramsset(struct sky_dev *dev, const struct sky_dev_params *params);

/**
 * sky_asyncreq_paramsset() - Inits and submits an asynchronous request to
 *                            save device configuration parameters.
 *
 * See synchronous sky_paramsset() variant for details.
 */
int sky_asyncreq_paramsset(struct sky_async *async,
			   struct sky_dev *dev,
			   const struct sky_dev_params *params,
			   struct sky_async_req *req);

/**
 * sky_chargingstate() - Charging device state.
 * @dev:	Device context.
 * @state:	Charging device state.
 *
 * Returns charging state of a device.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_chargingstate(struct sky_dev *dev, struct sky_charging_state *state);

/**
 * sky_asyncreq_chargingstate() - Inits and submits an asynchronous request to
 *                                get charging device state.
 *
 * See synchronous sky_chargingstate() variant for details.
 */
int sky_asyncreq_chargingstate(struct sky_async *async,
			       struct sky_dev *dev,
			       struct sky_charging_state *state,
			       struct sky_async_req *req);

/**
 * sky_reset() - Resets device.
 * @dev:	Device context.
 *
 * Resets device.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_reset(struct sky_dev *dev);

/**
 * sky_asyncreq_reset() - Inits and submits an asynchronous request to
 *                        reset a device.
 *
 * See synchronous sky_reset() variant for details.
 */
int sky_asyncreq_reset(struct sky_async *async,
		       struct sky_dev *dev,
		       struct sky_async_req *req);

/**
 * sky_chargestart() - Starts device charge.
 * @dev:	Device context.
 *
 * Starts device charge.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_chargestart(struct sky_dev *dev);

/**
 * sky_asyncreq_chargestart() - Inits and submits an asynchronous request to
 *                              start charging.
 *
 * See synchronous sky_chargestart() variant for details.
 */
int sky_asyncreq_chargestart(struct sky_async *async,
			     struct sky_dev *dev,
			     struct sky_async_req *req);

/**
 * sky_chargestop() - Stops device charge.
 * @dev:	Device context.
 *
 * Stops device charge.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_chargestop(struct sky_dev *dev);

/**
 * sky_asyncreq_chargestop() - Inits and submits an asynchronous request to
 *                             stop charging.
 *
 * See synchronous sky_chargestop() variant for details.
 */
int sky_asyncreq_chargestop(struct sky_async *async,
			    struct sky_dev *dev,
			    struct sky_async_req *req);

/**
 * sky_droneport_open() - Opens a cover of a drone port.
 * @dev:		  Device context.
 *
 * Opens a drone port cover. Device must be @SKY_OUTDOOR.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EOPNOTSUPP if device is not a @SKY_OUTDOOR.
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_droneport_open(struct sky_dev *dev);

/**
 * sky_asyncreq_droneport_open() - Inits and submits an asynchronous request to
 *                                 open a cover of a drone port.
 *
 * See synchronous sky_droneport_open() variant for details.
 */
int sky_asyncreq_droneport_open(struct sky_async *async,
				struct sky_dev *dev,
				struct sky_async_req *req);

/**
 * sky_droneport_close() - Closes a cover of a drone port.
 * @dev:		   Device context.
 *
 * Closes a drone port cover. Device must be @SKY_OUTDOOR.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EOPNOTSUPP if device is not a @SKY_OUTDOOR.
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_droneport_close(struct sky_dev *dev);

/**
 * sky_asyncreq_droneport_close() - Inits and submits an asynchronous request to
 *                                  close a cover of a drone port.
 *
 * See synchronous sky_droneport_close() variant for details.
 */
int sky_asyncreq_droneport_close(struct sky_async *async,
				 struct sky_dev *dev,
				 struct sky_async_req *req);

/**
 * sky_droneport_state() - Retrieves droneport state.
 * @dev:		   Device context.
 *
 * Returns the current state of a droneport. Device must be @SKY_OUTDOOR.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EOPNOTSUPP if device is not a @SKY_OUTDOOR.
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of a remote connection)
 */
int sky_droneport_state(struct sky_dev *dev, struct sky_droneport_state *state);

/**
 * sky_asyncreq_droneport_state() - Retrieves droneport state.
 *
 * See synchronous sky_droneport_state() variant for details.
 */
int sky_asyncreq_droneport_state(struct sky_async *async,
				 struct sky_dev *dev,
				 struct sky_droneport_state *state,
				 struct sky_async_req *req);

/**
 * sky_gpsdata() - Gets the data collected by the GPS module.
 * @dev:	Device context.
 * @gps:	GPS data.
 *
 * Gets the data collected by the GPS.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EOPNOTSUPP GPS module is not connected
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_gpsdata(struct sky_dev *dev, struct sky_gpsdata *gpsdata);

/**
 * sky_asyncreq_gpsdata() - Inits and submits an asynchronous request to
 *                          get the gps data.
 *
 * See synchronous sky_gpsdata() variant for details.
 */
int sky_asyncreq_gpsdata(struct sky_async *async,
			 struct sky_dev *dev,
			 struct sky_gpsdata *gpsdata,
			 struct sky_async_req *req);

/**
 * sky_dronedetect() - Detects drone on charging pad and returns status.
 * @dev:	Device context.
 * @status:	Drone status on charging pad.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_dronedetect(struct sky_dev *dev, enum sky_drone_status *status);

/**
 * sky_asyncreq_dronedetect() - Inits and submits an asynchronous request to
 *                              detect a drone on the charging pad.
 *
 * See synchronous sky_dronedetect() variant for details.
 */
int sky_asyncreq_dronedetect(struct sky_async *async,
			     struct sky_dev *dev,
			     enum sky_drone_status *status,
			     struct sky_async_req *req);

/**
 * sky_sink_infoget() - Fetches sink device information.
 * @dev:	Device context.
 * @params:	Device params to be filled in.
 *
 * Accesses the hardware and fetches current parameters of a sink device.
 * Required bits of a device parameter must be set in a bitfield
 * @params->dev_params_bits.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EINVAL invalid parameter, e.g. @params->dev_params_bits were not set.
 * -ENOLINK no link with sink device
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_sink_infoget(struct sky_dev *dev, struct sky_sink_info *info);

/**
 * sky_asyncreq_sink_infoget() - Inits and submits an asynchronous request to
 *                               fetch sink device information.
 *
 * See synchronous sky_sink_infoget() variant for details.
 */
int sky_asyncreq_sink_infoget(struct sky_async *async,
			      struct sky_dev *dev,
			      struct sky_sink_info *info,
			      struct sky_async_req *req);

/**
 * sky_sink_paramsget() - Fetches sink device configuration parameters.
 * @dev:	Device context.
 * @params:	Device params to be filled in.
 *
 * Accesses the hardware and fetches current parameters of a sink device.
 * Required bits of a device parameter must be set in a bitfield
 * @params->dev_params_bits.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EINVAL invalid parameter, e.g. @params->dev_params_bits were not set.
 * -ENOLINK no link with sink device
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_sink_paramsget(struct sky_dev *dev, struct sky_dev_params *params);

/**
 * sky_asyncreq_sink_paramsget() - Inits and submits an asynchronous request to
 *                                 fetch sink device configuration parameters.
 *
 * See synchronous sky_sink_paramsget() variant for details.
 */
int sky_asyncreq_sink_paramsget(struct sky_async *async,
				struct sky_dev *dev,
				struct sky_dev_params *params,
				struct sky_async_req *req);

/**
 * sky_sink_paramsset() - Saves sink device configuration parameters.
 * @dev:	Device context.
 * @params:	Device params to be saved.
 *
 * Accesses the hardware and saves parameters of a sink device.
 * Required bits of a device parameter must be set in a bitfield
 * @params->dev_params_bits.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EINVAL invalid parameter, e.g. @params->dev_params_bits were not set.
 * -ENOLINK no link with sink device
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_sink_paramsset(struct sky_dev *dev, const struct sky_dev_params *params);

/**
 * sky_asyncreq_sink_paramsset() - Inits and submits an asynchronous request to
 *                                 save device sink configuration parameters.
 *
 * See synchronous sky_sink_paramsset() variant for details.
 */
int sky_asyncreq_sink_paramsset(struct sky_async *async,
				struct sky_dev *dev,
				const struct sky_dev_params *params,
				struct sky_async_req *req);

/**
 * sky_sink_chargestart() - Starts charge on a sink.
 * @dev:	Device context.
 *
 * Accesses the hardware and starts charge on a sink.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -ENOLINK no link with sink device
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_sink_chargestart(struct sky_dev *dev);

/**
 * sky_asyncreq_sink_chargestart() - Start charge on a sink.
 *
 * See synchronous sky_sink_chargestart() variant for details.
 */
int sky_asyncreq_sink_chargestart(struct sky_async *async,
				  struct sky_dev *dev,
				  struct sky_async_req *req);
/**
 * sky_sink_chargestop() - Stops charge on a sink.
 * @dev:	Device context.
 *
 * Accesses the hardware and stops charge on a sink.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -ENOLINK no link with sink device
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_sink_chargestop(struct sky_dev *dev);

/**
 * sky_asyncreq_sink_chargestop() - Stop charge on a sink.
 *
 * See synchronous sky_sink_chargestop() variant for details.
 */
int sky_asyncreq_sink_chargestop(struct sky_async *async,
				  struct sky_dev *dev,
				  struct sky_async_req *req);

#ifdef __cplusplus
}
#endif

#endif /* LIBSKYCHARGE_H */

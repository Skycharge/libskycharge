#ifndef LIBSKYSENSE_H
#define LIBSKYSENSE_H

#include <stdlib.h>
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
 * enum sky_con_type - Connection type.
 */
enum sky_con_type {
	SKY_LOCAL  = 0,
	SKY_REMOTE = 1,
};

/**
 * enum sky_dev_type - Device type.
 */
enum sky_dev_type {
	SKY_INDOOR  = 0,
	SKY_OUTDOOR = 1,
};

/**
 * enum sky_dev_hw_state - Hardware state of the device.
 */
enum sky_dev_hw_state {
	SKY_UNKNOWN			= 0,
	SKY_SCANNING_INIT		= 1,
	SKY_SCANNING_RUN		= 2,
	SKY_SCANNING_CHECK_MATRIX	= 3,
	SKY_SCANNING_PRINT		= 4,
	SKY_SCANNING_CHECK_WATER	= 5,
	SKY_SCANNING_WET		= 6,
	SKY_SCANNING_DETECTING		= 7,
	SKY_PRE_CHARGING_INIT		= 8,
	SKY_PRE_CHARGING_RUN		= 9,
	SKY_PRE_CHARGING_CHECK_MATRIX	= 10,
	SKY_PRE_CHARGING_PRINT		= 11,
	SKY_PRE_CHARGING_CHECK_WATER	= 12,
	SKY_PRE_CHARGING_WET		= 13,
	SKY_PRE_CHARGING_FIND_CHARGERS	= 14,
	SKY_CHARGING_INIT		= 15,
	SKY_CHARGING_RUN		= 16,
	SKY_CHARGING_MONITOR_CURRENT	= 17,
	SKY_POST_CHARGING_INIT		= 18,
	SKY_POST_CHARGING_RUN		= 19,
	SKY_POST_CHARGING_CHECK_MATRIX	= 20,
	SKY_POST_CHARGING_PRINT		= 21,
	SKY_POST_CHARGING_CHECK_WATER	= 22,
	SKY_POST_CHARGING_WET		= 23,
	SKY_POST_CHARGING_FIND_CHARGERS = 24,
	SKY_OVERLOAD			= 25,
	SKY_AUTOSCAN_DISABLED		= 250,
};

/**
 * enum sky_dev_param - Configuration device parameter.
 */
enum sky_dev_param {
	SKY_EEPROM_INITED	     = 0,
	SKY_SCANNING_INTERVAL	     = 1,
	SKY_PRECHARGING_INTERVAL     = 2,
	SKY_PRECHARGING_COUNTER	     = 3,
	SKY_POSTCHARGING_INTERVAL    = 4,
	SKY_POSTCHARGING_DELAY	     = 5,
	SKY_WET_DELAY		     = 6,
	SKY_SHORTCIRC_DELAY	     = 7,
	SKY_THRESH_FINISH_CHARGING   = 8,
	SKY_THRESH_NOCHARGER_PRESENT = 9,
	SKY_THRESH_SHORTCIRC	     = 10,
	SKY_CURRENT_MON_INTERVAL     = 11,
	SKY_WAIT_START_CHARGING_SEC  = 12,

	SKY_NUM_DEVPARAM, /* Should be the last */
};

/**
 * struct sky_dev_params - Device configuration parameters.
 */
struct sky_dev_params {
	uint32_t dev_params_bits;
	uint32_t dev_params[SKY_NUM_DEVPARAM];
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
	struct sky_dev_params mux_hw1_params;

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
 * struct sky_dev_desc - Device descriptor.
 */
struct sky_dev_desc {
	struct sky_dev_desc *next;
	struct sky_conf   conf;
	enum sky_dev_type dev_type;
	const struct sky_dev_ops *dev_ops;
	const struct sky_hw_ops *hw_ops;
	unsigned firmware_version;
	char dev_name[16];
	unsigned char dev_uuid[16];
	char portname[32];
};

#define version_major(v) (((v) >> 16) & 0xff)
#define version_minor(v) (((v) >> 8) & 0xff)
#define version_revis(v) ((v) & 0xff)

#define foreach_devdesc(devdesc, head)		\
	for (devdesc = head; devdesc; devdesc = devdesc->next)

/**
 * struct sky_charging_state - Charging device state.
 */
struct sky_charging_state {
	enum sky_dev_hw_state dev_hw_state;
	unsigned short current;
	unsigned short voltage;
	struct {
		unsigned short charge_perc; /* 0-100% */
		unsigned short charge_time; /* seconds */
	} bms;
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
 * sky_hw_is_charging() - returns true if hardware is in charge state
 */
static inline int sky_hw_is_charging(enum sky_dev_hw_state hw_state)
{
	return  hw_state == SKY_CHARGING_RUN ||
		hw_state == SKY_CHARGING_MONITOR_CURRENT;
}

/**
 * sky_confinit() - Inits generic configuration to zero.
 * @cfg:        Conf structure to be zeroed out.
 *
 * Sets all values of the @cfg structure to zero.
 */
void sky_confinit(struct sky_conf *cfg);

/**
 * sky_confparse() - Parse generic skysense configuration.
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
 * -EPERM  if operation is not permitted.
 * -ENOMEM if memory allocation failed.
 */
int sky_devslist(const struct sky_conf *conf,
		 struct sky_dev_desc **list);

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
 * @dev:		Device context.
 * @devdesc:	Device structure to be filled in.
 *
 * Function fills in information about a device to which connection is
 * established (local or remote).
 *
 * E.g. that is needed to get a valid device type to be sure that functions
 * sky_coveropen() and sky_coverclose() will function.
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
 * sky_coveropen() - Opens a charging pad cover.
 * @dev:	Device context.
 *
 * Opens a charging pad cover.	Device must be @SKY_OUTDOOR.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EOPNOTSUPP if device is not a @SKY_OUTDOOR.
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_coveropen(struct sky_dev *dev);

/**
 * sky_coverclose() - Closes a charging pad cover.
 * @dev:	Device context.
 *
 * Closes a charging pad cover.	 Device must be @SKY_OUTDOOR.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EOPNOTSUPP if device is not a @SKY_OUTDOOR.
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_coverclose(struct sky_dev *dev);

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
 * -ENODEV GPS module is not connected
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_gpsdata(struct sky_dev *dev, struct sky_gpsdata *gpsdata);

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

#ifdef __cplusplus
}
#endif

#endif /* LIBSKYSENSE_H */

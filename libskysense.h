#ifndef LIBSKYSENSE_H
#define LIBSKYSENSE_H

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * struct sky_lib - Opaque library context.
 */
struct sky_lib;

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
	SKY_SCANNING_RUN_STATE		= 2,
	SKY_SCANNING_CHECK_MATRIX	= 3,
	SKY_SCANNING_CHECK_WATER	= 5,
	SKY_SCANNING_WET		= 6,
	SKY_SCANNING_DETECTING		= 7,
	SKY_PRE_CHARGING_INIT		= 8,
	SKY_PRE_CHARGING_RUN		= 9,
	SKY_PRE_CHARGING_CHECK_MATRIX	= 10,
	SKY_PRE_CHARGING_CHECK_WATER	= 12,
	SKY_PRE_CHARGING_WET		= 13,
	SKY_PRE_CHARGING_FIND_CHARGERS	= 14,
	SKY_CHARGING_INIT		= 15,
	SKY_CHARGING_RUN		= 16,
	SKY_CHARGING_MONITOR_CURRENT	= 17,
	SKY_POST_CHARGING_INIT		= 18,
	SKY_POST_CHARGING_RUN		= 19,
	SKY_POST_CHARGING_CHECK_MATRIX	= 20,
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
 * struct sky_lib_conf - Library configuration options.
 */
struct sky_lib_conf {
	enum sky_con_type contype;
	union {
		struct {
			char hostname[64];     /**< Remote hostname */
			unsigned cmdport;      /**< TCP/IP command port */
			unsigned subport;      /**< TCP/IP subscription port */
		} remote;
		struct {
			char portname[256];
		} local;
	};
};

/**
 * struct sky_dev_desc - Device descriptor.
 */
struct sky_dev_desc {
	struct sky_dev_desc *next;
	enum sky_dev_type dev_type;
	char portname[256];
};

/**
 * struct sky_charging_state - Charging device state.
 */
struct sky_charging_state {
	enum sky_dev_hw_state dev_hw_state;
	unsigned short current;
	unsigned short voltage;
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
 * struct sky_dev_conf - Device configuration.
 */
struct sky_dev_conf {
	uint32_t dev_params_bits;
	uint32_t dev_params[SKY_NUM_DEVPARAM];
};

/**
 * sky_devslist() - Returns list of found local devices.
 * @list:     Fills list of all found local devices.
 *
 * Functions scans for all local Sky devices and puts them to the
 * list provided as an argument.  Do not forget to call sky_devsfree().
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:

 * -EPERM  if operation is not permitted.
 * -ENOMEM if memory allocation failed.
 */
int sky_devslist(struct sky_dev_desc **list);

/**
 * sky_devsfree() - Frees allocated list.
 * @list:     Devices list.
 *
 * Frees a list which was allocated by sky_devslist().
 */
void sky_devsfree(struct sky_dev_desc *list);

/**
 * sky_libopen() - Opens a library and returns its context.
 * @conf:      Configuration options.
 * @lib:       Output pointer for storing library context.
 *
 * Function establishes connection either to local or to remote device,
 * see @conf->contype member.	 In case of successful connection valid
 * library context will be returned in @lib argument.  Do not forget to
 * close the library with calling sky_libclose().
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EPERM  if operation is not permitted.
 * -ECONNREFUSED no-one listening on the remote address (remote connection)
 * -ENOMEM if memory allocation failed.
 */
int sky_libopen(const struct sky_lib_conf *conf, struct sky_lib **lib);

/**
 * sky_libclose() - Closes a library context.
 * @lib:	Library context to be closed.
 *
 * Function disconnects from a device and frees all corresponding memory.
 */
void sky_libclose(struct sky_lib *lib);

/**
 * sky_devinfo() - Returns device information.
 * @lib:	Library context.
 * @dev:	Device structure to be filled in.
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
int sky_devinfo(struct sky_lib *lib, struct sky_dev_desc *dev);

/**
 * sky_confget() - Fetches device configuration.
 * @lib:	Library context.
 * @conf:	Configuration to be filled in.
 *
 * Accesses the hardware and fetches current configration of a device.
 * Required bits of a device parameter must be set in a bitfield
 * @conf->dev_params_bits.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EINVAL invalid parameter, e.g. @conf->dev_params_bits were not set.
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_confget(struct sky_lib *lib, struct sky_dev_conf *conf);

/**
 * sky_confset() - Saves device configuration.
 * @lib:	Library context.
 * @conf:	Configuration to be saved.
 *
 * Accesses the hardware and saves configration of a device.
 * Required bits of a device parameter must be set in a bitfield
 * @conf->dev_params_bits.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EINVAL invalid parameter, e.g. @conf->dev_params_bits were not set.
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_confset(struct sky_lib *lib, struct sky_dev_conf *conf);

/**
 * sky_chargingstate() - Charging device state.
 * @lib:	Library context.
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
int sky_chargingstate(struct sky_lib *lib, struct sky_charging_state *state);

/**
 * sky_subscribe() - Subscribe on @sky_charging_state update events.
 * @lib:	Library context.
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
int sky_subscribe(struct sky_lib *lib, struct sky_subscription *sub);

/**
 * sky_unsubscribe() - Unsubscribe from @sky_charging_state update events.
 * @lib:	Library context.
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
int sky_unsubscribe(struct sky_lib *lib);

/**
 * sky_reset() - Resets device.
 * @lib:	Library context.
 *
 * Resets device.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_reset(struct sky_lib *lib);

/**
 * sky_chargestart() - Starts device charge.
 * @lib:	Library context.
 *
 * Starts device charge.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_chargestart(struct sky_lib *lib);

/**
 * sky_chargestop() - Stops device charge.
 * @lib:	Library context.
 *
 * Stops device charge.
 *
 * RETURNS:
 * Returns 0 on success and <0 otherwise:
 *
 * -EPERM  if operation is not permitted.
 * -ECONNRESET connection reset by peer (in case of remote connection)
 */
int sky_chargestop(struct sky_lib *lib);

/**
 * sky_coveropen() - Opens a charging pad cover.
 * @lib:	Library context.
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
int sky_coveropen(struct sky_lib *lib);

/**
 * sky_coverclose() - Closes a charging pad cover.
 * @lib:	Library context.
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
int sky_coverclose(struct sky_lib *lib);

#ifdef __cplusplus
}
#endif

#endif /* LIBSKYSENSE_H */

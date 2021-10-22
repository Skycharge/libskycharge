#ifndef HW2_PRI_H
#define HW2_PRI_H

enum sky_hw2_errno {
	SKY_HW2_SUCCESS    = 0,
	SKY_HW2_EINVAL     = 1,
	SKY_HW2_ETIMEDOUT  = 2,
	SKY_HW2_EOPNOTSUPP = 3,
	SKY_HW2_EPROTO     = 4,
	SKY_HW2_EOVERFLOW  = 5,
	SKY_HW2_ECRC       = 6,
	SKY_HW2_EINTR      = 7,
	SKY_HW2_EBUSY      = 8,
	SKY_HW2_ENOLINK    = 9,


	SKY_MAX_ERRNO  = 16 /* Due to the uart response type,
			     * which embeds 4 bits for errno */
};

enum sky_hw2_serial_cmd {
	SKY_HW2_RESET_CMD                      = 0x00,
	SKY_HW2_STOP_CMD                       = 0x01,
	SKY_HW2_RESUME_CMD                     = 0x02,
	SKY_HW2_SEND_PASSTHRU_DATA_CMD         = 0x03,
	SKY_HW2_GET_CHARGING_STATE_CMD         = 0x04,
	SKY_HW2_GET_MUX_INFO_CMD               = 0x05,
	SKY_HW2_GET_MUX_SETTINGS_CMD           = 0x06,
	SKY_HW2_SET_MUX_SETTINGS_CMD           = 0x07,
	SKY_HW2_GET_SINK_INFO_CMD              = 0x08,
	SKY_HW2_GET_SINK_CHARGING_SETTINGS_CMD = 0x09,
	SKY_HW2_SET_SINK_CHARGING_SETTINGS_CMD = 0x0a,
	SKY_HW2_SINK_START_CHARGE_CMD          = 0x0b,
	SKY_HW2_SINK_STOP_CHARGE_CMD           = 0x0c,

	/*
	 * Types of messages from mux
	 */
	SKY_HW2_SYNC_RESPONSE_CMD              = 0x00, /* response on each sync command */
	SKY_HW2_ASYNC_PASSTHRU_DATA_CMD        = 0x01, /* async passthru data */

	SKY_HW2_MAX_RESP_CMD                   = 0x10  /* Due to the fact that we have
							* only 4 lsb for type, other
							* 4 are used for errno. */
};

enum sky_hw2_mux_settings_bits {
	SKY_HW2_IGNORE_INVAL_CHARGING_SETTINGS_BIT = 1<<0,
	SKY_HW2_IGNORE_LOW_BATT_VOLTAGE_BIT        = 1<<1,
	SKY_HW2_KEEP_SILENCE_BIT                   = 1<<2,
	SKY_HW2_USE_FIXED_V_I_BIT                  = 1<<3,
	SKY_HW2_IGNORE_VOLTAGE_ON_OUTPUT_BIT       = 1<<4,
};

struct calib_point {
	uint16_t set;
	uint16_t read;
};

struct sky_hw2_mux_settings {
	struct {
		struct calib_point voltage_p1_mV;
		struct calib_point voltage_p2_mV;
		struct calib_point current_p1_mA;
		struct calib_point current_p2_mA;
	}        sense_calib;
	struct {
		struct calib_point voltage_p1_mV;
		struct calib_point voltage_p2_mV;
		struct calib_point current_p1_mA;
		struct calib_point current_p2_mA;
	}        psu_calib;

	uint16_t bool_settings;
	uint16_t psu_fixed_voltage_mV;
	uint16_t psu_fixed_current_mA;
	uint8_t  psu_type;
	uint8_t  detect_mode;
	uint8_t  nr_bad_heartbeats; /* 0 - ignore bad heartbeats */
	uint8_t  error_indication_timeout_secs; /* 0 - don't indicate errors */
	uint8_t  min_sense_current_mA;
	uint8_t  padding;
	uint16_t repeat_charge_after_mins; /* 0 - never */
};

/*
 * Explicit set of charging settings which describe a battery
 * and the charging voltage and current.
 */
struct sky_hw2_charging_settings {
	uint8_t  capabilities;
	uint8_t  batt_type;
	uint16_t batt_capacity_mAh;
	uint16_t batt_min_voltage_mV; /* Min voltage of a battery, error will pop up
				       * on mux if voltage is detected below this
				       * value */
	uint16_t batt_max_voltage_mV; /* Max voltage of a battery we use for charging,
				       * we also use min/max voltage of a battery to
				       * calculate percentage of the charge and time
				       * until battery is full */

	uint16_t charging_max_current_mA; /* Max charging current */
	uint16_t cutoff_min_current_mA; /* Min current threshold when we stop charging */
	uint16_t cutoff_timeout_ms; /* For how long it is allowed to detect min current */
	uint8_t  precharge_current_coef; /* Precharge current coef in percentage, so in
					  * range [0 100]. Is used for slow current
					  * increase while charging */
	uint8_t  padding1;
	uint16_t precharge_delay_secs; /* For how many seconds we delay the charging
					* in order to cool down the battery */
	uint16_t precharge_secs; /* How many seconds we slowly increase current on the
				  * first phase */
	uint16_t total_charge_secs; /* How many seconds do we charge in total, charging
				     * stops when time elapses */
	uint8_t  user_data[16];
};


static inline int skyerrno_to_errno(enum sky_hw2_errno err)
{
	switch (err) {
	case SKY_HW2_SUCCESS:
		return 0;
	case SKY_HW2_EINVAL:
		return EINVAL;
	case SKY_HW2_ETIMEDOUT:
		return ETIMEDOUT;
	case SKY_HW2_EOPNOTSUPP:
		return EOPNOTSUPP;
	case SKY_HW2_EPROTO:
		return EPROTO;
	case SKY_HW2_EOVERFLOW:
		return EOVERFLOW;
	case SKY_HW2_ECRC:
		return EILSEQ;
	case SKY_HW2_EINTR:
		return EINTR;
	case SKY_HW2_EBUSY:
		return EBUSY;
	case SKY_HW2_ENOLINK:
		return ENOLINK;
	default:
		sky_err("Unknown error %d\n", err);
		return 0;
	}
}

#endif /* HW2_PRI_H */
#ifndef HW2_PRI_H
#define HW2_PRI_H

enum sky_hw2_serial_cmd {
	SKY_HW2_RESET_CMD                      = 0x00,
	SKY_HW2_STOP_CMD                       = 0x01,
	SKY_HW2_PASSTHRU_CMD                   = 0x02,
	SKY_HW2_RESUME_CMD                     = 0x03,
	SKY_HW2_SEND_PASSTHRU_DATA_CMD         = 0x04,
	SKY_HW2_GET_CHARGING_STATE_CMD         = 0x05,
	SKY_HW2_GET_MUX_INFO_CMD               = 0x06,
	SKY_HW2_GET_MUX_SETTINGS_CMD           = 0x07,
	SKY_HW2_SET_MUX_SETTINGS_CMD           = 0x08,
	SKY_HW2_GET_SINK_INFO_CMD              = 0x09,
	SKY_HW2_GET_SINK_CHARGING_SETTINGS_CMD = 0x0a,
	SKY_HW2_SET_SINK_CHARGING_SETTINGS_CMD = 0x0b,

	/*
	 * Types of messages from mux
	 */
	SKY_HW2_SYNC_RESPONSE_CMD              = 0x80, /* response on each sync command */
	SKY_HW2_ASYNC_PASSTHRU_DATA_CMD        = 0x81, /* async passthru data */
};

struct sky_hw2_mux_settings {
	uint8_t psu_type;
	uint8_t nr_bad_heartbeats; /* 0 - ignore bad heartbeats */
	uint8_t ignore_inval_charging_settings;
	uint8_t ignore_low_voltage;
	uint8_t error_indication_timeout_secs; /* 0 - don't indicate errors */
	uint8_t keep_silence; /* don't beep, never ever! */
	uint8_t padding[2];
	float   current_sense_calibration;
};

#endif /* HW2_PRI_H */

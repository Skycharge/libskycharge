#ifndef HW1_PRI_H
#define HW1_PRI_H

enum sky_hw1_serial_cmd {
	SKY_HW1_RESET_CMD              = 0x01,
	SKY_HW1_SAVE_DATA_TO_EEP_CMD   = 0x02,
	SKY_HW1_READ_DATA_FROM_EEP_CMD = 0x03,
	SKY_HW1_AUTOMATIC_SCAN_CMD     = 0x04,
	SKY_HW1_SET_PARAMETER_CMD      = 0x05,
	SKY_HW1_GET_PARAMETER_CMD      = 0x06,
	SKY_HW1_GET_STATUS_CMD         = 0x07,
	SKY_HW1_GET_CURRENT_CMD        = 0x08,
	SKY_HW1_GET_VOLTAGE_CMD        = 0x09,
	SKY_HW1_COUPLE_SCAN_CMD        = 0x0a,
	SKY_HW1_COUPLE_ACTIVATE_CMD    = 0x0b,
	SKY_HW1_COUPLE_DEACTIVATE_CMD  = 0x0c,
	SKY_HW1_FIRMWARE_VERSION_CMD   = 0x0d,

	SKY_HW1_ERROR                  = 0xfd,
};

#endif /* HW1_PRI_H */
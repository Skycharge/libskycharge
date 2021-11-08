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

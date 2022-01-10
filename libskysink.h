/*
 * Copyright 2022 Skycharge GmbH
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

#ifndef LIBSKYSINK_H
#define LIBSKYSINK_H

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

enum uart_cmd_type {
	USR_CMD_HANDSHAKE                  = 0x00,
	USR_CMD_GET_SINK_STATE             = 0x01,
	USR_CMD_PASSTHRU_MSG_SEND          = 0x02,
	USR_CMD_PASSTHRU_MSG_RECV          = 0x03,
};

enum passthru_recv_mode {
	PASSTHRU_IGNORE = 0,
	PASSTHRU_SYNC   = 1,
	PASSTHRU_ASYNC  = 2,
};

enum state_bits {
	CHARGING_ON_BIT    = 1<<0,
	TRANSISTORS_ON_BIT = 1<<1,
};

#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-Wpadded"

struct sink_state {
	uint16_t voltage_mV;
	int16_t  sink_temperature_C;
	uint8_t  passthru_recv_bytes;
	uint8_t  state_bits;
};

struct uart_cmd_hdr {
	uint8_t magic;
	uint8_t crc;
	uint8_t len;
	uint8_t type;
};

struct uart_cmd_handshake {
	struct uart_cmd_hdr hdr;
	uint8_t             mode;
};

struct uart_cmd_sink_state {
	struct uart_cmd_hdr hdr;
	struct sink_state   state;
};

struct uart_cmd_passthru_msg {
	struct uart_cmd_hdr hdr;
	uint8_t             buf[128];
};

#pragma GCC diagnostic pop

#endif /* LIBSKYSINK_H */

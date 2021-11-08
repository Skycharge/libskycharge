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

#include <assert.h>
#include <endian.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/file.h>

#include <libserialport.h>

#include "types.h"

enum {
	TIMEOUT_MS = 1000,
};

const unsigned char crc_tab[256] = {
	0,  94, 188, 226,  97,  63, 221, 131, 194, 156, 126,  32, 163, 253,  31,  65,
	157, 195,  33, 127, 252, 162,  64,  30,  95,   1, 227, 189,  62,  96, 130, 220,
	35, 125, 159, 193,  66,  28, 254, 160, 225, 191,  93,   3, 128, 222,  60,  98,
	190, 224,   2,  92, 223, 129,  99,  61, 124,  34, 192, 158,  29,  67, 161, 255,
	70,  24, 250, 164,  39, 121, 155, 197, 132, 218,  56, 102, 229, 187,  89,   7,
	219, 133, 103,  57, 186, 228,   6,  88,  25,  71, 165, 251, 120,  38, 196, 154,
	101,  59, 217, 135,   4,  90, 184, 230, 167, 249,  27,  69, 198, 152, 122,  36,
	248, 166,  68,  26, 153, 199,  37, 123,  58, 100, 134, 216,  91,   5, 231, 185,
	140, 210,  48, 110, 237, 179,  81,  15,  78,  16, 242, 172,  47, 113, 147, 205,
	17,  79, 173, 243, 112,  46, 204, 146, 211, 141, 111,  49, 178, 236,  14,  80,
	175, 241,  19,  77, 206, 144, 114,  44, 109,  51, 209, 143,  12,  82, 176, 238,
	50, 108, 142, 208,  83,  13, 239, 177, 240, 174,  76,  18, 145, 207,  45, 115,
	202, 148, 118,  40, 171, 245,  23,  73,   8,  86, 180, 234, 105,  55, 213, 139,
	87,   9, 235, 181,  54, 104, 138, 212, 149, 203,  41, 119, 244, 170,  72,  22,
	233, 183,  85,  11, 136, 214,  52, 106,  43, 117, 151, 201,  74,  20, 246, 168,
	116,  42, 200, 150,  21,  75, 169, 247, 182, 232,  10,  84, 215, 137, 107,  53
};

static uint8_t crc8(unsigned char *data, size_t n)
{
	unsigned char cks = 0, i;

	for (i = 0; i < n; i++)
		cks = crc_tab[cks ^ data[i]];

	return cks;
}

static inline int sprc_to_errno(enum sp_return sprc)
{
	switch (sprc) {
	case SP_OK:
		return 0;
	case SP_ERR_ARG:
		/** Invalid arguments were passed to the function. */
		return -EINVAL;
	case SP_ERR_FAIL:
		/** A system error occurred while executing the operation. */
		/** Here we pretend this is is EBADF */
		return -EBADF;
	case SP_ERR_MEM:
		/** A memory allocation failed while executing the operation. */
		return -ENOMEM;
	case SP_ERR_SUPP:
		/** The requested operation is not supported. */
		return -EOPNOTSUPP;
	default:
		/* WTF? */
		return -EOPNOTSUPP;
	}
}

static enum sp_return bms_read_line(struct sp_port *port, char *buf, size_t sz)
{
	enum sp_return sprc;
	size_t len = 0;

	while (sz) {
		sprc = sp_blocking_read(port, buf + len, 1, TIMEOUT_MS);
		if (sprc < 0)
			return sprc;
		if (sprc == 0)
			return -ETIMEDOUT;

		sz--;

		if (buf[len++] == '\n') {
			if (sz)
				buf[len] = '\0';
			return len;
		}
	}

	return -EPROTO;
}

static enum sp_return bms_suck_data(struct sp_port *port)
{
	enum sp_return sprc;
	int nonblock = 1;
	char buf[128];

	while (1) {
		sprc = nonblock ?
			sp_nonblocking_read(port, buf, sizeof(buf)) :
			sp_blocking_read(port, buf, 1, TIMEOUT_MS);
		if (sprc < 0)
			return sprc;
		if (sprc == 0 && !nonblock)
			/* Wait, got nothing and return */
			break;

		nonblock ^= 1;
	}

	return 0;
}

static enum sp_return bms_to_init_state(struct sp_port *port)
{
	enum sp_return sprc;

	/* Firstly hang up */
	sprc = sp_blocking_write(port, "ATH\r", 5, TIMEOUT_MS);
	if (sprc < 0)
		return sprc;

	//XXX
	printf(">>> sent ATH\n");
	sprc = bms_suck_data(port);
	if (sprc)
		return sprc;

	/* Then disconnect */
	sprc = sp_blocking_write(port, "<DS>", 4, TIMEOUT_MS);
	if (sprc < 0)
		return sprc_to_errno(sprc);

	printf(">>> sent <DS>\n");
	sprc = bms_suck_data(port);
	if (sprc)
		return sprc;

	/* And again hang up, \n at the beginning to separate <DS> from ATH */
	sprc = sp_blocking_write(port, "\nATH\r", 5, TIMEOUT_MS);
	if (sprc < 0)
		return sprc;

	//XXX
	printf(">>> sent ATH\n");
	sprc = bms_suck_data(port);

	printf(">>> suck_data():%d\n", sprc);


	return sprc;
}

struct bms_peer {
	uint8_t  addr[6];
	uint16_t signal;
};

static int bms_parse_peer(const char *buf, struct bms_peer *p)
{

	uint32_t something1, something2;
	int res;

	res = sscanf(buf, "RESP %hhx:%hhx:%hhx:%hhx:%hhx:%hhx %x %x %hd\n",
		     &p->addr[0], &p->addr[1], &p->addr[2],
		     &p->addr[3], &p->addr[4], &p->addr[5],
		     &something1, &something2, &p->signal);
	if (res != 9)
		return -EINVAL;

	return 0;
}

static int compar_peers(const void *a_, const void *b_)
{
	const struct bms_peer *a = a_;
	const struct bms_peer *b = b_;

	return a->signal < b->signal ? 1 :
	       a->signal > b->signal ? -1 : 0;
}

static enum sp_return bms_connect_nearest(struct sp_port *port)
{
	struct bms_peer peers[3];
	enum sp_return sprc;
	const int REPEAT_NUM = 5;
	int peers_num = 0;
	int repeat_num;
	char buf[128];

	int rc;

	//XXX
	printf(">> %s\n", __func__);

	repeat_num = REPEAT_NUM;

repeat:
	sprc = sp_blocking_write(port, "ATS\r", 4, TIMEOUT_MS);
	if (sprc < 0)
		return sprc;

	while (peers_num < ARRAY_SIZE(peers)) {
		sprc = bms_read_line(port, buf, sizeof(buf));

		//XXX
		printf(">> %s: read_line: %d\n", __func__, sprc);

		if (sprc == -ETIMEDOUT && peers_num)
			/* Use peers which were already discovered */
			break;

		if (sprc < 0)
			return sprc;

		if (!strncmp(buf, "ERR", 3))
			goto repeat_or_err;

		if (!strncmp(buf, "RESP", 4)) {
			rc = bms_parse_peer(buf, peers + peers_num);
			if (rc)
				goto repeat_or_err;
			peers_num++;
		}
		continue;

	repeat_or_err:
		if (!--repeat_num)
			return -EAGAIN;
		goto repeat;
	}
	sprc = sp_blocking_write(port, "ATH\r", 4, TIMEOUT_MS);
	if (sprc < 0)
		return sprc;

	sprc = bms_suck_data(port);
	if (sprc)
		return sprc;

	qsort(peers, ARRAY_SIZE(peers), sizeof(peers[0]), compar_peers);
	rc = snprintf(buf, sizeof(buf),
		      "ATD %02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx\r",
		      peers[0].addr[0], peers[0].addr[1], peers[0].addr[2],
		      peers[0].addr[3], peers[0].addr[4], peers[0].addr[5]);

	sprc = sp_blocking_write(port, buf, rc, TIMEOUT_MS);
	if (sprc < 0)
		return sprc;

	printf(">>> sent ATD\n");
	while (1) {
		sprc = bms_read_line(port, buf, sizeof(buf));
		if (sprc == -ETIMEDOUT) {
			if (!--repeat_num) {
				return 0;
			}
			continue;
		}
		if (sprc < 0)
			return sprc;

		if (!strncmp(buf, "ERR", 3) ||
		    !strncmp(buf, "NORX", 4))
			return 0;

		if (!strncmp(buf, "DATA", 4))
			break;
	}

	return 1;
}

struct bms_data {
	char something1[64];
	uint16_t charge_perc;
	char something2[40];
	uint32_t charge_volt;
	char something3[28];
	unsigned char crc;
} __attribute__((packed));

static enum sp_return bms_request_peer(struct sp_port *port,
				       uint16_t *charge_perc,
				       uint32_t *charge_volt)
{
	struct bms_data bms_data;
	unsigned char *buf = (unsigned char *)&bms_data;
	enum sp_return sprc;

	printf(">> send DATA SEQUENCE\n");

	sprc = sp_blocking_write(port, "\x09\x3F\x00\x93\x77\x01\x69\x40\xED",
				 9, TIMEOUT_MS);
	if (sprc < 0)
		return sprc;

	memset(&bms_data, 0, sizeof(bms_data));
	sprc = sp_blocking_read(port, &bms_data, sizeof(bms_data), TIMEOUT_MS);
	if (sprc < 0)
		return sprc;

	if (sprc != sizeof(bms_data) || crc8(buf, 138) != bms_data.crc)
		return 0;

	printf("READ: %d, sz %ld\n", sprc, sizeof(bms_data));
	printf("CRC : %x, expected %x\n", crc8(buf, 138), bms_data.crc);

	*charge_perc = be16toh(bms_data.charge_perc);
	*charge_volt = be32toh(bms_data.charge_volt);

	return 1;
}

static int devopen(const char *portname)
{
	struct sp_port *port;
	struct sp_port_config* spconf;
	enum sp_return sprc;

	sprc = sp_get_port_by_name(portname, &port);
	if (sprc)
		goto free_dev;

	sprc = sp_open(port, SP_MODE_READ_WRITE);
	if (sprc)
		goto free_port;

	sprc = sp_new_config(&spconf);
	if (sprc)
		goto close_port;

	sprc = sp_get_config(port, spconf);
	if (sprc)
		goto free_conf;

	sprc = sp_set_config_flowcontrol(spconf, SP_FLOWCONTROL_NONE);
	if (sprc)
		goto free_conf;

	sprc = sp_set_config_baudrate(spconf, 9600);
	if (sprc)
		goto free_conf;

	sprc = sp_set_config_bits(spconf, 8);
	if (sprc)
		goto free_conf;

	sprc = sp_set_config_stopbits(spconf, 1);
	if (sprc)
		goto free_conf;

	sprc = sp_set_config_parity(spconf, SP_PARITY_NONE);
	if (sprc)
		goto free_conf;

	sprc = sp_set_config(port, spconf);
	if (sprc)
		goto free_conf;

repeat:
	printf("START CONNECTING\n");

	sprc = bms_to_init_state(port);
	if (sprc)
		goto free_conf;

	sprc = bms_connect_nearest(port);
	if (sprc < 0)
		goto free_conf;
	if (sprc == 0)
		/* Nothing found */
		goto repeat;

	{
		uint16_t perc;
		uint32_t volt;

		sprc = bms_request_peer(port, &perc, &volt);
		if (sprc < 0)
			goto free_conf;
		if (sprc == 0)
			/* Nothing found */
			goto repeat;

		printf("PERC: %d\n", perc);
		printf("VOLT: %d\n", volt);
	}

	return 0;

free_conf:
	sp_free_config(spconf);
close_port:
	sp_close(port);
free_port:
	sp_free_port(port);
free_dev:

	return sprc;
}

int main()
{
	struct sp_port **ports = NULL;
	enum sp_return sprc;
	int i, rc;

	sprc = sp_list_ports(&ports);
	assert(sprc >= 0);

	for (i = 0; ports[i]; i++) {
		const char *desc, *name;

		desc = sp_get_port_description(ports[i]);
		name = sp_get_port_name(ports[i]);
		if (!desc || !name || strncasecmp(desc, "low energy dongle", 17))
			continue;

		printf(">> %s, %s\n", desc, name);

		while (1) {
			rc = devopen(name);
			printf(">> devopen(): %d\n", rc);
		}

	}
	sp_free_port_list(ports);

	return 0;
}

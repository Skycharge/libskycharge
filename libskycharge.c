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

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <ctype.h>
#include <sys/stat.h>

#include <uuid/uuid.h>

#include "libskycharge-pri.h"
#include "types.h"
#include "skyproto.h"

static struct sky_dev_ops *devops;

void __sky_register_devops(struct sky_dev_ops *ops)
{
	struct sky_dev_ops *other_ops;

	foreach_devops(other_ops, devops) {
		if (ops->contype == other_ops->contype) {
			sky_err("Duplicate ops type: %d\n", ops->contype);
			exit(1);
		}
	}
	ops->next = devops;
	devops = ops;
}

static const char hex_asc[] = "0123456789abcdef";

#define hex_asc_lo(x)	hex_asc[((x) & 0x0f)]
#define hex_asc_hi(x)	hex_asc[((x) & 0xf0) >> 4]

/**
 * hex_dump_to_buffer_oneline - convert a blob of data to "hex ASCII" in memory
 * @buf: data blob to dump
 * @len: number of bytes in the @buf
 * @rowsize: number of bytes to print per line; must be 16 or 32
 * @groupsize: number of bytes to print at a time (1, 2, 4, 8; default = 1)
 * @linebuf: where to put the converted data
 * @linebuflen: total size of @linebuf, including space for terminating NUL
 * @ascii: include ASCII after the hex output
 *
 * hex_dump_to_buffer() works on one "line" of output at a time, i.e.,
 * 16 or 32 bytes of input data converted to hex + ASCII output.
 *
 * Given a buffer of u8 data, hex_dump_to_buffer() converts the input data
 * to a hex + ASCII dump at the supplied memory location.
 * The converted output is always NUL-terminated.
 *
 * E.g.:
 *   hex_dump_to_buffer(frame->data, frame->len, 16, 1,
 *			linebuf, sizeof(linebuf), true);
 *
 * example output buffer:
 * 40 41 42 43 44 45 46 47 48 49 4a 4b 4c 4d 4e 4f  @ABCDEFGHIJKLMNO
 *
 * Return:
 * The amount of bytes placed in the buffer without terminating NUL. If the
 * output was truncated, then the return value is the number of bytes
 * (excluding the terminating NUL) which would have been written to the final
 * string if enough space had been available.
 */
int hex_dump_to_buffer_oneline(const void *buf, size_t len, int rowsize,
			       int groupsize, char *linebuf, size_t linebuflen,
			       bool ascii)
{
	const uint8_t *ptr = buf;
	int ngroups;
	uint8_t ch;
	int j, lx = 0;
	int ascii_column;
	int ret;

	if (rowsize != 16 && rowsize != 32)
		rowsize = 16;

	if (len > rowsize)		/* limit to one line at a time */
		len = rowsize;
	if (!is_power_of_2(groupsize) || groupsize > 8)
		groupsize = 1;
	if ((len % groupsize) != 0)	/* no mixed size output */
		groupsize = 1;

	ngroups = len / groupsize;
	ascii_column = rowsize * 2 + rowsize / groupsize + 1;

	if (!linebuflen)
		goto overflow1;

	if (!len)
		goto nil;

	if (groupsize == 8) {
		const uint64_t *ptr8 = buf;

		for (j = 0; j < ngroups; j++) {
			ret = snprintf(linebuf + lx, linebuflen - lx,
				       "%s%16.16" PRIx64, j ? " " : "",
				       *(ptr8 + j));
			if (ret >= linebuflen - lx)
				goto overflow1;
			lx += ret;
		}
	} else if (groupsize == 4) {
		const uint32_t *ptr4 = buf;

		for (j = 0; j < ngroups; j++) {
			ret = snprintf(linebuf + lx, linebuflen - lx,
				       "%s%8.8x", j ? " " : "",
				       *(ptr4 + j));
			if (ret >= linebuflen - lx)
				goto overflow1;
			lx += ret;
		}
	} else if (groupsize == 2) {
		const uint16_t *ptr2 = buf;

		for (j = 0; j < ngroups; j++) {
			ret = snprintf(linebuf + lx, linebuflen - lx,
				       "%s%4.4x", j ? " " : "",
				       *(ptr2 + j));
			if (ret >= linebuflen - lx)
				goto overflow1;
			lx += ret;
		}
	} else {
		for (j = 0; j < len; j++) {
			if (linebuflen < lx + 2)
				goto overflow2;
			ch = ptr[j];
			linebuf[lx++] = hex_asc_hi(ch);
			if (linebuflen < lx + 2)
				goto overflow2;
			linebuf[lx++] = hex_asc_lo(ch);
			if (linebuflen < lx + 2)
				goto overflow2;
			linebuf[lx++] = ' ';
		}
		if (j)
			lx--;
	}
	if (!ascii)
		goto nil;

	while (lx < ascii_column) {
		if (linebuflen < lx + 2)
			goto overflow2;
		linebuf[lx++] = ' ';
	}
	for (j = 0; j < len; j++) {
		if (linebuflen < lx + 2)
			goto overflow2;
		ch = ptr[j];
		linebuf[lx++] = (isascii(ch) && isprint(ch)) ? ch : '.';
	}
nil:
	linebuf[lx] = '\0';
	return lx;
overflow2:
	linebuf[lx++] = '\0';
overflow1:
	return ascii ? ascii_column + len : (groupsize * 2 + 1) * ngroups - 1;
}

int hex_dump_to_buffer(const void *buf, size_t len, int rowsize,
		       int groupsize, char *linebuf, size_t linebuflen,
		       bool ascii)
{
	int i, linelen, wr, written = 0, remaining = len;

	if (rowsize != 16 && rowsize != 32)
		rowsize = 16;

	for (i = 0; i < len && linebuflen; i += rowsize) {
		linelen = min(remaining, rowsize);
		remaining -= rowsize;

		wr = hex_dump_to_buffer_oneline(buf + i, linelen, rowsize,
						groupsize, linebuf, linebuflen,
						ascii);
		linebuf += wr;
		linebuflen -= wr;
		written += wr;

		if (linebuflen) {
			wr = snprintf(linebuf, linebuflen, "\n");
			linebuf += wr;
			linebuflen -= wr;
			written += wr;
		}
	}

	return written;
}

static void trim(char *s)
{
	char *new = s;

	while (*s) {
		if (!isspace(*s))
			*new++ = *s;
		s++;
	}
	*new = *s;
}

static bool is_dash_or_low_dash(unsigned char ch)
{
	return (ch == '-' || ch == '_');
}

static int strcasecmp_ignore_dashes(const char *s1, const char *s2)
{
	unsigned char ch1, ch2;

	do {
		ch1 = tolower(*s1++);
		ch2 = tolower(*s2++);

		if (is_dash_or_low_dash(ch1) && is_dash_or_low_dash(ch2))
			ch1 = ch2;

	} while (ch1 == ch2 && ch1 != 0);

	return ch1 - ch2;
}

static int parse_bool(const char *str, unsigned int *val)
{
	if (!strcasecmp(str, "false") ||
	    !strcasecmp(str, "no") ||
	    !strcasecmp(str, "0"))
		*val = 0;
	else if (!strcasecmp(str, "true") ||
		 !strcasecmp(str, "yes") ||
		 !strcasecmp(str, "1"))
		*val = 1;
	else
		return -EINVAL;

	return 0;
}

static int parse_unsigned(const char *str, unsigned int *val)
{
	int rc;

	rc = sscanf(str, "0x%x", val);
	if (rc == 0)
		rc = sscanf(str, "%u", val);

	return (rc == 0 ? -EINVAL : 0);
}

static int parse_psu_type(const char *str, enum sky_psu_type *psu_type)
{
	if (0 == strcasecmp(str, "rsp-750-48") ||
	    0 == strcasecmp(str, "rsp750-48") ||
	    0 == strcasecmp(str, "rsp-750") ||
	    0 == strcasecmp(str, "rsp750"))
		*psu_type = SKY_PSU_RSP_750_48;
	else if (0 == strcasecmp(str, "rsp-1600-48") ||
		 0 == strcasecmp(str, "rsp1600-48") ||
		 0 == strcasecmp(str, "rsp-1600") ||
		 0 == strcasecmp(str, "rsp1600"))
		*psu_type = SKY_PSU_RSP_1600_48;
	else
		return -EINVAL;

	return 0;
}

static int parse_batt_type(const char *str, enum sky_batt_type *batt_type)
{
	if (0 == strcasecmp(str, "li-po") ||
	    0 == strcasecmp(str, "lipo"))
		*batt_type = SKY_BATT_LIPO;
	else if (0 == strcasecmp(str, "li-ion") ||
		 0 == strcasecmp(str, "liion") ||
		 0 == strcasecmp(str, "lion"))
		*batt_type = SKY_BATT_LION;
	else
		return -EINVAL;

	return 0;
}

static int parse_detect_mode(const char *str, enum sky_detect_mode *detect_mode)
{
	if (0 == strcasecmp(str, "plc"))
		*detect_mode = SKY_DETECT_PLC;
	else if (0 == strcasecmp(str, "resistance"))
		*detect_mode = SKY_DETECT_RESISTANCE;
	else if (0 == strcasecmp(str, "capacity"))
		*detect_mode = SKY_DETECT_CAPACITY;
	else
		return -EINVAL;

	return 0;
}

static int parse_calib_point(const char *str, uint32_t *v)
{
	uint16_t set, read;
	int rc;

	rc = sscanf(str, "%hu:%hu", &set, &read);
	if (rc != 2)
		return -EINVAL;

	*v = calib_point_to_uint32(set, read);

	return 0;
}

static char *devparam_to_config_str(enum sky_dev_type dev_type,
				    enum sky_dev_param param,
				    char *buf, size_t len)
{
	const char *str;
	int i, ret;

	str = sky_devparam_to_str(dev_type, param);
	ret = snprintf(buf, len, "mux-hw%d-%s=", dev_type + 1, str);
	if (ret < 0)
		return NULL;

	for (i = 0; i <= ret; i++) {
		if (buf[i] == '_')
			buf[i] = '-';
		else
			buf[i] = tolower((unsigned char) buf[i]);
	}

	return buf;
}

static char *sinkparam_to_config_str(enum sky_dev_param param,
				     char *buf, size_t len)
{
	const char *str;
	int i, ret;

	str = sky_sinkparam_to_str(param);
	ret = snprintf(buf, len, "sink-%s=", str);
	if (ret < 0)
		return NULL;

	for (i = 0; i <= ret; i++) {
		if (buf[i] == '_')
			buf[i] = '-';
		else
			buf[i] = tolower((unsigned char) buf[i]);
	}

	return buf;
}

typedef int (parse_fn_t)(const char *line, uint32_t *param_value);

static bool parse_devparam(const char *line,
			   enum sky_dev_type dev_type,
			   enum sky_dev_param param,
			   struct sky_dev_params *params,
			   parse_fn_t *parse_fn,
			   int *ret)
{
	const char *configstr;
	char buf[256];
	int rc;

	configstr = devparam_to_config_str(dev_type, param, buf, sizeof(buf));
	if (!configstr)
		return false;
	if (!strcasestr(line, configstr))
		return false;

	rc = parse_fn(line + strlen(configstr), &params->dev_params[param]);
	if (!rc)
		params->dev_params_bits |= 1<<param;

	*ret = rc;

	return true;
}

static bool parse_sinkparam(const char *line,
			   enum sky_dev_param param,
			   struct sky_dev_params *params,
			   parse_fn_t *parse_fn,
			   int *ret)
{
	const char *configstr;
	char buf[256];
	int rc;

	configstr = sinkparam_to_config_str(param, buf, sizeof(buf));
	if (!configstr)
		return false;
	if (!strcasestr(line, configstr))
		return false;

	rc = parse_fn(line + strlen(configstr), &params->dev_params[param]);
	if (!rc)
		params->dev_params_bits |= 1<<param;

	*ret = rc;

	return true;
}

static int parse_line(char *line, struct sky_conf *cfg)
{
	char *str;
	int rc, ret = 0;

	trim(line);
	*(strchrnul(line, '#')) = '\0';

	/*
	 * Generic config
	 */

	if ((str = strstr(line, "user-uuid="))) {
		rc = uuid_parse(str + 10, cfg->usruuid);
		if (rc)
			return -ENODATA;

	} else if ((str = strstr(line, "device-uuid="))) {
		rc = uuid_parse(str + 12, cfg->devuuid);
		if (rc)
			return -ENODATA;

	} else if ((str = strstr(line, "device-name="))) {
		strncpy(cfg->devname, str + 12, sizeof(cfg->devname) - 1);

	} else if ((str = strstr(line, "device-is-dummy="))) {
		unsigned is_dummy;
		rc = sscanf(str + 16, "%d", &is_dummy);
		if (rc != 1)
			return -ENODATA;

		if (is_dummy)
			cfg->contype = SKY_DUMMY;

	} else if ((str = strstr(line, "broker-url="))) {
		rc = sscanf(str + 11, "%64[^:]:%u,%u", cfg->hostname,
			    &cfg->srvport, &cfg->cliport);
		if (rc != 3)
			return -ENODATA;

		cfg->subport = cfg->srvport + 1;
		cfg->pubport = cfg->cliport + 1;
	}

	/*
	 * MUX common config
	 */

	else if ((str = strstr(line, "mux-type="))) {
		if (0 == strcasecmp(str + 9, "hw1"))
			cfg->mux_type = SKY_MUX_HW1;
		else if (0 == strcasecmp(str + 9, "hw2"))
			cfg->mux_type = SKY_MUX_HW2;
		else
			return -ENODATA;
	} else if ((str = strstr(line, "mux-dev="))) {
		strncpy(cfg->mux_dev, str + 8, sizeof(cfg->mux_dev) - 1);

	/*
	 * MUX HW1 config
	 */

	} else if (parse_devparam(line, SKY_MUX_HW1, SKY_HW1_EEPROM_INITED,
				  &cfg->mux_hw1_params, parse_unsigned, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW1, SKY_HW1_SCANNING_INTERVAL,
				  &cfg->mux_hw1_params, parse_unsigned, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW1, SKY_HW1_PRECHARGING_INTERVAL,
				  &cfg->mux_hw1_params, parse_unsigned, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW1, SKY_HW1_PRECHARGING_COUNTER,
				  &cfg->mux_hw1_params, parse_unsigned, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW1, SKY_HW1_POSTCHARGING_INTERVAL,
				  &cfg->mux_hw1_params, parse_unsigned, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW1, SKY_HW1_POSTCHARGING_DELAY,
				  &cfg->mux_hw1_params, parse_unsigned, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW1, SKY_HW1_WET_DELAY,
				  &cfg->mux_hw1_params, parse_unsigned, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW1, SKY_HW1_SHORTCIRC_DELAY,
				  &cfg->mux_hw1_params, parse_unsigned, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW1, SKY_HW1_THRESH_FINISH_CHARGING,
				  &cfg->mux_hw1_params, parse_unsigned, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW1, SKY_HW1_THRESH_NOCHARGER_PRESENT,
				  &cfg->mux_hw1_params, parse_unsigned, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW1, SKY_HW1_THRESH_SHORTCIRC,
				  &cfg->mux_hw1_params, parse_unsigned, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW1, SKY_HW1_CURRENT_MON_INTERVAL,
				  &cfg->mux_hw1_params, parse_unsigned, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW1, SKY_HW1_WAIT_START_CHARGING_SEC,
				  &cfg->mux_hw1_params, parse_unsigned, &ret)) {
	}

	/*
	 * MUX HW2 config
	 */

	else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_PSU_TYPE,
				&cfg->mux_hw2_params, parse_psu_type, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_DETECT_MODE,
				  &cfg->mux_hw2_params, parse_detect_mode, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_PSU_FIXED_VOLTAGE_MV,
				  &cfg->mux_hw2_params, parse_unsigned, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_PSU_FIXED_CURRENT_MA,
				  &cfg->mux_hw2_params, parse_unsigned, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_NR_BAD_HEARTBEATS,
				  &cfg->mux_hw2_params, parse_unsigned, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_IGNORE_INVAL_CHARGING_SETTINGS,
				  &cfg->mux_hw2_params, parse_bool, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_IGNORE_LOW_BATT_VOLTAGE,
				  &cfg->mux_hw2_params, parse_bool, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_ERROR_INDICATION_TIMEOUT_SECS,
				  &cfg->mux_hw2_params, parse_unsigned, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_KEEP_SILENCE,
				  &cfg->mux_hw2_params, parse_bool, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_USE_FIXED_V_I,
				  &cfg->mux_hw2_params, parse_bool, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_IGNORE_VOLTAGE_ON_OUTPUT,
				  &cfg->mux_hw2_params, parse_bool, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_MIN_SENSE_CURRENT_MA,
				  &cfg->mux_hw2_params, parse_unsigned, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_REPEAT_CHARGE_AFTER_MINS,
				  &cfg->mux_hw2_params, parse_unsigned, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_SENSE_VOLTAGE_CALIB_POINT1_MV,
				  &cfg->mux_hw2_params, parse_calib_point, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_SENSE_VOLTAGE_CALIB_POINT2_MV,
				  &cfg->mux_hw2_params, parse_calib_point, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_SENSE_CURRENT_CALIB_POINT1_MA,
				  &cfg->mux_hw2_params, parse_calib_point, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_SENSE_CURRENT_CALIB_POINT2_MA,
				  &cfg->mux_hw2_params, parse_calib_point, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_PSU_VOLTAGE_CALIB_POINT1_MV,
				  &cfg->mux_hw2_params, parse_calib_point, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_PSU_VOLTAGE_CALIB_POINT2_MV,
				  &cfg->mux_hw2_params, parse_calib_point, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_PSU_CURRENT_CALIB_POINT1_MA,
				  &cfg->mux_hw2_params, parse_calib_point, &ret)) {

	} else if (parse_devparam(line, SKY_MUX_HW2, SKY_HW2_PSU_CURRENT_CALIB_POINT2_MA,
				  &cfg->mux_hw2_params, parse_calib_point, &ret)) {
	}

	/*
	 * MUX HW2 sink config (makes sense only for RESISTANCE/CAPACITY
	 * detection modes)
	 */

	else if (parse_sinkparam(line, SKY_SINK_BATT_TYPE,
				 &cfg->sink_params, parse_batt_type, &ret)) {

	} else if (parse_sinkparam(line, SKY_SINK_BATT_CAPACITY_MAH,
				   &cfg->sink_params, parse_unsigned, &ret)) {

	} else if (parse_sinkparam(line, SKY_SINK_BATT_MIN_VOLTAGE_MV,
				   &cfg->sink_params, parse_unsigned, &ret)) {

	} else if (parse_sinkparam(line, SKY_SINK_BATT_MAX_VOLTAGE_MV,
				   &cfg->sink_params, parse_unsigned, &ret)) {

	} else if (parse_sinkparam(line, SKY_SINK_CHARGING_MAX_CURRENT_MA,
				   &cfg->sink_params, parse_unsigned, &ret)) {

	} else if (parse_sinkparam(line, SKY_SINK_CUTOFF_MIN_CURRENT_MA,
				   &cfg->sink_params, parse_unsigned, &ret)) {

	} else if (parse_sinkparam(line, SKY_SINK_CUTOFF_TIMEOUT_MS,
				   &cfg->sink_params, parse_unsigned, &ret)) {

	} else if (parse_sinkparam(line, SKY_SINK_PRECHARGE_CURRENT_MA,
				   &cfg->sink_params, parse_unsigned, &ret)) {

	} else if (parse_sinkparam(line, SKY_SINK_PRECHARGE_DELAY_SECS,
				   &cfg->sink_params, parse_unsigned, &ret)) {

	} else if (parse_sinkparam(line, SKY_SINK_PRECHARGE_SECS,
				   &cfg->sink_params, parse_unsigned, &ret)) {

	} else if (parse_sinkparam(line, SKY_SINK_TOTAL_CHARGE_SECS,
				   &cfg->sink_params, parse_unsigned, &ret)) {
	}

	/*
	 * PSU HW1 config
	 */

	else if ((str = strstr(line, "psu-type="))) {
		rc = parse_psu_type(str + 9, &cfg->psu.type);
		if (rc)
			return -ENODATA;

		if (cfg->psu.type == SKY_PSU_RSP_1600_48)
			/* Not supported */
			return -ENODATA;

	} else if ((str = strstr(line, "psu-voltage="))) {
		rc = sscanf(str + 12, "%f", &cfg->psu.voltage);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "psu-current="))) {
		rc = sscanf(str + 12, "%f", &cfg->psu.current);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "psu-precharge-current="))) {
		rc = sscanf(str + 22, "%f", &cfg->psu.precharge_current);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "psu-precharge-current-coef="))) {
		rc = sscanf(str + 27, "%f", &cfg->psu.precharge_current_coef);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "psu-precharge-secs="))) {
		rc = sscanf(str + 19, "%u", &cfg->psu.precharge_secs);
		if (rc != 1)
			return -ENODATA;

	}

	/*
	 * Drone port config
	 */

	else if ((str = strstr(line, "dp-hw-interface="))) {
		char buf[16];

		strncpy(buf, str + 16, sizeof(buf) - 1);
		if (!strcmp(buf, "gpio"))
			cfg->dp.hw_interface = SKY_DP_GPIO;
		else
			return -ENODATA;

	} else if ((str = strstr(line, "dp-open-pin="))) {
		rc = sscanf(str + 12, "%8s", cfg->dp.gpio.open_pin);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "dp-close-pin="))) {
		rc = sscanf(str + 13, "%8s", cfg->dp.gpio.close_pin);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "dp-is-opened-pin="))) {
		rc = sscanf(str + 17, "%8s", cfg->dp.gpio.is_opened_pin);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "dp-is-closed-pin="))) {
		rc = sscanf(str + 17, "%8s", cfg->dp.gpio.is_closed_pin);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "dp-in-progress-pin="))) {
		rc = sscanf(str + 19, "%8s", cfg->dp.gpio.in_progress_pin);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "dp-is-landing-err-pin="))) {
		rc = sscanf(str + 22, "%8s", cfg->dp.gpio.is_landing_err_pin);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "dp-is-ready-pin="))) {
		rc = sscanf(str + 16, "%8s", cfg->dp.gpio.is_ready_pin);
		if (rc != 1)
			return -ENODATA;

	} else if ((str = strstr(line, "dp-is-drone-detected-pin="))) {
		rc = sscanf(str + 25, "%8s", cfg->dp.gpio.is_drone_detected_pin);
		if (rc != 1)
			return -ENODATA;
	}

	/*
	 * No else: ignore unknown parameters
	 */

	return ret;
}

static const char *sky_hw1_devstate_to_str(enum sky_dev_state state)
{
	switch(state) {
	case SKY_HW1_UNKNOWN:
		return "UNKNOWN";
	case SKY_HW1_SCANNING_INIT:
		return "SCANNING_INIT";
	case SKY_HW1_SCANNING_RUN:
		return "SCANNING_RUN";
	case SKY_HW1_SCANNING_CHECK_MATRIX:
		return "SCANNING_CHECK_MATRIX";
	case SKY_HW1_SCANNING_PRINT:
		return "SCANNING_PRINT";
	case SKY_HW1_SCANNING_CHECK_WATER:
		return "SCANNING_CHECK_WATER";
	case SKY_HW1_SCANNING_WET:
		return "SCANNING_WET";
	case SKY_HW1_SCANNING_DETECTING:
		return "SCANNING_DETECTING";
	case SKY_HW1_PRE_CHARGING_INIT:
		return "PRE_CHARGING_INIT";
	case SKY_HW1_PRE_CHARGING_RUN:
		return "PRE_CHARGING_RUN";
	case SKY_HW1_PRE_CHARGING_CHECK_MATRIX:
		return "PRE_CHARGING_CHECK_MATRIX";
	case SKY_HW1_PRE_CHARGING_PRINT:
		return "PRE_CHARGING_PRINT";
	case SKY_HW1_PRE_CHARGING_CHECK_WATER:
		return "PRE_CHARGING_CHECK_WATER";
	case SKY_HW1_PRE_CHARGING_WET:
		return "PRE_CHARGING_WET";
	case SKY_HW1_PRE_CHARGING_FIND_CHARGERS:
		return "PRE_CHARGING_FIND_CHARGERS";
	case SKY_HW1_CHARGING_INIT:
		return "CHARGING_INIT";
	case SKY_HW1_CHARGING_RUN:
		return "CHARGING_RUN";
	case SKY_HW1_CHARGING_MONITOR_CURRENT:
		return "CHARGING_MONITOR_CURRENT";
	case SKY_HW1_POST_CHARGING_INIT:
		return "POST_CHARGING_INIT";
	case SKY_HW1_POST_CHARGING_RUN:
		return "POST_CHARGING_RUN";
	case SKY_HW1_POST_CHARGING_CHECK_MATRIX:
		return "POST_CHARGING_CHECK_MATRIX";
	case SKY_HW1_POST_CHARGING_PRINT:
		return "POST_CHARGING_PRINT";
	case SKY_HW1_POST_CHARGING_CHECK_WATER:
		return "POST_CHARGING_CHECK_WATER";
	case SKY_HW1_POST_CHARGING_WET:
		return "POST_CHARGING_WET";
	case SKY_HW1_POST_CHARGING_FIND_CHARGERS:
		return "POST_CHARGING_FIND_CHARGERS";
	case SKY_HW1_OVERLOAD:
		return "OVERLOAD";
	case SKY_HW1_AUTOSCAN_DISABLED:
		return "AUTOSCAN_DISABLED";
	default:
		sky_err("unknown state: %d\n", state);
		return "UNKNOWN";
	}
}

static const char *sky_hw2_devstate_to_str(enum sky_dev_state state)
{
	switch(state) {
	case SKY_HW2_STOPPED:
		return "STOPPED";
	case SKY_HW2_SCANNING:
		return "SCANNING";
	case SKY_HW2_LINK_ESTABLISHED:
		return "LINK_ESTABLISHED";
	case SKY_HW2_PRECHARGE_DELAYED:
		return "PRECHARGE_DELAYED";
	case SKY_HW2_PRECHARGING:
		return "PRECHARGING";
	case SKY_HW2_CHARGING:
		return "CHARGING";
	case SKY_HW2_CHARGING_FINISHED:
		return "CHARGING_FINISHED";
	case SKY_HW2_ERR_INVAL_CHARGING_SETTINGS:
		return "ERR_INVAL_CHARGING_SETTINGS";
	case SKY_HW2_ERR_BAD_LINK:
		return "ERR_BAD_LINK";
	case SKY_HW2_ERR_LOW_BATT_VOLTAGE:
		return "ERR_LOW_BATT_VOLTAGE";
	case SKY_HW2_ERR_LOW_MUX_VOLTAGE:
		return "ERR_LOW_MUX_VOLTAGE";
	case SKY_HW2_ERR_VOLTAGE_ON_OUTPUT:
		return "ERR_VOLTAGE_ON_OUTPUT";
	default:
		sky_err("unknown state: %d\n", state);
		return "UNKNOWN";
	}
}

const char *sky_devstate_to_str(enum sky_dev_type dev_type,
				enum sky_dev_state state)
{
	if (dev_type == SKY_MUX_HW1)
		return sky_hw1_devstate_to_str(state);

	return sky_hw2_devstate_to_str(state);
}

static const char *sky_hw1_devparam_to_str(enum sky_dev_param param)
{
	switch(param) {
	case SKY_HW1_EEPROM_INITED:
		return "eeprom-inited";
	case SKY_HW1_SCANNING_INTERVAL:
		return "scanning-interval";
	case SKY_HW1_PRECHARGING_INTERVAL:
		return "precharging-interval";
	case SKY_HW1_PRECHARGING_COUNTER:
		return "precharging-counter";
	case SKY_HW1_POSTCHARGING_INTERVAL:
		return "postcharging-interval";
	case SKY_HW1_POSTCHARGING_DELAY:
		return "postcharging-delay";
	case SKY_HW1_WET_DELAY:
		return "wet-delay";
	case SKY_HW1_SHORTCIRC_DELAY:
		return "shortcirc-delay";
	case SKY_HW1_THRESH_FINISH_CHARGING:
		return "thresh-finish-charging";
	case SKY_HW1_THRESH_NOCHARGER_PRESENT:
		return "thresh-nocharger-present";
	case SKY_HW1_THRESH_SHORTCIRC:
		return "thresh-shortcirc";
	case SKY_HW1_CURRENT_MON_INTERVAL:
		return "current-mon-interval";
	case SKY_HW1_WAIT_START_CHARGING_SEC:
		return "wait-start-charging-sec";
	default:
		sky_err("unknown param: %d\n", param);
		return "unknown-param";
	}
}

static enum sky_dev_param sky_hw1_devparam_from_str(const char *str)
{
	const char *paramstr;
	int p;

	for (p = 0; p < SKY_HW1_NUM_DEVPARAM; p++) {
		paramstr = sky_hw1_devparam_to_str(p);

		if (!strcasecmp_ignore_dashes(paramstr, str))
			return p;
	}

	/* No luck */
	return SKY_HW1_NUM_DEVPARAM;
}

static int
sky_hw1_devparam_value_to_str(enum sky_dev_param param,
			      const struct sky_dev_params *params,
			      enum sky_param_value_format value_format,
			      char *buf, size_t size)
{
	(void)value_format;

	if (param >= SKY_HW1_NUM_DEVPARAM)
		return -EINVAL;

	return snprintf(buf, size, "%u", params->dev_params[param]);
}

static int sky_hw1_devparam_value_from_str(const char *str,
					   enum sky_dev_param param,
					   struct sky_dev_params *params)
{
	unsigned int v;
	int rc;

	if (param >= SKY_HW1_NUM_DEVPARAM)
		return -EINVAL;

	rc = parse_unsigned(str, &v);
	if (rc)
		return -EINVAL;

	params->dev_params_bits |= (1 << param);
	params->dev_params[param] = v;

	return 0;
}

static const char *sky_hw2_devparam_to_str(enum sky_dev_param param)
{
	switch(param) {
	case SKY_HW2_PSU_TYPE:
		return "psu-type";
	case SKY_HW2_DETECT_MODE:
		return "detect-mode";
	case SKY_HW2_PSU_FIXED_VOLTAGE_MV:
		return "psu-fixed-voltage-mv";
	case SKY_HW2_PSU_FIXED_CURRENT_MA:
		return "psu-fixed-current-ma";
	case SKY_HW2_NR_BAD_HEARTBEATS:
		return "nr-bad-heartbeats";
	case SKY_HW2_IGNORE_INVAL_CHARGING_SETTINGS:
		return "ignore-inval-charging-settings";
	case SKY_HW2_IGNORE_LOW_BATT_VOLTAGE:
		return "ignore-low-batt-voltage";
	case SKY_HW2_ERROR_INDICATION_TIMEOUT_SECS:
		return "error-indication-timeout-secs";
	case SKY_HW2_KEEP_SILENCE:
		return "keep-silence";
	case SKY_HW2_USE_FIXED_V_I:
		return "psu-use-fixed-v-i";
	case SKY_HW2_IGNORE_VOLTAGE_ON_OUTPUT:
		return "ignore-voltage-on-output";
	case SKY_HW2_MIN_SENSE_CURRENT_MA:
		return "min-sense-current-ma";
	case SKY_HW2_REPEAT_CHARGE_AFTER_MINS:
		return "repeat-charge-after-mins";
	case SKY_HW2_SENSE_VOLTAGE_CALIB_POINT1_MV:
		return "sense-voltage-calib-point1-mv";
	case SKY_HW2_SENSE_VOLTAGE_CALIB_POINT2_MV:
		return "sense-voltage-calib-point2-mv";
	case SKY_HW2_SENSE_CURRENT_CALIB_POINT1_MA:
		return "sense-current-calib-point1-ma";
	case SKY_HW2_SENSE_CURRENT_CALIB_POINT2_MA:
		return "sense-current-calib-point2-ma";
	case SKY_HW2_PSU_VOLTAGE_CALIB_POINT1_MV:
		return "psu-voltage-calib-point1-mv";
	case SKY_HW2_PSU_VOLTAGE_CALIB_POINT2_MV:
		return "psu-voltage-calib-point2-mv";
	case SKY_HW2_PSU_CURRENT_CALIB_POINT1_MA:
		return "psu-current-calib-point1-ma";
	case SKY_HW2_PSU_CURRENT_CALIB_POINT2_MA:
		return "psu-current-calib-point2-ma";
	default:
		sky_err("unknown param: %d\n", param);
		return "unknown-param";
	}
}

static enum sky_dev_param sky_hw2_devparam_from_str(const char *str)
{
	const char *paramstr;
	int p;

	for (p = 0; p < SKY_HW2_NUM_DEVPARAM; p++) {
		paramstr = sky_hw2_devparam_to_str(p);

		if (!strcasecmp_ignore_dashes(paramstr, str))
			return p;
	}

	/* No luck */
	return SKY_HW2_NUM_DEVPARAM;
}

static int
sky_hw2_devparam_value_to_str(enum sky_dev_param param,
			      const struct sky_dev_params *params,
			      enum sky_param_value_format value_format,
			      char *buf, size_t size)
{
	uint32_t v;

	if (param >= SKY_HW2_NUM_DEVPARAM)
		return -EINVAL;

	v = params->dev_params[param];
	switch(param) {
	case SKY_HW2_PSU_TYPE:
		switch (v) {
		case SKY_PSU_RSP_750_48:
			if (value_format == SKY_PARAM_VALUE_TEXT_AND_NUMERIC)
				return snprintf(buf, size, "RSP-750-48 (0x%02x)", v);
			else
				return snprintf(buf, size, "RSP-750-48");
		case SKY_PSU_RSP_1600_48:
			if (value_format == SKY_PARAM_VALUE_TEXT_AND_NUMERIC)
				return snprintf(buf, size, "RSP-1600-48 (0x%02x)", v);
			else
				return snprintf(buf, size, "RSP-1600-48");
		default:
			if (value_format == SKY_PARAM_VALUE_TEXT_AND_NUMERIC)
				return snprintf(buf, size, "unknown psu (0x%02x)", v);
			else
				return snprintf(buf, size, "unknown psu");
		}
	case SKY_HW2_DETECT_MODE:
		switch (v) {
		case SKY_DETECT_PLC:
			if (value_format == SKY_PARAM_VALUE_TEXT_AND_NUMERIC)
				return snprintf(buf, size, "PLC (0x%02x)", v);
			else
				return snprintf(buf, size, "PLC");
		case SKY_DETECT_RESISTANCE:
			if (value_format == SKY_PARAM_VALUE_TEXT_AND_NUMERIC)
				return snprintf(buf, size, "RESISTANCE (0x%02x)", v);
			else
				return snprintf(buf, size, "RESISTANCE");
		case SKY_DETECT_CAPACITY:
			if (value_format == SKY_PARAM_VALUE_TEXT_AND_NUMERIC)
				return snprintf(buf, size, "CAPACITY (0x%02x)", v);
			else
				return snprintf(buf, size, "CAPACITY");
		default:
			if (value_format == SKY_PARAM_VALUE_TEXT_AND_NUMERIC)
				return snprintf(buf, size, "unknown mode (0x%02x)", v);
			else
				return snprintf(buf, size, "unknown mode");
		}
	case SKY_HW2_NR_BAD_HEARTBEATS:
	case SKY_HW2_ERROR_INDICATION_TIMEOUT_SECS:
	case SKY_HW2_PSU_FIXED_VOLTAGE_MV:
	case SKY_HW2_PSU_FIXED_CURRENT_MA:
	case SKY_HW2_MIN_SENSE_CURRENT_MA:
	case SKY_HW2_REPEAT_CHARGE_AFTER_MINS:
		return snprintf(buf, size, "%u", v);
	case SKY_HW2_SENSE_VOLTAGE_CALIB_POINT1_MV:
	case SKY_HW2_SENSE_VOLTAGE_CALIB_POINT2_MV:
	case SKY_HW2_SENSE_CURRENT_CALIB_POINT1_MA:
	case SKY_HW2_SENSE_CURRENT_CALIB_POINT2_MA:
	case SKY_HW2_PSU_VOLTAGE_CALIB_POINT1_MV:
	case SKY_HW2_PSU_VOLTAGE_CALIB_POINT2_MV:
	case SKY_HW2_PSU_CURRENT_CALIB_POINT1_MA:
	case SKY_HW2_PSU_CURRENT_CALIB_POINT2_MA: {
		uint16_t set, read;

		uint32_to_calib_point(v, &set, &read);
		if (value_format == SKY_PARAM_VALUE_TEXT_AND_NUMERIC)
			return snprintf(buf, size, "%5u:%u", set, read);
		else
			return snprintf(buf, size, "%u:%u", set, read);
	}
	case SKY_HW2_IGNORE_INVAL_CHARGING_SETTINGS:
	case SKY_HW2_IGNORE_LOW_BATT_VOLTAGE:
	case SKY_HW2_IGNORE_VOLTAGE_ON_OUTPUT:
	case SKY_HW2_KEEP_SILENCE:
	case SKY_HW2_USE_FIXED_V_I:
		return snprintf(buf, size, v ? "true" : "false");
	default:
		if (value_format == SKY_PARAM_VALUE_TEXT_AND_NUMERIC)
			return snprintf(buf, size, "unknown param (0x%02x)", v);
		else
			return snprintf(buf, size, "unknown param");
	}
}

static int sky_hw2_devparam_value_from_str(const char *str,
					   enum sky_dev_param param,
					   struct sky_dev_params *params)
{
	unsigned int v = 0;
	int rc = 0;

	if (param >= SKY_HW2_NUM_DEVPARAM)
		return -EINVAL;

	switch(param) {
	case SKY_HW2_PSU_TYPE: {
		enum sky_psu_type psu_type;

		if (parse_psu_type(str, &psu_type)) {
			if (parse_unsigned(str, &v))
				return -EINVAL;
			if (v != SKY_PSU_RSP_750_48 &&
			    v != SKY_PSU_RSP_1600_48)
				return -EINVAL;
		} else {
			v = psu_type;
		}
		break;
	}
	case SKY_HW2_DETECT_MODE: {
		enum sky_detect_mode detect_mode;

		if (parse_detect_mode(str, &detect_mode)) {
			if (parse_unsigned(str, &v))
				return -EINVAL;
			if (v != SKY_DETECT_PLC &&
			    v != SKY_DETECT_RESISTANCE &&
			    v != SKY_DETECT_CAPACITY)
				return -EINVAL;
		} else {
			v = detect_mode;
		}
		break;
	}
	case SKY_HW2_NR_BAD_HEARTBEATS:
	case SKY_HW2_ERROR_INDICATION_TIMEOUT_SECS:
	case SKY_HW2_PSU_FIXED_VOLTAGE_MV:
	case SKY_HW2_PSU_FIXED_CURRENT_MA:
	case SKY_HW2_MIN_SENSE_CURRENT_MA:
	case SKY_HW2_REPEAT_CHARGE_AFTER_MINS:
		rc = parse_unsigned(str, &v);
		break;
	case SKY_HW2_SENSE_VOLTAGE_CALIB_POINT1_MV:
	case SKY_HW2_SENSE_VOLTAGE_CALIB_POINT2_MV:
	case SKY_HW2_SENSE_CURRENT_CALIB_POINT1_MA:
	case SKY_HW2_SENSE_CURRENT_CALIB_POINT2_MA:
	case SKY_HW2_PSU_VOLTAGE_CALIB_POINT1_MV:
	case SKY_HW2_PSU_VOLTAGE_CALIB_POINT2_MV:
	case SKY_HW2_PSU_CURRENT_CALIB_POINT1_MA:
	case SKY_HW2_PSU_CURRENT_CALIB_POINT2_MA: {
		rc = parse_calib_point(str, &v);
		break;
	}
	case SKY_HW2_IGNORE_INVAL_CHARGING_SETTINGS:
	case SKY_HW2_IGNORE_LOW_BATT_VOLTAGE:
	case SKY_HW2_IGNORE_VOLTAGE_ON_OUTPUT:
	case SKY_HW2_KEEP_SILENCE:
	case SKY_HW2_USE_FIXED_V_I:
		rc = parse_bool(str, &v);
		break;
	default:
		return -EINVAL;
	}

	if (rc)
		return -EINVAL;

	params->dev_params_bits |= (1 << param);
	params->dev_params[param] = v;

	return 0;
}

const char *sky_devparam_to_str(enum sky_dev_type dev_type,
				enum sky_dev_param param)
{
	if (dev_type == SKY_MUX_HW1)
		return sky_hw1_devparam_to_str(param);

	return sky_hw2_devparam_to_str(param);
}

enum sky_dev_param sky_devparam_from_str(enum sky_dev_type dev_type,
					 const char *str)
{
	if (dev_type == SKY_MUX_HW1)
		return sky_hw1_devparam_from_str(str);

	return sky_hw2_devparam_from_str(str);
}

int sky_devparam_value_to_str(enum sky_dev_type dev_type,
			      enum sky_dev_param param,
			      const struct sky_dev_params *params,
			      enum sky_param_value_format value_format,
			      char *buf, size_t size)
{
	if (dev_type == SKY_MUX_HW1)
		return sky_hw1_devparam_value_to_str(param, params, value_format,
						     buf, size);

	return sky_hw2_devparam_value_to_str(param, params, value_format,
					     buf, size);
}

int sky_devparam_value_from_str(const char *str,
				enum sky_dev_type dev_type,
				enum sky_dev_param param,
				struct sky_dev_params *params)
{
	if (dev_type == SKY_MUX_HW1)
		return sky_hw1_devparam_value_from_str(str, param, params);

	return sky_hw2_devparam_value_from_str(str, param, params);
}

const char *sky_sinkparam_to_str(enum sky_sink_param param)
{
	switch(param) {
	case SKY_SINK_CAPABILITIES:
		return "capabilties";
	case SKY_SINK_BATT_TYPE:
		return "batt-type";
	case SKY_SINK_BATT_CAPACITY_MAH:
		return "batt-capacity-mah";
	case SKY_SINK_BATT_MIN_VOLTAGE_MV:
		return "batt-min-voltage-mv";
	case SKY_SINK_BATT_MAX_VOLTAGE_MV:
		return "batt-max-voltage-mv";
	case SKY_SINK_CHARGING_MAX_CURRENT_MA:
		return "charging-max-current-ma";
	case SKY_SINK_CUTOFF_MIN_CURRENT_MA:
		return "cutoff-min-current-ma";
	case SKY_SINK_CUTOFF_TIMEOUT_MS:
		return "cutoff-timeout-ms";
	case SKY_SINK_PRECHARGE_CURRENT_MA:
		return "precharge-current-ma";
	case SKY_SINK_PRECHARGE_DELAY_SECS:
		return "precharge-delay-secs";
	case SKY_SINK_PRECHARGE_SECS:
		return "precharge-secs";
	case SKY_SINK_TOTAL_CHARGE_SECS:
		return "total-charge-secs";
	case SKY_SINK_USER_DATA1:
		return "user-data1";
	case SKY_SINK_USER_DATA2:
		return "user-data2";
	case SKY_SINK_USER_DATA3:
		return "user-data3";
	case SKY_SINK_USER_DATA4:
		return "user-data4";
	default:
		sky_err("unknown param: %d\n", param);
		return "unknown-param";
	}
}

enum sky_sink_param sky_sinkparam_from_str(const char *str)
{
	const char *paramstr;
	int p;

	for (p = 0; p < SKY_SINK_NUM_DEVPARAM; p++) {
		paramstr = sky_sinkparam_to_str(p);

		if (!strcasecmp_ignore_dashes(paramstr, str))
			return p;
	}

	/* No luck */
	return SKY_SINK_NUM_DEVPARAM;
}

int sky_sinkparam_value_to_str(enum sky_sink_param param,
			       const struct sky_dev_params *params,
			       enum sky_param_value_format value_format,
			       char *buf, size_t size)
{
	uint32_t v;

	if (param >= SKY_SINK_NUM_DEVPARAM)
		return -EINVAL;

	v = params->dev_params[param];
	switch(param) {
	case SKY_SINK_CAPABILITIES:
		if (value_format == SKY_PARAM_VALUE_TEXT_AND_NUMERIC)
			return snprintf(buf, size, "%s (0x%02x)",
					v & (1 << SKY_CAP_PLC_WHILE_CHARGING) ?
					"PLC_WHILE_CHARGING" : "NIL", v);
		else
			return snprintf(buf, size, "%s",
					v & (1 << SKY_CAP_PLC_WHILE_CHARGING) ?
					"PLC_WHILE_CHARGING" : "NIL");
	case SKY_SINK_BATT_TYPE:
		switch (v) {
		case SKY_BATT_LIPO:
			if (value_format == SKY_PARAM_VALUE_TEXT_AND_NUMERIC)
				return snprintf(buf, size, "Li-Po (0x%02x)", v);
			else
				return snprintf(buf, size, "Li-Po");
		case SKY_BATT_LION:
			if (value_format == SKY_PARAM_VALUE_TEXT_AND_NUMERIC)
				return snprintf(buf, size, "Li-Ion (0x%02x)", v);
			else
				return snprintf(buf, size, "Li-Ion");
		default:
			if (value_format == SKY_PARAM_VALUE_TEXT_AND_NUMERIC)
				return snprintf(buf, size, "unknown type (0x%02x)", v);
			else
				return snprintf(buf, size, "unknown type");
		}
	case SKY_SINK_BATT_CAPACITY_MAH:
	case SKY_SINK_BATT_MIN_VOLTAGE_MV:
	case SKY_SINK_BATT_MAX_VOLTAGE_MV:
	case SKY_SINK_CHARGING_MAX_CURRENT_MA:
	case SKY_SINK_CUTOFF_MIN_CURRENT_MA:
	case SKY_SINK_CUTOFF_TIMEOUT_MS:
	case SKY_SINK_PRECHARGE_DELAY_SECS:
	case SKY_SINK_PRECHARGE_SECS:
	case SKY_SINK_TOTAL_CHARGE_SECS:
		return snprintf(buf, size, "%u", v);
	case SKY_SINK_PRECHARGE_CURRENT_MA:
		return snprintf(buf, size, "%u", v);
	case SKY_SINK_USER_DATA1:
	case SKY_SINK_USER_DATA2:
	case SKY_SINK_USER_DATA3:
	case SKY_SINK_USER_DATA4:
		return snprintf(buf, size, "0x%08x", v);
	default:
		if (value_format == SKY_PARAM_VALUE_TEXT_AND_NUMERIC)
			return snprintf(buf, size, "unknown param (0x%02x)", v);
		else
			return snprintf(buf, size, "unknown param");
	}
}

int sky_sinkparam_value_from_str(const char *str,
				 enum sky_sink_param param,
				 struct sky_dev_params *params)
{
	unsigned int v = 0;
	int rc = 0;

	if (param >= SKY_SINK_NUM_DEVPARAM)
		return -EINVAL;

	switch(param) {
	case SKY_SINK_CAPABILITIES:
		if (0 == strcasecmp(str, "PLC_WHILE_CHARGING")) {
			v |= (1 << SKY_CAP_PLC_WHILE_CHARGING);
		} else if (0 == strcasecmp(str, "NIL")) {
			v = 0;
		} else {
			if (parse_unsigned(str, &v))
				return -EINVAL;
			if (v & ~(1 << SKY_CAP_PLC_WHILE_CHARGING))
				return -EINVAL;
		}
		break;
	case SKY_SINK_BATT_TYPE: {
		enum sky_batt_type batt_type;

		if (parse_batt_type(str, &batt_type)) {
			if (parse_unsigned(str, &v))
				return -EINVAL;
			if (v != SKY_BATT_LIPO &&
			    v != SKY_BATT_LION)
				return -EINVAL;
		} else {
			v = batt_type;
		}
		break;
	}
	case SKY_SINK_BATT_CAPACITY_MAH:
	case SKY_SINK_BATT_MIN_VOLTAGE_MV:
	case SKY_SINK_BATT_MAX_VOLTAGE_MV:
	case SKY_SINK_CHARGING_MAX_CURRENT_MA:
	case SKY_SINK_CUTOFF_MIN_CURRENT_MA:
	case SKY_SINK_CUTOFF_TIMEOUT_MS:
	case SKY_SINK_PRECHARGE_DELAY_SECS:
	case SKY_SINK_PRECHARGE_SECS:
	case SKY_SINK_TOTAL_CHARGE_SECS:
	case SKY_SINK_USER_DATA1:
	case SKY_SINK_USER_DATA2:
	case SKY_SINK_USER_DATA3:
	case SKY_SINK_USER_DATA4:
		rc = parse_unsigned(str, &v);
		break;
	case SKY_SINK_PRECHARGE_CURRENT_MA:
		rc = parse_unsigned(str, &v);
		break;
	default:
		return -EINVAL;
	}

	if (rc)
		return -EINVAL;

	params->dev_params_bits |= (1 << param);
	params->dev_params[param] = v;

	return 0;
}

const char *sky_gpsstatus_to_str(enum sky_gps_status status)
{
	switch(status) {
	case SKY_GPS_STATUS_NO_FIX:
		return "GPS_STATUS_NO_FIX";
	case SKY_GPS_STATUS_FIX:
		return "GPS_STATUS_FIX";
	case SKY_GPS_STATUS_DGPS_FIX:
		return "GPS_STATUS_DGPS_FIX";
	default:
		sky_err("unknown status: %d\n", status);
		return "UNKNOWN_STATUS";
	}
}

const char *sky_gpsmode_to_str(enum sky_gps_mode mode)
{
	switch(mode) {
	case SKY_GPS_MODE_NOT_SEEN:
		return "GPS_MODE_NOT_SEEN";
	case SKY_GPS_MODE_NO_FIX:
		return "GPS_MODE_NO_FIX";
	case SKY_GPS_MODE_2D:
		return "GPS_MODE_2D";
	case SKY_GPS_MODE_3D:
		return "GPS_MODE_3D";
	default:
		sky_err("unknown mode: %d\n", mode);
		return "UNKNOWN_MODE";
	}
}

void sky_confinit(struct sky_conf *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
}

static int validate_conf(struct sky_conf *cfg)
{
	if (cfg->mux_type == SKY_MUX_HW1 && !strlen(cfg->mux_dev)) {
		/* A bit of compatibility */
		strcpy(cfg->mux_dev, "/dev/skysenseUSB");
	} else if (!strlen(cfg->mux_dev)) {
		sky_err("Config error: 'mux_dev' is not specified\n");
		return -ENODATA;
	}

	if (cfg->mux_type == SKY_MUX_HW2) {
		unsigned p;

		if (cfg->psu.type != SKY_PSU_UNKNOWN ||
		    cfg->psu.voltage ||
		    cfg->psu.current ||
		    cfg->psu.precharge_current_coef ||
		    cfg->psu.precharge_secs) {
			sky_err("Config error: The MUX type is HW2, but the PSU configuration parameters are present. Comment out all PSU related settings and only use HW2 specific settings.\n");
			return -ENODATA;
		}

		/* Restore PSU type from HW2 params */
		p = SKY_HW2_PSU_TYPE;
		if (cfg->mux_hw2_params.dev_params_bits & 1<<p)
			cfg->psu.type = cfg->mux_hw2_params.dev_params[p];
	}

	return 0;
}

int sky_confparse(const char *path, struct sky_conf *cfg)
{
	FILE *fp;
	char *line = NULL;
	size_t len = 0;
	struct stat st;
	int rc;

	sky_confinit(cfg);

	rc = stat(path, &st);
	if (rc)
		return -errno;

	if (S_IFREG != (st.st_mode & S_IFMT))
		return -ENOENT;

	fp = fopen(path, "r");
	if (fp == NULL)
		return -ENOENT;

	while (getline(&line, &len, fp) != -1)
		if ((rc = parse_line(line, cfg)))
			break;

	free(line);
	fclose(fp);

	return validate_conf(cfg);
}

int sky_discoverbroker(struct sky_brokerinfo *brokerinfo,
		       unsigned int timeout_ms)
{
	struct sky_dev_ops *ops;

	foreach_devops(ops, devops) {
		if (ops->contype != SKY_REMOTE)
			continue;
		if (!ops->discoverbroker)
			continue;
		return ops->discoverbroker(ops, brokerinfo, timeout_ms);
	}

	return -EOPNOTSUPP;
}

int sky_asyncopen(const struct sky_conf *conf,
		  struct sky_async **async)
{
	struct sky_dev_ops *ops;

	foreach_devops(ops, devops) {
		if (ops->contype != conf->contype)
			continue;
		return ops->asyncopen(conf, ops, async);
	}

	return -EOPNOTSUPP;
}

void sky_asyncclose(struct sky_async *async)
{
	if (async)
		async->ops->asyncclose(async);
}

int sky_asyncfd(struct sky_async *async)
{
	return async->ops->asyncfd(async);
}

int sky_asyncexecute(struct sky_async *async, bool wait)
{
	return async->ops->asyncexecute(async, wait);
}

void sky_asyncreq_completionset(struct sky_async_req *req,
				sky_async_completion_t *completion)
{
	req->completion = completion;
}

void sky_asyncreq_useruuidset(struct sky_async_req *req,
			      const uint8_t *usruuid)
{
	req->usruuid = usruuid;
}

bool sky_asyncreq_cancel(struct sky_async *async, struct sky_async_req *req)
{
	return async->ops->asyncreq_cancel(async, req);
}

int sky_asyncreq_peerinfo(struct sky_async *async,
			  struct sky_peerinfo *peerinfo,
			  struct sky_async_req *req)
{
	struct sky_dev *dev = NULL;

	sky_asyncreq_init(SKY_PEERINFO_REQ, dev, NULL, peerinfo, req);
	sky_asyncreq_add(async, req);
	return 0;
}

int sky_asyncreq_devslist(struct sky_async *async,
			  struct sky_dev_desc **list,
			  struct sky_async_req *req)
{
	struct sky_dev *dev = NULL;

	*list = NULL;

	sky_asyncreq_init(SKY_DEVS_LIST_REQ, dev, NULL, list, req);
	sky_asyncreq_add(async, req);
	return 0;
}

int sky_asyncreq_paramsget(struct sky_async *async,
			   struct sky_dev *dev,
			   struct sky_dev_params *params,
			   struct sky_async_req *req)
{
	if (!params->dev_params_bits)
		return -EINVAL;

	sky_asyncreq_init(SKY_GET_DEV_PARAMS_REQ, dev, params, params, req);
	sky_asyncreq_add(async, req);
	return 0;
}

int sky_asyncreq_paramsset(struct sky_async *async,
			   struct sky_dev *dev,
			   const struct sky_dev_params *params,
			   struct sky_async_req *req)
{
	if (!params->dev_params_bits)
		return -EINVAL;

	sky_asyncreq_init(SKY_SET_DEV_PARAMS_REQ, dev, params, NULL, req);
	sky_asyncreq_add(async, req);
	return 0;
}

int sky_asyncreq_chargingstate(struct sky_async *async,
			       struct sky_dev *dev,
			       struct sky_charging_state *state,
			       struct sky_async_req *req)
{
	sky_asyncreq_init(SKY_CHARGING_STATE_REQ, dev, NULL, state, req);
	sky_asyncreq_add(async, req);
	return 0;
}

int sky_asyncreq_reset(struct sky_async *async,
		       struct sky_dev *dev,
		       struct sky_async_req *req)
{
	sky_asyncreq_init(SKY_RESET_DEV_REQ, dev, NULL, NULL, req);
	sky_asyncreq_add(async, req);
	return 0;
}

int sky_asyncreq_scanresume(struct sky_async *async,
			     struct sky_dev *dev,
			     struct sky_async_req *req)
{
	sky_asyncreq_init(SKY_RESUME_SCAN_REQ, dev, NULL, NULL, req);
	sky_asyncreq_add(async, req);
	return 0;
}

int sky_asyncreq_scanstop(struct sky_async *async,
			    struct sky_dev *dev,
			    struct sky_async_req *req)
{
	sky_asyncreq_init(SKY_STOP_SCAN_REQ, dev, NULL, NULL, req);
	sky_asyncreq_add(async, req);
	return 0;
}

int sky_asyncreq_droneport_open(struct sky_async *async,
				struct sky_dev *dev,
				struct sky_async_req *req)
{
	sky_asyncreq_init(SKY_OPEN_DRONEPORT_REQ, dev, NULL, NULL, req);
	sky_asyncreq_add(async, req);
	return 0;
}

int sky_asyncreq_droneport_close(struct sky_async *async,
				 struct sky_dev *dev,
				 struct sky_async_req *req)
{
	sky_asyncreq_init(SKY_CLOSE_DRONEPORT_REQ, dev, NULL, NULL, req);
	sky_asyncreq_add(async, req);
	return 0;
}

int sky_asyncreq_droneport_state(struct sky_async *async,
				 struct sky_dev *dev,
				 struct sky_droneport_state *state,
				 struct sky_async_req *req)
{
	sky_asyncreq_init(SKY_DRONEPORT_STATE_REQ, dev, NULL, state, req);
	sky_asyncreq_add(async, req);
	return 0;
}

int sky_asyncreq_gpsdata(struct sky_async *async,
			 struct sky_dev *dev,
			 struct sky_gpsdata *gpsdata,
			 struct sky_async_req *req)
{
	sky_asyncreq_init(SKY_GPSDATA_REQ, dev, NULL, gpsdata, req);
	sky_asyncreq_add(async, req);
	return 0;
}

int sky_asyncreq_dronedetect(struct sky_async *async,
			     struct sky_dev *dev,
			     enum sky_drone_status *status,
			     struct sky_async_req *req)
{
	sky_asyncreq_init(SKY_DRONEDETECT_REQ, dev, NULL, status, req);
	sky_asyncreq_add(async, req);
	return 0;
}

int sky_asyncreq_sink_paramsget(struct sky_async *async,
				struct sky_dev *dev,
				struct sky_dev_params *params,
				struct sky_async_req *req)
{
	if (!params->dev_params_bits)
		return -EINVAL;

	sky_asyncreq_init(SKY_SINK_GET_DEV_PARAMS_REQ, dev, params, params, req);
	sky_asyncreq_add(async, req);
	return 0;
}

int sky_asyncreq_sink_paramsset(struct sky_async *async,
				struct sky_dev *dev,
				const struct sky_dev_params *params,
				struct sky_async_req *req)
{
	if (!params->dev_params_bits)
		return -EINVAL;

	sky_asyncreq_init(SKY_SINK_SET_DEV_PARAMS_REQ, dev, params, NULL, req);
	sky_asyncreq_add(async, req);
	return 0;
}

int sky_asyncreq_sink_infoget(struct sky_async *async,
			      struct sky_dev *dev,
			      struct sky_sink_info *info,
			      struct sky_async_req *req)
{
	sky_asyncreq_init(SKY_SINK_GET_INFO_REQ, dev, NULL, info, req);
	sky_asyncreq_add(async, req);
	return 0;
}

int sky_asyncreq_sink_chargestart(struct sky_async *async,
				  struct sky_dev *dev,
				  struct sky_async_req *req)
{
	sky_asyncreq_init(SKY_SINK_START_CHARGE_REQ, dev, NULL, NULL, req);
	sky_asyncreq_add(async, req);
	return 0;
}

int sky_asyncreq_sink_chargestop(struct sky_async *async,
				 struct sky_dev *dev,
				 struct sky_async_req *req)
{
	sky_asyncreq_init(SKY_SINK_STOP_CHARGE_REQ, dev, NULL, NULL, req);
	sky_asyncreq_add(async, req);
	return 0;
}

int sky_asyncreq_subscription_token(struct sky_async *async,
				    struct sky_dev *dev,
				    struct sky_subscription_token *token,
				    struct sky_async_req *req)
{
	sky_asyncreq_init(SKY_GET_SUBSCRIPTION_TOKEN_REQ, dev, NULL, token, req);
	sky_asyncreq_add(async, req);
	return 0;
}

static int sky_asyncexecute_on_stack(struct sky_async *async,
				     struct sky_async_req *req)
{
	int rc;

	rc = sky_asyncexecute(async, true);
	if (rc < 0)
		return rc;

	return req->out.rc;
}

int sky_peerinfo(const struct sky_conf *conf,
		 struct sky_peerinfo *peerinfo)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(conf, &async);
	if (!rc)
		rc = sky_asyncreq_peerinfo(async, peerinfo, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_devslist(const struct sky_conf *conf,
		 struct sky_dev_desc **list)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(conf, &async);
	if (!rc)
		rc = sky_asyncreq_devslist(async, list, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	if (!rc && !*list) {
		/*
		 * Server relies on that error for the synchronous call,
		 * but sky_asynreq_devslist() returns 0 and list will be set
		 * to NULL if no devices found. Please consider also clients
		 * and remote protocol before changing this behaviour.
		 */
		rc = -ENODEV;
	}

	return rc;
}

int sky_paramsget(struct sky_dev *dev, struct sky_dev_params *params)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_paramsget(async, dev, params, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_paramsset(struct sky_dev *dev, const struct sky_dev_params *params)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_paramsset(async, dev, params, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_chargingstate(struct sky_dev *dev, struct sky_charging_state *state)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_chargingstate(async, dev, state, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_reset(struct sky_dev *dev)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_reset(async, dev, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_scanresume(struct sky_dev *dev)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_scanresume(async, dev, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_scanstop(struct sky_dev *dev)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_scanstop(async, dev, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_droneport_open(struct sky_dev *dev)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_droneport_open(async, dev, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_droneport_close(struct sky_dev *dev)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_droneport_close(async, dev, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_droneport_state(struct sky_dev *dev, struct sky_droneport_state *state)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_droneport_state(async, dev, state, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_gpsdata(struct sky_dev *dev, struct sky_gpsdata *gpsdata)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_gpsdata(async, dev, gpsdata, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_dronedetect(struct sky_dev *dev, enum sky_drone_status *status)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_dronedetect(async, dev, status, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_sink_paramsget(struct sky_dev *dev, struct sky_dev_params *params)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_sink_paramsget(async, dev, params, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_sink_paramsset(struct sky_dev *dev, const struct sky_dev_params *params)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_sink_paramsset(async, dev, params, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_sink_infoget(struct sky_dev *dev, struct sky_sink_info *info)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_sink_infoget(async, dev, info, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_sink_chargestart(struct sky_dev *dev)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_sink_chargestart(async, dev, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_sink_chargestop(struct sky_dev *dev)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_sink_chargestop(async, dev, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_subscription_token(struct sky_dev *dev,
			   struct sky_subscription_token *token)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_subscription_token(async, dev, token, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

void sky_devsfree(struct sky_dev_desc *head)
{
	struct sky_dev_desc *next;

	while (head) {
		next = head->next;
		free(head);
		head = next;
	}
}

int sky_devopen(const struct sky_dev_desc *devdesc, struct sky_dev **dev_)
{
	const struct sky_dev_ops *ops = devdesc->dev_ops;
	int rc;

	rc = ops->devopen(devdesc, dev_);
	if (!rc) {
		struct sky_dev *dev = *dev_;

		dev->devdesc = *devdesc;
		dev->devdesc.next = NULL;
	}

	return rc;
}

void sky_devclose(struct sky_dev *dev)
{
	if (!dev)
		return;
	(void)sky_unsubscribe(dev);
	get_devops(dev)->devclose(dev);
}

int sky_devinfo(struct sky_dev *dev, struct sky_dev_desc *devdesc)
{
	*devdesc = dev->devdesc;

	return 0;
}

static void *subscription_work(void *data)
{
	struct sky_charging_state state;
	struct sky_dev *dev = data;
	unsigned long long ms;
	int rc;

	while (!dev->unsubscribed) {
		ms = msecs_epoch();
		memset(&state, 0, sizeof(state));
		rc = get_devops(dev)->subscription_work(dev, &state);
		ms = msecs_epoch() - ms;

		if (rc == -EBADF && dev->devdesc.conf.contype == SKY_LOCAL) {
			/*
			 * We exit the loop only in case of a fatal error for the
			 * local connection, i.e. when MUX returns an error ("bad file
			 * descriptor" when HW1 MUX was unplugged). We don't do the same
			 * for the remote connection because we want the connection to
			 * be reestablished.
			 */
			sky_err("Got fatal error inside the subscription loop, exit\n");
			exit(-1);
			break;
		}

		if (rc >= 0)
			/* Notify subscribers only in case of success */
			dev->subsc.on_state(dev->subsc.data, &state);

		/* We sleep only if subscription_work wants us to */
		if (rc == 0 && ms < dev->subsc.interval_msecs) {
			ms = dev->subsc.interval_msecs - ms;
			usleep(ms * 1000);
		}
	}

	return NULL;
}

int sky_subscribe(struct sky_dev *dev,
		  struct sky_subscription_token *token,
		  struct sky_subscription *subsc)
{
	int rc;

	if (dev->thread)
		return -EEXIST;
	if (subsc->on_state == NULL || !subsc->interval_msecs)
		return -EINVAL;

	memcpy(&dev->subsc, subsc, sizeof(*subsc));

	rc = get_devops(dev)->subscribe(dev, token);
	if (rc)
		return rc;
	rc = pthread_create(&dev->thread, NULL, subscription_work, dev);
	if (rc)
		get_devops(dev)->unsubscribe(dev);

	return -rc;
}

int sky_unsubscribe(struct sky_dev *dev)
{
	if (!dev->thread)
		return -ENOENT;

	dev->unsubscribed = true;
	pthread_join(dev->thread, NULL);

	dev->unsubscribed = false;
	dev->thread = 0;

	get_devops(dev)->unsubscribe(dev);

	return 0;
}

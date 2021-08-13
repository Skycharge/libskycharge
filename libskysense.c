#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <ctype.h>
#include <sys/stat.h>

#include <uuid/uuid.h>

#include "libskysense-pri.h"
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

static int parse_line(char *line, struct sky_conf *cfg)
{
	char *str;
	int rc;

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

	} else if ((str = strstr(line, "mux-hw1-eeprom-inited="))) {
		unsigned p = SKY_HW1_EEPROM_INITED;
		rc = sscanf(str + 22, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-scanning-interval="))) {
		unsigned p = SKY_HW1_SCANNING_INTERVAL;
		rc = sscanf(str + 26, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-precharging-interval="))) {
		unsigned p = SKY_HW1_PRECHARGING_INTERVAL;
		rc = sscanf(str + 29, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-precharging-counter="))) {
		unsigned p = SKY_HW1_PRECHARGING_COUNTER;
		rc = sscanf(str + 28, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-postcharging-interval="))) {
		unsigned p = SKY_HW1_POSTCHARGING_INTERVAL;
		rc = sscanf(str + 30, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-postcharging-delay="))) {
		unsigned p = SKY_HW1_POSTCHARGING_DELAY;
		rc = sscanf(str + 27, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-wet-delay="))) {
		unsigned p = SKY_HW1_WET_DELAY;
		rc = sscanf(str + 18, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-shortcirc-delay="))) {
		unsigned p = SKY_HW1_SHORTCIRC_DELAY;
		rc = sscanf(str + 24, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-thresh-finish-charging="))) {
		unsigned p = SKY_HW1_THRESH_FINISH_CHARGING;
		rc = sscanf(str + 31, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-thresh-nocharger-present="))) {
		unsigned p = SKY_HW1_THRESH_NOCHARGER_PRESENT;
		rc = sscanf(str + 33, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-thresh-shortcirc="))) {
		unsigned p = SKY_HW1_THRESH_SHORTCIRC;
		rc = sscanf(str + 25, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-current-mon-interval="))) {
		unsigned p = SKY_HW1_CURRENT_MON_INTERVAL;
		rc = sscanf(str + 29, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	} else if ((str = strstr(line, "mux-hw1-wait-start-charging-sec="))) {
		unsigned p = SKY_HW1_WAIT_START_CHARGING_SEC;
		rc = sscanf(str + 32, "%u", &cfg->mux_hw1_params.dev_params[p]);
		if (rc != 1)
			return -ENODATA;
		cfg->mux_hw1_params.dev_params_bits |= 1<<p;

	}

	/*
	 * PSU config
	 */

	else if ((str = strstr(line, "psu-type="))) {
		if (0 == strcasecmp(str + 9, "rsp-750-48") ||
		    0 == strcasecmp(str + 9, "rsp750-48"))
			cfg->psu.type = SKY_PSU_RSP_750_48;
		/* TODO not yet supported! */
		/* else if (0 == strcasecmp(str + 9, "rsp-1600-48") || */
		/* 	 0 == strcasecmp(str + 9, "rsp1600-48")) */
		/* 	cfg->psu.type = SKY_PSU_RSP_1600_48; */
		else
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

	return 0;
}

void sky_confinit(struct sky_conf *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
}

static int validate_conf(struct sky_conf *cfg)
{
	if (cfg->mux_type == SKY_MUX_HW1 && !strlen(cfg->mux_dev)) {
		/* A bit of compatibility */
		sky_err("Config warning: 'mux_dev' is not specified, assume /dev/skysenseUSB\n");
		strcpy(cfg->mux_dev, "/dev/skysenseUSB");
	} else if (!strlen(cfg->mux_dev)) {
		sky_err("Config error: 'mux_dev' is not specified\n");
		return -ENODATA;
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

int sky_asyncreq_chargestart(struct sky_async *async,
			     struct sky_dev *dev,
			     struct sky_async_req *req)
{
	sky_asyncreq_init(SKY_START_CHARGE_REQ, dev, NULL, NULL, req);
	sky_asyncreq_add(async, req);
	return 0;
}

int sky_asyncreq_chargestop(struct sky_async *async,
			    struct sky_dev *dev,
			    struct sky_async_req *req)
{
	sky_asyncreq_init(SKY_STOP_CHARGE_REQ, dev, NULL, NULL, req);
	sky_asyncreq_add(async, req);
	return 0;
}

int sky_asyncreq_psu_typeset(struct sky_async *async,
			     struct sky_dev *dev,
			     enum sky_psu_type psu_type,
			     struct sky_async_req *req)
{
	uint16_t *ptype, type = psu_type;

	sky_asyncreq_init(SKY_PSU_SET_TYPE_REQ, dev, req->in.storage, NULL, req);
	sky_asyncreq_add(async, req);
	BUILD_BUG_ON(sizeof(type) > sizeof(req->in.storage));
	ptype = (typeof(&type))req->in.storage;
	*ptype = type;
	return 0;
}

int sky_asyncreq_psu_voltageset(struct sky_async *async,
				struct sky_dev *dev,
				uint16_t mv,
				struct sky_async_req *req)
{
	uint16_t *pmv;

	sky_asyncreq_init(SKY_PSU_SET_VOLTAGE_REQ, dev, req->in.storage, NULL, req);
	sky_asyncreq_add(async, req);
	BUILD_BUG_ON(sizeof(mv) > sizeof(req->in.storage));
	pmv = (typeof(&mv))req->in.storage;
	*pmv = mv;
	return 0;
}

int sky_asyncreq_psu_currentset(struct sky_async *async,
				struct sky_dev *dev,
				uint16_t ma,
				struct sky_async_req *req)
{
	uint16_t *pma;

	sky_asyncreq_init(SKY_PSU_SET_CURRENT_REQ, dev, req->in.storage, NULL, req);
	sky_asyncreq_add(async, req);
	BUILD_BUG_ON(sizeof(ma) > sizeof(req->in.storage));
	pma = (typeof(&ma))req->in.storage;
	*pma = ma;
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

int sky_chargestart(struct sky_dev *dev)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_chargestart(async, dev, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_chargestop(struct sky_dev *dev)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_chargestop(async, dev, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_psu_typeset(struct sky_dev *dev, enum sky_psu_type psu_type)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_psu_typeset(async, dev, psu_type, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_psu_voltageset(struct sky_dev *dev, uint16_t mv)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_psu_voltageset(async, dev, mv, &req);
	if (!rc)
		rc = sky_asyncexecute_on_stack(async, &req);
	sky_asyncclose(async);

	return rc;
}

int sky_psu_currentset(struct sky_dev *dev, uint16_t ma)
{
	struct sky_async *async = NULL;
	struct sky_async_req req;
	int rc;

	rc = sky_asyncopen(&dev->devdesc.conf, &async);
	if (!rc)
		rc = sky_asyncreq_psu_currentset(async, dev, ma, &req);
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
		rc = get_devops(dev)->subscription_work(dev, &state);
		ms = msecs_epoch() - ms;

		if (rc < 0 && dev->devdesc.conf.contype == SKY_LOCAL) {
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
		  struct sky_subscription *subsc)
{
	int rc;

	if (dev->thread)
		return -EEXIST;
	if (subsc->on_state == NULL || !subsc->interval_msecs)
		return -EINVAL;

	memcpy(&dev->subsc, subsc, sizeof(*subsc));

	rc = get_devops(dev)->subscribe(dev);
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

#include <assert.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <stdarg.h>
#include <sys/file.h>

#include <libserialport.h>
#include <gps.h>

#include "libskysense-pri.h"
#include "libskybms.h"
#include "libskydp.h"
#include "types.h"
#include "crc8.h"
#include "hw1-pri.h"
#include "hw2-pri.h"

enum {
	TO_BUF   = 0,
	FROM_BUF = 1,
	COUNT_SZ = 2,

	TIMEOUT_MS = 5000,

	CHECK_ALL   = 0,
	CHECK_MAGIC = 1,
};

struct skyloc_dev {
	struct sky_dev dev;
	struct sp_port *port;
	struct gps_data_t gpsdata;
	struct bms_lib bms;
	bool gps_nodev;
	pthread_mutex_t mutex;
	int lockfd;
};

struct skyserial_desc {
	uint8_t req_hdr_len;
	uint8_t req_data_off;

	uint8_t rsp_hdr_len;
	uint8_t rsp_data_off;

	void (*fill_cmd_hdr)(uint8_t *cmd_buf, uint8_t len, uint8_t cmd);
	bool (*is_valid_rsp_hdr)(const uint8_t *rsp_buf, uint8_t len, uint8_t cmd,
				 bool only_magic, int *rc);
	uint8_t (*get_rsp_len)(const uint8_t *rsp_buf);
	bool (*check_crc)(const uint8_t *rsp_buf);
};

struct sky_hw_ops {
	int (*get_hw_info)(struct skyloc_dev *dev, struct sky_hw_info *info);
	int (*get_params)(struct skyloc_dev *dev,
			  struct sky_dev_params *params);
	int (*set_params)(struct skyloc_dev *dev,
			  const struct sky_dev_params *params);
	int (*get_state)(struct skyloc_dev *dev,
			 struct sky_charging_state *state);
	int (*reset)(struct skyloc_dev *dev);
	int (*scan)(struct skyloc_dev *dev, unsigned autoscan);
	int (*get_sink_params)(struct skyloc_dev *dev,
			       struct sky_dev_params *params);
	int (*set_sink_params)(struct skyloc_dev *dev,
			       const struct sky_dev_params *params);
	int (*get_sink_info)(struct skyloc_dev *dev,
			     struct sky_sink_info *info);
};

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

static int devlock(struct skyloc_dev *dev)
{
	int rc;

	rc = flock(dev->lockfd, LOCK_EX);
	if (rc < 0) {
		rc = -errno;
		sky_err("flock(): %s\n", strerror(-rc));
	}
	pthread_mutex_lock(&dev->mutex);

	return rc;
}

static void devunlock(struct skyloc_dev *dev)
{
	(void)flock(dev->lockfd, LOCK_UN);
	pthread_mutex_unlock(&dev->mutex);
}

static int skycmd_arg_copy(va_list *ap, int dir, void *buf,
			   size_t off, size_t maxlen)
{
	uint32_t val32, *val32p;
	uint16_t val16, *val16p;
	void *p;

	uint8_t sz;

	sz = va_arg(*ap, int);
	switch (sz) {
	case 0:
		return -EINVAL;
	case 2:
		val16p = va_arg(*ap, typeof(val16p));
		if (off + 2 > maxlen)
			return -EINVAL;
		if (dir == TO_BUF) {
			val16 = htole16(*val16p);
			memcpy(buf + off, &val16, 2);
		} else if (dir == FROM_BUF) {
			memcpy(&val16, buf + off, 2);
			*val16p = htole16(val16);
		}

		return 2;
	case 4:
		val32p = va_arg(*ap, typeof(val32p));
		if (off + 4 > maxlen)
			return -EINVAL;
		if (dir == TO_BUF) {
			val32 = htole32(*val32p);
			memcpy(buf + off, &val32, 4);
		} else if (dir == FROM_BUF) {
			memcpy(&val32, buf + off, 4);
			*val32p = htole32(val32);
		}

		return 4;
	default:
		/* No attempt to do conversion between bytes order */
		p = va_arg(*ap, typeof(p));
		if (off + sz > maxlen)
			return -EINVAL;
		if (dir == TO_BUF)
			memcpy(buf + off, p, sz);
		else if (dir == FROM_BUF)
			memcpy(p, buf + off, sz);

		return sz;
	}
}

static int skycmd_args_inbytes(va_list ap, size_t num, size_t maxlen)
{
	va_list ap_cpy;
	int off, i;
	int rc;

	va_copy(ap_cpy, ap);
	for (off = 0, i = 0; i < num; i++) {
		rc = skycmd_arg_copy(&ap_cpy, COUNT_SZ, NULL, off, maxlen);
		if (rc < 0) {
			off = rc;
			goto out;
		}
		off += rc;
	}
out:
	va_end(ap_cpy);

	return off;
}

static int skycmd_serial_cmd(struct skyloc_dev *dev,
			     struct skyserial_desc *proto,
			     uint8_t cmd, unsigned req_num,
			     int rsp_num,
			     ...)
{
	uint8_t len, cmd_len, rsp_len;
	uint8_t cmd_buf[256], rsp_buf[256];
	enum sp_return sprc;
	int rc, args, off;
	va_list ap;

	char strdump_req[512];
	char strdump_rsp[512];
	bool valid_crc;

	va_start(ap, rsp_num);
	for (len = 0, off = proto->req_data_off, args = 0; args < req_num; args++) {
		rc = skycmd_arg_copy(&ap, TO_BUF, cmd_buf, off,
				     sizeof(cmd_buf) - 1);
		if (rc < 0)
			goto out;

		len += rc;
		off += rc;
	}
	proto->fill_cmd_hdr(cmd_buf, len, cmd);

	rc = devlock(dev);
	if (rc)
		goto out;

	sprc = sp_flush(dev->port, SP_BUF_BOTH);
	if (sprc < 0) {
		rc = sprc_to_errno(sprc);
		sky_err("sp_flush(): %s\n", strerror(-rc));
		goto out_unlock;
	}
	cmd_len = len + proto->req_hdr_len;
	sprc = sp_blocking_write(dev->port, cmd_buf, cmd_len, TIMEOUT_MS);
	if (sprc < 0) {
		rc = sprc_to_errno(sprc);
		sky_err("sp_blocking_write(): %s\n", strerror(-rc));
		goto out_unlock;
	} else if (sprc != cmd_len) {
		sky_err("sp_blocking_write(): failed to write within %d ms timeout, only %d is written, but expected %d\n",
			TIMEOUT_MS, sprc, cmd_len);
		rc = -ETIMEDOUT;
		goto out_unlock;
	}
	if (rsp_num >= 0) {
		rc = skycmd_args_inbytes(ap, rsp_num, sizeof(rsp_buf));
		if (rc < 0) {
			sky_err("skycmd_args_inbytes(): %s\n", strerror(-rc));
			goto out_unlock;
		}

		len = rc;
		if (len > sizeof(rsp_buf) - proto->rsp_hdr_len) {
			rc = -EINVAL;
			sky_err("skycmd_args_inbytes(): %s\n", strerror(-rc));
			goto out_unlock;
		}
		/* Firstly read header */
		sprc = sp_blocking_read(dev->port, rsp_buf, proto->rsp_data_off,
					TIMEOUT_MS);
		if (sprc < 0) {
			rc = sprc_to_errno(sprc);
			sky_err("sp_blocking_read(): %s\n", strerror(-rc));
			goto out_unlock;
		} else if (sprc != proto->rsp_data_off) {
			sky_err("sp_blocking_read(): failed to read within %d ms timeout, only %d is read, but expected %d\n",
				TIMEOUT_MS, sprc, proto->rsp_data_off);
			rc = -ETIMEDOUT;
			goto out_unlock;
		} else if (!proto->is_valid_rsp_hdr(rsp_buf, len + proto->rsp_hdr_len,
						    cmd, CHECK_MAGIC, NULL)) {
			hex_dump_to_buffer(cmd_buf, cmd_len, 16, 1,
					   strdump_req, sizeof(strdump_req),
					   false);
			hex_dump_to_buffer(rsp_buf, proto->rsp_data_off, 16, 1,
					   strdump_rsp, sizeof(strdump_rsp),
					   false);

			sky_err("Invalid response header\n\n"
				"Request hexdump:\n%s\n"
				"Response header hexdump:\n%s\n",
				strdump_req, strdump_rsp);

			rc = -EPROTO;
			goto out_unlock;
		}
		/*
		 * Read the rest, for HW2 we don't care about the size for
		 * the compatibility sake.
		 */
		rsp_len = min(len, proto->get_rsp_len(rsp_buf)) + proto->rsp_hdr_len;
		sprc = sp_blocking_read(dev->port, rsp_buf + proto->rsp_data_off,
				rsp_len - proto->rsp_data_off,
				TIMEOUT_MS);
		if (sprc < 0) {
			rc = sprc_to_errno(sprc);
			sky_err("sp_blocking_read(): %s\n", strerror(-rc));
			goto out_unlock;
		} else if (sprc != rsp_len - proto->rsp_data_off) {
			sky_err("sp_blocking_read(): failed to read within %d ms timeout, only %d is read, but expected %d\n",
				TIMEOUT_MS, sprc, rsp_len - proto->rsp_data_off);
			rc = -ETIMEDOUT;
			goto out_unlock;
		} else if (!(valid_crc = proto->check_crc(rsp_buf)) ||
			   !proto->is_valid_rsp_hdr(rsp_buf, rsp_len, cmd,
						    CHECK_ALL, &rc)) {
			hex_dump_to_buffer(cmd_buf, cmd_len, 16, 1,
					   strdump_req, sizeof(strdump_req), false);
			hex_dump_to_buffer(rsp_buf, rsp_len, 16, 1,
					   strdump_rsp, sizeof(strdump_rsp), false);

			if (!valid_crc)
				sky_err("Invalid response CRC\n\n");
			else
				sky_err("Invalid response\n\n");

			sky_err("Request hexdump:\n%s\n"
				"Response hexdump:\n%s\n",
				strdump_req, strdump_rsp);

			rc = -EPROTO;
			goto out_unlock;
		}
		/* Check response errno */
		if (rc)
			goto out_unlock;

		for (off = proto->rsp_data_off, args = 0;
		     off < rsp_len + proto->rsp_data_off && args < rsp_num;
		     args++) {
			rc = skycmd_arg_copy(&ap, FROM_BUF, rsp_buf, off,
					     rsp_len + proto->rsp_data_off);
			if (rc < 0) {
				sky_err("skycmd_arg_copy(): %s\n", strerror(-rc));
				goto out_unlock;
			}

			off += rc;
		}
	}
	rc = 0;

out_unlock:
	devunlock(dev);
out:
	va_end(ap);

	return rc;
}

/*
 * HW1 specific operations
 */

static bool hw1_sky_is_valid_rsp_hdr(const uint8_t *rsp_buf, uint8_t len,
				     uint8_t cmd, bool only_magic, int *rc)
{
	if (only_magic)
		return (rsp_buf[0] == 0x55);

	*rc = 0;

	return (rsp_buf[0] == 0x55 && rsp_buf[1] == len && rsp_buf[2] == cmd);
}

static uint8_t hw1_sky_get_rsp_len(const uint8_t *rsp_buf)
{
	/* Without header */
	return rsp_buf[1] - 4;
}

static void hw1_sky_fill_cmd_hdr(uint8_t *cmd_buf, uint8_t len, uint8_t cmd)
{
	cmd_buf[0] = 0x55;
	cmd_buf[1] = len + 4;
	cmd_buf[2] = cmd;
	cmd_buf[len + 3] = 0x00;
}

static bool hw1_sky_check_crc(const uint8_t *rsp_buf)
{
	return true;
}

struct skyserial_desc hw1_sky_serial = {
	.req_hdr_len  = 4,
	.req_data_off = 3,
	.rsp_hdr_len  = 4,
	.rsp_data_off = 3,
	.fill_cmd_hdr     = hw1_sky_fill_cmd_hdr,
	.is_valid_rsp_hdr = hw1_sky_is_valid_rsp_hdr,
	.get_rsp_len      = hw1_sky_get_rsp_len,
	.check_crc        = hw1_sky_check_crc,
};

static int hw1_sky_get_hw_info(struct skyloc_dev *dev,
			       struct sky_hw_info *info)
{
	uint8_t major, minor, revis;
	int rc;

	rc = skycmd_serial_cmd(dev, &hw1_sky_serial,
			       SKY_HW1_FIRMWARE_VERSION_CMD,
			       0, 3,
			       sizeof(major), &major,
			       sizeof(minor), &minor,
			       sizeof(revis), &revis);
	if (rc)
		return rc;

	*info = (struct sky_hw_info) {
		.fw_version = major << 16 | minor << 8 | revis,
		.hw_version = 2 << 16 | 30 << 8 | 2 /* 2.30b, latest HW1, I hope */
	};

	return 0;
}

static int hw1_sky_get_param(struct skyloc_dev *dev,
			     uint8_t param, uint16_t *val)
{
	return skycmd_serial_cmd(dev, &hw1_sky_serial,
				 SKY_HW1_GET_PARAMETER_CMD,
				 1, 2,
				 sizeof(param), &param,
				 sizeof(param), &param, sizeof(*val), val);
}

static int hw1_sky_get_params(struct skyloc_dev *dev,
			      struct sky_dev_params *params)
{
	uint16_t val = 0;
	int rc, i;

	/* Fetch params from EEPROM */
	rc = skycmd_serial_cmd(dev, &hw1_sky_serial,
			       SKY_HW1_READ_DATA_FROM_EEP_CMD,
			       0, 0);
	if (rc)
		return rc;

	for (i = 0; i < SKY_HW1_NUM_DEVPARAM; i++) {
		if (!(params->dev_params_bits & (1<<i)))
				continue;
		rc = hw1_sky_get_param(dev, i, &val);
		if (rc)
			return rc;
		params->dev_params[i] = val;
	}

	return 0;
}

static int hw1_sky_set_param(struct skyloc_dev *dev,
			     uint8_t param, uint16_t val)
{
	return skycmd_serial_cmd(dev, &hw1_sky_serial,
				 SKY_HW1_SET_PARAMETER_CMD,
				 2, 2,
				 sizeof(param), &param, sizeof(val), &val,
				 sizeof(param), &param, sizeof(val), &val);
}

static int hw1_sky_set_params(struct skyloc_dev *dev,
			      const struct sky_dev_params *params)
{
	uint16_t val;
	int rc, i;

	for (i = 0; i < SKY_HW1_NUM_DEVPARAM; i++) {
		if (!(params->dev_params_bits & (1<<i)))
				continue;
		val = params->dev_params[i];
		rc = hw1_sky_set_param(dev, i, val);
		if (rc)
			return rc;
	}
	/* Commit params to EEPROM */
	return skycmd_serial_cmd(dev, &hw1_sky_serial,
				 SKY_HW1_SAVE_DATA_TO_EEP_CMD,
				 0, 0);
}

static int hw1_sky_get_state(struct skyloc_dev *dev,
			     struct sky_charging_state *state)
{
	uint16_t vol, cur;
	uint8_t status;
	int rc;

	rc = skycmd_serial_cmd(dev, &hw1_sky_serial,
			       SKY_HW1_GET_VOLTAGE_CMD,
			       0, 1,
			       sizeof(vol), &vol);
	if (rc)
		return rc;
	rc = skycmd_serial_cmd(dev, &hw1_sky_serial,
			       SKY_HW1_GET_CURRENT_CMD,
			       0, 1,
			       sizeof(cur), &cur);
	if (rc)
		return rc;
	rc = skycmd_serial_cmd(dev, &hw1_sky_serial,
			       SKY_HW1_GET_STATUS_CMD,
			       0, 1,
			       sizeof(status), &status);
	if (rc)
		return rc;

	state->voltage_mV = vol;
	state->current_mA = cur;
	state->dev_hw_state = status;

	return 0;
}

static int hw1_sky_reset(struct skyloc_dev *dev)
{
	return skycmd_serial_cmd(dev, &hw1_sky_serial,
				 SKY_HW1_RESET_CMD, 0, -1);
}

static int hw1_sky_scan(struct skyloc_dev *dev, unsigned autoscan)
{
	uint8_t ascan = !!autoscan;

	return skycmd_serial_cmd(dev, &hw1_sky_serial,
				 SKY_HW1_AUTOMATIC_SCAN_CMD,
				 1, 1,
				 sizeof(ascan), &ascan,
				 sizeof(ascan), &ascan);
}

static int hw1_sky_get_sink_params(struct skyloc_dev *dev,
				   struct sky_dev_params *params)
{
	return -EOPNOTSUPP;
}

static int hw1_sky_set_sink_params(struct skyloc_dev *dev,
				   const struct sky_dev_params *params)
{
	return -EOPNOTSUPP;
}

static int hw1_sky_get_sink_info(struct skyloc_dev *dev,
				 struct sky_sink_info *info)
{
	return -EOPNOTSUPP;
}

static struct sky_hw_ops hw1_sky_ops = {
	.get_hw_info      = hw1_sky_get_hw_info,
	.get_params       = hw1_sky_get_params,
	.set_params       = hw1_sky_set_params,
	.get_state        = hw1_sky_get_state,
	.reset            = hw1_sky_reset,
	.scan             = hw1_sky_scan,
	.get_sink_params  = hw1_sky_get_sink_params,
	.set_sink_params  = hw1_sky_set_sink_params,
	.get_sink_info    = hw1_sky_get_sink_info,
};

/*
 * HW2 specific operations
 */

static bool hw2_sky_is_valid_rsp_hdr(const uint8_t *rsp_buf, uint8_t len,
				     uint8_t cmd, bool only_magic, int *rc)
{
	uint8_t type;

	if (only_magic)
		return (rsp_buf[0] == 0x42);

	type = rsp_buf[3] & 0xf;
	*rc  = -skyerrno_to_errno(rsp_buf[3] >> 4);

	/*
	 * A bit of compatibility: we don't care about the length at all
	 * TODO: the 3'rd byte is type of response, in future need to
	 *       support events as well
	 */
	return (rsp_buf[0] == 0x42 && type == SKY_HW2_SYNC_RESPONSE_CMD);
}

static uint8_t hw2_sky_get_rsp_len(const uint8_t *rsp_buf)
{
	return rsp_buf[2];
}

static void hw2_sky_fill_cmd_hdr(uint8_t *cmd_buf, uint8_t len, uint8_t cmd)
{
	cmd_buf[0] = 0x42;
	cmd_buf[2] = len;
	cmd_buf[3] = cmd;
	cmd_buf[1] = crc8(cmd_buf + 2, len + 4 - 2);
}

static bool hw2_sky_check_crc(const uint8_t *rsp_buf)
{
	uint8_t crc;

	crc = crc8((uint8_t *)rsp_buf + 2, rsp_buf[2] + 2);

	return (crc == rsp_buf[1]);
}

struct skyserial_desc hw2_sky_serial = {
	.req_hdr_len      = 4,
	.req_data_off     = 4,
	.rsp_hdr_len      = 4,
	.rsp_data_off     = 4,
	.fill_cmd_hdr     = hw2_sky_fill_cmd_hdr,
	.is_valid_rsp_hdr = hw2_sky_is_valid_rsp_hdr,
	.get_rsp_len      = hw2_sky_get_rsp_len,
	.check_crc        = hw2_sky_check_crc,
};

static int hw2_sky_get_hw_info(struct skyloc_dev *dev,
			       struct sky_hw_info *info)
{
	return skycmd_serial_cmd(dev, &hw2_sky_serial,
				 SKY_HW2_GET_MUX_INFO_CMD,
				 0, 1,
				 sizeof(*info), info);
}

static int hw2_sky_get_params(struct skyloc_dev *dev,
			      struct sky_dev_params *params)
{
	struct sky_hw2_mux_settings settings;
	int p, rc;

	memset(&settings, 0, sizeof(settings));
	rc = skycmd_serial_cmd(dev, &hw2_sky_serial,
			       SKY_HW2_GET_MUX_SETTINGS_CMD,
			       0, 1,
			       sizeof(settings), &settings);
	if (rc)
		return rc;

	for (p = 0; p < SKY_HW2_NUM_DEVPARAM; p++) {
		if (!(params->dev_params_bits & (1<<p)))
			continue;

		switch (p) {
		case SKY_HW2_IGNORE_INVAL_CHARGING_SETTINGS:
			params->dev_params[p] =
				settings.bool_settings & SKY_HW2_IGNORE_INVAL_CHARGING_SETTINGS_BIT;
			break;
		case SKY_HW2_IGNORE_LOW_BATT_VOLTAGE:
			params->dev_params[p] =
				settings.bool_settings & SKY_HW2_IGNORE_LOW_BATT_VOLTAGE_BIT;
			break;
		case SKY_HW2_KEEP_SILENCE:
			params->dev_params[p] =
				settings.bool_settings & SKY_HW2_KEEP_SILENCE_BIT;
			break;
		case SKY_HW2_USE_FIXED_V_I:
			params->dev_params[p] =
				settings.bool_settings & SKY_HW2_USE_FIXED_V_I_BIT;
			break;
		case SKY_HW2_IGNORE_VOLTAGE_ON_OUTPUT:
			params->dev_params[p] =
				settings.bool_settings & SKY_HW2_IGNORE_VOLTAGE_ON_OUTPUT_BIT;
			break;
		case SKY_HW2_PSU_TYPE:
			params->dev_params[p] =
				settings.psu_type;
			break;
		case SKY_HW2_PSU_FIXED_VOLTAGE_MV:
			params->dev_params[p] =
				settings.psu_fixed_voltage_mV;
			break;
		case SKY_HW2_PSU_FIXED_CURRENT_MA:
			params->dev_params[p] =
				settings.psu_fixed_current_mA;
			break;
		case SKY_HW2_NR_BAD_HEARTBEATS:
			params->dev_params[p] =
				settings.nr_bad_heartbeats;
			break;
		case SKY_HW2_ERROR_INDICATION_TIMEOUT_SECS:
			params->dev_params[p] =
				settings.error_indication_timeout_secs;
			break;
		case SKY_HW2_MIN_SENSE_CURRENT_MA:
			params->dev_params[p] =
				settings.min_sense_current_mA;
			break;
		case SKY_HW2_REPEAT_CHARGE_AFTER_MINS:
			params->dev_params[p] =
				settings.repeat_charge_after_mins;
			break;
		case SKY_HW2_SENSE_VOLTAGE_CALIB_POINT1_MV:
			params->dev_params[p] =
				calib_point_to_uint32(settings.sense_calib.voltage_p1_mV.set,
						      settings.sense_calib.voltage_p1_mV.read);
			break;
		case SKY_HW2_SENSE_VOLTAGE_CALIB_POINT2_MV:
			params->dev_params[p] =
				calib_point_to_uint32(settings.sense_calib.voltage_p2_mV.set,
						      settings.sense_calib.voltage_p2_mV.read);
			break;
		case SKY_HW2_SENSE_CURRENT_CALIB_POINT1_MA:
			params->dev_params[p] =
				calib_point_to_uint32(settings.sense_calib.current_p1_mA.set,
						      settings.sense_calib.current_p1_mA.read);
			break;
		case SKY_HW2_SENSE_CURRENT_CALIB_POINT2_MA:
			params->dev_params[p] =
				calib_point_to_uint32(settings.sense_calib.current_p2_mA.set,
						      settings.sense_calib.current_p2_mA.read);
			break;
		case SKY_HW2_PSU_VOLTAGE_CALIB_POINT1_MV:
			params->dev_params[p] =
				calib_point_to_uint32(settings.psu_calib.voltage_p1_mV.set,
						      settings.psu_calib.voltage_p1_mV.read);
			break;
		case SKY_HW2_PSU_VOLTAGE_CALIB_POINT2_MV:
			params->dev_params[p] =
				calib_point_to_uint32(settings.psu_calib.voltage_p2_mV.set,
						      settings.psu_calib.voltage_p2_mV.read);
			break;
		case SKY_HW2_PSU_CURRENT_CALIB_POINT1_MA:
			params->dev_params[p] =
				calib_point_to_uint32(settings.psu_calib.current_p1_mA.set,
						      settings.psu_calib.current_p1_mA.read);
			break;
		case SKY_HW2_PSU_CURRENT_CALIB_POINT2_MA:
			params->dev_params[p] =
				calib_point_to_uint32(settings.psu_calib.current_p2_mA.set,
						      settings.psu_calib.current_p2_mA.read);
			break;
		}
	}

	return 0;
}

static int hw2_sky_set_params(struct skyloc_dev *dev,
			      const struct sky_dev_params *params)
{
	struct sky_hw2_mux_settings settings;
	enum sky_hw2_mux_settings_bits bit;
	int p, rc;

	/* First retreive settings */
	memset(&settings, 0, sizeof(settings));
	rc = skycmd_serial_cmd(dev, &hw2_sky_serial,
			       SKY_HW2_GET_MUX_SETTINGS_CMD,
			       0, 1,
			       sizeof(settings), &settings);
	if (rc)
		return rc;

	for (p = 0; p < SKY_HW2_NUM_DEVPARAM; p++) {
		if (!(params->dev_params_bits & (1<<p)))
			continue;

		switch (p) {
		case SKY_HW2_IGNORE_INVAL_CHARGING_SETTINGS:
			bit = SKY_HW2_IGNORE_INVAL_CHARGING_SETTINGS_BIT;
			if (params->dev_params[p])
				settings.bool_settings |= bit;
			else
				settings.bool_settings &= ~bit;
			break;
		case SKY_HW2_IGNORE_LOW_BATT_VOLTAGE:
			bit = SKY_HW2_IGNORE_LOW_BATT_VOLTAGE_BIT;
			if (params->dev_params[p])
				settings.bool_settings |= bit;
			else
				settings.bool_settings &= ~bit;
			break;
		case SKY_HW2_KEEP_SILENCE:
			bit = SKY_HW2_KEEP_SILENCE_BIT;
			if (params->dev_params[p])
				settings.bool_settings |= bit;
			else
				settings.bool_settings &= ~bit;
			break;
		case SKY_HW2_USE_FIXED_V_I:
			bit = SKY_HW2_USE_FIXED_V_I_BIT;
			if (params->dev_params[p])
				settings.bool_settings |= bit;
			else
				settings.bool_settings &= ~bit;
			break;
		case SKY_HW2_IGNORE_VOLTAGE_ON_OUTPUT:
			bit = SKY_HW2_IGNORE_VOLTAGE_ON_OUTPUT_BIT;
			if (params->dev_params[p])
				settings.bool_settings |= bit;
			else
				settings.bool_settings &= ~bit;
			break;
		case SKY_HW2_PSU_TYPE:
			settings.psu_type =
				params->dev_params[p];
			break;
		case SKY_HW2_PSU_FIXED_VOLTAGE_MV:
			settings.psu_fixed_voltage_mV =
				params->dev_params[p];
			break;
		case SKY_HW2_PSU_FIXED_CURRENT_MA:
			settings.psu_fixed_current_mA =
				params->dev_params[p];
			break;
		case SKY_HW2_NR_BAD_HEARTBEATS:
			settings.nr_bad_heartbeats =
				params->dev_params[p];
			break;
		case SKY_HW2_ERROR_INDICATION_TIMEOUT_SECS:
			settings.error_indication_timeout_secs =
				params->dev_params[p];
			break;
		case SKY_HW2_MIN_SENSE_CURRENT_MA:
			settings.min_sense_current_mA =
				params->dev_params[p];
			break;
		case SKY_HW2_REPEAT_CHARGE_AFTER_MINS:
			settings.repeat_charge_after_mins =
				params->dev_params[p];
			break;
		case SKY_HW2_SENSE_VOLTAGE_CALIB_POINT1_MV:
			uint32_to_calib_point(params->dev_params[p],
					      &settings.sense_calib.voltage_p1_mV.set,
					      &settings.sense_calib.voltage_p1_mV.read);
			break;
		case SKY_HW2_SENSE_VOLTAGE_CALIB_POINT2_MV:
			uint32_to_calib_point(params->dev_params[p],
					      &settings.sense_calib.voltage_p2_mV.set,
					      &settings.sense_calib.voltage_p2_mV.read);
			break;
		case SKY_HW2_SENSE_CURRENT_CALIB_POINT1_MA:
			uint32_to_calib_point(params->dev_params[p],
					      &settings.sense_calib.current_p1_mA.set,
					      &settings.sense_calib.current_p1_mA.read);
			break;
		case SKY_HW2_SENSE_CURRENT_CALIB_POINT2_MA:
			uint32_to_calib_point(params->dev_params[p],
					      &settings.sense_calib.current_p2_mA.set,
					      &settings.sense_calib.current_p2_mA.read);
			break;
		case SKY_HW2_PSU_VOLTAGE_CALIB_POINT1_MV:
			uint32_to_calib_point(params->dev_params[p],
					      &settings.psu_calib.voltage_p1_mV.set,
					      &settings.psu_calib.voltage_p1_mV.read);
			break;
		case SKY_HW2_PSU_VOLTAGE_CALIB_POINT2_MV:
			uint32_to_calib_point(params->dev_params[p],
					      &settings.psu_calib.voltage_p2_mV.set,
					      &settings.psu_calib.voltage_p2_mV.read);
			break;
		case SKY_HW2_PSU_CURRENT_CALIB_POINT1_MA:
			uint32_to_calib_point(params->dev_params[p],
					      &settings.psu_calib.current_p1_mA.set,
					      &settings.psu_calib.current_p1_mA.read);
			break;
		case SKY_HW2_PSU_CURRENT_CALIB_POINT2_MA:
			uint32_to_calib_point(params->dev_params[p],
					      &settings.psu_calib.current_p2_mA.set,
					      &settings.psu_calib.current_p2_mA.read);
			break;
		}
	}

	/* Commit changed settings */
	return skycmd_serial_cmd(dev, &hw2_sky_serial,
				 SKY_HW2_SET_MUX_SETTINGS_CMD,
				 1, 0,
				 sizeof(settings), &settings);
}

static int hw2_sky_get_state(struct skyloc_dev *dev,
			     struct sky_charging_state *state)
{
	return skycmd_serial_cmd(dev, &hw2_sky_serial,
				 SKY_HW2_GET_CHARGING_STATE_CMD,
				 0, 1,
				 sizeof(*state), state);
}

static int hw2_sky_reset(struct skyloc_dev *dev)
{
	return skycmd_serial_cmd(dev, &hw2_sky_serial,
				 SKY_HW2_RESET_CMD, 0, -1);
}

static int hw2_sky_scan(struct skyloc_dev *dev, unsigned do_resume)
{
	uint8_t cmd;

	cmd = do_resume ? SKY_HW2_RESUME_CMD : SKY_HW2_STOP_CMD;

	return skycmd_serial_cmd(dev, &hw2_sky_serial,
				 cmd, 0, 0);
}

static int hw2_sky_get_sink_params(struct skyloc_dev *dev,
				   struct sky_dev_params *params)
{
	struct sky_hw2_charging_settings chg_settings;
	uint32_t *user_data;
	int p, rc;

	memset(&chg_settings, 0, sizeof(chg_settings));
	rc = skycmd_serial_cmd(dev, &hw2_sky_serial,
			       SKY_HW2_GET_SINK_CHARGING_SETTINGS_CMD,
			       0, 1,
			       sizeof(chg_settings), &chg_settings);
	if (rc)
		return rc;

	for (p = 0; p < SKY_SINK_NUM_DEVPARAM; p++) {
		if (!(params->dev_params_bits & (1<<p)))
			continue;

		switch (p) {
		case SKY_SINK_CAPABILITIES:
			params->dev_params[p] =
				chg_settings.capabilities;
			break;
		case SKY_SINK_BATT_TYPE:
			params->dev_params[p] =
				chg_settings.batt_type;
			break;
		case SKY_SINK_BATT_CAPACITY_MAH:
			params->dev_params[p] =
				chg_settings.batt_capacity_mAh;
			break;
		case SKY_SINK_BATT_MIN_VOLTAGE_MV:
			params->dev_params[p] =
				chg_settings.batt_min_voltage_mV;
			break;
		case SKY_SINK_BATT_MAX_VOLTAGE_MV:
			params->dev_params[p] =
				chg_settings.batt_max_voltage_mV;
			break;
		case SKY_SINK_CHARGING_MAX_CURRENT_MA:
			params->dev_params[p] =
				chg_settings.charging_max_current_mA;
			break;
		case SKY_SINK_CUTOFF_MIN_CURRENT_MA:
			params->dev_params[p] =
				chg_settings.cutoff_min_current_mA;
			break;
		case SKY_SINK_CUTOFF_TIMEOUT_MS:
			params->dev_params[p] =
				chg_settings.cutoff_timeout_ms;
			break;
		case SKY_SINK_PRECHARGE_CURRENT_COEF:
			params->dev_params[p] =
				chg_settings.precharge_current_coef;
			break;
		case SKY_SINK_PRECHARGE_DELAY_SECS:
			params->dev_params[p] =
				chg_settings.precharge_delay_secs;
			break;
		case SKY_SINK_PRECHARGE_SECS:
			params->dev_params[p] =
				chg_settings.precharge_secs;
			break;
		case SKY_SINK_TOTAL_CHARGE_SECS:
			params->dev_params[p] =
				chg_settings.total_charge_secs;
			break;
		case SKY_SINK_USER_DATA1:
			user_data = (uint32_t *)chg_settings.user_data;
			params->dev_params[p] = user_data[0];
			break;
		case SKY_SINK_USER_DATA2:
			user_data = (uint32_t *)chg_settings.user_data;
			params->dev_params[p] = user_data[1];
			break;
		case SKY_SINK_USER_DATA3:
			user_data = (uint32_t *)chg_settings.user_data;
			params->dev_params[p] = user_data[2];
			break;
		case SKY_SINK_USER_DATA4:
			user_data = (uint32_t *)chg_settings.user_data;
			params->dev_params[p] = user_data[3];
			break;
		}
	}

	return 0;
}

static int hw2_sky_set_sink_params(struct skyloc_dev *dev,
				   const struct sky_dev_params *params)
{
	struct sky_hw2_charging_settings chg_settings;
	uint32_t *user_data;
	int p, rc;

	/* First retreive settings */
	memset(&chg_settings, 0, sizeof(chg_settings));
	rc = skycmd_serial_cmd(dev, &hw2_sky_serial,
			       SKY_HW2_GET_SINK_CHARGING_SETTINGS_CMD,
			       0, 1,
			       sizeof(chg_settings), &chg_settings);
	if (rc)
		return rc;

	for (p = 0; p < SKY_SINK_NUM_DEVPARAM; p++) {
		if (!(params->dev_params_bits & (1<<p)))
			continue;

		switch (p) {
		case SKY_SINK_CAPABILITIES:
			chg_settings.capabilities =
				params->dev_params[p];
			break;
		case SKY_SINK_BATT_TYPE:
			chg_settings.batt_type =
				params->dev_params[p];
			break;
		case SKY_SINK_BATT_CAPACITY_MAH:
			chg_settings.batt_capacity_mAh =
				params->dev_params[p];
			break;
		case SKY_SINK_BATT_MIN_VOLTAGE_MV:
			chg_settings.batt_min_voltage_mV =
				params->dev_params[p];
			break;
		case SKY_SINK_BATT_MAX_VOLTAGE_MV:
			chg_settings.batt_max_voltage_mV =
				params->dev_params[p];
			break;
		case SKY_SINK_CHARGING_MAX_CURRENT_MA:
			chg_settings.charging_max_current_mA =
				params->dev_params[p];
			break;
		case SKY_SINK_CUTOFF_MIN_CURRENT_MA:
			chg_settings.cutoff_min_current_mA =
				params->dev_params[p];
			break;
		case SKY_SINK_CUTOFF_TIMEOUT_MS:
			chg_settings.cutoff_timeout_ms =
				params->dev_params[p];
			break;
		case SKY_SINK_PRECHARGE_CURRENT_COEF:
			chg_settings.precharge_current_coef =
				params->dev_params[p];
			break;
		case SKY_SINK_PRECHARGE_DELAY_SECS:
			chg_settings.precharge_delay_secs =
				params->dev_params[p];
			break;
		case SKY_SINK_PRECHARGE_SECS:
			chg_settings.precharge_secs =
				params->dev_params[p];
			break;
		case SKY_SINK_TOTAL_CHARGE_SECS:
			chg_settings.total_charge_secs =
				params->dev_params[p];
			break;
		case SKY_SINK_USER_DATA1:
			user_data = (uint32_t *)chg_settings.user_data;
			user_data[0] = params->dev_params[p];
			break;
		case SKY_SINK_USER_DATA2:
			user_data = (uint32_t *)chg_settings.user_data;
			user_data[1] = params->dev_params[p];
			break;
		case SKY_SINK_USER_DATA3:
			user_data = (uint32_t *)chg_settings.user_data;
			user_data[2] = params->dev_params[p];
			break;
		case SKY_SINK_USER_DATA4:
			user_data = (uint32_t *)chg_settings.user_data;
			user_data[3] = params->dev_params[p];
			break;
		}
	}

	/* Commit changed settings */
	return skycmd_serial_cmd(dev, &hw2_sky_serial,
				 SKY_HW2_SET_SINK_CHARGING_SETTINGS_CMD,
				 1, 0,
				 sizeof(chg_settings), &chg_settings);
}

static int hw2_sky_get_sink_info(struct skyloc_dev *dev,
				 struct sky_sink_info *info)
{
	/* First retreive settings */
	memset(info, 0, sizeof(*info));
	return skycmd_serial_cmd(dev, &hw2_sky_serial,
				 SKY_HW2_GET_SINK_INFO_CMD,
				 0, 1,
				 sizeof(*info), info);
}

static struct sky_hw_ops hw2_sky_ops = {
	.get_hw_info      = hw2_sky_get_hw_info,
	.get_params       = hw2_sky_get_params,
	.set_params       = hw2_sky_set_params,
	.get_state        = hw2_sky_get_state,
	.reset            = hw2_sky_reset,
	.scan             = hw2_sky_scan,
	.get_sink_params  = hw2_sky_get_sink_params,
	.set_sink_params  = hw2_sky_set_sink_params,
	.get_sink_info    = hw2_sky_get_sink_info,
};

/*
 * Common API
 */

static int skyloc_peerinfo(const struct sky_dev_ops *ops,
			   const struct sky_conf *conf,
			   struct sky_peerinfo *peerinfo)
{
	return -EOPNOTSUPP;
}

static int devopen(const struct sky_dev_desc *devdesc,
		   struct skyloc_dev **dev_, bool probbing)
{
	struct skyloc_dev *dev;
	struct sp_port_config* spconf;
	enum sp_return sprc;
	int rc;

	dev = calloc(1, sizeof(*dev));
	if (!dev)
		return -ENOMEM;

	pthread_mutex_init(&dev->mutex, NULL);

	/*
	 * TODO: this is very ugly, but shitty devserialport does not
	 * provide any way to get fd or any lock mechanism.  So we have
	 * to do locking ourselves.
	 */
	dev->lockfd = open(devdesc->portname, O_WRONLY);
	if (dev->lockfd < 0) {
		rc = dev->lockfd;
		goto free_dev;
	}

	rc = devlock(dev);
	if (rc)
		goto close_lockfd;

	sprc = sp_get_port_by_name(devdesc->portname, &dev->port);
	if (sprc) {
		rc = sprc_to_errno(sprc);
		goto unlock;
	}

	sprc = sp_open(dev->port, SP_MODE_READ_WRITE);
	if (sprc) {
		rc = sprc_to_errno(sprc);
		goto free_port;
	}

	sprc = sp_new_config(&spconf);
	if (sprc) {
		rc = sprc_to_errno(sprc);
		goto close_port;
	}

	sprc = sp_get_config(dev->port, spconf);
	if (sprc) {
		rc = sprc_to_errno(sprc);
		goto free_conf;
	}

	sprc = sp_set_config_flowcontrol(spconf, SP_FLOWCONTROL_NONE);
	if (sprc) {
		rc = sprc_to_errno(sprc);
		goto free_conf;
	}

	sprc = sp_set_config_parity(spconf, SP_PARITY_NONE);
	if (sprc) {
		rc = sprc_to_errno(sprc);
		goto free_conf;
	}

	sprc = sp_set_config_bits(spconf, 8);
	if (sprc) {
		rc = sprc_to_errno(sprc);
		goto free_conf;
	}

	sprc = sp_set_config_baudrate(spconf, 9600);
	if (sprc) {
		rc = sprc_to_errno(sprc);
		goto free_conf;
	}

	sprc = sp_set_config(dev->port, spconf);
	if (sprc) {
		rc = sprc_to_errno(sprc);
		goto free_conf;
	}

	/*
	 * Init GPS and DP only on real open
	 */
	if (!probbing) {
		rc = gps_open(GPSD_SHARED_MEMORY, NULL, &dev->gpsdata);
		if (rc)
			/* Do not make much noise if GPS does not exist */
			dev->gps_nodev = true;

		rc = dp_configure(devdesc);
		if (rc && rc != -EOPNOTSUPP)
			goto free_conf;
	} else {
		dev->gps_nodev = true;
	}
	bms_init(&dev->bms);
	sp_free_config(spconf);
	*dev_ = dev;

	devunlock(dev);

	return 0;

free_conf:
	sp_free_config(spconf);
close_port:
	sp_close(dev->port);
free_port:
	sp_free_port(dev->port);
unlock:
	devunlock(dev);
close_lockfd:
	close(dev->lockfd);
free_dev:
	free(dev);

	return rc;
}

static void devclose(struct skyloc_dev *dev)
{
	if (!dev->gps_nodev)
		gps_close(&dev->gpsdata);
	bms_deinit(&dev->bms);
	close(dev->lockfd);
	sp_close(dev->port);
	sp_free_port(dev->port);
	free(dev);
}

static int devprobe(struct sky_dev_desc *devdesc)
{
	struct skyloc_dev *dev;

	int rc;

	rc = devopen(devdesc, &dev, true);
	if (rc)
		return rc;

	rc = devdesc->hw_ops->get_hw_info(dev, &devdesc->hw_info);
	devclose(dev);

	return rc;
}

static int skyloc_devslist(const struct sky_dev_ops *dev_ops,
			   const struct sky_conf *conf,
			   struct sky_dev_desc **out)
{
	struct sky_dev_desc *devdesc, *head = NULL, *tail = NULL;
	struct sp_port **ports = NULL;
	struct sky_hw_ops *hw_ops;

	char mux_dev_path[PATH_MAX];
	int rc;

	/*
	 * Devprobe requires real TTY device path, not the udev reference,
	 * so resolve all references and make sure we have a real path.
	 */
	if (realpath(conf->mux_dev, mux_dev_path)) {
		if (conf->mux_type == SKY_MUX_HW1)
			hw_ops = &hw1_sky_ops;
		else if (conf->mux_type == SKY_MUX_HW2)
			hw_ops = &hw2_sky_ops;
		else
			assert(0);

		devdesc = calloc(1, sizeof(*devdesc));
		if (devdesc == NULL) {
			rc = -ENOMEM;
			goto err;
		}
		memcpy(devdesc->portname, mux_dev_path,
		       min(strlen(mux_dev_path), sizeof(devdesc->portname) - 1));
		devdesc->dev_type = conf->mux_type;
		devdesc->conf = *conf;
		devdesc->dev_ops = dev_ops;
		devdesc->hw_ops = hw_ops;
		devdesc->proto_version = 0; /* 0 for the local driver */
		rc = devprobe(devdesc);
		if (rc) {
			free(devdesc);
			goto err;
		}
		devdesc->next = head;
		head = devdesc;
		if (tail == NULL)
			tail = devdesc;
	}
	rc = 0;
	if (head)
		tail->next = *out;
	*out = head;
out:
	sp_free_port_list(ports);

	return rc;

err:
	while (head) {
		devdesc = head;
		head = head->next;
		free(devdesc);
	}
	goto out;
}

static int skyloc_devopen(const struct sky_dev_desc *devdesc,
			  struct sky_dev **dev_)
{
	struct skyloc_dev *dev = NULL;
	int rc;

	rc = devopen(devdesc, &dev, false);
	if (rc)
		return rc;

	*dev_ = &dev->dev;

	return 0;
}

static void skyloc_devclose(struct sky_dev *dev_)
{
	struct skyloc_dev *dev;

	dev = container_of(dev_, struct skyloc_dev, dev);
	devclose(dev);
}

static int skyloc_paramsget(struct sky_dev *dev_, struct sky_dev_params *params)
{
	struct skyloc_dev *dev;

	if (params->dev_params_bits == 0)
		return -EINVAL;

	dev = container_of(dev_, struct skyloc_dev, dev);

	return get_hwops(dev_)->get_params(dev, params);
}

static int skyloc_paramsset(struct sky_dev *dev_,
			    const struct sky_dev_params *params)
{
	struct skyloc_dev *dev;

	if (params->dev_params_bits == 0)
		/* Nothing to set */
		return 0;

	dev = container_of(dev_, struct skyloc_dev, dev);

	return get_hwops(dev_)->set_params(dev, params);
}

static int skyloc_chargingstate(struct sky_dev *dev_,
				struct sky_charging_state *state)
{
	struct bms_data bms_data;
	struct skyloc_dev *dev;

	int rc;

	dev = container_of(dev_, struct skyloc_dev, dev);

	memset(state, 0, sizeof(*state));
	rc = get_hwops(dev_)->get_state(dev, state);
	if (rc)
		return rc;

	rc = bms_request_data(&dev->bms, &bms_data);
	if (!rc) {
		state->until_full_secs = bms_data.charge_time;
		state->state_of_charge = bms_data.charge_perc;
	}

	return 0;
}

static int skyloc_subscribe(struct sky_dev *dev_)
{
	/* Nop for now */
	return 0;
}

static void skyloc_unsubscribe(struct sky_dev *dev_)
{
	/* Nop for now */
}

static int skyloc_subscription_work(struct sky_dev *dev_,
				    struct sky_charging_state *state)
{
	return skyloc_chargingstate(dev_, state);
}


static int skyloc_reset(struct sky_dev *dev_)
{
	struct skyloc_dev *dev;

	dev = container_of(dev_, struct skyloc_dev, dev);

	return get_hwops(dev_)->reset(dev);
}


static int skyloc_autoscan(struct sky_dev *dev_, unsigned autoscan)
{
	struct skyloc_dev *dev;

	dev = container_of(dev_, struct skyloc_dev, dev);

	return get_hwops(dev_)->scan(dev, autoscan);
}

static int skyloc_chargestart(struct sky_dev *dev_)
{
	/* Start charging enabling autoscan */
	return skyloc_autoscan(dev_, 1);
}

static int skyloc_chargestop(struct sky_dev *dev_)
{
	/* Stop charging disabling autoscan */
	return skyloc_autoscan(dev_, 0);
}

static int skyloc_droneport_open(struct sky_dev *dev)
{
	return dp_open(dev);
}

static int skyloc_droneport_close(struct sky_dev *dev)
{
	return dp_close(dev);
}

static int
skyloc_droneport_state(struct sky_dev *dev, struct sky_droneport_state *state)
{
	return dp_state(dev, state);
}

static int skyloc_gpsdata(struct sky_dev *dev_, struct sky_gpsdata *gpsdata)
{
	struct skyloc_dev *dev;
	int rc;

	dev = container_of(dev_, struct skyloc_dev, dev);
	if (dev->gps_nodev)
		return -EOPNOTSUPP;

#if GPSD_API_MAJOR_VERSION <= 6
	rc = gps_read(&dev->gpsdata);
#else
	rc = gps_read(&dev->gpsdata, NULL, 0);
#endif
	if (rc < 0)
		return -EOPNOTSUPP;

	memset(gpsdata, 0, sizeof(*gpsdata));
#if GPSD_API_MAJOR_VERSION < 10
	gpsdata->status = dev->gpsdata.status;
#else
	gpsdata->status = dev->gpsdata.fix.status;
#endif
	gpsdata->satellites_used = dev->gpsdata.satellites_used;
	BUILD_BUG_ON(sizeof(gpsdata->dop) != sizeof(dev->gpsdata.dop));
	memcpy(&gpsdata->dop, &dev->gpsdata.dop, sizeof(gpsdata->dop));
	gpsdata->fix.mode = dev->gpsdata.fix.mode;
#if GPSD_API_MAJOR_VERSION < 9
	gpsdata->fix.time = dev->gpsdata.fix.time;
#else
	{
		double tstamp = dev->gpsdata.fix.time.tv_sec;

		tstamp += dev->gpsdata.fix.time.tv_nsec / 1000000000.0;
		gpsdata->fix.time = tstamp;
	}
#endif
	gpsdata->fix.latitude  = dev->gpsdata.fix.latitude;
	gpsdata->fix.longitude = dev->gpsdata.fix.longitude;
	gpsdata->fix.altitude  = dev->gpsdata.fix.altitude;

	return 0;
}

static int skyloc_dronedetect(struct sky_dev *dev_,
			      enum sky_drone_status *status)
{
	struct sky_charging_state state;
	int rc;

	/* Firstly try to detect a drone with drone port sensors */
	rc = dp_drone_detect(dev_);
	if (rc >= 0) {
		*status = rc ? SKY_DRONE_DETECTED : SKY_DRONE_NOT_DETECTED;
		return 0;
	}

	/*
	 * TODO: Here we use HW state for drone detection.
	 *       Obviously that is not enough and in the future
	 *       would be nice to have separate device (ultrasonic?)
	 *       for real detection of drone existance.
	 */
	rc = skyloc_chargingstate(dev_, &state);
	if (rc)
		return rc;

	*status = sky_hw_is_charging(dev_->devdesc.dev_type, state.dev_hw_state) ?
		SKY_DRONE_DETECTED :
		SKY_DRONE_NOT_DETECTED;

	return 0;
}

static int skyloc_sink_infoget(struct sky_dev *dev_,
			       struct sky_sink_info *info)
{
	struct skyloc_dev *dev;

	dev = container_of(dev_, struct skyloc_dev, dev);

	return get_hwops(dev_)->get_sink_info(dev, info);
}

static int skyloc_sink_paramsget(struct sky_dev *dev_,
				 struct sky_dev_params *params)
{
	struct skyloc_dev *dev;

	if (params->dev_params_bits == 0)
		return -EINVAL;

	dev = container_of(dev_, struct skyloc_dev, dev);

	return get_hwops(dev_)->get_sink_params(dev, params);
}

static int skyloc_sink_paramsset(struct sky_dev *dev_,
				 const struct sky_dev_params *params)
{
	struct skyloc_dev *dev;

	if (params->dev_params_bits == 0)
		/* Nothing to set */
		return 0;

	dev = container_of(dev_, struct skyloc_dev, dev);

	return get_hwops(dev_)->set_sink_params(dev, params);
}

static bool skyloc_asyncreq_cancel(struct sky_async *async,
				   struct sky_async_req *req)
{
	bool res = false;

	if (req->next != req) {
		/* Request still in the submit queue */
		assert(!req->tag);
		sky_asyncreq_del(req);
		res = true;
	}

	return res;
}

static int skyloc_asyncopen(const struct sky_conf *conf,
			    const struct sky_dev_ops *ops,
			    struct sky_async **async_)
{
	struct sky_async *async;

	async = calloc(1, sizeof(*async));
	if (!async)
		return -ENOMEM;

	sky_async_init(conf, ops, async);

	*async_ = async;

	return 0;
}

static void skyloc_asyncclose(struct sky_async *async)
{
	while (!sky_async_empty(async)) {
		struct sky_async_req *req;

		req = sky_asyncreq_pop(async);
		sky_asyncreq_complete(async, req, -EIO);
	}
	free(async);
}

static int skyloc_asyncfd(struct sky_async *async)
{
	/* Probably in future we support that */
	return -EOPNOTSUPP;
}

static int skyloc_asyncreq_execute(struct sky_async *async,
				   struct sky_async_req *req)
{
	int rc;

	switch (req->type) {
	case SKY_GET_DEV_PARAMS_REQ:
		rc = skyloc_paramsget(req->dev, req->out.ptr);
		break;
	case SKY_SET_DEV_PARAMS_REQ:
		rc = skyloc_paramsset(req->dev, req->in.ptr);
		break;
	case SKY_START_CHARGE_REQ:
		rc = skyloc_chargestart(req->dev);
		break;
	case SKY_STOP_CHARGE_REQ:
		rc = skyloc_chargestop(req->dev);
		break;
	case SKY_OPEN_DRONEPORT_REQ:
		rc = skyloc_droneport_open(req->dev);
		break;
	case SKY_CLOSE_DRONEPORT_REQ:
		rc = skyloc_droneport_close(req->dev);
		break;
	case SKY_DRONEPORT_STATE_REQ:
		rc = skyloc_droneport_state(req->dev, req->out.ptr);
		break;
	case SKY_RESET_DEV_REQ:
		rc = skyloc_reset(req->dev);
		break;
	case SKY_CHARGING_STATE_REQ:
		rc = skyloc_chargingstate(req->dev, req->out.ptr);
		break;
	case SKY_DEVS_LIST_REQ:
		rc = skyloc_devslist(async->ops, async->conf, req->out.ptr);
		break;
	case SKY_PEERINFO_REQ:
		rc = skyloc_peerinfo(async->ops, async->conf, req->out.ptr);
		break;
	case SKY_GPSDATA_REQ:
		rc = skyloc_gpsdata(req->dev, req->out.ptr);
		break;
	case SKY_DRONEDETECT_REQ:
		rc = skyloc_dronedetect(req->dev, req->out.ptr);
		break;
	case SKY_SINK_GET_DEV_PARAMS_REQ:
		rc = skyloc_sink_paramsget(req->dev, req->out.ptr);
		break;
	case SKY_SINK_SET_DEV_PARAMS_REQ:
		rc = skyloc_sink_paramsset(req->dev, req->in.ptr);
		break;
	case SKY_SINK_GET_INFO_REQ:
		rc = skyloc_sink_infoget(req->dev, req->out.ptr);
		break;
	default:
		/* Consider fatal */
		sky_err("Unknown request: %d\n", req->type);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int skyloc_asyncexecute(struct sky_async *async, bool wait)
{
	int completed = 0;

	while (!sky_async_empty(async)) {
		struct sky_async_req *req;
		int rc;

		req = sky_asyncreq_pop(async);
		rc = skyloc_asyncreq_execute(async, req);
		sky_asyncreq_complete(async, req, rc);
		completed++;
	}

	return completed;
}

static struct sky_dev_ops sky_local_devops = {
	.contype           = SKY_LOCAL,
	.asyncopen         = skyloc_asyncopen,
	.asyncclose        = skyloc_asyncclose,
	.asyncexecute      = skyloc_asyncexecute,
	.asyncfd           = skyloc_asyncfd,
	.asyncreq_cancel   = skyloc_asyncreq_cancel,
	.devopen           = skyloc_devopen,
	.devclose          = skyloc_devclose,
	.subscribe         = skyloc_subscribe,
	.unsubscribe       = skyloc_unsubscribe,
	.subscription_work = skyloc_subscription_work,
};
sky_register_devops(&sky_local_devops);

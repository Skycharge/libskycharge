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

enum hw1_sky_serial_cmd {
	HW1_SKY_RESET_CMD              = 0x01,
	HW1_SKY_SAVE_DATA_TO_EEP_CMD   = 0x02,
	HW1_SKY_READ_DATA_FROM_EEP_CMD = 0x03,
	HW1_SKY_AUTOMATIC_SCAN_CMD     = 0x04,
	HW1_SKY_SET_PARAMETER_CMD      = 0x05,
	HW1_SKY_GET_PARAMETER_CMD      = 0x06,
	HW1_SKY_GET_STATUS_CMD         = 0x07,
	HW1_SKY_GET_CURRENT_CMD        = 0x08,
	HW1_SKY_GET_VOLTAGE_CMD        = 0x09,
	HW1_SKY_COUPLE_SCAN_CMD        = 0x0a,
	HW1_SKY_COUPLE_ACTIVATE_CMD    = 0x0b,
	HW1_SKY_COUPLE_DEACTIVATE_CMD  = 0x0c,
	HW1_SKY_FIRMWARE_VERSION_CMD   = 0x0d,
};

enum hw2_sky_serial_cmd {
	HW2_SKY_UNKNOWN_CMD            = 0x00,
	HW2_SKY_FIRMWARE_VERSION_CMD   = 0x01,
	HW2_SKY_RESET_CMD              = 0x02,
	HW2_SKY_STATE_CMD              = 0x03,
	HW2_SKY_SCAN_CMD               = 0x04,

	/* The last one */
	HW2_SKY_ERROR                  = 0xff,
};

enum {
	TO_BUF   = 0,
	FROM_BUF = 1,

	TIMEOUT_MS = 1000,
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
	uint8_t hdr_len;
	uint8_t data_off;
	void (*fill_cmd_hdr)(char *cmd_buf, uint8_t len, uint8_t cmd);
	int (*is_valid_rsp_hdr)(const char *rsp_buf, uint8_t len, uint8_t cmd);
	int (*check_crc)(const char *rsp_buf);
};

struct sky_hw_ops {
	int (*firmware_version)(struct skyloc_dev *dev, uint8_t *major,
				uint8_t *minor, uint8_t *revis);
	int (*get_params)(struct skyloc_dev *dev,
			  struct sky_dev_params *params);
	int (*set_params)(struct skyloc_dev *dev,
			  const struct sky_dev_params *params);
	int (*get_state)(struct skyloc_dev *dev,
			 struct sky_charging_state *state);
	int (*reset)(struct skyloc_dev *dev);
	int (*scan)(struct skyloc_dev *dev, unsigned autoscan);
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

static int skycmd_arg_copy(va_list *ap, int dir, void *buf,
			   size_t off, size_t maxlen)
{
	uint16_t val16, *val16p;
	uint8_t *val8p;
	uint8_t sz;

	sz = va_arg(*ap, int);
	switch (sz) {
	case 1:
		val8p = va_arg(*ap, typeof(val8p));
		if (off + 1 > maxlen)
			return -EINVAL;
		if (dir == TO_BUF)
			memcpy(buf + off, val8p, 1);
		else
			memcpy(val8p, buf + off, 1);

		return 1;
	case 2:
		val16p = va_arg(*ap, typeof(val16p));
		if (off + 2 > maxlen)
			return -EINVAL;
		if (dir == TO_BUF) {
			val16 = htole16(*val16p);
			memcpy(buf + off, &val16, 2);
		} else {
			memcpy(&val16, buf + off, 2);
			*val16p = htole16(val16);
		}

		return 2;
	default:
		assert(0);
		return -EINVAL;
	}
}

static int skycmd_args_inbytes(va_list ap, size_t num)
{
	va_list ap_cpy;
	char buf[64];
	int len, i;
	int rc;

	va_copy(ap_cpy, ap);
	for (len = 0, i = 0; i < num; i++) {
		rc = skycmd_arg_copy(&ap_cpy, TO_BUF, buf, 0, sizeof(buf));
		if (rc < 0) {
			len = rc;
			goto out;
		}
		len += rc;
	}
out:
	va_end(ap_cpy);

	return len;
}

static int skycmd_serial_cmd(struct skyloc_dev *dev,
			     struct skyserial_desc *proto,
			     uint8_t cmd, unsigned req_num,
			     int rsp_num,
			     ...)
{
	char cmd_buf[8], rsp_buf[16];
	enum sp_return sprc;
	int rc, args, off;
	uint8_t len;
	va_list ap;

	va_start(ap, rsp_num);
	for (len = 0, off = proto->data_off, args = 0; args < req_num; args++) {
		rc = skycmd_arg_copy(&ap, TO_BUF, cmd_buf, off,
				     sizeof(cmd_buf) - 1);
		if (rc < 0)
			goto out;

		len += rc;
		off += rc;
	}
	proto->fill_cmd_hdr(cmd_buf, len, cmd);

	rc = flock(dev->lockfd, LOCK_EX);
	if (rc < 0) {
		rc = -errno;
		goto out;
	}
	pthread_mutex_lock(&dev->mutex);
	sprc = sp_flush(dev->port, SP_BUF_BOTH);
	if (sprc < 0) {
		rc = sprc_to_errno(sprc);
		goto out_unlock;
	}
	sprc = sp_blocking_write(dev->port, cmd_buf, len + proto->hdr_len,
				 TIMEOUT_MS);
	if (sprc < 0) {
		rc = sprc_to_errno(sprc);
		goto out_unlock;
	} else if (sprc != len + proto->hdr_len) {
		rc = -ETIMEDOUT;
		goto out_unlock;
	}
	if (rsp_num >= 0) {
		rc = skycmd_args_inbytes(ap, rsp_num);
		if (rc < 0)
			goto out_unlock;

		len = rc;
		if (len > sizeof(rsp_buf) - proto->hdr_len) {
			rc = -EINVAL;
			goto out_unlock;
		}
		/* Firstly read header */
		sprc = sp_blocking_read(dev->port, rsp_buf, proto->data_off,
					TIMEOUT_MS);
		if (sprc < 0) {
			rc = sprc_to_errno(sprc);
			goto out_unlock;
		} else if (sprc != proto->data_off) {
			rc = -ETIMEDOUT;
			goto out_unlock;
		} else if (!proto->is_valid_rsp_hdr(rsp_buf,
					len + proto->hdr_len, cmd)) {
			rc = -EPROTO;
			goto out_unlock;
		}
		/* Read the rest */
		sprc = sp_blocking_read(dev->port, rsp_buf + proto->data_off,
				len + (proto->hdr_len - proto->data_off),
				TIMEOUT_MS);
		if (sprc < 0) {
			rc = sprc_to_errno(sprc);
			goto out_unlock;
		} else if (sprc != len + (proto->hdr_len - proto->data_off)) {
			rc = -ETIMEDOUT;
			goto out_unlock;
		} else if (!proto->check_crc(rsp_buf)) {
			rc = -EPROTO;
			goto out_unlock;
		}
		for (off = proto->data_off, args = 0; args < rsp_num; args++) {
			rc = skycmd_arg_copy(&ap, FROM_BUF, rsp_buf, off,
					     len + proto->data_off);
			if (rc < 0)
				goto out_unlock;

			off += rc;
		}
	}
	rc = 0;

out_unlock:
	(void)flock(dev->lockfd, LOCK_UN);
	pthread_mutex_unlock(&dev->mutex);
out:
	va_end(ap);

	return rc;
}

/*
 * HW1 specific operations
 */

static int hw1_sky_is_valid_rsp_hdr(const char *rsp_buf, uint8_t len,
				    uint8_t cmd)
{
	return (rsp_buf[0] == 0x55 && rsp_buf[1] == len && rsp_buf[2] == cmd);
}

static void hw1_sky_fill_cmd_hdr(char *cmd_buf, uint8_t len, uint8_t cmd)
{
	cmd_buf[0] = 0x55;
	cmd_buf[1] = len + 4;
	cmd_buf[2] = cmd;
	cmd_buf[len + 3] = 0x00;
}

static int hw1_sky_check_crc(const char *rsp_buf)
{
	return 1;
}

struct skyserial_desc hw1_sky_serial = {
	.hdr_len  = 4,
	.data_off = 3,
	.fill_cmd_hdr     = hw1_sky_fill_cmd_hdr,
	.is_valid_rsp_hdr = hw1_sky_is_valid_rsp_hdr,
	.check_crc        = hw1_sky_check_crc,
};

static int hw1_sky_firmware_version(struct skyloc_dev *dev,
				    uint8_t *major,
				    uint8_t *minor,
				    uint8_t *revis)
{
	return skycmd_serial_cmd(dev, &hw1_sky_serial,
				 HW1_SKY_FIRMWARE_VERSION_CMD,
				 0, 3,
				 sizeof(*major), major,
				 sizeof(*minor), minor,
				 sizeof(*revis), revis);
}

static int hw1_sky_get_param(struct skyloc_dev *dev,
			     uint8_t param, uint16_t *val)
{
	return skycmd_serial_cmd(dev, &hw1_sky_serial,
				 HW1_SKY_GET_PARAMETER_CMD,
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
			       HW1_SKY_READ_DATA_FROM_EEP_CMD,
			       0, 0);
	if (rc)
		return rc;

	for (i = 0; i < SKY_NUM_DEVPARAM; i++) {
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
				 HW1_SKY_SET_PARAMETER_CMD,
				 2, 2,
				 sizeof(param), &param, sizeof(val), &val,
				 sizeof(param), &param, sizeof(val), &val);
}

static int hw1_sky_set_params(struct skyloc_dev *dev,
			      const struct sky_dev_params *params)
{
	uint16_t val;
	int rc, i;

	for (i = 0; i < SKY_NUM_DEVPARAM; i++) {
		if (!(params->dev_params_bits & (1<<i)))
				continue;
		val = params->dev_params[i];
		rc = hw1_sky_set_param(dev, i, val);
		if (rc)
			return rc;
	}
	/* Commit params to EEPROM */
	return skycmd_serial_cmd(dev, &hw1_sky_serial,
				 HW1_SKY_SAVE_DATA_TO_EEP_CMD,
				 0, 0);
}

static int hw1_sky_get_state(struct skyloc_dev *dev,
			     struct sky_charging_state *state)
{
	uint16_t vol, cur;
	uint8_t status;
	int rc;

	rc = skycmd_serial_cmd(dev, &hw1_sky_serial,
			       HW1_SKY_GET_VOLTAGE_CMD,
			       0, 1,
			       sizeof(vol), &vol);
	if (rc)
		return rc;
	rc = skycmd_serial_cmd(dev, &hw1_sky_serial,
			       HW1_SKY_GET_CURRENT_CMD,
			       0, 1,
			       sizeof(cur), &cur);
	if (rc)
		return rc;
	rc = skycmd_serial_cmd(dev, &hw1_sky_serial,
			       HW1_SKY_GET_STATUS_CMD,
			       0, 1,
			       sizeof(status), &status);
	if (rc)
		return rc;

	state->voltage = vol;
	state->current = cur;
	state->dev_hw_state = status;

	return 0;
}

static int hw1_sky_reset(struct skyloc_dev *dev)
{
	return skycmd_serial_cmd(dev, &hw1_sky_serial,
				 HW1_SKY_RESET_CMD, 0, -1);
}

static int hw1_sky_scan(struct skyloc_dev *dev, unsigned autoscan)
{
	uint8_t ascan = !!autoscan;

	return skycmd_serial_cmd(dev, &hw1_sky_serial,
				 HW1_SKY_AUTOMATIC_SCAN_CMD,
				 1, 1,
				 sizeof(ascan), &ascan,
				 sizeof(ascan), &ascan);
}

static struct sky_hw_ops hw1_sky_ops = {
	.firmware_version = hw1_sky_firmware_version,
	.get_params       = hw1_sky_get_params,
	.set_params       = hw1_sky_set_params,
	.get_state        = hw1_sky_get_state,
	.reset            = hw1_sky_reset,
	.scan             = hw1_sky_scan,
};

/*
 * HW2 specific operations
 */

static inline uint8_t crc8(uint8_t *data, uint16_t len)
{
    uint8_t crc = 0xff;
    uint8_t i;

    while (len--) {
        crc ^= *data++;

        for (i = 0; i < 8; i++)
		crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
    }

    return crc;
}

static int hw2_sky_is_valid_rsp_hdr(const char *rsp_buf, uint8_t len,
				    uint8_t cmd)
{
	if (!(rsp_buf[0] == 0x42 && rsp_buf[2] == len && rsp_buf[3] == cmd))
		return 0;

	return 1;
}

static void hw2_sky_fill_cmd_hdr(char *cmd_buf, uint8_t len, uint8_t cmd)
{
	cmd_buf[0] = 0x42;
	cmd_buf[2] = len + 4;
	cmd_buf[3] = cmd;
	cmd_buf[1] = crc8((uint8_t *)cmd_buf + 2, len + 4 - 2);
}

static int hw2_sky_check_crc(const char *rsp_buf)
{
	uint8_t crc = crc8((uint8_t *)rsp_buf + 2, rsp_buf[2] - 2);

	return (crc == (uint8_t)rsp_buf[1]);
}

struct skyserial_desc hw2_sky_serial = {
	.hdr_len  = 4,
	.data_off = 4,
	.fill_cmd_hdr     = hw2_sky_fill_cmd_hdr,
	.is_valid_rsp_hdr = hw2_sky_is_valid_rsp_hdr,
	.check_crc        = hw2_sky_check_crc,
};

static int hw2_sky_firmware_version(struct skyloc_dev *dev,
				    uint8_t *major,
				    uint8_t *minor,
				    uint8_t *revis)
{
	return skycmd_serial_cmd(dev, &hw2_sky_serial,
				 HW2_SKY_FIRMWARE_VERSION_CMD,
				 0, 3,
				 sizeof(*major), major,
				 sizeof(*minor), minor,
				 sizeof(*revis), revis);
}

static int hw2_sky_get_params(struct skyloc_dev *dev,
			      struct sky_dev_params *params)
{
	return -EOPNOTSUPP;
}

static int hw2_sky_set_params(struct skyloc_dev *dev,
			      const struct sky_dev_params *params)
{
	return -EOPNOTSUPP;
}

static int hw2_sky_get_state(struct skyloc_dev *dev,
			     struct sky_charging_state *state)
{
	uint16_t vol, cur, st;
	int rc;

	rc = skycmd_serial_cmd(dev, &hw2_sky_serial,
			       HW2_SKY_STATE_CMD,
			       0, 3,
			       sizeof(vol), &vol,
			       sizeof(cur), &cur,
			       sizeof(st),  &st);
	if (rc)
		return rc;

	state->voltage = vol;
	state->current = cur;
	/* HW1 and HW2 states are equal */
	state->dev_hw_state = st;

	return 0;
}

static int hw2_sky_reset(struct skyloc_dev *dev)
{
	return skycmd_serial_cmd(dev, &hw2_sky_serial,
				 HW2_SKY_RESET_CMD, 0, -1);
}

static int hw2_sky_scan(struct skyloc_dev *dev, unsigned autoscan)
{
	uint8_t scan = !!autoscan, ret;

	return skycmd_serial_cmd(dev, &hw2_sky_serial,
				 HW2_SKY_SCAN_CMD,
				 1, 1,
				 sizeof(scan), &scan,
				 sizeof(ret),  &ret);
}

static struct sky_hw_ops hw2_sky_ops = {
	.firmware_version = hw2_sky_firmware_version,
	.get_params       = hw2_sky_get_params,
	.set_params       = hw2_sky_set_params,
	.get_state        = hw2_sky_get_state,
	.reset            = hw2_sky_reset,
	.scan             = hw2_sky_scan,
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

	sprc = sp_get_port_by_name(devdesc->portname, &dev->port);
	if (sprc)
		goto free_dev;

	sprc = sp_open(dev->port, SP_MODE_READ_WRITE);
	if (sprc)
		goto free_port;

	sprc = sp_new_config(&spconf);
	if (sprc)
		goto close_port;

	sprc = sp_get_config(dev->port, spconf);
	if (sprc)
		goto free_conf;

	sprc = sp_set_config_flowcontrol(spconf, SP_FLOWCONTROL_NONE);
	if (sprc)
		goto free_conf;

	sprc = sp_set_config_parity(spconf, SP_PARITY_NONE);
	if (sprc)
		goto free_conf;

	sprc = sp_set_config_bits(spconf, 8);
	if (sprc)
		goto free_conf;

	sprc = sp_set_config(dev->port, spconf);
	if (sprc)
		goto free_conf;

	/*
	 * TODO: this is very ugly, but shitty devserialport does not
	 * provide any way to get fd or any lock mechanism.  So we have
	 * to do locking ourselves.
	 */
	dev->lockfd = open(devdesc->portname, O_RDWR);
	if (dev->lockfd < 0) {
		sprc = SP_ERR_FAIL;
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
	pthread_mutex_init(&dev->mutex, NULL);
	*dev_ = dev;

	return 0;

free_conf:
	sp_free_config(spconf);
close_port:
	sp_close(dev->port);
free_port:
	sp_free_port(dev->port);
free_dev:
	free(dev);

	return sprc_to_errno(sprc);
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

	uint8_t major, minor, revis;
	int rc;

	rc = devopen(devdesc, &dev, true);
	if (rc)
		return rc;

	rc = devdesc->hw_ops->firmware_version(dev, &major, &minor, &revis);
	if (!rc)
		devdesc->firmware_version = major << 16 | minor << 8 | revis;

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
	enum sp_return sprc;
	int i, rc;

	sprc = sp_list_ports(&ports);
	if (sprc < 0)
		return sprc_to_errno(sprc);

	for (i = 0; ports[i]; i++) {
		const char *desc, *name;

		desc = sp_get_port_description(ports[i]);
		name = sp_get_port_name(ports[i]);
		if (!desc || !name)
			continue;

		if (!strncasecmp(desc, "skysense", 8))
			hw_ops = &hw1_sky_ops;
		else if (!strncasecmp(desc, "sky-hw2", 7))
			hw_ops = &hw2_sky_ops;
		else
			continue;

		devdesc = calloc(1, sizeof(*devdesc));
		if (devdesc == NULL) {
			rc = -ENOMEM;
			goto err;
		}
		strncpy(devdesc->portname, name, sizeof(devdesc->portname) - 1);
		devdesc->dev_type = SKY_INDOOR;
		devdesc->conf = *conf;
		devdesc->dev_ops = dev_ops;
		devdesc->hw_ops = hw_ops;
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
	if (head) {
		tail->next = *out;
		*out = head;
	}
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

	BUILD_BUG_ON(sizeof(params->dev_params_bits) * 8 <
		     SKY_NUM_DEVPARAM);

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

	BUILD_BUG_ON(sizeof(params->dev_params_bits) * 8 <
		     SKY_NUM_DEVPARAM);

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

	rc = get_hwops(dev_)->get_state(dev, state);
	if (rc)
		return rc;

	rc = bms_request_data(&dev->bms, &bms_data);
	if (!rc) {
		state->bms.charge_time = bms_data.charge_time;
		state->bms.charge_perc = bms_data.charge_perc;
	} else {
		state->bms.charge_time = 0;
		state->bms.charge_perc = 0;
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

static int skyloc_coveropen(struct sky_dev *dev)
{
	return dp_open(dev);
}

static int skyloc_coverclose(struct sky_dev *dev)
{
	return dp_close(dev);
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

	*status = sky_hw_is_charging(state.dev_hw_state) ?
		SKY_DRONE_DETECTED :
		SKY_DRONE_NOT_DETECTED;

	return 0;
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
	case SKY_OPEN_COVER_REQ:
		rc = skyloc_coveropen(req->dev);
		break;
	case SKY_CLOSE_COVER_REQ:
		rc = skyloc_coverclose(req->dev);
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

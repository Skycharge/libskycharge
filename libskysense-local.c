#include <assert.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <stdarg.h>
#include <sys/file.h>

#include <libserialport.h>

#include "libskysense-pri.h"
#include "types.h"

enum sky_serial_cmd {
	SKY_RESET_CMD              = 0x01,
	SKY_SAVE_DATA_TO_EEP_CMD   = 0x02,
	SKY_READ_DATA_FROM_EEP_CMD = 0x03,
	SKY_AUTOMATIC_SCAN_CMD     = 0x04,
	SKY_SET_PARAMETER_CMD      = 0x05,
	SKY_GET_PARAMETER_CMD      = 0x06,
	SKY_GET_STATUS_CMD         = 0x07,
	SKY_GET_CURRENT_CMD        = 0x08,
	SKY_GET_VOLTAGE_CMD        = 0x09,
	SKY_COUPLE_SCAN_CMD        = 0x0a,
	SKY_COUPLE_ACTIVATE_CMD    = 0x0b,
	SKY_COUPLE_DEACTIVATE_CMD  = 0x0c,
	SKY_FIRMWARE_VERSION_CMD   = 0x0d,
};

enum {
	TO_BUF   = 0,
	FROM_BUF = 1,

	TIMEOUT_MS = 1000,
};

struct skyloc_dev {
	struct sky_dev dev;
	struct sp_port *port;
	struct sky_charging_state state;
	struct sky_dev_params params;
	struct sky_dev_desc devdesc;
	pthread_mutex_t mutex;
	int lockfd;
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

static inline int is_valid_rsp_hdr(const char *rsp, uint8_t len, uint8_t cmd)
{
	return (rsp[0] == 0x55 && rsp[1] == len && rsp[2] == cmd);
}

static int skycmd_serial_cmd(struct skyloc_dev *dev, uint8_t cmd,
			     unsigned req_num, int rsp_num,
			     ...)
{
	char cmd_buf[8], rsp_buf[8];
	enum sp_return sprc;
	int rc, args, off;
	uint8_t len;
	va_list ap;

	va_start(ap, rsp_num);
	for (len = 0, off = 3, args = 0; args < req_num; args++) {
		rc = skycmd_arg_copy(&ap, TO_BUF, cmd_buf, off,
				     sizeof(cmd_buf) - 1);
		if (rc < 0)
			goto out;

		len += rc;
		off += rc;
	}

	cmd_buf[0] = 0x55;
	cmd_buf[1] = len + 4;
	cmd_buf[2] = cmd;
	cmd_buf[len + 3] = 0x00;

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
	sprc = sp_blocking_write(dev->port, cmd_buf, len + 4, TIMEOUT_MS);
	if (sprc < 0) {
		rc = sprc_to_errno(sprc);
		goto out_unlock;
	} else if (sprc != len + 4) {
		rc = -ETIMEDOUT;
		goto out_unlock;
	}
	if (rsp_num >= 0) {
		rc = skycmd_args_inbytes(ap, rsp_num);
		if (rc < 0)
			goto out_unlock;

		len = rc;
		if (len > sizeof(rsp_buf) - 4) {
			rc = -EINVAL;
			goto out_unlock;
		}
		/* Firstly read header */
		sprc = sp_blocking_read(dev->port, rsp_buf, 3, TIMEOUT_MS);
		if (sprc < 0) {
			rc = sprc_to_errno(sprc);
			goto out_unlock;
		} else if (sprc != 3) {
			rc = -ETIMEDOUT;
			goto out_unlock;
		} else if (!is_valid_rsp_hdr(rsp_buf, len + 4, cmd)) {
			rc = -EPROTO;
			goto out_unlock;
		}
		/* Read the rest */
		sprc = sp_blocking_read(dev->port, rsp_buf + 3,
					len + 1, TIMEOUT_MS);
		if (sprc < 0) {
			rc = sprc_to_errno(sprc);
			goto out_unlock;
		} else if (sprc != len + 1) {
			rc = -ETIMEDOUT;
			goto out_unlock;
		}
		for (off = 3, args = 0; args < rsp_num; args++) {
			rc = skycmd_arg_copy(&ap, FROM_BUF, rsp_buf, off,
					     len + 4 - 1);
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

static int skyloc_peerinfo(const struct sky_dev_ops *ops,
			   const struct sky_dev_conf *conf,
			   struct sky_peerinfo *peerinfo)
{
	return -EOPNOTSUPP;
}

static int devopen(const struct sky_dev_desc *devdesc,
			  struct skyloc_dev **dev_)
{
	struct skyloc_dev *dev;
	struct sp_port_config* spconf;
	enum sp_return sprc;

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

	rc = devopen(devdesc, &dev);
	if (rc)
		return rc;

	rc = skycmd_serial_cmd(dev, SKY_FIRMWARE_VERSION_CMD,
			       0, 3,
			       sizeof(major), &major,
			       sizeof(minor), &minor,
			       sizeof(revis), &revis);
	if (!rc)
		devdesc->firmware_version = major << 16 | minor << 8 | revis;

	devclose(dev);

	return rc;
}

static int skyloc_devslist(const struct sky_dev_ops *ops,
			   const struct sky_dev_conf *conf,
			   struct sky_dev_desc **out)
{
	struct sky_dev_desc *devdesc, *head = NULL, *tail = NULL;
	struct sp_port **ports = NULL;
	enum sp_return sprc;
	int i, rc;

	sprc = sp_list_ports(&ports);
	if (sprc < 0)
		return sprc_to_errno(sprc);

	for (i = 0; ports[i]; i++) {
		const char *desc, *name;

		desc = sp_get_port_description(ports[i]);
		name = sp_get_port_name(ports[i]);
		if (!desc || !name || strncasecmp(desc, "skysense", 8))
			continue;

		devdesc = calloc(1, sizeof(*devdesc));
		if (devdesc == NULL) {
			rc = -ENOMEM;
			goto err;
		}
		strncpy(devdesc->portname, name, sizeof(devdesc->portname));
		devdesc->dev_type = SKY_INDOOR;
		devdesc->conf = *conf;
		devdesc->opaque_ops = ops;
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

	rc = devopen(devdesc, &dev);
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

static int skyloc_serial_get_param(struct skyloc_dev *dev,
				   uint8_t param, uint16_t *val)
{
	return skycmd_serial_cmd(dev, SKY_GET_PARAMETER_CMD,
			1, 2,
			sizeof(param), &param,
			sizeof(param), &param, sizeof(*val), val);
}

static int skyloc_paramsget(struct sky_dev *dev_, struct sky_dev_params *params)
{
	struct skyloc_dev *dev;
	uint16_t val = 0;
	int rc, i;

	if (params->dev_params_bits == 0)
		return -EINVAL;

	BUILD_BUG_ON(sizeof(params->dev_params_bits) * 8 <
		     SKY_NUM_DEVPARAM);

	dev = container_of(dev_, struct skyloc_dev, dev);

	/* Fetch params from EEPROM */
	rc = skycmd_serial_cmd(dev, SKY_READ_DATA_FROM_EEP_CMD, 0, 0);
	if (rc)
		return rc;

	for (i = 0; i < SKY_NUM_DEVPARAM; i++) {
		if (!(params->dev_params_bits & (1<<i)))
				continue;
		rc = skyloc_serial_get_param(dev, i, &val);
		if (rc)
			return rc;
		params->dev_params[i] = val;
	}

	return 0;
}

static int skyloc_serial_set_param(struct skyloc_dev *dev,
				   uint8_t param, uint16_t val)
{
	return skycmd_serial_cmd(dev, SKY_SET_PARAMETER_CMD,
			2, 2,
			sizeof(param), &param, sizeof(val), &val,
			sizeof(param), &param, sizeof(val), &val);
}

static int skyloc_paramsset(struct sky_dev *dev_,
			    const struct sky_dev_params *params)
{
	struct skyloc_dev *dev;
	uint16_t val;
	int rc, i;

	if (params->dev_params_bits == 0)
		return -EINVAL;

	BUILD_BUG_ON(sizeof(params->dev_params_bits) * 8 <
		     SKY_NUM_DEVPARAM);

	dev = container_of(dev_, struct skyloc_dev, dev);
	for (i = 0; i < SKY_NUM_DEVPARAM; i++) {
		if (!(params->dev_params_bits & (1<<i)))
				continue;
		val = params->dev_params[i];
		rc = skyloc_serial_set_param(dev, i, val);
		if (rc)
			return rc;
	}
	/* Commit params to EEPROM */
	return skycmd_serial_cmd(dev, SKY_SAVE_DATA_TO_EEP_CMD, 0, 0);
}

static int skyloc_chargingstate(struct sky_dev *dev_,
				struct sky_charging_state *state)
{
	struct skyloc_dev *dev;
	uint16_t vol, cur;
	uint8_t status;
	int rc;

	dev = container_of(dev_, struct skyloc_dev, dev);

	rc = skycmd_serial_cmd(dev, SKY_GET_VOLTAGE_CMD,
			       0, 1,
			       sizeof(vol), &vol);
	if (rc)
		return rc;
	rc = skycmd_serial_cmd(dev, SKY_GET_CURRENT_CMD,
			       0, 1,
			       sizeof(cur), &cur);
	if (rc)
		return rc;
	rc = skycmd_serial_cmd(dev, SKY_GET_STATUS_CMD,
			       0, 1,
			       sizeof(status), &status);
	if (rc)
		return rc;

	state->voltage = vol;
	state->current = cur;
	state->dev_hw_state = status;

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

	return skycmd_serial_cmd(dev, SKY_RESET_CMD, 0, -1);
}

static int skyloc_autoscan(struct sky_dev *dev_, unsigned autoscan)
{
	struct skyloc_dev *dev;
	uint8_t ascan;

	dev = container_of(dev_, struct skyloc_dev, dev);
	ascan = autoscan;

	return skycmd_serial_cmd(dev, SKY_AUTOMATIC_SCAN_CMD,
				 1, 1,
				 sizeof(ascan), &ascan,
				 sizeof(ascan), &ascan);
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

static int skyloc_coveropen(struct sky_dev *dev_)
{
	return -EOPNOTSUPP;
}

static int skyloc_coverclose(struct sky_dev *dev_)
{
	return -EOPNOTSUPP;
}

static struct sky_dev_ops sky_local_devops = {
	.contype = SKY_LOCAL,
	.peerinfo = skyloc_peerinfo,
	.devslist = skyloc_devslist,
	.devopen = skyloc_devopen,
	.devclose = skyloc_devclose,
	.paramsget = skyloc_paramsget,
	.paramsset = skyloc_paramsset,
	.chargingstate = skyloc_chargingstate,
	.subscribe = skyloc_subscribe,
	.unsubscribe = skyloc_unsubscribe,
	.subscription_work = skyloc_subscription_work,
	.reset = skyloc_reset,
	.chargestart = skyloc_chargestart,
	.chargestop = skyloc_chargestop,
	.coveropen = skyloc_coveropen,
	.coverclose = skyloc_coverclose
};
sky_register_devops(&sky_local_devops);

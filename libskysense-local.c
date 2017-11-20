#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <stdarg.h>

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
};

enum {
	TO_BUF   = 0,
	FROM_BUF = 1,

	MAX_OHM  = 4000,
	MIN_OHM  = 800,
};

struct skyloc_lib {
	struct sky_lib lib;
	struct sp_port *port;
	struct sky_charging_state state;
	struct sky_dev_conf conf;
	struct sky_dev dev;
	pthread_mutex_t mutex;
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

static int skycmd_serial_cmd(struct skyloc_lib *lib, uint8_t cmd,
			     unsigned req_num, int rsp_num,
			     ...)
{
	char cmd_buf[8], rsp_buf[8];
	enum sp_return sprc;
	int rc, args, off;
	uint8_t len;
	va_list ap;

	va_start(ap, rsp_num);
	pthread_mutex_lock(&lib->mutex);
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

	sprc = sp_blocking_write(lib->port, cmd_buf, len + 4, 0);
	if (sprc < 0) {
		rc = sprc_to_errno(sprc);
		goto out;
	}
	if (rsp_num >= 0) {
		rc = skycmd_args_inbytes(ap, rsp_num);
		if (rc < 0)
			goto out;

		len = rc;
		if (len > sizeof(rsp_buf) - 4) {
			rc = -EINVAL;
			goto out;
		}
		sprc = sp_blocking_read(lib->port, rsp_buf, len + 4, 0);
		if (sprc < 0) {
			rc = sprc_to_errno(sprc);
			goto out;
		}
		for (off = 3, args = 0; args < rsp_num; args++) {
			rc = skycmd_arg_copy(&ap, FROM_BUF, rsp_buf, off,
					     len + 4 - 1);
			if (rc < 0)
				goto out;

			off += rc;
		}
	}
	rc = 0;

out:
	pthread_mutex_unlock(&lib->mutex);
	va_end(ap);

	return rc;
}

static int skyloc_devslist(struct sky_dev **head)
{
	struct sky_dev *dev, *prev = NULL;
	struct sp_port **ports = NULL;
	enum sp_return sprc;
	int i, rc;

	sprc = sp_list_ports(&ports);
	if (sprc < 0)
		return sprc_to_errno(sprc);

	*head = NULL;

	for (i = 0; ports[i]; i++) {
		const char *desc, *name;

		desc = sp_get_port_description(ports[i]);
		name = sp_get_port_name(ports[i]);
		if (!desc || !name || strncasecmp(desc, "skysense", 8))
			continue;

		dev = calloc(1, sizeof(*dev));
		if (dev == NULL) {
			rc = -ENOMEM;
			goto err;
		}
		if (*head == NULL)
			*head = dev;
		if (prev)
			prev->next = dev;
		prev = dev;
		strncpy(dev->portname, name, sizeof(dev->portname));
		/* XXX: TODO */
		dev->dev_type = SKY_INDOOR;
	}
	rc = 0;

out:
	sp_free_port_list(ports);

	return rc;

err:
	while (*head) {
		dev = *head;
		*head = (*head)->next;
		free(dev);
	}
	goto out;
}

static int skyloc_libopen(const struct sky_lib_conf *conf,
			  struct sky_lib **lib_)
{
	struct skyloc_lib *lib;
	enum sp_return sprc;

	lib = calloc(1, sizeof(*lib));
	if (!lib)
		return -ENOMEM;

	sprc = sp_get_port_by_name(conf->local.portname, &lib->port);
	if (sprc) {
		free(lib);
		return sprc_to_errno(sprc);
	}
	sprc = sp_open(lib->port, SP_MODE_READ_WRITE);
	if (sprc) {
		sp_free_port(lib->port);
		free(lib);
		return sprc_to_errno(sprc);
	}
	pthread_mutex_init(&lib->mutex, NULL);

	*lib_ = &lib->lib;

	return 0;
}

static void skyloc_libclose(struct sky_lib *lib_)
{
	struct skyloc_lib *lib;

	lib = container_of(lib_, struct skyloc_lib, lib);
	sp_close(lib->port);
	sp_free_port(lib->port);
	free(lib);
}

static int skyloc_devinfo(struct sky_lib *lib_, struct sky_dev *dev)
{
	struct skyloc_lib *lib;

	lib = container_of(lib_, struct skyloc_lib, lib);
	strncpy(dev->portname, sp_get_port_name(lib->port),
		sizeof(dev->portname));
	/* XXX: TODO: */
	dev->dev_type = SKY_INDOOR;

	return 0;
}

static int skyloc_serial_get_param(struct skyloc_lib *lib,
				   uint8_t param, uint16_t *val)
{
	return skycmd_serial_cmd(lib, SKY_GET_PARAMETER_CMD,
			1, 2,
			sizeof(param), &param,
			sizeof(param), &param, sizeof(*val), val);
}

static int skyloc_confget(struct sky_lib *lib_, struct sky_dev_conf *conf)
{
	struct skyloc_lib *lib;
	uint16_t val = 0;
	int rc, i;

	if (conf->dev_params_bits == 0)
		return -EINVAL;

	BUILD_BUG_ON(sizeof(conf->dev_params_bits) * 8 <
		     SKY_NUM_DEVPARAM);

	lib = container_of(lib_, struct skyloc_lib, lib);

	/* Fetch params from EEPROM */
	rc = skycmd_serial_cmd(lib, SKY_READ_DATA_FROM_EEP_CMD, 0, 0);
	if (rc)
		return rc;

	for (i = 0; i < SKY_NUM_DEVPARAM; i++) {
		if (!(conf->dev_params_bits & (1<<i)))
				continue;
		rc = skyloc_serial_get_param(lib, i, &val);
		if (rc)
			return rc;
		conf->dev_params[i] = val;
	}

	return 0;
}

static int skyloc_serial_set_param(struct skyloc_lib *lib,
				   uint8_t param, uint16_t val)
{
	return skycmd_serial_cmd(lib, SKY_SET_PARAMETER_CMD,
			2, 2,
			sizeof(param), &param, sizeof(val), &val,
			sizeof(param), &param, sizeof(val), &val);
}

static int skyloc_confset(struct sky_lib *lib_, struct sky_dev_conf *conf)
{
	struct skyloc_lib *lib;
	uint16_t val;
	int rc, i;

	if (conf->dev_params_bits == 0)
		return -EINVAL;

	BUILD_BUG_ON(sizeof(conf->dev_params_bits) * 8 <
		     SKY_NUM_DEVPARAM);

	lib = container_of(lib_, struct skyloc_lib, lib);
	for (i = 0; i < SKY_NUM_DEVPARAM; i++) {
		if (!(conf->dev_params_bits & (1<<i)))
				continue;
		val = conf->dev_params[i];
		rc = skyloc_serial_set_param(lib, i, val);
		if (rc)
			return rc;
	}
	/* Commit params to EEPROM */
	return skycmd_serial_cmd(lib, SKY_SAVE_DATA_TO_EEP_CMD, 0, 0);
}

static int skyloc_chargingstate(struct sky_lib *lib_,
				struct sky_charging_state *state)
{
	struct skyloc_lib *lib;
	uint16_t vol, cur;
	uint8_t status;
	int rc;

	lib = container_of(lib_, struct skyloc_lib, lib);

	rc = skycmd_serial_cmd(lib, SKY_GET_VOLTAGE_CMD,
			       0, 1,
			       sizeof(vol), &vol);
	if (rc)
		return rc;
	rc = skycmd_serial_cmd(lib, SKY_GET_CURRENT_CMD,
			       0, 1,
			       sizeof(cur), &cur);
	if (rc)
		return rc;
	rc = skycmd_serial_cmd(lib, SKY_GET_STATUS_CMD,
			       0, 1,
			       sizeof(status), &status);
	if (rc)
		return rc;

	state->voltage = vol;
	state->current = cur;
	state->dev_hw_state = status;

	return 0;
}

static int skyloc_subscribe(struct sky_lib *lib_)
{
	/* Nop for now */
	return 0;
}

static void skyloc_unsubscribe(struct sky_lib *lib_)
{
	/* Nop for now */
}

static int skyloc_subscription_work(struct sky_lib *lib_,
				    struct sky_charging_state *state)
{
	return skyloc_chargingstate(lib_, state);
}

static int skyloc_reset(struct sky_lib *lib_)
{
	struct skyloc_lib *lib;

	lib = container_of(lib_, struct skyloc_lib, lib);

	return skycmd_serial_cmd(lib, SKY_RESET_CMD, 0, -1);
}

static int skyloc_autoscan(struct sky_lib *lib_, unsigned autoscan)
{
	struct skyloc_lib *lib;
	uint8_t ascan;

	lib = container_of(lib_, struct skyloc_lib, lib);
	ascan = autoscan;

	return skycmd_serial_cmd(lib, SKY_AUTOMATIC_SCAN_CMD,
				 1, 1,
				 sizeof(ascan), &ascan,
				 sizeof(ascan), &ascan);
}

static int skyloc_chargestart(struct sky_lib *lib_)
{
	/* Start charging enabling autoscan */
	return skyloc_autoscan(lib_, 1);
}

static int skyloc_chargestop(struct sky_lib *lib_)
{
	/* Stop charging disabling autoscan */
	return skyloc_autoscan(lib_, 0);
}

static int skyloc_coveropen(struct sky_lib *lib_)
{
	return -EOPNOTSUPP;
}

static int skyloc_coverclose(struct sky_lib *lib_)
{
	return -EOPNOTSUPP;
}

struct sky_lib_ops sky_local_lib_ops = {
	.devslist = skyloc_devslist,
	.libopen = skyloc_libopen,
	.libclose = skyloc_libclose,
	.devinfo = skyloc_devinfo,
	.confget = skyloc_confget,
	.confset = skyloc_confset,
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

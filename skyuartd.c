#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <limits.h>
#include <assert.h>
#include <libserialport.h>

#include "skyuartd-cmd.h"
#include "libskysense.h"
#include "libskysense-pri.h"
#include "types.h"
#include "daemon.h"
#include "skyproto.h"
#include "crc8.h"

#define UART_MAGIC 0xe5b5

enum {
	UART_BAD_CRC       = 0xf0,
	UART_CMD_UNKNOWN   = 0xf1,
	UART_CMD_MALFORMED = 0xf2,
};

#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-Wpadded"

struct uart_packet {
	le16     magic;
	uint8_t  crc8;
	uint8_t  data_len;
	uint8_t  data[];
};

struct uart_generic_rsp {
	struct uart_packet uart_hdr;
	struct sky_rsp_hdr rsp;
};

struct uart_req {
	struct uart_packet uart_hdr;
	union sky_req      req;
};

#pragma GCC diagnostic pop

struct uartd {
	struct cli       cli;
	struct sky_conf  conf;
	struct sky_dev   *dev;
	struct sp_port   *port;
};

static int uartd_prepare_sky_conf_and_dev(struct uartd *uartd)
{
	struct sky_conf *conf = &uartd->conf;
	struct cli *cli = &uartd->cli;

	struct sky_dev_desc *devdescs, *devdesc;
	int rc;

	rc = sky_confparse(uartd->cli.conff, conf);
	if (rc) {
		sky_err("sky_confparse(): %s\n", strerror(-rc));
		return rc;
	}

	conf->contype = SKY_REMOTE;
	conf->cliport = strtol(cli->skyport, NULL, 10);
	conf->subport = conf->cliport + 1;
	strncpy(conf->hostname, cli->skyaddr,
		sizeof(conf->hostname)-1);

	rc = sky_devslist(conf, &devdescs);
	if (rc) {
		sky_err("sky_devslist(): %s\n", strerror(-rc));
		return rc;
	}

	foreach_devdesc(devdesc, devdescs) {
		if (memcmp(devdesc->dev_uuid, uartd->conf.devuuid,
			    sizeof(uartd->conf.devuuid)))
			continue;
		break;
	}
	if (!devdesc) {
		sky_err("No device found!\n");
		rc = -ENODEV;
		goto out;
	}
	rc = sky_devopen(devdesc, &uartd->dev);
	if (rc) {
		sky_err("sky_devopen(): %s\n", strerror(-rc));
		goto out;
	}

out:
	sky_devsfree(devdescs);

	return rc;
}

/*
 * FIXME: duplicate in libskysense-local.c
 */
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

static int uartd_devopen(struct uartd *uartd)
{
	struct sp_port_config* spconf;
	enum sp_return sprc;

	sprc = sp_get_port_by_name(uartd->cli.dev, &uartd->port);
	if (sprc) {
		sky_err("sp_get_port_by_name(): sprc=%d\n", sprc);
		goto out;
	}
	sprc = sp_open(uartd->port, SP_MODE_READ_WRITE);
	if (sprc) {
		sky_err("sp_open(): sprc=%d\n", sprc);
		goto free_port;
	}
	sprc = sp_new_config(&spconf);
	if (sprc) {
		sky_err("sp_new_config(): sprc=%d\n", sprc);
		goto close_port;
	}
	sprc = sp_get_config(uartd->port, spconf);
	if (sprc) {
		sky_err("sp_get_config(): sprc=%d\n", sprc);
		goto free_conf;
	}
	sprc = sp_set_config_flowcontrol(spconf, SP_FLOWCONTROL_NONE);
	if (sprc) {
		sky_err("sp_set_config_flowcontrol(): sprc=%d\n", sprc);
		goto free_conf;
	}
	sprc = sp_set_config_parity(spconf, SP_PARITY_NONE);
	if (sprc) {
		sky_err("sp_set_config_parity(): sprc=%d\n", sprc);
		goto free_conf;
	}
	sprc = sp_set_config_bits(spconf, 8);
	if (sprc) {
		sky_err("sp_set_config_bits(): sprc=%d\n", sprc);
		goto free_conf;
	}
	sprc = sp_set_config_baudrate(spconf, atoi(uartd->cli.baudrate));
	if (sprc) {
		sky_err("sp_set_baudrate(): sprc=%d\n", sprc);
		goto free_conf;
	}
	sprc = sp_set_config(uartd->port, spconf);
	if (sprc) {
		sky_err("sp_set_config(): sprc=%d\n", sprc);
		goto free_conf;
	}
	sp_free_config(spconf);

	return 0;

free_conf:
	sp_free_config(spconf);
close_port:
	sp_close(uartd->port);
free_port:
	sp_free_port(uartd->port);
out:

	return sprc_to_errno(sprc);
}

static int uartd_send_rsp(struct uartd *uartd, struct uart_packet *uart_hdr,
			  uint8_t data_len)
{
	enum sp_return sprc;
	int rc;

	uart_hdr->magic = htole16(UART_MAGIC);
	uart_hdr->data_len = data_len;
	uart_hdr->crc8 = crc8((uint8_t *)&uart_hdr->data_len,
			      data_len + sizeof(uart_hdr->data_len));

	sprc = sp_blocking_write(uartd->port, uart_hdr,
				 data_len + sizeof(*uart_hdr), 0);
	if (sprc < 0) {
		rc = sprc_to_errno(sprc);
		sky_err("sp_blocking_write(): %s\n", strerror(-rc));
		return rc;
	}

	return 0;
}

static int uartd_send_generic_rsp(struct uartd *uartd, int type, int error)
{
	struct uart_generic_rsp uart_rsp = {
		.rsp.type  = htole16(type),
		.rsp.error = htole16(error)
	};

	return uartd_send_rsp(uartd, &uart_rsp.uart_hdr, sizeof(uart_rsp.rsp));
}

static void uartd_start_charge_req(struct uartd *uartd)
{
	int rc;

	rc = sky_chargestart(uartd->dev);
	if (rc)
		sky_err("sky_chargestart(): %s\n", strerror(-rc));

	(void)uartd_send_generic_rsp(uartd, SKY_START_CHARGE_RSP, -rc);
}

static void uartd_stop_charge_req(struct uartd *uartd)
{
	int rc;

	rc = sky_chargestop(uartd->dev);
	if (rc)
		sky_err("sky_chargestop(): %s\n", strerror(-rc));

	(void)uartd_send_generic_rsp(uartd, SKY_STOP_CHARGE_RSP, -rc);
}

static void uartd_open_droneport_req(struct uartd *uartd)
{
	int rc;

	rc = sky_droneport_open(uartd->dev);
	if (rc)
		sky_err("sky_droneport_open(): %s\n", strerror(-rc));

	(void)uartd_send_generic_rsp(uartd, SKY_OPEN_DRONEPORT_RSP, -rc);
}

static void uartd_close_droneport_req(struct uartd *uartd)
{
	int rc;

	rc = sky_droneport_close(uartd->dev);
	if (rc)
		sky_err("sky_droneport_close(): %s\n", strerror(-rc));

	(void)uartd_send_generic_rsp(uartd, SKY_CLOSE_DRONEPORT_RSP, -rc);
}

static void uartd_charging_state_req(struct uartd *uartd)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-Wpadded"

	struct {
		struct uart_packet uart_hdr;
		struct sky_charging_state_rsp rsp;
	} uart_rsp;
	struct sky_charging_state state;
	int rc;

	rc = sky_chargingstate(uartd->dev, &state);
	if (rc)
		sky_err("sky_chargingstate(): %s\n", strerror(-rc));

	memset(&uart_rsp, 0, sizeof(uart_rsp));
	uart_rsp.rsp.hdr.type  = htole16(SKY_CHARGING_STATE_RSP);
	uart_rsp.rsp.hdr.error = htole16(-rc);
	if (!rc) {
		uart_rsp.rsp.dev_hw_state = htole32(state.dev_hw_state);
		uart_rsp.rsp.current_mA = htole16(state.current_mA);
		uart_rsp.rsp.voltage_mV = htole16(state.voltage_mV);
		uart_rsp.rsp.state_of_charge = htole16(state.state_of_charge);
		uart_rsp.rsp.until_full_secs = htole16(state.until_full_secs);
		uart_rsp.rsp.charging_secs = htole16(state.charging_secs);
		uart_rsp.rsp.mux_temperature_C
			= htole16(state.mux_temperature_C);
		uart_rsp.rsp.sink_temperature_C
			= htole16(state.sink_temperature_C);
		uart_rsp.rsp.energy_mWh = htole32(state.energy_mWh);
		uart_rsp.rsp.charge_mAh = htole32(state.charge_mAh);
		uart_rsp.rsp.mux_humidity_perc = state.mux_humidity_perc;
		uart_rsp.rsp.link_quality_factor = state.link_quality_factor;
		uart_rsp.rsp.tx.bytes       = htole16(state.tx.bytes);
		uart_rsp.rsp.tx.packets     = htole16(state.tx.packets);
		uart_rsp.rsp.tx.err_bytes   = htole16(state.tx.err_bytes);
		uart_rsp.rsp.tx.err_packets = htole16(state.tx.err_packets);
		uart_rsp.rsp.rx.bytes       = htole16(state.rx.bytes);
		uart_rsp.rsp.rx.packets     = htole16(state.rx.packets);
		uart_rsp.rsp.rx.err_bytes   = htole16(state.rx.err_bytes);
		uart_rsp.rsp.rx.err_packets = htole16(state.rx.err_packets);

	}

	(void)uartd_send_rsp(uartd, &uart_rsp.uart_hdr, sizeof(uart_rsp.rsp));
}

static void uartd_droneport_state_req(struct uartd *uartd)
{
	struct {
		struct uart_packet uart_hdr;
		struct sky_droneport_state_rsp rsp;
	} uart_rsp;
	struct sky_droneport_state state;
	int rc;

	rc = sky_droneport_state(uartd->dev, &state);
	if (rc)
		sky_err("sky_droneport_state(): %s\n", strerror(-rc));

	memset(&uart_rsp, 0, sizeof(uart_rsp));
	uart_rsp.rsp.hdr.type  = htole16(SKY_DRONEPORT_STATE_RSP);
	uart_rsp.rsp.hdr.error = htole16(-rc);
	if (!rc) {
		uart_rsp.rsp.status = htole32(state.status);
	}

	(void)uartd_send_rsp(uartd, &uart_rsp.uart_hdr, sizeof(uart_rsp.rsp));
}

__attribute__((unused))
static int do_some_tests(void)
{
	enum sp_return sprc;
	int rc;

	struct uartd uartd;

	uartd.cli.dev      = "/dev/ttyUSB0";
	uartd.cli.baudrate = "9600";

	rc = uartd_devopen(&uartd);
	if (rc)
		return 1;

	{
		struct {
			struct uart_packet uart_hdr;
			struct sky_req_hdr req;
		} uart_req;
		uart_req.req.type  = htole16(SKY_STOP_CHARGE_REQ);
		(void)uartd_send_rsp(&uartd, &uart_req.uart_hdr, sizeof(uart_req.req));


		struct uart_generic_rsp gen_rsp;

		/* Firstly read header, wait infinitely */
		sprc = sp_blocking_read(uartd.port, &gen_rsp.uart_hdr,
					sizeof(gen_rsp.uart_hdr), 0);
		if (sprc < 0) {
			rc = sprc_to_errno(sprc);
			sky_err("sp_blocking_read(): %s\n", strerror(-rc));
			return 1;
		}
		if (gen_rsp.uart_hdr.magic != htole16(UART_MAGIC)) {
			sky_err("warning: garbage on the wire\n");
			return 1;
		}
		if (gen_rsp.uart_hdr.data_len > sizeof(gen_rsp.rsp)) {
			sky_err("warning: data length is tooo big %d\n",
				gen_rsp.uart_hdr.data_len);
			return 1;
		}

		sprc = sp_blocking_read(uartd.port, &gen_rsp.rsp,
					sizeof(gen_rsp.rsp), 0);
		if (sprc < 0) {
			rc = sprc_to_errno(sprc);
			sky_err("sp_blocking_read(): %s\n", strerror(-rc));
			return 1;
		}

		printf("\n>> type=%d, error=%d\n", gen_rsp.rsp.type,
		       gen_rsp.rsp.error);


	}
	{
		struct {
			struct uart_packet uart_hdr;
			struct sky_req_hdr req;
		} uart_req;
		uart_req.req.type  = htole16(SKY_START_CHARGE_REQ);
		(void)uartd_send_rsp(&uartd, &uart_req.uart_hdr, sizeof(uart_req.req));

		struct uart_generic_rsp gen_rsp;

		/* Firstly read header, wait infinitely */
		sprc = sp_blocking_read(uartd.port, &gen_rsp.uart_hdr,
					sizeof(gen_rsp.uart_hdr), 0);
		if (sprc < 0) {
			rc = sprc_to_errno(sprc);
			sky_err("sp_blocking_read(): %s\n", strerror(-rc));
			return 1;
		}
		if (gen_rsp.uart_hdr.magic != htole16(UART_MAGIC)) {
			sky_err("warning: garbage on the wire\n");
			return 1;
		}
		if (gen_rsp.uart_hdr.data_len > sizeof(gen_rsp.rsp)) {
			sky_err("warning: data length is tooo big %d\n",
				gen_rsp.uart_hdr.data_len);
			return 1;
		}

		sprc = sp_blocking_read(uartd.port, &gen_rsp.rsp,
					sizeof(gen_rsp.rsp), 0);
		if (sprc < 0) {
			rc = sprc_to_errno(sprc);
			sky_err("sp_blocking_read(): %s\n", strerror(-rc));
			return 1;
		}

		printf("\n>> type=%d, error=%d\n", gen_rsp.rsp.type,
		       gen_rsp.rsp.error);
	}
	{
		struct {
			struct uart_packet uart_hdr;
			struct sky_req_hdr req;
		} uart_req;
		uart_req.req.type  = htole16(SKY_CHARGING_STATE_REQ);
		(void)uartd_send_rsp(&uartd, &uart_req.uart_hdr, sizeof(uart_req.req));

		struct {
			struct uart_packet uart_hdr;
			struct sky_charging_state_rsp rsp;
		} __attribute__((packed)) chg_rsp;

		/* Firstly read header, wait infinitely */
		sprc = sp_blocking_read(uartd.port, &chg_rsp.uart_hdr,
					sizeof(chg_rsp.uart_hdr), 0);
		if (sprc < 0) {
			rc = sprc_to_errno(sprc);
			sky_err("sp_blocking_read(): %s\n", strerror(-rc));
			return 1;
		}
		if (chg_rsp.uart_hdr.magic != htole16(UART_MAGIC)) {
			sky_err("warning: garbage on the wire\n");
			return 1;
		}
		if (chg_rsp.uart_hdr.data_len > sizeof(chg_rsp.rsp)) {
			sky_err("warning: data length is tooo big %d\n",
				chg_rsp.uart_hdr.data_len);
			return 1;
		}

		sprc = sp_blocking_read(uartd.port, &chg_rsp.rsp,
					sizeof(chg_rsp.rsp), 0);
		if (sprc < 0) {
			rc = sprc_to_errno(sprc);
			sky_err("sp_blocking_read(): %s\n", strerror(-rc));
			return 1;
		}

		printf("\n>> type=%d, error=%d\n", chg_rsp.rsp.hdr.type,
		       chg_rsp.rsp.hdr.error);
		printf(">> hw_state=%d\n", chg_rsp.rsp.dev_hw_state);
		printf(">> voltage=%d\n", chg_rsp.rsp.voltage_mV);
		printf(">> current=%d\n", chg_rsp.rsp.current_mA);
	}
	{
		struct {
			struct uart_packet uart_hdr;
			struct sky_req_hdr req;
		} uart_req;
		uart_req.req.type  = htole16(SKY_DRONEPORT_STATE_REQ);
		(void)uartd_send_rsp(&uartd, &uart_req.uart_hdr, sizeof(uart_req.req));

		struct {
			struct uart_packet uart_hdr;
			struct sky_droneport_state_rsp rsp;
		} __attribute__((packed)) dp_rsp;

		/* Firstly read header, wait infinitely */
		sprc = sp_blocking_read(uartd.port, &dp_rsp.uart_hdr,
					sizeof(dp_rsp.uart_hdr), 0);
		if (sprc < 0) {
			rc = sprc_to_errno(sprc);
			sky_err("sp_blocking_read(): %s\n", strerror(-rc));
			return 1;
		}
		if (dp_rsp.uart_hdr.magic != htole16(UART_MAGIC)) {
			sky_err("warning: garbage on the wire\n");
			return 1;
		}
		if (dp_rsp.uart_hdr.data_len > sizeof(dp_rsp.rsp)) {
			sky_err("warning: data length is tooo big %d\n",
				dp_rsp.uart_hdr.data_len);
			return 1;
		}

		sprc = sp_blocking_read(uartd.port, &dp_rsp.rsp,
					sizeof(dp_rsp.rsp), 0);
		if (sprc < 0) {
			rc = sprc_to_errno(sprc);
			sky_err("sp_blocking_read(): %s\n", strerror(-rc));
			return 1;
		}

		printf("\n>> type=%d, error=%d\n", dp_rsp.rsp.hdr.type,
		       dp_rsp.rsp.hdr.error);
		printf(">> dp_state=%d\n", dp_rsp.rsp.status);
	}


	printf("Good\n");

	return 0;
}

int main (int argc, char **argv)
{
	struct uartd uartd = {
		.port = NULL
	};
	enum sp_return sprc;
	int rc;

#if 0
	do_some_tests();
	return 1;
#endif

	rc = cli_parse(argc, argv, &uartd.cli);
	if (rc) {
		sky_err("%s\n", cli_usage);
		return 1;
	}

	/*
	 * Daemonize before creating zmq socket, ZMQ is written by
	 * people who are not able to deal with fork() gracefully.
	 */
	if (uartd.cli.daemon)
		sky_daemonize(uartd.cli.pidf);

	rc = uartd_prepare_sky_conf_and_dev(&uartd);
	if (rc)
		return 1;

	rc = uartd_devopen(&uartd);
	if (rc)
		return 1;

	while (1) {
		struct uart_req req;
		le16 type, error;

		/* Firstly read header, wait infinitely */
		sprc = sp_blocking_read(uartd.port, &req.uart_hdr,
					sizeof(req.uart_hdr), 0);
		if (sprc < 0) {
			rc = sprc_to_errno(sprc);
			sky_err("sp_blocking_read(): %s\n", strerror(-rc));
			goto out;
		}
		if (req.uart_hdr.magic != htole16(UART_MAGIC)) {
			sky_err("warning: garbage on the wire\n");
			type = SKY_UNKNOWN_REQRSP;
			error = UART_CMD_MALFORMED;
			goto reply_w_error;
		}
		if (req.uart_hdr.data_len > sizeof(req.req)) {
			sky_err("warning: data length is tooo big %d\n",
				req.uart_hdr.data_len);
			type = SKY_UNKNOWN_REQRSP;
			error = UART_CMD_MALFORMED;
			goto reply_w_error;
		}

		/* Read the rest, wait infinitely */
		sprc = sp_blocking_read(uartd.port, &req.uart_hdr.data,
					req.uart_hdr.data_len, 0);
		if (sprc < 0) {
			rc = sprc_to_errno(sprc);
			sky_err("sp_blocking_read(): %s\n", strerror(-rc));
			goto out;
		}
		if (req.uart_hdr.crc8 !=
		    crc8((uint8_t *)&req.uart_hdr.data_len,
			 req.uart_hdr.data_len + sizeof(req.uart_hdr.data_len))) {
			sky_err("warning: incorrect crc8 for request %d\n", req.req.hdr.type);
			type = req.req.hdr.type + 1;
			error = UART_BAD_CRC;
			goto reply_w_error;
		}

		switch (req.req.hdr.type) {
		case SKY_START_CHARGE_REQ:
			uartd_start_charge_req(&uartd);
			break;
		case SKY_STOP_CHARGE_REQ:
			uartd_stop_charge_req(&uartd);
			break;
		case SKY_OPEN_DRONEPORT_REQ:
			uartd_open_droneport_req(&uartd);
			break;
		case SKY_CLOSE_DRONEPORT_REQ:
			uartd_close_droneport_req(&uartd);
			break;
		case SKY_CHARGING_STATE_REQ:
			uartd_charging_state_req(&uartd);
			break;
		case SKY_DRONEPORT_STATE_REQ:
			uartd_droneport_state_req(&uartd);
			break;
		default:
			type = SKY_UNKNOWN_REQRSP;
			error = UART_CMD_UNKNOWN;
			goto reply_w_error;
		}

		continue;

reply_w_error:
		rc = uartd_send_generic_rsp(&uartd, type, error);
		if (rc)
			goto out;
	}

out:
	sp_close(uartd.port);
	sp_free_port(uartd.port);
	sky_devclose(uartd.dev);

	return 0;
}

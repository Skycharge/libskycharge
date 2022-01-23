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

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <getopt.h>
#include <time.h>

#include "libskysink.h"
#include "crc8.h"

#define stringify_1(x...)	#x
#define stringify(x...)	stringify_1(x)

#define err(fmt, ...)							\
	fprintf(stderr, __FILE__ ":%s():" stringify(__LINE__) ": " fmt, \
		__func__, ##__VA_ARGS__)

static int setup_termios(int fd, int speed)
{
	struct termios tty;
	int rc;

	if (tcgetattr(fd, &tty) < 0) {
		rc = -errno;
		perror("tcgetattr()");
		return rc;
	}

	cfsetospeed(&tty, (speed_t)speed);
	cfsetispeed(&tty, (speed_t)speed);

	tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;         /* 8-bit characters */
	tty.c_cflag &= ~PARENB;     /* no parity bit */
	tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
	tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

	/* Setup for non-canonical mode */
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	/* Fetch byte as it become available */
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		rc = -errno;
		perror("tcsetattr()");
		return rc;
	}

	return 0;
}

static int read_tty(int fd, void *buf, size_t size)
{
	ssize_t len = 0;
	size_t off = 0;

	for (off = 0; off < size; off += len) {
		len = read(fd, buf + off, size - off);
		if (len < 0)
			return len;
        }

	return size;
}

static int recv_passthru(int fd, void *buf, size_t maxlen)
{
	struct uart_cmd_hdr hdr = {
		.magic = 0x42,
		.crc   = 0,
		.len   = 0,
		.type  = USR_CMD_PASSTHRU_MSG_RECV,
	};
	uint8_t crc;
	int len;

	hdr.crc = crc8((void *)&hdr + 2, sizeof(hdr) - 2, 0);

	/* Write request */
	len = write(fd, &hdr, sizeof(hdr));
	if (len < 0) {
		len = -errno;
		perror("write()");
		return len;
	} else if (len != sizeof(hdr)) {
		err("write() failed, written not expected size\n");
		return -EIO;
	}
	tcdrain(fd);

	/* Read response header */
	len = read_tty(fd, &hdr, sizeof(hdr));
	if (len < 0) {
		len = -errno;
		perror("read()");
		return len;
	} else if (len != sizeof(hdr)) {
		err("read() failed, read not expected size\n");
		return -EIO;
	}
	if (hdr.type != 0) {
		err("Got wrong .type %x, expected 0\n",
		    hdr.type);
		return -EPROTO;
	}
	crc = crc8((void *)&hdr + 2, sizeof(hdr) - 2, 0);

	len = 0;
	if (hdr.len) {
		if (hdr.len > maxlen) {
			err("Input buffer size is not enough\n");
			return -EOVERFLOW;
		}
		/* Read response buffer */
		len = read_tty(fd, buf, hdr.len);
		if (len < 0) {
			len = -errno;
			perror("read()");
			return len;
		} else if (len != hdr.len) {
			err("read() failed, read not expected size\n");
			return -EIO;
		}
		crc = crc8(buf, hdr.len, crc);
	}
	if (hdr.crc != crc) {
		err("Got wrong .crc %x, expected %x\n",
		    hdr.crc, crc);
		return -EPROTO;
	}

	return len;
}

static int send_passthru(int fd, const void *buf, size_t sz)
{
	struct uart_cmd_hdr hdr = {
		.magic = 0x42,
		.crc   = 0,
		.len   = sz,
		.type  = USR_CMD_PASSTHRU_MSG_SEND,
	};
	uint8_t crc;
	int len;

	crc = crc8((void *)&hdr + 2, sizeof(hdr) - 2, 0);
	hdr.crc = crc8(buf, sz, crc);

	/* Write request header */
	len = write(fd, &hdr, sizeof(hdr));
	if (len < 0) {
		len = -errno;
		perror("write()");
		return len;
	} else if (len != sizeof(hdr)) {
		err("write() failed, written not expected size\n");
		return -EIO;
	}

	/* Write request buffer */
	len = write(fd, buf, hdr.len);
	if (len < 0) {
		len = -errno;
		perror("write()");
		return len;
	} else if (len != hdr.len) {
		err("write() failed, written not expected size\n");
		return -EIO;
	}

	tcdrain(fd);

	/* Read response header */
	len = read_tty(fd, &hdr, sizeof(hdr));
	if (len < 0) {
		len = -errno;
		perror("read()");
		return len;
	} else if (len != sizeof(hdr)) {
		err("read() failed, read not expected size\n");
		return -EIO;
	}
	if (hdr.type != 0) {
		err("Got wrong .type %x, expected 0\n",
		    hdr.type);
		return -EPROTO;
	}

	crc = crc8((void *)&hdr + 2, sizeof(hdr) - 2, 0);
	if (hdr.crc != crc) {
		err("Got wrong .crc %x, expected %x\n",
		    hdr.crc, crc);
		return -EPROTO;
	}

	return 0;
}

static int send_handshake(int fd)
{
	struct uart_cmd_handshake handshake = {
		.hdr.magic = 0x42,
		.hdr.crc   = 0,
		.hdr.len   = 1,
		.hdr.type  = USR_CMD_HANDSHAKE,
		.mode      = PASSTHRU_SYNC,
	};
	uint8_t crc;
	int len;

	handshake.hdr.crc = crc8((void *)&handshake.hdr + 2,
				 sizeof(handshake) - 2, 0);

	/* Write request header */
	len = write(fd, &handshake, sizeof(handshake));
	if (len < 0) {
		len = -errno;
		perror("write()");
		return len;
	} else if (len != sizeof(handshake)) {
		err("write() failed, written not expected size\n");
		return -EIO;
	}
	tcdrain(fd);

	/* Receive response header */
	len = read_tty(fd, &handshake.hdr, sizeof(handshake.hdr));
	if (len < 0) {
		len = -errno;
		perror("read()");
		return len;
	} else if (len != sizeof(handshake.hdr)) {
		err("read() failed, read not expected size\n");
		return -EIO;
	}
	if (handshake.hdr.type != 0) {
		err("Got wrong .type %x, expected 0\n",
		    handshake.hdr.type);
		return -EPROTO;
	}

	crc = crc8((void *)&handshake.hdr + 2, sizeof(handshake.hdr) - 2, 0);
	if (handshake.hdr.crc != crc) {
		err("Got wrong .crc %x, expected %x\n",
		    handshake.hdr.crc, crc);
		return -EPROTO;
	}

	return 0;
}

static int recv_sink_state(int fd, struct sink_state *state)
{
	struct uart_cmd_hdr hdr = {
		.magic = 0x42,
		.crc   = 0,
		.len   = 0,
		.type  = USR_CMD_GET_SINK_STATE,
	};
	uint8_t crc;
	int len;

	hdr.crc = crc8((void *)&hdr + 2, sizeof(hdr) - 2, 0);

	/* Write request header */
	len = write(fd, &hdr, sizeof(hdr));
	if (len < 0) {
		len = -errno;
		perror("write()");
		return len;
	} else if (len != sizeof(hdr)) {
		err("write() failed, written not expected size\n");
		return -EIO;
	}
	tcdrain(fd);

	/* Read response header */
	len = read_tty(fd, &hdr, sizeof(hdr));
	if (len < 0) {
		len = -errno;
		perror("read()");
		return len;
	} else if (len != sizeof(hdr)) {
		err("read() failed, read not expected size\n");
		return -EIO;
	}
	if (hdr.type != 0) {
		err("Got wrong .type %x, expected 0\n",
		    hdr.type);
		return -EPROTO;
	}
	crc = crc8((void *)&hdr + 2, sizeof(hdr) - 2, 0);

	if (hdr.len != sizeof(*state)) {
		err("Got wrong .len\n");
		return -EPROTO;
	}

	/* Read state */
	len = read_tty(fd, state, hdr.len);
	if (len < 0) {
		len = -errno;
		perror("read()");
		return len;
	} else if (len != hdr.len) {
		err("read() failed, read not expected size\n");
		return -EIO;
	}
	crc = crc8(state, hdr.len, crc);

	if (hdr.crc != crc) {
		err("Got wrong .crc %x, expected %x\n",
		    hdr.crc, crc);
		return -EPROTO;
	}

	return 0;
}

static int report_time;

static struct option long_options[] = {
	{"report-time", no_argument, &report_time, 1},
	{0, 0, 0, 0}
};

static void usage(void)
{
	printf("Usage: tty-sink-pong <tty-device> [OPTIONS]\n"
	       "Options:\n"
	       "   --report-time - sends time string to the MUX each second\n");
}

int main(int argc, char *argv[])
{
	int option_index = 0;
	int rc, fd;

	if (argc < 2) {
		usage();
		return -1;
	}

	(void)getopt_long(argc, argv, "", long_options, &option_index);

	fd = open(argv[1], O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		perror("open()");
		return -1;
	}
	rc = setup_termios(fd, B9600);
	if (rc)
		return -1;

	rc = send_handshake(fd);
	if (rc)
		return -1;

	while (1) {
		struct sink_state state;
		struct tm *tmp;
		char timestr[30];
		size_t timelen;
		time_t t;

		t = time(NULL);
		tmp = localtime(&t);

		timelen = strftime(timestr, sizeof(timestr),
				   "%Y-%m-%d %H:%M:%S\n", tmp);

		rc = recv_sink_state(fd, &state);
		if (rc)
			return -1;

		printf("Timestamp:   %s"
		       "Voltage      %.3f V\n"
		       "Temperature  %u C\n"
		       "Passthru     %u bytes\n\n",
		       timestr,
		       state.voltage_mV / 1000.0f,
		       state.sink_temperature_C,
		       state.passthru_recv_bytes);

		if (state.passthru_recv_bytes) {
			char buf[128];
			int len;

			/* Get passthru */
			len = recv_passthru(fd, buf, sizeof(buf));
			if (len < 0)
				return -1;

			/* Pong it back */
			rc = send_passthru(fd, buf, len);
			if (rc)
				return -1;
		}
		if (report_time) {
			rc = send_passthru(fd, timestr, timelen);
			if (rc)
				return -1;
		}
		sleep(1);
	}

	return 0;
}

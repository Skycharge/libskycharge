#include <assert.h>
#include <endian.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdlib.h>
#include <sys/file.h>

#include <libserialport.h>

#include "types.h"
#include "bms-btle.h"

#if 0
#define bms_log printf
#else
static inline void bms_log() {}
#endif

enum {
	TIMEOUT_MS     = 1000,  /* Timeout to request data from BTLE */
	DATA_EXPIRE_MS = 30000, /* When data expires if no updates */
};

struct bms {
	struct sp_port *port;
	pthread_mutex_t mutex;
	pthread_t thread;
	bool stopped;
	int lockfd;

	unsigned long long expire_msecs;
	uint16_t charge_perc;
	uint32_t charge_volt;
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

static inline uint8_t crc8(unsigned char *data, size_t n)
{
	unsigned char cks = 0, i;

	for (i = 0; i < n; i++)
		cks = crc_tab[cks ^ data[i]];

	return cks;
}

static inline unsigned long long msecs_epoch(void)
{
	struct timespec ts;
	unsigned long long msecs;

	clock_gettime(CLOCK_REALTIME, &ts);
	msecs  = ts.tv_sec * 1000ull;
	msecs += ts.tv_nsec / 1000000ull;

	return msecs;
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

	bms_log(">>> sent ATH\n");

	sprc = bms_suck_data(port);
	if (sprc)
		return sprc;

	/* Then disconnect */
	sprc = sp_blocking_write(port, "<DS>", 4, TIMEOUT_MS);
	if (sprc < 0)
		return sprc_to_errno(sprc);

	bms_log(">>> sent <DS>\n");

	sprc = bms_suck_data(port);
	if (sprc)
		return sprc;

	/* And again hang up, \n at the beginning to separate <DS> from ATH */
	sprc = sp_blocking_write(port, "\nATH\r", 5, TIMEOUT_MS);
	if (sprc < 0)
		return sprc;

	bms_log(">>> sent ATH\n");

	sprc = bms_suck_data(port);

	bms_log(">>> suck_data():%d\n", sprc);

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

/**
 * compar_peers() - sorts peers by signal, from max -dB to min -dB,
 *                  i.e. finds nearest.
 */
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

	bms_log(">> %s\n", __func__);

	repeat_num = REPEAT_NUM;
repeat:
	sprc = sp_blocking_write(port, "ATS\r", 4, TIMEOUT_MS);
	if (sprc < 0)
		return sprc;

	while (peers_num < ARRAY_SIZE(peers)) {
		sprc = bms_read_line(port, buf, sizeof(buf));

		bms_log(">> %s: read_line: %d\n", __func__, sprc);

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

	bms_log(">>> sent ATD\n");

	repeat_num = REPEAT_NUM;
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

	bms_log(">> send DATA SEQUENCE\n");

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

	bms_log("READ: %d, sz %zu\n", sprc, sizeof(bms_data));
	bms_log("CRC : %x, expected %x\n", crc8(buf, 138), bms_data.crc);

	*charge_perc = be16toh(bms_data.charge_perc);
	*charge_volt = be32toh(bms_data.charge_volt);

	return 1;
}

static int bms_portopen(const char *portname, struct bms *bms)
{
	struct sp_port_config* spconf;
	enum sp_return sprc;

	sprc = sp_get_port_by_name(portname, &bms->port);
	if (sprc)
		goto free_dev;

	sprc = sp_open(bms->port, SP_MODE_READ_WRITE);
	if (sprc)
		goto free_port;

	sprc = sp_new_config(&spconf);
	if (sprc)
		goto close_port;

	sprc = sp_get_config(bms->port, spconf);
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

	sprc = sp_set_config(bms->port, spconf);
	if (sprc)
		goto free_conf;

	/*
	 * TODO: this is very ugly, but shitty devserialport does not
	 * provide any way to get fd or any lock mechanism.  So we have
	 * to do locking ourselves.
	 */
	bms->lockfd = open(portname, O_RDWR);
	if (bms->lockfd < 0) {
		sprc = SP_ERR_FAIL;
		goto free_conf;
	}
	sp_free_config(spconf);
	pthread_mutex_init(&bms->mutex, NULL);

	return 0;

free_conf:
	sp_free_config(spconf);
close_port:
	sp_close(bms->port);
free_port:
	sp_free_port(bms->port);
free_dev:

	return sprc;
}

static int bms_lock(struct bms *bms)
{
	int rc;

	rc = flock(bms->lockfd, LOCK_EX);
	if (rc < 0)
		return -errno;

	return 0;
}

static void bms_unlock(struct bms *bms)
{
	(void)flock(bms->lockfd, LOCK_UN);
}

static int bms_request_data(struct bms *bms)
{
	enum sp_return sprc;

	uint16_t charge_perc;
	uint32_t charge_volt;

	while (1) {
		bms_log("START CONNECTING\n");

		sprc = bms_to_init_state(bms->port);
		if (sprc)
			return sprc_to_errno(sprc);

		sprc = bms_connect_nearest(bms->port);
		if (sprc < 0)
			return sprc_to_errno(sprc);
		if (sprc == 0)
			/* Nothing found */
			continue;

		sprc = bms_request_peer(bms->port, &charge_perc, &charge_volt);
		if (sprc < 0)
			return sprc_to_errno(sprc);
		if (sprc == 0)
			continue;

		charge_perc = min(charge_perc, 100);

		pthread_mutex_lock(&bms->mutex);
		bms->expire_msecs = msecs_epoch() + DATA_EXPIRE_MS;
		bms->charge_perc = charge_perc;
		bms->charge_volt = charge_volt;
		pthread_mutex_unlock(&bms->mutex);

		return 0;
	}
}

static void *bms_worker(void *arg)
{
	struct bms *bms = arg;
	int rc;

	while (!bms->stopped) {
		rc = bms_lock(bms);
		if (rc) {
			bms_log(">> bms_lock(): %d\n", rc);
			usleep(TIMEOUT_MS * 1000);
			continue;
		}

		(void)bms_request_data(bms);

		bms_unlock(bms);
	}

	return NULL;
}

int bms_open(struct bms **_bms)
{
	struct sp_port **ports = NULL;
	const char *name = NULL;
	enum sp_return sprc;
	struct bms *bms;
	int i, rc;

	bms = calloc(1, sizeof(*bms));
	if (!bms)
		return -ENOMEM;

	sprc = sp_list_ports(&ports);
	if (sprc < 0) {
		free(bms);
		return sprc_to_errno(sprc);
	}
	for (i = 0; ports[i]; i++) {
		const char *desc;

		desc = sp_get_port_description(ports[i]);
		name = sp_get_port_name(ports[i]);
		if (!desc || !name ||
		    strncasecmp(desc, "low energy dongle", 17)) {
			name = NULL;
			continue;
		}
		bms_log(">> %s, %s\n", desc, name);
		break;
	}
	if (!name) {
		free(bms);
		sp_free_port_list(ports);
		return -ENODEV;
	}
	rc = bms_portopen(name, bms);
	if (rc) {
		free(bms);
		sp_free_port_list(ports);
		return rc;
	}
	sp_free_port_list(ports);

	rc = pthread_create(&bms->thread, NULL, bms_worker, bms);
	if (rc) {
		bms->thread = 0;
		bms_close(bms);
		return -rc;
	}

	*_bms = bms;

	return 0;
}

void bms_close(struct bms *bms)
{
	if (bms->thread) {
		bms->stopped = true;
		pthread_join(bms->thread, NULL);
	}
	close(bms->lockfd);
	sp_close(bms->port);
	sp_free_port(bms->port);
	free(bms);
}

int bms_request_nearest(struct bms *bms, uint16_t *perc, uint32_t *volt)
{
	int rc = -ENODATA;

	pthread_mutex_lock(&bms->mutex);
	if (msecs_epoch() < bms->expire_msecs) {
		if (perc)
			*perc = bms->charge_perc;
		if (volt)
			*volt = bms->charge_volt;
		rc = 0;
	}
	pthread_mutex_unlock(&bms->mutex);

	return rc;
}

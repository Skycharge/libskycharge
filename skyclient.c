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

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <endian.h>
#include <limits.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <netdb.h>
#include <fcntl.h>
#include <zlib.h>
#include <math.h>
#include <regex.h>
#include <libelf.h>
#include <gelf.h>

#include "libskycharge.h"
#include "libskycharge-pri.h"
#include "skyclient-cmd.h"
#include "version.h"
#include "types.h"

#define CONFFILE         "/etc/skycharge.conf"
#define SINKFWFILE       "/usr/lib/firmware/skysink.elf"
#define PROGRESSBAR      "=================================================="
#define PROGRESSBARWIDTH (sizeof(PROGRESSBAR) - 1)

struct progress_bar_state {
	unsigned long long prev_ts_ms;
	unsigned long long avg_eta_msecs;
	unsigned           prev_text_len;
	double             prev_perc;
};

void print_progress(struct progress_bar_state *s,
		    double percentage, const char *text)
{
	unsigned long long eta_msecs = 0;
	unsigned min, sec, len_diff;
	unsigned val, lpad, rpad;

	percentage = clamp_val(percentage, 0.0, 1.0);
	val = percentage * 100;
	lpad = percentage * PROGRESSBARWIDTH;
	rpad = PROGRESSBARWIDTH - lpad;

	/* ETA estimation */
	if (s->prev_ts_ms) {
		double left;

		left = (1.0 - percentage) / (percentage - s->prev_perc);
		/* Integrate and add 1s to have smooth ending */
		eta_msecs = (msecs_epoch() - s->prev_ts_ms) * left + 1000;
	}
	s->prev_ts_ms = msecs_epoch();
	s->prev_perc  = percentage;

	if (percentage >= 1.0) {
		s->avg_eta_msecs = 0;
	} else {
		s->avg_eta_msecs = ema(s->avg_eta_msecs, eta_msecs, 0.8);
	}

	seconds_to_hms(s->avg_eta_msecs/1000, NULL, &min, &sec);

	if (s->prev_text_len > strlen(text)) {
		len_diff = s->prev_text_len - strlen(text);
	} else {
		len_diff = 0;
	}
	s->prev_text_len = strlen(text);

	printf("\r%3d%% |%.*s%*s| %02um:%02us | %s%*s", val, lpad,
	       PROGRESSBAR, rpad, "", min, sec, text, len_diff, "");
	fflush(stdout);
}

static inline unsigned int sky_dev_desc_crc32(struct sky_dev_desc *devdesc)
{
	uLong crc;

	crc = crc32(0L, Z_NULL, 0);
	crc = crc32(crc, (void *)devdesc->dev_uuid, sizeof(devdesc->dev_uuid));
	crc = crc32(crc, (void *)devdesc->portname, strlen(devdesc->portname));

	return crc;
}

static bool resolve_hostname_and_connection_probe(struct cli *cli)
{
	struct addrinfo hints, *res, *rp;
	bool resolved = false;
	int rc;

	memset(&hints, 0, sizeof (hints));
	hints.ai_family   = AF_INET; /* IPv4 only */
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags    = AI_NUMERICSERV;

	rc = getaddrinfo(cli->addr, cli->port, &hints, &res);
	if (rc) {
		if (rc != EAI_NONAME)
			sky_err("getaddrinfo(%s): %s\n", cli->addr,
				gai_strerror(rc));
		goto out;
	}
	for (rp = res; rp != NULL; rp = rp->ai_next) {
		struct timeval timeout;
		int fd;

		fd = socket(rp->ai_family, rp->ai_socktype,
			    rp->ai_protocol);
		if (fd == -1)
			continue;

		/* Few seconds should be enough */
		timeout.tv_sec = 5;
		timeout.tv_usec = 0;
		setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO,
			   &timeout, sizeof(timeout));

		rc = connect(fd, rp->ai_addr, rp->ai_addrlen);
		resolved = (rc == 0);
		close(fd);
		break;
	}
	freeaddrinfo(res);
out:
	return resolved;
}


/**
 * Unix UTC time to ISO8601, no timezone adjustment.
 * example: 2007-12-11T23:38:51.033Z
 *
 * Taken from gpsd/gpsutils.c
 */
static void unix_to_iso8601(double fixtime, char isotime[], size_t len)
{
	struct tm when;
	double integral, fractional;
	time_t intfixtime;
	char timestr[30];
	char fractstr[10];

	fractional = modf(fixtime, &integral);
	intfixtime = (time_t) integral;

	(void)gmtime_r(&intfixtime, &when);
	(void)strftime(timestr, sizeof(timestr), "%Y-%m-%dT%H:%M:%S", &when);
	/*
	 * Do not mess casually with the number of decimal digits in the
	 * format!  Most GPSes report over serial links at 0.01s or 0.001s
	 * precision.
	 */
	(void)snprintf(fractstr, sizeof(fractstr), "%.3f", fractional);
	/* add fractional part, ignore leading 0; "0.2" -> ".2" */
	(void)snprintf(isotime, len, "%s%sZ", timestr, strchr(fractstr,'.'));
}

static void sky_prepare_conf(struct cli *cli, struct sky_conf *conf)
{
	const char *conffile = cli->conff ?: CONFFILE;
	int rc;

	rc = sky_confparse(conffile, conf);

	if (cli->addr && cli->port) {
		/* Remote client, config should exist and parsed successfully */

		if (rc) {
			sky_err("sky_confparse(): %s\n", strerror(-rc));
			exit(-1);
		}
		conf->contype = SKY_REMOTE;
		/* TODO: we still get addr and port from command line */
		conf->cliport = strtol(cli->port, NULL, 10);
		conf->subport = conf->cliport + 1;
		strncpy(conf->hostname, cli->addr,
			sizeof(conf->hostname)-1);
	} else {
		/* Local client, config may be missing */

		if (rc && rc != -ENOENT) {
			sky_err("sky_confparse(): %s\n", strerror(-rc));
			exit(-1);
		}
		conf->contype = SKY_LOCAL;
	}
}

static struct sky_dev_desc *sky_find_devdesc_by_id(struct sky_dev_desc *head,
						   const char *devid)
{
	struct sky_dev_desc *devdesc;
	char id[16];

	foreach_devdesc(devdesc, head) {
		snprintf(id, sizeof(id), "%08X", sky_dev_desc_crc32(devdesc));
		if (!strcmp(devid, id))
			return devdesc;
	}

	return NULL;
}

static void sky_prepare_dev(struct cli *cli, struct sky_dev **dev,
			    struct sky_dev_desc **devdesc,
			    struct sky_dev_desc **devdescs)
{
	struct sky_conf conf;
	int rc;

	sky_prepare_conf(cli, &conf);

	rc = sky_devslist(&conf, devdescs);
	if (rc) {
		if (rc == -ENODEV)
			fprintf(stderr, "No devices found.\n");
		else
			sky_err("sky_devslist(): %s\n", strerror(-rc));
		exit(-1);
	}
	if (!cli->listdevs) {
		if (!cli->devid) {
			if ((*devdescs)->next) {
				sky_err("Error: multiple devices found, please specify --id <dev-id>\n");
				exit(-1);
			}
			*devdesc = *devdescs;
		} else {
			*devdesc = sky_find_devdesc_by_id(*devdescs, cli->devid);
			if (!*devdesc) {
				sky_err("Error: device by id '%s' was no found\n",
					cli->devid);
				exit(-1);
			}
		}
		rc = sky_devopen(*devdesc, dev);
		if (rc) {
			sky_err("sky_devopen(): %s\n", strerror(-rc));
			exit(-1);
		}
	} else {
		*dev = NULL;
		*devdesc = NULL;
	}
}

static void sky_print_charging_state(enum sky_dev_type dev_type,
				     struct cli *cli,
				     struct sky_charging_state *state)
{
	char timestr[64];
	struct timeval tv;
	struct tm *tm;
	size_t len;

	gettimeofday(&tv, NULL);
	tm = localtime(&tv.tv_sec);
	if (tm == NULL) {
		sky_err("localtime(): %s\n", strerror(errno));
		exit(-1);
	}
	if (dev_type == SKY_MUX_HW1 && cli->linkstat) {
		sky_err("--link-stat is not supported for the HW1\n");
		exit(-1);
	}
	len = strftime(timestr, sizeof(timestr), "%Y-%m-%d %H:%M:%S", tm);
	snprintf(timestr + len, sizeof(timestr) - len,
		 ".%03ld", tv.tv_usec/1000);

	if (cli->json) {
		unsigned hours, mins, secs;

		printf("{\n");
		printf("\t\"timestamp\": \"%s\",\n", timestr);
		printf("\t\"state\": \"%s\",\n",
		       sky_devstate_to_str(dev_type, state->dev_hw_state));
		printf("\t\"voltage\": \"%.3f\",\n", state->voltage_mV / 1000.0f);
		printf("\t\"current\": \"%.3f\",\n", state->current_mA / 1000.0f);
		printf("\t\"power\": \"%.3f\"%s\n",
		       (float)state->voltage_mV * state->current_mA / 1000000.0f,
		       dev_type == SKY_MUX_HW2 ? "," : "");
		if (dev_type == SKY_MUX_HW2) {
			printf("\t\"energy\": \"%.3f\",\n", state->energy_mWh / 1000.0f);
			printf("\t\"charge\": \"%.3f\",\n", state->charge_mAh / 1000.0f);
			printf("\t\"source-humidity\": \"%u\",\n", state->mux_humidity_perc);
			printf("\t\"source-temperature\": \"%d\",\n", state->mux_temperature_C);
			printf("\t\"sink-temperature\": \"%d\",\n", state->sink_temperature_C);
			seconds_to_hms(state->charging_secs, &hours, &mins, &secs);
			printf("\t\"charging\": \"%02uh:%02um:%02us\",\n", hours, mins, secs);
			seconds_to_hms(state->until_full_secs, &hours, &mins, &secs);
			printf("\t\"soc\":  \"%u%% ~ %02uh:%02um\"%s\n",
			       state->state_of_charge, hours, mins,
			       cli->linkstat ? "," : "");
			if (cli->linkstat) {
				printf("\t\"link-stat\" : {\n");
				printf("\t\t\"link-quality\": \"%u\",\n"
				       "\t\t\"tx-bytes\": \"%u\",\n"
				       "\t\t\"tx-packets\": \"%u\",\n"
				       "\t\t\"tx-err-bytes\": \"%u\",\n"
				       "\t\t\"tx-err-packets\": \"%u\",\n"
				       "\t\t\"rx-bytes\": \"%u\",\n"
				       "\t\t\"rx-packets\": \"%u\",\n"
				       "\t\t\"rx-err-bytes\": \"%u\",\n"
				       "\t\t\"rx-err-packets\": \"%u\"\n",
				       state->link_quality_factor,
				       state->tx.bytes,
				       state->tx.packets,
				       state->tx.err_bytes,
				       state->tx.err_packets,
				       state->rx.bytes,
				       state->rx.packets,
				       state->rx.err_bytes,
				       state->rx.err_packets);
				printf("\t}\n");
			}
		}
		printf("}\n");

	} else if (!cli->pretty) {
		if (dev_type == SKY_MUX_HW2) {
			printf("%s\t%-30s\t%.3fV\t%.3fA\t%.3fW\t%.3fWh\t%.3fAh\t%us\t%us\t",
			       timestr,
			       sky_devstate_to_str(dev_type, state->dev_hw_state),
			       state->voltage_mV / 1000.0f,
			       state->current_mA / 1000.0f,
			       state->voltage_mV * state->current_mA / 1000000.0f,
			       state->energy_mWh / 1000.0f,
			       state->charge_mAh / 1000.0f,
			       state->charging_secs,
			       state->until_full_secs);

			if (cli->linkstat) {
				printf("\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u%%\n",
				       state->tx.bytes,
				       state->tx.packets,
				       state->tx.err_bytes,
				       state->tx.err_packets,
				       state->rx.bytes,
				       state->rx.packets,
				       state->rx.err_bytes,
				       state->rx.err_packets,
				       state->link_quality_factor);
			}
			printf("\n");
		} else {
			printf("%s\t%-30s\t%.3fV\t%.3fA\t%.3fW\n",
			       timestr,
			       sky_devstate_to_str(dev_type, state->dev_hw_state),
			       state->voltage_mV / 1000.0f,
			       state->current_mA / 1000.0f,
			       state->voltage_mV * state->current_mA / 1000000.0f);
		}
	} else {
		unsigned hours, mins, secs;

		printf("Timestamp:       %s\n", timestr);
		printf("Dev state:       %s\n",
		       sky_devstate_to_str(dev_type, state->dev_hw_state));
		printf("Voltage:         %.3fV\n", state->voltage_mV / 1000.0f);
		printf("Current:         %.3fA\n", state->current_mA / 1000.0f);
		printf("Power:           %.3fW\n",
		       (float)state->voltage_mV * state->current_mA / 1000000.0f);
		if (dev_type == SKY_MUX_HW2) {
			printf("Energy:          %.3fWh\n", state->energy_mWh / 1000.0f);
			printf("Charge:          %.3fAh\n", state->charge_mAh / 1000.0f);
			printf("Source humidity: %d%%\n", state->mux_humidity_perc);
			printf("Source temp:     %dC\n", state->mux_temperature_C);
			printf("Sink temp:       %dC\n", state->sink_temperature_C);
			seconds_to_hms(state->charging_secs, &hours, &mins, &secs);
			printf("Charging:        %02uh:%02um:%02us\n", hours, mins, secs);
			seconds_to_hms(state->until_full_secs, &hours, &mins, &secs);
			if (cli->linkstat) {
				printf("Link quality:    %u%%\n"
				       "TX stat:\n"
				       "    bytes:       %u\n"
				       "    packets:     %u\n"
				       "    err bytes:   %u\n"
				       "    err packets: %u\n"
				       "RX stat:\n"
				       "    bytes:       %u\n"
				       "    packets:     %u\n"
				       "    err bytes:   %u\n"
				       "    err packets: %u\n",
				       state->link_quality_factor,
				       state->tx.bytes,
				       state->tx.packets,
				       state->tx.err_bytes,
				       state->tx.err_packets,
				       state->rx.bytes,
				       state->rx.packets,
				       state->rx.err_bytes,
				       state->rx.err_packets);
			}
		}
		printf("\n");
	}
}

static void sky_print_gpsdata(struct cli *cli, struct sky_gpsdata *gpsdata)
{
	char isotime[64];

	unix_to_iso8601(gpsdata->fix.time, isotime, sizeof(isotime));

	printf("Time:     %s\n", isotime);
	printf("Lat/Lon:  %f %f\n", gpsdata->fix.latitude,
	       gpsdata->fix.longitude);
        if (isnan(gpsdata->fix.altitude))
		printf("Altitude: ?\n");
	else
		printf("Altitude: %f\n", gpsdata->fix.altitude);

	printf("Status:   STATUS_%s\n",
	       gpsdata->status == SKY_GPS_STATUS_DGPS_FIX ? "DGPS_FIX" :
	       gpsdata->status == SKY_GPS_STATUS_FIX ? "FIX" :
	       "NO_FIX");
        printf("Mode:     MODE_%s\n",
	       gpsdata->fix.mode == SKY_GPS_MODE_3D ? "3D" :
	       gpsdata->fix.mode == SKY_GPS_MODE_2D ? "2D" :
	       gpsdata->fix.mode == SKY_GPS_MODE_NO_FIX ? "NO_FIX" :
	       "ZERO");
        printf("Quality:  %d p=%2.2f h=%2.2f v=%2.2f t=%2.2f g=%2.2f\n",
	       gpsdata->satellites_used, gpsdata->dop.pdop, gpsdata->dop.hdop,
	       gpsdata->dop.vdop, gpsdata->dop.tdop, gpsdata->dop.gdop);
}

struct sky_on_charging_state_arg {
	struct sky_dev_desc *devdesc;
	struct cli *cli;
};

static void sky_on_charging_state(void *data, struct sky_charging_state *state)
{
	struct sky_on_charging_state_arg *arg = data;

	sky_print_charging_state(arg->devdesc->dev_type, arg->cli, state);
}

static void
parse_key_value_params(bool is_dev_params, struct sky_dev_desc *devdesc,
		       struct sky_dev_params *params,
		       const char *param, const char *value)
{
	unsigned nr_params, i;
	int rc;

	if (is_dev_params) {
		if (devdesc->dev_type == SKY_MUX_HW1)
			nr_params = SKY_HW1_NUM_DEVPARAM;
		else
			nr_params = SKY_HW2_NUM_DEVPARAM;
	} else {
		nr_params = SKY_SINK_NUM_DEVPARAM;
	}

	i = is_dev_params ?
		sky_devparam_from_str(devdesc->dev_type, param) :
		sky_sinkparam_from_str(param);
	if (i == nr_params) {
		fprintf(stderr, "Incorrect parameter: %s\n", param);
		fprintf(stderr, "Please, use one of the following:\n");
		for (i = 0; i < nr_params; i++) {
			const char *str;

			str = is_dev_params ?
				sky_devparam_to_str(devdesc->dev_type, i) :
				sky_sinkparam_to_str(i);

			fprintf(stderr, "\t%s\n", str);
		}
		exit(-1);
	}
	if (is_dev_params) {
		rc = sky_devparam_value_from_str(value,
						 devdesc->dev_type,
						 i, params);
		if (rc) {
			sky_err("Can't parse value '%s' for param %s\n",
				value, param);
			exit(-1);
		}
	} else {
		rc = sky_sinkparam_value_from_str(value, i, params);
		if (rc) {
			sky_err("Can't parse value '%s' for param %s\n",
				value, param);
			exit(-1);
		}
	}

}

/**
 * parse_dev_or_sink_params() - parse device or sink params and fills in bitset
 *
 * NOTE: if @devdesc or @params is NULL function verifies command line arguments
 *       without updating the params bitset
 *
 */
static void parse_dev_or_sink_params(struct cli *cli, struct sky_dev_desc *devdesc,
				     struct sky_dev_params *params)
{
	const char params_delims[] = ";, ";
	const char kv_delims[] = "=:";
	char *tmp1, *tmp2, *kv;
	char *param, *value;
	char *params_str;
	bool is_dev_params;

	if (params)
		memset(params, 0, sizeof(*params));
	is_dev_params = cli->setdevparam || cli->setdevparams;

	if (cli->setdevparam || cli->setsinkparam) {
		size_t len = strlen(cli->key) + strlen(cli->value) + 2;
		params_str = alloca(len);
		snprintf(params_str, len, "%s:%s", cli->key, cli->value);
	} else if (cli->setdevparams || cli->setsinkparams) {
		params_str = strdupa(cli->keyvaluepairsstring);
	} else {
		sky_err("Neither sink, neither dev params are going to be set\n");
		exit(-1);
	}

	while ((kv = strtok_r(params_str, params_delims, &tmp1))) {
		params_str = NULL;

		param = strtok_r(kv, kv_delims, &tmp2);
		value = tmp2;

		if (!strlen(value)) {
			sky_err("Value for a parameter '%s' is missing\n",
				param);
			exit(-1);
		}

		if (devdesc && params)
			parse_key_value_params(is_dev_params, devdesc,
					       params, param, value);
	}
}


static void dump_section_and_flash_firmware(const struct cli *cli,
					    const struct sky_flash_info *info,
					    struct sky_dev *dev, int fd,
					    Elf_Scn* scn)
{
	size_t start_addr, end_addr;
	struct sky_flash_chunk chunk;
	ssize_t len, to_read;
	GElf_Shdr shdr;
	int rc, retries;

	char buf[64];
	double progress, step;

	struct progress_bar_state progress_state;
	const int max_retries = 5;

	memset(&progress_state, 0, sizeof(progress_state));

	if (gelf_getshdr(scn , &shdr) != &shdr) {
		fprintf(stderr, "ELF error: can't get section data\n");
		exit(-1);
	}

	start_addr = shdr.sh_addr;
	end_addr = shdr.sh_addr + shdr.sh_size;

	progress = 0.0;
	step = 0.02 / ((end_addr - start_addr) / (double)info->page_size);

	/* Erase loop */
	for (; start_addr < end_addr; start_addr += info->page_size) {
		snprintf(buf, sizeof(buf), "erasing flash page 0x%08zx", start_addr);
		print_progress(&progress_state, progress, buf);
		progress += step;

		rc = sky_sink_flash_pageerase(dev, start_addr);
		if (rc) {
			sky_err("sky_sink_flash_pageerase(0x%08zx): %s\n",
				start_addr, strerror(-rc));
			exit(-1);
		}
	}

	start_addr = shdr.sh_addr;

	rc = lseek(fd, shdr.sh_offset, SEEK_SET);
	if (rc < 0) {
		fprintf(stderr, "seek(): %s\n", strerror(errno));
		exit(-1);
	}

	step = 0.97 / ((end_addr - start_addr) / (double)info->max_chunk_size);

	/* Write loop */
	to_read = min_t(uint32_t, sizeof(chunk.buf), info->max_chunk_size);
	while (start_addr < end_addr) {
		len = read(fd, &chunk.buf, to_read);
		if (len < 0) {
			fprintf(stderr, "Error: read from file failed: %s\n",
				strerror(errno));
			exit(-1);
		} else if (len == 0) {
			fprintf(stderr, "Error: unexpected end of file while read\n");
			exit(-1);
		}
		chunk.addr = start_addr;
		chunk.size = len;

		retries = max_retries;
retry:
		snprintf(buf, sizeof(buf), "writing %zu bytes to 0x%08zx", len, start_addr);
		print_progress(&progress_state, progress, buf);
		progress += step;

		rc = sky_sink_flash_chunkwrite(dev, &chunk);
		if (rc) {
			if (--retries)
				goto retry;
			fprintf(stderr, "sky_sink_flash_chunkwrite(0x%08zx, %zu): %s\n",
				start_addr, len, strerror(-rc));
			exit(-1);
		}
		start_addr += len;
	}

	/* Commit start firmware address */
	if (!cli->bootloader) {
		rc = sky_sink_flash_startaddrset(dev, shdr.sh_addr);
		if (rc) {
			fprintf(stderr, "sky_sink_flash_startaddrset(0x%08" PRIx64 "): %s\n",
				shdr.sh_addr, strerror(-rc));
			exit(-1);
		}
	}

	print_progress(&progress_state, 1.0, "firmware update completed");
	printf("\n");
}

static void sink_firmware_update(const struct cli *cli, struct sky_dev *dev)
{
	struct sky_flash_info info;
	Elf_Scn* scn = NULL;
	size_t shstrndx;
	GElf_Shdr shdr;
	Elf* elf;

	const char *section_name = cli->bootloader ? ".bootloader" : NULL;
	const char *fwfile = cli->elffile ? cli->elffile : SINKFWFILE;

	int fd, rc;

	printf("Firmware file: %s\n", fwfile);

	fd = open(fwfile, O_RDONLY);
	if (fd == -1) {
		fprintf(stderr, "Error: can't open '%s' file for reading: %s\n",
			fwfile, strerror(errno));
		exit(-1);
	}

	rc = sky_sink_flash_info(dev, &info);
	if (rc) {
		fprintf(stderr, "sky_sink_flash_info(): %s\n",
			strerror(-rc));
		exit(-1);
	}

	if (elf_version(EV_CURRENT) == EV_NONE) {
		fprintf(stderr, "Could not initialize libelf\n");
		exit(-1);
	}

	elf = elf_begin(fd, ELF_C_READ, NULL);
	if (!elf) {
		fprintf(stderr, "Error: can't read file '%s' aself: %s\n",
			fwfile, elf_errmsg(elf_errno()));
		exit(-1);
	}

	if (elf_getshdrstrndx(elf, &shstrndx)) {
		fprintf(stderr, "ELF error: can't find section name section\n");
		exit(-1);
	}

	while ((scn = elf_nextscn(elf, scn))) {
		const char* scn_name;

		if (gelf_getshdr(scn , &shdr) != &shdr) {
			fprintf(stderr, "ELF error: can't get section data\n");
			exit(-1);
		}
		scn_name = elf_strptr(elf, shstrndx , shdr.sh_name);
		if (scn_name == NULL) {
			fprintf(stderr, "ELF error: can't get section name\n");
			exit(-1);
		}
		if (section_name && !strcmp(scn_name, section_name)) {
			break;
		} else if (!section_name &&
			   (!strcmp(scn_name, ".main1") || !strcmp(scn_name, ".main2"))) {
			if (shdr.sh_addr != info.real_addr)
				/* We search for a section with different address */
				break;
		}
	}
	if (!scn) {
		fprintf(stderr, "Firmware section was not found, is the '%s' correct sink firmware file?\n",
			fwfile);
		exit(-1);
	}

	/* Section found, write data to the sink device */
	dump_section_and_flash_firmware(cli, &info, dev, fd, scn);

	elf_end(elf);
	close(fd);
}

static void sink_passthru_send(struct sky_dev *dev)
{
	struct {
		struct sky_passthru_msg msg;
		uint8_t byte;
	} buffer;
	int rc, len, off;

	for (off = 0; off < sizeof(buffer); ) {
		len = read(0, buffer.msg.buf + off, sizeof(buffer) - off);
		if (len < 0) {
			fprintf(stderr, "read(stdin): %s\n", strerror(errno));
			exit(-1);
		}
		if (!len)
			break;
		off += len;
	}
	if (off > sizeof(buffer.msg.buf)) {
		fprintf(stderr, "Buffer overflow, expects not more than %zu bytes\n",
			sizeof(buffer.msg.buf));
		exit(-1);
	}

	buffer.msg.len = off;
	rc = sky_sink_passthru_msgsend(dev, &buffer.msg);
	if (rc) {
		fprintf(stderr, "sky_sink_passthru_msgsend(): %s\n", strerror(-rc));
		exit(-1);
	}
}

static void sink_passthru_recv(struct sky_dev *dev)
{
	struct sky_passthru_msg msg;
	int len, rc;

	rc = sky_sink_passthru_msgrecv(dev, &msg);
	if (rc) {
		fprintf(stderr, "sky_sink_passthru_msgrecv(): %s\n", strerror(-rc));
		exit(-1);
	}

	if (msg.len) {
		len = write(1, msg.buf, msg.len);
		if (len < 0) {
			fprintf(stderr, "write(stdout): %s\n", strerror(-rc));
			exit(-1);
		}
	}
}

/**
 * List of commands which should be passed directly to the device
 * bypassing the server.
 */
static bool only_directly_permitted_command(struct cli *cli)
{
	return (cli->sinkfirmwareupdate ||
		cli->sinkflashinfo);
}

int main(int argc, char *argv[])
{
	struct sky_dev_desc *devdescs = NULL;
	struct sky_dev_desc *devdesc;
	struct sky_dev *dev;
	struct cli cli;
	int rc, i;

	rc = cli_parse(argc, argv, &cli);
	if (rc) {
		fprintf(stderr, "%s\n", cli_usage);
		return -1;
	}
	if (!cli.addr) {
		/* By default connect to the server locally */
		cli.addr = strdup("localhost");
	}
	if (only_directly_permitted_command(&cli) ||
	    !strcmp(cli.addr, "directly")) {
		/* Still have a possibility to bypass the server */
		free(cli.addr);
		cli.addr = NULL;
	}

	if (cli.port) {
		const char *regexp = "[^[:digit:]]+";
		regmatch_t match;
		regex_t rgx;

		rc = regcomp(&rgx, regexp, REG_EXTENDED | REG_NEWLINE);
		assert(!rc);

		rc = regexec(&rgx, cli.port, 1, &match, 0);
		regfree(&rgx);
		if (rc != REG_NOMATCH) {
			fprintf(stderr, "Error: You've called a remote command invocation and specified the remote address '%s' and port '%s', but port is an invalid number.\n"
				"       If you attempted to call the 'monitor' command with the '1000 --raw' parameters, be aware that these parameters don't exist any more.\n"
				"       See help for details.\n", cli.addr, cli.port);
			return -1;
		}
	} else {
		/* Default broker port */
		cli.port = strdup("5555");
	}

	if (cli.discoverbroker) {
		struct sky_brokerinfo brokerinfo;
		unsigned int timeout_ms;
		int rc;

		if (cli.timeoutsecs)
			timeout_ms = atoi(cli.timeoutsecs) * 1000;
		else
			timeout_ms = 5000;
		rc = sky_discoverbroker(&brokerinfo, timeout_ms);
		if (rc) {
			sky_err("sky_discoverbroker(): %s\n", strerror(-rc));
			exit(-1);
		}
		printf("Found skybroker:\n");
		printf("\t%s addr:        %s\n",
		       brokerinfo.af_family == AF_INET6 ? "IPv6" : "IPv4",
		       brokerinfo.addr);
		printf("\tServers port:     %u\n", brokerinfo.servers_port);
		printf("\tClients port:     %u\n", brokerinfo.clients_port);
		printf("\tProtocol version: %u.%u\n",
		       (brokerinfo.proto_version >> 8)  & 0xff,
		       brokerinfo.proto_version & 0xff);

		cli_free(&cli);

		return 0;

	} else if (cli.peerinfo) {
		struct sky_peerinfo peerinfo;
		struct sky_conf conf;
		int rc;

		sky_prepare_conf(&cli, &conf);

		rc = sky_peerinfo(&conf, &peerinfo);
		if (rc) {
			sky_err("sky_peerinfo(): %s\n", strerror(-rc));
			exit(-1);
		}
		printf("Remote peer %s:%d:\n", conf.hostname,
		       conf.cliport);
		printf("\tServer version:   %u.%u.%u\n",
		       version_major(peerinfo.server_version),
		       version_minor(peerinfo.server_version),
		       version_revis(peerinfo.server_version));
		printf("\tProtocol version: %u.%u\n",
		       version_minor(peerinfo.proto_version),
		       version_revis(peerinfo.proto_version));

		cli_free(&cli);

		return 0;
	}

	if (cli.setdevparam || cli.setsinkparam ||
	    cli.setdevparams || cli.setsinkparams)
		/*
		 * Verify command line arguments before making a
		 * connection to a device. If parameters are specified
		 * incorrectly (delimiter is missing, etc) then we
		 * rise an error before making a connection.
		 */
		parse_dev_or_sink_params(&cli, NULL, NULL);

	if (cli.addr && !resolve_hostname_and_connection_probe(&cli)) {
		fprintf(stderr, "Unable to connect to '%s:%s'\n",
			cli.addr, cli.port);
		return -1;
	}
	sky_prepare_dev(&cli, &dev, &devdesc, &devdescs);

	if (cli.listdevs) {
		size_t max_devname = strlen("DEV-NAME");
		struct sky_dev_desc *devdesc;

		foreach_devdesc(devdesc, devdescs) {
			max_devname = max(strnlen(devdesc->dev_name,
						  sizeof(devdesc->dev_name)),
					  max_devname);
		}

		printf("Found sky devices:\n");
		printf("\t  DEV-ID     TYPE  %*s      FW      HW\n",
		       (int)max_devname, "DEV-NAME");
		foreach_devdesc(devdesc, devdescs) {
			printf("\t%08X  %s  %*s  v%u.%u.%u  v%u.%u.%u\n",
			       sky_dev_desc_crc32(devdesc),
			       sky_devtype_to_str(devdesc->dev_type),
			       (int)max_devname,
			       devdesc->dev_name,
			       version_major(devdesc->hw_info.fw_version),
			       version_minor(devdesc->hw_info.fw_version),
			       version_revis(devdesc->hw_info.fw_version),
			       version_major(devdesc->hw_info.hw_version),
			       version_minor(devdesc->hw_info.hw_version),
			       version_revis(devdesc->hw_info.hw_version));
		}
	} else if (cli.monitor) {
		struct sky_on_charging_state_arg arg = {
			.cli = &cli,
			.devdesc = devdesc,
		};
		struct sky_subscription sub = {
			.data = &arg,
			.on_state = sky_on_charging_state,
			.interval_msecs = 1000,
		};
		struct sky_subscription_token token;

		rc = sky_subscription_token(dev, &token);
		if (rc) {
			sky_err("sky_subscription_token(): %s\n", strerror(-rc));
			exit(-1);
		}
		rc = sky_subscribe(dev, &token, &sub);
		if (rc) {
			sky_err("sky_subscribe(): %s\n", strerror(-rc));
			exit(-1);
		}
		while (1)
			/* Yeah, nothing to do here, simply die on Ctrl-C */
			sleep(1);
	} else if (cli.showdevparams || cli.showsinkparams) {
		struct sky_dev_params params;
		unsigned nr_params;

		if (cli.showdevparams) {
			if (devdesc->dev_type == SKY_MUX_HW1)
				nr_params = SKY_HW1_NUM_DEVPARAM;
			else
				nr_params = SKY_HW2_NUM_DEVPARAM;
		} else {
			nr_params = SKY_SINK_NUM_DEVPARAM;
		}

		/* Get all params */
		params.dev_params_bits = ~0;
		if (cli.showdevparams)
			rc = sky_paramsget(dev, &params);
		else
			rc = sky_sink_paramsget(dev, &params);
		if (rc) {
			if (cli.showdevparams)
				sky_err("sky_paramsget(): %s\n", strerror(-rc));
			else
				sky_err("sky_sink_paramsget(): %s\n", strerror(-rc));
			exit(-1);
		}

		if (cli.json)
			printf("{\n");
		else
			printf("%s has the following parameters:\n",
			       cli.showdevparams ? "Source" : "Sink");

		for (i = 0; i < nr_params; i++) {
			enum sky_param_value_format value_format = cli.json ?
				SKY_PARAM_VALUE_TEXT :
				SKY_PARAM_VALUE_TEXT_AND_NUMERIC;
			const char *str;
			char val[128];

			str = cli.showdevparams ?
				sky_devparam_to_str(devdesc->dev_type, i) :
				sky_sinkparam_to_str(i);
			cli.showdevparams ?
				sky_devparam_value_to_str(devdesc->dev_type, i, &params,
							  value_format,
							  val, sizeof(val)) :
				sky_sinkparam_value_to_str(i, &params,
							   value_format,
							   val, sizeof(val));

			if (cli.json)
				printf("\t\"%s\" : \"%s\"%s\n", str, val,
				       i < nr_params - 1 ? "," : "");
			else
				printf("\t%-34s %s\n", str, val);
		}
		if (cli.json)
			printf("}\n");

	} else if (cli.setdevparam || cli.setsinkparam ||
		   cli.setdevparams || cli.setsinkparams) {
		struct sky_dev_params params;

		parse_dev_or_sink_params(&cli, devdesc, &params);
		if (cli.setdevparam || cli.setdevparams) {
			rc = sky_paramsset(dev, &params);
			if (rc) {
				sky_err("sky_paramsset(): %s\n", strerror(-rc));
				exit(-1);
			}
		} else {
			rc = sky_sink_paramsset(dev, &params);
			if (rc) {
				sky_err("sky_sink_paramsset(): %s\n", strerror(-rc));
				exit(-1);
			}
		}
	} else if (cli.resumescan) {
		rc = sky_scanresume(dev);
		if (rc) {
			sky_err("sky_resumescan(): %s\n", strerror(-rc));
			exit(-1);
		}
	} else if (cli.stopscan) {
		rc = sky_scanstop(dev);
		if (rc) {
			sky_err("sky_scanstop(): %s\n", strerror(-rc));
			exit(-1);
		}
	} else if (cli.opendroneport) {
		rc = sky_droneport_open(dev);
		if (rc) {
			sky_err("sky_droneport_open(): %s\n", strerror(-rc));
			exit(-1);
		}
	} else if (cli.closedroneport) {
		rc = sky_droneport_close(dev);
		if (rc) {
			sky_err("sky_droneport_close(): %s\n", strerror(-rc));
			exit(-1);
		}
	} else if (cli.showdroneportstate) {
		struct sky_droneport_state state;

		rc = sky_droneport_state(dev, &state);
		if (rc) {
			sky_err("sky_droneport_state(): %s\n", strerror(-rc));
			exit(-1);
		}

		printf("Skycharge Drone Port:\n"
		       "   IS READY      %s\n"
		       "   IS OPENED     %s\n"
		       "   IS CLOSED     %s\n"
		       "   IN PROGRESS   %s\n"
		       "   LANDING ERROR %s\n",
		       state.status & SKY_DP_IS_READY ? "yes": " no",
		       state.status & SKY_DP_IS_OPENED ? "yes": " no",
		       state.status & SKY_DP_IS_CLOSED ? "yes": " no",
		       state.status & SKY_DP_IN_PROGRESS ? "yes": " no",
		       state.status & SKY_DP_LANDING_ERROR ? "yes": " no");

	} else if (cli.dronedetect) {
		enum sky_drone_status status;

		rc = sky_dronedetect(dev, &status);
		if (rc) {
			sky_err("sky_dronedetect(): %s\n", strerror(-rc));
			exit(-1);
		}
		switch (status) {
		case SKY_DRONE_NOT_DETECTED:
			printf("Drone is not detected on the charging pad.\n");
			break;
		case SKY_DRONE_DETECTED:
			printf("Drone is detected on the charging pad.\n");
			break;
		default:
			printf("Unknown drone status: 0x%x\n", status);
		}
	} else if (cli.showchargingstate) {
		struct sky_charging_state state;

		rc = sky_chargingstate(dev, &state);
		if (rc) {
			sky_err("sky_chargingstate(): %s\n", strerror(-rc));
			exit(-1);
		}
		cli.pretty = !cli.nopretty;
		sky_print_charging_state(devdesc->dev_type, &cli, &state);
	} else if (cli.reset) {
		rc = sky_reset(dev);
		if (rc) {
			sky_err("sky_reset(): %s\n", strerror(-rc));
			exit(-1);
		}
	} else if (cli.devinfo) {
		printf("Source info:\n"
		       "   FW:        v%d.%d.%d\n"
		       "   HW:        v%d.%d.%d\n"
		       "   PLC proto: v%d.%d.%d\n"
		       "   UID:       %08x%08x%08x\n",
		       version_major(devdesc->hw_info.fw_version),
		       version_minor(devdesc->hw_info.fw_version),
		       version_revis(devdesc->hw_info.fw_version),
		       version_major(devdesc->hw_info.hw_version),
		       version_minor(devdesc->hw_info.hw_version),
		       version_revis(devdesc->hw_info.hw_version),
		       version_major(devdesc->hw_info.plc_proto_version),
		       version_minor(devdesc->hw_info.plc_proto_version),
		       version_revis(devdesc->hw_info.plc_proto_version),
		       devdesc->hw_info.uid.part1,
		       devdesc->hw_info.uid.part2,
		       devdesc->hw_info.uid.part3);
	} else if (cli.gpsinfo) {
		struct sky_gpsdata gpsdata;

		rc = sky_gpsdata(dev, &gpsdata);
		if (rc) {
			sky_err("sky_gpsdata(): %s\n", strerror(-rc));
			exit(-1);
		}

		sky_print_gpsdata(&cli, &gpsdata);
	} else if (cli.sinkinfo) {
		struct sky_sink_info info;

		rc = sky_sink_infoget(dev, &info);
		if (rc) {
			sky_err("sky_sink_infoget(): %s\n", strerror(-rc));
			exit(-1);
		}

		printf("Sink info:\n"
		       "   FW:        v%d.%d.%d\n"
		       "   HW:        v%d.%d.%d\n"
		       "   PLC proto: v%d.%d.%d\n"
		       "   UID:       %08x%08x%08x\n",
		       version_major(info.hw_info.fw_version),
		       version_minor(info.hw_info.fw_version),
		       version_revis(info.hw_info.fw_version),
		       version_major(info.hw_info.hw_version),
		       version_minor(info.hw_info.hw_version),
		       version_revis(info.hw_info.hw_version),
		       version_major(info.hw_info.plc_proto_version),
		       version_minor(info.hw_info.plc_proto_version),
		       version_revis(info.hw_info.plc_proto_version),
		       info.hw_info.uid.part1,
		       info.hw_info.uid.part2,
		       info.hw_info.uid.part3);

	} else if (cli.sinkstartcharge) {
		rc = sky_sink_chargestart(dev);
		if (rc) {
			sky_err("sky_sink_chargestart(): %s\n", strerror(-rc));
			exit(-1);
		}
	} else if (cli.sinkstopcharge) {
		rc = sky_sink_chargestop(dev);
		if (rc) {
			sky_err("sky_sink_chargestop(): %s\n", strerror(-rc));
			exit(-1);
		}
	} else if (cli.sinkfirmwareupdate) {
		sink_firmware_update(&cli, dev);

	} else if (cli.sinkflashinfo) {
		struct sky_flash_info info;

		rc = sky_sink_flash_info(dev, &info);
		if (rc) {
			sky_err("sky_sink_flash_info(): %s\n",
				strerror(-rc));
			exit(-1);
		}

		printf("Sink FLASH info:\n"
		       "   real start addr: 0x%08x\n"
		       "    set start addr: 0x%08x\n"
		       "         page size: %u\n"
		       "    max chunk size: %u\n",
		       info.real_addr, info.set_addr,
		       info.page_size, info.max_chunk_size);

	} else if (cli.sinkpassthrusend) {
		sink_passthru_send(dev);

	} else if (cli.sinkpassthrurecv) {
		sink_passthru_recv(dev);

	} else
		assert(0);

	sky_devsfree(devdescs);
	if (dev)
		sky_devclose(dev);
	cli_free(&cli);

	return 0;
}

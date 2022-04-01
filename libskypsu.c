/*
 * Copyright (C) 2021-2022 Skycharge GmbH
 * Author: Roman Penyaev <r.peniaev@gmail.com>
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

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>

#include "types.h"
#include "libskypsu.h"

/* See MCP47FEB12A0 spec */

#define DAC_ADDR 0x60
#define DAC0_REG 0x00
#define DAC1_REG 0x01
#define VREF_REG 0x08
#define PWRD_REG 0x09
#define GAIN_REG 0x0A
#define WILO_REG 0x0B
#define DAC0_NVREG 0x10
#define DAC1_NVREG 0x11
#define VREF_NVREG 0x18
#define PWRD_NVREG 0x19
#define GAIN_NVREG 0x1A
#define WILO_NVREG 0x1B

#define RESOLUTION 0x3FF /* 10 bit */

#define DAC_VREF_CFG 0x0005 /* Internal Band Gap (1.22V typical) DAC0 + DAC1 */
#define DAC_PWRD_CFG 0x0000 /* Normal Operation (Not powered-down) */
#define DAC_GAIN_CFG 0x0300 /* DAC0 + DAC1 gain = 2 */

#define DAC_VMAX (4.88 * 2.0) /* 4.88V with internal band gap and
			       * x gain = 2 => Vmax = 2 * G * 1,22 */

static struct {
	float voltage_coef;
	float current_coef;
} psu_table[] = {
	[SKY_PSU_UNKNOWN]    = {},
	[SKY_PSU_RSP_750_48] = {
		.voltage_coef = 9.6,  /* Vout = Vin * 20 / 100 * 48V */
		.current_coef = 3.14  /* Iout = Vin * 20 / 100 * 15.7A */
	},
};

static int dac_write_register(struct sky_psu *psu, unsigned addr, uint16_t val)
{
	int wr;

	val = htons(val);
	wr = i2c_write(&psu->i2c_dev, addr << 3, &val, 2);

	return (wr == 2 ? 0 : -1);
}

static int dac_read_register(struct sky_psu *psu, unsigned addr)
{
	uint16_t val;
	int rd;

	rd = i2c_read(&psu->i2c_dev, addr << 3 | 0x06, &val, 2);
	if (rd != 2)
		return -1;

	return ntohs(val);
}

static int dac_init(struct sky_psu *psu, unsigned vref,
		    unsigned pwrd, unsigned gain)
{
	int rc;

	rc = dac_write_register(psu, VREF_REG, vref);
	if (rc)
		return rc;

	rc = dac_write_register(psu, PWRD_REG, pwrd);
	if (rc)
		return rc;

	rc = dac_write_register(psu, GAIN_REG, gain);
	if (rc)
		return rc;

	if (vref != dac_read_register(psu, VREF_NVREG)) {
		rc = dac_write_register(psu, VREF_NVREG, vref);
		if (rc)
			return rc;
        }
	if (pwrd != dac_read_register(psu, PWRD_NVREG)) {
		rc = dac_write_register(psu, PWRD_NVREG, pwrd);
		if (rc)
			return rc;
        }
        gain |= 0x00E0;
        if (gain != dac_read_register(psu, GAIN_NVREG)) {
		rc = dac_write_register(psu, GAIN_NVREG, gain);
		if (rc)
			return rc;
	}

	return 0;
}

static int validate_conf(struct sky_conf *conf)
{
	if (conf->psu.type != SKY_PSU_RSP_750_48)
		return -EINVAL;

	if (conf->psu.voltage < 10.0 ||
	    conf->psu.voltage > 60.0) {
		sky_err("incorrect psu voltage: %f, expected "
			"in range [10, 60]\n",
			conf->psu.voltage);
		return -EINVAL;
	}
	if (conf->psu.current < 1.0 ||
	    conf->psu.current > 60.0) {
		sky_err("incorrect psu current: %f, expected "
			"in range [1, 60]\n",
			conf->psu.current);
		return -EINVAL;
	}
	if (conf->psu.precharge_secs ||
	    conf->psu.precharge_current != 0.0 ||
	    conf->psu.precharge_current_coef != 0.0) {
		if (!conf->psu.precharge_secs) {
			sky_err("psu precharge secs is not specified\n");
			return -EINVAL;
		}
		if (conf->psu.precharge_current == 0.0 &&
		    conf->psu.precharge_current_coef == 0.0) {
			sky_err("neither psu precharge current nor precharge current coefficient are specified\n");
			return -EINVAL;
		}
		if (conf->psu.precharge_current != 0.0 &&
		    conf->psu.precharge_current_coef != 0.0) {
			sky_err("either psu precharge current or precharge current coefficient should be specified\n");
			return -EINVAL;
		}
		if (conf->psu.precharge_current != 0.0) {
			if (conf->psu.precharge_current < 1.0 ||
			    conf->psu.precharge_current >= conf->psu.current) {
				sky_err("incorrect psu precharge current: %f, expected "
					"in range [1, %f)\n",
					conf->psu.precharge_current,
					conf->psu.current);
				return -EINVAL;
			}
		}
		else {
			float precharge_current;

			if (conf->psu.precharge_current_coef >= 1.0) {
				sky_err("incorrect psu precharge current coefficient: %f, expected "
					"< 1.0\n",
					conf->psu.precharge_current_coef);
				return -EINVAL;
			}

			precharge_current = conf->psu.current * conf->psu.precharge_current_coef;
			if (precharge_current < 1.0) {
				sky_err("incorrect psu precharge current coefficient: %f, result current should be >= 1.0\n",
					conf->psu.precharge_current_coef);
				return -EINVAL;
			}
			/* Set precharge current according to the coefficient */
			conf->psu.precharge_current = precharge_current;
		}
	}

	return 0;
}

int sky_psu_init(struct sky_conf *conf, struct sky_psu *psu)
{
	int rc;

	rc = validate_conf(conf);
	if (rc)
		return rc;

	psu->i2c_bus = i2c_open("/dev/i2c-2");
	if (psu->i2c_bus < 0)
		return -errno;

	memset(&psu->i2c_dev, 0, sizeof(psu->i2c_dev));
	i2c_init_device(&psu->i2c_dev);

	psu->i2c_dev.bus = psu->i2c_bus;
	psu->i2c_dev.addr = DAC_ADDR;

	rc = dac_init(psu, DAC_VREF_CFG, DAC_PWRD_CFG, DAC_GAIN_CFG);
	if (rc)
		return rc;

	psu->type = conf->psu.type;
	psu->precharge_set = !!conf->psu.precharge_secs;
	psu->voltage = 0.0;
	psu->current = 0.0;

	return 0;
}

void sky_psu_deinit(struct sky_psu *psu)
{
	i2c_close(psu->i2c_bus);
}

bool sky_psu_is_precharge_set(struct sky_psu *psu)
{
	return psu->precharge_set;
}

static float map_voltage_to_external_voltage(struct sky_psu *psu,
					     float voltage)
{
	return voltage / psu_table[psu->type].voltage_coef;
}

static float map_current_to_external_voltage(struct sky_psu *psu,
					     float current)
{
	return current / psu_table[psu->type].current_coef;
}

static float map_external_voltage_to_voltage(struct sky_psu *psu,
					     float voltage)
{
	return voltage * psu_table[psu->type].voltage_coef;
}

static float map_external_voltage_to_current(struct sky_psu *psu,
					     float voltage)
{
	return voltage * psu_table[psu->type].current_coef;
}

static int __sky_psu_set_voltage(struct sky_psu *psu, float voltage)
{
	float ext_voltage;
	unsigned value;
	int rc;

	ext_voltage = map_voltage_to_external_voltage(psu, voltage);
	value = (ext_voltage / DAC_VMAX) * RESOLUTION;

	rc = dac_write_register(psu, DAC0_REG, value);
	if (rc)
		return rc;
	rc = dac_write_register(psu, DAC0_NVREG, value);
	if (!rc)
		psu->voltage = voltage;

	return rc;
}

int sky_psu_set_voltage(struct sky_psu *psu, float voltage)
{
	float actual_voltage;
	int i, rc, steps;

	if (psu->voltage == voltage)
		return 0;

	rc = sky_psu_get_voltage(psu, &actual_voltage);
	if (rc)
		return rc;

	/*
	 * The following 'if' branches below are needed only because RSP
	 * goes to error (solid red) if voltage is decreased too fast
	 * (I could not find anything in the spec :( Experimentally I
	 * found out that we can safely decrease step by step on 3v
	 * and wait for 500 ms between calls.
	 */

	/* We can safely increase voltage */
	if (actual_voltage < voltage)
		return __sky_psu_set_voltage(psu, voltage);

	/* We can safely decrease voltage on 3v */
	if (actual_voltage - voltage <= 3.0)
		return __sky_psu_set_voltage(psu, voltage);

	steps = (actual_voltage - voltage) / 3.0;

	for (i = 1; i <= steps; i++) {
		rc = __sky_psu_set_voltage(psu, actual_voltage - 3.0 * i);
		if (rc)
			return rc;

		/* 500ms */
		usleep(500000);
	}

	return __sky_psu_set_voltage(psu, voltage);
}

int sky_psu_set_current(struct sky_psu *psu, float current)
{
	float ext_voltage;
	unsigned value;
	int rc;

	if (psu->current == current)
		return 0;

	ext_voltage = map_current_to_external_voltage(psu, current);
	value = (ext_voltage / DAC_VMAX) * RESOLUTION;

	rc = dac_write_register(psu, DAC1_REG, value);
	if (rc)
		return rc;
	rc = dac_write_register(psu, DAC1_NVREG, value);
	if (!rc)
		psu->current = current;

	return rc;
}

int sky_psu_get_voltage(struct sky_psu *psu, float *voltage)
{
	float ext_voltage;
	int value;

	value = dac_read_register(psu, DAC0_REG);
	if (value < 0)
		return value;

	ext_voltage = value * DAC_VMAX / RESOLUTION;
	*voltage = map_external_voltage_to_voltage(psu, ext_voltage);

	return 0;
}

int sky_psu_get_current(struct sky_psu *psu, float *current)
{
	float ext_voltage;
	int value;

	value = dac_read_register(psu, DAC1_REG);
	if (value < 0)
		return value;

	ext_voltage = value * DAC_VMAX / RESOLUTION;
	*current = map_external_voltage_to_current(psu, ext_voltage);

	return 0;
}

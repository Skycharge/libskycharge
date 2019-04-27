#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>

#include "types.h"
#include "skypsu.h"

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
	    conf->psu.precharge_current != 0.0) {
		if (!conf->psu.precharge_secs) {
			sky_err("psu precharge secs is not specified\n");
			return -EINVAL;
		}
		if (conf->psu.precharge_current == 0.0) {
			sky_err("psu precharge current is not specified\n");
			return -EINVAL;
		}
		if (conf->psu.precharge_current < 1.0 ||
		    conf->psu.precharge_current >= conf->psu.current) {
			sky_err("incorrect psu precharge current: %f, expected "
				"in range [1, %f)\n",
				conf->psu.precharge_current,
				conf->psu.current);
			return -EINVAL;
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
	return psu && psu->type != SKY_PSU_UNKNOWN && psu->precharge_set;
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

int sky_psu_set_voltage(struct sky_psu *psu, float voltage)
{
	float ext_voltage;
	unsigned value;
	int rc;

	if (psu->voltage == voltage)
		return 0;

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

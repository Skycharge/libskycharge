#ifndef LIBSKYPSU_H
#define LIBSKYPSU_H

#include "libi2c/i2c.h"
#include "libskysense.h"
#include "types.h"

struct sky_psu {
	enum sky_psu_type type;
	bool precharge_set;
	float voltage;
	float current;
	int i2c_bus;
	struct i2c_device i2c_dev;
};

int sky_psu_init(struct sky_conf *conf, struct sky_psu *psu);
void sky_psu_deinit(struct sky_psu *psu);
bool sky_psu_is_precharge_set(struct sky_psu *psu);

int sky_psu_set_voltage(struct sky_psu *psu, float voltage);
int sky_psu_set_current(struct sky_psu *psu, float current);

int sky_psu_get_voltage(struct sky_psu *psu, float *voltage);
int sky_psu_get_current(struct sky_psu *psu, float *current);

#endif /* LIBSKYPSU_H */

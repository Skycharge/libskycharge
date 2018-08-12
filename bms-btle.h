#ifndef BMS_BTLE_H
#define BMS_BTLE_H

#include <stdint.h>
#include "libskybms.h"

struct bms_btle;

int bms_btle_open(struct bms_btle **bms_btle);
void bms_btle_close(struct bms_btle *bms_btle);

int bms_btle_request_data(struct bms_btle *bms, struct bms_data *data);

#endif /* BMS_BTLE_H */

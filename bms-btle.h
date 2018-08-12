#ifndef BMS_BTLE_H
#define BMS_BTLE_H

#include <stdint.h>

struct bms;

int bms_open(struct bms **bms);
void bms_close(struct bms *bms);

int bms_request_nearest(struct bms *bms, uint16_t *perc, uint32_t *volt);

#endif /* BMS_BTLE_H */

#ifndef LIBSKYBMS_H
#define LIBSKYBMS_H

#include <stdint.h>

struct bms_data {
	uint32_t charge_volt;  /* Current charging voltage */
	uint16_t charge_perc;  /* Current charge percentage */
	uint16_t charge_time;  /* Seconds left till full charge */
};

struct bms_lib {
	struct {
		uint64_t update_msecs;
		struct bms_data data;
	} *mem;
	int fd;
};

void bms_init(struct bms_lib *bms_lib);
void bms_deinit(struct bms_lib *bms_lib);
int bms_update_data(struct bms_lib *bms_lib, const struct bms_data *data);
int bms_request_data(struct bms_lib *bms_lib, struct bms_data *data);

#endif /* LIBSKYBMS_H */

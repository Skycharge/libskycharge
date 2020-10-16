#ifndef GPIO_H
#define GPIO_H

#include "types.h"

enum {
	GPIO_DIR_IN,
	GPIO_DIR_OUT,
};

int gpio_configure(const char *pin, int dir);
int gpio_read(const char *pin);
int gpio_write(const char *pin, bool value);

int gpio_lock(void);
void gpio_unlock(int lockfd);

#endif /* GPIO_H */

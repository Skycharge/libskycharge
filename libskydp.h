#ifndef LIBSKYDP_H
#define LIBSKYDP_H

#include "libskysense.h"

int dp_configure(const struct sky_dev_desc *desc);
int dp_open(struct sky_dev *dev);
int dp_close(struct sky_dev *dev);
int dp_state(struct sky_dev *dev, struct sky_droneport_state *state);

/**
 * dp_drone_detect() - detects drone with drone port sensors
 *
 * Returns: 1 if drone detected, 0 otherwise, -errno in case of error.
 */
int dp_drone_detect(struct sky_dev *dev);

#endif /* LIBSKYDP_H */

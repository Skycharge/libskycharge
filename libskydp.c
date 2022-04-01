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

#include <errno.h>
#include <unistd.h>
#include <string.h>

#include "libskydp.h"
#include "libskycharge-pri.h"
#include "gpio.h"

static int dp_is_ready(struct sky_conf *conf)
{
	return gpio_read(conf->dp.gpio.is_ready_pin);
}

static int dp_in_progress(struct sky_conf *conf)
{
	return gpio_read(conf->dp.gpio.in_progress_pin);
}

static int dp_is_opened(struct sky_conf *conf)
{
	return gpio_read(conf->dp.gpio.is_opened_pin);
}

static int dp_is_closed(struct sky_conf *conf)
{
	return gpio_read(conf->dp.gpio.is_closed_pin);
}

static int dp_is_drone_detected(struct sky_conf *conf)
{
	return gpio_read(conf->dp.gpio.is_drone_detected_pin);
}

int dp_configure(const struct sky_dev_desc *desc)
{
	const struct sky_conf *conf = &desc->conf;
	int lockfd, rc;

	if (conf->dp.hw_interface == SKY_DP_UNKNOWN)
		return -EOPNOTSUPP;
	if (conf->dp.hw_interface != SKY_DP_GPIO)
		return -EOPNOTSUPP;

	/* GPIO Interface */

	lockfd = gpio_lock();
	if (lockfd < 0)
		return lockfd;

	rc = gpio_configure(conf->dp.gpio.is_closed_pin,
			    GPIO_DIR_IN);
	if (rc) {
		sky_err("%s: configuration of is_closed_pin failed: %s\n",
			__func__, strerror(-rc));
		goto unlock;
	}
	rc = gpio_configure(conf->dp.gpio.in_progress_pin,
			    GPIO_DIR_IN);
	if (rc) {
		sky_err("%s: configuration of in_progress_pin failed: %s\n",
			__func__, strerror(-rc));
		goto unlock;
	}
	rc = gpio_configure(conf->dp.gpio.is_drone_detected_pin,
			    GPIO_DIR_IN);
	if (rc) {
		sky_err("%s: configuration of is_drone_detected_pin failed: %s\n",
			__func__, strerror(-rc));
		goto unlock;
	}
	rc = gpio_configure(conf->dp.gpio.is_opened_pin,
			    GPIO_DIR_IN);
	if (rc) {
		sky_err("%s: configuration of is_opened_pin failed: %s\n",
			__func__, strerror(-rc));
		goto unlock;
	}
	rc = gpio_configure(conf->dp.gpio.open_pin,
			    GPIO_DIR_OUT);
	if (rc) {
		sky_err("%s: configuration of open_pin failed: %s\n",
			__func__, strerror(-rc));
		goto unlock;
	}
	rc = gpio_configure(conf->dp.gpio.close_pin,
			    GPIO_DIR_OUT);
	if (rc) {
		sky_err("%s: configuration of close_pin failed: %s\n",
			__func__, strerror(-rc));
		goto unlock;
	}
	rc = gpio_configure(conf->dp.gpio.is_landing_err_pin,
			    GPIO_DIR_IN);
	if (rc) {
		sky_err("%s: configuration of is_landing_err_pin failed: %s\n",
			__func__, strerror(-rc));
		goto unlock;
	}
	rc = gpio_configure(conf->dp.gpio.is_ready_pin,
			    GPIO_DIR_IN);
	if (rc) {
		sky_err("%s: configuration of is_ready_pin failed: %s\n",
			__func__, strerror(-rc));
		goto unlock;
	}

unlock:
	gpio_unlock(lockfd);

	return rc;
}

int dp_open(struct sky_dev *dev)
{
	struct sky_conf *conf = &dev->devdesc.conf;
	const char* pin = conf->dp.gpio.open_pin;
	int lockfd, rc;

	if (conf->dp.hw_interface == SKY_DP_UNKNOWN)
		return -EOPNOTSUPP;
	if (conf->dp.hw_interface != SKY_DP_GPIO)
		return -EOPNOTSUPP;

	/* GPIO Interface */

	lockfd = gpio_lock();
	if (lockfd < 0)
		return lockfd;

	rc = dp_is_ready(conf);
	if (rc < 0)
		goto unlock;
	if (!rc) {
		/* DP is not ready */
		rc = -EBUSY;
		goto unlock;
	}

	rc = dp_in_progress(conf);
	if (rc < 0)
		goto unlock;
	if (rc) {
		/* DP is opening or closing */
		rc = -EINPROGRESS;
		goto unlock;
	}

	rc = dp_is_opened(conf);
	if (rc < 0)
		goto unlock;
	if (rc) {
		/* DP is already opened */
		rc = -EALREADY;
		goto unlock;
	}

	rc = gpio_write(pin, true);
	if (!rc) {
		/* Sleep 100ms */
		usleep(100000);

		rc = gpio_write(pin, false);
	}

unlock:
	gpio_unlock(lockfd);

	return rc;
}

int dp_close(struct sky_dev *dev)
{
	struct sky_conf *conf = &dev->devdesc.conf;
	const char *pin = conf->dp.gpio.close_pin;
	int lockfd, rc;

	if (conf->dp.hw_interface == SKY_DP_UNKNOWN)
		return -EOPNOTSUPP;
	if (conf->dp.hw_interface != SKY_DP_GPIO)
		return -EOPNOTSUPP;

	/* GPIO Interface */

	lockfd = gpio_lock();
	if (lockfd < 0)
		return lockfd;

	rc = dp_is_ready(conf);
	if (rc < 0)
		goto unlock;
	if (!rc) {
		/* DP is not ready */
		rc = -EBUSY;
		goto unlock;
	}

	rc = dp_in_progress(conf);
	if (rc < 0)
		goto unlock;
	if (rc) {
		/* DP is opening or closing */
		rc = -EINPROGRESS;
		goto unlock;
	}

	rc = dp_is_closed(conf);
	if (rc < 0)
		goto unlock;
	if (rc) {
		/* DP is already closed */
		rc = -EALREADY;
		goto unlock;
	}

	rc = gpio_write(pin, true);
	if (!rc) {
		/* Sleep 100ms */
		usleep(100000);

		rc = gpio_write(pin, false);
	}

unlock:
	gpio_unlock(lockfd);

	return rc;
}

int dp_state(struct sky_dev *dev, struct sky_droneport_state *state)
{
	struct sky_conf *conf = &dev->devdesc.conf;
	uint32_t status = 0;
	int lockfd, rc;

	if (conf->dp.hw_interface == SKY_DP_UNKNOWN)
		return -EOPNOTSUPP;
	if (conf->dp.hw_interface != SKY_DP_GPIO)
		return -EOPNOTSUPP;

	/* GPIO Interface */

	lockfd = gpio_lock();
	if (lockfd < 0)
		return lockfd;

	rc = dp_is_ready(conf);
	if (rc < 0)
		goto unlock;
	if (rc)
		status |= SKY_DP_IS_READY;

	rc = dp_is_opened(conf);
	if (rc < 0)
		goto unlock;
	if (rc)
		status |= SKY_DP_IS_OPENED;

	rc = dp_is_closed(conf);
	if (rc < 0)
		goto unlock;
	if (rc)
		status |= SKY_DP_IS_CLOSED;

	rc = dp_in_progress(conf);
	if (rc < 0)
		goto unlock;
	if (rc)
		status |= SKY_DP_IN_PROGRESS;

	rc = 0;
	state->status = status;

unlock:
	gpio_unlock(lockfd);

	return rc;
}

int dp_drone_detect(struct sky_dev *dev)
{
	struct sky_conf *conf = &dev->devdesc.conf;

	if (conf->dp.hw_interface == SKY_DP_UNKNOWN)
		return -EOPNOTSUPP;
	if (conf->dp.hw_interface != SKY_DP_GPIO)
		return -EOPNOTSUPP;

	/* GPIO Interface */

	return dp_is_drone_detected(conf);
}

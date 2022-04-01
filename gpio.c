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

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include "gpio.h"

int gpio_lock(void)
{
	int rc, lockfd;

	lockfd = open("/sys/class/gpio/export", O_WRONLY);
	if (lockfd < 0)
		return -errno;

	rc = flock(lockfd, LOCK_EX);
	if (rc) {
		rc = -errno;
		close(lockfd);
		return rc;
	}

	return lockfd;
}

void gpio_unlock(int lockfd)
{
	(void)flock(lockfd, LOCK_UN);
	close(lockfd);
}

int gpio_configure(const char *gpio, int dir)
{
	const char *cmd = "config-pin %s %s >/dev/null 2>&1";
	char cmd_buf[64];
	int rc;

	snprintf(cmd_buf, sizeof(cmd_buf), cmd, gpio,
		 dir == GPIO_DIR_IN ? "in" : "out");

	rc = system(cmd_buf);
	if (rc < 0) {
		rc = errno;
		sky_err("%s: failed to created child process: %s\n",
			__func__, strerror(errno));
		return -rc;
	} else if (WIFEXITED(rc)) {
		rc = WEXITSTATUS(rc);
		if (rc)
			sky_err("%s: cmd \"%s\" failed with: %d\n",
				__func__, cmd_buf, rc);
		return rc;
	} else if (WIFSIGNALED(rc)) {
		sky_err("%s: cmd \"%s\" signaled\n", __func__, cmd);
		/* What we can do? */
		return -EINVAL;
	}

	sky_err("%s: unreachable line\n", __func__);
	/* What we can do? */
	return -EINVAL;
}

int gpio_read(const char *pin)
{
	char cmd_buf[256], buf[3];
	FILE *f;
	int rc;

	/* Convert GPIO name to kernel GPIO number and return value */
	const char *cmd =
		"GPIO=`cat /sys/kernel/debug/gpio | "
		"      grep -oP '(?<=gpio-)\\d+(?=.*%s)'`; "
		" if [ $GPIO ]; then cat /sys/class/gpio/gpio$GPIO/value; fi";

	snprintf(cmd_buf, sizeof(cmd_buf), cmd, pin);

	f = popen(cmd_buf, "re");
	if (!f) {
		rc = errno;
		sky_err("%s: popen failed: %s\n", __func__, strerror(errno));
		return -rc;
	}
	rc = fread(buf, sizeof(buf), 1, f);
	if (rc < 0) {
		rc = errno;
		sky_err("%s: fread failed: %s\n", __func__, strerror(errno));
		return -rc;
	}
	rc = pclose(f);
	if (rc < 0) {
		rc = errno;
		sky_err("%s: failed pclose: %s\n", __func__, strerror(errno));
		return -rc;
	}
	else if (WIFEXITED(rc)) {
		rc = WEXITSTATUS(rc);
		if (rc) {
			sky_err("%s: cmd %s failed with: %d\n",
				__func__, cmd_buf, rc);
			return rc;
		}

		return buf[0] == '1' ? 1 : 0;

	} else if (WIFSIGNALED(rc)) {
		sky_err("%s: cmd %s signaled\n", __func__, cmd);
		/* What we can do? */
		return -EINVAL;
	} else if (WIFSTOPPED(rc)) {
		sky_err("%s: cmd %s stopped\n", __func__, cmd);
		/* What we can do? */
		return -EINVAL;
	}

	sky_err("%s: unreachable line\n", __func__);
	/* What we can do? */
	return -EINVAL;
}

int gpio_write(const char *pin, bool value)
{
	char cmd_buf[256];
	int rc;

	/* Convert GPIO name to kernel GPIO number and write value */
	const char *cmd =
		"GPIO=`cat /sys/kernel/debug/gpio | "
		"      grep -oP '(?<=gpio-)\\d+(?=.*%s)'`; "
		" if [ $GPIO ]; then echo %s > /sys/class/gpio/gpio$GPIO/value; fi";

	snprintf(cmd_buf, sizeof(cmd_buf), cmd, pin, value ? "1" : "0");

	rc = system(cmd_buf);
	if (rc < 0) {
		rc = errno;
		sky_err("%s: failed to created child process: %s\n",
			__func__, strerror(errno));
		return -rc;
	} else if (WIFEXITED(rc)) {
		rc = WEXITSTATUS(rc);
		if (rc)
			sky_err("%s: cmd \"%s\" failed with: %d\n",
				__func__, cmd, rc);
		return rc;
	} else if (WIFSIGNALED(rc)) {
		sky_err("%s: cmd \"%s\" signaled\n", __func__, cmd);
		/* What we can do? */
		return -EINVAL;
	}

	sky_err("%s: unreachable line\n", __func__);
	/* What we can do? */
	return -EINVAL;
}

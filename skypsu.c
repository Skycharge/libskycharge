/*
 * Copyright 2021 Skycharge GmbH
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
#include <stdio.h>

#include "libskypsu.h"

static void usage(void)
{
	printf("<voltage> <current>\n"
	       "Sets desired voltage and current to RSP-750-48 device.\n");
}

int main(int argc, char *argv[])
{
	struct sky_conf conf = {
		.psu = {
			/* For now we use only RSP-750-48 */
			.type = SKY_PSU_RSP_750_48,
			/* Any valid values */
			.voltage = 10.0,
			.current = 1.0,
		}
	};
	struct sky_psu psu;

	float voltage, current;
	int rc;

	if (argc != 1 && argc != 3) {
		usage();
		return 1;
	}
	if (argc == 3) {
		rc = sscanf(argv[1], "%f", &voltage);
		if (rc != 1) {
			usage();
			return 1;
		}
		rc = sscanf(argv[2], "%f", &current);
		if (rc != 1) {
			usage();
			return 1;
		}
	}

	rc = sky_psu_init(&conf, &psu);
	if (rc) {
		printf("sky_psu_init: failed!\n");
		return 1;
	}

	if (argc == 3) {
		rc = sky_psu_set_voltage(&psu, voltage);
		if (rc) {
			printf("sky_psu_set_voltage: failed!\n");
			return 1;
		}
		rc = sky_psu_set_current(&psu, current);
		if (rc) {
			printf("sky_psu_set_current: failed!\n");
			return 1;
		}
	} else {
		rc = sky_psu_get_voltage(&psu, &voltage);
		if (rc) {
			printf("sky_psu_get_voltage: failed!\n");
			return 1;
		}
		rc = sky_psu_get_current(&psu, &current);
		if (rc) {
			printf("sky_psu_get_current: failed!\n");
			return 1;
		}

		printf("Voltage %.1fV\nCurrent %.1fA\n", voltage, current);
	}
	sky_psu_deinit(&psu);

	return 0;
}

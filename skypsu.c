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

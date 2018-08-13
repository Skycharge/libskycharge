#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/mman.h>
#include <assert.h>

#include "skybms-cmd.h"
#include "skyproto.h"
#include "libskybms.h"
#include "bms-btle.h"

/* TODO: same function exists in skyserver.c, skybroker.c */
static int sky_pidfile_create(const char *pidfile, int pid)
{
	int rc, fd, n;
	char buf[16];

	fd = open(pidfile, O_CREAT | O_EXCL | O_TRUNC | O_WRONLY, 0600);
	if (fd < 0) {
		sky_err("sky_pidfile_create: failed open(%s) errno=%d\n",
			pidfile, errno);

		return -1;
	}
	n = snprintf(buf, sizeof(buf), "%u\n", pid);
	rc = write(fd, buf, n);
	if (rc < 0) {
		sky_err("sky_pidfile_create: failed write(%s) errno=%d\n",
			pidfile, errno);
		close(fd);

		return -1;
	}
	close(fd);

	return 0;
}

/* TODO: same function exists in skyserver.c, skybroker.c */
/*
 * This routine is aimed to be used instead of standard call daemon(),
 * because we want to create pidfile from a parent process not to race
 * with a systemd PIDFile handler.
 */
static int sky_daemonize(const char *pidfile)
{
	int pid, rc;

	pid = fork();
	if (pid == -1) {
		sky_err("sky_daemonize: fork() failed, errno=%d\n", errno);
		return -1;
	}
	else if (pid != 0) {
		if (pidfile) {
			/*
			 * Yes, we write pid from a parent to avoid any races
			 * inside systemd PIDFile handlers.  In case of errors
			 * child is gracefully killed.
			 */

			rc = sky_pidfile_create(pidfile, pid);
			if (rc) {
				kill(pid, SIGTERM);

				return -1;
			}
		}
		/* Parent exits */
		exit(0);
	}
	if (setsid() == -1) {
		sky_err("sky_daemonize: setsid() failed, errno=%d\n", errno);
		return -1;
	}
	pid = chdir("/");
	(void)pid;

	return 0;

}

/**
 * XXX Proper time should be returned by the hardware!
 * XXX The following is not accurate and crappy!
 */
static void calculate_left_time_to_full_charge(struct cli *cli,
					       struct bms_data *bms_data)
{
	float secs_in_h = 3600.0;
	float till_full_secs =
		(float)atoi(cli->capacitymah) /
		(float)atoi(cli->currentma) *
		secs_in_h;

	unsigned left_secs =
		till_full_secs * (100 - bms_data->charge_perc) / 100.0;

	bms_data->charge_time = left_secs;
}

int main(int argc, char *argv[])
{
	struct bms_btle *bms_btle;
	struct bms_data bms_data;
	struct bms_lib bms_lib;
	struct cli cli;
	int rc;

	rc = cli_parse(argc, argv, &cli);
	if (rc) {
		fprintf(stderr, "%s\n", cli_usage);
		return -1;
	}
	rc = bms_btle_open(&bms_btle);
	if (rc) {
		fprintf(stderr, "bms_btle_open(): %s\n", strerror(-rc));
		return -1;
	}

	/* Daemonize */
	if (cli.daemon) {
		sky_daemonize(cli.pidf);
	}

	bms_init(&bms_lib);
	while (1) {
		rc = bms_btle_request_data(bms_btle, &bms_data);
		if (rc) {
			fprintf(stderr, "bms_btle_request_data(): %s\n",
				strerror(-rc));
			sleep(1);
		}

		//XXX REMOVE ASAP, should be done by BMS
		calculate_left_time_to_full_charge(&cli, &bms_data);

		rc = bms_update_data(&bms_lib, &bms_data);
		if (rc) {
			fprintf(stderr, "bms_update_data(): %s\n",
				strerror(-rc));
			sleep(1);
		}
	}
	/* TODO: actually unreachable, do we need to catch a signal? */
	bms_deinit(&bms_lib);
	bms_btle_close(bms_btle);

	return 0;
}

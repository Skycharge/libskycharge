#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>

#include "libskysense-pri.h"
#include "types.h"

static const struct sky_lib_ops *local_ops =
	/* XXX &sky_local_lib_ops; */
	&sky_dummy_lib_ops;

int sky_devslist(struct sky_dev **head)
{
	/* Forward to local implementation */
	return local_ops->devslist(head);
}

void sky_devsfree(struct sky_dev *head)
{
	struct sky_dev *next;

	while (head) {
		next = head->next;
		free(head);
		head = next;
	}
}

int sky_libopen(const struct sky_lib_conf *conf, struct sky_lib **lib_)
{
	const struct sky_lib_ops *ops;
	int rc;

	if (conf->conn_type == SKY_LOCAL)
		ops = local_ops;
	else if (conf->conn_type == SKY_REMOTE)
		ops = &sky_remote_lib_ops;
	else
		return -EINVAL;

	rc = ops->libopen(conf, lib_);
	if (!rc) {
		struct sky_lib *lib = *lib_;

		lib->ops  = *ops;
		lib->conf = *conf;
	}

	return rc;
}

void sky_libclose(struct sky_lib *lib)
{
	lib->ops.libclose(lib);
}

int sky_devinfo(struct sky_lib *lib, struct sky_dev *dev)
{
	return lib->ops.devinfo(lib, dev);
}

int sky_confget(struct sky_lib *lib, struct sky_dev_conf *conf)
{
	return lib->ops.confget(lib, conf);
}

int sky_confset(struct sky_lib *lib, struct sky_dev_conf *conf)
{
	return lib->ops.confset(lib, conf);
}

int sky_chargingstate(struct sky_lib *lib, struct sky_charging_state *state)
{
	return lib->ops.chargingstate(lib, state);
}

static inline unsigned long long msecs_epoch(void)
{
	struct timespec ts;
	unsigned long long msecs;

	clock_gettime(CLOCK_REALTIME, &ts);
	msecs  = ts.tv_sec * 1000ull;
	msecs += ts.tv_nsec / 1000000ull;

	return msecs;
}

static void *subscription_work(void *data)
{
	struct sky_charging_state state;
	struct sky_lib *lib = data;
	unsigned long long ms;
	int rc;

	while (!lib->unsubscribed) {
		ms = msecs_epoch();
		rc = lib->ops.subscription_work(lib, &state);
		ms = msecs_epoch() - ms;
		if (!rc)
			/* Notify subscribers only in case of success */
			lib->subsc.on_state(lib->subsc.data, &state);

		if (ms < lib->subsc.interval_msecs) {
			ms = lib->subsc.interval_msecs - ms;
			usleep(ms * 1000);
		}
	}

	return NULL;
}

int sky_subscribe(struct sky_lib *lib,
		  struct sky_subscription *subsc)
{
	int rc;

	if (lib->thread)
		return -EEXIST;
	if (subsc->on_state == NULL || !subsc->interval_msecs)
		return -EINVAL;

	memcpy(&lib->subsc, subsc, sizeof(*subsc));

	rc = lib->ops.subscribe(lib);
	if (rc)
		return rc;
	rc = pthread_create(&lib->thread, NULL, subscription_work, lib);
	if (rc)
		lib->ops.unsubscribe(lib);

	return -rc;
}

int sky_unsubscribe(struct sky_lib *lib)
{
	if (!lib->thread)
		return -ENOENT;

	lib->unsubscribed = true;
	pthread_join(lib->thread, NULL);

	lib->unsubscribed = false;
	lib->thread = 0;

	lib->ops.unsubscribe(lib);

	return 0;
}

int sky_reset(struct sky_lib *lib)
{
	return lib->ops.reset(lib);
}

int sky_autoscan(struct sky_lib *lib, unsigned autoscan)
{
	return lib->ops.autoscan(lib, autoscan);
}

int sky_chargestart(struct sky_lib *lib)
{
	return lib->ops.chargestart(lib);
}

int sky_chargestop(struct sky_lib *lib)
{
	return lib->ops.chargestop(lib);
}

int sky_coveropen(struct sky_lib *lib)
{
	return lib->ops.coveropen(lib);
}

int sky_coverclose(struct sky_lib *lib)
{
	return lib->ops.coverclose(lib);
}

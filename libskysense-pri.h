#ifndef LIBSKYSENSE_PRI_H
#define LIBSKYSENSE_PRI_H

#include "libskysense.h"
#include "types.h"

struct sky_lib_ops {
	int (*devslist)(struct sky_dev_desc **head);
	int (*libopen)(const struct sky_lib_conf *conf, struct sky_lib **lib);
	void (*libclose)(struct sky_lib *lib);
	int (*devinfo)(struct sky_lib *lib, struct sky_dev_desc *dev);
	int (*paramsget)(struct sky_lib *lib, struct sky_dev_params *params);
	int (*paramsset)(struct sky_lib *lib, struct sky_dev_params *params);
	int (*chargingstate)(struct sky_lib *lib, struct sky_charging_state *state);
	int (*subscribe)(struct sky_lib *lib);
	void (*unsubscribe)(struct sky_lib *lib);
	int (*subscription_work)(struct sky_lib *lib,
				 struct sky_charging_state *state);
	int (*reset)(struct sky_lib *lib);
	int (*chargestart)(struct sky_lib *lib);
	int (*chargestop)(struct sky_lib *lib);
	int (*coveropen)(struct sky_lib *lib);
	int (*coverclose)(struct sky_lib *lib);
};

struct sky_lib {
	struct sky_lib_ops ops;
	struct sky_lib_conf conf;
	struct sky_subscription subsc;
	bool unsubscribed;
	pthread_t thread;
};

extern struct sky_lib_ops sky_remote_lib_ops;
extern struct sky_lib_ops sky_local_lib_ops;
extern struct sky_lib_ops sky_dummy_lib_ops;

#endif /* LIBSKYSENSE_PRI_H */

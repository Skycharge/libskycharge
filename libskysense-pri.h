#ifndef LIBSKYSENSE_PRI_H
#define LIBSKYSENSE_PRI_H

#include "libskysense.h"
#include "types.h"

struct sky_dev_ops {
	int (*devslist)(struct sky_dev_desc **head);
	int (*devopen)(const struct sky_dev_conf *conf, struct sky_dev **dev);
	void (*devclose)(struct sky_dev *dev);
	int (*devinfo)(struct sky_dev *dev, struct sky_dev_desc *devdesc);
	int (*paramsget)(struct sky_dev *dev, struct sky_dev_params *params);
	int (*paramsset)(struct sky_dev *dev, struct sky_dev_params *params);
	int (*chargingstate)(struct sky_dev *dev, struct sky_charging_state *state);
	int (*subscribe)(struct sky_dev *dev);
	void (*unsubscribe)(struct sky_dev *dev);
	int (*subscription_work)(struct sky_dev *dev,
				 struct sky_charging_state *state);
	int (*reset)(struct sky_dev *dev);
	int (*chargestart)(struct sky_dev *dev);
	int (*chargestop)(struct sky_dev *dev);
	int (*coveropen)(struct sky_dev *dev);
	int (*coverclose)(struct sky_dev *dev);
};

struct sky_dev {
	struct sky_dev_ops ops;
	struct sky_dev_conf conf;
	struct sky_subscription subsc;
	bool unsubscribed;
	pthread_t thread;
};

extern struct sky_dev_ops sky_remote_dev_ops;
extern struct sky_dev_ops sky_local_dev_ops;
extern struct sky_dev_ops sky_dummy_dev_ops;

#endif /* LIBSKYSENSE_PRI_H */

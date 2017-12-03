#ifndef LIBSKYSENSE_PRI_H
#define LIBSKYSENSE_PRI_H

#include "libskysense.h"
#include "types.h"

struct sky_dev_ops {
	enum sky_con_type contype;
	int (*devslist)(const struct sky_dev_ops *ops,
			const struct sky_dev_conf *conf, struct sky_dev_desc **head);
	int (*devopen)(const struct sky_dev_desc *devdesc, struct sky_dev **dev);
	void (*devclose)(struct sky_dev *dev);
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
	struct sky_dev_ops *next;
};

extern void __sky_register_devops(struct sky_dev_ops *ops);

#define sky_register_devops(ops)                                       \
       __attribute__((constructor)) static void register_devops(void)  \
       {                                                               \
               __sky_register_devops(ops);			       \
       }                                                               \

#define foreach_devops(ops, devops)			\
       for (ops = devops; ops; ops = ops->next)

struct sky_dev {
	struct sky_dev_desc devdesc;
	struct sky_subscription subsc;
	bool unsubscribed;
	pthread_t thread;
};

#endif /* LIBSKYSENSE_PRI_H */

#ifndef AVAHI_H
#define AVAHI_H

#include <stdint.h>

struct avahi;

int avahi_init(struct avahi **);
void avahi_deinit(struct avahi *);
int avahi_publish_service(struct avahi *, const char *name, const char *type,
			  const char *domain, const char *host, uint16_t port,
			  ...);

#endif /* AVAHI_H */

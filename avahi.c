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

#include "avahi.h"
#include "types.h"

#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include <avahi-client/client.h>
#include <avahi-client/publish.h>
#include <avahi-common/simple-watch.h>
#include <avahi-common/error.h>

struct avahi {
	AvahiEntryGroup *group;
	AvahiSimplePoll *poll;
	AvahiClient     *client;
};

static void entry_group_callback(AvahiEntryGroup *g, AvahiEntryGroupState state,
				 void *arg) {
	struct avahi *avahi = arg;

	assert(g == avahi->group || avahi->group == NULL);

	/* Called whenever the entry group state changes */
	switch (state) {
        case AVAHI_ENTRY_GROUP_ESTABLISHED:
		/* The entry group has been established successfully */
		break;
        case AVAHI_ENTRY_GROUP_COLLISION: {
		sky_err("avahi: service name collision\n");
		/* And recreate the services */
		break;
        }
        case AVAHI_ENTRY_GROUP_FAILURE:
		sky_err("avahi: entry group failure: %s\n",
			avahi_strerror(avahi_client_errno(avahi->client)));
		/* Some kind of failure happened while we were registering our services */
		break;
        case AVAHI_ENTRY_GROUP_UNCOMMITED:
        case AVAHI_ENTRY_GROUP_REGISTERING:
		break;
	}
}

static void client_callback(AvahiClient *client, AvahiClientState state, void *arg)
{
	struct avahi *avahi = arg;
	AvahiEntryGroup *group;

	assert(client);

	/* Called whenever the client or server state changes */
	switch (state) {
        case AVAHI_CLIENT_S_RUNNING:
		group = avahi_entry_group_new(client, entry_group_callback, avahi);
		if (group)
			avahi->group = group;

		/* The server has startup successfully and registered its host
		 * name on the network, so it's time to create our services */
		break;
        case AVAHI_CLIENT_FAILURE:
		sky_err("avahi: %s\n", avahi_strerror(avahi_client_errno(client)));
		break;
        case AVAHI_CLIENT_S_COLLISION:
		/* Let's drop our registered services. When the server is back
		 * in AVAHI_SERVER_RUNNING state we will register them
		 * again with the new host name. */
        case AVAHI_CLIENT_S_REGISTERING:
		/* The server records are now being established. This
		 * might be caused by a host name change. We need to wait
		 * for our own records to register until the host name is
		 * properly esatblished. */
		if (avahi->group)
			avahi_entry_group_reset(avahi->group);
		break;
        case AVAHI_CLIENT_CONNECTING:
		break;
	}
}

int avahi_init(struct avahi **pavahi)
{
	struct avahi *avahi;
	AvahiSimplePoll *poll;
	AvahiClient *client;
	int err;

	*pavahi = NULL;

	avahi = calloc(1, sizeof(*avahi));
	if (!avahi)
		return -ENOMEM;

	poll = avahi_simple_poll_new();
	if (!poll) {
		free(avahi);
		return -ENOMEM;
	}
	avahi->poll = poll;

	client = avahi_client_new(avahi_simple_poll_get(poll), 0,
				  client_callback, avahi, &err);
	if (!client) {
		avahi_simple_poll_free(poll);
		free(avahi);
		return -ENOMEM;
	}
	avahi->client = client;
	*pavahi = avahi;

	return 0;
}

void avahi_deinit(struct avahi *avahi)
{
	if (!avahi)
		return;

	avahi_entry_group_reset(avahi->group);
	avahi_client_free(avahi->client);
	avahi_simple_poll_free(avahi->poll);
	free(avahi);
}

int avahi_publish_service_va(struct avahi *avahi, const char *name, const char *type,
			     const char *domain, const char *host, uint16_t port,
			     va_list va)
{
	AvahiStringList *list;
	int ret;

	list = avahi_string_list_new_va(va);
	ret = avahi_entry_group_add_service_strlst(avahi->group, AVAHI_IF_UNSPEC,
						   AVAHI_PROTO_INET, 0, name, type,
						   domain, host, port, list);

	avahi_string_list_free(list);

	if (!ret)
		ret = avahi_entry_group_commit(avahi->group);

	return ret;
}


int avahi_publish_service(struct avahi *avahi, const char *name, const char *type,
			  const char *domain, const char *host, uint16_t port,
			  ...)
{
	va_list va;
	int ret;

	va_start(va, port);
	ret = avahi_publish_service_va(avahi, name, type, domain,
				       host, port, va);
	va_end(va);

	return ret;
}

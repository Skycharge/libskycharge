#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/mman.h>

#include "types.h"
#include "libskybms.h"

#define LIBSKYBMS_NAME "/run/skybms.map"

static int mem_open(struct bms_lib *bms_lib, bool rdonly)
{
	int flags, proto;
	int rc, fd;
	void *mem;

	flags = rdonly ? O_RDONLY : O_CREAT|O_RDWR;
	proto = PROT_READ | (rdonly ? 0 : PROT_WRITE);

	fd = open(LIBSKYBMS_NAME, flags, 0644);
	if (fd < 0)
		return -errno;

	if (!rdonly) {
		rc = ftruncate(fd, sizeof(*bms_lib->mem));
		if (rc) {
			rc = -errno;
			close(fd);
			return rc;
		}
	}
	mem = mmap(NULL, sizeof(*bms_lib->mem), proto, MAP_SHARED, fd, 0);
	if (mem == MAP_FAILED) {
		rc = -errno;
		close(fd);
		return rc;
	}

	bms_lib->fd = fd;
	bms_lib->mem = mem;

	return 0;
}

static void mem_close(struct bms_lib *bms_lib)
{
	if (bms_lib->fd >= 0) {
		munmap(bms_lib->mem, sizeof(*bms_lib->mem));
		close(bms_lib->fd);
		bms_lib->fd = -1;
		bms_lib->mem = NULL;
	}
}

static int mem_open_or_reopen(struct bms_lib *bms_lib, bool rdonly)
{
	int rc;

	if (bms_lib->fd < 0) {
		return mem_open(bms_lib, rdonly);
	} else {
		rc = access(LIBSKYBMS_NAME, rdonly ? R_OK : R_OK|W_OK);
		if (rc) {
			mem_close(bms_lib);
			return mem_open(bms_lib, rdonly);
		}

		return 0;
	}
}

static inline uint64_t msecs_raw(void)
{
	struct timespec ts;
	uint64_t msecs;

	clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
	msecs  = ts.tv_sec * 1000ull;
	msecs += ts.tv_nsec / 1000000ull;

	return msecs;
}

void bms_init(struct bms_lib *bms_lib)
{
	bms_lib->mem = NULL;
	bms_lib->fd = -1;
}

void bms_deinit(struct bms_lib *bms_lib)
{
	mem_close(bms_lib);
}

int bms_update_data(struct bms_lib *bms_lib, const struct bms_data *data)
{
	uint64_t msecs;
	int rc;

	if (bms_lib->fd < 0) {
		rc = mem_open(bms_lib, false);
		if (rc)
			return rc;
	}

	msecs = msecs_raw();

	/* Lock with odd */
	bms_lib->mem->update_msecs = msecs | 1ull;
	__sync_synchronize();
	bms_lib->mem->data = *data;
	__sync_synchronize();
	/* Unlock with even */
	bms_lib->mem->update_msecs &= ~1ull;

	return 0;
}

int bms_request_data(struct bms_lib *bms_lib, struct bms_data *data)
{
	const unsigned DATA_EXPIRE_MS = 30000;
	const int MAX_ITERS = 1000;

	int rc, iters = 0;
	uint64_t msecs;

	rc = mem_open_or_reopen(bms_lib, true);
	if (rc)
		return rc;
	do {
		msecs = bms_lib->mem->update_msecs;
		if (msecs_raw() >= msecs + DATA_EXPIRE_MS ||
		    ++iters >= MAX_ITERS)
			return -ETIMEDOUT;
		__sync_synchronize();
		*data = bms_lib->mem->data;
	} while (msecs & 1 || msecs != bms_lib->mem->update_msecs);

	return 0;
}

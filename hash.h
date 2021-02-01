#ifndef HASH_H
#define HASH_H

#include <sys/queue.h>
#include "types.h"

/* Must be power of 2 */
#define MAX_BUCKETS 1024

/*
 * This is the "One-at-a-Time" algorithm by Bob Jenkins
 * from requirements by Colin Plumb.
 * (http://burtleburtle.net/bob/hash/doobs.html)
 */
static inline unsigned int jhash(const char *key, size_t len)
{
	unsigned int hash, i;

	for (hash = 0, i = 0; i < len; ++i) {
		hash += key[i];
		hash += (hash << 10);
		hash ^= (hash >> 6);
	}
	hash += (hash << 3);
	hash ^= (hash >> 11);
	hash += (hash << 15);

	return hash;
}

struct hash_entry {
	CIRCLEQ_ENTRY(hash_entry) entry;
	const void   *key;
	size_t        key_len;
};

CIRCLEQ_HEAD(hash_bucket, hash_entry);

struct hash_table {
	struct hash_bucket buckets[MAX_BUCKETS];
};

static inline void hash_table_init(struct hash_table *tbl)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tbl->buckets); i++)
		CIRCLEQ_INIT(&tbl->buckets[i]);
}

static inline void
hash_entry_init(struct hash_entry *e, const void *key,
		size_t key_len)
{
	e->key = key;
	e->key_len = key_len;
}

static inline struct hash_entry *
hash_lookup(const struct hash_table *tbl, const void *key,
	    size_t key_len, unsigned int *hint)
{
	const struct hash_bucket *bucket;
	struct hash_entry *e;
	unsigned int hash, ind;

	hash = jhash((const char *)key, key_len);
	ind = hash & (MAX_BUCKETS - 1);
	bucket = &tbl->buckets[ind];
	if (hint)
		*hint = ind;

	CIRCLEQ_FOREACH(e, bucket, entry) {
		if (e->key_len != key_len)
			continue;
		if (!memcmp(e->key, key, key_len))
			return e;
	}
	return NULL;
}

static inline void
hash_insert(struct hash_entry *e, unsigned int *hint,
	    struct hash_table *tbl)
{
	unsigned int hash, ind;

	if (hint && *hint < MAX_BUCKETS)
		ind = *hint;
	else {
		hash = jhash(e->key, e->key_len);
		ind = hash & (MAX_BUCKETS - 1);
	}
	CIRCLEQ_INSERT_TAIL(&tbl->buckets[ind], e, entry);
}

/* Does not require head */
#define CIRCLEQ_DEL(e, field) do {				\
	typeof((e)->field.cqe_prev) prev = (e)->field.cqe_prev;	\
	typeof((e)->field.cqe_next) next = (e)->field.cqe_next;	\
	prev->field.cqe_next = next;				\
	next->field.cqe_prev = prev;				\
	/* In optimization case pointers can be prefetched, so	\
	 * introduce compiler barrier. */			\
	barrier();						\
} while (0)

static inline void
hash_remove(struct hash_entry *e)
{
	CIRCLEQ_DEL(e, entry);
}

#endif /* HASH_H */

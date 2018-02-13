#ifndef TYPES_H
#define TYPES_H

#include <inttypes.h>
#include <stdbool.h>

#define MAX_ERRNO	255
#define IS_ERR_VALUE(x) unlikely((x) >= (unsigned long)-MAX_ERRNO)

#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2*!!(condition)]))
#define ARRAY_SIZE(arr) (sizeof(arr)/sizeof((arr)[0]))

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef offsetof
#ifdef __compiler_offsetof
#define offsetof(TYPE,MEMBER) __compiler_offsetof(TYPE,MEMBER)
#else
#define offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)
#endif
#endif

#define container_of(ptr, type, member) ({                      \
		const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
		(type *)( (char *)__mptr - offsetof(type,member) );})

#define round_up(x, y)   (((x) + (y) - 1) & ~((y)-1))
#define round_down(x, y) (((x)) & ~((y)-1))
#define div_round_up(x, y)   (((x) + (y) - 1) / (y) * (y))
#define div_round_down(x, y) ((x) / (y) * (y))

#define stringify_1(x...)	#x
#define stringify(x...)	stringify_1(x)

typedef uint16_t le16;
typedef uint32_t le32;

#endif /* TYPES_H */

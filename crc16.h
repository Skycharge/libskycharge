#ifndef _CRC16_H
#define _CRC16_H

#include <stdint.h>

static uint16_t crc16(const void *ptr, size_t len)
{
	const uint8_t *data = ptr;
	uint16_t crc = 0xffff;
	uint8_t x;

	while (len--){
		x = crc >> 8 ^ *data++;
		x ^= x>>4;
		crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);
	}

	return crc;
}

#endif /* _CRC16_H */

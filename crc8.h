#ifndef CRC8_H
#define CRC8_H

/**
 * CRC8, polynomial 0x31, initial value 0x0, final xor 0x0
 * can be checked here: http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
 */
static inline uint8_t crc8(const uint8_t *data, uint16_t len)
{
	uint8_t crc = 0x0;
	uint8_t i;

	while (len--) {
		crc ^= *data++;

		for (i = 0; i < 8; i++)
			crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
	}

    return crc;
}

#endif /* CRC8_H */

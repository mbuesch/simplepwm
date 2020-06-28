#ifndef CRC_H_
#define CRC_H_

uint8_t crc8_update(uint8_t crc, uint8_t data);
uint8_t crc8(const void *buf, uint8_t size);

#endif /* CRC_H_ */

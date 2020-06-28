/*
 * base85.h
 *
 * Created: 26.06.2020 21:17:32
 *  Author: Philipp
 */ 


#ifndef BASE85_H_
#define BASE85_H_

#include <inttypes.h>

int decode_85(uint8_t *dst, const uint8_t *buffer, int len);
int encode_85(uint8_t *buf, const uint8_t *data, int bytes);

#endif /* BASE85_H_ */
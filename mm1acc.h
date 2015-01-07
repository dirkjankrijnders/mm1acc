/*
 * mm1acc.h
 *
 *  Created on: Jan 15, 2014
 *      Author: dirkjan
 */

#ifndef MM1ACC_H_
#define MM1ACC_H_

#include <inttypes.h>

#define MAX_DCC_BYTES 6

typedef struct acc_data {
	uint8_t address;
	uint8_t port;
	uint8_t function;
} acc_data;


typedef enum {
	NONE = 0,
	MM1,
	MM1_VALID,
	DCC_VALID,
	DCC_PREAMBLE,
	DCC_PACKET_START_BIT,
	DCC_BYTE,
	DCC_BYTE_END_BIT,
	DCC_PACKET_END_BIT
} state_t;

void mm1acc_init();
state_t mm1acc_check(acc_data* mm1acc_data);
state_t dccacc_check(uint8_t* data, uint8_t* bytes);
#endif /* MM1ACC_H_ */


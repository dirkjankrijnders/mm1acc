/*
 * mm1acc.h
 *
 *  Created on: Jan 15, 2014
 *      Author: dirkjan
 */

#ifndef MM1ACC_H_
#define MM1ACC_H_

#include <inttypes.h>

typedef struct acc_data {
	uint8_t address;
	uint8_t port;
	uint8_t function;
} acc_data;

void mm1acc_init();
uint8_t mm1acc_check(acc_data* mm1acc_data);

#endif /* MM1ACC_H_ */


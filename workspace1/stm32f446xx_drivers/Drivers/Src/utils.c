/*
 * utils.c
 *
 *  Created on: 23 באוג׳ 2020
 *      Author: amita
 */

#include "utils.h"


void delay(uint8_t delay_loop) {
	for (uint8_t i = 0; i < delay_loop; i++) {
		for (uint32_t j = 0; j < 500000; j++);
	}
}


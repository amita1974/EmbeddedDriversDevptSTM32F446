/*
 * utils.c
 *
 *  Created on: 23 באוג׳ 2020
 *      Author: Amit Alon
 */

#include "utils.h"

//TODO: Calibrate the busy loop value to meet specific time unit
void delay(uint8_t delay_loop) {
	for (uint8_t i = 0; i < delay_loop; i++) {
		for (uint32_t j = 0; j < 500000; j++);
	}
}

void delayBusyCustom(uint32_t uint32_loop_delay_param) {
	for (uint32_t i = 0; i < uint32_loop_delay_param; i++) {
		;
	}
}

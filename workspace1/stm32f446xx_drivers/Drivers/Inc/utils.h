/*
 * utils.h
 *
 *  Created on: 23 באוג׳ 2020
 *      Author: amita
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <stdint.h>

// Busy wait delay function
// TODO: should be calibrated to hold the relevant loop value to fit specific time base
void delay(uint8_t delay_loop);

#endif /* INC_UTILS_H_ */
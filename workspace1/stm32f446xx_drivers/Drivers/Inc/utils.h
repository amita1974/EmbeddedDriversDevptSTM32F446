/*
 * utils.h
 *
 *  Created on: Aug 23 2020
 *      Author: Amit Alon
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <stdint.h>

// Busy wait delay function
// TODO: should be calibrated to hold the relevant loop value to fit specific time base
void delay(uint8_t delay_loop);

#endif /* INC_UTILS_H_ */

/*
 * utils.h
 *
 *  Created on: Aug 23 2020
 *      Author: Amit Alon
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <stdint.h>
//#include "stm32f446xx_gpio_driver.h"

/*
 * Development Board Related Defines
 */
#define NUCLEO64_STM32F446RE
//#define STM32F407G_DISC1

#ifdef NUCLEO64_STM32F446RE
#define BUTTON_PRESSED			0
#define BUTTON_GPIO_PORT 		GPIOC
#define BUTTON_GPIO_PIN_NUM		GPIO_PIN_NUM__13
#else
#ifdef STM32F407G_DISC1
#define BUTTON_PRESSED			1
#define BUTTON_GPIO_PORT 		GPIOA
#define BUTTON_GPIO_PIN_NUM		GPIO_PIN_NUM__0
#else
#error board is not defined. supported develpment boards: NUCLEO64_STM32F446RE or STM32F407G_DISC1
#endif
#endif


// Busy wait delay function
// TODO: should be calibrated to hold the relevant loop value to fit specific time base
void delay(uint8_t delay_loop);
void delayBusyCustom(uint32_t uint32_loop_delay_param);

#endif /* INC_UTILS_H_ */

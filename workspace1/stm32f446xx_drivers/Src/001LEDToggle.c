/*
 * 001LEDToggle.c
 *
 *  Created on: 21 באוג׳ 2020
 *      Author: Amit Alon
 */



/*
 * On Nucleo development board the On board LED, LD2, is connected to PA5,
 * and the button, B1, is connected to PC13.
 * When the button is pressed, the port will be shorted to GND, else pulled up to VCC.
 */

#include "stm32f446xx.h"

//void delay(void) {
//	for (uint32_t j = 0; j < 500000; j++);
//}

int main (void) {
	GPIO_Handle_t GpioLED;
	GpioLED.pGPIOx = GPIOA;
	GpioLED.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NUM__5;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE__OUT;
	GpioLED.GPIO_PinConfig.GPIO_OPType = GPIO_OP_TYPE__PP;
	GpioLED.GPIO_PinConfig.GPIO_OutputSpeed = GPIO_OP_SPEED__FAST;
	GpioLED.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PU_PD__NONE;

	GPIOA_PCLK_EN();
	GPIO_Init(&GpioLED);

	while (1) {
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM__5);
		delay(1);
	}



	return 0;
}

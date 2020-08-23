/*
 * 001LEDToggle.c
 *
 *  Created on: 21 באוג׳ 2020
 *      Author: amita
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
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_5;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;
	GpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU_PD_NONE;

	GPIOA_PCLK_EN();
	GPIO_Init(&GpioLED);

	while (1) {
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);
		delay();
	}



	return 0;
}

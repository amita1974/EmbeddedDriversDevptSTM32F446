/*
 * 002LEDButton.c
 *
 *  Created on: 22 באוג׳ 2020
 *      Author: amita
 */


/*
 * On Nucleo development board the On board LED, LD2, is connected to PA5,
 * and the button, B1, is connected to PC13.
 * When the button is pressed, the port will be shorted to GND, else pulled up to VCC.
 */

#include "stm32f446xx.h"
#define BUTTON_PRESSED	0

void delay(void) {
	for (uint32_t j = 0; j < 500000; j++);
}

int main (void) {
	GPIO_Handle_t GpioLED;
	GpioLED.pGPIOx = GPIOA;
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_5;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;
	GpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU_PD_NONE;

	GPIO_Handle_t GpioUserButton;
	GpioUserButton.pGPIOx = GPIOC;
	GpioUserButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	GpioUserButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioUserButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;
	GpioUserButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU_PD_NONE;

	GPIOA_PCLK_EN();
	GPIOC_PCLK_EN();
	GPIO_Init(&GpioLED);
	GPIO_Init(&GpioUserButton);

	//uint8_t button_state = GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13);
	while (1) {
		if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13) == BUTTON_PRESSED) {
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);
			delay();
		}
//		if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13) != button_state) {
//			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);
//			button_state = !button_state;
//			delay();
//		}
	}



	return 0;
}

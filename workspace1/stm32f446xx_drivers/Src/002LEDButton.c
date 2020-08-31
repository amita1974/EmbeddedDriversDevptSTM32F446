/*
 * 002LEDButton.c
 *
 *  Created on: 22 באוג׳ 2020
 *      Author: Amit Alon
 */


/*
 * On Nucleo development board the On board LED, LD2, is connected to PA5,
 * and the button, B1, is connected to PC13.
 * When the button is pressed, the port will be shorted to GND, else pulled up to VCC.
 */

#include "stm32f446xx.h"
#include "utils.h"
#define BUTTON_PRESSED	0


int main (void) {
	GPIO_Handle_t GpioLED;
	GpioLED.pGPIOx = GPIOA;
	GpioLED.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NUM__5;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE__OUT;
	GpioLED.GPIO_PinConfig.GPIO_OPType = GPIO_OP_TYPE__PP;
	GpioLED.GPIO_PinConfig.GPIO_OutputSpeed = GPIO_OP_SPEED__FAST;
	GpioLED.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PU_PD__NONE;

	GPIO_Handle_t GpioUserButton;
	GpioUserButton.pGPIOx = GPIOC;
	GpioUserButton.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NUM__13;
	GpioUserButton.GPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE__IN;
	GpioUserButton.GPIO_PinConfig.GPIO_OutputSpeed = GPIO_OP_SPEED__FAST;
	GpioUserButton.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PU_PD__NONE;

	GPIOA_PCLK_EN();
	GPIOC_PCLK_EN();
	GPIO_Init(&GpioLED);
	GPIO_Init(&GpioUserButton);

	//uint8_t button_state = GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM__13);
	while (1) {
		if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM__13) == BUTTON_PRESSED) {
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM__5);
			delay(1);
		}
//		if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM__13) != button_state) {
//			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM__5);
//			delay(1);
//		}
	}



	return 0;
}

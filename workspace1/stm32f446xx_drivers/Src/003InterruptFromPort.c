/*
 * 003InterruptFromPort.c
 *
 *  Created on: 23 באוג׳ 2020
 *      Author: amita
 */

#include "stm32f446xx.h"
#include <string.h>

#define BUTTON_PRESSED	0

void EXTI15_10_IRQHandler (void)
{
	GPIO_IRQHandle(13);
}

int main (void) {
//	*((volatile uint32_t*) (NVIC_IPR_BASE + 0x28)) |= 5;

	GPIO_Handle_t GpioLED;
	memset (&GpioLED, 0, sizeof(GpioLED));
	GpioLED.pGPIOx = GPIOA;
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_5;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;
	GpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU_PD_NONE;

	GPIO_Handle_t GpioUserButton;
	memset (&GpioUserButton, 0, sizeof(GpioUserButton));
	GpioUserButton.pGPIOx = GPIOC;
	GpioUserButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	GpioUserButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INT_REI;
	GpioUserButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;
	GpioUserButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU_PD_NONE;

	GPIOA_PCLK_EN();
	GPIOC_PCLK_EN();
	GPIO_Init(&GpioLED);
	GPIO_Init(&GpioUserButton);

	GPIO_IRQInterrupPriCfg(IRQ_NUM_EXTI15_10, 15);
	GPIO_IRQInterrupEnDisCfg(IRQ_NUM_EXTI15_10, 1);

	while (1) {
	}
}


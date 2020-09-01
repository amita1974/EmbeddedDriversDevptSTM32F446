/*
 * 006spi_tx_test.c
 *
 *  Created on: 31 Aug 2020
 *      Author: Amit Alon
 */

#include <string.h>
#include "stm32f446xx.h"



void SPI2_GPIOInits(void) {
	/*
	 * Using AF5 the following pins are configured for SPI2 (taken from Table 11. Alternate function in the STM32 DS)
	 * SPI2_MOSI: PB15
	 * SPI2_MISO: PB14
	 * SPI2_SCK:  PB13 [Optional also: PB10]
	 * SPI2_NSS:  PB12 [Optional also: PB9]
	 */

	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE__ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_AltFunMode = GPIO_AF__5;
	SPIPins.GPIO_PinConfig.GPIO_OutputSpeed = GPIO_OP_SPEED__FAST;
	SPIPins.GPIO_PinConfig.GPIO_OPType = GPIO_OP_TYPE__PP;
	SPIPins.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PU_PD__NONE;

	// SPI2_MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NUM__15;
	GPIO_Init(&SPIPins);

	//// SPI2_MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NUM__14;
	//GPIO_Init(&SPIPins);

	// SPI2_SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NUM__13;
	GPIO_Init(&SPIPins);

	//// SPI2_NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NUM__12;
	//GPIO_Init(&SPIPins);
}

void SPI2_Inits(void) {
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG__FULL_DUPLEX;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DVICE_MODE__MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_BUS_SPEED__DIV2; // 8MHz clock
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL__CK_IDLE_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA__1ST_CLK_SAMPLE;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF__8_BITS;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM__SW; // Software Slave management enabled instead of the NSS pin
	SPI_Init(&SPI2Handle);
}

int main (void) {
	/*
	 * test the following scenarios:
	 * SPI-2 Master mode\SCLK speed = max possible
	 * DFF = 0 and also test with DFF = 1
	 */

	char user_data[] = "Hello world";
	// set up the desired GPIOs to operate as interface pins of SPI_2 peripheral
	SPI2_GPIOInits();
	// Init the SPI peripheral
	SPI2_Inits();

	// Once configured, enable the SPI2 Peripheral
	SPI_PerControl(SPI2, ENABLE);

	// Send the data
	SPI_DataTx(SPI2, (uint8_t*)user_data, strlen(user_data));

	SPI_PerControl(SPI2, DISABLE);

	while (1);
	return 0;
}

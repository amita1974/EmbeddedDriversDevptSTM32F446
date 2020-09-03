/*
 * 006spi_tx_test.c
 *
 *  Created on: Sep 1, 2020
 *      Author: Amit Alon
 */

/*
 * This program sends data from the STM32 MCU using the SPI TX function to another MCU on an Arduino Uno board.
 * The Arduino is configured to be in Slave mode and the STM will be in Master mode.
 * The wires connections are:
 * STM32 GND		<==> Arduino GND
 * STM32 SPI MOSI	<==> Arduino SPI MOSI (11)
 * STM32 SPI SCLK	<==> Arduino SPI SCLK (13)
 * STM32 SPI NSS	<==> Arduino SPI SS   (10)
 * STM32 SPI MISO NC, 	 Arduino SPI MISO NC (12)
 * Logic Analyzer can be connected on the above 3 connection lines
 */

#include <string.h>
#include "stm32f446xx.h"
#include "utils.h"


#ifdef NUCLEO64_STM32F446RE_SPI1_PORTS // development board
#define SPIx	SPI1
#else // STM32F4DISCOVERY (STM32F407G-DISC1) development board or NUCLEO64_STM32F446RE_SPI2_PORTS
#define SPIx	SPI2
#endif // Board based configuration


void SPIx_GPIOInits(void) {
	GPIO_Handle_t SPIPins;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE__ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_AltFunMode = GPIO_AF__5;
	SPIPins.GPIO_PinConfig.GPIO_OutputSpeed = GPIO_OP_SPEED__FAST;
	SPIPins.GPIO_PinConfig.GPIO_OPType = GPIO_OP_TYPE__PP;
	SPIPins.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PU_PD__PU;

#ifdef NUCLEO64_STM32F446RE_SPI1_PORTS // development board
	/*
	 * Values for Nucleo-64 (STM32F446RE based) board: (source: D:Nucleo 64 development board\STM32 Nucleo-64 boards User Manual.pdf, Table 19. Arduino connectors on NUCLEO-F446RE)
	 * SPI1_MOSI: PA7, CN5 Pin:4 PinName:D11
	 * SPI1_MISO: PA6, CN5 Pin:5 PinName:D12
	 * SPI1_SCK:  PA5, CN5 Pin:6 PinName:D13
	 * SPI1_CS:   PB6, CN5 Pin:3 PinName:D10
	 */

	SPIPins.pGPIOx = GPIOA;

	// SPI2_MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NUM__7;
	GPIO_Init(&SPIPins);

	//// SPI2_MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NUM__6;
	//GPIO_Init(&SPIPins);

	// SPI2_SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NUM__5;
	GPIO_Init(&SPIPins);

	// SPI2_NSS
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NUM__6;
	GPIO_Init(&SPIPins);
#else // STM32F4DISCOVERY (STM32F407G-DISC1) development board or NUCLEO64_STM32F446RE_SPI2_PORTS
	/*
	 * Values for STM32F4DISCOVERY (STM32F407G-DISC1) board:
	 * Using AF5 the following pins are configured for SPI2 (taken from Table 11. Alternate function in the STM32 DS)
	 * SPI2_MOSI: PB15
	 * SPI2_MISO: PB14
	 * SPI2_SCK:  PB13 [Optional also: PB10]
	 * SPI2_NSS:  PB12 [Optional also: PB9]
	 */

	SPIPins.pGPIOx = GPIOB;

	// SPI2_MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NUM__15;
	GPIO_Init(&SPIPins);

	//// SPI2_MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NUM__14;
	//GPIO_Init(&SPIPins);

	// SPI2_SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NUM__13;
	GPIO_Init(&SPIPins);

	// SPI2_NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NUM__12;
	GPIO_Init(&SPIPins);
#endif // Board based configuration

}

void SPIx_Inits(void) {
	SPI_Handle_t SPIHandle;
#ifdef NUCLEO64_STM32F446RE_SPI1_PORTS // development board
	SPIHandle.pSPIx = SPI1;
#else // STM32F4DISCOVERY (STM32F407G-DISC1) development board or NUCLEO64_STM32F446RE_SPI2_PORTS
	SPIHandle.pSPIx = SPI2;
#endif // Board based configuration
	SPIHandle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG__FULL_DUPLEX;
	SPIHandle.SPIConfig.SPI_DeviceMode = SPI_DVICE_MODE__MASTER;
	SPIHandle.SPIConfig.SPI_SclkSpeed = SPI_BUS_SPEED__DIV8; // 2MHz clock
	SPIHandle.SPIConfig.SPI_CPOL = SPI_CPOL__CK_IDLE_LOW;
	SPIHandle.SPIConfig.SPI_CPHA = SPI_CPHA__1ST_CLK_SAMPLE;
	SPIHandle.SPIConfig.SPI_DFF = SPI_DFF__8_BITS;
	SPIHandle.SPIConfig.SPI_SSM = SPI_SSM__HW; // Software Slave management enabled instead of the NSS pin
	SPI_Init(&SPIHandle);
}

int main (void) {
	/*
	 * test the following scenarios:
	 * SPI-2 Master mode\SCLK speed = max possible
	 * DFF = 0 and also test with DFF = 1
	 */

	char user_data[] = "Hello world"; // in this program the user data should not exceed 255 bytes.
//	char user_data[] = "A"; // in this program the user data should not exceed 255 bytes.
//	char user_data[] = "1234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890";

	// set up the user button port
	GPIO_Handle_t GpioUserButton;
	GpioUserButton.pGPIOx = BUTTON_GPIO_PORT;
	GpioUserButton.GPIO_PinConfig.GPIO_PinNum = BUTTON_GPIO_PIN_NUM;//
	GpioUserButton.GPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE__IN;
	GpioUserButton.GPIO_PinConfig.GPIO_OutputSpeed = GPIO_OP_SPEED__FAST;
	GpioUserButton.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PU_PD__NONE;
	GPIOC_PCLK_EN();
	GPIO_Init(&GpioUserButton);

	// set up the desired GPIOs to operate as interface pins of SPI_2 peripheral
	SPIx_GPIOInits();
	// Init the relevant SPI peripheral
	SPIx_Inits();

	SPI_SSOEConfig(SPIx, ENABLE);

	while (1) {
		while (GPIO_ReadFromInputPin(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN_NUM) != BUTTON_PRESSED);
		delay(1);// to distinguish between different button presses - allow time to the user to release the button and not see this as another press.

		// Enable the SPIx Peripheral
		SPI_PerControl(SPIx, ENABLE);

		// Send the data
		uint8_t data_len = strlen(user_data);
		if (data_len > 255) {
			// trim the transmitted data in case that it is larger than 255 bytes to the first 255 bytes
			data_len = 255;
		}
		SPI_DataTx(SPIx, &data_len, 1); // send length byte
		SPI_DataTx(SPIx, (uint8_t*)user_data, data_len); // send the data

		// Disable the SPIx Peripheral
		SPI_PerControl(SPIx, DISABLE);
	}

	return 0;
}

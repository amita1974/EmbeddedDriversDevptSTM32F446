/*
 * 008spi_cmd_handling.c
 * (Sec.42, Lec. 155 )
 *
 *  Created on: Sep 4 2020
 *      Author: Amit Alon
 */


/*
 * This program shows communication between master and slave using the driver, and shows use of both SPI Tx and SPI Rx functions.
 * SPI master (STM32) and SPI Slave (Arduino) Command and response based communication.
 * When the user button is pressed master is sending a command to the slave. The slave may respond as needed - per the command implementation.
 *
 * Note:
 * This program version is for basic tests only - the implemented communication protocol is not intended to be robust.
 * Only Basic communication protocol with a very basic errors detection/recovery is implemented.
 *
 *  In this example:
 *  1. there is a demonstration of use of the SPI interface in Full-duplex mode.
 *  2. ST board in SPI master mode, Arduino in SPI Slave mode.
 *  3. DFF = 0 (8 bits data)
 *  4. Hardware Slave Management (SSM = 0)
 *  SCLK Speed = 2MHz, fclk = 16MHz.
 *  Connectivity:
 *  STM MASTER MOSI PB15 (pin 26)<==> Slave MOSI Arduino Pin 11		Yello
 *  STM MASTER MISO PB14 (pin 28)<==> Slave MISO Arduino Pin 12		Green
 *  STM MASTER SCLK PB13 (pin 30)<==> Slave SCLK Arduino Pin 13		Red
 *  STM MASTER NSS  PB12 (pin 16) <==> Slave CS  Arduino Pin 10		Orange
 *  STM MASTER GND       (pin 20) <==> Slave GND Arduino *			Brown
 * Communication protocol:
 *  1. Upon a button press the Master sends cmd opcode (in the order of commands is described below)
 *  2. Slave respond with ACK (0xF5) / NACK byte (0xA5)
 *  3.A. If received ACK master sends more command arguments, if applicable
 *  3.B  If received NACK master display error message.
 *  4. Slave performs the command and may responds with data, if applicable for the specific command.
 *
 *  Supported Commands:
 *	1CMD_LED_CTRL	: <Pin num (1 byte, value 0..9)> <Value(1 byte, 1=On, 0=Off)>. Slave returns: none. (for testing connect LED on pin 9 with 470 Ohm resistor).
 *	2CMD_SENSOR_READ	: <Analog Pin Num (1 byte, Port A0..A5)>. Slave returns: 1 byte Analog value.
 *	3CMD_LED_READ: <Pin num (1 byte, value 0..9)>. Slave returns: 1 byte Digital Value 0/1.
 *	4CMD_PRINT: <LEN(2)> <message(len)>. Slave returns: none.
 *	5CMD_ID_READ. Slave returns: 10 bytes of board string ID.
 *
 * Program order:
 * Start
 * Enter main
 * All inits
 * Wait till button pressed
 * Execute command from the list
 * Command to perform next time = next command.
 * Wait till button pressed
 * Jump back to wait till button pressed.
 *
 */

#include <stdio.h> // for printf
#include "stm32f446xx.h"
#include "utils.h"


// STM32F4DISCOVERY (STM32F407G-DISC1) development board or NUCLEO64_STM32F446RE_SPI2_PORTS
#define SPIx	SPI2


#define CMD_LED_CTRL		0x50
#define CMD_SENSOR_READ		0x51
#define CMD_LED_READ		0x52
#define CMD_PRINT			0x53
#define CMD_ID_READ			0x54

#define LED_OFF			0
#define LED_ON			1

#define ANALOG_PIN_0	0
#define ANALOG_PIN_1	1
#define ANALOG_PIN_2	2
#define ANALOG_PIN_3	3
#define ANALOG_PIN_4	4
#define ANALOG_PIN_5	5

#define LED_PIN			9 // The Arduino pin there the LED is connected

#define ACK_BYTE__VAL_ACK	0xF5
#define ACK_BYTE__VAL_NACK	0xA5

uint8_t dummyWrite = 0xAA; // just a dummy byte to send in order to shift back the data from the slave to the master.
uint8_t dummyRead; // used to clear the RXNE after writing (and as a result getting a byte in return.
uint8_t ackByte;
uint8_t responseByte;
uint8_t led_value = 0;
uint8_t args[2];
uint8_t commandOpcode;

void Button_GPIOInit(void) {
	GPIO_Handle_t GpioUserButton;
	GpioUserButton.pGPIOx = BUTTON_GPIO_PORT;
	GpioUserButton.GPIO_PinConfig.GPIO_PinNum = BUTTON_GPIO_PIN_NUM;//
	GpioUserButton.GPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE__IN;
	GpioUserButton.GPIO_PinConfig.GPIO_OutputSpeed = GPIO_OP_SPEED__FAST;
	GpioUserButton.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PU_PD__NONE;
	GPIO_Init(&GpioUserButton);
}

void SPI2_GPIOInits(void) {
	GPIO_Handle_t SPIPins;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE__ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_AltFunMode = GPIO_AF__5;
	SPIPins.GPIO_PinConfig.GPIO_OutputSpeed = GPIO_OP_SPEED__FAST;
	SPIPins.GPIO_PinConfig.GPIO_OPType = GPIO_OP_TYPE__PP;
	SPIPins.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PU_PD__PU;

// STM32F4DISCOVERY (STM32F407G-DISC1) development board or NUCLEO64_STM32F446RE_SPI2_PORTS
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

	// SPI2_MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NUM__14;
	GPIO_Init(&SPIPins);

	// SPI2_SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NUM__13;
	GPIO_Init(&SPIPins);

	// SPI2_NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NUM__12;
	GPIO_Init(&SPIPins);
}

void SPIx_Inits(void) {
	SPI_Handle_t SPIHandle;
	SPIHandle.pSPIx = SPIx;
	SPIHandle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG__FULL_DUPLEX;
	SPIHandle.SPIConfig.SPI_DeviceMode = SPI_DVICE_MODE__MASTER;
	SPIHandle.SPIConfig.SPI_SclkSpeed = SPI_BUS_SPEED__DIV8; // 2MHz clock. Arduino was able to interpret the opcode value when SPI speed was 4MHz but at 8MHz it couldn't.
															 // TODO: check if changing its clock divider will allow faster SPI clock speed.
	//SPIHandle.SPIConfig.SPI_SclkSpeed = SPI_BUS_SPEED__DIV2; // 8MHz clock, Max possible on STM32, too fast to communicate with ARduino Uno without changing its current SW.
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
	// Set up the user button port
	Button_GPIOInit();

	// Set up the desired GPIOs to operate as interface pins of SPI_2 peripheral
	SPI2_GPIOInits();
	// Init the relevant SPI peripheral
	SPIx_Inits();

	// Set the SSOE, since working in single master mode SSM = 1 (HW based Slave Select Management)
	SPI_SSOEConfig(SPIx, ENABLE);

	while (1) {
		while (GPIO_ReadFromInputPin(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN_NUM) != BUTTON_PRESSED);
		/* User button was pressed. Perform a short delay in order to distinguish between different button
		 * presses - allow time to the user to release the button and not see this as another press. */
		delay(1);

		// Enable the SPIx Peripheral
		SPI_PerControl(SPIx, ENABLE);

		// CMD_LED_CTRL	: <Pin num (1 byte, value 0..9)> <Value(1 byte, 1=On, 0=Off)>. Slave returns: none. (for testing connect LED on pin 9 with 470 Ohm resistor).
		printf("Sending CMD LED Control... ");
		// 1. Send the opcode
		commandOpcode = CMD_LED_CTRL;
		SPI_DataTx(SPIx, &commandOpcode, 1);
		// Dummy read to clear RXNE bit in SR that will be set as a result of the TX.
		SPI_DataRx(SPIx, &dummyRead, 1);

		// Delay is needed in order to allow the Arduino Slave to read the opcode and reply with ACK/NACK.
		// Without the delay the Arduino slave will not have enough time to send the reply and we will read...
		// ..the opcode back before the Arduino could write its response.
		delay(1);

		// 2. Read the ACK/NACK from the slave:
		// 2.1 Send a dummy byte in order to generate clock and shift the result back
		SPI_DataTx(SPIx, &dummyWrite, 1);
		// 2.2 Now the ACK/NACK should be in the master's DR. Read it (after RXNE bit will be set).
		SPI_DataRx(SPIx, &ackByte, 1);
		if (ackByte == ACK_BYTE__VAL_ACK) {
			led_value = (led_value + 1) & 1; // toggle the last led value between the values 0 and 1.
			printf("Slave responded with a ACK (0x%x). Sending CMD LED Control parameters: Pin %d, set to %s\n", ACK_BYTE__VAL_ACK, LED_PIN, led_value == 0 ? "Off" : "On");
		} else if (ackByte == ACK_BYTE__VAL_NACK) {
			printf("Error: Slave responded with a NACK (0x%x) to LED Control command\n", ACK_BYTE__VAL_NACK);
			SPI_PerControl(SPIx, DISABLE);
			continue;
		} else {
			printf("Error: Unexpected ACK value received from the slave on LED control command: 0x%x\n", ackByte);
			SPI_PerControl(SPIx, DISABLE);
			continue;
		}
		args[0] = LED_PIN;
		args[1] = led_value;
		SPI_DataTx(SPIx, args, 2);
		SPI_PerControl(SPIx, DISABLE);


		// CMD_SENSOR_READ	: <Analog Pin Num (1 byte, Port A0..A5)>. Slave returns: 1 byte Analog value.
		while (GPIO_ReadFromInputPin(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN_NUM) != BUTTON_PRESSED);
		/* User button was pressed. Perform a short delay in order to distinguish between different button
		 * presses - allow time to the user to release the button and not see this as another press. */
		delay(1);

		// Enable the SPIx Peripheral
		SPI_PerControl(SPIx, ENABLE);

		// CMD_LED_CTRL	: <Pin num (1 byte, value 0..9)> <Value(1 byte, 1=On, 0=Off)>. Slave returns: none. (for testing connect LED on pin 9 with 470 Ohm resistor).
		printf("Sending Sensor read command... ");
		// 1. Send the opcode
		commandOpcode = CMD_SENSOR_READ;
		SPI_DataTx(SPIx, &commandOpcode, 1);
		// Dummy read to clear RXNE bit in SR that will be set as a result of the TX.
		SPI_DataRx(SPIx, &dummyRead, 1);

		// Delay is needed in order to allow the Arduino Slave to read the opcode and reply with ACK/NACK.
		// Without the delay the Arduino slave will not have enough time to send the reply and we will read...
		// ..the opcode back before the Arduino could write its response.
		delay(1);

		// 2. Read the ACK/NACK from the slave:
		// 2.1 Send a dummy byte in order to generate clock and shift the result back
		SPI_DataTx(SPIx, &dummyWrite, 1);
		// 2.2 Now the ACK/NACK should be in the master's DR. Read it (after RXNE bit will be set).
		SPI_DataRx(SPIx, &ackByte, 1);
		if (ackByte == ACK_BYTE__VAL_ACK) {
			printf("Slave responded with a ACK (0x%x). Sending Sensor read parameters: Pin %d\n", ACK_BYTE__VAL_ACK, ANALOG_PIN_0);
		} else if (ackByte == ACK_BYTE__VAL_NACK) {
			printf("Error: Slave responded with a NACK (0x%x) to Sensor read command\n", ACK_BYTE__VAL_NACK);
			SPI_PerControl(SPIx, DISABLE);
			continue;
		} else {
			printf("Error: Unexpected ACK value received from the slave on Sensor read command: 0x%x\n", ackByte);
			SPI_PerControl(SPIx, DISABLE);
			continue;
		}
		args[0] = ANALOG_PIN_0;
		SPI_DataTx(SPIx, args, 1);

		// Dummy read to clear RXNE bit in SR that will be set as a result of the TX.
		SPI_DataRx(SPIx, &dummyRead, 1);

		// Delay is needed in order to allow the Arduino Slave to read the operand and reply with the sensor value.
		// Without the delay the Arduino slave will not have enough time to measure and send the reply.
		delay(1);

		// 2. Read the Sensor value from the slave:
		// 2.1 Send a dummy byte in order to generate clock and shift the result back
		SPI_DataTx(SPIx, &dummyWrite, 1);
		// 2.2 Now the the Sensor value should be in the master's DR. Read it (after RXNE bit will be set).
		SPI_DataRx(SPIx, &responseByte, 1);
		printf("The sensor value is 0x%x\n", responseByte);

		//TODO: Continue implementation of the following commands.
		// CMD_LED_READ: <Pin num (1 byte, value 0..9)>. Slave returns: 1 byte Digital Value 0/1.

		// CMD_PRINT: <LEN(2)> <message(len)>. Slave returns: none.
		// CMD_ID_READ. Slave returns: 10 bytes of board string ID.

//		// Send the data
//		uint8_t data_len = strlen(user_data);
//		if (data_len > 255) {
//			// trim the transmitted data in case that it is larger than 255 bytes to the first 255 bytes
//			data_len = 255;
//		}
//		SPI_DataTx(SPIx, &data_len, 1); // send length byte
//		SPI_DataTx(SPIx, (uint8_t*)user_data, data_len); // send the data

		// Disable the SPIx Peripheral
		SPI_PerControl(SPIx, DISABLE);
	}

	return 0;
}


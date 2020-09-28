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
 * The program runs with Arduino sketch 002SPISlaveCmdHandling.ino
 * When the user button is pressed master is sending a command to the slave. The slave may respond as needed - per the command implementation.
 *
 * Note:
 * This program version is for basic tests only - the implemented communication protocol is not intended to be robust.
 * Only Basic communication protocol with a very basic (or no) errors detection/recovery is implemented.
 *
 *  In this example:
 *  1. there is a demonstration of use of the SPI interface in Full-duplex mode.
 *  2. ST board in SPI master mode, Arduino in SPI Slave mode.
 *  3. DFF = 0 (8 bits data)
 *  4. Hardware Slave Management (SSM = 0)
 *  SCLK Speed = 2MHz, fclk = 16MHz.
 *  Connectivity:
 *  STM Nucleo board					Arduino Board				Wire color used in the setup
 *  STM MASTER MOSI PB15 (pin 26)<==> Slave MOSI Arduino Pin 11		Yellow
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

#include <stdio.h>  // for printf
#include "stm32f446xx.h"
#include "utils.h"


// STM32F4DISCOVERY (STM32F407G-DISC1) development board or NUCLEO64_STM32F446RE_SPI2_PORTS
#define SPIx	SPI2


#define CMD_LED_CTRL		0x50
#define CMD_SENSOR_READ		0x51
#define CMD_LED_READ		0x52
#define CMD_PRINT			0x53
#define CMD_ID_READ			0x54

#define	CMD_SUCCESS		0
#define	CMD_FAIL		1

#define LED_OFF			0
#define LED_ON			1

#define ANALOG_PIN_0	0
#define ANALOG_PIN_1	1
#define ANALOG_PIN_2	2
#define ANALOG_PIN_3	3
#define ANALOG_PIN_4	4
#define ANALOG_PIN_5	5

#define LED_PIN			9 // The Arduino pin where the LED is connected

#define ACK_BYTE__VAL_ACK	0xF5
#define ACK_BYTE__VAL_NACK	0xA5
#define ACK_BYTE__VAL_OTHER	0xFF // represent any value other than ACK or NACK.

#define MAX_SLAVE_ID_STRLEN 50

char printStringVal[] = "Hello Slave, this is a text message from Master !\n";
uint16_t stringLen = sizeof(printStringVal);


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

	/*
	 * STM32F4DISCOVERY (STM32F407G-DISC1) development board and NUCLEO64_STM32F446RE_SPI2_PORTS
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
	//SPIHandle.SPIConfig.SPI_SclkSpeed = SPI_BUS_SPEED__DIV8; // 2MHz clock. works well with Arduno Uno as SPI Slave
	SPIHandle.SPIConfig.SPI_SclkSpeed = SPI_BUS_SPEED__DIV4;   // 4MHz clock. works well with Arduino Uno as SPI Slave
	//SPIHandle.SPIConfig.SPI_SclkSpeed = SPI_BUS_SPEED__DIV2; // 8MHz clock, Max possible on STM32, too fast to communicate with Arduino Uno without changing its current SW. It is possible to check if changing the Arduino's clock divider will allow faster SPI clock speed.
	SPIHandle.SPIConfig.SPI_CPOL = SPI_CPOL__CK_IDLE_LOW;
	SPIHandle.SPIConfig.SPI_CPHA = SPI_CPHA__1ST_CLK_SAMPLE;
	SPIHandle.SPIConfig.SPI_DFF = SPI_DFF__8_BITS;
	SPIHandle.SPIConfig.SPI_SSM = SPI_SSM__HW; // Software Slave management enabled instead of the NSS pin
	SPI_Init(&SPIHandle);
}

void busyWaitForUserButton(void)
{
	while (GPIO_ReadFromInputPin(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN_NUM) != BUTTON_PRESSED);
	// User button was pressed. Perform a short delay in order to distinguish between different button...
	// ..presses - allow time to the user to release the button and not see this as another press.
	delay(1);
	printf("\n"); // just to make a space line from previous command prints
}

uint8_t receiveAndCheckAckFromSlave(void){
	uint8_t dummyWrite = 0xAA; // just a dummy byte to send in order to shift back the data from the slave to the master.
	uint8_t ackByte;
	// Delay is needed in order to allow the Arduino Slave to read the opcode and reply with ACK/NACK.
	// Without the delay the Arduino slave will not have enough time to send the reply and we will read...
	// ..the opcode back before the Arduino could write its response.
	delay(1);

	// Read the ACK/NACK from the slave:
	// 1 Send a dummy byte in order to generate clock and shift the result back
	SPI_DataTx(SPIx, &dummyWrite, 1);
	// 2 Now the ACK/NACK should be in the master's DR. Read it (after RXNE bit will be set).
	SPI_DataRx(SPIx, &ackByte, 1);
	if (ackByte == ACK_BYTE__VAL_ACK) {
		printf("Slave responded with a ACK (0x%x)\n", ACK_BYTE__VAL_ACK);
		return ACK_BYTE__VAL_ACK;
	} else if (ackByte == ACK_BYTE__VAL_NACK) {
		printf("Error: Slave responded with a NACK (0x%x)\n", ACK_BYTE__VAL_NACK);
		SPI_PerControl(SPIx, DISABLE);
		return ACK_BYTE__VAL_NACK;
	} else {
		printf("Error: Unexpected ACK/NACK value received from the slave: 0x%x\n", ackByte);
		SPI_PerControl(SPIx, DISABLE);
		return ACK_BYTE__VAL_OTHER;
	}
}

// CMD_LED_CTRL	: <Pin num (1 byte, value 0..9)> <Value(1 byte, 1=On, 0=Off)>. Slave returns: none. (for testing connect LED on pin 9 with 470 Ohm resistor).
int i2cSendCmdLedCtrl(void)
{
	uint8_t args[2];
	uint8_t dummyRead; // used to clear the RXNE after writing (and as a result getting a byte in return.
	static uint8_t led_value = 0;
	uint8_t commandOpcode;

	// Enable the SPIx Peripheral
	SPI_PerControl(SPIx, ENABLE);

	printf("Sending CMD LED Control...\n");
	// Send the opcode
	commandOpcode = CMD_LED_CTRL;
	SPI_DataTx(SPIx, &commandOpcode, 1);
	// Dummy read to clear RXNE bit in SR that will be set as a result of the TX.
	SPI_DataRx(SPIx, &dummyRead, 1);

	if (receiveAndCheckAckFromSlave() != ACK_BYTE__VAL_ACK) {
		SPI_PerControl(SPIx, DISABLE);
		return CMD_FAIL;
	}

	led_value = (led_value + 1) & 1; // toggle the last led value between the values 0 and 1.
	printf("LED Control parameters: Pin %d, set to %s\n", LED_PIN, led_value == 0 ? "Off" : "On");
	args[0] = LED_PIN;
	args[1] = led_value;
	SPI_DataTx(SPIx, args, 2);
	SPI_PerControl(SPIx, DISABLE);
	return CMD_SUCCESS;
}

// CMD_SENSOR_READ	: <Analog Pin Num (1 byte, Port A0..A5)>. Slave returns: 1 byte Analog value.
uint8_t i2cSendCmdSensorRead(void)
{
	uint8_t dummyWrite = 0xAA; // just a dummy byte to send in order to shift back the data from the slave to the master.
	uint8_t dummyRead; // used to clear the RXNE after writing (and as a result getting a byte in return.
	uint8_t args[2];
	uint8_t responseByte;
	uint8_t commandOpcode;

	// Enable the SPIx Peripheral
	SPI_PerControl(SPIx, ENABLE);

	printf("Sending Sensor read command...\n");
	// Send the opcode
	commandOpcode = CMD_SENSOR_READ;
	SPI_DataTx(SPIx, &commandOpcode, 1);
	// Dummy read to clear RXNE bit in SR that will be set as a result of the TX.
	SPI_DataRx(SPIx, &dummyRead, 1);

	if (receiveAndCheckAckFromSlave() != ACK_BYTE__VAL_ACK) {
		SPI_PerControl(SPIx, DISABLE);
		return CMD_FAIL;
	}
	printf("Sending Sensor read parameters on Analog pin %d\n", ANALOG_PIN_0);
	args[0] = ANALOG_PIN_0;
	SPI_DataTx(SPIx, args, 1);

	// Dummy read to clear RXNE bit in SR that will be set as a result of the TX.
	SPI_DataRx(SPIx, &dummyRead, 1);

	// Delay is needed in order to allow the Arduino Slave to read the operand and reply with the sensor value.
	// Without the delay the Arduino slave will not have enough time to measure and send the reply.
	delay(1);

	// Read the Sensor value from the slave:
	// 1. Send a dummy byte in order to generate clock and shift the result back
	SPI_DataTx(SPIx, &dummyWrite, 1);
	// 2. Now the the Sensor value should be in the master's DR. Read it (after RXNE bit will be set).
	SPI_DataRx(SPIx, &responseByte, 1);
	printf("The sensor value is %d (0x%x)\n", responseByte, responseByte);
	return CMD_SUCCESS;
}

// CMD_LED_READ: <Pin num (1 byte, value 0..9)>. Slave returns: 1 byte Digital Value 0/1.
uint8_t i2cSendCmdLedRead(uint8_t pinNum) {
	uint8_t dummyWrite = 0xAA; // just a dummy byte to send in order to shift back the data from the slave to the master.
	uint8_t dummyRead; // used to clear the RXNE after writing (and as a result getting a byte in return.
	uint8_t args[2];
	uint8_t responseByte;
	uint8_t commandOpcode;

	// Enable the SPIx Peripheral
	SPI_PerControl(SPIx, ENABLE);

	printf("Sending LED status read command...\n");
	// Send the opcode
	commandOpcode = CMD_LED_READ;
	SPI_DataTx(SPIx, &commandOpcode, 1);
	// Dummy read to clear RXNE bit in SR that will be set as a result of the TX.
	SPI_DataRx(SPIx, &dummyRead, 1);

	if (receiveAndCheckAckFromSlave() != ACK_BYTE__VAL_ACK) {
		SPI_PerControl(SPIx, DISABLE);
		return CMD_FAIL;
	}
	printf("LED status read parameters: Pin %d\n", pinNum);
	args[0] = pinNum;
	SPI_DataTx(SPIx, args, 1);

	// Dummy read to clear RXNE bit in SR that will be set as a result of the TX.
	SPI_DataRx(SPIx, &dummyRead, 1);

	// Delay is needed in order to allow the Arduino Slave to read the operand and reply with the sensor value.
	// Without the delay the Arduino slave will not have enough time to measure and send the reply.
	delay(1);

	// Read the Sensor value from the slave:
	// 1. Send a dummy byte in order to generate clock and shift the result back
	SPI_DataTx(SPIx, &dummyWrite, 1);
	// 2. Now the the Led status value should be in the master's DR. Read it (after RXNE bit will be set).
	SPI_DataRx(SPIx, &responseByte, 1);
	printf("The Led status in pin %d is %d\n", pinNum, responseByte);
	return CMD_SUCCESS;
}

// CMD_PRINT: <LEN(2)> <message(len)>. Slave returns: none.
uint8_t i2cSendCmdPrint(uint16_t stringLen, char* pprintStringVal)
{
	uint8_t args[2];
	uint8_t dummyRead; // used to clear the RXNE after writing (and as a result getting a byte in return.
	uint8_t commandOpcode;

	// Enable the SPIx Peripheral
	SPI_PerControl(SPIx, ENABLE);

	printf("Sending print command...\n");
	// Send the opcode
	commandOpcode = CMD_PRINT;
	SPI_DataTx(SPIx, &commandOpcode, 1);
	// Dummy read to clear RXNE bit in SR that will be set as a result of the TX.
	SPI_DataRx(SPIx, &dummyRead, 1);

	if (receiveAndCheckAckFromSlave() != ACK_BYTE__VAL_ACK) {
		SPI_PerControl(SPIx, DISABLE);
		return CMD_FAIL;
	}
	printf("The print string and length parameters: Len: %d, string: %s\n", stringLen, pprintStringVal);
	args[0] = (uint16_t)stringLen;
	SPI_DataTx(SPIx, args, 1);
	SPI_DataTx(SPIx, (uint8_t*)printStringVal, (uint32_t)stringLen);
	return CMD_SUCCESS;
}

// CMD_ID_READ. Slave returns: 10 bytes of board string ID.
uint8_t i2cSendCmdIdRead()
{
	uint8_t dummyWrite = 0xAA; // just a dummy byte to send in order to shift back the data from the slave to the master.
	uint8_t dummyRead; // used to clear the RXNE after writing (and as a result getting a byte in return.
	char slaveIdSrting[MAX_SLAVE_ID_STRLEN];
	uint8_t slaveIdLen;
	uint8_t commandOpcode;

	// Enable the SPIx Peripheral
	SPI_PerControl(SPIx, ENABLE);

	printf("Sending Slave ID Read command...\n");
	// Send the opcode
	commandOpcode = CMD_ID_READ;
	SPI_DataTx(SPIx, &commandOpcode, 1);
	// Dummy read to clear RXNE bit in SR that will be set as a result of the TX.
	SPI_DataRx(SPIx, &dummyRead, 1);

	if (receiveAndCheckAckFromSlave() != ACK_BYTE__VAL_ACK) {
		SPI_PerControl(SPIx, DISABLE);
		return CMD_FAIL;
	}

	// Delay is needed in order to allow the Arduino Slave to measure and reply with the sensor value.
	delay(1);

	// Read the Slave ID from the slave:
	// 1 Send a dummy byte in order to generate clock and shift the result back
	SPI_DataTx(SPIx, &dummyWrite, 1);
	// 2 Now the the Led status value should be in the master's DR. Read it (after RXNE bit will be set).
	SPI_DataRx(SPIx, &slaveIdLen, 1);
	// Limit the ID string to maximum allocated board ID length to prevent memory leak as a result of length parameter received from the Arduino Slave.
	// TODO: need to update the code to read all the extra bytes from the SPI interface according to the length in order to allow the following commands to work.
	if (slaveIdLen >= MAX_SLAVE_ID_STRLEN) {
		slaveIdLen = MAX_SLAVE_ID_STRLEN - 1;
	}
	printf("Slave ID's length: %d\n", slaveIdLen);
	for (int i = 0; i < slaveIdLen; i++) {
		SPI_DataTx(SPIx, &dummyWrite, 1);
		SPI_DataRx(SPIx, (uint8_t*)&slaveIdSrting[i], 1);
	}
	slaveIdSrting[slaveIdLen] = 0; // null termination at the end of the received string.
	printf("%s\n", (char*)&slaveIdSrting);
	return CMD_SUCCESS;
}



int main (void) {
	/*
	 * test the following scenarios:
	 * SPI-2 Master mode. SCLK speed = max possible
	 * DFF = 0 (TODO: test also with DFF = 1)
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
		do {
			busyWaitForUserButton();
		} while (i2cSendCmdLedCtrl() != CMD_SUCCESS);
		do {
			busyWaitForUserButton();
		} while (i2cSendCmdLedRead(LED_PIN) != CMD_SUCCESS);
		do {
			busyWaitForUserButton();
		} while (i2cSendCmdSensorRead() != CMD_SUCCESS);
		do {
			busyWaitForUserButton();
		} while (i2cSendCmdPrint(stringLen, &printStringVal[0]) != CMD_SUCCESS);
		do {
			busyWaitForUserButton();
		} while (i2cSendCmdIdRead() != CMD_SUCCESS);

		// Disable the SPIx Peripheral
		SPI_PerControl(SPIx, DISABLE);
	}

	return 0;
}


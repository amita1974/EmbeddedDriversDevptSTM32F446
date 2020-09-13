/*
 * 009spi_cmd_handling_int.c
 * (Sec.45, After Lec. 166 (no Lec.)
 *
 *  Created on: Sep 9 2020
 *      Author: Amit Alon
 */


/*
 * SPI tx and rx interrupt based functions basic testing.
 * based on 008spi_cmd_handling.c
 * TODO: enable the errors interrupt and check the OVR error reporting and recovery.
 * The program runs with Arduino sketch 002SPISlaveCmdHandling.ino
 * This program shows communication between master and slave using the driver, and shows use of both SPI Tx and SPI Rx functions.
 * SPI master (STM32) and SPI Slave (Arduino) Command and response based communication.
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
#include <stdlib.h>  // for malloc
#include <string.h>  // for memset
#include "stm32f446xx.h"
#include "utils.h"

#define RX_BUFFER_LEN 500
#define TX_BUFFER_LEN 500

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
#define ACK_BYTE__VAL_TX_BUSY_ERROR	0xFD // Error - when trying to send data on the SPI - the device was busy
#define ACK_BYTE__VAL_RX_BUSY_ERROR	0xFE // Error - when trying to receive data on the SPI - the device was busy

#define MAX_SLAVE_ID_STRLEN 50

/*
 * Global variables
 */
char printStringVal[] = "Hello Slave, this is a text message from Master !\n";
uint16_t stringLen = sizeof(printStringVal);
SPI_Handle_t SPI2Handle;

/*
 * Functions implementation
 */

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


void SPIx_Inits(SPI_Handle_t* pSPIHandle) {
	// interrup based functionality related variables
	pSPIHandle->RxLen = 0;
	pSPIHandle->TxLen = 0;
	pSPIHandle->RxState = SPI_TXRX_STATE__INACTIVE;
	pSPIHandle->TxState = SPI_TXRX_STATE__INACTIVE;
	pSPIHandle->pRxBuff = NULL;
	pSPIHandle->pTxBuff = NULL;
	// SPI registers related variables - relevant both to interrupt and non-interrupt based functions.
	pSPIHandle->pSPIx = SPIx;
	pSPIHandle->SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG__FULL_DUPLEX;
	pSPIHandle->SPIConfig.SPI_DeviceMode = SPI_DVICE_MODE__MASTER;
	pSPIHandle->SPIConfig.SPI_SclkSpeed = SPI_BUS_SPEED__DIV8; // 2MHz clock. works well with Arduno Uno as SPI Slave
	//pSPIHandle->SPIConfig.SPI_SclkSpeed = SPI_BUS_SPEED__DIV4;   // 4MHz clock. works well with Arduino Uno as SPI Slave
	//pSPIHandle->SPIConfig.SPI_SclkSpeed = SPI_BUS_SPEED__DIV2; // 8MHz clock, Max possible on STM32, too fast to communicate with Arduino Uno without changing its current SW. It is possible to check if changing the Arduino's clock divider will allow faster SPI clock speed.
	pSPIHandle->SPIConfig.SPI_CPOL = SPI_CPOL__CK_IDLE_LOW;
	pSPIHandle->SPIConfig.SPI_CPHA = SPI_CPHA__1ST_CLK_SAMPLE;
	pSPIHandle->SPIConfig.SPI_DFF = SPI_DFF__8_BITS;
	pSPIHandle->SPIConfig.SPI_SSM = SPI_SSM__HW; // Software Slave management enabled instead of the NSS pin
	SPI_Init(pSPIHandle);
}


void busyWaitForUserButton(void)
{
	while (GPIO_ReadFromInputPin(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN_NUM) != BUTTON_PRESSED);
	// User button was pressed. Perform a short delay in order to distinguish between different button...
	// ..presses - allow time to the user to release the button and not see this as another press.
	delay(1);
	printf("\n"); // just to make a space line from previous command prints
}


uint8_t receiveAndCheckAckFromSlave(SPI_Handle_t* pSPIxHandle){
	uint8_t dummyWrite = 0xAA; // just a dummy byte to send in order to shift back the data from the slave to the master.
	uint8_t ackByte;
	// Delay is needed in order to allow the Arduino Slave to read the opcode and reply with ACK/NACK.
	// Without the delay the Arduino slave will not have enough time to send the reply and we will read...
	// ..the opcode back before the Arduino could write its response.
	delay(1);

	// Read the ACK/NACK from the slave:
	// 1 Send a dummy byte in order to generate clock and shift the result back
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataTxInt(pSPIxHandle, &dummyWrite, 1)) {
		return ACK_BYTE__VAL_TX_BUSY_ERROR;
	}

	// 2 Now the ACK/NACK should be in the master's DR. Read it (after RXNE bit will be set).
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataRxInt(pSPIxHandle, &ackByte, 1)) {
		return ACK_BYTE__VAL_RX_BUSY_ERROR;
	}

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
int i2cSendCmdLedCtrl(SPI_Handle_t* pSPIxHandle)
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
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataTxInt(pSPIxHandle, &commandOpcode, 1)) {
		return CMD_FAIL;
	}
	// Dummy read to clear RXNE bit in SR that will be set as a result of the TX.
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataRxInt(pSPIxHandle, &dummyRead, 1)) {
		return CMD_FAIL;
	}

	while (receiveAndCheckAckFromSlave(pSPIxHandle) != ACK_BYTE__VAL_ACK) {
		// The application can decide to wait until it will get an ACK or it can continue and do other
		// things until it will get an ACK received indication. This is application-design depended.
	}

	led_value = (led_value + 1) & 1; // toggle the last led value between the values 0 and 1.
	printf("LED Control parameters: Pin %d, set to %s\n", LED_PIN, led_value == 0 ? "Off" : "On");
	args[0] = LED_PIN;
	args[1] = led_value;
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataTxInt(pSPIxHandle, args, 2)) {
		return CMD_FAIL;
	}
	SPI_PerControl(SPIx, DISABLE);
	return CMD_SUCCESS;
}


// CMD_LED_READ: <Pin num (1 byte, value 0..9)>. Slave returns: 1 byte Digital Value 0/1.
uint8_t i2cSendCmdLedRead(SPI_Handle_t* pSPIxHandle, uint8_t pinNum) {
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
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataTxInt(pSPIxHandle, &commandOpcode, 1)) {
		return CMD_FAIL;
	}
	// Dummy read to clear RXNE bit in SR that will be set as a result of the TX.
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataRxInt(pSPIxHandle, &dummyRead, 1)) {
		return CMD_FAIL;
	}

	if (receiveAndCheckAckFromSlave(pSPIxHandle) != ACK_BYTE__VAL_ACK) {
		SPI_PerControl(SPIx, DISABLE);
		return CMD_FAIL;
	}
	printf("LED status read parameters: Pin %d\n", pinNum);
	args[0] = pinNum;
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataTxInt(pSPIxHandle, args, 1)) {
		return CMD_FAIL;
	}

	// Dummy read to clear RXNE bit in SR that will be set as a result of the TX.
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataRxInt(pSPIxHandle, &dummyRead, 1)) {
		return CMD_FAIL;
	}

	// Delay is needed in order to allow the Arduino Slave to read the operand and reply with the sensor value.
	// Without the delay the Arduino slave will not have enough time to measure and send the reply.
	delay(1);

	// Read the Sensor value from the slave:
	// 1. Send a dummy byte in order to generate clock and shift the result back
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataTxInt(pSPIxHandle, &dummyWrite, 1)) {
		return CMD_FAIL;
	}

	// 2. Now the the Led status value should be in the master's DR. Read it (after RXNE bit will be set).
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataRxInt(pSPIxHandle, &responseByte, 1)) {
		return CMD_FAIL;
	}
	printf("The Led status in pin %d is %d\n", pinNum, responseByte);
	return CMD_SUCCESS;
}


// CMD_SENSOR_READ	: <Analog Pin Num (1 byte, Port A0..A5)>. Slave returns: 1 byte Analog value.
uint8_t i2cSendCmdSensorRead(SPI_Handle_t* pSPIxHandle)
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
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataTxInt(pSPIxHandle, &commandOpcode, 1)) {
		return CMD_FAIL;
	}
	// Dummy read to clear RXNE bit in SR that will be set as a result of the TX.
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataRxInt(pSPIxHandle, &dummyRead, 1)) {
		return CMD_FAIL;
	}

	if (receiveAndCheckAckFromSlave(pSPIxHandle) != ACK_BYTE__VAL_ACK) {
		SPI_PerControl(SPIx, DISABLE);
		return CMD_FAIL;
	}
	printf("Sending Sensor read parameters on Analog pin %d\n", ANALOG_PIN_0);
	args[0] = ANALOG_PIN_0;
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataTxInt(pSPIxHandle, args, 1)) {
		return CMD_FAIL;
	}

	// Dummy read to clear RXNE bit in SR that will be set as a result of the TX.
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataRxInt(pSPIxHandle, &dummyRead, 1)) {
		return CMD_FAIL;
	}

	// Delay is needed in order to allow the Arduino Slave to read the operand and reply with the sensor value.
	// Without the delay the Arduino slave will not have enough time to measure and send the reply.
	delay(1);

	// Read the Sensor value from the slave:
	// 1. Send a dummy byte in order to generate clock and shift the result back
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataTxInt(pSPIxHandle, &dummyWrite, 1)) {
		return CMD_FAIL;
	}
	// 2. Now the the Sensor value should be in the master's DR. Read it (after RXNE bit will be set).
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataRxInt(pSPIxHandle, &responseByte, 1)) {
		return CMD_FAIL;
	}
	printf("The sensor value is %d (0x%x)\n", responseByte, responseByte);
	return CMD_SUCCESS;
}


// CMD_PRINT: <LEN(2)> <message(len)>. Slave returns: none.
uint8_t i2cSendCmdPrint(SPI_Handle_t* pSPIxHandle, uint16_t stringLen, char* pprintStringVal)
{
	uint8_t args[2];
	uint8_t dummyRead; // used to clear the RXNE after writing (and as a result getting a byte in return.
	uint8_t commandOpcode;

	// Enable the SPIx Peripheral
	SPI_PerControl(SPIx, ENABLE);

	printf("Sending print command...\n");
	// Send the opcode
	commandOpcode = CMD_PRINT;
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataTxInt(pSPIxHandle, &commandOpcode, 1)) {
		return CMD_FAIL;
	}
	// Dummy read to clear RXNE bit in SR that will be set as a result of the TX.
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataRxInt(pSPIxHandle, &dummyRead, 1)) {
		return CMD_FAIL;
	}

	if (receiveAndCheckAckFromSlave(pSPIxHandle) != ACK_BYTE__VAL_ACK) {
		SPI_PerControl(SPIx, DISABLE);
		return CMD_FAIL;
	}
	printf("The print string and length parameters: Len: %d, string: %s\n", stringLen, pprintStringVal);
	args[0] = (uint16_t)stringLen;
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataTxInt(pSPIxHandle, args, 1)) {
		return CMD_FAIL;
	}
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataTxInt(pSPIxHandle, (uint8_t*)printStringVal, (uint32_t)stringLen)) {
		return CMD_FAIL;
	}
	return CMD_SUCCESS;
}


// CMD_ID_READ. Slave returns: 10 bytes of board string ID.
uint8_t i2cSendCmdIdRead(SPI_Handle_t* pSPIxHandle)
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
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataTxInt(pSPIxHandle, &commandOpcode, 1)) {
		return CMD_FAIL;
	}
	// Dummy read to clear RXNE bit in SR that will be set as a result of the TX.
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataRxInt(pSPIxHandle, &dummyRead, 1)) {
		return CMD_FAIL;
	}

	if (receiveAndCheckAckFromSlave(pSPIxHandle) != ACK_BYTE__VAL_ACK) {
		SPI_PerControl(SPIx, DISABLE);
		return CMD_FAIL;
	}

	// Delay is needed in order to allow the Arduino Slave to measure and reply with the sensor value.
	delay(1);

	// Read the Slave ID from the slave:
	// 1 Send a dummy byte in order to generate clock and shift the result back
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataTxInt(pSPIxHandle, &dummyWrite, 1)) {
		return CMD_FAIL;
	}
	// 2 Now the the Led status value should be in the master's DR. Read it (after RXNE bit will be set).
	if (SPI_TXRX_STATE__ACTIVE == SPI_DataRxInt(pSPIxHandle, &slaveIdLen, 1)) {
		return CMD_FAIL;
	}
	// Limit the ID string to maximum allocated board ID length to prevent memory leak as a result of length parameter received from the Arduino Slave.
	// TODO: need to update the code to read all the extra bytes from the SPI interface according to the length in order to allow the following commands to work.
	if (slaveIdLen >= MAX_SLAVE_ID_STRLEN) {
		slaveIdLen = MAX_SLAVE_ID_STRLEN - 1;
	}
	printf("Slave ID's length: %d\n", slaveIdLen);
	for (int i = 0; i < slaveIdLen; i++) {
		if (SPI_TXRX_STATE__ACTIVE == SPI_DataTxInt(pSPIxHandle, &dummyWrite, 1)) {
			return CMD_FAIL;
		}
		if (SPI_TXRX_STATE__ACTIVE == SPI_DataRxInt(pSPIxHandle, (uint8_t*)&slaveIdSrting[i], 1)) {
			return CMD_FAIL;
		}
	}
	slaveIdSrting[slaveIdLen] = 0; // null termination at the end of the received string.
	printf("%s\n", (char*)&slaveIdSrting);
	return CMD_SUCCESS;
}

void SPI2_IRQHandler(void) {
	SPI_IRQHandle(&SPI2Handle);
}


// TODO: DEBUG - why is this function running in the driver and not in the application layer? the driver is weak attributed !
void SPI_ApplicationEventCallback(SPI_Handle_t* pSPIHandle, SPI_EventAppCallback_t SPI_EventApp)
{
	printf("in App implementaion of SPI_ApplicationEventCallback()\n");
	/*
	 * This is the user application implementation that overwrites the weak function that is implemented in the driver
	 */
	switch (SPI_EventApp) {
		case SPI_EVENT_APP_CALLBACK__TX_CMPLT: {
			// The requested buffer transmission was just completed.
			printf("Application Layer - SPI Event received: TX complete\n");
			break;
		}
		case SPI_EVENT_APP_CALLBACK__RX_CMPLT: {
			// The requested number of bytes reception was just completed. the data is pending in the rxBuffer
			printf("Application Layer - SPI Event received: RX complete\n");
			break;
		}
		case SPI_EVENT_APP_CALLBACK__OVR_ERROR: {
			// OVR Error occurred in the RX channel
			printf("Application Layer - SPI Event received: OVR Error occurred in the RX channel\n");
			break;
		}
		case SPI_EVENT_APP_CALLBACK__CRC_ERROR: {
			// CRC Error identified in the SPI communication
			printf("Application Layer - SPI Event received: CRC Error occurred in the RX channel\n");
			break;
		}
		case SPI_EVENT_APP_CALLBACK__MODF_ERROR: {
			// MODF Error identified in the SPI communication
			printf("Application Layer - SPI Event received: MODF Error occurred\n");
			break;
		}
		case SPI_EVENT_APP_CALLBACK__FRE_ERROR: {
			// FRE Error identified in the SPI communication
			printf("Application Layer - SPI Event received: FRE Error (TI mode frame format error) occurred\n");
			break;
		}
		default: {
			printf("Application Layer - ERROR ! Unexpected Event received\n");
		}
	}
}


int main (void) {
	/*
	 * test the following scenarios:
	 * SPI-2 Master mode. SCLK speed = 2MHz
	 * DFF = 0 (TODO: test also with DFF = 1)
	 */


	// Set up the user button port
	Button_GPIOInit();

	// Set up the desired GPIOs to operate as interface pins of SPI_2 peripheral
	SPI2_GPIOInits();
	// Init the relevant SPI peripheral
	SPIx_Inits(&SPI2Handle);

	// Set the SSOE, since working in single master mode SSM = 1 (HW based Slave Select Management)
	SPI_SSOEConfig(SPIx, ENABLE);

	// enable the SPI2 interrupt source
	SPI_IRQInterrupEnDisCfg(IRQ_NUM_SPI2, ENABLE);

	while (1) {
		do {
			busyWaitForUserButton();
		} while (i2cSendCmdLedCtrl(&SPI2Handle) != CMD_SUCCESS);
		do {
			busyWaitForUserButton();
		} while (i2cSendCmdLedRead(&SPI2Handle, LED_PIN) != CMD_SUCCESS);
		do {
			busyWaitForUserButton();
		} while (i2cSendCmdSensorRead(&SPI2Handle) != CMD_SUCCESS);
		do {
			busyWaitForUserButton();
		} while (i2cSendCmdPrint(&SPI2Handle, stringLen, &printStringVal[0]) != CMD_SUCCESS);
		do {
			busyWaitForUserButton();
		} while (i2cSendCmdIdRead(&SPI2Handle) != CMD_SUCCESS);

		// Disable the SPIx Peripheral
		SPI_PerControl(SPIx, DISABLE);
	}

	return 0;
}


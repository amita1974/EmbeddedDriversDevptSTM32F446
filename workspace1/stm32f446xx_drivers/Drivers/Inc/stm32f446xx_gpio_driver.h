/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: 20 באוג׳ 2020
 *      Author: Amit Alon
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"
#include <stdint.h>

#ifdef DEBUG
#include <assert.h>
#endif

/* Configuration structure for a GPIO pin */
typedef struct {
	uint8_t GPIO_PinNumber;			/*!< Possible values from @GPIO_PIN_NUMBER >*/
	uint8_t GPIO_PinMode;			/*!< Possible values from @GPIO_PIN_MODE >*/
	uint8_t GPIO_PinSpeed;			/*!< Possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;	/*!< Possible values from @GPIO_PU_PD >*/
	uint8_t GPIO_PinOPType;			/*!< Possible values from @GPIO_OUTPUT_TYPE >*/
	uint8_t GPIO_PinAltFunMode;		/*!< Possible values from ??? . Only applicable when GPIO_PinMode is set to GPIO_MODE_ALTFN, else the value here is ignored. >*/
} GPIO_PinConfig_t;

typedef struct {
	GPIO_RegDef_t	*pGPIOx; /*!< The base address of the port of the GPIO */
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

/*
 * @GPIO_PIN_MODE
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2 // alternate function
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_INT_FEI		4 // Falling Edge Interrupt
#define GPIO_MODE_INT_REI		5 // Rising Edge Interrupt
#define GPIO_MODE_INT_RFEI		6 // Rising and Falling Edge Interrupt

/*
 * @GPIO_OUTPUT_TYPE
 * GPIO Output type
 */
#define GPIO_OP_TYPE_PP			0 // Push Pull
#define GPIO_OP_TYPE_OD			1 // Open Drain

/*
 * @GPIO_PIN_SPEED
 * GPIO Output Speed
 */
#define GPIO_OP_SPEED_LOW		0
#define GPIO_OP_SPEED_MED		1
#define GPIO_OP_SPEED_FAST		2
#define GPIO_OP_SPEED_HIGH		3

/*
 * @GPIO_PU_PD
 * GPIO PU/PD
 */
#define GPIO_PU_PD_NONE		0
#define GPIO_PU_PD_PU		1
#define GPIO_PU_PD_PD		2

/*
 * @GPIO_PIN_NUM
 * GPIO PIN Number
 */
#define GPIO_PIN_NUM_0		0
#define GPIO_PIN_NUM_1		1
#define GPIO_PIN_NUM_2		2
#define GPIO_PIN_NUM_3		3
#define GPIO_PIN_NUM_4		4
#define GPIO_PIN_NUM_5		5
#define GPIO_PIN_NUM_6		6
#define GPIO_PIN_NUM_7		7
#define GPIO_PIN_NUM_8		8
#define GPIO_PIN_NUM_9		9
#define GPIO_PIN_NUM_10		10
#define GPIO_PIN_NUM_11		11
#define GPIO_PIN_NUM_12		12
#define GPIO_PIN_NUM_13		13
#define GPIO_PIN_NUM_14		14
#define GPIO_PIN_NUM_15		15

#define NUM_PRIO_BITS_UNUSED_STM32MCU	4 // TODO: understand from which spec this info is coming from - I took it from lecture 113 but could not find any reference that says 4 LSbytes of the priority fields are NA out of each 8...


/* ***************************** */
/* APIs supported by this driver */
/* ***************************** */

/*
 * Peripheral clock setup
 */
void GPIO_PerClockControl (GPIO_RegDef_t * pGPIOx, uint8_t EnOrDis);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t* pGPIOHanlde);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx); // this will reset ALL the registers of specific GPIO port.

/*
 * Data read and write
 */
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum, uint8_t Val);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Val);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQInterrupEnDisCfg(uint8_t IRQNum, uint8_t EnOrDis);
void GPIO_IRQInterrupPriCfg(uint8_t IRQNum, uint8_t IRQPrio);

void GPIO_IRQHandle(uint8_t PinNum);

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */

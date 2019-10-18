/*
 * stm32f303xx_gpio_driver.h
 *
 *  Created on: 09 ott 2019
 *      Author: Pippo
 */

#ifndef INC_STM32F303XX_GPIO_DRIVER_H_
#define INC_STM32F303XX_GPIO_DRIVER_H_

#include "stm32f303xe.h"

/*
 * This is a Configuration structure for a GPIO pin
 */

typedef struct {
	uint8_t GPIO_PinNumber;			// possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;			// possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			// possible values from @GPIO_PIN_SPEEDS
	uint8_t GPIO_PinPuPdControl;	// possible values from @GPIO_PIN_PUPD_CONF
	uint8_t GPIO_PinOpType;			// possible values from @GPIO_PIN_OP_TYPES
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/*
 * This a Handle structure for a GPIO pin
 */
typedef struct {
	GPIO_RegDef_t *pGPIOx_base;	//pointer to hold the address of the GPIO peripheral
	GPIO_PinConfig_t GPIO_PinConfig;	//structure to hold GPIO pin settings
} GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin possible numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN		0 // input mode (reset)
#define GPIO_MODE_OUT		1 // general purpose output mode
#define GPIO_MODE_AF		2 // alternate function mode
#define GPIO_MODE_ANALOG	3 // analog mode
#define GPIO_MODE_IT_FT		4 // interrupt falling edge trigger
#define GPIO_MODE_IT_RT		5 // interrupt rising edge trigger
#define GPIO_MODE_IT_FRT	6 // interrupt falling/rising edge trigger

/*
 * @GPIO_PIN_SPEEDS
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW		0 // speed type low
#define GPIO_SPEED_MEDIUM	1 // speed type medium
#define GPIO_SPEED_HIGH		2 // speed type high

/*
 * @GPIO_PIN_PUPD_CONF
 * GPIO pin possible pull-up/down configurations
 */
#define GPIO_PIN_NO_PUPD	0 // no pull-up/pull-down configuration
#define GPIO_PIN_PU			1 // pull-up configuration
#define GPIO_PIN_PD			2 // pull-down configuration

/*
 * @GPIO_PIN_OP_TYPES
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP		0 // output type push-pull (reset)
#define GPIO_OP_TYPE_OD		1 // output type open drain

/*
 * @GPIO_PIN_AFS
 * GPIO pin possible alternate functions (when AF mode selected)
 */
#define GPIO_AF0			0
#define GPIO_AF1			1
#define GPIO_AF2			2
#define GPIO_AF3			3
#define GPIO_AF4			4
#define GPIO_AF5			5
#define GPIO_AF6			6
#define GPIO_AF7			7
#define GPIO_AF8			8
#define GPIO_AF9			9
#define GPIO_AF10			10
#define GPIO_AF11			11
#define GPIO_AF12			12
#define GPIO_AF13			13
#define GPIO_AF14			14
#define GPIO_AF15			15

/***************************************************************************************************
 * 							APIs supported by the driver
 ***************************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, bool EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t Value);
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, bool EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F303XX_GPIO_DRIVER_H_ */

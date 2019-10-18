/*
 * stm32f303xx_gpio_driver.c
 *
 *  Created on: 09 ott 2019
 *      Author: Pippo
 */

#include <stdlib.h>
#include "stm32f303xx_gpio_driver.h"

/*
 * Peripheral Clock Setup
 */
/************************************************************************************
 * @fn			- GPIO_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the given
 * 				  GPIO port
 *
 * @param[in]	pGPIOx		- Base address of the GPIO peripheral
 * @param[in]	EnorDi		- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, bool EnorDi) {
  //check valid address
  if (pGPIOx == NULL) {
    return;
  }

  if (EnorDi == ENABLE) {
    //check input GPIO and enable its clock
    if (pGPIOx == GPIOA) {
      GPIOA_PCLK_EN();
    } else if (pGPIOx == GPIOB) {
      GPIOB_PCLK_EN();
    } else if (pGPIOx == GPIOC) {
      GPIOC_PCLK_EN();
    } else if (pGPIOx == GPIOD) {
      GPIOD_PCLK_EN();
    } else if (pGPIOx == GPIOE) {
      GPIOE_PCLK_EN();
    } else if (pGPIOx == GPIOF) {
      GPIOF_PCLK_EN();
    } else if (pGPIOx == GPIOG) {
      GPIOG_PCLK_EN();
    } else if (pGPIOx == GPIOH) {
      GPIOH_PCLK_EN();
    }
  } else {
    //check input GPIO and disable its clock
    if (pGPIOx == GPIOA) {
      GPIOA_PCLK_DI();
    } else if (pGPIOx == GPIOB) {
      GPIOB_PCLK_DI();
    } else if (pGPIOx == GPIOC) {
      GPIOC_PCLK_DI();
    } else if (pGPIOx == GPIOD) {
      GPIOD_PCLK_DI();
    } else if (pGPIOx == GPIOE) {
      GPIOE_PCLK_DI();
    } else if (pGPIOx == GPIOF) {
      GPIOF_PCLK_DI();
    } else if (pGPIOx == GPIOG) {
      GPIOG_PCLK_DI();
    } else if (pGPIOx == GPIOH) {
      GPIOH_PCLK_DI();
    }
  }
}

/*
 * Init and De-init
 */
/************************************************************************************
 * @fn			- GPIO_Init
 *
 * @brief		- This function initialize the GPIO peripheral registers according
 * 				  to the configuration structure passed as input
 *
 * @param[in]	pGPIOHandle		- pointer to GPIO handling structure, used to
 * 								  retrieve the GPIO base address and its configuration
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

  uint32_t tempReg = 0;
  //1. configure the mode of gpio pin
  if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
    // non interrupt mode
    tempReg = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
        << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx_base->MODER &= ~(0x3
        << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
    pGPIOHandle->pGPIOx_base->MODER |= tempReg; //setting
  } else {
    // interrupt mode
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
      // configure falling edge register (EXTI_FTSR1)
      EXTI->EXTI_FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
      EXTI->EXTI_RTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    } else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
      // configure rising edge register (EXTI_RTSR1)
      EXTI->EXTI_RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
      EXTI->EXTI_FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    } else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FRT) {
      // configure falling/rising edge registers (EXTI_RTSR1, EXTI_FTSR1)
      EXTI->EXTI_FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
      EXTI->EXTI_RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }

    // 2. configure SYSCFG register to connect GPIO port to EXTI line
    uint8_t reg_n = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) / 4;
    uint8_t shift_n = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 4;
    SYSCFG_PCKL_EN();
    uint8_t portCode = GPIO_BASEADDR_TO_PCODE(pGPIOHandle->pGPIOx_base);
    SYSCFG->SYSCFG_EXTICR[reg_n] |= (portCode << (shift_n * 4));

    // 3. enable EXTI line
    EXTI->EXTI_IMR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
  }

  //2. configure the speed of gpio pin
  tempReg = 0;
  tempReg = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed
      << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
  pGPIOHandle->pGPIOx_base->OSPEEDR &= ~(0x3
      << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
  pGPIOHandle->pGPIOx_base->OSPEEDR |= tempReg; //setting

  //3. configure the pull-up/down of gpio pin
  tempReg = 0;
  tempReg = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl
      << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
  pGPIOHandle->pGPIOx_base->PUPDR &= ~(0x3
      << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
  pGPIOHandle->pGPIOx_base->PUPDR |= tempReg; //setting

  //4. configure the output type of gpio pin
  tempReg = 0;
  tempReg = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType
      << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
  pGPIOHandle->pGPIOx_base->OTYPER &= ~(0x1
      << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
  pGPIOHandle->pGPIOx_base->OTYPER |= tempReg; //setting

  //5. configure the alternate function of gpio pin
  tempReg = 0;
  if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ANALOG) {
    uint8_t temp1, temp2;

    temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
    temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

    tempReg = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
    pGPIOHandle->pGPIOx_base->AFR[temp1] &= ~(0xF << (4 * temp2)); //clearing
    pGPIOHandle->pGPIOx_base->AFR[temp1] |= tempReg; //setting
  }
}

/************************************************************************************
 * @fn			- GPIO_DeInit
 *
 * @brief		- This function de-initialize the GPIO peripheral registers writing
 * 				  to the Peripheral Reset Register in the RCC block
 *
 * @param[in]	pGPIOx		- base address of the GPIO peripheral
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
  //check input GPIO is valid
  if (pGPIOx == NULL) {
    return;
  }

  if (pGPIOx == GPIOA) {
    GPIOA_REG_RST();
  } else if (pGPIOx == GPIOB) {
    GPIOB_REG_RST();
  } else if (pGPIOx == GPIOC) {
    GPIOC_REG_RST();
  } else if (pGPIOx == GPIOD) {
    GPIOD_REG_RST();
  } else if (pGPIOx == GPIOE) {
    GPIOE_REG_RST();
  } else if (pGPIOx == GPIOF) {
    GPIOF_REG_RST();
  } else if (pGPIOx == GPIOG) {
    GPIOG_REG_RST();
  } else if (pGPIOx == GPIOH) {
    GPIOH_REG_RST();
  }
}

/*
 * Data read and write
 */
/************************************************************************************
 * @fn			- GPIO_ReadInputPin
 *
 * @brief		- This function reads the value of the GPIO pin passed as input
 * 				  parameter
 *
 * @param[in]	pGPIOx		- base address of the GPIO peripheral
 * 				PinNumber	- pin number to be read
 *
 * @return		- pin value read from input data register
 *
 * @Note		- none
 */
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

  uint8_t pinValue;
  pinValue = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
  return pinValue;
}

/************************************************************************************
 * @fn			- GPIO_ReadInputPort
 *
 * @brief		- This function reads the value of the GPIO port passed as input
 * 				  parameter
 *
 * @param[in]	pGPIOx		- base address of the GPIO peripheral
 *
 * @return		- port value read from input data register
 *
 * @Note		- none
 */
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx) {

  uint16_t portValue;
  portValue = (uint16_t) (pGPIOx->IDR);
  return portValue;
}

/************************************************************************************
 * @fn			- GPIO_WriteOutputPin
 *
 * @brief		- This function writes the value of a GPIO pin
 *
 * @param[in]	pGPIOx		- base address of the GPIO peripheral
 * 				PinNumber	- pin number to be written
 * 				Value		- value to be written (GPIO_PIN_SET/GPIO_PIN_RESET)
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
    uint8_t Value) {

  if (Value == GPIO_PIN_SET) {
    pGPIOx->ODR |= (1 << PinNumber);
  } else if (Value == GPIO_PIN_RESET) {
    pGPIOx->ODR &= ~(1 << PinNumber);
  }
}

/************************************************************************************
 * @fn			- GPIO_WriteOutputPort
 *
 * @brief		- This function writes the value of a GPIO port
 *
 * @param[in]	pGPIOx		- base address of the GPIO peripheral
 * 				Value		- value to be written (GPIO_PIN_SET/GPIO_PIN_RESET)
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {

  pGPIOx->ODR = Value;
}

/************************************************************************************
 * @fn			- GPIO_ToggleOutputPin
 *
 * @brief		- This function toggles the value of a GPIO pin
 *
 * @param[in]	pGPIOx		- base address of the GPIO peripheral
 * 				PinNumber	- pin number to be toggled
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

  pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ configuration (processor side)
 */
/************************************************************************************
 * @fn			- GPIO_IRQConfig
 *
 * @brief		- This function configures the NVIC to enable/disable an interrupt
 * 				  coming from a given IRQ number (IRQ_NO_EXTI*)
 *
 * @param[in]	IRQNumber	- IRQ number of the EXTI line
 * 				EnorDi		- enable/disable interrupt flag
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_IRQConfig(uint8_t IRQNumber, bool EnorDi) {
  if (EnorDi == ENABLE) {
    if (IRQNumber <= 31) {
      // set NVIC_ISER0 bit
      *(NVIC_ISER0) |= (1 << IRQNumber);
    } else if (IRQNumber >= 32 && IRQNumber <= 63) {
      // set NVIC_ISER1 bit
      *(NVIC_ISER1) |= (1 << IRQNumber % 32);
    }
  } else if (EnorDi == DISABLE) {
    if (IRQNumber <= 31) {
      // set NVIC_ICER0 bit
      *(NVIC_ICER0) |= (1 << IRQNumber);
    } else if (IRQNumber >= 32 && IRQNumber <= 63) {
      // set NVIC_ICER1 bit
      *(NVIC_ICER1) |= (1 << IRQNumber % 32);
    }
  }
}

/************************************************************************************
 * @fn			- GPIO_IRQPriorityConfig
 *
 * @brief		- This function configures the IRQ priority of a given IRQ number
 * 				  (IRQ_NO_EXTI*) in the NVIC
 *
 * @param[in]	IRQNumber	- IRQ number of the EXTI line
 * 				IRQPriority	- IRQ priority number
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
  uint8_t reg_n = IRQNumber / 4;
  uint8_t sect_n = IRQNumber % 4;
  uint8_t shift_amount = (sect_n * 8) + (8 - NO_PR_BITS_IMPLEMENTED);

  *(NVIC_IPR_BASEADDR + reg_n) &= ~(0xF << shift_amount);
  *(NVIC_IPR_BASEADDR + reg_n) |= (IRQPriority << shift_amount);
}

/************************************************************************************
 * @fn			- GPIO_IRQHandling
 *
 * @brief		- This function tests and clears the EXTI pending register of the
 * 				  GPIO pin number triggering the interrupt
 *
 * @param[in]	PinNumber	- pin number triggering the interrupt
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_IRQHandling(uint8_t PinNumber) {
  if ((EXTI->EXTI_PR1 & (1 << PinNumber))) {
    EXTI->EXTI_PR1 |= (1 << PinNumber);
  }
}

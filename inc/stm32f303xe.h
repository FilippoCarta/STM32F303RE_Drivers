/*
 * stm32f303xe.h
 *
 *  Created on: 07 ott 2019
 *      Author: Pippo
 */

#ifndef INC_STM32F303XE_H_
#define INC_STM32F303XE_H_

#include <stdint.h>

#define __vo volatile

/************************** PROCESSOR SPECIFIC DETAILS **************************/
//TODO: implement NVIC registers definition structure
/*
 * NVIC_ISERx : interrupt set-enable register addresses
 */
#define	NVIC_ISER0    ((__vo uint32_t *) 0xE000E100)
#define	NVIC_ISER1    ((__vo uint32_t *) 0xE000E104)
#define	NVIC_ISER2    ((__vo uint32_t *) 0xE000E108)
#define	NVIC_ISER3    ((__vo uint32_t *) 0xE000E10C)

/*
 * NVIC_ICERx : interrupt clear-enable register addresses
 */
#define	NVIC_ICER0    ((__vo uint32_t *) 0xE000E180)
#define	NVIC_ICER1    ((__vo uint32_t *) 0xE000E184)
#define	NVIC_ICER2    ((__vo uint32_t *) 0xE000E188)
#define	NVIC_ICER3    ((__vo uint32_t *) 0xE000E18C)

/*
 * NVIX_IPRx : interrupt priority register address
 */
#define NVIC_IPR_BASEADDR   ((__vo uint32_t *) 0xE000E400)

#define NO_PR_BITS_IMPLEMENTED    4	// number of priority bits implemented

/********************************************************************************/

/******************************** BASE ADDRESSES ********************************/
/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR    0x08000000U
#define SRAM_BASEADDR     0x20000000U
#define ROM_BASEADDR      0x1FFFD800U

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define	PERIPH_BASEADDR   0x40000000U
#define	APB1PER_BASEADDR  PERIPH_BASEADDR
#define	APB2PER_BASEADDR  0x40010000U
#define AHB1PER_BASEADDR  0x40020000U
#define AHB2PER_BASEADDR  0x48000000U
#define AHB3PER_BASEADDR  0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define RCC_BASEADDR      (AHB1PER_BASEADDR + 0x1000)

/*
 * Base addresses of peripherals which are hanging on AHB2 bus
 */

#define GPIOA_BASEADDR    (AHB2PER_BASEADDR + 0x0000)
#define GPIOB_BASEADDR    (AHB2PER_BASEADDR + 0x0400)
#define GPIOC_BASEADDR    (AHB2PER_BASEADDR + 0x0800)
#define GPIOD_BASEADDR    (AHB2PER_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR    (AHB2PER_BASEADDR + 0x1000)
#define GPIOF_BASEADDR    (AHB2PER_BASEADDR + 0x1400)
#define GPIOG_BASEADDR    (AHB2PER_BASEADDR + 0x1800)
#define GPIOH_BASEADDR    (AHB2PER_BASEADDR + 0x1C00)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define SPI2_BASEADDR     (APB1PER_BASEADDR + 0x3800)
#define SPI3_BASEADDR     (APB1PER_BASEADDR + 0x3C00)

#define USART2_BASEADDR   (APB1PER_BASEADDR + 0x4400)
#define USART3_BASEADDR   (APB1PER_BASEADDR + 0x4800)
#define UART4_BASEADDR    (APB1PER_BASEADDR + 0x4C00)
#define UART5_BASEADDR    (APB1PER_BASEADDR + 0x5000)

#define I2C1_BASEADDR     (APB1PER_BASEADDR + 0x5400)
#define I2C2_BASEADDR     (APB1PER_BASEADDR + 0x5800)
#define I2C3_BASEADDR     (APB1PER_BASEADDR + 0x7800)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define SYSCFG_BASEADDR   (APB2PER_BASEADDR + 0x0000)
#define EXTI_BASEADDR     (APB2PER_BASEADDR + 0x0400)

#define SPI1_BASEADDR     (APB2PER_BASEADDR + 0x3000)
#define SPI4_BASEADDR     (APB2PER_BASEADDR + 0x3C00)

#define USART1_BASEADDR   (APB2PER_BASEADDR + 0x3800)

/********************************************************************************/

/****************** PERIPHERAL REGISTER DEFINITION STRUCTURES *******************/
/*
 * GPIOx registers definition structure
 */
typedef struct {
  __vo uint32_t MODER;    // mode register
  __vo uint32_t OTYPER;   // output type register
  __vo uint32_t OSPEEDR;  // output speed register
  __vo uint32_t PUPDR;    // pull-up/pull-down register
  __vo uint32_t IDR;      // input data register
  __vo uint32_t ODR;      // output data register
  __vo uint32_t BSRR;     // bit set/reset register
  __vo uint32_t LCKR;     // configuration lock register
  __vo uint32_t AFR[2];   // alternate function register (low/high)
  __vo uint32_t BRR;      // bit reset register
} GPIO_RegDef_t;

/*
 * RCC registers definition structure
 */
typedef struct {
  __vo uint32_t RCC_CR;       // clock control register
  __vo uint32_t RCC_CFGR;     // clock config register
  __vo uint32_t RCC_CIR;      // clock interrupt register
  __vo uint32_t APB2RSTR;     // APB2 peripheral reset register
  __vo uint32_t RCC_APB1RSTR; // APB1 peripheral reset register
  __vo uint32_t RCC_AHBENR;   // AHB peripheral clock enable register
  __vo uint32_t RCC_APB2ENR;  // APB2 peripheral clock enable register
  __vo uint32_t RCC_APB1ENR;  // APB1 peripheral clock enable register
  __vo uint32_t RCC_BDCR;     // RTC domain control register
  __vo uint32_t RCC_CSR;      // control/status register
  __vo uint32_t RCC_AHBRSTR;  // AHB peripheral reset register
  __vo uint32_t RCC_CFGR2;    // clock configuration register 2
  __vo uint32_t RCC_CFGR3;    // clock configuration register 3
} RCC_RegDef_t;

/*
 * EXTI registers definition structure
 */
typedef struct {
  __vo uint32_t EXTI_IMR1;    // interrupt mask register 1
  __vo uint32_t EXTI_EMR1;    // event mask register 1
  __vo uint32_t EXTI_RTSR1;   // rising trigger selection register 1
  __vo uint32_t EXTI_FTSR1;   // falling trigger selection register 1
  __vo uint32_t EXTI_SWIER1;  // software interrupt event register 1
  __vo uint32_t EXTI_PR1;     // pending register 1
  __vo uint32_t EXTI_IMR2;    // interrupt mask register 2
  __vo uint32_t EXTI_EMR2;    // event mask register 2
  __vo uint32_t EXTI_RTSR2;   // rising trigger selection register 2
  __vo uint32_t EXTI_FTSR2;   // falling trigger selection register 2
  __vo uint32_t EXTI_SWIER2;  // software interrupt event register 2
  __vo uint32_t EXTI_PR2;     // pending register 2
} EXTI_RegDef_t;

/*
 * SYSCFG registers definition structure
 */
typedef struct {
  __vo uint32_t SYSCFG_CFGR1;     // configuration register 1
  __vo uint32_t SYSCFG_RCR;       // CCM SRAM protection register
  __vo uint32_t SYSCFG_EXTICR[4]; // ext interrupt configuration register
  __vo uint32_t SYSCFG_CFGR2;     // configuration register 2
  uint32_t RESERVED1[12];         // reserved
  __vo uint32_t SYSCFG_CFGR3;     // configuration register 3
} SYSCFG_RegDef_t;

/********************************************************************************/

/**************************** PERIPHERAL DEFINITIONS ****************************/
/*
 * Peripheral base addresses type-casted to x_RegDef_t*
 */
#define GPIOA     ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB     ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC     ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD     ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE     ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF     ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG     ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH     ((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC       ((RCC_RegDef_t *) RCC_BASEADDR)
#define EXTI      ((EXTI_RegDef_t *) EXTI_BASEADDR)
#define SYSCFG    ((SYSCFG_RegDef_t *) SYSCFG_BASEADDR)

/********************************************************************************/

/************************* CLOCK ENABLE/DISABLE MACROS **************************/
/*
 * Clock enable/disable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()   (RCC->RCC_AHBENR |= (1 << 17))
#define GPIOB_PCLK_EN()   (RCC->RCC_AHBENR |= (1 << 18))
#define GPIOC_PCLK_EN()   (RCC->RCC_AHBENR |= (1 << 19))
#define GPIOD_PCLK_EN()   (RCC->RCC_AHBENR |= (1 << 20))
#define GPIOE_PCLK_EN()   (RCC->RCC_AHBENR |= (1 << 21))
#define GPIOF_PCLK_EN()   (RCC->RCC_AHBENR |= (1 << 22))
#define GPIOG_PCLK_EN()   (RCC->RCC_AHBENR |= (1 << 23))
#define GPIOH_PCLK_EN()   (RCC->RCC_AHBENR |= (1 << 16))

#define GPIOA_PCLK_DI()   (RCC->RCC_AHBENR &= ~(1 << 17))
#define GPIOB_PCLK_DI()   (RCC->RCC_AHBENR &= ~(1 << 18))
#define GPIOC_PCLK_DI()   (RCC->RCC_AHBENR &= ~(1 << 19))
#define GPIOD_PCLK_DI()   (RCC->RCC_AHBENR &= ~(1 << 20))
#define GPIOE_PCLK_DI()   (RCC->RCC_AHBENR &= ~(1 << 21))
#define GPIOF_PCLK_DI()   (RCC->RCC_AHBENR &= ~(1 << 22))
#define GPIOG_PCLK_DI()   (RCC->RCC_AHBENR &= ~(1 << 23))
#define GPIOH_PCLK_DI()   (RCC->RCC_AHBENR &= ~(1 << 16))

/*
 * Clock enable/disable macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()    (RCC->RCC_APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()    (RCC->RCC_APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()    (RCC->RCC_APB1ENR |= (1 << 30))

#define I2C1_PCLK_DI()    (RCC->RCC_APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()    (RCC->RCC_APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()    (RCC->RCC_APB1ENR &= ~(1 << 30))

/*
 * Clock enable/disable macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()    (RCC->RCC_APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()    (RCC->RCC_APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()    (RCC->RCC_APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()    (RCC->RCC_APB2ENR |= (1 << 15))

#define SPI1_PCLK_DI()    (RCC->RCC_APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()    (RCC->RCC_APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()    (RCC->RCC_APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()    (RCC->RCC_APB2ENR &= ~(1 << 15))

/*
 * Clock enable/disable macros for UARTx/USARTx peripherals
 */
#define USART1_PCLK_EN()    (RCC->RCC_APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()    (RCC->RCC_APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()    (RCC->RCC_APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()     (RCC->RCC_APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()     (RCC->RCC_APB1ENR |= (1 << 20))

#define USART1_PCLK_DI()    (RCC->RCC_APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()    (RCC->RCC_APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()    (RCC->RCC_APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()     (RCC->RCC_APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()     (RCC->RCC_APB1ENR &= ~(1 << 20))

/*
 * Clock enable/disable macros for SYSCFG peripheral
 */
#define SYSCFG_PCKL_EN()     (RCC->RCC_APB2ENR |= (1 << 0))
#define SYSCFG_PCKL_DI()     (RCC->RCC_APB2ENR &= ~(1 << 0))

/********************************************************************************/

/*
 * Reset GPIOx peripherals macros
 */
#define GPIOA_REG_RST()   do { (RCC->RCC_AHBRSTR |= (1 << 17)); (RCC->RCC_AHBRSTR &= ~(1 << 17)); } while (0)
#define GPIOB_REG_RST()   do { (RCC->RCC_AHBRSTR |= (1 << 18)); (RCC->RCC_AHBRSTR &= ~(1 << 18)); } while (0)
#define GPIOC_REG_RST()   do { (RCC->RCC_AHBRSTR |= (1 << 19)); (RCC->RCC_AHBRSTR &= ~(1 << 19)); } while (0)
#define GPIOD_REG_RST()   do { (RCC->RCC_AHBRSTR |= (1 << 20)); (RCC->RCC_AHBRSTR &= ~(1 << 20)); } while (0)
#define GPIOE_REG_RST()   do { (RCC->RCC_AHBRSTR |= (1 << 21)); (RCC->RCC_AHBRSTR &= ~(1 << 21)); } while (0)
#define GPIOF_REG_RST()   do { (RCC->RCC_AHBRSTR |= (1 << 22)); (RCC->RCC_AHBRSTR &= ~(1 << 22)); } while (0)
#define GPIOG_REG_RST()   do { (RCC->RCC_AHBRSTR |= (1 << 23)); (RCC->RCC_AHBRSTR &= ~(1 << 23)); } while (0)
#define GPIOH_REG_RST()   do { (RCC->RCC_AHBRSTR |= (1 << 16)); (RCC->RCC_AHBRSTR &= ~(1 << 16)); } while (0)

/*
 * Return the port code for a given GPIOx base address
 */
#define GPIO_BASEADDR_TO_PCODE(x)	(     (x == GPIOA) ? 0 :\
                                        (x == GPIOB) ? 1 :\
                                        (x == GPIOC) ? 2 :\
                                        (x == GPIOD) ? 3 :\
                                        (x == GPIOE) ? 4 :\
                                        (x == GPIOF) ? 5 :\
                                        (x == GPIOG) ? 6 :\
                                        (x == GPIOH) ? 7 : 0 )

/*
 * IRQ Numbers for STM32F303xe MCU (TODO:complete for all peripherals)
 */
#define IRQ_NO_EXTI0      6
#define IRQ_NO_EXTI1      7
#define IRQ_NO_EXTI2      8
#define IRQ_NO_EXTI3      9
#define IRQ_NO_EXTI4      10
#define IRQ_NO_EXTI9_5    23
#define IRQ_NO_EXTI15_10  40

/******************************* GENERIC MACROS *********************************/
typedef enum {
  ENABLE = 1, DISABLE = 0
} bool;
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET

/********************************************************************************/

#endif /* INC_STM32F303XE_H_ */

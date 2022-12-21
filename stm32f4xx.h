/*
 * stm32f4xx.h
 *
 *  Created on: Jul 1, 2022
 *      Author: soulaimane OuladBelayachi
 */

#ifndef STM32F4XX_H_
#define STM32F4XX_H_


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define 	__VO	volatile


/**
  * @brief Reset and Clock Control
  */

typedef struct
{
  __VO uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  __VO uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  __VO uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  __VO uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  __VO uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  __VO uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  __VO uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  __VO uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  __VO uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  __VO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  __VO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  __VO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  __VO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  __VO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  __VO uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  __VO uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  __VO uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  __VO uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  __VO uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  __VO uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  __VO uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  __VO uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  __VO uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
  __VO uint32_t PLLSAICFGR;    /*!< RCC PLLSAI configuration register,                           Address offset: 0x88 */
  __VO uint32_t DCKCFGR;       /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */


} RCC_TypeDef;



/**
  * @brief General Purpose I/O
  */

typedef struct
{
  __VO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  __VO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  __VO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  __VO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __VO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  __VO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  __VO uint16_t BSRRL;    /*!< GPIO port bit set/reset low register,  Address offset: 0x18      */
  __VO uint16_t BSRRH;    /*!< GPIO port bit set/reset high register, Address offset: 0x1A      */
  __VO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __VO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;

/**
  * @brief Inter-integrated Circuit Interface
  */

typedef struct
{
  __VO uint16_t CR1;        /*!< I2C Control register 1,     Address offset: 0x00 */
  uint16_t      RESERVED0;  /*!< Reserved, 0x02                                   */
  __VO uint16_t CR2;        /*!< I2C Control register 2,     Address offset: 0x04 */
  uint16_t      RESERVED1;  /*!< Reserved, 0x06                                   */
  __VO uint16_t OAR1;       /*!< I2C Own address register 1, Address offset: 0x08 */
  uint16_t      RESERVED2;  /*!< Reserved, 0x0A                                   */
  __VO uint16_t OAR2;       /*!< I2C Own address register 2, Address offset: 0x0C */
  uint16_t      RESERVED3;  /*!< Reserved, 0x0E                                   */
  __VO uint16_t DR;         /*!< I2C Data register,          Address offset: 0x10 */
  uint16_t      RESERVED4;  /*!< Reserved, 0x12                                   */
  __VO uint16_t SR1;        /*!< I2C Status register 1,      Address offset: 0x14 */
  uint16_t      RESERVED5;  /*!< Reserved, 0x16                                   */
  __VO uint16_t SR2;        /*!< I2C Status register 2,      Address offset: 0x18 */
  uint16_t      RESERVED6;  /*!< Reserved, 0x1A                                   */
  __VO uint16_t CCR;        /*!< I2C Clock control register, Address offset: 0x1C */
  uint16_t      RESERVED7;  /*!< Reserved, 0x1E                                   */
  __VO uint16_t TRISE;      /*!< I2C TRISE register,         Address offset: 0x20 */
  uint16_t      RESERVED8;  /*!< Reserved, 0x22                                   */
  __VO uint16_t FLTR;       /*!< I2C FLTR register,          Address offset: 0x24 */
  uint16_t      RESERVED9;  /*!< Reserved, 0x26                                   */
} I2C_TypeDef;


/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */

typedef struct
{
  __VO uint16_t SR;         /*!< USART Status register,                   Address offset: 0x00 */
  uint16_t      RESERVED0;  /*!< Reserved, 0x02                                                */
  __VO uint16_t DR;         /*!< USART Data register,                     Address offset: 0x04 */
  uint16_t      RESERVED1;  /*!< Reserved, 0x06                                                */
  __VO uint16_t BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
  uint16_t      RESERVED2;  /*!< Reserved, 0x0A                                                */
  __VO uint16_t CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
  uint16_t      RESERVED3;  /*!< Reserved, 0x0E                                                */
  __VO uint16_t CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
  uint16_t      RESERVED4;  /*!< Reserved, 0x12                                                */
  __VO uint16_t CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
  uint16_t      RESERVED5;  /*!< Reserved, 0x16                                                */
  __VO uint16_t GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */
  uint16_t      RESERVED6;  /*!< Reserved, 0x1A                                                */
} USART_TypeDef;

/**
  * @brief Serial Peripheral Interface
  */

typedef struct
{
  __VO uint16_t CR1;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
  uint16_t      RESERVED0;  /*!< Reserved, 0x02                                                           */
  __VO uint16_t CR2;        /*!< SPI control register 2,                             Address offset: 0x04 */
  uint16_t      RESERVED1;  /*!< Reserved, 0x06                                                           */
  __VO uint16_t SR;         /*!< SPI status register,                                Address offset: 0x08 */
  uint16_t      RESERVED2;  /*!< Reserved, 0x0A                                                           */
  __VO uint16_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
  uint16_t      RESERVED3;  /*!< Reserved, 0x0E                                                           */
  __VO uint16_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
  uint16_t      RESERVED4;  /*!< Reserved, 0x12                                                           */
  __VO uint16_t RXCRCR;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
  uint16_t      RESERVED5;  /*!< Reserved, 0x16                                                           */
  __VO uint16_t TXCRCR;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
  uint16_t      RESERVED6;  /*!< Reserved, 0x1A                                                           */
  __VO uint16_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
  uint16_t      RESERVED7;  /*!< Reserved, 0x1E                                                           */
  __VO uint16_t I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
  uint16_t      RESERVED8;  /*!< Reserved, 0x22                                                           */
} SPI_TypeDef;

/**
  * @brief TIM
  */

typedef struct
{
  __VO uint16_t CR1;         /*!< TIM control register 1,              Address offset: 0x00 */
  uint16_t      RESERVED0;   /*!< Reserved, 0x02                                            */
  __VO uint16_t CR2;         /*!< TIM control register 2,              Address offset: 0x04 */
  uint16_t      RESERVED1;   /*!< Reserved, 0x06                                            */
  __VO uint16_t SMCR;        /*!< TIM slave mode control register,     Address offset: 0x08 */
  uint16_t      RESERVED2;   /*!< Reserved, 0x0A                                            */
  __VO uint16_t DIER;        /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
  uint16_t      RESERVED3;   /*!< Reserved, 0x0E                                            */
  __VO uint16_t SR;          /*!< TIM status register,                 Address offset: 0x10 */
  uint16_t      RESERVED4;   /*!< Reserved, 0x12                                            */
  __VO uint16_t EGR;         /*!< TIM event generation register,       Address offset: 0x14 */
  uint16_t      RESERVED5;   /*!< Reserved, 0x16                                            */
  __VO uint16_t CCMR1;       /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
  uint16_t      RESERVED6;   /*!< Reserved, 0x1A                                            */
  __VO uint16_t CCMR2;       /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
  uint16_t      RESERVED7;   /*!< Reserved, 0x1E                                            */
  __VO uint16_t CCER;        /*!< TIM capture/compare enable register, Address offset: 0x20 */
  uint16_t      RESERVED8;   /*!< Reserved, 0x22                                            */
  __VO uint32_t CNT;         /*!< TIM counter register,                Address offset: 0x24 */
  __VO uint16_t PSC;         /*!< TIM prescaler,                       Address offset: 0x28 */
  uint16_t      RESERVED9;   /*!< Reserved, 0x2A                                            */
  __VO uint32_t ARR;         /*!< TIM auto-reload register,            Address offset: 0x2C */
  __VO uint16_t RCR;         /*!< TIM repetition counter register,     Address offset: 0x30 */
  uint16_t      RESERVED10;  /*!< Reserved, 0x32                                            */
  __VO uint32_t CCR1;        /*!< TIM capture/compare register 1,      Address offset: 0x34 */
  __VO uint32_t CCR2;        /*!< TIM capture/compare register 2,      Address offset: 0x38 */
  __VO uint32_t CCR3;        /*!< TIM capture/compare register 3,      Address offset: 0x3C */
  __VO uint32_t CCR4;        /*!< TIM capture/compare register 4,      Address offset: 0x40 */
  __VO uint16_t BDTR;        /*!< TIM break and dead-time register,    Address offset: 0x44 */
  uint16_t      RESERVED11;  /*!< Reserved, 0x46                                            */
  __VO uint16_t DCR;         /*!< TIM DMA control register,            Address offset: 0x48 */
  uint16_t      RESERVED12;  /*!< Reserved, 0x4A                                            */
  __VO uint16_t DMAR;        /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
  uint16_t      RESERVED13;  /*!< Reserved, 0x4E                                            */
  __VO uint16_t OR;          /*!< TIM option register,                 Address offset: 0x50 */
  uint16_t      RESERVED14;  /*!< Reserved, 0x52                                            */
} TIM_TypeDef;

/**
  * @brief System configuration controller
  */

typedef struct
{
  __VO uint32_t MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  __VO uint32_t PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
  __VO uint32_t EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  uint32_t      RESERVED[2];  /*!< Reserved, 0x18-0x1C                                                          */
  __VO uint32_t CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_TypeDef;



/**
  * @brief SysTick
  */

typedef struct
{
  __VO uint32_t STCSR;       /*!< SysTick Control and Status Register,                      Address offset: 0x00      */
  __VO uint32_t STRVR;       /*!< SysTick Reload Value Register,     Address offset: 0x04      */
  __VO uint32_t STCVR;   	 /*!< SysTick Current Value Register, Address offset: 0x08-0x14 */
  __VO uint32_t STCR;  		 /*!< SysTick Calibre Register, 0x18-0x1C                                                          */
} SysTick_TypeDef;



/**
 * @brief STM32F4XX Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
typedef enum IRQn
{

/******  STM32 specific Interrupt Numbers **********************************************************************/

  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts  					      			*/
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SPI5_IRQn                   = 85,     /*!< SPI5 global Interrupt                                             */
  SPI6_IRQn                   = 86      /*!< SPI6 global Interrupt                                             */
} IRQn_Type;


/**
  * @brief External Interrupt/Event Controller
  */

typedef struct
{
  __VO uint32_t IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
  __VO uint32_t EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
  __VO uint32_t RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
  __VO uint32_t FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
  __VO uint32_t SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
  __VO uint32_t PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_TypeDef;


/** @addtogroup Peripheral_memory_map
  * @{
  */

#define FLASH_BASE            ((uint32_t)0x08000000) /*!< FLASH(up to 1 MB) base address in the alias region                         */
#define CCMDATARAM_BASE       ((uint32_t)0x10000000) /*!< CCM(core coupled memory) data RAM(64 KB) base address in the alias region  */
#define SRAM1_BASE            ((uint32_t)0x20000000) /*!< SRAM1(112 KB) base address in the alias region                             */
#define SRAM2_BASE            ((uint32_t)0x2001C000) /*!< SRAM2(16 KB) base address in the alias region                              */
#define SRAM3_BASE            ((uint32_t)0x20020000) /*!< SRAM3(64 KB) base address in the alias region                              */
#define PERIPH_BASE           ((uint32_t)0x40000000) /*!< Peripheral base address in the alias region                                */
#define BKPSRAM_BASE          ((uint32_t)0x40024000) /*!< Backup SRAM(4 KB) base address in the alias region                         */
#define FMC_R_BASE            ((uint32_t)0xA0000000) /*!< FMC registers base address                                                 */

#define CCMDATARAM_BB_BASE    ((uint32_t)0x12000000) /*!< CCM(core coupled memory) data RAM(64 KB) base address in the bit-band region  */
#define SRAM1_BB_BASE         ((uint32_t)0x22000000) /*!< SRAM1(112 KB) base address in the bit-band region                             */
#define SRAM2_BB_BASE         ((uint32_t)0x2201C000) /*!< SRAM2(16 KB) base address in the bit-band region                              */
#define SRAM3_BB_BASE         ((uint32_t)0x22020000) /*!< SRAM3(64 KB) base address in the bit-band region                              */
#define PERIPH_BB_BASE        ((uint32_t)0x42000000) /*!< Peripheral base address in the bit-band region                                */
#define BKPSRAM_BB_BASE       ((uint32_t)0x42024000) /*!< Backup SRAM(4 KB) base address in the bit-band region                         */



/*
 * ARM Cortex M4 NVIC_ISERx registers :
 */

#define NVIC_ISER0			  ((__VO uint32_t*) 0xE000E100)
#define NVIC_ISER1			  ((__VO uint32_t*) 0xE000E104)
#define NVIC_ISER2			  ((__VO uint32_t*) 0xE000E108)
#define NVIC_ISER3			  ((__VO uint32_t*) 0xE000E10C)
#define NVIC_ISER4			  ((__VO uint32_t*) 0xE000E110)
#define NVIC_ISER5			  ((__VO uint32_t*) 0xE000E114)
#define NVIC_ISER6			  ((__VO uint32_t*) 0xE000E118)
#define NVIC_ISER7			  ((__VO uint32_t*) 0xE000E11C)

/*
 * ARM Cortex M4 NVIC_ICERx registers :
 */

#define NVIC_ICER0			  ((__VO uint32_t*) 0xE000E180)
#define NVIC_ICER1			  ((__VO uint32_t*) 0xE000E184)
#define NVIC_ICER2			  ((__VO uint32_t*) 0xE000E188)
#define NVIC_ICER3			  ((__VO uint32_t*) 0xE000E18C)
#define NVIC_ICER4			  ((__VO uint32_t*) 0xE000E190)
#define NVIC_ICER5			  ((__VO uint32_t*) 0xE000E194)
#define NVIC_ICER6			  ((__VO uint32_t*) 0xE000E198)
#define NVIC_ICER7			  ((__VO uint32_t*) 0xE000E19C)

/*
 * ARM Cortex Priority configuration registers
 */

#define NVIC_IPR			  ((__VO uint32_t*) 0xE000E400)

/*
 * ARM Cortex M4 Systick registers
 */
#define SysTick_BASE			0xE000E010


/*
 * ARM Cortex Priority number of bits implemented
 */

#define NO_BITS_IMPLEMENTED		4

/* Legacy defines */
#define SRAM_BASE             SRAM1_BASE
#define SRAM_BB_BASE          SRAM1_BB_BASE


/*!< Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000)

/*!< APB1 peripherals */
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800)
#define UART4_BASE            (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASE            (APB1PERIPH_BASE + 0x5000)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASE             (APB1PERIPH_BASE + 0x5C00)
#define UART7_BASE            (APB1PERIPH_BASE + 0x7800)
#define UART8_BASE            (APB1PERIPH_BASE + 0x7C00)
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x1000)
#define TIM7_BASE             (APB1PERIPH_BASE + 0x1400)
#define TIM12_BASE            (APB1PERIPH_BASE + 0x1800)
#define TIM13_BASE            (APB1PERIPH_BASE + 0x1C00)
#define TIM14_BASE            (APB1PERIPH_BASE + 0x2000)

/*!< APB2 peripherals */
#define USART1_BASE           (APB2PERIPH_BASE + 0x1000)
#define USART6_BASE           (APB2PERIPH_BASE + 0x1400)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000)
#define SPI4_BASE             (APB2PERIPH_BASE + 0x3400)
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00)
#define SPI5_BASE             (APB2PERIPH_BASE + 0x5000)
#define SPI6_BASE             (APB2PERIPH_BASE + 0x5400)
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000)
#define TIM8_BASE             (APB2PERIPH_BASE + 0x0400)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x4400)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x4800)

/*!< AHB1 peripherals */

#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800)

#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASE            (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASE            (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASE            (AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASE            (AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASE            (AHB1PERIPH_BASE + 0x2800)



/**
  * @}
  */

/** @addtogroup Peripheral_declaration
  * @{
  */
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                ((SPI_TypeDef *) SPI3_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define USART3              ((USART_TypeDef *) USART3_BASE)
#define UART4               ((USART_TypeDef *) UART4_BASE)
#define UART5               ((USART_TypeDef *) UART5_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define I2C3                ((I2C_TypeDef *) I2C3_BASE)
#define UART7               ((USART_TypeDef *) UART7_BASE)
#define UART8               ((USART_TypeDef *) UART8_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define USART6              ((USART_TypeDef *) USART6_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define SPI4                ((SPI_TypeDef *) SPI4_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)

#define SPI5                ((SPI_TypeDef *) SPI5_BASE)
#define SPI6                ((SPI_TypeDef *) SPI6_BASE)


#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
#define GPIOI               ((GPIO_TypeDef *) GPIOI_BASE)
#define GPIOJ               ((GPIO_TypeDef *) GPIOJ_BASE)
#define GPIOK               ((GPIO_TypeDef *) GPIOK_BASE)

#define RCC                 ((RCC_TypeDef *) RCC_BASE)

#define SysTick				((SysTick_TypeDef *) SysTick_BASE)

#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define TIM5                ((TIM_TypeDef *) TIM5_BASE)
#define TIM6                ((TIM_TypeDef *) TIM6_BASE)
#define TIM7                ((TIM_TypeDef *) TIM7_BASE)
#define TIM12               ((TIM_TypeDef *) TIM12_BASE)
#define TIM13               ((TIM_TypeDef *) TIM13_BASE)
#define TIM14               ((TIM_TypeDef *) TIM14_BASE)
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define TIM8                ((TIM_TypeDef *) TIM8_BASE)
#define TIM9                ((TIM_TypeDef *) TIM9_BASE)
#define TIM10               ((TIM_TypeDef *) TIM10_BASE)
#define TIM11               ((TIM_TypeDef *) TIM11_BASE)

/*
 * GPIOx Clock Enable and Disable
 */

#define GPIOA_CLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_CLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_CLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_CLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_CLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_CLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_CLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_CLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_CLK_EN()		(RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_CLK_EN()		(RCC->AHB1ENR |= (1 << 9))
#define GPIOK_CLK_EN()		(RCC->AHB1ENR |= (1 << 10))

#define GPIOA_CLK_DIS()		do{(RCC->AHB1RSTR) |= (1 << 0);(RCC->AHB1RSTR) &= ~(1 << 0);}while(0)
#define GPIOB_CLK_DIS()		do{(RCC->AHB1RSTR) |= (1 << 1);(RCC->AHB1RSTR) &= ~(1 << 1);}while(0)
#define GPIOC_CLK_DIS()		do{(RCC->AHB1RSTR) |= (1 << 2);(RCC->AHB1RSTR) &= ~(1 << 2);}while(0)
#define GPIOD_CLK_DIS()		do{(RCC->AHB1RSTR) |= (1 << 3);(RCC->AHB1RSTR) &= ~(1 << 3);}while(0)
#define GPIOE_CLK_DIS()		do{(RCC->AHB1RSTR) |= (1 << 4);(RCC->AHB1RSTR) &= ~(1 << 4);}while(0)
#define GPIOF_CLK_DIS()		do{(RCC->AHB1RSTR) |= (1 << 5);(RCC->AHB1RSTR) &= ~(1 << 5);}while(0)
#define GPIOG_CLK_DIS()		do{(RCC->AHB1RSTR) |= (1 << 6);(RCC->AHB1RSTR) &= ~(1 << 6);}while(0)
#define GPIOH_CLK_DIS()		do{(RCC->AHB1RSTR) |= (1 << 7);(RCC->AHB1RSTR) &= ~(1 << 7);}while(0)
#define GPIOI_CLK_DIS()		do{(RCC->AHB1RSTR) |= (1 << 8);(RCC->AHB1RSTR) &= ~(1 << 8);}while(0)
#define GPIOJ_CLK_DIS()		do{(RCC->AHB1RSTR) |= (1 << 9);(RCC->AHB1RSTR) &= ~(1 << 9);}while(0)
#define GPIOK_CLK_DIS()		do{(RCC->AHB1RSTR) |= (1 << 10);(RCC->AHB1RSTR) &= ~(1 << 10);}while(0)

/*
 * SPIx Clock Enable and Disable
 */

#define SPI1_CLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_CLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_CLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_CLK_EN()		(RCC->APB2ENR |= (1 << 13))
#define SPI5_CLK_EN()		(RCC->APB2ENR |= (1 << 20))
#define SPI6_CLK_EN()		(RCC->APB2ENR |= (1 << 21))

#define SPI1_CLK_DIS()		do{(RCC->APB2RSTR) |= (1 << 12);(RCC->APB2RSTR) &= ~(1 << 12);}while(0)
#define SPI2_CLK_DIS()		do{(RCC->APB1RSTR) |= (1 << 14);(RCC->APB1RSTR) &= ~(1 << 14);}while(0)
#define SPI3_CLK_DIS()		do{(RCC->APB1RSTR) |= (1 << 15);(RCC->APB1RSTR) &= ~(1 << 15);}while(0)
#define SPI4_CLK_DIS()		do{(RCC->APB2RSTR) |= (1 << 13);(RCC->APB2RSTR) &= ~(1 << 13);}while(0)
#define SPI5_CLK_DIS()		do{(RCC->APB2RSTR) |= (1 << 20);(RCC->APB2RSTR) &= ~(1 << 20);}while(0)
#define SPI6_CLK_DIS()		do{(RCC->APB2RSTR) |= (1 << 21);(RCC->APB2RSTR) &= ~(1 << 21);}while(0)

/*
 * return GPIO PORT Number :
 */

#define GPIO_Port_Number(x)	(x == GPIOA) ? 0 : \
							(x == GPIOB) ? 1 : \
							(x == GPIOC) ? 2 : \
							(x == GPIOD) ? 3 : \
							(x == GPIOE) ? 4 : \
							(x == GPIOF) ? 5 : \
							(x == GPIOG) ? 6 : \
							(x == GPIOH) ? 7 : \
							(x == GPIOI) ? 8 : \
							(x == GPIOJ) ? 9 : 10

/*
 * UART Peripheral Clock Enable
 */

#define USART1_CLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define USART2_CLK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART3_CLK_EN()		(RCC->APB1ENR |= (1 << 18))
#define UART4_CLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_CLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_CLK_EN()		(RCC->APB2ENR |= (1 << 5))
#define UART7_CLK_EN()		(RCC->APB1ENR |= (1 << 30))
#define UART8_CLK_EN()		(RCC->APB1ENR |= (1 << 31))

/*
 * SPI Peripheral Clock Enable
 */

#define SPI1_CLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_CLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_CLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_CLK_EN()		(RCC->APB2ENR |= (1 << 13))
#define SPI5_CLK_EN()		(RCC->APB2ENR |= (1 << 20))
#define SPI6_CLK_EN()		(RCC->APB2ENR |= (1 << 21))

/*
 * I2C Peripheral Clock Enable
 */

#define I2C1_CLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_CLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_CLK_EN()		(RCC->APB1ENR |= (1 << 23))

#define I2C1_CLK_DIS()		do{(RCC->APB1RSTR) |= (1 << 21);(RCC->APB2RSTR) &= ~(1 << 21);}while(0)
#define I2C2_CLK_DIS()		do{(RCC->APB1RSTR) |= (1 << 22);(RCC->APB1RSTR) &= ~(1 << 22);}while(0)
#define I2C3_CLK_DIS()		do{(RCC->APB1RSTR) |= (1 << 23);(RCC->APB1RSTR) &= ~(1 << 23);}while(0)

/*
 * CFGR Peripheral Clock Enable
 */
#define SYSCFGR_CLK_EN()		(RCC->APB2ENR |= (1 << 14))



#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_SET			SET
#define FLAG_RESET			RESET

#endif /* STM32F4XX_H_ */

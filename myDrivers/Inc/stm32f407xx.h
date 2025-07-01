/*
 * stm32f407xx.h
 *
 *  Created on: Apr 5, 2025
 *      Author: erhan
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __IO volatile
#define UNUSED(X) (void)X
#define SET_BIT(REG, BIT)	( (REG) |= 	(BIT) )
#define CLEAR_BIT(REG, BIT) ( (REG) &= ~(BIT) )
#define READ_BIT(REG, BIT)	( (REG) & 	(BIT) )

/*
 * Memory Base Addresses
 *
 */
#define FLASH_BASE_ADDR 	0x08000000UL /*!< FLASH Base Addres (up to 1 MB) 	*/
#define SRAM1_BASE_ADDR 	0x20000000UL /*!< SRAM1 Base Address 112 KB 		*/
#define SRAM2_BASE_ADDR		0x2001C000UL /*!< SRAM2 Base Address 16 KB 			*/

/*
 * Peripheral Base Addresses
 *
 */
#define PERIPH_BASE_ADDR 	(0x40000000UL)						/*!< Base Address for all peripherals 				*/
#define APB1_BASE_ADDR		(PERIPH_BASE_ADDR + 0x00000000UL) 	/*!< APB1 Bus Domain Base Address 					*/
#define APB2_BASE_ADDR		(PERIPH_BASE_ADDR + 0x00010000UL)	/*!< APB2 Bus Domain Base Address					*/
#define AHB1_BASE_ADDR		(PERIPH_BASE_ADDR + 0x00020000UL)	/*!< AHB1 Bus Domain Base Address 					*/
#define AHB2_BASE_ADDR		(PERIPH_BASE_ADDR + 0x10000000UL) 	/*!< AHB2 Bus Domain Base Address 					*/

/*
 * APB1 Peripherals Base Addresses
 *
 */
#define TIM2_BASE			(APB1_BASE_ADDR + 0x0000UL)			/*!< TIM2 Base Address 					*/
#define TIM3_BASE			(APB1_BASE_ADDR + 0x0400UL)			/*!< TIM3 Base Address 					*/
#define TIM4_BASE			(APB1_BASE_ADDR + 0x0800UL)			/*!< TIM4 Base Address 					*/
#define TIM5_BASE			(APB1_BASE_ADDR + 0x0C00UL)			/*!< TIM5 Base Address 					*/
#define TIM6_BASE			(APB1_BASE_ADDR + 0x1000UL)			/*!< TIM6 Base Address 					*/
#define TIM7_BASE			(APB1_BASE_ADDR + 0x1400UL)			/*!< TIM7 Base Address 					*/
#define TIM12_BASE			(APB1_BASE_ADDR + 0x1800UL)			/*!< TIM12 Base Address 				*/
#define TIM13_BASE			(APB1_BASE_ADDR + 0x1C00UL)			/*!< TIM13 Base Address 				*/
#define TIM14_BASE			(APB1_BASE_ADDR + 0x2000UL)			/*!< TIM14 Base Address 				*/
#define RTC_BKP_BASE		(APB1_BASE_ADDR + 0x2C00UL)			/*!< RTC & BKP Registers Base Address 	*/
#define WWDG_BASE			(APB1_BASE_ADDR + 0x2C00UL)			/*!< WWDG Base Address 					*/
#define IWDG_BASE			(APB1_BASE_ADDR + 0x3000UL)			/*!< IWDG Base Address 					*/
#define I2S2_EXT_BASE		(APB1_BASE_ADDR + 0x3400UL)			/*!< I2S2ext Base Address  				*/
#define SPI2_BASE			(APB1_BASE_ADDR + 0x3800UL)			/*!< SPI2 Base Address					*/
#define SPI3_BASE			(APB1_BASE_ADDR + 0x3C00UL)			/*!< SPI3 Base Address 					*/
#define I2S3_EXT_BASE		(APB1_BASE_ADDR + 0x4000UL)			/*!< I2S3ext Base Address 				*/
#define USART2_BASE			(APB1_BASE_ADDR + 0x4400UL)			/*!< USART2 Base Address 				*/
#define USART3_BASE			(APB1_BASE_ADDR + 0x4800UL)			/*!< USART3 Base Address 				*/
#define UART4_BASE			(APB1_BASE_ADDR + 0x4C00UL)			/*!< UART4 Base Address 				*/
#define UART5_BASE			(APB1_BASE_ADDR + 0x5000UL)			/*!< UART5 Base Address 				*/
#define I2C1_BASE			(APB1_BASE_ADDR + 0x5400UL)			/*!< I2C1 Base Address 					*/
#define I2C2_BASE			(APB1_BASE_ADDR + 0x5800UL)			/*!< I2C2 Base Address 					*/
#define I2C3_BASE			(APB1_BASE_ADDR + 0x5C00UL)			/*!< I2C3 Base Address 					*/
#define CAN1_BASE			(APB1_BASE_ADDR + 0x6400UL)			/*!< CAN1 Base Address 					*/
#define CAN2_BASE			(APB1_BASE_ADDR + 0x6800UL)			/*!< CAN2 Base Address 					*/
#define PWR_BASE			(APB1_BASE_ADDR + 0x7000UL)			/*!< PWR Base Address 					*/
#define DAC_BASE			(APB1_BASE_ADDR + 0x7400UL)			/*!< DAC Base Address 					*/
#define UART7_BASE			(APB1_BASE_ADDR + 0x7800UL)			/*!< UART7 Base Address 				*/
#define UART8_BASE			(APB1_BASE_ADDR + 0x7C00UL)			/*!< UART8 Base Address 				*/

/*
 * APB2 Peripherals Base Addresses
 *
 */
#define TIM1_BASE			(APB2_BASE_ADDR + 0x0000UL)			/*!< TIM1 Base Address 		*/
#define TIM8_BASE			(APB2_BASE_ADDR + 0x0400UL)			/*!< TIM8 Base Address 		*/
#define	USART1_BASE			(APB2_BASE_ADDR + 0x1000UL)			/*!< USART1 Base Address 	*/
#define	USART6_BASE			(APB2_BASE_ADDR + 0x1400UL)			/*!< USART6 Base Address 	*/
#define	ADC1_BASE			(APB2_BASE_ADDR + 0x2000UL)			/*!< ADC1 Base Address 		*/
#define ADC2_BASE			(APB2_BASE_ADDR + 0x2100UL)			/*!< ADC2 Base Address 		*/
#define ADC3_BASE			(APB2_BASE_ADDR + 0x2200UL)			/*!< ADC3 Base Address 		*/
#define SDIO_BASE			(APB2_BASE_ADDR + 0x2C00UL)			/*!< SDIO Base Address 		*/
#define SPI1_BASE			(APB2_BASE_ADDR + 0x3000UL)			/*!< SPI1 Base Address 		*/
#define SPI4_BASE			(APB2_BASE_ADDR + 0x3400UL)			/*!< SPI4 Base Address 		*/
#define SYSCFG_BASE			(APB2_BASE_ADDR + 0x3800UL)			/*!< SYSCFG Base Address 	*/
#define EXTI_BASE			(APB2_BASE_ADDR + 0x3C00UL)			/*!< EXTI Base Address 		*/
#define TIM9_BASE			(APB2_BASE_ADDR + 0x4000UL)			/*!< TIM9 Base Address 		*/
#define TIM10_BASE			(APB2_BASE_ADDR + 0x4400UL)			/*!< TIM10 Base Address 	*/
#define TIM11_BASE			(APB2_BASE_ADDR + 0x4800UL)			/*!< TIM11 Base Address 	*/
#define SPI5_BASE			(APB2_BASE_ADDR + 0x5000UL)			/*!< SPI5 Base Address 		*/
#define SPI6_BASE			(APB2_BASE_ADDR + 0x5400UL)			/*!< SPI6 Base Address 		*/
#define SAI1_BASE			(APB2_BASE_ADDR + 0x5800UL)			/*!< SAI1 Base Address 		*/
#define LCD_TFT_BASE		(APB2_BASE_ADDR + 0x6800UL)			/*!< LCD-TFT Base Address 	*/

/*
 * AHB1 Peripherals Base Addresses
 *
 */
#define GPIOA_BASE			(AHB1_BASE_ADDR + 0x0000UL)			/*!< GPIOA Base Address 					*/
#define GPIOB_BASE			(AHB1_BASE_ADDR + 0x0400UL)			/*!< GPIOB Base Address 					*/
#define GPIOC_BASE			(AHB1_BASE_ADDR + 0x0800UL)			/*!< GPIOC Base Address  					*/
#define GPIOD_BASE			(AHB1_BASE_ADDR + 0x0C00UL)			/*!< GPIOD Base Address  					*/
#define GPIOE_BASE			(AHB1_BASE_ADDR + 0x1000UL)			/*!< GPIOE Base Address  					*/
#define GPIOF_BASE			(AHB1_BASE_ADDR + 0x1400UL)			/*!< GPIOF Base Address  					*/
#define GPIOG_BASE			(AHB1_BASE_ADDR + 0x1800UL)			/*!< GPIOG Base Address 					*/
#define GPIOH_BASE			(AHB1_BASE_ADDR + 0x1C00UL)			/*!< GPIOH Base Address  					*/
#define GPIOI_BASE			(AHB1_BASE_ADDR + 0x2000UL)			/*!< GPIOI Base Address  					*/
#define GPIOJ_BASE			(AHB1_BASE_ADDR + 0x2400UL)			/*!< GPIOJ Base Address  					*/
#define GPIOK_BASE			(AHB1_BASE_ADDR + 0x2800UL)			/*!< GPIOK Base Address  					*/
#define CRC_BASE			(AHB1_BASE_ADDR + 0x3000UL)			/*!< CRC Base Address 						*/
#define RCC_BASE			(AHB1_BASE_ADDR + 0x3800UL)			/*!< RCC Base Address 						*/
#define FLASH_IR_BASE		(AHB1_BASE_ADDR + 0x3C00UL)			/*!< Flash Interface Registers Base Address */
#define BKPSRAM_BASE		(AHB1_BASE_ADDR + 0x4000UL)			/*!< BKPSRAM Base Address 					*/
#define DMA1_BASE			(AHB1_BASE_ADDR + 0x6000UL)			/*!< DMA1 Base Address 						*/
#define DMA2_BASE			(AHB1_BASE_ADDR + 0x6400UL)			/*!< DMA2 Base Address 						*/
#define ETH_MAC_BASE		(AHB1_BASE_ADDR + 0x8000UL)			/*!< Ethernet MAC Base Address 				*/
#define DMA2D_BASE			(AHB1_BASE_ADDR + 0xB000UL)			/*!< DMA2D Base Address 					*/

/*
 * USB OTG HS & USB OTG FS Base Addresses
 *
 */
#define USBOTG_HS_BASE		(0x40040000UL)			/*!< USB OTG HS Base Address */
#define USBOTG_FS_BASE		(0x50000000UL)			/*!< USB OTG FS Base Address */

/*
 * AHB2 Peripherals Base Addresses
 *
 */
#define DCMI_BASE			(AHB2_BASE_ADDR + 0x00050000UL)			/*!< DCMI Base Address 	*/
#define RNG_BASE			(AHB2_BASE_ADDR + 0x00060800UL)			/*!< RNG Base Address 	*/


/*!< General Purpose I/O */
typedef struct
{
	__IO uint32_t MODER; 			/*!< GPIO port mode register 					address offset = 0x00 		*/
	__IO uint32_t OTYPER;			/*!< GPIO port output type register 			address offset = 0x04 		*/
	__IO uint32_t OSPEEDR;			/*!< GPIO port output speed register 			address offset = 0x08 		*/
	__IO uint32_t PUPDR;			/*!< GPIO port pull-up/pull-down register 		address offset = 0x0C 		*/
	__IO uint32_t IDR;				/*!< GPIO port input data register 				address offset = 0x10 		*/
	__IO uint32_t ODR;				/*!< GPIO port output data register 			address offset = 0x14 		*/
	__IO uint32_t BSRR;				/*!< GPIO port bit set/reset register 			address offset = 0x18  		*/
	__IO uint32_t LCKR;				/*!< GPIO port configuration lock register 		address offset = 0x1C 		*/
	__IO uint32_t AFR[2];			/*!< GPIO alternate function low/high register 	address offset = 0x20/0x24 	*/
}GPIO_TypeDef;

/*!< Reset and Clock Control */
typedef struct
{
	__IO uint32_t CR;				/*!< RCC clock control register 											Address offset: 0x00		*/
	__IO uint32_t PLLCFGR;			/*!< RCC configuration register 											Address offset: 0x04		*/
	__IO uint32_t CFGR;				/*!< RCC clock configuration register 										Address offset: 0x08		*/
	__IO uint32_t CIR;				/*!< RCC clock interrupt register 											Address offset: 0x0C		*/
	__IO uint32_t AHB1RSTR;			/*!< RCC AHB1 peripheral reset register 									Address offset: 0x10		*/
	__IO uint32_t AHB2RSTR;			/*!< RCC AHB2 peripheral reset register 									Address offset: 0x14		*/
	__IO uint32_t AHB3RSTR;			/*!< RCC AHB3 peripheral reset register 									Address offset: 0x18		*/
	uint32_t 	  RESERVED0;		/*!< RESERVED 																Address offset: 0x1C		*/
	__IO uint32_t APB1RSTR;			/*!< RCC APB1 peripheral reset register 									Address offset: 0x20		*/
	__IO uint32_t APB2RSTR;			/*!< RCC APB2 peripheral reset register 									Address offset: 0x24		*/
	uint32_t 	  RESERVED1[2];		/*!< RESERVED 																Address offset: 0x28		*/
	__IO uint32_t AHB1ENR;			/*!< RCC AHB1 peripheral enable register 									Address offset: 0x30		*/
	__IO uint32_t AHB2ENR;			/*!< RCC AHB2 peripheral enable register 									Address offset: 0x34		*/
	__IO uint32_t AHB3ENR;			/*!< RCC AHB3 peripheral enable register 									Address offset: 0x38		*/
	uint32_t 	  RESERVED2;		/*!< RESERVED 																Address offset: 0x3C		*/
	__IO uint32_t APB1ENR;			/*!< RCC APB1 peripheral enable register 									Address offset: 0x40		*/
	__IO uint32_t APB2ENR;			/*!< RCC APB2 peripheral enable register 									Address offset: 0x44		*/
	uint32_t 	  RESERVED3[2];		/*!< RESERVED 																Address offset: 0x48		*/
	__IO uint32_t AHB1LPENR;		/*!< RCC AHB1 peripheral clock enable in low power mode register register 	Address offset: 0x50		*/
	__IO uint32_t AHB2LPENR;		/*!< RCC AHB2 peripheral clock enable in low power mode register register 	Address offset: 0x54		*/
	__IO uint32_t AHB3LPENR;		/*!< RCC AHB3 peripheral clock enable in low power mode register register 	Address offset: 0x58		*/
	uint32_t 	  RESERVED4;		/*!< RESERVED 																Address offset: 0x5C		*/
	__IO uint32_t APB1LPENR;		/*!< RCC APB1 peripheral clock enable in low power mode register register 	Address offset: 0x60		*/
	__IO uint32_t APB2LPENR;		/*!< RCC APB2 peripheral clock enable in low power mode register register 	Address offset: 0x64		*/
	uint32_t 	  RESERVED5[2];		/*!< RESERVED 																Address offset: 0x68		*/
	__IO uint32_t BDCR;				/*!< RCC Backup domain control register 									Address offset: 0x70		*/
	__IO uint32_t CSR;				/*!< RCC clock control & status register 									Address offset: 0x74		*/
	uint32_t 	  RESERVED6[2];		/*!< RESERVED 																Address offset: 0x78		*/
	__IO uint32_t SSCGR;			/*!< RCC spread spectrum clock generation register 							Address offset: 0x80		*/
	__IO uint32_t PLLI2SCFGR;		/*!< RCC PLLI2S configuration register 										Address offset: 0x84		*/
}RCC_TypeDef;

/*
 * Peripheral Definitions
 *
 */

#define GPIOA		( (GPIO_TypeDef *)(GPIOA_BASE) )
#define GPIOB		( (GPIO_TypeDef *)(GPIOB_BASE) )
#define GPIOC		( (GPIO_TypeDef *)(GPIOC_BASE) )
#define GPIOD		( (GPIO_TypeDef *)(GPIOD_BASE) )
#define GPIOE		( (GPIO_TypeDef *)(GPIOE_BASE) )
#define GPIOF		( (GPIO_TypeDef *)(GPIOF_BASE) )
#define GPIOG		( (GPIO_TypeDef *)(GPIOG_BASE) )
#define GPIOH		( (GPIO_TypeDef *)(GPIOH_BASE) )
#define GPIOI		( (GPIO_TypeDef *)(GPIOI_BASE) )
#define GPIOJ		( (GPIO_TypeDef *)(GPIOJ_BASE) )
#define GPIOK		( (GPIO_TypeDef *)(GPIOK_BASE) )
#define RCC			( (RCC_TypeDef *)(RCC_BASE) )

/*
 * RCC Bit Definitions
 *
 */

#define RCC_AHB1ENR_GPIOAEN_Pos			(0U)								// RCC AHB1ENR register GPIOAEN Bit Position
#define RCC_AHB1ENR_GPIOAEN_Mask 		(0x1 << RCC_AHB1ENR_GPIOAEN_Pos)	// RCC AHB1ENR register GPIOAEN Bit Mask
#define RCC_AHB1ENR_GPIOAEN				RCC_AHB1ENR_GPIOAEN_Mask			// RCC AHB1ENR register GPIOAEN Macro

#define RCC_AHB1ENR_GPIOBEN_Pos			(1U)								// RCC AHB1ENR register GPIOBEN Bit Position
#define RCC_AHB1ENR_GPIOBEN_Mask 		(0x1 << RCC_AHB1ENR_GPIOBEN_Pos)	// RCC AHB1ENR register GPIOBEN Bit Mask
#define RCC_AHB1ENR_GPIOBEN				RCC_AHB1ENR_GPIOBEN_Mask			// RCC AHB1ENR register GPIOBEN Macro

#define RCC_AHB1ENR_GPIOCEN_Pos			(2U)								// RCC AHB1ENR register GPIOCEN Bit Position
#define RCC_AHB1ENR_GPIOCEN_Mask 		(0x1 << RCC_AHB1ENR_GPIOCEN_Pos)	// RCC AHB1ENR register GPIOCEN Bit Mask
#define RCC_AHB1ENR_GPIOCEN				RCC_AHB1ENR_GPIOCEN_Mask			// RCC AHB1ENR register GPIOCEN Macro

#define RCC_AHB1ENR_GPIODEN_Pos			(3U)								// RCC AHB1ENR register GPIODEN Bit Position
#define RCC_AHB1ENR_GPIODEN_Mask 		(0x1 << RCC_AHB1ENR_GPIODEN_Pos)	// RCC AHB1ENR register GPIODEN Bit Mask
#define RCC_AHB1ENR_GPIODEN				RCC_AHB1ENR_GPIODEN_Mask			// RCC AHB1ENR register GPIODEN Macro

#define RCC_AHB1ENR_GPIOEEN_Pos			(4U)								// RCC AHB1ENR register GPIOEEN Bit Position
#define RCC_AHB1ENR_GPIOEEN_Mask 		(0x1 << RCC_AHB1ENR_GPIOEEN_Pos)	// RCC AHB1ENR register GPIOEEN Bit Mask
#define RCC_AHB1ENR_GPIOEEN				RCC_AHB1ENR_GPIOEEN_Mask			// RCC AHB1ENR register GPIOEEN Macro

#define RCC_AHB1ENR_GPIOFEN_Pos			(5U)								// RCC AHB1ENR register GPIOFEN Bit Position
#define RCC_AHB1ENR_GPIOFEN_Mask 		(0x1 << RCC_AHB1ENR_GPIOFEN_Pos)	// RCC AHB1ENR register GPIOFEN Bit Mask
#define RCC_AHB1ENR_GPIOFEN				RCC_AHB1ENR_GPIOFEN_Mask			// RCC AHB1ENR register GPIOFEN Macro

#define RCC_AHB1ENR_GPIOGEN_Pos			(6U)								// RCC AHB1ENR register GPIOGEN Bit Position
#define RCC_AHB1ENR_GPIOGEN_Mask 		(0x1 << RCC_AHB1ENR_GPIOGEN_Pos)	// RCC AHB1ENR register GPIOGEN Bit Mask
#define RCC_AHB1ENR_GPIOGEN				RCC_AHB1ENR_GPIOGEN_Mask			// RCC AHB1ENR register GPIOGEN Macro

#define RCC_AHB1ENR_GPIOHEN_Pos			(7U)								// RCC AHB1ENR register GPIOHEN Bit Position
#define RCC_AHB1ENR_GPIOHEN_Mask 		(0x1 << RCC_AHB1ENR_GPIOHEN_Pos)	// RCC AHB1ENR register GPIOHEN Bit Mask
#define RCC_AHB1ENR_GPIOHEN				RCC_AHB1ENR_GPIOHEN_Mask			// RCC AHB1ENR register GPIOHEN Macro

#define RCC_AHB1ENR_GPIOIEN_Pos			(8U)								// RCC AHB1ENR register GPIOIEN Bit Position
#define RCC_AHB1ENR_GPIOIEN_Mask 		(0x1 << RCC_AHB1ENR_GPIOIEN_Pos)	// RCC AHB1ENR register GPIOIEN Bit Mask
#define RCC_AHB1ENR_GPIOIEN				RCC_AHB1ENR_GPIOIEN_Mask			// RCC AHB1ENR register GPIOIEN Macro

#define RCC_AHB1ENR_GPIOJEN_Pos			(9U)								// RCC AHB1ENR register GPIOJEN Bit Position
#define RCC_AHB1ENR_GPIOJEN_Mask 		(0x1 << RCC_AHB1ENR_GPIOJEN_Pos)	// RCC AHB1ENR register GPIOJEN Bit Mask
#define RCC_AHB1ENR_GPIOJEN				RCC_AHB1ENR_GPIOJEN_Mask			// RCC AHB1ENR register GPIOJEN Macro

#define RCC_AHB1ENR_GPIOKEN_Pos			(10U)								// RCC AHB1ENR register GPIOKEN Bit Position
#define RCC_AHB1ENR_GPIOKEN_Mask 		(0x1 << RCC_AHB1ENR_GPIOKEN_Pos)	// RCC AHB1ENR register GPIOKEN Bit Mask
#define RCC_AHB1ENR_GPIOKEN				RCC_AHB1ENR_GPIOKEN_Mask			// RCC AHB1ENR register GPIOKEN Macro

#include "RCC.h"
#include "GPIO.h"

#endif /* INC_STM32F407XX_H_ */

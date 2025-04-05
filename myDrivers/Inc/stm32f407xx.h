/*
 * stm32f407xx.h
 *
 *  Created on: Apr 5, 2025
 *      Author: erhan
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#define __IO volatile

/*
 * Memory Base Addresses
 *
 */
#define FLASH_BASE_ADDR 	0x08000000UL /*!< FLASH Base Addres (up to 1 MB) */
#define SRAM1_BASE_ADDR 	0x20000000UL /*!< SRAM1 Base Address 112 KB */
#define SRAM2_BASE_ADDR		0x2001C000UL /*!< SRAM2 Base Address 16 KB */

/*
 * Peripheral Base Addresses
 *
 */
#define PERIPH_BASE_ADDR 	(0x40000000UL)						/*!< Base Address for all peripherals 	*/
#define APB1_BASE_ADDR		(PERIPH_BASE_ADDR) 					/*!< APB1 Bus Domain Base Address 					*/
#define APB2_BASE_ADDR		(PERIPH_BASE_ADDR + 0x00010000UL)	/*!< APB2 Bus Domain Base Address					*/
#define AHB1_BASE_ADDR		(PERIPH_BASE_ADDR + 0x00020000UL)	/*!< AHB1 Bus Domain Base Address 					*/
#define AHB2_BASE_ADDR		(PERIPH_BASE_ADDR + 0x10000000UL) 	/*!< AHB2 Bus Domain Base Address 					*/

/*
 * APB1 Peripherals Base Addresses
 *
 */
#define TIM2_BASE			(APB1_BASE_ADDR)					/*!< TIM2 Base Address */
#define TIM3_BASE			(APB1_BASE_ADDR + 0x0400UL)			/*!< TIM3 Base Address */
#define TIM4_BASE			(APB1_BASE_ADDR + 0x0800UL)			/*!< TIM4 Base Address */
#define TIM5_BASE			(APB1_BASE_ADDR + 0x0C00UL)			/*!< TIM5 Base Address */
#define TIM6_BASE			(APB1_BASE_ADDR + 0x1000UL)			/*!< TIM6 Base Address */
#define TIM7_BASE			(APB1_BASE_ADDR + 0x1400UL)			/*!< TIM7 Base Address */
#define TIM12_BASE			(APB1_BASE_ADDR + 0x1800UL)			/*!< TIM12 Base Address */
#define TIM13_BASE			(APB1_BASE_ADDR + 0x1C00UL)			/*!< TIM13 Base Address */
#define TIM14_BASE			(APB1_BASE_ADDR + 0x2000UL)			/*!< TIM14 Base Address */

#define RTC_BKP_BASE		(APB1_BASE_ADDR + 0x2C00UL)			/*!< RTC & BKP Registers Base Address */

#define WWDG_BASE			(APB1_BASE_ADDR + 0x2C00UL)			/*!< WWDG Base Address */
#define IWDG_BASE			(APB1_BASE_ADDR + 0x3000UL)			/*!< IWDG Base Address */

#define I2S2_EXT_BASE		(APB1_BASE_ADDR + 0x3400UL)			/*!< I2S2ext Base Address  */
#define SPI2_BASE			(APB1_BASE_ADDR + 0x3800UL)			/*!< SPI2 Base Address */
#define SPI3_BASE			(APB1_BASE_ADDR + 0x3C00UL)			/*!< SPI3 Base Address */
#define I2S3_EXT_BASE		(APB1_BASE_ADDR + 0x4000UL)			/*!< I2S3ext Base Address */

#define USART2_BASE			(APB1_BASE_ADDR + 0x4400UL)			/*!< USART2 Base Address */
#define USART3_BASE			(APB1_BASE_ADDR + 0x4800UL)			/*!< USART3 Base Address */
#define UART4_BASE			(APB1_BASE_ADDR + 0x4C00UL)			/*!< UART4 Base Address */
#define UART5_BASE			(APB1_BASE_ADDR + 0x5000UL)			/*!< UART5 Base Address */

#define I2C1_BASE			(APB1_BASE_ADDR + 0x5400UL)			/*!< I2C1 Base Address */
#define I2C2_BASE			(APB1_BASE_ADDR + 0x5800UL)			/*!< I2C2 Base Address */
#define I2C3_BASE			(APB1_BASE_ADDR + 0x5C00UL)			/*!< I2C3 Base Address */

#define CAN1_BASE			(APB1_BASE_ADDR + 0x6400UL)			/*!< CAN1 Base Address */
#define CAN2_BASE			(APB1_BASE_ADDR + 0x6800UL)			/*!< CAN2 Base Address */

#define PWR_BASE			(APB1_BASE_ADDR + 0x7000UL)			/*!< PWR Base Address */
#define DAC_BASE			(APB1_BASE_ADDR + 0x7400UL)			/*!< DAC Base Address */

#define UART7_BASE			(APB1_BASE_ADDR + 0x7800UL)			/*!< UART7 Base Address */
#define UART8_BASE			(APB1_BASE_ADDR + 0x7C00UL)			/*!< UART8 Base Address */

/*
 * APB2 Peripherals Base Addresses
 *
 */
#define TIM1_BASE			(APB2_BASE_ADDR)					/*!< TIM1 Base Address */
#define TIM8_BASE			(APB2_BASE_ADDR + 0x0400UL)			/*!< TIM8 Base Address */

#define	USART1_BASE			(APB2_BASE_ADDR + 0x1000UL)			/*!< USART1 Base Address */
#define	USART6_BASE			(APB2_BASE_ADDR + 0x1400UL)			/*!< USART6 Base Address */

#define	ADC1_BASE			(APB2_BASE_ADDR + 0x2000UL)			/*!< ADC1 Base Address */
#define ADC2_BASE			(APB2_BASE_ADDR + 0x2100UL)			/*!< ADC2 Base Address */
#define ADC3_BASE			(APB2_BASE_ADDR + 0x2200UL)			/*!< ADC3 Base Address */

#define SDIO_BASE			(APB2_BASE_ADDR + 0x2C00UL)			/*!< SDIO Base Address */

#define SPI1_BASE			(APB2_BASE_ADDR + 0x3000UL)			/*!< SPI1 Base Address */
#define SPI4_BASE			(APB2_BASE_ADDR + 0x3400UL)			/*!< SPI4 Base Address */

#define SYSCFG_BASE			(APB2_BASE_ADDR + 0x3800UL)			/*!< SYSCFG Base Address */
#define EXTI_BASE			(APB2_BASE_ADDR + 0x3C00UL)			/*!< EXTI Base Address */

#define TIM9_BASE			(APB2_BASE_ADDR + 0x4000UL)			/*!< TIM9 Base Address */
#define TIM10_BASE			(APB2_BASE_ADDR + 0x4400UL)			/*!< TIM10 Base Address */
#define TIM11_BASE			(APB2_BASE_ADDR + 0x4800UL)			/*!< TIM11 Base Address */

#define SPI5_BASE			(APB2_BASE_ADDR + 0x5000UL)			/*!< SPI5 Base Address */
#define SPI6_BASE			(APB2_BASE_ADDR + 0x5400UL)			/*!< SPI6 Base Address */

#define SAI1_BASE			(APB2_BASE_ADDR + 0x5800UL)			/*!< SAI1 Base Address */
#define LCD_TFT_BASE		(APB2_BASE_ADDR + 0x6800UL)			/*!< LCD-TFT Base Address */

/*
 * AHB1 Peripherals Base Addresses
 *
 */
#define GPIOA_BASE			(AHB1_BASE_ADDR)					/*!< GPIOA Base Address */
#define GPIOB_BASE			(AHB1_BASE_ADDR + 0x0400UL)			/*!< GPIOB Base Address */
#define GPIOC_BASE			(AHB1_BASE_ADDR + 0x0800UL)			/*!< GPIOC Base Address */
#define GPIOD_BASE			(AHB1_BASE_ADDR + 0x0C00UL)			/*!< GPIOD Base Address */
#define GPIOE_BASE			(AHB1_BASE_ADDR + 0x1000UL)			/*!< GPIOE Base Address */
#define GPIOF_BASE			(AHB1_BASE_ADDR + 0x1400UL)			/*!< GPIOF Base Address */
#define GPIOG_BASE			(AHB1_BASE_ADDR + 0x1800UL)			/*!< GPIOG Base Address */
#define GPIOH_BASE			(AHB1_BASE_ADDR + 0x1C00UL)			/*!< GPIOH Base Address */
#define GPIOI_BASE			(AHB1_BASE_ADDR + 0x2000UL)			/*!< GPIOI Base Address */
#define GPIOJ_BASE			(AHB1_BASE_ADDR + 0x2400UL)			/*!< GPIOJ Base Address */
#define GPIOK_BASE			(AHB1_BASE_ADDR + 0x2800UL)			/*!< GPIOK Base Address */

#define CRC_BASE			(AHB1_BASE_ADDR + 0x3000UL)			/*!< CRC Base Address */

#define RCC_BASE			(AHB1_BASE_ADDR + 0x3800UL)			/*!< RCC Base Address */

#define FLASH_IR_BASE		(AHB1_BASE_ADDR + 0x3C00UL)			/*!< Flash Interface Registers Base Address */
#define BKPSRAM_BASE		(AHB1_BASE_ADDR + 0x4000UL)			/*!< BKPSRAM Base Address */
#define DMA1_BASE			(AHB1_BASE_ADDR + 0x6000UL)			/*!< DMA1 Base Address */
#define DMA2_BASE			(AHB1_BASE_ADDR + 0x6400UL)			/*!< DMA2 Base Address */
#define ETH_MAC_BASE		(AHB1_BASE_ADDR + 0x8000UL)			/*!< Ethernet MAC Base Address */
#define DMA2D_BASE			(AHB1_BASE_ADDR + 0xB000UL)			/*!< DMA2D Base Address */

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
#define DCMI_BASE			(AHB2_BASE_ADDR + 0x00050000UL)			/*!< DCMI Base Address */
#define RNG_BASE			(AHB2_BASE_ADDR + 0x00060800UL)			/*!< RNG Base Address */


/*!< General Purpose I/O */
typedef struct
{
	__IO uint32_t MODER;
	__IO uint32_t OTYPER;
	__IO uint32_t OSPEEDR;
	__IO uint32_t PUPDR;
	__IO uint32_t IDR;
	__IO uint32_t ODR;
	__IO uint32_t BSRR;
	__IO uint32_t LCKR;
	__IO uint32_t AFRL;
	__IO uint32_t AFRH;
}GPIO_TypeDef_t;

#endif /* INC_STM32F407XX_H_ */

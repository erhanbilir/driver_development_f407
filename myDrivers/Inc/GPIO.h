/*
 * GPIO.h
 *
 *  Created on: May 1, 2025
 *      Author: erhan
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "stm32f407xx.h"

/*
 * @def_group GPIO_PINS
 *
 */
#define GPIO_PIN_0			(uint16_t)(0x0001)	/*!< GPIO Pin 0 Selected */
#define GPIO_PIN_1			(uint16_t)(0x0002)	/*!< GPIO Pin 1 Selected */
#define GPIO_PIN_2			(uint16_t)(0x0004)	/*!< GPIO Pin 2 Selected */
#define GPIO_PIN_3			(uint16_t)(0x0008)	/*!< GPIO Pin 3 Selected */
#define GPIO_PIN_4			(uint16_t)(0x0010)	/*!< GPIO Pin 4 Selected */
#define GPIO_PIN_5			(uint16_t)(0x0020)	/*!< GPIO Pin 5 Selected */
#define GPIO_PIN_6			(uint16_t)(0x0040)	/*!< GPIO Pin 6 Selected */
#define GPIO_PIN_7			(uint16_t)(0x0080)	/*!< GPIO Pin 7 Selected */
#define GPIO_PIN_8			(uint16_t)(0x0100)	/*!< GPIO Pin 8 Selected */
#define GPIO_PIN_9			(uint16_t)(0x0200)	/*!< GPIO Pin 9 Selected */
#define GPIO_PIN_10			(uint16_t)(0x0400)	/*!< GPIO Pin 10 Selected */
#define GPIO_PIN_11			(uint16_t)(0x0800)	/*!< GPIO Pin 11 Selected */
#define GPIO_PIN_12			(uint16_t)(0x1000)	/*!< GPIO Pin 12 Selected */
#define GPIO_PIN_13			(uint16_t)(0x2000)	/*!< GPIO Pin 13 Selected */
#define GPIO_PIN_14			(uint16_t)(0x4000)	/*!< GPIO Pin 14 Selected */
#define GPIO_PIN_15			(uint16_t)(0x8000)	/*!< GPIO Pin 15 Selected */
#define GPIO_PIN_ALL		(uint16_t)(0xFFFF)	/*!< GPIO Pin ALL Selected */
#define GPIO_NUMBER_OF_PINS	(16U)

/*
 * @def_group GPIO_Pin_Modes
 *
 */
#define GPIO_MODE_INPUT		(0x0U)
#define GPIO_MODE_OUTPUT	(0x1U)
#define GPIO_MODE_AF		(0x2U)
#define GPIO_MODE_ANALOG	(0x3U)

/*
 * @def_group GPIO_OTYPE_Modes
 *
 */
#define GPIO_OTYPE_PP		(0x0U) 	/*!< Push-Pull */
#define GPIO_OTYPE_OD		(0x1U)	/*!< Open-Drain */

/*
 * @def_group GPIO_OSPEEDR_Modes
 *
 */
#define GPIO_OSPEED_LOW		(0x0U)
#define GPIO_OSPEED_MED		(0x1U)
#define GPIO_OSPEED_HIGH	(0x2U)
#define GPIO_OSPEED_V_HIGH	(0x3U)

/*
 * @def_group GPIO_PuPdR_Modes
 *
 */
#define GPIO_PUPDR_NOPULL	(0x00U)
#define GPIO_PUPDR_PULLUP	(0x1U)
#define GPIO_PUPDR_PULLDOWN	(0x2U)

typedef enum
{
	GPIO_PIN_RESET = 0x0U,
	GPIO_PIN_SET = !GPIO_PIN_RESET
}GPIO_PinState;

typedef struct
{
	uint32_t pinNumber; /*!< GPIO Pin numbers @def_group GPIO_PINS */
	uint32_t Mode;		/*!< GPIO Pin modes @def_group GPIO_Pin_Modes */
	uint32_t Otype;		/*!< GPIO Otype modes @def_group GPIO_OTYPE_Modes */
	uint32_t PuPd;		/*!< GPIO PuPdR modes @def_group GPIO_PuPdR_Modes */
	uint32_t Speed;		/*!< GPIO OSPEEDR modes @def_group GPIO_OSPEEDR_Modes */
	uint32_t Alternate;

}GPIO_InitTypeDef;

void GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_ConfigStruct);
void GPIO_Write_Pin(GPIO_TypeDef *GPIOx, uint16_t pinNumber, GPIO_PinState pinState);
GPIO_PinState GPIO_Read_Pin(GPIO_TypeDef *GPIOx, uint16_t pinNumber);
void GPIO_Lock_Pin(GPIO_TypeDef *GPIOx, uint16_t pinNumber);

#endif /* INC_GPIO_H_ */

/*
 * GPIO.c
 *
 *  Created on: May 1, 2025
 *      Author: erhan
 */
#include "GPIO.h"

/*
 * @brief GPIO_Init, configures the port and pins
 *
 * @param GPIOx = GPIO Port Base Address
 *
 * @param GPIO_ConfigStruct = GPIO configuration structure
 *
 * @retval void
 */
void GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_ConfigStruct)
{
	uint32_t position;
	uint32_t fakePosition = 0;
	uint32_t lastPosition = 0;
	for (position = 0; position < GPIO_NUMBER_OF_PINS; position++)
	{
		fakePosition = (0x1 << position);
		lastPosition = (uint32_t)( GPIO_ConfigStruct->pinNumber ) & fakePosition;

		if (fakePosition == lastPosition)
		{
			/*!< Mode config */
			uint32_t tempValue = GPIOx->MODER;
			tempValue &= ~( 0x3U << (position * 2) );
			tempValue |= ( GPIO_ConfigStruct->Mode << (position * 2) );
			GPIOx->MODER = tempValue;

			if (GPIO_ConfigStruct->Mode == GPIO_MODE_AF || GPIO_ConfigStruct->Mode == GPIO_MODE_OUTPUT)
			{
				/*!< Otype Config */
				tempValue = GPIOx->OTYPER;
				tempValue &= ~( 0x1U << position );
				tempValue |= ( GPIO_ConfigStruct->Otype << position );
				GPIOx->OTYPER = tempValue;

				/*!< Ospeed Config */
				tempValue = GPIOx->OSPEEDR;
				tempValue &= ~(0x3U <<(position * 2) );
				tempValue |= ( GPIO_ConfigStruct->Speed << (position * 2) );
				GPIOx->OSPEEDR = tempValue;
			}

			/*!< Push-Pull Config */
			tempValue = GPIOx->PUPDR;
			tempValue &= ~( 0x3U << (position * 2) );
			tempValue |= ( GPIO_ConfigStruct->PuPd << (position * 2) );
			GPIOx->PUPDR = tempValue;
		}
	}
}

/*
 * @brief GPIO_Write_Pin, Makes pin high or low
 *
 * @param GPIOx = GPIO Port Base Address
 *
 * @param pinNumber = GPIO Pin Numbers 0 - 15
 *
 * @param pinState = GPIO_PIN_SET or GPIO_PIN_RESET
 *
 * @retval void
 */
void GPIO_Write_Pin(GPIO_TypeDef *GPIOx, uint16_t pinNumber, GPIO_PinState pinState)
{
	if (pinState == GPIO_PIN_SET)
	{
		GPIOx->BSRR = pinNumber;
	}
	else
	{
		GPIOx->BSRR = (pinNumber << 16);
	}
}

/*
 * @brief GPIO_Read_Pin, reads the pin of GPIOx Port
 *
 * @param GPIOx = GPIO Port Base Address
 *
 * @param pinNumber = GPIO Pin Numbers 0 - 15
 *
 * @retval GPIO_PinState
 */
GPIO_PinState GPIO_Read_Pin(GPIO_TypeDef *GPIOx, uint16_t pinNumber)
{
	GPIO_PinState bitStatus = GPIO_PIN_RESET;

	if ((GPIOx->IDR & pinNumber) != GPIO_PIN_RESET)
	{
		bitStatus = GPIO_PIN_SET;
	}

	return bitStatus;
}

/*
 * @brief GPIO_Lock_Pin, locks the pin of GPIOx port
 *
 * @param GPIOx = GPIO Port Base Address
 *
 * @param pinNumber = GPIO Pin Numbers 0 - 15
 *
 * @retval void
 */
void GPIO_Lock_Pin(GPIO_TypeDef *GPIOx, uint16_t pinNumber)
{
	uint32_t tempValue = (0x1U << 16) | pinNumber;

	GPIOx->LCKR = tempValue; // LCKR[16] = '1' + LCKR[15:0] = pinNumber
	GPIOx->LCKR = pinNumber; // LCKR[16] = '0' + LCKR[15:0] = pinNumber
	GPIOx->LCKR = tempValue; // LCKR[16] = '1' + LCKR[15:0] = pinNumber

	tempValue = GPIOx->LCKR; // Read Lock Register
}

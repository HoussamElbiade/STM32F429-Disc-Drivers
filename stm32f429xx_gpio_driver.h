/*
 * stm32f429xx_gpio_driver.h
 * Created on: Nov 23, 2023
 * Author: Houssame ELBIADE
 */

#ifndef INC_STM32F429XX_GPIO_DRIVER_H_
#define INC_STM32F429XX_GPIO_DRIVER_H_

#include <stm32f429xx.h>
#include <stdint.h>

typedef struct {
	uint8_t PinNumber;			/*<Possible values from @GPIO_PIN_NUMBER>*/
	uint8_t PinMode;			/*<Possible values from @GPIO_PIN_MODES>*/
	uint8_t PinOType;			/*<Possible values from @GPIO_OUTPUT_TYPES>*/
	uint8_t PinSpeed;			/*<Possible values from @GPIO_OUTPUT_SPEED>*/
	uint8_t PinPuPdControl;		/*<Possible values from @GPIO_PU_PD>*/
	uint8_t PinAltFunction;
}GPIO_PinConfig_t;

typedef struct {
	//base address of the peripheral (GPIO in this case)
	GPIOx_RegDef_t* pGPIOx;
	//the Peripheral Configuration
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handel_t;

/* Configuration Macros */

// @GPIO_PIN_NUMBER
#define GPIO_PIN_0				0
#define GPIO_PIN_1				1
#define GPIO_PIN_2				2
#define GPIO_PIN_3				3
#define GPIO_PIN_4				4
#define GPIO_PIN_5				5
#define GPIO_PIN_6				6
#define GPIO_PIN_7				7
#define GPIO_PIN_8				8
#define GPIO_PIN_9				9
#define GPIO_PIN_10				10
#define GPIO_PIN_11				11
#define GPIO_PIN_12				12
#define GPIO_PIN_13				13
#define GPIO_PIN_14				14
#define GPIO_PIN_15				15

// @GPIO_PIN_MODES
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6

// @GPIO_OUTPUT_TYPES
#define GPIO_OTYPE_PP			0
#define GPIO_OTYPE_OD			1

// @GPIO_OUTPUT_SPEED
#define GPIO_OSPEED_L			0
#define GPIO_OSPEED_M			1
#define GPIO_OSPEED_H			2
#define GPIO_OSPEED_V			3

// @GPIO_PU_PD
#define GPIO_NO_PUPD			0
#define GPIO_PU					1
#define GPIO_PD					2

// @GPIO_ALT_FUN
#define GPIO_AF0				0
#define GPIO_AF1				1
#define GPIO_AF2				2
#define GPIO_AF3				3
#define GPIO_AF4				4
#define GPIO_AF5				5
#define GPIO_AF6				6
#define GPIO_AF7				7
#define GPIO_AF8				8
#define GPIO_AF9				9
#define GPIO_AF10				10
#define GPIO_AF11				11
#define GPIO_AF12				12
#define GPIO_AF13				13
#define GPIO_AF14				14
#define GPIO_AF15				15




/***********************************************************************************************************************
*											 GPIO API DEFINITIONS													   /
*********************************************************************************************************************/

// Peripheral clock setup

void GPIO_ClkControl(GPIOx_RegDef_t* pGPIOx, uint8_t EnOrDi);

// Init - De-Init

void Init(GPIO_Handel_t* pGPIOxHandle);
void DeInit(GPIOx_RegDef_t* pGPIOx); // use the RCC AHB1 peripheral reset register

// Data read and write

uint8_t GPIO_ReadFromInputPin(GPIOx_RegDef_t* pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIOx_RegDef_t* pGPIOx);
void GPIO_WriteToOutputPin(GPIOx_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Data);
void GPIO_WriteToOutputPort(GPIOx_RegDef_t* pGPIOx, uint16_t Data);
void GPIO_ToggleOutputPin(GPIOx_RegDef_t* pGPIOx, uint8_t PinNumber);

// IRQ configure and handle

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t Priority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F429XX_GPIO_DRIVER_H_ */

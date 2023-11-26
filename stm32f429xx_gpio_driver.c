/*
 * stm32f429xx_gpio_driver.c
 *
 *  Created on: Nov 23, 2023
 *      Author: Asus
 */
#include <stm32f429xx_gpio_driver.h>

// Peripheral clock setup
/************************************************************************************
 * @fun_name		- GPIO_ClkControl
 *
 * @description		- enables or disables peripheral clock for given GPIO port
 *
 * @param_1			- Pointer to the base address of a given GPIO peripheral
 * @param_2			- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @Note			- none
 *
 *************************************************************************************/
void GPIO_ClkControl(GPIOx_RegDef_t* pGPIOx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
		if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}
		if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
		if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
		if(pGPIOx == GPIOJ){
			GPIOJ_PCLK_EN();
		}
		if(pGPIOx == GPIOK){
			GPIOK_PCLK_EN();
		}
	}
	else {
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}
		if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}
		if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
		if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}
		if(pGPIOx == GPIOJ){
			GPIOJ_PCLK_DI();
		}
		if(pGPIOx == GPIOK){
			GPIOK_PCLK_DI();
		}
	}
}

// Init - De-Init
/************************************************************************************
 * @fun_name		- Init
 *
 * @description		- Initialize the GPIO peripheral with a given Pin Configuration
 *
 * @param_1			- Pointer to the GPIO port Handler
 *
 * @return			- none
 *
 * @Note			- none
 *
 *************************************************************************************/
void Init(GPIO_Handel_t* pGPIOxHandle){

	// 1. Configure the GPIO direction
	uint32_t temp = 0;
	if(pGPIOxHandle->GPIO_PinConfig.PinMode <= GPIO_MODE_ANALOG){
		temp = (pGPIOxHandle->GPIO_PinConfig.PinMode << (2 * pGPIOxHandle->GPIO_PinConfig.PinNumber));
		//make sure that the modification are correctly applied, So before any Or-bit wise we must clear those bits
		pGPIOxHandle->pGPIOx->MODER &= (3 << (2 * pGPIOxHandle->GPIO_PinConfig.PinNumber));
		pGPIOxHandle->pGPIOx->MODER |= temp;
	}
	else{
		// 1. Configure the edge trigger
		if(pGPIOxHandle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_FT){
			// Configure the corresponding FTSR bit
			EXTI->FTSR |= (1 << pGPIOxHandle->GPIO_PinConfig.PinNumber);
			// tip : to make sure all is going to be done correctly: Clear the corresponding bit of the RTSR
			EXTI->RTSR &= ~(1 << pGPIOxHandle->GPIO_PinConfig.PinNumber);
		}else if(pGPIOxHandle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_RT){
			// Configure the corresponding RTSR bit
			EXTI->RTSR |= (1 << pGPIOxHandle->GPIO_PinConfig.PinNumber);
			// tip : to make sure all is going to be done correctly: Clear the corresponding bit of the FTSR
			EXTI->FTSR &= ~(1 << pGPIOxHandle->GPIO_PinConfig.PinNumber);
		}else if(pGPIOxHandle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_RFT){
			// Configure the corresponding RTSR & FTSR bit
			EXTI->RTSR |= (1 << pGPIOxHandle->GPIO_PinConfig.PinNumber);
			EXTI->FTSR |= (1 << pGPIOxHandle->GPIO_PinConfig.PinNumber);
		}

		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp_1;
		uint8_t temp_2;
		uint8_t value;
		temp_1 = pGPIOxHandle->GPIO_PinConfig.PinNumber / 4; // select which control register to configure
		temp_2 = pGPIOxHandle->GPIO_PinConfig.PinNumber % 4; // select witch section of the selected register

		SYSCFG_REG_EN();
		value = GPIO_PORT_EXTI_VALUE(pGPIOxHandle->pGPIOx);
		SYSCFG->EXTICR[4] &= ~(0xF << (4 * temp_2));
		SYSCFG->EXTICR[temp_1] |= (value << (4 * temp_2));

		// 3. Enable the EXTI interruption delivery (Periph ---> Proc) using IMR
		EXTI->IMR |= (1 << pGPIOxHandle->GPIO_PinConfig.PinNumber);

	}
	temp = 0;

	// 2. configure the the speed
	temp = (pGPIOxHandle->GPIO_PinConfig.PinSpeed << (2 * pGPIOxHandle->GPIO_PinConfig.PinNumber));
	pGPIOxHandle->pGPIOx->OSPEEDR &= ~(3 << (2 * pGPIOxHandle->GPIO_PinConfig.PinNumber));
	pGPIOxHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// 3. configure the pull-up pull-down
	temp = (pGPIOxHandle->GPIO_PinConfig.PinPuPdControl << (2 * pGPIOxHandle->GPIO_PinConfig.PinNumber));
	pGPIOxHandle->pGPIOx->PUPDR &= ~(3 << (2 * pGPIOxHandle->GPIO_PinConfig.PinNumber));
	pGPIOxHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	// 4. configure the output type
	temp = (pGPIOxHandle->GPIO_PinConfig.PinOType << pGPIOxHandle->GPIO_PinConfig.PinNumber);
	pGPIOxHandle->pGPIOx->OTYPER &= ~(1 << (pGPIOxHandle->GPIO_PinConfig.PinNumber));
	pGPIOxHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	// 5. configure the alternate function
	if(pGPIOxHandle->GPIO_PinConfig.PinMode == GPIO_MODE_ALTFN){
		uint8_t temp_1 = pGPIOxHandle->GPIO_PinConfig.PinNumber / 8; // is 1 or 0 => decide which AFR register we will change
		uint8_t temp_2 = pGPIOxHandle->GPIO_PinConfig.PinNumber % 8; // how much the AF will be shifted

		pGPIOxHandle->pGPIOx->AFR[temp_1] &= ~(0xF << (4 * temp_2));
		pGPIOxHandle->pGPIOx->AFR[temp_1] |= (pGPIOxHandle->GPIO_PinConfig.PinAltFunction << (4 * temp_2));
	}

}
/************************************************************************************
 * @fun_name		- DeInit
 *
 * @description		- Reset all the register for a given GPIO Port
 *
 * @param_1			- Pointer to the base address of a given GPIO peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 *************************************************************************************/
void DeInit(GPIOx_RegDef_t* pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RST();
	}
	if(pGPIOx == GPIOB){
		GPIOB_REG_RST();
	}
	if(pGPIOx == GPIOC){
		GPIOC_REG_RST();
	}
	if(pGPIOx == GPIOD){
		GPIOD_REG_RST();
	}
	if(pGPIOx == GPIOE){
		GPIOE_REG_RST();
	}
	if(pGPIOx == GPIOF){
		GPIOF_REG_RST();
	}
	if(pGPIOx == GPIOG){
		GPIOG_REG_RST();
	}
	if(pGPIOx == GPIOH){
		GPIOH_REG_RST();
	}
	if(pGPIOx == GPIOI){
		GPIOI_REG_RST();
	}
	if(pGPIOx == GPIOJ){
		GPIOJ_REG_RST();
	}
	if(pGPIOx == GPIOK){
		GPIOK_REG_RST();
	}
}


// Data read and write
/************************************************************************************
 * @fun_name		- GPIO_ReadFromInputPin
 *
 * @description		- Read the value of a pin in a given GPIO port
 *
 * @param_1			- Pointer to the base address of a given GPIO peripheral
 * @param_2			- Pin Number
 *
 * @return			- Pin value (1 or 0)
 *
 * @Note			- none
 *
 *************************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIOx_RegDef_t* pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & (0x00000001));
	return value;
}
/************************************************************************************
 * @fun_name		- GPIO_ReadFromInputPort
 *
 * @description		- Read the Value of a GPIO port
 *
 * @param_1			- Pointer to the base address of a given GPIO peripheral
 *
 * @return			- Port Value (16 bits value)
 *
 * @Note			- none
 *
 *************************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIOx_RegDef_t* pGPIOx){
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}
/************************************************************************************
 * @fun_name		- GPIO_WriteToOutputPin
 *
 * @description		- Write Some data to a specific Pin of a given GPIO Port
 *
 * @param_1			- Pointer to the base address of a given GPIO peripheral
 * @param_2			- Pin Number
 * @param_3			- 8 bits data
 * @param_4
 *
 * @return			- none
 *
 * @Note			- none
 *
 *************************************************************************************/
void GPIO_WriteToOutputPin(GPIOx_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Data){
	if(Data == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}else{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}
/************************************************************************************
 * @fun_name		- GPIO_WriteToOutputPort
 *
 * @description		- Write Some data to a specific GPIO Port
 *
 * @param_1			- Pointer to the base address of a given GPIO peripheral
 * @param_3			- 16 bits data
 *
 * @return			- none
 *
 * @Note			- none
 *
 *************************************************************************************/
void GPIO_WriteToOutputPort(GPIOx_RegDef_t* pGPIOx, uint16_t Data){
	pGPIOx->ODR = Data;
}
/************************************************************************************
 * @fun_name		- GPIO_ToggleOutputPin
 *
 * @description		- Toggle a Pin (Switch state 0->1->0->...)
 *
 * @param_1			- Pointer to the base address of a given GPIO peripheral
 * @param_2			- Pin Number
 *
 * @return			- none
 *
 * @Note			- none
 *
 *************************************************************************************/
void GPIO_ToggleOutputPin(GPIOx_RegDef_t* pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}

// IRQ configure and handle
/************************************************************************************
 * @fun_name		- GPIO_IRQInterruptConfig
 *
 * @description		- Configure an Interrupt Request(IRQ)
 *
 * @param_1			- IRQ Number
 * @param_2			- ENABLE or DISABLE Macros
 *
 * @return			- none
 *
 * @Note			- none
 *
 *************************************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi){

	uint8_t temp_1;
	uint8_t temp_2;

	temp_1 = IRQNumber / 32;
	temp_2 = IRQNumber % 32;

	if(EnOrDi == ENABLE){
		*(NVIC_ISER_BASE_ADDRESS + (temp_1)) |= (1 << temp_2);
	}
	else if (EnOrDi == DISABLE){
		*(NVIC_ICER_BASE_ADDRESS + (temp_1)) |= (1 << temp_2);
	}
}
/************************************************************************************
 * @fun_name		- GPIO_IRQPriorityConfig
 *
 * @description		- Set the Priority of an IRQ Interruption
 *
 * @param_1			- IRQ Number
 * @param_2			- Priority
 *
 * @return			- none
 *
 * @Note			- none
 *
 *************************************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t Priority){
	uint8_t temp_1;
	uint8_t temp_2;
	uint8_t shift_val;
	temp_1 = IRQNumber / 4;
	temp_2 = IRQNumber % 4;

	*(NVIC_IPR__BASE_ADDRESS + (temp_1 * 4)) &= ~(0xFF << (8 * temp_2));
	shift_val = (8 * temp_2) + (8 - NON_PR_IMPLEMENTED_BITS);
	*(NVIC_IPR__BASE_ADDRESS + (temp_1)) |= (Priority << shift_val);

}
/************************************************************************************
 * @fun_name		- GPIO_IRQHandling
 *
 * @description		- Handle the Interrupt Request if Enabled
 *
 * @param_1			- Pin Number
 *
 * @return			- none
 *
 * @Note			- none
 *
 *************************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber){
	if(EXTI->PR & (1 << PinNumber)){
		EXTI->PR |= (1 << PinNumber);
	}
}

/*
 * stm32f429xx.h
 *
 *  Created on: Nov 19, 2023
 *      Author: Houssame ELBIADE
 */

#ifndef INC_STM32F429XX_H_
#define INC_STM32F429XX_H_

#include <stdint.h>

#define __vo						volatile


/*********************************** Cortex-M4 Specific Registers*********************/
//Interrupt Set-enable Register ISERx
#define NVIC_ISER_BASE_ADDRESS		((__vo uint32_t*)0xE000E100UL)

//Interrupt Clear-enable Registers ICERx
#define NVIC_ICER_BASE_ADDRESS		((__vo uint32_t*)0XE000E180UL)

//Interrupt Priority Register IPRx
#define NVIC_IPR__BASE_ADDRESS		((__vo uint32_t*)0xE000E400UL)
#define NON_PR_IMPLEMENTED_BITS		4

/*
 * base addresses of Flash and SRAM memories
 */
#define FLASH_BASE_ADDRESS			(0x08000000UL)	// 2 MB
#define SRAM1_BASE_ADDRESS			(0x20000000UL)  // 112 KB
#define SRAM2_BASE_ADDRESS			(0x2001C000UL) // 16 KB
#define SRAM3_BASE_ADDRESS			(0x20020000UL) // 64 KB
#define ROM					(0x1FFF0000UL)
#define SRAM					SRAM1_BASE_ADDRESS

/*
 * base address of bus domains
 */
#define APB1_BASE_ADDRESS			(0x40000000UL)
#define APB2_BASE_ADDRESS			(0x40010000UL)
#define AHB1_BASE_ADDRESS			(0x40020000UL)
#define AHB2_BASE_ADDRESS			(0x50000000UL)
#define AHB3_BASE_ADDRESS			(0xA0000000UL)
#define PERIPH_BASE					APB1_BASE_ADDRESS


/*
 * APB1 peripherals base addresses
 */
#define TIM2_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x0000UL)
#define TIM3_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x0400UL)
#define TIM4_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x0800UL)
#define TIM5_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x0C00UL)
#define TIM6_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x1000UL)
#define TIM7_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x1400UL)
#define TIM12_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x1800UL)
#define TIM13_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x1C00UL)
#define TIM14_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x2000UL)
#define RTC_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x2800UL)
#define WWDG_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x2C00UL)
#define IWDG_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x3000UL)
#define I2S2ext_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x3400UL)
#define SPI2_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x3800UL)
#define I2S2_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x3800UL)
#define SPI3_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x3C00UL)
#define I2S3_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x3C00UL)
#define I2S3ext_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x4000UL)
#define USART2_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x4400UL)
#define USART3_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x4800UL)
#define UART4_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x4C00UL)
#define UART5_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x5000UL)
#define I2C1_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x5400UL)
#define I2C2_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x5800UL)
#define I2C3_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x5C00UL)
#define CAN1_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x6400UL)
#define CAN2_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x6800UL)
#define PWR_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x7000UL)
#define DAC_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x7400UL)
#define UART7_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x7800UL)
#define UART8_BASE_ADDRESS			(APB1_BASE_ADDRESS + 0x7C00UL)


/*
 * APB2 peripherals base addresses
 */
#define TIM1_BASE_ADDRESS			(APB2_BASE_ADDRESS + 0x0000UL)
#define TIM8_BASE_ADDRESS			(APB2_BASE_ADDRESS + 0x0400UL)
#define USART1_BASE_ADDRESS			(APB2_BASE_ADDRESS + 0x1000UL)
#define USART6_BASE_ADDRESS			(APB2_BASE_ADDRESS + 0x1400UL)
#define ADC1_OFFSET				(0x000UL)
#define ADC2_OFFSET				(0x100UL)
#define ADC3_OFFSET				(0x200UL)
#define ADC_COMM_REG_OFFSET			(0x300UL)
#define ADC1_BASE_ADDRESS			(APB2_BASE_ADDRESS + 0x2000UL + ADC1_OFFSET)
#define ADC2_BASE_ADDRESS			(APB2_BASE_ADDRESS + 0x2000UL + ADC2_OFFSET)
#define ADC3_BASE_ADDRESS			(APB2_BASE_ADDRESS + 0x2000UL + ADC3_OFFSET)
#define ADC_COMM_REG_BASE_ADDRESS		(APB2_BASE_ADDRESS + 0x2000UL + ADC_COMM_REG_OFFSET)
#define SDIO_BASE_ADDRESS			(APB2_BASE_ADDRESS + 0x2C00UL)
#define SPI1_BASE_ADDRESS			(APB2_BASE_ADDRESS + 0x3000UL)
#define SPI4_BASE_ADDRESS			(APB2_BASE_ADDRESS + 0x3400UL)
#define SYSCFG_BASE_ADDRESS			(APB2_BASE_ADDRESS + 0x3800UL)
#define EXTI_BASE_ADDRESS			(APB2_BASE_ADDRESS + 0x3C00UL)
#define TIM9_BASE_ADDRESS			(APB2_BASE_ADDRESS + 0x4000UL)
#define TIM10_BASE_ADDRESS			(APB2_BASE_ADDRESS + 0x4400UL)
#define TIM11_BASE_ADDRESS			(APB2_BASE_ADDRESS + 0x4800UL)
#define SPI5_BASE_ADDRESS			(APB2_BASE_ADDRESS + 0x5000UL)
#define SPI6_BASE_ADDRESS			(APB2_BASE_ADDRESS + 0x5400UL)
#define SAI1_BASE_ADDRESS			(APB2_BASE_ADDRESS + 0x5800UL)
#define LCD_TFT_BASE_ADDRESS			(APB2_BASE_ADDRESS + 0x6800UL)


/*
 * AHB1 peripherals base addresses
 */
#define GPIOA_BASE_ADDRESS			(AHB1_BASE_ADDRESS + 0x0000UL)
#define GPIOB_BASE_ADDRESS			(AHB1_BASE_ADDRESS + 0x0400UL)
#define GPIOC_BASE_ADDRESS			(AHB1_BASE_ADDRESS + 0x0800UL)
#define GPIOD_BASE_ADDRESS			(AHB1_BASE_ADDRESS + 0x0C00UL)
#define GPIOE_BASE_ADDRESS			(AHB1_BASE_ADDRESS + 0x1000UL)
#define GPIOF_BASE_ADDRESS			(AHB1_BASE_ADDRESS + 0x1400UL)
#define GPIOG_BASE_ADDRESS			(AHB1_BASE_ADDRESS + 0x1800UL)
#define GPIOH_BASE_ADDRESS			(AHB1_BASE_ADDRESS + 0x1C00UL)
#define GPIOI_BASE_ADDRESS			(AHB1_BASE_ADDRESS + 0x2000UL)
#define GPIOJ_BASE_ADDRESS			(AHB1_BASE_ADDRESS + 0x2400UL)
#define GPIOK_BASE_ADDRESS			(AHB1_BASE_ADDRESS + 0x2800UL)
#define CRC_BASE_ADDRESS			(AHB1_BASE_ADDRESS + 0x3000UL)
#define RCC_BASE_ADDRESS			(AHB1_BASE_ADDRESS + 0x3800UL)
#define FIR_BASE_ADDRESS			(AHB1_BASE_ADDRESS + 0x3C00UL) // Flash interface register
#define BKPSRAM_BASE_ADDRESS			(AHB1_BASE_ADDRESS + 0x4000UL)
#define DMA1_BASE_ADDRESS			(AHB1_BASE_ADDRESS + 0x6000UL)
#define DMA2_BASE_ADDRESS			(AHB1_BASE_ADDRESS + 0x6400UL)
#define ETHERNET_MAC_BASE_ADDRESS		(AHB1_BASE_ADDRESS + 0x8000UL)
#define DMA2D_BASE_ADDRESS			(AHB1_BASE_ADDRESS + 0xB000UL)
#define USB_OTG_HS_BASE_ADDRESS			(AHB1_BASE_ADDRESS + 0x20000UL) // base address = 0x40040000 - 0x40020000

/*
 * AHB2 peripherals base addresses
 */
#define USB_OTG_FS_BASE_ADDRESS			(AHB2_BASE_ADDRESS + 0x00000UL)
#define DCMI_BASE_ADDRESS			(AHB2_BASE_ADDRESS + 0x50000UL)
#define CRYP_BASE_ADDRESS			(AHB2_BASE_ADDRESS + 0x60000UL)
#define HASH_BASE_ADDRESS			(AHB2_BASE_ADDRESS + 0x60400UL)
#define RNG_BASE_ADDRESS			(AHB2_BASE_ADDRESS + 0x60800UL)

/*
 * AHB3 peripherals base addresses
 */
#define FMC_CR_BASE_ADDRESS			(AHB3_BASE_ADDRESS + 0x0000UL)


/******************************Peripheral register definition structures**************************************/
/*RCC Structure*/
typedef struct {
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RES_01;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RES_02[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RES_03;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RES_04[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RES_05;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RES_06[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RES_07[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
}RCC_RegDef_t;

/* GPIOx Structure */
typedef struct {
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
}GPIOx_RegDef_t;

/* SYSCFG Structure */
typedef struct {
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t RES[2];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;

/* DMA Structure */
typedef struct {
	__vo uint32_t LISR;
	__vo uint32_t HISR;
	__vo uint32_t LIFCR;
	__vo uint32_t HIFCR;
	__vo uint32_t S0CR;
	__vo uint32_t S0NDTR;
	__vo uint32_t S0PAR;
	__vo uint32_t S0M0AR;
	__vo uint32_t S0M1AR;
	__vo uint32_t S0FCR;
	__vo uint32_t S1CR;
	__vo uint32_t S1NDTR;
	__vo uint32_t S1PAR;
	__vo uint32_t S1M0AR;
	__vo uint32_t S1M1AR;
	__vo uint32_t S1FCR;
	__vo uint32_t S2CR;
	__vo uint32_t S2NDTR;
	__vo uint32_t S2PAR;
	__vo uint32_t S2M0AR;
	__vo uint32_t S2M1AR;
	__vo uint32_t S2FCR;
	__vo uint32_t S3CR;
	__vo uint32_t S3NDTR;
	__vo uint32_t S3PAR;
	__vo uint32_t S3M0AR;
	__vo uint32_t S3M1AR;
	__vo uint32_t S3FCR;
	__vo uint32_t S4CR;
	__vo uint32_t S4NDTR;
	__vo uint32_t S4PAR;
	__vo uint32_t S4M0AR;
	__vo uint32_t S4M1AR;
	__vo uint32_t S4FCR;
	__vo uint32_t S5CR;
	__vo uint32_t S5NDTR;
	__vo uint32_t S5PAR;
	__vo uint32_t S5M0AR;
	__vo uint32_t S5M1AR;
	__vo uint32_t S5FCR;
	__vo uint32_t S6CR;
	__vo uint32_t S6NDTR;
	__vo uint32_t S6PAR;
	__vo uint32_t S6M0AR;
	__vo uint32_t S6M1AR;
	__vo uint32_t S6FCR;
	__vo uint32_t S7CR;
	__vo uint32_t S7NDTR;
	__vo uint32_t S7PAR;
	__vo uint32_t S7M0AR;
	__vo uint32_t S7M1AR;
	__vo uint32_t S7FCR;
}DMA_RegDef_t;


/* EXTI Structure */
typedef struct {
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;


/* ADC Structure*/
typedef struct {
	__vo uint32_t SR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SMPR1;
	__vo uint32_t SMPR2;
	__vo uint32_t JOFR1;
	__vo uint32_t JOFR2;
	__vo uint32_t JOFR3;
	__vo uint32_t JOFR4;
	__vo uint32_t HTR;
	__vo uint32_t LTR;
	__vo uint32_t SQR1;
	__vo uint32_t SQR2;
	__vo uint32_t SQR3;
	__vo uint32_t JSQR;
	__vo uint32_t JDR1;
	__vo uint32_t JDR2;
	__vo uint32_t JDR3;
	__vo uint32_t JDR4;
	__vo uint32_t DR;
}ADC_RegDef_t;

// Common ADC registers Structure
typedef struct {
	__vo uint32_t CSR;
	__vo uint32_t CCR;
	__vo uint32_t CDR;
}ADC_COMM_RegDef;

/* DAC Structure*/
typedef struct {
	__vo uint32_t CR ;
	__vo uint32_t SWTRIGR;
	__vo uint32_t DHR12R1;
	__vo uint32_t DHR12L1;
	__vo uint32_t DHR8R1;
	__vo uint32_t DHR12R2;
	__vo uint32_t DHR12L2;
	__vo uint32_t DHR8R2;
	__vo uint32_t DHR12RD;
	__vo uint32_t DHR12LD;
	__vo uint32_t DHR8RD;
	__vo uint32_t DOR1;
	__vo uint32_t DOR2;
	__vo uint32_t SR;
}DAC_RegDef_t;

/* DCMI (Digital Camera Interface) Structure */
typedef struct {
	__vo uint32_t CR;
	__vo uint32_t SR;
	__vo uint32_t RIS;
	__vo uint32_t IER;
	__vo uint32_t MIS;
	__vo uint32_t ICR;
	__vo uint32_t ESCR;
	__vo uint32_t ESUR;
	__vo uint32_t CWSTRT;
	__vo uint32_t CWSIZE;
	__vo uint32_t DR;

}DCMI_RegDef_t;

/* LTDC Structure */
typedef struct{
	__vo uint32_t Nop[2];
	__vo uint32_t SSCR;
	__vo uint32_t BPCR;
	__vo uint32_t AWCR;
	__vo uint32_t TWCR;
	__vo uint32_t GCR; // offset : 0x0018
	__vo uint32_t RES_01[3];
	__vo uint32_t SRCR; // offset : 0x0024
	__vo uint32_t RES_02;
	__vo uint32_t BCCR; // offset : 0x002C
	__vo uint32_t RES_03[2];
	__vo uint32_t IER; // offset : 0x0034
	__vo uint32_t ISR;
	__vo uint32_t ICR;
	__vo uint32_t LIPCR;
	__vo uint32_t CPSR;
	__vo uint32_t CDSR; // offset : 0x0048
	__vo uint32_t RES_04[15];
	__vo uint32_t L1CR; // offset : 0x0084
	__vo uint32_t L1WHPCR;
	__vo uint32_t L1WVPCR;
	__vo uint32_t L1CKCR;
	__vo uint32_t L1PFCR;
	__vo uint32_t L1CACR;
	__vo uint32_t L1DCCR;
	__vo uint32_t L1BFCR;
	__vo uint32_t L1CFBAR;
	__vo uint32_t L1CFBLR;
	__vo uint32_t L1CFBLNR; // offset : 0x00B4
	__vo uint32_t RES_05[4];
	__vo uint32_t L1CLUTWR; // offset : 0x00C4
	__vo uint32_t RES_06[16];
	__vo uint32_t L2CR;// offset : 0x0104
	__vo uint32_t L2WHPCR;
	__vo uint32_t L2WVPCR;
	__vo uint32_t L2CKCR;
	__vo uint32_t L2PFCR;
	__vo uint32_t L2CACR;
	__vo uint32_t L2DCCR;
	__vo uint32_t L2BFCR;
	__vo uint32_t RES_07[3];
	__vo uint32_t L2CFBAR;
	__vo uint32_t L2CFBLR;
	__vo uint32_t L2CFBLNR;
	__vo uint32_t RES_08[4];
	__vo uint32_t L2CLUTWR;
}LTDC_RegDef_t;

/*TIM1 and TIM8 Structure*/
typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SMCR;
	__vo uint32_t DIER;
	__vo uint32_t SR;
	__vo uint32_t EGR;
	__vo uint32_t CCMR1;
	__vo uint32_t CCMR2;
	__vo uint32_t CCER;
	__vo uint32_t CNT;
	__vo uint32_t PSC;
	__vo uint32_t ARR;
	__vo uint32_t RCR;
	__vo uint32_t CCR1;
	__vo uint32_t CCR2;
	__vo uint32_t CCR3;
	__vo uint32_t CCR4;
	__vo uint32_t BDTR;
	__vo uint32_t DCR;
	__vo uint32_t DMAR;
}TIM7_TIM8_RegDef_t;

/*TIM2 and TIM5 Structure*/
typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SMCR;
	__vo uint32_t DIER;
	__vo uint32_t SR;
	__vo uint32_t EGR;
	__vo uint32_t CCMR1;
	__vo uint32_t CCMR2;
	__vo uint32_t CCER;
	__vo uint32_t CNT;
	__vo uint32_t PSC;
	__vo uint32_t ARR;
	__vo uint32_t RCR;
	__vo uint32_t CCR1;
	__vo uint32_t CCR2;
	__vo uint32_t CCR3;
	__vo uint32_t CCR4;
	__vo uint32_t RES;
	__vo uint32_t DCR;
	__vo uint32_t DMAR;
	__vo uint32_t OR;
}TIM2_TIM5_RegDef_t;

/*TIM10/11/13/14 Structure*/
typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t RES_01;
	__vo uint32_t SMCR;
	__vo uint32_t DIER;
	__vo uint32_t SR;
	__vo uint32_t EGR;
	__vo uint32_t CCMR1;
	__vo uint32_t RES_02;
	__vo uint32_t CCER;
	__vo uint32_t CNT;
	__vo uint32_t PSC;
	__vo uint32_t ARR;
	__vo uint32_t RES_03;
	__vo uint32_t CCR1;
	__vo uint32_t RES[6];
	__vo uint32_t OR;
}TIM10_11_13_14_RegDef_t;

/* TIM6 TIM7 Structure*/
typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t RES_01;
	__vo uint32_t DIER;
	__vo uint32_t SR;
	__vo uint32_t EGR;
	__vo uint32_t RES_02[3];
	__vo uint32_t CNT;
	__vo uint32_t PSC;
	__vo uint32_t ARR;
}TIM6_TIM7_RegDef_t;


/* IWDG Structure */
typedef struct {
	__vo uint32_t KR;
	__vo uint32_t PR;
	__vo uint32_t PLR;
	__vo uint32_t SR;
}IWDG_RegDef_t;

/* WWDG Structure */
typedef struct {
	__vo uint32_t CR;
	__vo uint32_t CFR;
	__vo uint32_t SR;
}WWDG_RegDef_t;

/*Cryptographic processor : Skipped*/

/*RNG Structure*/
typedef struct {
	__vo uint32_t CR;
	__vo uint32_t SR;
	__vo uint32_t DR;
}RNG_RegDef_t;

/* RTC Structure */
typedef struct {
	__vo uint32_t TR;
	__vo uint32_t DR;
	__vo uint32_t CR;
	__vo uint32_t ISR;
	__vo uint32_t PRER;
	__vo uint32_t WUTR;
	__vo uint32_t CALIBR;
	__vo uint32_t ALRMAR;
	__vo uint32_t ALRMBR;
	__vo uint32_t WPR;
	__vo uint32_t SSR;
	__vo uint32_t SHIFTR;
	__vo uint32_t TSTR;
	__vo uint32_t RES_01;
	__vo uint32_t TSSSR;
	__vo uint32_t CALR;
	__vo uint32_t TAFCR;
	__vo uint32_t ALRMASSR;
	__vo uint32_t ALRMBSSR;
	__vo uint32_t RTC_BKP0R;
	__vo uint32_t RTC_BKP1R;
	__vo uint32_t RTC_BKP2R;
	__vo uint32_t RTC_BKP3R;
	__vo uint32_t RTC_BKP4R;
	__vo uint32_t RTC_BKP5R;
	__vo uint32_t RTC_BKP6R;
	__vo uint32_t RTC_BKP7R;
	__vo uint32_t RTC_BKP8R;
	__vo uint32_t RTC_BKP9R;
	__vo uint32_t RTC_BKP10R;
	__vo uint32_t RTC_BKP11R;
	__vo uint32_t RTC_BKP12R;
	__vo uint32_t RTC_BKP13R;
	__vo uint32_t RTC_BKP14R;
	__vo uint32_t RTC_BKP15R;
	__vo uint32_t RTC_BKP16R;
	__vo uint32_t RTC_BKP17R;
	__vo uint32_t RTC_BKP18R;
	__vo uint32_t RTC_BKP19R;
}RTC_RegDef_t;

/* I2C Structure */
typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
}I2C_RegDef_t;


/* SPI Structure */
typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;

/* SAI A Structure */
typedef struct {
	__vo uint32_t Nop;
	__vo uint32_t xCR1;
	__vo uint32_t xCR2;
	__vo uint32_t xFRCR;
	__vo uint32_t xSLOTR;
	__vo uint32_t xIM;
	__vo uint32_t xSR;
	__vo uint32_t xCLRFR;
	__vo uint32_t xDR;
}SAI_A_RegDef_t;

/* SAI B Structure*/
typedef struct {
	__vo uint32_t Nop[9];
	__vo uint32_t xCR1;
	__vo uint32_t xCR2;
	__vo uint32_t xFRCR;
	__vo uint32_t xSLOTR;
	__vo uint32_t xIM;
	__vo uint32_t xSR;
	__vo uint32_t xCLRFR;
	__vo uint32_t xDR;
}SAI_B_RegDef_t;

/*USART Structure*/
typedef struct {
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
}USART_RegDef_t;

/*SDIO Secure Digital Input Output : Skipped */











/*
 * Peripheral definition macros
 */
// GPIOx
#define GPIOA			((GPIOx_RegDef_t*)GPIOA_BASE_ADDRESS)
#define GPIOB			((GPIOx_RegDef_t*)GPIOB_BASE_ADDRESS)
#define GPIOC			((GPIOx_RegDef_t*)GPIOC_BASE_ADDRESS)
#define GPIOD			((GPIOx_RegDef_t*)GPIOD_BASE_ADDRESS)
#define GPIOE			((GPIOx_RegDef_t*)GPIOE_BASE_ADDRESS)
#define GPIOF			((GPIOx_RegDef_t*)GPIOF_BASE_ADDRESS)
#define GPIOG			((GPIOx_RegDef_t*)GPIOG_BASE_ADDRESS)
#define GPIOH			((GPIOx_RegDef_t*)GPIOH_BASE_ADDRESS)
#define GPIOI			((GPIOx_RegDef_t*)GPIOI_BASE_ADDRESS)
#define GPIOJ			((GPIOx_RegDef_t*)GPIOJ_BASE_ADDRESS)
#define GPIOK			((GPIOx_RegDef_t*)GPIOK_BASE_ADDRESS)


#define RCC				((RCC_RegDef_t*)RCC_BASE_ADDRESS)
/*
 * Clock enable for MCU peripherals
 */

/* GPIO peripherals */
#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()			(RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN()			(RCC->AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN()			(RCC->AHB1ENR |= (1 << 10))

/*
 * Clock disable for MCU peripherals
 */
/*GPIO peripherals*/
#define GPIOA_PCLK_DI()			(RCC->AHB1ENR &= ~(0 << 0))
#define GPIOB_PCLK_DI()			(RCC->AHB1ENR &= ~(0 << 1))
#define GPIOC_PCLK_DI()			(RCC->AHB1ENR &= ~(0 << 2))
#define GPIOD_PCLK_DI()			(RCC->AHB1ENR &= ~(0 << 3))
#define GPIOE_PCLK_DI()			(RCC->AHB1ENR &= ~(0 << 4))
#define GPIOF_PCLK_DI()			(RCC->AHB1ENR &= ~(0 << 5))
#define GPIOG_PCLK_DI()			(RCC->AHB1ENR &= ~(0 << 6))
#define GPIOH_PCLK_DI()			(RCC->AHB1ENR &= ~(0 << 7))
#define GPIOI_PCLK_DI()			(RCC->AHB1ENR &= ~(0 << 8))
#define GPIOJ_PCLK_DI()			(RCC->AHB1ENR &= ~(0 << 9))
#define GPIOK_PCLK_DI()			(RCC->AHB1ENR &= ~(0 << 10))


/*
 *  Macros to Reset GPIOx Registers
 */
#define GPIOA_REG_RST()			do{RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); }while(0)
#define GPIOB_REG_RST()			do{RCC->AHB1RSTR |= (1 << 1); RCC->AHB1RSTR &= ~(1 << 1); }while(0)
#define GPIOC_REG_RST()			do{RCC->AHB1RSTR |= (1 << 2); RCC->AHB1RSTR &= ~(1 << 2); }while(0)
#define GPIOD_REG_RST()			do{RCC->AHB1RSTR |= (1 << 3); RCC->AHB1RSTR &= ~(1 << 3); }while(0)
#define GPIOE_REG_RST()			do{RCC->AHB1RSTR |= (1 << 4); RCC->AHB1RSTR &= ~(1 << 4); }while(0)
#define GPIOF_REG_RST()			do{RCC->AHB1RSTR |= (1 << 5); RCC->AHB1RSTR &= ~(1 << 5); }while(0)
#define GPIOG_REG_RST()			do{RCC->AHB1RSTR |= (1 << 6); RCC->AHB1RSTR &= ~(1 << 6); }while(0)
#define GPIOH_REG_RST()			do{RCC->AHB1RSTR |= (1 << 7); RCC->AHB1RSTR &= ~(1 << 7); }while(0)
#define GPIOI_REG_RST()			do{RCC->AHB1RSTR |= (1 << 8); RCC->AHB1RSTR &= ~(1 << 8); }while(0)
#define GPIOJ_REG_RST()			do{RCC->AHB1RSTR |= (1 << 9); RCC->AHB1RSTR &= ~(1 << 9); }while(0)
#define GPIOK_REG_RST()			do{RCC->AHB1RSTR |= (1 << 10); RCC->AHB1RSTR &= ~(1 << 10); }while(0)

#define GPIO_PORT_EXTI_VALUE(x)	((x == GPIOA)?0:\
				(x == GPIOB)?1:\
				(x == GPIOC)?2:\
				(x == GPIOD)?3:\
				(x == GPIOE)?4:\
				(x == GPIOF)?5:\
				(x == GPIOG)?6:\
				(x == GPIOH)?7:\
				(x == GPIOI)?8:\
				(x == GPIOJ)?9:\
				(x == GPIOK)?10:0)


/* EXTI Peripheral Macro definition*/
#define EXTI				((EXTI_RegDef_t*)EXTI_BASE_ADDRESS)

/*EXTI IRQ number definition*/
#define IRQ_EXTI_0				6
#define IRQ_EXTI_1				7
#define IRQ_EXTI_2				8
#define IRQ_EXTI_3				9
#define IRQ_EXTI_4				10
#define IRQ_EXTI_9_5				23
#define IRQ_EXTI_15_10				40


/* SYSCFG Peripheral Macro definition */
#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDRESS)

/*SYSCFG Clock Enable Macro*/
#define SYSCFG_REG_EN()				(RCC->APB2ENR |= (1 << 14))



/*
 * Global macros
 */
#define ENABLE					1
#define DISABLE					0
#define SET					ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET


#endif /* INC_STM32F429XX_H_ */

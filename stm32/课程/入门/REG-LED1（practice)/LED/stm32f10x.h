#ifndef __STM32F10X_H
#define __STM32F10X_H

#define PERIPH_BASE          (unsigned int)0x40000000
	
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE+0x10000)
#define AHBPERIPH_BASE       (PERIPH_BASE+0x20000)

#define RCC_BASE       (AHBPERIPH_BASE+0x1000)

#define GPIOB_BASE       (APB2PERIPH_BASE+0x0C00)

typedef unsigned int uint32_t;
typedef unsigned short uint16_t;

#define _IO  volatile

typedef struct
{
	_IO uint32_t CR;
	_IO uint32_t CFGR;
	_IO uint32_t CIR;
	_IO uint32_t APB2RSTR;
	_IO uint32_t APB1RSTR;
	_IO uint32_t AHBENR;
	_IO uint32_t APB2ENR;
	_IO uint32_t APB1ENR;
	_IO uint32_t BDCR;
	_IO uint32_t CSR;
}RCC_TypeDef;

typedef struct
{
	_IO uint32_t CRL;
	_IO uint32_t CRH;
	_IO uint32_t IDR;
	_IO uint32_t ODR;
	_IO uint32_t BSRR;
	_IO uint32_t BRR;
	_IO uint32_t LCKR;
}GPIO_TypeDef;

#define GPIOB     ((GPIO_TypeDef*)GPIOB_BASE)
#define RCC       ((RCC_TypeDef*)RCC_BASE)

#endif /* __STM32F10X_H */

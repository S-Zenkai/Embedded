#ifndef STM32F10X_H
#define STM32F10X_H
//#if 0
//外设与总线基地址
#define PHRIPH_BASE    (unsigned int) 0x40000000
#define APB1PHRIPH_BASE    PHRIPH_BASE
#define APB2PHRIPH_BASE    (PHRIPH_BASE + 0x10000)
#define AHBPHRIPH_BASE    (PHRIPH_BASE + 0x20000)

//GPIOB与时钟基地址
#define GPIO_BASE    (APB2PHRIPH_BASE + 0x0C00)
#define RCC_BASE    (AHBPHRIPH_BASE + 0x1000)

////寄存器地址
//#define GPIO_CRL    *(unsigned int*)(GPIO_BASE + 0)
//#define GPIO_CRH    *(unsigned int*)(GPIO_BASE + 0x04)
//#define GPIO_ODR    *(unsigned int*)(GPIO_BASE + 0x0C)


#define RCC_APB2ENR    *(unsigned int*)(RCC_BASE + 0x18)

//#elseif 1


//GPIO寄存器结构体声明
typedef unsigned int    uint32_t;
typedef unsigned short    uint16_t;
typedef struct
{
	uint32_t CRL;
	uint32_t CRH;
	uint32_t IDR;
	uint32_t ODR;
	uint32_t BSRR;
	uint32_t BRR;
	uint32_t LCKR;
}GPIO_Typedef;
#define GPIOB     ((GPIO_Typedef*) GPIO_BASE)

//RCC外设的寄存器结构体声明
typedef struct
{
	uint32_t CR;
	uint32_t CFGR;
	uint32_t CIR;
	uint32_t APB2RSTR;
	uint32_t APB1RSTR;
	uint32_t AHBENR;
	uint32_t APB2ENR;
	uint32_t APB1ENR;
	uint32_t BDCR;
	uint32_t CSR;
}RCC_Typedef;
#define RCC   ((RCC_Typedef*) RCC_BASE)



//#endif
#endif //STM32F10X_H

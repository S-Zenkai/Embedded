

#define PHRIPH_BASE       ((unsigned int) 0x40000000)
#define PHB1PHRIPH_BASE   (PHRIPH_BASE)
#define PHB2PHRIPH_BASE   (PHRIPH_BASE + 0x10000)
#define AHBPHRIPH_BASE    (PHRIPH_BASE + 0x20000)


#define RCC_BASE          (AHBPHRIPH_BASE + 0x1000)
#define GPIOB_BASE        (PHB2PHRIPH_BASE + 0x0C00)

#define RCC_APB2ENR      *((unsigned int*) (RCC_BASE+0x18))

#define GPIOB_CRL        *((unsigned int*) (GPIOB_BASE+0))
#define GPIOB_CRH        *((unsigned int*) (GPIOB_BASE+0x04))
#define GPIOB_ODR        *((unsigned int*) (GPIOB_BASE+0x0C))



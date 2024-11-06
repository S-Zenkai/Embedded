# 1 "..\\Drivers\\BSP\\PWM\\bsp_pwm.c"
# 1 "..\\Drivers\\BSP\\PWM\\bsp_pwm.h"
# 1 "..\\Drivers\\CMSIS\\stm32f10x.h"






































 



 



 
    






  


 
  


 

# 74 "..\\Drivers\\CMSIS\\stm32f10x.h"


















 





# 106 "..\\Drivers\\CMSIS\\stm32f10x.h"







            
# 121 "..\\Drivers\\CMSIS\\stm32f10x.h"




 










 
# 145 "..\\Drivers\\CMSIS\\stm32f10x.h"



 



 



 
# 165 "..\\Drivers\\CMSIS\\stm32f10x.h"




 
typedef enum IRQn
{
 
  NonMaskableInt_IRQn         = -14,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      

 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMPER_IRQn                 = 2,       
  RTC_IRQn                    = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Channel1_IRQn          = 11,      
  DMA1_Channel2_IRQn          = 12,      
  DMA1_Channel3_IRQn          = 13,      
  DMA1_Channel4_IRQn          = 14,      
  DMA1_Channel5_IRQn          = 15,      
  DMA1_Channel6_IRQn          = 16,      
  DMA1_Channel7_IRQn          = 17,      

# 224 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 245 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 273 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 299 "..\\Drivers\\CMSIS\\stm32f10x.h"


  ADC1_2_IRQn                 = 18,      
  USB_HP_CAN1_TX_IRQn         = 19,      
  USB_LP_CAN1_RX0_IRQn        = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_IRQn               = 24,      
  TIM1_UP_IRQn                = 25,      
  TIM1_TRG_COM_IRQn           = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,      
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  USART3_IRQn                 = 39,      
  EXTI15_10_IRQn              = 40,      
  RTCAlarm_IRQn               = 41,      
  USBWakeUp_IRQn              = 42,      
  TIM8_BRK_IRQn               = 43,      
  TIM8_UP_IRQn                = 44,      
  TIM8_TRG_COM_IRQn           = 45,      
  TIM8_CC_IRQn                = 46,      
  ADC3_IRQn                   = 47,      
  FSMC_IRQn                   = 48,      
  SDIO_IRQn                   = 49,      
  TIM5_IRQn                   = 50,      
  SPI3_IRQn                   = 51,      
  UART4_IRQn                  = 52,      
  UART5_IRQn                  = 53,      
  TIM6_IRQn                   = 54,      
  TIM7_IRQn                   = 55,      
  DMA2_Channel1_IRQn          = 56,      
  DMA2_Channel2_IRQn          = 57,      
  DMA2_Channel3_IRQn          = 58,      
  DMA2_Channel4_5_IRQn        = 59       


# 384 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 429 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 475 "..\\Drivers\\CMSIS\\stm32f10x.h"
} IRQn_Type;



 

# 1 "..\\Drivers\\CMSIS\\core_cm3.h"
 




















 





































 

 
 
 
 
 
 
 
 








 











# 1 "D:\\AppData\\keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
# 27 "D:\\AppData\\keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











# 46 "D:\\AppData\\keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
# 216 "D:\\AppData\\keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



# 241 "D:\\AppData\\keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











# 305 "D:\\AppData\\keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
# 91 "..\\Drivers\\CMSIS\\core_cm3.h"

















 

# 117 "..\\Drivers\\CMSIS\\core_cm3.h"





 


 





 
typedef struct
{
  volatile uint32_t ISER[8];                       
       uint32_t RESERVED0[24];                                   
  volatile uint32_t ICER[8];                       
       uint32_t RSERVED1[24];                                    
  volatile uint32_t ISPR[8];                       
       uint32_t RESERVED2[24];                                   
  volatile uint32_t ICPR[8];                       
       uint32_t RESERVED3[24];                                   
  volatile uint32_t IABR[8];                       
       uint32_t RESERVED4[56];                                   
  volatile uint8_t  IP[240];                       
       uint32_t RESERVED5[644];                                  
  volatile  uint32_t STIR;                          
}  NVIC_Type;                                               
   





 
typedef struct
{
  volatile const  uint32_t CPUID;                         
  volatile uint32_t ICSR;                          
  volatile uint32_t VTOR;                          
  volatile uint32_t AIRCR;                         
  volatile uint32_t SCR;                           
  volatile uint32_t CCR;                           
  volatile uint8_t  SHP[12];                       
  volatile uint32_t SHCSR;                         
  volatile uint32_t CFSR;                          
  volatile uint32_t HFSR;                          
  volatile uint32_t DFSR;                          
  volatile uint32_t MMFAR;                         
  volatile uint32_t BFAR;                          
  volatile uint32_t AFSR;                          
  volatile const  uint32_t PFR[2];                        
  volatile const  uint32_t DFR;                           
  volatile const  uint32_t ADR;                           
  volatile const  uint32_t MMFR[4];                       
  volatile const  uint32_t ISAR[5];                       
} SCB_Type;                                                

 












 






























 






 





















 









 


















 
































                                     









 









 









 














   





 
typedef struct
{
  volatile uint32_t CTRL;                          
  volatile uint32_t LOAD;                          
  volatile uint32_t VAL;                           
  volatile const  uint32_t CALIB;                         
} SysTick_Type;

 












 



 



 








   





 
typedef struct
{
  volatile  union  
  {
    volatile  uint8_t    u8;                        
    volatile  uint16_t   u16;                       
    volatile  uint32_t   u32;                       
  }  PORT [32];                                
       uint32_t RESERVED0[864];                                 
  volatile uint32_t TER;                           
       uint32_t RESERVED1[15];                                  
  volatile uint32_t TPR;                           
       uint32_t RESERVED2[15];                                  
  volatile uint32_t TCR;                           
       uint32_t RESERVED3[29];                                  
  volatile uint32_t IWR;                           
  volatile uint32_t IRR;                           
  volatile uint32_t IMCR;                          
       uint32_t RESERVED4[43];                                  
  volatile uint32_t LAR;                           
  volatile uint32_t LSR;                           
       uint32_t RESERVED5[6];                                   
  volatile const  uint32_t PID4;                          
  volatile const  uint32_t PID5;                          
  volatile const  uint32_t PID6;                          
  volatile const  uint32_t PID7;                          
  volatile const  uint32_t PID0;                          
  volatile const  uint32_t PID1;                          
  volatile const  uint32_t PID2;                          
  volatile const  uint32_t PID3;                          
  volatile const  uint32_t CID0;                          
  volatile const  uint32_t CID1;                          
  volatile const  uint32_t CID2;                          
  volatile const  uint32_t CID3;                          
} ITM_Type;                                                

 



 
























 



 



 



 








   





 
typedef struct
{
       uint32_t RESERVED0;
  volatile const  uint32_t ICTR;                          

  volatile uint32_t ACTLR;                         



} InterruptType_Type;

 



 








   


# 614 "..\\Drivers\\CMSIS\\core_cm3.h"





 
typedef struct
{
  volatile uint32_t DHCSR;                         
  volatile  uint32_t DCRSR;                         
  volatile uint32_t DCRDR;                         
  volatile uint32_t DEMCR;                         
} CoreDebug_Type;

 




































 






 






































   


 
# 721 "..\\Drivers\\CMSIS\\core_cm3.h"

# 728 "..\\Drivers\\CMSIS\\core_cm3.h"






   




 





# 758 "..\\Drivers\\CMSIS\\core_cm3.h"


 


 




# 783 "..\\Drivers\\CMSIS\\core_cm3.h"


 
 
 
 








 
extern uint32_t __get_PSP(void);








 
extern void __set_PSP(uint32_t topOfProcStack);








 
extern uint32_t __get_MSP(void);








 
extern void __set_MSP(uint32_t topOfMainStack);








 
extern uint32_t __REV16(uint16_t value);








 
extern int32_t __REVSH(int16_t value);


# 933 "..\\Drivers\\CMSIS\\core_cm3.h"





 








 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}







 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xff);
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}







 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}







 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & 1);
}







 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}





# 1445 "..\\Drivers\\CMSIS\\core_cm3.h"







 
 

 











 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);                          
  
  reg_value  =  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR;                                                    
  reg_value &= ~((0xFFFFul << 16) | (7ul << 8));              
  reg_value  =  (reg_value                       |
                (0x5FA << 16) | 
                (PriorityGroupTmp << 8));                                      
  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR =  reg_value;
}








 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR & (7ul << 8)) >> 8);    
}








 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}









 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000) + 0x0100))->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}








 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}









 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((((NVIC_Type *) ((0xE000E000) + 0x0100))->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}












 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 4)) & 0xff); }  
  else {
    ((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)] = ((priority << (8 - 4)) & 0xff);    }         
}















 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 4)));  }  
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)]           >> (8 - 4)));  }  
}
















 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;
 
  return (
           ((PreemptPriority & ((1 << (PreemptPriorityBits)) - 1)) << SubPriorityBits) |
           ((SubPriority     & ((1 << (SubPriorityBits    )) - 1)))
         );
}
















 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;
  
  *pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
  *pSubPriority     = (Priority                   ) & ((1 << (SubPriorityBits    )) - 1);
}



 












 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{ 
  if (ticks > (0xFFFFFFul << 0))  return (1);             
                                                               
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->LOAD  = (ticks & (0xFFFFFFul << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<4) - 1);   
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->CTRL  = (1ul << 2) | 
                   (1ul << 1)   | 
                   (1ul << 0);                     
  return (0);                                                   
}






 





 
static __inline void NVIC_SystemReset(void)
{
  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR  = ((0x5FA << 16)      | 
                 (((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR & (7ul << 8)) | 
                 (1ul << 2));                    
  __dsb(0);                                                                    
  while(1);                                                     
}

   



 






 
 

extern volatile int ITM_RxBuffer;                     












 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if ((((CoreDebug_Type *) (0xE000EDF0))->DEMCR & (1ul << 24))  &&       
      (((ITM_Type *) (0xE0000000))->TCR & (1ul << 0))                  &&       
      (((ITM_Type *) (0xE0000000))->TER & (1ul << 0)        )                    )      
  {
    while (((ITM_Type *) (0xE0000000))->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000))->PORT[0].u8 = (uint8_t) ch;
  }  
  return (ch);
}










 
static __inline int ITM_ReceiveChar (void) {
  int ch = -1;                                

  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;        
  }
  
  return (ch); 
}









 
static __inline int ITM_CheckChar (void) {

  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);                                  
  } else {
    return (1);                                  
  }
}

   






   



 
# 482 "..\\Drivers\\CMSIS\\stm32f10x.h"
# 1 "..\\Drivers\\CMSIS\\system_stm32f10x.h"


















 



 



   
  


 









 



 




 

extern uint32_t SystemCoreClock;           



 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
# 483 "..\\Drivers\\CMSIS\\stm32f10x.h"
# 484 "..\\Drivers\\CMSIS\\stm32f10x.h"



   

 
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;   
typedef const int16_t sc16;   
typedef const int8_t sc8;    

typedef volatile int32_t  vs32;
typedef volatile int16_t  vs16;
typedef volatile int8_t   vs8;

typedef volatile const int32_t vsc32;   
typedef volatile const int16_t vsc16;   
typedef volatile const int8_t vsc8;    

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;   
typedef const uint16_t uc16;   
typedef const uint8_t uc8;    

typedef volatile uint32_t  vu32;
typedef volatile uint16_t vu16;
typedef volatile uint8_t  vu8;

typedef volatile const uint32_t vuc32;   
typedef volatile const uint16_t vuc16;   
typedef volatile const uint8_t vuc8;    

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

 





 



    



 

typedef struct
{
  volatile uint32_t SR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t JOFR1;
  volatile uint32_t JOFR2;
  volatile uint32_t JOFR3;
  volatile uint32_t JOFR4;
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t JSQR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  volatile uint32_t DR;
} ADC_TypeDef;



 

typedef struct
{
  uint32_t  RESERVED0;
  volatile uint16_t DR1;
  uint16_t  RESERVED1;
  volatile uint16_t DR2;
  uint16_t  RESERVED2;
  volatile uint16_t DR3;
  uint16_t  RESERVED3;
  volatile uint16_t DR4;
  uint16_t  RESERVED4;
  volatile uint16_t DR5;
  uint16_t  RESERVED5;
  volatile uint16_t DR6;
  uint16_t  RESERVED6;
  volatile uint16_t DR7;
  uint16_t  RESERVED7;
  volatile uint16_t DR8;
  uint16_t  RESERVED8;
  volatile uint16_t DR9;
  uint16_t  RESERVED9;
  volatile uint16_t DR10;
  uint16_t  RESERVED10; 
  volatile uint16_t RTCCR;
  uint16_t  RESERVED11;
  volatile uint16_t CR;
  uint16_t  RESERVED12;
  volatile uint16_t CSR;
  uint16_t  RESERVED13[5];
  volatile uint16_t DR11;
  uint16_t  RESERVED14;
  volatile uint16_t DR12;
  uint16_t  RESERVED15;
  volatile uint16_t DR13;
  uint16_t  RESERVED16;
  volatile uint16_t DR14;
  uint16_t  RESERVED17;
  volatile uint16_t DR15;
  uint16_t  RESERVED18;
  volatile uint16_t DR16;
  uint16_t  RESERVED19;
  volatile uint16_t DR17;
  uint16_t  RESERVED20;
  volatile uint16_t DR18;
  uint16_t  RESERVED21;
  volatile uint16_t DR19;
  uint16_t  RESERVED22;
  volatile uint16_t DR20;
  uint16_t  RESERVED23;
  volatile uint16_t DR21;
  uint16_t  RESERVED24;
  volatile uint16_t DR22;
  uint16_t  RESERVED25;
  volatile uint16_t DR23;
  uint16_t  RESERVED26;
  volatile uint16_t DR24;
  uint16_t  RESERVED27;
  volatile uint16_t DR25;
  uint16_t  RESERVED28;
  volatile uint16_t DR26;
  uint16_t  RESERVED29;
  volatile uint16_t DR27;
  uint16_t  RESERVED30;
  volatile uint16_t DR28;
  uint16_t  RESERVED31;
  volatile uint16_t DR29;
  uint16_t  RESERVED32;
  volatile uint16_t DR30;
  uint16_t  RESERVED33; 
  volatile uint16_t DR31;
  uint16_t  RESERVED34;
  volatile uint16_t DR32;
  uint16_t  RESERVED35;
  volatile uint16_t DR33;
  uint16_t  RESERVED36;
  volatile uint16_t DR34;
  uint16_t  RESERVED37;
  volatile uint16_t DR35;
  uint16_t  RESERVED38;
  volatile uint16_t DR36;
  uint16_t  RESERVED39;
  volatile uint16_t DR37;
  uint16_t  RESERVED40;
  volatile uint16_t DR38;
  uint16_t  RESERVED41;
  volatile uint16_t DR39;
  uint16_t  RESERVED42;
  volatile uint16_t DR40;
  uint16_t  RESERVED43;
  volatile uint16_t DR41;
  uint16_t  RESERVED44;
  volatile uint16_t DR42;
  uint16_t  RESERVED45;    
} BKP_TypeDef;
  


 

typedef struct
{
  volatile uint32_t TIR;
  volatile uint32_t TDTR;
  volatile uint32_t TDLR;
  volatile uint32_t TDHR;
} CAN_TxMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RIR;
  volatile uint32_t RDTR;
  volatile uint32_t RDLR;
  volatile uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t FR1;
  volatile uint32_t FR2;
} CAN_FilterRegister_TypeDef;



 
  
typedef struct
{
  volatile uint32_t MCR;
  volatile uint32_t MSR;
  volatile uint32_t TSR;
  volatile uint32_t RF0R;
  volatile uint32_t RF1R;
  volatile uint32_t IER;
  volatile uint32_t ESR;
  volatile uint32_t BTR;
  uint32_t  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  uint32_t  RESERVED1[12];
  volatile uint32_t FMR;
  volatile uint32_t FM1R;
  uint32_t  RESERVED2;
  volatile uint32_t FS1R;
  uint32_t  RESERVED3;
  volatile uint32_t FFA1R;
  uint32_t  RESERVED4;
  volatile uint32_t FA1R;
  uint32_t  RESERVED5[8];

  CAN_FilterRegister_TypeDef sFilterRegister[14];



} CAN_TypeDef;



 
typedef struct
{
  volatile uint32_t CFGR;
  volatile uint32_t OAR;
  volatile uint32_t PRES;
  volatile uint32_t ESR;
  volatile uint32_t CSR;
  volatile uint32_t TXD;
  volatile uint32_t RXD;  
} CEC_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;
  volatile uint8_t  IDR;
  uint8_t   RESERVED0;
  uint16_t  RESERVED1;
  volatile uint32_t CR;
} CRC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t SWTRIGR;
  volatile uint32_t DHR12R1;
  volatile uint32_t DHR12L1;
  volatile uint32_t DHR8R1;
  volatile uint32_t DHR12R2;
  volatile uint32_t DHR12L2;
  volatile uint32_t DHR8R2;
  volatile uint32_t DHR12RD;
  volatile uint32_t DHR12LD;
  volatile uint32_t DHR8RD;
  volatile uint32_t DOR1;
  volatile uint32_t DOR2;



} DAC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;
  volatile uint32_t CR;	
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CCR;
  volatile uint32_t CNDTR;
  volatile uint32_t CPAR;
  volatile uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  volatile uint32_t ISR;
  volatile uint32_t IFCR;
} DMA_TypeDef;



 

typedef struct
{
  volatile uint32_t MACCR;
  volatile uint32_t MACFFR;
  volatile uint32_t MACHTHR;
  volatile uint32_t MACHTLR;
  volatile uint32_t MACMIIAR;
  volatile uint32_t MACMIIDR;
  volatile uint32_t MACFCR;
  volatile uint32_t MACVLANTR;              
       uint32_t RESERVED0[2];
  volatile uint32_t MACRWUFFR;              
  volatile uint32_t MACPMTCSR;
       uint32_t RESERVED1[2];
  volatile uint32_t MACSR;                  
  volatile uint32_t MACIMR;
  volatile uint32_t MACA0HR;
  volatile uint32_t MACA0LR;
  volatile uint32_t MACA1HR;
  volatile uint32_t MACA1LR;
  volatile uint32_t MACA2HR;
  volatile uint32_t MACA2LR;
  volatile uint32_t MACA3HR;
  volatile uint32_t MACA3LR;                
       uint32_t RESERVED2[40];
  volatile uint32_t MMCCR;                  
  volatile uint32_t MMCRIR;
  volatile uint32_t MMCTIR;
  volatile uint32_t MMCRIMR;
  volatile uint32_t MMCTIMR;                
       uint32_t RESERVED3[14];
  volatile uint32_t MMCTGFSCCR;             
  volatile uint32_t MMCTGFMSCCR;
       uint32_t RESERVED4[5];
  volatile uint32_t MMCTGFCR;
       uint32_t RESERVED5[10];
  volatile uint32_t MMCRFCECR;
  volatile uint32_t MMCRFAECR;
       uint32_t RESERVED6[10];
  volatile uint32_t MMCRGUFCR;
       uint32_t RESERVED7[334];
  volatile uint32_t PTPTSCR;
  volatile uint32_t PTPSSIR;
  volatile uint32_t PTPTSHR;
  volatile uint32_t PTPTSLR;
  volatile uint32_t PTPTSHUR;
  volatile uint32_t PTPTSLUR;
  volatile uint32_t PTPTSAR;
  volatile uint32_t PTPTTHR;
  volatile uint32_t PTPTTLR;
       uint32_t RESERVED8[567];
  volatile uint32_t DMABMR;
  volatile uint32_t DMATPDR;
  volatile uint32_t DMARPDR;
  volatile uint32_t DMARDLAR;
  volatile uint32_t DMATDLAR;
  volatile uint32_t DMASR;
  volatile uint32_t DMAOMR;
  volatile uint32_t DMAIER;
  volatile uint32_t DMAMFBOCR;
       uint32_t RESERVED9[9];
  volatile uint32_t DMACHTDR;
  volatile uint32_t DMACHRDR;
  volatile uint32_t DMACHTBAR;
  volatile uint32_t DMACHRBAR;
} ETH_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t AR;
  volatile uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
# 923 "..\\Drivers\\CMSIS\\stm32f10x.h"
} FLASH_TypeDef;



 
  
typedef struct
{
  volatile uint16_t RDP;
  volatile uint16_t USER;
  volatile uint16_t Data0;
  volatile uint16_t Data1;
  volatile uint16_t WRP0;
  volatile uint16_t WRP1;
  volatile uint16_t WRP2;
  volatile uint16_t WRP3;
} OB_TypeDef;



 

typedef struct
{
  volatile uint32_t BTCR[8];   
} FSMC_Bank1_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t BWTR[7];
} FSMC_Bank1E_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR2;
  volatile uint32_t SR2;
  volatile uint32_t PMEM2;
  volatile uint32_t PATT2;
  uint32_t  RESERVED0;   
  volatile uint32_t ECCR2; 
} FSMC_Bank2_TypeDef;  



 
  
typedef struct
{
  volatile uint32_t PCR3;
  volatile uint32_t SR3;
  volatile uint32_t PMEM3;
  volatile uint32_t PATT3;
  uint32_t  RESERVED0;   
  volatile uint32_t ECCR3; 
} FSMC_Bank3_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t PCR4;
  volatile uint32_t SR4;
  volatile uint32_t PMEM4;
  volatile uint32_t PATT4;
  volatile uint32_t PIO4; 
} FSMC_Bank4_TypeDef; 



 

typedef struct
{
  volatile uint32_t CRL;
  volatile uint32_t CRH;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t BRR;
  volatile uint32_t LCKR;
} GPIO_TypeDef;



 

typedef struct
{
  volatile uint32_t EVCR;
  volatile uint32_t MAPR;
  volatile uint32_t EXTICR[4];
  uint32_t RESERVED0;
  volatile uint32_t MAPR2;  
} AFIO_TypeDef;


 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t OAR1;
  uint16_t  RESERVED2;
  volatile uint16_t OAR2;
  uint16_t  RESERVED3;
  volatile uint16_t DR;
  uint16_t  RESERVED4;
  volatile uint16_t SR1;
  uint16_t  RESERVED5;
  volatile uint16_t SR2;
  uint16_t  RESERVED6;
  volatile uint16_t CCR;
  uint16_t  RESERVED7;
  volatile uint16_t TRISE;
  uint16_t  RESERVED8;
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
} IWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CSR;
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;










} RCC_TypeDef;



 

typedef struct
{
  volatile uint16_t CRH;
  uint16_t  RESERVED0;
  volatile uint16_t CRL;
  uint16_t  RESERVED1;
  volatile uint16_t PRLH;
  uint16_t  RESERVED2;
  volatile uint16_t PRLL;
  uint16_t  RESERVED3;
  volatile uint16_t DIVH;
  uint16_t  RESERVED4;
  volatile uint16_t DIVL;
  uint16_t  RESERVED5;
  volatile uint16_t CNTH;
  uint16_t  RESERVED6;
  volatile uint16_t CNTL;
  uint16_t  RESERVED7;
  volatile uint16_t ALRH;
  uint16_t  RESERVED8;
  volatile uint16_t ALRL;
  uint16_t  RESERVED9;
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t POWER;
  volatile uint32_t CLKCR;
  volatile uint32_t ARG;
  volatile uint32_t CMD;
  volatile const uint32_t RESPCMD;
  volatile const uint32_t RESP1;
  volatile const uint32_t RESP2;
  volatile const uint32_t RESP3;
  volatile const uint32_t RESP4;
  volatile uint32_t DTIMER;
  volatile uint32_t DLEN;
  volatile uint32_t DCTRL;
  volatile const uint32_t DCOUNT;
  volatile const uint32_t STA;
  volatile uint32_t ICR;
  volatile uint32_t MASK;
  uint32_t  RESERVED0[2];
  volatile const uint32_t FIFOCNT;
  uint32_t  RESERVED1[13];
  volatile uint32_t FIFO;
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SR;
  uint16_t  RESERVED2;
  volatile uint16_t DR;
  uint16_t  RESERVED3;
  volatile uint16_t CRCPR;
  uint16_t  RESERVED4;
  volatile uint16_t RXCRCR;
  uint16_t  RESERVED5;
  volatile uint16_t TXCRCR;
  uint16_t  RESERVED6;
  volatile uint16_t I2SCFGR;
  uint16_t  RESERVED7;
  volatile uint16_t I2SPR;
  uint16_t  RESERVED8;  
} SPI_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SMCR;
  uint16_t  RESERVED2;
  volatile uint16_t DIER;
  uint16_t  RESERVED3;
  volatile uint16_t SR;
  uint16_t  RESERVED4;
  volatile uint16_t EGR;
  uint16_t  RESERVED5;
  volatile uint16_t CCMR1;
  uint16_t  RESERVED6;
  volatile uint16_t CCMR2;
  uint16_t  RESERVED7;
  volatile uint16_t CCER;
  uint16_t  RESERVED8;
  volatile uint16_t CNT;
  uint16_t  RESERVED9;
  volatile uint16_t PSC;
  uint16_t  RESERVED10;
  volatile uint16_t ARR;
  uint16_t  RESERVED11;
  volatile uint16_t RCR;
  uint16_t  RESERVED12;
  volatile uint16_t CCR1;
  uint16_t  RESERVED13;
  volatile uint16_t CCR2;
  uint16_t  RESERVED14;
  volatile uint16_t CCR3;
  uint16_t  RESERVED15;
  volatile uint16_t CCR4;
  uint16_t  RESERVED16;
  volatile uint16_t BDTR;
  uint16_t  RESERVED17;
  volatile uint16_t DCR;
  uint16_t  RESERVED18;
  volatile uint16_t DMAR;
  uint16_t  RESERVED19;
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint16_t SR;
  uint16_t  RESERVED0;
  volatile uint16_t DR;
  uint16_t  RESERVED1;
  volatile uint16_t BRR;
  uint16_t  RESERVED2;
  volatile uint16_t CR1;
  uint16_t  RESERVED3;
  volatile uint16_t CR2;
  uint16_t  RESERVED4;
  volatile uint16_t CR3;
  uint16_t  RESERVED5;
  volatile uint16_t GTPR;
  uint16_t  RESERVED6;
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
} WWDG_TypeDef;



 
  


 











 




# 1315 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 1338 "..\\Drivers\\CMSIS\\stm32f10x.h"



# 1357 "..\\Drivers\\CMSIS\\stm32f10x.h"




















 
  


   

# 1457 "..\\Drivers\\CMSIS\\stm32f10x.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 

 



 



 


 
 
 
 
 

 











 
# 1518 "..\\Drivers\\CMSIS\\stm32f10x.h"




 





 
 
 
 
 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 





 



 






 
 
 
 
 

 
# 1694 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 1701 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
 








 








 






# 1737 "..\\Drivers\\CMSIS\\stm32f10x.h"

 











 











 













 






# 1853 "..\\Drivers\\CMSIS\\stm32f10x.h"




# 1873 "..\\Drivers\\CMSIS\\stm32f10x.h"

 





# 1886 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 1905 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 1914 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 1922 "..\\Drivers\\CMSIS\\stm32f10x.h"



















# 1947 "..\\Drivers\\CMSIS\\stm32f10x.h"












 













# 1979 "..\\Drivers\\CMSIS\\stm32f10x.h"





# 1993 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 2000 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 2010 "..\\Drivers\\CMSIS\\stm32f10x.h"











 


















# 2046 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 2054 "..\\Drivers\\CMSIS\\stm32f10x.h"



















# 2079 "..\\Drivers\\CMSIS\\stm32f10x.h"












 













# 2111 "..\\Drivers\\CMSIS\\stm32f10x.h"





# 2125 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 2132 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 2142 "..\\Drivers\\CMSIS\\stm32f10x.h"











 








 








   
# 2181 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 2276 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 2303 "..\\Drivers\\CMSIS\\stm32f10x.h"
 
 
 
 
 
 

 




































































 




































































 
# 2465 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 2483 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 2501 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 2518 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 2536 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 2555 "..\\Drivers\\CMSIS\\stm32f10x.h"

 

 






 
# 2582 "..\\Drivers\\CMSIS\\stm32f10x.h"






 








 









 








 








 









 










 




# 2657 "..\\Drivers\\CMSIS\\stm32f10x.h"

 










# 2688 "..\\Drivers\\CMSIS\\stm32f10x.h"

 





 
# 2703 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 2712 "..\\Drivers\\CMSIS\\stm32f10x.h"

   
# 2721 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 2730 "..\\Drivers\\CMSIS\\stm32f10x.h"

 





 
# 2745 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 2754 "..\\Drivers\\CMSIS\\stm32f10x.h"

   
# 2763 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 2772 "..\\Drivers\\CMSIS\\stm32f10x.h"

 





 
# 2787 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 2796 "..\\Drivers\\CMSIS\\stm32f10x.h"

   
# 2805 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 2814 "..\\Drivers\\CMSIS\\stm32f10x.h"

 





 
# 2829 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 2838 "..\\Drivers\\CMSIS\\stm32f10x.h"

   
# 2847 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 2856 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 2865 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 2874 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 2884 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
 
 
 
 

 





 


 


 




 
 
 
 
 

 
# 2948 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 2983 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 3018 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 3053 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 3088 "..\\Drivers\\CMSIS\\stm32f10x.h"

 





 





 





 





 





 





 





 





 






 
# 3155 "..\\Drivers\\CMSIS\\stm32f10x.h"

 



 









 
# 3179 "..\\Drivers\\CMSIS\\stm32f10x.h"




 




 
# 3195 "..\\Drivers\\CMSIS\\stm32f10x.h"

 





 
# 3217 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
 





 
# 3232 "..\\Drivers\\CMSIS\\stm32f10x.h"
 
# 3239 "..\\Drivers\\CMSIS\\stm32f10x.h"

 




 






 


 


 


 
 
 
 
 

 
# 3288 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 3310 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 3332 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 3354 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 3376 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 3398 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
 
 
 
 

 
# 3434 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 3464 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 3474 "..\\Drivers\\CMSIS\\stm32f10x.h"















 
# 3498 "..\\Drivers\\CMSIS\\stm32f10x.h"















 
# 3522 "..\\Drivers\\CMSIS\\stm32f10x.h"















 
# 3546 "..\\Drivers\\CMSIS\\stm32f10x.h"















 
# 3570 "..\\Drivers\\CMSIS\\stm32f10x.h"















 
# 3594 "..\\Drivers\\CMSIS\\stm32f10x.h"















 
# 3618 "..\\Drivers\\CMSIS\\stm32f10x.h"















 


 


 


 


 


 


 


 


 


 



 


 


 



 


 


 


 



 


 


 


 


 
 
 
 
 

 






 
# 3719 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 3728 "..\\Drivers\\CMSIS\\stm32f10x.h"















  
 
# 3751 "..\\Drivers\\CMSIS\\stm32f10x.h"


















 








































 


















































 


 


 


 


 


 


 
# 3886 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 3893 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 3900 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 3907 "..\\Drivers\\CMSIS\\stm32f10x.h"







 
# 3921 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 3928 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 3935 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 3942 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 3949 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 3956 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 3964 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 3971 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 3978 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 3985 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 3992 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 3999 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 4007 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 4014 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 4021 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 4028 "..\\Drivers\\CMSIS\\stm32f10x.h"





 


 


 


 


 



 
 
 
 
 

 














































 



 


 


 


 


 


 


 



 



 



 


 


 



 
 
 
 
 
 





 






 


 
# 4175 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 4185 "..\\Drivers\\CMSIS\\stm32f10x.h"

 


 


 
 
 
 
 

 
















 









# 4233 "..\\Drivers\\CMSIS\\stm32f10x.h"

 

























 
# 4276 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 4290 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 4300 "..\\Drivers\\CMSIS\\stm32f10x.h"

 




























 





















 




























 





















 
# 4419 "..\\Drivers\\CMSIS\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
# 4454 "..\\Drivers\\CMSIS\\stm32f10x.h"





# 4465 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 4473 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 4480 "..\\Drivers\\CMSIS\\stm32f10x.h"

 


 
 
 
 
 

 




 
# 4502 "..\\Drivers\\CMSIS\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
 
 
 
 

 


 





 


 



 
 
 
 
 

 
# 4564 "..\\Drivers\\CMSIS\\stm32f10x.h"



 
# 4576 "..\\Drivers\\CMSIS\\stm32f10x.h"







 


 
 
 
 
 

 











# 4614 "..\\Drivers\\CMSIS\\stm32f10x.h"

 











# 4637 "..\\Drivers\\CMSIS\\stm32f10x.h"

 











# 4660 "..\\Drivers\\CMSIS\\stm32f10x.h"

 











# 4683 "..\\Drivers\\CMSIS\\stm32f10x.h"

 












# 4706 "..\\Drivers\\CMSIS\\stm32f10x.h"























 












# 4751 "..\\Drivers\\CMSIS\\stm32f10x.h"























 












# 4796 "..\\Drivers\\CMSIS\\stm32f10x.h"























 












# 4841 "..\\Drivers\\CMSIS\\stm32f10x.h"























 












# 4886 "..\\Drivers\\CMSIS\\stm32f10x.h"

















 












# 4925 "..\\Drivers\\CMSIS\\stm32f10x.h"

















 












# 4964 "..\\Drivers\\CMSIS\\stm32f10x.h"

















 












# 5003 "..\\Drivers\\CMSIS\\stm32f10x.h"

















 



























 



























 



























 
# 5112 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 5121 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 5130 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 5141 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5151 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5161 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5171 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 5182 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5192 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5202 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5212 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 5223 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5233 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5243 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5253 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 5264 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5274 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5284 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5294 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 5305 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5315 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5325 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5335 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 5346 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5356 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5366 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5376 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 5387 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5397 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5407 "..\\Drivers\\CMSIS\\stm32f10x.h"

# 5417 "..\\Drivers\\CMSIS\\stm32f10x.h"

 


 


 
 
 
 
 

 




 












 


 






# 5465 "..\\Drivers\\CMSIS\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
















 


 
# 5535 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 5550 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 5576 "..\\Drivers\\CMSIS\\stm32f10x.h"

 


 


 
 
 
 
 

 
 























 























 























 























 























 























 























 























 
 
# 5797 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 5809 "..\\Drivers\\CMSIS\\stm32f10x.h"

 






 
# 5826 "..\\Drivers\\CMSIS\\stm32f10x.h"



     


 
 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


# 5970 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 5982 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 5994 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 6006 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 6018 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 6030 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 6042 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 6054 "..\\Drivers\\CMSIS\\stm32f10x.h"



 

 


# 6068 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 6080 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 6092 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 6104 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 6116 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 6128 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 6140 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 6152 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 6164 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 6176 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 6188 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 6200 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 6212 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 6224 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 6236 "..\\Drivers\\CMSIS\\stm32f10x.h"



 


# 6248 "..\\Drivers\\CMSIS\\stm32f10x.h"



 
 
 
 
 

 
 
# 6268 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 6279 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 6297 "..\\Drivers\\CMSIS\\stm32f10x.h"











 





 





 
# 6335 "..\\Drivers\\CMSIS\\stm32f10x.h"

 












 
# 6356 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
 






 




 





 





 






 




 





 





 






   




 





 





 





 




 





 





 





 




 





 





 
 


 
# 6496 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 6513 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 6530 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 6547 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 6581 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 6615 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 6649 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 6683 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 6717 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 6751 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 6785 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 6819 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 6853 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 6887 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 6921 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 6955 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 6989 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7023 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7057 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7091 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7125 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7159 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7193 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7227 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7261 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7295 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7329 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7363 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7397 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7431 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7465 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7499 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
 
 
 
 

 









# 7526 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7534 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7544 "..\\Drivers\\CMSIS\\stm32f10x.h"

 


 


 


 


 





















 




 
 
 
 
 

 
# 7605 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7614 "..\\Drivers\\CMSIS\\stm32f10x.h"







 



# 7635 "..\\Drivers\\CMSIS\\stm32f10x.h"



 



 


 
# 7660 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7670 "..\\Drivers\\CMSIS\\stm32f10x.h"

 




 


 
 
 
 
 

 
# 7696 "..\\Drivers\\CMSIS\\stm32f10x.h"

 


 



 
# 7720 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7729 "..\\Drivers\\CMSIS\\stm32f10x.h"







 
# 7749 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
# 7760 "..\\Drivers\\CMSIS\\stm32f10x.h"



 
 
 
 
 

 


# 7789 "..\\Drivers\\CMSIS\\stm32f10x.h"

 









# 7823 "..\\Drivers\\CMSIS\\stm32f10x.h"

 
 
 
 
 

 









 


 




 


 





 
# 7868 "..\\Drivers\\CMSIS\\stm32f10x.h"

 


 









 


 

 



 



 



 



 



 



 



 



# 8332 "..\\Drivers\\CMSIS\\stm32f10x.h"



 

 

  

# 1 "..\\User\\stm32f10x_conf.h"



















 

 



 
 
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"



















 

 







 
# 1 "..\\Drivers\\CMSIS\\stm32f10x.h"






































 



 



 
    
# 8372 "..\\Drivers\\CMSIS\\stm32f10x.h"



 

  

 

# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"



 



 



 



 

typedef struct
{
  uint32_t ADC_Mode;                      

 

  FunctionalState ADC_ScanConvMode;       

 

  FunctionalState ADC_ContinuousConvMode; 

 

  uint32_t ADC_ExternalTrigConv;          

 

  uint32_t ADC_DataAlign;                 
 

  uint8_t ADC_NbrOfChannel;               

 
}ADC_InitTypeDef;


 



 










 

# 103 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"

# 114 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 

# 128 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"




# 138 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"

# 153 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 







 



 

# 191 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"




# 204 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 

# 228 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 

















# 265 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 

# 281 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 

# 296 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"

# 304 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 











 



 

# 337 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 





 



 





 



 





 



 





  




 




 



 





 



 





 



 



 



 



 

void ADC_DeInit(ADC_TypeDef* ADCx);
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct);
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct);
void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_ITConfig(ADC_TypeDef* ADCx, uint16_t ADC_IT, FunctionalState NewState);
void ADC_ResetCalibration(ADC_TypeDef* ADCx);
FlagStatus ADC_GetResetCalibrationStatus(ADC_TypeDef* ADCx);
void ADC_StartCalibration(ADC_TypeDef* ADCx);
FlagStatus ADC_GetCalibrationStatus(ADC_TypeDef* ADCx);
void ADC_SoftwareStartConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
FlagStatus ADC_GetSoftwareStartConvStatus(ADC_TypeDef* ADCx);
void ADC_DiscModeChannelCountConfig(ADC_TypeDef* ADCx, uint8_t Number);
void ADC_DiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_RegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_ExternalTrigConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx);
uint32_t ADC_GetDualModeConversionValue(void);
void ADC_AutoInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_InjectedDiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_ExternalTrigInjectedConvConfig(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigInjecConv);
void ADC_ExternalTrigInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_SoftwareStartInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
FlagStatus ADC_GetSoftwareStartInjectedConvCmdStatus(ADC_TypeDef* ADCx);
void ADC_InjectedChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_InjectedSequencerLengthConfig(ADC_TypeDef* ADCx, uint8_t Length);
void ADC_SetInjectedOffset(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel, uint16_t Offset);
uint16_t ADC_GetInjectedConversionValue(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel);
void ADC_AnalogWatchdogCmd(ADC_TypeDef* ADCx, uint32_t ADC_AnalogWatchdog);
void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef* ADCx, uint16_t HighThreshold, uint16_t LowThreshold);
void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel);
void ADC_TempSensorVrefintCmd(FunctionalState NewState);
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
void ADC_ClearFlag(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
ITStatus ADC_GetITStatus(ADC_TypeDef* ADCx, uint16_t ADC_IT);
void ADC_ClearITPendingBit(ADC_TypeDef* ADCx, uint16_t ADC_IT);









 



 



 

# 29 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_bkp.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_bkp.h"



 



 



 



 



 



 







 



 

# 77 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_bkp.h"


 



 

# 127 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_bkp.h"

# 142 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_bkp.h"




 



 



 



 



 

void BKP_DeInit(void);
void BKP_TamperPinLevelConfig(uint16_t BKP_TamperPinLevel);
void BKP_TamperPinCmd(FunctionalState NewState);
void BKP_ITConfig(FunctionalState NewState);
void BKP_RTCOutputConfig(uint16_t BKP_RTCOutputSource);
void BKP_SetRTCCalibrationValue(uint8_t CalibrationValue);
void BKP_WriteBackupRegister(uint16_t BKP_DR, uint16_t Data);
uint16_t BKP_ReadBackupRegister(uint16_t BKP_DR);
FlagStatus BKP_GetFlagStatus(void);
void BKP_ClearFlag(void);
ITStatus BKP_GetITStatus(void);
void BKP_ClearITPendingBit(void);








 



 



 

# 30 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"



 



 



 






 

typedef struct
{
  uint16_t CAN_Prescaler;   
 
  
  uint8_t CAN_Mode;         

 

  uint8_t CAN_SJW;          



 

  uint8_t CAN_BS1;          

 

  uint8_t CAN_BS2;          


 
  
  FunctionalState CAN_TTCM; 

 
  
  FunctionalState CAN_ABOM;  

 

  FunctionalState CAN_AWUM;  

 

  FunctionalState CAN_NART;  

 

  FunctionalState CAN_RFLM;  

 

  FunctionalState CAN_TXFP;  

 
} CAN_InitTypeDef;



 

typedef struct
{
  uint16_t CAN_FilterIdHigh;         

 

  uint16_t CAN_FilterIdLow;          

 

  uint16_t CAN_FilterMaskIdHigh;     


 

  uint16_t CAN_FilterMaskIdLow;      


 

  uint16_t CAN_FilterFIFOAssignment; 
 
  
  uint8_t CAN_FilterNumber;           

  uint8_t CAN_FilterMode;            
 

  uint8_t CAN_FilterScale;           
 

  FunctionalState CAN_FilterActivation; 
 
} CAN_FilterInitTypeDef;



 

typedef struct
{
  uint32_t StdId;  
 

  uint32_t ExtId;  
 

  uint8_t IDE;     

 

  uint8_t RTR;     

 

  uint8_t DLC;     

 

  uint8_t Data[8]; 
 
} CanTxMsg;



 

typedef struct
{
  uint32_t StdId;  
 

  uint32_t ExtId;  
 

  uint8_t IDE;     

 

  uint8_t RTR;     

 

  uint8_t DLC;     
 

  uint8_t Data[8]; 
 

  uint8_t FMI;     

 
} CanRxMsg;



 



 



 






 



 












 





   










 
  



   







 



 










 



 

# 300 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"




 



 

# 318 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"





 



 





 



 







 



 








 



 









 



 







 



 



 



 








 



 







 



 







 



 








 



 








 



 






 



 






 




   
                                                                
# 492 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"




 



 

 
 

 




 
# 517 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"

 



 

 





# 538 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"








 

  


 


  


 
# 564 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"

 



 






 





# 589 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"

# 596 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"



 



 
# 620 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"



 



 



 



 



 
  
void CAN_DeInit(CAN_TypeDef* CANx);

  
uint8_t CAN_Init(CAN_TypeDef* CANx, CAN_InitTypeDef* CAN_InitStruct);
void CAN_FilterInit(CAN_FilterInitTypeDef* CAN_FilterInitStruct);
void CAN_StructInit(CAN_InitTypeDef* CAN_InitStruct);
void CAN_SlaveStartBank(uint8_t CAN_BankNumber); 
void CAN_DBGFreeze(CAN_TypeDef* CANx, FunctionalState NewState);
void CAN_TTComModeCmd(CAN_TypeDef* CANx, FunctionalState NewState);

 
uint8_t CAN_Transmit(CAN_TypeDef* CANx, CanTxMsg* TxMessage);
uint8_t CAN_TransmitStatus(CAN_TypeDef* CANx, uint8_t TransmitMailbox);
void CAN_CancelTransmit(CAN_TypeDef* CANx, uint8_t Mailbox);

 
void CAN_Receive(CAN_TypeDef* CANx, uint8_t FIFONumber, CanRxMsg* RxMessage);
void CAN_FIFORelease(CAN_TypeDef* CANx, uint8_t FIFONumber);
uint8_t CAN_MessagePending(CAN_TypeDef* CANx, uint8_t FIFONumber);


 
uint8_t CAN_OperatingModeRequest(CAN_TypeDef* CANx, uint8_t CAN_OperatingMode);
uint8_t CAN_Sleep(CAN_TypeDef* CANx);
uint8_t CAN_WakeUp(CAN_TypeDef* CANx);

 
uint8_t CAN_GetLastErrorCode(CAN_TypeDef* CANx);
uint8_t CAN_GetReceiveErrorCounter(CAN_TypeDef* CANx);
uint8_t CAN_GetLSBTransmitErrorCounter(CAN_TypeDef* CANx);

 
void CAN_ITConfig(CAN_TypeDef* CANx, uint32_t CAN_IT, FunctionalState NewState);
FlagStatus CAN_GetFlagStatus(CAN_TypeDef* CANx, uint32_t CAN_FLAG);
void CAN_ClearFlag(CAN_TypeDef* CANx, uint32_t CAN_FLAG);
ITStatus CAN_GetITStatus(CAN_TypeDef* CANx, uint32_t CAN_IT);
void CAN_ClearITPendingBit(CAN_TypeDef* CANx, uint32_t CAN_IT);








 



 



 

# 31 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_cec.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_cec.h"



 



 
  



 
   


  
typedef struct
{
  uint16_t CEC_BitTimingMode; 
 
  uint16_t CEC_BitPeriodMode; 
 
}CEC_InitTypeDef;



 



  
  


  







 



  







  




  
# 99 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_cec.h"


  




  



  



  




 



 
   


  
# 135 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_cec.h"



  
# 146 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_cec.h"


                               
# 156 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_cec.h"



  



  



 
 


 



  
void CEC_DeInit(void);
void CEC_Init(CEC_InitTypeDef* CEC_InitStruct);
void CEC_Cmd(FunctionalState NewState);
void CEC_ITConfig(FunctionalState NewState);
void CEC_OwnAddressConfig(uint8_t CEC_OwnAddress);
void CEC_SetPrescaler(uint16_t CEC_Prescaler);
void CEC_SendDataByte(uint8_t Data);
uint8_t CEC_ReceiveDataByte(void);
void CEC_StartOfMessage(void);
void CEC_EndOfMessageCmd(FunctionalState NewState);
FlagStatus CEC_GetFlagStatus(uint32_t CEC_FLAG);
void CEC_ClearFlag(uint32_t CEC_FLAG);
ITStatus CEC_GetITStatus(uint8_t CEC_IT);
void CEC_ClearITPendingBit(uint16_t CEC_IT);









  



  



  

# 32 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_crc.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_crc.h"



 



 



 



 



 



 



 



 



 

void CRC_ResetDR(void);
uint32_t CRC_CalcCRC(uint32_t Data);
uint32_t CRC_CalcBlockCRC(uint32_t pBuffer[], uint32_t BufferLength);
uint32_t CRC_GetCRC(void);
void CRC_SetIDRegister(uint8_t IDValue);
uint8_t CRC_GetIDRegister(void);








 



 



 

# 33 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dac.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dac.h"



 



 



 



 

typedef struct
{
  uint32_t DAC_Trigger;                      
 

  uint32_t DAC_WaveGeneration;               

 

  uint32_t DAC_LFSRUnmask_TriangleAmplitude; 

 

  uint32_t DAC_OutputBuffer;                 
 
}DAC_InitTypeDef;



 



 



 

# 93 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dac.h"

# 103 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dac.h"



 



 

# 118 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dac.h"


 



 

# 150 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dac.h"

# 175 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dac.h"


 



 







 



 







 



 

# 213 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dac.h"


 



 







 



 




 
# 260 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dac.h"



 



 



 



 

void DAC_DeInit(void);
void DAC_Init(uint32_t DAC_Channel, DAC_InitTypeDef* DAC_InitStruct);
void DAC_StructInit(DAC_InitTypeDef* DAC_InitStruct);
void DAC_Cmd(uint32_t DAC_Channel, FunctionalState NewState);



void DAC_DMACmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_SoftwareTriggerCmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_DualSoftwareTriggerCmd(FunctionalState NewState);
void DAC_WaveGenerationCmd(uint32_t DAC_Channel, uint32_t DAC_Wave, FunctionalState NewState);
void DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data);
void DAC_SetChannel2Data(uint32_t DAC_Align, uint16_t Data);
void DAC_SetDualChannelData(uint32_t DAC_Align, uint16_t Data2, uint16_t Data1);
uint16_t DAC_GetDataOutputValue(uint32_t DAC_Channel);
# 298 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dac.h"








 



 



 

# 34 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dbgmcu.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dbgmcu.h"



 



 



 



 



 

# 79 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dbgmcu.h"
                                              



  



 



 



 

uint32_t DBGMCU_GetREVID(void);
uint32_t DBGMCU_GetDEVID(void);
void DBGMCU_Config(uint32_t DBGMCU_Periph, FunctionalState NewState);








 



 



 

# 35 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"



 



 



 



 

typedef struct
{
  uint32_t DMA_PeripheralBaseAddr;  

  uint32_t DMA_MemoryBaseAddr;      

  uint32_t DMA_DIR;                
 

  uint32_t DMA_BufferSize;         

 

  uint32_t DMA_PeripheralInc;      
 

  uint32_t DMA_MemoryInc;          
 

  uint32_t DMA_PeripheralDataSize; 
 

  uint32_t DMA_MemoryDataSize;     
 

  uint32_t DMA_Mode;               


 

  uint32_t DMA_Priority;           
 

  uint32_t DMA_M2M;                
 
}DMA_InitTypeDef;



 



 

# 106 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"



 







 



 







 



 







 



 

# 153 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"


 



 

# 167 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"


 



 






 



 

# 194 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"


 



 







 



 






# 247 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"

# 268 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"



# 295 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"



 



 
# 331 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"

# 352 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"



# 379 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"


 



 





 



 



 



 



 

void DMA_DeInit(DMA_Channel_TypeDef* DMAy_Channelx);
void DMA_Init(DMA_Channel_TypeDef* DMAy_Channelx, DMA_InitTypeDef* DMA_InitStruct);
void DMA_StructInit(DMA_InitTypeDef* DMA_InitStruct);
void DMA_Cmd(DMA_Channel_TypeDef* DMAy_Channelx, FunctionalState NewState);
void DMA_ITConfig(DMA_Channel_TypeDef* DMAy_Channelx, uint32_t DMA_IT, FunctionalState NewState);
void DMA_SetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx, uint16_t DataNumber); 
uint16_t DMA_GetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx);
FlagStatus DMA_GetFlagStatus(uint32_t DMAy_FLAG);
void DMA_ClearFlag(uint32_t DMAy_FLAG);
ITStatus DMA_GetITStatus(uint32_t DMAy_IT);
void DMA_ClearITPendingBit(uint32_t DMAy_IT);








 



 



 

# 36 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_exti.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_exti.h"



 



 



 



 

typedef enum
{
  EXTI_Mode_Interrupt = 0x00,
  EXTI_Mode_Event = 0x04
}EXTIMode_TypeDef;





 

typedef enum
{
  EXTI_Trigger_Rising = 0x08,
  EXTI_Trigger_Falling = 0x0C,  
  EXTI_Trigger_Rising_Falling = 0x10
}EXTITrigger_TypeDef;






 

typedef struct
{
  uint32_t EXTI_Line;               
 
   
  EXTIMode_TypeDef EXTI_Mode;       
 

  EXTITrigger_TypeDef EXTI_Trigger; 
 

  FunctionalState EXTI_LineCmd;     
  
}EXTI_InitTypeDef;



 



 



 

# 123 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_exti.h"
                                          
# 135 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_exti.h"

                    


 



 



 



 



 

void EXTI_DeInit(void);
void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_StructInit(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_GenerateSWInterrupt(uint32_t EXTI_Line);
FlagStatus EXTI_GetFlagStatus(uint32_t EXTI_Line);
void EXTI_ClearFlag(uint32_t EXTI_Line);
ITStatus EXTI_GetITStatus(uint32_t EXTI_Line);
void EXTI_ClearITPendingBit(uint32_t EXTI_Line);








 



 



 

# 37 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"



 



 



 



 

typedef enum
{ 
  FLASH_BUSY = 1,
  FLASH_ERROR_PG,
  FLASH_ERROR_WRP,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
}FLASH_Status;



 



 



 

# 76 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"


 



 







 



 







 



 

 
# 117 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"

 
# 143 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"

 
# 210 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"











 



 







 



 







 



 





# 269 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"


 


 
# 290 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"






 



 
# 332 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"





 
# 345 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"



 



 



 



 



 

 
void FLASH_SetLatency(uint32_t FLASH_Latency);
void FLASH_HalfCycleAccessCmd(uint32_t FLASH_HalfCycleAccess);
void FLASH_PrefetchBufferCmd(uint32_t FLASH_PrefetchBuffer);
void FLASH_Unlock(void);
void FLASH_Lock(void);
FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
FLASH_Status FLASH_EraseAllPages(void);
FLASH_Status FLASH_EraseOptionBytes(void);
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status FLASH_ProgramOptionByteData(uint32_t Address, uint8_t Data);
FLASH_Status FLASH_EnableWriteProtection(uint32_t FLASH_Pages);
FLASH_Status FLASH_ReadOutProtection(FunctionalState NewState);
FLASH_Status FLASH_UserOptionByteConfig(uint16_t OB_IWDG, uint16_t OB_STOP, uint16_t OB_STDBY);
uint32_t FLASH_GetUserOptionByte(void);
uint32_t FLASH_GetWriteProtectionOptionByte(void);
FlagStatus FLASH_GetReadOutProtectionStatus(void);
FlagStatus FLASH_GetPrefetchBufferStatus(void);
void FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState);
FlagStatus FLASH_GetFlagStatus(uint32_t FLASH_FLAG);
void FLASH_ClearFlag(uint32_t FLASH_FLAG);
FLASH_Status FLASH_GetStatus(void);
FLASH_Status FLASH_WaitForLastOperation(uint32_t Timeout);

 
void FLASH_UnlockBank1(void);
void FLASH_LockBank1(void);
FLASH_Status FLASH_EraseAllBank1Pages(void);
FLASH_Status FLASH_GetBank1Status(void);
FLASH_Status FLASH_WaitForLastBank1Operation(uint32_t Timeout);

# 407 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"








 



 



 

# 38 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"



 



 



 



 

typedef struct
{
  uint32_t FSMC_AddressSetupTime;       


 

  uint32_t FSMC_AddressHoldTime;        


 

  uint32_t FSMC_DataSetupTime;          


 

  uint32_t FSMC_BusTurnAroundDuration;  


 

  uint32_t FSMC_CLKDivision;            

 

  uint32_t FSMC_DataLatency;            





 

  uint32_t FSMC_AccessMode;             
 
}FSMC_NORSRAMTimingInitTypeDef;



 

typedef struct
{
  uint32_t FSMC_Bank;                
 

  uint32_t FSMC_DataAddressMux;      

 

  uint32_t FSMC_MemoryType;          

 

  uint32_t FSMC_MemoryDataWidth;     
 

  uint32_t FSMC_BurstAccessMode;     

 
                                       
  uint32_t FSMC_AsynchronousWait;     

 

  uint32_t FSMC_WaitSignalPolarity;  

 

  uint32_t FSMC_WrapMode;            

 

  uint32_t FSMC_WaitSignalActive;    


 

  uint32_t FSMC_WriteOperation;      
 

  uint32_t FSMC_WaitSignal;          

 

  uint32_t FSMC_ExtendedMode;        
 

  uint32_t FSMC_WriteBurst;          
  

  FSMC_NORSRAMTimingInitTypeDef* FSMC_ReadWriteTimingStruct;    

  FSMC_NORSRAMTimingInitTypeDef* FSMC_WriteTimingStruct;            
}FSMC_NORSRAMInitTypeDef;



 

typedef struct
{
  uint32_t FSMC_SetupTime;      



 

  uint32_t FSMC_WaitSetupTime;  



 

  uint32_t FSMC_HoldSetupTime;  




 

  uint32_t FSMC_HiZSetupTime;   



 
}FSMC_NAND_PCCARDTimingInitTypeDef;



 

typedef struct
{
  uint32_t FSMC_Bank;              
 

  uint32_t FSMC_Waitfeature;      
 

  uint32_t FSMC_MemoryDataWidth;  
 

  uint32_t FSMC_ECC;              
 

  uint32_t FSMC_ECCPageSize;      
 

  uint32_t FSMC_TCLRSetupTime;    

 

  uint32_t FSMC_TARSetupTime;     

  

  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_CommonSpaceTimingStruct;     

  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_AttributeSpaceTimingStruct;  
}FSMC_NANDInitTypeDef;



 

typedef struct
{
  uint32_t FSMC_Waitfeature;    
 

  uint32_t FSMC_TCLRSetupTime;  

 

  uint32_t FSMC_TARSetupTime;   

  

  
  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_CommonSpaceTimingStruct;  

  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_AttributeSpaceTimingStruct;    
  
  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_IOSpaceTimingStruct;    
}FSMC_PCCARDInitTypeDef;



 



 



 






 



   




 



     



 



















 



 








 



 

# 316 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"



 



 








 



 







 
  


 







 
  


 








 



 








 



 








 



 





                              


 



 







 



 









 



 







 



 





 



 





 



 





 



 





 



 





 



 





 



 

# 520 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"



 



 
  


 



 








 




 








 



 

# 576 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"



 



 





 



 





 



 





 



 





 



 





 



 





 



 

# 652 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"


 



 

# 668 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"





 



 



 



 



 



 

void FSMC_NORSRAMDeInit(uint32_t FSMC_Bank);
void FSMC_NANDDeInit(uint32_t FSMC_Bank);
void FSMC_PCCARDDeInit(void);
void FSMC_NORSRAMInit(FSMC_NORSRAMInitTypeDef* FSMC_NORSRAMInitStruct);
void FSMC_NANDInit(FSMC_NANDInitTypeDef* FSMC_NANDInitStruct);
void FSMC_PCCARDInit(FSMC_PCCARDInitTypeDef* FSMC_PCCARDInitStruct);
void FSMC_NORSRAMStructInit(FSMC_NORSRAMInitTypeDef* FSMC_NORSRAMInitStruct);
void FSMC_NANDStructInit(FSMC_NANDInitTypeDef* FSMC_NANDInitStruct);
void FSMC_PCCARDStructInit(FSMC_PCCARDInitTypeDef* FSMC_PCCARDInitStruct);
void FSMC_NORSRAMCmd(uint32_t FSMC_Bank, FunctionalState NewState);
void FSMC_NANDCmd(uint32_t FSMC_Bank, FunctionalState NewState);
void FSMC_PCCARDCmd(FunctionalState NewState);
void FSMC_NANDECCCmd(uint32_t FSMC_Bank, FunctionalState NewState);
uint32_t FSMC_GetECC(uint32_t FSMC_Bank);
void FSMC_ITConfig(uint32_t FSMC_Bank, uint32_t FSMC_IT, FunctionalState NewState);
FlagStatus FSMC_GetFlagStatus(uint32_t FSMC_Bank, uint32_t FSMC_FLAG);
void FSMC_ClearFlag(uint32_t FSMC_Bank, uint32_t FSMC_FLAG);
ITStatus FSMC_GetITStatus(uint32_t FSMC_Bank, uint32_t FSMC_IT);
void FSMC_ClearITPendingBit(uint32_t FSMC_Bank, uint32_t FSMC_IT);








 



 



  

# 39 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



 



 



 

# 52 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"
                                     


 

typedef enum
{ 
  GPIO_Speed_10MHz = 1,
  GPIO_Speed_2MHz, 
  GPIO_Speed_50MHz
}GPIOSpeed_TypeDef;





 

typedef enum
{ GPIO_Mode_AIN = 0x0,
  GPIO_Mode_IN_FLOATING = 0x04,
  GPIO_Mode_IPD = 0x28,
  GPIO_Mode_IPU = 0x48,
  GPIO_Mode_Out_OD = 0x14,
  GPIO_Mode_Out_PP = 0x10,
  GPIO_Mode_AF_OD = 0x1C,
  GPIO_Mode_AF_PP = 0x18
}GPIOMode_TypeDef;








 

typedef struct
{
  uint16_t GPIO_Pin;             
 

  GPIOSpeed_TypeDef GPIO_Speed;  
 

  GPIOMode_TypeDef GPIO_Mode;    
 
}GPIO_InitTypeDef;




 

typedef enum
{ Bit_RESET = 0,
  Bit_SET
}BitAction;





 



 



 

# 143 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



# 162 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



 



 

# 203 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"







# 216 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"






# 244 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"
                              


  



 

# 265 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"

# 273 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



 



 

# 298 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"

# 315 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



 



  








                                                 


 



 



 



 

void GPIO_DeInit(GPIO_TypeDef* GPIOx);
void GPIO_AFIODeInit(void);
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct);
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);
void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_EventOutputConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
void GPIO_EventOutputCmd(FunctionalState NewState);
void GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState);
void GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
void GPIO_ETH_MediaInterfaceConfig(uint32_t GPIO_ETH_MediaInterface);








 



 



 

# 40 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_i2c.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_i2c.h"



 



 



 



 

typedef struct
{
  uint32_t I2C_ClockSpeed;          
 

  uint16_t I2C_Mode;                
 

  uint16_t I2C_DutyCycle;           
 

  uint16_t I2C_OwnAddress1;         
 

  uint16_t I2C_Ack;                 
 

  uint16_t I2C_AcknowledgedAddress; 
 
}I2C_InitTypeDef;



  




 





 

# 91 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_i2c.h"


 



 







  



 







 



 







 



 







  



 

# 165 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_i2c.h"


 



 







 



 







  



 







  



 







  



 

# 235 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_i2c.h"



# 245 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_i2c.h"


 



 



 

# 264 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_i2c.h"



 

# 283 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_i2c.h"



# 297 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_i2c.h"


 



 




 







 
 

























 

 


 





























 

  
 


 
 

 






 
























 

    
 



 



 



























 

  
 

 


 
 


 


 

# 495 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_i2c.h"


 



 




 



 




 



 



 



 



 

void I2C_DeInit(I2C_TypeDef* I2Cx);
void I2C_Init(I2C_TypeDef* I2Cx, I2C_InitTypeDef* I2C_InitStruct);
void I2C_StructInit(I2C_InitTypeDef* I2C_InitStruct);
void I2C_Cmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_DMACmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_DMALastTransferCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTART(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTOP(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_AcknowledgeConfig(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_OwnAddress2Config(I2C_TypeDef* I2Cx, uint8_t Address);
void I2C_DualAddressCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GeneralCallCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_ITConfig(I2C_TypeDef* I2Cx, uint16_t I2C_IT, FunctionalState NewState);
void I2C_SendData(I2C_TypeDef* I2Cx, uint8_t Data);
uint8_t I2C_ReceiveData(I2C_TypeDef* I2Cx);
void I2C_Send7bitAddress(I2C_TypeDef* I2Cx, uint8_t Address, uint8_t I2C_Direction);
uint16_t I2C_ReadRegister(I2C_TypeDef* I2Cx, uint8_t I2C_Register);
void I2C_SoftwareResetCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_NACKPositionConfig(I2C_TypeDef* I2Cx, uint16_t I2C_NACKPosition);
void I2C_SMBusAlertConfig(I2C_TypeDef* I2Cx, uint16_t I2C_SMBusAlert);
void I2C_TransmitPEC(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_PECPositionConfig(I2C_TypeDef* I2Cx, uint16_t I2C_PECPosition);
void I2C_CalculatePEC(I2C_TypeDef* I2Cx, FunctionalState NewState);
uint8_t I2C_GetPEC(I2C_TypeDef* I2Cx);
void I2C_ARPCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_StretchClockCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_FastModeDutyCycleConfig(I2C_TypeDef* I2Cx, uint16_t I2C_DutyCycle);













































































 





 
ErrorStatus I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);




 
uint32_t I2C_GetLastEvent(I2C_TypeDef* I2Cx);




 
FlagStatus I2C_GetFlagStatus(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG);



 

void I2C_ClearFlag(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG);
ITStatus I2C_GetITStatus(I2C_TypeDef* I2Cx, uint32_t I2C_IT);
void I2C_ClearITPendingBit(I2C_TypeDef* I2Cx, uint32_t I2C_IT);








  



  



  

# 41 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_iwdg.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_iwdg.h"



 



 



 



 



 



 







 



 

# 83 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_iwdg.h"


 



 







 



 



 



 



 

void IWDG_WriteAccessCmd(uint16_t IWDG_WriteAccess);
void IWDG_SetPrescaler(uint8_t IWDG_Prescaler);
void IWDG_SetReload(uint16_t Reload);
void IWDG_ReloadCounter(void);
void IWDG_Enable(void);
FlagStatus IWDG_GetFlagStatus(uint16_t IWDG_FLAG);








 



 



 

# 42 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_pwr.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_pwr.h"



 



  



  



  



  



  

# 69 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_pwr.h"


 



 







 



 




 


 



 










 



 



 



 



 

void PWR_DeInit(void);
void PWR_BackupAccessCmd(FunctionalState NewState);
void PWR_PVDCmd(FunctionalState NewState);
void PWR_PVDLevelConfig(uint32_t PWR_PVDLevel);
void PWR_WakeUpPinCmd(FunctionalState NewState);
void PWR_EnterSTOPMode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry);
void PWR_EnterSTANDBYMode(void);
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG);
void PWR_ClearFlag(uint32_t PWR_FLAG);








 



 



 

# 43 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"



 



 



 

typedef struct
{
  uint32_t SYSCLK_Frequency;   
  uint32_t HCLK_Frequency;     
  uint32_t PCLK1_Frequency;    
  uint32_t PCLK2_Frequency;    
  uint32_t ADCCLK_Frequency;   
}RCC_ClocksTypeDef;



 



 



 









  



 



# 93 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"



  



 
# 125 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"

# 140 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 
# 174 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 




 
# 195 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 

# 282 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"




 

# 294 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 

# 316 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


  



 

# 332 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 

# 346 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"

# 363 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"




 




 








 
# 395 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


# 422 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"
  



 

# 434 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 








 



 

# 461 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 







# 488 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 

# 517 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"




  



 

# 552 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"
 




 



 







# 585 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"



 



 

# 605 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"

# 624 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"




 



 



 



 



 

void RCC_DeInit(void);
void RCC_HSEConfig(uint32_t RCC_HSE);
ErrorStatus RCC_WaitForHSEStartUp(void);
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue);
void RCC_HSICmd(FunctionalState NewState);
void RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t RCC_PLLMul);
void RCC_PLLCmd(FunctionalState NewState);





# 665 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"

void RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource);
uint8_t RCC_GetSYSCLKSource(void);
void RCC_HCLKConfig(uint32_t RCC_SYSCLK);
void RCC_PCLK1Config(uint32_t RCC_HCLK);
void RCC_PCLK2Config(uint32_t RCC_HCLK);
void RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState);


 void RCC_USBCLKConfig(uint32_t RCC_USBCLKSource);




void RCC_ADCCLKConfig(uint32_t RCC_PCLK2);






void RCC_LSEConfig(uint8_t RCC_LSE);
void RCC_LSICmd(FunctionalState NewState);
void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource);
void RCC_RTCCLKCmd(FunctionalState NewState);
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);
void RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);





void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_BackupResetCmd(FunctionalState NewState);
void RCC_ClockSecuritySystemCmd(FunctionalState NewState);
void RCC_MCOConfig(uint8_t RCC_MCO);
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);
void RCC_ClearFlag(void);
ITStatus RCC_GetITStatus(uint8_t RCC_IT);
void RCC_ClearITPendingBit(uint8_t RCC_IT);








 



 



  

# 44 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rtc.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rtc.h"



 



  



  



  



 



 

# 63 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rtc.h"


  



 

# 81 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rtc.h"



 



 



 



 



 

void RTC_ITConfig(uint16_t RTC_IT, FunctionalState NewState);
void RTC_EnterConfigMode(void);
void RTC_ExitConfigMode(void);
uint32_t  RTC_GetCounter(void);
void RTC_SetCounter(uint32_t CounterValue);
void RTC_SetPrescaler(uint32_t PrescalerValue);
void RTC_SetAlarm(uint32_t AlarmValue);
uint32_t  RTC_GetDivider(void);
void RTC_WaitForLastTask(void);
void RTC_WaitForSynchro(void);
FlagStatus RTC_GetFlagStatus(uint16_t RTC_FLAG);
void RTC_ClearFlag(uint16_t RTC_FLAG);
ITStatus RTC_GetITStatus(uint16_t RTC_IT);
void RTC_ClearITPendingBit(uint16_t RTC_IT);








 



 



 

# 45 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_sdio.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_sdio.h"



 



 



 

typedef struct
{
  uint32_t SDIO_ClockEdge;            
 

  uint32_t SDIO_ClockBypass;          

 

  uint32_t SDIO_ClockPowerSave;       

 

  uint32_t SDIO_BusWide;              
 

  uint32_t SDIO_HardwareFlowControl;  
 

  uint8_t SDIO_ClockDiv;              
 
                                           
} SDIO_InitTypeDef;

typedef struct
{
  uint32_t SDIO_Argument;  


 

  uint32_t SDIO_CmdIndex;   

  uint32_t SDIO_Response;  
 

  uint32_t SDIO_Wait;      
 

  uint32_t SDIO_CPSM;      

 
} SDIO_CmdInitTypeDef;

typedef struct
{
  uint32_t SDIO_DataTimeOut;     

  uint32_t SDIO_DataLength;      
 
  uint32_t SDIO_DataBlockSize;  
 
 
  uint32_t SDIO_TransferDir;    

 
 
  uint32_t SDIO_TransferMode;   
 
 
  uint32_t SDIO_DPSM;           

 
} SDIO_DataInitTypeDef;



  



 



 







 



 







  



 







 



 









 



 







 



 






  




 

# 221 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_sdio.h"


  



 




 



 

# 244 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_sdio.h"


 



 








 



 






  



 

# 282 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_sdio.h"


 



 




 



 

# 329 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_sdio.h"


 



 







 



 







 



 






 



 

# 420 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_sdio.h"



# 447 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_sdio.h"





 



 







 



 



 



 



 

void SDIO_DeInit(void);
void SDIO_Init(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_StructInit(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_ClockCmd(FunctionalState NewState);
void SDIO_SetPowerState(uint32_t SDIO_PowerState);
uint32_t SDIO_GetPowerState(void);
void SDIO_ITConfig(uint32_t SDIO_IT, FunctionalState NewState);
void SDIO_DMACmd(FunctionalState NewState);
void SDIO_SendCommand(SDIO_CmdInitTypeDef *SDIO_CmdInitStruct);
void SDIO_CmdStructInit(SDIO_CmdInitTypeDef* SDIO_CmdInitStruct);
uint8_t SDIO_GetCommandResponse(void);
uint32_t SDIO_GetResponse(uint32_t SDIO_RESP);
void SDIO_DataConfig(SDIO_DataInitTypeDef* SDIO_DataInitStruct);
void SDIO_DataStructInit(SDIO_DataInitTypeDef* SDIO_DataInitStruct);
uint32_t SDIO_GetDataCounter(void);
uint32_t SDIO_ReadData(void);
void SDIO_WriteData(uint32_t Data);
uint32_t SDIO_GetFIFOCount(void);
void SDIO_StartSDIOReadWait(FunctionalState NewState);
void SDIO_StopSDIOReadWait(FunctionalState NewState);
void SDIO_SetSDIOReadWaitMode(uint32_t SDIO_ReadWaitMode);
void SDIO_SetSDIOOperation(FunctionalState NewState);
void SDIO_SendSDIOSuspendCmd(FunctionalState NewState);
void SDIO_CommandCompletionCmd(FunctionalState NewState);
void SDIO_CEATAITCmd(FunctionalState NewState);
void SDIO_SendCEATACmd(FunctionalState NewState);
FlagStatus SDIO_GetFlagStatus(uint32_t SDIO_FLAG);
void SDIO_ClearFlag(uint32_t SDIO_FLAG);
ITStatus SDIO_GetITStatus(uint32_t SDIO_IT);
void SDIO_ClearITPendingBit(uint32_t SDIO_IT);








 



 



 

# 46 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_spi.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_spi.h"



 



  



 



 

typedef struct
{
  uint16_t SPI_Direction;           
 

  uint16_t SPI_Mode;                
 

  uint16_t SPI_DataSize;            
 

  uint16_t SPI_CPOL;                
 

  uint16_t SPI_CPHA;                
 

  uint16_t SPI_NSS;                 

 
 
  uint16_t SPI_BaudRatePrescaler;   



 

  uint16_t SPI_FirstBit;            
 

  uint16_t SPI_CRCPolynomial;        
}SPI_InitTypeDef;



 

typedef struct
{

  uint16_t I2S_Mode;         
 

  uint16_t I2S_Standard;     
 

  uint16_t I2S_DataFormat;   
 

  uint16_t I2S_MCLKOutput;   
 

  uint32_t I2S_AudioFreq;    
 

  uint16_t I2S_CPOL;         
 
}I2S_InitTypeDef;



 



 










 
  
# 135 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_spi.h"


 



 







 



 







  



 







 



 







 



 







  



 

# 219 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_spi.h"


  



 







 



 

# 247 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_spi.h"


 



 

# 265 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_spi.h"


 



 

# 281 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_spi.h"


  



 







 



 

# 311 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_spi.h"






  



 







 



 






 



 







 



 






 



 







 



 

# 395 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_spi.h"


 



 

# 416 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_spi.h"


 



 




 



 



 



 



 

void SPI_I2S_DeInit(SPI_TypeDef* SPIx);
void SPI_Init(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct);
void I2S_Init(SPI_TypeDef* SPIx, I2S_InitTypeDef* I2S_InitStruct);
void SPI_StructInit(SPI_InitTypeDef* SPI_InitStruct);
void I2S_StructInit(I2S_InitTypeDef* I2S_InitStruct);
void SPI_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void I2S_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_I2S_ITConfig(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT, FunctionalState NewState);
void SPI_I2S_DMACmd(SPI_TypeDef* SPIx, uint16_t SPI_I2S_DMAReq, FunctionalState NewState);
void SPI_I2S_SendData(SPI_TypeDef* SPIx, uint16_t Data);
uint16_t SPI_I2S_ReceiveData(SPI_TypeDef* SPIx);
void SPI_NSSInternalSoftwareConfig(SPI_TypeDef* SPIx, uint16_t SPI_NSSInternalSoft);
void SPI_SSOutputCmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_DataSizeConfig(SPI_TypeDef* SPIx, uint16_t SPI_DataSize);
void SPI_TransmitCRC(SPI_TypeDef* SPIx);
void SPI_CalculateCRC(SPI_TypeDef* SPIx, FunctionalState NewState);
uint16_t SPI_GetCRC(SPI_TypeDef* SPIx, uint8_t SPI_CRC);
uint16_t SPI_GetCRCPolynomial(SPI_TypeDef* SPIx);
void SPI_BiDirectionalLineConfig(SPI_TypeDef* SPIx, uint16_t SPI_Direction);
FlagStatus SPI_I2S_GetFlagStatus(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
void SPI_I2S_ClearFlag(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
ITStatus SPI_I2S_GetITStatus(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);
void SPI_I2S_ClearITPendingBit(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);








 



 



 

# 47 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"



 



  



  




 

typedef struct
{
  uint16_t TIM_Prescaler;         
 

  uint16_t TIM_CounterMode;       
 

  uint16_t TIM_Period;            

  

  uint16_t TIM_ClockDivision;     
 

  uint8_t TIM_RepetitionCounter;  






 
} TIM_TimeBaseInitTypeDef;       



 

typedef struct
{
  uint16_t TIM_OCMode;        
 

  uint16_t TIM_OutputState;   
 

  uint16_t TIM_OutputNState;  

 

  uint16_t TIM_Pulse;         
 

  uint16_t TIM_OCPolarity;    
 

  uint16_t TIM_OCNPolarity;   

 

  uint16_t TIM_OCIdleState;   

 

  uint16_t TIM_OCNIdleState;  

 
} TIM_OCInitTypeDef;



 

typedef struct
{

  uint16_t TIM_Channel;      
 

  uint16_t TIM_ICPolarity;   
 

  uint16_t TIM_ICSelection;  
 

  uint16_t TIM_ICPrescaler;  
 

  uint16_t TIM_ICFilter;     
 
} TIM_ICInitTypeDef;




 

typedef struct
{

  uint16_t TIM_OSSRState;        
 

  uint16_t TIM_OSSIState;        
 

  uint16_t TIM_LOCKLevel;        
  

  uint16_t TIM_DeadTime;         

 

  uint16_t TIM_Break;            
 

  uint16_t TIM_BreakPolarity;    
 

  uint16_t TIM_AutomaticOutput;  
 
} TIM_BDTRInitTypeDef;



 

# 185 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"

 



 






 
# 204 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"
									                                 
 
# 215 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"

                                             
# 224 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"

 
# 235 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"

 
# 248 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"

                                         
# 265 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"

 
# 278 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"
                                                                                                                                                                                                                          


  



 

# 307 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


 



 







  



 

# 340 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

# 354 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


 



 

# 372 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 







 



 
  






 



 







  



 







  



 







  



 







  



 







  



 







  



 







  



 

# 496 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 







 



 







  



 







  



 







  



 

# 560 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

# 576 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

# 592 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

# 609 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"

# 618 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

# 664 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

# 708 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

# 724 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"



  



 

# 741 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

# 769 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

# 783 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



  






 



 







  



 







  



 

# 832 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  




 

# 850 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"



  



 

# 865 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 







  



 





                                     


  



 







  



 

# 926 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

# 942 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 







  



 

# 986 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"
                               
                               



  



 




  



 




  



 

# 1033 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


 



 



 



  



 

void TIM_DeInit(TIM_TypeDef* TIMx);
void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC2Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC3Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC4Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_ICInit(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_PWMIConfig(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_BDTRConfig(TIM_TypeDef* TIMx, TIM_BDTRInitTypeDef *TIM_BDTRInitStruct);
void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_OCStructInit(TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_ICStructInit(TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_BDTRStructInit(TIM_BDTRInitTypeDef* TIM_BDTRInitStruct);
void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_CtrlPWMOutputs(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState);
void TIM_GenerateEvent(TIM_TypeDef* TIMx, uint16_t TIM_EventSource);
void TIM_DMAConfig(TIM_TypeDef* TIMx, uint16_t TIM_DMABase, uint16_t TIM_DMABurstLength);
void TIM_DMACmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState NewState);
void TIM_InternalClockConfig(TIM_TypeDef* TIMx);
void TIM_ITRxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_TIxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_TIxExternalCLKSource,
                                uint16_t TIM_ICPolarity, uint16_t ICFilter);
void TIM_ETRClockMode1Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                             uint16_t ExtTRGFilter);
void TIM_ETRClockMode2Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, 
                             uint16_t TIM_ExtTRGPolarity, uint16_t ExtTRGFilter);
void TIM_ETRConfig(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                   uint16_t ExtTRGFilter);
void TIM_PrescalerConfig(TIM_TypeDef* TIMx, uint16_t Prescaler, uint16_t TIM_PSCReloadMode);
void TIM_CounterModeConfig(TIM_TypeDef* TIMx, uint16_t TIM_CounterMode);
void TIM_SelectInputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_EncoderInterfaceConfig(TIM_TypeDef* TIMx, uint16_t TIM_EncoderMode,
                                uint16_t TIM_IC1Polarity, uint16_t TIM_IC2Polarity);
void TIM_ForcedOC1Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC2Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC3Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC4Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ARRPreloadConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectCOM(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectCCDMA(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_CCPreloadControl(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_OC1PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC2PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC3PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC4PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC1FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC2FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC3FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC4FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_ClearOC1Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC2Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC3Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC4Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_OC1PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC1NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC2PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC2NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC3PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC3NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC4PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_CCxCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCx);
void TIM_CCxNCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCxN);
void TIM_SelectOCxM(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_OCMode);
void TIM_UpdateDisableConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_UpdateRequestConfig(TIM_TypeDef* TIMx, uint16_t TIM_UpdateSource);
void TIM_SelectHallSensor(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectOnePulseMode(TIM_TypeDef* TIMx, uint16_t TIM_OPMode);
void TIM_SelectOutputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_TRGOSource);
void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_SlaveMode);
void TIM_SelectMasterSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_MasterSlaveMode);
void TIM_SetCounter(TIM_TypeDef* TIMx, uint16_t Counter);
void TIM_SetAutoreload(TIM_TypeDef* TIMx, uint16_t Autoreload);
void TIM_SetCompare1(TIM_TypeDef* TIMx, uint16_t Compare1);
void TIM_SetCompare2(TIM_TypeDef* TIMx, uint16_t Compare2);
void TIM_SetCompare3(TIM_TypeDef* TIMx, uint16_t Compare3);
void TIM_SetCompare4(TIM_TypeDef* TIMx, uint16_t Compare4);
void TIM_SetIC1Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC2Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC3Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC4Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetClockDivision(TIM_TypeDef* TIMx, uint16_t TIM_CKD);
uint16_t TIM_GetCapture1(TIM_TypeDef* TIMx);
uint16_t TIM_GetCapture2(TIM_TypeDef* TIMx);
uint16_t TIM_GetCapture3(TIM_TypeDef* TIMx);
uint16_t TIM_GetCapture4(TIM_TypeDef* TIMx);
uint16_t TIM_GetCounter(TIM_TypeDef* TIMx);
uint16_t TIM_GetPrescaler(TIM_TypeDef* TIMx);
FlagStatus TIM_GetFlagStatus(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
void TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT);
void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT);








  



  



 

# 48 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_usart.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_usart.h"



 



  



  



  
  
typedef struct
{
  uint32_t USART_BaudRate;            


 

  uint16_t USART_WordLength;          
 

  uint16_t USART_StopBits;            
 

  uint16_t USART_Parity;              




 
 
  uint16_t USART_Mode;                
 

  uint16_t USART_HardwareFlowControl; 

 
} USART_InitTypeDef;



  
  
typedef struct
{

  uint16_t USART_Clock;   
 

  uint16_t USART_CPOL;    
 

  uint16_t USART_CPHA;    
 

  uint16_t USART_LastBit; 

 
} USART_ClockInitTypeDef;



  



  
  
















  
  


                                    




  



  
  
# 145 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_usart.h"


  



  
  
# 159 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_usart.h"


  



  
  





  



  
# 186 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_usart.h"


  



  






  



 
  






  



 







 



 







  



 
  
# 253 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_usart.h"



 



 
  





# 273 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_usart.h"





 



 







  



 







 



 
  







 



 







  



 

# 348 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_usart.h"
                              








  



  



  



  



 

void USART_DeInit(USART_TypeDef* USARTx);
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);
void USART_StructInit(USART_InitTypeDef* USART_InitStruct);
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_ClockStructInit(USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_ITConfig(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState);
void USART_DMACmd(USART_TypeDef* USARTx, uint16_t USART_DMAReq, FunctionalState NewState);
void USART_SetAddress(USART_TypeDef* USARTx, uint8_t USART_Address);
void USART_WakeUpConfig(USART_TypeDef* USARTx, uint16_t USART_WakeUp);
void USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_LINBreakDetectLengthConfig(USART_TypeDef* USARTx, uint16_t USART_LINBreakDetectLength);
void USART_LINCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
uint16_t USART_ReceiveData(USART_TypeDef* USARTx);
void USART_SendBreak(USART_TypeDef* USARTx);
void USART_SetGuardTime(USART_TypeDef* USARTx, uint8_t USART_GuardTime);
void USART_SetPrescaler(USART_TypeDef* USARTx, uint8_t USART_Prescaler);
void USART_SmartCardCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SmartCardNACKCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_HalfDuplexCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_OverSampling8Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_OneBitMethodCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_IrDAConfig(USART_TypeDef* USARTx, uint16_t USART_IrDAMode);
void USART_IrDACmd(USART_TypeDef* USARTx, FunctionalState NewState);
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG);
void USART_ClearFlag(USART_TypeDef* USARTx, uint16_t USART_FLAG);
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint16_t USART_IT);
void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint16_t USART_IT);








  



  



  

# 49 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_wwdg.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_wwdg.h"



 



  



  
  


  



  
  


  
  
# 67 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_wwdg.h"



  



  



  


  



  
  
void WWDG_DeInit(void);
void WWDG_SetPrescaler(uint32_t WWDG_Prescaler);
void WWDG_SetWindowValue(uint8_t WindowValue);
void WWDG_EnableIT(void);
void WWDG_SetCounter(uint8_t Counter);
void WWDG_Enable(uint8_t Counter);
FlagStatus WWDG_GetFlagStatus(void);
void WWDG_ClearFlag(void);









  



  



  

# 50 "..\\User\\stm32f10x_conf.h"
# 1 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\misc.h"



















 

 







 
# 32 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\misc.h"



 



 



 



 

typedef struct
{
  uint8_t NVIC_IRQChannel;                    


 

  uint8_t NVIC_IRQChannelPreemptionPriority;  

 

  uint8_t NVIC_IRQChannelSubPriority;         

 

  FunctionalState NVIC_IRQChannelCmd;         

    
} NVIC_InitTypeDef;
 


 



 
























 



 



 



 







 



 

# 132 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\misc.h"


 



 

# 150 "..\\Drivers\\STM32F10x_StdPeriph_Driver\\inc\\misc.h"















 



 







 



 



 



 



 

void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset);
void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState);
void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource);









 



 



 

# 51 "..\\User\\stm32f10x_conf.h"

 
 

 
 

 
# 74 "..\\User\\stm32f10x_conf.h"



 
# 8343 "..\\Drivers\\CMSIS\\stm32f10x.h"




 

















 









 

  

 

# 4 "..\\Drivers\\BSP\\PWM\\bsp_pwm.h"

 












void TimOCxInit(void);

# 2 "..\\Drivers\\BSP\\PWM\\bsp_pwm.c"


static void TimBaseInit(void)
{
   TIM_TimeBaseInitTypeDef TimBaseInitStructure;
   RCC_APB1PeriphClockCmd(((uint32_t)0x00000002), ENABLE);
    
   TIM_InternalClockConfig(((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400)));
   TimBaseInitStructure.TIM_ClockDivision = ((uint16_t)0x0000);
   TimBaseInitStructure.TIM_CounterMode = ((uint16_t)0x0000);
   TimBaseInitStructure.TIM_Period=100-1;
   TimBaseInitStructure.TIM_Prescaler=720-1;
   TimBaseInitStructure.TIM_RepetitionCounter = 0;
   TIM_TimeBaseInit(((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400)), &TimBaseInitStructure);
}


static void GPIOTimInit(void)
{
    GPIO_InitTypeDef GPIOInitStructuer;
    RCC_APB2PeriphClockCmd(((uint32_t)0x00000008), ENABLE);

    GPIOInitStructuer.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIOInitStructuer.GPIO_Pin = ((uint16_t)0x0001);
    GPIOInitStructuer.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00)), &GPIOInitStructuer);

    GPIOInitStructuer.GPIO_Pin = ((uint16_t)0x0020);
    GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00)), &GPIOInitStructuer);

    GPIOInitStructuer.GPIO_Pin = ((uint16_t)0x0002);
    GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00)), &GPIOInitStructuer);
}








 
void TimOCxInit(void)
{
    TIM_OCInitTypeDef TimOCxInitStructuer;

    TimBaseInit(); 

    GPIOTimInit();

    TIM_OCStructInit(&TimOCxInitStructuer);
    
    TimOCxInitStructuer.TIM_OCMode = ((uint16_t)0x0060);
    
    
    TimOCxInitStructuer.TIM_OCPolarity = ((uint16_t)0x0000); 
    
    TimOCxInitStructuer.TIM_OutputState = ((uint16_t)0x0001);
    TimOCxInitStructuer.TIM_Pulse=100; 
    TIM_OC3Init(((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400)), &TimOCxInitStructuer);

    TIM_OC4Init(((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400)), &TimOCxInitStructuer);

    TIM_OC2Init(((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400)), &TimOCxInitStructuer);

    TIM_Cmd(((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400)), ENABLE);
}

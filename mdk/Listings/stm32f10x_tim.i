#line 1 "..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c"



















 

 
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"




















 

 







 
#line 1 "..\\cortex-m3\\stm32f10x.h"







































 



 



 
    






  


 
  


 

#line 75 "..\\cortex-m3\\stm32f10x.h"


















 





#line 107 "..\\cortex-m3\\stm32f10x.h"







            
#line 122 "..\\cortex-m3\\stm32f10x.h"





 






 
#line 143 "..\\cortex-m3\\stm32f10x.h"



 



 



 
#line 162 "..\\cortex-m3\\stm32f10x.h"




 
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

#line 221 "..\\cortex-m3\\stm32f10x.h"

#line 242 "..\\cortex-m3\\stm32f10x.h"

#line 270 "..\\cortex-m3\\stm32f10x.h"


  ADC1_IRQn                   = 18,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_TIM15_IRQn         = 24,      
  TIM1_UP_TIM16_IRQn          = 25,      
  TIM1_TRG_COM_TIM17_IRQn     = 26,      
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
  CEC_IRQn                    = 42,      
  TIM6_DAC_IRQn               = 54,      
  TIM7_IRQn                   = 55              


#line 341 "..\\cortex-m3\\stm32f10x.h"

#line 381 "..\\cortex-m3\\stm32f10x.h"

#line 426 "..\\cortex-m3\\stm32f10x.h"

#line 472 "..\\cortex-m3\\stm32f10x.h"
} IRQn_Type;



 

#line 1 "..\\cortex-m3\\core_cm3.h"
 




















 





































 

 
 
 
 
 
 
 
 








 











#line 1 "D:\\EngineeringSoftware\\Keil_core\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "D:\\EngineeringSoftware\\Keil_core\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "D:\\EngineeringSoftware\\Keil_core\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
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




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "D:\\EngineeringSoftware\\Keil_core\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "D:\\EngineeringSoftware\\Keil_core\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "D:\\EngineeringSoftware\\Keil_core\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 91 "..\\cortex-m3\\core_cm3.h"

















 

#line 117 "..\\cortex-m3\\core_cm3.h"





 


 





 
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



       uint32_t RESERVED1;

} InterruptType_Type;

 



 








   


#line 614 "..\\cortex-m3\\core_cm3.h"





 
typedef struct
{
  volatile uint32_t DHCSR;                         
  volatile  uint32_t DCRSR;                         
  volatile uint32_t DCRDR;                         
  volatile uint32_t DEMCR;                         
} CoreDebug_Type;

 




































 






 






































   


 
#line 721 "..\\cortex-m3\\core_cm3.h"

#line 728 "..\\cortex-m3\\core_cm3.h"






   




 





#line 758 "..\\cortex-m3\\core_cm3.h"


 


 




#line 783 "..\\cortex-m3\\core_cm3.h"


 
 
 
 








 
extern uint32_t __get_PSP(void);








 
extern void __set_PSP(uint32_t topOfProcStack);








 
extern uint32_t __get_MSP(void);








 
extern void __set_MSP(uint32_t topOfMainStack);








 
extern uint32_t __REV16(uint16_t value);








 
extern int32_t __REVSH(int16_t value);


#line 933 "..\\cortex-m3\\core_cm3.h"





 








 
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





#line 1445 "..\\cortex-m3\\core_cm3.h"







 
 

 











 
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

   






   



 
#line 479 "..\\cortex-m3\\stm32f10x.h"
#line 1 "..\\cortex-m3\\system_stm32f10x.h"



















 



 



   
  


 









 



 




 

extern uint32_t SystemCoreClock;           



 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 480 "..\\cortex-m3\\stm32f10x.h"
#line 481 "..\\cortex-m3\\stm32f10x.h"



   

 
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

  volatile uint32_t SR;

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
#line 920 "..\\cortex-m3\\stm32f10x.h"
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







  uint32_t RESERVED0;
  volatile uint32_t CFGR2;

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



 
  


 











 




#line 1312 "..\\cortex-m3\\stm32f10x.h"

#line 1335 "..\\cortex-m3\\stm32f10x.h"



#line 1354 "..\\cortex-m3\\stm32f10x.h"




















 
  


   

#line 1454 "..\\cortex-m3\\stm32f10x.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 

 



 



 


 
 
 
 
 

 











 
#line 1515 "..\\cortex-m3\\stm32f10x.h"




 





 
 
 
 
 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 





 



 






 
 
 
 
 

 
#line 1691 "..\\cortex-m3\\stm32f10x.h"

#line 1698 "..\\cortex-m3\\stm32f10x.h"

 
 








 








 






#line 1734 "..\\cortex-m3\\stm32f10x.h"

 











 











 













 






#line 1816 "..\\cortex-m3\\stm32f10x.h"




#line 1835 "..\\cortex-m3\\stm32f10x.h"

 





#line 1883 "..\\cortex-m3\\stm32f10x.h"

 
#line 1902 "..\\cortex-m3\\stm32f10x.h"

#line 1911 "..\\cortex-m3\\stm32f10x.h"

 
#line 1919 "..\\cortex-m3\\stm32f10x.h"



















#line 1944 "..\\cortex-m3\\stm32f10x.h"












 













#line 1976 "..\\cortex-m3\\stm32f10x.h"





#line 1990 "..\\cortex-m3\\stm32f10x.h"

#line 1997 "..\\cortex-m3\\stm32f10x.h"

#line 2007 "..\\cortex-m3\\stm32f10x.h"











 


















#line 2043 "..\\cortex-m3\\stm32f10x.h"

 
#line 2051 "..\\cortex-m3\\stm32f10x.h"



















#line 2076 "..\\cortex-m3\\stm32f10x.h"












 













#line 2108 "..\\cortex-m3\\stm32f10x.h"





#line 2122 "..\\cortex-m3\\stm32f10x.h"

#line 2129 "..\\cortex-m3\\stm32f10x.h"

#line 2139 "..\\cortex-m3\\stm32f10x.h"











 








 








   
#line 2178 "..\\cortex-m3\\stm32f10x.h"

#line 2273 "..\\cortex-m3\\stm32f10x.h"


 
 






#line 2300 "..\\cortex-m3\\stm32f10x.h"
 
 
 
 
 
 

 




































































 




































































 
#line 2462 "..\\cortex-m3\\stm32f10x.h"

 
#line 2480 "..\\cortex-m3\\stm32f10x.h"

 
#line 2498 "..\\cortex-m3\\stm32f10x.h"

#line 2515 "..\\cortex-m3\\stm32f10x.h"

 
#line 2533 "..\\cortex-m3\\stm32f10x.h"

 
#line 2552 "..\\cortex-m3\\stm32f10x.h"

 

 






 
#line 2579 "..\\cortex-m3\\stm32f10x.h"






 








 









 








 








 









 










 




#line 2654 "..\\cortex-m3\\stm32f10x.h"

 










#line 2685 "..\\cortex-m3\\stm32f10x.h"

 





 
#line 2700 "..\\cortex-m3\\stm32f10x.h"

 
#line 2709 "..\\cortex-m3\\stm32f10x.h"

   
#line 2718 "..\\cortex-m3\\stm32f10x.h"

 
#line 2727 "..\\cortex-m3\\stm32f10x.h"

 





 
#line 2742 "..\\cortex-m3\\stm32f10x.h"

 
#line 2751 "..\\cortex-m3\\stm32f10x.h"

   
#line 2760 "..\\cortex-m3\\stm32f10x.h"

 
#line 2769 "..\\cortex-m3\\stm32f10x.h"

 





 
#line 2784 "..\\cortex-m3\\stm32f10x.h"

 
#line 2793 "..\\cortex-m3\\stm32f10x.h"

   
#line 2802 "..\\cortex-m3\\stm32f10x.h"

 
#line 2811 "..\\cortex-m3\\stm32f10x.h"

 





 
#line 2826 "..\\cortex-m3\\stm32f10x.h"

 
#line 2835 "..\\cortex-m3\\stm32f10x.h"

   
#line 2844 "..\\cortex-m3\\stm32f10x.h"

 
#line 2853 "..\\cortex-m3\\stm32f10x.h"


 
#line 2862 "..\\cortex-m3\\stm32f10x.h"

#line 2871 "..\\cortex-m3\\stm32f10x.h"

#line 2881 "..\\cortex-m3\\stm32f10x.h"

 
 
 
 
 

 





 


 


 




 
 
 
 
 

 
#line 2945 "..\\cortex-m3\\stm32f10x.h"

 
#line 2980 "..\\cortex-m3\\stm32f10x.h"

 
#line 3015 "..\\cortex-m3\\stm32f10x.h"

 
#line 3050 "..\\cortex-m3\\stm32f10x.h"

 
#line 3085 "..\\cortex-m3\\stm32f10x.h"

 





 





 





 





 





 





 





 





 






 
#line 3152 "..\\cortex-m3\\stm32f10x.h"

 



 









 
#line 3176 "..\\cortex-m3\\stm32f10x.h"




 




 
#line 3192 "..\\cortex-m3\\stm32f10x.h"

 





 
#line 3214 "..\\cortex-m3\\stm32f10x.h"

 
 





 
#line 3229 "..\\cortex-m3\\stm32f10x.h"
 
#line 3236 "..\\cortex-m3\\stm32f10x.h"

 




 






 


 


 


 
 
 
 
 

 
#line 3285 "..\\cortex-m3\\stm32f10x.h"

 
#line 3307 "..\\cortex-m3\\stm32f10x.h"

 
#line 3329 "..\\cortex-m3\\stm32f10x.h"

 
#line 3351 "..\\cortex-m3\\stm32f10x.h"

 
#line 3373 "..\\cortex-m3\\stm32f10x.h"

 
#line 3395 "..\\cortex-m3\\stm32f10x.h"

 
 
 
 
 

 
#line 3431 "..\\cortex-m3\\stm32f10x.h"

 
#line 3461 "..\\cortex-m3\\stm32f10x.h"

 
#line 3471 "..\\cortex-m3\\stm32f10x.h"















 
#line 3495 "..\\cortex-m3\\stm32f10x.h"















 
#line 3519 "..\\cortex-m3\\stm32f10x.h"















 
#line 3543 "..\\cortex-m3\\stm32f10x.h"















 
#line 3567 "..\\cortex-m3\\stm32f10x.h"















 
#line 3591 "..\\cortex-m3\\stm32f10x.h"















 
#line 3615 "..\\cortex-m3\\stm32f10x.h"















 


 


 


 


 


 


 


 


 


 



 


 


 



 


 


 


 



 


 


 


 


 
 
 
 
 

 






 
#line 3716 "..\\cortex-m3\\stm32f10x.h"

#line 3725 "..\\cortex-m3\\stm32f10x.h"















  
 
#line 3748 "..\\cortex-m3\\stm32f10x.h"


















 








































 


















































 


 


 


 


 


 


 
#line 3883 "..\\cortex-m3\\stm32f10x.h"

#line 3890 "..\\cortex-m3\\stm32f10x.h"

#line 3897 "..\\cortex-m3\\stm32f10x.h"

#line 3904 "..\\cortex-m3\\stm32f10x.h"







 
#line 3918 "..\\cortex-m3\\stm32f10x.h"

#line 3925 "..\\cortex-m3\\stm32f10x.h"

#line 3932 "..\\cortex-m3\\stm32f10x.h"

#line 3939 "..\\cortex-m3\\stm32f10x.h"

#line 3946 "..\\cortex-m3\\stm32f10x.h"

#line 3953 "..\\cortex-m3\\stm32f10x.h"

 
#line 3961 "..\\cortex-m3\\stm32f10x.h"

#line 3968 "..\\cortex-m3\\stm32f10x.h"

#line 3975 "..\\cortex-m3\\stm32f10x.h"

#line 3982 "..\\cortex-m3\\stm32f10x.h"

#line 3989 "..\\cortex-m3\\stm32f10x.h"

#line 3996 "..\\cortex-m3\\stm32f10x.h"

 
#line 4004 "..\\cortex-m3\\stm32f10x.h"

#line 4011 "..\\cortex-m3\\stm32f10x.h"

#line 4018 "..\\cortex-m3\\stm32f10x.h"

#line 4025 "..\\cortex-m3\\stm32f10x.h"





 


 


 


 


 



 
 
 
 
 

 









































 



 


 


 


 


 


 


 



 



 



 


 


 



 
 
 
 
 
 





 






 


 
#line 4167 "..\\cortex-m3\\stm32f10x.h"

 
#line 4177 "..\\cortex-m3\\stm32f10x.h"

 


 


 
 
 
 
 

 
















 









#line 4225 "..\\cortex-m3\\stm32f10x.h"

 

























 
#line 4268 "..\\cortex-m3\\stm32f10x.h"

 
#line 4282 "..\\cortex-m3\\stm32f10x.h"

 
#line 4292 "..\\cortex-m3\\stm32f10x.h"

 




























 





















 




























 





















 
#line 4411 "..\\cortex-m3\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
#line 4446 "..\\cortex-m3\\stm32f10x.h"





#line 4457 "..\\cortex-m3\\stm32f10x.h"

 
#line 4465 "..\\cortex-m3\\stm32f10x.h"

#line 4472 "..\\cortex-m3\\stm32f10x.h"

 


 
 
 
 
 

 




 
#line 4494 "..\\cortex-m3\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
 
 
 
 

 


 





 


 



 
 
 
 
 

 
#line 4556 "..\\cortex-m3\\stm32f10x.h"



 
#line 4568 "..\\cortex-m3\\stm32f10x.h"







 


 
 
 
 
 

 











#line 4606 "..\\cortex-m3\\stm32f10x.h"

 











#line 4629 "..\\cortex-m3\\stm32f10x.h"

 











#line 4652 "..\\cortex-m3\\stm32f10x.h"

 











#line 4675 "..\\cortex-m3\\stm32f10x.h"

 








































 








































 








































 








































 


































 


































 


































 


































 



























 



























 



























 
#line 5072 "..\\cortex-m3\\stm32f10x.h"

 
#line 5081 "..\\cortex-m3\\stm32f10x.h"

 
#line 5090 "..\\cortex-m3\\stm32f10x.h"

 
#line 5101 "..\\cortex-m3\\stm32f10x.h"

#line 5111 "..\\cortex-m3\\stm32f10x.h"

#line 5121 "..\\cortex-m3\\stm32f10x.h"

#line 5131 "..\\cortex-m3\\stm32f10x.h"

 
#line 5142 "..\\cortex-m3\\stm32f10x.h"

#line 5152 "..\\cortex-m3\\stm32f10x.h"

#line 5162 "..\\cortex-m3\\stm32f10x.h"

#line 5172 "..\\cortex-m3\\stm32f10x.h"

 
#line 5183 "..\\cortex-m3\\stm32f10x.h"

#line 5193 "..\\cortex-m3\\stm32f10x.h"

#line 5203 "..\\cortex-m3\\stm32f10x.h"

#line 5213 "..\\cortex-m3\\stm32f10x.h"

 
#line 5224 "..\\cortex-m3\\stm32f10x.h"

#line 5234 "..\\cortex-m3\\stm32f10x.h"

#line 5244 "..\\cortex-m3\\stm32f10x.h"

#line 5254 "..\\cortex-m3\\stm32f10x.h"

 
#line 5265 "..\\cortex-m3\\stm32f10x.h"

#line 5275 "..\\cortex-m3\\stm32f10x.h"

#line 5285 "..\\cortex-m3\\stm32f10x.h"

#line 5295 "..\\cortex-m3\\stm32f10x.h"

 
#line 5306 "..\\cortex-m3\\stm32f10x.h"

#line 5316 "..\\cortex-m3\\stm32f10x.h"

#line 5326 "..\\cortex-m3\\stm32f10x.h"

#line 5336 "..\\cortex-m3\\stm32f10x.h"

 
#line 5347 "..\\cortex-m3\\stm32f10x.h"

#line 5357 "..\\cortex-m3\\stm32f10x.h"

#line 5367 "..\\cortex-m3\\stm32f10x.h"

#line 5377 "..\\cortex-m3\\stm32f10x.h"

 


 


 
 
 
 
 

 




 












 


 






#line 5425 "..\\cortex-m3\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
















 


 
#line 5495 "..\\cortex-m3\\stm32f10x.h"

 
#line 5510 "..\\cortex-m3\\stm32f10x.h"

 
#line 5536 "..\\cortex-m3\\stm32f10x.h"

 


 


 
 
 
 
 

 
 























 























 























 























 























 























 























 























 
 
#line 5757 "..\\cortex-m3\\stm32f10x.h"

 
#line 5769 "..\\cortex-m3\\stm32f10x.h"

 






 
#line 5786 "..\\cortex-m3\\stm32f10x.h"



     


 
 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


#line 5930 "..\\cortex-m3\\stm32f10x.h"



 


#line 5942 "..\\cortex-m3\\stm32f10x.h"



 


#line 5954 "..\\cortex-m3\\stm32f10x.h"



 


#line 5966 "..\\cortex-m3\\stm32f10x.h"



 


#line 5978 "..\\cortex-m3\\stm32f10x.h"



 


#line 5990 "..\\cortex-m3\\stm32f10x.h"



 


#line 6002 "..\\cortex-m3\\stm32f10x.h"



 


#line 6014 "..\\cortex-m3\\stm32f10x.h"



 

 


#line 6028 "..\\cortex-m3\\stm32f10x.h"



 


#line 6040 "..\\cortex-m3\\stm32f10x.h"



 


#line 6052 "..\\cortex-m3\\stm32f10x.h"



 


#line 6064 "..\\cortex-m3\\stm32f10x.h"



 


#line 6076 "..\\cortex-m3\\stm32f10x.h"



 


#line 6088 "..\\cortex-m3\\stm32f10x.h"



 


#line 6100 "..\\cortex-m3\\stm32f10x.h"



 


#line 6112 "..\\cortex-m3\\stm32f10x.h"



 


#line 6124 "..\\cortex-m3\\stm32f10x.h"



 


#line 6136 "..\\cortex-m3\\stm32f10x.h"



 


#line 6148 "..\\cortex-m3\\stm32f10x.h"



 


#line 6160 "..\\cortex-m3\\stm32f10x.h"



 


#line 6172 "..\\cortex-m3\\stm32f10x.h"



 


#line 6184 "..\\cortex-m3\\stm32f10x.h"



 


#line 6196 "..\\cortex-m3\\stm32f10x.h"



 


#line 6208 "..\\cortex-m3\\stm32f10x.h"



 
 
 
 
 

 
 
#line 6228 "..\\cortex-m3\\stm32f10x.h"

 
#line 6239 "..\\cortex-m3\\stm32f10x.h"

 
#line 6257 "..\\cortex-m3\\stm32f10x.h"











 





 





 
#line 6295 "..\\cortex-m3\\stm32f10x.h"

 












 
#line 6316 "..\\cortex-m3\\stm32f10x.h"

 
 






 




 





 





 






 




 





 





 






   




 





 





 





 




 





 





 





 




 





 





 
 


 
#line 6456 "..\\cortex-m3\\stm32f10x.h"

 
#line 6473 "..\\cortex-m3\\stm32f10x.h"

 
#line 6490 "..\\cortex-m3\\stm32f10x.h"

 
#line 6507 "..\\cortex-m3\\stm32f10x.h"

 
#line 6541 "..\\cortex-m3\\stm32f10x.h"

 
#line 6575 "..\\cortex-m3\\stm32f10x.h"

 
#line 6609 "..\\cortex-m3\\stm32f10x.h"

 
#line 6643 "..\\cortex-m3\\stm32f10x.h"

 
#line 6677 "..\\cortex-m3\\stm32f10x.h"

 
#line 6711 "..\\cortex-m3\\stm32f10x.h"

 
#line 6745 "..\\cortex-m3\\stm32f10x.h"

 
#line 6779 "..\\cortex-m3\\stm32f10x.h"

 
#line 6813 "..\\cortex-m3\\stm32f10x.h"

 
#line 6847 "..\\cortex-m3\\stm32f10x.h"

 
#line 6881 "..\\cortex-m3\\stm32f10x.h"

 
#line 6915 "..\\cortex-m3\\stm32f10x.h"

 
#line 6949 "..\\cortex-m3\\stm32f10x.h"

 
#line 6983 "..\\cortex-m3\\stm32f10x.h"

 
#line 7017 "..\\cortex-m3\\stm32f10x.h"

 
#line 7051 "..\\cortex-m3\\stm32f10x.h"

 
#line 7085 "..\\cortex-m3\\stm32f10x.h"

 
#line 7119 "..\\cortex-m3\\stm32f10x.h"

 
#line 7153 "..\\cortex-m3\\stm32f10x.h"

 
#line 7187 "..\\cortex-m3\\stm32f10x.h"

 
#line 7221 "..\\cortex-m3\\stm32f10x.h"

 
#line 7255 "..\\cortex-m3\\stm32f10x.h"

 
#line 7289 "..\\cortex-m3\\stm32f10x.h"

 
#line 7323 "..\\cortex-m3\\stm32f10x.h"

 
#line 7357 "..\\cortex-m3\\stm32f10x.h"

 
#line 7391 "..\\cortex-m3\\stm32f10x.h"

 
#line 7425 "..\\cortex-m3\\stm32f10x.h"

 
#line 7459 "..\\cortex-m3\\stm32f10x.h"

 
 
 
 
 

 









#line 7486 "..\\cortex-m3\\stm32f10x.h"

 
#line 7494 "..\\cortex-m3\\stm32f10x.h"

 
#line 7504 "..\\cortex-m3\\stm32f10x.h"

 


 


 


 


 





















 




 
 
 
 
 

 
#line 7565 "..\\cortex-m3\\stm32f10x.h"

 
#line 7574 "..\\cortex-m3\\stm32f10x.h"







 



#line 7595 "..\\cortex-m3\\stm32f10x.h"



 



 


 
#line 7620 "..\\cortex-m3\\stm32f10x.h"

 
#line 7630 "..\\cortex-m3\\stm32f10x.h"

 




 


 
 
 
 
 

 
#line 7656 "..\\cortex-m3\\stm32f10x.h"

 


 



 
#line 7680 "..\\cortex-m3\\stm32f10x.h"

 
#line 7689 "..\\cortex-m3\\stm32f10x.h"







 
#line 7709 "..\\cortex-m3\\stm32f10x.h"

 
#line 7720 "..\\cortex-m3\\stm32f10x.h"



 
 
 
 
 

 


#line 7749 "..\\cortex-m3\\stm32f10x.h"

 









#line 7783 "..\\cortex-m3\\stm32f10x.h"

 
 
 
 
 

 









 


 


 





 
#line 7823 "..\\cortex-m3\\stm32f10x.h"

 


 









 


 

 



 



 



 



 



 



 



 



#line 8287 "..\\cortex-m3\\stm32f10x.h"



 

 

  

#line 1 "..\\bsp\\stm32f10x_conf.h"



















 

 



 
 
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"




















 

 







 
#line 1 "..\\cortex-m3\\stm32f10x.h"







































 



 



 
    
#line 8327 "..\\cortex-m3\\stm32f10x.h"



 

  

 

 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"



 



 



 



 

typedef struct
{
  uint32_t ADC_Mode;                      

 

  FunctionalState ADC_ScanConvMode;       

 

  FunctionalState ADC_ContinuousConvMode; 

 

  uint32_t ADC_ExternalTrigConv;          

 

  uint32_t ADC_DataAlign;                 
 

  uint8_t ADC_NbrOfChannel;               

 
}ADC_InitTypeDef;


 



 










 

#line 104 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"

#line 115 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 

#line 129 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"




#line 139 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"

#line 154 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 







 



 

#line 192 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"




#line 205 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 

#line 229 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 

















#line 266 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 

#line 282 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 

#line 297 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"

#line 305 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 











 



 

#line 338 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 





 



 





 



 





 



 





  




 




 



 





 



 





 



 



 



 



 

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









 



 



 

 
#line 29 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_bkp.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_bkp.h"



 



 



 



 



 



 







 



 

#line 78 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_bkp.h"


 



 

#line 128 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_bkp.h"

#line 143 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_bkp.h"




 



 



 



 



 

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








 



 



 

 
#line 30 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"



 



 



 






 

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



 



 



 






 



 












 





   










 
  



   







 



 










 



 

#line 301 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"




 



 

#line 319 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"





 



 





 



 







 



 








 



 









 



 







 



 



 



 








 



 







 



 







 



 








 



 








 



 






 



 






 




   
                                                                
#line 493 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"




 



 

 
 

 




 
#line 518 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"

 



 

 





#line 539 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"








 

  


 


  


 
#line 565 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"

 



 






 





#line 590 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"

#line 597 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"



 



 
#line 621 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_can.h"



 



 



 



 



 
  
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








 



 



 

 
#line 31 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_cec.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_cec.h"



 



 
  



 
   


  
typedef struct
{
  uint16_t CEC_BitTimingMode; 
 
  uint16_t CEC_BitPeriodMode; 
 
}CEC_InitTypeDef;



 



  
  


  







 



  







  




  
#line 100 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_cec.h"


  




  



  



  




 



 
   


  
#line 136 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_cec.h"



  
#line 147 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_cec.h"


                               
#line 157 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_cec.h"



  



  



 
 


 



  
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









  



  



  

 
#line 32 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_crc.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_crc.h"



 



 



 



 



 



 



 



 



 

void CRC_ResetDR(void);
uint32_t CRC_CalcCRC(uint32_t Data);
uint32_t CRC_CalcBlockCRC(uint32_t pBuffer[], uint32_t BufferLength);
uint32_t CRC_GetCRC(void);
void CRC_SetIDRegister(uint8_t IDValue);
uint8_t CRC_GetIDRegister(void);








 



 



 

 
#line 33 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dac.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dac.h"



 



 



 



 

typedef struct
{
  uint32_t DAC_Trigger;                      
 

  uint32_t DAC_WaveGeneration;               

 

  uint32_t DAC_LFSRUnmask_TriangleAmplitude; 

 

  uint32_t DAC_OutputBuffer;                 
 
}DAC_InitTypeDef;



 



 



 

#line 94 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dac.h"

#line 104 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dac.h"



 



 

#line 119 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dac.h"


 



 

#line 151 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dac.h"

#line 176 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dac.h"


 



 







 



 







 



 

#line 214 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dac.h"


 



 







 



 




 



  
  





  



  
  





 




 



 



 



 

void DAC_DeInit(void);
void DAC_Init(uint32_t DAC_Channel, DAC_InitTypeDef* DAC_InitStruct);
void DAC_StructInit(DAC_InitTypeDef* DAC_InitStruct);
void DAC_Cmd(uint32_t DAC_Channel, FunctionalState NewState);

void DAC_ITConfig(uint32_t DAC_Channel, uint32_t DAC_IT, FunctionalState NewState);

void DAC_DMACmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_SoftwareTriggerCmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_DualSoftwareTriggerCmd(FunctionalState NewState);
void DAC_WaveGenerationCmd(uint32_t DAC_Channel, uint32_t DAC_Wave, FunctionalState NewState);
void DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data);
void DAC_SetChannel2Data(uint32_t DAC_Align, uint16_t Data);
void DAC_SetDualChannelData(uint32_t DAC_Align, uint16_t Data2, uint16_t Data1);
uint16_t DAC_GetDataOutputValue(uint32_t DAC_Channel);

FlagStatus DAC_GetFlagStatus(uint32_t DAC_Channel, uint32_t DAC_FLAG);
void DAC_ClearFlag(uint32_t DAC_Channel, uint32_t DAC_FLAG);
ITStatus DAC_GetITStatus(uint32_t DAC_Channel, uint32_t DAC_IT);
void DAC_ClearITPendingBit(uint32_t DAC_Channel, uint32_t DAC_IT);









 



 



 

 
#line 34 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dbgmcu.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dbgmcu.h"



 



 



 



 



 

#line 80 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dbgmcu.h"
                                              



  



 



 



 

uint32_t DBGMCU_GetREVID(void);
uint32_t DBGMCU_GetDEVID(void);
void DBGMCU_Config(uint32_t DBGMCU_Periph, FunctionalState NewState);








 



 



 

 
#line 35 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"



 



 



 



 

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



 



 

#line 107 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"



 







 



 







 



 







 



 

#line 154 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"


 



 

#line 168 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"


 



 






 



 

#line 195 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"


 



 







 



 






#line 248 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"

#line 269 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"



#line 296 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"



 



 
#line 332 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"

#line 353 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"



#line 380 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_dma.h"


 



 





 



 



 



 



 

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








 



 



 

 
#line 36 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_exti.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_exti.h"



 



 



 



 

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



 



 



 

#line 124 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_exti.h"
                                          
#line 136 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_exti.h"

                    


 



 



 



 



 

void EXTI_DeInit(void);
void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_StructInit(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_GenerateSWInterrupt(uint32_t EXTI_Line);
FlagStatus EXTI_GetFlagStatus(uint32_t EXTI_Line);
void EXTI_ClearFlag(uint32_t EXTI_Line);
ITStatus EXTI_GetITStatus(uint32_t EXTI_Line);
void EXTI_ClearITPendingBit(uint32_t EXTI_Line);








 



 



 

 
#line 37 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"



 



 



 



 

typedef enum
{ 
  FLASH_BUSY = 1,
  FLASH_ERROR_PG,
  FLASH_ERROR_WRP,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
}FLASH_Status;



 



 



 

#line 77 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"


 



 







 



 







 



 

 
#line 118 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"

 
#line 144 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"

 
#line 211 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"











 



 







 



 







 



 





#line 270 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"


 


 
#line 291 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"






 



 
#line 333 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"





 
#line 346 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"



 



 



 



 



 

 
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

#line 408 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"








 



 



 

 
#line 38 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"



 



 



 



 

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



 



 



 






 



   




 



     



 



















 



 








 



 

#line 317 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"



 



 








 



 







 
  


 







 
  


 








 



 








 



 








 



 





                              


 



 







 



 









 



 







 



 





 



 





 



 





 



 





 



 





 



 





 



 

#line 521 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"



 



 
  


 



 








 




 








 



 

#line 577 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"



 



 





 



 





 



 





 



 





 



 





 



 





 



 

#line 653 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"


 



 

#line 669 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"





 



 



 



 



 



 

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








 



 



  

 
#line 39 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



 



 



 

#line 53 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"
                                     


 

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





 



 



 

#line 144 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



#line 163 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



 



 

#line 204 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"







#line 217 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"






#line 245 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"
                              


  



 

#line 266 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"

#line 274 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



 



 

#line 299 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"

#line 316 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



 



  








                                                 


 



 



 



 

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








 



 



 

 
#line 40 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_i2c.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_i2c.h"



 



 



 



 

typedef struct
{
  uint32_t I2C_ClockSpeed;          
 

  uint16_t I2C_Mode;                
 

  uint16_t I2C_DutyCycle;           
 

  uint16_t I2C_OwnAddress1;         
 

  uint16_t I2C_Ack;                 
 

  uint16_t I2C_AcknowledgedAddress; 
 
}I2C_InitTypeDef;



  




 





 

#line 92 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_i2c.h"


 



 







  



 







 



 







 



 







  



 

#line 166 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_i2c.h"


 



 







 



 







  



 







  



 







  



 

#line 236 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_i2c.h"



#line 246 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_i2c.h"


 



 



 

#line 265 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_i2c.h"



 

#line 284 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_i2c.h"



#line 298 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_i2c.h"


 



 




 







 
 

























 

 


 





























 

  
 


 
 

 






 
























 

    
 



 



 



























 

  
 

 


 
 


 


 

#line 496 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_i2c.h"


 



 




 



 




 



 



 



 



 

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








  



  



  

 
#line 41 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_iwdg.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_iwdg.h"



 



 



 



 



 



 







 



 

#line 84 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_iwdg.h"


 



 







 



 



 



 



 

void IWDG_WriteAccessCmd(uint16_t IWDG_WriteAccess);
void IWDG_SetPrescaler(uint8_t IWDG_Prescaler);
void IWDG_SetReload(uint16_t Reload);
void IWDG_ReloadCounter(void);
void IWDG_Enable(void);
FlagStatus IWDG_GetFlagStatus(uint16_t IWDG_FLAG);








 



 



 

 
#line 42 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_pwr.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_pwr.h"



 



  



  



  



  



  

#line 70 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_pwr.h"


 



 







 



 




 


 



 










 



 



 



 



 

void PWR_DeInit(void);
void PWR_BackupAccessCmd(FunctionalState NewState);
void PWR_PVDCmd(FunctionalState NewState);
void PWR_PVDLevelConfig(uint32_t PWR_PVDLevel);
void PWR_WakeUpPinCmd(FunctionalState NewState);
void PWR_EnterSTOPMode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry);
void PWR_EnterSTANDBYMode(void);
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG);
void PWR_ClearFlag(uint32_t PWR_FLAG);








 



 



 

 
#line 43 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"



 



 



 

typedef struct
{
  uint32_t SYSCLK_Frequency;   
  uint32_t HCLK_Frequency;     
  uint32_t PCLK1_Frequency;    
  uint32_t PCLK2_Frequency;    
  uint32_t ADCCLK_Frequency;   
}RCC_ClocksTypeDef;



 



 



 









  



 



#line 94 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"



  



 
#line 126 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"

#line 141 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 
#line 165 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"

#line 175 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 




 
#line 191 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"
 






 

#line 283 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"




 

#line 295 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 

#line 317 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


  



 

#line 333 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 

#line 347 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"

#line 364 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"




 




 








 
#line 396 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


#line 423 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"
  



 

#line 435 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 








 



 

#line 462 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 







#line 489 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 

#line 518 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"




  



 

#line 553 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"
 




 



 







#line 586 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"



 



 

#line 606 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"

#line 625 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"




 



 



 



 



 

void RCC_DeInit(void);
void RCC_HSEConfig(uint32_t RCC_HSE);
ErrorStatus RCC_WaitForHSEStartUp(void);
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue);
void RCC_HSICmd(FunctionalState NewState);
void RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t RCC_PLLMul);
void RCC_PLLCmd(FunctionalState NewState);


 void RCC_PREDIV1Config(uint32_t RCC_PREDIV1_Source, uint32_t RCC_PREDIV1_Div);


#line 666 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"

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








 



 



  

 
#line 44 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rtc.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rtc.h"



 



  



  



  



 



 

#line 64 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rtc.h"


  



 

#line 82 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rtc.h"



 



 



 



 



 

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








 



 



 

 
#line 45 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_sdio.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_sdio.h"



 



 



 

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



  



 



 







 



 







  



 







 



 









 



 







 



 






  




 

#line 222 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_sdio.h"


  



 




 



 

#line 245 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_sdio.h"


 



 








 



 






  



 

#line 283 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_sdio.h"


 



 




 



 

#line 330 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_sdio.h"


 



 







 



 







 



 






 



 

#line 421 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_sdio.h"



#line 448 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_sdio.h"





 



 







 



 



 



 



 

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








 



 



 

 
#line 46 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_spi.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_spi.h"



 



  



 



 

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



 



 










 
  
#line 136 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_spi.h"


 



 







 



 







  



 







 



 







 



 







  



 

#line 220 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_spi.h"


  



 







 



 

#line 248 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_spi.h"


 



 

#line 266 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_spi.h"


 



 

#line 282 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_spi.h"


  



 







 



 

#line 312 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_spi.h"






  



 







 



 






 



 







 



 






 



 







 



 

#line 396 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_spi.h"


 



 

#line 417 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_spi.h"


 



 




 



 



 



 



 

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








 



 



 

 
#line 47 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"




















 

 
#line 1152 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



  



 

 
#line 48 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_usart.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_usart.h"



 



  



  



  
  
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



  



  
  
















  
  


                                    




  



  
  
#line 146 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_usart.h"


  



  
  
#line 160 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_usart.h"


  



  
  





  



  
#line 187 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_usart.h"


  



  






  



 
  






  



 







 



 







  



 
  
#line 264 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_usart.h"


 



 







  



 







 



 
  







 



 







  



 

#line 336 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_usart.h"
                              
#line 344 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_usart.h"



  



  



  



  



 

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








  



  



  

 
#line 49 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_wwdg.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_wwdg.h"



 



  



  
  


  



  
  


  
  
#line 68 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_wwdg.h"



  



  



  


  



  
  
void WWDG_DeInit(void);
void WWDG_SetPrescaler(uint32_t WWDG_Prescaler);
void WWDG_SetWindowValue(uint8_t WindowValue);
void WWDG_EnableIT(void);
void WWDG_SetCounter(uint8_t Counter);
void WWDG_Enable(uint8_t Counter);
FlagStatus WWDG_GetFlagStatus(void);
void WWDG_ClearFlag(void);









  



  



  

 
#line 50 "..\\bsp\\stm32f10x_conf.h"
#line 1 "..\\STM32F10x_StdPeriph_Driver\\inc\\misc.h"




















 

 







 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\misc.h"



 



 



 



 

typedef struct
{
  uint8_t NVIC_IRQChannel;                    


 

  uint8_t NVIC_IRQChannelPreemptionPriority;  

 

  uint8_t NVIC_IRQChannelSubPriority;         

 

  FunctionalState NVIC_IRQChannelCmd;         

    
} NVIC_InitTypeDef;
 


 



 
























 



 



 



 







 



 

#line 133 "..\\STM32F10x_StdPeriph_Driver\\inc\\misc.h"


 



 

#line 151 "..\\STM32F10x_StdPeriph_Driver\\inc\\misc.h"















 



 







 



 



 



 



 

void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset);
void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState);
void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource);









 



 



 

 
#line 51 "..\\bsp\\stm32f10x_conf.h"

 
 

 


 








 

 
  void assert_failed(uint8_t* file, uint32_t line);






 
#line 8298 "..\\cortex-m3\\stm32f10x.h"




 

















 









 

  

 

 
#line 33 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"



 



  



  




 

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



 

#line 186 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"

 



 






 
#line 205 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"
									                                 
 
#line 216 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"

                                             
#line 225 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"

 
#line 236 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"

 
#line 249 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"

                                         
#line 266 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"

 
#line 279 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"
                                                                                                                                                                                                                          


  



 

#line 308 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


 



 







  



 

#line 341 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

#line 355 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


 



 

#line 373 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 







 



 
  






 



 







  



 







  



 







  



 







  



 







  



 







  



 







  



 

#line 497 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 







 



 







  



 







  



 







  



 

#line 561 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

#line 577 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

#line 593 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

#line 610 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"

#line 619 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

#line 665 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

#line 709 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

#line 725 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"



  



 

#line 742 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

#line 770 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

#line 784 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



  






 



 







  



 







  



 

#line 833 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  




 

#line 851 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"



  



 

#line 866 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 







  



 





                                     


  



 







  



 

#line 927 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

#line 943 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 







  



 

#line 987 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"
                               
                               



  



 




  



 




  



 

#line 1034 "..\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_tim.h"


 



 



 



  



 

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








  



  



 

 
#line 24 "..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c"
#line 25 "..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c"



 




 



 



 



 

 







 



 



 



 



 



 

static void TI1_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
                       uint16_t TIM_ICFilter);
static void TI2_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
                       uint16_t TIM_ICFilter);
static void TI3_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
                       uint16_t TIM_ICFilter);
static void TI4_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
                       uint16_t TIM_ICFilter);


 



 



 



 



 



 



 



 





 
void TIM_DeInit(TIM_TypeDef* TIMx)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 125)); 
 
  if (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00)))
  {
    RCC_APB2PeriphResetCmd(((uint32_t)0x00000800), ENABLE);
    RCC_APB2PeriphResetCmd(((uint32_t)0x00000800), DISABLE);  
  }     
  else if (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000)))
  {
    RCC_APB1PeriphResetCmd(((uint32_t)0x00000001), ENABLE);
    RCC_APB1PeriphResetCmd(((uint32_t)0x00000001), DISABLE);
  }
  else if (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400)))
  {
    RCC_APB1PeriphResetCmd(((uint32_t)0x00000002), ENABLE);
    RCC_APB1PeriphResetCmd(((uint32_t)0x00000002), DISABLE);
  }
  else if (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800)))
  {
    RCC_APB1PeriphResetCmd(((uint32_t)0x00000004), ENABLE);
    RCC_APB1PeriphResetCmd(((uint32_t)0x00000004), DISABLE);
  } 
  else if (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00)))
  {
    RCC_APB1PeriphResetCmd(((uint32_t)0x00000008), ENABLE);
    RCC_APB1PeriphResetCmd(((uint32_t)0x00000008), DISABLE);
  } 
  else if (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000)))
  {
    RCC_APB1PeriphResetCmd(((uint32_t)0x00000010), ENABLE);
    RCC_APB1PeriphResetCmd(((uint32_t)0x00000010), DISABLE);
  } 
  else if (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400)))
  {
    RCC_APB1PeriphResetCmd(((uint32_t)0x00000020), ENABLE);
    RCC_APB1PeriphResetCmd(((uint32_t)0x00000020), DISABLE);
  } 
  else if (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400)))
  {
    RCC_APB2PeriphResetCmd(((uint32_t)0x00002000), ENABLE);
    RCC_APB2PeriphResetCmd(((uint32_t)0x00002000), DISABLE);
  }
  else if (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00)))
  {      
    RCC_APB2PeriphResetCmd(((uint32_t)0x00080000), ENABLE);
    RCC_APB2PeriphResetCmd(((uint32_t)0x00080000), DISABLE);  
   }  
  else if (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))
  {      
    RCC_APB2PeriphResetCmd(((uint32_t)0x00100000), ENABLE);
    RCC_APB2PeriphResetCmd(((uint32_t)0x00100000), DISABLE);  
  }  
  else if (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400))) 
  {     
    RCC_APB2PeriphResetCmd(((uint32_t)0x00200000), ENABLE);
    RCC_APB2PeriphResetCmd(((uint32_t)0x00200000), DISABLE);  
  }  
  else if (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))
  {      
    RCC_APB1PeriphResetCmd(((uint32_t)0x00000040), ENABLE);
    RCC_APB1PeriphResetCmd(((uint32_t)0x00000040), DISABLE);  
  }  
  else if (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00))) 
  {       
    RCC_APB1PeriphResetCmd(((uint32_t)0x00000080), ENABLE);
    RCC_APB1PeriphResetCmd(((uint32_t)0x00000080), DISABLE);  
  }
  else if (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000))) 
  {       
    RCC_APB1PeriphResetCmd(((uint32_t)0x00000100), ENABLE);
    RCC_APB1PeriphResetCmd(((uint32_t)0x00000100), DISABLE);  
  }        
  else if (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))
  {
    RCC_APB2PeriphResetCmd(((uint32_t)0x00010000), ENABLE);
    RCC_APB2PeriphResetCmd(((uint32_t)0x00010000), DISABLE);
  } 
  else if (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))
  {
    RCC_APB2PeriphResetCmd(((uint32_t)0x00020000), ENABLE);
    RCC_APB2PeriphResetCmd(((uint32_t)0x00020000), DISABLE);
  } 
  else
  {
    if (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800)))
    {
      RCC_APB2PeriphResetCmd(((uint32_t)0x00040000), ENABLE);
      RCC_APB2PeriphResetCmd(((uint32_t)0x00040000), DISABLE);
    }  
  }
}









 
void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct)
{
  uint16_t tmpcr1 = 0;

   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 231)); 
  (((((TIM_TimeBaseInitStruct->TIM_CounterMode) == ((uint16_t)0x0000)) || ((TIM_TimeBaseInitStruct->TIM_CounterMode) == ((uint16_t)0x0010)) || ((TIM_TimeBaseInitStruct->TIM_CounterMode) == ((uint16_t)0x0020)) || ((TIM_TimeBaseInitStruct->TIM_CounterMode) == ((uint16_t)0x0040)) || ((TIM_TimeBaseInitStruct->TIM_CounterMode) == ((uint16_t)0x0060)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 232));
  (((((TIM_TimeBaseInitStruct->TIM_ClockDivision) == ((uint16_t)0x0000)) || ((TIM_TimeBaseInitStruct->TIM_ClockDivision) == ((uint16_t)0x0100)) || ((TIM_TimeBaseInitStruct->TIM_ClockDivision) == ((uint16_t)0x0200)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 233));

  tmpcr1 = TIMx->CR1;  

  if((TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400)))|| (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400)))||
     (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00)))) 
  {
     
    tmpcr1 &= (uint16_t)(~((uint16_t)(((uint16_t)0x0010) | ((uint16_t)0x0060))));
    tmpcr1 |= (uint32_t)TIM_TimeBaseInitStruct->TIM_CounterMode;
  }
 
  if((TIMx != ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) && (TIMx != ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))))
  {
     
    tmpcr1 &= (uint16_t)(~((uint16_t)((uint16_t)0x0300)));
    tmpcr1 |= (uint32_t)TIM_TimeBaseInitStruct->TIM_ClockDivision;
  }

  TIMx->CR1 = tmpcr1;

   
  TIMx->ARR = TIM_TimeBaseInitStruct->TIM_Period ;
 
   
  TIMx->PSC = TIM_TimeBaseInitStruct->TIM_Prescaler;
    
  if ((TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400)))|| (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400))) || (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))  
  {
     
    TIMx->RCR = TIM_TimeBaseInitStruct->TIM_RepetitionCounter;
  }

  
 
  TIMx->EGR = ((uint16_t)0x0001);           
}








 
void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct)
{
  uint16_t tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;
   
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 284));
  (((((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0000)) || ((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0010)) || ((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0020)) || ((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0030))|| ((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0060)) || ((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0070)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 285));
  (((((TIM_OCInitStruct->TIM_OutputState) == ((uint16_t)0x0000)) || ((TIM_OCInitStruct->TIM_OutputState) == ((uint16_t)0x0001)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 286));
  (((((TIM_OCInitStruct->TIM_OCPolarity) == ((uint16_t)0x0000)) || ((TIM_OCInitStruct->TIM_OCPolarity) == ((uint16_t)0x0002)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 287));   
  
  TIMx->CCER &= (uint16_t)(~(uint16_t)((uint16_t)0x0001));
   
  tmpccer = TIMx->CCER;
   
  tmpcr2 =  TIMx->CR2;
  
   
  tmpccmrx = TIMx->CCMR1;
    
   
  tmpccmrx &= (uint16_t)(~((uint16_t)((uint16_t)0x0070)));
  tmpccmrx &= (uint16_t)(~((uint16_t)((uint16_t)0x0003)));

   
  tmpccmrx |= TIM_OCInitStruct->TIM_OCMode;
  
   
  tmpccer &= (uint16_t)(~((uint16_t)((uint16_t)0x0002)));
   
  tmpccer |= TIM_OCInitStruct->TIM_OCPolarity;
  
   
  tmpccer |= TIM_OCInitStruct->TIM_OutputState;
    
  if((TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400)))|| (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))||
     (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))
  {
    (((((TIM_OCInitStruct->TIM_OutputNState) == ((uint16_t)0x0000)) || ((TIM_OCInitStruct->TIM_OutputNState) == ((uint16_t)0x0004)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 316));
    (((((TIM_OCInitStruct->TIM_OCNPolarity) == ((uint16_t)0x0000)) || ((TIM_OCInitStruct->TIM_OCNPolarity) == ((uint16_t)0x0008)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 317));
    (((((TIM_OCInitStruct->TIM_OCNIdleState) == ((uint16_t)0x0200)) || ((TIM_OCInitStruct->TIM_OCNIdleState) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 318));
    (((((TIM_OCInitStruct->TIM_OCIdleState) == ((uint16_t)0x0100)) || ((TIM_OCInitStruct->TIM_OCIdleState) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 319));
    
     
    tmpccer &= (uint16_t)(~((uint16_t)((uint16_t)0x0008)));
     
    tmpccer |= TIM_OCInitStruct->TIM_OCNPolarity;
    
     
    tmpccer &= (uint16_t)(~((uint16_t)((uint16_t)0x0004)));    
     
    tmpccer |= TIM_OCInitStruct->TIM_OutputNState;
    
     
    tmpcr2 &= (uint16_t)(~((uint16_t)((uint16_t)0x0100)));
    tmpcr2 &= (uint16_t)(~((uint16_t)((uint16_t)0x0200)));
    
     
    tmpcr2 |= TIM_OCInitStruct->TIM_OCIdleState;
     
    tmpcr2 |= TIM_OCInitStruct->TIM_OCNIdleState;
  }
   
  TIMx->CR2 = tmpcr2;
  
   
  TIMx->CCMR1 = tmpccmrx;

   
  TIMx->CCR1 = TIM_OCInitStruct->TIM_Pulse; 
 
   
  TIMx->CCER = tmpccer;
}









 
void TIM_OC2Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct)
{
  uint16_t tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;
   
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 367)); 
  (((((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0000)) || ((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0010)) || ((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0020)) || ((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0030))|| ((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0060)) || ((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0070)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 368));
  (((((TIM_OCInitStruct->TIM_OutputState) == ((uint16_t)0x0000)) || ((TIM_OCInitStruct->TIM_OutputState) == ((uint16_t)0x0001)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 369));
  (((((TIM_OCInitStruct->TIM_OCPolarity) == ((uint16_t)0x0000)) || ((TIM_OCInitStruct->TIM_OCPolarity) == ((uint16_t)0x0002)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 370));   
    
  TIMx->CCER &= (uint16_t)(~((uint16_t)((uint16_t)0x0010)));
  
     
  tmpccer = TIMx->CCER;
   
  tmpcr2 =  TIMx->CR2;
  
   
  tmpccmrx = TIMx->CCMR1;
    
   
  tmpccmrx &= (uint16_t)(~((uint16_t)((uint16_t)0x7000)));
  tmpccmrx &= (uint16_t)(~((uint16_t)((uint16_t)0x0300)));
  
   
  tmpccmrx |= (uint16_t)(TIM_OCInitStruct->TIM_OCMode << 8);
  
   
  tmpccer &= (uint16_t)(~((uint16_t)((uint16_t)0x0020)));
   
  tmpccer |= (uint16_t)(TIM_OCInitStruct->TIM_OCPolarity << 4);
  
   
  tmpccer |= (uint16_t)(TIM_OCInitStruct->TIM_OutputState << 4);
    
  if((TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))
  {
    (((((TIM_OCInitStruct->TIM_OutputNState) == ((uint16_t)0x0000)) || ((TIM_OCInitStruct->TIM_OutputNState) == ((uint16_t)0x0004)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 399));
    (((((TIM_OCInitStruct->TIM_OCNPolarity) == ((uint16_t)0x0000)) || ((TIM_OCInitStruct->TIM_OCNPolarity) == ((uint16_t)0x0008)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 400));
    (((((TIM_OCInitStruct->TIM_OCNIdleState) == ((uint16_t)0x0200)) || ((TIM_OCInitStruct->TIM_OCNIdleState) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 401));
    (((((TIM_OCInitStruct->TIM_OCIdleState) == ((uint16_t)0x0100)) || ((TIM_OCInitStruct->TIM_OCIdleState) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 402));
    
     
    tmpccer &= (uint16_t)(~((uint16_t)((uint16_t)0x0080)));
     
    tmpccer |= (uint16_t)(TIM_OCInitStruct->TIM_OCNPolarity << 4);
    
     
    tmpccer &= (uint16_t)(~((uint16_t)((uint16_t)0x0040)));    
     
    tmpccer |= (uint16_t)(TIM_OCInitStruct->TIM_OutputNState << 4);
    
     
    tmpcr2 &= (uint16_t)(~((uint16_t)((uint16_t)0x0400)));
    tmpcr2 &= (uint16_t)(~((uint16_t)((uint16_t)0x0800)));
    
     
    tmpcr2 |= (uint16_t)(TIM_OCInitStruct->TIM_OCIdleState << 2);
     
    tmpcr2 |= (uint16_t)(TIM_OCInitStruct->TIM_OCNIdleState << 2);
  }
   
  TIMx->CR2 = tmpcr2;
  
   
  TIMx->CCMR1 = tmpccmrx;

   
  TIMx->CCR2 = TIM_OCInitStruct->TIM_Pulse;
  
   
  TIMx->CCER = tmpccer;
}








 
void TIM_OC3Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct)
{
  uint16_t tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;
   
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 449)); 
  (((((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0000)) || ((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0010)) || ((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0020)) || ((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0030))|| ((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0060)) || ((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0070)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 450));
  (((((TIM_OCInitStruct->TIM_OutputState) == ((uint16_t)0x0000)) || ((TIM_OCInitStruct->TIM_OutputState) == ((uint16_t)0x0001)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 451));
  (((((TIM_OCInitStruct->TIM_OCPolarity) == ((uint16_t)0x0000)) || ((TIM_OCInitStruct->TIM_OCPolarity) == ((uint16_t)0x0002)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 452));   
   
  TIMx->CCER &= (uint16_t)(~((uint16_t)((uint16_t)0x0100)));
  
   
  tmpccer = TIMx->CCER;
   
  tmpcr2 =  TIMx->CR2;
  
   
  tmpccmrx = TIMx->CCMR2;
    
   
  tmpccmrx &= (uint16_t)(~((uint16_t)((uint16_t)0x0070)));
  tmpccmrx &= (uint16_t)(~((uint16_t)((uint16_t)0x0003)));  
   
  tmpccmrx |= TIM_OCInitStruct->TIM_OCMode;
  
   
  tmpccer &= (uint16_t)(~((uint16_t)((uint16_t)0x0200)));
   
  tmpccer |= (uint16_t)(TIM_OCInitStruct->TIM_OCPolarity << 8);
  
   
  tmpccer |= (uint16_t)(TIM_OCInitStruct->TIM_OutputState << 8);
    
  if((TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))
  {
    (((((TIM_OCInitStruct->TIM_OutputNState) == ((uint16_t)0x0000)) || ((TIM_OCInitStruct->TIM_OutputNState) == ((uint16_t)0x0004)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 480));
    (((((TIM_OCInitStruct->TIM_OCNPolarity) == ((uint16_t)0x0000)) || ((TIM_OCInitStruct->TIM_OCNPolarity) == ((uint16_t)0x0008)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 481));
    (((((TIM_OCInitStruct->TIM_OCNIdleState) == ((uint16_t)0x0200)) || ((TIM_OCInitStruct->TIM_OCNIdleState) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 482));
    (((((TIM_OCInitStruct->TIM_OCIdleState) == ((uint16_t)0x0100)) || ((TIM_OCInitStruct->TIM_OCIdleState) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 483));
    
     
    tmpccer &= (uint16_t)(~((uint16_t)((uint16_t)0x0800)));
     
    tmpccer |= (uint16_t)(TIM_OCInitStruct->TIM_OCNPolarity << 8);
     
    tmpccer &= (uint16_t)(~((uint16_t)((uint16_t)0x0400)));
    
     
    tmpccer |= (uint16_t)(TIM_OCInitStruct->TIM_OutputNState << 8);
     
    tmpcr2 &= (uint16_t)(~((uint16_t)((uint16_t)0x1000)));
    tmpcr2 &= (uint16_t)(~((uint16_t)((uint16_t)0x2000)));
     
    tmpcr2 |= (uint16_t)(TIM_OCInitStruct->TIM_OCIdleState << 4);
     
    tmpcr2 |= (uint16_t)(TIM_OCInitStruct->TIM_OCNIdleState << 4);
  }
   
  TIMx->CR2 = tmpcr2;
  
   
  TIMx->CCMR2 = tmpccmrx;

   
  TIMx->CCR3 = TIM_OCInitStruct->TIM_Pulse;
  
   
  TIMx->CCER = tmpccer;
}








 
void TIM_OC4Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct)
{
  uint16_t tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;
   
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 528)); 
  (((((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0000)) || ((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0010)) || ((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0020)) || ((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0030))|| ((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0060)) || ((TIM_OCInitStruct->TIM_OCMode) == ((uint16_t)0x0070)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 529));
  (((((TIM_OCInitStruct->TIM_OutputState) == ((uint16_t)0x0000)) || ((TIM_OCInitStruct->TIM_OutputState) == ((uint16_t)0x0001)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 530));
  (((((TIM_OCInitStruct->TIM_OCPolarity) == ((uint16_t)0x0000)) || ((TIM_OCInitStruct->TIM_OCPolarity) == ((uint16_t)0x0002)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 531));   
   
  TIMx->CCER &= (uint16_t)(~((uint16_t)((uint16_t)0x1000)));
  
   
  tmpccer = TIMx->CCER;
   
  tmpcr2 =  TIMx->CR2;
  
   
  tmpccmrx = TIMx->CCMR2;
    
   
  tmpccmrx &= (uint16_t)(~((uint16_t)((uint16_t)0x7000)));
  tmpccmrx &= (uint16_t)(~((uint16_t)((uint16_t)0x0300)));
  
   
  tmpccmrx |= (uint16_t)(TIM_OCInitStruct->TIM_OCMode << 8);
  
   
  tmpccer &= (uint16_t)(~((uint16_t)((uint16_t)0x2000)));
   
  tmpccer |= (uint16_t)(TIM_OCInitStruct->TIM_OCPolarity << 12);
  
   
  tmpccer |= (uint16_t)(TIM_OCInitStruct->TIM_OutputState << 12);
    
  if((TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))
  {
    (((((TIM_OCInitStruct->TIM_OCIdleState) == ((uint16_t)0x0100)) || ((TIM_OCInitStruct->TIM_OCIdleState) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 560));
     
    tmpcr2 &= (uint16_t)(~((uint16_t)((uint16_t)0x4000)));
     
    tmpcr2 |= (uint16_t)(TIM_OCInitStruct->TIM_OCIdleState << 6);
  }
   
  TIMx->CR2 = tmpcr2;
  
     
  TIMx->CCMR2 = tmpccmrx;

   
  TIMx->CCR4 = TIM_OCInitStruct->TIM_Pulse;
  
   
  TIMx->CCER = tmpccer;
}








 
void TIM_ICInit(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct)
{
   
  (((((TIM_ICInitStruct->TIM_Channel) == ((uint16_t)0x0000)) || ((TIM_ICInitStruct->TIM_Channel) == ((uint16_t)0x0004)) || ((TIM_ICInitStruct->TIM_Channel) == ((uint16_t)0x0008)) || ((TIM_ICInitStruct->TIM_Channel) == ((uint16_t)0x000C)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 590));  
  (((((TIM_ICInitStruct->TIM_ICSelection) == ((uint16_t)0x0001)) || ((TIM_ICInitStruct->TIM_ICSelection) == ((uint16_t)0x0002)) || ((TIM_ICInitStruct->TIM_ICSelection) == ((uint16_t)0x0003)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 591));
  (((((TIM_ICInitStruct->TIM_ICPrescaler) == ((uint16_t)0x0000)) || ((TIM_ICInitStruct->TIM_ICPrescaler) == ((uint16_t)0x0004)) || ((TIM_ICInitStruct->TIM_ICPrescaler) == ((uint16_t)0x0008)) || ((TIM_ICInitStruct->TIM_ICPrescaler) == ((uint16_t)0x000C)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 592));
  ((((TIM_ICInitStruct->TIM_ICFilter) <= 0xF)) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 593));
  
  if((TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) ||
     (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) ||(TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))))
  {
    (((((TIM_ICInitStruct->TIM_ICPolarity) == ((uint16_t)0x0000)) || ((TIM_ICInitStruct->TIM_ICPolarity) == ((uint16_t)0x0002)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 598));
  }
  else
  {
    (((((TIM_ICInitStruct->TIM_ICPolarity) == ((uint16_t)0x0000)) || ((TIM_ICInitStruct->TIM_ICPolarity) == ((uint16_t)0x0002))|| ((TIM_ICInitStruct->TIM_ICPolarity) == ((uint16_t)0x000A)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 602));
  }
  if (TIM_ICInitStruct->TIM_Channel == ((uint16_t)0x0000))
  {
    (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 606));
     
    TI1_Config(TIMx, TIM_ICInitStruct->TIM_ICPolarity,
               TIM_ICInitStruct->TIM_ICSelection,
               TIM_ICInitStruct->TIM_ICFilter);
     
    TIM_SetIC1Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
  }
  else if (TIM_ICInitStruct->TIM_Channel == ((uint16_t)0x0004))
  {
    (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 616));
     
    TI2_Config(TIMx, TIM_ICInitStruct->TIM_ICPolarity,
               TIM_ICInitStruct->TIM_ICSelection,
               TIM_ICInitStruct->TIM_ICFilter);
     
    TIM_SetIC2Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
  }
  else if (TIM_ICInitStruct->TIM_Channel == ((uint16_t)0x0008))
  {
    (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 626));
     
    TI3_Config(TIMx,  TIM_ICInitStruct->TIM_ICPolarity,
               TIM_ICInitStruct->TIM_ICSelection,
               TIM_ICInitStruct->TIM_ICFilter);
     
    TIM_SetIC3Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
  }
  else
  {
    (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 636));
     
    TI4_Config(TIMx, TIM_ICInitStruct->TIM_ICPolarity,
               TIM_ICInitStruct->TIM_ICSelection,
               TIM_ICInitStruct->TIM_ICFilter);
     
    TIM_SetIC4Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
  }
}








 
void TIM_PWMIConfig(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct)
{
  uint16_t icoppositepolarity = ((uint16_t)0x0000);
  uint16_t icoppositeselection = ((uint16_t)0x0001);
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 659));
   
  if (TIM_ICInitStruct->TIM_ICPolarity == ((uint16_t)0x0000))
  {
    icoppositepolarity = ((uint16_t)0x0002);
  }
  else
  {
    icoppositepolarity = ((uint16_t)0x0000);
  }
   
  if (TIM_ICInitStruct->TIM_ICSelection == ((uint16_t)0x0001))
  {
    icoppositeselection = ((uint16_t)0x0002);
  }
  else
  {
    icoppositeselection = ((uint16_t)0x0001);
  }
  if (TIM_ICInitStruct->TIM_Channel == ((uint16_t)0x0000))
  {
     
    TI1_Config(TIMx, TIM_ICInitStruct->TIM_ICPolarity, TIM_ICInitStruct->TIM_ICSelection,
               TIM_ICInitStruct->TIM_ICFilter);
     
    TIM_SetIC1Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
     
    TI2_Config(TIMx, icoppositepolarity, icoppositeselection, TIM_ICInitStruct->TIM_ICFilter);
     
    TIM_SetIC2Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
  }
  else
  { 
     
    TI2_Config(TIMx, TIM_ICInitStruct->TIM_ICPolarity, TIM_ICInitStruct->TIM_ICSelection,
               TIM_ICInitStruct->TIM_ICFilter);
     
    TIM_SetIC2Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
     
    TI1_Config(TIMx, icoppositepolarity, icoppositeselection, TIM_ICInitStruct->TIM_ICFilter);
     
    TIM_SetIC1Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
  }
}








 
void TIM_BDTRConfig(TIM_TypeDef* TIMx, TIM_BDTRInitTypeDef *TIM_BDTRInitStruct)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 715));
  (((((TIM_BDTRInitStruct->TIM_OSSRState) == ((uint16_t)0x0800)) || ((TIM_BDTRInitStruct->TIM_OSSRState) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 716));
  (((((TIM_BDTRInitStruct->TIM_OSSIState) == ((uint16_t)0x0400)) || ((TIM_BDTRInitStruct->TIM_OSSIState) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 717));
  (((((TIM_BDTRInitStruct->TIM_LOCKLevel) == ((uint16_t)0x0000)) || ((TIM_BDTRInitStruct->TIM_LOCKLevel) == ((uint16_t)0x0100)) || ((TIM_BDTRInitStruct->TIM_LOCKLevel) == ((uint16_t)0x0200)) || ((TIM_BDTRInitStruct->TIM_LOCKLevel) == ((uint16_t)0x0300)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 718));
  (((((TIM_BDTRInitStruct->TIM_Break) == ((uint16_t)0x1000)) || ((TIM_BDTRInitStruct->TIM_Break) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 719));
  (((((TIM_BDTRInitStruct->TIM_BreakPolarity) == ((uint16_t)0x0000)) || ((TIM_BDTRInitStruct->TIM_BreakPolarity) == ((uint16_t)0x2000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 720));
  (((((TIM_BDTRInitStruct->TIM_AutomaticOutput) == ((uint16_t)0x4000)) || ((TIM_BDTRInitStruct->TIM_AutomaticOutput) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 721));
  
 
  TIMx->BDTR = (uint32_t)TIM_BDTRInitStruct->TIM_OSSRState | TIM_BDTRInitStruct->TIM_OSSIState |
             TIM_BDTRInitStruct->TIM_LOCKLevel | TIM_BDTRInitStruct->TIM_DeadTime |
             TIM_BDTRInitStruct->TIM_Break | TIM_BDTRInitStruct->TIM_BreakPolarity |
             TIM_BDTRInitStruct->TIM_AutomaticOutput;
}






 
void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct)
{
   
  TIM_TimeBaseInitStruct->TIM_Period = 0xFFFF;
  TIM_TimeBaseInitStruct->TIM_Prescaler = 0x0000;
  TIM_TimeBaseInitStruct->TIM_ClockDivision = ((uint16_t)0x0000);
  TIM_TimeBaseInitStruct->TIM_CounterMode = ((uint16_t)0x0000);
  TIM_TimeBaseInitStruct->TIM_RepetitionCounter = 0x0000;
}






 
void TIM_OCStructInit(TIM_OCInitTypeDef* TIM_OCInitStruct)
{
   
  TIM_OCInitStruct->TIM_OCMode = ((uint16_t)0x0000);
  TIM_OCInitStruct->TIM_OutputState = ((uint16_t)0x0000);
  TIM_OCInitStruct->TIM_OutputNState = ((uint16_t)0x0000);
  TIM_OCInitStruct->TIM_Pulse = 0x0000;
  TIM_OCInitStruct->TIM_OCPolarity = ((uint16_t)0x0000);
  TIM_OCInitStruct->TIM_OCNPolarity = ((uint16_t)0x0000);
  TIM_OCInitStruct->TIM_OCIdleState = ((uint16_t)0x0000);
  TIM_OCInitStruct->TIM_OCNIdleState = ((uint16_t)0x0000);
}






 
void TIM_ICStructInit(TIM_ICInitTypeDef* TIM_ICInitStruct)
{
   
  TIM_ICInitStruct->TIM_Channel = ((uint16_t)0x0000);
  TIM_ICInitStruct->TIM_ICPolarity = ((uint16_t)0x0000);
  TIM_ICInitStruct->TIM_ICSelection = ((uint16_t)0x0001);
  TIM_ICInitStruct->TIM_ICPrescaler = ((uint16_t)0x0000);
  TIM_ICInitStruct->TIM_ICFilter = 0x00;
}






 
void TIM_BDTRStructInit(TIM_BDTRInitTypeDef* TIM_BDTRInitStruct)
{
   
  TIM_BDTRInitStruct->TIM_OSSRState = ((uint16_t)0x0000);
  TIM_BDTRInitStruct->TIM_OSSIState = ((uint16_t)0x0000);
  TIM_BDTRInitStruct->TIM_LOCKLevel = ((uint16_t)0x0000);
  TIM_BDTRInitStruct->TIM_DeadTime = 0x00;
  TIM_BDTRInitStruct->TIM_Break = ((uint16_t)0x0000);
  TIM_BDTRInitStruct->TIM_BreakPolarity = ((uint16_t)0x0000);
  TIM_BDTRInitStruct->TIM_AutomaticOutput = ((uint16_t)0x0000);
}







 
void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 809));
  (((((NewState) == DISABLE) || ((NewState) == ENABLE))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 810));
  
  if (NewState != DISABLE)
  {
     
    TIMx->CR1 |= ((uint16_t)0x0001);
  }
  else
  {
     
    TIMx->CR1 &= (uint16_t)(~((uint16_t)((uint16_t)0x0001)));
  }
}







 
void TIM_CtrlPWMOutputs(TIM_TypeDef* TIMx, FunctionalState NewState)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 834));
  (((((NewState) == DISABLE) || ((NewState) == ENABLE))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 835));
  if (NewState != DISABLE)
  {
     
    TIMx->BDTR |= ((uint16_t)0x8000);
  }
  else
  {
     
    TIMx->BDTR &= (uint16_t)(~((uint16_t)((uint16_t)0x8000)));
  }  
}
























 
void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState)
{  
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 875));
  ((((((TIM_IT) & (uint16_t)0xFF00) == 0x0000) && ((TIM_IT) != 0x0000))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 876));
  (((((NewState) == DISABLE) || ((NewState) == ENABLE))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 877));
  
  if (NewState != DISABLE)
  {
     
    TIMx->DIER |= TIM_IT;
  }
  else
  {
     
    TIMx->DIER &= (uint16_t)~TIM_IT;
  }
}


















 
void TIM_GenerateEvent(TIM_TypeDef* TIMx, uint16_t TIM_EventSource)
{ 
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 912));
  ((((((TIM_EventSource) & (uint16_t)0xFF00) == 0x0000) && ((TIM_EventSource) != 0x0000))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 913));
  
   
  TIMx->EGR = TIM_EventSource;
}


















 
void TIM_DMAConfig(TIM_TypeDef* TIMx, uint16_t TIM_DMABase, uint16_t TIM_DMABurstLength)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 940));
  (((((TIM_DMABase) == ((uint16_t)0x0000)) || ((TIM_DMABase) == ((uint16_t)0x0001)) || ((TIM_DMABase) == ((uint16_t)0x0002)) || ((TIM_DMABase) == ((uint16_t)0x0003)) || ((TIM_DMABase) == ((uint16_t)0x0004)) || ((TIM_DMABase) == ((uint16_t)0x0005)) || ((TIM_DMABase) == ((uint16_t)0x0006)) || ((TIM_DMABase) == ((uint16_t)0x0007)) || ((TIM_DMABase) == ((uint16_t)0x0008)) || ((TIM_DMABase) == ((uint16_t)0x0009)) || ((TIM_DMABase) == ((uint16_t)0x000A)) || ((TIM_DMABase) == ((uint16_t)0x000B)) || ((TIM_DMABase) == ((uint16_t)0x000C)) || ((TIM_DMABase) == ((uint16_t)0x000D)) || ((TIM_DMABase) == ((uint16_t)0x000E)) || ((TIM_DMABase) == ((uint16_t)0x000F)) || ((TIM_DMABase) == ((uint16_t)0x0010)) || ((TIM_DMABase) == ((uint16_t)0x0011)) || ((TIM_DMABase) == ((uint16_t)0x0012)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 941));
  (((((TIM_DMABurstLength) == ((uint16_t)0x0000)) || ((TIM_DMABurstLength) == ((uint16_t)0x0100)) || ((TIM_DMABurstLength) == ((uint16_t)0x0200)) || ((TIM_DMABurstLength) == ((uint16_t)0x0300)) || ((TIM_DMABurstLength) == ((uint16_t)0x0400)) || ((TIM_DMABurstLength) == ((uint16_t)0x0500)) || ((TIM_DMABurstLength) == ((uint16_t)0x0600)) || ((TIM_DMABurstLength) == ((uint16_t)0x0700)) || ((TIM_DMABurstLength) == ((uint16_t)0x0800)) || ((TIM_DMABurstLength) == ((uint16_t)0x0900)) || ((TIM_DMABurstLength) == ((uint16_t)0x0A00)) || ((TIM_DMABurstLength) == ((uint16_t)0x0B00)) || ((TIM_DMABurstLength) == ((uint16_t)0x0C00)) || ((TIM_DMABurstLength) == ((uint16_t)0x0D00)) || ((TIM_DMABurstLength) == ((uint16_t)0x0E00)) || ((TIM_DMABurstLength) == ((uint16_t)0x0F00)) || ((TIM_DMABurstLength) == ((uint16_t)0x1000)) || ((TIM_DMABurstLength) == ((uint16_t)0x1100)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 942));
   
  TIMx->DCR = TIM_DMABase | TIM_DMABurstLength;
}

















 
void TIM_DMACmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState NewState)
{ 
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 967));
  ((((((TIM_DMASource) & (uint16_t)0x80FF) == 0x0000) && ((TIM_DMASource) != 0x0000))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 968));
  (((((NewState) == DISABLE) || ((NewState) == ENABLE))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 969));
  
  if (NewState != DISABLE)
  {
     
    TIMx->DIER |= TIM_DMASource; 
  }
  else
  {
     
    TIMx->DIER &= (uint16_t)~TIM_DMASource;
  }
}






 
void TIM_InternalClockConfig(TIM_TypeDef* TIMx)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 992));
   
  TIMx->SMCR &=  (uint16_t)(~((uint16_t)((uint16_t)0x0007)));
}











 
void TIM_ITRxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1011));
  (((((TIM_InputTriggerSource) == ((uint16_t)0x0000)) || ((TIM_InputTriggerSource) == ((uint16_t)0x0010)) || ((TIM_InputTriggerSource) == ((uint16_t)0x0020)) || ((TIM_InputTriggerSource) == ((uint16_t)0x0030)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1012));
   
  TIM_SelectInputTrigger(TIMx, TIM_InputTriggerSource);
   
  TIMx->SMCR |= ((uint16_t)0x0007);
}
















 
void TIM_TIxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_TIxExternalCLKSource,
                                uint16_t TIM_ICPolarity, uint16_t ICFilter)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1039));
  (((((TIM_TIxExternalCLKSource) == ((uint16_t)0x0050)) || ((TIM_TIxExternalCLKSource) == ((uint16_t)0x0060)) || ((TIM_TIxExternalCLKSource) == ((uint16_t)0x0040)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1040));
  (((((TIM_ICPolarity) == ((uint16_t)0x0000)) || ((TIM_ICPolarity) == ((uint16_t)0x0002)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1041));
  ((((ICFilter) <= 0xF)) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1042));
   
  if (TIM_TIxExternalCLKSource == ((uint16_t)0x0060))
  {
    TI2_Config(TIMx, TIM_ICPolarity, ((uint16_t)0x0001), ICFilter);
  }
  else
  {
    TI1_Config(TIMx, TIM_ICPolarity, ((uint16_t)0x0001), ICFilter);
  }
   
  TIM_SelectInputTrigger(TIMx, TIM_TIxExternalCLKSource);
   
  TIMx->SMCR |= ((uint16_t)0x0007);
}

















 
void TIM_ETRClockMode1Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                             uint16_t ExtTRGFilter)
{
  uint16_t tmpsmcr = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1080));
  (((((TIM_ExtTRGPrescaler) == ((uint16_t)0x0000)) || ((TIM_ExtTRGPrescaler) == ((uint16_t)0x1000)) || ((TIM_ExtTRGPrescaler) == ((uint16_t)0x2000)) || ((TIM_ExtTRGPrescaler) == ((uint16_t)0x3000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1081));
  (((((TIM_ExtTRGPolarity) == ((uint16_t)0x8000)) || ((TIM_ExtTRGPolarity) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1082));
  ((((ExtTRGFilter) <= 0xF)) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1083));
   
  TIM_ETRConfig(TIMx, TIM_ExtTRGPrescaler, TIM_ExtTRGPolarity, ExtTRGFilter);
  
   
  tmpsmcr = TIMx->SMCR;
   
  tmpsmcr &= (uint16_t)(~((uint16_t)((uint16_t)0x0007)));
   
  tmpsmcr |= ((uint16_t)0x0007);
   
  tmpsmcr &= (uint16_t)(~((uint16_t)((uint16_t)0x0070)));
  tmpsmcr |= ((uint16_t)0x0070);
   
  TIMx->SMCR = tmpsmcr;
}

















 
void TIM_ETRClockMode2Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, 
                             uint16_t TIM_ExtTRGPolarity, uint16_t ExtTRGFilter)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1121));
  (((((TIM_ExtTRGPrescaler) == ((uint16_t)0x0000)) || ((TIM_ExtTRGPrescaler) == ((uint16_t)0x1000)) || ((TIM_ExtTRGPrescaler) == ((uint16_t)0x2000)) || ((TIM_ExtTRGPrescaler) == ((uint16_t)0x3000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1122));
  (((((TIM_ExtTRGPolarity) == ((uint16_t)0x8000)) || ((TIM_ExtTRGPolarity) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1123));
  ((((ExtTRGFilter) <= 0xF)) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1124));
   
  TIM_ETRConfig(TIMx, TIM_ExtTRGPrescaler, TIM_ExtTRGPolarity, ExtTRGFilter);
   
  TIMx->SMCR |= ((uint16_t)0x4000);
}

















 
void TIM_ETRConfig(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                   uint16_t ExtTRGFilter)
{
  uint16_t tmpsmcr = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1153));
  (((((TIM_ExtTRGPrescaler) == ((uint16_t)0x0000)) || ((TIM_ExtTRGPrescaler) == ((uint16_t)0x1000)) || ((TIM_ExtTRGPrescaler) == ((uint16_t)0x2000)) || ((TIM_ExtTRGPrescaler) == ((uint16_t)0x3000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1154));
  (((((TIM_ExtTRGPolarity) == ((uint16_t)0x8000)) || ((TIM_ExtTRGPolarity) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1155));
  ((((ExtTRGFilter) <= 0xF)) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1156));
  tmpsmcr = TIMx->SMCR;
   
  tmpsmcr &= ((uint16_t)0x00FF);
   
  tmpsmcr |= (uint16_t)(TIM_ExtTRGPrescaler | (uint16_t)(TIM_ExtTRGPolarity | (uint16_t)(ExtTRGFilter << (uint16_t)8)));
   
  TIMx->SMCR = tmpsmcr;
}










 
void TIM_PrescalerConfig(TIM_TypeDef* TIMx, uint16_t Prescaler, uint16_t TIM_PSCReloadMode)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1179));
  (((((TIM_PSCReloadMode) == ((uint16_t)0x0000)) || ((TIM_PSCReloadMode) == ((uint16_t)0x0001)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1180));
   
  TIMx->PSC = Prescaler;
   
  TIMx->EGR = TIM_PSCReloadMode;
}












 
void TIM_CounterModeConfig(TIM_TypeDef* TIMx, uint16_t TIM_CounterMode)
{
  uint16_t tmpcr1 = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1203));
  (((((TIM_CounterMode) == ((uint16_t)0x0000)) || ((TIM_CounterMode) == ((uint16_t)0x0010)) || ((TIM_CounterMode) == ((uint16_t)0x0020)) || ((TIM_CounterMode) == ((uint16_t)0x0040)) || ((TIM_CounterMode) == ((uint16_t)0x0060)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1204));
  tmpcr1 = TIMx->CR1;
   
  tmpcr1 &= (uint16_t)(~((uint16_t)(((uint16_t)0x0010) | ((uint16_t)0x0060))));
   
  tmpcr1 |= TIM_CounterMode;
   
  TIMx->CR1 = tmpcr1;
}















 
void TIM_SelectInputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource)
{
  uint16_t tmpsmcr = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1233));
  (((((TIM_InputTriggerSource) == ((uint16_t)0x0000)) || ((TIM_InputTriggerSource) == ((uint16_t)0x0010)) || ((TIM_InputTriggerSource) == ((uint16_t)0x0020)) || ((TIM_InputTriggerSource) == ((uint16_t)0x0030)) || ((TIM_InputTriggerSource) == ((uint16_t)0x0040)) || ((TIM_InputTriggerSource) == ((uint16_t)0x0050)) || ((TIM_InputTriggerSource) == ((uint16_t)0x0060)) || ((TIM_InputTriggerSource) == ((uint16_t)0x0070)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1234));
   
  tmpsmcr = TIMx->SMCR;
   
  tmpsmcr &= (uint16_t)(~((uint16_t)((uint16_t)0x0070)));
   
  tmpsmcr |= TIM_InputTriggerSource;
   
  TIMx->SMCR = tmpsmcr;
}



















 
void TIM_EncoderInterfaceConfig(TIM_TypeDef* TIMx, uint16_t TIM_EncoderMode,
                                uint16_t TIM_IC1Polarity, uint16_t TIM_IC2Polarity)
{
  uint16_t tmpsmcr = 0;
  uint16_t tmpccmr1 = 0;
  uint16_t tmpccer = 0;
    
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1272));
  (((((TIM_EncoderMode) == ((uint16_t)0x0001)) || ((TIM_EncoderMode) == ((uint16_t)0x0002)) || ((TIM_EncoderMode) == ((uint16_t)0x0003)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1273));
  (((((TIM_IC1Polarity) == ((uint16_t)0x0000)) || ((TIM_IC1Polarity) == ((uint16_t)0x0002)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1274));
  (((((TIM_IC2Polarity) == ((uint16_t)0x0000)) || ((TIM_IC2Polarity) == ((uint16_t)0x0002)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1275));

   
  tmpsmcr = TIMx->SMCR;
  
   
  tmpccmr1 = TIMx->CCMR1;
  
   
  tmpccer = TIMx->CCER;
  
   
  tmpsmcr &= (uint16_t)(~((uint16_t)((uint16_t)0x0007)));
  tmpsmcr |= TIM_EncoderMode;
  
   
  tmpccmr1 &= (uint16_t)(((uint16_t)~((uint16_t)((uint16_t)0x0003))) & (uint16_t)(~((uint16_t)((uint16_t)0x0300))));
  tmpccmr1 |= ((uint16_t)0x0001) | ((uint16_t)0x0100);
  
   
  tmpccer &= (uint16_t)(((uint16_t)~((uint16_t)((uint16_t)0x0002))) & ((uint16_t)~((uint16_t)((uint16_t)0x0020))));
  tmpccer |= (uint16_t)(TIM_IC1Polarity | (uint16_t)(TIM_IC2Polarity << (uint16_t)4));
  
   
  TIMx->SMCR = tmpsmcr;
   
  TIMx->CCMR1 = tmpccmr1;
   
  TIMx->CCER = tmpccer;
}









 
void TIM_ForcedOC1Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction)
{
  uint16_t tmpccmr1 = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1319));
  (((((TIM_ForcedAction) == ((uint16_t)0x0050)) || ((TIM_ForcedAction) == ((uint16_t)0x0040)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1320));
  tmpccmr1 = TIMx->CCMR1;
   
  tmpccmr1 &= (uint16_t)~((uint16_t)((uint16_t)0x0070));
   
  tmpccmr1 |= TIM_ForcedAction;
   
  TIMx->CCMR1 = tmpccmr1;
}









 
void TIM_ForcedOC2Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction)
{
  uint16_t tmpccmr1 = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1343));
  (((((TIM_ForcedAction) == ((uint16_t)0x0050)) || ((TIM_ForcedAction) == ((uint16_t)0x0040)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1344));
  tmpccmr1 = TIMx->CCMR1;
   
  tmpccmr1 &= (uint16_t)~((uint16_t)((uint16_t)0x7000));
   
  tmpccmr1 |= (uint16_t)(TIM_ForcedAction << 8);
   
  TIMx->CCMR1 = tmpccmr1;
}









 
void TIM_ForcedOC3Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction)
{
  uint16_t tmpccmr2 = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1367));
  (((((TIM_ForcedAction) == ((uint16_t)0x0050)) || ((TIM_ForcedAction) == ((uint16_t)0x0040)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1368));
  tmpccmr2 = TIMx->CCMR2;
   
  tmpccmr2 &= (uint16_t)~((uint16_t)((uint16_t)0x0070));
   
  tmpccmr2 |= TIM_ForcedAction;
   
  TIMx->CCMR2 = tmpccmr2;
}









 
void TIM_ForcedOC4Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction)
{
  uint16_t tmpccmr2 = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1391));
  (((((TIM_ForcedAction) == ((uint16_t)0x0050)) || ((TIM_ForcedAction) == ((uint16_t)0x0040)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1392));
  tmpccmr2 = TIMx->CCMR2;
   
  tmpccmr2 &= (uint16_t)~((uint16_t)((uint16_t)0x7000));
   
  tmpccmr2 |= (uint16_t)(TIM_ForcedAction << 8);
   
  TIMx->CCMR2 = tmpccmr2;
}







 
void TIM_ARRPreloadConfig(TIM_TypeDef* TIMx, FunctionalState NewState)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1412));
  (((((NewState) == DISABLE) || ((NewState) == ENABLE))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1413));
  if (NewState != DISABLE)
  {
     
    TIMx->CR1 |= ((uint16_t)0x0080);
  }
  else
  {
     
    TIMx->CR1 &= (uint16_t)~((uint16_t)((uint16_t)0x0080));
  }
}







 
void TIM_SelectCOM(TIM_TypeDef* TIMx, FunctionalState NewState)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1436));
  (((((NewState) == DISABLE) || ((NewState) == ENABLE))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1437));
  if (NewState != DISABLE)
  {
     
    TIMx->CR2 |= ((uint16_t)0x0004);
  }
  else
  {
     
    TIMx->CR2 &= (uint16_t)~((uint16_t)((uint16_t)0x0004));
  }
}








 
void TIM_SelectCCDMA(TIM_TypeDef* TIMx, FunctionalState NewState)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1461));
  (((((NewState) == DISABLE) || ((NewState) == ENABLE))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1462));
  if (NewState != DISABLE)
  {
     
    TIMx->CR2 |= ((uint16_t)0x0008);
  }
  else
  {
     
    TIMx->CR2 &= (uint16_t)~((uint16_t)((uint16_t)0x0008));
  }
}








 
void TIM_CCPreloadControl(TIM_TypeDef* TIMx, FunctionalState NewState)
{ 
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1486));
  (((((NewState) == DISABLE) || ((NewState) == ENABLE))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1487));
  if (NewState != DISABLE)
  {
     
    TIMx->CR2 |= ((uint16_t)0x0001);
  }
  else
  {
     
    TIMx->CR2 &= (uint16_t)~((uint16_t)((uint16_t)0x0001));
  }
}









 
void TIM_OC1PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload)
{
  uint16_t tmpccmr1 = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1513));
  (((((TIM_OCPreload) == ((uint16_t)0x0008)) || ((TIM_OCPreload) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1514));
  tmpccmr1 = TIMx->CCMR1;
   
  tmpccmr1 &= (uint16_t)~((uint16_t)((uint16_t)0x0008));
   
  tmpccmr1 |= TIM_OCPreload;
   
  TIMx->CCMR1 = tmpccmr1;
}










 
void TIM_OC2PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload)
{
  uint16_t tmpccmr1 = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1538));
  (((((TIM_OCPreload) == ((uint16_t)0x0008)) || ((TIM_OCPreload) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1539));
  tmpccmr1 = TIMx->CCMR1;
   
  tmpccmr1 &= (uint16_t)~((uint16_t)((uint16_t)0x0800));
   
  tmpccmr1 |= (uint16_t)(TIM_OCPreload << 8);
   
  TIMx->CCMR1 = tmpccmr1;
}









 
void TIM_OC3PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload)
{
  uint16_t tmpccmr2 = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1562));
  (((((TIM_OCPreload) == ((uint16_t)0x0008)) || ((TIM_OCPreload) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1563));
  tmpccmr2 = TIMx->CCMR2;
   
  tmpccmr2 &= (uint16_t)~((uint16_t)((uint16_t)0x0008));
   
  tmpccmr2 |= TIM_OCPreload;
   
  TIMx->CCMR2 = tmpccmr2;
}









 
void TIM_OC4PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload)
{
  uint16_t tmpccmr2 = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1586));
  (((((TIM_OCPreload) == ((uint16_t)0x0008)) || ((TIM_OCPreload) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1587));
  tmpccmr2 = TIMx->CCMR2;
   
  tmpccmr2 &= (uint16_t)~((uint16_t)((uint16_t)0x0800));
   
  tmpccmr2 |= (uint16_t)(TIM_OCPreload << 8);
   
  TIMx->CCMR2 = tmpccmr2;
}









 
void TIM_OC1FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast)
{
  uint16_t tmpccmr1 = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1610));
  (((((TIM_OCFast) == ((uint16_t)0x0004)) || ((TIM_OCFast) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1611));
   
  tmpccmr1 = TIMx->CCMR1;
   
  tmpccmr1 &= (uint16_t)~((uint16_t)((uint16_t)0x0004));
   
  tmpccmr1 |= TIM_OCFast;
   
  TIMx->CCMR1 = tmpccmr1;
}










 
void TIM_OC2FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast)
{
  uint16_t tmpccmr1 = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1636));
  (((((TIM_OCFast) == ((uint16_t)0x0004)) || ((TIM_OCFast) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1637));
   
  tmpccmr1 = TIMx->CCMR1;
   
  tmpccmr1 &= (uint16_t)~((uint16_t)((uint16_t)0x0400));
   
  tmpccmr1 |= (uint16_t)(TIM_OCFast << 8);
   
  TIMx->CCMR1 = tmpccmr1;
}









 
void TIM_OC3FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast)
{
  uint16_t tmpccmr2 = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1661));
  (((((TIM_OCFast) == ((uint16_t)0x0004)) || ((TIM_OCFast) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1662));
   
  tmpccmr2 = TIMx->CCMR2;
   
  tmpccmr2 &= (uint16_t)~((uint16_t)((uint16_t)0x0004));
   
  tmpccmr2 |= TIM_OCFast;
   
  TIMx->CCMR2 = tmpccmr2;
}









 
void TIM_OC4FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast)
{
  uint16_t tmpccmr2 = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1686));
  (((((TIM_OCFast) == ((uint16_t)0x0004)) || ((TIM_OCFast) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1687));
   
  tmpccmr2 = TIMx->CCMR2;
   
  tmpccmr2 &= (uint16_t)~((uint16_t)((uint16_t)0x0400));
   
  tmpccmr2 |= (uint16_t)(TIM_OCFast << 8);
   
  TIMx->CCMR2 = tmpccmr2;
}









 
void TIM_ClearOC1Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear)
{
  uint16_t tmpccmr1 = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1711));
  (((((TIM_OCClear) == ((uint16_t)0x0080)) || ((TIM_OCClear) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1712));

  tmpccmr1 = TIMx->CCMR1;

   
  tmpccmr1 &= (uint16_t)~((uint16_t)((uint16_t)0x0080));
   
  tmpccmr1 |= TIM_OCClear;
   
  TIMx->CCMR1 = tmpccmr1;
}









 
void TIM_ClearOC2Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear)
{
  uint16_t tmpccmr1 = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1737));
  (((((TIM_OCClear) == ((uint16_t)0x0080)) || ((TIM_OCClear) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1738));
  tmpccmr1 = TIMx->CCMR1;
   
  tmpccmr1 &= (uint16_t)~((uint16_t)((uint16_t)0x8000));
   
  tmpccmr1 |= (uint16_t)(TIM_OCClear << 8);
   
  TIMx->CCMR1 = tmpccmr1;
}









 
void TIM_ClearOC3Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear)
{
  uint16_t tmpccmr2 = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1761));
  (((((TIM_OCClear) == ((uint16_t)0x0080)) || ((TIM_OCClear) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1762));
  tmpccmr2 = TIMx->CCMR2;
   
  tmpccmr2 &= (uint16_t)~((uint16_t)((uint16_t)0x0080));
   
  tmpccmr2 |= TIM_OCClear;
   
  TIMx->CCMR2 = tmpccmr2;
}









 
void TIM_ClearOC4Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear)
{
  uint16_t tmpccmr2 = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1785));
  (((((TIM_OCClear) == ((uint16_t)0x0080)) || ((TIM_OCClear) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1786));
  tmpccmr2 = TIMx->CCMR2;
   
  tmpccmr2 &= (uint16_t)~((uint16_t)((uint16_t)0x8000));
   
  tmpccmr2 |= (uint16_t)(TIM_OCClear << 8);
   
  TIMx->CCMR2 = tmpccmr2;
}









 
void TIM_OC1PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity)
{
  uint16_t tmpccer = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1809));
  (((((TIM_OCPolarity) == ((uint16_t)0x0000)) || ((TIM_OCPolarity) == ((uint16_t)0x0002)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1810));
  tmpccer = TIMx->CCER;
   
  tmpccer &= (uint16_t)~((uint16_t)((uint16_t)0x0002));
  tmpccer |= TIM_OCPolarity;
   
  TIMx->CCER = tmpccer;
}









 
void TIM_OC1NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity)
{
  uint16_t tmpccer = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1832));
  (((((TIM_OCNPolarity) == ((uint16_t)0x0000)) || ((TIM_OCNPolarity) == ((uint16_t)0x0008)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1833));
   
  tmpccer = TIMx->CCER;
   
  tmpccer &= (uint16_t)~((uint16_t)((uint16_t)0x0008));
  tmpccer |= TIM_OCNPolarity;
   
  TIMx->CCER = tmpccer;
}









 
void TIM_OC2PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity)
{
  uint16_t tmpccer = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1856));
  (((((TIM_OCPolarity) == ((uint16_t)0x0000)) || ((TIM_OCPolarity) == ((uint16_t)0x0002)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1857));
  tmpccer = TIMx->CCER;
   
  tmpccer &= (uint16_t)~((uint16_t)((uint16_t)0x0020));
  tmpccer |= (uint16_t)(TIM_OCPolarity << 4);
   
  TIMx->CCER = tmpccer;
}









 
void TIM_OC2NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity)
{
  uint16_t tmpccer = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1879));
  (((((TIM_OCNPolarity) == ((uint16_t)0x0000)) || ((TIM_OCNPolarity) == ((uint16_t)0x0008)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1880));
  
  tmpccer = TIMx->CCER;
   
  tmpccer &= (uint16_t)~((uint16_t)((uint16_t)0x0080));
  tmpccer |= (uint16_t)(TIM_OCNPolarity << 4);
   
  TIMx->CCER = tmpccer;
}









 
void TIM_OC3PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity)
{
  uint16_t tmpccer = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1903));
  (((((TIM_OCPolarity) == ((uint16_t)0x0000)) || ((TIM_OCPolarity) == ((uint16_t)0x0002)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1904));
  tmpccer = TIMx->CCER;
   
  tmpccer &= (uint16_t)~((uint16_t)((uint16_t)0x0200));
  tmpccer |= (uint16_t)(TIM_OCPolarity << 8);
   
  TIMx->CCER = tmpccer;
}









 
void TIM_OC3NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity)
{
  uint16_t tmpccer = 0;
 
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1927));
  (((((TIM_OCNPolarity) == ((uint16_t)0x0000)) || ((TIM_OCNPolarity) == ((uint16_t)0x0008)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1928));
    
  tmpccer = TIMx->CCER;
   
  tmpccer &= (uint16_t)~((uint16_t)((uint16_t)0x0800));
  tmpccer |= (uint16_t)(TIM_OCNPolarity << 8);
   
  TIMx->CCER = tmpccer;
}









 
void TIM_OC4PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity)
{
  uint16_t tmpccer = 0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1951));
  (((((TIM_OCPolarity) == ((uint16_t)0x0000)) || ((TIM_OCPolarity) == ((uint16_t)0x0002)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1952));
  tmpccer = TIMx->CCER;
   
  tmpccer &= (uint16_t)~((uint16_t)((uint16_t)0x2000));
  tmpccer |= (uint16_t)(TIM_OCPolarity << 12);
   
  TIMx->CCER = tmpccer;
}













 
void TIM_CCxCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCx)
{
  uint16_t tmp = 0;

   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1979));
  (((((TIM_Channel) == ((uint16_t)0x0000)) || ((TIM_Channel) == ((uint16_t)0x0004)) || ((TIM_Channel) == ((uint16_t)0x0008)) || ((TIM_Channel) == ((uint16_t)0x000C)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1980));
  (((((TIM_CCx) == ((uint16_t)0x0001)) || ((TIM_CCx) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 1981));

  tmp = ((uint16_t)0x0001) << TIM_Channel;

   
  TIMx->CCER &= (uint16_t)~ tmp;

    
  TIMx->CCER |=  (uint16_t)(TIM_CCx << TIM_Channel);
}












 
void TIM_CCxNCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCxN)
{
  uint16_t tmp = 0;

   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2009));
  (((((TIM_Channel) == ((uint16_t)0x0000)) || ((TIM_Channel) == ((uint16_t)0x0004)) || ((TIM_Channel) == ((uint16_t)0x0008)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2010));
  (((((TIM_CCxN) == ((uint16_t)0x0004)) || ((TIM_CCxN) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2011));

  tmp = ((uint16_t)0x0004) << TIM_Channel;

   
  TIMx->CCER &= (uint16_t) ~tmp;

    
  TIMx->CCER |=  (uint16_t)(TIM_CCxN << TIM_Channel);
}























 
void TIM_SelectOCxM(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_OCMode)
{
  uint32_t tmp = 0;
  uint16_t tmp1 = 0;

   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2051));
  (((((TIM_Channel) == ((uint16_t)0x0000)) || ((TIM_Channel) == ((uint16_t)0x0004)) || ((TIM_Channel) == ((uint16_t)0x0008)) || ((TIM_Channel) == ((uint16_t)0x000C)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2052));
  (((((TIM_OCMode) == ((uint16_t)0x0000)) || ((TIM_OCMode) == ((uint16_t)0x0010)) || ((TIM_OCMode) == ((uint16_t)0x0020)) || ((TIM_OCMode) == ((uint16_t)0x0030))|| ((TIM_OCMode) == ((uint16_t)0x0060)) || ((TIM_OCMode) == ((uint16_t)0x0070)) || ((TIM_OCMode) == ((uint16_t)0x0050)) || ((TIM_OCMode) == ((uint16_t)0x0040)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2053));

  tmp = (uint32_t) TIMx;
  tmp += ((uint16_t)0x0018);

  tmp1 = ((uint16_t)0x0001) << (uint16_t)TIM_Channel;

   
  TIMx->CCER &= (uint16_t) ~tmp1;

  if((TIM_Channel == ((uint16_t)0x0000)) ||(TIM_Channel == ((uint16_t)0x0008)))
  {
    tmp += (TIM_Channel>>1);

     
    *(volatile uint32_t *) tmp &= (uint32_t)~((uint32_t)((uint16_t)0x0070));
   
     
    *(volatile uint32_t *) tmp |= TIM_OCMode;
  }
  else
  {
    tmp += (uint16_t)(TIM_Channel - (uint16_t)4)>> (uint16_t)1;

     
    *(volatile uint32_t *) tmp &= (uint32_t)~((uint32_t)((uint16_t)0x7000));
    
     
    *(volatile uint32_t *) tmp |= (uint16_t)(TIM_OCMode << 8);
  }
}







 
void TIM_UpdateDisableConfig(TIM_TypeDef* TIMx, FunctionalState NewState)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2095));
  (((((NewState) == DISABLE) || ((NewState) == ENABLE))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2096));
  if (NewState != DISABLE)
  {
     
    TIMx->CR1 |= ((uint16_t)0x0002);
  }
  else
  {
     
    TIMx->CR1 &= (uint16_t)~((uint16_t)((uint16_t)0x0002));
  }
}











 
void TIM_UpdateRequestConfig(TIM_TypeDef* TIMx, uint16_t TIM_UpdateSource)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2123));
  (((((TIM_UpdateSource) == ((uint16_t)0x0000)) || ((TIM_UpdateSource) == ((uint16_t)0x0001)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2124));
  if (TIM_UpdateSource != ((uint16_t)0x0000))
  {
     
    TIMx->CR1 |= ((uint16_t)0x0004);
  }
  else
  {
     
    TIMx->CR1 &= (uint16_t)~((uint16_t)((uint16_t)0x0004));
  }
}







 
void TIM_SelectHallSensor(TIM_TypeDef* TIMx, FunctionalState NewState)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2147));
  (((((NewState) == DISABLE) || ((NewState) == ENABLE))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2148));
  if (NewState != DISABLE)
  {
     
    TIMx->CR2 |= ((uint16_t)0x0080);
  }
  else
  {
     
    TIMx->CR2 &= (uint16_t)~((uint16_t)((uint16_t)0x0080));
  }
}









 
void TIM_SelectOnePulseMode(TIM_TypeDef* TIMx, uint16_t TIM_OPMode)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2173));
  (((((TIM_OPMode) == ((uint16_t)0x0008)) || ((TIM_OPMode) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2174));
   
  TIMx->CR1 &= (uint16_t)~((uint16_t)((uint16_t)0x0008));
   
  TIMx->CR1 |= TIM_OPMode;
}





















 
void TIM_SelectOutputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_TRGOSource)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2205));
  (((((TIM_TRGOSource) == ((uint16_t)0x0000)) || ((TIM_TRGOSource) == ((uint16_t)0x0010)) || ((TIM_TRGOSource) == ((uint16_t)0x0020)) || ((TIM_TRGOSource) == ((uint16_t)0x0030)) || ((TIM_TRGOSource) == ((uint16_t)0x0040)) || ((TIM_TRGOSource) == ((uint16_t)0x0050)) || ((TIM_TRGOSource) == ((uint16_t)0x0060)) || ((TIM_TRGOSource) == ((uint16_t)0x0070)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2206));
   
  TIMx->CR2 &= (uint16_t)~((uint16_t)((uint16_t)0x0070));
   
  TIMx->CR2 |=  TIM_TRGOSource;
}












 
void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_SlaveMode)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2228));
  (((((TIM_SlaveMode) == ((uint16_t)0x0004)) || ((TIM_SlaveMode) == ((uint16_t)0x0005)) || ((TIM_SlaveMode) == ((uint16_t)0x0006)) || ((TIM_SlaveMode) == ((uint16_t)0x0007)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2229));
  
  TIMx->SMCR &= (uint16_t)~((uint16_t)((uint16_t)0x0007));
   
  TIMx->SMCR |= TIM_SlaveMode;
}










 
void TIM_SelectMasterSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_MasterSlaveMode)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2249));
  (((((TIM_MasterSlaveMode) == ((uint16_t)0x0080)) || ((TIM_MasterSlaveMode) == ((uint16_t)0x0000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2250));
   
  TIMx->SMCR &= (uint16_t)~((uint16_t)((uint16_t)0x0080));
  
   
  TIMx->SMCR |= TIM_MasterSlaveMode;
}






 
void TIM_SetCounter(TIM_TypeDef* TIMx, uint16_t Counter)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2267));
   
  TIMx->CNT = Counter;
}






 
void TIM_SetAutoreload(TIM_TypeDef* TIMx, uint16_t Autoreload)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2281));
   
  TIMx->ARR = Autoreload;
}






 
void TIM_SetCompare1(TIM_TypeDef* TIMx, uint16_t Compare1)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2295));
   
  TIMx->CCR1 = Compare1;
}






 
void TIM_SetCompare2(TIM_TypeDef* TIMx, uint16_t Compare2)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2309));
   
  TIMx->CCR2 = Compare2;
}






 
void TIM_SetCompare3(TIM_TypeDef* TIMx, uint16_t Compare3)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2323));
   
  TIMx->CCR3 = Compare3;
}






 
void TIM_SetCompare4(TIM_TypeDef* TIMx, uint16_t Compare4)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2337));
   
  TIMx->CCR4 = Compare4;
}











 
void TIM_SetIC1Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2356));
  (((((TIM_ICPSC) == ((uint16_t)0x0000)) || ((TIM_ICPSC) == ((uint16_t)0x0004)) || ((TIM_ICPSC) == ((uint16_t)0x0008)) || ((TIM_ICPSC) == ((uint16_t)0x000C)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2357));
   
  TIMx->CCMR1 &= (uint16_t)~((uint16_t)((uint16_t)0x000C));
   
  TIMx->CCMR1 |= TIM_ICPSC;
}











 
void TIM_SetIC2Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2378));
  (((((TIM_ICPSC) == ((uint16_t)0x0000)) || ((TIM_ICPSC) == ((uint16_t)0x0004)) || ((TIM_ICPSC) == ((uint16_t)0x0008)) || ((TIM_ICPSC) == ((uint16_t)0x000C)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2379));
   
  TIMx->CCMR1 &= (uint16_t)~((uint16_t)((uint16_t)0x0C00));
   
  TIMx->CCMR1 |= (uint16_t)(TIM_ICPSC << 8);
}











 
void TIM_SetIC3Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2400));
  (((((TIM_ICPSC) == ((uint16_t)0x0000)) || ((TIM_ICPSC) == ((uint16_t)0x0004)) || ((TIM_ICPSC) == ((uint16_t)0x0008)) || ((TIM_ICPSC) == ((uint16_t)0x000C)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2401));
   
  TIMx->CCMR2 &= (uint16_t)~((uint16_t)((uint16_t)0x000C));
   
  TIMx->CCMR2 |= TIM_ICPSC;
}











 
void TIM_SetIC4Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC)
{  
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2422));
  (((((TIM_ICPSC) == ((uint16_t)0x0000)) || ((TIM_ICPSC) == ((uint16_t)0x0004)) || ((TIM_ICPSC) == ((uint16_t)0x0008)) || ((TIM_ICPSC) == ((uint16_t)0x000C)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2423));
   
  TIMx->CCMR2 &= (uint16_t)~((uint16_t)((uint16_t)0x0C00));
   
  TIMx->CCMR2 |= (uint16_t)(TIM_ICPSC << 8);
}











 
void TIM_SetClockDivision(TIM_TypeDef* TIMx, uint16_t TIM_CKD)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2444));
  (((((TIM_CKD) == ((uint16_t)0x0000)) || ((TIM_CKD) == ((uint16_t)0x0100)) || ((TIM_CKD) == ((uint16_t)0x0200)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2445));
   
  TIMx->CR1 &= (uint16_t)~((uint16_t)((uint16_t)0x0300));
   
  TIMx->CR1 |= TIM_CKD;
}





 
uint16_t TIM_GetCapture1(TIM_TypeDef* TIMx)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2460));
   
  return TIMx->CCR1;
}





 
uint16_t TIM_GetCapture2(TIM_TypeDef* TIMx)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2473));
   
  return TIMx->CCR2;
}





 
uint16_t TIM_GetCapture3(TIM_TypeDef* TIMx)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2486)); 
   
  return TIMx->CCR3;
}





 
uint16_t TIM_GetCapture4(TIM_TypeDef* TIMx)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2499));
   
  return TIMx->CCR4;
}





 
uint16_t TIM_GetCounter(TIM_TypeDef* TIMx)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2512));
   
  return TIMx->CNT;
}





 
uint16_t TIM_GetPrescaler(TIM_TypeDef* TIMx)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2525));
   
  return TIMx->PSC;
}


























 
FlagStatus TIM_GetFlagStatus(TIM_TypeDef* TIMx, uint16_t TIM_FLAG)
{ 
  ITStatus bitstatus = RESET;  
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2560));
  (((((TIM_FLAG) == ((uint16_t)0x0001)) || ((TIM_FLAG) == ((uint16_t)0x0002)) || ((TIM_FLAG) == ((uint16_t)0x0004)) || ((TIM_FLAG) == ((uint16_t)0x0008)) || ((TIM_FLAG) == ((uint16_t)0x0010)) || ((TIM_FLAG) == ((uint16_t)0x0020)) || ((TIM_FLAG) == ((uint16_t)0x0040)) || ((TIM_FLAG) == ((uint16_t)0x0080)) || ((TIM_FLAG) == ((uint16_t)0x0200)) || ((TIM_FLAG) == ((uint16_t)0x0400)) || ((TIM_FLAG) == ((uint16_t)0x0800)) || ((TIM_FLAG) == ((uint16_t)0x1000)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2561));
  
  if ((TIMx->SR & TIM_FLAG) != (uint16_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}


























 
void TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t TIM_FLAG)
{  
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2603));
  ((((((TIM_FLAG) & (uint16_t)0xE100) == 0x0000) && ((TIM_FLAG) != 0x0000))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2604));
   
   
  TIMx->SR = (uint16_t)~TIM_FLAG;
}






















 
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT)
{
  ITStatus bitstatus = RESET;  
  uint16_t itstatus = 0x0, itenable = 0x0;
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2637));
  (((((TIM_IT) == ((uint16_t)0x0001)) || ((TIM_IT) == ((uint16_t)0x0002)) || ((TIM_IT) == ((uint16_t)0x0004)) || ((TIM_IT) == ((uint16_t)0x0008)) || ((TIM_IT) == ((uint16_t)0x0010)) || ((TIM_IT) == ((uint16_t)0x0020)) || ((TIM_IT) == ((uint16_t)0x0040)) || ((TIM_IT) == ((uint16_t)0x0080)))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2638));
   
  itstatus = TIMx->SR & TIM_IT;
  
  itenable = TIMx->DIER & TIM_IT;
  if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}






















 
void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT)
{
   
  (((((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1000))) || ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4C00))) || ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x5400)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1800)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x1C00)))|| ((TIMx) == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x2000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4000)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4400)))|| ((TIMx) == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x4800))))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2679));
  ((((((TIM_IT) & (uint16_t)0xFF00) == 0x0000) && ((TIM_IT) != 0x0000))) ? (void)0 : assert_failed((uint8_t *)"..\\STM32F10x_StdPeriph_Driver\\src\\stm32f10x_tim.c", 2680));
   
  TIMx->SR = (uint16_t)~TIM_IT;
}
















 
static void TI1_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
                       uint16_t TIM_ICFilter)
{
  uint16_t tmpccmr1 = 0, tmpccer = 0;
   
  TIMx->CCER &= (uint16_t)~((uint16_t)((uint16_t)0x0001));
  tmpccmr1 = TIMx->CCMR1;
  tmpccer = TIMx->CCER;
   
  tmpccmr1 &= (uint16_t)(((uint16_t)~((uint16_t)((uint16_t)0x0003))) & ((uint16_t)~((uint16_t)((uint16_t)0x00F0))));
  tmpccmr1 |= (uint16_t)(TIM_ICSelection | (uint16_t)(TIM_ICFilter << (uint16_t)4));
  
  if((TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) ||
     (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) ||(TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))))
  {
     
    tmpccer &= (uint16_t)~((uint16_t)(((uint16_t)0x0002)));
    tmpccer |= (uint16_t)(TIM_ICPolarity | (uint16_t)((uint16_t)0x0001));
  }
  else
  {
     
    tmpccer &= (uint16_t)~((uint16_t)(((uint16_t)0x0002) | ((uint16_t)0x0008)));
    tmpccer |= (uint16_t)(TIM_ICPolarity | (uint16_t)((uint16_t)0x0001));
  }

   
  TIMx->CCMR1 = tmpccmr1;
  TIMx->CCER = tmpccer;
}
















 
static void TI2_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
                       uint16_t TIM_ICFilter)
{
  uint16_t tmpccmr1 = 0, tmpccer = 0, tmp = 0;
   
  TIMx->CCER &= (uint16_t)~((uint16_t)((uint16_t)0x0010));
  tmpccmr1 = TIMx->CCMR1;
  tmpccer = TIMx->CCER;
  tmp = (uint16_t)(TIM_ICPolarity << 4);
   
  tmpccmr1 &= (uint16_t)(((uint16_t)~((uint16_t)((uint16_t)0x0300))) & ((uint16_t)~((uint16_t)((uint16_t)0xF000))));
  tmpccmr1 |= (uint16_t)(TIM_ICFilter << 12);
  tmpccmr1 |= (uint16_t)(TIM_ICSelection << 8);
  
  if((TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) ||
     (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) ||(TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))))
  {
     
    tmpccer &= (uint16_t)~((uint16_t)(((uint16_t)0x0020)));
    tmpccer |=  (uint16_t)(tmp | (uint16_t)((uint16_t)0x0010));
  }
  else
  {
     
    tmpccer &= (uint16_t)~((uint16_t)(((uint16_t)0x0020) | ((uint16_t)0x0080)));
    tmpccer |= (uint16_t)(TIM_ICPolarity | (uint16_t)((uint16_t)0x0010));
  }
  
   
  TIMx->CCMR1 = tmpccmr1 ;
  TIMx->CCER = tmpccer;
}
















 
static void TI3_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
                       uint16_t TIM_ICFilter)
{
  uint16_t tmpccmr2 = 0, tmpccer = 0, tmp = 0;
   
  TIMx->CCER &= (uint16_t)~((uint16_t)((uint16_t)0x0100));
  tmpccmr2 = TIMx->CCMR2;
  tmpccer = TIMx->CCER;
  tmp = (uint16_t)(TIM_ICPolarity << 8);
   
  tmpccmr2 &= (uint16_t)(((uint16_t)~((uint16_t)((uint16_t)0x0003))) & ((uint16_t)~((uint16_t)((uint16_t)0x00F0))));
  tmpccmr2 |= (uint16_t)(TIM_ICSelection | (uint16_t)(TIM_ICFilter << (uint16_t)4));
    
  if((TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) ||
     (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) ||(TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))))
  {
     
    tmpccer &= (uint16_t)~((uint16_t)(((uint16_t)0x0200)));
    tmpccer |= (uint16_t)(tmp | (uint16_t)((uint16_t)0x0100));
  }
  else
  {
     
    tmpccer &= (uint16_t)~((uint16_t)(((uint16_t)0x0200) | ((uint16_t)0x0800)));
    tmpccer |= (uint16_t)(TIM_ICPolarity | (uint16_t)((uint16_t)0x0100));
  }
  
   
  TIMx->CCMR2 = tmpccmr2;
  TIMx->CCER = tmpccer;
}
















 
static void TI4_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
                       uint16_t TIM_ICFilter)
{
  uint16_t tmpccmr2 = 0, tmpccer = 0, tmp = 0;

    
  TIMx->CCER &= (uint16_t)~((uint16_t)((uint16_t)0x1000));
  tmpccmr2 = TIMx->CCMR2;
  tmpccer = TIMx->CCER;
  tmp = (uint16_t)(TIM_ICPolarity << 12);
   
  tmpccmr2 &= (uint16_t)((uint16_t)(~(uint16_t)((uint16_t)0x0300)) & ((uint16_t)~((uint16_t)((uint16_t)0xF000))));
  tmpccmr2 |= (uint16_t)(TIM_ICSelection << 8);
  tmpccmr2 |= (uint16_t)(TIM_ICFilter << 12);
  
  if((TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))) || (TIMx == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3400))) || (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000))) || (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400))) ||
     (TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0800))) ||(TIMx == ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0C00))))
  {
     
    tmpccer &= (uint16_t)~((uint16_t)(((uint16_t)0x2000)));
    tmpccer |= (uint16_t)(tmp | (uint16_t)((uint16_t)0x1000));
  }
  else
  {
     
    tmpccer &= (uint16_t)~((uint16_t)(((uint16_t)0x0200) | ((uint16_t)0x8000)));
    tmpccer |= (uint16_t)(TIM_ICPolarity | (uint16_t)((uint16_t)0x1000));
  }
   
  TIMx->CCMR2 = tmpccmr2;
  TIMx->CCER = tmpccer;
}



 



 



 

 

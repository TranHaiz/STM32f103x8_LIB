#ifndef DEFAULT_H_
#define DEFAULT_H_



//--------------------------------------------
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
//--------------------------------------------
#define ENABLE 1
#define DISABLE 0
#define HIGH 1
#define LOW 0
#define NULL 0
//--------------------------------------------
extern uint32_t SystemCoreClock; //define SystemcoreClock

//CORETEX M
#define SCS_BASE            	(0xE000E000UL)
//--------------------------------------------
#define FLASH_BASE            ((uint32_t)0x08000000) /*!< FLASH base address in the alias region */
#define SRAM_BASE             ((uint32_t)0x20000000) /*!< SRAM base address in the alias region */
#define PERIPH_BASE           ((uint32_t)0x40000000) /*!< Peripheral base address in the alias region */

#define SRAM_BB_BASE          ((uint32_t)0x22000000) /*!< SRAM base address in the bit-band region */
#define PERIPH_BB_BASE        ((uint32_t)0x42000000) /*!< Peripheral base address in the bit-band region */

#define FSMC_R_BASE           ((uint32_t)0xA0000000) /*!< FSMC registers base address */

/*!< Peripheral memory map */
#define RCC_BASE 0x40021000UL
#define RCC_CFGR        (*(volatile unsigned int *)(RCC_BASE + 0x04))
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x20000)

#define GPIOA_BASE 0x40010800UL
#define GPIOB_BASE 0x40010C00UL
#define GPIOC_BASE 0x40011000UL
#define GPIOD_BASE            (APB2PERIPH_BASE + 0x1400)
#define AFIO_BASE             (APB2PERIPH_BASE + 0x0000)

#define NVIC_BASE           	(SCS_BASE +  0x0100UL)
//--------------------------------------------
typedef struct
{
  volatile uint32_t ISER[8U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
        uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
        uint32_t RESERVED1[24U];
  volatile uint32_t ISPR[8U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
        uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
        uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];               /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
        uint32_t RESERVED4[56U];
  volatile uint8_t  IP[240U];               /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
        uint32_t RESERVED5[644U];
  volatile  uint32_t STIR;                   /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
}  NVIC_Type;
typedef struct {
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

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t APB1RSTR;
    volatile uint32_t AHBENR;
    volatile uint32_t APB2ENR;
		volatile uint32_t APB1ENR;
		volatile uint32_t BDCR;
} RCC_TypeDef;

//--------------------------------------------
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)  *((volatile unsigned long*)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //ngo ra
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //ngo vao

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)   

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //ngo ra
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //ngo vao

#define RCC_GPIOA_EN (RCC->APB2ENR |= (1<<2))
#define RCC_GPIOB_EN (RCC->APB2ENR |= (1<<3))
#define RCC_GPIOC_EN (RCC->APB2ENR |= (1<<4))
#define RCC_GPIOD_EN (RCC->APB2ENR |= (1<<5))
#define RCC_GPIOE_EN (RCC->APB2ENR |= (1<<6))
#define RCC_GPIOF_EN (RCC->APB2ENR |= (1<<7))
#define RCC_GPIOG_EN (RCC->APB2ENR |= (1<<8))
//------------------------------------------------------
//pull
#define NO_PULL 0
#define PULL_UP 1
#define PULL_DOWN 2
//speed
#define LOW_SPEED 2
#define MEDIUM_SPEED 1
#define HIGH_SPEED 3
//MODE
#define MODE_OUTPUT_PP 0
#define MODE_OUTPUT_OD 1
#define MODE_OUTPUT_AF_PP 2
#define MODE_OUTPUT_AF_OD 3

#define MODE_INPUT_FLOAT 4
#define MODE_INPUT_ANALOG 5
#define MODE_INPUT_PP 8
#define MODE_INPUT MODE_INPUT_PP
#define MODE_INPUT_AF MODE_INPUT
//------------------------------------------------------
typedef enum IRQn
{
  /* Cortex-M3 Processor Exceptions Numbers */
  NonMaskableInt_IRQn         = -14, /* Non Maskable Interrupt */
  MemoryManagement_IRQn       = -12, /* Memory Management Interrupt */
  BusFault_IRQn               = -11, /* Bus Fault Interrupt */
  UsageFault_IRQn             = -10, /* Usage Fault Interrupt */
  SVCall_IRQn                 = -5,  /* System Service Call via SVC instruction */
  DebugMonitor_IRQn           = -4,  /* Debug Monitor Interrupt */
  PendSV_IRQn                 = -2,  /* Pendable request for system service */
  SysTick_IRQn                = -1,  /* System Tick Timer */

  /* STM32F103C8T6 Specific Interrupt Numbers */
  WWDG_IRQn                   = 0,   /* Window WatchDog Interrupt */
  PVD_IRQn                    = 1,   /* PVD through EXTI Line Detection Interrupt */
  TAMPER_IRQn                 = 2,   /* Tamper Interrupt */
  RTC_IRQn                    = 3,   /* RTC global Interrupt */
  FLASH_IRQn                  = 4,   /* FLASH global Interrupt */
  RCC_IRQn                    = 5,   /* RCC global Interrupt */
  EXTI0_IRQn                  = 6,   /* EXTI Line0 Interrupt */
  EXTI1_IRQn                  = 7,   /* EXTI Line1 Interrupt */
  EXTI2_IRQn                  = 8,   /* EXTI Line2 Interrupt */
  EXTI3_IRQn                  = 9,   /* EXTI Line3 Interrupt */
  EXTI4_IRQn                  = 10,  /* EXTI Line4 Interrupt */
  DMA1_Channel1_IRQn          = 11,  /* DMA1 Channel1 global Interrupt */
  DMA1_Channel2_IRQn          = 12,  /* DMA1 Channel2 global Interrupt */
  DMA1_Channel3_IRQn          = 13,  /* DMA1 Channel3 global Interrupt */
  DMA1_Channel4_IRQn          = 14,  /* DMA1 Channel4 global Interrupt */
  DMA1_Channel5_IRQn          = 15,  /* DMA1 Channel5 global Interrupt */
  DMA1_Channel6_IRQn          = 16,  /* DMA1 Channel6 global Interrupt */
  DMA1_Channel7_IRQn          = 17,  /* DMA1 Channel7 global Interrupt */
  ADC1_2_IRQn                 = 18,  /* ADC1 and ADC2 global Interrupt */
  USB_HP_CAN1_TX_IRQn         = 19,  /* USB High Priority or CAN1 TX Interrupt */
  USB_LP_CAN1_RX0_IRQn        = 20,  /* USB Low Priority or CAN1 RX0 Interrupt */
  CAN1_RX1_IRQn               = 21,  /* CAN1 RX1 Interrupt */
  CAN1_SCE_IRQn               = 22,  /* CAN1 SCE Interrupt */
  EXTI9_5_IRQn                = 23,  /* EXTI Line[9:5] Interrupts */
  TIM1_BRK_IRQn               = 24,  /* TIM1 Break Interrupt */
  TIM1_UP_IRQn                = 25,  /* TIM1 Update Interrupt */
  TIM1_TRG_COM_IRQn           = 26,  /* TIM1 Trigger and Commutation Interrupt */
  TIM1_CC_IRQn                = 27,  /* TIM1 Capture Compare Interrupt */
  TIM2_IRQn                   = 28,  /* TIM2 global Interrupt */
  TIM3_IRQn                   = 29,  /* TIM3 global Interrupt */
  TIM4_IRQn                   = 30,  /* TIM4 global Interrupt */
  I2C1_EV_IRQn                = 31,  /* I2C1 Event Interrupt */
  I2C1_ER_IRQn                = 32,  /* I2C1 Error Interrupt */
  I2C2_EV_IRQn                = 33,  /* I2C2 Event Interrupt */
  I2C2_ER_IRQn                = 34,  /* I2C2 Error Interrupt */
  SPI1_IRQn                   = 35,  /* SPI1 global Interrupt */
  SPI2_IRQn                   = 36,  /* SPI2 global Interrupt */
  USART1_IRQn                 = 37,  /* USART1 global Interrupt */
  USART2_IRQn                 = 38,  /* USART2 global Interrupt */
  USART3_IRQn                 = 39,  /* USART3 global Interrupt */
  EXTI15_10_IRQn              = 40,  /* EXTI Line[15:10] Interrupts */
  RTCAlarm_IRQn               = 41,  /* RTC Alarm through EXTI Line Interrupt */
  USBWakeUp_IRQn              = 42,  /* USB Wakeup from suspend through EXTI Line Interrupt */
  TIM8_BRK_IRQn               = 43,  /* TIM8 Break Interrupt */
  TIM8_UP_IRQn                = 44,  /* TIM8 Update Interrupt */
  TIM8_TRG_COM_IRQn           = 45,  /* TIM8 Trigger and Commutation Interrupt */
  TIM8_CC_IRQn                = 46,  /* TIM8 Capture Compare Interrupt */
  ADC3_IRQn                   = 47,  /* ADC3 global Interrupt */
  FSMC_IRQn                   = 48,  /* FSMC global Interrupt */
  SDIO_IRQn                   = 49,  /* SDIO global Interrupt */
  TIM5_IRQn                   = 50,  /* TIM5 global Interrupt */
  SPI3_IRQn                   = 51,  /* SPI3 global Interrupt */
  UART4_IRQn                  = 52,  /* UART4 global Interrupt */
  UART5_IRQn                  = 53,  /* UART5 global Interrupt */
  TIM6_IRQn                   = 54,  /* TIM6 global Interrupt */
  TIM7_IRQn                   = 55,  /* TIM7 global Interrupt */
  DMA2_Channel1_IRQn          = 56,  /* DMA2 Channel1 global Interrupt */
  DMA2_Channel2_IRQn          = 57,  /* DMA2 Channel2 global Interrupt */
  DMA2_Channel3_IRQn          = 58,  /* DMA2 Channel3 global Interrupt */
  DMA2_Channel4_5_IRQn        = 59   /* DMA2 Channel4 and DMA2 Channel5 global Interrupt */
} IRQn_Type;

//-----------------------------------------------------------------
#define GPIOA 							((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB 							((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC 							((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define RCC 								((RCC_TypeDef *) RCC_BASE)
#define RCC_APB1ENR 				( RCC_BASE+ 0x1C )
#define RCC_APB2ENR 				(RCC_BASE+ 0x18 )
//#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
//#define ADC2                ((ADC_TypeDef *) ADC2_BASE)
//#define USART1              ((USART_TypeDef *) USART1_BASE)
//#define USART2              ((USART_TypeDef *) USART2_BASE)
//#define USART3              ((USART_TypeDef *) USART3_BASE)
//#define UART4               ((USART_TypeDef *) UART4_BASE)
//#define UART5               ((USART_TypeDef *) UART5_BASE)
#define AFIO                ((AFIO_TypeDef *) AFIO_BASE)
#define NVIC                ((NVIC_Type*)NVIC_BASE)
//-----------------------------------------------------------------
//GPIO
void GPIOx_INIT(GPIO_TypeDef* GPIOx,uint8_t pin, uint8_t mode, uint8_t pull, uint8_t speed);
void GPIOx_WRITE(GPIO_TypeDef* GPIOx,uint8_t pin,uint8_t state);
uint8_t GPIOx_READ(GPIO_TypeDef* GPIOx,uint8_t pin);
void GPIOx_TOGGLE(GPIO_TypeDef* GPIOx,uint8_t pin);

//NVIC
void NVICx_enable(IRQn_Type IRQx);
#endif

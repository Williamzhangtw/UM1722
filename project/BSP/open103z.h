/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OPEN103Z_H
#define __OPEN103Z_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/**
  * @brief STM3210E EVAL BSP Driver version number
    */
#define __STM3210E_EVAL_BSP_VERSION_MAIN       (0x06) /*!< [31:24] main version */
#define __STM3210E_EVAL_BSP_VERSION_SUB1       (0x00) /*!< [23:16] sub1 version */
#define __STM3210E_EVAL_BSP_VERSION_SUB2       (0x02) /*!< [15:8]  sub2 version */
#define __STM3210E_EVAL_BSP_VERSION_RC         (0x00) /*!< [7:0]  release candidate */
#define __STM3210E_EVAL_BSP_VERSION            ((__STM3210E_EVAL_BSP_VERSION_MAIN << 24)\
                                               |(__STM3210E_EVAL_BSP_VERSION_SUB1 << 16)\
                                               |(__STM3210E_EVAL_BSP_VERSION_SUB2 << 8 )\
                                               |(__STM3210E_EVAL_BSP_VERSION_RC))   
   
   
/**
 * @brief LED Types Definition
 */
typedef enum 
{
  LED1 = 0,
  LED2 = 1,
  LED3 = 2,
  LED4 = 3,

  LED_GREEN  = LED1,
  LED_ORANGE = LED2,
  LED_RED    = LED3,
  LED_BLUE   = LED4

} Led_TypeDef;

/**
 * @brief BUTTON Types Definition
 */
typedef enum 
{
  BUTTON_USER    = 0,
  BUTTON_SEL    = 1,
  BUTTON_LEFT   = 2,
  BUTTON_RIGHT  = 3,
  BUTTON_DOWN   = 4,
  BUTTON_UP     = 5,

} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1,
  BUTTON_MODE_EVT  = 2

} ButtonMode_TypeDef;

/**
 * @brief JOYSTICK Types Definition
 */
typedef enum 
{ 
  JOY_SEL   = 0,
  JOY_LEFT  = 1,
  JOY_RIGHT = 2,
  JOY_DOWN  = 3,
  JOY_UP    = 4,
  JOY_NONE  = 5

}JOYState_TypeDef;

typedef enum 
{ 
  JOY_MODE_GPIO = 0,
  JOY_MODE_EXTI = 1

}JOYMode_TypeDef;

/**
 * @brief COM Types Definition
 */
typedef enum 
{
  COM1 = 0,
  COM2 = 1

} COM_TypeDef;  




/** 
  * @brief  Define for STM3210E_EVAL board  
  */ 
#if !defined (USE_OPEN103Z)
 #define USE_OPEN103Z
#endif
 

  
/** @addtogroup STM3210E_EVAL_LED
  * @{
  */
#define LEDn                             4

//#define LED1_PIN                         GPIO_PIN_6             /* PF.06*/
//#define LED1_GPIO_PORT                   GPIOF
#define LED1_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOF_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOF_CLK_DISABLE()
 
//#define LED2_PIN                         GPIO_PIN_7             /* PF.07*/
//#define LED2_GPIO_PORT                   GPIOF
#define LED2_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOF_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOF_CLK_DISABLE()


//#define LED3_PIN                         GPIO_PIN_8            /* PF.08*/
//#define LED3_GPIO_PORT                   GPIOF
#define LED3_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOF_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOF_CLK_DISABLE()


//#define LED4_PIN                         GPIO_PIN_9            /* PF.09*/
//#define LED4_GPIO_PORT                   GPIOF
#define LED4_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOF_CLK_ENABLE()
#define LED4_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOF_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__LED__) do { if ((__LED__) == LED1) LED1_GPIO_CLK_ENABLE(); else\
                                           if ((__LED__) == LED2) LED2_GPIO_CLK_ENABLE(); else \
                                           if ((__LED__) == LED3) LED3_GPIO_CLK_ENABLE(); else\
                                           if ((__LED__) == LED4) LED4_GPIO_CLK_ENABLE();} while(0)

#define LEDx_GPIO_CLK_DISABLE(__LED__)   (((__LED__) == LED1) ? LED1_GPIO_CLK_DISABLE() :\
                                          ((__LED__) == LED2) ? LED2_GPIO_CLK_DISABLE() :\
                                          ((__LED__) == LED3) ? LED3_GPIO_CLK_DISABLE() :\
                                          ((__LED__) == LED4) ? LED4_GPIO_CLK_DISABLE() : 0 )

/**
  * @}
  */
  
/** @addtogroup STM3210E_EVAL_BUTTON
  * @{
  */  
#define JOYn                             5
#define BUTTONn                          1 + JOYn


/**
 * @brief USER_KEY -button
 */
//#define USER_BUTTON_PIN                      GPIO_PIN_6             /* PG.08*/
//#define USER_BUTTON_GPIO_PORT                     GPIOG
#define USER_BUTTON_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOG_CLK_ENABLE()
#define USER_BUTTON_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOG_CLK_DISABLE()
#define USER_BUTTON_EXTI_IRQn                EXTI9_5_IRQn


/**
 * @brief Joystick Right push-button
 */
//#define RIGHT_JOY_PIN                       GPIO_PIN_0             /* PC.0*/
//#define RIGHT_JOY_GPIO_PORT                 GPIOC
#define RIGHT_JOY_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()
#define RIGHT_JOY_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOC_CLK_DISABLE()
#define RIGHT_JOY_EXTI_IRQn                 EXTI0_IRQn

/**
 * @brief Joystick Left push-button
 */    
//#define LEFT_JOY_PIN                        GPIO_PIN_3             /* PC.3*/
//#define LEFT_JOY_GPIO_PORT                  GPIOC
#define LEFT_JOY_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOC_CLK_ENABLE()
#define LEFT_JOY_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOC_CLK_DISABLE()
#define LEFT_JOY_EXTI_IRQn                  EXTI3_IRQn

/**
 * @brief Joystick Up push-button
 */
//#define UP_JOY_PIN                          GPIO_PIN_1             /* PC.1*/
//#define UP_JOY_GPIO_PORT                    GPIOC
#define UP_JOY_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define UP_JOY_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOC_CLK_DISABLE()
#define UP_JOY_EXTI_IRQn                    EXTI1_IRQn

/**
 * @brief Joystick Down push-button
 */   
//#define DOWN_JOY_PIN                        GPIO_PIN_2             /* Pc.02*/
//#define DOWN_JOY_GPIO_PORT                  GPIOC
#define DOWN_JOY_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOC_CLK_ENABLE()
#define DOWN_JOY_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOC_CLK_DISABLE()
#define DOWN_JOY_EXTI_IRQn                  EXTI2_IRQn

/**
 * @brief Joystick Sel push-button
 */  
//#define SEL_JOY_PIN                         GPIO_PIN_8             /* PG.08*/
//#define SEL_JOY_GPIO_PORT                   GPIOG
#define SEL_JOY_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOG_CLK_ENABLE()
#define SEL_JOY_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOG_CLK_DISABLE()
#define SEL_JOY_EXTI_IRQn                   EXTI9_5_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__BUTTON__) do { if ((__BUTTON__) == BUTTON_USER) USER_BUTTON_GPIO_CLK_ENABLE(); else\
                                                 if ((__BUTTON__) == BUTTON_SEL) SEL_JOY_GPIO_CLK_ENABLE(); else\
                                                 if ((__BUTTON__) == BUTTON_LEFT) LEFT_JOY_GPIO_CLK_ENABLE(); else\
                                                 if ((__BUTTON__) == BUTTON_RIGHT) RIGHT_JOY_GPIO_CLK_ENABLE(); else\
                                                 if ((__BUTTON__) == BUTTON_DOWN) DOWN_JOY_GPIO_CLK_ENABLE(); else\
                                                 if ((__BUTTON__) == BUTTON_UP) UP_JOY_GPIO_CLK_ENABLE();} while(0)

#define BUTTONx_GPIO_CLK_DISABLE(__BUTTON__)    (((__BUTTON__) == BUTTON_USER) ? USER_BUTTON_GPIO_CLK_DISABLE()  :\
                                                 ((__BUTTON__) == BUTTON_SEL) ? SEL_JOY_GPIO_CLK_DISABLE()  :\
                                                 ((__BUTTON__) == BUTTON_LEFT) ? LEFT_JOY_GPIO_CLK_DISABLE()  :\
                                                 ((__BUTTON__) == BUTTON_RIGHT) ? RIGHT_JOY_GPIO_CLK_DISABLE()  :\
                                                 ((__BUTTON__) == BUTTON_DOWN) ? DOWN_JOY_GPIO_CLK_DISABLE()  :\
                                                 ((__BUTTON__) == BUTTON_UP) ? UP_JOY_GPIO_CLK_DISABLE() : 0 )

#define JOYx_GPIO_CLK_ENABLE(__JOY__)    do { if ((__JOY__) == JOY_SEL) SEL_JOY_GPIO_CLK_ENABLE(); else\
                                              if ((__JOY__) == JOY_LEFT) LEFT_JOY_GPIO_CLK_ENABLE(); else\
                                              if ((__JOY__) == JOY_RIGHT) RIGHT_JOY_GPIO_CLK_ENABLE(); else\
                                              if ((__JOY__) == JOY_DOWN) DOWN_JOY_GPIO_CLK_ENABLE(); else\
                                              if ((__JOY__) == JOY_UP) UP_JOY_GPIO_CLK_ENABLE();} while(0)

#define JOYx_GPIO_CLK_DISABLE(__JOY__)   (((__JOY__) == JOY_SEL) ? SEL_JOY_GPIO_CLK_DISABLE() :\
                                          ((__JOY__) == JOY_LEFT) ? LEFT_JOY_GPIO_CLK_DISABLE() :\
                                          ((__JOY__) == JOY_RIGHT) ? RIGHT_JOY_GPIO_CLK_DISABLE() :\
                                          ((__JOY__) == JOY_DOWN) ? DOWN_JOY_GPIO_CLK_DISABLE() :\
                                          ((__JOY__) == JOY_UP) ? UP_JOY_GPIO_CLK_DISABLE() : 0 )


/** @addtogroup STM3210E_EVAL_Exported_Functions
  * @{
  */ 
uint32_t                BSP_GetVersion(void);
void                    BSP_LED_Init(Led_TypeDef Led);
void                    BSP_LED_On(Led_TypeDef Led);
void                    BSP_LED_Off(Led_TypeDef Led);
void                    BSP_LED_Toggle(Led_TypeDef Led);
void                    BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode);
uint32_t                BSP_PB_GetState(Button_TypeDef Button);
#ifdef HAL_UART_MODULE_ENABLED
void                    BSP_COM_Init(COM_TypeDef COM, UART_HandleTypeDef* huart);
#endif /* HAL_UART_MODULE_ENABLED */
uint8_t                 BSP_JOY_Init(JOYMode_TypeDef Joy_Mode);
JOYState_TypeDef        BSP_JOY_GetState(void);

/**
  * @}
  */


/**
  * @}
  */
  
/**
  * @}
  */
  
/**
  * @}
  */
  
#ifdef __cplusplus
}
#endif
  


#endif

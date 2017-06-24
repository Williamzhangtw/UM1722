/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "adc.h"
#include "solder.h"
#include "usart.h"
#include "stdio.h"
#include "open103z.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId led1TaskHandle;
osThreadId led2TaskHandle;
osThreadId led3TaskHandle;
osThreadId led4TaskHandle;
osThreadId printfTaskHandle;
osThreadId sensor1TaskHandle;
osMutexId mutex01Handle;
osSemaphoreId keySemHandle;

/* USER CODE BEGIN Variables */


// used to detect and latch errors */
__IO uint32_t HighPriorityThreadCycles = 0, MediumPriorityThreadCycles = 0, LowPriorityThreadCycles = 0;

char  cPrint[1024];
volatile uint16_t uiADC[10];
char  cGetChar[1024];

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void vLed1Task(void const * argument);
void vLed2Task(void const * argument);
void vLed3Task(void const * argument);
void vLed4Task(void const * argument);
void vPrintfTask(void const * argument);
void vSensor1Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define VDD_APPLI                      ((uint32_t) 3300)   /* Value of analog voltage supply Vdda (unit: mV) */
#define RANGE_12BITS                   ((uint32_t) 4095)   /* Max value with a full range of 12 bits */
#define USERBUTTON_CLICK_COUNT_MAX     ((uint32_t)    4)   /* Maximum value of variable "UserButtonClickCount" */

#define ADCCONVERTEDVALUES_BUFFER_SIZE ((uint32_t)    3)   /* Size of array containing ADC converted values: set to ADC sequencer number of ranks converted, to have a rank in each address */

/* Internal temperature sensor: constants data used for indicative values in  */
/* this example. Refer to device datasheet for min/typ/max values.            */
/* For more accurate values, device should be calibrated on offset and slope  */
/* for application temperature range.                                         */
#define INTERNAL_TEMPSENSOR_V25        ((int32_t)1430)         /* Internal temperature sensor, parameter V25 (unit: mV). Refer to device datasheet for min/typ/max values. */
#define INTERNAL_TEMPSENSOR_AVGSLOPE   ((int32_t)4300)         /* Internal temperature sensor, parameter Avg_Slope (unit: uV/DegCelsius). Refer to device datasheet for min/typ/max values. */                                                               /* This calibration parameter is intended to calculate the actual VDDA from Vrefint ADC measurement. */

#define mutexSHORT_DELAY     ((uint32_t) 20)
#define mutexNO_DELAY        ((uint32_t) 0)
#define mutexTWO_TICK_DELAY  ((uint32_t) 2)


/* Private macro -------------------------------------------------------------*/

/**
  * @brief  Computation of temperature (unit: degree Celsius) from the internal
  *         temperature sensor measurement by ADC.
  *         Computation is using temperature sensor standard parameters (refer
  *         to device datasheet).
  *         Computation formula:
  *         Temperature = (VTS - V25)/Avg_Slope + 25
  *         with VTS = temperature sensor voltage
  *              Avg_Slope = temperature sensor slope (unit: uV/DegCelsius)
  *              V25 = temperature sensor @25degC and Vdda 3.3V (unit: mV)
  *         Calculation validity conditioned to settings: 
  *          - ADC resolution 12 bits (need to scale value if using a different 
  *            resolution).
  *          - Power supply of analog voltage Vdda 3.3V (need to scale value 
  *            if using a different analog voltage supply value).
  * @param TS_ADC_DATA: Temperature sensor digital value measured by ADC
  * @retval None
  */
#define COMPUTATION_TEMPERATURE_STD_PARAMS(TS_ADC_DATA)                        \
  ((((int32_t)(INTERNAL_TEMPSENSOR_V25 - (((TS_ADC_DATA) * VDD_APPLI) / RANGE_12BITS)   \
     ) * 1000                                                                  \
    ) / INTERNAL_TEMPSENSOR_AVGSLOPE                                           \
   ) + 25                                                                      \
  )

/**
  * @brief  Computation of voltage (unit: mV) from ADC measurement digital
  *         value on range 12 bits.
  *         Calculation validity conditioned to settings: 
  *          - ADC resolution 12 bits (need to scale value if using a different 
  *            resolution).
  *          - Power supply of analog voltage Vdda 3.3V (need to scale value 
  *            if using a different analog voltage supply value).
  * @param ADC_DATA: Digital value measured by ADC
  * @retval None
  */
#define COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(ADC_DATA)                        \
  ( (ADC_DATA) * VDD_APPLI / RANGE_12BITS)
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of mutex01 */
  osMutexDef(mutex01);
  mutex01Handle = osMutexCreate(osMutex(mutex01));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of keySem */
//  osSemaphoreDef(keySem);
//  keySemHandle = osSemaphoreCreate(osSemaphore(keySem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of led1Task */
  osThreadDef(led1Task, vLed1Task, osPriorityNormal, 0, 128);
  led1TaskHandle = osThreadCreate(osThread(led1Task), NULL);

  /* definition and creation of led2Task */
  osThreadDef(led2Task, vLed2Task, osPriorityBelowNormal, 0, 128);
  led2TaskHandle = osThreadCreate(osThread(led2Task), NULL);

  /* definition and creation of led3Task */
  osThreadDef(led3Task, vLed3Task, osPriorityLow, 0, 128);
  led3TaskHandle = osThreadCreate(osThread(led3Task), NULL);

  /* definition and creation of led4Task */
//  osThreadDef(led4Task, vLed4Task, osPriorityNormal, 0, 128);
//  led4TaskHandle = osThreadCreate(osThread(led4Task), NULL);

//  /* definition and creation of printfTask */
//  osThreadDef(printfTask, vPrintfTask, osPriorityNormal, 0, 128);
//  printfTaskHandle = osThreadCreate(osThread(printfTask), NULL);

//  /* definition and creation of sensor1Task */
//  osThreadDef(sensor1Task, vSensor1Task, osPriorityAboveNormal, 0, 128);
//  sensor1TaskHandle = osThreadCreate(osThread(sensor1Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* vLed1Task function */
void vLed1Task(void const * argument)
{

  /* USER CODE BEGIN vLed1Task */
  /* Infinite loop */
   for (;;)
  {
    /* The first time through the mutex will be immediately available, on
    subsequent times through the mutex will be held by the low priority thread
    at this point and this Take will cause the low priority thread to inherit
    the priority of this tadhr.  In this case the block time must be
    long enough to ensure the low priority thread will execute again before the
    block time expires.  If the block time does expire then the error
    flag will be set here */
    if (osMutexWait(mutex01Handle, mutexTWO_TICK_DELAY) != osOK)
    {
      /* Toggle LED3 to indicate error */
      BSP_LED_Toggle(LED3);
    }

    /* Ensure the other thread attempting to access the mutex
    are able to execute to ensure they either block (where a block
    time is specified) or return an error (where no block time is
    specified) as the mutex is held by this task */
    osDelay(mutexSHORT_DELAY);

    /* We should now be able to release the mutex .
    When the mutex is available again the medium priority thread
    should be unblocked but not run because it has a lower priority
    than this thread.  The low priority thread should also not run
    at this point as it too has a lower priority than this thread */
    if (osMutexRelease(mutex01Handle) != osOK)
    {
      /* Toggle LED3 to indicate error */
      BSP_LED_Toggle(LED3);
    }

    /* Keep count of the number of cycles this thread has performed */
    HighPriorityThreadCycles++;
    BSP_LED_Toggle(LED1);

    /* Suspend ourselves to the medium priority thread can execute */
    osThreadSuspend(NULL);
  }
  /* USER CODE END vLed1Task */
}

/* vLed2Task function */
void vLed2Task(void const * argument)
{
  /* USER CODE BEGIN vLed2Task */
  (void) argument;
  /* Infinite loop */
    for (;;)
  {
    /* This thread will run while the high-priority thread is blocked, and the
    high-priority thread will block only once it has the mutex - therefore
    this call should block until the high-priority thread has given up the
    mutex, and not actually execute past this call until the high-priority
    thread is suspended */
    if (osMutexWait(mutex01Handle, osWaitForever) == osOK)
    {
      if (osThreadGetState(led1TaskHandle) != osThreadSuspended)
      {
        /* Did not expect to execute until the high priority thread was
        suspended.
        Toggle LED3 to indicate error */
        BSP_LED_Toggle(LED3);
      }
      else
      {
        /* Give the mutex back before suspending ourselves to allow
        the low priority thread to obtain the mutex */
        if (osMutexRelease(mutex01Handle) != osOK)
        {
          /* Toggle LED3 to indicate error */
          BSP_LED_Toggle(LED3);
        }
        osThreadSuspend(NULL);
      }
    }
    else
    {
      /* We should not leave the osMutexWait() function
      until the mutex was obtained.
      Toggle LED3 to indicate error */
      BSP_LED_Toggle(LED3);
    }

    /* The High and Medium priority threads should be in lock step */
    if (HighPriorityThreadCycles != (MediumPriorityThreadCycles + 1))
    {
      /* Toggle LED3 to indicate error */
      BSP_LED_Toggle(LED3);
    }

    /* Keep count of the number of cycles this task has performed so a
    stall can be detected */
    MediumPriorityThreadCycles++;
    BSP_LED_Toggle(LED2);
  }
  /* USER CODE END vLed2Task */
}

/* vLed3Task function */
void vLed3Task(void const * argument)
{
  /* USER CODE BEGIN vLed3Task */
  (void) argument;
  /* Infinite loop */
  for (;;)
  {
    /* Keep attempting to obtain the mutex.  We should only obtain it when
    the medium-priority thread has suspended itself, which in turn should only
    happen when the high-priority thread is also suspended */
    if (osMutexWait(mutex01Handle, 0) == osOK)
    {
      /* Is the haigh and medium-priority threads suspended? */
      if ((osThreadGetState(led1TaskHandle) != osThreadSuspended) || (osThreadGetState(led2TaskHandle) != osThreadSuspended))
      {
        /* Toggle LED3 to indicate error */
        BSP_LED_Toggle(LED3);
      }
      else
      {
        /* Keep count of the number of cycles this task has performed
        so a stall can be detected */
        LowPriorityThreadCycles++;

        /* We can resume the other tasks here even though they have a
        higher priority than the this thread. When they execute they
        will attempt to obtain the mutex but fail because the low-priority 
        thread is still the mutex holder.  this thread will then inherit 
        the higher priority.  The medium-priority thread will block indefinitely
        when it attempts to obtain the mutex, the high-priority thread will only
        block for a fixed period and an error will be latched if the
        high-priority thread has not returned the mutex by the time this
        fixed period has expired */
        osThreadResume(led2TaskHandle);
        osThreadResume(led1TaskHandle);

        /* The other two tasks should now have executed and no longer
        be suspended */
        if ((osThreadGetState(led1TaskHandle) == osThreadSuspended) || (osThreadGetState(led2TaskHandle) == osThreadSuspended))
        {
          /* Toggle LED3 to indicate error */
          BSP_LED_Toggle(LED3);
        }

        /* Release the mutex, disinheriting the higher priority again */
        if (osMutexRelease(mutex01Handle) != osOK)
        {
          /* Toggle LED3 to indicate error */
          BSP_LED_Toggle(LED3);
        }
      }
    }
    #if configUSE_PREEMPTION == 0
    {
      taskYIELD();
    }
#endif
  }
  /* USER CODE END vLed3Task */
}


/* vLed4Task function */
void vLed4Task(void const * argument)
{
  /* USER CODE BEGIN vLed4Task */
  /* Infinite loop */
  for(;;)
  {
    BSP_LED_Toggle(LED4);
    osDelay(100);
  }
  /* USER CODE END vLed4Task */
}

/* vPrintfTask function */
void vPrintfTask(void const * argument)
{
  /* USER CODE BEGIN vPrintfTask */
  /* Infinite loop */
  for(;;)
  {
    //temperature uiADC[2]
    
    //VOL  uiADC[3]
//    printf("Temperature now: %d \n",COMPUTATION_TEMPERATURE_STD_PARAMS(uiADC[2]));
//    printf("voltage now: %d \n",COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(uiADC[3]));
//    printf("Temperature now: %d \n",(uiADC[0]));
//    printf("voltage now: %d \n",(uiADC[3]));
    printf("realTemp: %d \n",realTemp);
//    char _ok[] = "OK";
//    HAL_StatusTypeDef state;
//    state = HAL_UART_Transmit_IT(&huart2,(uint8_t *)_ok, 1) ;
//    if (state  != HAL_OK)
//    {
//       traceString UART_Error = xTraceRegisterString("UART_STATE");
//       vTracePrintF(UART_Error,"ERROR %d",state);
//    }
    osDelay(500);
   
  }
  /* USER CODE END vPrintfTask */
}

/* vSensor1Task function */
void vSensor1Task(void const * argument)
{
  /* USER CODE BEGIN vSensor1Task */
  /* Infinite loop */
     /* Start ADC conversion on regular group with transfer by DMA */
if (HAL_ADC_Start_DMA(&hadc1,
                        (uint32_t *)uiADC,
                        4
                       ) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
  /* Infinite loop */
  for(;;)
  {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1,0xff); 
    vTempCtrl(uiADC[0]);
//    uiADC = HAL_ADC_GetValue(&hadc1);
    osDelay(1);
  /* USER CODE END vSensor1Task */
}

/* USER CODE BEGIN Application */
 

}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

#include "solder.h"



#define HotterTempSet 100

uint16_t realTemp;

static uint16_t configureTemp = 100;

inline void vHeat()
{
  HAL_GPIO_WritePin(HEAT_GPIO_Port,HEAT_Pin,GPIO_PIN_SET);
}

inline void vUnHeat()
{
  HAL_GPIO_WritePin(HEAT_GPIO_Port,HEAT_Pin,GPIO_PIN_RESET);
}


uint16_t uGetTemperature (uint16_t adc)
{
 
  realTemp = 0.3105*adc + -157; 
  return realTemp;
}


void  vTempCtrl(uint16_t adc)
{
  uint16_t temperature;
  temperature = uGetTemperature(adc);
  if (temperature > configureTemp)
    vUnHeat();
  else 
    vHeat();
}



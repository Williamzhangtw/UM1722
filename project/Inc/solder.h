#ifndef SOLDER_H
#define SOLDER_H
#include "stm32f1xx_hal.h"




extern uint16_t realTemp;

uint16_t uGetTemperature (uint16_t adc);





void vTempCtrl(uint16_t adc);














#endif 

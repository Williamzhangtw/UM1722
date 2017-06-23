#include "stm32f1xx_hal.h" 

#ifndef _HOTER1321_H
#define _HOTER1321_H

#define T_MAX  500
#define T_MIN	100

#define C_MAX  100
#define C_MIN  -100

#define KS   0.2
#define BS   56

#define SENSOR_ERR_ADC 4000

#define SLEEP_TEMP   100

#define T_PRECISE    5

#define NO_ERR 0
#define Sensor_ERR 1
#define Hotter_ERR 2

typedef struct _m_hotter
{
	GPIO_TypeDef*  GPIO_PORT;// CLK
	uint16_t  GPIO_PIN;// CLK

	uint16_t _adc;//
	int16_t _temperature;//
	uint8_t _err;
	uint8_t _state;//heating,stop heating,cooling,keep constant temperature
	 

	int16_t _targetTemperature;//
	int16_t _adjustTemperature;//
	
	
	


}m_hotter;
 

 typedef enum 
 {
	HEATTING,//
	OVER_HEAT,//
	NOT_HEAT,//
	TEMP_BALANCE,//
 }
Hotter_state_enum;
 
 

void HotterInit(m_hotter * hotter,GPIO_TypeDef*  GPIO_PORT,uint16_t GPIO_PIN);	
// 
int	GetTemperature (m_hotter *hotter);
// 
void HotterCtrl (m_hotter *hotter,_Bool enHeat);
#endif

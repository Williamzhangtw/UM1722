#include "../bsp/hotter.h"

/* USER CODE BEGIN Includes */



void HotterInit(m_hotter * hotter,GPIO_TypeDef*  GPIO_PORT,uint16_t GPIO_PIN)	
{
	hotter-> GPIO_PORT =GPIO_PORT ;
	hotter-> GPIO_PIN = GPIO_PIN;//

	hotter-> _adc =0;//
	hotter->  _temperature =0;//
	hotter->  _err = NO_ERR ;
	hotter->  _state = HEATTING;//heating,stop heating,cooling,keep constant temperature
	 

	hotter->  _targetTemperature = 150;//
	hotter->  _adjustTemperature =0;//
	
	

	
	
	
	
	
}


void GetHotterState(m_hotter * hotter)
{
	int8_t  dif ;
	dif =  hotter->_temperature -hotter->_targetTemperature ;  
	switch(hotter->_state )
	{
		case HEATTING : //
			if ( dif  >=0 )
			{
				 hotter->_state =OVER_HEAT ;
			
			}
			break ;

		case OVER_HEAT  : //
			if (  dif  <0 )
			{
				 hotter->_state =TEMP_BALANCE ;
			}
			break ;
			
		case NOT_HEAT : 	//
			if (  dif  <=0 )
			{
				hotter->_state  = TEMP_BALANCE ;
			}
			break ;
			
		case TEMP_BALANCE  ://
			if (  dif  > T_PRECISE)
				hotter->_state  = NOT_HEAT  ; 
			if (  dif  <T_PRECISE )
				hotter->_state  = HEATTING  ; 	
			break;
		
		default :
			break ;
	}
}

inline int GetHotterADC(m_hotter *hotter)
{
	return 0;
}




int	GetTemperature (m_hotter *hotter)
{
	int temperature;
	temperature = hotter->_adc*KS+BS-hotter ->_adjustTemperature ; 
	hotter->_temperature = temperature;
	return temperature;
}

 
/*heated
*/
inline void EnableHeat(m_hotter *hotter)
{
	HAL_GPIO_WritePin(hotter->GPIO_PORT,hotter->GPIO_PIN,GPIO_PIN_SET);
}

inline void DisableHeat(m_hotter *hotter)
{
	HAL_GPIO_WritePin(hotter->GPIO_PORT,hotter->GPIO_PIN,GPIO_PIN_RESET);
}



void HotterCtrl (m_hotter *hotter,_Bool enHeat)
{
  if(enHeat)// heated
  {
    EnableHeat(hotter);
  }
  else // not heated
  {
		DisableHeat(hotter);
  }
}


 





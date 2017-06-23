#include "TM1650.h"
#include "font_7seg.h"


/** Generate Start condition for TM1650
  *  @param  none
  *  @return none
  */ 

void TM1650_Start(m_tm1650 * tm1650) {

//  wait_us(1);
  HAL_GPIO_WritePin(tm1650->SCL_GPIOx,tm1650->SCL_GPIO_Pin,GPIO_PIN_SET);	
	HAL_GPIO_WritePin(tm1650->SDA_GPIOx,tm1650->SDA_GPIO_Pin,GPIO_PIN_SET);
	
  HAL_GPIO_WritePin(tm1650->SDA_GPIOx,tm1650->SDA_GPIO_Pin,GPIO_PIN_RESET);
//  wait_us(1);
  HAL_GPIO_WritePin(tm1650->SCL_GPIOx,tm1650->SCL_GPIO_Pin,GPIO_PIN_RESET);
//  wait_us(1);
}
  
/** Generate Stop condition for TM1650
  *  @param  none
  *  @return none
  */ 
void TM1650_Stop(m_tm1650 * tm1650) {
  HAL_GPIO_WritePin(tm1650->SCL_GPIOx,tm1650->SCL_GPIO_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(tm1650->SDA_GPIOx,tm1650->SDA_GPIO_Pin,GPIO_PIN_RESET);
//  wait_us(1);  

//  wait_us(1);
  HAL_GPIO_WritePin(tm1650->SDA_GPIOx,tm1650->SDA_GPIO_Pin,GPIO_PIN_SET);
//  wait_us(1);
}

/** Send byte to TM1650
  *  @param  int data
  *  @return none
  */ 
void TM1650_Write(m_tm1650 * tm1650,uint8_t data) {
 
uint8_t i;
for(i=0;i<8;i++)
{
	if(data&0x80)
		HAL_GPIO_WritePin(tm1650 ->SDA_GPIOx,tm1650 ->SDA_GPIO_Pin,GPIO_PIN_SET);
	else 
		HAL_GPIO_WritePin(tm1650 ->SDA_GPIOx,tm1650 ->SDA_GPIO_Pin,GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin(tm1650 ->SCL_GPIOx,tm1650 ->SCL_GPIO_Pin,GPIO_PIN_SET); 
	data<<=1;
	HAL_GPIO_WritePin(tm1650 ->SCL_GPIOx,tm1650 ->SCL_GPIO_Pin,GPIO_PIN_RESET);
} 

	
	 HAL_GPIO_WritePin(tm1650->SDA_GPIOx,tm1650->SDA_GPIO_Pin,GPIO_PIN_SET); //I ADD
	
  // dummy Ack
  HAL_GPIO_WritePin(tm1650->SCL_GPIOx,tm1650->SCL_GPIO_Pin,GPIO_PIN_SET);
 // wait_us(1);
  HAL_GPIO_WritePin(tm1650->SCL_GPIOx,tm1650->SCL_GPIO_Pin,GPIO_PIN_RESET);  
 // wait_us(1); 
}
/** Write command and parameter to TM1650
  *  @param  int cmd Command byte
  *  &Param  int data Parameters for command
  *  @return none
  */  
void TM1650_WriteCmd(m_tm1650 * tm1650,uint8_t cmd1,uint8_t cmd2){
    
  TM1650_Start(tm1650);

//  TM1650_Write((cmd & TM1650_CMD_MSK) | (data & ~TM1650_CMD_MSK));   
	TM1650_Write(tm1650,cmd1);
	TM1650_Write(tm1650,cmd2);
  TM1650_Stop(tm1650);          
}  



/** Set Brightness
  *
  * @param  char brightness (3 significant bits, valid range 0..7 (1/16 .. 14/14 dutycycle)  
  * @return none
  */
void TM1650_SetBrightness(m_tm1650 * tm1650,char brightness){

  tm1650->_bright = brightness & TM1650_BRT_MSK; // mask invalid bits
  
  TM1650_WriteCmd(tm1650,TM1650_DSP_CTRL_CMD, tm1650->_segment |tm1650->_display | tm1650->_bright );  // Display control cmd, display on/off, brightness  
}
//either can be 7 digi or 8 digi
void TM1650_SetSegment(m_tm1650 * tm1650,char segment){

  tm1650->_segment = segment & TM1650_SEGMENT_MSK; // mask invalid bits
  
  TM1650_WriteCmd(tm1650,TM1650_DSP_CTRL_CMD, tm1650->_segment |tm1650->_display | tm1650->_bright );  // Display control cmd, display on/off, brightness  
}



/** Set the Display mode On/off
  *
  * @param _Bool display mode
  */
void TM1650_SetDisplay(m_tm1650 * tm1650,_Bool on) {
  
  if (on) {
    tm1650->_display = TM1650_DSP_ON;
  }
  else {
    tm1650->_display = TM1650_DSP_OFF;
  }
  	
  TM1650_WriteCmd(tm1650,TM1650_DSP_CTRL_CMD, tm1650->_display | tm1650->_bright );  // Display control cmd, display on/off, brightness   
}





void TM1650_Init(m_tm1650 * tm1650,	GPIO_TypeDef  *SCL_GPIOx,uint32_t SCL_GPIO_Pin,GPIO_TypeDef  *SDA_GPIOx,uint32_t SDA_GPIO_Pin)
{
  
//TM1650 uses a Serial bus that looks like I2C, but really is not.
//It has Start and Stop conditions like I2C and an Ack pulse, but instead of slaveaddresses and a RW bit it just transmits commands and data.
//init Serial bus
  
  tm1650->SCL_GPIOx = SCL_GPIOx;
	tm1650->SCL_GPIO_Pin =SCL_GPIO_Pin;
	tm1650->SDA_GPIOx = SDA_GPIOx;
	tm1650->SDA_GPIO_Pin =SDA_GPIO_Pin;
//init controller  
  tm1650->_display = TM1650_DSP_ON;
  tm1650->_bright  = TM1650_BRT0; 
  tm1650->_segment =TM1650_8_SEGMENT;
	HAL_GPIO_WritePin(tm1650->SDA_GPIOx,tm1650->SDA_GPIO_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(tm1650->SCL_GPIOx,tm1650->SCL_GPIO_Pin,GPIO_PIN_SET);
	//TM1650_WriteCmd(tm1650,TM1650_DSP_CTRL_CMD, tm1650->_segment |tm1650->_display | tm1650->_bright );  
	TM1650_WriteCmd(tm1650,0X48,0x01 ); 	// Display control cmd, display on/off, brightness   
	//TM1650_WriteCmd(tm1650,TM1650_DATA_SET_CMD, TM1650_DATA_WR | TM1650_ADDR_INC | TM1650_MODE_NORM); // Data set cmd, normal mode, auto incr, write data  
}   


void TM1650_ClearDisplay( m_tm1650 *tm1650 ) {
	for( uint8_t i=0; i<5; i+=2) {
		
		TM1650_WriteCmd( tm1650,0x68+i, 0 );
		
	}
	TM1650_WriteCmd(tm1650,0X68,0 ); 
 	TM1650_WriteCmd(tm1650,0X6A,0);  //
	TM1650_WriteCmd(tm1650,0X6C,0);
}
void TM1650_DisplayNum(  m_tm1650 *tm1650,int num ) {
	int8_t value[4];
	int value_temp;
	if(num>=0)
	{
		value[0] = num %10;
		value[1] = num /10%10;
		value[2] = num /100%10;
	//	value[3] = num /1000%10;
	}
	else
	{
		value_temp = __fabs (num);
		value[0] = value_temp %10;
		value[1] = value_temp /10%10;
		value[2] = value_temp /100%10;
	//	value[3] = num /1000%10;
	}
	uint8_t j =0;
	for (uint8_t i=0;i<3;i++)
	{
		
		TM1650_WriteCmd(tm1650,0X68+j,FONT_7S[value[i]+16] ); 
		j =j +2;
	}
}
 
void TM1650_DisplayStrings(  m_tm1650 *tm1650,char *data ) {
	uint8_t i=0; 
	while ((*data !=0 ))
	
	{
			
		  TM1650_WriteCmd(tm1650,0X6c-i,FONT_7S[*data-32] ); 
			data++;i=i+2;
	}
}



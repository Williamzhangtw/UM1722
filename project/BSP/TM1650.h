/** Constructor for class for driving TM1650 LED controller with Serial bus interface device. 
* @brief Supports 4 digits @ 8 segments.cathode 阴
* @auther   William Chang
* @time   July 10, 2017.
*/

#ifndef TM1650_H
#define TM1650_H

//!include "main.h" "stm32f1xx_hal_conf.h" "stm32f1xx_hal_xx_h"#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"  


#define LOCAL_NUM_DIGITS 3

//Reserved bits for commands
#define TM1650_CMD_MSK      0xC0

#define TM1650_CONTROL_BASE		0x24	// Address of the control register of the left-most digit
#define TM1650_DISPLAY_BASE		0x34	// Address of the left-most digit 
#define TM1650_NUM_DIGITS		4	// 4 segments

#define TM1650_MASK_BRIGHTNESS		0b10001111
#define TM1650_MASK_COLON		0b11110111
#define TM1650_BIT_COLON		0b00001000
#define TM1650_MASK_ONOFF		0b11111110
#define TM1650_BIT_ONOFF		0b00000001
/*
Brightness settings
MSB				LSB
B7 B6 B5 B4 B3 B2 B1 B0 Explanation
 ×  0  0  0     ×  ×	8 brightness
 ×  0  0  1     ×  ×	1 brightness
 ×  0  1  0     ×  ×	2 brightness
 ×  0  1  1     ×  ×	3 Brightness
 ×  1  0  0     ×  ×	4 brightness
 ×  1  0  1     ×  ×	5 brightness
 ×  1  1  0     ×  ×	6 brightness
 ×  1  1  1     ×  ×	7 brightness
On / off the display position
MSB				LSB
B7 B6 B5 B4 B3 B2 B1 B0 Explanation
 ×              ×  ×  0 Off Display
 ×              ×  ×  1 On Display
 */

////Address setting commands
#define TM1650_ADDR_SET_CMD 0xC0
#define TM1650_ADDR_MSK     0x07 //0..5

//Display control commands
#define TM1650_DSP_CTRL_CMD 0x48
#define TM1650_BRT_MSK      0x70
#define TM1650_BRT0         0x00 //Pulsewidth 1/16
#define TM1650_BRT1         0x10
#define TM1650_BRT2         0x20
#define TM1650_BRT3         0x30
#define TM1650_BRT4         0x40
#define TM1650_BRT5         0x50
#define TM1650_BRT6         0x60
#define TM1650_BRT7         0x70 //Pulsewidth 14/16

//Segment mode
#define TM1650_SEGMENT_MSK  0x40
#define TM1650_7_SEGMENT  	0x40
#define TM1650_8_SEGMENT  	0x00


//#define TM1650_BRT_DEF      TM1650_BRT3

#define TM1650_DSP_OFF      0x00
#define TM1650_DSP_ON       0x01




typedef struct _TM1650 {


  GPIO_TypeDef  *SCL_GPIOx;
  uint32_t SCL_GPIO_Pin;

  GPIO_TypeDef  *SDA_GPIOx;
  uint32_t SDA_GPIO_Pin;

  char _display;

  char _bright; 
  //display o
  char _segment;

}m_tm1650;


//init
void TM1650_Init(m_tm1650 * tm1650,	GPIO_TypeDef  *SCL_GPIOx,uint32_t SCL_GPIO_Pin,GPIO_TypeDef  *SDA_GPIOx,uint32_t SDA_GPIO_Pin);
//set brightness
void TM1650_SetBrightness(m_tm1650 * tm1650,char brightness);
//display strings
void TM1650_DisplayStrings(  m_tm1650 *tm1650,char *data );
//clear display
void TM1650_ClearDisplay( m_tm1650 *tm1650 ) ;
//display num
void TM1650_DisplayNum(  m_tm1650 *tm1650,int num );


/*test file
  * 1st: Pin initialize
  m_tm1650 _disp1;
  TM1650_Init(&_disp1,TM1651_CLK_2_GPIO_Port,TM1651_CLK_2_Pin,TM1651_DIO_2_GPIO_Port,TM1651_DIO_2_Pin);
  delay 100ms //
  TM1650_ClearDisplay(&_disp1);	
	TM1650_DisplayNum(&_disp1,_disPlayNum);

  }
*/

#endif

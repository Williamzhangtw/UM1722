/*
This is used to control buttons (with hardware debounce)
if external pull-ups,variable active should be 0;else be 1; 
Auother : William Chang
Time : July 10, 2017.
*/
#ifndef __BUTTON_H
#define __BUTTON_H

//!include "main.h" "stm32f1xx_hal_conf.h" "stm32f1xx_hal_xx_h"#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"  


/** Function: Use to read mechanical button
  * Example on How to use it.
  * @Code
  * #include "button.h"
  * 1st: we should initialize the PIN which connect to the button,set as an input port. If possible,we can set it as pull-ups
  * 2nd: Create buttons instance
  *   m_button _btnSet;
  * 3rd: Init the instance
  *   function protype
  *   void ButtonInit(m_button*button,GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin,_Bool active);
  *     @param *button where the instance's address
  *     @param *GPIOx  where the button's GPIO port
  *     @param GPIO_Pin where the button's GPIO_Pin
  *     @param active logic level that gives you when the button is pressed
  *   e.g. ButtonInit(&_btnSet,SET_KEY_GPIO_Port,SET_KEY_Pin,0);
  * 4th: Now we can use it by calling IsPressed(m_button*button)or (IsReleased(m_button*button))function,they will return the buttons is pressed or not
  * e.g.  int task(*)
  *       { 
  * 	      for(;;){ 
  *           delay 10ms //it is needed,let me explain below.
  *           if (IsPressed(&_btnSet))         
  *			      //do something
  *           ...or....
  *           if(IsReleased(&_btnSet))
  *			      //do something
  *		      }
  *       }
  *
  * why we need to delay.because we have a task to distinguish the pressed is real or not,CheckBtnStatus()do this job
  *  if we scan too fast, we can consider the noise as a press,so we need to have a delay.
  * the delay time consider to be 10ms, so it won't too slow to react and won't judge wrong too.
  *
  *
  * @endcode
  */

//! button modual structure
typedef struct _mButton
{
//variable need to be set  
  
  //the button GPIO Port
	GPIO_TypeDef  *GPIOx;
  //the button GPIO Pin
	uint32_t GPIO_Pin;
  //the logic level when button is pressed
	_Bool active;   



  
//private
  //stored the logic level of button now
	_Bool _buttonState;
  //stored the logic level of button last or we can say pre
	_Bool _lastButtonState;	
	
  //the flag indicate the button status,1 means ture,it is pressed.
	_Bool _isPressed;
  //the flag indicate the button status,1 means ture,it is released.
	_Bool _isReleased;
  
}m_button;


/** Init the instance
* @return void
*
* Example:
* @code
*   
* @endcode
*/
void ButtonInit(m_button*button,GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin,_Bool active);

/** Tells you if a button has been pressed (only once)
* @return '1' if the buttons has been pressed
*
* Example:
* @code
*   if( btn.isPressed( ) )
*       // do something
* @endcode
*/

_Bool IsPressed(m_button  *button);
/** Tells you if a button has been Released (only once)
* @return '1' if the buttons has been Released
*
* Example:
* @code
*   if( btn.isReleased( ) )
*       // do something
* @endcode
*/
_Bool IsReleased(m_button  *button);

_Bool IsClick(m_button  *button);


/*test file
  * 1st: we should initialize the PIN which connect to the button,set as an input port. If possible,we can set it as pull-ups
  m_button _btnSet;
  void ButtonInit(m_button*button,GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin,_Bool active);
  for(;;){ 
  delay 10ms //it is needed,let me explain below.
    if (IsPressed(&_btnSet))         
    //do something

  }
*/




#endif

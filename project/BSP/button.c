#include "../bsp/button.h"
/** Tells you the button state
* @return '1' if the buttons is pressed
*
* Example:
* @code
*   if( btn.istatus( ) )
*       // button pressed
*   else
*       // button not pressed
* @endcode
*/
inline static _Bool Status(m_button *button)
{
	_Bool status;
	status = HAL_GPIO_ReadPin(button->GPIOx,button->GPIO_Pin);
	return status;
}



/** make sure the button state if right,
*regenerate mark flag _isPressed or _isReleased
* @return void
*
* Example:
* @code
*   CheckBtnStatus( ) 
* @endcode
*/

void CheckBtnStatus(m_button *button)
{
  
	button->_buttonState  = Status(button);
	if (button->_buttonState == button->_lastButtonState)
	{
		if(button->_buttonState == button->active)
		{
			button->_isPressed = 1;
		}
		else
		{
			button->_isReleased = 1;
		}
	}
	button->_lastButtonState = button->_buttonState;
}

/** Tells you if a button has been pressed (only once)
* @return '1' if the buttons has been pressed
*
* Example:
* @code
*   if( btn.isPressed( ) )
*       // do something
* @endcode
*/
_Bool IsPressed(m_button  *button)
{
	_Bool isPressed;
	CheckBtnStatus(button);
	isPressed = button-> _isPressed;
	button-> _isPressed = 0;
	return isPressed;
}
/** Tells you if a button has been Released (only once)
* @return '1' if the buttons has been Released
*
* Example:
* @code
*   if( btn.isReleased( ) )
*       // do something
* @endcode
*/
_Bool IsReleased(m_button  *button)
{
	_Bool isReleased;
	CheckBtnStatus(button);
	isReleased =  button-> _isReleased;
	button-> _isReleased = 0;
	return isReleased;
}

_Bool IsClick(m_button  *button)
{
  static _Bool dir = 0;
  CheckBtnStatus(button);
  if (dir == 0)
  {
    if (button-> _isPressed == 1)
    {
      button-> _isPressed =0;
      button-> _isReleased = 0;
      dir = 1;
    }
 
  }
  else 
  {
    if (button->_isReleased ==1)
    {
      button-> _isReleased = 0;
      button-> _isPressed =0;
      dir =0;
      return 1;
    }
  }
	return 0;
}



/** Init the instance
* @return void
*
* Example:
* @code
*   
* @endcode
*/

void ButtonInit(m_button *button,GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin,_Bool active)
{
	button->GPIOx =GPIOx;
	button->GPIO_Pin=GPIO_Pin;
	
	button->active=active;
	button->_buttonState=0;
	button->_lastButtonState=0;
	button->_isPressed=0;
	button->_isReleased=0;
	
}






#include "ExHardware.h"
#include "cmsis_os.h"
#include "main.h"

/**
 * usd in backlight function
 **/
extern TIM_HandleTypeDef htim3;


/**
 * usd in buzzer function
 **/
bool TimerFlag = false;

uint8_t backlight = 0xFF;

void Backlight_Control ( BACKLIGHT_CONTROL BL_Ctrl )
{
	switch( BL_Ctrl )
	{
		//==============================================
		case BL_ON:
			
			HAL_GPIO_WritePin( LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_SET ); // Set the EN pin of BL driver IC to high, enable it
			Set_Backlight_Duty( backlight );
			HAL_TIM_PWM_Start( &htim3, TIM_CHANNEL_3 );
			
			break;
		//==============================================
		case BL_OFF:
		default:
			
			HAL_TIM_PWM_Stop( &htim3, TIM_CHANNEL_3 );
			Set_Backlight_Duty( 0 );
			HAL_GPIO_WritePin( LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET ); // Set the EN pin of BL driver IC to low, disable it
			
			break;
		//==============================================
	}
}

void Set_Backlight_Duty ( uint8_t d )
{
	#define MaxTrueDuty 92
	#define minTrueDuty 20
	
	if( backlight == d )
		return;

	d = ( d > 100 ) ? 100 : d;
	
	backlight = d;
	
	if( d == 0 )
		goto set_duty;

	d = ((MaxTrueDuty - minTrueDuty) * d ) / 100 + minTrueDuty;
	
	set_duty:
	__HAL_TIM_SET_COMPARE( &htim3, TIM_CHANNEL_3, d );
}

#include <gui/screen_screen/screenView.hpp>
#include <touchgfx/Color.hpp>
#include "stm32f7xx_hal.h"
#include <BitmapDatabase.hpp>
#include "main.h"
#include "ExHardware.h"
int test = 0;

screenView::screenView():
		ButtonClickCallback(this,&screenView::ButtonClickHandler),
		tick(0)
{

}

void screenView::setupScreen()
{
    screenViewBase::setupScreen();
    // open backlight
    Backlight_Control( BL_ON ); // blacklight on
    Set_Backlight_Duty( 100 );
    button1.setClickAction(ButtonClickCallback);
}

void screenView::tearDownScreen()
{
    screenViewBase::tearDownScreen();
}

void screenView::handleTickEvent()
{
	calendar_t calendar;
	  if ( (--tick) > 0 )
	    return;

	  tick = 10;

	  get_clock_from_RTC( &calendar );

	  digitalClock1.setTime24Hour(calendar.map.Hours, calendar.map.Minutes, calendar.map.Seconds);
	  digitalClock1.invalidate();
}
void screenView::ButtonClickHandler(const Button& ta,const ClickEvent& evt)
{
//	if(&ta == &button1 && evt.getType() == ClickEvent::PRESSED)
//	{
//
//	}
//	else
//	{
//
//	}
}

void screenView::updateScreen( Data_t Data)
{
	Unicode::snprintf(textArea1Buffer, TEXTAREA1_SIZE, "%d", Data.num1);
	textArea1.invalidate();
}

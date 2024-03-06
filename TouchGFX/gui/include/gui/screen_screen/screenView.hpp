#ifndef SCREENVIEW_HPP
#define SCREENVIEW_HPP

#include <gui_generated/screen_screen/screenViewBase.hpp>
#include <gui/screen_screen/screenPresenter.hpp>

class screenView : public screenViewBase
{
public:
    screenView();
    virtual ~screenView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();
    void ButtonClickHandler(const Button& ta,const ClickEvent& e);
    void updateScreen(Data_t Data);
protected:
    Callback<screenView,const Button&,const ClickEvent&>ButtonClickCallback;
protected:
    int32_t tick;
};

#endif // SCREENVIEW_HPP

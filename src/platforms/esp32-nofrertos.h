#include <Arduino.h>

#define TIMER_ID	0

hw_timer_t * _uclockTimer = NULL;
portMUX_TYPE _uclockTimerMux = portMUX_INITIALIZER_UNLOCKED;
#define ATOMIC(X) portENTER_CRITICAL_ISR(&_uclockTimerMux); X; portEXIT_CRITICAL_ISR(&_uclockTimerMux);

// forward declaration of uClockHandler
void uClockHandler();

// ISR handler
void ARDUINO_ISR_ATTR handlerISR(void)
{
    uClockHandler();
}

void initTimer(uint32_t init_clock)
{
    _uclockTimer = timerBegin(init_clock);

    // attach to generic uclock ISR
    timerAttachInterrupt(_uclockTimer, &handlerISR);

    // init clock tick time
    timerAlarm(_uclockTimer, init_clock, true, 0); 
}

void setTimer(uint32_t us_interval)
{
    timerAlarmWrite(_uclockTimer, us_interval, true); 
}
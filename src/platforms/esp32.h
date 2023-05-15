#include <Arduino.h>

#define TIMER_ID	0

hw_timer_t * _uclockTimer = NULL;
portMUX_TYPE _uclockTimerMux = portMUX_INITIALIZER_UNLOCKED;
#define ATOMIC(X) portENTER_CRITICAL_ISR(&_uclockTimerMux); X; portEXIT_CRITICAL_ISR(&_uclockTimerMux);

// forward declaration of ISR
void ARDUINO_ISR_ATTR uclockISR();

void initTimer(uint32_t init_clock)
{
    _uclockTimer = timerBegin(TIMER_ID, 80, true);

    // attach to generic uclock ISR
    timerAttachInterrupt(_uclockTimer, &uclockISR, true);

    // init clock tick time
    timerAlarmWrite(_uclockTimer, init_clock, true); 

    // activate it!
    timerAlarmEnable(_uclockTimer);
}

void setTimer(uint32_t us_interval)
{
	timerAlarmWrite(_uclockTimer, us_interval, true); 
}
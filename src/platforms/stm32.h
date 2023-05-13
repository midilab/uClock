#include <Arduino.h>

#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x01090000)
  #error "Due to API change, this library is compatible with STM32_CORE_VERSION >= 0x01090000. Please update your stm32duino core."
#endif

#if defined(TIM1)
  TIM_TypeDef * TimerInstance = TIM1;
#else
  TIM_TypeDef * TimerInstance = TIM2;
#endif

// Instantiate HardwareTimer object. Thanks to 'new' instanciation, HardwareTimer is not destructed when setup() function is finished.
HardwareTimer * _uclockTimer = new HardwareTimer(TimerInstance);

#define ATOMIC(X) noInterrupts(); X; interrupts();

// forward declaration of ISR
void uclockISR();

void initTimer(uint32_t us_interval)
{
  _uclockTimer->setOverflow(us_interval, MICROSEC_FORMAT);
  _uclockTimer->attachInterrupt(uclockISR);
  _uclockTimer->resume();
}

void setTimer(uint32_t us_interval)
{
  _uclockTimer->setOverflow(us_interval, MICROSEC_FORMAT);
  _uclockTimer->refresh();
}


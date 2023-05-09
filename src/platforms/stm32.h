#include <Arduino.h>

static TIM_HandleTypeDef _uclockTimer = {0};

#define ATOMIC(X) noInterrupts(); X; interrupts();

// forward declaration of ISR
void uclockISR();

void timer_attachInterrupt(TIM_TypeDef *tim, uint32_t us_interval)
{
  // Enable timer clock
  if (tim == TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
  else if (tim == TIM3) __HAL_RCC_TIM3_CLK_ENABLE();
  else if (tim == TIM4) __HAL_RCC_TIM4_CLK_ENABLE();

  // Calculate the prescaler value
  uint32_t prescaler = (SystemCoreClock / 1000000UL) - 1;

  // Calculate the period value
  uint32_t period = (us_interval * 2UL) - 1UL;

  // Set up the timer instance
  _uclockTimer.Instance = tim;
  _uclockTimer.Init.Prescaler = prescaler;
  _uclockTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
  _uclockTimer.Init.Period = period;
  _uclockTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  // Configure the timer instance
  HAL_TIM_Base_Init(&_uclockTimer);
  HAL_TIM_Base_Start_IT(&_uclockTimer);
}

void TIM2_IRQHandler()
{
  // Call the uClock ISR handler
  uclockISR();

  // Clear the interrupt flag
  __HAL_TIM_CLEAR_FLAG(&_uclockTimer, TIM_FLAG_UPDATE);
}

void initTimer(uint32_t us_interval)
{
  // Set up the timer to call the callback function every us_interval microseconds
  timer_attachInterrupt(TIM2, us_interval);
}

void setTimer(uint32_t us_interval)
{
  // Calculate the period value
  uint32_t period = (us_interval * 2UL) - 1UL;

  // Update the timer instance with the new period value
  _uclockTimer.Init.Period = period;
  HAL_TIM_Base_Init(&_uclockTimer);
}


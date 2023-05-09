#include <Arduino.h>

static TIM_HandleTypeDef s_TimerInstance = {0};

typedef void (*timer_callback_t)(void);
timer_callback_t timer_callback = NULL;

#define ATOMIC(X) noInterrupts(); X; interrupts();

// forward declaration of ISR
void uclockISR();

void timer_attachInterrupt(TIM_TypeDef *tim, uint32_t microseconds, timer_callback_t callback)
{
  // Enable timer clock
  if (tim == TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
  else if (tim == TIM3) __HAL_RCC_TIM3_CLK_ENABLE();
  else if (tim == TIM4) __HAL_RCC_TIM4_CLK_ENABLE();

  // Calculate the prescaler value
  uint32_t prescaler = (SystemCoreClock / 1000000UL) - 1;

  // Calculate the period value
  uint32_t period = (microseconds * 2UL) - 1UL;

  // Set up the timer instance
  s_TimerInstance.Instance = tim;
  s_TimerInstance.Init.Prescaler = prescaler;
  s_TimerInstance.Init.CounterMode = TIM_COUNTERMODE_UP;
  s_TimerInstance.Init.Period = period;
  s_TimerInstance.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  // Configure the timer instance
  HAL_TIM_Base_Init(&s_TimerInstance);
  HAL_TIM_Base_Start_IT(&s_TimerInstance);

  // Save the callback function
  timer_callback = callback;
}

void TIM2_IRQHandler()
{
  // Call the callback function
  timer_callback();

  // Clear the interrupt flag
  __HAL_TIM_CLEAR_FLAG(&s_TimerInstance, TIM_FLAG_UPDATE);
}

void initTimer(uint32_t us_interval)
{
  // Set up the timer to call the callback function every us_interval microseconds
  timer_attachInterrupt(TIM2, us_interval, uclockISR);
}

void setTimer(uint32_t us_interval)
{
  // Calculate the period value
  uint32_t period = (us_interval * 2UL) - 1UL;

  // Update the timer instance with the new period value
  s_TimerInstance.Init.Period = period;
  HAL_TIM_Base_Init(&s_TimerInstance);
}


#include <Arduino.h>
#include <util/atomic.h>

#define ATOMIC(X) ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { X; }

#if defined(__IMXRT1062__)
    #define UCLOCK_HAS_PLATFORM_EXTERNAL_CLOCK_TIMESTAMP

    static inline uint32_t uclockPlatformExternalClockTimestamp()
    {
        return ARM_DWT_CYCCNT;
    }

    static inline uint32_t uclockPlatformExternalClockIntervalUs(
        uint32_t previous_timestamp, uint32_t current_timestamp)
    {
        uint32_t elapsed_cycles = current_timestamp - previous_timestamp;
        return (uint32_t)(((uint64_t)elapsed_cycles * 1000000ULL) /
                          (uint32_t)F_CPU_ACTUAL);
    }
#endif

IntervalTimer _uclockTimer;

// forward declaration of ISR
void uClockHandler();

void initTimer(uint32_t init_clock)
{
    _uclockTimer.begin(uClockHandler, init_clock);

    // Set the interrupt priority level, controlling which other interrupts
    // this timer is allowed to interrupt. Lower numbers are higher priority,
    // with 0 the highest and 255 the lowest. Most other interrupts default to 128.
    // As a general guideline, interrupt routines that run longer should be given
    // lower priority (higher numerical values).
    _uclockTimer.priority(80);
}

void setTimer(uint32_t us_interval)
{
    _uclockTimer.update(us_interval);
}

#pragma once

// enable timer module, the library needs the definitions
#define HAL_TIM_MODULE_ENABLED

// include HAL framework regardless of CPU type, this will include the timer module
#include "stm32f4xx_hal.h"

#include "CallbackChain.hpp"
#include "Timer.hpp"


// Callback chain setup for HAL_TIM_OC_DelayElapsedCallback function
struct TIM_OC_DelayElapsed_CallbackChainID{};
using TIM_OC_DelayElapsed_CallbackChain = CallbackChain<TIM_OC_DelayElapsed_CallbackChainID, TIM_HandleTypeDef*>;


// Implements timer controller for hardware handling,
// it encapsulates any hardware related issue and presents a simple common API.
// Requires a timer with capture compare capabilities.
// By default has 10 kHz tick speed, a 1 ms delay needs a tick value of 10, 0.5 sec is 5000 ticks.
// 
// fclk: timer's input clock speed, will be divided by clkdiv
// clkdiv: how much clock division is required, maximum allowed value depends on the specific timer's prescale register's size
//         currently limited for every timer to 65536 (16 bit prescale register), it could become a setting if needed
// bits: the number of bits in the counter register (16 or 32)
// prescaler: minimum of 65536 and clkdiv, compare with clkdiv to find out if selected prescale is possible
// fcnt: the actual counting frequency based on the settings and limitations
class TimerArrayControl : TIM_OC_DelayElapsed_CallbackChain{
public:
    TimerArrayControl(TIM_HandleTypeDef *const htim, const uint32_t fclk=F_CPU, const uint32_t clkdiv=F_CPU/10000, const uint8_t bits=16);

    void begin(); // start interrupt generation for the listeners
    void stop(); // halt the hardware timer, stop interrupt generation
    void attachTimer(Timer* timer); // add a timer to the array, when it fires, the callback function is called
    void detachTimer(Timer* timer); // remove a timer from the array, stopping the callback event
    void changeTimerDelay(Timer* timer, uint32_t delay); // change the delay of the timer, fire if necessary (ruining synchrony)
    void attachTimerInSync(Timer* timer, Timer* reference); // add timer to the array, like it was attached the same time as the reference timer
    void manualFire(Timer* timer);

    void disableInterrupt();
    void enableInterrupt();

    void sleep(uint32_t ticks) const; // waits for the given amount of ticks to pass

    uint32_t remainingTicks(Timer* timer) const;
    uint32_t elapsedTicks(Timer* timer) const;
    float actualTickFrequency() const;
    bool isRunning() const;

    static const uint8_t prescaler_bits = 16;
    static const auto max_prescale = (1 << prescaler_bits);

    const uint32_t fclk;
    const uint32_t clkdiv;
    const uint32_t prescaler = clkdiv > max_prescale ? max_prescale : clkdiv;

protected:
    struct TimerFeed{
        Timer root;
        TIM_HandleTypeDef *const htim;
        const uint8_t bits;
        const uint32_t max_count = (1 << bits) - 1;
        uint32_t cnt; // current value of timer counter (saved to freeze while calculating)

        TimerFeed(TIM_HandleTypeDef *const htim, const uint8_t bits);
        Timer* findTimerInsertionLink(Timer* it, Timer* timer);
        void insertTimer(Timer* it, Timer* timer);
        void insertTimer(Timer* timer);
        void removeTimer(Timer* timer);
        void updateTimerTarget(Timer* timer, uint32_t target);

        // check if target comes sooner than reference if we are at cnt
        bool isSooner(uint32_t target, uint32_t reference);
        
        // calculate the next target while staying in sync with the previous one and the current time
        uint32_t calculateNextFireInSync(uint32_t target, uint32_t delay) const;

        void updateTime();
        void updateTickTime();
    };

    void tick();
    void registerAttachedTimer(Timer* timer);
    void registerDetachedTimer(Timer* timer);
    void registerDelayChange(Timer* timer, uint32_t delay);
    void registerAttachedTimerInSync(Timer* timer, Timer* reference);
    void registerManualFire(Timer* timer);

    void chainedCallback(TIM_HandleTypeDef*);

    TimerFeed timerFeed;
    volatile bool isTickOngoing;
};





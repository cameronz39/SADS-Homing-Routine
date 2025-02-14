
#include <cstdint>
// #define F_CPU  84000000
#define F_CPU 180000000
// Represents a timer, handled by a TimerArrayControl object.
// Attach it to a controller to receive callbacks.
//
// delay: ticks of timer array controller until firing
// periodic: does the timer restart immedietely when fires
// f: static function called when timer is firing
class Timer{
public:
    using callback_function = void(*)();
    Timer(const callback_function f);
    Timer(uint32_t delay, bool periodic, const callback_function f);
    
    bool isRunning() const;
    bool isPeriodic() const;
    uint32_t delay() const;

    void periodic(bool val);
    void delay(uint32_t val);

    // Changing the timers delay will not affect the current firing event, only the next one.
    // To restart the timer with the new delay, detach and attach it.
    // To change the timer's delay without restart, use the changeTimerDelay method
    // in the TimerArrayControl.

protected:
    uint32_t _delay; // required delay of timer (in ticks)
    bool _periodic; // should the timer be immedietely restarted after firing
    uint32_t target; // counter value that the timer fires at next
    void *const f; // WARNING: unsafe if you force the call of a certain fire method instead of letting the inheritance decide
    bool running;
    Timer* next;

    virtual void fire();

    friend class TimerArrayControl;
};

// Represents a Timer with context.
//
// delay: ticks of timer array controller until firing
// isPeriodic: does the timer restart immedietely when fires
// ctx: context pointer to an object, will be passed to the callback function
// ctxf: static function called when timer is firing, takes a Context* pointer argument
template<typename Context>
class ContextTimer : public Timer{
public:
    using dynamic_callback_function = void(*)(Context*);
    ContextTimer(Context* ctx, const dynamic_callback_function ctxf) : Timer((callback_function)ctxf), ctx(ctx) {}
    ContextTimer(uint32_t delay, bool isPeriodic, Context* ctx, const dynamic_callback_function ctxf) : Timer(delay, isPeriodic, (callback_function)ctxf), ctx(ctx) {}
protected:
    Context* ctx;

    virtual void fire(){
        ((dynamic_callback_function)f)(ctx);
    }
};

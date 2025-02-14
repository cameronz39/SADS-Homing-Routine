
#include "TimerArrayInc/Timer.hpp"

// -----                      -----
// ----- Timer implementation -----
// -----                      -----

Timer::Timer(const callback_function f)
    : _delay(10), _periodic(false), f((void*)f), running(false), next(nullptr)
{}

Timer::Timer(uint32_t delay, bool isPeriodic, const callback_function f)
    : _delay(delay), _periodic(isPeriodic), f((void*)f), running(false), next(nullptr)
{}

bool Timer::isRunning() const {
    return running;
}

bool Timer::isPeriodic() const {
    return _periodic;
}

uint32_t Timer::delay() const {
    return _delay;
}

void Timer::periodic(bool val){
    if (running) return; // can't change parameters directly if running
    _periodic = val;
}

void Timer::delay(uint32_t val){
    if (running) return; // can't change parameters directly if running
    _delay = val;
}

void Timer::fire(){
    ((callback_function)f)();
}

#include "TimerArrayInc/TimerArrayControl.hpp"

// capture update events and fire the timer array's callback chain
// a single call to tick would suffice in case of one timer array,
// but this way multiple callback handlers for the same interrupt
// routine can exist independently, without requiring rewriting
// the function for the current setup at all times
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim){
    TIM_OC_DelayElapsed_CallbackChain::fire(htim);
}

#define TARGET_CC_CHANNEL (TIM_CHANNEL_1)
#define __HAL_IS_TIMER_ENABLED(htim) (htim->Instance->CR1 & TIM_CR1_CEN)
#define __HAL_GENERATE_INTERRUPT(htim, EGR_FLAG) (htim->Instance->EGR |= TIM_EGR_CC1G)
#define COUNTER_MODULO(x) (timerFeed.max_count & ((uint32_t)(x)))
#define DISABLE_INTERRUPT() (__HAL_TIM_DISABLE_IT(timerFeed.htim, TIM_IT_CC1))
#define ENABLE_INTERRUPT() (__HAL_TIM_ENABLE_IT(timerFeed.htim, TIM_IT_CC1))
#define SET_TARGET(val) (__HAL_TIM_SET_COMPARE(htim, TARGET_CC_CHANNEL, val))
#define GET_TARGET(val) (__HAL_TIM_GET_COMPARE(htim, TARGET_CC_CHANNEL))
#define CALLBACK_JITTER 1000

// -----                            -----
// ----- TimerString implementation -----
// -----                            -----

TimerArrayControl::TimerFeed::TimerFeed(TIM_HandleTypeDef *const htim, const uint8_t bits) :
    root(nullptr),
    htim(htim),
    bits(bits)
{}

Timer* TimerArrayControl::TimerFeed::findTimerInsertionLink(Timer* it, Timer* timer){
    while(it->next && isSooner(it->next->target, timer->target)){
        // while there are more timers and the next timer's target is sooner than the new one's
        // advance it on the timer string
        it = it->next;
    }
    return it;
}

// insert timer after the iterator
void TimerArrayControl::TimerFeed::insertTimer(Timer* it, Timer* timer){
    
    // insert the new timer between it and next of it
    timer->running = true;
    timer->next = it->next;
    it->next = timer;

    // if the first timer changed, adjust interrupt target
    if (root.next == timer) SET_TARGET(timer->target);
}

// insert timer based on target
void TimerArrayControl::TimerFeed::insertTimer(Timer* timer){
    insertTimer(findTimerInsertionLink(&root, timer), timer);
}

// remove timer from feed
void TimerArrayControl::TimerFeed::removeTimer(Timer* timer){
    Timer* it = &root;
    while(it->next && it->next != timer) it = it->next;

    if (it->next == timer){
        it->next = timer->next;
        timer->next = nullptr;
        timer->running = false;
    }

    // if the removed timer was the first in the feed, update interrupt target
    if (&root == it && root.next) SET_TARGET(root.next->target);
}

// remove and insert timer in one operation, according to it's target
void TimerArrayControl::TimerFeed::updateTimerTarget(Timer* timer, uint32_t target){
    
    // find fitting place for timer in string
    Timer* ins = &root;
    Timer* rem = ins;

    // search attach position
    while(ins->next && isSooner(ins->next->target, target)){
        // while there are more timers and the next timer's target is sooner than the modified one's
        // advance |ins| on the timer string
        ins = ins->next;

        // if the next timer is not our's to remove, advance |rem| on the string
        if (rem->next != timer) rem = ins;
    }

    // search where the timer was, to detach it from that position
    while(rem->next && rem->next != timer) rem = rem->next;

    // only move timer if the predecessor changed and it is not itself
    if (ins != rem && ins != timer){
        // remove our timer from the string
        rem->next = timer->next;

        // insert our timer between |ins| and next of |ins|
        timer->next = ins->next;
        ins->next = timer;
    }

    // update the timer's target
    timer->target = target;
    
    // If the interrupt was set to a timer that has changed, set new target.
    // If ins is first timer, the timer was put to first place.
    // If rem is first timer, the timer was moved from first place.
    // If both, the first timers target was probably changed.
    // In all cases new target is needed.
    if (&root == ins || &root == rem) {
        SET_TARGET(root.next->target);
    }
}

bool TimerArrayControl::TimerFeed::isSooner(uint32_t target, uint32_t reference){
    return (max_count & ((uint32_t)(target - cnt))) < (max_count & ((uint32_t)(reference - cnt)));
}

void TimerArrayControl::TimerFeed::updateTime(){
    cnt = __HAL_TIM_GET_COUNTER(htim);
}

void TimerArrayControl::TimerFeed::updateTickTime(){
    cnt = GET_TARGET();
    uint32_t tim_cnt = __HAL_TIM_GET_COUNTER(htim);
    
    if ((max_count & ((uint32_t)(tim_cnt - cnt))) >= CALLBACK_JITTER){
        // if CNT passed CCR more than the acceptable jitter, use the CNT value
        cnt = tim_cnt;
    }
}

uint32_t TimerArrayControl::TimerFeed::calculateNextFireInSync(uint32_t target, uint32_t delay) const{
    uint32_t diff = (max_count & ((uint32_t)(cnt - target)));
    uint32_t subt = diff - (diff/delay)*delay;
    uint32_t incr = delay - subt;
    uint32_t tnext = (max_count & ((uint32_t)(cnt + incr)));
    return tnext;
}

// -----                                  -----
// ----- TimerArrayControl implementation -----
// -----                                  -----


TimerArrayControl::TimerArrayControl(TIM_HandleTypeDef *const htim, const uint32_t fclk, const uint32_t clkdiv, const uint8_t bits) : 
    fclk(fclk),
    clkdiv(clkdiv),
    timerFeed(htim, bits),
    isTickOngoing(false)
{}

void TimerArrayControl::begin(){

    // stop timer if it was running
    HAL_TIM_OC_Stop_IT(timerFeed.htim, TARGET_CC_CHANNEL);

    timerFeed.htim->Init.CounterMode = TIM_COUNTERMODE_UP; // all STM32 counters support it
    #ifdef TIM_AUTORELOAD_PRELOAD_DISABLE  // not used by STM32F4
    timerFeed.htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // by disabling, write to ARR shadow regs happens immedietely
    #endif
    timerFeed.htim->Init.Period = timerFeed.max_count; // set max period for maximum amount of possible delay
    timerFeed.htim->Init.Prescaler = prescaler - 1; // prescaler divides clock by Prescaler+1

    TIM_OC_InitTypeDef oc_init;
    oc_init.OCMode = TIM_OCMODE_TIMING;

    HAL_TIM_OC_Init(timerFeed.htim);
    HAL_TIM_OC_ConfigChannel(timerFeed.htim, &oc_init, TARGET_CC_CHANNEL);
    uint32_t cnt = __HAL_TIM_GET_COUNTER(timerFeed.htim);
    uint32_t target = timerFeed.root.next == nullptr ? (timerFeed.max_count & (cnt-1)) : timerFeed.root.next->target;
    __HAL_TIM_SET_COMPARE(timerFeed.htim, TARGET_CC_CHANNEL, target); // if no timers to fire yet, set max delay between unneeded interrupts
    HAL_TIM_OC_Start_IT(timerFeed.htim, TARGET_CC_CHANNEL);
}

void TimerArrayControl::stop(){
    // stop timer if it was running
    HAL_TIM_OC_Stop_IT(timerFeed.htim, TARGET_CC_CHANNEL);
}

/*
 * Subscribed to interrupts generated by timerFeed.htim.
 * Only call tick if really timerFeed.htim was the source.
 */
void TimerArrayControl::chainedCallback(TIM_HandleTypeDef* htim){
    if (timerFeed.htim == htim) tick();
}

/**
 * This method can only be called from interupts.
 * */
void TimerArrayControl::tick(){

    isTickOngoing = true;

    timerFeed.updateTickTime();

    // handle timeout
    while (timerFeed.root.next && COUNTER_MODULO(timerFeed.cnt - timerFeed.root.next->target) < CALLBACK_JITTER){
        Timer* timer = timerFeed.root.next;

        // set up the next interrupt generation
        if (timer->_periodic){

            // set new target for timer
            uint32_t target = COUNTER_MODULO(timer->target + timer->_delay);

            // find fitting place for timer in string
            timerFeed.updateTimerTarget(timer, target);

        } else {
            // if timer is not periodic, it is done, we can detach it
            Timer* timer = timerFeed.root.next;
            timer->running = false;
            timerFeed.root.next = timer->next;
            timer->next = nullptr;
        }

        // set the new target
        uint32_t target = timerFeed.root.next == nullptr ? COUNTER_MODULO(timerFeed.cnt - 1) : timerFeed.root.next->target;
        __HAL_TIM_SET_COMPARE(timerFeed.htim, TARGET_CC_CHANNEL, target);

        // fire callback
        timer->fire();

        timerFeed.updateTickTime();
    }

    isTickOngoing = false;
}

void TimerArrayControl::registerAttachedTimer(Timer* timer){

    // if timer is already attached to a controller, do nothing
    if (timer->running) return;

    // get current time in ticks and add the requested delay to find the target time
    timer->target = COUNTER_MODULO(timer->_delay + timerFeed.cnt);

    // insert timer based on the target time
    timerFeed.insertTimer(timer);
}

void TimerArrayControl::registerDetachedTimer(Timer* timer){
    if (!timer->running) return;
    timerFeed.removeTimer(timer);
}

void TimerArrayControl::registerDelayChange(Timer* timer, uint32_t delay){

    if (!timer->running) {
        timer->_delay = delay;
        return;
    }

    uint32_t target;
    
    if (elapsedTicks(timer) > delay){
        // according to the new delay the timer should have been fired, fire it immedietely
        timer->fire(); // firing will ruin timer synchrony
        target = COUNTER_MODULO(timerFeed.cnt + delay); // new target is counted from now
    } else {
        // the timer will be fired in the future
        // since the target will certainly increase, delay - timer->delay is positive,
        // no special handling is needed
        target = COUNTER_MODULO(timer->target + delay - timer->_delay);
    }

    timer->_delay = delay;

    // update the position of timer in the feed
    timerFeed.updateTimerTarget(timer, target);
}

void TimerArrayControl::registerAttachedTimerInSync(Timer* timer, Timer* reference){

    // won't reattach timer (if attached to this controller, it would be possible)
    if (timer->running) return;

    // TODO: negative calculation might be also needed, for more complicated cases
    // put start time in timer's target, find the next firing time with timer's delay
    timer->target = COUNTER_MODULO(reference->target - reference->_delay);
    timer->target = timerFeed.calculateNextFireInSync(timer->target, timer->_delay);

    // find fitting place for timer in string
    timerFeed.insertTimer(timer);
}

void TimerArrayControl::registerManualFire(Timer* timer){

    // fire timer manually, even if it is not running
    // firing a periodic timer will start it
    timer->fire();
    
    // if timer was running detach it, if it was periodic it will be immedietely reattached
    if (timer->running) timerFeed.removeTimer(timer);

    // if timer is periodic, restart it at this moment
    if (timer->_periodic){
        timer->running = false;
        registerAttachedTimer(timer);
    }
}


//
// Public members
//


void TimerArrayControl::attachTimer(Timer* timer){

    if (!isTickOngoing){
        // timer is running and this is not on interrupt thread, use interrupt safe attach
        
        DISABLE_INTERRUPT();
        timerFeed.updateTime(); // fetch counter
        registerAttachedTimer(timer);
        ENABLE_INTERRUPT();

    } else {
        // timer is not running or this is an interrupt handler, attach is safe
        registerAttachedTimer(timer);
    }

}

void TimerArrayControl::detachTimer(Timer* timer){
    

    if (!isTickOngoing){
        // timer is running and this is not on interrupt thread, use interrupt safe attach
        
        DISABLE_INTERRUPT();
        registerDetachedTimer(timer);
        ENABLE_INTERRUPT();

    } else {
        // timer is not running or this is an interrupt handler, attach is safe
        registerDetachedTimer(timer);
    }

}

void TimerArrayControl::changeTimerDelay(Timer* timer, uint32_t delay){

    if (!isTickOngoing){
        // timer is running and this is not on interrupt thread, use interrupt safe attach
        
        DISABLE_INTERRUPT();
        timerFeed.updateTime(); // fetch counter
        registerDelayChange(timer, delay);
        ENABLE_INTERRUPT();

    } else {
        // timer is not running or this is an interrupt handler, attach is safe
        registerDelayChange(timer, delay);
    }
}

void TimerArrayControl::attachTimerInSync(Timer* timer, Timer* reference){
    
    if (!isTickOngoing){
        // timer is running and this is not on interrupt thread, use interrupt safe attach
        
        DISABLE_INTERRUPT();
        timerFeed.updateTime(); // fetch counter
        registerAttachedTimerInSync(timer, reference);
        ENABLE_INTERRUPT();

    } else {
        // timer is not running or this is an interrupt handler, attach is safe
        registerAttachedTimerInSync(timer, reference);
    }
}

void TimerArrayControl::manualFire(Timer* timer){
    
    if (!isTickOngoing){
        // timer is running and this is not on interrupt thread, use interrupt safe attach
        
        DISABLE_INTERRUPT();
        timerFeed.updateTime(); // fetch counter
        registerManualFire(timer);
        ENABLE_INTERRUPT();

    } else {
        // timer is not running or this is an interrupt handler, attach is safe
        registerManualFire(timer);
    }
}

void TimerArrayControl::disableInterrupt(){
    DISABLE_INTERRUPT();
}

void TimerArrayControl::enableInterrupt(){
    ENABLE_INTERRUPT();
}


bool TimerArrayControl::isRunning() const{
    return __HAL_IS_TIMER_ENABLED(timerFeed.htim);
}


void TimerArrayControl::sleep(uint32_t ticks) const{
    if (!isRunning()) return;

    uint32_t prev = __HAL_TIM_GET_COUNTER(timerFeed.htim);
    uint32_t diff;
    while(1){
        diff = COUNTER_MODULO(__HAL_TIM_GET_COUNTER(timerFeed.htim) - prev);

        // if the remaining ticks are not more than the time passed between checks, return
        // simply: more time passed than ticks were remaining
        if (diff >= ticks) return;
        
        ticks -= diff;
        prev = COUNTER_MODULO(diff + prev);
    }
}


uint32_t TimerArrayControl::remainingTicks(Timer* timer) const {
    if (!timer->running) return 0;
    const uint32_t cnt = __HAL_TIM_GET_COUNTER(timerFeed.htim);
    return COUNTER_MODULO(timer->target - cnt);
}

uint32_t TimerArrayControl::elapsedTicks(Timer* timer) const {
    if (!timer->running) return 0;
    return timer->_delay - remainingTicks(timer);
}

float TimerArrayControl::actualTickFrequency() const {
    return ((float)fclk)/prescaler;
}


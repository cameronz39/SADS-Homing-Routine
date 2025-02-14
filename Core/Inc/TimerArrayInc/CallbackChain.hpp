#pragma once

template<typename ChainID, typename ... Args>
class CallbackChain{
public:
    CallbackChain();
    virtual ~CallbackChain();
    static void fire(Args... args);
protected:
    static CallbackChain<ChainID, Args...>* last;
    CallbackChain<ChainID, Args...>* next;
    virtual void chainedCallback(Args...) = 0;
};

// ----- Implementation -----

template<typename ChainID, typename ... Args>
CallbackChain<ChainID, Args...>* CallbackChain<ChainID, Args...>::last = nullptr;

template<typename ChainID, typename ... Args>
CallbackChain<ChainID, Args...>::CallbackChain() : next(last){
    last = this;
}

template<typename ChainID, typename ... Args>
CallbackChain<ChainID, Args...>::~CallbackChain() {
    CallbackChain<ChainID, Args...>** hnext = &last;
    while(*hnext != this){
        hnext = &((*hnext)->next);
    }
    *hnext = this->next;
}

template<typename ChainID, typename ... Args>
void CallbackChain<ChainID, Args...>::fire(Args... args){
    CallbackChain<ChainID, Args...>* obj = last;
    while(obj){
        obj->chainedCallback(args...);
        obj = obj->next;
    }
}

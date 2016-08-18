
#pragma once

#include <uavcan_stm32/build_config.hpp>

#if UAVCAN_STM32_BAREMETAL
#else
# error "Unknown OS"
#endif

#include <uavcan/uavcan.hpp>

namespace uavcan_stm32
{

class CanDriver;

class BusEvent
{
    volatile bool ready;

public:
    BusEvent(CanDriver& can_driver)
     : ready(false)
    {
        (void)can_driver;
    }

    bool wait(uavcan::MonotonicDuration duration)
    {
        (void)duration;
        bool lready = ready;
        return __atomic_exchange_n (&lready, false, __ATOMIC_SEQ_CST);
    }

    void signal()
    {
        __atomic_store_n (&ready, true, __ATOMIC_SEQ_CST);
    }

    void signalFromInterrupt()
    {
        __atomic_store_n (&ready, true, __ATOMIC_SEQ_CST);
    }
};

class Mutex
{
public:
    void lock() { }
    void unlock() { }
};


class MutexLocker
{
    Mutex& mutex_;

public:
    MutexLocker(Mutex& mutex)
        : mutex_(mutex)
    {
        mutex_.lock();
    }
    ~MutexLocker()
    {
        mutex_.unlock();
    }
};

}

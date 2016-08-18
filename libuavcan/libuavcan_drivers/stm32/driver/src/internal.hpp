
#pragma once

#include <uavcan_stm32/build_config.hpp>

#if UAVCAN_STM32_BAREMETAL
//# include <chip.h>
#include <stm32f4xx.h>
#else
# error "Unknown OS"
#endif

/**
 * Debug output
 */
#ifndef UAVCAN_STM32_LOG
// lowsyslog() crashes the system in this context
// # if UAVCAN_STM32_NUTTX && CONFIG_ARCH_LOWPUTC
# if 0
#  define UAVCAN_STM32_LOG(fmt, ...)  lowsyslog("uavcan_stm32: " fmt "\n", ##__VA_ARGS__)
# else
#  define UAVCAN_STM32_LOG(...)       ((void)0)
# endif
#endif

/**
 * IRQ handler macros
 */

# define UAVCAN_STM32_IRQ_HANDLER(id)  void id(void)
# define UAVCAN_STM32_IRQ_PROLOGUE()
# define UAVCAN_STM32_IRQ_EPILOGUE()

#if UAVCAN_STM32_BAREMETAL
/**
 * Priority mask for timer and CAN interrupts.
 */
# ifndef UAVCAN_STM32_IRQ_PRIORITY_MASK
#  define UAVCAN_STM32_IRQ_PRIORITY_MASK  0
# endif
#endif


/**
 * Glue macros
 */
#define UAVCAN_STM32_GLUE2_(A, B)       A##B
#define UAVCAN_STM32_GLUE2(A, B)        UAVCAN_STM32_GLUE2_(A, B)

#define UAVCAN_STM32_GLUE3_(A, B, C)    A##B##C
#define UAVCAN_STM32_GLUE3(A, B, C)     UAVCAN_STM32_GLUE3_(A, B, C)

namespace uavcan_stm32
{

struct CriticalSectionLocker
{

    CriticalSectionLocker()
    {
      __disable_irq();
    }

    ~CriticalSectionLocker()
    {
      __enable_irq();
    }
};


namespace clock
{
uavcan::uint64_t getUtcUSecFromCanInterrupt();
}
}

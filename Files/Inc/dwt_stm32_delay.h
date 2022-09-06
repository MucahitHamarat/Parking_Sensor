
#ifndef DWT_STM32_DELAY_H
#define DWT_STM32_DELAY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

uint32_t DWT_Delay_Init(void);

__STATIC_INLINE void DWT_Delay_us(volatile uint32_t microseconds)
{

uint32_t clk_cycle_start = DWT->CYCCNT ;
microseconds *=(HAL_RCC_GetHCLKFreq() / 1000000);
/*Delay till end*/
while ((DWT->CYCCNT - clk_cycle_start) < microseconds);

}


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

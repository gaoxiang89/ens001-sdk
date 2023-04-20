#ifndef DELAY_SYSTICK_H_
#define DELAY_SYSTICK_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void delay_systicl_init(void);

void delay_ms_systick(uint32_t ms);

#ifdef __cplusplus
}
#endif
#endif /* DELAY_SYSTICK_H_ */

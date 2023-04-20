#include "delay_systick.h"
#include "ens001.h"

static volatile uint32_t ms_ticks;

void SysTick_Handler(void)
{
	if (ms_ticks != 0) {
		ms_ticks--;
	}
}

void delay_systicl_init(void)
{
	SysTick_Config(32000);

	NVIC_EnableIRQ(SysTick_IRQn);
}

void delay_ms_systick(uint32_t ms)
{
	ms_ticks = ms;
	while (ms_ticks);
}

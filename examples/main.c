#include <stdio.h>
#include "uart_stdout.h"
#include "ens001.h"
#include "delay_systick.h"


int main()
{
	uint32_t count = 0;

	// APB clock
	CMSDK_SYSCON->APB_CLKEN = 0x1001 | 0x4000;
	
	delay_systicl_init();

	UartStdOutInit();

	printf("\r\n******hello world, ens001 board*******\r\n");

	while(1) {
		delay_ms_systick(1000);
		printf("count %u\r\n", count++);
	}
}

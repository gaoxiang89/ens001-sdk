#include <stdio.h>
#include "uart_stdout.h"
#include "ens001.h"


int main()
{
	// APB clock
	CMSDK_SYSCON->APB_CLKEN = 0x1001 | 0x4000;

	UartStdOutInit();

	printf("\r\n******hello world, ens001 board*******\r\n");

	while(1) {

	}
}

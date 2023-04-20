#include <stdio.h>
#include "uart_stdout.h"
#include "ens001.h"
#include "delay_systick.h"
#include "ens001_gpio.h"

#define LED_PIN0 8
#define LED_PIN1 9
#define LED_PIN2 10

int main()
{
	uint32_t count = 0;

	// APB clock
	CMSDK_SYSCON->APB_CLKEN = 0x1001 | 0x4000;

	delay_systicl_init();

	UartStdOutInit();

	printf("\r\n******hello world, ens001 board*******\r\n");

	pin_cfg_t cfg;
	cfg.mode = GPIO_MODE_OUTPUT;
	cfg.pullup = GPIO_PULL_UP;
	gpio_pin_cfg(LED_PIN0, &cfg);
	gpio_pin_cfg(LED_PIN1, &cfg);

	while (1) {
		gpio_pin_set(LED_PIN0);
		gpio_pin_clear(LED_PIN1);
		printf("led0: %d, led1: %d\r\n", gpio_pin_get(LED_PIN0), gpio_pin_get(LED_PIN1));
		delay_ms_systick(1000);

		gpio_pin_clear(LED_PIN0);
		gpio_pin_set(LED_PIN1);
		printf("led0: %d, led1: %d\r\n", gpio_pin_get(LED_PIN0), gpio_pin_get(LED_PIN1));
		delay_ms_systick(1000);
	}
}

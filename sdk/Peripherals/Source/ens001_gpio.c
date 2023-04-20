#include "ens001_gpio.h"

static void pin_cfg_pullup(int pin, int pullup)
{
	switch (pullup) {
	case GPIO_PULL_UP:
		ENS001_GPIO->PU |= BIT(pin);
		break;

	case GPIO_PULL_DOWN:
		ENS001_GPIO->PU |= BIT(pin);
		break;

	case GPIO_OPEN_DRAIN:
		ENS001_GPIO->PU |= BIT(pin);
		break;

	default:
		break;
	}
}

static void pin_cfg_mode(int pin, int mode)
{
	switch (mode) {
	case GPIO_MODE_OUTPUT:
		ENS001_GPIO->OE |= BIT(pin);
		break;

	case GPIO_MODE_INPUT:
		ENS001_GPIO->IE |= BIT(pin);
		break;

	case GPIO_MODE_ALTERNATE:
		ENS001_GPIO->PU |= BIT(pin);
		break;

	case GPIO_MODE_ANALOG:
		ENS001_GPIO->PU |= BIT(pin);
		break;

	default:
		break;
	}
}

void gpio_pin_cfg(int pin, pin_cfg_t *cfg)
{
	pin_cfg_pullup(pin, cfg->pullup);
	pin_cfg_mode(pin, cfg->mode);
}

void gpio_pin_set(int pin)
{
	/* 读取值是输出Pin脚电平， 所以直接赋值即可，不能用 |= */
	ENS001_GPIO->BITSET = BIT(pin);
}

void gpio_pin_clear(int pin)
{
	/* 读取值是输出Pin脚电平， 所以直接赋值即可，不能用 |= */
	ENS001_GPIO->BITCLR = BIT(pin);
}


int gpio_pin_get(int pin)
{
	int value = 0;
	int input_mode = 0;

	input_mode = (ENS001_GPIO->IE &= BIT(pin)) >> pin;

	if (input_mode) {
		value = (ENS001_GPIO->DATAIN & BIT(pin)) >> pin;
	} else {
		value = (ENS001_GPIO->DATAOUT & BIT(pin)) >> pin;
	}

	return value;
}

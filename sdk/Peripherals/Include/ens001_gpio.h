#ifndef ENS001_GPIO_H_
#define ENS001_GPIO_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <ens001.h>

#define GPIO_MODE_OUTPUT    1
#define GPIO_MODE_INPUT	    2
#define GPIO_MODE_ALTERNATE 3
#define GPIO_MODE_ANALOG    4

#define GPIO_PULL_UP	1
#define GPIO_PULL_DOWN	2
#define GPIO_OPEN_DRAIN 3

typedef struct {
	uint8_t mode;
	uint8_t pullup;
} pin_cfg_t;

void gpio_pin_cfg(int pin, pin_cfg_t *cfg);

void gpio_pin_set(int pin);

void gpio_pin_clear(int pin);

int gpio_pin_get(int pin);

#ifdef __cplusplus
}
#endif
#endif /* ENS001_GPIO_H_ */

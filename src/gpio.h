#ifndef GPIO_H
#define GPIO_H

#include <bcm2835.h>
#include <stdint.h>

int gpioInit(void);
void gpioSetDir(uint8_t pin, uint8_t isOutput);
void gpioWrite(uint8_t pin, uint8_t high);
uint8_t gpioRead(uint8_t pin);

#endif
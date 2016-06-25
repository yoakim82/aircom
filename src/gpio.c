// Build command:  gcc IOToggle_Clib2.c -o IOToggle_Clib2 -I/usr/local/include -L/usr/local/lib -lbcm2835
#include <bcm2835.h>

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

int gpioInit(void)
{
	// init gpio
    if (!bcm2835_init())
        return 1;
	else
		return 0;
}

void gpioSetDir(uint8_t pin, uint8_t isOutput)
{
	bcm2835_gpio_fsel(pin, (isOutput == 1 ? BCM2835_GPIO_FSEL_OUTP : BCM2835_GPIO_FSEL_INPT));
}

void gpioWrite(uint8_t pin, uint8_t high)
{
	bcm2835_gpio_write(pin, (high == 1 ? HIGH : LOW));
}

uint8_t gpioRead(uint8_t pin)
{
	return (bcm2835_gpio_lev(pin) == HIGH ? 1 : 0);
}

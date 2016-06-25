/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */
 
 // Build command:  gcc IOToggle_Clib2.c -o IOToggle_Clib2 -I/usr/local/include -L/usr/local/lib -lbcm2835
#include <bcm2835.h>

// Blinks on RPi pin GPIO 18
#define EN_N  RPI_V2_GPIO_P1_12
#define GPIO0 RPI_V2_GPIO_P1_22
#define GPIO1 RPI_V2_GPIO_P1_18
#define GPIO2 RPI_V2_GPIO_P1_16
#define IRQ_N RPI_V2_GPIO_P1_11
#define RXANT RPI_V2_GPIO_P1_13
#define TXANT RPI_V2_GPIO_P1_15



#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

int setupSPI(int fd);

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *device = "/dev/spidev0.0";
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 500000;
static uint16_t delay;

static uint8_t[] transfer(int fd, uint8_t tx[])
{
	int ret;
	
	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

	for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
		if (!(ret % 6))
			puts("");
		printf("%.2X ", rx[ret]);
	}
	puts("");
	
	return rx;
}

int main(int argc, char *argv[])
{
	int ret = 0;
	int fd;
		
	uint8_t tx[] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0x40, 0x00, 0x00, 0x00, 0x00, 0x95,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xDE, 0xAD, 0xBE, 0xEF, 0xBA, 0xAD,
	0xF0, 0x0D,
	};

	uint8_t *rx; 
	
	int i = 0;

	// init gpio
    if (!bcm2835_init())
        return 1;

    // Set gpio pin directions
	bcm2835_gpio_fsel(IRQ_N, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_fsel(GPIO0, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_fsel(GPIO1, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_fsel(GPIO2, BCM2835_GPIO_FSEL_INPT);
	
    bcm2835_gpio_fsel(RXANT, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(TXANT, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(EN_N, BCM2835_GPIO_FSEL_OUTP);
	
	// set gpio initial output values
	bcm2835_gpio_write(RXANT, HIGH);
	bcm2835_gpio_write(TXANT, LOW);
	bcm2835_gpio_write(EN_N, LOW);
	
	trxSPI(tx, rx);
	
	return ret;
}


void trxSPI(uint8_t tx[], uint8_t rx[])
{
	
	// open SPI device
	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");
	
	// setup SPI device parameters
	setupSPI(fd);
	
	rx = transfer(fd, tx);

	close(fd);

}

int setupSPI(int fd)
{
	// set spi mode
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	//bits per word
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	// max speed hz
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	//printf("spi mode: %d\n", mode);
	//printf("bits per word: %d\n", bits);
	//printf("max speed: %d Hz (%d kHz)\n", speed, speed/1000);
	
	return ret;
}
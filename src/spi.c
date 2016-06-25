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
 
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "spi.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))


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

static int transfer(int fd, uint8_t tx[], uint8_t rx[], int size)
{
	int ret;
	
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = (unsigned int) size,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");
	
	return ret;
}

void trxSpiBlock(uint8_t tx[], uint8_t rx[], int size)
{
	int fd;
	// open SPI device
	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");
	
	// setup SPI device parameters
	setupSPI(fd);
	
	transfer(fd, tx, rx, size);

	close(fd);

}

uint8_t trxSpi(uint8_t txbyte)
{
	int fd;
	uint8_t tx[] = {txbyte};
	uint8_t rx[] = {0};
	
	// open SPI device
	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");
	
	// setup SPI device parameters
	setupSPI(fd);
	
	transfer(fd, tx, rx, 1);

	close(fd);
	
	return rx[0];

}

int setupSPI(int fd)
{
	int ret;
	
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

// int main(void)
// {
	// uint8_t rx, tx = 0xa5;
	// int i;
	// uint8_t rxb[] = {0,0,0,};
	// uint8_t txb[] = {0xa5, 0x5a, 0x11,};
	
	// rx = trxSpi(tx);
	
	// printf("sent %d, received %d\n", tx, rx);
	
	// trxSpiBlock(txb, rxb, ARRAY_SIZE(txb));
	
	// for (i = 0; i < ARRAY_SIZE(txb); i++)
	// {
		// printf("block sent %x, received %x\n", txb[i], rxb[i]);
	// }
// }
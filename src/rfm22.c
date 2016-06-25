#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include "rfm22.h"
#include "spi.h"
#include "gpio.h"

void writeRfm(uint8_t address, uint8_t data)
{
	//write any data byte to any single address
	//sets the MSB of the address byte (WRITE mode)

	uint8_t tx[] = {address | 0x80, data,};
	uint8_t rx[] = {0, 0,};
	int size = 2;

	trxSpiBlock(tx, rx, size);
}

uint8_t readRfm(uint8_t address)
{
	//write any data byte to any single address
	//sets the MSB of the address byte (WRITE mode)
	
	uint8_t tx[] = {address & 0x7F, 0x55,};
	uint8_t rx[] = {0, 0,};
	int size = 2;

	trxSpiBlock(tx, rx, size);
	
	return rx[1];
}

void rx_reset(void)
{
	//unsigned char i;
	
	//printf("r");	
	
	writeRfm(0x07, 0x01);	// to ready mode
	//printf("c");
	readRfm(0x03);
	readRfm(0x04);
	
	writeRfm(0x7E, 17);
	
	writeRfm(0x08, 0x03);
	writeRfm(0x08, 0x00);
	
	writeRfm(0x07, 5);
	
	writeRfm(0x05, 2);
}

int checkINT(void)
{
//int retval = 0;
/*
	if ((PIND & (1<<NIRQ)) == 0)
		printf("INT == 0\n");
	else
	{
		printf("INT == 1\n");
		retval = 1;
	}
	*/
	
	return gpioRead(IRQ_N);
}

void to_rx_mode(void)
{
	writeRfm(0x07, 0x01);	// to ready mode	
	gpioWrite(RXANT, 1);
	gpioWrite(TXANT, 0);
	
	//sbi(PORTD, RXANT);
	//cbi(PORTD, TXANT);

	rx_reset();
}

void to_tx_mode(uint8_t txData[])
{
	unsigned char i;
	
	writeRfm(0x07, 0x01);	// To ready mode
	//cbi(PORTD, RXANT);
	gpioWrite(RXANT, 0);

	//sbi(PORTD, TXANT);
	gpioWrite(TXANT, 1);
	
	usleep(50000);
	//delay_ms(50);
	
	writeRfm(0x08, 0x03);	// FIFO reset
	writeRfm(0x08, 0x00);	// Clear FIFO
	
	writeRfm(0x34, 64);	// preamble = 64nibble
	writeRfm(0x3E, 17);	// packet length = 17bytes
	for (i=0; i<17; i++)
	{
		writeRfm(0x7F, txData[i]);	// send payload to the FIFO
	}

	writeRfm(0x05, 0x04);	// enable packet sent interrupt
	i = readRfm(0x03);		// Read Interrupt status1 register
	i = readRfm(0x04);
	
	writeRfm(0x07, 9);	// Start TX
	
	while (gpioRead(IRQ_N) != 0)
		;
	//while ((PIND & (1<<NIRQ)) != 0)
	//	; 	// need to check interrupt here
	
	writeRfm(0x07, 0x01);	// to ready mode
	
	gpioWrite(RXANT, 0);
	//cbi(PORTD, RXANT);	// disable all interrupts
	
	gpioWrite(TXANT, 0);
	//cbi(PORTD, TXANT);
}



void init_RFM22(void)
{
	writeRfm(INTEN2, 0x00);	// Disable all interrupts
	writeRfm(OMFC1, 0x01);		// Set READY mode
	writeRfm(0x09, 0x7F);		// Cap = 12.5pF
	//writeRfm(0x0A, 0x05);		// Clk output is 2 MHz
	//writeRfm(0x0A, 0x06);		// Clk output is 1 MHz (default)		
	
	writeRfm(0x0B, 0xF4);		// GPIO0 is for RX data output
	writeRfm(0x0C, 0xEF);		// GPIO1 is TX/RX data CLK output
	writeRfm(0x0D, 0x00);		// GPIO2 for MCLK output
	writeRfm(0x0E, 0x00);		// GPIO port use default value
	
	writeRfm(0x0F, 0x70);		// NO ADC used
	writeRfm(0x10, 0x00);		// no ADC used
	writeRfm(0x12, 0x00);		// No temp sensor used
	writeRfm(0x13, 0x00);		// no temp sensor used
	
	writeRfm(0x70, 0x20);		// No manchester code, no data whiting, data rate < 30Kbps
	
	writeRfm(0x1C, 0x1D);		// IF filter bandwidth
	writeRfm(0x1D, 0x40);		// AFC Loop
	//writeRfm(0x1E, 0x0A);	// AFC timing
	
	writeRfm(0x20, 0xA1);		// clock recovery
	writeRfm(0x21, 0x20);		// clock recovery
	writeRfm(0x22, 0x4E);		// clock recovery
	writeRfm(0x23, 0xA5);		// clock recovery
	writeRfm(0x24, 0x00);		// clock recovery timing
	writeRfm(0x25, 0x0A);		// clock recovery timing
	
	//writeRfm(0x2A, 0x18);
	writeRfm(0x2C, 0x00);
	writeRfm(0x2D, 0x00);
	writeRfm(0x2E, 0x00);
	
	writeRfm(0x6E, 0x27);		// TX data rate 1
	writeRfm(0x6F, 0x52);		// TX data rate 0
	
	writeRfm(0x30, 0x8C);		// Data access control
	
	writeRfm(0x32, 0xFF);		// Header control
	
	writeRfm(0x33, 0x42);		// Header 3, 2, 1, 0 used for head length, fixed packet length, synchronize word length 3, 2,
	
	writeRfm(0x34, 64);		// 64 nibble = 32 byte preamble
	writeRfm(0x35, 0x20);		// 0x35 need to detect 20bit preamble
	writeRfm(0x36, 0x2D);		// synchronize word
	writeRfm(0x37, 0xD4);
	writeRfm(0x38, 0x00);
	writeRfm(0x39, 0x00);
	writeRfm(0x3A, 's');		// set tx header 3
	writeRfm(0x3B, 'o');		// set tx header 2
	writeRfm(0x3C, 'n');		// set tx header 1
	writeRfm(0x3D, 'g');		// set tx header 0
	writeRfm(0x3E, 17);		// set packet length to 17 bytes
	
	writeRfm(0x3F, 's');		// set rx header
	writeRfm(0x40, 'o');
	writeRfm(0x41, 'n');
	writeRfm(0x42, 'g');
	writeRfm(0x43, 0xFF);		// check all bits
	writeRfm(0x44, 0xFF);		// Check all bits
	writeRfm(0x45, 0xFF);		// check all bits
	writeRfm(0x46, 0xFF);		// Check all bits
	
	writeRfm(0x56, 0x01);
	
	writeRfm(0x6D, 0x07);		// Tx power to max
	
	writeRfm(0x79, 0x00);		// no frequency hopping
	writeRfm(0x7A, 0x00);		// no frequency hopping
	
	writeRfm(0x71, 0x22);		// GFSK, fd[8]=0, no invert for TX/RX data, FIFO mode, txclk-->gpio
	
	writeRfm(0x72, 0x48);		// Frequency deviation setting to 45K=72*625
	
	writeRfm(0x73, 0x00);		// No frequency offset
	writeRfm(0x74, 0x00);		// No frequency offset
	
	writeRfm(0x75, 0x53);		// frequency set to 434MHz
	writeRfm(0x76, 0x64);		// frequency set to 434MHz
	writeRfm(0x77, 0x00);		// frequency set to 434Mhz
	
	writeRfm(0x5A, 0x7F);
	writeRfm(0x59, 0x40);
	writeRfm(0x58, 0x80);
	
	writeRfm(0x6A, 0x0B);
	writeRfm(0x68, 0x04);
	writeRfm(0x1F, 0x03);
}

void initDiscretes(void)
{

	gpioInit();
	
	// inputs
	gpioSetDir(IRQ_N, 0);
	gpioSetDir(GPIO0, 0);
	gpioSetDir(GPIO1, 0);
	gpioSetDir(GPIO2, 0);
	
	// outputs
    gpioSetDir(RXANT, 1);
	gpioSetDir(TXANT, 1);
	gpioSetDir(EN_N,  1);

	// set gpio initial output values
	gpioWrite(RXANT, 1);
	gpioWrite(TXANT, 0);
	gpioWrite(EN_N, 0);
}
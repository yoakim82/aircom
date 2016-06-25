#include "rfm22.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/types.h>
#include "aircom.h"

#include "dbconnect.h"
#include "gpio.h"
//#include "hdlc.h"


#include <math.h>

//unsigned char parseMsg(msgType msg)
//{
/*
	msgType txMsg;

	uint16_t temp;
	
	txMsg.devAddr = 0;
	txMsg.command = msg.command;
	txMsg.size = 0;
	txMsg.chksum = 0;
	txMsg.data_ptr = &txData[0];

	
	
	if (msg.devAddr != MYADDRESS)
		return 0;
	else
	{
		switch (msg.command)
		{
			case READ_POWER :
				
				temp = power;
				
				txMsg.size = 2;
				txData[0] = (uint8_t) (temp >> 8);		// msb
				txData[1] = (uint8_t) (temp & 0xFF);	// lsb
				
				break;
				
			case READ_PULSES : 
				
				temp = pulse;
				pulse = 0;
				
				txMsg.size = 2;
				txData[0] = (uint8_t) (temp >> 8);		// msb
				txData[1] = (uint8_t) (temp & 0xFF);	// lsb
				
				break;
				
			case READ_VOLTAGE :
			
				
				txMsg.size = 1;
				txData[0] = voltage;
				
				break;
			
			default : 

				return 0;
				
				break;
		}

		sendMsg(txMsg);
	*/
	//	return 1;
	//}

//}




/* ****************************************************************
*
* Name			: main
*
* Function		: main routine
*
* Parameters	: void
*
* Returns		: int 0
*
***************************************************************** */


//Global Variables

int main(void)
{
	uint8_t rfRxBuf[35];
	//int msgComplete = 0;
	int i, chksum, ret;
	uint16_t power, pulse;
	uint8_t voltage;
	
	initDiscretes();
	
	usleep(100);
	//delay_us(100);
	
	writeRfm(OMFC1, 0x80);	// issue sw reset

	usleep(10000);
	//delay_us(10000);		// wait for devices to startup
	
	
	//sbi(CSPORT,CS);
	
	printf("Reading DTYPE register, should be 0x08: %d", readRfm(DTYPE));
	
	printf("\n*****************************************\n\n");

	init_RFM22();

	printf("Startup Completed\n\r");
	printf("Entering RX Mode...\n");
	to_rx_mode();
	//printf("done!\n");
    // loop forever, the interrupts are doing the rest
		
		/*
		pulse = 10;
		power = 20;
		voltage = 30;
		sendValuesToDB(pulse, power, voltage);
		exit(0);
		*/
	while(1)
	{
		if(gpioRead(IRQ_N) == 0)	//Interrupt will be generated if data is recieved
		{
			// Read 1-byte packet into rfRxBuf[]:
			for(i=0; i<=16; i++)
			{
				rfRxBuf[i] = readRfm(0x7F);
				//printf("%i: %x\n", i, rfRxBuf[i]);
			}
			
			// Check that the checksum matches up:
			chksum = 0;
			for(i=0; i<5; i++)
				chksum += rfRxBuf[i];
		
			chksum &= 0xFF; // mask checksum to 8 bits
			// If checksum is good, print out what was received into the terminal
			if(chksum == rfRxBuf[16])
			{
				writeRfm(0x07, 0x01);	// To ready mode
				//printf("Received: ");
				//for(i=0; i<6; i++)
				//	printf("%x ", rfRxBuf[i]);	// Print characters if passed the checksum
				//printf("\n");
				pulse = (rfRxBuf[0] << 8) | (uint16_t)rfRxBuf[1]; 
				power = (rfRxBuf[2] << 8) | (uint16_t)rfRxBuf[3];
				voltage = rfRxBuf[4];
				printf("Puls nr %i: %u W, v: %u mV\n", pulse, power, voltage);
				
				if (power < 10000)
					ret = sendValuesToDB(pulse, power, voltage);
				
				//printf("ret = %i\n", ret);
				//printf("To Ready mode\n");
				rx_reset();
			}
			// otherwise reset and start listening again
			else
			{
				printf("chksum = %x, rxbuf = %x", chksum, rfRxBuf[16]);
				printf("Chk failed! Received: ");
				for(i=0; i<6; i++)
					printf("%x ", rfRxBuf[i]);	// Print characters if passed the checksum
				printf("\n");
				
				pulse = (rfRxBuf[0] << 8) | (uint16_t)rfRxBuf[1]; 
				power = (rfRxBuf[2] << 8) | (uint16_t)rfRxBuf[3];
				voltage = rfRxBuf[4];
				printf("Puls nr %i: %u W, v: %u mV\n", pulse, power, voltage);
				rx_reset();
				printf("Bad checksum RX RESET...Listening again\n");
			}
			usleep(50000);
			//delay_ms(50);
		}
		usleep(50000);
	}

}
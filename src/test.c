#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/types.h>
#include <stdint.h>


int main(void)
{
	uint8_t rfRxBuf[35];
	//int msgComplete = 0;
	int i, chksum;
	uint16_t power, pulse;
	uint8_t voltage;
	

	printf("start!\n");

		rfRxBuf[0] = 1;
		rfRxBuf[1] = 0xff;
		rfRxBuf[2] = 2;
		rfRxBuf[3] = 0xff;
		rfRxBuf[4] = 0xa0;
		
				for(i=0; i<6; i++)
					printf("%x ", rfRxBuf[i]);	// Print characters if passed the checksum
				printf("\n");
				printf("u: %u \n", (rfRxBuf[0] << 8)|(uint16_t)rfRxBuf[1]);
				printf("i: %i \n", (rfRxBuf[0] << 8)|(uint16_t)rfRxBuf[1]);
				
				pulse =(uint16_t) ((uint16_t)rfRxBuf[0] << 8) | (uint16_t)rfRxBuf[1]; 
				power = (uint16_t)((uint16_t)rfRxBuf[2] << 8) | (uint16_t)rfRxBuf[3];
				voltage = rfRxBuf[4];
				printf("Puls nr %u: %u W, v: %i mV\n", pulse, power, voltage);

}

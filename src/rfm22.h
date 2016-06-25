#ifndef RFM22_H_
#define RFM22_H_

#include <stdint.h>

#define EN_N  RPI_V2_GPIO_P1_12
#define GPIO0 RPI_V2_GPIO_P1_22
#define GPIO1 RPI_V2_GPIO_P1_18
#define GPIO2 RPI_V2_GPIO_P1_16
#define IRQ_N RPI_V2_GPIO_P1_11
#define RXANT RPI_V2_GPIO_P1_13
#define TXANT RPI_V2_GPIO_P1_15


#define DTYPE 0x00
#define DVERS 0x01
#define DSTATUS 0x02

#define INTEN1 0x05
#define INTEN2 0x06
#define OMFC1 0x07
#define OMFC2 0x08
#define CRYSLOADCAP 0x09
#define UCOUTCLK 0x0A

#define TEMPCAL 0x12
#define TEMPOFF 0x13

void initDiscretes(void);

void writeRfm(uint8_t address, uint8_t data);

uint8_t readRfm(uint8_t address);

void init_RFM22(void);

void to_tx_mode(uint8_t txData[]);

void to_rx_mode(void);



int checkINT(void);

void rx_reset(void);

#endif /* RFM22_H_ */
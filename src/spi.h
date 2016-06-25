#ifndef SPI_H
#define SPI_H

uint8_t trxSpi(uint8_t txbyte);
void trxSpiBlock(uint8_t tx[], uint8_t rx[], int size);
int setupSPI(int fd);

#endif
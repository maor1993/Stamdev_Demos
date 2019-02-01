/*
 * spi_driver.h
 *
 *  Created on: 26 באפר׳ 2017
 *      Author: maor
 */

#ifndef SPI_DRIVER_H_
#define SPI_DRIVER_H_

#include <stdint.h>
int spi_init();

int spi_write_blocking(uint8_t* pData,uint16_t nSize);
int spi_read_blocking(uint8_t* pData,uint16_t nSize);
int spi_write_single(uint8_t nData);
int spi_write_read(uint8_t* pTxdata,uint16_t nTx_size,uint8_t* pRxdata,uint16_t nRx_size);
uint8_t spi_read_single();



#endif /* SPI_DRIVER_H_ */

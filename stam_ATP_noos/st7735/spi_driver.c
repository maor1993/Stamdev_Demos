/*
 * spi_driver.c
 *
 *  Created on: 26 באפר׳ 2017
 *      Author: maor
 */

#include "spi_driver.h"
#include "stm32f042x6.h"

SPI_TypeDef* pSPI = SPI1;
 //GPIO_TypeDef* pGP_NSS = GPIOA;
int spi_init()
{
	//setup the spi cr1:
	/*
	 * spi lines in unidiectional mode.
	 * crc disabled
	 * spi can transmit and recieve.
	 * cs is cs and not sync pulse
	 * msb is sent first
	 * baud rate is pclk/8
	 * spi is master
	 * clk is 0 when idle
	 * clk will rise on the first data bit.
	 */
	pSPI->CR1 &= (~SPI_CR1_BIDIMODE)&(~SPI_CR1_BIDIOE)&(~SPI_CR1_CRCEN)&(~SPI_CR1_RXONLY)&(~SPI_CR1_SSM)&(~SPI_CR1_LSBFIRST)&(~SPI_CR1_CPOL)&(~SPI_CR1_CPHA);
	pSPI->CR1 |= (SPI_CR1_MSTR)|(2<<SPI_CR1_BR_Pos);



	//setup the spi_cr2:
	/*disable dma and interrupts
	 * set the data size to 8 bits
	 * enable nss pulse.
	 */


	pSPI->CR2 &= (~SPI_CR2_TXEIE)&(~SPI_CR2_RXNEIE)&(~SPI_CR2_ERRIE)&(~SPI_CR2_FRF)&(~SPI_CR2_TXDMAEN)&(~SPI_CR2_RXDMAEN);
	pSPI->CR2 |= (SPI_CR2_NSSP)|(7<<SPI_CR2_DS_Pos);


	//enable the spi module.
	pSPI->CR1 |= (SPI_CR1_SPE);

	return 0;
}

int spi_write_blocking(uint8_t* pData,uint16_t nSize)
{
	uint16_t i;
//	pGP_NSS->BR15 = 1;
	for(i=0;i<nSize;i++)
	{

		//send data to buffer
		   *((__IO uint8_t *)&pSPI->DR) = *(pData+i);
		 //block until data was sent.
		while(pSPI->SR & SPI_SR_BSY_Msk);

	}
//	pGP_NSS->BS15 = 1;
	return 0;
}

int spi_read_blocking(uint8_t* pData,uint16_t nSize)
{
	uint16_t i;
//	pGP_NSS->BR15 = 1;
	for(i=0;i<nSize;i++)
	{
		//send dummy write to get clocks on the bus.
		 *((__IO uint8_t *)&pSPI->DR) = 0;
		 while(pSPI->SR & SPI_SR_BSY_Msk); //wait for the data to be sent.
		//ensure data is waiting in the buffer
		while(!(pSPI->SR & SPI_SR_RXNE_Msk));
		//pull data from buffer
		*(pData + i) = *((__IO uint8_t *)&pSPI->DR);

	}

//	 pGP_NSS->BS15 = 1;
	return 0;
}
int spi_write_single(uint8_t nData)
{
//	pGP_NSS->BR15 = 1;
	   *((__IO uint8_t *)&pSPI->DR) = nData;
	   while(pSPI->SR & SPI_SR_BSY_Msk); //wait for the data to be sent.

//	   pGP_NSS->BS15 = 1;
	return 0;
}
uint8_t spi_read_single()
{
	//send dummy write to get clocks on the bus.
//	pGP_NSS->BR15 = 1;
	 *((__IO uint8_t *)&pSPI->DR) = 0;
	 while(pSPI->SR & SPI_SR_BSY_Msk); //wait for the data to be sent.
	//ensure data is waiting in the buffer
	while(!(pSPI->SR & SPI_SR_RXNE_Msk));
	//pull data from buffer
	return  *((__IO uint8_t *)&pSPI->DR);
//	pGP_NSS->BS15 = 1;
}
int spi_write_read(uint8_t* pTxdata,uint16_t nTx_size,uint8_t* pRxdata,uint16_t nRx_size)
{
	uint16_t i=0;
	//lower the nss line.
//	pGP_NSS->BR15 = 1;
	//write the required data to the device.
	for(i=0;i<nTx_size;i++)
	{
		//send data to buffer
		   *((__IO uint8_t *)&pSPI->DR) = *(pTxdata+i);
		 //block until data was sent.
		while(pSPI->SR & SPI_SR_BSY_Msk);
	}
	//flush the empty rx data
//	do{
//		*((__IO uint8_t *)&pSPI->DR);
//	}while((pSPI->SR & SPI_SR_FRLVL_Msk) != 0);

	do
	{
		*((__IO uint8_t *)&pSPI->DR);
	}while(i--);




//	if(nRx_size != 1)
//	{
//
//		 *((__IO uint8_t *)&pSPI->DR) = 0;
//		 while(pSPI->SR & SPI_SR_BSY_Msk); //wait for the data to be sent.
//		//ensure data is waiting in the buffer
//		//while(!(pSPI->SR & SPI_SR_RXNE_Msk));
//		//pull data from buffer
//		*(pRxdata) = *((__IO uint8_t *)&pSPI->DR);
//	}



	//read data from dev
	for(i=0;i<nRx_size;i++)
	{
		//send dummy write to get clocks on the bus.
		 *((__IO uint8_t *)&pSPI->DR) = 0;
		 while(pSPI->SR & SPI_SR_BSY_Msk); //wait for the data to be sent.
		//ensure data is waiting in the buffer
		while((pSPI->SR&SPI_SR_FRLVL_Msk) == 0);
		//pull data from buffer
		*(pRxdata + i) = *((__IO uint8_t *)&pSPI->DR);
	}

//	if(nRx_size == 1)
//	{
//		*(pRxdata) = *((__IO uint8_t *)&pSPI->DR);
//	}

	//raise the nss line.
//	pGP_NSS->BS15 = 1;
	return 0;
}




/*
 * usb_icd.c
 *
 *  Created on: 4 Feb 2019
 *      Author: Maor
 */


#include "stm32f0xx_hal.h"
#include <stdint.h>


const uint8_t nIcdPremble[4] = {0x9a,0x5b,0xc0,0xdd};

extern TIM_HandleTypeDef htim1;


void handle_icd_msg(uint8_t* pData)
{

	//step 1: verify that premble is correct.

	if(memcmp(pData,nIcdPremble,4))
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,*(pData+4));
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,*(pData+5));
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,*(pData+6));

	}
}



#include "cppWrapper.hpp"

#include "Sanran.hpp"

#include <stdio.h>



Sanran *p_sanran;


void HAL_FDCAN_RxFifo0Callback (FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

	if(hfdcan->Instance == FDCAN2 && RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
	{
		p_sanran->CAN_Rx_Callback(hfdcan);
	}


}


void cppMain(void)
{

	Sanran sanran;

	p_sanran = &sanran;

	while(1)
	{

		sanran.UpdateAsync();

	}


}




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


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM12)
	{
		p_sanran->UpdateSyncHS();
	}
	else if(htim->Instance == TIM13)
	{
		p_sanran->UpdateSyncLS();
	}

}


void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{

	p_sanran->UART_Rx_Callback(huart);

}


void cppMain(void)
{

	Sanran sanran;

	p_sanran = &sanran;

	sanran.startCycle();

	while(1)
	{

		sanran.UpdateAsync();

	}


}




#include "cppWrapper.hpp"

#include "Sanran.hpp"

#include <stdio.h>

#include "ntshell.h"
#include "usrcmd.h"



Sanran sanran;


ntshell_t nts;



void HAL_FDCAN_RxFifo0Callback (FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

	if(hfdcan->Instance == FDCAN2 && RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
	{
		sanran.CAN_Rx_Callback(hfdcan);
	}

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM12)
	{
		sanran.UpdateSyncHS();
	}
	else if(htim->Instance == TIM13)
	{
		sanran.UpdateSyncLS();
	}

}


void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{

	sanran.UART_Rx_Callback(huart);

}


void cppMain(void)
{



	sanran.setup();

	sanran.startCycle();

//	ntshell_usr_init(&nts);

//	ntshell_execute(&nts);


	while(1)
	{

		sanran.UpdateAsync();

	}


}


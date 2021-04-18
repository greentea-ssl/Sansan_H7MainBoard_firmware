


#include "Sanran.hpp"



#include <stdio.h>
#include "main.h"



extern TIM_HandleTypeDef htim3;
extern FDCAN_HandleTypeDef hfdcan2;



/**
 * @fn  Sanran()
 * @brief Constructor of Sanran class
 *
 */
Sanran::Sanran()
	: onBrdLED(&htim3, TIM_CHANNEL_2, TIM_CHANNEL_1, TIM_CHANNEL_3),
	  canMotorIF(&hfdcan2)
{

	printf("oppai...\n");

	count = 0;

	onBrdLED.setRGB(0, 0, 0);

	deg = 0.0;

}


/**
 * @fn void UpdateAsync()
 * @brief　非同期ループ(タイミングクリティカルな制御は禁止)
 *
 */
void Sanran::UpdateAsync()
{

	power.update();

	HAL_Delay(100);

	//printf("count = %d\n", count++);

	power.enableSupply();

	deg += 0.01;
	if(deg > 1.0) deg -= 1.0;

	onBrdLED.setHSV(deg, 1.0, 1.0);


	if(canMotorIF.motor[0].resIsUpdated() || canMotorIF.motor[1].resIsUpdated() || canMotorIF.motor[2].resIsUpdated() || canMotorIF.motor[3].resIsUpdated())
	{
		printf("    | Iq \t omega \t\t theta \n");
		printf("------------------------------------------\n");
		for(int i = 0; i < 4; i++)
		{
			printf("[%d] | %4.2f \t %4.2f \t\t %4.2f\n", i, canMotorIF.motor[i].get_Iq(), canMotorIF.motor[i].get_omega(), canMotorIF.motor[i].get_theta() * 180.0 / M_PI);
		}

		printf("\e[6A");

		canMotorIF.motor[0].clearUpdateFlag();
	}
	else
	{
		printf(".\n");
	}




	canMotorIF.motor[0].set_Iq_ref(0.0);
	canMotorIF.motor[1].set_Iq_ref(0.0);
	canMotorIF.motor[2].set_Iq_ref(0.0);
	canMotorIF.motor[3].set_Iq_ref(0.0);

	canMotorIF.send_Iq_ref();



}

/**
 * @fn void UpdateSyncHS()
 * @brief 制御用高速ループ
 *
 */
void Sanran::UpdateSyncHS()
{

}

/**
 * @fn void UpdateSyncLS()
 * @brief 表示系用低速ループ
 *
 */
void Sanran::UpdateSyncLS()
{

}



void Sanran::CAN_Rx_Callback(FDCAN_HandleTypeDef *hfdcan)
{


	canMotorIF.update_CAN_Rx();

}






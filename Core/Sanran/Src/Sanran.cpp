


#include "Sanran.hpp"



#include <stdio.h>
#include "main.h"



extern TIM_HandleTypeDef htim3;



/**
 * @fn  Sanran()
 * @brief Constructor of Sanran class
 *
 */
Sanran::Sanran() : onBrdLED(&htim3, TIM_CHANNEL_2, TIM_CHANNEL_1, TIM_CHANNEL_3)
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

	HAL_Delay(10);

	//printf("count = %d\n", count++);

	power.enableSupply();

	deg += 0.0005;
	if(deg > 1.0) deg -= 1.0;

	onBrdLED.setHSV(deg, 1.0, 1.0);


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






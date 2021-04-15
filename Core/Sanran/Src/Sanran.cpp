



#include "Sanran.hpp"


#include <stdio.h>
#include "main.h"

/**
 * @fn  Sanran()
 * @brief Constructor of Sanran class
 *
 */
Sanran::Sanran()
{


	printf("oppai...\n");

	count = 0;

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


	HAL_Delay(100);

	//power.disableSupply();

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






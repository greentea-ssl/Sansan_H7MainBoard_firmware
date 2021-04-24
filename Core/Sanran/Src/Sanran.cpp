


#include "Sanran.hpp"



#include <stdio.h>
#include "main.h"



extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern FDCAN_HandleTypeDef hfdcan2;
extern I2C_HandleTypeDef hi2c2;



/**
 * @fn  Sanran()
 * @brief Constructor of Sanran class
 *
 */
Sanran::Sanran()
	: onBrdLED(&htim3, TIM_CHANNEL_2, TIM_CHANNEL_1, TIM_CHANNEL_3),
	  canMotorIF(&hfdcan2),
	  buzzer(&htim2, TIM_CHANNEL_1, 240E+6),
	  bno055(&hi2c2),
	  dribbler(&htim1, TIM_CHANNEL_1)
{

	printf("oppai...\n");

	count = 0;

	onBrdLED.setRGB(0, 0, 0);

	buzzer.sound_startup();

	//bno055.write(0x3D, 0x08);

	printf("BNO055, initialize result: %d\n", bno055.getStatus());

	printf("BNO055, who am i check : %d\n", bno055.checkChipID());

	deg = 0.0;

	dribbler.setSlow();

	delay_ms(1000);

	dribbler.setFast();

	delay_ms(2000);

	dribbler.setSlow();

	delay_ms(2000);

	dribbler.setStop();

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

	power.enableSupply();

	deg += 0.05;
	if(deg > 1.0) deg -= 1.0;

	onBrdLED.setHSV(deg, 1.0, 1.0);


	if(canMotorIF.motor[0].resIsUpdated() || canMotorIF.motor[1].resIsUpdated() || canMotorIF.motor[2].resIsUpdated() || canMotorIF.motor[3].resIsUpdated())
	{
		//printf("    | Iq \t omega \t\t theta \n");
		//printf("------------------------------------------\n");
		for(int i = 0; i < 4; i++)
		{
			//printf("[%d] | %4.2f \t %4.2f \t\t %4.2f\n", i, canMotorIF.motor[i].get_Iq(), canMotorIF.motor[i].get_omega(), canMotorIF.motor[i].get_theta() * 180.0 / M_PI);
		}
		//printf("\e[6A");
		canMotorIF.motor[0].clearUpdateFlag();
	}
	else
	{
		//printf(".\n");
	}
	canMotorIF.motor[0].set_Iq_ref(0.0);
	canMotorIF.motor[1].set_Iq_ref(0.0);
	canMotorIF.motor[2].set_Iq_ref(0.0);
	canMotorIF.motor[3].set_Iq_ref(0.0);
	//canMotorIF.send_Iq_ref();



	bno055.updateIMU();

	uint8_t st, er;
	bno055.read(0x39, &st);
	bno055.read(0x3D, &er);

	printf("Roll  = %f rad.\n", bno055.get_IMU_roll());
	printf("Pitch = %f rad.\n", bno055.get_IMU_pitch());
	printf("Yaw   = %f rad.\n", bno055.get_IMU_yaw());
	printf("status = %d, 0x%02x, 0x%02x\n", bno055.getStatus(), st, er);
	printf("\e[4A");

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









#include "Sanran.hpp"



#include <stdio.h>
#include "main.h"



extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern FDCAN_HandleTypeDef hfdcan2;
extern I2C_HandleTypeDef hi2c2;



/* Debug variables */

volatile float omega_w = 0.0;
volatile float omega_w_ref = 0.0;
volatile float Iq_ref = 0.0;
volatile float lpf_test = 0.0f;


/**
 * @fn  Sanran()
 * @brief Constructor of Sanran class
 *
 */
Sanran::Sanran()
	: htim_HS_cycle(&htim12),
	  htim_LS_cycle(&htim13),
	  onBrdLED(&htim3, TIM_CHANNEL_2, TIM_CHANNEL_1, TIM_CHANNEL_3),
	  canMotorIF(&hfdcan2),
	  buzzer(&htim2, TIM_CHANNEL_1, 240E+6),
	  bno055(&hi2c2),
	  dribbler(&htim1, TIM_CHANNEL_1),
	  kicker(0.01, 0.5),
	  omni(OmniWheel::TYPE_P_DOB, &canMotorIF)
{

	printf("oppai...\n");

	count = 0;

	onBrdLED.setRGB(0, 0, 0);

	buzzer.sound_startup();

	//bno055.write(0x3D, 0x08);

	printf("BNO055, initialize result: %d\n", bno055.getStatus());

	printf("BNO055, who am i check : %d\n", bno055.checkChipID());

	deg = 0.0;

	userButton0_prev = 1;
	userButton1_prev = 1;

	//dribbler.setStop();
	dribbler.setFast();

	power.enableSupply();

	delay_ms(4000);


}



void Sanran::startCycle()
{

	HAL_TIM_Base_Start_IT(htim_HS_cycle);
	HAL_TIM_Base_Start_IT(htim_LS_cycle);

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

	deg += 0.05;
	if(deg > 1.0) deg -= 1.0;

	onBrdLED.setHSV(deg, 1.0, 1.0);


#if 0

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

#endif

	bno055.updateIMU();

	uint8_t st, er;
	bno055.read(0x39, &st);
	bno055.read(0x3D, &er);
/*
	printf("Roll  = %f rad.\n", bno055.get_IMU_roll());
	printf("Pitch = %f rad.\n", bno055.get_IMU_pitch());
	printf("Yaw   = %f rad.\n", bno055.get_IMU_yaw());
	printf("status = %d, 0x%02x, 0x%02x\n", bno055.getStatus(), st, er);
	printf("\e[4A");
*/


	//printf("USER_SW0 = %f\n", omega_w_ref);

	uint8_t userButton0 = HAL_GPIO_ReadPin(USER_SW0_GPIO_Port, USER_SW0_Pin);
	uint8_t userButton1 = HAL_GPIO_ReadPin(USER_SW1_GPIO_Port, USER_SW1_Pin);

	if(userButton0 == 0 && userButton1_prev == 1)
	{
		kicker.kickStraight();
	}
	if(userButton1 == 0 && userButton1_prev == 1)
	{
		kicker.kickChip();
	}
	userButton0_prev = userButton0;
	userButton1_prev = userButton1;


	kicker.update();

}

/**
 * @fn void UpdateSyncHS()
 * @brief 制御用高速ループ
 *
 */
void Sanran::UpdateSyncHS()
{

	if(count < 2000)
	{
		omniCmd.vel_x = 1.0f;
		omniCmd.vel_y = 0.0f;
		omniCmd.omega = 0.0f;
	}
	else if(count < 4000)
	{
		omniCmd.vel_x = 0.0f;
		omniCmd.vel_y = 1.0f;
		omniCmd.omega = 0.0f;
	}
	else if(count < 6000)
	{
		omniCmd.vel_x = 0.0f;
		omniCmd.vel_y = 0.0f;
		omniCmd.omega = 10.0f;
	}
	else
	{
		omniCmd.vel_x = 0.0f;
		omniCmd.vel_y = 0.0f;
		omniCmd.omega = 0.0f;
	}

	omniCmd.omega_w[0] = 0.0f;
	omniCmd.omega_w[1] = 0.0f;
	omniCmd.omega_w[2] = 0.0f;
	omniCmd.omega_w[3] = 0.0f;

	omega_w = canMotorIF.motor[0].get_omega();

	count += 1;

	omni.update(&omniCmd);

	Iq_ref = canMotorIF.motor[0].get_Iq_ref();

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






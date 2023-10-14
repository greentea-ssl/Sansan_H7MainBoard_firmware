


#include "Sanran.hpp"



#include <stdio.h>
#include "main.h"


#include "stm32h753xx.h"


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern FDCAN_HandleTypeDef hfdcan2;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern SPI_HandleTypeDef hspi4;


/* Debug variables */

volatile float omega_w = 0.0;
volatile float omega_w_ref = 0.0;
volatile float Iq_ref = 0.0;
volatile float lpf_test = 0.0f;

volatile float odo_x = 0.0f;
volatile float odo_y = 0.0f;
volatile float odo_theta = 0.0f;

volatile float wheel_theta[4] = {0};


#define DISP_SETUP_RESULT(FUNC, NAME, IS_CRITICAL)	do{ \
														printf("\n Setting %s ... \n", NAME); \
														if(FUNC()){printf("\t\t\t\t[OK]\n"); \
														}else{ \
															printf("\t\t\t\t[ERROR]\n"); \
															if(IS_CRITICAL){Error_Handler();}} \
													}while(0);


void Sanran::captureBall(BallInformation &ball_data){
  omni.setControlType(OmniWheel::TYPE_ROBOT_P_DOB);
//  omni.correctPosition(matcha.normal_cmd.fb_x, matcha.normal_cmd.fb_y, matcha.normal_cmd.fb_theta); // Visionから降ってくる自己位置
  //
  float ref_rad =  M_PI / 2;
  float ref_world_x = 1.5;	//守備位置
  static bool lost_flag = true;
  static float ball_x = 0; //受信したボール位置 単位はm
  static float ball_y = 0; //受信したボール位置 単位はm
  float robot_radius = 0.09; //ロボット半径 m
  static float vel_x_for_ball = 0;		//ボール追従のためのロボットの制御速度
  static float vel_y_for_ball = 0;		//ボール追従のためのロボットの制御速度
  static float vel_w_for_ball = 0;		//ボール追従のためのロボットの制御速度

  static float world_robotpos_x = ref_world_x;
  static float world_robotpos_y = 0;
  static float world_robotpos_theta = 0;
  static float world_ball_pos_x = 0;
  static float world_ball_pos_y = 0;
  static int16_t vision_lost_counter = 0;
  int16_t vision_lost_threshold = 100;
  static bool vision_lost = true;

  static float world_x_error = 0;	//ワールド座標系における守備位置誤差
  static float world_y_error = 0;	//ワールド座標系における守備位置誤差
  float vel_x_for_world = 0;		//守備位置誤差修正のためのロボットの制御速度
  float vel_y_for_world = 0;		//守備位置誤差修正のためのロボットの制御速度
  float vel_w_for_world = 0;

  static float theta_error = 0;		//守備向きの誤差

  float gain_ball_x = 1;
  float gain_ball_w = 2.5;
  float gain_w = 2.5;
  float gain_postion_world_y = 2;
  float gain_postion_world_x = 1.2;

  float y_pos_max = 0.3;
  float y_pos_min = -0.3;

  static float x_error = 0;			//ボールの位置に対する誤差
  static float y_error = 0;			//ボールの位置に対する誤差

  omniCmd.robot_vel_x = 0;

  //信号処理
  //カメラからのボール位置
  static int16_t lost_counter = 0;
  const int lost_counter_threshold = 50;
  if(ball_data.status == 0){
	  lost_counter ++;
	  if(lost_counter > lost_counter_threshold){
		  lost_counter = lost_counter_threshold + 1;
		  printf("ball lost\n");
		  lost_flag = true;
		  ball_x= 0;
		  ball_y = 0;
	  }
  } else {
	  ball_x = ball_data.x / 100.0; //cm -> m
	  ball_y = ball_data.y / 100.0; //cm -> m

	  lost_counter = 0;
	  lost_flag = false;
  }

  //Visionから降ってくるやつ
  if(matcha.newDataAvailable() && matcha.getReceiveState() == MatchaSerial::RECEIVE_STATE_NORMAL){
	  vision_lost_counter = 0;
	  vision_lost = false;
	  world_robotpos_x = matcha.normal_cmd.fb_x;
	  world_robotpos_y = matcha.normal_cmd.fb_y;
	  world_robotpos_theta = matcha.normal_cmd.fb_theta;

	  auto offset_ball_y = ball_y + robot_radius;

	  auto rotate_ball_x = ball_x * cosf(world_robotpos_theta) - offset_ball_y * sinf(world_robotpos_theta);
	  auto rotate_ball_y = ball_x * sinf(world_robotpos_theta) + offset_ball_y * cos(world_robotpos_theta);

	  world_ball_pos_x = rotate_ball_x + world_robotpos_x;
	  world_ball_pos_y = rotate_ball_y + world_robotpos_y;

  }else{
	  vision_lost_counter += 1;

	  if(vision_lost_counter > vision_lost_threshold){
		  vision_lost_counter = vision_lost_threshold + 1;
		  vision_lost = true;

		  world_robotpos_x = ref_world_x;
		  world_robotpos_y = 0;
		  world_robotpos_theta = 0;
	  }
  }

  float imu_theta;
  float ball_error_theta;
  // 制御処理
  switch(opeMode){
  case OPE_MODE_DEBUG://ロボット座標のみで動く
	  imu_theta = bno055.get_IMU_yaw() * -1;

	  if(imu_theta > M_PI){
		  imu_theta -=2 * M_PI;
	  }
	  if(imu_theta < -M_PI){
		  imu_theta +=2 * M_PI;
	  }

	  theta_error = imu_theta;

	  vel_w_for_world = -gain_w * (theta_error);

	  if(!lost_flag){
		  x_error = ball_x;
		  vel_x_for_ball = gain_ball_x * x_error;
	  }else{
		  x_error = 0;
		  vel_x_for_ball = 0;
	  }
	  omniCmd.robot_vel_x = fminf(1, fmaxf(-1 , vel_x_for_ball));
	  omniCmd.robot_vel_y = 0;
	  omniCmd.robot_omega = fminf(4, fmaxf(-4 , vel_w_for_world));
	  break;
  case OPE_MODE_KEEPER_W_LOCALCAMERA://Visionから位置情報がもらえる

	  if(!lost_flag){

		  x_error = world_ball_pos_x - world_robotpos_x;	//ワールド基準で考える
		  y_error = world_ball_pos_y - world_robotpos_y;
		  ball_error_theta = atan2f(ball_y, ball_x) - M_PI / 2; //float atan2(float y, float x);

		  vel_x_for_ball = gain_ball_x * x_error * sinf(world_robotpos_theta);
		  vel_y_for_ball = gain_ball_x * x_error * cosf(world_robotpos_theta);
		  vel_w_for_ball = gain_ball_w * ball_error_theta;
		  vel_w_for_world = 0;
	  }else{
		  x_error = 0;
		  vel_x_for_ball = 0;
		  vel_y_for_ball = 0;
		  vel_w_for_ball = 0;
	  }

	  if(!vision_lost){
		  theta_error = world_robotpos_theta - ref_rad;
		  if(theta_error < -M_PI)
			  theta_error += 2 * M_PI;
		  else if(theta_error > M_PI)
			  theta_error -= 2 * M_PI;

		  if(!lost_flag)
			  vel_w_for_world = -gain_w * (theta_error);
		  else
			  vel_w_for_world = 0;

		  world_x_error = world_ball_pos_x - ref_world_x;

		  if (world_ball_pos_y < y_pos_min){
			  world_y_error = world_ball_pos_y - y_pos_min;
			  vel_x_for_ball = 0;							//出ていかないようにボール追従をやめる
			  vel_y_for_ball = 0;							//出ていかないようにボール追従をやめる

		  }else if (world_ball_pos_y > y_pos_max){
			  world_y_error = world_ball_pos_y - y_pos_max;
			  vel_x_for_ball = 0;							//出ていかないようにボール追従をやめる
			  vel_y_for_ball = 0;							//出ていかないようにボール追従をやめる
		  }else if(world_ball_pos_y > 0){
			  world_y_error = 0.01;
		  }else{
			  world_y_error = -0.01;
		  }

		  vel_x_for_world = (-gain_postion_world_x * world_x_error) * cosf(world_robotpos_theta) + (-gain_postion_world_y * world_y_error) * sinf(world_robotpos_theta);
		  vel_y_for_world = (gain_postion_world_x * world_x_error) * sinf(world_robotpos_theta) + (-gain_postion_world_y * world_y_error) * cosf(world_robotpos_theta);
	  }else{
		  vel_x_for_world = 0;
		  vel_y_for_world = 0;
	  }

	  omniCmd.robot_omega = fminf(4, fmaxf(-4 , vel_w_for_world + vel_w_for_ball));
	  omniCmd.robot_vel_x = fminf(1, fmaxf(-1 , vel_x_for_ball + vel_x_for_world));
	  omniCmd.robot_vel_y = vel_y_for_world;

	  //  printf("%f\r\n", matcha.normal_cmd.fb_theta);
	 //   printf("world_x_error: %f, world_y_error: %f\n", world_x_error, world_y_error);
	  break;
  default:
	  break;
  }

}


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
	  ballSensor(&hadc1),
	  dribbler(&htim1, TIM_CHANNEL_1),
	  kicker(&htim4, TIM_CHANNEL_1),
	  omni(OmniWheel::TYPE_WORLD_POSITION, &canMotorIF),
	  matcha(&huart5, 0.2, 5.0, 0.01),
	  dump(&hspi4),
	  ball_info_communication(&huart1)
{

	memset(&omniCmd, 0x00, sizeof(omniCmd));

}



void Sanran::setup()
{

	uint8_t userButton0 = HAL_GPIO_ReadPin(USER_SW0_GPIO_Port, USER_SW0_Pin);
	uint8_t userButton1 = HAL_GPIO_ReadPin(USER_SW1_GPIO_Port, USER_SW1_Pin);

	printf("\nHello. ");

	// ボタンは負論理
	if(userButton0 == 0)
	{
		opeMode = OPE_MODE_DEBUG;
		printf("[DEBUG_MODE]\n\n");
		buzzer.play(Buzzer::SOUND_STARTUP_DEBUG);
	}
	else if(userButton1 == 0)
	{
		opeMode = OPE_MODE_MANUAL;
		printf("[MANUAL_MODE]\n\n");
		buzzer.play(Buzzer::SOUND_STARTUP_MANUAL);
	}
	else
	{
		opeMode = OPE_MODE_KEEPER_W_LOCALCAMERA;
		printf("[KEEPER_W_LOCALCAMERA_MODE]\n\n");
		buzzer.play(Buzzer::SOUND_STARTUP_DEBUG);
	}

	power.enableSupply();

	delay_ms(200);

	printf("********** Initialize ********************\n\n");

	printf("\n Setting %s ... \n", "On Board LED");
	DISP_SETUP_RESULT(onBrdLED.setup, "On Board LED", false);

	DISP_SETUP_RESULT(canMotorIF.setup, "CAN motor IF", true);

	DISP_SETUP_RESULT(bno055.setup, "BNO055", false);

	DISP_SETUP_RESULT(omni.setup, "Omni Wheel", (opeMode==OPE_MODE_NORMAL));

	DISP_SETUP_RESULT(kicker.setup, "Kicker", true);

	DISP_SETUP_RESULT(matcha.setup, "Matcha Serial", true);

	timeElapsed_hs_count = 0;

	onBrdLED.setRGB(0, 0, 0);

	deg = 0.0;

	userButton0_prev = 1;
	userButton1_prev = 1;

	dribbler.setup();


	buzzer.play(Buzzer::SOUND_NOTIFY);

	// WachDog(software reset) initialize
	watchdog_CAN_count = 0;
	watchdog_CAN_threshold = 1000;
	watchdog_UART_count = 0;
	watchdog_UART_threshold = 10000;

	//setup communication with raspberry pi
	ball_info_communication.init();

	if(opeMode == OPE_MODE_NORMAL || opeMode == OPE_MODE_KEEPER_W_LOCALCAMERA)
	{
		watchdog_enable = true;
	}
	else
	{
		watchdog_enable = false;
	}


	printf("\n********** Start ********************\n");

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
}

/**
 * @fn void UpdateSyncHS()
 * @brief 制御用高速ループ
 *
 */
void Sanran::UpdateSyncHS()
{
	syncHS_timestamp.start_count = htim12.Instance->CNT;

	kicker.update();

	omni.update(&omniCmd);

	odo_x = omni.m_robotState.world_x;
	odo_y = omni.m_robotState.world_y;
	odo_theta = omni.m_robotState.world_theta;
	for(int ch = 0; ch < 4; ch++)
	{
		wheel_theta[ch] = canMotorIF.motor[ch].get_theta();
	}

	// Update WatchDog
	update_watchdog();

	update_dump();

	timeElapsed_hs_count += 1;
	syncHS_timestamp.end_count = htim12.Instance->CNT;

}

/**
 * @fn void UpdateSyncLS()
 * @brief 表示系用低速ループ
 *
 */
void Sanran::UpdateSyncLS()
{
	syncLS_timestamp.start_count = htim13.Instance->CNT;

	power.update();

	deg += 0.01;
	if(deg > 1.0) deg -= 1.0;
	onBrdLED.setHSV(deg, 1.0, 1.0);

	 bno055.updateIMU();
	ballSensor.update();

//	if(ballSensor.read() > 0.15) dribbler.setSlow();
//	else dribbler.setFast();


	matcha.Update();

	// get ball information
	BallInformationResult result = ball_info_communication.ReceiveBallInformation(&rx_ball);

  switch(result){
  case BALLINFO_TIMEOUT:
	  printf("BALLINFO_TIMEOUT\n");
	  rx_ball.x = 0;
	  rx_ball.y = 0;
	  rx_ball.vx = 0;
	  rx_ball.vy = 0;
	  rx_ball.status = 0;
	  break;
  case BALLINFO_SUCCESS:
//    printf("decoded:");
//    printf("x:%f, y:%f, status:%ld\n\r", rx_ball.x, rx_ball.y, rx_ball.status);
//    printf("\n\r");
    break;
  case BALLINFO_DECODE_NULL_POINTER:
	  rx_ball.x = 0;
	  rx_ball.y = 0;
	  rx_ball.vx = 0;
	  rx_ball.vy = 0;
	  rx_ball.status = 0;
    printf("BALLINFO_DECODE_NULL_POINTER\n\r");
    break;
  case BALLINFO_DECODE_OUT_BUFFER_OVERFLOW:
	  rx_ball.x = 0;
	  rx_ball.y = 0;
	  rx_ball.vx = 0;
	  rx_ball.vy = 0;
	  rx_ball.status = 0;
    printf("BALLINFO_DECODE_OUT_BUFFER_OVERFLOW\n\r");
    break;
  case BALLINFO_DECODE_ZERO_BYTE_IN_INPUT:
	  rx_ball.x = 0;
	  rx_ball.y = 0;
	  rx_ball.vx = 0;
	  rx_ball.vy = 0;
	  rx_ball.status = 0;
    printf("BALLINFO_DECODE_ZERO_BYTE_IN_INPUT\n\r");
    break;
  case BALLINFO_DECODE_INPUT_TOO_SHORT:
	  rx_ball.x = 0;
	  rx_ball.y = 0;
	  rx_ball.vx = 0;
	  rx_ball.vy = 0;
	  rx_ball.status = 0;
    printf("BALLINFO_DECODE_INPUT_TOO_SHORT\n\r");
  case BALLINFO_DECODE_UNKNOWN:
	  rx_ball.x = 0;
	  rx_ball.y = 0;
	  rx_ball.vx = 0;
	  rx_ball.vy = 0;
	  rx_ball.status = 0;
    printf("BALLINFO_DECODE_UNKNOWN\n\r");
    break;
  case BALLINFO_FRAME_FAIL:
	  rx_ball.x = 0;
	  rx_ball.y = 0;
	  rx_ball.vx = 0;
	  rx_ball.vy = 0;
	  rx_ball.status = 0;
//      printf("BALLINFO_FRAME_FAIL\n\r");
    break;
  case BALLINFO_FRAME_INVALID_LENGTH:
	  rx_ball.x = 0;
	  rx_ball.y = 0;
	  rx_ball.vx = 0;
	  rx_ball.vy = 0;
	  rx_ball.status = 0;
    printf("BALLINFO_FRAME_INVALID_LENGTH\n\r");
    break;
  default:
	  rx_ball.x = 0;
	  rx_ball.y = 0;
	  rx_ball.vx = 0;
	  rx_ball.vy = 0;
	  rx_ball.status = 0;
    printf("unknown error\n\r");
	  break;
  }
  captureBall(rx_ball);
	// decide robot speed

//	if(matcha.newDataAvailable() && matcha.getReceiveState() == MatchaSerial::RECEIVE_STATE_NORMAL)
//	{
//		omniCmd.world_x = matcha.normal_cmd.cmd_x;
//		omniCmd.world_y = matcha.normal_cmd.cmd_y;
//		omniCmd.world_theta = matcha.normal_cmd.cmd_theta;
//		omniCmd.world_vel_x = matcha.normal_cmd.cmd_vx;
//		omniCmd.world_vel_y = matcha.normal_cmd.cmd_vy;
//		omniCmd.world_omega = matcha.normal_cmd.cmd_omega;
//		omniCmd.vel_limit = matcha.normal_cmd.vel_limit;
//		omniCmd.omega_limit = matcha.normal_cmd.omega_limit;
//
//		if(matcha.normal_cmd.kick)
//		{
//			if(matcha.normal_cmd.chip)
//			{
//				kicker.kickChip(matcha.normal_cmd.kickPower);
//			}
//			else
//			{
//				kicker.kickStraight(matcha.normal_cmd.kickPower);
//			}
//		}
//
//		if(matcha.normal_cmd.dribble)
//		{
//			dribbler.setPower(matcha.normal_cmd.dribblePower);
//		}
//		else
//		{
//			dribbler.setPower(0);
//		}
//
//		if(matcha.normal_cmd.vision_error == false)
//		{
//			omni.correctPosition(matcha.normal_cmd.fb_x, matcha.normal_cmd.fb_y, matcha.normal_cmd.fb_theta); // Visionから降ってくる自己位置
//		}
//
//		omni.setControlType(OmniWheel::TYPE_WORLD_POSITION);
//
//	}
//	else if(matcha.newDataAvailable() && matcha.getReceiveState() == MatchaSerial::RECEIVE_STATE_MANUAL)
//	{
//		omniCmd.robot_vel_x = matcha.manual_cmd.cmd_vx;
//		omniCmd.robot_vel_y = matcha.manual_cmd.cmd_vy;
//		omniCmd.robot_omega = matcha.manual_cmd.cmd_omega;
//
//		if(matcha.manual_cmd.kick)
//		{
//			if(matcha.manual_cmd.chip)
//			{
//				kicker.kickChip(matcha.manual_cmd.kickPower);
//			}
//			else
//			{
//				kicker.kickStraight(matcha.manual_cmd.kickPower);
//			}
//		}
//
//		if(matcha.manual_cmd.dribble)
//		{
//			dribbler.setPower(matcha.manual_cmd.dribblePower);
//		}
//		else
//		{
//			dribbler.setPower(0);
//		}
//
//		omni.setControlType(OmniWheel::TYPE_ROBOT_P_DOB);
//
//	}
//	else if(matcha.getReceiveState() == MatchaSerial::RECEIVE_STATE_TIMEOUT)
//	{
//
//		omniCmd.vel_limit = 0.0f;
//
//		omni.setControlType(OmniWheel::TYPE_WORLD_POSITION);
//
//	}

	syncLS_timestamp.end_count = htim13.Instance->CNT;

}


void Sanran::CAN_Rx_Callback(FDCAN_HandleTypeDef *hfdcan)
{
	canMotorIF.update_CAN_Rx();
}



void Sanran::UART_Rx_Callback(UART_HandleTypeDef *huart)
{
	//simulink.dataReceivedCallback(huart);
	//matcha.dataReceivedCallback(huart);
}


bool Sanran::display_result(bool(*setup_func)(), char* component_name)
{
	printf("\n Setting %s ... \n", component_name);
	bool boolStatus = setup_func();
	if(boolStatus){
		printf("\t\t\t\t[OK]\n");
	}else{
		printf("\t\t\t\t[ERROR]\n");
	}
	return boolStatus;
}


void Sanran::update_watchdog()
{
	if(watchdog_enable)
	{
		if(watchdog_CAN_count >= watchdog_CAN_threshold)
		{
			power.disableSupply();
			HAL_NVIC_SystemReset();
		}
		if(watchdog_UART_count >= watchdog_UART_threshold)
		{
			power.disableSupply();
			HAL_NVIC_SystemReset();
		}
		watchdog_CAN_count++;
		watchdog_UART_count++;
	}
	else
	{
		watchdog_CAN_count = 0;
		watchdog_UART_count = 0;
	}

	if(omni.get_last_error_status() == OmniWheel::ERROR_NONE)
	{
		watchdog_CAN_count = 0;
	}
	if(matcha.getReceiveState() != MatchaSerial::RECEIVE_STATE_TIMEOUT)
	{
		watchdog_UART_count = 0;
	}
}


void Sanran::update_dump()
{

	dump.setValue( 0, timeElapsed_hs_count * 0.001f);

	dump.setValue( 1, canMotorIF.motor[0].get_Iq_ref() );
	dump.setValue( 2, canMotorIF.motor[0].get_Iq() );
	dump.setValue( 3, canMotorIF.motor[0].get_omega() );
	dump.setValue( 4, canMotorIF.motor[0].get_theta() );

	dump.setValue( 5, canMotorIF.motor[1].get_Iq_ref() );
	dump.setValue( 6, canMotorIF.motor[1].get_Iq() );
	dump.setValue( 7, canMotorIF.motor[1].get_omega() );
	dump.setValue( 8, canMotorIF.motor[1].get_theta() );

	dump.setValue( 9, canMotorIF.motor[2].get_Iq_ref() );
	dump.setValue(10, canMotorIF.motor[2].get_Iq() );
	dump.setValue(11, canMotorIF.motor[2].get_omega() );
	dump.setValue(12, canMotorIF.motor[2].get_theta() );

	dump.setValue(13, canMotorIF.motor[3].get_Iq_ref() );
	dump.setValue(14, canMotorIF.motor[3].get_Iq() );
	dump.setValue(15, canMotorIF.motor[3].get_omega() );
	dump.setValue(16, canMotorIF.motor[3].get_theta() );

	dump.setValue(17, omni.m_cmd.omega_w[0]);
	dump.setValue(18, omni.m_cmd.omega_w[1]);
	dump.setValue(19, omni.m_cmd.omega_w[2]);
	dump.setValue(20, omni.m_cmd.omega_w[3]);

	dump.setValue(21, omni.m_cmd.robot_vel_x);
	dump.setValue(22, omni.m_cmd.robot_vel_y);
	dump.setValue(23, omni.m_cmd.robot_omega);

	dump.setValue(24, omni.m_cmd.world_x);
	dump.setValue(25, omni.m_cmd.world_y);
	dump.setValue(26, omni.m_cmd.world_theta);

	dump.setValue(27, omni.m_cmd.world_vel_x);
	dump.setValue(28, omni.m_cmd.world_vel_y);
	dump.setValue(29, omni.m_cmd.world_omega);

	dump.setValue(30, omni.m_robotState.world_x);
	dump.setValue(31, omni.m_robotState.world_y);
	dump.setValue(32, omni.m_robotState.world_theta);

	dump.setValue(33, matcha.normal_cmd.fb_x);
	dump.setValue(34, matcha.normal_cmd.fb_y);
	dump.setValue(35, matcha.normal_cmd.fb_theta);
	dump.setValue(36, matcha.normal_cmd.fb_timestamp * 1E-3);

	dump.send();


}


void Sanran::Error_Handler()
{
	buzzer.setNoteNumber(76+12);
	while(1){
		buzzer.on();
		delay_ms(100);
		buzzer.off();
		delay_ms(100);
	}

}




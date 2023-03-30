

#ifndef _OMNI_WHEEL_HPP_
#define _OMNI_WHEEL_HPP_


#include "controlLib.hpp"
#include "canMotor.hpp"



class OmniWheel
{

public:

	typedef enum
	{
		TYPE_INDEP_P,
		TYPE_INDEP_P_DOB,
		TYPE_ROBOT_P_DOB,
		TYPE_WORLD_P_DOB,
		TYPE_WORLD_POSITION,
	}ControlType_t;

	typedef enum
	{
		ERROR_NONE,
		ERROR_CAN_PACKET_LOSS
	}ErrorStatus_t;

	typedef struct
	{
		float world_x;
		float world_y;
		float world_theta;
		float robot_vel_x;
		float robot_vel_y;
		float robot_omega;
		float world_vel_x;
		float world_vel_y;
		float world_omega;
		float accel_x;
		float accel_y;
		float accel_theta;
		float omega_w[4];
		float vel_limit;
		float omega_limit;
	}Cmd_t;

	typedef struct{
		float Jmn; /* Nominal wheel inertia */
		float g_dis; /* Disturbance observer bandwidth */
		float Ktn[4]; /* Nominal torque constant */
		float Kp; /* Speed control gain */
		float Ts; /* Sampling time */
		float Iq_limit[4]; /* Rated current value */

		float wheel_pos_r[4];
		float wheel_pos_theta_deg[4];
		float wheel_r[4];
	}Param_t;

	typedef struct{
		float odometry_dx;
		float odometry_dy;
		float odometry_dtheta;
		float world_x;
		float world_y;
		float world_theta;
	}RobotState_t;

	typedef struct{
		float theta_res;
		float theta_res_prev;
		float omega_res;
		float Iq_res;
	}WheelState_t;

	OmniWheel(ControlType_t type, CanMotorIF *canMotorIF, Param_t *param);

	OmniWheel(ControlType_t type, CanMotorIF *canMotorIF);

	bool setup();

	void setControlType(ControlType_t type){ m_type = type; }

	ErrorStatus_t update(Cmd_t *cmd);

	void correctAngle(float trueAngle)
	{
		m_robotState.world_theta = trueAngle;
	}

	void correctPosition(float x, float y, float theta)
	{
		m_robotState.world_x = x;
		m_robotState.world_y = y;
		m_robotState.world_theta = theta;
	}

	RobotState_t m_robotState;

	WheelState_t m_wheelState[4];


	ControlType_t get_controlType(){return m_type;}

	ErrorStatus_t get_last_error_status(){return last_error_status;}

	Cmd_t m_cmd;


private:

	void calcKinematics();

	void updateOdometry();


	bool firstSampleFlag;

	ErrorStatus_t last_error_status;

	ControlType_t m_type;

	CanMotorIF *m_canMotorIF;

	PI_Controller position_pi_x;
	PI_Controller position_pi_y;
	PI_Controller position_pi_theta;

	DOB dob[4];

	Param_t m_param;

	float m_convMat_robot2motor[4][3];
	float m_convMat_motor2robot[3][4];


};



#endif /* _OMNI_WHEEL_HPP_ */


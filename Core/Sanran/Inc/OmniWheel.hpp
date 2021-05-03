

#ifndef _OMNI_WHEEL_HPP_
#define _OMNI_WHEEL_HPP_


#include "canMotor.hpp"

#include "controlLib.hpp"


class OmniWheel
{

public:

	typedef enum
	{
		TYPE_INDEP_P,
		TYPE_INDEP_P_DOB,
		TYPE_P_DOB,
	}ControlType_t;

	typedef struct
	{
		float vel_x;
		float vel_y;
		float omega;
		float accel_x;
		float accel_y;
		float accel_theta;
		float omega_w[4];
	}Cmd_t;

	typedef struct{
		float Jmn; /* Nominal wheel inertia */
		float g_dis; /* Disturbance observer bandwidth */
		float Ktn; /* Nominal torque constant */
		float Kp; /* Speed control gain */
		float Ts; /* Sampling time */

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

	void update(Cmd_t *cmd);

	void correctAngle(float trueAngle)
	{
		m_robotState.world_theta = trueAngle;
	}

	RobotState_t m_robotState;

	WheelState_t m_wheelState[4];


private:

	void commonInit();

	void calcKinematics();

	void updateOdometry();


	bool firstSampleFlag;


	ControlType_t m_type;

	CanMotorIF *m_canMotorIF;

	DOB dob[4];

	Cmd_t m_cmd;

	Param_t m_param;

	float m_convMat_robot2motor[4][3];
	float m_convMat_motor2robot[3][4];


};



#endif /* _OMNI_WHEEL_HPP_ */


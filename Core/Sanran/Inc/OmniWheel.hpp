

#ifndef _OMNI_WHEEL_HPP_
#define _OMNI_WHEEL_HPP_


#include "canMotor.hpp"


class OmniWheel
{

public:

	typedef enum
	{
		TYPE_P_CONTROL,
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
	}Param_t;

	OmniWheel(ControlType_t type, CanMotorIF *canMotorIF, Param_t *param);

	OmniWheel(ControlType_t type, CanMotorIF *canMotorIF);

	void update(Cmd_t *cmd);

private:

	ControlType_t m_type;

	CanMotorIF *m_canMotorIF;

	Cmd_t m_cmd;

	Param_t m_param;


};



#endif /* _OMNI_WHEEL_HPP_ */


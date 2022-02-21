

#include "OmniWheel.hpp"

#include <math.h>

#include <stdio.h>



OmniWheel::OmniWheel(ControlType_t type, CanMotorIF *canMotorIF, Param_t *param) : m_type(type), m_canMotorIF(canMotorIF), m_param(*param)
{

}

OmniWheel::OmniWheel(ControlType_t type, CanMotorIF *canMotorIF) :
		m_type(type),
		m_canMotorIF(canMotorIF)
{

	m_param.Jmn = 5.2E-5;
	m_param.Kp = 0.2;
	m_param.Ktn = (60.0f / (320 * 2 * M_PI));
	m_param.Ts = 1E-3;
	m_param.g_dis = 30;

	for(int i = 0; i < 4; i++)
	{
		m_param.wheel_pos_r[i] = 81E-3;//74.7E-3;
		m_param.wheel_r[i] = 55E-3 / 2.0f;
	}

	m_param.wheel_pos_theta_deg[0] = 60;
	m_param.wheel_pos_theta_deg[1] = 135;
	m_param.wheel_pos_theta_deg[2] = -135;
	m_param.wheel_pos_theta_deg[3] = -60;

}




/**
 * @fn void setup()
 * @brief Initial calculation
 *
 */
void OmniWheel::setup()
{


	calcKinematics();

	position_pi_x.setParam(3.0, 0.0, 1E-3);
	position_pi_y.setParam(3.0, 0.0, 1E-3);
	position_pi_theta.setParam(2.0, 0.0, 1E-3);

	for(int i = 0; i < 4; i++)
		dob[i].setParam(m_param.Ktn, m_param.Jmn, m_param.g_dis, m_param.Ts);


	for(int ch = 0; ch < 4; ch++)
		m_canMotorIF->motor[ch].set_Iq_ref(0.0f);

	m_robotState.world_x = 0.0f;
	m_robotState.world_y = 0.0f;
	m_robotState.world_theta = 0.0f;

	firstSampleFlag = true;

}





void OmniWheel::update(Cmd_t *cmd)
{
	float theta_error;

	float world_vx_ref = 0.0;
	float world_vy_ref = 0.0;
	float world_omega_ref = 0.0;

	float world_vx_ref_lim = 0.0;
	float world_vy_ref_lim = 0.0;
	float world_omega_ref_lim = 0.0;

	const float Vmax = 1.0;
	const float Omega_max = 30.0;

	if(!m_canMotorIF->motor[0].resIsUpdated() || !m_canMotorIF->motor[1].resIsUpdated() || !m_canMotorIF->motor[2].resIsUpdated() || !m_canMotorIF->motor[3].resIsUpdated())
	{
		for(int ch = 0; ch < 4; ch++)
		{
			m_canMotorIF->motor[ch].set_Iq_ref(0.0f);
			m_canMotorIF->motor[ch].clearUpdateFlag();
		}

		m_canMotorIF->send_Iq_ref();

		return;
	}

	for(int ch = 0; ch < 4; ch++)
	{
		m_canMotorIF->motor[ch].clearUpdateFlag();
	}


	// update wheel state
	for(int ch = 0; ch < 4; ch++)
	{
		m_wheelState[ch].theta_res_prev = m_wheelState[ch].theta_res;
		m_wheelState[ch].theta_res = m_canMotorIF->motor[ch].get_theta();
		m_wheelState[ch].Iq_res = m_canMotorIF->motor[ch].get_Iq();
		float delta_theta = m_wheelState[ch].theta_res - m_wheelState[ch].theta_res_prev;
		if(delta_theta >= M_PI) delta_theta -= 2 * M_PI;
		else if(delta_theta <= -M_PI) delta_theta += 2 * M_PI;
		m_wheelState[ch].omega_res = delta_theta / m_param.Ts;
	}

	// First sample process
	if(firstSampleFlag)
	{
		for(int ch = 0; ch < 4; ch++)
		{
			m_wheelState[ch].theta_res_prev = m_wheelState[ch].theta_res;
			m_wheelState[ch].omega_res = 0.0f;
		}

		firstSampleFlag = false;
	}

	updateOdometry();


	// Robot velocity control
	switch(m_type)
	{
	case TYPE_INDEP_P:
		for(int i = 0; i < 4; i++)
		{
			m_cmd.omega_w[i] = cmd->omega_w[i];
			float Iq_ref = m_param.Kp * (m_cmd.omega_w[i] - m_canMotorIF->motor[i].get_omega());
			if(Iq_ref < -15.0) Iq_ref = -15.0;
			if(Iq_ref > 15.0) Iq_ref = 15.0;
			m_canMotorIF->motor[i].set_Iq_ref(Iq_ref);
		}
		break;

	case TYPE_INDEP_P_DOB:
		for(int i = 0; i < 4; i++)
		{
			m_cmd.omega_w[i] = cmd->omega_w[i];
			float error = m_cmd.omega_w[i] - m_canMotorIF->motor[i].get_omega();
			float estTorque = dob[i].update(m_canMotorIF->motor[i].get_Iq_ref(), m_canMotorIF->motor[i].get_omega());
			float Iq_ref = m_param.Kp * error + estTorque / m_param.Ktn;
			if(Iq_ref < -15.0) Iq_ref = -15.0;
			if(Iq_ref > 15.0) Iq_ref = 15.0;
			m_canMotorIF->motor[i].set_Iq_ref(Iq_ref);
		}
		break;

	case TYPE_ROBOT_P_DOB:

		for(int i = 0; i < 4; i++)
		{
			m_cmd.omega_w[i] =
					m_convMat_robot2motor[i][0] * cmd->robot_vel_x +
					m_convMat_robot2motor[i][1] * cmd->robot_vel_y +
					m_convMat_robot2motor[i][2] * cmd->robot_omega;
			float error = m_cmd.omega_w[i] - m_canMotorIF->motor[i].get_omega();
			float estTorque = dob[i].update(m_canMotorIF->motor[i].get_Iq_ref(), m_canMotorIF->motor[i].get_omega());
			float Iq_ref = m_param.Kp * error + estTorque / m_param.Ktn;
			if(Iq_ref < -15.0) Iq_ref = -15.0;
			if(Iq_ref > 15.0) Iq_ref = 15.0;
			m_canMotorIF->motor[i].set_Iq_ref(Iq_ref);
		}
		break;

	case TYPE_WORLD_P_DOB:
		m_cmd.robot_vel_x = cmd->world_vel_x * cos(m_robotState.world_theta) + cmd->world_vel_y * sin(m_robotState.world_theta);
		m_cmd.robot_vel_y = cmd->world_vel_x * -sin(m_robotState.world_theta) + cmd->world_vel_y * cos(m_robotState.world_theta);
		m_cmd.world_theta += cmd->world_omega * m_param.Ts;
		theta_error = m_cmd.world_theta - m_robotState.world_theta;
		if(theta_error < -M_PI) theta_error += 2 * M_PI;
		else if(theta_error > M_PI) theta_error -= 2 * M_PI;
		for(int i = 0; i < 4; i++)
		{
			m_cmd.omega_w[i] =
					m_convMat_robot2motor[i][0] * m_cmd.robot_vel_x +
					m_convMat_robot2motor[i][1] * m_cmd.robot_vel_y +
					m_convMat_robot2motor[i][2] * (cmd->robot_omega + 5.0 * theta_error);
			float error = m_cmd.omega_w[i] - m_canMotorIF->motor[i].get_omega();
			float estTorque = dob[i].update(m_canMotorIF->motor[i].get_Iq_ref(), m_canMotorIF->motor[i].get_omega());
			float Iq_ref = m_param.Kp * error + estTorque / m_param.Ktn;
			if(Iq_ref < -15.0) Iq_ref = -15.0;
			if(Iq_ref > 15.0) Iq_ref = 15.0;
			m_canMotorIF->motor[i].set_Iq_ref(Iq_ref);
		}
		break;

	case TYPE_WORLD_POSITION:

		m_cmd.world_vel_x = cmd->world_vel_x;
		m_cmd.world_vel_y = cmd->world_vel_y;
		m_cmd.world_omega = cmd->world_omega;
		m_cmd.world_x = cmd->world_x;
		m_cmd.world_y = cmd->world_y;
		m_cmd.world_theta = cmd->world_theta;

		theta_error = m_cmd.world_theta - m_robotState.world_theta;
		if(theta_error < -M_PI) theta_error += 2 * M_PI;
		else if(theta_error > M_PI) theta_error -= 2 * M_PI;

		world_vx_ref = position_pi_x.update(m_cmd.world_x - m_robotState.world_x) + m_cmd.world_vel_x;
		world_vy_ref = position_pi_x.update(m_cmd.world_y - m_robotState.world_y) + m_cmd.world_vel_y;
		world_omega_ref = position_pi_theta.update(theta_error) + m_cmd.world_omega;

		world_vx_ref_lim = limitter(world_vx_ref, -Vmax, Vmax);
		world_vy_ref_lim = limitter(world_vy_ref, -Vmax, Vmax);
		world_omega_ref_lim = limitter(world_omega_ref, -Omega_max, Omega_max);

		position_pi_x.set_limitError(world_vx_ref - world_vx_ref_lim);
		position_pi_y.set_limitError(world_vy_ref - world_vy_ref_lim);
		position_pi_theta.set_limitError(world_omega_ref - world_omega_ref_lim);

		m_cmd.robot_vel_x = world_vx_ref_lim * cos(m_robotState.world_theta) + world_vy_ref_lim * sin(m_robotState.world_theta);
		m_cmd.robot_vel_y = world_vx_ref_lim * -sin(m_robotState.world_theta) + world_vy_ref_lim * cos(m_robotState.world_theta);
		m_cmd.robot_omega = world_omega_ref_lim;
		for(int i = 0; i < 4; i++)
		{
			m_cmd.omega_w[i] =
					m_convMat_robot2motor[i][0] * m_cmd.robot_vel_x +
					m_convMat_robot2motor[i][1] * m_cmd.robot_vel_y +
					m_convMat_robot2motor[i][2] * m_cmd.robot_omega;
			float error = m_cmd.omega_w[i] - m_wheelState[i].omega_res;
			float estTorque = dob[i].update(m_canMotorIF->motor[i].get_Iq_ref(), m_wheelState[i].omega_res);
			float Iq_ref = m_param.Kp * error + estTorque / m_param.Ktn;
			if(Iq_ref < -15.0) Iq_ref = -15.0;
			if(Iq_ref > 15.0) Iq_ref = 15.0;
			m_canMotorIF->motor[i].set_Iq_ref(Iq_ref);
		}
		break;

	default:
		break;
	}

	m_canMotorIF->send_Iq_ref();

}





void OmniWheel::calcKinematics()
{

	// Inverse kinematics
	for(int ch = 0; ch < 4; ch++)
	{
		m_convMat_robot2motor[ch][0] = cosf(m_param.wheel_pos_theta_deg[ch] * M_PI / 180.0f) / m_param.wheel_r[ch];
		m_convMat_robot2motor[ch][1] = sinf(m_param.wheel_pos_theta_deg[ch] * M_PI / 180.0f) / m_param.wheel_r[ch];
		m_convMat_robot2motor[ch][2] = -m_param.wheel_pos_r[ch] / m_param.wheel_r[ch];
	}

	// forward kinematics
	float den_Vx, den_Vy, den_omega;

	den_Vx = cos(m_param.wheel_pos_theta_deg[1] * M_PI / 180.0f) - cos(m_param.wheel_pos_theta_deg[0] * M_PI / 180.0f);
	den_Vy = pow(sin(m_param.wheel_pos_theta_deg[1] * M_PI / 180.0f), 2) + pow(sin(m_param.wheel_pos_theta_deg[0] * M_PI / 180.0f), 2);
	den_omega = m_param.wheel_pos_r[0] * (cos(m_param.wheel_pos_theta_deg[1] * M_PI / 180.0f) - cos(m_param.wheel_pos_theta_deg[0] * M_PI / 180.0f));

	m_convMat_motor2robot[0][0] = -0.5 * m_param.wheel_r[0] / den_Vx;
	m_convMat_motor2robot[0][1] = 0.5 * m_param.wheel_r[0] / den_Vx;
	m_convMat_motor2robot[0][2] = 0.5 * m_param.wheel_r[0] / den_Vx;
	m_convMat_motor2robot[0][3] = -0.5 * m_param.wheel_r[0] / den_Vx;

	m_convMat_motor2robot[1][0] = 0.5 * m_param.wheel_r[0] * sin(m_param.wheel_pos_theta_deg[0] * M_PI / 180.0f) / den_Vy;
	m_convMat_motor2robot[1][1] = 0.5 * m_param.wheel_r[0] * sin(m_param.wheel_pos_theta_deg[1] * M_PI / 180.0f) / den_Vy;
	m_convMat_motor2robot[1][2] = -0.5 * m_param.wheel_r[0] * sin(m_param.wheel_pos_theta_deg[1] * M_PI / 180.0f) / den_Vy;
	m_convMat_motor2robot[1][3] = -0.5 * m_param.wheel_r[0] * sin(m_param.wheel_pos_theta_deg[0] * M_PI / 180.0f) / den_Vy;

	m_convMat_motor2robot[2][0] = -0.5 * m_param.wheel_r[0] * cos(m_param.wheel_pos_theta_deg[1] * M_PI / 180.0f) / den_omega;
	m_convMat_motor2robot[2][1] = 0.5 * m_param.wheel_r[0] * cos(m_param.wheel_pos_theta_deg[0] * M_PI / 180.0f) / den_omega;
	m_convMat_motor2robot[2][2] = 0.5 * m_param.wheel_r[0] * cos(m_param.wheel_pos_theta_deg[0] * M_PI / 180.0f) / den_omega;
	m_convMat_motor2robot[2][3] = -0.5 * m_param.wheel_r[0] * cos(m_param.wheel_pos_theta_deg[1] * M_PI / 180.0f) / den_omega;


	printf("\nm_convMat_motor2robot[][] = \n");
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 4; j++)
		{
			printf("%2.5e\t", m_convMat_motor2robot[i][j]);
		}
		printf("\n");
	}




}

void OmniWheel::updateOdometry()
{

	float delta_theta_res[4];

	float delta_x = 0.0f;
	float delta_y = 0.0f;
	float delta_theta = 0.0f;


	for(int ch = 0; ch < 4; ch++)
	{
		delta_theta_res[ch] = m_wheelState[ch].theta_res - m_wheelState[ch].theta_res_prev;

		m_wheelState[ch].theta_res_prev = m_wheelState[ch].theta_res;

		if(delta_theta_res[ch] < -M_PI) delta_theta_res[ch] += 2 * M_PI;
		else if(delta_theta_res[ch] > M_PI) delta_theta_res[ch] -= 2 * M_PI;

		delta_x += m_convMat_motor2robot[0][ch] * delta_theta_res[ch];
		delta_y += m_convMat_motor2robot[1][ch] * delta_theta_res[ch];
		delta_theta += m_convMat_motor2robot[2][ch] * delta_theta_res[ch];
	}


	float cos_theta = cosf(m_robotState.world_theta);
	float sin_theta = sinf(m_robotState.world_theta);

	float cosc_wt = delta_theta * 0.5f;
	float sinc_wt = 1.0f - delta_theta * delta_theta * 0.16666666666666666666666666666667f;


	m_robotState.odometry_dx = delta_x * (cos_theta * sinc_wt - sin_theta * cosc_wt) +
			delta_y * (-cos_theta * cosc_wt - sin_theta * sinc_wt);
	m_robotState.odometry_dy = delta_x * (cos_theta * cosc_wt + sin_theta * sinc_wt) +
			delta_y * (cos_theta * sinc_wt - sin_theta * cosc_wt);
	m_robotState.odometry_dtheta = delta_theta;

	m_robotState.world_x += m_robotState.odometry_dx;
	m_robotState.world_y += m_robotState.odometry_dy;
	m_robotState.world_theta += m_robotState.odometry_dtheta;

}





/**
 * @file usrcmd.c
 * @author CuBeatSystems
 * @author Shinichiro Nakamura
 * @copyright
 * ===============================================================
 * Natural Tiny Shell (NT-Shell) Version 0.3.1
 * ===============================================================
 * Copyright (c) 2010-2016 Shinichiro Nakamura
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include "ntopt.h"
#include "ntlibc.h"

#include "ntshell.h"

#include <stdio.h>
#include <stdlib.h>

#include "main.h"

#include "Sanran.hpp"


extern UART_HandleTypeDef huart6;

#define UartHandler (huart6)

#define uart_puts(str) puts(str)


typedef int (*USRCMDFUNC)(int argc, char **argv);


static int ntshell_serial_read(char *buf, int cnt, void *extobj);
static int ntshell_serial_write(const char *buf, int cnt, void *extobj);
static int ntshell_callback(const char *text, void *extobj);

void uart_putc(char c);
static char ntshell_serial_getc_timeout(int timeout_ms);

int usrcmd_execute(const char *text);

static int usrcmd_ntopt_callback(int argc, char **argv, void *extobj);
static int usrcmd_help(int argc, char **argv);
static int usrcmd_info(int argc, char **argv);
static int usrcmd_view(int argc, char **argv);
static int usrcmd_cls(int argc, char **argv);
static int usrcmd_kick(int argc, char **argv);
static int usrcmd_dribble(int argc, char **argv);
static int usrcmd_watchdog(int argc, char **argv);

typedef struct {
    char *cmd;
    char *desc;
    USRCMDFUNC func;
}cmd_table_t;

static const cmd_table_t cmdlist[] = {
    { "help", "This is a description text string for help command.", usrcmd_help },
    { "info", "This is a description text string for info command.", usrcmd_info },
    { "view", "This is a description text string for info command.", usrcmd_view },
    { "cls", "Clear display", usrcmd_cls },
    { "kick", "About kicker", usrcmd_kick },
    { "dribble", "Dribbler control", usrcmd_dribble },
    { "watchdog", "WatchDog setting", usrcmd_watchdog },
};





extern Sanran sanran;




void ntshell_usr_init(ntshell_t *p)
{

	void *extobj = 0;
	ntshell_init(p, ntshell_serial_read, ntshell_serial_write, ntshell_callback, extobj);
	ntshell_set_prompt(p, "ntshell>");

}

static int ntshell_serial_read(char *buf, int cnt, void *extobj)
{

	while(HAL_UART_Receive(&UartHandler, (uint8_t*)buf, cnt, 1000) != HAL_OK);

	return cnt;
}

static int ntshell_serial_write(const char *buf, int cnt, void *extobj)
{

	while(HAL_UART_Transmit(&UartHandler, (uint8_t*)buf, cnt, 1000) != HAL_OK);

	return cnt;
}

static int ntshell_callback(const char *text, void *extobj)
{

#if 0
    /*
     * This is a really simple example codes for the callback function.
     */
    uart_puts("USERINPUT[");
    uart_puts(text);
    uart_puts("]\r\n");
#else
    /*
     * This is a complete example for a real embedded application.
     */
    usrcmd_execute(text);
#endif

    return 0;

}

void uart_putc(char c)
{
	void *extobj = 0;
	ntshell_serial_write(&c, 1, extobj);
}

static char ntshell_serial_getc_timeout(int timeout_ms)
{
	char c;

	HAL_UART_Receive(&UartHandler, (uint8_t*)(&c), 1, timeout_ms);

	return c;
}


static int checkSuspens()
{
	char c;
	c = ntshell_serial_getc_timeout(1);
	if(c == 0x03)
	{
		puts("\r\n^C\r\n");
		return 1;
	}
	return 0;
}


/*
 * User command
 */



int usrcmd_execute(const char *text)
{
    return ntopt_parse(text, usrcmd_ntopt_callback, 0);
}

static int usrcmd_ntopt_callback(int argc, char **argv, void *extobj)
{
    if (argc == 0) {
        return 0;
    }
    const cmd_table_t *p = &cmdlist[0];
    for (uint i = 0; i < sizeof(cmdlist) / sizeof(cmdlist[0]); i++) {
        if (ntlibc_strcmp((const char *)argv[0], p->cmd) == 0) {
            return p->func(argc, argv);
        }
        p++;
    }
    uart_puts("Unknown command found.\r\n");
    return 0;
}

static int usrcmd_help(int argc, char **argv)
{
    const cmd_table_t *p = &cmdlist[0];
    for (uint i = 0; i < sizeof(cmdlist) / sizeof(cmdlist[0]); i++) {
        uart_puts(p->cmd);
        uart_puts("\t:");
        uart_puts(p->desc);
        uart_puts("\r\n");
        p++;
    }
    return 0;
}

static int usrcmd_info(int argc, char **argv)
{
    if (argc != 2) {
        uart_puts("info sys\r\n");
        uart_puts("info ver\r\n");
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "sys") == 0) {
        uart_puts("NXP LPC824 Monitor\r\n");
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "ver") == 0) {
        uart_puts("Version 0.0.0\r\n");
        return 0;
    }
    uart_puts("Unknown sub command found\r\n");
    return -1;
}

static int usrcmd_view(int argc, char **argv)
{

	if (argc != 2) {
		uart_puts("view resource\r\n");
		uart_puts("view motor\r\n");
		uart_puts("view gyro\r\n");
		uart_puts("view command\r\n");
		uart_puts("view kicker\r\n");
		uart_puts("view dribbler\r\n");
		uart_puts("view rxframe\r\n");
		uart_puts("view rxbuf\r\n");
		return 0;
	}

	if(ntlibc_strcmp(argv[1], "resource") == 0)
	{
		// View of interrupt resource

		printf("\r\n");

		while(1)
		{
			delay_ms(100);

			Sanran::Sync_loop_timestamp_t LS_timestamp;
			LS_timestamp = sanran.syncLS_timestamp;

			Sanran::Sync_loop_timestamp_t HS_timestamp;
			HS_timestamp = sanran.syncHS_timestamp;

			printf("         |  start |   end  | period | percent \r\n");
			printf("---------+--------+--------+--------+-------- \r\n");
			//     "*********| ****** | ****** | ****** | ******
			printf("HS Cycle | %6ld | %6ld | %6ld | %6ld%% \r\n",
					HS_timestamp.start_count, HS_timestamp.end_count, sanran.htim_HS_cycle->Init.Period,
					(HS_timestamp.end_count - HS_timestamp.start_count) * 100 / sanran.htim_HS_cycle->Init.Period);
			printf("LS Cycle | %6ld | %6ld | %6ld | %6ld%% ",
					LS_timestamp.start_count, LS_timestamp.end_count, sanran.htim_LS_cycle->Init.Period,
					(LS_timestamp.end_count - LS_timestamp.start_count) * 100 / sanran.htim_LS_cycle->Init.Period);

			printf("\r\n");

			if(checkSuspens()) break;


			uart_puts("\e[5A");

		}

	}
	else if(ntlibc_strcmp(argv[1], "motor") == 0)
	{
		// View of motor status

		printf("\r\n");

		while(1)
		{
			delay_ms(100);

			printf("        | Iq_ref [A] | Iq [A] | Angle [deg] | Speed [rad/s] \r\n");
			printf("--------+------------+--------+-------------+-------------- \r\n");
			for(int i = 0; i < 4; i++)
			{
				printf("Motor %d ", i + 1);
				printf("| %10.2f ", sanran.canMotorIF.motor[i].get_Iq_ref());
				printf("| %6.2f ", sanran.canMotorIF.motor[i].get_Iq());
				printf("| %11.2f ", sanran.canMotorIF.motor[i].get_theta() * 180 / M_PI);
				printf("| %13.1f ", sanran.canMotorIF.motor[i].get_omega());
				printf("\r\n");
			}

			if(checkSuspens()) break;

			uart_puts("\e[7A");

		}

	}
	else if(ntlibc_strcmp(argv[1], "gyro") == 0)
	{
		// View of gyro status

		printf("\r\n");

		while(1)
		{
			delay_ms(100);

			printf("         |  Roll [deg] | Pitch [deg] |   Yaw [deg] \r\n");
			printf("---------+-------------+-------------+------------ \r\n");
			//     "***********| ****** | ****** | ****** | ******
			printf("Gyro     ");

			printf("| %11.2f ", sanran.bno055.get_IMU_roll() * 180 / M_PI);
			printf("| %11.2f ", sanran.bno055.get_IMU_pitch() * 180 / M_PI);
			printf("| %11.2f ", sanran.bno055.get_IMU_yaw() * 180 / M_PI);

			printf("\r\n");

			if(checkSuspens()) break;


			uart_puts("\e[4A");
		}

	}
	else if(ntlibc_strcmp(argv[1], "command") == 0)
	{
		// View of command from matcha

		printf("\r\n");

		while(1)
		{
			delay_ms(10);

			printf(" Normal  |   pos_x [m] |   pos_y [m] | theta[deg.] | time_stamp[ms] | Vision Error |\r\n");
			printf("---------+-------------+-------------+-------------+----------------+--------------| \r\n");

			printf("FeedBack ");
			printf("| %11.2f ", sanran.matcha.normal_cmd.fb_x);
			printf("| %11.2f ", sanran.matcha.normal_cmd.fb_y);
			printf("| %11.2f ", sanran.matcha.normal_cmd.fb_theta / M_PI * 180.0f);
			printf("| %14d ", sanran.matcha.normal_cmd.fb_timestamp);
			printf("| %12d ", sanran.matcha.normal_cmd.vision_error);

			printf("\r\n\r\n");

			printf(" Normal  |   pos_x [m] |   pos_y [m] | theta[deg.] |\r\n");
			printf("---------+-------------+-------------+-------------| \r\n");

			printf("Command  ");
			printf("| %11.2f ", sanran.matcha.normal_cmd.cmd_x);
			printf("| %11.2f ", sanran.matcha.normal_cmd.cmd_y);
			printf("| %11.2f ", sanran.matcha.normal_cmd.cmd_theta / M_PI * 180.0f);

			printf("\r\n\r\n");

			printf(" Normal  | Vel_x [m/s] | Vel_y [m/s] | omega [rad/s] | Vel_limit [m/s] | Omega_limit [rad/s] | \r\n");
			printf("---------+-------------+-------------+---------------+-----------------+---------------------| \r\n");

			printf("Command  ");
			printf("| %11.2f ", sanran.matcha.normal_cmd.cmd_vx);
			printf("| %11.2f ", sanran.matcha.normal_cmd.cmd_vy);
			printf("| %13.2f ", sanran.matcha.normal_cmd.cmd_omega);
			printf("| %15.2f ", sanran.matcha.normal_cmd.vel_limit);
			printf("| %19.2f ", sanran.matcha.normal_cmd.omega_limit);

			printf("\r\n\r\n");

			printf(" Normal  | Dribble | Kick | Chip | DribblePower | KickPower | [ParseError] | [Receive State] |\r\n");
			printf("---------+---------+------+------+--------------+-----------+--------------+-----------------|\r\n");
			printf("Command  ");

			printf("| %7d ", sanran.matcha.normal_cmd.dribble);
			printf("| %4d ", sanran.matcha.normal_cmd.kick);
			printf("| %4d ", sanran.matcha.normal_cmd.chip);
			printf("| %12d ", sanran.matcha.normal_cmd.dribblePower);
			printf("| %9d ", sanran.matcha.normal_cmd.kickPower);
			printf("| %12d ", sanran.matcha.getPrevErrorCode());
			printf("| %15d ", sanran.matcha.getReceiveState());

			printf("\r\n\r\n");
			printf("==============================================================================================\r\n\r\n");

			printf(" Manual  | Vel_x [m/s] | Vel_y [m/s] | omega [rad/s] | \r\n");
			printf("---------+-------------+-------------+---------------| \r\n");

			printf("Command  ");
			printf("| %11.2f ", sanran.matcha.manual_cmd.cmd_vx);
			printf("| %11.2f ", sanran.matcha.manual_cmd.cmd_vy);
			printf("| %13.2f ", sanran.matcha.manual_cmd.cmd_omega);

			printf("\r\n\r\n");

			printf(" Manual  | Dribble | Kick | Chip | DribblePower | KickPower | [ParseError] | [Receive State] |\r\n");
			printf("---------+---------+------+------|--------------+-----------+--------------+-----------------|\r\n");
			printf("Command  ");

			printf("| %7d ", sanran.matcha.manual_cmd.dribble);
			printf("| %4d ", sanran.matcha.manual_cmd.kick);
			printf("| %4d ", sanran.matcha.manual_cmd.chip);
			printf("| %12d ", sanran.matcha.manual_cmd.dribblePower);
			printf("| %9d ", sanran.matcha.manual_cmd.kickPower);
			printf("| %12d ", sanran.matcha.getPrevErrorCode());
			printf("| %15d ", sanran.matcha.getReceiveState());

			printf("\r\n");

			if(checkSuspens()) break;

			uart_puts("\e[26A");
		}
	}
	else if(ntlibc_strcmp(argv[1], "kicker") == 0)
	{

		printf("\r\n");

		while(1)
		{
			delay_ms(10);

			printf("kickState = %d\r\n", sanran.kicker.kickState);

			if(checkSuspens()) break;

			uart_puts("\e[2A");

		}
	}
	else if(ntlibc_strcmp(argv[1], "dribbler") == 0)
	{

		printf("\r\n");

		while(1)
		{
			delay_ms(10);

			printf("Dribbler: \r\n");
			printf("PWM duty = %6f\r\n", sanran.dribbler.getDuty());
			printf("Ball sensor = %6f\r\n", sanran.ballSensor.read());

			if(checkSuspens()) break;

			uart_puts("\e[4A");

		}
	}
	else if(ntlibc_strcmp(argv[1], "rxframe") == 0)
	{

		printf("\r\n");

		while(1)
		{
			delay_ms(10);

			for(int i = 0; i < UART_BUF_SIZE; i++)
			{
				printf("%02x ", sanran.matcha.m_rxBytes[i]);
				//printf("%02x ", sanran.matcha.m_rxBuf[i]);
			}
			printf("\r\n");

			if(checkSuspens()) break;

		}
	}
	else if(ntlibc_strcmp(argv[1], "rxbuf") == 0)
	{

		printf("\r\n");

		while(1)
		{
			delay_ms(10);

			for(int i = 0; i < UART_BUF_SIZE; i++)
			{
				printf("%02x ", sanran.matcha.m_rxBuf[i]);
			}
			printf("\r\n");

			if(checkSuspens()) break;

		}
	}
	else
	{
		printf("Command Not Found.\r\n");
		return 0;
	}



	return 0;
}


static int usrcmd_cls(int argc, char **argv)
{
	uart_puts("\e[2J");
	uart_puts("\e[1;1H");

	return 0;
}


static int usrcmd_kick(int argc, char **argv)
{
	if (argc < 2) {
		uart_puts("kick state [stateNumber]\r\n");
		uart_puts("kick straight [on time (us)]\r\n");
		uart_puts("kick chip [on time (us)]\r\n");
		return 0;
	}
	if (ntlibc_strcmp(argv[1], "state") == 0 && argc == 3) {
		int state = atoi(argv[2]);
		if(state < 0 || state > 3)
		{
			printf("\"state\" must be [0-3].\r\n");
			return 0;
		}
		sanran.kicker.kickState = (Kicker::KickState_e)state;
		printf("state = [%d]\r\n", sanran.kicker.kickState);
		return 0;
	}
	else if (ntlibc_strcmp(argv[1], "straight") == 0 && argc == 3) {
		int width_us = atoi(argv[2]);
		if(width_us <= 0 || width_us > 65000)
		{
			printf("\"pulse_width_us\" must be [1-65000].\r\n");
			return 0;
		}
		sanran.kicker.kickStraight_width(width_us);
		return 0;
	}
	else if (ntlibc_strcmp(argv[1], "chip") == 0 && argc == 3) {
		int width_us = atoi(argv[2]);
		if(width_us <= 0 || width_us > 65000)
		{
			printf("\"pulse_width_us\" must be [1-65000].\r\n");
			return 0;
		}
		sanran.kicker.kickChip_width(width_us);
		return 0;
	}
	else
	{
		printf("Command Not Found.\r\n");
		return 0;
	}
	uart_puts("Unknown sub command found\r\n");
	return -1;
}



static int usrcmd_dribble(int argc, char **argv)
{

	if (argc < 2) {
		uart_puts("dribble set [duty (0.0 ~ 1.0)]\r\n");
		uart_puts("dribble setus [width (0 ~ 20000)]\r\n");
		return 0;
	}
	if (ntlibc_strcmp(argv[1], "set") == 0 && argc == 3) {
		float duty = atoff(argv[2]);
		if(duty < 0 || duty > 1.0)
		{
			printf("\"duty\" must be [0.0-1.0].\r\n");
			return 0;
		}
		sanran.dribbler.write(duty);
		printf("\r\nduty = %f \r\n", duty);
		return 0;
	}
	else if (ntlibc_strcmp(argv[1], "setus") == 0 && argc == 3) {
		uint32_t us = atoi(argv[2]);
		if(us < 0 || us > 20000)
		{
			printf("\"width\" must be [0-20000].\r\n");
			return 0;
		}
		sanran.dribbler.write_us(us);
		printf("\r\nwidth = %ld [us]\r\n", us);
		return 0;
	}
	else
	{
		printf("Command Not Found.\r\n");
		return 0;
	}
}

static int usrcmd_watchdog(int argc, char **argv)
{
	if (argc < 2) {
		uart_puts("watchdog enable\r\n");
		uart_puts("watchdog disable\r\n");
		return 0;
	}
	if (ntlibc_strcmp(argv[1], "enable") == 0 && argc == 2) {
		sanran.watchdog_enable = true;
		printf("\r\nwatchdog_enable = true \r\n");
		return 0;
	}
	else if (ntlibc_strcmp(argv[1], "disable") == 0 && argc == 2) {
		sanran.watchdog_enable = false;
		printf("\r\nwatchdog_enable = false \r\n");
		return 0;
	}
	else
	{
		printf("Command Not Found.\r\n");
		return 0;
	}
}






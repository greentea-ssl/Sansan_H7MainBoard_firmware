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

#include "main.h"

#include "Sanran.hpp"


extern UART_HandleTypeDef huart6;

#define UartHandler (huart6)

#define delay_ms(ms) HAL_Delay(ms)

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
    for (int i = 0; i < sizeof(cmdlist) / sizeof(cmdlist[0]); i++) {
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
    for (int i = 0; i < sizeof(cmdlist) / sizeof(cmdlist[0]); i++) {
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
		uart_puts("info sys\r\n");
		uart_puts("info ver\r\n");
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
			printf("HS Cycle | %6d | %6d | %6d | %6d%% \r\n",
					HS_timestamp.start_count, HS_timestamp.end_count, sanran.htim_HS_cycle->Init.Period,
					(HS_timestamp.end_count - HS_timestamp.start_count) * 100 / sanran.htim_HS_cycle->Init.Period);
			printf("LS Cycle | %6d | %6d | %6d | %6d%% ",
					LS_timestamp.start_count, LS_timestamp.end_count, sanran.htim_LS_cycle->Init.Period,
					(LS_timestamp.end_count - LS_timestamp.start_count) * 100 / sanran.htim_LS_cycle->Init.Period);

			printf("\r\n");

			if(checkSuspens()) break;


			uart_puts("\e[5A");

			//printf("start:%d,\t end:%d,\t period:%d\r\n", LS_timestamp.start_count, LS_timestamp.end_count, sanran.htim_LS_cycle->Init.Period);

			//printf("start:%d,\t end:%d,\t period:%d\r\n", HS_timestamp.start_count, HS_timestamp.end_count, sanran.htim_HS_cycle->Init.Period);

		}

	}
	else if(ntlibc_strcmp(argv[1], "motor") == 0)
	{
		// View of motor status

		printf("\r\n");

		while(1)
		{
			delay_ms(100);

			Sanran::Sync_loop_timestamp_t LS_timestamp;
			LS_timestamp = sanran.syncLS_timestamp;

			Sanran::Sync_loop_timestamp_t HS_timestamp;
			HS_timestamp = sanran.syncHS_timestamp;

			printf("              | Motor1 | Motor2 | Motor3 | Motor4 \r\n");
			printf("--------------+--------+--------+--------+-------- \r\n");
			//     "***********| ****** | ****** | ****** | ******
			//     "Iq_ref    [A] "
			//     "Iq        [A] "
			//     "Angle   [deg] "
			//     "Speed [rad/s] "
			printf("Iq_ref    [A] ");
			for(int i = 0; i < 4; i++)
			{
				printf("| %6.2f ", sanran.canMotorIF.motor[i].get_Iq_ref());
			}
			printf("\r\n");
			printf("Iq        [A] ");
			for(int i = 0; i < 4; i++)
			{
				printf("| %6.2f ", sanran.canMotorIF.motor[i].get_Iq());
			}
			printf("\r\n");
			printf("Angle   [deg] ");
			for(int i = 0; i < 4; i++)
			{
				printf("| %6.2f ", sanran.canMotorIF.motor[i].get_theta() * 180 / M_PI);
			}
			printf("\r\n");
			printf("Speed [rad/s] ");
			for(int i = 0; i < 4; i++)
			{
				printf("| %6.1f ", sanran.canMotorIF.motor[i].get_omega());
			}
			printf("\r\n");

			if(checkSuspens()) break;


			uart_puts("\e[7A");

			//printf("start:%d,\t end:%d,\t period:%d\r\n", LS_timestamp.start_count, LS_timestamp.end_count, sanran.htim_LS_cycle->Init.Period);

			//printf("start:%d,\t end:%d,\t period:%d\r\n", HS_timestamp.start_count, HS_timestamp.end_count, sanran.htim_HS_cycle->Init.Period);

		}

	}
	else
	{
		//printf("\e[%s", argv[1]);
	}



	return 0;
}


static int usrcmd_cls(int argc, char **argv)
{
	uart_puts("\e[2J");
	uart_puts("\e[1;1H");
}












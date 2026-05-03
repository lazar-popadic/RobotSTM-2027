/*
 * time.c
 *
 *  Created on: Nov 18, 2025
 *      Author: Lea
 */
#include "main.h"
#include "tim.h"

static uint32_t sys_time_ms = 0;
static uint8_t ctrl_psc = 10, ctrl_cnt = 0;
static double dt_ = 0.001;

void time_ISR()	// 1ms
{
	sys_time_ms++;
	update_odom(dt_);
	ctrl_cnt ++;
	if (ctrl_cnt >= ctrl_psc)
	{
		ctrl_cnt = 0;
		control_loop(dt_* ctrl_psc);
	}
	velocity_loop(dt_);
}

void time_start() {
	HAL_TIM_Base_Start_IT(&htim10);
}

void time_stop() {
	HAL_TIM_Base_Stop_IT(&htim10);
}

uint32_t get_time()
{
	return sys_time_ms;
}

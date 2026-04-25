/*
 * time.c
 *
 *  Created on: Nov 18, 2025
 *      Author: Lea
 */
#include "main.h"
#include "tim.h"

static uint32_t sys_time_ms = 0;

void time_ISR()
{
	sys_time_ms++;
	update_odom();
	control_loop();
}

void time_start() {
	HAL_TIM_Base_Start_IT(&htim10);
}

void time_stop() {
	HAL_TIM_Base_Stop_IT(&htim10);
}

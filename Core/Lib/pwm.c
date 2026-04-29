/*
 * pwm.c
 *
 *  Created on: Nov 25, 2025
 *      Author: Dell
 */

#include "main.h"

uint8_t pwm_on = 0;

void pwm_set_dc(TIM_HandleTypeDef *htim, uint32_t channel, int16_t duty_cycle) {
	__HAL_TIM_SET_COMPARE(htim, channel, duty_cycle);
}

void pwm_init() {
	if (!pwm_on) {
		HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
		pwm_on = 1;
	}
}

void pwm_kill() {
	if (pwm_on) {
		set_motor_r_dir(0);
		set_motor_l_dir(0);
		pwm_set_dc(&htim9, TIM_CHANNEL_1, 0);
		pwm_set_dc(&htim9, TIM_CHANNEL_2, 0);
		HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2);
		HAL_TIM_PWM_DeInit(&htim9);
		pwm_on = 0;
	}
}

void pwm_left(double vel, double max_vel) {
	int16_t pwm;
	int8_t dir = sign(vel);
	set_motor_l_dir(dir);
	pwm = clamp(fabs(vel) / max_vel * 1680, 0, 1680);
	pwm_set_dc(&htim9, TIM_CHANNEL_1, pwm);
}

void pwm_right(double vel, double max_vel) {
	int16_t pwm;
	int8_t dir = sign(vel);
	set_motor_r_dir(dir);
	pwm = clamp(fabs(vel) / max_vel * 1680, 0, 1680);
	pwm_set_dc(&htim9, TIM_CHANNEL_2, pwm);

}
void set_motor_l_dir(int8_t dir) {
	switch (dir) {
	case -1:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
		break;
	case 0:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
		break;
	}
}

void set_motor_r_dir(int8_t dir) {
	switch (dir) {
	case 1:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
		break;
	case -1:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
		break;
	case 0:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
		break;
	}
}

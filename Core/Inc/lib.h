/*
 * lib.h
 *
 *  Created on: Aug 13, 2024
 *      Author: lazar
 */

#ifndef LIB_LIB_H_
#define LIB_LIB_H_

#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "tim.h"
#include "gpio.h"

#define MAX_NUM_OF_MOVES	10

typedef struct st_pose_2D_stamped {
	double x;
	double y;
	double phi;
	double v;
	double w;
	uint32_t time_ms;
} pose_2D_stamped;

typedef struct st_move {
	// automatically set
	uint16_t id;
	uint8_t index;
	int8_t type;
	uint32_t time_start;
	uint32_t time_end;
	// input parameters
	double x;
	double y;
	double phi;
	int8_t direction;
	double v_max;
	double w_max;
	// feedback
	int8_t status;
} move;

typedef struct st_action {
	// input
	uint16_t id;
	uint8_t num_of_moves;
	move *moves_ptr;
	// output
	int8_t status;
} action;

typedef struct st_pid {
	double p;
	double i;
	double d;
	double lmt;
	double p_ctrl;
	double i_ctrl;
	double d_ctrl;
	double ctrl;
	double err_p;
	double err_sum;
	double err_dif;
	double sum_lmt;
} pid;

// pid.h
double
calc_pid(volatile pid *pid_ptr, double err, double dt);
double
calc_pid_2(volatile pid *pid_ptr, double ref, double val, double dt);
void
init_pid(volatile pid *pid_ptr, double p, double i, double d, double limit,
		double sum_limit);
void
reset_pid(volatile pid *pid_ptr);

// time.h
void
time_ISR();
void
time_start();
void
time_stop();
uint32_t get_time();

// encoder.h
int16_t
cnt_difference(TIM_HandleTypeDef *htim, volatile int16_t *cur);
void
enc_start(TIM_HandleTypeDef *htim);
void
enc_init();
double
enc_velocity(int16_t diff, double deltaT, uint16_t impulse_per_2Pi);

// pwm.h
void
pwm_set_dc(TIM_HandleTypeDef *htim, uint32_t channel, int16_t duty_cycle);
void
pwm_init();
void pwm_kill();
void
pwm_left(double vel, double max_vel);
void
pwm_right(double vel, double max_vel);
void
set_motor_l_dir(int8_t dir);
void
set_motor_r_dir(int8_t dir);

// odometry.h
void
odometry_init();
void
update_odom();
pose_2D_stamped
get_odom();

//signal.h
double trajectory_synthesis(double v_ref, double v_ref_prev, double v_end,
		double acc, double acc_stop, double dist, double dt);
void
wrap180(volatile double *signal);
void
wrap2Pi(volatile double *signal);
double clamp(double signal, double min, double max);
double correct_param(double param, double error, double eta, double min,
		double max);
unsigned char stacked(double time_limit, double v, double v_min, double freq,
		unsigned *cnt);
double velocity_synthesis(double distance, double velocity, double acceleration,
		double J_MAX, double stopping_distance, double v_max, double v_min,
		double dt, double v0, unsigned slowdown_status, double v_slowed_max,
		double v_min_acc);
double synthesis_v(double velocity, double acceleration, double a_step,
		double v_des, double dt, double v0);
double wrap(double signal, double min, double max);
void wrap_ptr(double *signal, double min, double max);
short sign(double num);
double scale_vel_ref(volatile double *ref_1, volatile double *ref_2,
		double limit);
double abs_max(double a, double b);
double abs_min(double a, double b);
unsigned long unsigned_min(unsigned long a, unsigned long b);
void vel_ramp_up_ptr(double *signal, double reference, double acc);
double vel_ramp_up(double signal, double reference, double acc);
double vel_ramp(double signal, double reference, double acc);
double vel_s_curve_up_webots(double *vel, double prev_vel, double vel_ref,
		double jerk);
double vel_s_curve_up(double vel, double accel, double vel_ref, double jerk);
double min3(double a, double b, double c);
double snap_angle(double angle, double step);
double snap_ortho_deg(double phi);

// comm.h

// control.h
void control_loop(double dt);
void velocity_loop(double dt);
void move_init();
double get_v_r();
double get_v_l();

#endif /* LIB_LIB_H_ */

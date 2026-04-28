/*
 * control.c
 *
 *  Created on: Mar 13, 2026
 *      Author: lazar
 */

#include "main.h"

static void stop();
static void rotate();
static void go_to_xy();
static void disassemble();

static void reset_all_pids();

move moves[MAX_NUM_OF_MOVES];
action ctrl_action;
volatile uint8_t ctrl_num_of_moves = 0;
uint8_t move_index = 0;

int8_t direction_;
static pose_2D_stamped odom_;
double x_ref_, y_ref_, phi_ref_;
double v_ref_, w_ref_, v_ctrl_, w_ctrl_;
double dt_ = 0.001;

double v_max_temp_, w_max_temp_;
double V_MIN_ = 0.1;
double V_MAX_ = 1.5;
double W_MIN_ = 0.628;
double W_MAX_ = 12.57;
double V_MIN_STACKED_ = 0.01;

double MOTOR_V_MAX_ = 1.6;
double V_STEP_MAX = 0.05;	// [m/ms]
double L_;
double STACKED_TIME_ = 0.06;
double D_TOL_ = 0.01; // absolute distance from target
double D_PROJ_TOL_ = 0.0; // projected distance from target
double D_LONG_TOL_ = 0.12; // distance before rotation is used fully
double D_SHORT_TOL_ = 0.03; // minimal distance for rotation during translation
double PHI_TOL_ = 0.0157; // absolute angle from target
double v_right_ref_, v_left_ref_;
double v_right_, v_left_;
double phi_error_, x_error_, y_error_;
unsigned stacked_cnt_ = 0;
double V_SLOWED_MAX_ = 0.75;
volatile pid v_loop, w_loop, d_loop, phi_loop;
double distance_, distance_proj_;        // [m]

void move_init() {
	ctrl_action.id = 0;
	ctrl_action.moves_ptr = moves;
	ctrl_action.num_of_moves = ctrl_num_of_moves;

	v_max_temp_ = V_MAX_;
	w_max_temp_ = W_MAX_;

	init_pid(&d_loop, 1.0, 0.0, 0.0, 2.0, 2.0);
	init_pid(&phi_loop, 1.0, 0.0, 0.0, 12.57, 12.57);
	init_pid(&v_loop, 12.0, 0.01, 1.0, 1680, 420);
	init_pid(&w_loop, 92.0, 0.025, 28.0, 1680, 280);
}

void control_loop(double dt) {
	pwm_init();
	odom_ = get_odom();

	if (moves[move_index].status < 0)
		move_index++;
	if (move_index >= ctrl_num_of_moves)
		ctrl_action.status = -1;

	switch (moves[move_index].type) {
	case -1:
		stop();
		break;
	case 0:
		disassemble();
		break;
	case 1:
		go_to_xy();
		break;
	case 2:
		// TODO: po kruznici
		break;
	case 3:
		rotate();
		break;
	}
}

void velocity_loop(double dt) {
	// ulaz: referenca za brzine levog i desnog: v_ref_, w_ref_
	double v_err = v_ref_ - get_odom().v;
	double w_err = w_ref_ - get_odom().w;
	v_ctrl_ = calc_pid(&v_loop, v_err, dt);
	w_ctrl_ = calc_pid(&w_loop, w_err, dt);

	// izlaz pwm
	v_right_ = vel_ramp(v_right_, v_ctrl_ + w_ctrl_ * L_ * 0.5, V_STEP_MAX);
	v_left_ = vel_ramp(v_left_, v_ctrl_ - w_ctrl_ * L_ * 0.5, V_STEP_MAX);
	scale_vel_ref(&v_right_, &v_left_, MOTOR_V_MAX_);

	pwm_right(v_right_, MOTOR_V_MAX_);
	pwm_left(v_left_, MOTOR_V_MAX_);
}

double get_v_r() {
	return v_right_;
}

double get_v_l() {
	return v_left_;
}

// TODO: uradi opet
static void rotate() {
	switch (moves[move_index].status) {
	case 0:
		reset_all_pids();
		/*
		 * TODO: calculate:
		 * - pocetne brzine
		 * - Max brzine
		 * - Step brzine (nagib rampi), i za ubrzavanje i usporavanje
		 * - Krajnje brzine
		 */
		moves[move_index].status = 1;
		break;
	case 1:
		phi_error_ = wrap(phi_ref_ - odom_.phi, -M_PI, M_PI);
		if (fabs(phi_error_) < PHI_TOL_ && fabs(odom_.w) < W_MIN_ * 2.0) {
			moves[move_index].status = -1;
		}

		v_ref_ = 0;
		// TODO: obicna sinteza trajektorije (feedforward) + pid (feedback)
		//	w_ref_ = velocity_synthesis(phi_error_, w_base_, alpha_, j_rot_max_temp_,
		//			stopping_angle_, w_max_temp_, W_MIN_, dt_, 0.0, 0, 0.0,
		//			W_MIN_ACC_temp_);
		break;
	}
}

// TODO: uradi opet
static void go_to_xy() {
	switch (moves[move_index].status) {
	case 0:
		reset_all_pids();
		/*
		 * TODO: calculate:
		 * - pocetne brzine
		 * - Max brzine
		 * - Step brzine (nagib rampi), i za ubrzavanje i usporavanje
		 * - Krajnje brzine
		 */
		moves[move_index].status = 1;
		break;
	case 1:
		if (fabs(phi_error_) < PHI_TOL_ * 3.0 && fabs(odom_.w) < W_MIN_ * 2.0) {
			moves[move_index].status = 2;
			w_max_temp_ = W_MAX_;
		}
		v_ref_ = 0;
// TODO: obicna sinteza trajektorije (feedforward) + pid (feedback)
//		w_ref_ = velocity_synthesis(phi_error_, w_base_, alpha_,
//				j_rot_max_temp_, stopping_angle_, w_max_temp_, W_MIN_, dt_, 0.0,
//				0, 0.0, W_MIN_ACC_temp_);
		break;
	case 2:
		reset_all_pids();
		/*
		 * TODO: calculate:
		 * - pocetne brzine
		 * - Max brzine
		 * - Step brzine (nagib rampi), i za ubrzavanje i usporavanje
		 * - Krajnje brzine
		 */
		moves[move_index].status = 3;
		break;
	case 3:

		if (distance_proj_ < D_PROJ_TOL_ && fabs(odom_.v) < V_MIN_ * 2.0
				&& fabs(odom_.w) < W_MIN_ * 2.0) {
			if (fabs(distance_) < D_TOL_) {
				moves[move_index].status = -1;
				break;
			} else {
				moves[move_index].status = -5;
				break;
			}
// TODO: automatski backing umesto da ubije kretnju (sa proverom da li moze da se izvuce)
//		} else if (obst_in_loop_budz_ && fabs(v_base_) < V_MIN_ * 2.0
//				&& fabs(w_base_) < W_MIN_ * 2.0) {
//			movement_state_ = -4;
//			break;
		} else if (stacked(STACKED_TIME_, odom_.v, V_MIN_STACKED_, 1.0 / dt_,
				&stacked_cnt_)) {
			moves[move_index].status = -3;
			break;
		}
		x_error_ = x_ref_ - odom_.x;
		y_error_ = y_ref_ - odom_.y;
		phi_error_ = wrap(
				atan2(y_error_, x_error_) - odom_.phi
						+ (direction_ - 1) * M_PI * 0.5, -M_PI, M_PI);
		distance_ = sqrt(x_error_ * x_error_ + y_error_ * y_error_);
		distance_proj_ = distance_ * cos(phi_error_);
// TODO: obicna sinteza trajektorije (feedforward) + pid (feedback)
//		v_ref_ = velocity_synthesis(distance_proj_ * direction_, v_base_, a_,
//				j_max_temp_, stopping_distance_, v_max_temp_, V_MIN_, dt_, 0.0,
//				obst_in_loop_budz_, V_SLOWED_MAX_, V_MIN_ACC_);
// TODO: samo pid feedback
//		w_ref_ = P_w_
//				* clamp(
//						(distance_ - D_SHORT_TOL_)
//								/ (D_LONG_TOL_ - D_SHORT_TOL_), 0.0, 1.0)
//				* phi_error_;
		break;
	}
}

// TODO: proveri za status
static void stop() {
	moves[move_index].status = 0;
	v_ref_ = 0;
	w_ref_ = 0;
	stacked_cnt_ = 0;
	x_ref_ = odom_.x;
	y_ref_ = odom_.y;
	phi_ref_ = odom_.phi;
	direction_ = 1;
	v_max_temp_ = V_MAX_;
	w_max_temp_ = W_MAX_;
	reset_all_pids();
}

static void disassemble() {
	stop();
	pwm_kill();
}

static void reset_all_pids() {
	reset_pid(&d_loop);
	reset_pid(&phi_loop);
	reset_pid(&v_loop);
	reset_pid(&w_loop);
}

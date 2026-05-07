/*
 * control.c
 *
 *  Created on: Mar 13, 2026
 *      Author: lazar
 */

#include "main.h"

static void stop();
static void rotate(double dt);
static void go_to_xy(double dt);
static void disassemble();

static void reset_all_pids();

move moves_[MAX_NUM_OF_MOVES];
action ctrl_action_;
volatile uint8_t ctrl_num_of_moves_ = 0;
uint8_t move_index_ = 0;
uint16_t prev_action_id_ = 0xffff;
double first_x_ = 0.0, first_y_ = 0.0, first_phi_ = 0.0;
uint8_t transition_ = 0;
int8_t ctrl_move_type_ = -1;

int8_t direction_;
static pose_2D_stamped odom_;
double x_ref_, y_ref_, phi_ref_;
double v_ref_ = 0.0, w_ref_ = 0.0;
double v_ctrl_ = 0.0, w_ctrl_ = 0.0;
double v_ref_ff_ = 0.0, w_ref_ff_ = 0.0;
double v_ref_fb_ = 0.0, w_ref_fb_ = 0.0;

double V_MIN_ = 0.1;
double V_MAX_ = 1.5;
double W_MIN_ = 0.628;
double W_MAX_ = 12.57;
double V_MIN_STACKED_ = 0.01;

double MOTOR_V_MAX_ = 1.6;
double A_MAX_ = 1.0;	// [m/s^2]
double L_;
double STACKED_TIME_ = 0.06;
double D_TOL_ = 0.003; // absolute distance from target
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
	ctrl_action_.id = 0;
	ctrl_action_.moves_ptr = moves_;
	ctrl_action_.num_of_moves = ctrl_num_of_moves_;

	init_pid(&d_loop, 1.0, 0.0, 0.0, 2.0, 2.0);
	init_pid(&phi_loop, 1.0, 0.0, 0.0, 12.57, 12.57);
	init_pid(&v_loop, 12.0, 0.01, 1.0, 1680, 420);
	init_pid(&w_loop, 92.0, 0.025, 28.0, 1680, 280);
}

void action_init(action *action_ptr) {
	first_x_ = get_odom().x;
	first_y_ = get_odom().y;
	first_phi_ = get_odom().phi;
	// TODO: mora ovo da ucita iz poruke: ctrl_num_of_moves_
	/*
	 TODO: uradi tranzicije (na osnovu referentnih vrednosti idealnih situacija))
	 uticu na:
	 - v_0, v_end
	 - a, a_stop
	 - w_0, w_end
	 - alpha, alpha_stop
	 */
	/*
	 uint8_t index;

	 double v_0;
	 double v_end;
	 double a;
	 double a_stop;
	 double v_max;

	 double w_0;
	 double w_end;
	 double alpha;
	 double alpha_stop;
	 double w_max;
	 */
	for (uint8_t mi = 0; mi < action_ptr->num_of_moves; mi++) {
		move *src = &action_ptr->moves_ptr[mi];
		move *prev = NULL;
		move *next = NULL;
		double move_phi = 0.0, prev_phi = 0.0;
		double move_distance = 0.0, x_e = 0.0, y_e = 0.0;
		if (mi > 0)
			prev = &action_ptr->moves_ptr[mi - 1];
		if (mi + 1 < action_ptr->num_of_moves)
			next = &action_ptr->moves_ptr[mi + 1];

		src->index = mi;
		switch (src->type) {
		default:
			src->v_0 = 0.0;
			src->v_end = 0.0;
			src->a = 0.0;
			src->a_stop = 0.0;
			src->v_max = 0.0;

			src->w_0 = 0.0;
			src->w_end = 0.0;
			src->alpha = 0.0;
			src->alpha_stop = 0.0;
			src->w_max = 0.0;
			break;
		case 1:	// go to xy
			if (transition_) {
				// TODO: tranzicije od prethodne kretnje:
				//			- prethodna je 1 ili 2: prekopiraj vrednosti end -> 0
				if (prev && (prev->type == 1 || prev->type == 2)) {
					src->v_0 = prev->v_end;
				} else {
					src->v_0 = prev->v_end;
				}
				// TODO: tranzicije za sledecu kretnju:
				//			- sledeca je 1
				//			- sledeca je 2
				if (next && next->type == 1) {
					src->v_end = fmin(src->v_des, next->v_des);
				} else if (next && next->type == 2) {

				} else {
					src->v_end = 0.0;
					src->a_stop = 0.0;

					src->w_end = 0.0;
					src->alpha_stop = 2 * A_MAX_ / L_;
				}
			} else {
				src->v_0 = 0.0;
				src->v_end = 0.0;
				src->a = A_MAX_;
				src->a_stop = A_MAX_;
				if (prev) {
					x_e = src->x - prev->x;
					y_e = src->y - prev->y;
				} else {
					x_e = src->x - first_x_;
					y_e = src->y - first_y_;
				}
				move_distance = sqrt(x_e * x_e + y_e * y_e);
				src->v_max = compute_v_peak(src->v_0, src->v_end, src->v_des,
						src->a, src->a_stop, move_distance);

				src->w_0 = 0.0;
				src->w_end = 0.0;
				src->alpha = 2 * A_MAX_ / L_;
				src->alpha_stop = src->alpha;
				if (prev)
					prev_phi = prev->phi;
				else
					prev_phi = first_phi_;
				move_phi = src->phi - prev_phi;
				src->w_max = compute_v_peak(src->w_0, src->w_end, src->w_des,
						src->alpha, src->alpha_stop, move_phi);
			}
			break;
		case 2:	// po kruznici
			break;
		case 3:	// rotacija
			src->v_0 = 0.0;
			src->v_end = 0.0;
			src->a = A_MAX_;
			src->a_stop = A_MAX_;
			src->v_max = 0.0;

			src->w_0 = 0.0;
			src->w_end = 0.0;
			src->alpha = 2 * A_MAX_ / L_;
			src->alpha_stop = src->alpha;
			if (prev)
				prev_phi = prev->phi;
			else
				prev_phi = first_phi_;
			move_phi = src->phi - prev_phi;
			src->w_max = compute_v_peak(src->w_0, src->w_end, src->w_des,
					src->alpha, src->alpha_stop, move_phi);
			break;
		case 4:	// bezier
			break;
		}
		memcpy(&moves_[mi], src, sizeof(move));
	}
}

void control_loop(double dt) {
	pwm_init();
	odom_ = get_odom();
	if (prev_action_id_ != ctrl_action_.id)
		action_init(&ctrl_action_);

	if (move_index_ < ctrl_num_of_moves_) {
		if (moves_[move_index_].status < 0)
			move_index_++;
	}
	if (move_index_ >= ctrl_num_of_moves_) {
		ctrl_action_.status = -1;
		ctrl_move_type_ = -1;
	} else
		ctrl_move_type_ = moves_[move_index_].type;

	switch (ctrl_move_type_) {
	case -1:
		stop();
		break;
	case 0:
		disassemble();
		break;
	case 1:
		go_to_xy(dt);
		break;
	case 2:
		// TODO: po kruznici
		break;
	case 3:
		rotate(dt);
		break;
	case 4:
		// TODO: po bezieru
		break;
	}
	v_ref_ = v_ref_ff_ + v_ref_fb_;
	w_ref_ = w_ref_ff_ + w_ref_fb_;
	prev_action_id_ = ctrl_action_.id;
}

void velocity_loop(double dt) {
	double v_err = v_ref_ - get_odom().v;
	double w_err = w_ref_ - get_odom().w;
	v_ctrl_ = calc_pid(&v_loop, v_err, dt);
	w_ctrl_ = calc_pid(&w_loop, w_err, dt);

	v_right_ = vel_ramp(v_right_, v_ctrl_ + w_ctrl_ * L_ * 0.5, A_MAX_ * dt);
	v_left_ = vel_ramp(v_left_, v_ctrl_ - w_ctrl_ * L_ * 0.5, A_MAX_ * dt);
	scale_vel_ref(&v_right_, &v_left_, MOTOR_V_MAX_);

	pwm_right(v_right_, MOTOR_V_MAX_);
	pwm_left(v_left_, MOTOR_V_MAX_);
}

static void rotate(double dt) {
	switch (moves_[move_index_].status) {
	case 0:
		reset_all_pids();
		moves_[move_index_].status = 1;
		// namerno izbacen break
	case 1:
		phi_error_ = wrap(phi_ref_ - odom_.phi, -M_PI, M_PI);
		if (fabs(phi_error_) < PHI_TOL_ && fabs(odom_.w) < W_MIN_) {
			moves_[move_index_].status = -1;
		}
		v_ref_ff_ = 0;
		w_ref_ff_ = trajectory_synthesis(moves_[move_index_].w_max, w_ref_ff_,
				moves_[move_index_].w_end, moves_[move_index_].alpha,
				moves_[move_index_].alpha_stop, phi_error_, dt);
		break;
	}
}

static void go_to_xy(double dt) {
	x_error_ = x_ref_ - odom_.x;
	y_error_ = y_ref_ - odom_.y;
	phi_error_ = wrap(
			atan2(y_error_, x_error_) - odom_.phi
					+ (direction_ - 1) * M_PI * 0.5, -M_PI, M_PI);
	switch (moves_[move_index_].status) {
	case 0:
		reset_all_pids();
		if (fabs(phi_error_) < PHI_TOL_ && fabs(odom_.w) < W_MIN_) {
			moves_[move_index_].status = 3;
		} else {
			moves_[move_index_].status = 1;
		}
		// namerno izbacen break
	case 1:
		if (fabs(phi_error_) < PHI_TOL_ && fabs(odom_.w) < W_MIN_) {
			moves_[move_index_].status = 2;
		}
		v_ref_ff_ = 0;
		w_ref_ff_ = trajectory_synthesis(moves_[move_index_].w_max, w_ref_ff_,
				moves_[move_index_].w_end, moves_[move_index_].alpha,
				moves_[move_index_].alpha_stop, phi_error_, dt);
		break;
	case 2:
		reset_all_pids();
		moves_[move_index_].status = 3;
		// namerno izbacen break
	case 3:
		distance_ = sqrt(x_error_ * x_error_ + y_error_ * y_error_);
		distance_proj_ = distance_ * cos(phi_error_);

		if (distance_proj_ < D_PROJ_TOL_ && fabs(odom_.v) < V_MIN_
				&& fabs(odom_.w) < W_MIN_) {
			if (fabs(distance_) < D_TOL_) {
				moves_[move_index_].status = -1;
				break;
			} else {
				moves_[move_index_].status = -5;
				break;
			}
		} else if (stacked(STACKED_TIME_, odom_.v, V_MIN_STACKED_, 1.0 / dt,
				&stacked_cnt_)) {
			moves_[move_index_].status = -3;
			break;
		}

		v_ref_ff_ = trajectory_synthesis(moves_[move_index_].v_max, v_ref_ff_,
				moves_[move_index_].v_end, moves_[move_index_].a,
				moves_[move_index_].a_stop, distance_, dt);
		w_ref_ff_ = 0.0;
		double phi_err_mod_ = clamp(
				(distance_ - D_SHORT_TOL_) / (D_LONG_TOL_ - D_SHORT_TOL_), 0.0,
				1.0) * phi_error_;
		w_ref_fb_ = calc_pid(&phi_loop, phi_err_mod_, dt);
		break;
	}
}

// TODO: proveri za status
static void stop() {
	moves_[move_index_].status = -1;
	v_ref_ = 0.0;
	w_ref_ = 0.0;
	stacked_cnt_ = 0;
	x_ref_ = odom_.x;
	y_ref_ = odom_.y;
	phi_ref_ = odom_.phi;
	direction_ = 1;
	reset_all_pids();
}

static void disassemble() {
	stop();
	pwm_kill();
	moves_[move_index_].status = 1;
}

static void reset_all_pids() {
	reset_pid(&d_loop);
	reset_pid(&phi_loop);
	reset_pid(&v_loop);
	reset_pid(&w_loop);
}

double get_v_r() {
	return v_right_;
}

double get_v_l() {
	return v_left_;
}

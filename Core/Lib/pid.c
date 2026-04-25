/*
 * pid.c
 *
 *  Created on: Mar 25, 2026
 *      Author: lazar
 */

#include "main.h"

double calc_pid(volatile pid *pid_ptr, double err, double dt) {
	// Prestani da povecavas err_sum ako je izlaz vec u saturaciji
	if (fabs(pid_ptr->ctrl) < pid_ptr->lmt)
		pid_ptr->err_sum += (err * dt);

	pid_ptr->err_sum = clamp(pid_ptr->err_sum, -pid_ptr->sum_lmt,
			pid_ptr->sum_lmt);
	pid_ptr->err_dif = (err - pid_ptr->err_p) / dt;

	pid_ptr->p_ctrl = pid_ptr->p * err;
	pid_ptr->i_ctrl = pid_ptr->i * pid_ptr->err_sum;
	pid_ptr->d_ctrl = pid_ptr->d * pid_ptr->err_dif;

	pid_ptr->ctrl = pid_ptr->p_ctrl + pid_ptr->i_ctrl + pid_ptr->d_ctrl;
	pid_ptr->ctrl = clamp(pid_ptr->ctrl, -pid_ptr->lmt, pid_ptr->lmt);

	// Resetuj nagomilanu gresku kada greska zameni znak, a izlaz je u saturaciji
	if (err * pid_ptr->err_p < 0 && fabs(pid_ptr->ctrl) >= pid_ptr->lmt)
		pid_ptr->err_sum = 0;

	pid_ptr->err_p = err;
	return pid_ptr->ctrl;
}

double calc_pid_2(volatile pid *pid_ptr, double ref, double val, double dt) {
	return calc_pid(pid_ptr, ref - val, dt);
}

void init_pid(volatile pid *pid_ptr, double p, double i, double d, double limit,
		double sum_limit) {
	pid_ptr->p = p;
	pid_ptr->i = i;
	pid_ptr->d = d;
	pid_ptr->lmt = limit;
	pid_ptr->sum_lmt = sum_limit;
	pid_ptr->ctrl = 0.0;
	pid_ptr->p_ctrl = 0.0;
	pid_ptr->i_ctrl = 0.0;
	pid_ptr->d_ctrl = 0.0;
	pid_ptr->err_p = 0.0;
	pid_ptr->err_sum = 0.0;
	pid_ptr->err_dif = 0.0;
}

void reset_pid(volatile pid *pid_ptr) {
	pid_ptr->p_ctrl = 0.0;
	pid_ptr->i_ctrl = 0.0;
	pid_ptr->d_ctrl = 0.0;
	pid_ptr->ctrl = 0.0;
	pid_ptr->err_p = 0.0;
	pid_ptr->err_sum = 0.0;
	pid_ptr->err_dif = 0.0;
}

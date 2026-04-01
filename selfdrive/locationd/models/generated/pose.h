#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_8052617336508823134);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_9128085987645078449);
void pose_H_mod_fun(double *state, double *out_3361570693863352746);
void pose_f_fun(double *state, double dt, double *out_6386770787553891211);
void pose_F_fun(double *state, double dt, double *out_476766260057365837);
void pose_h_4(double *state, double *unused, double *out_5361742893531860988);
void pose_H_4(double *state, double *unused, double *out_831405397157000068);
void pose_h_10(double *state, double *unused, double *out_9170580747203543004);
void pose_H_10(double *state, double *unused, double *out_7393744558597873191);
void pose_h_13(double *state, double *unused, double *out_2910083262545817831);
void pose_H_13(double *state, double *unused, double *out_2380868428175332733);
void pose_h_14(double *state, double *unused, double *out_3581530193976320576);
void pose_H_14(double *state, double *unused, double *out_3914193829452372364);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
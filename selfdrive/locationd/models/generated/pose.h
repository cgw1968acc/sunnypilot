#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_1425647101528206710);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1586401905831597934);
void pose_H_mod_fun(double *state, double *out_3061887074369766267);
void pose_f_fun(double *state, double dt, double *out_1911708801291879452);
void pose_F_fun(double *state, double dt, double *out_8941015638488863399);
void pose_h_4(double *state, double *unused, double *out_1305004607597586938);
void pose_H_4(double *state, double *unused, double *out_7184552115366560582);
void pose_h_10(double *state, double *unused, double *out_7882420333113587623);
void pose_H_10(double *state, double *unused, double *out_1143390991321892766);
void pose_h_13(double *state, double *unused, double *out_3776293256350619509);
void pose_H_13(double *state, double *unused, double *out_426079092950140347);
void pose_h_14(double *state, double *unused, double *out_7912039560915361515);
void pose_H_14(double *state, double *unused, double *out_3221311259027076053);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
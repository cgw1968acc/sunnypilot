#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_8855607464613347160);
void live_err_fun(double *nom_x, double *delta_x, double *out_7551466837254102229);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_7083847312756732083);
void live_H_mod_fun(double *state, double *out_2417236276350252613);
void live_f_fun(double *state, double dt, double *out_6846072783341992679);
void live_F_fun(double *state, double dt, double *out_7610157667537165676);
void live_h_4(double *state, double *unused, double *out_6522284497954344691);
void live_H_4(double *state, double *unused, double *out_9221396043523342834);
void live_h_9(double *state, double *unused, double *out_31636738801653628);
void live_H_9(double *state, double *unused, double *out_8984158383556618137);
void live_h_10(double *state, double *unused, double *out_3222731136212803882);
void live_H_10(double *state, double *unused, double *out_8529081772535983422);
void live_h_12(double *state, double *unused, double *out_4561469184455077752);
void live_H_12(double *state, double *unused, double *out_4205891622154246987);
void live_h_35(double *state, double *unused, double *out_7242907357144390178);
void live_H_35(double *state, double *unused, double *out_1460328589829233278);
void live_h_32(double *state, double *unused, double *out_6367014881279238872);
void live_H_32(double *state, double *unused, double *out_1323329574589184315);
void live_h_13(double *state, double *unused, double *out_8938041407761873948);
void live_H_13(double *state, double *unused, double *out_3336911374531877993);
void live_h_14(double *state, double *unused, double *out_31636738801653628);
void live_H_14(double *state, double *unused, double *out_8984158383556618137);
void live_h_33(double *state, double *unused, double *out_7291516856447189669);
void live_H_33(double *state, double *unused, double *out_1690228414809624326);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
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
void live_H(double *in_vec, double *out_5367603715788806568);
void live_err_fun(double *nom_x, double *delta_x, double *out_8672480663876722627);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5409770897010794672);
void live_H_mod_fun(double *state, double *out_2205553917779500208);
void live_f_fun(double *state, double dt, double *out_8488540452176989047);
void live_F_fun(double *state, double dt, double *out_4078911431398825703);
void live_h_4(double *state, double *unused, double *out_7401640267109160321);
void live_H_4(double *state, double *unused, double *out_4271971512898402797);
void live_h_9(double *state, double *unused, double *out_2022425426979752880);
void live_H_9(double *state, double *unused, double *out_3015247422366044673);
void live_h_10(double *state, double *unused, double *out_605794633205793514);
void live_H_10(double *state, double *unused, double *out_6413106874283637020);
void live_h_12(double *state, double *unused, double *out_1386540175845920763);
void live_H_12(double *state, double *unused, double *out_7793514183768415823);
void live_h_35(double *state, double *unused, double *out_6375613586548939164);
void live_H_35(double *state, double *unused, double *out_3493047927458572707);
void live_h_32(double *state, double *unused, double *out_1850449695951069973);
void live_H_32(double *state, double *unused, double *out_774144670585054952);
void live_h_13(double *state, double *unused, double *out_4412635403753113393);
void live_H_13(double *state, double *unused, double *out_6755595083425046279);
void live_h_14(double *state, double *unused, double *out_2022425426979752880);
void live_H_14(double *state, double *unused, double *out_3015247422366044673);
void live_h_33(double *state, double *unused, double *out_2623541125936388845);
void live_H_33(double *state, double *unused, double *out_6643604932097430311);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
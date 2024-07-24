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
void live_H(double *in_vec, double *out_2807864690648877240);
void live_err_fun(double *nom_x, double *delta_x, double *out_3029380303421640733);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1174517407583881423);
void live_H_mod_fun(double *state, double *out_2053258554012563662);
void live_f_fun(double *state, double dt, double *out_516476738374678989);
void live_F_fun(double *state, double dt, double *out_8024826052511320476);
void live_h_4(double *state, double *unused, double *out_6723843180459930878);
void live_H_4(double *state, double *unused, double *out_1960006525441801665);
void live_h_9(double *state, double *unused, double *out_844199080084644077);
void live_H_9(double *state, double *unused, double *out_2197161210912975818);
void live_h_10(double *state, double *unused, double *out_7855237628529450047);
void live_H_10(double *state, double *unused, double *out_5799196377413634969);
void live_h_12(double *state, double *unused, double *out_3409325819001084101);
void live_H_12(double *state, double *unused, double *out_2581105550489395332);
void live_h_35(double *state, double *unused, double *out_8840753699187684762);
void live_H_35(double *state, double *unused, double *out_5326668582814409041);
void live_h_32(double *state, double *unused, double *out_7655211720868577238);
void live_H_32(double *state, double *unused, double *out_1588195920879707315);
void live_h_13(double *state, double *unused, double *out_7456432172034262476);
void live_H_13(double *state, double *unused, double *out_1633618319572082820);
void live_h_14(double *state, double *unused, double *out_844199080084644077);
void live_H_14(double *state, double *unused, double *out_2197161210912975818);
void live_h_33(double *state, double *unused, double *out_8731060726310887016);
void live_H_33(double *state, double *unused, double *out_8477225587453266645);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
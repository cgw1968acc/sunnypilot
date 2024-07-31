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
void live_H(double *in_vec, double *out_5306600351938922877);
void live_err_fun(double *nom_x, double *delta_x, double *out_493132114022433796);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_13914812565757194);
void live_H_mod_fun(double *state, double *out_6311765979948340433);
void live_f_fun(double *state, double dt, double *out_8817043458826378326);
void live_F_fun(double *state, double dt, double *out_4686700619145198144);
void live_h_4(double *state, double *unused, double *out_4231253766944934747);
void live_H_4(double *state, double *unused, double *out_5582778213154545828);
void live_h_9(double *state, double *unused, double *out_6025627951987539770);
void live_H_9(double *state, double *unused, double *out_1704440722109901642);
void live_h_10(double *state, double *unused, double *out_685246647907776553);
void live_H_10(double *state, double *unused, double *out_6414994590408578783);
void live_h_12(double *state, double *unused, double *out_2546856239679354150);
void live_H_12(double *state, double *unused, double *out_6482707483512272792);
void live_h_35(double *state, double *unused, double *out_6732886733617403438);
void live_H_35(double *state, double *unused, double *out_4829913132852918373);
void live_h_32(double *state, double *unused, double *out_8300627046898151635);
void live_H_32(double *state, double *unused, double *out_6434815393453847522);
void live_h_13(double *state, double *unused, double *out_7473551091241035200);
void live_H_13(double *state, double *unused, double *out_2766590617203188566);
void live_h_14(double *state, double *unused, double *out_6025627951987539770);
void live_H_14(double *state, double *unused, double *out_1704440722109901642);
void live_h_33(double *state, double *unused, double *out_4189155859768253010);
void live_H_33(double *state, double *unused, double *out_7980470137491775977);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
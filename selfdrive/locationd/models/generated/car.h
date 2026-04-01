#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_2455886244297513722);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7932107212443189942);
void car_H_mod_fun(double *state, double *out_5697610214733121748);
void car_f_fun(double *state, double dt, double *out_828926163848828701);
void car_F_fun(double *state, double dt, double *out_1519319325476872803);
void car_h_25(double *state, double *unused, double *out_2607938834820806618);
void car_H_25(double *state, double *unused, double *out_8682767505268722525);
void car_h_24(double *state, double *unused, double *out_4723543934201376374);
void car_H_24(double *state, double *unused, double *out_5781653417043061193);
void car_h_30(double *state, double *unused, double *out_4058570227449689785);
void car_H_30(double *state, double *unused, double *out_6164434546761473898);
void car_h_26(double *state, double *unused, double *out_4923398944557150980);
void car_H_26(double *state, double *unused, double *out_6022473249566772867);
void car_h_27(double *state, double *unused, double *out_2004622950144329101);
void car_H_27(double *state, double *unused, double *out_8339197858561898809);
void car_h_29(double *state, double *unused, double *out_943103040211155860);
void car_H_29(double *state, double *unused, double *out_5654203202447081714);
void car_h_28(double *state, double *unused, double *out_5799256483651346534);
void car_H_28(double *state, double *unused, double *out_7710141854192939328);
void car_h_31(double *state, double *unused, double *out_2883132897105312507);
void car_H_31(double *state, double *unused, double *out_5396265147333421391);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
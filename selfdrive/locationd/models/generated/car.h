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
void car_err_fun(double *nom_x, double *delta_x, double *out_7939984129706908020);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4500603217124712497);
void car_H_mod_fun(double *state, double *out_5439614909828061126);
void car_f_fun(double *state, double dt, double *out_396057045295170870);
void car_F_fun(double *state, double dt, double *out_1763107264307268110);
void car_h_25(double *state, double *unused, double *out_2427701692207996360);
void car_H_25(double *state, double *unused, double *out_3014214633440783213);
void car_h_24(double *state, double *unused, double *out_6174353062213406939);
void car_H_24(double *state, double *unused, double *out_6213850552628412012);
void car_h_30(double *state, double *unused, double *out_2270515023856867215);
void car_H_30(double *state, double *unused, double *out_7541910963568391411);
void car_h_26(double *state, double *unused, double *out_5307301075301199900);
void car_H_26(double *state, double *unused, double *out_6755717952314839437);
void car_h_27(double *state, double *unused, double *out_7350902415471537191);
void car_H_27(double *state, double *unused, double *out_5318316892384448194);
void car_h_29(double *state, double *unused, double *out_4732247679936198564);
void car_H_29(double *state, double *unused, double *out_7031679619253999227);
void car_h_28(double *state, double *unused, double *out_6094168390577168780);
void car_H_28(double *state, double *unused, double *out_6332665437386021815);
void car_h_31(double *state, double *unused, double *out_1512454328617810106);
void car_H_31(double *state, double *unused, double *out_7381926054548190913);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
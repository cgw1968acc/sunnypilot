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
void car_err_fun(double *nom_x, double *delta_x, double *out_1556433197202642724);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7503824718266888943);
void car_H_mod_fun(double *state, double *out_4233392152597311220);
void car_f_fun(double *state, double dt, double *out_6213018529770018042);
void car_F_fun(double *state, double dt, double *out_487389055243717466);
void car_h_25(double *state, double *unused, double *out_5116885968642153577);
void car_H_25(double *state, double *unused, double *out_7205861781851670427);
void car_h_24(double *state, double *unused, double *out_1411115276105148042);
void car_H_24(double *state, double *unused, double *out_9015174507879012627);
void car_h_30(double *state, double *unused, double *out_2547928186945489211);
void car_H_30(double *state, double *unused, double *out_4324191950366264434);
void car_h_26(double *state, double *unused, double *out_3151244071621966728);
void car_H_26(double *state, double *unused, double *out_3464358462977614203);
void car_h_27(double *state, double *unused, double *out_7074033383108350785);
void car_H_27(double *state, double *unused, double *out_6498955262166689345);
void car_h_29(double *state, double *unused, double *out_3491438039847151646);
void car_H_29(double *state, double *unused, double *out_3813960606051872250);
void car_h_28(double *state, double *unused, double *out_6371037422940355186);
void car_H_28(double *state, double *unused, double *out_5152027067603780664);
void car_h_31(double *state, double *unused, double *out_3522991508076388307);
void car_H_31(double *state, double *unused, double *out_7236507743728630855);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
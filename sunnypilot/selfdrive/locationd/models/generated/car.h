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
void car_err_fun(double *nom_x, double *delta_x, double *out_1525856032200297312);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2947590959223528162);
void car_H_mod_fun(double *state, double *out_3078840736753877381);
void car_f_fun(double *state, double dt, double *out_8225664519178041008);
void car_F_fun(double *state, double dt, double *out_8925472146913299645);
void car_h_25(double *state, double *unused, double *out_6213069847998954130);
void car_H_25(double *state, double *unused, double *out_4029817760279522699);
void car_h_24(double *state, double *unused, double *out_4714750995414331089);
void car_H_24(double *state, double *unused, double *out_7587743359081845835);
void car_h_30(double *state, double *unused, double *out_8371990749282095834);
void car_H_30(double *state, double *unused, double *out_7500235971938412162);
void car_h_26(double *state, double *unused, double *out_609978422724744931);
void car_H_26(double *state, double *unused, double *out_288314441405466475);
void car_h_27(double *state, double *unused, double *out_4522622013263273170);
void car_H_27(double *state, double *unused, double *out_8771744789970714543);
void car_h_29(double *state, double *unused, double *out_3675711807013606636);
void car_H_29(double *state, double *unused, double *out_6990004627624019978);
void car_h_28(double *state, double *unused, double *out_1485835961818940485);
void car_H_28(double *state, double *unused, double *out_1975983046031632936);
void car_h_31(double *state, double *unused, double *out_7756602718772405027);
void car_H_31(double *state, double *unused, double *out_4060463722156483127);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6986492762196806579);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2263396047980688855);
void car_H_mod_fun(double *state, double *out_402436849917620658);
void car_f_fun(double *state, double dt, double *out_2168537855026263863);
void car_F_fun(double *state, double dt, double *out_7745800668543318320);
void car_h_25(double *state, double *unused, double *out_1421900190648173534);
void car_H_25(double *state, double *unused, double *out_8968546326991991873);
void car_h_24(double *state, double *unused, double *out_880869654375610157);
void car_H_24(double *state, double *unused, double *out_4148224822336003610);
void car_h_30(double *state, double *unused, double *out_8556234859568204194);
void car_H_30(double *state, double *unused, double *out_8839207379848751803);
void car_h_26(double *state, double *unused, double *out_7952918974891726677);
void car_H_26(double *state, double *unused, double *out_8821343682607247839);
void car_h_27(double *state, double *unused, double *out_268547644087507426);
void car_H_27(double *state, double *unused, double *out_6664444068048326892);
void car_h_29(double *state, double *unused, double *out_425734312438636571);
void car_H_29(double *state, double *unused, double *out_9097305349546407629);
void car_h_28(double *state, double *unused, double *out_4733125623573338219);
void car_H_28(double *state, double *unused, double *out_4267039707093613413);
void car_h_31(double *state, double *unused, double *out_5520203488927545398);
void car_H_31(double *state, double *unused, double *out_8999192288868952301);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
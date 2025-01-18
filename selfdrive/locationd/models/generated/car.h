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
void car_err_fun(double *nom_x, double *delta_x, double *out_5011033113370860982);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7387888235897089854);
void car_H_mod_fun(double *state, double *out_7046635644391981245);
void car_f_fun(double *state, double dt, double *out_2555231531843821050);
void car_F_fun(double *state, double dt, double *out_3197627356211555357);
void car_h_25(double *state, double *unused, double *out_3006913549208278608);
void car_H_25(double *state, double *unused, double *out_4903085469427540420);
void car_h_24(double *state, double *unused, double *out_7421029612925360575);
void car_H_24(double *state, double *unused, double *out_34270604399833568);
void car_h_30(double *state, double *unused, double *out_5954639539563763569);
void car_H_30(double *state, double *unused, double *out_375389139299932222);
void car_h_26(double *state, double *unused, double *out_8991425591008096254);
void car_H_26(double *state, double *unused, double *out_1161582150553484196);
void car_h_27(double *state, double *unused, double *out_3988997642543576720);
void car_H_27(double *state, double *unused, double *out_1799374172500492689);
void car_h_29(double *state, double *unused, double *out_4264191704828082609);
void car_H_29(double *state, double *unused, double *out_885620483614324406);
void car_h_28(double *state, double *unused, double *out_973382355137716512);
void car_H_28(double *state, double *unused, double *out_4196778533455206168);
void car_h_31(double *state, double *unused, double *out_4874359101545770577);
void car_H_31(double *state, double *unused, double *out_535374048320132720);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
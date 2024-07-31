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
void car_err_fun(double *nom_x, double *delta_x, double *out_5177580849545065691);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_632571021260590143);
void car_H_mod_fun(double *state, double *out_2143122989661037326);
void car_f_fun(double *state, double dt, double *out_5823499246669271172);
void car_F_fun(double *state, double dt, double *out_6908189206762765849);
void car_h_25(double *state, double *unused, double *out_8207864553779973215);
void car_H_25(double *state, double *unused, double *out_1901783904286363903);
void car_h_24(double *state, double *unused, double *out_3833629854148761959);
void car_H_24(double *state, double *unused, double *out_8477355710877882004);
void car_h_30(double *state, double *unused, double *out_8365051222131102360);
void car_H_30(double *state, double *unused, double *out_8818474245777980658);
void car_h_26(double *state, double *unused, double *out_3023183986245071797);
void car_H_26(double *state, double *unused, double *out_5206309874047164504);
void car_h_27(double *state, double *unused, double *out_7202093468485245716);
void car_H_27(double *state, double *unused, double *out_6643710933977555747);
void car_h_29(double *state, double *unused, double *out_431258242134894780);
void car_H_29(double *state, double *unused, double *out_9118038483617178774);
void car_h_28(double *state, double *unused, double *out_6018743389388980953);
void car_H_28(double *state, double *unused, double *out_152050809961525860);
void car_h_31(double *state, double *unused, double *out_8483058616064479104);
void car_H_31(double *state, double *unused, double *out_8978459154798181156);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
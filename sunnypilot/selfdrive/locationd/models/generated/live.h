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
void live_H(double *in_vec, double *out_7441282162866372314);
void live_err_fun(double *nom_x, double *delta_x, double *out_2479204932810487270);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_7447944962339131556);
void live_H_mod_fun(double *state, double *out_5204582100448785067);
void live_f_fun(double *state, double dt, double *out_8991564106194565126);
void live_F_fun(double *state, double dt, double *out_2896523700517080296);
void live_h_4(double *state, double *unused, double *out_6938460860380565614);
void live_H_4(double *state, double *unused, double *out_354742044719558498);
void live_h_9(double *state, double *unused, double *out_4645854161907534325);
void live_H_9(double *state, double *unused, double *out_595931691349149143);
void live_h_10(double *state, double *unused, double *out_6142871767430398832);
void live_H_10(double *state, double *unused, double *out_1311088658062718183);
void live_h_12(double *state, double *unused, double *out_3747100410071819361);
void live_H_12(double *state, double *unused, double *out_5374198452751520293);
void live_h_35(double *state, double *unused, double *out_2745840042203185760);
void live_H_35(double *state, double *unused, double *out_3721404102092165874);
void live_h_32(double *state, double *unused, double *out_3036416829438395629);
void live_H_32(double *state, double *unused, double *out_3193460401601950482);
void live_h_13(double *state, double *unused, double *out_3221353913193353598);
void live_H_13(double *state, double *unused, double *out_7948618051725327098);
void live_h_14(double *state, double *unused, double *out_4645854161907534325);
void live_H_14(double *state, double *unused, double *out_595931691349149143);
void live_h_33(double *state, double *unused, double *out_4850322567969407631);
void live_H_33(double *state, double *unused, double *out_6871961106731023478);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2455886244297513722) {
   out_2455886244297513722[0] = delta_x[0] + nom_x[0];
   out_2455886244297513722[1] = delta_x[1] + nom_x[1];
   out_2455886244297513722[2] = delta_x[2] + nom_x[2];
   out_2455886244297513722[3] = delta_x[3] + nom_x[3];
   out_2455886244297513722[4] = delta_x[4] + nom_x[4];
   out_2455886244297513722[5] = delta_x[5] + nom_x[5];
   out_2455886244297513722[6] = delta_x[6] + nom_x[6];
   out_2455886244297513722[7] = delta_x[7] + nom_x[7];
   out_2455886244297513722[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7932107212443189942) {
   out_7932107212443189942[0] = -nom_x[0] + true_x[0];
   out_7932107212443189942[1] = -nom_x[1] + true_x[1];
   out_7932107212443189942[2] = -nom_x[2] + true_x[2];
   out_7932107212443189942[3] = -nom_x[3] + true_x[3];
   out_7932107212443189942[4] = -nom_x[4] + true_x[4];
   out_7932107212443189942[5] = -nom_x[5] + true_x[5];
   out_7932107212443189942[6] = -nom_x[6] + true_x[6];
   out_7932107212443189942[7] = -nom_x[7] + true_x[7];
   out_7932107212443189942[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5697610214733121748) {
   out_5697610214733121748[0] = 1.0;
   out_5697610214733121748[1] = 0.0;
   out_5697610214733121748[2] = 0.0;
   out_5697610214733121748[3] = 0.0;
   out_5697610214733121748[4] = 0.0;
   out_5697610214733121748[5] = 0.0;
   out_5697610214733121748[6] = 0.0;
   out_5697610214733121748[7] = 0.0;
   out_5697610214733121748[8] = 0.0;
   out_5697610214733121748[9] = 0.0;
   out_5697610214733121748[10] = 1.0;
   out_5697610214733121748[11] = 0.0;
   out_5697610214733121748[12] = 0.0;
   out_5697610214733121748[13] = 0.0;
   out_5697610214733121748[14] = 0.0;
   out_5697610214733121748[15] = 0.0;
   out_5697610214733121748[16] = 0.0;
   out_5697610214733121748[17] = 0.0;
   out_5697610214733121748[18] = 0.0;
   out_5697610214733121748[19] = 0.0;
   out_5697610214733121748[20] = 1.0;
   out_5697610214733121748[21] = 0.0;
   out_5697610214733121748[22] = 0.0;
   out_5697610214733121748[23] = 0.0;
   out_5697610214733121748[24] = 0.0;
   out_5697610214733121748[25] = 0.0;
   out_5697610214733121748[26] = 0.0;
   out_5697610214733121748[27] = 0.0;
   out_5697610214733121748[28] = 0.0;
   out_5697610214733121748[29] = 0.0;
   out_5697610214733121748[30] = 1.0;
   out_5697610214733121748[31] = 0.0;
   out_5697610214733121748[32] = 0.0;
   out_5697610214733121748[33] = 0.0;
   out_5697610214733121748[34] = 0.0;
   out_5697610214733121748[35] = 0.0;
   out_5697610214733121748[36] = 0.0;
   out_5697610214733121748[37] = 0.0;
   out_5697610214733121748[38] = 0.0;
   out_5697610214733121748[39] = 0.0;
   out_5697610214733121748[40] = 1.0;
   out_5697610214733121748[41] = 0.0;
   out_5697610214733121748[42] = 0.0;
   out_5697610214733121748[43] = 0.0;
   out_5697610214733121748[44] = 0.0;
   out_5697610214733121748[45] = 0.0;
   out_5697610214733121748[46] = 0.0;
   out_5697610214733121748[47] = 0.0;
   out_5697610214733121748[48] = 0.0;
   out_5697610214733121748[49] = 0.0;
   out_5697610214733121748[50] = 1.0;
   out_5697610214733121748[51] = 0.0;
   out_5697610214733121748[52] = 0.0;
   out_5697610214733121748[53] = 0.0;
   out_5697610214733121748[54] = 0.0;
   out_5697610214733121748[55] = 0.0;
   out_5697610214733121748[56] = 0.0;
   out_5697610214733121748[57] = 0.0;
   out_5697610214733121748[58] = 0.0;
   out_5697610214733121748[59] = 0.0;
   out_5697610214733121748[60] = 1.0;
   out_5697610214733121748[61] = 0.0;
   out_5697610214733121748[62] = 0.0;
   out_5697610214733121748[63] = 0.0;
   out_5697610214733121748[64] = 0.0;
   out_5697610214733121748[65] = 0.0;
   out_5697610214733121748[66] = 0.0;
   out_5697610214733121748[67] = 0.0;
   out_5697610214733121748[68] = 0.0;
   out_5697610214733121748[69] = 0.0;
   out_5697610214733121748[70] = 1.0;
   out_5697610214733121748[71] = 0.0;
   out_5697610214733121748[72] = 0.0;
   out_5697610214733121748[73] = 0.0;
   out_5697610214733121748[74] = 0.0;
   out_5697610214733121748[75] = 0.0;
   out_5697610214733121748[76] = 0.0;
   out_5697610214733121748[77] = 0.0;
   out_5697610214733121748[78] = 0.0;
   out_5697610214733121748[79] = 0.0;
   out_5697610214733121748[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_828926163848828701) {
   out_828926163848828701[0] = state[0];
   out_828926163848828701[1] = state[1];
   out_828926163848828701[2] = state[2];
   out_828926163848828701[3] = state[3];
   out_828926163848828701[4] = state[4];
   out_828926163848828701[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_828926163848828701[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_828926163848828701[7] = state[7];
   out_828926163848828701[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1519319325476872803) {
   out_1519319325476872803[0] = 1;
   out_1519319325476872803[1] = 0;
   out_1519319325476872803[2] = 0;
   out_1519319325476872803[3] = 0;
   out_1519319325476872803[4] = 0;
   out_1519319325476872803[5] = 0;
   out_1519319325476872803[6] = 0;
   out_1519319325476872803[7] = 0;
   out_1519319325476872803[8] = 0;
   out_1519319325476872803[9] = 0;
   out_1519319325476872803[10] = 1;
   out_1519319325476872803[11] = 0;
   out_1519319325476872803[12] = 0;
   out_1519319325476872803[13] = 0;
   out_1519319325476872803[14] = 0;
   out_1519319325476872803[15] = 0;
   out_1519319325476872803[16] = 0;
   out_1519319325476872803[17] = 0;
   out_1519319325476872803[18] = 0;
   out_1519319325476872803[19] = 0;
   out_1519319325476872803[20] = 1;
   out_1519319325476872803[21] = 0;
   out_1519319325476872803[22] = 0;
   out_1519319325476872803[23] = 0;
   out_1519319325476872803[24] = 0;
   out_1519319325476872803[25] = 0;
   out_1519319325476872803[26] = 0;
   out_1519319325476872803[27] = 0;
   out_1519319325476872803[28] = 0;
   out_1519319325476872803[29] = 0;
   out_1519319325476872803[30] = 1;
   out_1519319325476872803[31] = 0;
   out_1519319325476872803[32] = 0;
   out_1519319325476872803[33] = 0;
   out_1519319325476872803[34] = 0;
   out_1519319325476872803[35] = 0;
   out_1519319325476872803[36] = 0;
   out_1519319325476872803[37] = 0;
   out_1519319325476872803[38] = 0;
   out_1519319325476872803[39] = 0;
   out_1519319325476872803[40] = 1;
   out_1519319325476872803[41] = 0;
   out_1519319325476872803[42] = 0;
   out_1519319325476872803[43] = 0;
   out_1519319325476872803[44] = 0;
   out_1519319325476872803[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1519319325476872803[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1519319325476872803[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1519319325476872803[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1519319325476872803[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1519319325476872803[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1519319325476872803[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1519319325476872803[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1519319325476872803[53] = -9.8100000000000005*dt;
   out_1519319325476872803[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1519319325476872803[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1519319325476872803[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1519319325476872803[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1519319325476872803[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1519319325476872803[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1519319325476872803[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1519319325476872803[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1519319325476872803[62] = 0;
   out_1519319325476872803[63] = 0;
   out_1519319325476872803[64] = 0;
   out_1519319325476872803[65] = 0;
   out_1519319325476872803[66] = 0;
   out_1519319325476872803[67] = 0;
   out_1519319325476872803[68] = 0;
   out_1519319325476872803[69] = 0;
   out_1519319325476872803[70] = 1;
   out_1519319325476872803[71] = 0;
   out_1519319325476872803[72] = 0;
   out_1519319325476872803[73] = 0;
   out_1519319325476872803[74] = 0;
   out_1519319325476872803[75] = 0;
   out_1519319325476872803[76] = 0;
   out_1519319325476872803[77] = 0;
   out_1519319325476872803[78] = 0;
   out_1519319325476872803[79] = 0;
   out_1519319325476872803[80] = 1;
}
void h_25(double *state, double *unused, double *out_2607938834820806618) {
   out_2607938834820806618[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8682767505268722525) {
   out_8682767505268722525[0] = 0;
   out_8682767505268722525[1] = 0;
   out_8682767505268722525[2] = 0;
   out_8682767505268722525[3] = 0;
   out_8682767505268722525[4] = 0;
   out_8682767505268722525[5] = 0;
   out_8682767505268722525[6] = 1;
   out_8682767505268722525[7] = 0;
   out_8682767505268722525[8] = 0;
}
void h_24(double *state, double *unused, double *out_4723543934201376374) {
   out_4723543934201376374[0] = state[4];
   out_4723543934201376374[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5781653417043061193) {
   out_5781653417043061193[0] = 0;
   out_5781653417043061193[1] = 0;
   out_5781653417043061193[2] = 0;
   out_5781653417043061193[3] = 0;
   out_5781653417043061193[4] = 1;
   out_5781653417043061193[5] = 0;
   out_5781653417043061193[6] = 0;
   out_5781653417043061193[7] = 0;
   out_5781653417043061193[8] = 0;
   out_5781653417043061193[9] = 0;
   out_5781653417043061193[10] = 0;
   out_5781653417043061193[11] = 0;
   out_5781653417043061193[12] = 0;
   out_5781653417043061193[13] = 0;
   out_5781653417043061193[14] = 1;
   out_5781653417043061193[15] = 0;
   out_5781653417043061193[16] = 0;
   out_5781653417043061193[17] = 0;
}
void h_30(double *state, double *unused, double *out_4058570227449689785) {
   out_4058570227449689785[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6164434546761473898) {
   out_6164434546761473898[0] = 0;
   out_6164434546761473898[1] = 0;
   out_6164434546761473898[2] = 0;
   out_6164434546761473898[3] = 0;
   out_6164434546761473898[4] = 1;
   out_6164434546761473898[5] = 0;
   out_6164434546761473898[6] = 0;
   out_6164434546761473898[7] = 0;
   out_6164434546761473898[8] = 0;
}
void h_26(double *state, double *unused, double *out_4923398944557150980) {
   out_4923398944557150980[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6022473249566772867) {
   out_6022473249566772867[0] = 0;
   out_6022473249566772867[1] = 0;
   out_6022473249566772867[2] = 0;
   out_6022473249566772867[3] = 0;
   out_6022473249566772867[4] = 0;
   out_6022473249566772867[5] = 0;
   out_6022473249566772867[6] = 0;
   out_6022473249566772867[7] = 1;
   out_6022473249566772867[8] = 0;
}
void h_27(double *state, double *unused, double *out_2004622950144329101) {
   out_2004622950144329101[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8339197858561898809) {
   out_8339197858561898809[0] = 0;
   out_8339197858561898809[1] = 0;
   out_8339197858561898809[2] = 0;
   out_8339197858561898809[3] = 1;
   out_8339197858561898809[4] = 0;
   out_8339197858561898809[5] = 0;
   out_8339197858561898809[6] = 0;
   out_8339197858561898809[7] = 0;
   out_8339197858561898809[8] = 0;
}
void h_29(double *state, double *unused, double *out_943103040211155860) {
   out_943103040211155860[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5654203202447081714) {
   out_5654203202447081714[0] = 0;
   out_5654203202447081714[1] = 1;
   out_5654203202447081714[2] = 0;
   out_5654203202447081714[3] = 0;
   out_5654203202447081714[4] = 0;
   out_5654203202447081714[5] = 0;
   out_5654203202447081714[6] = 0;
   out_5654203202447081714[7] = 0;
   out_5654203202447081714[8] = 0;
}
void h_28(double *state, double *unused, double *out_5799256483651346534) {
   out_5799256483651346534[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7710141854192939328) {
   out_7710141854192939328[0] = 1;
   out_7710141854192939328[1] = 0;
   out_7710141854192939328[2] = 0;
   out_7710141854192939328[3] = 0;
   out_7710141854192939328[4] = 0;
   out_7710141854192939328[5] = 0;
   out_7710141854192939328[6] = 0;
   out_7710141854192939328[7] = 0;
   out_7710141854192939328[8] = 0;
}
void h_31(double *state, double *unused, double *out_2883132897105312507) {
   out_2883132897105312507[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5396265147333421391) {
   out_5396265147333421391[0] = 0;
   out_5396265147333421391[1] = 0;
   out_5396265147333421391[2] = 0;
   out_5396265147333421391[3] = 0;
   out_5396265147333421391[4] = 0;
   out_5396265147333421391[5] = 0;
   out_5396265147333421391[6] = 0;
   out_5396265147333421391[7] = 0;
   out_5396265147333421391[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_2455886244297513722) {
  err_fun(nom_x, delta_x, out_2455886244297513722);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7932107212443189942) {
  inv_err_fun(nom_x, true_x, out_7932107212443189942);
}
void car_H_mod_fun(double *state, double *out_5697610214733121748) {
  H_mod_fun(state, out_5697610214733121748);
}
void car_f_fun(double *state, double dt, double *out_828926163848828701) {
  f_fun(state,  dt, out_828926163848828701);
}
void car_F_fun(double *state, double dt, double *out_1519319325476872803) {
  F_fun(state,  dt, out_1519319325476872803);
}
void car_h_25(double *state, double *unused, double *out_2607938834820806618) {
  h_25(state, unused, out_2607938834820806618);
}
void car_H_25(double *state, double *unused, double *out_8682767505268722525) {
  H_25(state, unused, out_8682767505268722525);
}
void car_h_24(double *state, double *unused, double *out_4723543934201376374) {
  h_24(state, unused, out_4723543934201376374);
}
void car_H_24(double *state, double *unused, double *out_5781653417043061193) {
  H_24(state, unused, out_5781653417043061193);
}
void car_h_30(double *state, double *unused, double *out_4058570227449689785) {
  h_30(state, unused, out_4058570227449689785);
}
void car_H_30(double *state, double *unused, double *out_6164434546761473898) {
  H_30(state, unused, out_6164434546761473898);
}
void car_h_26(double *state, double *unused, double *out_4923398944557150980) {
  h_26(state, unused, out_4923398944557150980);
}
void car_H_26(double *state, double *unused, double *out_6022473249566772867) {
  H_26(state, unused, out_6022473249566772867);
}
void car_h_27(double *state, double *unused, double *out_2004622950144329101) {
  h_27(state, unused, out_2004622950144329101);
}
void car_H_27(double *state, double *unused, double *out_8339197858561898809) {
  H_27(state, unused, out_8339197858561898809);
}
void car_h_29(double *state, double *unused, double *out_943103040211155860) {
  h_29(state, unused, out_943103040211155860);
}
void car_H_29(double *state, double *unused, double *out_5654203202447081714) {
  H_29(state, unused, out_5654203202447081714);
}
void car_h_28(double *state, double *unused, double *out_5799256483651346534) {
  h_28(state, unused, out_5799256483651346534);
}
void car_H_28(double *state, double *unused, double *out_7710141854192939328) {
  H_28(state, unused, out_7710141854192939328);
}
void car_h_31(double *state, double *unused, double *out_2883132897105312507) {
  h_31(state, unused, out_2883132897105312507);
}
void car_H_31(double *state, double *unused, double *out_5396265147333421391) {
  H_31(state, unused, out_5396265147333421391);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)

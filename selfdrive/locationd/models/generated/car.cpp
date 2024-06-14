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
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6986492762196806579) {
   out_6986492762196806579[0] = delta_x[0] + nom_x[0];
   out_6986492762196806579[1] = delta_x[1] + nom_x[1];
   out_6986492762196806579[2] = delta_x[2] + nom_x[2];
   out_6986492762196806579[3] = delta_x[3] + nom_x[3];
   out_6986492762196806579[4] = delta_x[4] + nom_x[4];
   out_6986492762196806579[5] = delta_x[5] + nom_x[5];
   out_6986492762196806579[6] = delta_x[6] + nom_x[6];
   out_6986492762196806579[7] = delta_x[7] + nom_x[7];
   out_6986492762196806579[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2263396047980688855) {
   out_2263396047980688855[0] = -nom_x[0] + true_x[0];
   out_2263396047980688855[1] = -nom_x[1] + true_x[1];
   out_2263396047980688855[2] = -nom_x[2] + true_x[2];
   out_2263396047980688855[3] = -nom_x[3] + true_x[3];
   out_2263396047980688855[4] = -nom_x[4] + true_x[4];
   out_2263396047980688855[5] = -nom_x[5] + true_x[5];
   out_2263396047980688855[6] = -nom_x[6] + true_x[6];
   out_2263396047980688855[7] = -nom_x[7] + true_x[7];
   out_2263396047980688855[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_402436849917620658) {
   out_402436849917620658[0] = 1.0;
   out_402436849917620658[1] = 0;
   out_402436849917620658[2] = 0;
   out_402436849917620658[3] = 0;
   out_402436849917620658[4] = 0;
   out_402436849917620658[5] = 0;
   out_402436849917620658[6] = 0;
   out_402436849917620658[7] = 0;
   out_402436849917620658[8] = 0;
   out_402436849917620658[9] = 0;
   out_402436849917620658[10] = 1.0;
   out_402436849917620658[11] = 0;
   out_402436849917620658[12] = 0;
   out_402436849917620658[13] = 0;
   out_402436849917620658[14] = 0;
   out_402436849917620658[15] = 0;
   out_402436849917620658[16] = 0;
   out_402436849917620658[17] = 0;
   out_402436849917620658[18] = 0;
   out_402436849917620658[19] = 0;
   out_402436849917620658[20] = 1.0;
   out_402436849917620658[21] = 0;
   out_402436849917620658[22] = 0;
   out_402436849917620658[23] = 0;
   out_402436849917620658[24] = 0;
   out_402436849917620658[25] = 0;
   out_402436849917620658[26] = 0;
   out_402436849917620658[27] = 0;
   out_402436849917620658[28] = 0;
   out_402436849917620658[29] = 0;
   out_402436849917620658[30] = 1.0;
   out_402436849917620658[31] = 0;
   out_402436849917620658[32] = 0;
   out_402436849917620658[33] = 0;
   out_402436849917620658[34] = 0;
   out_402436849917620658[35] = 0;
   out_402436849917620658[36] = 0;
   out_402436849917620658[37] = 0;
   out_402436849917620658[38] = 0;
   out_402436849917620658[39] = 0;
   out_402436849917620658[40] = 1.0;
   out_402436849917620658[41] = 0;
   out_402436849917620658[42] = 0;
   out_402436849917620658[43] = 0;
   out_402436849917620658[44] = 0;
   out_402436849917620658[45] = 0;
   out_402436849917620658[46] = 0;
   out_402436849917620658[47] = 0;
   out_402436849917620658[48] = 0;
   out_402436849917620658[49] = 0;
   out_402436849917620658[50] = 1.0;
   out_402436849917620658[51] = 0;
   out_402436849917620658[52] = 0;
   out_402436849917620658[53] = 0;
   out_402436849917620658[54] = 0;
   out_402436849917620658[55] = 0;
   out_402436849917620658[56] = 0;
   out_402436849917620658[57] = 0;
   out_402436849917620658[58] = 0;
   out_402436849917620658[59] = 0;
   out_402436849917620658[60] = 1.0;
   out_402436849917620658[61] = 0;
   out_402436849917620658[62] = 0;
   out_402436849917620658[63] = 0;
   out_402436849917620658[64] = 0;
   out_402436849917620658[65] = 0;
   out_402436849917620658[66] = 0;
   out_402436849917620658[67] = 0;
   out_402436849917620658[68] = 0;
   out_402436849917620658[69] = 0;
   out_402436849917620658[70] = 1.0;
   out_402436849917620658[71] = 0;
   out_402436849917620658[72] = 0;
   out_402436849917620658[73] = 0;
   out_402436849917620658[74] = 0;
   out_402436849917620658[75] = 0;
   out_402436849917620658[76] = 0;
   out_402436849917620658[77] = 0;
   out_402436849917620658[78] = 0;
   out_402436849917620658[79] = 0;
   out_402436849917620658[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2168537855026263863) {
   out_2168537855026263863[0] = state[0];
   out_2168537855026263863[1] = state[1];
   out_2168537855026263863[2] = state[2];
   out_2168537855026263863[3] = state[3];
   out_2168537855026263863[4] = state[4];
   out_2168537855026263863[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2168537855026263863[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2168537855026263863[7] = state[7];
   out_2168537855026263863[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7745800668543318320) {
   out_7745800668543318320[0] = 1;
   out_7745800668543318320[1] = 0;
   out_7745800668543318320[2] = 0;
   out_7745800668543318320[3] = 0;
   out_7745800668543318320[4] = 0;
   out_7745800668543318320[5] = 0;
   out_7745800668543318320[6] = 0;
   out_7745800668543318320[7] = 0;
   out_7745800668543318320[8] = 0;
   out_7745800668543318320[9] = 0;
   out_7745800668543318320[10] = 1;
   out_7745800668543318320[11] = 0;
   out_7745800668543318320[12] = 0;
   out_7745800668543318320[13] = 0;
   out_7745800668543318320[14] = 0;
   out_7745800668543318320[15] = 0;
   out_7745800668543318320[16] = 0;
   out_7745800668543318320[17] = 0;
   out_7745800668543318320[18] = 0;
   out_7745800668543318320[19] = 0;
   out_7745800668543318320[20] = 1;
   out_7745800668543318320[21] = 0;
   out_7745800668543318320[22] = 0;
   out_7745800668543318320[23] = 0;
   out_7745800668543318320[24] = 0;
   out_7745800668543318320[25] = 0;
   out_7745800668543318320[26] = 0;
   out_7745800668543318320[27] = 0;
   out_7745800668543318320[28] = 0;
   out_7745800668543318320[29] = 0;
   out_7745800668543318320[30] = 1;
   out_7745800668543318320[31] = 0;
   out_7745800668543318320[32] = 0;
   out_7745800668543318320[33] = 0;
   out_7745800668543318320[34] = 0;
   out_7745800668543318320[35] = 0;
   out_7745800668543318320[36] = 0;
   out_7745800668543318320[37] = 0;
   out_7745800668543318320[38] = 0;
   out_7745800668543318320[39] = 0;
   out_7745800668543318320[40] = 1;
   out_7745800668543318320[41] = 0;
   out_7745800668543318320[42] = 0;
   out_7745800668543318320[43] = 0;
   out_7745800668543318320[44] = 0;
   out_7745800668543318320[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7745800668543318320[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7745800668543318320[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7745800668543318320[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7745800668543318320[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7745800668543318320[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7745800668543318320[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7745800668543318320[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7745800668543318320[53] = -9.8000000000000007*dt;
   out_7745800668543318320[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7745800668543318320[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7745800668543318320[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7745800668543318320[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7745800668543318320[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7745800668543318320[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7745800668543318320[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7745800668543318320[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7745800668543318320[62] = 0;
   out_7745800668543318320[63] = 0;
   out_7745800668543318320[64] = 0;
   out_7745800668543318320[65] = 0;
   out_7745800668543318320[66] = 0;
   out_7745800668543318320[67] = 0;
   out_7745800668543318320[68] = 0;
   out_7745800668543318320[69] = 0;
   out_7745800668543318320[70] = 1;
   out_7745800668543318320[71] = 0;
   out_7745800668543318320[72] = 0;
   out_7745800668543318320[73] = 0;
   out_7745800668543318320[74] = 0;
   out_7745800668543318320[75] = 0;
   out_7745800668543318320[76] = 0;
   out_7745800668543318320[77] = 0;
   out_7745800668543318320[78] = 0;
   out_7745800668543318320[79] = 0;
   out_7745800668543318320[80] = 1;
}
void h_25(double *state, double *unused, double *out_1421900190648173534) {
   out_1421900190648173534[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8968546326991991873) {
   out_8968546326991991873[0] = 0;
   out_8968546326991991873[1] = 0;
   out_8968546326991991873[2] = 0;
   out_8968546326991991873[3] = 0;
   out_8968546326991991873[4] = 0;
   out_8968546326991991873[5] = 0;
   out_8968546326991991873[6] = 1;
   out_8968546326991991873[7] = 0;
   out_8968546326991991873[8] = 0;
}
void h_24(double *state, double *unused, double *out_880869654375610157) {
   out_880869654375610157[0] = state[4];
   out_880869654375610157[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4148224822336003610) {
   out_4148224822336003610[0] = 0;
   out_4148224822336003610[1] = 0;
   out_4148224822336003610[2] = 0;
   out_4148224822336003610[3] = 0;
   out_4148224822336003610[4] = 1;
   out_4148224822336003610[5] = 0;
   out_4148224822336003610[6] = 0;
   out_4148224822336003610[7] = 0;
   out_4148224822336003610[8] = 0;
   out_4148224822336003610[9] = 0;
   out_4148224822336003610[10] = 0;
   out_4148224822336003610[11] = 0;
   out_4148224822336003610[12] = 0;
   out_4148224822336003610[13] = 0;
   out_4148224822336003610[14] = 1;
   out_4148224822336003610[15] = 0;
   out_4148224822336003610[16] = 0;
   out_4148224822336003610[17] = 0;
}
void h_30(double *state, double *unused, double *out_8556234859568204194) {
   out_8556234859568204194[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8839207379848751803) {
   out_8839207379848751803[0] = 0;
   out_8839207379848751803[1] = 0;
   out_8839207379848751803[2] = 0;
   out_8839207379848751803[3] = 0;
   out_8839207379848751803[4] = 1;
   out_8839207379848751803[5] = 0;
   out_8839207379848751803[6] = 0;
   out_8839207379848751803[7] = 0;
   out_8839207379848751803[8] = 0;
}
void h_26(double *state, double *unused, double *out_7952918974891726677) {
   out_7952918974891726677[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8821343682607247839) {
   out_8821343682607247839[0] = 0;
   out_8821343682607247839[1] = 0;
   out_8821343682607247839[2] = 0;
   out_8821343682607247839[3] = 0;
   out_8821343682607247839[4] = 0;
   out_8821343682607247839[5] = 0;
   out_8821343682607247839[6] = 0;
   out_8821343682607247839[7] = 1;
   out_8821343682607247839[8] = 0;
}
void h_27(double *state, double *unused, double *out_268547644087507426) {
   out_268547644087507426[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6664444068048326892) {
   out_6664444068048326892[0] = 0;
   out_6664444068048326892[1] = 0;
   out_6664444068048326892[2] = 0;
   out_6664444068048326892[3] = 1;
   out_6664444068048326892[4] = 0;
   out_6664444068048326892[5] = 0;
   out_6664444068048326892[6] = 0;
   out_6664444068048326892[7] = 0;
   out_6664444068048326892[8] = 0;
}
void h_29(double *state, double *unused, double *out_425734312438636571) {
   out_425734312438636571[0] = state[1];
}
void H_29(double *state, double *unused, double *out_9097305349546407629) {
   out_9097305349546407629[0] = 0;
   out_9097305349546407629[1] = 1;
   out_9097305349546407629[2] = 0;
   out_9097305349546407629[3] = 0;
   out_9097305349546407629[4] = 0;
   out_9097305349546407629[5] = 0;
   out_9097305349546407629[6] = 0;
   out_9097305349546407629[7] = 0;
   out_9097305349546407629[8] = 0;
}
void h_28(double *state, double *unused, double *out_4733125623573338219) {
   out_4733125623573338219[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4267039707093613413) {
   out_4267039707093613413[0] = 1;
   out_4267039707093613413[1] = 0;
   out_4267039707093613413[2] = 0;
   out_4267039707093613413[3] = 0;
   out_4267039707093613413[4] = 0;
   out_4267039707093613413[5] = 0;
   out_4267039707093613413[6] = 0;
   out_4267039707093613413[7] = 0;
   out_4267039707093613413[8] = 0;
}
void h_31(double *state, double *unused, double *out_5520203488927545398) {
   out_5520203488927545398[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8999192288868952301) {
   out_8999192288868952301[0] = 0;
   out_8999192288868952301[1] = 0;
   out_8999192288868952301[2] = 0;
   out_8999192288868952301[3] = 0;
   out_8999192288868952301[4] = 0;
   out_8999192288868952301[5] = 0;
   out_8999192288868952301[6] = 0;
   out_8999192288868952301[7] = 0;
   out_8999192288868952301[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6986492762196806579) {
  err_fun(nom_x, delta_x, out_6986492762196806579);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2263396047980688855) {
  inv_err_fun(nom_x, true_x, out_2263396047980688855);
}
void car_H_mod_fun(double *state, double *out_402436849917620658) {
  H_mod_fun(state, out_402436849917620658);
}
void car_f_fun(double *state, double dt, double *out_2168537855026263863) {
  f_fun(state,  dt, out_2168537855026263863);
}
void car_F_fun(double *state, double dt, double *out_7745800668543318320) {
  F_fun(state,  dt, out_7745800668543318320);
}
void car_h_25(double *state, double *unused, double *out_1421900190648173534) {
  h_25(state, unused, out_1421900190648173534);
}
void car_H_25(double *state, double *unused, double *out_8968546326991991873) {
  H_25(state, unused, out_8968546326991991873);
}
void car_h_24(double *state, double *unused, double *out_880869654375610157) {
  h_24(state, unused, out_880869654375610157);
}
void car_H_24(double *state, double *unused, double *out_4148224822336003610) {
  H_24(state, unused, out_4148224822336003610);
}
void car_h_30(double *state, double *unused, double *out_8556234859568204194) {
  h_30(state, unused, out_8556234859568204194);
}
void car_H_30(double *state, double *unused, double *out_8839207379848751803) {
  H_30(state, unused, out_8839207379848751803);
}
void car_h_26(double *state, double *unused, double *out_7952918974891726677) {
  h_26(state, unused, out_7952918974891726677);
}
void car_H_26(double *state, double *unused, double *out_8821343682607247839) {
  H_26(state, unused, out_8821343682607247839);
}
void car_h_27(double *state, double *unused, double *out_268547644087507426) {
  h_27(state, unused, out_268547644087507426);
}
void car_H_27(double *state, double *unused, double *out_6664444068048326892) {
  H_27(state, unused, out_6664444068048326892);
}
void car_h_29(double *state, double *unused, double *out_425734312438636571) {
  h_29(state, unused, out_425734312438636571);
}
void car_H_29(double *state, double *unused, double *out_9097305349546407629) {
  H_29(state, unused, out_9097305349546407629);
}
void car_h_28(double *state, double *unused, double *out_4733125623573338219) {
  h_28(state, unused, out_4733125623573338219);
}
void car_H_28(double *state, double *unused, double *out_4267039707093613413) {
  H_28(state, unused, out_4267039707093613413);
}
void car_h_31(double *state, double *unused, double *out_5520203488927545398) {
  h_31(state, unused, out_5520203488927545398);
}
void car_H_31(double *state, double *unused, double *out_8999192288868952301) {
  H_31(state, unused, out_8999192288868952301);
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

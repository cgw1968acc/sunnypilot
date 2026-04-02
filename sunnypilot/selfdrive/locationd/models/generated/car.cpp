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
void err_fun(double *nom_x, double *delta_x, double *out_1525856032200297312) {
   out_1525856032200297312[0] = delta_x[0] + nom_x[0];
   out_1525856032200297312[1] = delta_x[1] + nom_x[1];
   out_1525856032200297312[2] = delta_x[2] + nom_x[2];
   out_1525856032200297312[3] = delta_x[3] + nom_x[3];
   out_1525856032200297312[4] = delta_x[4] + nom_x[4];
   out_1525856032200297312[5] = delta_x[5] + nom_x[5];
   out_1525856032200297312[6] = delta_x[6] + nom_x[6];
   out_1525856032200297312[7] = delta_x[7] + nom_x[7];
   out_1525856032200297312[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2947590959223528162) {
   out_2947590959223528162[0] = -nom_x[0] + true_x[0];
   out_2947590959223528162[1] = -nom_x[1] + true_x[1];
   out_2947590959223528162[2] = -nom_x[2] + true_x[2];
   out_2947590959223528162[3] = -nom_x[3] + true_x[3];
   out_2947590959223528162[4] = -nom_x[4] + true_x[4];
   out_2947590959223528162[5] = -nom_x[5] + true_x[5];
   out_2947590959223528162[6] = -nom_x[6] + true_x[6];
   out_2947590959223528162[7] = -nom_x[7] + true_x[7];
   out_2947590959223528162[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3078840736753877381) {
   out_3078840736753877381[0] = 1.0;
   out_3078840736753877381[1] = 0.0;
   out_3078840736753877381[2] = 0.0;
   out_3078840736753877381[3] = 0.0;
   out_3078840736753877381[4] = 0.0;
   out_3078840736753877381[5] = 0.0;
   out_3078840736753877381[6] = 0.0;
   out_3078840736753877381[7] = 0.0;
   out_3078840736753877381[8] = 0.0;
   out_3078840736753877381[9] = 0.0;
   out_3078840736753877381[10] = 1.0;
   out_3078840736753877381[11] = 0.0;
   out_3078840736753877381[12] = 0.0;
   out_3078840736753877381[13] = 0.0;
   out_3078840736753877381[14] = 0.0;
   out_3078840736753877381[15] = 0.0;
   out_3078840736753877381[16] = 0.0;
   out_3078840736753877381[17] = 0.0;
   out_3078840736753877381[18] = 0.0;
   out_3078840736753877381[19] = 0.0;
   out_3078840736753877381[20] = 1.0;
   out_3078840736753877381[21] = 0.0;
   out_3078840736753877381[22] = 0.0;
   out_3078840736753877381[23] = 0.0;
   out_3078840736753877381[24] = 0.0;
   out_3078840736753877381[25] = 0.0;
   out_3078840736753877381[26] = 0.0;
   out_3078840736753877381[27] = 0.0;
   out_3078840736753877381[28] = 0.0;
   out_3078840736753877381[29] = 0.0;
   out_3078840736753877381[30] = 1.0;
   out_3078840736753877381[31] = 0.0;
   out_3078840736753877381[32] = 0.0;
   out_3078840736753877381[33] = 0.0;
   out_3078840736753877381[34] = 0.0;
   out_3078840736753877381[35] = 0.0;
   out_3078840736753877381[36] = 0.0;
   out_3078840736753877381[37] = 0.0;
   out_3078840736753877381[38] = 0.0;
   out_3078840736753877381[39] = 0.0;
   out_3078840736753877381[40] = 1.0;
   out_3078840736753877381[41] = 0.0;
   out_3078840736753877381[42] = 0.0;
   out_3078840736753877381[43] = 0.0;
   out_3078840736753877381[44] = 0.0;
   out_3078840736753877381[45] = 0.0;
   out_3078840736753877381[46] = 0.0;
   out_3078840736753877381[47] = 0.0;
   out_3078840736753877381[48] = 0.0;
   out_3078840736753877381[49] = 0.0;
   out_3078840736753877381[50] = 1.0;
   out_3078840736753877381[51] = 0.0;
   out_3078840736753877381[52] = 0.0;
   out_3078840736753877381[53] = 0.0;
   out_3078840736753877381[54] = 0.0;
   out_3078840736753877381[55] = 0.0;
   out_3078840736753877381[56] = 0.0;
   out_3078840736753877381[57] = 0.0;
   out_3078840736753877381[58] = 0.0;
   out_3078840736753877381[59] = 0.0;
   out_3078840736753877381[60] = 1.0;
   out_3078840736753877381[61] = 0.0;
   out_3078840736753877381[62] = 0.0;
   out_3078840736753877381[63] = 0.0;
   out_3078840736753877381[64] = 0.0;
   out_3078840736753877381[65] = 0.0;
   out_3078840736753877381[66] = 0.0;
   out_3078840736753877381[67] = 0.0;
   out_3078840736753877381[68] = 0.0;
   out_3078840736753877381[69] = 0.0;
   out_3078840736753877381[70] = 1.0;
   out_3078840736753877381[71] = 0.0;
   out_3078840736753877381[72] = 0.0;
   out_3078840736753877381[73] = 0.0;
   out_3078840736753877381[74] = 0.0;
   out_3078840736753877381[75] = 0.0;
   out_3078840736753877381[76] = 0.0;
   out_3078840736753877381[77] = 0.0;
   out_3078840736753877381[78] = 0.0;
   out_3078840736753877381[79] = 0.0;
   out_3078840736753877381[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8225664519178041008) {
   out_8225664519178041008[0] = state[0];
   out_8225664519178041008[1] = state[1];
   out_8225664519178041008[2] = state[2];
   out_8225664519178041008[3] = state[3];
   out_8225664519178041008[4] = state[4];
   out_8225664519178041008[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8225664519178041008[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8225664519178041008[7] = state[7];
   out_8225664519178041008[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8925472146913299645) {
   out_8925472146913299645[0] = 1;
   out_8925472146913299645[1] = 0;
   out_8925472146913299645[2] = 0;
   out_8925472146913299645[3] = 0;
   out_8925472146913299645[4] = 0;
   out_8925472146913299645[5] = 0;
   out_8925472146913299645[6] = 0;
   out_8925472146913299645[7] = 0;
   out_8925472146913299645[8] = 0;
   out_8925472146913299645[9] = 0;
   out_8925472146913299645[10] = 1;
   out_8925472146913299645[11] = 0;
   out_8925472146913299645[12] = 0;
   out_8925472146913299645[13] = 0;
   out_8925472146913299645[14] = 0;
   out_8925472146913299645[15] = 0;
   out_8925472146913299645[16] = 0;
   out_8925472146913299645[17] = 0;
   out_8925472146913299645[18] = 0;
   out_8925472146913299645[19] = 0;
   out_8925472146913299645[20] = 1;
   out_8925472146913299645[21] = 0;
   out_8925472146913299645[22] = 0;
   out_8925472146913299645[23] = 0;
   out_8925472146913299645[24] = 0;
   out_8925472146913299645[25] = 0;
   out_8925472146913299645[26] = 0;
   out_8925472146913299645[27] = 0;
   out_8925472146913299645[28] = 0;
   out_8925472146913299645[29] = 0;
   out_8925472146913299645[30] = 1;
   out_8925472146913299645[31] = 0;
   out_8925472146913299645[32] = 0;
   out_8925472146913299645[33] = 0;
   out_8925472146913299645[34] = 0;
   out_8925472146913299645[35] = 0;
   out_8925472146913299645[36] = 0;
   out_8925472146913299645[37] = 0;
   out_8925472146913299645[38] = 0;
   out_8925472146913299645[39] = 0;
   out_8925472146913299645[40] = 1;
   out_8925472146913299645[41] = 0;
   out_8925472146913299645[42] = 0;
   out_8925472146913299645[43] = 0;
   out_8925472146913299645[44] = 0;
   out_8925472146913299645[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8925472146913299645[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8925472146913299645[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8925472146913299645[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8925472146913299645[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8925472146913299645[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8925472146913299645[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8925472146913299645[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8925472146913299645[53] = -9.8100000000000005*dt;
   out_8925472146913299645[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8925472146913299645[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8925472146913299645[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8925472146913299645[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8925472146913299645[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8925472146913299645[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8925472146913299645[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8925472146913299645[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8925472146913299645[62] = 0;
   out_8925472146913299645[63] = 0;
   out_8925472146913299645[64] = 0;
   out_8925472146913299645[65] = 0;
   out_8925472146913299645[66] = 0;
   out_8925472146913299645[67] = 0;
   out_8925472146913299645[68] = 0;
   out_8925472146913299645[69] = 0;
   out_8925472146913299645[70] = 1;
   out_8925472146913299645[71] = 0;
   out_8925472146913299645[72] = 0;
   out_8925472146913299645[73] = 0;
   out_8925472146913299645[74] = 0;
   out_8925472146913299645[75] = 0;
   out_8925472146913299645[76] = 0;
   out_8925472146913299645[77] = 0;
   out_8925472146913299645[78] = 0;
   out_8925472146913299645[79] = 0;
   out_8925472146913299645[80] = 1;
}
void h_25(double *state, double *unused, double *out_6213069847998954130) {
   out_6213069847998954130[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4029817760279522699) {
   out_4029817760279522699[0] = 0;
   out_4029817760279522699[1] = 0;
   out_4029817760279522699[2] = 0;
   out_4029817760279522699[3] = 0;
   out_4029817760279522699[4] = 0;
   out_4029817760279522699[5] = 0;
   out_4029817760279522699[6] = 1;
   out_4029817760279522699[7] = 0;
   out_4029817760279522699[8] = 0;
}
void h_24(double *state, double *unused, double *out_4714750995414331089) {
   out_4714750995414331089[0] = state[4];
   out_4714750995414331089[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7587743359081845835) {
   out_7587743359081845835[0] = 0;
   out_7587743359081845835[1] = 0;
   out_7587743359081845835[2] = 0;
   out_7587743359081845835[3] = 0;
   out_7587743359081845835[4] = 1;
   out_7587743359081845835[5] = 0;
   out_7587743359081845835[6] = 0;
   out_7587743359081845835[7] = 0;
   out_7587743359081845835[8] = 0;
   out_7587743359081845835[9] = 0;
   out_7587743359081845835[10] = 0;
   out_7587743359081845835[11] = 0;
   out_7587743359081845835[12] = 0;
   out_7587743359081845835[13] = 0;
   out_7587743359081845835[14] = 1;
   out_7587743359081845835[15] = 0;
   out_7587743359081845835[16] = 0;
   out_7587743359081845835[17] = 0;
}
void h_30(double *state, double *unused, double *out_8371990749282095834) {
   out_8371990749282095834[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7500235971938412162) {
   out_7500235971938412162[0] = 0;
   out_7500235971938412162[1] = 0;
   out_7500235971938412162[2] = 0;
   out_7500235971938412162[3] = 0;
   out_7500235971938412162[4] = 1;
   out_7500235971938412162[5] = 0;
   out_7500235971938412162[6] = 0;
   out_7500235971938412162[7] = 0;
   out_7500235971938412162[8] = 0;
}
void h_26(double *state, double *unused, double *out_609978422724744931) {
   out_609978422724744931[0] = state[7];
}
void H_26(double *state, double *unused, double *out_288314441405466475) {
   out_288314441405466475[0] = 0;
   out_288314441405466475[1] = 0;
   out_288314441405466475[2] = 0;
   out_288314441405466475[3] = 0;
   out_288314441405466475[4] = 0;
   out_288314441405466475[5] = 0;
   out_288314441405466475[6] = 0;
   out_288314441405466475[7] = 1;
   out_288314441405466475[8] = 0;
}
void h_27(double *state, double *unused, double *out_4522622013263273170) {
   out_4522622013263273170[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8771744789970714543) {
   out_8771744789970714543[0] = 0;
   out_8771744789970714543[1] = 0;
   out_8771744789970714543[2] = 0;
   out_8771744789970714543[3] = 1;
   out_8771744789970714543[4] = 0;
   out_8771744789970714543[5] = 0;
   out_8771744789970714543[6] = 0;
   out_8771744789970714543[7] = 0;
   out_8771744789970714543[8] = 0;
}
void h_29(double *state, double *unused, double *out_3675711807013606636) {
   out_3675711807013606636[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6990004627624019978) {
   out_6990004627624019978[0] = 0;
   out_6990004627624019978[1] = 1;
   out_6990004627624019978[2] = 0;
   out_6990004627624019978[3] = 0;
   out_6990004627624019978[4] = 0;
   out_6990004627624019978[5] = 0;
   out_6990004627624019978[6] = 0;
   out_6990004627624019978[7] = 0;
   out_6990004627624019978[8] = 0;
}
void h_28(double *state, double *unused, double *out_1485835961818940485) {
   out_1485835961818940485[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1975983046031632936) {
   out_1975983046031632936[0] = 1;
   out_1975983046031632936[1] = 0;
   out_1975983046031632936[2] = 0;
   out_1975983046031632936[3] = 0;
   out_1975983046031632936[4] = 0;
   out_1975983046031632936[5] = 0;
   out_1975983046031632936[6] = 0;
   out_1975983046031632936[7] = 0;
   out_1975983046031632936[8] = 0;
}
void h_31(double *state, double *unused, double *out_7756602718772405027) {
   out_7756602718772405027[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4060463722156483127) {
   out_4060463722156483127[0] = 0;
   out_4060463722156483127[1] = 0;
   out_4060463722156483127[2] = 0;
   out_4060463722156483127[3] = 0;
   out_4060463722156483127[4] = 0;
   out_4060463722156483127[5] = 0;
   out_4060463722156483127[6] = 0;
   out_4060463722156483127[7] = 0;
   out_4060463722156483127[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_1525856032200297312) {
  err_fun(nom_x, delta_x, out_1525856032200297312);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2947590959223528162) {
  inv_err_fun(nom_x, true_x, out_2947590959223528162);
}
void car_H_mod_fun(double *state, double *out_3078840736753877381) {
  H_mod_fun(state, out_3078840736753877381);
}
void car_f_fun(double *state, double dt, double *out_8225664519178041008) {
  f_fun(state,  dt, out_8225664519178041008);
}
void car_F_fun(double *state, double dt, double *out_8925472146913299645) {
  F_fun(state,  dt, out_8925472146913299645);
}
void car_h_25(double *state, double *unused, double *out_6213069847998954130) {
  h_25(state, unused, out_6213069847998954130);
}
void car_H_25(double *state, double *unused, double *out_4029817760279522699) {
  H_25(state, unused, out_4029817760279522699);
}
void car_h_24(double *state, double *unused, double *out_4714750995414331089) {
  h_24(state, unused, out_4714750995414331089);
}
void car_H_24(double *state, double *unused, double *out_7587743359081845835) {
  H_24(state, unused, out_7587743359081845835);
}
void car_h_30(double *state, double *unused, double *out_8371990749282095834) {
  h_30(state, unused, out_8371990749282095834);
}
void car_H_30(double *state, double *unused, double *out_7500235971938412162) {
  H_30(state, unused, out_7500235971938412162);
}
void car_h_26(double *state, double *unused, double *out_609978422724744931) {
  h_26(state, unused, out_609978422724744931);
}
void car_H_26(double *state, double *unused, double *out_288314441405466475) {
  H_26(state, unused, out_288314441405466475);
}
void car_h_27(double *state, double *unused, double *out_4522622013263273170) {
  h_27(state, unused, out_4522622013263273170);
}
void car_H_27(double *state, double *unused, double *out_8771744789970714543) {
  H_27(state, unused, out_8771744789970714543);
}
void car_h_29(double *state, double *unused, double *out_3675711807013606636) {
  h_29(state, unused, out_3675711807013606636);
}
void car_H_29(double *state, double *unused, double *out_6990004627624019978) {
  H_29(state, unused, out_6990004627624019978);
}
void car_h_28(double *state, double *unused, double *out_1485835961818940485) {
  h_28(state, unused, out_1485835961818940485);
}
void car_H_28(double *state, double *unused, double *out_1975983046031632936) {
  H_28(state, unused, out_1975983046031632936);
}
void car_h_31(double *state, double *unused, double *out_7756602718772405027) {
  h_31(state, unused, out_7756602718772405027);
}
void car_H_31(double *state, double *unused, double *out_4060463722156483127) {
  H_31(state, unused, out_4060463722156483127);
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

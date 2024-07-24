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
void err_fun(double *nom_x, double *delta_x, double *out_7939984129706908020) {
   out_7939984129706908020[0] = delta_x[0] + nom_x[0];
   out_7939984129706908020[1] = delta_x[1] + nom_x[1];
   out_7939984129706908020[2] = delta_x[2] + nom_x[2];
   out_7939984129706908020[3] = delta_x[3] + nom_x[3];
   out_7939984129706908020[4] = delta_x[4] + nom_x[4];
   out_7939984129706908020[5] = delta_x[5] + nom_x[5];
   out_7939984129706908020[6] = delta_x[6] + nom_x[6];
   out_7939984129706908020[7] = delta_x[7] + nom_x[7];
   out_7939984129706908020[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4500603217124712497) {
   out_4500603217124712497[0] = -nom_x[0] + true_x[0];
   out_4500603217124712497[1] = -nom_x[1] + true_x[1];
   out_4500603217124712497[2] = -nom_x[2] + true_x[2];
   out_4500603217124712497[3] = -nom_x[3] + true_x[3];
   out_4500603217124712497[4] = -nom_x[4] + true_x[4];
   out_4500603217124712497[5] = -nom_x[5] + true_x[5];
   out_4500603217124712497[6] = -nom_x[6] + true_x[6];
   out_4500603217124712497[7] = -nom_x[7] + true_x[7];
   out_4500603217124712497[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5439614909828061126) {
   out_5439614909828061126[0] = 1.0;
   out_5439614909828061126[1] = 0;
   out_5439614909828061126[2] = 0;
   out_5439614909828061126[3] = 0;
   out_5439614909828061126[4] = 0;
   out_5439614909828061126[5] = 0;
   out_5439614909828061126[6] = 0;
   out_5439614909828061126[7] = 0;
   out_5439614909828061126[8] = 0;
   out_5439614909828061126[9] = 0;
   out_5439614909828061126[10] = 1.0;
   out_5439614909828061126[11] = 0;
   out_5439614909828061126[12] = 0;
   out_5439614909828061126[13] = 0;
   out_5439614909828061126[14] = 0;
   out_5439614909828061126[15] = 0;
   out_5439614909828061126[16] = 0;
   out_5439614909828061126[17] = 0;
   out_5439614909828061126[18] = 0;
   out_5439614909828061126[19] = 0;
   out_5439614909828061126[20] = 1.0;
   out_5439614909828061126[21] = 0;
   out_5439614909828061126[22] = 0;
   out_5439614909828061126[23] = 0;
   out_5439614909828061126[24] = 0;
   out_5439614909828061126[25] = 0;
   out_5439614909828061126[26] = 0;
   out_5439614909828061126[27] = 0;
   out_5439614909828061126[28] = 0;
   out_5439614909828061126[29] = 0;
   out_5439614909828061126[30] = 1.0;
   out_5439614909828061126[31] = 0;
   out_5439614909828061126[32] = 0;
   out_5439614909828061126[33] = 0;
   out_5439614909828061126[34] = 0;
   out_5439614909828061126[35] = 0;
   out_5439614909828061126[36] = 0;
   out_5439614909828061126[37] = 0;
   out_5439614909828061126[38] = 0;
   out_5439614909828061126[39] = 0;
   out_5439614909828061126[40] = 1.0;
   out_5439614909828061126[41] = 0;
   out_5439614909828061126[42] = 0;
   out_5439614909828061126[43] = 0;
   out_5439614909828061126[44] = 0;
   out_5439614909828061126[45] = 0;
   out_5439614909828061126[46] = 0;
   out_5439614909828061126[47] = 0;
   out_5439614909828061126[48] = 0;
   out_5439614909828061126[49] = 0;
   out_5439614909828061126[50] = 1.0;
   out_5439614909828061126[51] = 0;
   out_5439614909828061126[52] = 0;
   out_5439614909828061126[53] = 0;
   out_5439614909828061126[54] = 0;
   out_5439614909828061126[55] = 0;
   out_5439614909828061126[56] = 0;
   out_5439614909828061126[57] = 0;
   out_5439614909828061126[58] = 0;
   out_5439614909828061126[59] = 0;
   out_5439614909828061126[60] = 1.0;
   out_5439614909828061126[61] = 0;
   out_5439614909828061126[62] = 0;
   out_5439614909828061126[63] = 0;
   out_5439614909828061126[64] = 0;
   out_5439614909828061126[65] = 0;
   out_5439614909828061126[66] = 0;
   out_5439614909828061126[67] = 0;
   out_5439614909828061126[68] = 0;
   out_5439614909828061126[69] = 0;
   out_5439614909828061126[70] = 1.0;
   out_5439614909828061126[71] = 0;
   out_5439614909828061126[72] = 0;
   out_5439614909828061126[73] = 0;
   out_5439614909828061126[74] = 0;
   out_5439614909828061126[75] = 0;
   out_5439614909828061126[76] = 0;
   out_5439614909828061126[77] = 0;
   out_5439614909828061126[78] = 0;
   out_5439614909828061126[79] = 0;
   out_5439614909828061126[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_396057045295170870) {
   out_396057045295170870[0] = state[0];
   out_396057045295170870[1] = state[1];
   out_396057045295170870[2] = state[2];
   out_396057045295170870[3] = state[3];
   out_396057045295170870[4] = state[4];
   out_396057045295170870[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_396057045295170870[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_396057045295170870[7] = state[7];
   out_396057045295170870[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1763107264307268110) {
   out_1763107264307268110[0] = 1;
   out_1763107264307268110[1] = 0;
   out_1763107264307268110[2] = 0;
   out_1763107264307268110[3] = 0;
   out_1763107264307268110[4] = 0;
   out_1763107264307268110[5] = 0;
   out_1763107264307268110[6] = 0;
   out_1763107264307268110[7] = 0;
   out_1763107264307268110[8] = 0;
   out_1763107264307268110[9] = 0;
   out_1763107264307268110[10] = 1;
   out_1763107264307268110[11] = 0;
   out_1763107264307268110[12] = 0;
   out_1763107264307268110[13] = 0;
   out_1763107264307268110[14] = 0;
   out_1763107264307268110[15] = 0;
   out_1763107264307268110[16] = 0;
   out_1763107264307268110[17] = 0;
   out_1763107264307268110[18] = 0;
   out_1763107264307268110[19] = 0;
   out_1763107264307268110[20] = 1;
   out_1763107264307268110[21] = 0;
   out_1763107264307268110[22] = 0;
   out_1763107264307268110[23] = 0;
   out_1763107264307268110[24] = 0;
   out_1763107264307268110[25] = 0;
   out_1763107264307268110[26] = 0;
   out_1763107264307268110[27] = 0;
   out_1763107264307268110[28] = 0;
   out_1763107264307268110[29] = 0;
   out_1763107264307268110[30] = 1;
   out_1763107264307268110[31] = 0;
   out_1763107264307268110[32] = 0;
   out_1763107264307268110[33] = 0;
   out_1763107264307268110[34] = 0;
   out_1763107264307268110[35] = 0;
   out_1763107264307268110[36] = 0;
   out_1763107264307268110[37] = 0;
   out_1763107264307268110[38] = 0;
   out_1763107264307268110[39] = 0;
   out_1763107264307268110[40] = 1;
   out_1763107264307268110[41] = 0;
   out_1763107264307268110[42] = 0;
   out_1763107264307268110[43] = 0;
   out_1763107264307268110[44] = 0;
   out_1763107264307268110[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1763107264307268110[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1763107264307268110[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1763107264307268110[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1763107264307268110[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1763107264307268110[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1763107264307268110[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1763107264307268110[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1763107264307268110[53] = -9.8000000000000007*dt;
   out_1763107264307268110[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1763107264307268110[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1763107264307268110[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1763107264307268110[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1763107264307268110[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1763107264307268110[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1763107264307268110[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1763107264307268110[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1763107264307268110[62] = 0;
   out_1763107264307268110[63] = 0;
   out_1763107264307268110[64] = 0;
   out_1763107264307268110[65] = 0;
   out_1763107264307268110[66] = 0;
   out_1763107264307268110[67] = 0;
   out_1763107264307268110[68] = 0;
   out_1763107264307268110[69] = 0;
   out_1763107264307268110[70] = 1;
   out_1763107264307268110[71] = 0;
   out_1763107264307268110[72] = 0;
   out_1763107264307268110[73] = 0;
   out_1763107264307268110[74] = 0;
   out_1763107264307268110[75] = 0;
   out_1763107264307268110[76] = 0;
   out_1763107264307268110[77] = 0;
   out_1763107264307268110[78] = 0;
   out_1763107264307268110[79] = 0;
   out_1763107264307268110[80] = 1;
}
void h_25(double *state, double *unused, double *out_2427701692207996360) {
   out_2427701692207996360[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3014214633440783213) {
   out_3014214633440783213[0] = 0;
   out_3014214633440783213[1] = 0;
   out_3014214633440783213[2] = 0;
   out_3014214633440783213[3] = 0;
   out_3014214633440783213[4] = 0;
   out_3014214633440783213[5] = 0;
   out_3014214633440783213[6] = 1;
   out_3014214633440783213[7] = 0;
   out_3014214633440783213[8] = 0;
}
void h_24(double *state, double *unused, double *out_6174353062213406939) {
   out_6174353062213406939[0] = state[4];
   out_6174353062213406939[1] = state[5];
}
void H_24(double *state, double *unused, double *out_6213850552628412012) {
   out_6213850552628412012[0] = 0;
   out_6213850552628412012[1] = 0;
   out_6213850552628412012[2] = 0;
   out_6213850552628412012[3] = 0;
   out_6213850552628412012[4] = 1;
   out_6213850552628412012[5] = 0;
   out_6213850552628412012[6] = 0;
   out_6213850552628412012[7] = 0;
   out_6213850552628412012[8] = 0;
   out_6213850552628412012[9] = 0;
   out_6213850552628412012[10] = 0;
   out_6213850552628412012[11] = 0;
   out_6213850552628412012[12] = 0;
   out_6213850552628412012[13] = 0;
   out_6213850552628412012[14] = 1;
   out_6213850552628412012[15] = 0;
   out_6213850552628412012[16] = 0;
   out_6213850552628412012[17] = 0;
}
void h_30(double *state, double *unused, double *out_2270515023856867215) {
   out_2270515023856867215[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7541910963568391411) {
   out_7541910963568391411[0] = 0;
   out_7541910963568391411[1] = 0;
   out_7541910963568391411[2] = 0;
   out_7541910963568391411[3] = 0;
   out_7541910963568391411[4] = 1;
   out_7541910963568391411[5] = 0;
   out_7541910963568391411[6] = 0;
   out_7541910963568391411[7] = 0;
   out_7541910963568391411[8] = 0;
}
void h_26(double *state, double *unused, double *out_5307301075301199900) {
   out_5307301075301199900[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6755717952314839437) {
   out_6755717952314839437[0] = 0;
   out_6755717952314839437[1] = 0;
   out_6755717952314839437[2] = 0;
   out_6755717952314839437[3] = 0;
   out_6755717952314839437[4] = 0;
   out_6755717952314839437[5] = 0;
   out_6755717952314839437[6] = 0;
   out_6755717952314839437[7] = 1;
   out_6755717952314839437[8] = 0;
}
void h_27(double *state, double *unused, double *out_7350902415471537191) {
   out_7350902415471537191[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5318316892384448194) {
   out_5318316892384448194[0] = 0;
   out_5318316892384448194[1] = 0;
   out_5318316892384448194[2] = 0;
   out_5318316892384448194[3] = 1;
   out_5318316892384448194[4] = 0;
   out_5318316892384448194[5] = 0;
   out_5318316892384448194[6] = 0;
   out_5318316892384448194[7] = 0;
   out_5318316892384448194[8] = 0;
}
void h_29(double *state, double *unused, double *out_4732247679936198564) {
   out_4732247679936198564[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7031679619253999227) {
   out_7031679619253999227[0] = 0;
   out_7031679619253999227[1] = 1;
   out_7031679619253999227[2] = 0;
   out_7031679619253999227[3] = 0;
   out_7031679619253999227[4] = 0;
   out_7031679619253999227[5] = 0;
   out_7031679619253999227[6] = 0;
   out_7031679619253999227[7] = 0;
   out_7031679619253999227[8] = 0;
}
void h_28(double *state, double *unused, double *out_6094168390577168780) {
   out_6094168390577168780[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6332665437386021815) {
   out_6332665437386021815[0] = 1;
   out_6332665437386021815[1] = 0;
   out_6332665437386021815[2] = 0;
   out_6332665437386021815[3] = 0;
   out_6332665437386021815[4] = 0;
   out_6332665437386021815[5] = 0;
   out_6332665437386021815[6] = 0;
   out_6332665437386021815[7] = 0;
   out_6332665437386021815[8] = 0;
}
void h_31(double *state, double *unused, double *out_1512454328617810106) {
   out_1512454328617810106[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7381926054548190913) {
   out_7381926054548190913[0] = 0;
   out_7381926054548190913[1] = 0;
   out_7381926054548190913[2] = 0;
   out_7381926054548190913[3] = 0;
   out_7381926054548190913[4] = 0;
   out_7381926054548190913[5] = 0;
   out_7381926054548190913[6] = 0;
   out_7381926054548190913[7] = 0;
   out_7381926054548190913[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7939984129706908020) {
  err_fun(nom_x, delta_x, out_7939984129706908020);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4500603217124712497) {
  inv_err_fun(nom_x, true_x, out_4500603217124712497);
}
void car_H_mod_fun(double *state, double *out_5439614909828061126) {
  H_mod_fun(state, out_5439614909828061126);
}
void car_f_fun(double *state, double dt, double *out_396057045295170870) {
  f_fun(state,  dt, out_396057045295170870);
}
void car_F_fun(double *state, double dt, double *out_1763107264307268110) {
  F_fun(state,  dt, out_1763107264307268110);
}
void car_h_25(double *state, double *unused, double *out_2427701692207996360) {
  h_25(state, unused, out_2427701692207996360);
}
void car_H_25(double *state, double *unused, double *out_3014214633440783213) {
  H_25(state, unused, out_3014214633440783213);
}
void car_h_24(double *state, double *unused, double *out_6174353062213406939) {
  h_24(state, unused, out_6174353062213406939);
}
void car_H_24(double *state, double *unused, double *out_6213850552628412012) {
  H_24(state, unused, out_6213850552628412012);
}
void car_h_30(double *state, double *unused, double *out_2270515023856867215) {
  h_30(state, unused, out_2270515023856867215);
}
void car_H_30(double *state, double *unused, double *out_7541910963568391411) {
  H_30(state, unused, out_7541910963568391411);
}
void car_h_26(double *state, double *unused, double *out_5307301075301199900) {
  h_26(state, unused, out_5307301075301199900);
}
void car_H_26(double *state, double *unused, double *out_6755717952314839437) {
  H_26(state, unused, out_6755717952314839437);
}
void car_h_27(double *state, double *unused, double *out_7350902415471537191) {
  h_27(state, unused, out_7350902415471537191);
}
void car_H_27(double *state, double *unused, double *out_5318316892384448194) {
  H_27(state, unused, out_5318316892384448194);
}
void car_h_29(double *state, double *unused, double *out_4732247679936198564) {
  h_29(state, unused, out_4732247679936198564);
}
void car_H_29(double *state, double *unused, double *out_7031679619253999227) {
  H_29(state, unused, out_7031679619253999227);
}
void car_h_28(double *state, double *unused, double *out_6094168390577168780) {
  h_28(state, unused, out_6094168390577168780);
}
void car_H_28(double *state, double *unused, double *out_6332665437386021815) {
  H_28(state, unused, out_6332665437386021815);
}
void car_h_31(double *state, double *unused, double *out_1512454328617810106) {
  h_31(state, unused, out_1512454328617810106);
}
void car_H_31(double *state, double *unused, double *out_7381926054548190913) {
  H_31(state, unused, out_7381926054548190913);
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

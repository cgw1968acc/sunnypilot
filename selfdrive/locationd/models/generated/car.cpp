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
void err_fun(double *nom_x, double *delta_x, double *out_1556433197202642724) {
   out_1556433197202642724[0] = delta_x[0] + nom_x[0];
   out_1556433197202642724[1] = delta_x[1] + nom_x[1];
   out_1556433197202642724[2] = delta_x[2] + nom_x[2];
   out_1556433197202642724[3] = delta_x[3] + nom_x[3];
   out_1556433197202642724[4] = delta_x[4] + nom_x[4];
   out_1556433197202642724[5] = delta_x[5] + nom_x[5];
   out_1556433197202642724[6] = delta_x[6] + nom_x[6];
   out_1556433197202642724[7] = delta_x[7] + nom_x[7];
   out_1556433197202642724[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7503824718266888943) {
   out_7503824718266888943[0] = -nom_x[0] + true_x[0];
   out_7503824718266888943[1] = -nom_x[1] + true_x[1];
   out_7503824718266888943[2] = -nom_x[2] + true_x[2];
   out_7503824718266888943[3] = -nom_x[3] + true_x[3];
   out_7503824718266888943[4] = -nom_x[4] + true_x[4];
   out_7503824718266888943[5] = -nom_x[5] + true_x[5];
   out_7503824718266888943[6] = -nom_x[6] + true_x[6];
   out_7503824718266888943[7] = -nom_x[7] + true_x[7];
   out_7503824718266888943[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4233392152597311220) {
   out_4233392152597311220[0] = 1.0;
   out_4233392152597311220[1] = 0;
   out_4233392152597311220[2] = 0;
   out_4233392152597311220[3] = 0;
   out_4233392152597311220[4] = 0;
   out_4233392152597311220[5] = 0;
   out_4233392152597311220[6] = 0;
   out_4233392152597311220[7] = 0;
   out_4233392152597311220[8] = 0;
   out_4233392152597311220[9] = 0;
   out_4233392152597311220[10] = 1.0;
   out_4233392152597311220[11] = 0;
   out_4233392152597311220[12] = 0;
   out_4233392152597311220[13] = 0;
   out_4233392152597311220[14] = 0;
   out_4233392152597311220[15] = 0;
   out_4233392152597311220[16] = 0;
   out_4233392152597311220[17] = 0;
   out_4233392152597311220[18] = 0;
   out_4233392152597311220[19] = 0;
   out_4233392152597311220[20] = 1.0;
   out_4233392152597311220[21] = 0;
   out_4233392152597311220[22] = 0;
   out_4233392152597311220[23] = 0;
   out_4233392152597311220[24] = 0;
   out_4233392152597311220[25] = 0;
   out_4233392152597311220[26] = 0;
   out_4233392152597311220[27] = 0;
   out_4233392152597311220[28] = 0;
   out_4233392152597311220[29] = 0;
   out_4233392152597311220[30] = 1.0;
   out_4233392152597311220[31] = 0;
   out_4233392152597311220[32] = 0;
   out_4233392152597311220[33] = 0;
   out_4233392152597311220[34] = 0;
   out_4233392152597311220[35] = 0;
   out_4233392152597311220[36] = 0;
   out_4233392152597311220[37] = 0;
   out_4233392152597311220[38] = 0;
   out_4233392152597311220[39] = 0;
   out_4233392152597311220[40] = 1.0;
   out_4233392152597311220[41] = 0;
   out_4233392152597311220[42] = 0;
   out_4233392152597311220[43] = 0;
   out_4233392152597311220[44] = 0;
   out_4233392152597311220[45] = 0;
   out_4233392152597311220[46] = 0;
   out_4233392152597311220[47] = 0;
   out_4233392152597311220[48] = 0;
   out_4233392152597311220[49] = 0;
   out_4233392152597311220[50] = 1.0;
   out_4233392152597311220[51] = 0;
   out_4233392152597311220[52] = 0;
   out_4233392152597311220[53] = 0;
   out_4233392152597311220[54] = 0;
   out_4233392152597311220[55] = 0;
   out_4233392152597311220[56] = 0;
   out_4233392152597311220[57] = 0;
   out_4233392152597311220[58] = 0;
   out_4233392152597311220[59] = 0;
   out_4233392152597311220[60] = 1.0;
   out_4233392152597311220[61] = 0;
   out_4233392152597311220[62] = 0;
   out_4233392152597311220[63] = 0;
   out_4233392152597311220[64] = 0;
   out_4233392152597311220[65] = 0;
   out_4233392152597311220[66] = 0;
   out_4233392152597311220[67] = 0;
   out_4233392152597311220[68] = 0;
   out_4233392152597311220[69] = 0;
   out_4233392152597311220[70] = 1.0;
   out_4233392152597311220[71] = 0;
   out_4233392152597311220[72] = 0;
   out_4233392152597311220[73] = 0;
   out_4233392152597311220[74] = 0;
   out_4233392152597311220[75] = 0;
   out_4233392152597311220[76] = 0;
   out_4233392152597311220[77] = 0;
   out_4233392152597311220[78] = 0;
   out_4233392152597311220[79] = 0;
   out_4233392152597311220[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6213018529770018042) {
   out_6213018529770018042[0] = state[0];
   out_6213018529770018042[1] = state[1];
   out_6213018529770018042[2] = state[2];
   out_6213018529770018042[3] = state[3];
   out_6213018529770018042[4] = state[4];
   out_6213018529770018042[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6213018529770018042[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6213018529770018042[7] = state[7];
   out_6213018529770018042[8] = state[8];
}
void F_fun(double *state, double dt, double *out_487389055243717466) {
   out_487389055243717466[0] = 1;
   out_487389055243717466[1] = 0;
   out_487389055243717466[2] = 0;
   out_487389055243717466[3] = 0;
   out_487389055243717466[4] = 0;
   out_487389055243717466[5] = 0;
   out_487389055243717466[6] = 0;
   out_487389055243717466[7] = 0;
   out_487389055243717466[8] = 0;
   out_487389055243717466[9] = 0;
   out_487389055243717466[10] = 1;
   out_487389055243717466[11] = 0;
   out_487389055243717466[12] = 0;
   out_487389055243717466[13] = 0;
   out_487389055243717466[14] = 0;
   out_487389055243717466[15] = 0;
   out_487389055243717466[16] = 0;
   out_487389055243717466[17] = 0;
   out_487389055243717466[18] = 0;
   out_487389055243717466[19] = 0;
   out_487389055243717466[20] = 1;
   out_487389055243717466[21] = 0;
   out_487389055243717466[22] = 0;
   out_487389055243717466[23] = 0;
   out_487389055243717466[24] = 0;
   out_487389055243717466[25] = 0;
   out_487389055243717466[26] = 0;
   out_487389055243717466[27] = 0;
   out_487389055243717466[28] = 0;
   out_487389055243717466[29] = 0;
   out_487389055243717466[30] = 1;
   out_487389055243717466[31] = 0;
   out_487389055243717466[32] = 0;
   out_487389055243717466[33] = 0;
   out_487389055243717466[34] = 0;
   out_487389055243717466[35] = 0;
   out_487389055243717466[36] = 0;
   out_487389055243717466[37] = 0;
   out_487389055243717466[38] = 0;
   out_487389055243717466[39] = 0;
   out_487389055243717466[40] = 1;
   out_487389055243717466[41] = 0;
   out_487389055243717466[42] = 0;
   out_487389055243717466[43] = 0;
   out_487389055243717466[44] = 0;
   out_487389055243717466[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_487389055243717466[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_487389055243717466[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_487389055243717466[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_487389055243717466[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_487389055243717466[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_487389055243717466[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_487389055243717466[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_487389055243717466[53] = -9.8000000000000007*dt;
   out_487389055243717466[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_487389055243717466[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_487389055243717466[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_487389055243717466[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_487389055243717466[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_487389055243717466[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_487389055243717466[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_487389055243717466[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_487389055243717466[62] = 0;
   out_487389055243717466[63] = 0;
   out_487389055243717466[64] = 0;
   out_487389055243717466[65] = 0;
   out_487389055243717466[66] = 0;
   out_487389055243717466[67] = 0;
   out_487389055243717466[68] = 0;
   out_487389055243717466[69] = 0;
   out_487389055243717466[70] = 1;
   out_487389055243717466[71] = 0;
   out_487389055243717466[72] = 0;
   out_487389055243717466[73] = 0;
   out_487389055243717466[74] = 0;
   out_487389055243717466[75] = 0;
   out_487389055243717466[76] = 0;
   out_487389055243717466[77] = 0;
   out_487389055243717466[78] = 0;
   out_487389055243717466[79] = 0;
   out_487389055243717466[80] = 1;
}
void h_25(double *state, double *unused, double *out_5116885968642153577) {
   out_5116885968642153577[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7205861781851670427) {
   out_7205861781851670427[0] = 0;
   out_7205861781851670427[1] = 0;
   out_7205861781851670427[2] = 0;
   out_7205861781851670427[3] = 0;
   out_7205861781851670427[4] = 0;
   out_7205861781851670427[5] = 0;
   out_7205861781851670427[6] = 1;
   out_7205861781851670427[7] = 0;
   out_7205861781851670427[8] = 0;
}
void h_24(double *state, double *unused, double *out_1411115276105148042) {
   out_1411115276105148042[0] = state[4];
   out_1411115276105148042[1] = state[5];
}
void H_24(double *state, double *unused, double *out_9015174507879012627) {
   out_9015174507879012627[0] = 0;
   out_9015174507879012627[1] = 0;
   out_9015174507879012627[2] = 0;
   out_9015174507879012627[3] = 0;
   out_9015174507879012627[4] = 1;
   out_9015174507879012627[5] = 0;
   out_9015174507879012627[6] = 0;
   out_9015174507879012627[7] = 0;
   out_9015174507879012627[8] = 0;
   out_9015174507879012627[9] = 0;
   out_9015174507879012627[10] = 0;
   out_9015174507879012627[11] = 0;
   out_9015174507879012627[12] = 0;
   out_9015174507879012627[13] = 0;
   out_9015174507879012627[14] = 1;
   out_9015174507879012627[15] = 0;
   out_9015174507879012627[16] = 0;
   out_9015174507879012627[17] = 0;
}
void h_30(double *state, double *unused, double *out_2547928186945489211) {
   out_2547928186945489211[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4324191950366264434) {
   out_4324191950366264434[0] = 0;
   out_4324191950366264434[1] = 0;
   out_4324191950366264434[2] = 0;
   out_4324191950366264434[3] = 0;
   out_4324191950366264434[4] = 1;
   out_4324191950366264434[5] = 0;
   out_4324191950366264434[6] = 0;
   out_4324191950366264434[7] = 0;
   out_4324191950366264434[8] = 0;
}
void h_26(double *state, double *unused, double *out_3151244071621966728) {
   out_3151244071621966728[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3464358462977614203) {
   out_3464358462977614203[0] = 0;
   out_3464358462977614203[1] = 0;
   out_3464358462977614203[2] = 0;
   out_3464358462977614203[3] = 0;
   out_3464358462977614203[4] = 0;
   out_3464358462977614203[5] = 0;
   out_3464358462977614203[6] = 0;
   out_3464358462977614203[7] = 1;
   out_3464358462977614203[8] = 0;
}
void h_27(double *state, double *unused, double *out_7074033383108350785) {
   out_7074033383108350785[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6498955262166689345) {
   out_6498955262166689345[0] = 0;
   out_6498955262166689345[1] = 0;
   out_6498955262166689345[2] = 0;
   out_6498955262166689345[3] = 1;
   out_6498955262166689345[4] = 0;
   out_6498955262166689345[5] = 0;
   out_6498955262166689345[6] = 0;
   out_6498955262166689345[7] = 0;
   out_6498955262166689345[8] = 0;
}
void h_29(double *state, double *unused, double *out_3491438039847151646) {
   out_3491438039847151646[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3813960606051872250) {
   out_3813960606051872250[0] = 0;
   out_3813960606051872250[1] = 1;
   out_3813960606051872250[2] = 0;
   out_3813960606051872250[3] = 0;
   out_3813960606051872250[4] = 0;
   out_3813960606051872250[5] = 0;
   out_3813960606051872250[6] = 0;
   out_3813960606051872250[7] = 0;
   out_3813960606051872250[8] = 0;
}
void h_28(double *state, double *unused, double *out_6371037422940355186) {
   out_6371037422940355186[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5152027067603780664) {
   out_5152027067603780664[0] = 1;
   out_5152027067603780664[1] = 0;
   out_5152027067603780664[2] = 0;
   out_5152027067603780664[3] = 0;
   out_5152027067603780664[4] = 0;
   out_5152027067603780664[5] = 0;
   out_5152027067603780664[6] = 0;
   out_5152027067603780664[7] = 0;
   out_5152027067603780664[8] = 0;
}
void h_31(double *state, double *unused, double *out_3522991508076388307) {
   out_3522991508076388307[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7236507743728630855) {
   out_7236507743728630855[0] = 0;
   out_7236507743728630855[1] = 0;
   out_7236507743728630855[2] = 0;
   out_7236507743728630855[3] = 0;
   out_7236507743728630855[4] = 0;
   out_7236507743728630855[5] = 0;
   out_7236507743728630855[6] = 0;
   out_7236507743728630855[7] = 0;
   out_7236507743728630855[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_1556433197202642724) {
  err_fun(nom_x, delta_x, out_1556433197202642724);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7503824718266888943) {
  inv_err_fun(nom_x, true_x, out_7503824718266888943);
}
void car_H_mod_fun(double *state, double *out_4233392152597311220) {
  H_mod_fun(state, out_4233392152597311220);
}
void car_f_fun(double *state, double dt, double *out_6213018529770018042) {
  f_fun(state,  dt, out_6213018529770018042);
}
void car_F_fun(double *state, double dt, double *out_487389055243717466) {
  F_fun(state,  dt, out_487389055243717466);
}
void car_h_25(double *state, double *unused, double *out_5116885968642153577) {
  h_25(state, unused, out_5116885968642153577);
}
void car_H_25(double *state, double *unused, double *out_7205861781851670427) {
  H_25(state, unused, out_7205861781851670427);
}
void car_h_24(double *state, double *unused, double *out_1411115276105148042) {
  h_24(state, unused, out_1411115276105148042);
}
void car_H_24(double *state, double *unused, double *out_9015174507879012627) {
  H_24(state, unused, out_9015174507879012627);
}
void car_h_30(double *state, double *unused, double *out_2547928186945489211) {
  h_30(state, unused, out_2547928186945489211);
}
void car_H_30(double *state, double *unused, double *out_4324191950366264434) {
  H_30(state, unused, out_4324191950366264434);
}
void car_h_26(double *state, double *unused, double *out_3151244071621966728) {
  h_26(state, unused, out_3151244071621966728);
}
void car_H_26(double *state, double *unused, double *out_3464358462977614203) {
  H_26(state, unused, out_3464358462977614203);
}
void car_h_27(double *state, double *unused, double *out_7074033383108350785) {
  h_27(state, unused, out_7074033383108350785);
}
void car_H_27(double *state, double *unused, double *out_6498955262166689345) {
  H_27(state, unused, out_6498955262166689345);
}
void car_h_29(double *state, double *unused, double *out_3491438039847151646) {
  h_29(state, unused, out_3491438039847151646);
}
void car_H_29(double *state, double *unused, double *out_3813960606051872250) {
  H_29(state, unused, out_3813960606051872250);
}
void car_h_28(double *state, double *unused, double *out_6371037422940355186) {
  h_28(state, unused, out_6371037422940355186);
}
void car_H_28(double *state, double *unused, double *out_5152027067603780664) {
  H_28(state, unused, out_5152027067603780664);
}
void car_h_31(double *state, double *unused, double *out_3522991508076388307) {
  h_31(state, unused, out_3522991508076388307);
}
void car_H_31(double *state, double *unused, double *out_7236507743728630855) {
  H_31(state, unused, out_7236507743728630855);
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

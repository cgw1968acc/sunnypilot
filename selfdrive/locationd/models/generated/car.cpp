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
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5011033113370860982) {
   out_5011033113370860982[0] = delta_x[0] + nom_x[0];
   out_5011033113370860982[1] = delta_x[1] + nom_x[1];
   out_5011033113370860982[2] = delta_x[2] + nom_x[2];
   out_5011033113370860982[3] = delta_x[3] + nom_x[3];
   out_5011033113370860982[4] = delta_x[4] + nom_x[4];
   out_5011033113370860982[5] = delta_x[5] + nom_x[5];
   out_5011033113370860982[6] = delta_x[6] + nom_x[6];
   out_5011033113370860982[7] = delta_x[7] + nom_x[7];
   out_5011033113370860982[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7387888235897089854) {
   out_7387888235897089854[0] = -nom_x[0] + true_x[0];
   out_7387888235897089854[1] = -nom_x[1] + true_x[1];
   out_7387888235897089854[2] = -nom_x[2] + true_x[2];
   out_7387888235897089854[3] = -nom_x[3] + true_x[3];
   out_7387888235897089854[4] = -nom_x[4] + true_x[4];
   out_7387888235897089854[5] = -nom_x[5] + true_x[5];
   out_7387888235897089854[6] = -nom_x[6] + true_x[6];
   out_7387888235897089854[7] = -nom_x[7] + true_x[7];
   out_7387888235897089854[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7046635644391981245) {
   out_7046635644391981245[0] = 1.0;
   out_7046635644391981245[1] = 0.0;
   out_7046635644391981245[2] = 0.0;
   out_7046635644391981245[3] = 0.0;
   out_7046635644391981245[4] = 0.0;
   out_7046635644391981245[5] = 0.0;
   out_7046635644391981245[6] = 0.0;
   out_7046635644391981245[7] = 0.0;
   out_7046635644391981245[8] = 0.0;
   out_7046635644391981245[9] = 0.0;
   out_7046635644391981245[10] = 1.0;
   out_7046635644391981245[11] = 0.0;
   out_7046635644391981245[12] = 0.0;
   out_7046635644391981245[13] = 0.0;
   out_7046635644391981245[14] = 0.0;
   out_7046635644391981245[15] = 0.0;
   out_7046635644391981245[16] = 0.0;
   out_7046635644391981245[17] = 0.0;
   out_7046635644391981245[18] = 0.0;
   out_7046635644391981245[19] = 0.0;
   out_7046635644391981245[20] = 1.0;
   out_7046635644391981245[21] = 0.0;
   out_7046635644391981245[22] = 0.0;
   out_7046635644391981245[23] = 0.0;
   out_7046635644391981245[24] = 0.0;
   out_7046635644391981245[25] = 0.0;
   out_7046635644391981245[26] = 0.0;
   out_7046635644391981245[27] = 0.0;
   out_7046635644391981245[28] = 0.0;
   out_7046635644391981245[29] = 0.0;
   out_7046635644391981245[30] = 1.0;
   out_7046635644391981245[31] = 0.0;
   out_7046635644391981245[32] = 0.0;
   out_7046635644391981245[33] = 0.0;
   out_7046635644391981245[34] = 0.0;
   out_7046635644391981245[35] = 0.0;
   out_7046635644391981245[36] = 0.0;
   out_7046635644391981245[37] = 0.0;
   out_7046635644391981245[38] = 0.0;
   out_7046635644391981245[39] = 0.0;
   out_7046635644391981245[40] = 1.0;
   out_7046635644391981245[41] = 0.0;
   out_7046635644391981245[42] = 0.0;
   out_7046635644391981245[43] = 0.0;
   out_7046635644391981245[44] = 0.0;
   out_7046635644391981245[45] = 0.0;
   out_7046635644391981245[46] = 0.0;
   out_7046635644391981245[47] = 0.0;
   out_7046635644391981245[48] = 0.0;
   out_7046635644391981245[49] = 0.0;
   out_7046635644391981245[50] = 1.0;
   out_7046635644391981245[51] = 0.0;
   out_7046635644391981245[52] = 0.0;
   out_7046635644391981245[53] = 0.0;
   out_7046635644391981245[54] = 0.0;
   out_7046635644391981245[55] = 0.0;
   out_7046635644391981245[56] = 0.0;
   out_7046635644391981245[57] = 0.0;
   out_7046635644391981245[58] = 0.0;
   out_7046635644391981245[59] = 0.0;
   out_7046635644391981245[60] = 1.0;
   out_7046635644391981245[61] = 0.0;
   out_7046635644391981245[62] = 0.0;
   out_7046635644391981245[63] = 0.0;
   out_7046635644391981245[64] = 0.0;
   out_7046635644391981245[65] = 0.0;
   out_7046635644391981245[66] = 0.0;
   out_7046635644391981245[67] = 0.0;
   out_7046635644391981245[68] = 0.0;
   out_7046635644391981245[69] = 0.0;
   out_7046635644391981245[70] = 1.0;
   out_7046635644391981245[71] = 0.0;
   out_7046635644391981245[72] = 0.0;
   out_7046635644391981245[73] = 0.0;
   out_7046635644391981245[74] = 0.0;
   out_7046635644391981245[75] = 0.0;
   out_7046635644391981245[76] = 0.0;
   out_7046635644391981245[77] = 0.0;
   out_7046635644391981245[78] = 0.0;
   out_7046635644391981245[79] = 0.0;
   out_7046635644391981245[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2555231531843821050) {
   out_2555231531843821050[0] = state[0];
   out_2555231531843821050[1] = state[1];
   out_2555231531843821050[2] = state[2];
   out_2555231531843821050[3] = state[3];
   out_2555231531843821050[4] = state[4];
   out_2555231531843821050[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2555231531843821050[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2555231531843821050[7] = state[7];
   out_2555231531843821050[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3197627356211555357) {
   out_3197627356211555357[0] = 1;
   out_3197627356211555357[1] = 0;
   out_3197627356211555357[2] = 0;
   out_3197627356211555357[3] = 0;
   out_3197627356211555357[4] = 0;
   out_3197627356211555357[5] = 0;
   out_3197627356211555357[6] = 0;
   out_3197627356211555357[7] = 0;
   out_3197627356211555357[8] = 0;
   out_3197627356211555357[9] = 0;
   out_3197627356211555357[10] = 1;
   out_3197627356211555357[11] = 0;
   out_3197627356211555357[12] = 0;
   out_3197627356211555357[13] = 0;
   out_3197627356211555357[14] = 0;
   out_3197627356211555357[15] = 0;
   out_3197627356211555357[16] = 0;
   out_3197627356211555357[17] = 0;
   out_3197627356211555357[18] = 0;
   out_3197627356211555357[19] = 0;
   out_3197627356211555357[20] = 1;
   out_3197627356211555357[21] = 0;
   out_3197627356211555357[22] = 0;
   out_3197627356211555357[23] = 0;
   out_3197627356211555357[24] = 0;
   out_3197627356211555357[25] = 0;
   out_3197627356211555357[26] = 0;
   out_3197627356211555357[27] = 0;
   out_3197627356211555357[28] = 0;
   out_3197627356211555357[29] = 0;
   out_3197627356211555357[30] = 1;
   out_3197627356211555357[31] = 0;
   out_3197627356211555357[32] = 0;
   out_3197627356211555357[33] = 0;
   out_3197627356211555357[34] = 0;
   out_3197627356211555357[35] = 0;
   out_3197627356211555357[36] = 0;
   out_3197627356211555357[37] = 0;
   out_3197627356211555357[38] = 0;
   out_3197627356211555357[39] = 0;
   out_3197627356211555357[40] = 1;
   out_3197627356211555357[41] = 0;
   out_3197627356211555357[42] = 0;
   out_3197627356211555357[43] = 0;
   out_3197627356211555357[44] = 0;
   out_3197627356211555357[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3197627356211555357[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3197627356211555357[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3197627356211555357[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3197627356211555357[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3197627356211555357[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3197627356211555357[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3197627356211555357[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3197627356211555357[53] = -9.8000000000000007*dt;
   out_3197627356211555357[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3197627356211555357[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3197627356211555357[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3197627356211555357[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3197627356211555357[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3197627356211555357[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3197627356211555357[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3197627356211555357[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3197627356211555357[62] = 0;
   out_3197627356211555357[63] = 0;
   out_3197627356211555357[64] = 0;
   out_3197627356211555357[65] = 0;
   out_3197627356211555357[66] = 0;
   out_3197627356211555357[67] = 0;
   out_3197627356211555357[68] = 0;
   out_3197627356211555357[69] = 0;
   out_3197627356211555357[70] = 1;
   out_3197627356211555357[71] = 0;
   out_3197627356211555357[72] = 0;
   out_3197627356211555357[73] = 0;
   out_3197627356211555357[74] = 0;
   out_3197627356211555357[75] = 0;
   out_3197627356211555357[76] = 0;
   out_3197627356211555357[77] = 0;
   out_3197627356211555357[78] = 0;
   out_3197627356211555357[79] = 0;
   out_3197627356211555357[80] = 1;
}
void h_25(double *state, double *unused, double *out_3006913549208278608) {
   out_3006913549208278608[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4903085469427540420) {
   out_4903085469427540420[0] = 0;
   out_4903085469427540420[1] = 0;
   out_4903085469427540420[2] = 0;
   out_4903085469427540420[3] = 0;
   out_4903085469427540420[4] = 0;
   out_4903085469427540420[5] = 0;
   out_4903085469427540420[6] = 1;
   out_4903085469427540420[7] = 0;
   out_4903085469427540420[8] = 0;
}
void h_24(double *state, double *unused, double *out_7421029612925360575) {
   out_7421029612925360575[0] = state[4];
   out_7421029612925360575[1] = state[5];
}
void H_24(double *state, double *unused, double *out_34270604399833568) {
   out_34270604399833568[0] = 0;
   out_34270604399833568[1] = 0;
   out_34270604399833568[2] = 0;
   out_34270604399833568[3] = 0;
   out_34270604399833568[4] = 1;
   out_34270604399833568[5] = 0;
   out_34270604399833568[6] = 0;
   out_34270604399833568[7] = 0;
   out_34270604399833568[8] = 0;
   out_34270604399833568[9] = 0;
   out_34270604399833568[10] = 0;
   out_34270604399833568[11] = 0;
   out_34270604399833568[12] = 0;
   out_34270604399833568[13] = 0;
   out_34270604399833568[14] = 1;
   out_34270604399833568[15] = 0;
   out_34270604399833568[16] = 0;
   out_34270604399833568[17] = 0;
}
void h_30(double *state, double *unused, double *out_5954639539563763569) {
   out_5954639539563763569[0] = state[4];
}
void H_30(double *state, double *unused, double *out_375389139299932222) {
   out_375389139299932222[0] = 0;
   out_375389139299932222[1] = 0;
   out_375389139299932222[2] = 0;
   out_375389139299932222[3] = 0;
   out_375389139299932222[4] = 1;
   out_375389139299932222[5] = 0;
   out_375389139299932222[6] = 0;
   out_375389139299932222[7] = 0;
   out_375389139299932222[8] = 0;
}
void h_26(double *state, double *unused, double *out_8991425591008096254) {
   out_8991425591008096254[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1161582150553484196) {
   out_1161582150553484196[0] = 0;
   out_1161582150553484196[1] = 0;
   out_1161582150553484196[2] = 0;
   out_1161582150553484196[3] = 0;
   out_1161582150553484196[4] = 0;
   out_1161582150553484196[5] = 0;
   out_1161582150553484196[6] = 0;
   out_1161582150553484196[7] = 1;
   out_1161582150553484196[8] = 0;
}
void h_27(double *state, double *unused, double *out_3988997642543576720) {
   out_3988997642543576720[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1799374172500492689) {
   out_1799374172500492689[0] = 0;
   out_1799374172500492689[1] = 0;
   out_1799374172500492689[2] = 0;
   out_1799374172500492689[3] = 1;
   out_1799374172500492689[4] = 0;
   out_1799374172500492689[5] = 0;
   out_1799374172500492689[6] = 0;
   out_1799374172500492689[7] = 0;
   out_1799374172500492689[8] = 0;
}
void h_29(double *state, double *unused, double *out_4264191704828082609) {
   out_4264191704828082609[0] = state[1];
}
void H_29(double *state, double *unused, double *out_885620483614324406) {
   out_885620483614324406[0] = 0;
   out_885620483614324406[1] = 1;
   out_885620483614324406[2] = 0;
   out_885620483614324406[3] = 0;
   out_885620483614324406[4] = 0;
   out_885620483614324406[5] = 0;
   out_885620483614324406[6] = 0;
   out_885620483614324406[7] = 0;
   out_885620483614324406[8] = 0;
}
void h_28(double *state, double *unused, double *out_973382355137716512) {
   out_973382355137716512[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4196778533455206168) {
   out_4196778533455206168[0] = 1;
   out_4196778533455206168[1] = 0;
   out_4196778533455206168[2] = 0;
   out_4196778533455206168[3] = 0;
   out_4196778533455206168[4] = 0;
   out_4196778533455206168[5] = 0;
   out_4196778533455206168[6] = 0;
   out_4196778533455206168[7] = 0;
   out_4196778533455206168[8] = 0;
}
void h_31(double *state, double *unused, double *out_4874359101545770577) {
   out_4874359101545770577[0] = state[8];
}
void H_31(double *state, double *unused, double *out_535374048320132720) {
   out_535374048320132720[0] = 0;
   out_535374048320132720[1] = 0;
   out_535374048320132720[2] = 0;
   out_535374048320132720[3] = 0;
   out_535374048320132720[4] = 0;
   out_535374048320132720[5] = 0;
   out_535374048320132720[6] = 0;
   out_535374048320132720[7] = 0;
   out_535374048320132720[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5011033113370860982) {
  err_fun(nom_x, delta_x, out_5011033113370860982);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7387888235897089854) {
  inv_err_fun(nom_x, true_x, out_7387888235897089854);
}
void car_H_mod_fun(double *state, double *out_7046635644391981245) {
  H_mod_fun(state, out_7046635644391981245);
}
void car_f_fun(double *state, double dt, double *out_2555231531843821050) {
  f_fun(state,  dt, out_2555231531843821050);
}
void car_F_fun(double *state, double dt, double *out_3197627356211555357) {
  F_fun(state,  dt, out_3197627356211555357);
}
void car_h_25(double *state, double *unused, double *out_3006913549208278608) {
  h_25(state, unused, out_3006913549208278608);
}
void car_H_25(double *state, double *unused, double *out_4903085469427540420) {
  H_25(state, unused, out_4903085469427540420);
}
void car_h_24(double *state, double *unused, double *out_7421029612925360575) {
  h_24(state, unused, out_7421029612925360575);
}
void car_H_24(double *state, double *unused, double *out_34270604399833568) {
  H_24(state, unused, out_34270604399833568);
}
void car_h_30(double *state, double *unused, double *out_5954639539563763569) {
  h_30(state, unused, out_5954639539563763569);
}
void car_H_30(double *state, double *unused, double *out_375389139299932222) {
  H_30(state, unused, out_375389139299932222);
}
void car_h_26(double *state, double *unused, double *out_8991425591008096254) {
  h_26(state, unused, out_8991425591008096254);
}
void car_H_26(double *state, double *unused, double *out_1161582150553484196) {
  H_26(state, unused, out_1161582150553484196);
}
void car_h_27(double *state, double *unused, double *out_3988997642543576720) {
  h_27(state, unused, out_3988997642543576720);
}
void car_H_27(double *state, double *unused, double *out_1799374172500492689) {
  H_27(state, unused, out_1799374172500492689);
}
void car_h_29(double *state, double *unused, double *out_4264191704828082609) {
  h_29(state, unused, out_4264191704828082609);
}
void car_H_29(double *state, double *unused, double *out_885620483614324406) {
  H_29(state, unused, out_885620483614324406);
}
void car_h_28(double *state, double *unused, double *out_973382355137716512) {
  h_28(state, unused, out_973382355137716512);
}
void car_H_28(double *state, double *unused, double *out_4196778533455206168) {
  H_28(state, unused, out_4196778533455206168);
}
void car_h_31(double *state, double *unused, double *out_4874359101545770577) {
  h_31(state, unused, out_4874359101545770577);
}
void car_H_31(double *state, double *unused, double *out_535374048320132720) {
  H_31(state, unused, out_535374048320132720);
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

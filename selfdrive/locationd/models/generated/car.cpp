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
void err_fun(double *nom_x, double *delta_x, double *out_5177580849545065691) {
   out_5177580849545065691[0] = delta_x[0] + nom_x[0];
   out_5177580849545065691[1] = delta_x[1] + nom_x[1];
   out_5177580849545065691[2] = delta_x[2] + nom_x[2];
   out_5177580849545065691[3] = delta_x[3] + nom_x[3];
   out_5177580849545065691[4] = delta_x[4] + nom_x[4];
   out_5177580849545065691[5] = delta_x[5] + nom_x[5];
   out_5177580849545065691[6] = delta_x[6] + nom_x[6];
   out_5177580849545065691[7] = delta_x[7] + nom_x[7];
   out_5177580849545065691[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_632571021260590143) {
   out_632571021260590143[0] = -nom_x[0] + true_x[0];
   out_632571021260590143[1] = -nom_x[1] + true_x[1];
   out_632571021260590143[2] = -nom_x[2] + true_x[2];
   out_632571021260590143[3] = -nom_x[3] + true_x[3];
   out_632571021260590143[4] = -nom_x[4] + true_x[4];
   out_632571021260590143[5] = -nom_x[5] + true_x[5];
   out_632571021260590143[6] = -nom_x[6] + true_x[6];
   out_632571021260590143[7] = -nom_x[7] + true_x[7];
   out_632571021260590143[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2143122989661037326) {
   out_2143122989661037326[0] = 1.0;
   out_2143122989661037326[1] = 0;
   out_2143122989661037326[2] = 0;
   out_2143122989661037326[3] = 0;
   out_2143122989661037326[4] = 0;
   out_2143122989661037326[5] = 0;
   out_2143122989661037326[6] = 0;
   out_2143122989661037326[7] = 0;
   out_2143122989661037326[8] = 0;
   out_2143122989661037326[9] = 0;
   out_2143122989661037326[10] = 1.0;
   out_2143122989661037326[11] = 0;
   out_2143122989661037326[12] = 0;
   out_2143122989661037326[13] = 0;
   out_2143122989661037326[14] = 0;
   out_2143122989661037326[15] = 0;
   out_2143122989661037326[16] = 0;
   out_2143122989661037326[17] = 0;
   out_2143122989661037326[18] = 0;
   out_2143122989661037326[19] = 0;
   out_2143122989661037326[20] = 1.0;
   out_2143122989661037326[21] = 0;
   out_2143122989661037326[22] = 0;
   out_2143122989661037326[23] = 0;
   out_2143122989661037326[24] = 0;
   out_2143122989661037326[25] = 0;
   out_2143122989661037326[26] = 0;
   out_2143122989661037326[27] = 0;
   out_2143122989661037326[28] = 0;
   out_2143122989661037326[29] = 0;
   out_2143122989661037326[30] = 1.0;
   out_2143122989661037326[31] = 0;
   out_2143122989661037326[32] = 0;
   out_2143122989661037326[33] = 0;
   out_2143122989661037326[34] = 0;
   out_2143122989661037326[35] = 0;
   out_2143122989661037326[36] = 0;
   out_2143122989661037326[37] = 0;
   out_2143122989661037326[38] = 0;
   out_2143122989661037326[39] = 0;
   out_2143122989661037326[40] = 1.0;
   out_2143122989661037326[41] = 0;
   out_2143122989661037326[42] = 0;
   out_2143122989661037326[43] = 0;
   out_2143122989661037326[44] = 0;
   out_2143122989661037326[45] = 0;
   out_2143122989661037326[46] = 0;
   out_2143122989661037326[47] = 0;
   out_2143122989661037326[48] = 0;
   out_2143122989661037326[49] = 0;
   out_2143122989661037326[50] = 1.0;
   out_2143122989661037326[51] = 0;
   out_2143122989661037326[52] = 0;
   out_2143122989661037326[53] = 0;
   out_2143122989661037326[54] = 0;
   out_2143122989661037326[55] = 0;
   out_2143122989661037326[56] = 0;
   out_2143122989661037326[57] = 0;
   out_2143122989661037326[58] = 0;
   out_2143122989661037326[59] = 0;
   out_2143122989661037326[60] = 1.0;
   out_2143122989661037326[61] = 0;
   out_2143122989661037326[62] = 0;
   out_2143122989661037326[63] = 0;
   out_2143122989661037326[64] = 0;
   out_2143122989661037326[65] = 0;
   out_2143122989661037326[66] = 0;
   out_2143122989661037326[67] = 0;
   out_2143122989661037326[68] = 0;
   out_2143122989661037326[69] = 0;
   out_2143122989661037326[70] = 1.0;
   out_2143122989661037326[71] = 0;
   out_2143122989661037326[72] = 0;
   out_2143122989661037326[73] = 0;
   out_2143122989661037326[74] = 0;
   out_2143122989661037326[75] = 0;
   out_2143122989661037326[76] = 0;
   out_2143122989661037326[77] = 0;
   out_2143122989661037326[78] = 0;
   out_2143122989661037326[79] = 0;
   out_2143122989661037326[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5823499246669271172) {
   out_5823499246669271172[0] = state[0];
   out_5823499246669271172[1] = state[1];
   out_5823499246669271172[2] = state[2];
   out_5823499246669271172[3] = state[3];
   out_5823499246669271172[4] = state[4];
   out_5823499246669271172[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5823499246669271172[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5823499246669271172[7] = state[7];
   out_5823499246669271172[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6908189206762765849) {
   out_6908189206762765849[0] = 1;
   out_6908189206762765849[1] = 0;
   out_6908189206762765849[2] = 0;
   out_6908189206762765849[3] = 0;
   out_6908189206762765849[4] = 0;
   out_6908189206762765849[5] = 0;
   out_6908189206762765849[6] = 0;
   out_6908189206762765849[7] = 0;
   out_6908189206762765849[8] = 0;
   out_6908189206762765849[9] = 0;
   out_6908189206762765849[10] = 1;
   out_6908189206762765849[11] = 0;
   out_6908189206762765849[12] = 0;
   out_6908189206762765849[13] = 0;
   out_6908189206762765849[14] = 0;
   out_6908189206762765849[15] = 0;
   out_6908189206762765849[16] = 0;
   out_6908189206762765849[17] = 0;
   out_6908189206762765849[18] = 0;
   out_6908189206762765849[19] = 0;
   out_6908189206762765849[20] = 1;
   out_6908189206762765849[21] = 0;
   out_6908189206762765849[22] = 0;
   out_6908189206762765849[23] = 0;
   out_6908189206762765849[24] = 0;
   out_6908189206762765849[25] = 0;
   out_6908189206762765849[26] = 0;
   out_6908189206762765849[27] = 0;
   out_6908189206762765849[28] = 0;
   out_6908189206762765849[29] = 0;
   out_6908189206762765849[30] = 1;
   out_6908189206762765849[31] = 0;
   out_6908189206762765849[32] = 0;
   out_6908189206762765849[33] = 0;
   out_6908189206762765849[34] = 0;
   out_6908189206762765849[35] = 0;
   out_6908189206762765849[36] = 0;
   out_6908189206762765849[37] = 0;
   out_6908189206762765849[38] = 0;
   out_6908189206762765849[39] = 0;
   out_6908189206762765849[40] = 1;
   out_6908189206762765849[41] = 0;
   out_6908189206762765849[42] = 0;
   out_6908189206762765849[43] = 0;
   out_6908189206762765849[44] = 0;
   out_6908189206762765849[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6908189206762765849[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6908189206762765849[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6908189206762765849[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6908189206762765849[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6908189206762765849[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6908189206762765849[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6908189206762765849[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6908189206762765849[53] = -9.8000000000000007*dt;
   out_6908189206762765849[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6908189206762765849[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6908189206762765849[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6908189206762765849[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6908189206762765849[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6908189206762765849[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6908189206762765849[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6908189206762765849[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6908189206762765849[62] = 0;
   out_6908189206762765849[63] = 0;
   out_6908189206762765849[64] = 0;
   out_6908189206762765849[65] = 0;
   out_6908189206762765849[66] = 0;
   out_6908189206762765849[67] = 0;
   out_6908189206762765849[68] = 0;
   out_6908189206762765849[69] = 0;
   out_6908189206762765849[70] = 1;
   out_6908189206762765849[71] = 0;
   out_6908189206762765849[72] = 0;
   out_6908189206762765849[73] = 0;
   out_6908189206762765849[74] = 0;
   out_6908189206762765849[75] = 0;
   out_6908189206762765849[76] = 0;
   out_6908189206762765849[77] = 0;
   out_6908189206762765849[78] = 0;
   out_6908189206762765849[79] = 0;
   out_6908189206762765849[80] = 1;
}
void h_25(double *state, double *unused, double *out_8207864553779973215) {
   out_8207864553779973215[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1901783904286363903) {
   out_1901783904286363903[0] = 0;
   out_1901783904286363903[1] = 0;
   out_1901783904286363903[2] = 0;
   out_1901783904286363903[3] = 0;
   out_1901783904286363903[4] = 0;
   out_1901783904286363903[5] = 0;
   out_1901783904286363903[6] = 1;
   out_1901783904286363903[7] = 0;
   out_1901783904286363903[8] = 0;
}
void h_24(double *state, double *unused, double *out_3833629854148761959) {
   out_3833629854148761959[0] = state[4];
   out_3833629854148761959[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8477355710877882004) {
   out_8477355710877882004[0] = 0;
   out_8477355710877882004[1] = 0;
   out_8477355710877882004[2] = 0;
   out_8477355710877882004[3] = 0;
   out_8477355710877882004[4] = 1;
   out_8477355710877882004[5] = 0;
   out_8477355710877882004[6] = 0;
   out_8477355710877882004[7] = 0;
   out_8477355710877882004[8] = 0;
   out_8477355710877882004[9] = 0;
   out_8477355710877882004[10] = 0;
   out_8477355710877882004[11] = 0;
   out_8477355710877882004[12] = 0;
   out_8477355710877882004[13] = 0;
   out_8477355710877882004[14] = 1;
   out_8477355710877882004[15] = 0;
   out_8477355710877882004[16] = 0;
   out_8477355710877882004[17] = 0;
}
void h_30(double *state, double *unused, double *out_8365051222131102360) {
   out_8365051222131102360[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8818474245777980658) {
   out_8818474245777980658[0] = 0;
   out_8818474245777980658[1] = 0;
   out_8818474245777980658[2] = 0;
   out_8818474245777980658[3] = 0;
   out_8818474245777980658[4] = 1;
   out_8818474245777980658[5] = 0;
   out_8818474245777980658[6] = 0;
   out_8818474245777980658[7] = 0;
   out_8818474245777980658[8] = 0;
}
void h_26(double *state, double *unused, double *out_3023183986245071797) {
   out_3023183986245071797[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5206309874047164504) {
   out_5206309874047164504[0] = 0;
   out_5206309874047164504[1] = 0;
   out_5206309874047164504[2] = 0;
   out_5206309874047164504[3] = 0;
   out_5206309874047164504[4] = 0;
   out_5206309874047164504[5] = 0;
   out_5206309874047164504[6] = 0;
   out_5206309874047164504[7] = 1;
   out_5206309874047164504[8] = 0;
}
void h_27(double *state, double *unused, double *out_7202093468485245716) {
   out_7202093468485245716[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6643710933977555747) {
   out_6643710933977555747[0] = 0;
   out_6643710933977555747[1] = 0;
   out_6643710933977555747[2] = 0;
   out_6643710933977555747[3] = 1;
   out_6643710933977555747[4] = 0;
   out_6643710933977555747[5] = 0;
   out_6643710933977555747[6] = 0;
   out_6643710933977555747[7] = 0;
   out_6643710933977555747[8] = 0;
}
void h_29(double *state, double *unused, double *out_431258242134894780) {
   out_431258242134894780[0] = state[1];
}
void H_29(double *state, double *unused, double *out_9118038483617178774) {
   out_9118038483617178774[0] = 0;
   out_9118038483617178774[1] = 1;
   out_9118038483617178774[2] = 0;
   out_9118038483617178774[3] = 0;
   out_9118038483617178774[4] = 0;
   out_9118038483617178774[5] = 0;
   out_9118038483617178774[6] = 0;
   out_9118038483617178774[7] = 0;
   out_9118038483617178774[8] = 0;
}
void h_28(double *state, double *unused, double *out_6018743389388980953) {
   out_6018743389388980953[0] = state[0];
}
void H_28(double *state, double *unused, double *out_152050809961525860) {
   out_152050809961525860[0] = 1;
   out_152050809961525860[1] = 0;
   out_152050809961525860[2] = 0;
   out_152050809961525860[3] = 0;
   out_152050809961525860[4] = 0;
   out_152050809961525860[5] = 0;
   out_152050809961525860[6] = 0;
   out_152050809961525860[7] = 0;
   out_152050809961525860[8] = 0;
}
void h_31(double *state, double *unused, double *out_8483058616064479104) {
   out_8483058616064479104[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8978459154798181156) {
   out_8978459154798181156[0] = 0;
   out_8978459154798181156[1] = 0;
   out_8978459154798181156[2] = 0;
   out_8978459154798181156[3] = 0;
   out_8978459154798181156[4] = 0;
   out_8978459154798181156[5] = 0;
   out_8978459154798181156[6] = 0;
   out_8978459154798181156[7] = 0;
   out_8978459154798181156[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5177580849545065691) {
  err_fun(nom_x, delta_x, out_5177580849545065691);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_632571021260590143) {
  inv_err_fun(nom_x, true_x, out_632571021260590143);
}
void car_H_mod_fun(double *state, double *out_2143122989661037326) {
  H_mod_fun(state, out_2143122989661037326);
}
void car_f_fun(double *state, double dt, double *out_5823499246669271172) {
  f_fun(state,  dt, out_5823499246669271172);
}
void car_F_fun(double *state, double dt, double *out_6908189206762765849) {
  F_fun(state,  dt, out_6908189206762765849);
}
void car_h_25(double *state, double *unused, double *out_8207864553779973215) {
  h_25(state, unused, out_8207864553779973215);
}
void car_H_25(double *state, double *unused, double *out_1901783904286363903) {
  H_25(state, unused, out_1901783904286363903);
}
void car_h_24(double *state, double *unused, double *out_3833629854148761959) {
  h_24(state, unused, out_3833629854148761959);
}
void car_H_24(double *state, double *unused, double *out_8477355710877882004) {
  H_24(state, unused, out_8477355710877882004);
}
void car_h_30(double *state, double *unused, double *out_8365051222131102360) {
  h_30(state, unused, out_8365051222131102360);
}
void car_H_30(double *state, double *unused, double *out_8818474245777980658) {
  H_30(state, unused, out_8818474245777980658);
}
void car_h_26(double *state, double *unused, double *out_3023183986245071797) {
  h_26(state, unused, out_3023183986245071797);
}
void car_H_26(double *state, double *unused, double *out_5206309874047164504) {
  H_26(state, unused, out_5206309874047164504);
}
void car_h_27(double *state, double *unused, double *out_7202093468485245716) {
  h_27(state, unused, out_7202093468485245716);
}
void car_H_27(double *state, double *unused, double *out_6643710933977555747) {
  H_27(state, unused, out_6643710933977555747);
}
void car_h_29(double *state, double *unused, double *out_431258242134894780) {
  h_29(state, unused, out_431258242134894780);
}
void car_H_29(double *state, double *unused, double *out_9118038483617178774) {
  H_29(state, unused, out_9118038483617178774);
}
void car_h_28(double *state, double *unused, double *out_6018743389388980953) {
  h_28(state, unused, out_6018743389388980953);
}
void car_H_28(double *state, double *unused, double *out_152050809961525860) {
  H_28(state, unused, out_152050809961525860);
}
void car_h_31(double *state, double *unused, double *out_8483058616064479104) {
  h_31(state, unused, out_8483058616064479104);
}
void car_H_31(double *state, double *unused, double *out_8978459154798181156) {
  H_31(state, unused, out_8978459154798181156);
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

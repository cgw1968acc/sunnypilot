#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8052617336508823134) {
   out_8052617336508823134[0] = delta_x[0] + nom_x[0];
   out_8052617336508823134[1] = delta_x[1] + nom_x[1];
   out_8052617336508823134[2] = delta_x[2] + nom_x[2];
   out_8052617336508823134[3] = delta_x[3] + nom_x[3];
   out_8052617336508823134[4] = delta_x[4] + nom_x[4];
   out_8052617336508823134[5] = delta_x[5] + nom_x[5];
   out_8052617336508823134[6] = delta_x[6] + nom_x[6];
   out_8052617336508823134[7] = delta_x[7] + nom_x[7];
   out_8052617336508823134[8] = delta_x[8] + nom_x[8];
   out_8052617336508823134[9] = delta_x[9] + nom_x[9];
   out_8052617336508823134[10] = delta_x[10] + nom_x[10];
   out_8052617336508823134[11] = delta_x[11] + nom_x[11];
   out_8052617336508823134[12] = delta_x[12] + nom_x[12];
   out_8052617336508823134[13] = delta_x[13] + nom_x[13];
   out_8052617336508823134[14] = delta_x[14] + nom_x[14];
   out_8052617336508823134[15] = delta_x[15] + nom_x[15];
   out_8052617336508823134[16] = delta_x[16] + nom_x[16];
   out_8052617336508823134[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_9128085987645078449) {
   out_9128085987645078449[0] = -nom_x[0] + true_x[0];
   out_9128085987645078449[1] = -nom_x[1] + true_x[1];
   out_9128085987645078449[2] = -nom_x[2] + true_x[2];
   out_9128085987645078449[3] = -nom_x[3] + true_x[3];
   out_9128085987645078449[4] = -nom_x[4] + true_x[4];
   out_9128085987645078449[5] = -nom_x[5] + true_x[5];
   out_9128085987645078449[6] = -nom_x[6] + true_x[6];
   out_9128085987645078449[7] = -nom_x[7] + true_x[7];
   out_9128085987645078449[8] = -nom_x[8] + true_x[8];
   out_9128085987645078449[9] = -nom_x[9] + true_x[9];
   out_9128085987645078449[10] = -nom_x[10] + true_x[10];
   out_9128085987645078449[11] = -nom_x[11] + true_x[11];
   out_9128085987645078449[12] = -nom_x[12] + true_x[12];
   out_9128085987645078449[13] = -nom_x[13] + true_x[13];
   out_9128085987645078449[14] = -nom_x[14] + true_x[14];
   out_9128085987645078449[15] = -nom_x[15] + true_x[15];
   out_9128085987645078449[16] = -nom_x[16] + true_x[16];
   out_9128085987645078449[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_3361570693863352746) {
   out_3361570693863352746[0] = 1.0;
   out_3361570693863352746[1] = 0.0;
   out_3361570693863352746[2] = 0.0;
   out_3361570693863352746[3] = 0.0;
   out_3361570693863352746[4] = 0.0;
   out_3361570693863352746[5] = 0.0;
   out_3361570693863352746[6] = 0.0;
   out_3361570693863352746[7] = 0.0;
   out_3361570693863352746[8] = 0.0;
   out_3361570693863352746[9] = 0.0;
   out_3361570693863352746[10] = 0.0;
   out_3361570693863352746[11] = 0.0;
   out_3361570693863352746[12] = 0.0;
   out_3361570693863352746[13] = 0.0;
   out_3361570693863352746[14] = 0.0;
   out_3361570693863352746[15] = 0.0;
   out_3361570693863352746[16] = 0.0;
   out_3361570693863352746[17] = 0.0;
   out_3361570693863352746[18] = 0.0;
   out_3361570693863352746[19] = 1.0;
   out_3361570693863352746[20] = 0.0;
   out_3361570693863352746[21] = 0.0;
   out_3361570693863352746[22] = 0.0;
   out_3361570693863352746[23] = 0.0;
   out_3361570693863352746[24] = 0.0;
   out_3361570693863352746[25] = 0.0;
   out_3361570693863352746[26] = 0.0;
   out_3361570693863352746[27] = 0.0;
   out_3361570693863352746[28] = 0.0;
   out_3361570693863352746[29] = 0.0;
   out_3361570693863352746[30] = 0.0;
   out_3361570693863352746[31] = 0.0;
   out_3361570693863352746[32] = 0.0;
   out_3361570693863352746[33] = 0.0;
   out_3361570693863352746[34] = 0.0;
   out_3361570693863352746[35] = 0.0;
   out_3361570693863352746[36] = 0.0;
   out_3361570693863352746[37] = 0.0;
   out_3361570693863352746[38] = 1.0;
   out_3361570693863352746[39] = 0.0;
   out_3361570693863352746[40] = 0.0;
   out_3361570693863352746[41] = 0.0;
   out_3361570693863352746[42] = 0.0;
   out_3361570693863352746[43] = 0.0;
   out_3361570693863352746[44] = 0.0;
   out_3361570693863352746[45] = 0.0;
   out_3361570693863352746[46] = 0.0;
   out_3361570693863352746[47] = 0.0;
   out_3361570693863352746[48] = 0.0;
   out_3361570693863352746[49] = 0.0;
   out_3361570693863352746[50] = 0.0;
   out_3361570693863352746[51] = 0.0;
   out_3361570693863352746[52] = 0.0;
   out_3361570693863352746[53] = 0.0;
   out_3361570693863352746[54] = 0.0;
   out_3361570693863352746[55] = 0.0;
   out_3361570693863352746[56] = 0.0;
   out_3361570693863352746[57] = 1.0;
   out_3361570693863352746[58] = 0.0;
   out_3361570693863352746[59] = 0.0;
   out_3361570693863352746[60] = 0.0;
   out_3361570693863352746[61] = 0.0;
   out_3361570693863352746[62] = 0.0;
   out_3361570693863352746[63] = 0.0;
   out_3361570693863352746[64] = 0.0;
   out_3361570693863352746[65] = 0.0;
   out_3361570693863352746[66] = 0.0;
   out_3361570693863352746[67] = 0.0;
   out_3361570693863352746[68] = 0.0;
   out_3361570693863352746[69] = 0.0;
   out_3361570693863352746[70] = 0.0;
   out_3361570693863352746[71] = 0.0;
   out_3361570693863352746[72] = 0.0;
   out_3361570693863352746[73] = 0.0;
   out_3361570693863352746[74] = 0.0;
   out_3361570693863352746[75] = 0.0;
   out_3361570693863352746[76] = 1.0;
   out_3361570693863352746[77] = 0.0;
   out_3361570693863352746[78] = 0.0;
   out_3361570693863352746[79] = 0.0;
   out_3361570693863352746[80] = 0.0;
   out_3361570693863352746[81] = 0.0;
   out_3361570693863352746[82] = 0.0;
   out_3361570693863352746[83] = 0.0;
   out_3361570693863352746[84] = 0.0;
   out_3361570693863352746[85] = 0.0;
   out_3361570693863352746[86] = 0.0;
   out_3361570693863352746[87] = 0.0;
   out_3361570693863352746[88] = 0.0;
   out_3361570693863352746[89] = 0.0;
   out_3361570693863352746[90] = 0.0;
   out_3361570693863352746[91] = 0.0;
   out_3361570693863352746[92] = 0.0;
   out_3361570693863352746[93] = 0.0;
   out_3361570693863352746[94] = 0.0;
   out_3361570693863352746[95] = 1.0;
   out_3361570693863352746[96] = 0.0;
   out_3361570693863352746[97] = 0.0;
   out_3361570693863352746[98] = 0.0;
   out_3361570693863352746[99] = 0.0;
   out_3361570693863352746[100] = 0.0;
   out_3361570693863352746[101] = 0.0;
   out_3361570693863352746[102] = 0.0;
   out_3361570693863352746[103] = 0.0;
   out_3361570693863352746[104] = 0.0;
   out_3361570693863352746[105] = 0.0;
   out_3361570693863352746[106] = 0.0;
   out_3361570693863352746[107] = 0.0;
   out_3361570693863352746[108] = 0.0;
   out_3361570693863352746[109] = 0.0;
   out_3361570693863352746[110] = 0.0;
   out_3361570693863352746[111] = 0.0;
   out_3361570693863352746[112] = 0.0;
   out_3361570693863352746[113] = 0.0;
   out_3361570693863352746[114] = 1.0;
   out_3361570693863352746[115] = 0.0;
   out_3361570693863352746[116] = 0.0;
   out_3361570693863352746[117] = 0.0;
   out_3361570693863352746[118] = 0.0;
   out_3361570693863352746[119] = 0.0;
   out_3361570693863352746[120] = 0.0;
   out_3361570693863352746[121] = 0.0;
   out_3361570693863352746[122] = 0.0;
   out_3361570693863352746[123] = 0.0;
   out_3361570693863352746[124] = 0.0;
   out_3361570693863352746[125] = 0.0;
   out_3361570693863352746[126] = 0.0;
   out_3361570693863352746[127] = 0.0;
   out_3361570693863352746[128] = 0.0;
   out_3361570693863352746[129] = 0.0;
   out_3361570693863352746[130] = 0.0;
   out_3361570693863352746[131] = 0.0;
   out_3361570693863352746[132] = 0.0;
   out_3361570693863352746[133] = 1.0;
   out_3361570693863352746[134] = 0.0;
   out_3361570693863352746[135] = 0.0;
   out_3361570693863352746[136] = 0.0;
   out_3361570693863352746[137] = 0.0;
   out_3361570693863352746[138] = 0.0;
   out_3361570693863352746[139] = 0.0;
   out_3361570693863352746[140] = 0.0;
   out_3361570693863352746[141] = 0.0;
   out_3361570693863352746[142] = 0.0;
   out_3361570693863352746[143] = 0.0;
   out_3361570693863352746[144] = 0.0;
   out_3361570693863352746[145] = 0.0;
   out_3361570693863352746[146] = 0.0;
   out_3361570693863352746[147] = 0.0;
   out_3361570693863352746[148] = 0.0;
   out_3361570693863352746[149] = 0.0;
   out_3361570693863352746[150] = 0.0;
   out_3361570693863352746[151] = 0.0;
   out_3361570693863352746[152] = 1.0;
   out_3361570693863352746[153] = 0.0;
   out_3361570693863352746[154] = 0.0;
   out_3361570693863352746[155] = 0.0;
   out_3361570693863352746[156] = 0.0;
   out_3361570693863352746[157] = 0.0;
   out_3361570693863352746[158] = 0.0;
   out_3361570693863352746[159] = 0.0;
   out_3361570693863352746[160] = 0.0;
   out_3361570693863352746[161] = 0.0;
   out_3361570693863352746[162] = 0.0;
   out_3361570693863352746[163] = 0.0;
   out_3361570693863352746[164] = 0.0;
   out_3361570693863352746[165] = 0.0;
   out_3361570693863352746[166] = 0.0;
   out_3361570693863352746[167] = 0.0;
   out_3361570693863352746[168] = 0.0;
   out_3361570693863352746[169] = 0.0;
   out_3361570693863352746[170] = 0.0;
   out_3361570693863352746[171] = 1.0;
   out_3361570693863352746[172] = 0.0;
   out_3361570693863352746[173] = 0.0;
   out_3361570693863352746[174] = 0.0;
   out_3361570693863352746[175] = 0.0;
   out_3361570693863352746[176] = 0.0;
   out_3361570693863352746[177] = 0.0;
   out_3361570693863352746[178] = 0.0;
   out_3361570693863352746[179] = 0.0;
   out_3361570693863352746[180] = 0.0;
   out_3361570693863352746[181] = 0.0;
   out_3361570693863352746[182] = 0.0;
   out_3361570693863352746[183] = 0.0;
   out_3361570693863352746[184] = 0.0;
   out_3361570693863352746[185] = 0.0;
   out_3361570693863352746[186] = 0.0;
   out_3361570693863352746[187] = 0.0;
   out_3361570693863352746[188] = 0.0;
   out_3361570693863352746[189] = 0.0;
   out_3361570693863352746[190] = 1.0;
   out_3361570693863352746[191] = 0.0;
   out_3361570693863352746[192] = 0.0;
   out_3361570693863352746[193] = 0.0;
   out_3361570693863352746[194] = 0.0;
   out_3361570693863352746[195] = 0.0;
   out_3361570693863352746[196] = 0.0;
   out_3361570693863352746[197] = 0.0;
   out_3361570693863352746[198] = 0.0;
   out_3361570693863352746[199] = 0.0;
   out_3361570693863352746[200] = 0.0;
   out_3361570693863352746[201] = 0.0;
   out_3361570693863352746[202] = 0.0;
   out_3361570693863352746[203] = 0.0;
   out_3361570693863352746[204] = 0.0;
   out_3361570693863352746[205] = 0.0;
   out_3361570693863352746[206] = 0.0;
   out_3361570693863352746[207] = 0.0;
   out_3361570693863352746[208] = 0.0;
   out_3361570693863352746[209] = 1.0;
   out_3361570693863352746[210] = 0.0;
   out_3361570693863352746[211] = 0.0;
   out_3361570693863352746[212] = 0.0;
   out_3361570693863352746[213] = 0.0;
   out_3361570693863352746[214] = 0.0;
   out_3361570693863352746[215] = 0.0;
   out_3361570693863352746[216] = 0.0;
   out_3361570693863352746[217] = 0.0;
   out_3361570693863352746[218] = 0.0;
   out_3361570693863352746[219] = 0.0;
   out_3361570693863352746[220] = 0.0;
   out_3361570693863352746[221] = 0.0;
   out_3361570693863352746[222] = 0.0;
   out_3361570693863352746[223] = 0.0;
   out_3361570693863352746[224] = 0.0;
   out_3361570693863352746[225] = 0.0;
   out_3361570693863352746[226] = 0.0;
   out_3361570693863352746[227] = 0.0;
   out_3361570693863352746[228] = 1.0;
   out_3361570693863352746[229] = 0.0;
   out_3361570693863352746[230] = 0.0;
   out_3361570693863352746[231] = 0.0;
   out_3361570693863352746[232] = 0.0;
   out_3361570693863352746[233] = 0.0;
   out_3361570693863352746[234] = 0.0;
   out_3361570693863352746[235] = 0.0;
   out_3361570693863352746[236] = 0.0;
   out_3361570693863352746[237] = 0.0;
   out_3361570693863352746[238] = 0.0;
   out_3361570693863352746[239] = 0.0;
   out_3361570693863352746[240] = 0.0;
   out_3361570693863352746[241] = 0.0;
   out_3361570693863352746[242] = 0.0;
   out_3361570693863352746[243] = 0.0;
   out_3361570693863352746[244] = 0.0;
   out_3361570693863352746[245] = 0.0;
   out_3361570693863352746[246] = 0.0;
   out_3361570693863352746[247] = 1.0;
   out_3361570693863352746[248] = 0.0;
   out_3361570693863352746[249] = 0.0;
   out_3361570693863352746[250] = 0.0;
   out_3361570693863352746[251] = 0.0;
   out_3361570693863352746[252] = 0.0;
   out_3361570693863352746[253] = 0.0;
   out_3361570693863352746[254] = 0.0;
   out_3361570693863352746[255] = 0.0;
   out_3361570693863352746[256] = 0.0;
   out_3361570693863352746[257] = 0.0;
   out_3361570693863352746[258] = 0.0;
   out_3361570693863352746[259] = 0.0;
   out_3361570693863352746[260] = 0.0;
   out_3361570693863352746[261] = 0.0;
   out_3361570693863352746[262] = 0.0;
   out_3361570693863352746[263] = 0.0;
   out_3361570693863352746[264] = 0.0;
   out_3361570693863352746[265] = 0.0;
   out_3361570693863352746[266] = 1.0;
   out_3361570693863352746[267] = 0.0;
   out_3361570693863352746[268] = 0.0;
   out_3361570693863352746[269] = 0.0;
   out_3361570693863352746[270] = 0.0;
   out_3361570693863352746[271] = 0.0;
   out_3361570693863352746[272] = 0.0;
   out_3361570693863352746[273] = 0.0;
   out_3361570693863352746[274] = 0.0;
   out_3361570693863352746[275] = 0.0;
   out_3361570693863352746[276] = 0.0;
   out_3361570693863352746[277] = 0.0;
   out_3361570693863352746[278] = 0.0;
   out_3361570693863352746[279] = 0.0;
   out_3361570693863352746[280] = 0.0;
   out_3361570693863352746[281] = 0.0;
   out_3361570693863352746[282] = 0.0;
   out_3361570693863352746[283] = 0.0;
   out_3361570693863352746[284] = 0.0;
   out_3361570693863352746[285] = 1.0;
   out_3361570693863352746[286] = 0.0;
   out_3361570693863352746[287] = 0.0;
   out_3361570693863352746[288] = 0.0;
   out_3361570693863352746[289] = 0.0;
   out_3361570693863352746[290] = 0.0;
   out_3361570693863352746[291] = 0.0;
   out_3361570693863352746[292] = 0.0;
   out_3361570693863352746[293] = 0.0;
   out_3361570693863352746[294] = 0.0;
   out_3361570693863352746[295] = 0.0;
   out_3361570693863352746[296] = 0.0;
   out_3361570693863352746[297] = 0.0;
   out_3361570693863352746[298] = 0.0;
   out_3361570693863352746[299] = 0.0;
   out_3361570693863352746[300] = 0.0;
   out_3361570693863352746[301] = 0.0;
   out_3361570693863352746[302] = 0.0;
   out_3361570693863352746[303] = 0.0;
   out_3361570693863352746[304] = 1.0;
   out_3361570693863352746[305] = 0.0;
   out_3361570693863352746[306] = 0.0;
   out_3361570693863352746[307] = 0.0;
   out_3361570693863352746[308] = 0.0;
   out_3361570693863352746[309] = 0.0;
   out_3361570693863352746[310] = 0.0;
   out_3361570693863352746[311] = 0.0;
   out_3361570693863352746[312] = 0.0;
   out_3361570693863352746[313] = 0.0;
   out_3361570693863352746[314] = 0.0;
   out_3361570693863352746[315] = 0.0;
   out_3361570693863352746[316] = 0.0;
   out_3361570693863352746[317] = 0.0;
   out_3361570693863352746[318] = 0.0;
   out_3361570693863352746[319] = 0.0;
   out_3361570693863352746[320] = 0.0;
   out_3361570693863352746[321] = 0.0;
   out_3361570693863352746[322] = 0.0;
   out_3361570693863352746[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_6386770787553891211) {
   out_6386770787553891211[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_6386770787553891211[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_6386770787553891211[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_6386770787553891211[3] = dt*state[12] + state[3];
   out_6386770787553891211[4] = dt*state[13] + state[4];
   out_6386770787553891211[5] = dt*state[14] + state[5];
   out_6386770787553891211[6] = state[6];
   out_6386770787553891211[7] = state[7];
   out_6386770787553891211[8] = state[8];
   out_6386770787553891211[9] = state[9];
   out_6386770787553891211[10] = state[10];
   out_6386770787553891211[11] = state[11];
   out_6386770787553891211[12] = state[12];
   out_6386770787553891211[13] = state[13];
   out_6386770787553891211[14] = state[14];
   out_6386770787553891211[15] = state[15];
   out_6386770787553891211[16] = state[16];
   out_6386770787553891211[17] = state[17];
}
void F_fun(double *state, double dt, double *out_476766260057365837) {
   out_476766260057365837[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_476766260057365837[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_476766260057365837[2] = 0;
   out_476766260057365837[3] = 0;
   out_476766260057365837[4] = 0;
   out_476766260057365837[5] = 0;
   out_476766260057365837[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_476766260057365837[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_476766260057365837[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_476766260057365837[9] = 0;
   out_476766260057365837[10] = 0;
   out_476766260057365837[11] = 0;
   out_476766260057365837[12] = 0;
   out_476766260057365837[13] = 0;
   out_476766260057365837[14] = 0;
   out_476766260057365837[15] = 0;
   out_476766260057365837[16] = 0;
   out_476766260057365837[17] = 0;
   out_476766260057365837[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_476766260057365837[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_476766260057365837[20] = 0;
   out_476766260057365837[21] = 0;
   out_476766260057365837[22] = 0;
   out_476766260057365837[23] = 0;
   out_476766260057365837[24] = 0;
   out_476766260057365837[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_476766260057365837[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_476766260057365837[27] = 0;
   out_476766260057365837[28] = 0;
   out_476766260057365837[29] = 0;
   out_476766260057365837[30] = 0;
   out_476766260057365837[31] = 0;
   out_476766260057365837[32] = 0;
   out_476766260057365837[33] = 0;
   out_476766260057365837[34] = 0;
   out_476766260057365837[35] = 0;
   out_476766260057365837[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_476766260057365837[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_476766260057365837[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_476766260057365837[39] = 0;
   out_476766260057365837[40] = 0;
   out_476766260057365837[41] = 0;
   out_476766260057365837[42] = 0;
   out_476766260057365837[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_476766260057365837[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_476766260057365837[45] = 0;
   out_476766260057365837[46] = 0;
   out_476766260057365837[47] = 0;
   out_476766260057365837[48] = 0;
   out_476766260057365837[49] = 0;
   out_476766260057365837[50] = 0;
   out_476766260057365837[51] = 0;
   out_476766260057365837[52] = 0;
   out_476766260057365837[53] = 0;
   out_476766260057365837[54] = 0;
   out_476766260057365837[55] = 0;
   out_476766260057365837[56] = 0;
   out_476766260057365837[57] = 1;
   out_476766260057365837[58] = 0;
   out_476766260057365837[59] = 0;
   out_476766260057365837[60] = 0;
   out_476766260057365837[61] = 0;
   out_476766260057365837[62] = 0;
   out_476766260057365837[63] = 0;
   out_476766260057365837[64] = 0;
   out_476766260057365837[65] = 0;
   out_476766260057365837[66] = dt;
   out_476766260057365837[67] = 0;
   out_476766260057365837[68] = 0;
   out_476766260057365837[69] = 0;
   out_476766260057365837[70] = 0;
   out_476766260057365837[71] = 0;
   out_476766260057365837[72] = 0;
   out_476766260057365837[73] = 0;
   out_476766260057365837[74] = 0;
   out_476766260057365837[75] = 0;
   out_476766260057365837[76] = 1;
   out_476766260057365837[77] = 0;
   out_476766260057365837[78] = 0;
   out_476766260057365837[79] = 0;
   out_476766260057365837[80] = 0;
   out_476766260057365837[81] = 0;
   out_476766260057365837[82] = 0;
   out_476766260057365837[83] = 0;
   out_476766260057365837[84] = 0;
   out_476766260057365837[85] = dt;
   out_476766260057365837[86] = 0;
   out_476766260057365837[87] = 0;
   out_476766260057365837[88] = 0;
   out_476766260057365837[89] = 0;
   out_476766260057365837[90] = 0;
   out_476766260057365837[91] = 0;
   out_476766260057365837[92] = 0;
   out_476766260057365837[93] = 0;
   out_476766260057365837[94] = 0;
   out_476766260057365837[95] = 1;
   out_476766260057365837[96] = 0;
   out_476766260057365837[97] = 0;
   out_476766260057365837[98] = 0;
   out_476766260057365837[99] = 0;
   out_476766260057365837[100] = 0;
   out_476766260057365837[101] = 0;
   out_476766260057365837[102] = 0;
   out_476766260057365837[103] = 0;
   out_476766260057365837[104] = dt;
   out_476766260057365837[105] = 0;
   out_476766260057365837[106] = 0;
   out_476766260057365837[107] = 0;
   out_476766260057365837[108] = 0;
   out_476766260057365837[109] = 0;
   out_476766260057365837[110] = 0;
   out_476766260057365837[111] = 0;
   out_476766260057365837[112] = 0;
   out_476766260057365837[113] = 0;
   out_476766260057365837[114] = 1;
   out_476766260057365837[115] = 0;
   out_476766260057365837[116] = 0;
   out_476766260057365837[117] = 0;
   out_476766260057365837[118] = 0;
   out_476766260057365837[119] = 0;
   out_476766260057365837[120] = 0;
   out_476766260057365837[121] = 0;
   out_476766260057365837[122] = 0;
   out_476766260057365837[123] = 0;
   out_476766260057365837[124] = 0;
   out_476766260057365837[125] = 0;
   out_476766260057365837[126] = 0;
   out_476766260057365837[127] = 0;
   out_476766260057365837[128] = 0;
   out_476766260057365837[129] = 0;
   out_476766260057365837[130] = 0;
   out_476766260057365837[131] = 0;
   out_476766260057365837[132] = 0;
   out_476766260057365837[133] = 1;
   out_476766260057365837[134] = 0;
   out_476766260057365837[135] = 0;
   out_476766260057365837[136] = 0;
   out_476766260057365837[137] = 0;
   out_476766260057365837[138] = 0;
   out_476766260057365837[139] = 0;
   out_476766260057365837[140] = 0;
   out_476766260057365837[141] = 0;
   out_476766260057365837[142] = 0;
   out_476766260057365837[143] = 0;
   out_476766260057365837[144] = 0;
   out_476766260057365837[145] = 0;
   out_476766260057365837[146] = 0;
   out_476766260057365837[147] = 0;
   out_476766260057365837[148] = 0;
   out_476766260057365837[149] = 0;
   out_476766260057365837[150] = 0;
   out_476766260057365837[151] = 0;
   out_476766260057365837[152] = 1;
   out_476766260057365837[153] = 0;
   out_476766260057365837[154] = 0;
   out_476766260057365837[155] = 0;
   out_476766260057365837[156] = 0;
   out_476766260057365837[157] = 0;
   out_476766260057365837[158] = 0;
   out_476766260057365837[159] = 0;
   out_476766260057365837[160] = 0;
   out_476766260057365837[161] = 0;
   out_476766260057365837[162] = 0;
   out_476766260057365837[163] = 0;
   out_476766260057365837[164] = 0;
   out_476766260057365837[165] = 0;
   out_476766260057365837[166] = 0;
   out_476766260057365837[167] = 0;
   out_476766260057365837[168] = 0;
   out_476766260057365837[169] = 0;
   out_476766260057365837[170] = 0;
   out_476766260057365837[171] = 1;
   out_476766260057365837[172] = 0;
   out_476766260057365837[173] = 0;
   out_476766260057365837[174] = 0;
   out_476766260057365837[175] = 0;
   out_476766260057365837[176] = 0;
   out_476766260057365837[177] = 0;
   out_476766260057365837[178] = 0;
   out_476766260057365837[179] = 0;
   out_476766260057365837[180] = 0;
   out_476766260057365837[181] = 0;
   out_476766260057365837[182] = 0;
   out_476766260057365837[183] = 0;
   out_476766260057365837[184] = 0;
   out_476766260057365837[185] = 0;
   out_476766260057365837[186] = 0;
   out_476766260057365837[187] = 0;
   out_476766260057365837[188] = 0;
   out_476766260057365837[189] = 0;
   out_476766260057365837[190] = 1;
   out_476766260057365837[191] = 0;
   out_476766260057365837[192] = 0;
   out_476766260057365837[193] = 0;
   out_476766260057365837[194] = 0;
   out_476766260057365837[195] = 0;
   out_476766260057365837[196] = 0;
   out_476766260057365837[197] = 0;
   out_476766260057365837[198] = 0;
   out_476766260057365837[199] = 0;
   out_476766260057365837[200] = 0;
   out_476766260057365837[201] = 0;
   out_476766260057365837[202] = 0;
   out_476766260057365837[203] = 0;
   out_476766260057365837[204] = 0;
   out_476766260057365837[205] = 0;
   out_476766260057365837[206] = 0;
   out_476766260057365837[207] = 0;
   out_476766260057365837[208] = 0;
   out_476766260057365837[209] = 1;
   out_476766260057365837[210] = 0;
   out_476766260057365837[211] = 0;
   out_476766260057365837[212] = 0;
   out_476766260057365837[213] = 0;
   out_476766260057365837[214] = 0;
   out_476766260057365837[215] = 0;
   out_476766260057365837[216] = 0;
   out_476766260057365837[217] = 0;
   out_476766260057365837[218] = 0;
   out_476766260057365837[219] = 0;
   out_476766260057365837[220] = 0;
   out_476766260057365837[221] = 0;
   out_476766260057365837[222] = 0;
   out_476766260057365837[223] = 0;
   out_476766260057365837[224] = 0;
   out_476766260057365837[225] = 0;
   out_476766260057365837[226] = 0;
   out_476766260057365837[227] = 0;
   out_476766260057365837[228] = 1;
   out_476766260057365837[229] = 0;
   out_476766260057365837[230] = 0;
   out_476766260057365837[231] = 0;
   out_476766260057365837[232] = 0;
   out_476766260057365837[233] = 0;
   out_476766260057365837[234] = 0;
   out_476766260057365837[235] = 0;
   out_476766260057365837[236] = 0;
   out_476766260057365837[237] = 0;
   out_476766260057365837[238] = 0;
   out_476766260057365837[239] = 0;
   out_476766260057365837[240] = 0;
   out_476766260057365837[241] = 0;
   out_476766260057365837[242] = 0;
   out_476766260057365837[243] = 0;
   out_476766260057365837[244] = 0;
   out_476766260057365837[245] = 0;
   out_476766260057365837[246] = 0;
   out_476766260057365837[247] = 1;
   out_476766260057365837[248] = 0;
   out_476766260057365837[249] = 0;
   out_476766260057365837[250] = 0;
   out_476766260057365837[251] = 0;
   out_476766260057365837[252] = 0;
   out_476766260057365837[253] = 0;
   out_476766260057365837[254] = 0;
   out_476766260057365837[255] = 0;
   out_476766260057365837[256] = 0;
   out_476766260057365837[257] = 0;
   out_476766260057365837[258] = 0;
   out_476766260057365837[259] = 0;
   out_476766260057365837[260] = 0;
   out_476766260057365837[261] = 0;
   out_476766260057365837[262] = 0;
   out_476766260057365837[263] = 0;
   out_476766260057365837[264] = 0;
   out_476766260057365837[265] = 0;
   out_476766260057365837[266] = 1;
   out_476766260057365837[267] = 0;
   out_476766260057365837[268] = 0;
   out_476766260057365837[269] = 0;
   out_476766260057365837[270] = 0;
   out_476766260057365837[271] = 0;
   out_476766260057365837[272] = 0;
   out_476766260057365837[273] = 0;
   out_476766260057365837[274] = 0;
   out_476766260057365837[275] = 0;
   out_476766260057365837[276] = 0;
   out_476766260057365837[277] = 0;
   out_476766260057365837[278] = 0;
   out_476766260057365837[279] = 0;
   out_476766260057365837[280] = 0;
   out_476766260057365837[281] = 0;
   out_476766260057365837[282] = 0;
   out_476766260057365837[283] = 0;
   out_476766260057365837[284] = 0;
   out_476766260057365837[285] = 1;
   out_476766260057365837[286] = 0;
   out_476766260057365837[287] = 0;
   out_476766260057365837[288] = 0;
   out_476766260057365837[289] = 0;
   out_476766260057365837[290] = 0;
   out_476766260057365837[291] = 0;
   out_476766260057365837[292] = 0;
   out_476766260057365837[293] = 0;
   out_476766260057365837[294] = 0;
   out_476766260057365837[295] = 0;
   out_476766260057365837[296] = 0;
   out_476766260057365837[297] = 0;
   out_476766260057365837[298] = 0;
   out_476766260057365837[299] = 0;
   out_476766260057365837[300] = 0;
   out_476766260057365837[301] = 0;
   out_476766260057365837[302] = 0;
   out_476766260057365837[303] = 0;
   out_476766260057365837[304] = 1;
   out_476766260057365837[305] = 0;
   out_476766260057365837[306] = 0;
   out_476766260057365837[307] = 0;
   out_476766260057365837[308] = 0;
   out_476766260057365837[309] = 0;
   out_476766260057365837[310] = 0;
   out_476766260057365837[311] = 0;
   out_476766260057365837[312] = 0;
   out_476766260057365837[313] = 0;
   out_476766260057365837[314] = 0;
   out_476766260057365837[315] = 0;
   out_476766260057365837[316] = 0;
   out_476766260057365837[317] = 0;
   out_476766260057365837[318] = 0;
   out_476766260057365837[319] = 0;
   out_476766260057365837[320] = 0;
   out_476766260057365837[321] = 0;
   out_476766260057365837[322] = 0;
   out_476766260057365837[323] = 1;
}
void h_4(double *state, double *unused, double *out_5361742893531860988) {
   out_5361742893531860988[0] = state[6] + state[9];
   out_5361742893531860988[1] = state[7] + state[10];
   out_5361742893531860988[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_831405397157000068) {
   out_831405397157000068[0] = 0;
   out_831405397157000068[1] = 0;
   out_831405397157000068[2] = 0;
   out_831405397157000068[3] = 0;
   out_831405397157000068[4] = 0;
   out_831405397157000068[5] = 0;
   out_831405397157000068[6] = 1;
   out_831405397157000068[7] = 0;
   out_831405397157000068[8] = 0;
   out_831405397157000068[9] = 1;
   out_831405397157000068[10] = 0;
   out_831405397157000068[11] = 0;
   out_831405397157000068[12] = 0;
   out_831405397157000068[13] = 0;
   out_831405397157000068[14] = 0;
   out_831405397157000068[15] = 0;
   out_831405397157000068[16] = 0;
   out_831405397157000068[17] = 0;
   out_831405397157000068[18] = 0;
   out_831405397157000068[19] = 0;
   out_831405397157000068[20] = 0;
   out_831405397157000068[21] = 0;
   out_831405397157000068[22] = 0;
   out_831405397157000068[23] = 0;
   out_831405397157000068[24] = 0;
   out_831405397157000068[25] = 1;
   out_831405397157000068[26] = 0;
   out_831405397157000068[27] = 0;
   out_831405397157000068[28] = 1;
   out_831405397157000068[29] = 0;
   out_831405397157000068[30] = 0;
   out_831405397157000068[31] = 0;
   out_831405397157000068[32] = 0;
   out_831405397157000068[33] = 0;
   out_831405397157000068[34] = 0;
   out_831405397157000068[35] = 0;
   out_831405397157000068[36] = 0;
   out_831405397157000068[37] = 0;
   out_831405397157000068[38] = 0;
   out_831405397157000068[39] = 0;
   out_831405397157000068[40] = 0;
   out_831405397157000068[41] = 0;
   out_831405397157000068[42] = 0;
   out_831405397157000068[43] = 0;
   out_831405397157000068[44] = 1;
   out_831405397157000068[45] = 0;
   out_831405397157000068[46] = 0;
   out_831405397157000068[47] = 1;
   out_831405397157000068[48] = 0;
   out_831405397157000068[49] = 0;
   out_831405397157000068[50] = 0;
   out_831405397157000068[51] = 0;
   out_831405397157000068[52] = 0;
   out_831405397157000068[53] = 0;
}
void h_10(double *state, double *unused, double *out_9170580747203543004) {
   out_9170580747203543004[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_9170580747203543004[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_9170580747203543004[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_7393744558597873191) {
   out_7393744558597873191[0] = 0;
   out_7393744558597873191[1] = 9.8100000000000005*cos(state[1]);
   out_7393744558597873191[2] = 0;
   out_7393744558597873191[3] = 0;
   out_7393744558597873191[4] = -state[8];
   out_7393744558597873191[5] = state[7];
   out_7393744558597873191[6] = 0;
   out_7393744558597873191[7] = state[5];
   out_7393744558597873191[8] = -state[4];
   out_7393744558597873191[9] = 0;
   out_7393744558597873191[10] = 0;
   out_7393744558597873191[11] = 0;
   out_7393744558597873191[12] = 1;
   out_7393744558597873191[13] = 0;
   out_7393744558597873191[14] = 0;
   out_7393744558597873191[15] = 1;
   out_7393744558597873191[16] = 0;
   out_7393744558597873191[17] = 0;
   out_7393744558597873191[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_7393744558597873191[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_7393744558597873191[20] = 0;
   out_7393744558597873191[21] = state[8];
   out_7393744558597873191[22] = 0;
   out_7393744558597873191[23] = -state[6];
   out_7393744558597873191[24] = -state[5];
   out_7393744558597873191[25] = 0;
   out_7393744558597873191[26] = state[3];
   out_7393744558597873191[27] = 0;
   out_7393744558597873191[28] = 0;
   out_7393744558597873191[29] = 0;
   out_7393744558597873191[30] = 0;
   out_7393744558597873191[31] = 1;
   out_7393744558597873191[32] = 0;
   out_7393744558597873191[33] = 0;
   out_7393744558597873191[34] = 1;
   out_7393744558597873191[35] = 0;
   out_7393744558597873191[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_7393744558597873191[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_7393744558597873191[38] = 0;
   out_7393744558597873191[39] = -state[7];
   out_7393744558597873191[40] = state[6];
   out_7393744558597873191[41] = 0;
   out_7393744558597873191[42] = state[4];
   out_7393744558597873191[43] = -state[3];
   out_7393744558597873191[44] = 0;
   out_7393744558597873191[45] = 0;
   out_7393744558597873191[46] = 0;
   out_7393744558597873191[47] = 0;
   out_7393744558597873191[48] = 0;
   out_7393744558597873191[49] = 0;
   out_7393744558597873191[50] = 1;
   out_7393744558597873191[51] = 0;
   out_7393744558597873191[52] = 0;
   out_7393744558597873191[53] = 1;
}
void h_13(double *state, double *unused, double *out_2910083262545817831) {
   out_2910083262545817831[0] = state[3];
   out_2910083262545817831[1] = state[4];
   out_2910083262545817831[2] = state[5];
}
void H_13(double *state, double *unused, double *out_2380868428175332733) {
   out_2380868428175332733[0] = 0;
   out_2380868428175332733[1] = 0;
   out_2380868428175332733[2] = 0;
   out_2380868428175332733[3] = 1;
   out_2380868428175332733[4] = 0;
   out_2380868428175332733[5] = 0;
   out_2380868428175332733[6] = 0;
   out_2380868428175332733[7] = 0;
   out_2380868428175332733[8] = 0;
   out_2380868428175332733[9] = 0;
   out_2380868428175332733[10] = 0;
   out_2380868428175332733[11] = 0;
   out_2380868428175332733[12] = 0;
   out_2380868428175332733[13] = 0;
   out_2380868428175332733[14] = 0;
   out_2380868428175332733[15] = 0;
   out_2380868428175332733[16] = 0;
   out_2380868428175332733[17] = 0;
   out_2380868428175332733[18] = 0;
   out_2380868428175332733[19] = 0;
   out_2380868428175332733[20] = 0;
   out_2380868428175332733[21] = 0;
   out_2380868428175332733[22] = 1;
   out_2380868428175332733[23] = 0;
   out_2380868428175332733[24] = 0;
   out_2380868428175332733[25] = 0;
   out_2380868428175332733[26] = 0;
   out_2380868428175332733[27] = 0;
   out_2380868428175332733[28] = 0;
   out_2380868428175332733[29] = 0;
   out_2380868428175332733[30] = 0;
   out_2380868428175332733[31] = 0;
   out_2380868428175332733[32] = 0;
   out_2380868428175332733[33] = 0;
   out_2380868428175332733[34] = 0;
   out_2380868428175332733[35] = 0;
   out_2380868428175332733[36] = 0;
   out_2380868428175332733[37] = 0;
   out_2380868428175332733[38] = 0;
   out_2380868428175332733[39] = 0;
   out_2380868428175332733[40] = 0;
   out_2380868428175332733[41] = 1;
   out_2380868428175332733[42] = 0;
   out_2380868428175332733[43] = 0;
   out_2380868428175332733[44] = 0;
   out_2380868428175332733[45] = 0;
   out_2380868428175332733[46] = 0;
   out_2380868428175332733[47] = 0;
   out_2380868428175332733[48] = 0;
   out_2380868428175332733[49] = 0;
   out_2380868428175332733[50] = 0;
   out_2380868428175332733[51] = 0;
   out_2380868428175332733[52] = 0;
   out_2380868428175332733[53] = 0;
}
void h_14(double *state, double *unused, double *out_3581530193976320576) {
   out_3581530193976320576[0] = state[6];
   out_3581530193976320576[1] = state[7];
   out_3581530193976320576[2] = state[8];
}
void H_14(double *state, double *unused, double *out_3914193829452372364) {
   out_3914193829452372364[0] = 0;
   out_3914193829452372364[1] = 0;
   out_3914193829452372364[2] = 0;
   out_3914193829452372364[3] = 0;
   out_3914193829452372364[4] = 0;
   out_3914193829452372364[5] = 0;
   out_3914193829452372364[6] = 1;
   out_3914193829452372364[7] = 0;
   out_3914193829452372364[8] = 0;
   out_3914193829452372364[9] = 0;
   out_3914193829452372364[10] = 0;
   out_3914193829452372364[11] = 0;
   out_3914193829452372364[12] = 0;
   out_3914193829452372364[13] = 0;
   out_3914193829452372364[14] = 0;
   out_3914193829452372364[15] = 0;
   out_3914193829452372364[16] = 0;
   out_3914193829452372364[17] = 0;
   out_3914193829452372364[18] = 0;
   out_3914193829452372364[19] = 0;
   out_3914193829452372364[20] = 0;
   out_3914193829452372364[21] = 0;
   out_3914193829452372364[22] = 0;
   out_3914193829452372364[23] = 0;
   out_3914193829452372364[24] = 0;
   out_3914193829452372364[25] = 1;
   out_3914193829452372364[26] = 0;
   out_3914193829452372364[27] = 0;
   out_3914193829452372364[28] = 0;
   out_3914193829452372364[29] = 0;
   out_3914193829452372364[30] = 0;
   out_3914193829452372364[31] = 0;
   out_3914193829452372364[32] = 0;
   out_3914193829452372364[33] = 0;
   out_3914193829452372364[34] = 0;
   out_3914193829452372364[35] = 0;
   out_3914193829452372364[36] = 0;
   out_3914193829452372364[37] = 0;
   out_3914193829452372364[38] = 0;
   out_3914193829452372364[39] = 0;
   out_3914193829452372364[40] = 0;
   out_3914193829452372364[41] = 0;
   out_3914193829452372364[42] = 0;
   out_3914193829452372364[43] = 0;
   out_3914193829452372364[44] = 1;
   out_3914193829452372364[45] = 0;
   out_3914193829452372364[46] = 0;
   out_3914193829452372364[47] = 0;
   out_3914193829452372364[48] = 0;
   out_3914193829452372364[49] = 0;
   out_3914193829452372364[50] = 0;
   out_3914193829452372364[51] = 0;
   out_3914193829452372364[52] = 0;
   out_3914193829452372364[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_8052617336508823134) {
  err_fun(nom_x, delta_x, out_8052617336508823134);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_9128085987645078449) {
  inv_err_fun(nom_x, true_x, out_9128085987645078449);
}
void pose_H_mod_fun(double *state, double *out_3361570693863352746) {
  H_mod_fun(state, out_3361570693863352746);
}
void pose_f_fun(double *state, double dt, double *out_6386770787553891211) {
  f_fun(state,  dt, out_6386770787553891211);
}
void pose_F_fun(double *state, double dt, double *out_476766260057365837) {
  F_fun(state,  dt, out_476766260057365837);
}
void pose_h_4(double *state, double *unused, double *out_5361742893531860988) {
  h_4(state, unused, out_5361742893531860988);
}
void pose_H_4(double *state, double *unused, double *out_831405397157000068) {
  H_4(state, unused, out_831405397157000068);
}
void pose_h_10(double *state, double *unused, double *out_9170580747203543004) {
  h_10(state, unused, out_9170580747203543004);
}
void pose_H_10(double *state, double *unused, double *out_7393744558597873191) {
  H_10(state, unused, out_7393744558597873191);
}
void pose_h_13(double *state, double *unused, double *out_2910083262545817831) {
  h_13(state, unused, out_2910083262545817831);
}
void pose_H_13(double *state, double *unused, double *out_2380868428175332733) {
  H_13(state, unused, out_2380868428175332733);
}
void pose_h_14(double *state, double *unused, double *out_3581530193976320576) {
  h_14(state, unused, out_3581530193976320576);
}
void pose_H_14(double *state, double *unused, double *out_3914193829452372364) {
  H_14(state, unused, out_3914193829452372364);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)

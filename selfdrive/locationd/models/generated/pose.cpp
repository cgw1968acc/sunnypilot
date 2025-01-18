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
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1425647101528206710) {
   out_1425647101528206710[0] = delta_x[0] + nom_x[0];
   out_1425647101528206710[1] = delta_x[1] + nom_x[1];
   out_1425647101528206710[2] = delta_x[2] + nom_x[2];
   out_1425647101528206710[3] = delta_x[3] + nom_x[3];
   out_1425647101528206710[4] = delta_x[4] + nom_x[4];
   out_1425647101528206710[5] = delta_x[5] + nom_x[5];
   out_1425647101528206710[6] = delta_x[6] + nom_x[6];
   out_1425647101528206710[7] = delta_x[7] + nom_x[7];
   out_1425647101528206710[8] = delta_x[8] + nom_x[8];
   out_1425647101528206710[9] = delta_x[9] + nom_x[9];
   out_1425647101528206710[10] = delta_x[10] + nom_x[10];
   out_1425647101528206710[11] = delta_x[11] + nom_x[11];
   out_1425647101528206710[12] = delta_x[12] + nom_x[12];
   out_1425647101528206710[13] = delta_x[13] + nom_x[13];
   out_1425647101528206710[14] = delta_x[14] + nom_x[14];
   out_1425647101528206710[15] = delta_x[15] + nom_x[15];
   out_1425647101528206710[16] = delta_x[16] + nom_x[16];
   out_1425647101528206710[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1586401905831597934) {
   out_1586401905831597934[0] = -nom_x[0] + true_x[0];
   out_1586401905831597934[1] = -nom_x[1] + true_x[1];
   out_1586401905831597934[2] = -nom_x[2] + true_x[2];
   out_1586401905831597934[3] = -nom_x[3] + true_x[3];
   out_1586401905831597934[4] = -nom_x[4] + true_x[4];
   out_1586401905831597934[5] = -nom_x[5] + true_x[5];
   out_1586401905831597934[6] = -nom_x[6] + true_x[6];
   out_1586401905831597934[7] = -nom_x[7] + true_x[7];
   out_1586401905831597934[8] = -nom_x[8] + true_x[8];
   out_1586401905831597934[9] = -nom_x[9] + true_x[9];
   out_1586401905831597934[10] = -nom_x[10] + true_x[10];
   out_1586401905831597934[11] = -nom_x[11] + true_x[11];
   out_1586401905831597934[12] = -nom_x[12] + true_x[12];
   out_1586401905831597934[13] = -nom_x[13] + true_x[13];
   out_1586401905831597934[14] = -nom_x[14] + true_x[14];
   out_1586401905831597934[15] = -nom_x[15] + true_x[15];
   out_1586401905831597934[16] = -nom_x[16] + true_x[16];
   out_1586401905831597934[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_3061887074369766267) {
   out_3061887074369766267[0] = 1.0;
   out_3061887074369766267[1] = 0.0;
   out_3061887074369766267[2] = 0.0;
   out_3061887074369766267[3] = 0.0;
   out_3061887074369766267[4] = 0.0;
   out_3061887074369766267[5] = 0.0;
   out_3061887074369766267[6] = 0.0;
   out_3061887074369766267[7] = 0.0;
   out_3061887074369766267[8] = 0.0;
   out_3061887074369766267[9] = 0.0;
   out_3061887074369766267[10] = 0.0;
   out_3061887074369766267[11] = 0.0;
   out_3061887074369766267[12] = 0.0;
   out_3061887074369766267[13] = 0.0;
   out_3061887074369766267[14] = 0.0;
   out_3061887074369766267[15] = 0.0;
   out_3061887074369766267[16] = 0.0;
   out_3061887074369766267[17] = 0.0;
   out_3061887074369766267[18] = 0.0;
   out_3061887074369766267[19] = 1.0;
   out_3061887074369766267[20] = 0.0;
   out_3061887074369766267[21] = 0.0;
   out_3061887074369766267[22] = 0.0;
   out_3061887074369766267[23] = 0.0;
   out_3061887074369766267[24] = 0.0;
   out_3061887074369766267[25] = 0.0;
   out_3061887074369766267[26] = 0.0;
   out_3061887074369766267[27] = 0.0;
   out_3061887074369766267[28] = 0.0;
   out_3061887074369766267[29] = 0.0;
   out_3061887074369766267[30] = 0.0;
   out_3061887074369766267[31] = 0.0;
   out_3061887074369766267[32] = 0.0;
   out_3061887074369766267[33] = 0.0;
   out_3061887074369766267[34] = 0.0;
   out_3061887074369766267[35] = 0.0;
   out_3061887074369766267[36] = 0.0;
   out_3061887074369766267[37] = 0.0;
   out_3061887074369766267[38] = 1.0;
   out_3061887074369766267[39] = 0.0;
   out_3061887074369766267[40] = 0.0;
   out_3061887074369766267[41] = 0.0;
   out_3061887074369766267[42] = 0.0;
   out_3061887074369766267[43] = 0.0;
   out_3061887074369766267[44] = 0.0;
   out_3061887074369766267[45] = 0.0;
   out_3061887074369766267[46] = 0.0;
   out_3061887074369766267[47] = 0.0;
   out_3061887074369766267[48] = 0.0;
   out_3061887074369766267[49] = 0.0;
   out_3061887074369766267[50] = 0.0;
   out_3061887074369766267[51] = 0.0;
   out_3061887074369766267[52] = 0.0;
   out_3061887074369766267[53] = 0.0;
   out_3061887074369766267[54] = 0.0;
   out_3061887074369766267[55] = 0.0;
   out_3061887074369766267[56] = 0.0;
   out_3061887074369766267[57] = 1.0;
   out_3061887074369766267[58] = 0.0;
   out_3061887074369766267[59] = 0.0;
   out_3061887074369766267[60] = 0.0;
   out_3061887074369766267[61] = 0.0;
   out_3061887074369766267[62] = 0.0;
   out_3061887074369766267[63] = 0.0;
   out_3061887074369766267[64] = 0.0;
   out_3061887074369766267[65] = 0.0;
   out_3061887074369766267[66] = 0.0;
   out_3061887074369766267[67] = 0.0;
   out_3061887074369766267[68] = 0.0;
   out_3061887074369766267[69] = 0.0;
   out_3061887074369766267[70] = 0.0;
   out_3061887074369766267[71] = 0.0;
   out_3061887074369766267[72] = 0.0;
   out_3061887074369766267[73] = 0.0;
   out_3061887074369766267[74] = 0.0;
   out_3061887074369766267[75] = 0.0;
   out_3061887074369766267[76] = 1.0;
   out_3061887074369766267[77] = 0.0;
   out_3061887074369766267[78] = 0.0;
   out_3061887074369766267[79] = 0.0;
   out_3061887074369766267[80] = 0.0;
   out_3061887074369766267[81] = 0.0;
   out_3061887074369766267[82] = 0.0;
   out_3061887074369766267[83] = 0.0;
   out_3061887074369766267[84] = 0.0;
   out_3061887074369766267[85] = 0.0;
   out_3061887074369766267[86] = 0.0;
   out_3061887074369766267[87] = 0.0;
   out_3061887074369766267[88] = 0.0;
   out_3061887074369766267[89] = 0.0;
   out_3061887074369766267[90] = 0.0;
   out_3061887074369766267[91] = 0.0;
   out_3061887074369766267[92] = 0.0;
   out_3061887074369766267[93] = 0.0;
   out_3061887074369766267[94] = 0.0;
   out_3061887074369766267[95] = 1.0;
   out_3061887074369766267[96] = 0.0;
   out_3061887074369766267[97] = 0.0;
   out_3061887074369766267[98] = 0.0;
   out_3061887074369766267[99] = 0.0;
   out_3061887074369766267[100] = 0.0;
   out_3061887074369766267[101] = 0.0;
   out_3061887074369766267[102] = 0.0;
   out_3061887074369766267[103] = 0.0;
   out_3061887074369766267[104] = 0.0;
   out_3061887074369766267[105] = 0.0;
   out_3061887074369766267[106] = 0.0;
   out_3061887074369766267[107] = 0.0;
   out_3061887074369766267[108] = 0.0;
   out_3061887074369766267[109] = 0.0;
   out_3061887074369766267[110] = 0.0;
   out_3061887074369766267[111] = 0.0;
   out_3061887074369766267[112] = 0.0;
   out_3061887074369766267[113] = 0.0;
   out_3061887074369766267[114] = 1.0;
   out_3061887074369766267[115] = 0.0;
   out_3061887074369766267[116] = 0.0;
   out_3061887074369766267[117] = 0.0;
   out_3061887074369766267[118] = 0.0;
   out_3061887074369766267[119] = 0.0;
   out_3061887074369766267[120] = 0.0;
   out_3061887074369766267[121] = 0.0;
   out_3061887074369766267[122] = 0.0;
   out_3061887074369766267[123] = 0.0;
   out_3061887074369766267[124] = 0.0;
   out_3061887074369766267[125] = 0.0;
   out_3061887074369766267[126] = 0.0;
   out_3061887074369766267[127] = 0.0;
   out_3061887074369766267[128] = 0.0;
   out_3061887074369766267[129] = 0.0;
   out_3061887074369766267[130] = 0.0;
   out_3061887074369766267[131] = 0.0;
   out_3061887074369766267[132] = 0.0;
   out_3061887074369766267[133] = 1.0;
   out_3061887074369766267[134] = 0.0;
   out_3061887074369766267[135] = 0.0;
   out_3061887074369766267[136] = 0.0;
   out_3061887074369766267[137] = 0.0;
   out_3061887074369766267[138] = 0.0;
   out_3061887074369766267[139] = 0.0;
   out_3061887074369766267[140] = 0.0;
   out_3061887074369766267[141] = 0.0;
   out_3061887074369766267[142] = 0.0;
   out_3061887074369766267[143] = 0.0;
   out_3061887074369766267[144] = 0.0;
   out_3061887074369766267[145] = 0.0;
   out_3061887074369766267[146] = 0.0;
   out_3061887074369766267[147] = 0.0;
   out_3061887074369766267[148] = 0.0;
   out_3061887074369766267[149] = 0.0;
   out_3061887074369766267[150] = 0.0;
   out_3061887074369766267[151] = 0.0;
   out_3061887074369766267[152] = 1.0;
   out_3061887074369766267[153] = 0.0;
   out_3061887074369766267[154] = 0.0;
   out_3061887074369766267[155] = 0.0;
   out_3061887074369766267[156] = 0.0;
   out_3061887074369766267[157] = 0.0;
   out_3061887074369766267[158] = 0.0;
   out_3061887074369766267[159] = 0.0;
   out_3061887074369766267[160] = 0.0;
   out_3061887074369766267[161] = 0.0;
   out_3061887074369766267[162] = 0.0;
   out_3061887074369766267[163] = 0.0;
   out_3061887074369766267[164] = 0.0;
   out_3061887074369766267[165] = 0.0;
   out_3061887074369766267[166] = 0.0;
   out_3061887074369766267[167] = 0.0;
   out_3061887074369766267[168] = 0.0;
   out_3061887074369766267[169] = 0.0;
   out_3061887074369766267[170] = 0.0;
   out_3061887074369766267[171] = 1.0;
   out_3061887074369766267[172] = 0.0;
   out_3061887074369766267[173] = 0.0;
   out_3061887074369766267[174] = 0.0;
   out_3061887074369766267[175] = 0.0;
   out_3061887074369766267[176] = 0.0;
   out_3061887074369766267[177] = 0.0;
   out_3061887074369766267[178] = 0.0;
   out_3061887074369766267[179] = 0.0;
   out_3061887074369766267[180] = 0.0;
   out_3061887074369766267[181] = 0.0;
   out_3061887074369766267[182] = 0.0;
   out_3061887074369766267[183] = 0.0;
   out_3061887074369766267[184] = 0.0;
   out_3061887074369766267[185] = 0.0;
   out_3061887074369766267[186] = 0.0;
   out_3061887074369766267[187] = 0.0;
   out_3061887074369766267[188] = 0.0;
   out_3061887074369766267[189] = 0.0;
   out_3061887074369766267[190] = 1.0;
   out_3061887074369766267[191] = 0.0;
   out_3061887074369766267[192] = 0.0;
   out_3061887074369766267[193] = 0.0;
   out_3061887074369766267[194] = 0.0;
   out_3061887074369766267[195] = 0.0;
   out_3061887074369766267[196] = 0.0;
   out_3061887074369766267[197] = 0.0;
   out_3061887074369766267[198] = 0.0;
   out_3061887074369766267[199] = 0.0;
   out_3061887074369766267[200] = 0.0;
   out_3061887074369766267[201] = 0.0;
   out_3061887074369766267[202] = 0.0;
   out_3061887074369766267[203] = 0.0;
   out_3061887074369766267[204] = 0.0;
   out_3061887074369766267[205] = 0.0;
   out_3061887074369766267[206] = 0.0;
   out_3061887074369766267[207] = 0.0;
   out_3061887074369766267[208] = 0.0;
   out_3061887074369766267[209] = 1.0;
   out_3061887074369766267[210] = 0.0;
   out_3061887074369766267[211] = 0.0;
   out_3061887074369766267[212] = 0.0;
   out_3061887074369766267[213] = 0.0;
   out_3061887074369766267[214] = 0.0;
   out_3061887074369766267[215] = 0.0;
   out_3061887074369766267[216] = 0.0;
   out_3061887074369766267[217] = 0.0;
   out_3061887074369766267[218] = 0.0;
   out_3061887074369766267[219] = 0.0;
   out_3061887074369766267[220] = 0.0;
   out_3061887074369766267[221] = 0.0;
   out_3061887074369766267[222] = 0.0;
   out_3061887074369766267[223] = 0.0;
   out_3061887074369766267[224] = 0.0;
   out_3061887074369766267[225] = 0.0;
   out_3061887074369766267[226] = 0.0;
   out_3061887074369766267[227] = 0.0;
   out_3061887074369766267[228] = 1.0;
   out_3061887074369766267[229] = 0.0;
   out_3061887074369766267[230] = 0.0;
   out_3061887074369766267[231] = 0.0;
   out_3061887074369766267[232] = 0.0;
   out_3061887074369766267[233] = 0.0;
   out_3061887074369766267[234] = 0.0;
   out_3061887074369766267[235] = 0.0;
   out_3061887074369766267[236] = 0.0;
   out_3061887074369766267[237] = 0.0;
   out_3061887074369766267[238] = 0.0;
   out_3061887074369766267[239] = 0.0;
   out_3061887074369766267[240] = 0.0;
   out_3061887074369766267[241] = 0.0;
   out_3061887074369766267[242] = 0.0;
   out_3061887074369766267[243] = 0.0;
   out_3061887074369766267[244] = 0.0;
   out_3061887074369766267[245] = 0.0;
   out_3061887074369766267[246] = 0.0;
   out_3061887074369766267[247] = 1.0;
   out_3061887074369766267[248] = 0.0;
   out_3061887074369766267[249] = 0.0;
   out_3061887074369766267[250] = 0.0;
   out_3061887074369766267[251] = 0.0;
   out_3061887074369766267[252] = 0.0;
   out_3061887074369766267[253] = 0.0;
   out_3061887074369766267[254] = 0.0;
   out_3061887074369766267[255] = 0.0;
   out_3061887074369766267[256] = 0.0;
   out_3061887074369766267[257] = 0.0;
   out_3061887074369766267[258] = 0.0;
   out_3061887074369766267[259] = 0.0;
   out_3061887074369766267[260] = 0.0;
   out_3061887074369766267[261] = 0.0;
   out_3061887074369766267[262] = 0.0;
   out_3061887074369766267[263] = 0.0;
   out_3061887074369766267[264] = 0.0;
   out_3061887074369766267[265] = 0.0;
   out_3061887074369766267[266] = 1.0;
   out_3061887074369766267[267] = 0.0;
   out_3061887074369766267[268] = 0.0;
   out_3061887074369766267[269] = 0.0;
   out_3061887074369766267[270] = 0.0;
   out_3061887074369766267[271] = 0.0;
   out_3061887074369766267[272] = 0.0;
   out_3061887074369766267[273] = 0.0;
   out_3061887074369766267[274] = 0.0;
   out_3061887074369766267[275] = 0.0;
   out_3061887074369766267[276] = 0.0;
   out_3061887074369766267[277] = 0.0;
   out_3061887074369766267[278] = 0.0;
   out_3061887074369766267[279] = 0.0;
   out_3061887074369766267[280] = 0.0;
   out_3061887074369766267[281] = 0.0;
   out_3061887074369766267[282] = 0.0;
   out_3061887074369766267[283] = 0.0;
   out_3061887074369766267[284] = 0.0;
   out_3061887074369766267[285] = 1.0;
   out_3061887074369766267[286] = 0.0;
   out_3061887074369766267[287] = 0.0;
   out_3061887074369766267[288] = 0.0;
   out_3061887074369766267[289] = 0.0;
   out_3061887074369766267[290] = 0.0;
   out_3061887074369766267[291] = 0.0;
   out_3061887074369766267[292] = 0.0;
   out_3061887074369766267[293] = 0.0;
   out_3061887074369766267[294] = 0.0;
   out_3061887074369766267[295] = 0.0;
   out_3061887074369766267[296] = 0.0;
   out_3061887074369766267[297] = 0.0;
   out_3061887074369766267[298] = 0.0;
   out_3061887074369766267[299] = 0.0;
   out_3061887074369766267[300] = 0.0;
   out_3061887074369766267[301] = 0.0;
   out_3061887074369766267[302] = 0.0;
   out_3061887074369766267[303] = 0.0;
   out_3061887074369766267[304] = 1.0;
   out_3061887074369766267[305] = 0.0;
   out_3061887074369766267[306] = 0.0;
   out_3061887074369766267[307] = 0.0;
   out_3061887074369766267[308] = 0.0;
   out_3061887074369766267[309] = 0.0;
   out_3061887074369766267[310] = 0.0;
   out_3061887074369766267[311] = 0.0;
   out_3061887074369766267[312] = 0.0;
   out_3061887074369766267[313] = 0.0;
   out_3061887074369766267[314] = 0.0;
   out_3061887074369766267[315] = 0.0;
   out_3061887074369766267[316] = 0.0;
   out_3061887074369766267[317] = 0.0;
   out_3061887074369766267[318] = 0.0;
   out_3061887074369766267[319] = 0.0;
   out_3061887074369766267[320] = 0.0;
   out_3061887074369766267[321] = 0.0;
   out_3061887074369766267[322] = 0.0;
   out_3061887074369766267[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_1911708801291879452) {
   out_1911708801291879452[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_1911708801291879452[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_1911708801291879452[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_1911708801291879452[3] = dt*state[12] + state[3];
   out_1911708801291879452[4] = dt*state[13] + state[4];
   out_1911708801291879452[5] = dt*state[14] + state[5];
   out_1911708801291879452[6] = state[6];
   out_1911708801291879452[7] = state[7];
   out_1911708801291879452[8] = state[8];
   out_1911708801291879452[9] = state[9];
   out_1911708801291879452[10] = state[10];
   out_1911708801291879452[11] = state[11];
   out_1911708801291879452[12] = state[12];
   out_1911708801291879452[13] = state[13];
   out_1911708801291879452[14] = state[14];
   out_1911708801291879452[15] = state[15];
   out_1911708801291879452[16] = state[16];
   out_1911708801291879452[17] = state[17];
}
void F_fun(double *state, double dt, double *out_8941015638488863399) {
   out_8941015638488863399[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8941015638488863399[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8941015638488863399[2] = 0;
   out_8941015638488863399[3] = 0;
   out_8941015638488863399[4] = 0;
   out_8941015638488863399[5] = 0;
   out_8941015638488863399[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8941015638488863399[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8941015638488863399[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8941015638488863399[9] = 0;
   out_8941015638488863399[10] = 0;
   out_8941015638488863399[11] = 0;
   out_8941015638488863399[12] = 0;
   out_8941015638488863399[13] = 0;
   out_8941015638488863399[14] = 0;
   out_8941015638488863399[15] = 0;
   out_8941015638488863399[16] = 0;
   out_8941015638488863399[17] = 0;
   out_8941015638488863399[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8941015638488863399[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8941015638488863399[20] = 0;
   out_8941015638488863399[21] = 0;
   out_8941015638488863399[22] = 0;
   out_8941015638488863399[23] = 0;
   out_8941015638488863399[24] = 0;
   out_8941015638488863399[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8941015638488863399[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8941015638488863399[27] = 0;
   out_8941015638488863399[28] = 0;
   out_8941015638488863399[29] = 0;
   out_8941015638488863399[30] = 0;
   out_8941015638488863399[31] = 0;
   out_8941015638488863399[32] = 0;
   out_8941015638488863399[33] = 0;
   out_8941015638488863399[34] = 0;
   out_8941015638488863399[35] = 0;
   out_8941015638488863399[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8941015638488863399[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8941015638488863399[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8941015638488863399[39] = 0;
   out_8941015638488863399[40] = 0;
   out_8941015638488863399[41] = 0;
   out_8941015638488863399[42] = 0;
   out_8941015638488863399[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8941015638488863399[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8941015638488863399[45] = 0;
   out_8941015638488863399[46] = 0;
   out_8941015638488863399[47] = 0;
   out_8941015638488863399[48] = 0;
   out_8941015638488863399[49] = 0;
   out_8941015638488863399[50] = 0;
   out_8941015638488863399[51] = 0;
   out_8941015638488863399[52] = 0;
   out_8941015638488863399[53] = 0;
   out_8941015638488863399[54] = 0;
   out_8941015638488863399[55] = 0;
   out_8941015638488863399[56] = 0;
   out_8941015638488863399[57] = 1;
   out_8941015638488863399[58] = 0;
   out_8941015638488863399[59] = 0;
   out_8941015638488863399[60] = 0;
   out_8941015638488863399[61] = 0;
   out_8941015638488863399[62] = 0;
   out_8941015638488863399[63] = 0;
   out_8941015638488863399[64] = 0;
   out_8941015638488863399[65] = 0;
   out_8941015638488863399[66] = dt;
   out_8941015638488863399[67] = 0;
   out_8941015638488863399[68] = 0;
   out_8941015638488863399[69] = 0;
   out_8941015638488863399[70] = 0;
   out_8941015638488863399[71] = 0;
   out_8941015638488863399[72] = 0;
   out_8941015638488863399[73] = 0;
   out_8941015638488863399[74] = 0;
   out_8941015638488863399[75] = 0;
   out_8941015638488863399[76] = 1;
   out_8941015638488863399[77] = 0;
   out_8941015638488863399[78] = 0;
   out_8941015638488863399[79] = 0;
   out_8941015638488863399[80] = 0;
   out_8941015638488863399[81] = 0;
   out_8941015638488863399[82] = 0;
   out_8941015638488863399[83] = 0;
   out_8941015638488863399[84] = 0;
   out_8941015638488863399[85] = dt;
   out_8941015638488863399[86] = 0;
   out_8941015638488863399[87] = 0;
   out_8941015638488863399[88] = 0;
   out_8941015638488863399[89] = 0;
   out_8941015638488863399[90] = 0;
   out_8941015638488863399[91] = 0;
   out_8941015638488863399[92] = 0;
   out_8941015638488863399[93] = 0;
   out_8941015638488863399[94] = 0;
   out_8941015638488863399[95] = 1;
   out_8941015638488863399[96] = 0;
   out_8941015638488863399[97] = 0;
   out_8941015638488863399[98] = 0;
   out_8941015638488863399[99] = 0;
   out_8941015638488863399[100] = 0;
   out_8941015638488863399[101] = 0;
   out_8941015638488863399[102] = 0;
   out_8941015638488863399[103] = 0;
   out_8941015638488863399[104] = dt;
   out_8941015638488863399[105] = 0;
   out_8941015638488863399[106] = 0;
   out_8941015638488863399[107] = 0;
   out_8941015638488863399[108] = 0;
   out_8941015638488863399[109] = 0;
   out_8941015638488863399[110] = 0;
   out_8941015638488863399[111] = 0;
   out_8941015638488863399[112] = 0;
   out_8941015638488863399[113] = 0;
   out_8941015638488863399[114] = 1;
   out_8941015638488863399[115] = 0;
   out_8941015638488863399[116] = 0;
   out_8941015638488863399[117] = 0;
   out_8941015638488863399[118] = 0;
   out_8941015638488863399[119] = 0;
   out_8941015638488863399[120] = 0;
   out_8941015638488863399[121] = 0;
   out_8941015638488863399[122] = 0;
   out_8941015638488863399[123] = 0;
   out_8941015638488863399[124] = 0;
   out_8941015638488863399[125] = 0;
   out_8941015638488863399[126] = 0;
   out_8941015638488863399[127] = 0;
   out_8941015638488863399[128] = 0;
   out_8941015638488863399[129] = 0;
   out_8941015638488863399[130] = 0;
   out_8941015638488863399[131] = 0;
   out_8941015638488863399[132] = 0;
   out_8941015638488863399[133] = 1;
   out_8941015638488863399[134] = 0;
   out_8941015638488863399[135] = 0;
   out_8941015638488863399[136] = 0;
   out_8941015638488863399[137] = 0;
   out_8941015638488863399[138] = 0;
   out_8941015638488863399[139] = 0;
   out_8941015638488863399[140] = 0;
   out_8941015638488863399[141] = 0;
   out_8941015638488863399[142] = 0;
   out_8941015638488863399[143] = 0;
   out_8941015638488863399[144] = 0;
   out_8941015638488863399[145] = 0;
   out_8941015638488863399[146] = 0;
   out_8941015638488863399[147] = 0;
   out_8941015638488863399[148] = 0;
   out_8941015638488863399[149] = 0;
   out_8941015638488863399[150] = 0;
   out_8941015638488863399[151] = 0;
   out_8941015638488863399[152] = 1;
   out_8941015638488863399[153] = 0;
   out_8941015638488863399[154] = 0;
   out_8941015638488863399[155] = 0;
   out_8941015638488863399[156] = 0;
   out_8941015638488863399[157] = 0;
   out_8941015638488863399[158] = 0;
   out_8941015638488863399[159] = 0;
   out_8941015638488863399[160] = 0;
   out_8941015638488863399[161] = 0;
   out_8941015638488863399[162] = 0;
   out_8941015638488863399[163] = 0;
   out_8941015638488863399[164] = 0;
   out_8941015638488863399[165] = 0;
   out_8941015638488863399[166] = 0;
   out_8941015638488863399[167] = 0;
   out_8941015638488863399[168] = 0;
   out_8941015638488863399[169] = 0;
   out_8941015638488863399[170] = 0;
   out_8941015638488863399[171] = 1;
   out_8941015638488863399[172] = 0;
   out_8941015638488863399[173] = 0;
   out_8941015638488863399[174] = 0;
   out_8941015638488863399[175] = 0;
   out_8941015638488863399[176] = 0;
   out_8941015638488863399[177] = 0;
   out_8941015638488863399[178] = 0;
   out_8941015638488863399[179] = 0;
   out_8941015638488863399[180] = 0;
   out_8941015638488863399[181] = 0;
   out_8941015638488863399[182] = 0;
   out_8941015638488863399[183] = 0;
   out_8941015638488863399[184] = 0;
   out_8941015638488863399[185] = 0;
   out_8941015638488863399[186] = 0;
   out_8941015638488863399[187] = 0;
   out_8941015638488863399[188] = 0;
   out_8941015638488863399[189] = 0;
   out_8941015638488863399[190] = 1;
   out_8941015638488863399[191] = 0;
   out_8941015638488863399[192] = 0;
   out_8941015638488863399[193] = 0;
   out_8941015638488863399[194] = 0;
   out_8941015638488863399[195] = 0;
   out_8941015638488863399[196] = 0;
   out_8941015638488863399[197] = 0;
   out_8941015638488863399[198] = 0;
   out_8941015638488863399[199] = 0;
   out_8941015638488863399[200] = 0;
   out_8941015638488863399[201] = 0;
   out_8941015638488863399[202] = 0;
   out_8941015638488863399[203] = 0;
   out_8941015638488863399[204] = 0;
   out_8941015638488863399[205] = 0;
   out_8941015638488863399[206] = 0;
   out_8941015638488863399[207] = 0;
   out_8941015638488863399[208] = 0;
   out_8941015638488863399[209] = 1;
   out_8941015638488863399[210] = 0;
   out_8941015638488863399[211] = 0;
   out_8941015638488863399[212] = 0;
   out_8941015638488863399[213] = 0;
   out_8941015638488863399[214] = 0;
   out_8941015638488863399[215] = 0;
   out_8941015638488863399[216] = 0;
   out_8941015638488863399[217] = 0;
   out_8941015638488863399[218] = 0;
   out_8941015638488863399[219] = 0;
   out_8941015638488863399[220] = 0;
   out_8941015638488863399[221] = 0;
   out_8941015638488863399[222] = 0;
   out_8941015638488863399[223] = 0;
   out_8941015638488863399[224] = 0;
   out_8941015638488863399[225] = 0;
   out_8941015638488863399[226] = 0;
   out_8941015638488863399[227] = 0;
   out_8941015638488863399[228] = 1;
   out_8941015638488863399[229] = 0;
   out_8941015638488863399[230] = 0;
   out_8941015638488863399[231] = 0;
   out_8941015638488863399[232] = 0;
   out_8941015638488863399[233] = 0;
   out_8941015638488863399[234] = 0;
   out_8941015638488863399[235] = 0;
   out_8941015638488863399[236] = 0;
   out_8941015638488863399[237] = 0;
   out_8941015638488863399[238] = 0;
   out_8941015638488863399[239] = 0;
   out_8941015638488863399[240] = 0;
   out_8941015638488863399[241] = 0;
   out_8941015638488863399[242] = 0;
   out_8941015638488863399[243] = 0;
   out_8941015638488863399[244] = 0;
   out_8941015638488863399[245] = 0;
   out_8941015638488863399[246] = 0;
   out_8941015638488863399[247] = 1;
   out_8941015638488863399[248] = 0;
   out_8941015638488863399[249] = 0;
   out_8941015638488863399[250] = 0;
   out_8941015638488863399[251] = 0;
   out_8941015638488863399[252] = 0;
   out_8941015638488863399[253] = 0;
   out_8941015638488863399[254] = 0;
   out_8941015638488863399[255] = 0;
   out_8941015638488863399[256] = 0;
   out_8941015638488863399[257] = 0;
   out_8941015638488863399[258] = 0;
   out_8941015638488863399[259] = 0;
   out_8941015638488863399[260] = 0;
   out_8941015638488863399[261] = 0;
   out_8941015638488863399[262] = 0;
   out_8941015638488863399[263] = 0;
   out_8941015638488863399[264] = 0;
   out_8941015638488863399[265] = 0;
   out_8941015638488863399[266] = 1;
   out_8941015638488863399[267] = 0;
   out_8941015638488863399[268] = 0;
   out_8941015638488863399[269] = 0;
   out_8941015638488863399[270] = 0;
   out_8941015638488863399[271] = 0;
   out_8941015638488863399[272] = 0;
   out_8941015638488863399[273] = 0;
   out_8941015638488863399[274] = 0;
   out_8941015638488863399[275] = 0;
   out_8941015638488863399[276] = 0;
   out_8941015638488863399[277] = 0;
   out_8941015638488863399[278] = 0;
   out_8941015638488863399[279] = 0;
   out_8941015638488863399[280] = 0;
   out_8941015638488863399[281] = 0;
   out_8941015638488863399[282] = 0;
   out_8941015638488863399[283] = 0;
   out_8941015638488863399[284] = 0;
   out_8941015638488863399[285] = 1;
   out_8941015638488863399[286] = 0;
   out_8941015638488863399[287] = 0;
   out_8941015638488863399[288] = 0;
   out_8941015638488863399[289] = 0;
   out_8941015638488863399[290] = 0;
   out_8941015638488863399[291] = 0;
   out_8941015638488863399[292] = 0;
   out_8941015638488863399[293] = 0;
   out_8941015638488863399[294] = 0;
   out_8941015638488863399[295] = 0;
   out_8941015638488863399[296] = 0;
   out_8941015638488863399[297] = 0;
   out_8941015638488863399[298] = 0;
   out_8941015638488863399[299] = 0;
   out_8941015638488863399[300] = 0;
   out_8941015638488863399[301] = 0;
   out_8941015638488863399[302] = 0;
   out_8941015638488863399[303] = 0;
   out_8941015638488863399[304] = 1;
   out_8941015638488863399[305] = 0;
   out_8941015638488863399[306] = 0;
   out_8941015638488863399[307] = 0;
   out_8941015638488863399[308] = 0;
   out_8941015638488863399[309] = 0;
   out_8941015638488863399[310] = 0;
   out_8941015638488863399[311] = 0;
   out_8941015638488863399[312] = 0;
   out_8941015638488863399[313] = 0;
   out_8941015638488863399[314] = 0;
   out_8941015638488863399[315] = 0;
   out_8941015638488863399[316] = 0;
   out_8941015638488863399[317] = 0;
   out_8941015638488863399[318] = 0;
   out_8941015638488863399[319] = 0;
   out_8941015638488863399[320] = 0;
   out_8941015638488863399[321] = 0;
   out_8941015638488863399[322] = 0;
   out_8941015638488863399[323] = 1;
}
void h_4(double *state, double *unused, double *out_1305004607597586938) {
   out_1305004607597586938[0] = state[6] + state[9];
   out_1305004607597586938[1] = state[7] + state[10];
   out_1305004607597586938[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_7184552115366560582) {
   out_7184552115366560582[0] = 0;
   out_7184552115366560582[1] = 0;
   out_7184552115366560582[2] = 0;
   out_7184552115366560582[3] = 0;
   out_7184552115366560582[4] = 0;
   out_7184552115366560582[5] = 0;
   out_7184552115366560582[6] = 1;
   out_7184552115366560582[7] = 0;
   out_7184552115366560582[8] = 0;
   out_7184552115366560582[9] = 1;
   out_7184552115366560582[10] = 0;
   out_7184552115366560582[11] = 0;
   out_7184552115366560582[12] = 0;
   out_7184552115366560582[13] = 0;
   out_7184552115366560582[14] = 0;
   out_7184552115366560582[15] = 0;
   out_7184552115366560582[16] = 0;
   out_7184552115366560582[17] = 0;
   out_7184552115366560582[18] = 0;
   out_7184552115366560582[19] = 0;
   out_7184552115366560582[20] = 0;
   out_7184552115366560582[21] = 0;
   out_7184552115366560582[22] = 0;
   out_7184552115366560582[23] = 0;
   out_7184552115366560582[24] = 0;
   out_7184552115366560582[25] = 1;
   out_7184552115366560582[26] = 0;
   out_7184552115366560582[27] = 0;
   out_7184552115366560582[28] = 1;
   out_7184552115366560582[29] = 0;
   out_7184552115366560582[30] = 0;
   out_7184552115366560582[31] = 0;
   out_7184552115366560582[32] = 0;
   out_7184552115366560582[33] = 0;
   out_7184552115366560582[34] = 0;
   out_7184552115366560582[35] = 0;
   out_7184552115366560582[36] = 0;
   out_7184552115366560582[37] = 0;
   out_7184552115366560582[38] = 0;
   out_7184552115366560582[39] = 0;
   out_7184552115366560582[40] = 0;
   out_7184552115366560582[41] = 0;
   out_7184552115366560582[42] = 0;
   out_7184552115366560582[43] = 0;
   out_7184552115366560582[44] = 1;
   out_7184552115366560582[45] = 0;
   out_7184552115366560582[46] = 0;
   out_7184552115366560582[47] = 1;
   out_7184552115366560582[48] = 0;
   out_7184552115366560582[49] = 0;
   out_7184552115366560582[50] = 0;
   out_7184552115366560582[51] = 0;
   out_7184552115366560582[52] = 0;
   out_7184552115366560582[53] = 0;
}
void h_10(double *state, double *unused, double *out_7882420333113587623) {
   out_7882420333113587623[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_7882420333113587623[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_7882420333113587623[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_1143390991321892766) {
   out_1143390991321892766[0] = 0;
   out_1143390991321892766[1] = 9.8100000000000005*cos(state[1]);
   out_1143390991321892766[2] = 0;
   out_1143390991321892766[3] = 0;
   out_1143390991321892766[4] = -state[8];
   out_1143390991321892766[5] = state[7];
   out_1143390991321892766[6] = 0;
   out_1143390991321892766[7] = state[5];
   out_1143390991321892766[8] = -state[4];
   out_1143390991321892766[9] = 0;
   out_1143390991321892766[10] = 0;
   out_1143390991321892766[11] = 0;
   out_1143390991321892766[12] = 1;
   out_1143390991321892766[13] = 0;
   out_1143390991321892766[14] = 0;
   out_1143390991321892766[15] = 1;
   out_1143390991321892766[16] = 0;
   out_1143390991321892766[17] = 0;
   out_1143390991321892766[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_1143390991321892766[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_1143390991321892766[20] = 0;
   out_1143390991321892766[21] = state[8];
   out_1143390991321892766[22] = 0;
   out_1143390991321892766[23] = -state[6];
   out_1143390991321892766[24] = -state[5];
   out_1143390991321892766[25] = 0;
   out_1143390991321892766[26] = state[3];
   out_1143390991321892766[27] = 0;
   out_1143390991321892766[28] = 0;
   out_1143390991321892766[29] = 0;
   out_1143390991321892766[30] = 0;
   out_1143390991321892766[31] = 1;
   out_1143390991321892766[32] = 0;
   out_1143390991321892766[33] = 0;
   out_1143390991321892766[34] = 1;
   out_1143390991321892766[35] = 0;
   out_1143390991321892766[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_1143390991321892766[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_1143390991321892766[38] = 0;
   out_1143390991321892766[39] = -state[7];
   out_1143390991321892766[40] = state[6];
   out_1143390991321892766[41] = 0;
   out_1143390991321892766[42] = state[4];
   out_1143390991321892766[43] = -state[3];
   out_1143390991321892766[44] = 0;
   out_1143390991321892766[45] = 0;
   out_1143390991321892766[46] = 0;
   out_1143390991321892766[47] = 0;
   out_1143390991321892766[48] = 0;
   out_1143390991321892766[49] = 0;
   out_1143390991321892766[50] = 1;
   out_1143390991321892766[51] = 0;
   out_1143390991321892766[52] = 0;
   out_1143390991321892766[53] = 1;
}
void h_13(double *state, double *unused, double *out_3776293256350619509) {
   out_3776293256350619509[0] = state[3];
   out_3776293256350619509[1] = state[4];
   out_3776293256350619509[2] = state[5];
}
void H_13(double *state, double *unused, double *out_426079092950140347) {
   out_426079092950140347[0] = 0;
   out_426079092950140347[1] = 0;
   out_426079092950140347[2] = 0;
   out_426079092950140347[3] = 1;
   out_426079092950140347[4] = 0;
   out_426079092950140347[5] = 0;
   out_426079092950140347[6] = 0;
   out_426079092950140347[7] = 0;
   out_426079092950140347[8] = 0;
   out_426079092950140347[9] = 0;
   out_426079092950140347[10] = 0;
   out_426079092950140347[11] = 0;
   out_426079092950140347[12] = 0;
   out_426079092950140347[13] = 0;
   out_426079092950140347[14] = 0;
   out_426079092950140347[15] = 0;
   out_426079092950140347[16] = 0;
   out_426079092950140347[17] = 0;
   out_426079092950140347[18] = 0;
   out_426079092950140347[19] = 0;
   out_426079092950140347[20] = 0;
   out_426079092950140347[21] = 0;
   out_426079092950140347[22] = 1;
   out_426079092950140347[23] = 0;
   out_426079092950140347[24] = 0;
   out_426079092950140347[25] = 0;
   out_426079092950140347[26] = 0;
   out_426079092950140347[27] = 0;
   out_426079092950140347[28] = 0;
   out_426079092950140347[29] = 0;
   out_426079092950140347[30] = 0;
   out_426079092950140347[31] = 0;
   out_426079092950140347[32] = 0;
   out_426079092950140347[33] = 0;
   out_426079092950140347[34] = 0;
   out_426079092950140347[35] = 0;
   out_426079092950140347[36] = 0;
   out_426079092950140347[37] = 0;
   out_426079092950140347[38] = 0;
   out_426079092950140347[39] = 0;
   out_426079092950140347[40] = 0;
   out_426079092950140347[41] = 1;
   out_426079092950140347[42] = 0;
   out_426079092950140347[43] = 0;
   out_426079092950140347[44] = 0;
   out_426079092950140347[45] = 0;
   out_426079092950140347[46] = 0;
   out_426079092950140347[47] = 0;
   out_426079092950140347[48] = 0;
   out_426079092950140347[49] = 0;
   out_426079092950140347[50] = 0;
   out_426079092950140347[51] = 0;
   out_426079092950140347[52] = 0;
   out_426079092950140347[53] = 0;
}
void h_14(double *state, double *unused, double *out_7912039560915361515) {
   out_7912039560915361515[0] = state[6];
   out_7912039560915361515[1] = state[7];
   out_7912039560915361515[2] = state[8];
}
void H_14(double *state, double *unused, double *out_3221311259027076053) {
   out_3221311259027076053[0] = 0;
   out_3221311259027076053[1] = 0;
   out_3221311259027076053[2] = 0;
   out_3221311259027076053[3] = 0;
   out_3221311259027076053[4] = 0;
   out_3221311259027076053[5] = 0;
   out_3221311259027076053[6] = 1;
   out_3221311259027076053[7] = 0;
   out_3221311259027076053[8] = 0;
   out_3221311259027076053[9] = 0;
   out_3221311259027076053[10] = 0;
   out_3221311259027076053[11] = 0;
   out_3221311259027076053[12] = 0;
   out_3221311259027076053[13] = 0;
   out_3221311259027076053[14] = 0;
   out_3221311259027076053[15] = 0;
   out_3221311259027076053[16] = 0;
   out_3221311259027076053[17] = 0;
   out_3221311259027076053[18] = 0;
   out_3221311259027076053[19] = 0;
   out_3221311259027076053[20] = 0;
   out_3221311259027076053[21] = 0;
   out_3221311259027076053[22] = 0;
   out_3221311259027076053[23] = 0;
   out_3221311259027076053[24] = 0;
   out_3221311259027076053[25] = 1;
   out_3221311259027076053[26] = 0;
   out_3221311259027076053[27] = 0;
   out_3221311259027076053[28] = 0;
   out_3221311259027076053[29] = 0;
   out_3221311259027076053[30] = 0;
   out_3221311259027076053[31] = 0;
   out_3221311259027076053[32] = 0;
   out_3221311259027076053[33] = 0;
   out_3221311259027076053[34] = 0;
   out_3221311259027076053[35] = 0;
   out_3221311259027076053[36] = 0;
   out_3221311259027076053[37] = 0;
   out_3221311259027076053[38] = 0;
   out_3221311259027076053[39] = 0;
   out_3221311259027076053[40] = 0;
   out_3221311259027076053[41] = 0;
   out_3221311259027076053[42] = 0;
   out_3221311259027076053[43] = 0;
   out_3221311259027076053[44] = 1;
   out_3221311259027076053[45] = 0;
   out_3221311259027076053[46] = 0;
   out_3221311259027076053[47] = 0;
   out_3221311259027076053[48] = 0;
   out_3221311259027076053[49] = 0;
   out_3221311259027076053[50] = 0;
   out_3221311259027076053[51] = 0;
   out_3221311259027076053[52] = 0;
   out_3221311259027076053[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_1425647101528206710) {
  err_fun(nom_x, delta_x, out_1425647101528206710);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1586401905831597934) {
  inv_err_fun(nom_x, true_x, out_1586401905831597934);
}
void pose_H_mod_fun(double *state, double *out_3061887074369766267) {
  H_mod_fun(state, out_3061887074369766267);
}
void pose_f_fun(double *state, double dt, double *out_1911708801291879452) {
  f_fun(state,  dt, out_1911708801291879452);
}
void pose_F_fun(double *state, double dt, double *out_8941015638488863399) {
  F_fun(state,  dt, out_8941015638488863399);
}
void pose_h_4(double *state, double *unused, double *out_1305004607597586938) {
  h_4(state, unused, out_1305004607597586938);
}
void pose_H_4(double *state, double *unused, double *out_7184552115366560582) {
  H_4(state, unused, out_7184552115366560582);
}
void pose_h_10(double *state, double *unused, double *out_7882420333113587623) {
  h_10(state, unused, out_7882420333113587623);
}
void pose_H_10(double *state, double *unused, double *out_1143390991321892766) {
  H_10(state, unused, out_1143390991321892766);
}
void pose_h_13(double *state, double *unused, double *out_3776293256350619509) {
  h_13(state, unused, out_3776293256350619509);
}
void pose_H_13(double *state, double *unused, double *out_426079092950140347) {
  H_13(state, unused, out_426079092950140347);
}
void pose_h_14(double *state, double *unused, double *out_7912039560915361515) {
  h_14(state, unused, out_7912039560915361515);
}
void pose_H_14(double *state, double *unused, double *out_3221311259027076053) {
  H_14(state, unused, out_3221311259027076053);
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

/*-----------------------------------------------------------------------*\
 |  file: Baumgarte_Pars.hh                                              |
 |                                                                       |
 |  version: 1.0   date 2/7/2020                                         |
 |                                                                       |
 |  Copyright (C) 2020                                                   |
 |                                                                       |
 |      Enrico Bertolazzi, Francesco Biral and Paolo Bosetti             |
 |      Dipartimento di Ingegneria Industriale                           |
 |      Universita` degli Studi di Trento                                |
 |      Via Sommarive 9, I-38123, Trento, Italy                          |
 |      email: enrico.bertolazzi@unitn.it                                |
 |             francesco.biral@unitn.it                                  |
 |             paolo.bosetti@unitn.it                                    |
\*-----------------------------------------------------------------------*/


#ifndef BAUMGARTEPARS_HH
#define BAUMGARTEPARS_HH

#define numBc                    0
#define numModelPars             275
#define numConstraint1D          11
#define numConstraint2D          0
#define numConstraintU           4
#define numXvars                 39
#define numLvars                 39
#define numUvars                 4
#define numOMEGAvars             0
#define numQvars                 13
#define numPvars                 0
#define numPostProcess           52
#define numIntegratedPostProcess 1
#define numContinuationSteps     5

// Xvars
#define iX_u                       0
#define iX_v                       1
#define iX_Omega                   2
#define iX_phi                     3
#define iX_theta                   4
#define iX_h                       5
#define iX_delta                   6
#define iX_eta                     7
#define iX_s__f                    8
#define iX_x__f                    9
#define iX_y__f                    10
#define iX_z__f                    11
#define iX_x__r                    12
#define iX_y__r                    13
#define iX_z__r                    14
#define iX_delta__f                15
#define iX_phi__f                  16
#define iX_omega__r                17
#define iX_omega__f                18
#define iX_phi__dot                19
#define iX_theta__dot              20
#define iX_h__dot                  21
#define iX_delta__dot              22
#define iX_eta__dot                23
#define iX_s__f__dot               24
#define iX_x__f__dot               25
#define iX_y__f__dot               26
#define iX_z__f__dot               27
#define iX_x__r__dot               28
#define iX_y__r__dot               29
#define iX_z__r__dot               30
#define iX_delta__f__dot           31
#define iX_phi__f__dot             32
#define iX_Ftr                     33
#define iX_Mbf                     34
#define iX_Mbr                     35
#define iX_tau                     36
#define iX_n                       37
#define iX_xi                      38

// Lvars
#define iL_lambda1__xo             0
#define iL_lambda2__xo             1
#define iL_lambda3__xo             2
#define iL_lambda4__xo             3
#define iL_lambda5__xo             4
#define iL_lambda6__xo             5
#define iL_lambda7__xo             6
#define iL_lambda8__xo             7
#define iL_lambda9__xo             8
#define iL_lambda10__xo            9
#define iL_lambda11__xo            10
#define iL_lambda12__xo            11
#define iL_lambda13__xo            12
#define iL_lambda14__xo            13
#define iL_lambda15__xo            14
#define iL_lambda16__xo            15
#define iL_lambda17__xo            16
#define iL_lambda18__xo            17
#define iL_lambda19__xo            18
#define iL_lambda20__xo            19
#define iL_lambda21__xo            20
#define iL_lambda22__xo            21
#define iL_lambda23__xo            22
#define iL_lambda24__xo            23
#define iL_lambda25__xo            24
#define iL_lambda26__xo            25
#define iL_lambda27__xo            26
#define iL_lambda28__xo            27
#define iL_lambda29__xo            28
#define iL_lambda30__xo            29
#define iL_lambda31__xo            30
#define iL_lambda32__xo            31
#define iL_lambda33__xo            32
#define iL_lambda34__xo            33
#define iL_lambda35__xo            34
#define iL_lambda36__xo            35
#define iL_lambda37__xo            36
#define iL_lambda38__xo            37
#define iL_lambda39__xo            38

// Uvars
#define iU_b__f__o                 0
#define iU_b__r__o                 1
#define iU_t__o                    2
#define iU_tau__o                  3

// Qvars
#define iQ_zeta                    0
#define iQ_kappa                   1
#define iQ_leftWidth               2
#define iQ_rightWidth              3
#define iQ_sectionSpeedLimit       4
#define iQ_adherence               5
#define iQ_xISOMidLane             6
#define iQ_yISOMidLane             7
#define iQ_xISOleft                8
#define iQ_yISOleft                9
#define iQ_xISOright               10
#define iQ_yISOright               11
#define iQ_ISOAngle                12

// Pvars

// ModelPars Maps
#define iM_CXZ                     0
#define iM_Ca                      1
#define iM_Cp__f                   2
#define iM_Cp__r                   3
#define iM_Ftr__ss                 4
#define iM_Fz0                     5
#define iM_Fz0__f                  6
#define iM_Fz0__r                  7
#define iM_Fzmin                   8
#define iM_Fznorm                  9
#define iM_IX                      10
#define iM_IY                      11
#define iM_IZ                      12
#define iM_Id__wf                  13
#define iM_Ix__rdr                 14
#define iM_Ix__swa                 15
#define iM_Iy__swa                 16
#define iM_Iy__wf                  17
#define iM_Iy__wr                  18
#define iM_Iz__rdr                 19
#define iM_Iz__swa                 20
#define iM_Kp__f                   21
#define iM_Kp__r                   22
#define iM_L__b                    23
#define iM_L__swa                  24
#define iM_M__tot                  25
#define iM_Mbf__ss                 26
#define iM_Mbr__ss                 27
#define iM_XG                      28
#define iM_ZG                      29
#define iM_a__1                    30
#define iM_braking                 31
#define iM_c__fs                   32
#define iM_c__rs                   33
#define iM_d1__f                   34
#define iM_d1__r                   35
#define iM_d2__f                   36
#define iM_d2__r                   37
#define iM_d3__f                   38
#define iM_d3__r                   39
#define iM_d4__f                   40
#define iM_d4__r                   41
#define iM_d5__f                   42
#define iM_d5__r                   43
#define iM_d6__f                   44
#define iM_d6__r                   45
#define iM_d7__f                   46
#define iM_d7__r                   47
#define iM_d8                      48
#define iM_e10                     49
#define iM_e1__f                   50
#define iM_e1__r                   51
#define iM_e2__f                   52
#define iM_e2__r                   53
#define iM_e4                      54
#define iM_e5                      55
#define iM_e6                      56
#define iM_e7                      57
#define iM_e9                      58
#define iM_eps_c0                  59
#define iM_eps_c1                  60
#define iM_eps_p0                  61
#define iM_eps_p1                  62
#define iM_epsilon                 63
#define iM_eta__00                 64
#define iM_eta__ss                 65
#define iM_g                       66
#define iM_h__rdr                  67
#define iM_h__ss                   68
#define iM_h_c                     69
#define iM_k__fs                   70
#define iM_k__rs                   71
#define iM_m__rdr                  72
#define iM_m__swa                  73
#define iM_m__wf                   74
#define iM_m__wr                   75
#define iM_n__ss                   76
#define iM_pCx1__f                 77
#define iM_pCx1__r                 78
#define iM_pDx1__f                 79
#define iM_pDx1__r                 80
#define iM_pDx2__f                 81
#define iM_pDx2__r                 82
#define iM_pKx1__f                 83
#define iM_pKx1__r                 84
#define iM_pKx2__f                 85
#define iM_pKx2__r                 86
#define iM_pKx3__f                 87
#define iM_pKx3__r                 88
#define iM_phi__ss                 89
#define iM_rBx1__f                 90
#define iM_rBx1__r                 91
#define iM_rBx2__f                 92
#define iM_rBx2__r                 93
#define iM_rBx3__f                 94
#define iM_rBx3__r                 95
#define iM_rBy1__f                 96
#define iM_rBy1__r                 97
#define iM_rBy2__f                 98
#define iM_rBy2__r                 99
#define iM_rBy3__f                 100
#define iM_rBy3__r                 101
#define iM_rCx1__f                 102
#define iM_rCx1__r                 103
#define iM_rCy1__f                 104
#define iM_rCy1__r                 105
#define iM_rHx1__f                 106
#define iM_rHx1__r                 107
#define iM_rHy1__f                 108
#define iM_rHy1__r                 109
#define iM_rHy2__f                 110
#define iM_rHy2__r                 111
#define iM_r__crw                  112
#define iM_r__pin                  113
#define iM_rf                      114
#define iM_rr                      115
#define iM_rtf                     116
#define iM_rtr                     117
#define iM_s__fs                   118
#define iM_tau__ss                 119
#define iM_tol_c0                  120
#define iM_tol_c1                  121
#define iM_tol_p0                  122
#define iM_tol_p1                  123
#define iM_u__ss                   124
#define iM_v__ss                   125
#define iM_w__LR                   126
#define iM_w__ic                   127
#define iM_w__ic0                  128
#define iM_w__ic1                  129
#define iM_w__n0                   130
#define iM_w__n00                  131
#define iM_w__n01                  132
#define iM_w__ss                   133
#define iM_w__ss0                  134
#define iM_w__ss1                  135
#define iM_w__t                    136
#define iM_w__t0                   137
#define iM_w__t1                   138
#define iM_x__a                    139
#define iM_x__off                  140
#define iM_x__rdr                  141
#define iM_xi__n                   142
#define iM_xi__ss                  143
#define iM_z__a                    144
#define iM_z__rdr                  145
#define iM_C__delta                146
#define iM_Cxz__delta              147
#define iM_Cxz__swa                148
#define iM_Ix__delta               149
#define iM_Iy__delta               150
#define iM_Iz__delta               151
#define iM_Mbf__max                152
#define iM_Mbr__max                153
#define iM_Omega__ss               154
#define iM_beta__max               155
#define iM_closed_gas              156
#define iM_delta__dot__ss          157
#define iM_delta__f__ss            158
#define iM_delta__max              159
#define iM_delta__ss               160
#define iM_epsilon__x__f           161
#define iM_epsilon__x__r           162
#define iM_eta__dot__ss            163
#define iM_h__dot__ss              164
#define iM_lambda__C__x__f         165
#define iM_lambda__C__x__r         166
#define iM_lambda__K__x__f         167
#define iM_lambda__K__x__r         168
#define iM_lat_peak_front          169
#define iM_lat_peak_rear           170
#define iM_long_peak_front         171
#define iM_long_peak_rear          172
#define iM_m__delta                173
#define iM_not_braking             174
#define iM_omega__f__ss            175
#define iM_omega__n                176
#define iM_omega__r__ss            177
#define iM_open_gas                178
#define iM_phi__dot__ss            179
#define iM_phi__f__dot__ss         180
#define iM_phi__f__ss              181
#define iM_phi__max                182
#define iM_s__f__00                183
#define iM_s__f__dot__ss           184
#define iM_s__f__ss                185
#define iM_stering_norm            186
#define iM_tau__m__f               187
#define iM_tau__m__r               188
#define iM_tau__m__s               189
#define iM_tau__m__t               190
#define iM_tau__max                191
#define iM_theta__d__00            192
#define iM_theta__dot__ss          193
#define iM_theta__ss               194
#define iM_w__Ftr__bc              195
#define iM_w__Mbf__bc              196
#define iM_w__Mbr__bc              197
#define iM_w__Omega__bc            198
#define iM_w__delta__bc            199
#define iM_w__delta__f__bc         200
#define iM_w__eta__bc              201
#define iM_w__eta__dot__bc         202
#define iM_w__eta__sserr           203
#define iM_w__h__bc                204
#define iM_w__h__dot__bc           205
#define iM_w__h__sserr             206
#define iM_w__n__bc                207
#define iM_w__omega__f__bc         208
#define iM_w__omega__r__bc         209
#define iM_w__phi__bc              210
#define iM_w__phi__dot__bc         211
#define iM_w__phi__f__bc           212
#define iM_w__s__f__bc             213
#define iM_w__s__f__sserr          214
#define iM_w__tau__bc              215
#define iM_w__theta__bc            216
#define iM_w__theta__sserr         217
#define iM_w__u__bc                218
#define iM_w__v__bc                219
#define iM_w__x__f__bc             220
#define iM_w__x__r__bc             221
#define iM_w__xi__bc               222
#define iM_w__y__f__bc             223
#define iM_w__y__r__bc             224
#define iM_w__z__f__bc             225
#define iM_w__z__r__bc             226
#define iM_x__Swing                227
#define iM_x__delta                228
#define iM_x__f__dot__ss           229
#define iM_x__f__ss                230
#define iM_x__r__dot__ss           231
#define iM_x__r__ss                232
#define iM_y__f__dot__ss           233
#define iM_y__f__ss                234
#define iM_y__r__dot__ss           235
#define iM_y__r__ss                236
#define iM_z__Swing                237
#define iM_z__delta                238
#define iM_z__f__dot__ss           239
#define iM_z__f__ss                240
#define iM_z__r__dot__ss           241
#define iM_z__r__ss                242
#define iM_delta__f__dot__ss       243
#define iM_lambda__mu__x__f        244
#define iM_lambda__mu__x__r        245
#define iM_long_peak_front_0       246
#define iM_long_peak_front_1       247
#define iM_long_peak_rear_0        248
#define iM_long_peak_rear_1        249
#define iM_w__delta__dot__bc       250
#define iM_w__delta__dot__sserr    251
#define iM_w__delta__f__dot__bc    252
#define iM_w__delta__f__dot__sserr 253
#define iM_w__eta__dot__sserr      254
#define iM_w__h__dot__sserr        255
#define iM_w__phi__dot__sserr      256
#define iM_w__phi__f__dot__bc      257
#define iM_w__phi__f__dot__sserr   258
#define iM_w__s__f__dot__bc        259
#define iM_w__s__f__dot__sserr     260
#define iM_w__theta__dot__bc       261
#define iM_w__theta__dot__sserr    262
#define iM_w__x__f__dot__bc        263
#define iM_w__x__f__dot__sserr     264
#define iM_w__x__r__dot__bc        265
#define iM_w__x__r__dot__sserr     266
#define iM_w__y__f__dot__bc        267
#define iM_w__y__f__dot__sserr     268
#define iM_w__y__r__dot__bc        269
#define iM_w__y__r__dot__sserr     270
#define iM_w__z__f__dot__bc        271
#define iM_w__z__f__dot__sserr     272
#define iM_w__z__r__dot__bc        273
#define iM_w__z__r__dot__sserr     274

#endif

// EOF: Baumgarte_Pars.hh

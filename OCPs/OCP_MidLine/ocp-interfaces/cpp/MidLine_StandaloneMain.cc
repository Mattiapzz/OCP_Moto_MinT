/*-----------------------------------------------------------------------*\
 |  file: MidLine_Main.cc                                                |
 |                                                                       |
 |  version: 1.0   date 25/6/2020                                        |
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


#include "MidLine.hh"
#include "MidLine_Pars.hh"

using namespace std;
using Mechatronix::real_type;
using Mechatronix::integer;
using Mechatronix::ostream_type;

// user class in namespaces
using Mechatronix::Engine;
using Mechatronix::Road2D;

using namespace MidLineLoad;
using GenericContainerNamespace::GenericContainer;

static bool SET  = true;
static bool FREE = false;

int
main() {

  #ifdef MECHATRONIX_OS_WINDOWS
  __try {
  #endif

  Mechatronix::Console    console(&std::cout,4);
  Mechatronix::ThreadPool TP(std::thread::hardware_concurrency());

  try {

    MidLine          model("MidLine",&TP,&console);
    GenericContainer gc_data;
    GenericContainer gc_solution;

    // user defined Object instances (external)
    Engine           engine( "engine" );
    Road2D           road( "road" );

    // Auxiliary values
   real_type stering_norm = 1;
   real_type delta__max = 1/18.0*Math::PI;
   real_type phi__max = 7/18.0*Math::PI;
   real_type not_braking = 0;
   real_type w__LR0 = 0;
   real_type tol_c0 = 0.1;
   real_type tol_p0 = 0.1;
   real_type w__LR = w__LR0;
   real_type long_peak_front = 0.11;
   real_type w__ss0 = 1;
   real_type w__ss = w__ss0;
   real_type eps_c0 = 0.1;
   real_type eps_c = eps_c0;
   real_type eps_p0 = 0.1;
   real_type lat_peak_rear = 1;
   real_type lat_peak_front = 1;
   real_type e__b0 = 0.1;
   real_type e__b = e__b0;
   real_type tol_c = tol_c0;
   real_type closed_gas = 0;
   real_type tol_p = tol_p0;
   real_type w__ic0 = 1;
   real_type braking = 1;
   real_type w__ic = w__ic0;
   real_type long_peak_rear = 0.11;
   real_type w__t0 = 1/100000.0;
   real_type w__t = w__t0;
   real_type eps_p = eps_p0;
   real_type p__b0 = 0.1;
   real_type p__b = p__b0;
   real_type open_gas = 1;
    integer InfoLevel = 4;

    GenericContainer &  data_ControlSolver = gc_data["ControlSolver"];
    // ==============================================================
    // 'LU', 'LUPQ', 'QR', 'QRP', 'SVD', 'LSS', 'LSY', 'MINIMIZATION'
    // :factorization => 'LU',
    // ==============================================================
    data_ControlSolver["Rcond"]     = 1e-14; // reciprocal condition number threshold for QR, SVD, LSS, LSY
    data_ControlSolver["MaxIter"]   = 50;
    data_ControlSolver["Tolerance"] = 1e-9;
    data_ControlSolver["Iterative"] = false;
    data_ControlSolver["InfoLevel"] = 1;

    // Enable doctor
    gc_data["Doctor"] = false;

    // Enable check jacobian
    gc_data["JacobianCheck"]            = false;
    gc_data["JacobianCheckFull"]        = false;
    gc_data["JacobianCheck_epsilon"]    = 1e-4;
    gc_data["FiniteDifferenceJacobian"] = false;

    // Redirect output to GenericContainer["stream_output"]
    gc_data["RedirectStreamToString"] = false;

    // Dump Function and Jacobian if uncommented
    gc_data["DumpFile"] = "MidLine_dump";

    // spline output (all values as function of "s")
    gc_data["OutputSplines"] = "s";

    // setup solver
    GenericContainer & data_Solver = gc_data["Solver"];
    // Linear algebra factorization selection:
    // 'LU', 'QR', 'QRP', 'SUPERLU'
    // =================
    data_Solver["factorization"] = "LU";
    // =================

    // Last Block selection:
    // 'LU', 'LUPQ', 'QR', 'QRP', 'SVD', 'LSS', 'LSY'
    // ==============================================
    data_Solver["last_factorization"] = "LU";
    // ==============================================

    // choose solver: Hyness, NewtonDumped
    // ===================================
    data_Solver["solver"] = "Hyness";
    // ===================================

    // solver parameters
    data_Solver["max_iter"]              = 300;
    data_Solver["max_step_iter"]         = 40;
    data_Solver["max_accumulated_iter"]  = 800;
    data_Solver["tolerance"]             = 9.999999999999999e-10;
    // continuation parameters
    data_Solver["ns_continuation_begin"] = 0;
    data_Solver["ns_continuation_end"]   = 4;
    GenericContainer & data_Continuation = data_Solver["continuation"];
    data_Continuation["initial_step"]   = 0.2;   // initial step for continuation
    data_Continuation["min_step"]       = 0.001; // minimum accepted step for continuation
    data_Continuation["reduce_factor"]  = 0.5;   // p fails, reduce step by this factor
    data_Continuation["augment_factor"] = 1.5;   // if step successful in less than few_iteration augment step by this factor
    data_Continuation["few_iterations"] = 8;

    // Boundary Conditions
     GenericContainer & data_BoundaryConditions = gc_data["BoundaryConditions"];

    // Guess
    GenericContainer & data_Guess = gc_data["Guess"];
    // possible value: zero, default, none, warm
    data_Guess["initialize"] = "zero";
    // possible value: default, none, warm, spline, table
    data_Guess["guess_type"] = "default";

    GenericContainer & data_Parameters = gc_data["Parameters"];
    // Model Parameters
    data_Parameters["CXZ"] = 0.38736092039245;
    data_Parameters["Ca"] = 0.3;
    data_Parameters["Fzmin"] = 200;
    data_Parameters["IX"] = 42.6268656266919;
    data_Parameters["IY"] = 79.5529675764739;
    data_Parameters["IZ"] = 39.0041019529552;
    data_Parameters["Id__wf"] = 0.216;
    data_Parameters["Ix__rdr"] = 4.946;
    data_Parameters["Ix__swa"] = 0.02;
    data_Parameters["Iy__swa"] = 0.8;
    data_Parameters["Iy__wf"] = 0.4683;
    data_Parameters["Iy__wr"] = 0.664;
    data_Parameters["Iz__rdr"] = 3.304;
    data_Parameters["Iz__swa"] = 0.8;
    data_Parameters["L__b"] = 0.73;
    data_Parameters["L__swa"] = 0.535;
    data_Parameters["M__tot"] = 269.85;
    data_Parameters["XG"] = 0.22776564612299;
    data_Parameters["ZG"] = 0.198334243175045;
    data_Parameters["a__1"] = 0.13526;
    data_Parameters["braking"] = braking;
    data_Parameters["c__fs"] = 1679.7;
    data_Parameters["c__rs"] = 12000;
    data_Parameters["epsilon"] = 0.428;
    data_Parameters["eta__00"] = 0.029581668127468;
    data_Parameters["g"] = 9.807;
    data_Parameters["h__rdr"] = 0.2;
    data_Parameters["k__fs"] = 19620;
    data_Parameters["k__rs"] = 147360;
    data_Parameters["m__rdr"] = 68.5;
    data_Parameters["m__swa"] = 10;
    data_Parameters["m__wf"] = 12;
    data_Parameters["m__wr"] = 16.2;
    data_Parameters["n__err"] = 1;
    data_Parameters["r__crw"] = 0.1;
    data_Parameters["rf"] = 0.292;
    data_Parameters["rr"] = 0.317;
    data_Parameters["rtf"] = 0.0624;
    data_Parameters["rtr"] = 0.097;
    data_Parameters["s__fs"] = 0.49;
    data_Parameters["w__ss"] = w__ss;
    data_Parameters["w__t"] = w__t;
    data_Parameters["x__a"] = 0;
    data_Parameters["x__off"] = 0.034;
    data_Parameters["x__rdr"] = 0.313;
    data_Parameters["xi__n"] = 1;
    data_Parameters["z__a"] = 0.8776;
    data_Parameters["z__rdr"] = 0.504;
    data_Parameters["C__delta"] = 10;
    data_Parameters["Cxz__delta"] = 0;
    data_Parameters["Cxz__swa"] = 0;
    data_Parameters["Ix__delta"] = 0.287;
    data_Parameters["Iy__delta"] = 0.143;
    data_Parameters["Iz__delta"] = 0.2063;
    data_Parameters["Mb__norm"] = 10;
    data_Parameters["Mbf__max"] = 800;
    data_Parameters["Mbr__max"] = 800;
    data_Parameters["closed_gas"] = closed_gas;
    data_Parameters["delta__dot__err"] = 1;
    data_Parameters["delta__dot__ss"] = 0;
    data_Parameters["eta__dot__err"] = 1;
    data_Parameters["eta__dot__ss"] = 0;
    data_Parameters["h__dot__err"] = 1;
    data_Parameters["h__dot__ss"] = 0;
    data_Parameters["m__delta"] = 8.75;
    data_Parameters["not_braking"] = not_braking;
    data_Parameters["omega__n"] = 10;
    data_Parameters["open_gas"] = open_gas;
    data_Parameters["phi__dot__err"] = 1;
    data_Parameters["phi__dot__ss"] = 0;
    data_Parameters["phi__f__dot__ss"] = 0;
    data_Parameters["s__f__00"] = 0.0568145809445757;
    data_Parameters["s__f__dot__err"] = 1;
    data_Parameters["s__f__dot__ss"] = 0;
    data_Parameters["stering_norm"] = stering_norm;
    data_Parameters["tau__m__f"] = 0.1;
    data_Parameters["tau__m__r"] = 0.1;
    data_Parameters["tau__m__s"] = 0.1;
    data_Parameters["tau__m__t"] = 0.1;
    data_Parameters["tau__max"] = 50;
    data_Parameters["theta__d__00"] = 0;
    data_Parameters["theta__dot__err"] = 1;
    data_Parameters["theta__dot__ss"] = 0;
    data_Parameters["x__Swing"] = 0.275;
    data_Parameters["x__delta"] = 0.023;
    data_Parameters["x__f__dot__err"] = 1;
    data_Parameters["x__f__dot__ss"] = 0;
    data_Parameters["x__r__dot__err"] = 1;
    data_Parameters["x__r__dot__ss"] = 0;
    data_Parameters["y__f__dot__err"] = 1;
    data_Parameters["y__f__dot__ss"] = 0;
    data_Parameters["y__r__dot__err"] = 1;
    data_Parameters["y__r__dot__ss"] = 0;
    data_Parameters["z__Swing"] = 0.052;
    data_Parameters["z__delta"] = -0.098;
    data_Parameters["z__f__dot__err"] = 1;
    data_Parameters["z__f__dot__ss"] = 0;
    data_Parameters["z__r__dot__err"] = 1;
    data_Parameters["z__r__dot__ss"] = 0;
    data_Parameters["delta__f__dot__err"] = 1;
    data_Parameters["delta__f__dot__ss"] = 0;
    data_Parameters["phi__f__dot__err"] = 1;

    // Guess Parameters
    data_Parameters[Ftr__ss] = 380.399999969783;
    data_Parameters[Mbf__ss] = 0;
    data_Parameters[Mbr__ss] = 0;
    data_Parameters[eta__ss] = 0.0307542604438726;
    data_Parameters[h__ss] = 0.45769645033686;
    data_Parameters[n__ss] = 0;
    data_Parameters[phi__ss] = 0;
    data_Parameters[tau__ss] = -7.00288564829757e-11;
    data_Parameters[u__ss] = 20;
    data_Parameters[v__ss] = -2.16104835964975e-10;
    data_Parameters[xi__ss] = 0;
    data_Parameters[Omega__ss] = -5.24305880064494e-14;
    data_Parameters[delta__f__ss] = -1.06378620631546e-11;
    data_Parameters[delta__ss] = -1.14755170317492e-11;
    data_Parameters[omega__f__ss] = 68.4931507012337;
    data_Parameters[omega__r__ss] = 63.2921421213881;
    data_Parameters[phi__f__ss] = 3.59906220911141e-12;
    data_Parameters[s__f__ss] = 0.0508979277750632;
    data_Parameters[theta__ss] = -0.109051230119918;
    data_Parameters[x__f__ss] = 0.86315668743341;
    data_Parameters[x__r__ss] = 0.512935849630048;
    data_Parameters[y__f__ss] = -1.20129903987853e-13;
    data_Parameters[y__r__ss] = 1.66702482026682e-13;
    data_Parameters[z__f__ss] = -0.011693410617639;
    data_Parameters[z__r__ss] = -0.0113621376018157;

    // Boundary Conditions
    data_Parameters["h__err"] = 1;
    data_Parameters["u__err"] = 1;
    data_Parameters["v__err"] = 1;
    data_Parameters["w__LR"] = w__LR;
    data_Parameters["xi__err"] = 1;
    data_Parameters["Ftr__err"] = 1;
    data_Parameters["Mbf__err"] = 1;
    data_Parameters["Mbr__err"] = 1;
    data_Parameters["Omega__err"] = 1;
    data_Parameters["delta__err"] = 1;
    data_Parameters["delta__f__err"] = 1;
    data_Parameters["eta__err"] = 1;
    data_Parameters["omega__f__err"] = 1;
    data_Parameters["omega__r__err"] = 1;
    data_Parameters["phi__err"] = 1;
    data_Parameters["phi__f__err"] = 1;
    data_Parameters["s__f__err"] = 1;
    data_Parameters["tau__err"] = 1;
    data_Parameters["theta__err"] = 1;
    data_Parameters["x__f__err"] = 1;
    data_Parameters["x__r__err"] = 1;
    data_Parameters["y__f__err"] = 1;
    data_Parameters["y__r__err"] = 1;
    data_Parameters["z__f__err"] = 1;
    data_Parameters["z__r__err"] = 1;

    // Post Processing Parameters

    // User Function Parameters
    data_Parameters["Cp__f"] = 70;
    data_Parameters["Cp__r"] = 100;
    data_Parameters["Fz0"] = 1600;
    data_Parameters["Fz0__f"] = 1700;
    data_Parameters["Fz0__r"] = 2000;
    data_Parameters["Kp__f"] = 100000;
    data_Parameters["Kp__r"] = 130000;
    data_Parameters["d1__f"] = 14;
    data_Parameters["d1__r"] = 13;
    data_Parameters["d2__f"] = 9;
    data_Parameters["d2__r"] = 4;
    data_Parameters["d3__f"] = 0.8;
    data_Parameters["d3__r"] = 0.8;
    data_Parameters["d4__f"] = 1.2;
    data_Parameters["d4__r"] = 1.2;
    data_Parameters["d5__f"] = 0.15;
    data_Parameters["d5__r"] = 0.4;
    data_Parameters["d6__f"] = 0.1;
    data_Parameters["d6__r"] = 0.1;
    data_Parameters["d7__f"] = 0.15;
    data_Parameters["d7__r"] = 0.15;
    data_Parameters["d8"] = 1.6;
    data_Parameters["e10"] = 1;
    data_Parameters["e1__f"] = 0.4;
    data_Parameters["e1__r"] = 0.4;
    data_Parameters["e2__f"] = 0.04;
    data_Parameters["e2__r"] = 0.07;
    data_Parameters["e4"] = 10;
    data_Parameters["e5"] = 2;
    data_Parameters["e6"] = 1.5;
    data_Parameters["e7"] = 50;
    data_Parameters["e9"] = 20;
    data_Parameters["pCx1__f"] = 1.6064;
    data_Parameters["pCx1__r"] = 1.6064;
    data_Parameters["pDx1__f"] = 1.2017;
    data_Parameters["pDx1__r"] = 1;
    data_Parameters["pDx2__f"] = -0.0922;
    data_Parameters["pDx2__r"] = 0;
    data_Parameters["pKx1__f"] = 25.94;
    data_Parameters["pKx1__r"] = 25.94;
    data_Parameters["pKx2__f"] = -4.233;
    data_Parameters["pKx2__r"] = -4.233;
    data_Parameters["pKx3__f"] = 0.3369;
    data_Parameters["pKx3__r"] = 0.3369;
    data_Parameters["rBx1__f"] = 10.27;
    data_Parameters["rBx1__r"] = 16.95;
    data_Parameters["rBx2__f"] = 7.72;
    data_Parameters["rBx2__r"] = 13.34;
    data_Parameters["rBx3__f"] = -0.44;
    data_Parameters["rBx3__r"] = 0.24;
    data_Parameters["rBy1__f"] = 4.67;
    data_Parameters["rBy1__r"] = 10.5;
    data_Parameters["rBy2__f"] = 7.18;
    data_Parameters["rBy2__r"] = 11.4;
    data_Parameters["rBy3__f"] = 0;
    data_Parameters["rBy3__r"] = 0;
    data_Parameters["rCx1__f"] = 1.26;
    data_Parameters["rCx1__r"] = 1.12;
    data_Parameters["rCy1__f"] = 1;
    data_Parameters["rCy1__r"] = 1;
    data_Parameters["rHx1__f"] = 0.016;
    data_Parameters["rHx1__r"] = 0.011;
    data_Parameters["rHy1__f"] = 0;
    data_Parameters["rHy1__r"] = 0;
    data_Parameters["rHy2__f"] = 0;
    data_Parameters["rHy2__r"] = 0;
    data_Parameters["r__pin"] = 0.05;
    data_Parameters["epsilon__x__f"] = 0;
    data_Parameters["epsilon__x__r"] = 0;
    data_Parameters["lambda__C__x__f"] = 1;
    data_Parameters["lambda__C__x__r"] = 1;
    data_Parameters["lambda__K__x__f"] = 1;
    data_Parameters["lambda__K__x__r"] = 1;
    data_Parameters["lambda__mu__x__f"] = 1;
    data_Parameters["lambda__mu__x__r"] = 1;

    // Continuation Parameters
    data_Parameters["e__b0"] = e__b0;
    data_Parameters["e__b1"] = 0.01;
    data_Parameters["eps_c0"] = eps_c0;
    data_Parameters["eps_c1"] = 0.01;
    data_Parameters["eps_p0"] = eps_p0;
    data_Parameters["eps_p1"] = 0.01;
    data_Parameters["p__b0"] = p__b0;
    data_Parameters["p__b1"] = 0.01;
    data_Parameters["tol_c0"] = tol_c0;
    data_Parameters["tol_c1"] = 0.01;
    data_Parameters["tol_p0"] = tol_p0;
    data_Parameters["tol_p1"] = 0.01;
    data_Parameters["w__LR0"] = w__LR0;
    data_Parameters["w__LR1"] = 1;
    data_Parameters["w__ic0"] = w__ic0;
    data_Parameters["w__ic1"] = 0;
    data_Parameters["w__ss0"] = w__ss0;
    data_Parameters["w__ss1"] = 0;
    data_Parameters["w__t0"] = w__t0;
    data_Parameters["w__t1"] = 1;

    // Constraints Parameters

    // functions mapped on objects

    // Controls
    // Penalty type controls: 'QUADRATIC', 'QUADRATIC2', 'PARABOLA', 'CUBIC'
    // Barrier type controls: 'LOGARITHMIC', 'COS_LOGARITHMIC', 'TAN2', 'HYPERBOLIC'

    GenericContainer & data_Controls = gc_data["Controls"];
    GenericContainer & data_t__oControl = data_Controls["t__oControl"];
    data_t__oControl["type"]      = "COS_LOGARITHMIC";
    data_t__oControl["epsilon"]   = eps_c;
    data_t__oControl["tolerance"] = tol_c;


    GenericContainer & data_b__f__oControl = data_Controls["b__f__oControl"];
    data_b__f__oControl["type"]      = "COS_LOGARITHMIC";
    data_b__f__oControl["epsilon"]   = eps_c;
    data_b__f__oControl["tolerance"] = tol_c;


    GenericContainer & data_b__r__oControl = data_Controls["b__r__oControl"];
    data_b__r__oControl["type"]      = "COS_LOGARITHMIC";
    data_b__r__oControl["epsilon"]   = eps_c;
    data_b__r__oControl["tolerance"] = tol_c;


    GenericContainer & data_tau__oControl = data_Controls["tau__oControl"];
    data_tau__oControl["type"]      = "COS_LOGARITHMIC";
    data_tau__oControl["epsilon"]   = eps_c;
    data_tau__oControl["tolerance"] = tol_c;



    // Constraint1D
    // Penalty subtype: 'PENALTY_REGULAR', 'PENALTY_SMOOTH', 'PENALTY_PIECEWISE'
    // Barrier subtype: 'BARRIER_LOG', 'BARRIER_LOG_EXP', 'BARRIER_LOG0'
    GenericContainer & data_Constraints = gc_data["Constraints"];
    // PenaltyBarrier1DGreaterThan
    GenericContainer & data_OnlyBrakingFront = data_Constraints["OnlyBrakingFront"];
    data_OnlyBrakingFront["subType"]   = 'PENALTY_REGULAR';
    data_OnlyBrakingFront["epsilon"]   = e__b;
    data_OnlyBrakingFront["tolerance"] = p__b;
    data_OnlyBrakingFront["active"]    = true;
    // PenaltyBarrier1DGreaterThan
    GenericContainer & data_OnlyBrakingRear = data_Constraints["OnlyBrakingRear"];
    data_OnlyBrakingRear["subType"]   = 'PENALTY_REGULAR';
    data_OnlyBrakingRear["epsilon"]   = e__b;
    data_OnlyBrakingRear["tolerance"] = p__b;
    data_OnlyBrakingRear["active"]    = true;
    // PenaltyBarrier1DGreaterThan
    GenericContainer & data_OnlyTractionRear = data_Constraints["OnlyTractionRear"];
    data_OnlyTractionRear["subType"]   = 'PENALTY_REGULAR';
    data_OnlyTractionRear["epsilon"]   = e__b;
    data_OnlyTractionRear["tolerance"] = p__b;
    data_OnlyTractionRear["active"]    = true;
    // PenaltyBarrier1DGreaterThan
    GenericContainer & data_FrontWheelContact = data_Constraints["FrontWheelContact"];
    data_FrontWheelContact["subType"]   = 'PENALTY_REGULAR';
    data_FrontWheelContact["epsilon"]   = eps_p;
    data_FrontWheelContact["tolerance"] = tol_p;
    data_FrontWheelContact["active"]    = true;
    // PenaltyBarrier1DGreaterThan
    GenericContainer & data_RearWheelContact = data_Constraints["RearWheelContact"];
    data_RearWheelContact["subType"]   = 'PENALTY_REGULAR';
    data_RearWheelContact["epsilon"]   = eps_p;
    data_RearWheelContact["tolerance"] = tol_p;
    data_RearWheelContact["active"]    = true;
    // PenaltyBarrier1DInterval
    GenericContainer & data_LongSlipFront = data_Constraints["LongSlipFront"];
    data_LongSlipFront["subType"]   = "PENALTY_REGULAR";
    data_LongSlipFront["epsilon"]   = eps_p;
    data_LongSlipFront["tolerance"] = tol_p;
    data_LongSlipFront["min"]       = -long_peak_front;
    data_LongSlipFront["max"]       = long_peak_front;
    data_LongSlipFront["active"]    = true;
    // PenaltyBarrier1DInterval
    GenericContainer & data_LongSlipRear = data_Constraints["LongSlipRear"];
    data_LongSlipRear["subType"]   = "PENALTY_REGULAR";
    data_LongSlipRear["epsilon"]   = eps_p;
    data_LongSlipRear["tolerance"] = tol_p;
    data_LongSlipRear["min"]       = -long_peak_rear;
    data_LongSlipRear["max"]       = long_peak_rear;
    data_LongSlipRear["active"]    = true;
    // PenaltyBarrier1DInterval
    GenericContainer & data_LatSlipFront = data_Constraints["LatSlipFront"];
    data_LatSlipFront["subType"]   = "PENALTY_REGULAR";
    data_LatSlipFront["epsilon"]   = eps_p;
    data_LatSlipFront["tolerance"] = tol_p;
    data_LatSlipFront["min"]       = -lat_peak_front;
    data_LatSlipFront["max"]       = lat_peak_front;
    data_LatSlipFront["active"]    = true;
    // PenaltyBarrier1DInterval
    GenericContainer & data_LatSlipRear = data_Constraints["LatSlipRear"];
    data_LatSlipRear["subType"]   = "PENALTY_REGULAR";
    data_LatSlipRear["epsilon"]   = eps_p;
    data_LatSlipRear["tolerance"] = tol_p;
    data_LatSlipRear["min"]       = -lat_peak_rear;
    data_LatSlipRear["max"]       = lat_peak_rear;
    data_LatSlipRear["active"]    = true;
    // PenaltyBarrier1DInterval
    GenericContainer & data_MaxSteerAngle = data_Constraints["MaxSteerAngle"];
    data_MaxSteerAngle["subType"]   = "PENALTY_REGULAR";
    data_MaxSteerAngle["epsilon"]   = eps_p;
    data_MaxSteerAngle["tolerance"] = tol_p;
    data_MaxSteerAngle["min"]       = -delta__max;
    data_MaxSteerAngle["max"]       = delta__max;
    data_MaxSteerAngle["active"]    = true;
    // PenaltyBarrier1DInterval
    GenericContainer & data_MaxRollAngle = data_Constraints["MaxRollAngle"];
    data_MaxRollAngle["subType"]   = "PENALTY_REGULAR";
    data_MaxRollAngle["epsilon"]   = eps_p;
    data_MaxRollAngle["tolerance"] = tol_p;
    data_MaxRollAngle["min"]       = -phi__max;
    data_MaxRollAngle["max"]       = phi__max;
    data_MaxRollAngle["active"]    = true;
    // PenaltyBarrier1DGreaterThan
    GenericContainer & data_roadRightLateralBorder = data_Constraints["roadRightLateralBorder"];
    data_roadRightLateralBorder["subType"]   = 'PENALTY_REGULAR';
    data_roadRightLateralBorder["epsilon"]   = eps_p;
    data_roadRightLateralBorder["tolerance"] = tol_p;
    data_roadRightLateralBorder["active"]    = true;
    // PenaltyBarrier1DGreaterThan
    GenericContainer & data_roadLeftLateralBorder = data_Constraints["roadLeftLateralBorder"];
    data_roadLeftLateralBorder["subType"]   = 'PENALTY_REGULAR';
    data_roadLeftLateralBorder["epsilon"]   = eps_p;
    data_roadLeftLateralBorder["tolerance"] = tol_p;
    data_roadLeftLateralBorder["active"]    = true;
    // Constraint2D: none defined

    // User defined classes initialization
    // User defined classes: E N G I N E
MidLine_data.Engine["Gear_ratio"][0] = 4.82;
MidLine_data.Engine["Gear_ratio"][1] = 2.5;
MidLine_data.Engine["Gear_ratio"][2] = 1.75;
MidLine_data.Engine["Gear_ratio"][3] = 1.368;
MidLine_data.Engine["Gear_ratio"][4] = 1.09;
MidLine_data.Engine["Gear_ratio"][5] = 0.956;
MidLine_data.Engine["Gear_ratio"][6] = 0.851;
MidLine_data.Engine["Rpm"][0] = 0;
MidLine_data.Engine["Rpm"][1] = 3000;
MidLine_data.Engine["Rpm"][2] = 3500;
MidLine_data.Engine["Rpm"][3] = 4000;
MidLine_data.Engine["Rpm"][4] = 4500;
MidLine_data.Engine["Rpm"][5] = 5000;
MidLine_data.Engine["Rpm"][6] = 5500;
MidLine_data.Engine["Rpm"][7] = 6000;
MidLine_data.Engine["Rpm"][8] = 6500;
MidLine_data.Engine["Rpm"][9] = 7000;
MidLine_data.Engine["Rpm"][10] = 7500;
MidLine_data.Engine["Rpm"][11] = 8000;
MidLine_data.Engine["Rpm"][12] = 8500;
MidLine_data.Engine["Rpm"][13] = 9000;
MidLine_data.Engine["Rpm"][14] = 9500;
MidLine_data.Engine["Rpm"][15] = 10000;
MidLine_data.Engine["Rpm"][16] = 10500;
MidLine_data.Engine["Rpm"][17] = 11000;
MidLine_data.Engine["Rpm"][18] = 11500;
MidLine_data.Engine["Rpm"][19] = 12000;
MidLine_data.Engine["Rpm"][20] = 12500;
MidLine_data.Engine["Rpm"][21] = 13000;
MidLine_data.Engine["Torque"][0] = 2;
MidLine_data.Engine["Torque"][1] = 47.088;
MidLine_data.Engine["Torque"][2] = 62.784;
MidLine_data.Engine["Torque"][3] = 70.632;
MidLine_data.Engine["Torque"][4] = 78.48;
MidLine_data.Engine["Torque"][5] = 84.366;
MidLine_data.Engine["Torque"][6] = 85.347;
MidLine_data.Engine["Torque"][7] = 83.385;
MidLine_data.Engine["Torque"][8] = 81.423;
MidLine_data.Engine["Torque"][9] = 90.252;
MidLine_data.Engine["Torque"][10] = 92.214;
MidLine_data.Engine["Torque"][11] = 87.309;
MidLine_data.Engine["Torque"][12] = 85.347;
MidLine_data.Engine["Torque"][13] = 84.366;
MidLine_data.Engine["Torque"][14] = 80.442;
MidLine_data.Engine["Torque"][15] = 79.461;
MidLine_data.Engine["Torque"][16] = 73.575;
MidLine_data.Engine["Torque"][17] = 63.765;
MidLine_data.Engine["Torque"][18] = 49.05;
MidLine_data.Engine["Torque"][19] = 29.43;
MidLine_data.Engine["Torque"][20] = 19.62;
MidLine_data.Engine["Torque"][21] = 2;
    // User defined classes: R O A D


    // alias for user object classes passed as pointers
    GenericContainer & ptrs = gc_data["Pointers"];
    // setup user object classes

    LW_ASSERT0(
      gc_data.exists("Engine"),
      "missing key: ``Engine'' in gc_data\n"
    );
    engine.setup(gc_data("Engine"));
    ptrs[ "pEngine" ] = &engine;

    LW_ASSERT0(
      gc_data.exists("Road"),
      "missing key: ``Road'' in gc_data\n"
    );
    road.setup(gc_data("Road"));
    ptrs[ "pRoad" ] = &road;

    // setup model
    model.setup( gc_data );

    // initialize nonlinear system initial point
    model.guess( gc_data("Guess","Missing `Guess` field") );

    // solve nonlinear system
    bool ok = model.solve(); // no spline

    // get solution (even if not converged)
    model.get_solution( gc_solution );
    model.diagnostic( gc_data );

    ofstream file;
    if ( ok ) {
      file.open( "data/MidLine_OCP_result.txt" );
    } else {
      cout << gc_solution("solver_message").get_string() << '\n';
      // dump solution to file
      file.open( "data/MidLine_OCP_not_converged.txt" );
    }
    file.precision(18);
    Mechatronix::saveOCPsolutionToStream(gc_solution,file);
    file.close();
    cout.precision(18);
    GenericContainer const & target = gc_solution("target");
    fmt::print(
      "Lagrange target    = {}\n"
      "Mayer target       = {}\n"
      "Penalties+Barriers = {}\n"
      "Control penalties  = {}\n",
      target("lagrange").get_number(),  target("mayer").get_number(),
      target("penalties").get_number(), target("control_penalties").get_number()
    );
    if ( gc_solution.exists("parameters") ) {
      cout << "Parameters:\n";
      gc_solution("parameters").print(cout);
    }
    if ( gc_solution.exists("diagnosis") ) gc_solution("diagnosis").print(cout);
  }
  catch ( exception const & exc ) {
    console.error(exc.what());
    ALL_DONE_FOLKS;
    exit(0);
  }
  catch ( char const exc[] ) {
    console.error(exc);
    ALL_DONE_FOLKS;
    exit(0);
  }
  catch (...) {
    console.error("ERRORE SCONOSCIUTO\n");
    ALL_DONE_FOLKS;
    exit(0);
  }

  ALL_DONE_FOLKS;

  #ifdef MECHATRONIX_OS_WINDOWS
  } __finally {
    cerr << "Unknown windows error found, exiting\n";
  }
  #endif

  return 0;
}

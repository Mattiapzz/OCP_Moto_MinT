/*-----------------------------------------------------------------------*\
 |  file: Test_Convergence.cc                                            |
 |                                                                       |
 |  version: 1.0   date 30/6/2020                                        |
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


// use pragma to include libraries
#include <MechatronixCore/MechatronixLibs.hh>

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif
#ifdef __clang__
#pragma clang diagnostic ignored "-Wunused-parameter"
#endif

#include "Test_Convergence.hh"
#include "Test_Convergence_Pars.hh"

#include <time.h> /* time_t, struct tm, time, localtime, asctime */

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif
#ifdef __clang__
#pragma clang diagnostic ignored "-Wunused-parameter"
#endif

namespace Test_ConvergenceDefine {

  using GenericContainerNamespace::vec_string_type;

  /*
  //   _ __   __ _ _ __ ___   ___  ___
  //  | '_ \ / _` | '_ ` _ \ / _ \/ __|
  //  | | | | (_| | | | | | |  __/\__ \
  //  |_| |_|\__,_|_| |_| |_|\___||___/
  //
  */

  char const *namesXvars[numXvars+1] = {
    "u",
    "v",
    "Omega",
    "phi",
    "theta",
    "h",
    "delta",
    "eta",
    "s__f",
    "x__f",
    "y__f",
    "z__f",
    "x__r",
    "y__r",
    "z__r",
    "delta__f",
    "phi__f",
    "omega__r",
    "omega__f",
    "phi__dot",
    "theta__dot",
    "h__dot",
    "delta__dot",
    "eta__dot",
    "s__f__dot",
    "x__f__dot",
    "y__f__dot",
    "z__f__dot",
    "x__r__dot",
    "y__r__dot",
    "z__r__dot",
    "delta__f__dot",
    "phi__f__dot",
    "Ftr",
    "Mbf",
    "Mbr",
    "tau",
    "n",
    "xi",
    nullptr
  };

  char const *namesLvars[numLvars+1] = {
    "lambda1__xo",
    "lambda2__xo",
    "lambda3__xo",
    "lambda4__xo",
    "lambda5__xo",
    "lambda6__xo",
    "lambda7__xo",
    "lambda8__xo",
    "lambda9__xo",
    "lambda10__xo",
    "lambda11__xo",
    "lambda12__xo",
    "lambda13__xo",
    "lambda14__xo",
    "lambda15__xo",
    "lambda16__xo",
    "lambda17__xo",
    "lambda18__xo",
    "lambda19__xo",
    "lambda20__xo",
    "lambda21__xo",
    "lambda22__xo",
    "lambda23__xo",
    "lambda24__xo",
    "lambda25__xo",
    "lambda26__xo",
    "lambda27__xo",
    "lambda28__xo",
    "lambda29__xo",
    "lambda30__xo",
    "lambda31__xo",
    "lambda32__xo",
    "lambda33__xo",
    "lambda34__xo",
    "lambda35__xo",
    "lambda36__xo",
    "lambda37__xo",
    "lambda38__xo",
    "lambda39__xo",
    nullptr
  };

  char const *namesUvars[numUvars+1] = {
    "b__f__o",
    "b__r__o",
    "t__o",
    "tau__o",
    nullptr
  };

  char const *namesQvars[numQvars+1] = {
    "zeta",
    "kappa",
    "leftWidth",
    "rightWidth",
    "sectionSpeedLimit",
    "adherence",
    "xISOMidLane",
    "yISOMidLane",
    "xISOleft",
    "yISOleft",
    "xISOright",
    "yISOright",
    "ISOAngle",
    nullptr
  };

  char const *namesPvars[numPvars+1] = {
    nullptr
  };

  char const *namesOMEGAvars[numOMEGAvars+1] = {
    nullptr
  };

  char const *namesPostProcess[numPostProcess+1] = {
    "t__oControl",
    "b__f__oControl",
    "b__r__oControl",
    "tau__oControl",
    "OnlyBrakingFront",
    "OnlyBrakingRear",
    "OnlyTractionRear",
    "FrontWheelContact",
    "RearWheelContact",
    "LongSlipFront",
    "LongSlipRear",
    "LatSlipFront",
    "LatSlipRear",
    "MaxSteerAngle",
    "MaxRollAngle",
    "roadRightLateralBorder",
    "roadLeftLateralBorder",
    "Fzf",
    "Fzr",
    "lambda__r",
    "lambda__f",
    "alpha__r",
    "alpha__f",
    "Fxf",
    "Fxr",
    "Fyf",
    "Fyr",
    "Mzf",
    "Mzr",
    "Mxf",
    "Mxr",
    "max_torque_at_wheel",
    "xISORight",
    "yISORight",
    "xISOLeft",
    "yISOLeft",
    "xISOTrajectory",
    "yISOTrajectory",
    "ALG__11",
    "ALG__12",
    "ALG__13",
    "ALG__14",
    "ALG__15",
    "ALG__16",
    "ALG__17",
    "ALG__18",
    "ALG__21",
    "ALG__22",
    "ALG__23",
    "ALG__24",
    "ALG__25",
    "ALG__26",
    "ALG__27",
    "ALG__28",
    nullptr
  };

  char const *namesIntegratedPostProcess[numIntegratedPostProcess+1] = {
    "time",
    nullptr
  };

  char const *namesModelPars[numModelPars+1] = {
    "CXZ",
    "Ca",
    "Cp__f",
    "Cp__r",
    "Ftr__ss",
    "Fz0",
    "Fz0__f",
    "Fz0__r",
    "Fzmin",
    "Fznorm",
    "IX",
    "IY",
    "IZ",
    "Id__wf",
    "Ix__rdr",
    "Ix__swa",
    "Iy__swa",
    "Iy__wf",
    "Iy__wr",
    "Iz__rdr",
    "Iz__swa",
    "Kp__f",
    "Kp__r",
    "L__b",
    "L__swa",
    "M__tot",
    "Mbf__ss",
    "Mbr__ss",
    "XG",
    "ZG",
    "a__1",
    "braking",
    "c__fs",
    "c__rs",
    "d1__f",
    "d1__r",
    "d2__f",
    "d2__r",
    "d3__f",
    "d3__r",
    "d4__f",
    "d4__r",
    "d5__f",
    "d5__r",
    "d6__f",
    "d6__r",
    "d7__f",
    "d7__r",
    "d8",
    "e10",
    "e1__f",
    "e1__r",
    "e2__f",
    "e2__r",
    "e4",
    "e5",
    "e6",
    "e7",
    "e9",
    "e__b0",
    "e__b1",
    "eps_c0",
    "eps_c1",
    "eps_p0",
    "eps_p1",
    "epsilon",
    "eta__00",
    "eta__ss",
    "g",
    "h__rdr",
    "h__ss",
    "h_c",
    "h_c0",
    "h_c1",
    "k__fs",
    "k__rs",
    "m__rdr",
    "m__swa",
    "m__wf",
    "m__wr",
    "n__ss",
    "pCx1__f",
    "pCx1__r",
    "pDx1__f",
    "pDx1__r",
    "pDx2__f",
    "pDx2__r",
    "pKx1__f",
    "pKx1__r",
    "pKx2__f",
    "pKx2__r",
    "pKx3__f",
    "pKx3__r",
    "p__b0",
    "p__b1",
    "phi__ss",
    "rBx1__f",
    "rBx1__r",
    "rBx2__f",
    "rBx2__r",
    "rBx3__f",
    "rBx3__r",
    "rBy1__f",
    "rBy1__r",
    "rBy2__f",
    "rBy2__r",
    "rBy3__f",
    "rBy3__r",
    "rCx1__f",
    "rCx1__r",
    "rCy1__f",
    "rCy1__r",
    "rHx1__f",
    "rHx1__r",
    "rHy1__f",
    "rHy1__r",
    "rHy2__f",
    "rHy2__r",
    "r__crw",
    "r__pin",
    "rf",
    "rr",
    "rtf",
    "rtr",
    "s__fs",
    "tau__ss",
    "tol_c0",
    "tol_c1",
    "tol_p0",
    "tol_p1",
    "u__ss",
    "v__ss",
    "w__LR",
    "w__ic",
    "w__ic0",
    "w__ic1",
    "w__n0",
    "w__n00",
    "w__n01",
    "w__ss",
    "w__ss0",
    "w__ss1",
    "w__t",
    "w__t0",
    "w__t1",
    "x__a",
    "x__off",
    "x__rdr",
    "xi__n",
    "xi__ss",
    "z__a",
    "z__rdr",
    "C__delta",
    "Cxz__delta",
    "Cxz__swa",
    "Ix__delta",
    "Iy__delta",
    "Iz__delta",
    "Mb__norm",
    "Mbf__max",
    "Mbr__max",
    "Omega__ss",
    "closed_gas",
    "delta__dot__ss",
    "delta__f__ss",
    "delta__max",
    "delta__ss",
    "epsilon__x__f",
    "epsilon__x__r",
    "eta__dot__ss",
    "h__dot__ss",
    "lambda__C__x__f",
    "lambda__C__x__r",
    "lambda__K__x__f",
    "lambda__K__x__r",
    "lat_peak_front",
    "lat_peak_rear",
    "long_peak_front",
    "long_peak_rear",
    "m__delta",
    "not_braking",
    "omega__f__ss",
    "omega__n",
    "omega__r__ss",
    "open_gas",
    "phi__dot__ss",
    "phi__f__dot__ss",
    "phi__f__ss",
    "phi__max",
    "s__f__00",
    "s__f__dot__ss",
    "s__f__ss",
    "stering_norm",
    "tau__m__f",
    "tau__m__r",
    "tau__m__s",
    "tau__m__t",
    "tau__max",
    "theta__d__00",
    "theta__dot__ss",
    "theta__ss",
    "w__Ftr__bc",
    "w__Mbf__bc",
    "w__Mbr__bc",
    "w__Omega__bc",
    "w__delta__bc",
    "w__delta__f__bc",
    "w__eta__bc",
    "w__eta__dot__bc",
    "w__eta__sserr",
    "w__h__bc",
    "w__h__dot__bc",
    "w__h__sserr",
    "w__n__bc",
    "w__omega__f__bc",
    "w__omega__r__bc",
    "w__phi__bc",
    "w__phi__dot__bc",
    "w__phi__f__bc",
    "w__s__f__bc",
    "w__s__f__sserr",
    "w__tau__bc",
    "w__theta__bc",
    "w__theta__sserr",
    "w__u__bc",
    "w__v__bc",
    "w__x__f__bc",
    "w__x__r__bc",
    "w__xi__bc",
    "w__y__f__bc",
    "w__y__r__bc",
    "w__z__f__bc",
    "w__z__r__bc",
    "x__Swing",
    "x__delta",
    "x__f__dot__ss",
    "x__f__ss",
    "x__r__dot__ss",
    "x__r__ss",
    "y__f__dot__ss",
    "y__f__ss",
    "y__r__dot__ss",
    "y__r__ss",
    "z__Swing",
    "z__delta",
    "z__f__dot__ss",
    "z__f__ss",
    "z__r__dot__ss",
    "z__r__ss",
    "delta__f__dot__ss",
    "lambda__mu__x__f",
    "lambda__mu__x__r",
    "w__delta__dot__bc",
    "w__delta__dot__sserr",
    "w__delta__f__dot__bc",
    "w__delta__f__dot__sserr",
    "w__eta__dot__sserr",
    "w__h__dot__sserr",
    "w__phi__dot__sserr",
    "w__phi__f__dot__bc",
    "w__phi__f__dot__sserr",
    "w__s__f__dot__bc",
    "w__s__f__dot__sserr",
    "w__theta__dot__bc",
    "w__theta__dot__sserr",
    "w__x__f__dot__bc",
    "w__x__f__dot__sserr",
    "w__x__r__dot__bc",
    "w__x__r__dot__sserr",
    "w__y__f__dot__bc",
    "w__y__f__dot__sserr",
    "w__y__r__dot__bc",
    "w__y__r__dot__sserr",
    "w__z__f__dot__bc",
    "w__z__f__dot__sserr",
    "w__z__r__dot__bc",
    "w__z__r__dot__sserr",
    nullptr
  };

  char const *namesConstraint1D[numConstraint1D+1] = {
    "OnlyBrakingFront",
    "OnlyBrakingRear",
    "OnlyTractionRear",
    "FrontWheelContact",
    "RearWheelContact",
    "LongSlipFront",
    "LongSlipRear",
    "LatSlipFront",
    "LatSlipRear",
    "MaxSteerAngle",
    "MaxRollAngle",
    "roadRightLateralBorder",
    "roadLeftLateralBorder",
    nullptr
  };

  char const *namesConstraint2D[numConstraint2D+1] = {
    nullptr
  };

  char const *namesConstraintU[numConstraintU+1] = {
    "t__oControl",
    "b__f__oControl",
    "b__r__oControl",
    "tau__oControl",
    nullptr
  };

  char const *namesBc[numBc+1] = {
    nullptr
  };

  /* --------------------------------------------------------------------------
  //    ___             _               _
  //   / __|___ _ _  __| |_ _ _ _  _ __| |_ ___ _ _
  //  | (__/ _ \ ' \(_-<  _| '_| || / _|  _/ _ \ '_|
  //   \___\___/_||_/__/\__|_|  \_,_\__|\__\___/_|
  */
  Test_Convergence::Test_Convergence(
    string const & name,
    ThreadPool   * _TP,
    Console      * _pConsole
  )
  : Discretized_Indirect_OCP( name, _TP, _pConsole )
  // Controls
  , t__oControl("t__oControl")
  , b__f__oControl("b__f__oControl")
  , b__r__oControl("b__r__oControl")
  , tau__oControl("tau__oControl")
  // Constraints 1D
  , OnlyBrakingFront("OnlyBrakingFront")
  , OnlyBrakingRear("OnlyBrakingRear")
  , OnlyTractionRear("OnlyTractionRear")
  , FrontWheelContact("FrontWheelContact")
  , RearWheelContact("RearWheelContact")
  , LongSlipFront("LongSlipFront")
  , LongSlipRear("LongSlipRear")
  , LatSlipFront("LatSlipFront")
  , LatSlipRear("LatSlipRear")
  , MaxSteerAngle("MaxSteerAngle")
  , MaxRollAngle("MaxRollAngle")
  , roadRightLateralBorder("roadRightLateralBorder")
  , roadLeftLateralBorder("roadLeftLateralBorder")
  // Constraints 2D
  // User classes
  {
    this->U_solve_iterative = false;

    // continuation
    this->ns_continuation_begin = 0;
    this->ns_continuation_end   = 4;
    // Initialize to NaN all the ModelPars
    std::fill( ModelPars, ModelPars + numModelPars, alglin::NaN<real_type>() );

    // Initialize string of names
    setupNames(
      numPvars,                 namesPvars,
      numXvars,                 namesXvars,
      numLvars,                 namesLvars,
      numUvars,                 namesUvars,
      numQvars,                 namesQvars,
      numPostProcess,           namesPostProcess,
      numIntegratedPostProcess, namesIntegratedPostProcess,
      numBc,                    namesBc
    );
    //pSolver = &this->solverNewtonDumped;
    pSolver = &this->solverHyness;

    #ifdef LAPACK_WRAPPER_USE_OPENBLAS
    openblas_set_num_threads(1);
    goto_set_num_threads(1);
    #endif
  }

  Test_Convergence::~Test_Convergence() {
  }

  /* --------------------------------------------------------------------------
  //                  _       _       ____  _
  //  _   _ _ __   __| | __ _| |_ ___|  _ \| |__   __ _ ___  ___
  // | | | | '_ \ / _` |/ _` | __/ _ \ |_) | '_ \ / _` / __|/ _ \
  // | |_| | |_) | (_| | (_| | ||  __/  __/| | | | (_| \__ \  __/
  //  \__,_| .__/ \__,_|\__,_|\__\___|_|   |_| |_|\__,_|___/\___|
  //       |_|
  */
  void
  Test_Convergence::updateContinuation( integer phase, real_type s ) {
    LW_ASSERT(
      s >= 0 && s <= 1,
      "Test_Convergence::updateContinuation( phase number = {}, s = {}) "
      "s must be in the interval [0,1]\n",
      phase, s
    );
    switch ( phase ) {
      case 0: continuationStep0( s ); break;
      case 1: continuationStep1( s ); break;
      case 2: continuationStep2( s ); break;
      case 3: continuationStep3( s ); break;
      default:
        LW_ERROR(
          "Test_Convergence::updateContinuation( phase number = {}, s = {} )"
          " phase N.{} is not defined\n",
          phase, s, phase
        );
    }
  }

  /* --------------------------------------------------------------------------
  //           _               ____                                _
  //  ___  ___| |_ _   _ _ __ |  _ \ __ _ _ __ __ _ _ __ ___   ___| |_ ___ _ __ ___
  // / __|/ _ \ __| | | | '_ \| |_) / _` | '__/ _` | '_ ` _ \ / _ \ __/ _ \ '__/ __|
  // \__ \  __/ |_| |_| | |_) |  __/ (_| | | | (_| | | | | | |  __/ ||  __/ |  \__ \
  // |___/\___|\__|\__,_| .__/|_|   \__,_|_|  \__,_|_| |_| |_|\___|\__\___|_|  |___/
  //                    |_|
  // initialize parameters using associative array
  */
  void
  Test_Convergence::setupParameters( GenericContainer const & gc_data ) {
    LW_ASSERT0(
      gc_data.exists("Parameters"),
      "Test_Convergence::setupParameters: Missing key `Parameters` in data\n"
    );
    GenericContainer const & gc = gc_data("Parameters");

    bool allfound = true;
    for ( integer i = 0; i < numModelPars; ++i ) {
      char const * namei = namesModelPars[i];
      if ( gc.exists( namei ) ) {
        ModelPars[i] = gc(namei).get_number();
      } else {
        pConsole->error( fmt::format( "Missing parameter: '{}'\n", namei ) );
        allfound = false;
      }
    }
    LW_ASSERT0(
      allfound, "in Test_Convergence::setup not all parameters are set!\n"
    );
  }

  void
  Test_Convergence::setupParameters( real_type const Pars[] ) {
    std::copy( Pars, Pars + numModelPars, ModelPars );
  }

  /* --------------------------------------------------------------------------
  //            _                ____ _
  //   ___  ___| |_ _   _ _ __  / ___| | __ _ ___ ___  ___  ___
  //  / __|/ _ \ __| | | | '_ \| |   | |/ _` / __/ __|/ _ \/ __|
  //  \__ \  __/ |_| |_| | |_) | |___| | (_| \__ \__ \  __/\__ \
  //  |___/\___|\__|\__,_| .__/ \____|_|\__,_|___/___/\___||___/
  //                     |_|
  */
  void
  Test_Convergence::setupClasses( GenericContainer const & gc_data ) {
    LW_ASSERT0(
      gc_data.exists("Constraints"),
      "Test_Convergence::setupClasses: Missing key `Parameters` in data\n"
    );
    GenericContainer const & gc = gc_data("Constraints");
    // Initialize Constraints 1D
    LW_ASSERT0(
      gc.exists("OnlyBrakingFront"),
      "in Test_Convergence::setupClasses(gc) missing key: ``OnlyBrakingFront''\n"
    );
    OnlyBrakingFront.setup( gc("OnlyBrakingFront") );

    LW_ASSERT0(
      gc.exists("OnlyBrakingRear"),
      "in Test_Convergence::setupClasses(gc) missing key: ``OnlyBrakingRear''\n"
    );
    OnlyBrakingRear.setup( gc("OnlyBrakingRear") );

    LW_ASSERT0(
      gc.exists("OnlyTractionRear"),
      "in Test_Convergence::setupClasses(gc) missing key: ``OnlyTractionRear''\n"
    );
    OnlyTractionRear.setup( gc("OnlyTractionRear") );

    LW_ASSERT0(
      gc.exists("FrontWheelContact"),
      "in Test_Convergence::setupClasses(gc) missing key: ``FrontWheelContact''\n"
    );
    FrontWheelContact.setup( gc("FrontWheelContact") );

    LW_ASSERT0(
      gc.exists("RearWheelContact"),
      "in Test_Convergence::setupClasses(gc) missing key: ``RearWheelContact''\n"
    );
    RearWheelContact.setup( gc("RearWheelContact") );

    LW_ASSERT0(
      gc.exists("LongSlipFront"),
      "in Test_Convergence::setupClasses(gc) missing key: ``LongSlipFront''\n"
    );
    LongSlipFront.setup( gc("LongSlipFront") );

    LW_ASSERT0(
      gc.exists("LongSlipRear"),
      "in Test_Convergence::setupClasses(gc) missing key: ``LongSlipRear''\n"
    );
    LongSlipRear.setup( gc("LongSlipRear") );

    LW_ASSERT0(
      gc.exists("LatSlipFront"),
      "in Test_Convergence::setupClasses(gc) missing key: ``LatSlipFront''\n"
    );
    LatSlipFront.setup( gc("LatSlipFront") );

    LW_ASSERT0(
      gc.exists("LatSlipRear"),
      "in Test_Convergence::setupClasses(gc) missing key: ``LatSlipRear''\n"
    );
    LatSlipRear.setup( gc("LatSlipRear") );

    LW_ASSERT0(
      gc.exists("MaxSteerAngle"),
      "in Test_Convergence::setupClasses(gc) missing key: ``MaxSteerAngle''\n"
    );
    MaxSteerAngle.setup( gc("MaxSteerAngle") );

    LW_ASSERT0(
      gc.exists("MaxRollAngle"),
      "in Test_Convergence::setupClasses(gc) missing key: ``MaxRollAngle''\n"
    );
    MaxRollAngle.setup( gc("MaxRollAngle") );

    LW_ASSERT0(
      gc.exists("roadRightLateralBorder"),
      "in Test_Convergence::setupClasses(gc) missing key: ``roadRightLateralBorder''\n"
    );
    roadRightLateralBorder.setup( gc("roadRightLateralBorder") );

    LW_ASSERT0(
      gc.exists("roadLeftLateralBorder"),
      "in Test_Convergence::setupClasses(gc) missing key: ``roadLeftLateralBorder''\n"
    );
    roadLeftLateralBorder.setup( gc("roadLeftLateralBorder") );

  }

  /* --------------------------------------------------------------------------
  //           _               _   _                ____ _
  //  ___  ___| |_ _   _ _ __ | | | |___  ___ _ __ / ___| | __ _ ___ ___  ___  ___
  // / __|/ _ \ __| | | | '_ \| | | / __|/ _ \ '__| |   | |/ _` / __/ __|/ _ \/ __|
  // \__ \  __/ |_| |_| | |_) | |_| \__ \  __/ |  | |___| | (_| \__ \__ \  __/\__ \
  // |___/\___|\__|\__,_| .__/ \___/|___/\___|_|   \____|_|\__,_|___/___/\___||___/
  //                    |_|
  */
  void
  Test_Convergence::setupUserClasses( GenericContainer const & gc ) {
  }

  /* --------------------------------------------------------------------------
  //           _             _   _
  //   ___ ___| |_ _  _ _ __| | | |___ ___ _ _
  //  (_-</ -_)  _| || | '_ \ |_| (_-</ -_) '_|
  //  /__/\___|\__|\_,_| .__/\___//__/\___|_|
  //                   |_|
  //   __  __                        _ ___             _   _
  //  |  \/  |__ _ _ __ _ __  ___ __| | __|  _ _ _  __| |_(_)___ _ _  ___
  //  | |\/| / _` | '_ \ '_ \/ -_) _` | _| || | ' \/ _|  _| / _ \ ' \(_-<
  //  |_|  |_\__,_| .__/ .__/\___\__,_|_| \_,_|_||_\__|\__|_\___/_||_/__/
  //              |_|  |_|
  */
  void
  Test_Convergence::setupUserMappedFunctions( GenericContainer const & gc_data ) {
  }
  /* --------------------------------------------------------------------------
  //            _                ____            _             _
  //   ___  ___| |_ _   _ _ __  / ___|___  _ __ | |_ _ __ ___ | |___
  //  / __|/ _ \ __| | | | '_ \| |   / _ \| '_ \| __| '__/ _ \| / __|
  //  \__ \  __/ |_| |_| | |_) | |__| (_) | | | | |_| | | (_) | \__ \
  //  |___/\___|\__|\__,_| .__/ \____\___/|_| |_|\__|_|  \___/|_|___/
  //                     |_|
  */
  void
  Test_Convergence::setupControls( GenericContainer const & gc_data ) {
    // initialize Control penalties
    LW_ASSERT0(
      gc_data.exists("Controls"),
      "Test_Convergence::setupClasses: Missing key `Controls` in data\n"
    );
    GenericContainer const & gc = gc_data("Controls");
    t__oControl.setup( gc("t__oControl") );
    b__f__oControl.setup( gc("b__f__oControl") );
    b__r__oControl.setup( gc("b__r__oControl") );
    tau__oControl.setup( gc("tau__oControl") );
    // setup iterative solver
    this->setupControlSolver( gc_data );
  }

  /* --------------------------------------------------------------------------
  //            _               ____       _       _
  //   ___  ___| |_ _   _ _ __ |  _ \ ___ (_)_ __ | |_ ___ _ __ ___
  //  / __|/ _ \ __| | | | '_ \| |_) / _ \| | '_ \| __/ _ \ '__/ __|
  //  \__ \  __/ |_| |_| | |_) |  __/ (_) | | | | | ||  __/ |  \__ \
  //  |___/\___|\__|\__,_| .__/|_|   \___/|_|_| |_|\__\___|_|  |___/
  //                     |_|
  */
  void
  Test_Convergence::setupPointers( GenericContainer const & gc_data ) {

    LW_ASSERT0(
      gc_data.exists("Pointers"),
      "Test_Convergence::setupPointers: Missing key `Pointers` in data\n"
    );
    GenericContainer const & gc = gc_data("Pointers");

    // Initialize user classes

    LW_ASSERT0(
      gc.exists("pEngine"),
      "in Test_Convergence::setupPointers(gc) cant find key `pEngine' in gc\n"
    );
    pEngine = gc("pEngine").get_pointer<Engine*>();

    LW_ASSERT0(
      gc.exists("pRoad"),
      "in Test_Convergence::setupPointers(gc) cant find key `pRoad' in gc\n"
    );
    pRoad = gc("pRoad").get_pointer<Road2D*>();
  }

  /* --------------------------------------------------------------------------
  //   _        __        ____ _
  //  (_)_ __  / _| ___  / ___| | __ _ ___ ___  ___  ___
  //  | | '_ \| |_ / _ \| |   | |/ _` / __/ __|/ _ \/ __|
  //  | | | | |  _| (_) | |___| | (_| \__ \__ \  __/\__ \
  //  |_|_| |_|_|  \___/ \____|_|\__,_|___/___/\___||___/
  */
  void
  Test_Convergence::infoClasses() const {
    int msg_level = 3;
    ostringstream mstr;

    pConsole->message("\nControls\n",msg_level);
    mstr.str(""); t__oControl   .info(mstr);
    pConsole->message(mstr.str(),msg_level);
    mstr.str(""); b__f__oControl.info(mstr);
    pConsole->message(mstr.str(),msg_level);
    mstr.str(""); b__r__oControl.info(mstr);
    pConsole->message(mstr.str(),msg_level);
    mstr.str(""); tau__oControl .info(mstr);
    pConsole->message(mstr.str(),msg_level);

    pConsole->message("\nConstraints 1D\n",msg_level);
    mstr.str(""); OnlyBrakingFront      .info(mstr);
    pConsole->message(mstr.str(),msg_level);
    mstr.str(""); OnlyBrakingRear       .info(mstr);
    pConsole->message(mstr.str(),msg_level);
    mstr.str(""); OnlyTractionRear      .info(mstr);
    pConsole->message(mstr.str(),msg_level);
    mstr.str(""); FrontWheelContact     .info(mstr);
    pConsole->message(mstr.str(),msg_level);
    mstr.str(""); RearWheelContact      .info(mstr);
    pConsole->message(mstr.str(),msg_level);
    mstr.str(""); LongSlipFront         .info(mstr);
    pConsole->message(mstr.str(),msg_level);
    mstr.str(""); LongSlipRear          .info(mstr);
    pConsole->message(mstr.str(),msg_level);
    mstr.str(""); LatSlipFront          .info(mstr);
    pConsole->message(mstr.str(),msg_level);
    mstr.str(""); LatSlipRear           .info(mstr);
    pConsole->message(mstr.str(),msg_level);
    mstr.str(""); MaxSteerAngle         .info(mstr);
    pConsole->message(mstr.str(),msg_level);
    mstr.str(""); MaxRollAngle          .info(mstr);
    pConsole->message(mstr.str(),msg_level);
    mstr.str(""); roadRightLateralBorder.info(mstr);
    pConsole->message(mstr.str(),msg_level);
    mstr.str(""); roadLeftLateralBorder .info(mstr);
    pConsole->message(mstr.str(),msg_level);

    pConsole->message("\nUser class (pointer)\n",msg_level);
    pConsole->message("User function `pEngine`: ",msg_level);
    mstr.str(""); pEngine->info(mstr);
    pConsole->message(mstr.str(),msg_level);
    pConsole->message("User function `pRoad`: ",msg_level);
    mstr.str(""); pRoad->info(mstr);
    pConsole->message(mstr.str(),msg_level);

    pConsole->message("\nModel Parameters\n",msg_level);
    for ( integer i = 0; i < numModelPars; ++i ) {
      pConsole->message(
        fmt::format("{:.>40} = {}\n",namesModelPars[i], ModelPars[i]),
        msg_level
      );
    }

  }

  /* --------------------------------------------------------------------------
  //            _
  //   ___  ___| |_ _   _ _ __
  //  / __|/ _ \ __| | | | '_ \
  //  \__ \  __/ |_| |_| | |_) |
  //  |___/\___|\__|\__,_| .__/
  //                     |_|
  */
  void
  Test_Convergence::setup( GenericContainer const & gc ) {

    if ( gc.get_map_bool("RedirectStreamToString") ) {
      ss_redirected_stream.str("");
      pConsole->changeStream(&ss_redirected_stream);
    }

    this->setupParameters( gc );
    this->setupClasses( gc );
    this->setupUserMappedFunctions( gc );
    this->setupUserClasses( gc );
    this->setupPointers( gc );
    this->setupBC( gc );
    this->setupControls( gc );

    // setup nonlinear system with object handling mesh domain
    this->setup( pRoad, gc );
    this->infoBC();
    this->infoClasses();
    this->info();
  }

  /* --------------------------------------------------------------------------
  //              _
  //    __ _  ___| |_     _ __   __ _ _ __ ___   ___  ___
  //   / _` |/ _ \ __|   | '_ \ / _` | '_ ` _ \ / _ \/ __|
  //  | (_| |  __/ |_    | | | | (_| | | | | | |  __/\__ \
  //   \__, |\___|\__|___|_| |_|\__,_|_| |_| |_|\___||___/
  //   |___/        |_____|
  */
  void
  Test_Convergence::get_names( GenericContainer & out ) const {
    vec_string_type & X_names = out["state_names"].set_vec_string();
    for ( integer i = 0; i < numXvars; ++i ) X_names.push_back(namesXvars[i]);

    vec_string_type & LM_names = out["lagrange_multiplier_names"].set_vec_string();
    for ( integer i = 0; i < numLvars; ++i ) LM_names.push_back(namesLvars[i]);

    vec_string_type & U_names = out["control_names"].set_vec_string();
    for ( integer i = 0; i < numUvars; ++i ) U_names.push_back(namesUvars[i]);

    vec_string_type & Q_names = out["mesh_variable_names"].set_vec_string();
    for ( integer i = 0; i < numQvars; ++i ) Q_names.push_back(namesQvars[i]);

    vec_string_type & P_names = out["parameter_names"].set_vec_string();
    for ( integer i = 0; i < numPvars; ++i ) P_names.push_back(namesPvars[i]);

    vec_string_type & OMEGA_names = out["bc_lagrange_multiplier_names"].set_vec_string();
    for ( integer i = 0; i < numOMEGAvars; ++i ) OMEGA_names.push_back(namesOMEGAvars[i]);

    vec_string_type & PP_names = out["post_processing_names"].set_vec_string();
    for ( integer i = 0; i < numPostProcess; ++i ) PP_names.push_back(namesPostProcess[i]);

    for ( integer i = 0; i < numIntegratedPostProcess; ++i )
      PP_names.push_back(namesIntegratedPostProcess[i]);

    vec_string_type & model_names = out["model_names"].set_vec_string();
    for ( integer i = 0; i < numModelPars; ++i )
      model_names.push_back(namesModelPars[i]);
  }

  /* --------------------------------------------------------------------------
  //      _ _                       _   _
  //   __| (_)__ _ __ _ _ _  ___ __| |_(_)__
  //  / _` | / _` / _` | ' \/ _ (_-<  _| / _|
  //  \__,_|_\__,_\__, |_||_\___/__/\__|_\__|
  //              |___/
  */
  void
  Test_Convergence::diagnostic( GenericContainer const & gc ) {

    // DA RIFARE--------------

    // If required save function and jacobian
    //if ( gc.exists("DumpFile") )
    //  this->dumpFunctionAndJacobian( pSolver->solution(),
    //                                 gc("DumpFile").get_string() );

    //bool do_diagnosis = gc.get_map_bool("Doctor");
    //if ( do_diagnosis )
    //  this->diagnosis( pSolver->solution(), gc["diagnosis"] );

    real_type epsi = 1e-5;
    gc.get_if_exists("JacobianCheck_epsilon",epsi);
    if ( gc.get_map_bool("JacobianCheck") )
      this->checkJacobian( pSolver->solution(), epsi );
    if ( gc.get_map_bool("JacobianCheckFull") )
      this->checkJacobianFull( pSolver->solution(), epsi );
  }

  // save model parameters
  void
  Test_Convergence::save_OCP_info( GenericContainer & gc ) const {
    for ( integer i = 0; i < numModelPars; ++i )
      gc[namesModelPars[i]] = ModelPars[i];

  }

}

// EOF: Test_Convergence.cc

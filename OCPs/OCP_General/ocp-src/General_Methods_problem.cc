/*-----------------------------------------------------------------------*\
 |  file: General_Methods1.cc                                            |
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


#include "General.hh"
#include "General_Pars.hh"

using namespace std;
using Mechatronix::real_type;
using Mechatronix::integer;
using Mechatronix::ostream_type;

// user class in namespaces
using Mechatronix::Engine;
using Mechatronix::Road2D;


#if defined(__clang__)
#pragma clang diagnostic ignored "-Wunused-variable"
#pragma clang diagnostic ignored "-Wunused-parameter"
#pragma clang diagnostic ignored "-Wsign-conversion"
#pragma clang diagnostic ignored "-Wunused-macros"
#elif defined(__llvm__) || defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wunused-macros"
#elif defined(_MSC_VER)
#pragma warning( disable : 4100 )
#pragma warning( disable : 4101 )
#endif

// map user defined functions and objects with macros
#define ALIAS_ISOAngle_R_DD(__t1) segmentRight.isoAngle_DD( __t1)
#define ALIAS_ISOAngle_R_D(__t1) segmentRight.isoAngle_D( __t1)
#define ALIAS_ISOAngle_R(__t1) segmentRight.isoAngle( __t1)
#define ALIAS_ISOAngle_L_DD(__t1) segmentLeft.isoAngle_DD( __t1)
#define ALIAS_ISOAngle_L_D(__t1) segmentLeft.isoAngle_D( __t1)
#define ALIAS_ISOAngle_L(__t1) segmentLeft.isoAngle( __t1)
#define ALIAS_ISOAngle_DD(__t1) segment.isoAngle_DD( __t1)
#define ALIAS_ISOAngle_D(__t1) segment.isoAngle_D( __t1)
#define ALIAS_ISOAngle(__t1) segment.isoAngle( __t1)
#define ALIAS_yISOright_R(__t1) segmentRight.isoRightY( __t1)
#define ALIAS_yISOright_L(__t1) segmentLeft.isoRightY( __t1)
#define ALIAS_yISOright(__t1) segment.isoRightY( __t1)
#define ALIAS_xISOright_R(__t1) segmentRight.isoRightX( __t1)
#define ALIAS_xISOright_L(__t1) segmentLeft.isoRightX( __t1)
#define ALIAS_xISOright(__t1) segment.isoRightX( __t1)
#define ALIAS_yISOleft_R(__t1) segmentRight.isoLeftY( __t1)
#define ALIAS_yISOleft_L(__t1) segmentLeft.isoLeftY( __t1)
#define ALIAS_yISOleft(__t1) segment.isoLeftY( __t1)
#define ALIAS_xISOleft_R(__t1) segmentRight.isoLeftX( __t1)
#define ALIAS_xISOleft_L(__t1) segmentLeft.isoLeftX( __t1)
#define ALIAS_xISOleft(__t1) segment.isoLeftX( __t1)
#define ALIAS_yISOMidLane_R_DD(__t1) segmentRight.isoY_DD( __t1)
#define ALIAS_yISOMidLane_R_D(__t1) segmentRight.isoY_D( __t1)
#define ALIAS_yISOMidLane_R(__t1) segmentRight.isoY( __t1)
#define ALIAS_yISOMidLane_L_DD(__t1) segmentLeft.isoY_DD( __t1)
#define ALIAS_yISOMidLane_L_D(__t1) segmentLeft.isoY_D( __t1)
#define ALIAS_yISOMidLane_L(__t1) segmentLeft.isoY( __t1)
#define ALIAS_yISOMidLane_DD(__t1) segment.isoY_DD( __t1)
#define ALIAS_yISOMidLane_D(__t1) segment.isoY_D( __t1)
#define ALIAS_yISOMidLane(__t1) segment.isoY( __t1)
#define ALIAS_xISOMidLane_R_DD(__t1) segmentRight.isoX_DD( __t1)
#define ALIAS_xISOMidLane_R_D(__t1) segmentRight.isoX_D( __t1)
#define ALIAS_xISOMidLane_R(__t1) segmentRight.isoX( __t1)
#define ALIAS_xISOMidLane_L_DD(__t1) segmentLeft.isoX_DD( __t1)
#define ALIAS_xISOMidLane_L_D(__t1) segmentLeft.isoX_D( __t1)
#define ALIAS_xISOMidLane_L(__t1) segmentLeft.isoX( __t1)
#define ALIAS_xISOMidLane_DD(__t1) segment.isoX_DD( __t1)
#define ALIAS_xISOMidLane_D(__t1) segment.isoX_D( __t1)
#define ALIAS_xISOMidLane(__t1) segment.isoX( __t1)
#define ALIAS_adherence_R(___dummy___) segmentRight.adherence()
#define ALIAS_adherence_L(___dummy___) segmentLeft.adherence()
#define ALIAS_adherence(___dummy___) segment.adherence()
#define ALIAS_sectionSpeedLimit_R(___dummy___) segmentRight.speedLimit()
#define ALIAS_sectionSpeedLimit_L(___dummy___) segmentLeft.speedLimit()
#define ALIAS_sectionSpeedLimit(___dummy___) segment.speedLimit()
#define ALIAS_rightWidth_R(__t1) segmentRight.rightWidth( __t1)
#define ALIAS_rightWidth_L(__t1) segmentLeft.rightWidth( __t1)
#define ALIAS_rightWidth(__t1) segment.rightWidth( __t1)
#define ALIAS_leftWidth_R_D(__t1) segmentRight.leftWidth_D( __t1)
#define ALIAS_leftWidth_R(__t1) segmentRight.leftWidth( __t1)
#define ALIAS_leftWidth_L_D(__t1) segmentLeft.leftWidth_D( __t1)
#define ALIAS_leftWidth_L(__t1) segmentLeft.leftWidth( __t1)
#define ALIAS_leftWidth_D(__t1) segment.leftWidth_D( __t1)
#define ALIAS_leftWidth(__t1) segment.leftWidth( __t1)
#define ALIAS_kappa_R_DD(__t1) segmentRight.saeCurvature_DD( __t1)
#define ALIAS_kappa_R_D(__t1) segmentRight.saeCurvature_D( __t1)
#define ALIAS_kappa_R(__t1) segmentRight.saeCurvature( __t1)
#define ALIAS_kappa_L_DD(__t1) segmentLeft.saeCurvature_DD( __t1)
#define ALIAS_kappa_L_D(__t1) segmentLeft.saeCurvature_D( __t1)
#define ALIAS_kappa_L(__t1) segmentLeft.saeCurvature( __t1)
#define ALIAS_kappa_DD(__t1) segment.saeCurvature_DD( __t1)
#define ALIAS_kappa_D(__t1) segment.saeCurvature_D( __t1)
#define ALIAS_kappa(__t1) segment.saeCurvature( __t1)
#define ALIAS_maxTorque_DD(__t1) pEngine -> maxTorqueOnWheelHub_DD( __t1)
#define ALIAS_maxTorque_D(__t1) pEngine -> maxTorqueOnWheelHub_D( __t1)
#define ALIAS_maxTorque(__t1) pEngine -> maxTorqueOnWheelHub( __t1)
#define ALIAS_roadLeftLateralBorder_DD(__t1) roadLeftLateralBorder.DD( __t1)
#define ALIAS_roadLeftLateralBorder_D(__t1) roadLeftLateralBorder.D( __t1)
#define ALIAS_roadRightLateralBorder_DD(__t1) roadRightLateralBorder.DD( __t1)
#define ALIAS_roadRightLateralBorder_D(__t1) roadRightLateralBorder.D( __t1)
#define ALIAS_MaxRollAngle_DD(__t1) MaxRollAngle.DD( __t1)
#define ALIAS_MaxRollAngle_D(__t1) MaxRollAngle.D( __t1)
#define ALIAS_MaxSteerAngle_DD(__t1) MaxSteerAngle.DD( __t1)
#define ALIAS_MaxSteerAngle_D(__t1) MaxSteerAngle.D( __t1)
#define ALIAS_LatSlipRear_DD(__t1) LatSlipRear.DD( __t1)
#define ALIAS_LatSlipRear_D(__t1) LatSlipRear.D( __t1)
#define ALIAS_LatSlipFront_DD(__t1) LatSlipFront.DD( __t1)
#define ALIAS_LatSlipFront_D(__t1) LatSlipFront.D( __t1)
#define ALIAS_LongSlipRear_DD(__t1) LongSlipRear.DD( __t1)
#define ALIAS_LongSlipRear_D(__t1) LongSlipRear.D( __t1)
#define ALIAS_LongSlipFront_DD(__t1) LongSlipFront.DD( __t1)
#define ALIAS_LongSlipFront_D(__t1) LongSlipFront.D( __t1)
#define ALIAS_RearWheelContact_DD(__t1) RearWheelContact.DD( __t1)
#define ALIAS_RearWheelContact_D(__t1) RearWheelContact.D( __t1)
#define ALIAS_FrontWheelContact_DD(__t1) FrontWheelContact.DD( __t1)
#define ALIAS_FrontWheelContact_D(__t1) FrontWheelContact.D( __t1)
#define ALIAS_OnlyTractionRear_DD(__t1) OnlyTractionRear.DD( __t1)
#define ALIAS_OnlyTractionRear_D(__t1) OnlyTractionRear.D( __t1)
#define ALIAS_OnlyBrakingRear_DD(__t1) OnlyBrakingRear.DD( __t1)
#define ALIAS_OnlyBrakingRear_D(__t1) OnlyBrakingRear.D( __t1)
#define ALIAS_OnlyBrakingFront_DD(__t1) OnlyBrakingFront.DD( __t1)
#define ALIAS_OnlyBrakingFront_D(__t1) OnlyBrakingFront.D( __t1)
#define ALIAS_tau__oControl_D_3(__t1, __t2, __t3) tau__oControl.D_3( __t1, __t2, __t3)
#define ALIAS_tau__oControl_D_2(__t1, __t2, __t3) tau__oControl.D_2( __t1, __t2, __t3)
#define ALIAS_tau__oControl_D_1(__t1, __t2, __t3) tau__oControl.D_1( __t1, __t2, __t3)
#define ALIAS_tau__oControl_D_3_3(__t1, __t2, __t3) tau__oControl.D_3_3( __t1, __t2, __t3)
#define ALIAS_tau__oControl_D_2_3(__t1, __t2, __t3) tau__oControl.D_2_3( __t1, __t2, __t3)
#define ALIAS_tau__oControl_D_2_2(__t1, __t2, __t3) tau__oControl.D_2_2( __t1, __t2, __t3)
#define ALIAS_tau__oControl_D_1_3(__t1, __t2, __t3) tau__oControl.D_1_3( __t1, __t2, __t3)
#define ALIAS_tau__oControl_D_1_2(__t1, __t2, __t3) tau__oControl.D_1_2( __t1, __t2, __t3)
#define ALIAS_tau__oControl_D_1_1(__t1, __t2, __t3) tau__oControl.D_1_1( __t1, __t2, __t3)
#define ALIAS_b__r__oControl_D_3(__t1, __t2, __t3) b__r__oControl.D_3( __t1, __t2, __t3)
#define ALIAS_b__r__oControl_D_2(__t1, __t2, __t3) b__r__oControl.D_2( __t1, __t2, __t3)
#define ALIAS_b__r__oControl_D_1(__t1, __t2, __t3) b__r__oControl.D_1( __t1, __t2, __t3)
#define ALIAS_b__r__oControl_D_3_3(__t1, __t2, __t3) b__r__oControl.D_3_3( __t1, __t2, __t3)
#define ALIAS_b__r__oControl_D_2_3(__t1, __t2, __t3) b__r__oControl.D_2_3( __t1, __t2, __t3)
#define ALIAS_b__r__oControl_D_2_2(__t1, __t2, __t3) b__r__oControl.D_2_2( __t1, __t2, __t3)
#define ALIAS_b__r__oControl_D_1_3(__t1, __t2, __t3) b__r__oControl.D_1_3( __t1, __t2, __t3)
#define ALIAS_b__r__oControl_D_1_2(__t1, __t2, __t3) b__r__oControl.D_1_2( __t1, __t2, __t3)
#define ALIAS_b__r__oControl_D_1_1(__t1, __t2, __t3) b__r__oControl.D_1_1( __t1, __t2, __t3)
#define ALIAS_b__f__oControl_D_3(__t1, __t2, __t3) b__f__oControl.D_3( __t1, __t2, __t3)
#define ALIAS_b__f__oControl_D_2(__t1, __t2, __t3) b__f__oControl.D_2( __t1, __t2, __t3)
#define ALIAS_b__f__oControl_D_1(__t1, __t2, __t3) b__f__oControl.D_1( __t1, __t2, __t3)
#define ALIAS_b__f__oControl_D_3_3(__t1, __t2, __t3) b__f__oControl.D_3_3( __t1, __t2, __t3)
#define ALIAS_b__f__oControl_D_2_3(__t1, __t2, __t3) b__f__oControl.D_2_3( __t1, __t2, __t3)
#define ALIAS_b__f__oControl_D_2_2(__t1, __t2, __t3) b__f__oControl.D_2_2( __t1, __t2, __t3)
#define ALIAS_b__f__oControl_D_1_3(__t1, __t2, __t3) b__f__oControl.D_1_3( __t1, __t2, __t3)
#define ALIAS_b__f__oControl_D_1_2(__t1, __t2, __t3) b__f__oControl.D_1_2( __t1, __t2, __t3)
#define ALIAS_b__f__oControl_D_1_1(__t1, __t2, __t3) b__f__oControl.D_1_1( __t1, __t2, __t3)
#define ALIAS_t__oControl_D_3(__t1, __t2, __t3) t__oControl.D_3( __t1, __t2, __t3)
#define ALIAS_t__oControl_D_2(__t1, __t2, __t3) t__oControl.D_2( __t1, __t2, __t3)
#define ALIAS_t__oControl_D_1(__t1, __t2, __t3) t__oControl.D_1( __t1, __t2, __t3)
#define ALIAS_t__oControl_D_3_3(__t1, __t2, __t3) t__oControl.D_3_3( __t1, __t2, __t3)
#define ALIAS_t__oControl_D_2_3(__t1, __t2, __t3) t__oControl.D_2_3( __t1, __t2, __t3)
#define ALIAS_t__oControl_D_2_2(__t1, __t2, __t3) t__oControl.D_2_2( __t1, __t2, __t3)
#define ALIAS_t__oControl_D_1_3(__t1, __t2, __t3) t__oControl.D_1_3( __t1, __t2, __t3)
#define ALIAS_t__oControl_D_1_2(__t1, __t2, __t3) t__oControl.D_1_2( __t1, __t2, __t3)
#define ALIAS_t__oControl_D_1_1(__t1, __t2, __t3) t__oControl.D_1_1( __t1, __t2, __t3)


namespace GeneralDefine {
  /*\
   |   ___         _   _               _   _
   |  / __|___ _ _| |_(_)_ _ _  _ __ _| |_(_)___ _ _
   | | (__/ _ \ ' \  _| | ' \ || / _` |  _| / _ \ ' \
   |  \___\___/_||_\__|_|_||_\_,_\__,_|\__|_\___/_||_|
  \*/

  void
  General::continuationStep0( real_type s ) {
    int msg_level = 3;
    pConsole->message(
      fmt::format( "\nContinuation step N.0 s = {}\n", s ),
      msg_level
    );
    real_type t1   = ModelPars[137];
    ModelPars[136] = t1 + (ModelPars[138] - t1) * s;
    real_type t5   = ModelPars[134];
    ModelPars[133] = t5 + (ModelPars[135] - t5) * s;
  }
  /*\
   |   ___         _   _               _   _
   |  / __|___ _ _| |_(_)_ _ _  _ __ _| |_(_)___ _ _
   | | (__/ _ \ ' \  _| | ' \ || / _` |  _| / _ \ ' \
   |  \___\___/_||_\__|_|_||_\_,_\__,_|\__|_\___/_||_|
  \*/

  void
  General::continuationStep1( real_type s ) {
    int msg_level = 3;
    pConsole->message(
      fmt::format( "\nContinuation step N.1 s = {}\n", s ),
      msg_level
    );
    real_type t1   = ModelPars[128];
    real_type w__LR = t1 + (ModelPars[129] - t1) * s;
  }
  /*\
   |   ___         _   _               _   _
   |  / __|___ _ _| |_(_)_ _ _  _ __ _| |_(_)___ _ _
   | | (__/ _ \ ' \  _| | ' \ || / _` |  _| / _ \ ' \
   |  \___\___/_||_\__|_|_||_\_,_\__,_|\__|_\___/_||_|
  \*/

  void
  General::continuationStep2( real_type s ) {
    int msg_level = 3;
    pConsole->message(
      fmt::format( "\nContinuation step N.2 s = {}\n", s ),
      msg_level
    );
    real_type t1   = ModelPars[131];
    ModelPars[130] = t1 + (ModelPars[132] - t1) * s;
  }
  /*\
   |   ___         _   _               _   _
   |  / __|___ _ _| |_(_)_ _ _  _ __ _| |_(_)___ _ _
   | | (__/ _ \ ' \  _| | ' \ || / _` |  _| / _ \ ' \
   |  \___\___/_||_\__|_|_||_\_,_\__,_|\__|_\___/_||_|
  \*/

  void
  General::continuationStep3( real_type s ) {
    int msg_level = 3;
    pConsole->message(
      fmt::format( "\nContinuation step N.3 s = {}\n", s ),
      msg_level
    );
    real_type t1   = ModelPars[60];
    real_type t5   = t1 + (ModelPars[61] - t1) * s;
    b__f__oControl.update_epsilon(t5);
    real_type t6   = ModelPars[122];
    real_type t10  = t6 + (ModelPars[123] - t6) * s;
    b__f__oControl.update_tolerance(t10);
    t__oControl.update_epsilon(t5);
    t__oControl.update_tolerance(t10);
    b__r__oControl.update_epsilon(t5);
    b__r__oControl.update_tolerance(t10);
    tau__oControl.update_epsilon(t5);
    tau__oControl.update_tolerance(t10);
    real_type t11  = ModelPars[62];
    real_type t15  = t11 + (ModelPars[63] - t11) * s;
    FrontWheelContact.update_epsilon(t15);
    real_type t16  = ModelPars[124];
    real_type t20  = t16 + (ModelPars[125] - t16) * s;
    FrontWheelContact.update_tolerance(t20);
    RearWheelContact.update_epsilon(t15);
    RearWheelContact.update_tolerance(t20);
    LongSlipFront.update_epsilon(t15);
    LongSlipFront.update_tolerance(t20);
    LatSlipFront.update_epsilon(t15);
    LatSlipFront.update_tolerance(t20);
    LongSlipRear.update_epsilon(t15);
    LongSlipRear.update_tolerance(t20);
    LatSlipRear.update_epsilon(t15);
    LatSlipRear.update_tolerance(t20);
    MaxSteerAngle.update_epsilon(t15);
    MaxSteerAngle.update_tolerance(t20);
    MaxRollAngle.update_epsilon(t15);
    MaxRollAngle.update_tolerance(t20);
    roadRightLateralBorder.update_epsilon(t15);
    roadRightLateralBorder.update_tolerance(t20);
    roadLeftLateralBorder.update_epsilon(t15);
    roadLeftLateralBorder.update_tolerance(t20);
    real_type t21  = ModelPars[58];
    real_type t25  = t21 + (ModelPars[59] - t21) * s;
    OnlyBrakingFront.update_epsilon(t25);
    real_type t26  = ModelPars[89];
    real_type t30  = t26 + (ModelPars[90] - t26) * s;
    OnlyBrakingFront.update_tolerance(t30);
    OnlyBrakingRear.update_epsilon(t25);
    OnlyBrakingRear.update_tolerance(t30);
    OnlyTractionRear.update_epsilon(t25);
    OnlyTractionRear.update_tolerance(t30);
  }

  /*\
   |  _   _               ___             _   _
   | | | | |___ ___ _ _  | __|  _ _ _  __| |_(_)___ _ _  ___
   | | |_| (_-</ -_) '_| | _| || | ' \/ _|  _| / _ \ ' \(_-<
   |  \___//__/\___|_|   |_| \_,_|_||_\__|\__|_\___/_||_/__/
  \*/
  // user defined functions which has a body defined in MAPLE
  real_type
  General::alpha__crw( real_type t__XO ) const {
    return asin(1.0 / ModelPars[23] * (ModelPars[114] - ModelPars[115]));
  }

  real_type
  General::alpha__crw_D( real_type t__XO ) const {
    return 0;
  }

  real_type
  General::alpha__crw_DD( real_type t__XO ) const {
    return 0;
  }

  real_type
  General::alpha__pin( real_type eta__XO, real_type alpha__crw__XO ) const {
    return alpha__crw__XO + eta__XO;
  }

  real_type
  General::alpha__pin_D_1( real_type eta__XO, real_type alpha__crw__XO ) const {
    return 1;
  }

  real_type
  General::alpha__pin_D_1_1( real_type eta__XO, real_type alpha__crw__XO ) const {
    return 0;
  }

  real_type
  General::alpha__pin_D_1_2( real_type eta__XO, real_type alpha__crw__XO ) const {
    return 0;
  }

  real_type
  General::alpha__pin_D_2( real_type eta__XO, real_type alpha__crw__XO ) const {
    return 1;
  }

  real_type
  General::alpha__pin_D_2_2( real_type eta__XO, real_type alpha__crw__XO ) const {
    return 0;
  }

  real_type
  General::Fzf( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return -z__f__XO * ModelPars[20] - z__f__dot__XO * ModelPars[2];
  }

  real_type
  General::Fzf_D_1( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return -ModelPars[20];
  }

  real_type
  General::Fzf_D_1_1( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return 0;
  }

  real_type
  General::Fzf_D_1_2( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return 0;
  }

  real_type
  General::Fzf_D_2( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return -ModelPars[2];
  }

  real_type
  General::Fzf_D_2_2( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return 0;
  }

  real_type
  General::Fzr( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return -z__r__XO * ModelPars[21] - z__r__dot__XO * ModelPars[3];
  }

  real_type
  General::Fzr_D_1( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return -ModelPars[21];
  }

  real_type
  General::Fzr_D_1_1( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return 0;
  }

  real_type
  General::Fzr_D_1_2( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return 0;
  }

  real_type
  General::Fzr_D_2( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return -ModelPars[3];
  }

  real_type
  General::Fzr_D_2_2( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return 0;
  }

  real_type
  General::alpha__r( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t7   = atan((-Omega__XO * x__r__XO + v__XO + y__r__dot__XO) / (-y__r__XO * Omega__XO + u__XO - x__r__dot__XO));
    return -t7;
  }

  real_type
  General::alpha__r_D_1( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t6   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t7   = t2 * t2;
    real_type t8   = 1.0 / t7;
    real_type t12  = t6 * t6;
    return -1.0 / (t8 * t12 + 1) * (-1.0 / t2 * x__r__XO + y__r__XO * t8 * t6);
  }

  real_type
  General::alpha__r_D_1_1( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t4   = 1.0 / t3;
    real_type t8   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t10  = 1.0 / t3 / t2;
    real_type t12  = y__r__XO * y__r__XO;
    real_type t16  = t8 * t8;
    real_type t18  = t4 * t16 + 1;
    real_type t23  = t4 * t8;
    real_type t26  = t18 * t18;
    return -1.0 / t18 * (2 * t12 * t10 * t8 - 2 * y__r__XO * t4 * x__r__XO) + (2 * y__r__XO * t10 * t16 - 2 * x__r__XO * t23) / t26 * (-1.0 / t2 * x__r__XO + y__r__XO * t23);
  }

  real_type
  General::alpha__r_D_1_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t4   = 1.0 / t3;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t9   = 1.0 / t3 / t2;
    real_type t14  = t7 * t7;
    real_type t16  = t4 * t14 + 1;
    real_type t24  = t16 * t16;
    return -1.0 / t16 * (-2 * y__r__XO * t9 * t7 + t4 * x__r__XO) - 2 * t9 * t14 / t24 * (-1.0 / t2 * x__r__XO + y__r__XO * t4 * t7);
  }

  real_type
  General::alpha__r_D_1_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t4   = 1.0 / t3;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t10  = t4 * t8 + 1;
    real_type t15  = t4 * t7;
    real_type t18  = t10 * t10;
    return -1.0 / t10 * y__r__XO * t4 + 2 * t15 / t18 * (-1.0 / t2 * x__r__XO + y__r__XO * t15);
  }

  real_type
  General::alpha__r_D_1_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = 1.0 / t2;
    real_type t4   = t2 * t2;
    real_type t5   = 1.0 / t4;
    real_type t10  = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t11  = t10 * t10;
    real_type t13  = t5 * t11 + 1;
    real_type t17  = t5 * t10;
    real_type t20  = t13 * t13;
    return -1.0 / t13 * (-y__r__XO * t5 * Omega__XO - t3) - 2 * Omega__XO * t17 / t20 * (y__r__XO * t17 - t3 * x__r__XO);
  }

  real_type
  General::alpha__r_D_1_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = -t1 + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t4   = 1.0 / t3;
    real_type t8   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t10  = 1.0 / t3 / t2;
    real_type t14  = t4 * t8;
    real_type t16  = t8 * t8;
    real_type t18  = t4 * t16 + 1;
    real_type t25  = t18 * t18;
    return -1.0 / t18 * (2 * t1 * t10 * t8 - Omega__XO * t4 * x__r__XO + t14) + 2 * Omega__XO * t10 * t16 / t25 * (-1.0 / t2 * x__r__XO + y__r__XO * t14);
  }

  real_type
  General::alpha__r_D_1_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t4   = 1.0 / t3;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t9   = 1.0 / t3 / t2;
    real_type t14  = t7 * t7;
    real_type t16  = t4 * t14 + 1;
    real_type t24  = t16 * t16;
    return -1.0 / t16 * (2 * y__r__XO * t9 * t7 - t4 * x__r__XO) + 2 * t9 * t14 / t24 * (-1.0 / t2 * x__r__XO + y__r__XO * t4 * t7);
  }

  real_type
  General::alpha__r_D_1_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t4   = 1.0 / t3;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t10  = t4 * t8 + 1;
    real_type t15  = t4 * t7;
    real_type t18  = t10 * t10;
    return -1.0 / t10 * y__r__XO * t4 + 2 * t15 / t18 * (-1.0 / t2 * x__r__XO + y__r__XO * t15);
  }

  real_type
  General::alpha__r_D_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t5   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t6   = 1.0 / t5;
    real_type t8   = t2 * t2;
    return 1.0 / (t6 * t8 + 1) * t6 * t2;
  }

  real_type
  General::alpha__r_D_2_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t4   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t5   = t4 * t4;
    real_type t9   = t2 * t2;
    real_type t12  = 1.0 / t5 * t9 + 1;
    real_type t16  = t5 * t5;
    real_type t20  = t12 * t12;
    return -2 / t12 / t5 / t4 * t2 + 2 / t20 / t16 / t4 * t9 * t2;
  }

  real_type
  General::alpha__r_D_2_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t7   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t9   = t4 * t7 + 1;
    real_type t12  = t3 * t3;
    real_type t15  = t9 * t9;
    return 1.0 / t9 * t4 - 2 / t15 / t12 * t7;
  }

  real_type
  General::alpha__r_D_2_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t8   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t10  = t4 * t8 + 1;
    real_type t13  = t3 * t3;
    real_type t16  = t10 * t10;
    return -1.0 / t10 * t4 * Omega__XO + 2 * Omega__XO / t16 / t13 * t8;
  }

  real_type
  General::alpha__r_D_2_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t4   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t5   = t4 * t4;
    real_type t9   = t2 * t2;
    real_type t12  = 1.0 / t5 * t9 + 1;
    real_type t17  = t5 * t5;
    real_type t21  = t12 * t12;
    return 2 * Omega__XO / t12 / t5 / t4 * t2 - 2 * Omega__XO / t21 / t17 / t4 * t9 * t2;
  }

  real_type
  General::alpha__r_D_2_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t4   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t5   = t4 * t4;
    real_type t9   = t2 * t2;
    real_type t12  = 1.0 / t5 * t9 + 1;
    real_type t16  = t5 * t5;
    real_type t20  = t12 * t12;
    return 2 / t12 / t5 / t4 * t2 - 2 / t20 / t16 / t4 * t9 * t2;
  }

  real_type
  General::alpha__r_D_2_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t7   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t9   = t4 * t7 + 1;
    real_type t12  = t3 * t3;
    real_type t15  = t9 * t9;
    return 1.0 / t9 * t4 - 2 / t15 / t12 * t7;
  }

  real_type
  General::alpha__r_D_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t6   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t7   = t2 * t2;
    return -1.0 / (1.0 / t7 * t6 + 1) / t2;
  }

  real_type
  General::alpha__r_D_3_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t12  = pow(1.0 / t3 * t8 + 1, 2);
    return 2 * t7 / t12 / t3 / t2;
  }

  real_type
  General::alpha__r_D_3_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t12  = pow(1.0 / t3 * t8 + 1, 2);
    return -2 * Omega__XO * t7 / t12 / t3 / t2;
  }

  real_type
  General::alpha__r_D_3_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t8   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t10  = t4 * t8 + 1;
    real_type t13  = t3 * t3;
    real_type t16  = t10 * t10;
    return -1.0 / t10 * t4 * Omega__XO + 2 * Omega__XO / t16 / t13 * t8;
  }

  real_type
  General::alpha__r_D_3_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t7   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t9   = t4 * t7 + 1;
    real_type t12  = t3 * t3;
    real_type t15  = t9 * t9;
    return -1.0 / t9 * t4 + 2 / t15 / t12 * t7;
  }

  real_type
  General::alpha__r_D_3_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t12  = pow(1.0 / t3 * t8 + 1, 2);
    return 2 * t7 / t12 / t3 / t2;
  }

  real_type
  General::alpha__r_D_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t7   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t8   = t2 * t2;
    return 1.0 / (1.0 / t8 * t7 + 1) / t2 * Omega__XO;
  }

  real_type
  General::alpha__r_D_4_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t1   = Omega__XO * Omega__XO;
    real_type t3   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t4   = t3 * t3;
    real_type t9   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t10  = t9 * t9;
    real_type t14  = pow(1.0 / t4 * t10 + 1, 2);
    return 2 * t9 / t14 / t4 / t3 * t1;
  }

  real_type
  General::alpha__r_D_4_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t1   = Omega__XO * Omega__XO;
    real_type t4   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t5   = 1.0 / t4;
    real_type t9   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t11  = t5 * t9 + 1;
    real_type t14  = t4 * t4;
    real_type t17  = t11 * t11;
    return 1.0 / t11 * t5 * t1 - 2 * t9 / t17 / t14 * t1;
  }

  real_type
  General::alpha__r_D_4_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t8   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t10  = t4 * t8 + 1;
    real_type t13  = t3 * t3;
    real_type t16  = t10 * t10;
    return 1.0 / t10 * t4 * Omega__XO - 2 * Omega__XO / t16 / t13 * t8;
  }

  real_type
  General::alpha__r_D_4_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t12  = pow(1.0 / t3 * t8 + 1, 2);
    return -2 * Omega__XO * t7 / t12 / t3 / t2;
  }

  real_type
  General::alpha__r_D_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t5   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t6   = 1.0 / t5;
    real_type t8   = t2 * t2;
    return -1.0 / (t6 * t8 + 1) * Omega__XO * t6 * t2;
  }

  real_type
  General::alpha__r_D_5_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t4   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t5   = t4 * t4;
    real_type t9   = Omega__XO * Omega__XO;
    real_type t10  = t2 * t2;
    real_type t13  = 1.0 / t5 * t10 + 1;
    real_type t18  = t5 * t5;
    real_type t22  = t13 * t13;
    return -2 / t13 * t9 / t5 / t4 * t2 + 2 / t22 * t9 / t18 / t4 * t10 * t2;
  }

  real_type
  General::alpha__r_D_5_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t4   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t5   = t4 * t4;
    real_type t9   = t2 * t2;
    real_type t12  = 1.0 / t5 * t9 + 1;
    real_type t17  = t5 * t5;
    real_type t21  = t12 * t12;
    return -2 * Omega__XO / t12 / t5 / t4 * t2 + 2 * Omega__XO / t21 / t17 / t4 * t9 * t2;
  }

  real_type
  General::alpha__r_D_5_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t8   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t10  = t4 * t8 + 1;
    real_type t13  = t3 * t3;
    real_type t16  = t10 * t10;
    return -1.0 / t10 * t4 * Omega__XO + 2 * Omega__XO / t16 / t13 * t8;
  }

  real_type
  General::alpha__r_D_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t5   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t6   = 1.0 / t5;
    real_type t8   = t2 * t2;
    return -1.0 / (t6 * t8 + 1) * t6 * t2;
  }

  real_type
  General::alpha__r_D_6_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t4   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t5   = t4 * t4;
    real_type t9   = t2 * t2;
    real_type t12  = 1.0 / t5 * t9 + 1;
    real_type t16  = t5 * t5;
    real_type t20  = t12 * t12;
    return -2 / t12 / t5 / t4 * t2 + 2 / t20 / t16 / t4 * t9 * t2;
  }

  real_type
  General::alpha__r_D_6_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t7   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t9   = t4 * t7 + 1;
    real_type t12  = t3 * t3;
    real_type t15  = t9 * t9;
    return -1.0 / t9 * t4 + 2 / t15 / t12 * t7;
  }

  real_type
  General::alpha__r_D_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t6   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t7   = t2 * t2;
    return -1.0 / (1.0 / t7 * t6 + 1) / t2;
  }

  real_type
  General::alpha__r_D_7_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t12  = pow(1.0 / t3 * t8 + 1, 2);
    return 2 * t7 / t12 / t3 / t2;
  }

  real_type
  General::alpha__f( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t2   = y__f__XO * Omega__XO;
    real_type t5   = x__f__XO * Omega__XO;
    real_type t12  = atan(1.0 / (-t2 + u__XO + x__f__dot__XO + delta__f__XO * (t5 + v__XO + y__f__dot__XO)) * (-x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t2 - u__XO) * delta__f__XO + t5 + v__XO));
    return -t12;
  }

  real_type
  General::alpha__f_D_1( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t3   = y__f__XO * Omega__XO;
    real_type t4   = x__f__XO * Omega__XO;
    real_type t7   = -t3 + u__XO + x__f__dot__XO + delta__f__XO * (t4 + v__XO + y__f__dot__XO);
    real_type t13  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t3 - u__XO) * delta__f__XO + t4 + v__XO;
    real_type t14  = t7 * t7;
    real_type t15  = 1.0 / t14;
    real_type t21  = t13 * t13;
    return -1.0 / (t15 * t21 + 1) * (1.0 / t7 * (delta__f__XO * y__f__XO + x__f__XO) - (delta__f__XO * x__f__XO - y__f__XO) * t15 * t13);
  }

  real_type
  General::alpha__f_D_1_1( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t2   = delta__f__XO * y__f__XO + x__f__XO;
    real_type t3   = y__f__XO * Omega__XO;
    real_type t4   = x__f__XO * Omega__XO;
    real_type t7   = -t3 + u__XO + x__f__dot__XO + delta__f__XO * (t4 + v__XO + y__f__dot__XO);
    real_type t8   = t7 * t7;
    real_type t9   = 1.0 / t8;
    real_type t12  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t17  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t3 - u__XO) * delta__f__XO + t4 + v__XO;
    real_type t19  = 1.0 / t8 / t7;
    real_type t21  = t12 * t12;
    real_type t25  = t17 * t17;
    real_type t27  = t9 * t25 + 1;
    real_type t32  = t9 * t17;
    real_type t35  = t27 * t27;
    return -1.0 / t27 * (-2 * t12 * t9 * t2 + 2 * t21 * t19 * t17) + (-2 * t12 * t19 * t25 + 2 * t2 * t32) / t35 * (1.0 / t7 * t2 - t12 * t32);
  }

  real_type
  General::alpha__f_D_1_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t2   = delta__f__XO * y__f__XO + x__f__XO;
    real_type t3   = y__f__XO * Omega__XO;
    real_type t4   = x__f__XO * Omega__XO;
    real_type t7   = -t3 + u__XO + x__f__dot__XO + delta__f__XO * (t4 + v__XO + y__f__dot__XO);
    real_type t8   = t7 * t7;
    real_type t9   = 1.0 / t8;
    real_type t13  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t18  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t3 - u__XO) * delta__f__XO + t4 + v__XO;
    real_type t20  = 1.0 / t8 / t7;
    real_type t25  = t18 * t18;
    real_type t27  = t9 * t25 + 1;
    real_type t32  = t9 * t18;
    real_type t35  = t27 * t27;
    return -1.0 / t27 * (2 * t13 * t20 * t18 + t13 * t9 * delta__f__XO - t9 * t2) + (-2 * t20 * t25 - 2 * delta__f__XO * t32) / t35 * (1.0 / t7 * t2 - t13 * t32);
  }

  real_type
  General::alpha__f_D_1_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t2   = delta__f__XO * y__f__XO + x__f__XO;
    real_type t3   = y__f__XO * Omega__XO;
    real_type t4   = x__f__XO * Omega__XO;
    real_type t7   = -t3 + u__XO + x__f__dot__XO + delta__f__XO * (t4 + v__XO + y__f__dot__XO);
    real_type t8   = t7 * t7;
    real_type t9   = 1.0 / t8;
    real_type t13  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t18  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t3 - u__XO) * delta__f__XO + t4 + v__XO;
    real_type t20  = 1.0 / t8 / t7;
    real_type t26  = t18 * t18;
    real_type t28  = t9 * t26 + 1;
    real_type t33  = t9 * t18;
    real_type t36  = t28 * t28;
    return -1.0 / t28 * (2 * delta__f__XO * t13 * t20 * t18 - delta__f__XO * t9 * t2 - t13 * t9) + (-2 * delta__f__XO * t20 * t26 + 2 * t33) / t36 * (1.0 / t7 * t2 - t13 * t33);
  }

  real_type
  General::alpha__f_D_1_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + delta__f__XO * (t2 + v__XO + y__f__dot__XO);
    real_type t6   = 1.0 / t5;
    real_type t8   = delta__f__XO * y__f__XO + x__f__XO;
    real_type t9   = t5 * t5;
    real_type t10  = 1.0 / t9;
    real_type t12  = Omega__XO * delta__f__XO;
    real_type t16  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t21  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t23  = 1.0 / t9 / t5;
    real_type t29  = t10 * t21;
    real_type t32  = t21 * t21;
    real_type t34  = t10 * t32 + 1;
    real_type t40  = t34 * t34;
    return -1.0 / t34 * (2 * delta__f__XO * Omega__XO * t16 * t23 * t21 - t12 * t10 * t8 - t16 * t10 * Omega__XO - delta__f__XO * t29 + t6) + (-2 * t12 * t23 * t32 + 2 * Omega__XO * t29) / t40 * (-t16 * t29 + t6 * t8);
  }

  real_type
  General::alpha__f_D_1_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + delta__f__XO * (t2 + v__XO + y__f__dot__XO);
    real_type t6   = 1.0 / t5;
    real_type t9   = delta__f__XO * y__f__XO + x__f__XO;
    real_type t10  = t5 * t5;
    real_type t11  = 1.0 / t10;
    real_type t14  = Omega__XO * delta__f__XO;
    real_type t16  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t22  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t24  = 1.0 / t10 / t5;
    real_type t29  = t11 * t22;
    real_type t31  = t22 * t22;
    real_type t33  = t11 * t31 + 1;
    real_type t39  = t33 * t33;
    return -1.0 / t33 * (-2 * Omega__XO * t16 * t24 * t22 - t16 * t11 * t14 + Omega__XO * t11 * t9 + t6 * delta__f__XO + t29) + (2 * Omega__XO * t24 * t31 + 2 * t14 * t29) / t39 * (-t16 * t29 + t6 * t9);
  }

  real_type
  General::alpha__f_D_1_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO + y__f__dot__XO;
    real_type t5   = delta__f__XO * t3 - t1 + u__XO + x__f__dot__XO;
    real_type t6   = 1.0 / t5;
    real_type t9   = delta__f__XO * y__f__XO + x__f__XO;
    real_type t10  = t5 * t5;
    real_type t11  = 1.0 / t10;
    real_type t14  = t1 - u__XO - x__f__dot__XO;
    real_type t17  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t22  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t24  = 1.0 / t10 / t5;
    real_type t29  = t11 * t22;
    real_type t32  = t22 * t22;
    real_type t34  = t11 * t32 + 1;
    real_type t40  = t34 * t34;
    return -1.0 / t34 * (2 * t3 * t17 * t24 * t22 - t17 * t11 * t14 - t3 * t11 * t9 - x__f__XO * t29 + t6 * y__f__XO) + (-2 * t3 * t24 * t32 + 2 * t14 * t29) / t40 * (-t17 * t29 + t6 * t9);
  }

  real_type
  General::alpha__f_D_1_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t2   = delta__f__XO * y__f__XO + x__f__XO;
    real_type t3   = y__f__XO * Omega__XO;
    real_type t4   = x__f__XO * Omega__XO;
    real_type t7   = -t3 + u__XO + x__f__dot__XO + delta__f__XO * (t4 + v__XO + y__f__dot__XO);
    real_type t8   = t7 * t7;
    real_type t9   = 1.0 / t8;
    real_type t13  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t18  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t3 - u__XO) * delta__f__XO + t4 + v__XO;
    real_type t20  = 1.0 / t8 / t7;
    real_type t25  = t18 * t18;
    real_type t27  = t9 * t25 + 1;
    real_type t32  = t9 * t18;
    real_type t35  = t27 * t27;
    return -1.0 / t27 * (2 * t13 * t20 * t18 + t13 * t9 * delta__f__XO - t9 * t2) + (-2 * t20 * t25 - 2 * delta__f__XO * t32) / t35 * (1.0 / t7 * t2 - t13 * t32);
  }

  real_type
  General::alpha__f_D_1_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t2   = delta__f__XO * y__f__XO + x__f__XO;
    real_type t3   = y__f__XO * Omega__XO;
    real_type t4   = x__f__XO * Omega__XO;
    real_type t7   = -t3 + u__XO + x__f__dot__XO + delta__f__XO * (t4 + v__XO + y__f__dot__XO);
    real_type t8   = t7 * t7;
    real_type t9   = 1.0 / t8;
    real_type t13  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t18  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t3 - u__XO) * delta__f__XO + t4 + v__XO;
    real_type t20  = 1.0 / t8 / t7;
    real_type t26  = t18 * t18;
    real_type t28  = t9 * t26 + 1;
    real_type t33  = t9 * t18;
    real_type t36  = t28 * t28;
    return -1.0 / t28 * (2 * delta__f__XO * t13 * t20 * t18 - delta__f__XO * t9 * t2 - t13 * t9) + (-2 * delta__f__XO * t20 * t26 + 2 * t33) / t36 * (1.0 / t7 * t2 - t13 * t33);
  }

  real_type
  General::alpha__f_D_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + delta__f__XO * (t2 + v__XO + y__f__dot__XO);
    real_type t11  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t12  = t5 * t5;
    real_type t13  = 1.0 / t12;
    real_type t16  = t11 * t11;
    return -1.0 / (t13 * t16 + 1) * (-1.0 / t5 * delta__f__XO - t13 * t11);
  }

  real_type
  General::alpha__f_D_2_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + delta__f__XO * (t2 + v__XO + y__f__dot__XO);
    real_type t6   = t5 * t5;
    real_type t7   = 1.0 / t6;
    real_type t12  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t14  = 1.0 / t6 / t5;
    real_type t18  = t12 * t12;
    real_type t20  = t7 * t18 + 1;
    real_type t25  = t7 * t12;
    real_type t27  = t20 * t20;
    return -1.0 / t20 * (2 * t14 * t12 + 2 * t7 * delta__f__XO) + (-2 * t14 * t18 - 2 * delta__f__XO * t25) / t27 * (-1.0 / t5 * delta__f__XO - t25);
  }

  real_type
  General::alpha__f_D_2_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * delta__f__XO;
    real_type t2   = y__f__XO * Omega__XO;
    real_type t3   = x__f__XO * Omega__XO;
    real_type t6   = -t2 + u__XO + x__f__dot__XO + delta__f__XO * (t3 + v__XO + y__f__dot__XO);
    real_type t7   = t6 * t6;
    real_type t8   = 1.0 / t7;
    real_type t13  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t2 - u__XO) * delta__f__XO + t3 + v__XO;
    real_type t15  = 1.0 / t7 / t6;
    real_type t20  = t13 * t13;
    real_type t22  = t8 * t20 + 1;
    real_type t27  = t8 * t13;
    real_type t29  = t22 * t22;
    return -1.0 / t22 * (2 * delta__f__XO * t15 * t13 + t8 * t1 - t8) + (-2 * delta__f__XO * t15 * t20 + 2 * t27) / t29 * (-1.0 / t6 * delta__f__XO - t27);
  }

  real_type
  General::alpha__f_D_2_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * delta__f__XO;
    real_type t2   = y__f__XO * Omega__XO;
    real_type t3   = x__f__XO * Omega__XO;
    real_type t6   = -t2 + u__XO + x__f__dot__XO + delta__f__XO * (t3 + v__XO + y__f__dot__XO);
    real_type t7   = t6 * t6;
    real_type t8   = 1.0 / t7;
    real_type t15  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t2 - u__XO) * delta__f__XO + t3 + v__XO;
    real_type t17  = 1.0 / t7 / t6;
    real_type t19  = Omega__XO * delta__f__XO;
    real_type t23  = t15 * t15;
    real_type t25  = t8 * t23 + 1;
    real_type t30  = t8 * t15;
    real_type t32  = t25 * t25;
    return -1.0 / t25 * (Omega__XO * t8 * t1 + 2 * t19 * t17 * t15 - t8 * Omega__XO) + (-2 * t19 * t17 * t23 + 2 * Omega__XO * t30) / t32 * (-1.0 / t6 * delta__f__XO - t30);
  }

  real_type
  General::alpha__f_D_2_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + delta__f__XO * (t2 + v__XO + y__f__dot__XO);
    real_type t6   = t5 * t5;
    real_type t7   = 1.0 / t6;
    real_type t13  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t15  = 1.0 / t6 / t5;
    real_type t20  = t13 * t13;
    real_type t22  = t7 * t20 + 1;
    real_type t27  = t7 * t13;
    real_type t29  = t22 * t22;
    return -1.0 / t22 * (-2 * Omega__XO * t15 * t13 - 2 * Omega__XO * t7 * delta__f__XO) + (2 * Omega__XO * t15 * t20 + 2 * Omega__XO * delta__f__XO * t27) / t29 * (-1.0 / t5 * delta__f__XO - t27);
  }

  real_type
  General::alpha__f_D_2_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO + y__f__dot__XO;
    real_type t5   = delta__f__XO * t3 - t1 + u__XO + x__f__dot__XO;
    real_type t6   = 1.0 / t5;
    real_type t7   = t5 * t5;
    real_type t8   = 1.0 / t7;
    real_type t11  = t1 - u__XO - x__f__dot__XO;
    real_type t16  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t18  = 1.0 / t7 / t5;
    real_type t23  = t16 * t16;
    real_type t25  = t8 * t23 + 1;
    real_type t29  = t8 * t16;
    real_type t31  = t25 * t25;
    return -1.0 / t25 * (2 * t3 * t18 * t16 + t3 * t8 * delta__f__XO - t8 * t11 - t6) + (-2 * t3 * t18 * t23 + 2 * t11 * t29) / t31 * (-t6 * delta__f__XO - t29);
  }

  real_type
  General::alpha__f_D_2_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + delta__f__XO * (t2 + v__XO + y__f__dot__XO);
    real_type t6   = t5 * t5;
    real_type t7   = 1.0 / t6;
    real_type t12  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t14  = 1.0 / t6 / t5;
    real_type t18  = t12 * t12;
    real_type t20  = t7 * t18 + 1;
    real_type t25  = t7 * t12;
    real_type t27  = t20 * t20;
    return -1.0 / t20 * (2 * t14 * t12 + 2 * t7 * delta__f__XO) + (-2 * t14 * t18 - 2 * delta__f__XO * t25) / t27 * (-1.0 / t5 * delta__f__XO - t25);
  }

  real_type
  General::alpha__f_D_2_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * delta__f__XO;
    real_type t2   = y__f__XO * Omega__XO;
    real_type t3   = x__f__XO * Omega__XO;
    real_type t6   = -t2 + u__XO + x__f__dot__XO + delta__f__XO * (t3 + v__XO + y__f__dot__XO);
    real_type t7   = t6 * t6;
    real_type t8   = 1.0 / t7;
    real_type t13  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t2 - u__XO) * delta__f__XO + t3 + v__XO;
    real_type t15  = 1.0 / t7 / t6;
    real_type t20  = t13 * t13;
    real_type t22  = t8 * t20 + 1;
    real_type t27  = t8 * t13;
    real_type t29  = t22 * t22;
    return -1.0 / t22 * (2 * delta__f__XO * t15 * t13 + t8 * t1 - t8) + (-2 * delta__f__XO * t15 * t20 + 2 * t27) / t29 * (-1.0 / t6 * delta__f__XO - t27);
  }

  real_type
  General::alpha__f_D_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + delta__f__XO * (t2 + v__XO + y__f__dot__XO);
    real_type t10  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t11  = t5 * t5;
    real_type t12  = 1.0 / t11;
    real_type t16  = t10 * t10;
    return -1.0 / (t12 * t16 + 1) * (1.0 / t5 - delta__f__XO * t12 * t10);
  }

  real_type
  General::alpha__f_D_3_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + delta__f__XO * (t2 + v__XO + y__f__dot__XO);
    real_type t6   = t5 * t5;
    real_type t7   = 1.0 / t6;
    real_type t12  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t14  = 1.0 / t6 / t5;
    real_type t16  = delta__f__XO * delta__f__XO;
    real_type t20  = t12 * t12;
    real_type t22  = t7 * t20 + 1;
    real_type t26  = t7 * t12;
    real_type t29  = t22 * t22;
    return -1.0 / t22 * (2 * t16 * t14 * t12 - 2 * t7 * delta__f__XO) + (-2 * delta__f__XO * t14 * t20 + 2 * t26) / t29 * (1.0 / t5 - delta__f__XO * t26);
  }

  real_type
  General::alpha__f_D_3_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + delta__f__XO * (t2 + v__XO + y__f__dot__XO);
    real_type t6   = t5 * t5;
    real_type t7   = 1.0 / t6;
    real_type t13  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t15  = 1.0 / t6 / t5;
    real_type t17  = delta__f__XO * delta__f__XO;
    real_type t22  = t13 * t13;
    real_type t24  = t7 * t22 + 1;
    real_type t28  = t7 * t13;
    real_type t31  = t24 * t24;
    return -1.0 / t24 * (2 * Omega__XO * t17 * t15 * t13 - 2 * Omega__XO * t7 * delta__f__XO) + (-2 * Omega__XO * delta__f__XO * t15 * t22 + 2 * Omega__XO * t28) / t31 * (1.0 / t5 - delta__f__XO * t28);
  }

  real_type
  General::alpha__f_D_3_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + delta__f__XO * (t2 + v__XO + y__f__dot__XO);
    real_type t6   = t5 * t5;
    real_type t7   = 1.0 / t6;
    real_type t9   = delta__f__XO * delta__f__XO;
    real_type t15  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t17  = 1.0 / t6 / t5;
    real_type t19  = Omega__XO * delta__f__XO;
    real_type t23  = t15 * t15;
    real_type t25  = t7 * t23 + 1;
    real_type t29  = t7 * t15;
    real_type t32  = t25 * t25;
    return -1.0 / t25 * (-2 * t19 * t17 * t15 - Omega__XO * t7 * t9 + t7 * Omega__XO) + (2 * Omega__XO * t17 * t23 + 2 * t19 * t29) / t32 * (1.0 / t5 - delta__f__XO * t29);
  }

  real_type
  General::alpha__f_D_3_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO + y__f__dot__XO;
    real_type t4   = delta__f__XO * t3;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + t4;
    real_type t6   = t5 * t5;
    real_type t7   = 1.0 / t6;
    real_type t9   = t1 - u__XO - x__f__dot__XO;
    real_type t15  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t17  = 1.0 / t6 / t5;
    real_type t21  = t7 * t15;
    real_type t23  = t15 * t15;
    real_type t25  = t7 * t23 + 1;
    real_type t31  = t25 * t25;
    return -1.0 / t25 * (2 * t4 * t17 * t15 - delta__f__XO * t7 * t9 - t3 * t7 - t21) + (-2 * t3 * t17 * t23 + 2 * t9 * t21) / t31 * (1.0 / t5 - delta__f__XO * t21);
  }

  real_type
  General::alpha__f_D_3_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * delta__f__XO;
    real_type t2   = y__f__XO * Omega__XO;
    real_type t3   = x__f__XO * Omega__XO;
    real_type t6   = -t2 + u__XO + x__f__dot__XO + delta__f__XO * (t3 + v__XO + y__f__dot__XO);
    real_type t7   = t6 * t6;
    real_type t8   = 1.0 / t7;
    real_type t13  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t2 - u__XO) * delta__f__XO + t3 + v__XO;
    real_type t15  = 1.0 / t7 / t6;
    real_type t20  = t13 * t13;
    real_type t22  = t8 * t20 + 1;
    real_type t27  = delta__f__XO * t8 * t13;
    real_type t29  = t22 * t22;
    return -1.0 / t22 * (2 * delta__f__XO * t15 * t13 + t8 * t1 - t8) + (-2 * t15 * t20 - 2 * t27) / t29 * (1.0 / t6 - t27);
  }

  real_type
  General::alpha__f_D_3_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + delta__f__XO * (t2 + v__XO + y__f__dot__XO);
    real_type t6   = t5 * t5;
    real_type t7   = 1.0 / t6;
    real_type t12  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t14  = 1.0 / t6 / t5;
    real_type t16  = delta__f__XO * delta__f__XO;
    real_type t20  = t12 * t12;
    real_type t22  = t7 * t20 + 1;
    real_type t26  = t7 * t12;
    real_type t29  = t22 * t22;
    return -1.0 / t22 * (2 * t16 * t14 * t12 - 2 * t7 * delta__f__XO) + (-2 * delta__f__XO * t14 * t20 + 2 * t26) / t29 * (1.0 / t5 - delta__f__XO * t26);
  }

  real_type
  General::alpha__f_D_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + delta__f__XO * (t2 + v__XO + y__f__dot__XO);
    real_type t11  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t12  = t5 * t5;
    real_type t13  = 1.0 / t12;
    real_type t18  = t11 * t11;
    return -1.0 / (t13 * t18 + 1) * (1.0 / t5 * Omega__XO - Omega__XO * delta__f__XO * t13 * t11);
  }

  real_type
  General::alpha__f_D_4_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = Omega__XO * Omega__XO;
    real_type t2   = y__f__XO * Omega__XO;
    real_type t3   = x__f__XO * Omega__XO;
    real_type t6   = -t2 + u__XO + x__f__dot__XO + delta__f__XO * (t3 + v__XO + y__f__dot__XO);
    real_type t7   = t6 * t6;
    real_type t8   = 1.0 / t7;
    real_type t14  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t2 - u__XO) * delta__f__XO + t3 + v__XO;
    real_type t16  = 1.0 / t7 / t6;
    real_type t18  = delta__f__XO * delta__f__XO;
    real_type t23  = t14 * t14;
    real_type t25  = t8 * t23 + 1;
    real_type t30  = t8 * t14;
    real_type t31  = Omega__XO * delta__f__XO;
    real_type t34  = t25 * t25;
    return -1.0 / t25 * (2 * t18 * t1 * t16 * t14 - 2 * delta__f__XO * t8 * t1) + (-2 * t31 * t16 * t23 + 2 * Omega__XO * t30) / t34 * (1.0 / t6 * Omega__XO - t31 * t30);
  }

  real_type
  General::alpha__f_D_4_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = Omega__XO * Omega__XO;
    real_type t2   = y__f__XO * Omega__XO;
    real_type t3   = x__f__XO * Omega__XO;
    real_type t6   = -t2 + u__XO + x__f__dot__XO + delta__f__XO * (t3 + v__XO + y__f__dot__XO);
    real_type t7   = t6 * t6;
    real_type t8   = 1.0 / t7;
    real_type t10  = delta__f__XO * delta__f__XO;
    real_type t16  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t2 - u__XO) * delta__f__XO + t3 + v__XO;
    real_type t18  = 1.0 / t7 / t6;
    real_type t24  = t16 * t16;
    real_type t26  = t8 * t24 + 1;
    real_type t33  = Omega__XO * delta__f__XO * t8 * t16;
    real_type t35  = t26 * t26;
    return -1.0 / t26 * (-2 * delta__f__XO * t1 * t18 * t16 - t8 * t10 * t1 + t8 * t1) + (2 * Omega__XO * t18 * t24 + 2 * t33) / t35 * (1.0 / t6 * Omega__XO - t33);
  }

  real_type
  General::alpha__f_D_4_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO + y__f__dot__XO;
    real_type t5   = delta__f__XO * t3 - t1 + u__XO + x__f__dot__XO;
    real_type t6   = t5 * t5;
    real_type t7   = 1.0 / t6;
    real_type t10  = t1 - u__XO - x__f__dot__XO;
    real_type t12  = Omega__XO * delta__f__XO;
    real_type t17  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t19  = 1.0 / t6 / t5;
    real_type t24  = t7 * t17;
    real_type t27  = t17 * t17;
    real_type t29  = t7 * t27 + 1;
    real_type t36  = t29 * t29;
    return -1.0 / t29 * (2 * t3 * t12 * t19 * t17 - t12 * t7 * t10 - t3 * t7 * Omega__XO - Omega__XO * t24) + (-2 * t3 * t19 * t27 + 2 * t10 * t24) / t36 * (1.0 / t5 * Omega__XO - t12 * t24);
  }

  real_type
  General::alpha__f_D_4_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * delta__f__XO;
    real_type t2   = y__f__XO * Omega__XO;
    real_type t3   = x__f__XO * Omega__XO;
    real_type t6   = -t2 + u__XO + x__f__dot__XO + delta__f__XO * (t3 + v__XO + y__f__dot__XO);
    real_type t7   = t6 * t6;
    real_type t8   = 1.0 / t7;
    real_type t15  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t2 - u__XO) * delta__f__XO + t3 + v__XO;
    real_type t17  = 1.0 / t7 / t6;
    real_type t19  = Omega__XO * delta__f__XO;
    real_type t23  = t15 * t15;
    real_type t25  = t8 * t23 + 1;
    real_type t30  = t8 * t15;
    real_type t33  = t25 * t25;
    return -1.0 / t25 * (Omega__XO * t8 * t1 + 2 * t19 * t17 * t15 - t8 * Omega__XO) + (-2 * t17 * t23 - 2 * delta__f__XO * t30) / t33 * (1.0 / t6 * Omega__XO - t19 * t30);
  }

  real_type
  General::alpha__f_D_4_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + delta__f__XO * (t2 + v__XO + y__f__dot__XO);
    real_type t6   = t5 * t5;
    real_type t7   = 1.0 / t6;
    real_type t13  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t15  = 1.0 / t6 / t5;
    real_type t17  = delta__f__XO * delta__f__XO;
    real_type t22  = t13 * t13;
    real_type t24  = t7 * t22 + 1;
    real_type t29  = t7 * t13;
    real_type t33  = t24 * t24;
    return -1.0 / t24 * (2 * Omega__XO * t17 * t15 * t13 - 2 * Omega__XO * t7 * delta__f__XO) + (-2 * delta__f__XO * t15 * t22 + 2 * t29) / t33 * (1.0 / t5 * Omega__XO - Omega__XO * delta__f__XO * t29);
  }

  real_type
  General::alpha__f_D_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t2   = y__f__XO * Omega__XO;
    real_type t3   = x__f__XO * Omega__XO;
    real_type t6   = -t2 + u__XO + x__f__dot__XO + delta__f__XO * (t3 + v__XO + y__f__dot__XO);
    real_type t12  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t2 - u__XO) * delta__f__XO + t3 + v__XO;
    real_type t13  = t6 * t6;
    real_type t14  = 1.0 / t13;
    real_type t18  = t12 * t12;
    return -1.0 / (t14 * t18 + 1) * (1.0 / t6 * delta__f__XO * Omega__XO + Omega__XO * t14 * t12);
  }

  real_type
  General::alpha__f_D_5_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = Omega__XO * Omega__XO;
    real_type t2   = y__f__XO * Omega__XO;
    real_type t3   = x__f__XO * Omega__XO;
    real_type t6   = -t2 + u__XO + x__f__dot__XO + delta__f__XO * (t3 + v__XO + y__f__dot__XO);
    real_type t7   = t6 * t6;
    real_type t8   = 1.0 / t7;
    real_type t14  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t2 - u__XO) * delta__f__XO + t3 + v__XO;
    real_type t16  = 1.0 / t7 / t6;
    real_type t21  = t14 * t14;
    real_type t23  = t8 * t21 + 1;
    real_type t26  = delta__f__XO * Omega__XO;
    real_type t29  = t8 * t14;
    real_type t32  = t23 * t23;
    return -1.0 / t23 * (2 * t1 * t16 * t14 + 2 * delta__f__XO * t8 * t1) + (2 * Omega__XO * t16 * t21 + 2 * t26 * t29) / t32 * (1.0 / t6 * t26 + Omega__XO * t29);
  }

  real_type
  General::alpha__f_D_5_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO + y__f__dot__XO;
    real_type t5   = delta__f__XO * t3 - t1 + u__XO + x__f__dot__XO;
    real_type t6   = 1.0 / t5;
    real_type t8   = delta__f__XO * Omega__XO;
    real_type t9   = t5 * t5;
    real_type t10  = 1.0 / t9;
    real_type t13  = t1 - u__XO - x__f__dot__XO;
    real_type t19  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t21  = 1.0 / t9 / t5;
    real_type t27  = t19 * t19;
    real_type t29  = t10 * t27 + 1;
    real_type t33  = t10 * t19;
    real_type t36  = t29 * t29;
    return -1.0 / t29 * (-2 * t3 * Omega__XO * t21 * t19 + Omega__XO * t10 * t13 - t3 * t10 * t8 + t6 * Omega__XO) + (-2 * t3 * t21 * t27 + 2 * t13 * t33) / t36 * (Omega__XO * t33 + t6 * t8);
  }

  real_type
  General::alpha__f_D_5_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + delta__f__XO * (t2 + v__XO + y__f__dot__XO);
    real_type t6   = t5 * t5;
    real_type t7   = 1.0 / t6;
    real_type t13  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t15  = 1.0 / t6 / t5;
    real_type t20  = t13 * t13;
    real_type t22  = t7 * t20 + 1;
    real_type t28  = t7 * t13;
    real_type t31  = t22 * t22;
    return -1.0 / t22 * (-2 * Omega__XO * t15 * t13 - 2 * Omega__XO * t7 * delta__f__XO) + (-2 * t15 * t20 - 2 * delta__f__XO * t28) / t31 * (1.0 / t5 * delta__f__XO * Omega__XO + Omega__XO * t28);
  }

  real_type
  General::alpha__f_D_5_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + delta__f__XO * (t2 + v__XO + y__f__dot__XO);
    real_type t6   = t5 * t5;
    real_type t7   = 1.0 / t6;
    real_type t9   = delta__f__XO * delta__f__XO;
    real_type t15  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t17  = 1.0 / t6 / t5;
    real_type t19  = delta__f__XO * Omega__XO;
    real_type t23  = t15 * t15;
    real_type t25  = t7 * t23 + 1;
    real_type t30  = t7 * t15;
    real_type t33  = t25 * t25;
    return -1.0 / t25 * (-2 * t19 * t17 * t15 - Omega__XO * t7 * t9 + t7 * Omega__XO) + (-2 * delta__f__XO * t17 * t23 + 2 * t30) / t33 * (1.0 / t5 * t19 + Omega__XO * t30);
  }

  real_type
  General::alpha__f_D_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t3   = x__f__XO * Omega__XO;
    real_type t4   = t3 + v__XO + y__f__dot__XO;
    real_type t6   = delta__f__XO * t4 - t1 + u__XO + x__f__dot__XO;
    real_type t12  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t3 + v__XO;
    real_type t13  = t6 * t6;
    real_type t14  = 1.0 / t13;
    real_type t18  = t12 * t12;
    return -1.0 / (t14 * t18 + 1) * (1.0 / t6 * (t1 - u__XO - x__f__dot__XO) - t4 * t14 * t12);
  }

  real_type
  General::alpha__f_D_6_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = t1 - u__XO - x__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO;
    real_type t4   = t3 + v__XO + y__f__dot__XO;
    real_type t6   = delta__f__XO * t4 - t1 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t8   = 1.0 / t7;
    real_type t14  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t3 + v__XO;
    real_type t16  = 1.0 / t7 / t6;
    real_type t18  = t4 * t4;
    real_type t22  = t14 * t14;
    real_type t24  = t8 * t22 + 1;
    real_type t29  = t8 * t14;
    real_type t32  = t24 * t24;
    return -1.0 / t24 * (2 * t18 * t16 * t14 - 2 * t4 * t8 * t2) + (-2 * t4 * t16 * t22 + 2 * t2 * t29) / t32 * (1.0 / t6 * t2 - t4 * t29);
  }

  real_type
  General::alpha__f_D_6_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO + y__f__dot__XO;
    real_type t5   = delta__f__XO * t3 - t1 + u__XO + x__f__dot__XO;
    real_type t6   = 1.0 / t5;
    real_type t7   = t5 * t5;
    real_type t8   = 1.0 / t7;
    real_type t11  = t1 - u__XO - x__f__dot__XO;
    real_type t16  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t18  = 1.0 / t7 / t5;
    real_type t23  = t16 * t16;
    real_type t25  = t8 * t23 + 1;
    real_type t29  = t8 * t16;
    real_type t32  = t25 * t25;
    return -1.0 / t25 * (2 * t3 * t18 * t16 + t3 * t8 * delta__f__XO - t8 * t11 - t6) + (-2 * t18 * t23 - 2 * delta__f__XO * t29) / t32 * (t6 * t11 - t3 * t29);
  }

  real_type
  General::alpha__f_D_6_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO + y__f__dot__XO;
    real_type t4   = delta__f__XO * t3;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + t4;
    real_type t6   = t5 * t5;
    real_type t7   = 1.0 / t6;
    real_type t9   = t1 - u__XO - x__f__dot__XO;
    real_type t15  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t17  = 1.0 / t6 / t5;
    real_type t21  = t7 * t15;
    real_type t23  = t15 * t15;
    real_type t25  = t7 * t23 + 1;
    real_type t32  = t25 * t25;
    return -1.0 / t25 * (2 * t4 * t17 * t15 - delta__f__XO * t7 * t9 - t3 * t7 - t21) + (-2 * delta__f__XO * t17 * t23 + 2 * t21) / t32 * (1.0 / t5 * t9 - t3 * t21);
  }

  real_type
  General::alpha__f_D_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + delta__f__XO * (t2 + v__XO + y__f__dot__XO);
    real_type t11  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t12  = t5 * t5;
    real_type t13  = 1.0 / t12;
    real_type t16  = t11 * t11;
    return -1.0 / (t13 * t16 + 1) * (-1.0 / t5 * delta__f__XO - t13 * t11);
  }

  real_type
  General::alpha__f_D_7_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + delta__f__XO * (t2 + v__XO + y__f__dot__XO);
    real_type t6   = t5 * t5;
    real_type t7   = 1.0 / t6;
    real_type t12  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t14  = 1.0 / t6 / t5;
    real_type t18  = t12 * t12;
    real_type t20  = t7 * t18 + 1;
    real_type t25  = t7 * t12;
    real_type t27  = t20 * t20;
    return -1.0 / t20 * (2 * t14 * t12 + 2 * t7 * delta__f__XO) + (-2 * t14 * t18 - 2 * delta__f__XO * t25) / t27 * (-1.0 / t5 * delta__f__XO - t25);
  }

  real_type
  General::alpha__f_D_7_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * delta__f__XO;
    real_type t2   = y__f__XO * Omega__XO;
    real_type t3   = x__f__XO * Omega__XO;
    real_type t6   = -t2 + u__XO + x__f__dot__XO + delta__f__XO * (t3 + v__XO + y__f__dot__XO);
    real_type t7   = t6 * t6;
    real_type t8   = 1.0 / t7;
    real_type t13  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t2 - u__XO) * delta__f__XO + t3 + v__XO;
    real_type t15  = 1.0 / t7 / t6;
    real_type t20  = t13 * t13;
    real_type t22  = t8 * t20 + 1;
    real_type t27  = t8 * t13;
    real_type t29  = t22 * t22;
    return -1.0 / t22 * (2 * delta__f__XO * t15 * t13 + t8 * t1 - t8) + (-2 * delta__f__XO * t15 * t20 + 2 * t27) / t29 * (-1.0 / t6 * delta__f__XO - t27);
  }

  real_type
  General::alpha__f_D_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + delta__f__XO * (t2 + v__XO + y__f__dot__XO);
    real_type t10  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t11  = t5 * t5;
    real_type t12  = 1.0 / t11;
    real_type t16  = t10 * t10;
    return -1.0 / (t12 * t16 + 1) * (1.0 / t5 - delta__f__XO * t12 * t10);
  }

  real_type
  General::alpha__f_D_8_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t5   = -t1 + u__XO + x__f__dot__XO + delta__f__XO * (t2 + v__XO + y__f__dot__XO);
    real_type t6   = t5 * t5;
    real_type t7   = 1.0 / t6;
    real_type t12  = -x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t1 - u__XO) * delta__f__XO + t2 + v__XO;
    real_type t14  = 1.0 / t6 / t5;
    real_type t16  = delta__f__XO * delta__f__XO;
    real_type t20  = t12 * t12;
    real_type t22  = t7 * t20 + 1;
    real_type t26  = t7 * t12;
    real_type t29  = t22 * t22;
    return -1.0 / t22 * (2 * t16 * t14 * t12 - 2 * t7 * delta__f__XO) + (-2 * delta__f__XO * t14 * t20 + 2 * t26) / t29 * (1.0 / t5 - delta__f__XO * t26);
  }

  real_type
  General::lambda__r( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t4   = ModelPars[119];
    real_type t6   = cos(phi__XO);
    return (-x__r__dot__XO - t6 * omega__r__XO * t4 + (-ModelPars[117] + t4) * omega__r__XO - t1 + u__XO) / (t1 - u__XO + x__r__dot__XO);
  }

  real_type
  General::lambda__r_D_1( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t5   = ModelPars[119];
    real_type t7   = cos(phi__XO);
    return -y__r__XO * (-x__r__dot__XO - t7 * omega__r__XO * t5 + (-ModelPars[117] + t5) * omega__r__XO - t1 + u__XO) / t3 - y__r__XO / t2;
  }

  real_type
  General::lambda__r_D_1_1( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[119];
    real_type t8   = cos(phi__XO);
    real_type t15  = y__r__XO * y__r__XO;
    return 2 * t15 * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[117] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 * t15 / t3;
  }

  real_type
  General::lambda__r_D_1_2( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t7   = sin(phi__XO);
    return -y__r__XO * t7 * omega__r__XO * ModelPars[119] / t3;
  }

  real_type
  General::lambda__r_D_1_3( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[119];
    real_type t8   = cos(phi__XO);
    return -2 * y__r__XO * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[117] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 - 2 * y__r__XO / t3;
  }

  real_type
  General::lambda__r_D_1_4( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[119];
    real_type t8   = cos(phi__XO);
    real_type t13  = -x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[117] + t6) * omega__r__XO - t1 + u__XO;
    real_type t17  = 1.0 / t3;
    return 2 * t1 * t13 / t3 / t2 + 2 * y__r__XO * Omega__XO * t17 - t13 * t17 - 1.0 / t2;
  }

  real_type
  General::lambda__r_D_1_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t5   = ModelPars[119];
    real_type t6   = cos(phi__XO);
    return -y__r__XO * (-t6 * t5 + t5 - ModelPars[117]) / t3;
  }

  real_type
  General::lambda__r_D_1_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[119];
    real_type t8   = cos(phi__XO);
    return 2 * y__r__XO * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[117] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 * y__r__XO / t3;
  }

  real_type
  General::lambda__r_D_2( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t6   = sin(phi__XO);
    return t6 * omega__r__XO / (y__r__XO * Omega__XO - u__XO + x__r__dot__XO) * ModelPars[119];
  }

  real_type
  General::lambda__r_D_2_2( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t6   = cos(phi__XO);
    return t6 * omega__r__XO / (y__r__XO * Omega__XO - u__XO + x__r__dot__XO) * ModelPars[119];
  }

  real_type
  General::lambda__r_D_2_3( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t7   = sin(phi__XO);
    return t7 * omega__r__XO * ModelPars[119] / t3;
  }

  real_type
  General::lambda__r_D_2_4( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t7   = sin(phi__XO);
    return -Omega__XO * t7 * omega__r__XO * ModelPars[119] / t3;
  }

  real_type
  General::lambda__r_D_2_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t6   = sin(phi__XO);
    return t6 * ModelPars[119] / (y__r__XO * Omega__XO - u__XO + x__r__dot__XO);
  }

  real_type
  General::lambda__r_D_2_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t7   = sin(phi__XO);
    return -t7 * omega__r__XO * ModelPars[119] / t3;
  }

  real_type
  General::lambda__r_D_3( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t5   = ModelPars[119];
    real_type t7   = cos(phi__XO);
    return (-x__r__dot__XO - t7 * omega__r__XO * t5 + (-ModelPars[117] + t5) * omega__r__XO - t1 + u__XO) / t3 + 1.0 / t2;
  }

  real_type
  General::lambda__r_D_3_3( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[119];
    real_type t8   = cos(phi__XO);
    return 2 * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[117] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 / t3;
  }

  real_type
  General::lambda__r_D_3_4( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[119];
    real_type t8   = cos(phi__XO);
    return -2 * Omega__XO * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[117] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 - 2 * Omega__XO / t3;
  }

  real_type
  General::lambda__r_D_3_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t5   = ModelPars[119];
    real_type t6   = cos(phi__XO);
    return (-t6 * t5 + t5 - ModelPars[117]) / t3;
  }

  real_type
  General::lambda__r_D_3_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[119];
    real_type t8   = cos(phi__XO);
    return -2 * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[117] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 - 2 / t3;
  }

  real_type
  General::lambda__r_D_4( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t5   = ModelPars[119];
    real_type t7   = cos(phi__XO);
    return -Omega__XO * (-x__r__dot__XO - t7 * omega__r__XO * t5 + (-ModelPars[117] + t5) * omega__r__XO - t1 + u__XO) / t3 - Omega__XO / t2;
  }

  real_type
  General::lambda__r_D_4_4( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[119];
    real_type t8   = cos(phi__XO);
    real_type t15  = Omega__XO * Omega__XO;
    return 2 * t15 * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[117] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 * t15 / t3;
  }

  real_type
  General::lambda__r_D_4_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t5   = ModelPars[119];
    real_type t6   = cos(phi__XO);
    return -Omega__XO * (-t6 * t5 + t5 - ModelPars[117]) / t3;
  }

  real_type
  General::lambda__r_D_4_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[119];
    real_type t8   = cos(phi__XO);
    return 2 * Omega__XO * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[117] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 * Omega__XO / t3;
  }

  real_type
  General::lambda__r_D_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t4   = ModelPars[119];
    real_type t5   = cos(phi__XO);
    return (-t5 * t4 + t4 - ModelPars[117]) / (y__r__XO * Omega__XO - u__XO + x__r__dot__XO);
  }

  real_type
  General::lambda__r_D_5_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    return 0;
  }

  real_type
  General::lambda__r_D_5_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t5   = ModelPars[119];
    real_type t6   = cos(phi__XO);
    return -(-t6 * t5 + t5 - ModelPars[117]) / t3;
  }

  real_type
  General::lambda__r_D_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t5   = ModelPars[119];
    real_type t7   = cos(phi__XO);
    return -(-x__r__dot__XO - t7 * omega__r__XO * t5 + (-ModelPars[117] + t5) * omega__r__XO - t1 + u__XO) / t3 - 1.0 / t2;
  }

  real_type
  General::lambda__r_D_6_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[119];
    real_type t8   = cos(phi__XO);
    return 2 * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[117] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 / t3;
  }

  real_type
  General::lambda__f( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t8   = ModelPars[118];
    real_type t10  = cos(phi__f__XO);
    return (-x__f__dot__XO - t1 + t10 * omega__f__XO * t8 - t3 * delta__f__XO + t5 + (ModelPars[116] - t8) * omega__f__XO - u__XO) / (t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO);
  }

  real_type
  General::lambda__f_D_1( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[118];
    real_type t11  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * x__f__XO - y__f__XO;
    return -t21 * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[116] - t9) * omega__f__XO - u__XO) / t7 - t21 / t6;
  }

  real_type
  General::lambda__f_D_1_1( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t23  = t22 * t22;
    return 2 * t23 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * t22 / t7;
  }

  real_type
  General::lambda__f_D_1_2( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return (delta__f__XO * x__f__XO - y__f__XO) * t11 * omega__f__XO * ModelPars[118] / t7;
  }

  real_type
  General::lambda__f_D_1_3( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t25  = 1.0 / t7;
    return 2 * t22 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * t25;
  }

  real_type
  General::lambda__f_D_1_4( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t26  = 1.0 / t7;
    return 2 * delta__f__XO * t22 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * delta__f__XO * t26;
  }

  real_type
  General::lambda__f_D_1_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO;
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t27  = 1.0 / t7;
    return 2 * delta__f__XO * Omega__XO * t22 * t19 / t7 / t6 + 2 * t22 * delta__f__XO * Omega__XO * t27 - delta__f__XO * t19 * t27 - delta__f__XO / t6;
  }

  real_type
  General::lambda__f_D_1_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO;
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t26  = 1.0 / t7;
    return -2 * Omega__XO * t22 * t19 / t7 / t6 - 2 * t22 * Omega__XO * t26 + t19 * t26 + 1.0 / t6;
  }

  real_type
  General::lambda__f_D_1_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO;
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t23  = t2 + v__XO + y__f__dot__XO;
    real_type t27  = 1.0 / t7;
    return 2 * t23 * t22 * t19 / t7 / t6 + 2 * t22 * t23 * t27 - x__f__XO * t19 * t27 - x__f__XO / t6;
  }

  real_type
  General::lambda__f_D_1_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[118];
    real_type t10  = cos(phi__f__XO);
    return -(delta__f__XO * x__f__XO - y__f__XO) * (t10 * t9 - t9 + ModelPars[116]) / t7;
  }

  real_type
  General::lambda__f_D_1_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t25  = 1.0 / t7;
    return 2 * t22 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * t25;
  }

  real_type
  General::lambda__f_D_1_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t26  = 1.0 / t7;
    return 2 * delta__f__XO * t22 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * delta__f__XO * t26;
  }

  real_type
  General::lambda__f_D_2( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t10  = sin(phi__f__XO);
    return -t10 * omega__f__XO / (x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO) * ModelPars[118];
  }

  real_type
  General::lambda__f_D_2_2( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t10  = cos(phi__f__XO);
    return -t10 * omega__f__XO / (x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO) * ModelPars[118];
  }

  real_type
  General::lambda__f_D_2_3( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return t11 * omega__f__XO * ModelPars[118] / t7;
  }

  real_type
  General::lambda__f_D_2_4( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return delta__f__XO * t11 * omega__f__XO * ModelPars[118] / t7;
  }

  real_type
  General::lambda__f_D_2_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t12  = sin(phi__f__XO);
    return delta__f__XO * Omega__XO * t12 * omega__f__XO * ModelPars[118] / t7;
  }

  real_type
  General::lambda__f_D_2_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return -Omega__XO * t11 * omega__f__XO * ModelPars[118] / t7;
  }

  real_type
  General::lambda__f_D_2_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t2   = x__f__XO * Omega__XO;
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + (t2 + v__XO) * delta__f__XO - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return (t2 + v__XO + y__f__dot__XO) * t11 * omega__f__XO * ModelPars[118] / t7;
  }

  real_type
  General::lambda__f_D_2_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t10  = sin(phi__f__XO);
    return -t10 * ModelPars[118] / (x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO);
  }

  real_type
  General::lambda__f_D_2_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return t11 * omega__f__XO * ModelPars[118] / t7;
  }

  real_type
  General::lambda__f_D_2_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return delta__f__XO * t11 * omega__f__XO * ModelPars[118] / t7;
  }

  real_type
  General::lambda__f_D_3( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[118];
    real_type t11  = cos(phi__f__XO);
    return -(-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[116] - t9) * omega__f__XO - u__XO) / t7 - 1.0 / t6;
  }

  real_type
  General::lambda__f_D_3_3( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    return 2 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 / t7;
  }

  real_type
  General::lambda__f_D_3_4( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    return 2 * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO / t7;
  }

  real_type
  General::lambda__f_D_3_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    return 2 * Omega__XO * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO * Omega__XO / t7;
  }

  real_type
  General::lambda__f_D_3_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    return -2 * Omega__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * Omega__XO / t7;
  }

  real_type
  General::lambda__f_D_3_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t24  = 1.0 / t7;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 * t24;
  }

  real_type
  General::lambda__f_D_3_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[118];
    real_type t10  = cos(phi__f__XO);
    return -(t10 * t9 - t9 + ModelPars[116]) / t7;
  }

  real_type
  General::lambda__f_D_3_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    return 2 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 / t7;
  }

  real_type
  General::lambda__f_D_3_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    return 2 * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO / t7;
  }

  real_type
  General::lambda__f_D_4( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[118];
    real_type t11  = cos(phi__f__XO);
    return -delta__f__XO * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[116] - t9) * omega__f__XO - u__XO) / t7 - delta__f__XO / t6;
  }

  real_type
  General::lambda__f_D_4_4( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * delta__f__XO;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 / t7;
  }

  real_type
  General::lambda__f_D_4_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * delta__f__XO;
    return 2 * Omega__XO * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 * Omega__XO / t7;
  }

  real_type
  General::lambda__f_D_4_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    return -2 * Omega__XO * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * delta__f__XO * Omega__XO / t7;
  }

  real_type
  General::lambda__f_D_4_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO;
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t25  = 1.0 / t7;
    return 2 * t21 * delta__f__XO * t19 / t7 / t6 + 2 * delta__f__XO * t21 * t25 - t19 * t25 - 1.0 / t6;
  }

  real_type
  General::lambda__f_D_4_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[118];
    real_type t10  = cos(phi__f__XO);
    return -delta__f__XO * (t10 * t9 - t9 + ModelPars[116]) / t7;
  }

  real_type
  General::lambda__f_D_4_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    return 2 * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO / t7;
  }

  real_type
  General::lambda__f_D_4_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * delta__f__XO;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 / t7;
  }

  real_type
  General::lambda__f_D_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[118];
    real_type t11  = cos(phi__f__XO);
    return -Omega__XO * delta__f__XO * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[116] - t9) * omega__f__XO - u__XO) / t7 - delta__f__XO * Omega__XO / t6;
  }

  real_type
  General::lambda__f_D_5_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = Omega__XO * Omega__XO;
    real_type t22  = delta__f__XO * delta__f__XO;
    return 2 * t22 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * t21 / t7;
  }

  real_type
  General::lambda__f_D_5_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = Omega__XO * Omega__XO;
    return -2 * delta__f__XO * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * delta__f__XO * t21 / t7;
  }

  real_type
  General::lambda__f_D_5_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO;
    real_type t21  = delta__f__XO * Omega__XO;
    real_type t22  = t2 + v__XO + y__f__dot__XO;
    real_type t26  = 1.0 / t7;
    return 2 * t22 * t21 * t19 / t7 / t6 + t21 * t22 * t26 - Omega__XO * t19 * t26 + t22 * delta__f__XO * Omega__XO * t26 - Omega__XO / t6;
  }

  real_type
  General::lambda__f_D_5_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[118];
    real_type t10  = cos(phi__f__XO);
    return -Omega__XO * delta__f__XO * (t10 * t9 - t9 + ModelPars[116]) / t7;
  }

  real_type
  General::lambda__f_D_5_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    return 2 * Omega__XO * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO * Omega__XO / t7;
  }

  real_type
  General::lambda__f_D_5_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * delta__f__XO;
    return 2 * Omega__XO * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 * Omega__XO / t7;
  }

  real_type
  General::lambda__f_D_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[118];
    real_type t11  = cos(phi__f__XO);
    return Omega__XO * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[116] - t9) * omega__f__XO - u__XO) / t7 + Omega__XO / t6;
  }

  real_type
  General::lambda__f_D_6_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = Omega__XO * Omega__XO;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 / t7;
  }

  real_type
  General::lambda__f_D_6_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t25  = 1.0 / t7;
    return -2 * t21 * Omega__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * Omega__XO * t21 * t25;
  }

  real_type
  General::lambda__f_D_6_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[118];
    real_type t10  = cos(phi__f__XO);
    return Omega__XO * (t10 * t9 - t9 + ModelPars[116]) / t7;
  }

  real_type
  General::lambda__f_D_6_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    return -2 * Omega__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * Omega__XO / t7;
  }

  real_type
  General::lambda__f_D_6_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    return -2 * Omega__XO * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * delta__f__XO * Omega__XO / t7;
  }

  real_type
  General::lambda__f_D_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[118];
    real_type t11  = cos(phi__f__XO);
    real_type t20  = t2 + v__XO + y__f__dot__XO;
    return -t20 * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[116] - t9) * omega__f__XO - u__XO) / t7 - t20 / t6;
  }

  real_type
  General::lambda__f_D_7_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t22  = t21 * t21;
    return 2 * t22 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 * t21 / t7;
  }

  real_type
  General::lambda__f_D_7_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t2   = x__f__XO * Omega__XO;
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + (t2 + v__XO) * delta__f__XO - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[118];
    real_type t10  = cos(phi__f__XO);
    return -(t2 + v__XO + y__f__dot__XO) * (t10 * t9 - t9 + ModelPars[116]) / t7;
  }

  real_type
  General::lambda__f_D_7_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t24  = 1.0 / t7;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 * t24;
  }

  real_type
  General::lambda__f_D_7_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO;
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t25  = 1.0 / t7;
    return 2 * t21 * delta__f__XO * t19 / t7 / t6 + 2 * delta__f__XO * t21 * t25 - t19 * t25 - 1.0 / t6;
  }

  real_type
  General::lambda__f_D_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t8   = ModelPars[118];
    real_type t9   = cos(phi__f__XO);
    return (t9 * t8 - t8 + ModelPars[116]) / (x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO);
  }

  real_type
  General::lambda__f_D_8_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    return 0;
  }

  real_type
  General::lambda__f_D_8_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[118];
    real_type t10  = cos(phi__f__XO);
    return -(t10 * t9 - t9 + ModelPars[116]) / t7;
  }

  real_type
  General::lambda__f_D_8_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[118];
    real_type t10  = cos(phi__f__XO);
    return -delta__f__XO * (t10 * t9 - t9 + ModelPars[116]) / t7;
  }

  real_type
  General::lambda__f_D_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[118];
    real_type t11  = cos(phi__f__XO);
    return -(-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[116] - t9) * omega__f__XO - u__XO) / t7 - 1.0 / t6;
  }

  real_type
  General::lambda__f_D_9_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    return 2 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 / t7;
  }

  real_type
  General::lambda__f_D_9_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    return 2 * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO / t7;
  }

  real_type
  General::lambda__f_D_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[118];
    real_type t11  = cos(phi__f__XO);
    return -delta__f__XO * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[116] - t9) * omega__f__XO - u__XO) / t7 - delta__f__XO / t6;
  }

  real_type
  General::lambda__f_D_10_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[118];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * delta__f__XO;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[116] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 / t7;
  }

  real_type
  General::Fxf( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t11  = Fzf__XO * ModelPars[206] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t14  = ModelPars[77] * ModelPars[164];
    real_type t24  = exp(t5 * t3 * ModelPars[87]);
    real_type t34  = atan(lambda__f__XO / (t11 * t14 + ModelPars[160]) * ModelPars[166] * t24 * (t5 * t3 * ModelPars[85] + ModelPars[83]) * Fzf__XO);
    real_type t36  = sin(t34 * t14);
    real_type t37  = ModelPars[104];
    real_type t42  = lambda__f__XO * lambda__f__XO;
    real_type t44  = ModelPars[94] * ModelPars[94];
    real_type t47  = sqrt(t44 * t42 + 1);
    real_type t49  = 1.0 / t47 * (phi__f__XO * ModelPars[96] + ModelPars[92]);
    real_type t50  = ModelPars[108];
    real_type t53  = atan((alpha__f__XO + t50) * t49);
    real_type t55  = cos(t53 * t37);
    real_type t58  = atan(t50 * t49);
    real_type t60  = cos(t58 * t37);
    return 1.0 / t60 * t55 * t36 * t11;
  }

  real_type
  General::Fxf_D_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[81];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[206];
    real_type t9   = ModelPars[77] * ModelPars[164];
    real_type t10  = ModelPars[85];
    real_type t11  = Fzf__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[83];
    real_type t16  = t15 * Fzf__XO;
    real_type t17  = ModelPars[87];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[166];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[79]);
    real_type t29  = Fzf__XO * t27 * t9;
    real_type t31  = t29 + ModelPars[160];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__f__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t40  = ModelPars[104];
    real_type t45  = lambda__f__XO * lambda__f__XO;
    real_type t47  = ModelPars[94] * ModelPars[94];
    real_type t50  = sqrt(t47 * t45 + 1);
    real_type t52  = 1.0 / t50 * (phi__f__XO * ModelPars[96] + ModelPars[92]);
    real_type t53  = ModelPars[108];
    real_type t56  = atan((alpha__f__XO + t53) * t52);
    real_type t58  = cos(t56 * t40);
    real_type t60  = atan(t53 * t52);
    real_type t62  = cos(t60 * t40);
    real_type t63  = 1.0 / t62;
    real_type t76  = lambda__f__XO * t32 * t22 * t20;
    real_type t81  = t31 * t31;
    real_type t82  = 1.0 / t81;
    real_type t94  = Fzf__XO * Fzf__XO;
    real_type t95  = t15 * t15;
    real_type t97  = t20 * t20;
    real_type t99  = t22 * t22;
    real_type t106 = cos(t37);
    return t63 * t58 * t38 * Fzf__XO * t5 * t3 * t1 + t63 * t58 * t38 * t27 + t63 * t58 * t106 / (t45 * t82 * t99 * t97 * t95 * t94 + 1) * (t34 * t20 * t15 + t76 * t3 * t10 * Fzf__XO + t76 * t3 * t17 * t16 - (Fzf__XO * t5 * t3 * t1 * t9 + t27 * t9) * lambda__f__XO * t82 * t22 * t21) * t29;
  }

  real_type
  General::Fxf_D_1_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[81];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t4   = t3 * t1;
    real_type t5   = ModelPars[206];
    real_type t6   = t5 * t4;
    real_type t7   = ModelPars[77];
    real_type t8   = ModelPars[164];
    real_type t9   = t8 * t7;
    real_type t10  = ModelPars[85];
    real_type t11  = Fzf__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[83];
    real_type t16  = t15 * Fzf__XO;
    real_type t17  = ModelPars[87];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[166];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[79]);
    real_type t29  = Fzf__XO * t27 * t9;
    real_type t31  = t29 + ModelPars[160];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__f__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t39  = ModelPars[104];
    real_type t44  = lambda__f__XO * lambda__f__XO;
    real_type t46  = ModelPars[94] * ModelPars[94];
    real_type t49  = sqrt(t46 * t44 + 1);
    real_type t51  = 1.0 / t49 * (phi__f__XO * ModelPars[96] + ModelPars[92]);
    real_type t52  = ModelPars[108];
    real_type t55  = atan((alpha__f__XO + t52) * t51);
    real_type t57  = cos(t55 * t39);
    real_type t60  = atan(t52 * t51);
    real_type t62  = cos(t60 * t39);
    real_type t63  = 1.0 / t62;
    real_type t64  = t63 * t57 * t38;
    real_type t70  = t20 * t15;
    real_type t72  = t10 * Fzf__XO;
    real_type t74  = t22 * t20;
    real_type t76  = lambda__f__XO * t32 * t74;
    real_type t78  = t3 * t17;
    real_type t79  = t78 * t16;
    real_type t81  = t31 * t31;
    real_type t82  = 1.0 / t81;
    real_type t88  = t27 * t9;
    real_type t89  = Fzf__XO * t5 * t3 * t1 * t9 + t88;
    real_type t91  = t89 * lambda__f__XO * t82 * t22;
    real_type t93  = t76 * t3 * t72 - t91 * t21 + t34 * t70 + t76 * t79;
    real_type t95  = Fzf__XO * Fzf__XO;
    real_type t96  = t15 * t15;
    real_type t97  = t96 * t95;
    real_type t98  = t20 * t20;
    real_type t99  = t98 * t97;
    real_type t100 = t22 * t22;
    real_type t102 = t44 * t82 * t100;
    real_type t104 = t102 * t99 + 1;
    real_type t105 = 1.0 / t104;
    real_type t107 = cos(t37);
    real_type t109 = t63 * t57 * t107;
    real_type t117 = t3 * t10;
    real_type t127 = t89 * lambda__f__XO * t82;
    real_type t130 = t2 * t2;
    real_type t131 = 1.0 / t130;
    real_type t140 = t17 * t17;
    real_type t148 = 1.0 / t81 / t31;
    real_type t150 = t89 * t89;
    real_type t166 = t104 * t104;
    real_type t167 = 1.0 / t166;
    real_type t175 = t100 * t98;
    real_type t177 = t44 * t82;
    real_type t192 = t7 * t7;
    real_type t194 = t8 * t8;
    real_type t197 = t93 * t93;
    return 2 * t64 * t6 + 2 * t109 * t105 * t93 * t8 * t7 * Fzf__XO * t5 * t4 + 2 * t109 * t105 * t93 * t88 + t109 * t105 * (-2 * t6 * t8 * t7 * lambda__f__XO * t82 * t74 * t16 + 2 * t150 * lambda__f__XO * t148 * t22 * t21 + t76 * t131 * t140 * t16 + 2 * t76 * t17 * t131 * t72 + 2 * t76 * t3 * t17 * t15 - 2 * t91 * t20 * t3 * t72 + 2 * t34 * t20 * t117 - 2 * t127 * t22 * t70 - 2 * t127 * t74 * t79) * t29 - (-2 * t89 * t44 * t148 * t100 * t99 + 2 * t117 * t177 * t175 * t15 * t95 + 2 * t102 * t98 * t96 * Fzf__XO + 2 * t78 * t177 * t175 * t97) * t63 * t57 * t107 * t167 * t93 * t29 - t64 * t167 * t197 * t194 * t192 * Fzf__XO * t27;
  }

  real_type
  General::Fxf_D_1_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[81];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[206];
    real_type t6   = t5 * t3 * t1;
    real_type t7   = ModelPars[77];
    real_type t8   = ModelPars[164];
    real_type t9   = t8 * t7;
    real_type t10  = ModelPars[85];
    real_type t11  = Fzf__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[83];
    real_type t16  = t15 * Fzf__XO;
    real_type t17  = ModelPars[87];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[166];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[79]);
    real_type t28  = Fzf__XO * t27;
    real_type t31  = t28 * t9 + ModelPars[160];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__f__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t39  = t38 * Fzf__XO;
    real_type t40  = ModelPars[104];
    real_type t43  = ModelPars[96];
    real_type t44  = lambda__f__XO * lambda__f__XO;
    real_type t46  = ModelPars[94] * ModelPars[94];
    real_type t48  = t46 * t44 + 1;
    real_type t49  = sqrt(t48);
    real_type t50  = 1.0 / t49;
    real_type t51  = t50 * t43;
    real_type t52  = ModelPars[108];
    real_type t53  = alpha__f__XO + t52;
    real_type t57  = t43 * phi__f__XO + ModelPars[92];
    real_type t58  = t57 * t57;
    real_type t60  = 1.0 / t48 * t58;
    real_type t61  = t53 * t53;
    real_type t64  = 1.0 / (t61 * t60 + 1);
    real_type t65  = t50 * t57;
    real_type t67  = atan(t53 * t65);
    real_type t68  = t67 * t40;
    real_type t69  = sin(t68);
    real_type t72  = atan(t52 * t65);
    real_type t73  = t72 * t40;
    real_type t74  = cos(t73);
    real_type t75  = 1.0 / t74;
    real_type t76  = t75 * t69 * t64;
    real_type t79  = cos(t68);
    real_type t82  = t74 * t74;
    real_type t83  = 1.0 / t82;
    real_type t87  = t52 * t52;
    real_type t90  = 1.0 / (t87 * t60 + 1);
    real_type t91  = sin(t73);
    real_type t93  = t91 * t90 * t52 * t50;
    real_type t105 = t43 * t40;
    real_type t117 = lambda__f__XO * t32 * t22 * t20;
    real_type t122 = t31 * t31;
    real_type t123 = 1.0 / t122;
    real_type t134 = t34 * t20 * t15 + t117 * t3 * t10 * Fzf__XO + t117 * t3 * t17 * t16 - (Fzf__XO * t5 * t3 * t1 * t9 + t27 * t9) * lambda__f__XO * t123 * t22 * t21;
    real_type t135 = Fzf__XO * Fzf__XO;
    real_type t136 = t15 * t15;
    real_type t138 = t20 * t20;
    real_type t140 = t22 * t22;
    real_type t145 = 1.0 / (t44 * t123 * t140 * t138 * t136 * t135 + 1);
    real_type t149 = cos(t37);
    return t93 * t105 * t83 * t79 * t149 * t145 * t134 * t8 * t7 * Fzf__XO * t27 - t75 * t69 * t64 * t53 * t51 * t40 * t149 * t145 * t134 * t9 * t28 + t91 * t90 * t52 * t50 * t105 * t83 * t79 * t38 * t27 - t76 * t53 * t50 * t43 * t40 * t38 * t27 + t93 * t43 * t40 * t83 * t79 * t39 * t6 - t76 * t53 * t51 * t40 * t39 * t6;
  }

  real_type
  General::Fxf_D_1_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[81];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[206];
    real_type t9   = ModelPars[77] * ModelPars[164];
    real_type t10  = ModelPars[85];
    real_type t11  = Fzf__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[83];
    real_type t16  = t15 * Fzf__XO;
    real_type t17  = ModelPars[87];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[166];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[79]);
    real_type t28  = Fzf__XO * t27;
    real_type t31  = t28 * t9 + ModelPars[160];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__f__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t41  = ModelPars[104];
    real_type t45  = phi__f__XO * ModelPars[96] + ModelPars[92];
    real_type t47  = lambda__f__XO * lambda__f__XO;
    real_type t49  = ModelPars[94] * ModelPars[94];
    real_type t51  = t49 * t47 + 1;
    real_type t52  = sqrt(t51);
    real_type t53  = 1.0 / t52;
    real_type t55  = t45 * t45;
    real_type t58  = ModelPars[108];
    real_type t59  = alpha__f__XO + t58;
    real_type t60  = t59 * t59;
    real_type t63  = 1.0 / (t60 / t51 * t55 + 1);
    real_type t64  = t53 * t45;
    real_type t66  = atan(t59 * t64);
    real_type t68  = sin(t66 * t41);
    real_type t71  = atan(t58 * t64);
    real_type t73  = cos(t71 * t41);
    real_type t74  = 1.0 / t73;
    real_type t75  = t74 * t68 * t63;
    real_type t88  = lambda__f__XO * t32 * t22 * t20;
    real_type t93  = t31 * t31;
    real_type t94  = 1.0 / t93;
    real_type t106 = Fzf__XO * Fzf__XO;
    real_type t107 = t15 * t15;
    real_type t109 = t20 * t20;
    real_type t111 = t22 * t22;
    real_type t120 = cos(t37);
    return -t75 * t53 * t45 * t41 * t38 * Fzf__XO * t5 * t3 * t1 - t75 * t64 * t41 * t38 * t27 - t74 * t68 * t63 * t53 * t45 * t41 * t120 / (t47 * t94 * t111 * t109 * t107 * t106 + 1) * (t34 * t20 * t15 + t88 * t3 * t10 * Fzf__XO + t88 * t3 * t17 * t16 - (Fzf__XO * t5 * t3 * t1 * t9 + t27 * t9) * lambda__f__XO * t94 * t22 * t21) * t9 * t28;
  }

  real_type
  General::Fxf_D_1_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[81];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[206];
    real_type t6   = t5 * t3 * t1;
    real_type t7   = Fzf__XO * Fzf__XO;
    real_type t8   = ModelPars[77];
    real_type t10  = ModelPars[164];
    real_type t11  = ModelPars[85];
    real_type t12  = Fzf__XO - t2;
    real_type t16  = t3 * t12 * t11 + ModelPars[83];
    real_type t20  = ModelPars[87];
    real_type t23  = exp(t3 * t12 * t20);
    real_type t24  = ModelPars[166];
    real_type t25  = t24 * t23;
    real_type t26  = t10 * t8;
    real_type t31  = t5 * (t3 * t12 * t1 + ModelPars[79]);
    real_type t33  = Fzf__XO * t31 * t26;
    real_type t35  = t33 + ModelPars[160];
    real_type t36  = 1.0 / t35;
    real_type t37  = t36 * t25;
    real_type t38  = t16 * t16;
    real_type t40  = t23 * t23;
    real_type t42  = t24 * t24;
    real_type t43  = t35 * t35;
    real_type t44  = 1.0 / t43;
    real_type t46  = lambda__f__XO * lambda__f__XO;
    real_type t49  = t46 * t44 * t42 * t40 * t38 * t7 + 1;
    real_type t50  = 1.0 / t49;
    real_type t51  = t16 * Fzf__XO;
    real_type t52  = t23 * t51;
    real_type t53  = t36 * t24;
    real_type t54  = lambda__f__XO * t53;
    real_type t56  = atan(t54 * t52);
    real_type t57  = t56 * t26;
    real_type t58  = cos(t57);
    real_type t59  = t58 * t50;
    real_type t60  = ModelPars[104];
    real_type t64  = phi__f__XO * ModelPars[96] + ModelPars[92];
    real_type t66  = ModelPars[94] * ModelPars[94];
    real_type t68  = t66 * t46 + 1;
    real_type t69  = sqrt(t68);
    real_type t71  = 1.0 / t69 * t64;
    real_type t72  = ModelPars[108];
    real_type t73  = alpha__f__XO + t72;
    real_type t75  = atan(t73 * t71);
    real_type t76  = t75 * t60;
    real_type t77  = cos(t76);
    real_type t79  = atan(t72 * t71);
    real_type t80  = t79 * t60;
    real_type t81  = cos(t80);
    real_type t82  = 1.0 / t81;
    real_type t83  = t82 * t77;
    real_type t85  = t83 * t59 * t37;
    real_type t87  = sin(t57);
    real_type t88  = t87 * Fzf__XO;
    real_type t89  = t64 * t60;
    real_type t93  = 1.0 / t69 / t68;
    real_type t94  = t73 * t93;
    real_type t96  = t64 * t64;
    real_type t98  = 1.0 / t68 * t96;
    real_type t99  = t73 * t73;
    real_type t102 = 1.0 / (t99 * t98 + 1);
    real_type t104 = sin(t76);
    real_type t109 = t81 * t81;
    real_type t110 = 1.0 / t109;
    real_type t111 = t110 * t77;
    real_type t116 = lambda__f__XO * t66;
    real_type t117 = t72 * t72;
    real_type t120 = 1.0 / (t117 * t98 + 1);
    real_type t121 = sin(t80);
    real_type t123 = t121 * t120 * t116;
    real_type t131 = t87 * t31;
    real_type t137 = t82 * t104 * t102;
    real_type t142 = t93 * t64;
    real_type t146 = t23 * t16;
    real_type t149 = t3 * t11 * Fzf__XO;
    real_type t155 = t44 * t24;
    real_type t161 = Fzf__XO * t5 * t3 * t1 * t26 + t31 * t26;
    real_type t166 = t77 * t58;
    real_type t174 = lambda__f__XO * t36 * t25;
    real_type t182 = -t161 * lambda__f__XO * t155 * t52 + t174 * t3 * t20 * t51 + t54 * t146 + t174 * t149;
    real_type t183 = t49 * t49;
    real_type t185 = 1.0 / t183 * t182;
    real_type t197 = t8 * t8;
    real_type t198 = t10 * t10;
    real_type t208 = t8 * Fzf__XO * t31;
    real_type t209 = t182 * t10;
    return t85 * t16 * t10 * t8 * t7 * t6 + t82 * t104 * t102 * lambda__f__XO * t66 * t94 * t89 * t88 * t6 - t123 * t72 * t93 * t89 * t111 * t88 * t6 + t85 * t16 * Fzf__XO * t10 * t8 * t31 + t137 * lambda__f__XO * t66 * t73 * t93 * t89 * t131 - t123 * t72 * t142 * t60 * t111 * t131 + t82 * t166 * t50 * (t53 * t23 * t3 * t20 * t51 - t161 * t155 * t52 + t53 * t146 + t37 * t149) * t33 - 2 * lambda__f__XO * t44 * t42 * t40 * t38 * t82 * t166 * t185 * t26 * t7 * Fzf__XO * t31 - t83 * t87 * t36 * t24 * t146 * t185 * t198 * t197 * t7 * t31 + t137 * t116 * t94 * t89 * t59 * t209 * t208 - t121 * t120 * lambda__f__XO * t66 * t72 * t142 * t60 * t110 * t77 * t59 * t209 * t208;
  }

  real_type
  General::Fxf_D_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[206] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t13  = ModelPars[77] * ModelPars[164];
    real_type t23  = exp(t5 * t3 * ModelPars[87]);
    real_type t26  = Fzf__XO * t10;
    real_type t34  = atan(lambda__f__XO / (t26 * t13 + ModelPars[160]) * ModelPars[166] * t23 * (t5 * t3 * ModelPars[85] + ModelPars[83]) * Fzf__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[104];
    real_type t41  = ModelPars[96];
    real_type t42  = lambda__f__XO * lambda__f__XO;
    real_type t44  = ModelPars[94] * ModelPars[94];
    real_type t46  = t44 * t42 + 1;
    real_type t47  = sqrt(t46);
    real_type t48  = 1.0 / t47;
    real_type t50  = ModelPars[108];
    real_type t51  = alpha__f__XO + t50;
    real_type t55  = t41 * phi__f__XO + ModelPars[92];
    real_type t56  = t55 * t55;
    real_type t58  = 1.0 / t46 * t56;
    real_type t59  = t51 * t51;
    real_type t63  = t48 * t55;
    real_type t65  = atan(t51 * t63);
    real_type t66  = t65 * t38;
    real_type t67  = sin(t66);
    real_type t70  = atan(t50 * t63);
    real_type t71  = t70 * t38;
    real_type t72  = cos(t71);
    real_type t77  = cos(t66);
    real_type t79  = t72 * t72;
    real_type t85  = t50 * t50;
    real_type t90  = sin(t71);
    return -1.0 / t72 * t67 / (t59 * t58 + 1) * t51 * t48 * t41 * t38 * t36 * Fzf__XO * t10 + t90 / (t85 * t58 + 1) * t50 * t48 * t41 * t38 / t79 * t77 * t36 * t26;
  }

  real_type
  General::Fxf_D_2_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[206] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t11  = Fzf__XO * t10;
    real_type t14  = ModelPars[77] * ModelPars[164];
    real_type t24  = exp(t5 * t3 * ModelPars[87]);
    real_type t34  = atan(lambda__f__XO / (t11 * t14 + ModelPars[160]) * ModelPars[166] * t24 * (t5 * t3 * ModelPars[85] + ModelPars[83]) * Fzf__XO);
    real_type t36  = sin(t34 * t14);
    real_type t37  = ModelPars[104];
    real_type t39  = ModelPars[96];
    real_type t40  = t39 * t39;
    real_type t43  = lambda__f__XO * lambda__f__XO;
    real_type t45  = ModelPars[94] * ModelPars[94];
    real_type t47  = t45 * t43 + 1;
    real_type t48  = sqrt(t47);
    real_type t50  = 1.0 / t48 / t47;
    real_type t51  = ModelPars[108];
    real_type t52  = alpha__f__XO + t51;
    real_type t53  = t52 * t52;
    real_type t58  = t39 * phi__f__XO + ModelPars[92];
    real_type t59  = t58 * t58;
    real_type t60  = 1.0 / t47;
    real_type t61  = t60 * t59;
    real_type t63  = t53 * t61 + 1;
    real_type t64  = t63 * t63;
    real_type t65  = 1.0 / t64;
    real_type t68  = 1.0 / t48 * t58;
    real_type t70  = atan(t52 * t68);
    real_type t71  = t70 * t37;
    real_type t72  = sin(t71);
    real_type t74  = atan(t51 * t68);
    real_type t75  = t74 * t37;
    real_type t76  = cos(t75);
    real_type t77  = 1.0 / t76;
    real_type t83  = t36 * Fzf__XO;
    real_type t84  = t37 * t37;
    real_type t87  = t60 * t40;
    real_type t89  = cos(t71);
    real_type t100 = t76 * t76;
    real_type t101 = 1.0 / t100;
    real_type t103 = t51 * t51;
    real_type t105 = t103 * t61 + 1;
    real_type t107 = sin(t75);
    real_type t113 = t89 * t36;
    real_type t120 = t105 * t105;
    real_type t121 = 1.0 / t120;
    real_type t123 = t107 * t107;
    return 2 * t58 * t77 * t72 * t65 * t53 * t52 * t50 * t40 * t37 * t36 * t11 - t77 * t89 * t65 * t53 * t87 * t84 * t83 * t10 - 2 * t107 / t105 * t51 * t101 * t72 / t63 * t52 * t87 * t84 * t36 * t11 + 2 * t123 * t121 * t103 * t60 * t40 * t84 / t100 / t76 * t113 * t11 - 2 * t58 * t107 * t121 * t103 * t51 * t50 * t40 * t37 * t101 * t113 * t11 + t121 * t103 * t60 * t40 * t84 * t77 * t89 * t83 * t10;
  }

  real_type
  General::Fxf_D_2_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[206] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t13  = ModelPars[77] * ModelPars[164];
    real_type t23  = exp(t5 * t3 * ModelPars[87]);
    real_type t26  = Fzf__XO * t10;
    real_type t34  = atan(lambda__f__XO / (t26 * t13 + ModelPars[160]) * ModelPars[166] * t23 * (t5 * t3 * ModelPars[85] + ModelPars[83]) * Fzf__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[104];
    real_type t41  = ModelPars[96];
    real_type t42  = lambda__f__XO * lambda__f__XO;
    real_type t44  = ModelPars[94] * ModelPars[94];
    real_type t46  = t44 * t42 + 1;
    real_type t47  = sqrt(t46);
    real_type t48  = 1.0 / t47;
    real_type t52  = t41 * phi__f__XO + ModelPars[92];
    real_type t53  = t52 * t52;
    real_type t54  = 1.0 / t46;
    real_type t55  = t54 * t53;
    real_type t56  = ModelPars[108];
    real_type t57  = alpha__f__XO + t56;
    real_type t58  = t57 * t57;
    real_type t60  = t58 * t55 + 1;
    real_type t62  = t48 * t52;
    real_type t64  = atan(t57 * t62);
    real_type t65  = t64 * t38;
    real_type t66  = sin(t65);
    real_type t67  = t66 / t60;
    real_type t69  = atan(t56 * t62);
    real_type t70  = t69 * t38;
    real_type t71  = cos(t70);
    real_type t72  = 1.0 / t71;
    real_type t82  = t60 * t60;
    real_type t83  = 1.0 / t82;
    real_type t90  = t38 * t38;
    real_type t91  = t90 * t36;
    real_type t96  = cos(t65);
    real_type t104 = t71 * t71;
    real_type t108 = t56 * t56;
    real_type t112 = sin(t70);
    return -t72 * t67 * t48 * t41 * t38 * t36 * Fzf__XO * t10 + 2 * t53 * t72 * t66 * t83 * t58 / t47 / t46 * t41 * t38 * t36 * t26 - t72 * t96 * t52 * t83 * t57 * t54 * t41 * t91 * t26 - t112 / (t108 * t55 + 1) * t56 * t41 / t104 * t67 * t54 * t52 * t91 * t26;
  }

  real_type
  General::Fxf_D_2_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[206] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t11  = Fzf__XO * Fzf__XO;
    real_type t12  = ModelPars[77];
    real_type t15  = ModelPars[164];
    real_type t20  = t5 * t3 * ModelPars[85] + ModelPars[83];
    real_type t25  = exp(t5 * t3 * ModelPars[87]);
    real_type t26  = ModelPars[166];
    real_type t28  = t15 * t12;
    real_type t29  = Fzf__XO * t10;
    real_type t32  = t29 * t28 + ModelPars[160];
    real_type t33  = 1.0 / t32;
    real_type t36  = t33 * t26 * t25 * t20 * t15 * t12 * t11 * t10;
    real_type t37  = t20 * t20;
    real_type t39  = t25 * t25;
    real_type t41  = t26 * t26;
    real_type t42  = t32 * t32;
    real_type t45  = lambda__f__XO * lambda__f__XO;
    real_type t55  = atan(lambda__f__XO * t33 * t26 * t25 * t20 * Fzf__XO);
    real_type t56  = t55 * t28;
    real_type t57  = cos(t56);
    real_type t58  = t57 / (t45 / t42 * t41 * t39 * t37 * t11 + 1);
    real_type t59  = ModelPars[104];
    real_type t60  = ModelPars[96];
    real_type t64  = ModelPars[94] * ModelPars[94];
    real_type t66  = t64 * t45 + 1;
    real_type t67  = sqrt(t66);
    real_type t68  = 1.0 / t67;
    real_type t69  = ModelPars[108];
    real_type t70  = alpha__f__XO + t69;
    real_type t74  = t60 * phi__f__XO + ModelPars[92];
    real_type t75  = t74 * t74;
    real_type t77  = 1.0 / t66 * t75;
    real_type t78  = t70 * t70;
    real_type t80  = t78 * t77 + 1;
    real_type t81  = 1.0 / t80;
    real_type t82  = t68 * t74;
    real_type t84  = atan(t70 * t82);
    real_type t85  = t84 * t59;
    real_type t86  = sin(t85);
    real_type t87  = t86 * t81;
    real_type t89  = atan(t69 * t82);
    real_type t90  = t89 * t59;
    real_type t91  = cos(t90);
    real_type t92  = 1.0 / t91;
    real_type t97  = sin(t56);
    real_type t98  = t59 * t97;
    real_type t102 = 1.0 / t67 / t66;
    real_type t106 = lambda__f__XO * t64;
    real_type t110 = t66 * t66;
    real_type t113 = 1.0 / t67 / t110 * t60;
    real_type t117 = t80 * t80;
    real_type t118 = 1.0 / t117;
    real_type t126 = t59 * t59;
    real_type t128 = 1.0 / t110;
    real_type t129 = t128 * t60;
    real_type t134 = cos(t85);
    real_type t145 = t91 * t91;
    real_type t146 = 1.0 / t145;
    real_type t150 = t69 * t69;
    real_type t152 = t150 * t77 + 1;
    real_type t153 = 1.0 / t152;
    real_type t155 = sin(t90);
    real_type t170 = t134 * t97;
    real_type t176 = t152 * t152;
    real_type t177 = 1.0 / t176;
    real_type t180 = t155 * t155;
    real_type t188 = t59 * t146 * t170 * t29;
    return -t92 * t87 * t70 * t68 * t60 * t59 * t58 * t36 + t106 * t92 * t86 * t81 * t70 * t102 * t60 * t98 * t29 - 2 * t106 * t75 * t92 * t86 * t118 * t78 * t70 * t113 * t98 * t29 + t92 * t134 * t106 * t74 * t118 * t78 * t129 * t126 * t97 * t29 + 2 * t155 * t153 * lambda__f__XO * t64 * t69 * t74 * t146 * t87 * t70 * t128 * t60 * t126 * t97 * Fzf__XO * t10 + t155 * t153 * t69 * t68 * t60 * t59 * t146 * t134 * t58 * t36 - 2 * t106 * t74 * t180 * t177 * t150 * t129 * t126 / t145 / t91 * t170 * t29 - t106 * t155 * t153 * t69 * t102 * t60 * t188 + 2 * t106 * t75 * t155 * t177 * t150 * t69 * t113 * t188 - t106 * t74 * t177 * t150 * t129 * t126 * t92 * t170 * t29;
  }

  real_type
  General::Fxf_D_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[206] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t13  = ModelPars[77] * ModelPars[164];
    real_type t23  = exp(t5 * t3 * ModelPars[87]);
    real_type t34  = atan(lambda__f__XO / (Fzf__XO * t10 * t13 + ModelPars[160]) * ModelPars[166] * t23 * (t5 * t3 * ModelPars[85] + ModelPars[83]) * Fzf__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[104];
    real_type t44  = phi__f__XO * ModelPars[96] + ModelPars[92];
    real_type t45  = lambda__f__XO * lambda__f__XO;
    real_type t47  = ModelPars[94] * ModelPars[94];
    real_type t49  = t47 * t45 + 1;
    real_type t50  = sqrt(t49);
    real_type t52  = 1.0 / t50 * t44;
    real_type t53  = t44 * t44;
    real_type t56  = ModelPars[108];
    real_type t57  = alpha__f__XO + t56;
    real_type t58  = t57 * t57;
    real_type t63  = atan(t57 * t52);
    real_type t65  = sin(t63 * t38);
    real_type t68  = atan(t56 * t52);
    real_type t70  = cos(t68 * t38);
    return -1.0 / t70 * t65 / (t58 / t49 * t53 + 1) * t52 * t38 * t36 * Fzf__XO * t10;
  }

  real_type
  General::Fxf_D_3_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[206] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t13  = ModelPars[77] * ModelPars[164];
    real_type t23  = exp(t5 * t3 * ModelPars[87]);
    real_type t34  = atan(lambda__f__XO / (Fzf__XO * t10 * t13 + ModelPars[160]) * ModelPars[166] * t23 * (t5 * t3 * ModelPars[85] + ModelPars[83]) * Fzf__XO);
    real_type t36  = sin(t34 * t13);
    real_type t37  = t36 * Fzf__XO;
    real_type t38  = ModelPars[104];
    real_type t44  = phi__f__XO * ModelPars[96] + ModelPars[92];
    real_type t45  = t44 * t44;
    real_type t47  = lambda__f__XO * lambda__f__XO;
    real_type t49  = ModelPars[94] * ModelPars[94];
    real_type t51  = t49 * t47 + 1;
    real_type t52  = sqrt(t51);
    real_type t57  = 1.0 / t51 * t45;
    real_type t58  = ModelPars[108];
    real_type t59  = alpha__f__XO + t58;
    real_type t60  = t59 * t59;
    real_type t63  = pow(t60 * t57 + 1, 2);
    real_type t64  = 1.0 / t63;
    real_type t67  = 1.0 / t52 * t44;
    real_type t69  = atan(t59 * t67);
    real_type t70  = t69 * t38;
    real_type t71  = sin(t70);
    real_type t73  = atan(t58 * t67);
    real_type t75  = cos(t73 * t38);
    real_type t76  = 1.0 / t75;
    real_type t82  = t38 * t38;
    real_type t85  = cos(t70);
    return 2 * t59 * t76 * t71 * t64 / t52 / t51 * t45 * t44 * t38 * t37 * t10 - t76 * t85 * t64 * t57 * t82 * t37 * t10;
  }

  real_type
  General::Fxf_D_3_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[206] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t11  = Fzf__XO * Fzf__XO;
    real_type t12  = ModelPars[77];
    real_type t15  = ModelPars[164];
    real_type t20  = t5 * t3 * ModelPars[85] + ModelPars[83];
    real_type t25  = exp(t5 * t3 * ModelPars[87]);
    real_type t26  = ModelPars[166];
    real_type t30  = t15 * t12;
    real_type t31  = Fzf__XO * t10;
    real_type t34  = t31 * t30 + ModelPars[160];
    real_type t35  = 1.0 / t34;
    real_type t36  = t20 * t20;
    real_type t38  = t25 * t25;
    real_type t40  = t26 * t26;
    real_type t41  = t34 * t34;
    real_type t44  = lambda__f__XO * lambda__f__XO;
    real_type t55  = atan(lambda__f__XO * t35 * t26 * t25 * t20 * Fzf__XO);
    real_type t56  = t55 * t30;
    real_type t57  = cos(t56);
    real_type t58  = ModelPars[104];
    real_type t64  = phi__f__XO * ModelPars[96] + ModelPars[92];
    real_type t66  = ModelPars[94] * ModelPars[94];
    real_type t68  = t66 * t44 + 1;
    real_type t69  = sqrt(t68);
    real_type t71  = 1.0 / t69 * t64;
    real_type t72  = t64 * t64;
    real_type t74  = 1.0 / t68 * t72;
    real_type t75  = ModelPars[108];
    real_type t76  = alpha__f__XO + t75;
    real_type t77  = t76 * t76;
    real_type t79  = t77 * t74 + 1;
    real_type t80  = 1.0 / t79;
    real_type t82  = atan(t76 * t71);
    real_type t83  = t82 * t58;
    real_type t84  = sin(t83);
    real_type t85  = t84 * t80;
    real_type t87  = atan(t75 * t71);
    real_type t88  = t87 * t58;
    real_type t89  = cos(t88);
    real_type t90  = 1.0 / t89;
    real_type t95  = sin(t56);
    real_type t96  = t58 * t95;
    real_type t110 = t68 * t68;
    real_type t113 = t79 * t79;
    real_type t114 = 1.0 / t113;
    real_type t118 = lambda__f__XO * t66;
    real_type t123 = t58 * t58;
    real_type t124 = t123 * t95;
    real_type t127 = 1.0 / t110;
    real_type t130 = cos(t83);
    real_type t138 = t89 * t89;
    real_type t142 = t75 * t75;
    real_type t146 = sin(t88);
    return -t90 * t85 * t71 * t58 * t57 / (t44 / t41 * t40 * t38 * t36 * t11 + 1) * t35 * t26 * t25 * t20 * t15 * t12 * t11 * t10 + lambda__f__XO * t66 * t90 * t84 * t80 / t69 / t68 * t64 * t96 * t31 - 2 * t118 * t77 * t90 * t84 * t114 / t69 / t110 * t72 * t64 * t96 * t31 + t90 * t130 * t118 * t76 * t114 * t127 * t72 * t124 * t31 + t146 / (t142 * t74 + 1) * t118 * t75 / t138 * t85 * t127 * t72 * t124 * t31;
  }

  real_type
  General::Fxf_D_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[206] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t11  = Fzf__XO * Fzf__XO;
    real_type t15  = ModelPars[77] * ModelPars[164];
    real_type t20  = t5 * t3 * ModelPars[85] + ModelPars[83];
    real_type t26  = exp(t5 * t3 * ModelPars[87]);
    real_type t27  = ModelPars[166];
    real_type t29  = Fzf__XO * t10;
    real_type t32  = t29 * t15 + ModelPars[160];
    real_type t33  = 1.0 / t32;
    real_type t35  = t20 * t20;
    real_type t37  = t26 * t26;
    real_type t39  = t27 * t27;
    real_type t40  = t32 * t32;
    real_type t43  = lambda__f__XO * lambda__f__XO;
    real_type t53  = atan(lambda__f__XO * t33 * t27 * t26 * t20 * Fzf__XO);
    real_type t54  = t53 * t15;
    real_type t55  = cos(t54);
    real_type t57  = ModelPars[104];
    real_type t61  = phi__f__XO * ModelPars[96] + ModelPars[92];
    real_type t63  = ModelPars[94] * ModelPars[94];
    real_type t65  = t63 * t43 + 1;
    real_type t66  = sqrt(t65);
    real_type t68  = 1.0 / t66 * t61;
    real_type t69  = ModelPars[108];
    real_type t70  = alpha__f__XO + t69;
    real_type t72  = atan(t70 * t68);
    real_type t73  = t72 * t57;
    real_type t74  = cos(t73);
    real_type t76  = atan(t69 * t68);
    real_type t77  = t76 * t57;
    real_type t78  = cos(t77);
    real_type t79  = 1.0 / t78;
    real_type t84  = sin(t54);
    real_type t89  = 1.0 / t66 / t65;
    real_type t92  = t61 * t61;
    real_type t94  = 1.0 / t65 * t92;
    real_type t95  = t70 * t70;
    real_type t100 = sin(t73);
    real_type t106 = t78 * t78;
    real_type t114 = t69 * t69;
    real_type t118 = sin(t77);
    return t79 * t74 * t55 / (t43 / t40 * t39 * t37 * t35 * t11 + 1) * t33 * t27 * t26 * t20 * t15 * t11 * t10 + t79 * t100 / (t95 * t94 + 1) * lambda__f__XO * t63 * t70 * t89 * t61 * t57 * t84 * t29 - t118 / (t114 * t94 + 1) * lambda__f__XO * t63 * t69 * t89 * t61 * t57 / t106 * t74 * t84 * t29;
  }

  real_type
  General::Fxf_D_4_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[206] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t11  = Fzf__XO * Fzf__XO;
    real_type t12  = t11 * t11;
    real_type t14  = ModelPars[77];
    real_type t15  = ModelPars[164];
    real_type t16  = t15 * t14;
    real_type t21  = t5 * t3 * ModelPars[85] + ModelPars[83];
    real_type t22  = t21 * t21;
    real_type t27  = exp(t5 * t3 * ModelPars[87]);
    real_type t28  = t27 * t27;
    real_type t33  = ModelPars[166];
    real_type t34  = t33 * t33;
    real_type t36  = Fzf__XO * t10;
    real_type t39  = t36 * t16 + ModelPars[160];
    real_type t40  = t39 * t39;
    real_type t46  = 1.0 / t40;
    real_type t48  = lambda__f__XO * lambda__f__XO;
    real_type t51  = t48 * t46 * t34 * t28 * t22 * t11 + 1;
    real_type t52  = t51 * t51;
    real_type t53  = 1.0 / t52;
    real_type t58  = 1.0 / t39 * t33;
    real_type t61  = atan(lambda__f__XO * t58 * t27 * t21 * Fzf__XO);
    real_type t62  = t61 * t16;
    real_type t63  = cos(t62);
    real_type t64  = ModelPars[104];
    real_type t68  = phi__f__XO * ModelPars[96] + ModelPars[92];
    real_type t70  = ModelPars[94] * ModelPars[94];
    real_type t72  = t70 * t48 + 1;
    real_type t73  = sqrt(t72);
    real_type t75  = 1.0 / t73 * t68;
    real_type t76  = ModelPars[108];
    real_type t77  = alpha__f__XO + t76;
    real_type t79  = atan(t77 * t75);
    real_type t80  = t79 * t64;
    real_type t81  = cos(t80);
    real_type t82  = t81 * t63;
    real_type t84  = atan(t76 * t75);
    real_type t85  = t84 * t64;
    real_type t86  = cos(t85);
    real_type t87  = 1.0 / t86;
    real_type t95  = t14 * t14;
    real_type t96  = t15 * t15;
    real_type t102 = sin(t62);
    real_type t104 = t87 * t81;
    real_type t115 = 1.0 / t51 * t58 * t27 * t21 * t15 * t14 * t11 * t10;
    real_type t118 = 1.0 / t73 / t72;
    real_type t123 = t68 * t68;
    real_type t125 = 1.0 / t72 * t123;
    real_type t126 = t77 * t77;
    real_type t128 = t126 * t125 + 1;
    real_type t129 = 1.0 / t128;
    real_type t130 = sin(t80);
    real_type t131 = t130 * t129;
    real_type t132 = t87 * t131;
    real_type t137 = t86 * t86;
    real_type t138 = 1.0 / t137;
    real_type t139 = t64 * t138;
    real_type t144 = t76 * t76;
    real_type t146 = t144 * t125 + 1;
    real_type t147 = 1.0 / t146;
    real_type t149 = sin(t85);
    real_type t155 = t64 * t102;
    real_type t157 = t68 * t155 * t36;
    real_type t158 = t72 * t72;
    real_type t160 = 1.0 / t73 / t158;
    real_type t162 = t70 * t70;
    real_type t165 = t87 * t130;
    real_type t174 = t123 * t68;
    real_type t177 = t158 * t72;
    real_type t179 = 1.0 / t73 / t177;
    real_type t183 = t128 * t128;
    real_type t185 = 1.0 / t183 * t48;
    real_type t190 = t64 * t64;
    real_type t194 = 1.0 / t177;
    real_type t202 = t123 * t190;
    real_type t206 = t48 * t162;
    real_type t209 = t149 * t147;
    real_type t214 = t81 * t102;
    real_type t222 = t146 * t146;
    real_type t223 = 1.0 / t222;
    real_type t224 = t149 * t149;
    real_type t231 = t139 * t214 * t36;
    return -2 * lambda__f__XO * t87 * t82 * t53 / t40 / t39 * t34 * t33 * t28 * t27 * t22 * t21 * t16 * t12 * t10 - t104 * t102 * t53 * t46 * t34 * t28 * t22 * t96 * t95 * t11 * Fzf__XO * t10 + 2 * t132 * lambda__f__XO * t70 * t77 * t118 * t68 * t64 * t63 * t115 - 2 * t149 * t147 * lambda__f__XO * t70 * t76 * t118 * t68 * t139 * t82 * t115 - 3 * t165 * t129 * t48 * t162 * t77 * t160 * t157 + t132 * t70 * t77 * t118 * t157 + 2 * t165 * t185 * t162 * t126 * t77 * t179 * t174 * t155 * t36 - t104 * t185 * t162 * t126 * t194 * t123 * t190 * t102 * t36 - 2 * t209 * t76 * t138 * t131 * t206 * t77 * t194 * t202 * t102 * Fzf__XO * t10 + 2 * t224 * t223 * t206 * t144 * t194 * t123 * t190 / t137 / t86 * t214 * t36 + 3 * t209 * t206 * t76 * t160 * t68 * t231 - t209 * t70 * t76 * t118 * t68 * t64 * t138 * t214 * t36 - 2 * t149 * t223 * t206 * t144 * t76 * t179 * t174 * t231 + t223 * t48 * t162 * t144 * t194 * t202 * t87 * t214 * t36;
  }

  real_type
  General::Fxr( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t11  = Fzr__XO * ModelPars[207] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t14  = ModelPars[78] * ModelPars[165];
    real_type t24  = exp(t5 * t3 * ModelPars[88]);
    real_type t34  = atan(lambda__r__XO / (t11 * t14 + ModelPars[161]) * ModelPars[167] * t24 * (t5 * t3 * ModelPars[86] + ModelPars[84]) * Fzr__XO);
    real_type t36  = sin(t34 * t14);
    real_type t37  = ModelPars[105];
    real_type t42  = lambda__r__XO * lambda__r__XO;
    real_type t44  = ModelPars[95] * ModelPars[95];
    real_type t47  = sqrt(t44 * t42 + 1);
    real_type t49  = 1.0 / t47 * (phi__XO * ModelPars[97] + ModelPars[93]);
    real_type t50  = ModelPars[109];
    real_type t53  = atan((alpha__r__XO + t50) * t49);
    real_type t55  = cos(t53 * t37);
    real_type t58  = atan(t50 * t49);
    real_type t60  = cos(t58 * t37);
    return 1.0 / t60 * t55 * t36 * t11;
  }

  real_type
  General::Fxr_D_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[82];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[207];
    real_type t9   = ModelPars[78] * ModelPars[165];
    real_type t10  = ModelPars[86];
    real_type t11  = Fzr__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[84];
    real_type t16  = t15 * Fzr__XO;
    real_type t17  = ModelPars[88];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[167];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[80]);
    real_type t29  = Fzr__XO * t27 * t9;
    real_type t31  = t29 + ModelPars[161];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__r__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t40  = ModelPars[105];
    real_type t45  = lambda__r__XO * lambda__r__XO;
    real_type t47  = ModelPars[95] * ModelPars[95];
    real_type t50  = sqrt(t47 * t45 + 1);
    real_type t52  = 1.0 / t50 * (phi__XO * ModelPars[97] + ModelPars[93]);
    real_type t53  = ModelPars[109];
    real_type t56  = atan((alpha__r__XO + t53) * t52);
    real_type t58  = cos(t56 * t40);
    real_type t60  = atan(t53 * t52);
    real_type t62  = cos(t60 * t40);
    real_type t63  = 1.0 / t62;
    real_type t76  = lambda__r__XO * t32 * t22 * t20;
    real_type t81  = t31 * t31;
    real_type t82  = 1.0 / t81;
    real_type t94  = Fzr__XO * Fzr__XO;
    real_type t95  = t15 * t15;
    real_type t97  = t20 * t20;
    real_type t99  = t22 * t22;
    real_type t106 = cos(t37);
    return t63 * t58 * t38 * Fzr__XO * t5 * t3 * t1 + t63 * t58 * t38 * t27 + t63 * t58 * t106 / (t45 * t82 * t99 * t97 * t95 * t94 + 1) * (t34 * t20 * t15 + t76 * t3 * t10 * Fzr__XO + t76 * t3 * t17 * t16 - (Fzr__XO * t5 * t3 * t1 * t9 + t27 * t9) * lambda__r__XO * t82 * t22 * t21) * t29;
  }

  real_type
  General::Fxr_D_1_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[82];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t4   = t3 * t1;
    real_type t5   = ModelPars[207];
    real_type t6   = t5 * t4;
    real_type t7   = ModelPars[78];
    real_type t8   = ModelPars[165];
    real_type t9   = t8 * t7;
    real_type t10  = ModelPars[86];
    real_type t11  = Fzr__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[84];
    real_type t16  = t15 * Fzr__XO;
    real_type t17  = ModelPars[88];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[167];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[80]);
    real_type t29  = Fzr__XO * t27 * t9;
    real_type t31  = t29 + ModelPars[161];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__r__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t39  = ModelPars[105];
    real_type t44  = lambda__r__XO * lambda__r__XO;
    real_type t46  = ModelPars[95] * ModelPars[95];
    real_type t49  = sqrt(t46 * t44 + 1);
    real_type t51  = 1.0 / t49 * (phi__XO * ModelPars[97] + ModelPars[93]);
    real_type t52  = ModelPars[109];
    real_type t55  = atan((alpha__r__XO + t52) * t51);
    real_type t57  = cos(t55 * t39);
    real_type t60  = atan(t52 * t51);
    real_type t62  = cos(t60 * t39);
    real_type t63  = 1.0 / t62;
    real_type t64  = t63 * t57 * t38;
    real_type t70  = t20 * t15;
    real_type t72  = t10 * Fzr__XO;
    real_type t74  = t22 * t20;
    real_type t76  = lambda__r__XO * t32 * t74;
    real_type t78  = t3 * t17;
    real_type t79  = t78 * t16;
    real_type t81  = t31 * t31;
    real_type t82  = 1.0 / t81;
    real_type t88  = t27 * t9;
    real_type t89  = Fzr__XO * t5 * t3 * t1 * t9 + t88;
    real_type t91  = t89 * lambda__r__XO * t82 * t22;
    real_type t93  = t76 * t3 * t72 - t91 * t21 + t34 * t70 + t76 * t79;
    real_type t95  = Fzr__XO * Fzr__XO;
    real_type t96  = t15 * t15;
    real_type t97  = t96 * t95;
    real_type t98  = t20 * t20;
    real_type t99  = t98 * t97;
    real_type t100 = t22 * t22;
    real_type t102 = t44 * t82 * t100;
    real_type t104 = t102 * t99 + 1;
    real_type t105 = 1.0 / t104;
    real_type t107 = cos(t37);
    real_type t109 = t63 * t57 * t107;
    real_type t117 = t3 * t10;
    real_type t127 = t89 * lambda__r__XO * t82;
    real_type t130 = t2 * t2;
    real_type t131 = 1.0 / t130;
    real_type t140 = t17 * t17;
    real_type t148 = 1.0 / t81 / t31;
    real_type t150 = t89 * t89;
    real_type t166 = t104 * t104;
    real_type t167 = 1.0 / t166;
    real_type t175 = t100 * t98;
    real_type t177 = t44 * t82;
    real_type t192 = t7 * t7;
    real_type t194 = t8 * t8;
    real_type t197 = t93 * t93;
    return 2 * t64 * t6 + 2 * t109 * t105 * t93 * t8 * t7 * Fzr__XO * t5 * t4 + 2 * t109 * t105 * t93 * t88 + t109 * t105 * (-2 * t6 * t8 * t7 * lambda__r__XO * t82 * t74 * t16 + 2 * t150 * lambda__r__XO * t148 * t22 * t21 + t76 * t131 * t140 * t16 + 2 * t76 * t17 * t131 * t72 + 2 * t76 * t3 * t17 * t15 - 2 * t91 * t20 * t3 * t72 + 2 * t34 * t20 * t117 - 2 * t127 * t22 * t70 - 2 * t127 * t74 * t79) * t29 - (-2 * t89 * t44 * t148 * t100 * t99 + 2 * t117 * t177 * t175 * t15 * t95 + 2 * t102 * t98 * t96 * Fzr__XO + 2 * t78 * t177 * t175 * t97) * t63 * t57 * t107 * t167 * t93 * t29 - t64 * t167 * t197 * t194 * t192 * Fzr__XO * t27;
  }

  real_type
  General::Fxr_D_1_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[82];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[207];
    real_type t6   = t5 * t3 * t1;
    real_type t7   = ModelPars[78];
    real_type t8   = ModelPars[165];
    real_type t9   = t8 * t7;
    real_type t10  = ModelPars[86];
    real_type t11  = Fzr__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[84];
    real_type t16  = t15 * Fzr__XO;
    real_type t17  = ModelPars[88];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[167];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[80]);
    real_type t28  = Fzr__XO * t27;
    real_type t31  = t28 * t9 + ModelPars[161];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__r__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t39  = t38 * Fzr__XO;
    real_type t40  = ModelPars[105];
    real_type t43  = ModelPars[97];
    real_type t44  = lambda__r__XO * lambda__r__XO;
    real_type t46  = ModelPars[95] * ModelPars[95];
    real_type t48  = t46 * t44 + 1;
    real_type t49  = sqrt(t48);
    real_type t50  = 1.0 / t49;
    real_type t51  = t50 * t43;
    real_type t52  = ModelPars[109];
    real_type t53  = alpha__r__XO + t52;
    real_type t57  = t43 * phi__XO + ModelPars[93];
    real_type t58  = t57 * t57;
    real_type t60  = 1.0 / t48 * t58;
    real_type t61  = t53 * t53;
    real_type t64  = 1.0 / (t61 * t60 + 1);
    real_type t65  = t50 * t57;
    real_type t67  = atan(t53 * t65);
    real_type t68  = t67 * t40;
    real_type t69  = sin(t68);
    real_type t72  = atan(t52 * t65);
    real_type t73  = t72 * t40;
    real_type t74  = cos(t73);
    real_type t75  = 1.0 / t74;
    real_type t76  = t75 * t69 * t64;
    real_type t79  = cos(t68);
    real_type t82  = t74 * t74;
    real_type t83  = 1.0 / t82;
    real_type t87  = t52 * t52;
    real_type t90  = 1.0 / (t87 * t60 + 1);
    real_type t91  = sin(t73);
    real_type t93  = t91 * t90 * t52 * t50;
    real_type t105 = t43 * t40;
    real_type t117 = lambda__r__XO * t32 * t22 * t20;
    real_type t122 = t31 * t31;
    real_type t123 = 1.0 / t122;
    real_type t134 = t34 * t20 * t15 + t117 * t3 * t10 * Fzr__XO + t117 * t3 * t17 * t16 - (Fzr__XO * t5 * t3 * t1 * t9 + t27 * t9) * lambda__r__XO * t123 * t22 * t21;
    real_type t135 = Fzr__XO * Fzr__XO;
    real_type t136 = t15 * t15;
    real_type t138 = t20 * t20;
    real_type t140 = t22 * t22;
    real_type t145 = 1.0 / (t44 * t123 * t140 * t138 * t136 * t135 + 1);
    real_type t149 = cos(t37);
    return t93 * t105 * t83 * t79 * t149 * t145 * t134 * t8 * t7 * Fzr__XO * t27 - t75 * t69 * t64 * t53 * t51 * t40 * t149 * t145 * t134 * t9 * t28 + t91 * t90 * t52 * t50 * t105 * t83 * t79 * t38 * t27 - t76 * t53 * t50 * t43 * t40 * t38 * t27 + t93 * t43 * t40 * t83 * t79 * t39 * t6 - t76 * t53 * t51 * t40 * t39 * t6;
  }

  real_type
  General::Fxr_D_1_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[82];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[207];
    real_type t9   = ModelPars[78] * ModelPars[165];
    real_type t10  = ModelPars[86];
    real_type t11  = Fzr__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[84];
    real_type t16  = t15 * Fzr__XO;
    real_type t17  = ModelPars[88];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[167];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[80]);
    real_type t28  = Fzr__XO * t27;
    real_type t31  = t28 * t9 + ModelPars[161];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__r__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t41  = ModelPars[105];
    real_type t45  = phi__XO * ModelPars[97] + ModelPars[93];
    real_type t47  = lambda__r__XO * lambda__r__XO;
    real_type t49  = ModelPars[95] * ModelPars[95];
    real_type t51  = t49 * t47 + 1;
    real_type t52  = sqrt(t51);
    real_type t53  = 1.0 / t52;
    real_type t55  = t45 * t45;
    real_type t58  = ModelPars[109];
    real_type t59  = alpha__r__XO + t58;
    real_type t60  = t59 * t59;
    real_type t63  = 1.0 / (t60 / t51 * t55 + 1);
    real_type t64  = t53 * t45;
    real_type t66  = atan(t59 * t64);
    real_type t68  = sin(t66 * t41);
    real_type t71  = atan(t58 * t64);
    real_type t73  = cos(t71 * t41);
    real_type t74  = 1.0 / t73;
    real_type t75  = t74 * t68 * t63;
    real_type t88  = lambda__r__XO * t32 * t22 * t20;
    real_type t93  = t31 * t31;
    real_type t94  = 1.0 / t93;
    real_type t106 = Fzr__XO * Fzr__XO;
    real_type t107 = t15 * t15;
    real_type t109 = t20 * t20;
    real_type t111 = t22 * t22;
    real_type t120 = cos(t37);
    return -t75 * t53 * t45 * t41 * t38 * Fzr__XO * t5 * t3 * t1 - t75 * t64 * t41 * t38 * t27 - t74 * t68 * t63 * t53 * t45 * t41 * t120 / (t47 * t94 * t111 * t109 * t107 * t106 + 1) * (t34 * t20 * t15 + t88 * t3 * t10 * Fzr__XO + t88 * t3 * t17 * t16 - (Fzr__XO * t5 * t3 * t1 * t9 + t27 * t9) * lambda__r__XO * t94 * t22 * t21) * t9 * t28;
  }

  real_type
  General::Fxr_D_1_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[82];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[207];
    real_type t6   = t5 * t3 * t1;
    real_type t7   = Fzr__XO * Fzr__XO;
    real_type t8   = ModelPars[78];
    real_type t10  = ModelPars[165];
    real_type t11  = ModelPars[86];
    real_type t12  = Fzr__XO - t2;
    real_type t16  = t3 * t12 * t11 + ModelPars[84];
    real_type t20  = ModelPars[88];
    real_type t23  = exp(t3 * t12 * t20);
    real_type t24  = ModelPars[167];
    real_type t25  = t24 * t23;
    real_type t26  = t10 * t8;
    real_type t31  = t5 * (t3 * t12 * t1 + ModelPars[80]);
    real_type t33  = Fzr__XO * t31 * t26;
    real_type t35  = t33 + ModelPars[161];
    real_type t36  = 1.0 / t35;
    real_type t37  = t36 * t25;
    real_type t38  = t16 * t16;
    real_type t40  = t23 * t23;
    real_type t42  = t24 * t24;
    real_type t43  = t35 * t35;
    real_type t44  = 1.0 / t43;
    real_type t46  = lambda__r__XO * lambda__r__XO;
    real_type t49  = t46 * t44 * t42 * t40 * t38 * t7 + 1;
    real_type t50  = 1.0 / t49;
    real_type t51  = t16 * Fzr__XO;
    real_type t52  = t23 * t51;
    real_type t53  = t36 * t24;
    real_type t54  = lambda__r__XO * t53;
    real_type t56  = atan(t54 * t52);
    real_type t57  = t56 * t26;
    real_type t58  = cos(t57);
    real_type t59  = t58 * t50;
    real_type t60  = ModelPars[105];
    real_type t64  = phi__XO * ModelPars[97] + ModelPars[93];
    real_type t66  = ModelPars[95] * ModelPars[95];
    real_type t68  = t66 * t46 + 1;
    real_type t69  = sqrt(t68);
    real_type t71  = 1.0 / t69 * t64;
    real_type t72  = ModelPars[109];
    real_type t73  = alpha__r__XO + t72;
    real_type t75  = atan(t73 * t71);
    real_type t76  = t75 * t60;
    real_type t77  = cos(t76);
    real_type t79  = atan(t72 * t71);
    real_type t80  = t79 * t60;
    real_type t81  = cos(t80);
    real_type t82  = 1.0 / t81;
    real_type t83  = t82 * t77;
    real_type t85  = t83 * t59 * t37;
    real_type t87  = sin(t57);
    real_type t88  = t87 * Fzr__XO;
    real_type t89  = t64 * t60;
    real_type t93  = 1.0 / t69 / t68;
    real_type t94  = t73 * t93;
    real_type t96  = t64 * t64;
    real_type t98  = 1.0 / t68 * t96;
    real_type t99  = t73 * t73;
    real_type t102 = 1.0 / (t99 * t98 + 1);
    real_type t104 = sin(t76);
    real_type t109 = t81 * t81;
    real_type t110 = 1.0 / t109;
    real_type t111 = t110 * t77;
    real_type t116 = lambda__r__XO * t66;
    real_type t117 = t72 * t72;
    real_type t120 = 1.0 / (t117 * t98 + 1);
    real_type t121 = sin(t80);
    real_type t123 = t121 * t120 * t116;
    real_type t131 = t87 * t31;
    real_type t137 = t82 * t104 * t102;
    real_type t142 = t93 * t64;
    real_type t146 = t23 * t16;
    real_type t149 = t3 * t11 * Fzr__XO;
    real_type t155 = t44 * t24;
    real_type t161 = Fzr__XO * t5 * t3 * t1 * t26 + t31 * t26;
    real_type t166 = t77 * t58;
    real_type t174 = lambda__r__XO * t36 * t25;
    real_type t182 = -t161 * lambda__r__XO * t155 * t52 + t174 * t3 * t20 * t51 + t54 * t146 + t174 * t149;
    real_type t183 = t49 * t49;
    real_type t185 = 1.0 / t183 * t182;
    real_type t197 = t8 * t8;
    real_type t198 = t10 * t10;
    real_type t208 = t8 * Fzr__XO * t31;
    real_type t209 = t182 * t10;
    return t85 * t16 * t10 * t8 * t7 * t6 + t82 * t104 * t102 * lambda__r__XO * t66 * t94 * t89 * t88 * t6 - t123 * t72 * t93 * t89 * t111 * t88 * t6 + t85 * t16 * Fzr__XO * t10 * t8 * t31 + t137 * lambda__r__XO * t66 * t73 * t93 * t89 * t131 - t123 * t72 * t142 * t60 * t111 * t131 + t82 * t166 * t50 * (t53 * t23 * t3 * t20 * t51 - t161 * t155 * t52 + t53 * t146 + t37 * t149) * t33 - 2 * lambda__r__XO * t44 * t42 * t40 * t38 * t82 * t166 * t185 * t26 * t7 * Fzr__XO * t31 - t83 * t87 * t36 * t24 * t146 * t185 * t198 * t197 * t7 * t31 + t137 * t116 * t94 * t89 * t59 * t209 * t208 - t121 * t120 * lambda__r__XO * t66 * t72 * t142 * t60 * t110 * t77 * t59 * t209 * t208;
  }

  real_type
  General::Fxr_D_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[207] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t13  = ModelPars[78] * ModelPars[165];
    real_type t23  = exp(t5 * t3 * ModelPars[88]);
    real_type t26  = t10 * Fzr__XO;
    real_type t34  = atan(lambda__r__XO / (t26 * t13 + ModelPars[161]) * ModelPars[167] * t23 * (t5 * t3 * ModelPars[86] + ModelPars[84]) * Fzr__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[105];
    real_type t41  = ModelPars[97];
    real_type t42  = lambda__r__XO * lambda__r__XO;
    real_type t44  = ModelPars[95] * ModelPars[95];
    real_type t46  = t44 * t42 + 1;
    real_type t47  = sqrt(t46);
    real_type t48  = 1.0 / t47;
    real_type t50  = ModelPars[109];
    real_type t51  = alpha__r__XO + t50;
    real_type t55  = t41 * phi__XO + ModelPars[93];
    real_type t56  = t55 * t55;
    real_type t58  = 1.0 / t46 * t56;
    real_type t59  = t51 * t51;
    real_type t63  = t48 * t55;
    real_type t65  = atan(t51 * t63);
    real_type t66  = t65 * t38;
    real_type t67  = sin(t66);
    real_type t70  = atan(t50 * t63);
    real_type t71  = t70 * t38;
    real_type t72  = cos(t71);
    real_type t77  = cos(t66);
    real_type t79  = t72 * t72;
    real_type t85  = t50 * t50;
    real_type t90  = sin(t71);
    return -1.0 / t72 * t67 / (t59 * t58 + 1) * t51 * t48 * t41 * t38 * t36 * Fzr__XO * t10 + t90 / (t85 * t58 + 1) * t50 * t48 * t41 * t38 / t79 * t77 * t36 * t26;
  }

  real_type
  General::Fxr_D_2_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[207] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t11  = t10 * Fzr__XO;
    real_type t14  = ModelPars[78] * ModelPars[165];
    real_type t24  = exp(t5 * t3 * ModelPars[88]);
    real_type t34  = atan(lambda__r__XO / (t11 * t14 + ModelPars[161]) * ModelPars[167] * t24 * (t5 * t3 * ModelPars[86] + ModelPars[84]) * Fzr__XO);
    real_type t36  = sin(t34 * t14);
    real_type t37  = ModelPars[105];
    real_type t39  = ModelPars[97];
    real_type t40  = t39 * t39;
    real_type t43  = lambda__r__XO * lambda__r__XO;
    real_type t45  = ModelPars[95] * ModelPars[95];
    real_type t47  = t45 * t43 + 1;
    real_type t48  = sqrt(t47);
    real_type t50  = 1.0 / t48 / t47;
    real_type t51  = ModelPars[109];
    real_type t52  = alpha__r__XO + t51;
    real_type t53  = t52 * t52;
    real_type t58  = t39 * phi__XO + ModelPars[93];
    real_type t59  = t58 * t58;
    real_type t60  = 1.0 / t47;
    real_type t61  = t60 * t59;
    real_type t63  = t53 * t61 + 1;
    real_type t64  = t63 * t63;
    real_type t65  = 1.0 / t64;
    real_type t68  = 1.0 / t48 * t58;
    real_type t70  = atan(t52 * t68);
    real_type t71  = t70 * t37;
    real_type t72  = sin(t71);
    real_type t74  = atan(t51 * t68);
    real_type t75  = t74 * t37;
    real_type t76  = cos(t75);
    real_type t77  = 1.0 / t76;
    real_type t83  = t36 * Fzr__XO;
    real_type t84  = t37 * t37;
    real_type t87  = t60 * t40;
    real_type t89  = cos(t71);
    real_type t100 = t76 * t76;
    real_type t101 = 1.0 / t100;
    real_type t103 = t51 * t51;
    real_type t105 = t103 * t61 + 1;
    real_type t107 = sin(t75);
    real_type t113 = t89 * t36;
    real_type t120 = t105 * t105;
    real_type t121 = 1.0 / t120;
    real_type t123 = t107 * t107;
    return 2 * t58 * t77 * t72 * t65 * t53 * t52 * t50 * t40 * t37 * t36 * t11 - t77 * t89 * t65 * t53 * t87 * t84 * t83 * t10 - 2 * t107 / t105 * t51 * t101 * t72 / t63 * t52 * t87 * t84 * t36 * t11 + 2 * t123 * t121 * t103 * t60 * t40 * t84 / t100 / t76 * t113 * t11 - 2 * t58 * t107 * t121 * t103 * t51 * t50 * t40 * t37 * t101 * t113 * t11 + t121 * t103 * t60 * t40 * t84 * t77 * t89 * t83 * t10;
  }

  real_type
  General::Fxr_D_2_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[207] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t13  = ModelPars[78] * ModelPars[165];
    real_type t23  = exp(t5 * t3 * ModelPars[88]);
    real_type t26  = t10 * Fzr__XO;
    real_type t34  = atan(lambda__r__XO / (t26 * t13 + ModelPars[161]) * ModelPars[167] * t23 * (t5 * t3 * ModelPars[86] + ModelPars[84]) * Fzr__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[105];
    real_type t41  = ModelPars[97];
    real_type t42  = lambda__r__XO * lambda__r__XO;
    real_type t44  = ModelPars[95] * ModelPars[95];
    real_type t46  = t44 * t42 + 1;
    real_type t47  = sqrt(t46);
    real_type t48  = 1.0 / t47;
    real_type t52  = t41 * phi__XO + ModelPars[93];
    real_type t53  = t52 * t52;
    real_type t54  = 1.0 / t46;
    real_type t55  = t54 * t53;
    real_type t56  = ModelPars[109];
    real_type t57  = alpha__r__XO + t56;
    real_type t58  = t57 * t57;
    real_type t60  = t58 * t55 + 1;
    real_type t62  = t48 * t52;
    real_type t64  = atan(t57 * t62);
    real_type t65  = t64 * t38;
    real_type t66  = sin(t65);
    real_type t67  = t66 / t60;
    real_type t69  = atan(t56 * t62);
    real_type t70  = t69 * t38;
    real_type t71  = cos(t70);
    real_type t72  = 1.0 / t71;
    real_type t82  = t60 * t60;
    real_type t83  = 1.0 / t82;
    real_type t90  = t38 * t38;
    real_type t91  = t90 * t36;
    real_type t96  = cos(t65);
    real_type t104 = t71 * t71;
    real_type t108 = t56 * t56;
    real_type t112 = sin(t70);
    return -t72 * t67 * t48 * t41 * t38 * t36 * Fzr__XO * t10 + 2 * t53 * t72 * t66 * t83 * t58 / t47 / t46 * t41 * t38 * t36 * t26 - t72 * t96 * t52 * t83 * t57 * t54 * t41 * t91 * t26 - t112 / (t108 * t55 + 1) * t56 * t41 / t104 * t67 * t54 * t52 * t91 * t26;
  }

  real_type
  General::Fxr_D_2_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[207] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t11  = Fzr__XO * Fzr__XO;
    real_type t12  = ModelPars[78];
    real_type t15  = ModelPars[165];
    real_type t20  = t5 * t3 * ModelPars[86] + ModelPars[84];
    real_type t25  = exp(t5 * t3 * ModelPars[88]);
    real_type t26  = ModelPars[167];
    real_type t28  = t15 * t12;
    real_type t29  = t10 * Fzr__XO;
    real_type t32  = t29 * t28 + ModelPars[161];
    real_type t33  = 1.0 / t32;
    real_type t36  = t33 * t26 * t25 * t20 * t15 * t12 * t11 * t10;
    real_type t37  = t20 * t20;
    real_type t39  = t25 * t25;
    real_type t41  = t26 * t26;
    real_type t42  = t32 * t32;
    real_type t45  = lambda__r__XO * lambda__r__XO;
    real_type t55  = atan(lambda__r__XO * t33 * t26 * t25 * t20 * Fzr__XO);
    real_type t56  = t55 * t28;
    real_type t57  = cos(t56);
    real_type t58  = t57 / (t45 / t42 * t41 * t39 * t37 * t11 + 1);
    real_type t59  = ModelPars[105];
    real_type t60  = ModelPars[97];
    real_type t64  = ModelPars[95] * ModelPars[95];
    real_type t66  = t64 * t45 + 1;
    real_type t67  = sqrt(t66);
    real_type t68  = 1.0 / t67;
    real_type t69  = ModelPars[109];
    real_type t70  = alpha__r__XO + t69;
    real_type t74  = t60 * phi__XO + ModelPars[93];
    real_type t75  = t74 * t74;
    real_type t77  = 1.0 / t66 * t75;
    real_type t78  = t70 * t70;
    real_type t80  = t78 * t77 + 1;
    real_type t81  = 1.0 / t80;
    real_type t82  = t68 * t74;
    real_type t84  = atan(t70 * t82);
    real_type t85  = t84 * t59;
    real_type t86  = sin(t85);
    real_type t87  = t86 * t81;
    real_type t89  = atan(t69 * t82);
    real_type t90  = t89 * t59;
    real_type t91  = cos(t90);
    real_type t92  = 1.0 / t91;
    real_type t97  = sin(t56);
    real_type t98  = t59 * t97;
    real_type t102 = 1.0 / t67 / t66;
    real_type t106 = lambda__r__XO * t64;
    real_type t110 = t66 * t66;
    real_type t113 = 1.0 / t67 / t110 * t60;
    real_type t117 = t80 * t80;
    real_type t118 = 1.0 / t117;
    real_type t126 = t59 * t59;
    real_type t128 = 1.0 / t110;
    real_type t129 = t128 * t60;
    real_type t134 = cos(t85);
    real_type t145 = t91 * t91;
    real_type t146 = 1.0 / t145;
    real_type t150 = t69 * t69;
    real_type t152 = t150 * t77 + 1;
    real_type t153 = 1.0 / t152;
    real_type t155 = sin(t90);
    real_type t170 = t134 * t97;
    real_type t176 = t152 * t152;
    real_type t177 = 1.0 / t176;
    real_type t180 = t155 * t155;
    real_type t188 = t59 * t146 * t170 * t29;
    return -t92 * t87 * t70 * t68 * t60 * t59 * t58 * t36 + t106 * t92 * t86 * t81 * t70 * t102 * t60 * t98 * t29 - 2 * t106 * t75 * t92 * t86 * t118 * t78 * t70 * t113 * t98 * t29 + t92 * t134 * t106 * t74 * t118 * t78 * t129 * t126 * t97 * t29 + 2 * t155 * t153 * lambda__r__XO * t64 * t69 * t74 * t146 * t87 * t70 * t128 * t60 * t126 * t97 * Fzr__XO * t10 + t155 * t153 * t69 * t68 * t60 * t59 * t146 * t134 * t58 * t36 - 2 * t106 * t74 * t180 * t177 * t150 * t129 * t126 / t145 / t91 * t170 * t29 - t106 * t155 * t153 * t69 * t102 * t60 * t188 + 2 * t106 * t75 * t155 * t177 * t150 * t69 * t113 * t188 - t106 * t74 * t177 * t150 * t129 * t126 * t92 * t170 * t29;
  }

  real_type
  General::Fxr_D_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[207] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t13  = ModelPars[78] * ModelPars[165];
    real_type t23  = exp(t5 * t3 * ModelPars[88]);
    real_type t34  = atan(lambda__r__XO / (Fzr__XO * t10 * t13 + ModelPars[161]) * ModelPars[167] * t23 * (t5 * t3 * ModelPars[86] + ModelPars[84]) * Fzr__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[105];
    real_type t44  = phi__XO * ModelPars[97] + ModelPars[93];
    real_type t45  = lambda__r__XO * lambda__r__XO;
    real_type t47  = ModelPars[95] * ModelPars[95];
    real_type t49  = t47 * t45 + 1;
    real_type t50  = sqrt(t49);
    real_type t52  = 1.0 / t50 * t44;
    real_type t53  = t44 * t44;
    real_type t56  = ModelPars[109];
    real_type t57  = alpha__r__XO + t56;
    real_type t58  = t57 * t57;
    real_type t63  = atan(t57 * t52);
    real_type t65  = sin(t63 * t38);
    real_type t68  = atan(t56 * t52);
    real_type t70  = cos(t68 * t38);
    return -1.0 / t70 * t65 / (t58 / t49 * t53 + 1) * t52 * t38 * t36 * Fzr__XO * t10;
  }

  real_type
  General::Fxr_D_3_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[207] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t13  = ModelPars[78] * ModelPars[165];
    real_type t23  = exp(t5 * t3 * ModelPars[88]);
    real_type t34  = atan(lambda__r__XO / (Fzr__XO * t10 * t13 + ModelPars[161]) * ModelPars[167] * t23 * (t5 * t3 * ModelPars[86] + ModelPars[84]) * Fzr__XO);
    real_type t36  = sin(t34 * t13);
    real_type t37  = t36 * Fzr__XO;
    real_type t38  = ModelPars[105];
    real_type t44  = phi__XO * ModelPars[97] + ModelPars[93];
    real_type t45  = t44 * t44;
    real_type t47  = lambda__r__XO * lambda__r__XO;
    real_type t49  = ModelPars[95] * ModelPars[95];
    real_type t51  = t49 * t47 + 1;
    real_type t52  = sqrt(t51);
    real_type t57  = 1.0 / t51 * t45;
    real_type t58  = ModelPars[109];
    real_type t59  = alpha__r__XO + t58;
    real_type t60  = t59 * t59;
    real_type t63  = pow(t60 * t57 + 1, 2);
    real_type t64  = 1.0 / t63;
    real_type t67  = 1.0 / t52 * t44;
    real_type t69  = atan(t59 * t67);
    real_type t70  = t69 * t38;
    real_type t71  = sin(t70);
    real_type t73  = atan(t58 * t67);
    real_type t75  = cos(t73 * t38);
    real_type t76  = 1.0 / t75;
    real_type t82  = t38 * t38;
    real_type t85  = cos(t70);
    return 2 * t59 * t76 * t71 * t64 / t52 / t51 * t45 * t44 * t38 * t37 * t10 - t76 * t85 * t64 * t57 * t82 * t37 * t10;
  }

  real_type
  General::Fxr_D_3_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[207] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t11  = Fzr__XO * Fzr__XO;
    real_type t12  = ModelPars[78];
    real_type t15  = ModelPars[165];
    real_type t20  = t5 * t3 * ModelPars[86] + ModelPars[84];
    real_type t25  = exp(t5 * t3 * ModelPars[88]);
    real_type t26  = ModelPars[167];
    real_type t30  = t15 * t12;
    real_type t31  = t10 * Fzr__XO;
    real_type t34  = t31 * t30 + ModelPars[161];
    real_type t35  = 1.0 / t34;
    real_type t36  = t20 * t20;
    real_type t38  = t25 * t25;
    real_type t40  = t26 * t26;
    real_type t41  = t34 * t34;
    real_type t44  = lambda__r__XO * lambda__r__XO;
    real_type t55  = atan(lambda__r__XO * t35 * t26 * t25 * t20 * Fzr__XO);
    real_type t56  = t55 * t30;
    real_type t57  = cos(t56);
    real_type t58  = ModelPars[105];
    real_type t64  = phi__XO * ModelPars[97] + ModelPars[93];
    real_type t66  = ModelPars[95] * ModelPars[95];
    real_type t68  = t66 * t44 + 1;
    real_type t69  = sqrt(t68);
    real_type t71  = 1.0 / t69 * t64;
    real_type t72  = t64 * t64;
    real_type t74  = 1.0 / t68 * t72;
    real_type t75  = ModelPars[109];
    real_type t76  = alpha__r__XO + t75;
    real_type t77  = t76 * t76;
    real_type t79  = t77 * t74 + 1;
    real_type t80  = 1.0 / t79;
    real_type t82  = atan(t76 * t71);
    real_type t83  = t82 * t58;
    real_type t84  = sin(t83);
    real_type t85  = t84 * t80;
    real_type t87  = atan(t75 * t71);
    real_type t88  = t87 * t58;
    real_type t89  = cos(t88);
    real_type t90  = 1.0 / t89;
    real_type t95  = sin(t56);
    real_type t96  = t58 * t95;
    real_type t110 = t68 * t68;
    real_type t113 = t79 * t79;
    real_type t114 = 1.0 / t113;
    real_type t118 = lambda__r__XO * t66;
    real_type t123 = t58 * t58;
    real_type t124 = t123 * t95;
    real_type t127 = 1.0 / t110;
    real_type t130 = cos(t83);
    real_type t138 = t89 * t89;
    real_type t142 = t75 * t75;
    real_type t146 = sin(t88);
    return -t90 * t85 * t71 * t58 * t57 / (t44 / t41 * t40 * t38 * t36 * t11 + 1) * t35 * t26 * t25 * t20 * t15 * t12 * t11 * t10 + lambda__r__XO * t66 * t90 * t84 * t80 / t69 / t68 * t64 * t96 * t31 - 2 * t118 * t77 * t90 * t84 * t114 / t69 / t110 * t72 * t64 * t96 * t31 + t90 * t130 * t118 * t76 * t114 * t127 * t72 * t124 * t31 + t146 / (t142 * t74 + 1) * t118 * t75 / t138 * t85 * t127 * t72 * t124 * t31;
  }

  real_type
  General::Fxr_D_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[207] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t11  = Fzr__XO * Fzr__XO;
    real_type t15  = ModelPars[78] * ModelPars[165];
    real_type t20  = t5 * t3 * ModelPars[86] + ModelPars[84];
    real_type t26  = exp(t5 * t3 * ModelPars[88]);
    real_type t27  = ModelPars[167];
    real_type t29  = Fzr__XO * t10;
    real_type t32  = t29 * t15 + ModelPars[161];
    real_type t33  = 1.0 / t32;
    real_type t35  = t20 * t20;
    real_type t37  = t26 * t26;
    real_type t39  = t27 * t27;
    real_type t40  = t32 * t32;
    real_type t43  = lambda__r__XO * lambda__r__XO;
    real_type t53  = atan(lambda__r__XO * t33 * t27 * t26 * t20 * Fzr__XO);
    real_type t54  = t53 * t15;
    real_type t55  = cos(t54);
    real_type t57  = ModelPars[105];
    real_type t61  = phi__XO * ModelPars[97] + ModelPars[93];
    real_type t63  = ModelPars[95] * ModelPars[95];
    real_type t65  = t63 * t43 + 1;
    real_type t66  = sqrt(t65);
    real_type t68  = 1.0 / t66 * t61;
    real_type t69  = ModelPars[109];
    real_type t70  = alpha__r__XO + t69;
    real_type t72  = atan(t70 * t68);
    real_type t73  = t72 * t57;
    real_type t74  = cos(t73);
    real_type t76  = atan(t69 * t68);
    real_type t77  = t76 * t57;
    real_type t78  = cos(t77);
    real_type t79  = 1.0 / t78;
    real_type t84  = sin(t54);
    real_type t89  = 1.0 / t66 / t65;
    real_type t92  = t61 * t61;
    real_type t94  = 1.0 / t65 * t92;
    real_type t95  = t70 * t70;
    real_type t100 = sin(t73);
    real_type t106 = t78 * t78;
    real_type t114 = t69 * t69;
    real_type t118 = sin(t77);
    return t79 * t74 * t55 / (t43 / t40 * t39 * t37 * t35 * t11 + 1) * t33 * t27 * t26 * t20 * t15 * t11 * t10 + t79 * t100 / (t95 * t94 + 1) * lambda__r__XO * t63 * t70 * t89 * t61 * t57 * t84 * t29 - t118 / (t114 * t94 + 1) * lambda__r__XO * t63 * t69 * t89 * t61 * t57 / t106 * t74 * t84 * t29;
  }

  real_type
  General::Fxr_D_4_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[207] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t11  = Fzr__XO * Fzr__XO;
    real_type t12  = t11 * t11;
    real_type t14  = ModelPars[78];
    real_type t15  = ModelPars[165];
    real_type t16  = t15 * t14;
    real_type t21  = t5 * t3 * ModelPars[86] + ModelPars[84];
    real_type t22  = t21 * t21;
    real_type t27  = exp(t5 * t3 * ModelPars[88]);
    real_type t28  = t27 * t27;
    real_type t33  = ModelPars[167];
    real_type t34  = t33 * t33;
    real_type t36  = Fzr__XO * t10;
    real_type t39  = t36 * t16 + ModelPars[161];
    real_type t40  = t39 * t39;
    real_type t46  = 1.0 / t40;
    real_type t48  = lambda__r__XO * lambda__r__XO;
    real_type t51  = t48 * t46 * t34 * t28 * t22 * t11 + 1;
    real_type t52  = t51 * t51;
    real_type t53  = 1.0 / t52;
    real_type t58  = 1.0 / t39 * t33;
    real_type t61  = atan(lambda__r__XO * t58 * t27 * t21 * Fzr__XO);
    real_type t62  = t61 * t16;
    real_type t63  = cos(t62);
    real_type t64  = ModelPars[105];
    real_type t68  = phi__XO * ModelPars[97] + ModelPars[93];
    real_type t70  = ModelPars[95] * ModelPars[95];
    real_type t72  = t70 * t48 + 1;
    real_type t73  = sqrt(t72);
    real_type t75  = 1.0 / t73 * t68;
    real_type t76  = ModelPars[109];
    real_type t77  = alpha__r__XO + t76;
    real_type t79  = atan(t77 * t75);
    real_type t80  = t79 * t64;
    real_type t81  = cos(t80);
    real_type t82  = t81 * t63;
    real_type t84  = atan(t76 * t75);
    real_type t85  = t84 * t64;
    real_type t86  = cos(t85);
    real_type t87  = 1.0 / t86;
    real_type t95  = t14 * t14;
    real_type t96  = t15 * t15;
    real_type t102 = sin(t62);
    real_type t104 = t87 * t81;
    real_type t115 = 1.0 / t51 * t58 * t27 * t21 * t15 * t14 * t11 * t10;
    real_type t118 = 1.0 / t73 / t72;
    real_type t123 = t68 * t68;
    real_type t125 = 1.0 / t72 * t123;
    real_type t126 = t77 * t77;
    real_type t128 = t126 * t125 + 1;
    real_type t129 = 1.0 / t128;
    real_type t130 = sin(t80);
    real_type t131 = t130 * t129;
    real_type t132 = t87 * t131;
    real_type t137 = t86 * t86;
    real_type t138 = 1.0 / t137;
    real_type t139 = t64 * t138;
    real_type t144 = t76 * t76;
    real_type t146 = t144 * t125 + 1;
    real_type t147 = 1.0 / t146;
    real_type t149 = sin(t85);
    real_type t155 = t64 * t102;
    real_type t157 = t68 * t155 * t36;
    real_type t158 = t72 * t72;
    real_type t160 = 1.0 / t73 / t158;
    real_type t162 = t70 * t70;
    real_type t165 = t87 * t130;
    real_type t174 = t123 * t68;
    real_type t177 = t158 * t72;
    real_type t179 = 1.0 / t73 / t177;
    real_type t183 = t128 * t128;
    real_type t185 = 1.0 / t183 * t48;
    real_type t190 = t64 * t64;
    real_type t194 = 1.0 / t177;
    real_type t202 = t123 * t190;
    real_type t206 = t48 * t162;
    real_type t209 = t149 * t147;
    real_type t214 = t81 * t102;
    real_type t222 = t146 * t146;
    real_type t223 = 1.0 / t222;
    real_type t224 = t149 * t149;
    real_type t231 = t139 * t214 * t36;
    return -2 * lambda__r__XO * t87 * t82 * t53 / t40 / t39 * t34 * t33 * t28 * t27 * t22 * t21 * t16 * t12 * t10 - t104 * t102 * t53 * t46 * t34 * t28 * t22 * t96 * t95 * t11 * Fzr__XO * t10 + 2 * t132 * lambda__r__XO * t70 * t77 * t118 * t68 * t64 * t63 * t115 - 2 * t149 * t147 * lambda__r__XO * t70 * t76 * t118 * t68 * t139 * t82 * t115 - 3 * t165 * t129 * t48 * t162 * t77 * t160 * t157 + t132 * t70 * t77 * t118 * t157 + 2 * t165 * t185 * t162 * t126 * t77 * t179 * t174 * t155 * t36 - t104 * t185 * t162 * t126 * t194 * t123 * t190 * t102 * t36 - 2 * t209 * t76 * t138 * t131 * t206 * t77 * t194 * t202 * t102 * Fzr__XO * t10 + 2 * t224 * t223 * t206 * t144 * t194 * t123 * t190 / t137 / t86 * t214 * t36 + 3 * t209 * t206 * t76 * t160 * t68 * t231 - t209 * t70 * t76 * t118 * t68 * t64 * t138 * t214 * t36 - 2 * t149 * t223 * t206 * t144 * t76 * t179 * t174 * t231 + t223 * t48 * t162 * t144 * t194 * t202 * t87 * t214 * t36;
  }

  real_type
  General::Fyf( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[39];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t8   = ModelPars[45] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t12  = sqrt(1.0 / t9 * t3 * t2);
    real_type t13  = ModelPars[47];
    real_type t15  = ModelPars[6];
    real_type t18  = Fzf__XO - t15;
    real_type t20  = t15 * ModelPars[33] + t18 * ModelPars[35];
    real_type t23  = ModelPars[41] * t5 + 1;
    real_type t28  = 1.0 / t1;
    real_type t37  = 1.0 / t20;
    real_type t45  = ModelPars[43] * phi__f__XO;
    real_type t55  = atan(((-t23 * t37 * phi__f__XO * ModelPars[37] * Fzf__XO + alpha__f__XO) / t12 / t8 * Fzf__XO * t1 - t23 * t37 * t8 * t28 * t12 * t45) * t8 / Fzf__XO * t28 / t13 / t23 * t20);
    real_type t57  = sin(t55 * t13);
    real_type t63  = ModelPars[106];
    real_type t66  = ModelPars[100] * ModelPars[100];
    real_type t67  = tan(alpha__f__XO);
    real_type t70  = pow(t67 - ModelPars[102], 2);
    real_type t73  = sqrt(t70 * t66 + 1);
    real_type t75  = 1.0 / t73 * ModelPars[98];
    real_type t76  = ModelPars[110];
    real_type t80  = 1.0 / t15 * t18 * ModelPars[112];
    real_type t83  = atan((lambda__f__XO + t76 + t80) * t75);
    real_type t85  = cos(t83 * t63);
    real_type t89  = atan((t76 + t80) * t75);
    real_type t91  = cos(t89 * t63);
    return 1.0 / t91 * t85 * (t8 * t28 * t12 * t45 + t57 * t12);
  }

  real_type
  General::Fyf_D_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[39];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t8   = ModelPars[45] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[47];
    real_type t16  = ModelPars[6];
    real_type t18  = ModelPars[35];
    real_type t19  = Fzf__XO - t16;
    real_type t21  = t16 * ModelPars[33] + t19 * t18;
    real_type t24  = ModelPars[41] * t5 + 1;
    real_type t25  = 1.0 / t24;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t25 * t21;
    real_type t29  = 1.0 / t1;
    real_type t31  = 1.0 / Fzf__XO * t29;
    real_type t32  = Fzf__XO * t1;
    real_type t33  = 1.0 / t8;
    real_type t34  = t13 * t33;
    real_type t35  = ModelPars[37];
    real_type t36  = Fzf__XO * t35;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__f__XO * t36 + alpha__f__XO;
    real_type t45  = ModelPars[43] * phi__f__XO;
    real_type t48  = t24 * t37;
    real_type t51  = -t48 * t8 * t29 * t12 * t45 + t41 * t34 * t32;
    real_type t52  = t51 * t8;
    real_type t53  = t52 * t31;
    real_type t55  = atan(t53 * t28);
    real_type t56  = t55 * t14;
    real_type t57  = sin(t56);
    real_type t66  = 1.0 / t3;
    real_type t70  = t33 * t1;
    real_type t85  = t21 * t21;
    real_type t86  = 1.0 / t85;
    real_type t99  = t29 * t12;
    real_type t110 = t24 * t24;
    real_type t113 = t14 * t14;
    real_type t118 = t51 * t51;
    real_type t125 = cos(t56);
    real_type t132 = ModelPars[106];
    real_type t133 = ModelPars[98];
    real_type t135 = ModelPars[100] * ModelPars[100];
    real_type t136 = tan(alpha__f__XO);
    real_type t139 = pow(t136 - ModelPars[102], 2);
    real_type t141 = t139 * t135 + 1;
    real_type t142 = sqrt(t141);
    real_type t143 = 1.0 / t142;
    real_type t144 = t143 * t133;
    real_type t145 = ModelPars[110];
    real_type t146 = ModelPars[112];
    real_type t148 = 1.0 / t16;
    real_type t149 = t148 * t19 * t146;
    real_type t150 = lambda__f__XO + t145 + t149;
    real_type t152 = atan(t150 * t144);
    real_type t153 = t152 * t132;
    real_type t154 = cos(t153);
    real_type t156 = t145 + t149;
    real_type t158 = atan(t156 * t144);
    real_type t159 = t158 * t132;
    real_type t160 = cos(t159);
    real_type t161 = 1.0 / t160;
    real_type t166 = t8 * t99 * t45 + t57 * t12;
    real_type t170 = t133 * t133;
    real_type t172 = 1.0 / t141 * t170;
    real_type t173 = t150 * t150;
    real_type t177 = sin(t153);
    real_type t183 = t160 * t160;
    real_type t189 = t156 * t156;
    real_type t194 = sin(t159);
    return t161 * t154 * (t10 * Fzf__XO * t2 * t57 * t13 + t125 / (t118 * t9 * t66 / t2 / t113 / t110 * t85 + 1) * (t53 * t27 * t25 * t18 - t52 * t66 * t29 * t28 + (t41 * t13 * t70 - t41 / t12 / t11 / t9 / t8 * t3 * t2 * t1 + (t18 * t24 * t86 * phi__f__XO * t36 - t48 * phi__f__XO * t35) * t34 * t32 - Fzf__XO * t24 * t37 * t33 * t1 * t13 * t45 + t18 * t24 * t86 * t8 * t99 * t45) * t8 * t31 * t28) * t14 * t12 + Fzf__XO * t70 * t13 * t45) - t161 * t177 / (t173 * t172 + 1) * t148 * t146 * t144 * t132 * t166 + t194 / (t189 * t172 + 1) * t148 * t146 * t143 * t133 * t132 / t183 * t154 * t166;
  }

  real_type
  General::Fyf_D_1_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[39];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t8   = ModelPars[45] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t14  = 1.0 / t12 / t11;
    real_type t15  = ModelPars[47];
    real_type t17  = ModelPars[6];
    real_type t19  = ModelPars[35];
    real_type t20  = Fzf__XO - t17;
    real_type t22  = t17 * ModelPars[33] + t20 * t19;
    real_type t25  = ModelPars[41] * t5 + 1;
    real_type t26  = 1.0 / t25;
    real_type t28  = 1.0 / t15;
    real_type t29  = t28 * t26 * t22;
    real_type t30  = 1.0 / t1;
    real_type t32  = 1.0 / Fzf__XO * t30;
    real_type t33  = Fzf__XO * t1;
    real_type t34  = 1.0 / t8;
    real_type t35  = 1.0 / t12;
    real_type t36  = t35 * t34;
    real_type t37  = ModelPars[37];
    real_type t38  = Fzf__XO * t37;
    real_type t39  = 1.0 / t22;
    real_type t43  = -t25 * t39 * phi__f__XO * t38 + alpha__f__XO;
    real_type t47  = ModelPars[43] * phi__f__XO;
    real_type t50  = t25 * t39;
    real_type t53  = -t50 * t8 * t30 * t12 * t47 + t43 * t36 * t33;
    real_type t54  = t53 * t8;
    real_type t55  = t54 * t32;
    real_type t57  = atan(t55 * t29);
    real_type t58  = t57 * t15;
    real_type t59  = sin(t58);
    real_type t61  = t2 * t2;
    real_type t63  = t9 * t9;
    real_type t64  = 1.0 / t63;
    real_type t69  = t28 * t26 * t19;
    real_type t71  = 1.0 / t3;
    real_type t72  = t71 * t30;
    real_type t73  = t54 * t72;
    real_type t75  = t34 * t1;
    real_type t78  = t2 * t1;
    real_type t79  = t3 * t78;
    real_type t81  = 1.0 / t9 / t8;
    real_type t82  = t14 * t81;
    real_type t85  = phi__f__XO * t37;
    real_type t87  = phi__f__XO * t38;
    real_type t88  = t22 * t22;
    real_type t89  = 1.0 / t88;
    real_type t91  = t19 * t25 * t89;
    real_type t93  = -t50 * t85 + t91 * t87;
    real_type t96  = t1 * t35;
    real_type t97  = t96 * t47;
    real_type t99  = Fzf__XO * t25;
    real_type t102 = t30 * t12;
    real_type t103 = t102 * t47;
    real_type t108 = t19 * t25 * t89 * t8 * t103 - t99 * t39 * t34 * t97 + t93 * t36 * t33 + t43 * t35 * t75 - t43 * t82 * t79;
    real_type t109 = t108 * t8;
    real_type t110 = t109 * t32;
    real_type t112 = t110 * t29 - t73 * t29 + t55 * t69;
    real_type t113 = t25 * t25;
    real_type t114 = 1.0 / t113;
    real_type t115 = t114 * t88;
    real_type t116 = t15 * t15;
    real_type t117 = 1.0 / t116;
    real_type t118 = t117 * t115;
    real_type t119 = 1.0 / t2;
    real_type t121 = t53 * t53;
    real_type t122 = t121 * t9;
    real_type t125 = t122 * t71 * t119 * t118 + 1;
    real_type t126 = 1.0 / t125;
    real_type t127 = t126 * t112;
    real_type t129 = cos(t58);
    real_type t135 = t59 * t35;
    real_type t138 = t15 * t12;
    real_type t143 = t3 * Fzf__XO;
    real_type t144 = 1.0 / t143;
    real_type t152 = t81 * t78;
    real_type t164 = t3 * t3;
    real_type t178 = 1.0 / t88 / t22;
    real_type t180 = t19 * t19;
    real_type t198 = t35 * t47;
    real_type t215 = t125 * t125;
    real_type t216 = 1.0 / t215;
    real_type t219 = t119 * t117;
    real_type t221 = t9 * t71;
    real_type t237 = t112 * t112;
    real_type t247 = ModelPars[106];
    real_type t248 = ModelPars[98];
    real_type t250 = ModelPars[100] * ModelPars[100];
    real_type t251 = tan(alpha__f__XO);
    real_type t254 = pow(t251 - ModelPars[102], 2);
    real_type t256 = t254 * t250 + 1;
    real_type t257 = sqrt(t256);
    real_type t258 = 1.0 / t257;
    real_type t259 = t258 * t248;
    real_type t260 = ModelPars[110];
    real_type t261 = ModelPars[112];
    real_type t263 = 1.0 / t17;
    real_type t264 = t263 * t20 * t261;
    real_type t265 = lambda__f__XO + t260 + t264;
    real_type t267 = atan(t265 * t259);
    real_type t268 = t267 * t247;
    real_type t269 = cos(t268);
    real_type t271 = t260 + t264;
    real_type t273 = atan(t271 * t259);
    real_type t274 = t273 * t247;
    real_type t275 = cos(t274);
    real_type t276 = 1.0 / t275;
    real_type t285 = t10 * Fzf__XO * t2 * t135 + Fzf__XO * t75 * t198 + t129 * t127 * t138;
    real_type t289 = t248 * t248;
    real_type t290 = 1.0 / t256;
    real_type t291 = t290 * t289;
    real_type t292 = t265 * t265;
    real_type t294 = t292 * t291 + 1;
    real_type t295 = 1.0 / t294;
    real_type t296 = sin(t268);
    real_type t303 = t275 * t275;
    real_type t304 = 1.0 / t303;
    real_type t305 = t247 * t304;
    real_type t309 = t271 * t271;
    real_type t311 = t309 * t291 + 1;
    real_type t312 = 1.0 / t311;
    real_type t314 = sin(t274);
    real_type t322 = t8 * t102 * t47 + t59 * t12;
    real_type t324 = t289 * t248;
    real_type t326 = 1.0 / t257 / t256;
    real_type t328 = t261 * t261;
    real_type t331 = t17 * t17;
    real_type t332 = 1.0 / t331;
    real_type t333 = t294 * t294;
    real_type t334 = 1.0 / t333;
    real_type t341 = t247 * t247;
    real_type t342 = t341 * t322;
    real_type t344 = t332 * t328;
    real_type t358 = t269 * t322;
    real_type t365 = t311 * t311;
    real_type t366 = 1.0 / t365;
    real_type t368 = t314 * t314;
    return t276 * t269 * (-t64 * t3 * t61 * t59 * t14 + 2 * t10 * Fzf__XO * t2 * t129 * t127 * t15 * t35 + t10 * t2 * t135 + t129 * t126 * (-2 * t73 * t69 + 2 * t110 * t69 + 2 * t54 * t144 * t30 * t29 - 2 * t109 * t72 * t29 + (-3 * Fzf__XO * t43 * t14 * t152 + 2 * t93 * t35 * t75 + 3 * t43 / t12 / t64 / t164 / t63 / t8 * t143 * t1 - 2 * t93 * t82 * t79 + (-2 * t180 * t25 * t178 * t87 + 2 * t91 * t85) * t36 * t33 + t3 * t25 * t39 * t81 * t78 * t14 * t47 + 2 * t19 * t99 * t89 * t34 * t97 - t50 * t75 * t198 - 2 * t180 * t25 * t178 * t8 * t103) * t8 * t32 * t29) * t138 - (2 * t19 * t121 * t221 * t219 * t114 * t22 + 2 * t108 * t53 * t221 * t219 * t115 - 2 * t122 * t144 * t119 * t118) * t129 * t216 * t112 * t138 - t59 * t216 * t237 * t116 * t12 - t3 * t152 * t14 * t47 + t34 * t96 * t47) - 2 * t276 * t296 * t295 * t263 * t261 * t259 * t247 * t285 + 2 * t314 * t312 * t263 * t261 * t258 * t248 * t305 * t269 * t285 + 2 * t265 * t276 * t296 * t334 * t332 * t328 * t326 * t324 * t247 * t322 - t276 * t269 * t334 * t344 * t291 * t342 - 2 * t314 * t312 * t304 * t296 * t295 * t332 * t328 * t291 * t342 + 2 * t368 * t366 * t332 * t328 * t290 * t289 * t341 / t303 / t275 * t358 - 2 * t271 * t314 * t366 * t332 * t328 * t326 * t324 * t305 * t358 + t366 * t344 * t291 * t341 * t276 * t358;
  }

  real_type
  General::Fyf_D_1_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[39];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t4   = t3 * t2;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t6   = ModelPars[45];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t4;
    real_type t12  = sqrt(t11);
    real_type t14  = 1.0 / t12 / t11;
    real_type t15  = ModelPars[47];
    real_type t17  = ModelPars[6];
    real_type t19  = ModelPars[35];
    real_type t20  = Fzf__XO - t17;
    real_type t22  = t17 * ModelPars[33] + t20 * t19;
    real_type t23  = ModelPars[41];
    real_type t25  = t23 * t5 + 1;
    real_type t26  = 1.0 / t25;
    real_type t27  = t26 * t22;
    real_type t28  = 1.0 / t15;
    real_type t29  = t28 * t27;
    real_type t30  = 1.0 / t1;
    real_type t31  = 1.0 / Fzf__XO;
    real_type t32  = t31 * t30;
    real_type t33  = Fzf__XO * t1;
    real_type t34  = 1.0 / t8;
    real_type t35  = 1.0 / t12;
    real_type t36  = t35 * t34;
    real_type t37  = ModelPars[37];
    real_type t38  = Fzf__XO * t37;
    real_type t39  = 1.0 / t22;
    real_type t43  = -t25 * t39 * phi__f__XO * t38 + alpha__f__XO;
    real_type t46  = ModelPars[43];
    real_type t47  = phi__f__XO * t46;
    real_type t49  = t8 * t30;
    real_type t50  = t25 * t39;
    real_type t53  = -t50 * t49 * t12 * t47 + t43 * t36 * t33;
    real_type t54  = t53 * t8;
    real_type t55  = t54 * t32;
    real_type t57  = atan(t55 * t29);
    real_type t58  = t57 * t15;
    real_type t59  = sin(t58);
    real_type t61  = t2 * t2;
    real_type t63  = t3 * Fzf__XO;
    real_type t64  = t9 * t9;
    real_type t68  = phi__f__XO * t6;
    real_type t72  = t15 * t35;
    real_type t73  = t25 * t25;
    real_type t74  = 1.0 / t73;
    real_type t76  = t30 * t28;
    real_type t77  = t76 * t74 * t22;
    real_type t78  = t8 * t31;
    real_type t80  = phi__f__XO * t23 * t53;
    real_type t81  = t80 * t78;
    real_type t84  = t76 * t27;
    real_type t85  = t6 * t31;
    real_type t86  = t53 * phi__f__XO;
    real_type t87  = t86 * t85;
    real_type t90  = t10 * t33;
    real_type t91  = t43 * t35;
    real_type t95  = t2 * t1;
    real_type t97  = 1.0 / t64;
    real_type t98  = t97 * t63 * t95;
    real_type t108 = -2 * t23 * t39 * t5 * t38 - t50 * t38;
    real_type t111 = t12 * t46;
    real_type t112 = t30 * t111;
    real_type t116 = t5 * t46;
    real_type t117 = t1 * t35;
    real_type t118 = t117 * t116;
    real_type t119 = t39 * t10;
    real_type t125 = t12 * t116;
    real_type t130 = t23 * t39;
    real_type t134 = 2 * t6 * t3 * t25 * t119 * t118 - t25 * t39 * t8 * t112 - 2 * t50 * t6 * t30 * t125 + 2 * t68 * t43 * t14 * t98 + t108 * t36 * t33 - 2 * t130 * t49 * t125 - 2 * t68 * t91 * t90;
    real_type t135 = t134 * t8;
    real_type t136 = t135 * t32;
    real_type t138 = t136 * t29 - 2 * t81 * t77 + 2 * t87 * t84;
    real_type t139 = t22 * t22;
    real_type t140 = t74 * t139;
    real_type t141 = t15 * t15;
    real_type t142 = 1.0 / t141;
    real_type t144 = 1.0 / t2;
    real_type t145 = 1.0 / t3;
    real_type t147 = t53 * t53;
    real_type t151 = t147 * t9 * t145 * t144 * t142 * t140 + 1;
    real_type t152 = 1.0 / t151;
    real_type t153 = t152 * t138;
    real_type t155 = cos(t58);
    real_type t161 = t2 * t59 * t35;
    real_type t163 = 1.0 / t9 / t8;
    real_type t168 = t26 * t19;
    real_type t169 = t28 * t168;
    real_type t171 = t145 * t30;
    real_type t174 = t34 * t1;
    real_type t176 = t3 * t95;
    real_type t177 = t14 * t163;
    real_type t183 = 1.0 / t139;
    real_type t184 = t25 * t183;
    real_type t185 = t19 * t184;
    real_type t187 = t185 * phi__f__XO * t38 - t50 * phi__f__XO * t37;
    real_type t191 = t39 * t34;
    real_type t192 = Fzf__XO * t25;
    real_type t193 = t192 * t191;
    real_type t195 = t30 * t12;
    real_type t197 = t183 * t8;
    real_type t198 = t19 * t25;
    real_type t199 = t198 * t197;
    real_type t201 = -t193 * t117 * t47 - t43 * t177 * t176 + t187 * t36 * t33 + t199 * t195 * t47 + t91 * t174;
    real_type t205 = t201 * t8 * t32 * t29 - t54 * t171 * t29 + t55 * t169;
    real_type t214 = t15 * t12;
    real_type t223 = t8 * t145;
    real_type t242 = t10 * t1;
    real_type t243 = t35 * t242;
    real_type t248 = t97 * t95;
    real_type t257 = t3 * t3;
    real_type t294 = t35 * t46;
    real_type t320 = t195 * t116;
    real_type t329 = -2 * phi__f__XO * t6 * t43 * t243 + 8 * t68 * t3 * t43 * t14 * t248 + t108 * t35 * t174 - 6 * t68 * t43 / t12 / t97 / t64 / t9 * t1 - t108 * t177 * t176 - 2 * t68 * t187 * t35 * t90 + 2 * t68 * t187 * t14 * t98 + (2 * t19 * t23 * t183 * t5 * t38 - 2 * t130 * t5 * t37 - t25 * t39 * t37 + t185 * t38) * t36 * t33 - t193 * t1 * t294 - 2 * t6 * t63 * t25 * t39 * t97 * t95 * t14 * t116 + 2 * t6 * t192 * t119 * t118 - 2 * Fzf__XO * t23 * t191 * t118 + t199 * t112 - 2 * t6 * t3 * t19 * t184 * t243 * t116 + 2 * t198 * t183 * t6 * t320 + 2 * t19 * t23 * t197 * t320;
    real_type t338 = t151 * t151;
    real_type t339 = 1.0 / t338;
    real_type t344 = t144 * t142;
    real_type t346 = t9 * t145;
    real_type t352 = t344 * t140;
    real_type t377 = t35 * t116;
    real_type t383 = ModelPars[106];
    real_type t384 = ModelPars[98];
    real_type t386 = ModelPars[100] * ModelPars[100];
    real_type t387 = tan(alpha__f__XO);
    real_type t390 = pow(t387 - ModelPars[102], 2);
    real_type t392 = t390 * t386 + 1;
    real_type t393 = sqrt(t392);
    real_type t394 = 1.0 / t393;
    real_type t395 = t394 * t384;
    real_type t396 = ModelPars[110];
    real_type t397 = ModelPars[112];
    real_type t399 = 1.0 / t17;
    real_type t400 = t399 * t20 * t397;
    real_type t401 = lambda__f__XO + t396 + t400;
    real_type t403 = atan(t401 * t395);
    real_type t404 = t403 * t383;
    real_type t405 = cos(t404);
    real_type t407 = t396 + t400;
    real_type t409 = atan(t407 * t395);
    real_type t410 = t409 * t383;
    real_type t411 = cos(t410);
    real_type t412 = 1.0 / t411;
    real_type t428 = -2 * t68 * t163 * t3 * t161 - 2 * t6 * t3 * t242 * t377 + 2 * t6 * t195 * t116 + t155 * t153 * t214 + t49 * t111;
    real_type t432 = t384 * t384;
    real_type t434 = 1.0 / t392 * t432;
    real_type t435 = t401 * t401;
    real_type t439 = sin(t404);
    real_type t445 = t411 * t411;
    real_type t451 = t407 * t407;
    real_type t456 = sin(t410);
    return t412 * t405 * (2 * t68 / t64 / t8 * t63 * t61 * t59 * t14 + t10 * Fzf__XO * t2 * t155 * t153 * t72 - 4 * t68 * t163 * Fzf__XO * t161 - 2 * phi__f__XO * t6 * t163 * t4 * t155 * t152 * t205 * t72 + t155 * t152 * (-2 * phi__f__XO * t23 * t201 * t78 * t77 - 2 * t86 * t6 * t145 * t84 - 2 * t81 * t76 * t74 * t19 + 2 * t201 * phi__f__XO * t85 * t84 + t329 * t8 * t32 * t29 - t135 * t171 * t29 + 2 * t87 * t76 * t168 + 2 * t80 * t223 * t77 + t136 * t169) * t214 - (-4 * phi__f__XO * t23 * t147 * t346 * t344 / t73 / t25 * t139 + 4 * phi__f__XO * t6 * t147 * t223 * t352 + 2 * t134 * t53 * t346 * t352) * t155 * t339 * t205 * t214 - t59 * t138 * t339 * t205 * t141 * t12 + Fzf__XO * t174 * t294 + 2 * t6 * t63 * t248 * t14 * t116 - 2 * t6 * Fzf__XO * t242 * t377) - t412 * t439 / (t435 * t434 + 1) * t399 * t397 * t395 * t383 * t428 + t456 / (t451 * t434 + 1) * t399 * t397 * t394 * t384 * t383 / t445 * t405 * t428;
  }

  real_type
  General::Fyf_D_1_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = 1.0 / Fzf__XO;
    real_type t3   = ModelPars[6];
    real_type t5   = ModelPars[35];
    real_type t6   = Fzf__XO - t3;
    real_type t8   = t3 * ModelPars[33] + t6 * t5;
    real_type t10  = phi__f__XO * phi__f__XO;
    real_type t13  = ModelPars[41] * t10 + 1;
    real_type t14  = 1.0 / t13;
    real_type t15  = t8 * t8;
    real_type t16  = t13 * t13;
    real_type t17  = 1.0 / t16;
    real_type t19  = ModelPars[47];
    real_type t20  = t19 * t19;
    real_type t23  = ModelPars[39];
    real_type t24  = t23 * t23;
    real_type t26  = Fzf__XO * Fzf__XO;
    real_type t27  = 1.0 / t26;
    real_type t31  = ModelPars[45] * t10 + 1;
    real_type t32  = t31 * t31;
    real_type t33  = Fzf__XO * t23;
    real_type t34  = 1.0 / t31;
    real_type t36  = 1.0 / t32;
    real_type t37  = t36 * t26 * t24;
    real_type t38  = sqrt(t37);
    real_type t39  = 1.0 / t38;
    real_type t40  = t39 * t34;
    real_type t41  = ModelPars[37];
    real_type t42  = Fzf__XO * t41;
    real_type t43  = 1.0 / t8;
    real_type t47  = -t13 * t43 * phi__f__XO * t42 + alpha__f__XO;
    real_type t51  = ModelPars[43] * phi__f__XO;
    real_type t53  = 1.0 / t23;
    real_type t55  = t13 * t43;
    real_type t58  = -t55 * t31 * t53 * t38 * t51 + t47 * t40 * t33;
    real_type t59  = t58 * t58;
    real_type t63  = t59 * t32 * t27 / t24 / t20 * t17 * t15 + 1;
    real_type t64  = 1.0 / t63;
    real_type t66  = t14 * t8;
    real_type t67  = 1.0 / t19;
    real_type t68  = t67 * t66;
    real_type t69  = t1 * t53;
    real_type t70  = t58 * t31;
    real_type t71  = t70 * t69;
    real_type t73  = atan(t71 * t68);
    real_type t74  = t73 * t19;
    real_type t75  = cos(t74);
    real_type t78  = t19 * t38;
    real_type t79  = t14 * t5;
    real_type t85  = t34 * t23;
    real_type t88  = t26 * t24 * t23;
    real_type t93  = 1.0 / t38 / t37 / t32 / t31;
    real_type t115 = 1.0 / t15;
    real_type t128 = t53 * t38;
    real_type t138 = t71 * t67 * t79 - t70 * t27 * t53 * t68 + (t47 * t39 * t85 - t47 * t93 * t88 + (t5 * t13 * t115 * phi__f__XO * t42 - t55 * phi__f__XO * t41) * t40 * t33 - Fzf__XO * t13 * t43 * t34 * t23 * t39 * t51 + t5 * t13 * t115 * t31 * t128 * t51) * t31 * t69 * t68;
    real_type t140 = t63 * t63;
    real_type t141 = 1.0 / t140;
    real_type t153 = sin(t74);
    real_type t157 = ModelPars[106];
    real_type t158 = ModelPars[98];
    real_type t160 = ModelPars[100] * ModelPars[100];
    real_type t161 = tan(alpha__f__XO);
    real_type t163 = t161 - ModelPars[102];
    real_type t164 = t163 * t163;
    real_type t166 = t164 * t160 + 1;
    real_type t167 = sqrt(t166);
    real_type t168 = 1.0 / t167;
    real_type t169 = t168 * t158;
    real_type t170 = ModelPars[110];
    real_type t171 = ModelPars[112];
    real_type t173 = 1.0 / t3;
    real_type t174 = t173 * t6 * t171;
    real_type t175 = lambda__f__XO + t170 + t174;
    real_type t177 = atan(t175 * t169);
    real_type t178 = t177 * t157;
    real_type t179 = cos(t178);
    real_type t181 = t170 + t174;
    real_type t183 = atan(t181 * t169);
    real_type t184 = t183 * t157;
    real_type t185 = cos(t184);
    real_type t186 = 1.0 / t185;
    real_type t198 = t36 * Fzf__XO * t24 * t153 * t39 + Fzf__XO * t85 * t39 * t51 + t75 * t64 * t138 * t78;
    real_type t201 = 1.0 / t167 / t166;
    real_type t205 = t163 * t160;
    real_type t206 = t161 * t161;
    real_type t207 = 1 + t206;
    real_type t209 = t158 * t158;
    real_type t211 = 1.0 / t166 * t209;
    real_type t212 = t175 * t175;
    real_type t214 = t212 * t211 + 1;
    real_type t215 = 1.0 / t214;
    real_type t216 = sin(t178);
    real_type t218 = t186 * t216 * t215;
    real_type t219 = t218 * t207 * t205;
    real_type t222 = t185 * t185;
    real_type t223 = 1.0 / t222;
    real_type t225 = t158 * t157;
    real_type t226 = t201 * t225;
    real_type t228 = t160 * t181;
    real_type t230 = t181 * t181;
    real_type t232 = t230 * t211 + 1;
    real_type t233 = 1.0 / t232;
    real_type t235 = sin(t184);
    real_type t239 = t64 * t66;
    real_type t250 = t31 * t128 * t51 + t153 * t38;
    real_type t251 = t157 * t250;
    real_type t257 = t209 * t158;
    real_type t259 = t166 * t166;
    real_type t262 = t171 / t167 / t259;
    real_type t265 = t214 * t214;
    real_type t266 = 1.0 / t265;
    real_type t270 = t207 * t163;
    real_type t275 = t157 * t157;
    real_type t277 = t209 * t275 * t250;
    real_type t278 = 1.0 / t259;
    real_type t279 = t171 * t278;
    real_type t291 = t223 * t216;
    real_type t293 = t235 * t233;
    real_type t301 = t173 * t171;
    real_type t302 = t293 * t301;
    real_type t312 = t179 * t250;
    real_type t316 = t209 * t275;
    real_type t319 = t232 * t232;
    real_type t320 = 1.0 / t319;
    real_type t321 = t320 * t173;
    real_type t322 = t235 * t235;
    real_type t324 = t270 * t228;
    real_type t328 = t223 * t312;
    return t186 * t179 * (t75 * t64 * t14 * t8 * t1 + t75 * t64 * (t39 * t67 * t79 - t39 * t1 * t67 * t66 + (t39 * t85 - t93 * t88) * t31 * t69 * t68) * t78 - 2 * t58 * t31 * t1 * t53 * t17 * t15 * t75 * t141 * t138 * t67 - t153 * t66 * t141 * t138 * t19) + t219 * t175 * t201 * t158 * t157 * t198 - t235 * t233 * t207 * t163 * t228 * t226 * t223 * t179 * t198 - t218 * t173 * t171 * t168 * t158 * t157 * t75 * t239 + t219 * t173 * t171 * t201 * t158 * t251 - 2 * t270 * t160 * t212 * t186 * t216 * t266 * t173 * t262 * t257 * t251 + t186 * t179 * t270 * t160 * t175 * t266 * t173 * t279 * t277 + t293 * t270 * t228 * t291 * t215 * t173 * t279 * t277 + t302 * t168 * t225 * t223 * t179 * t75 * t239 + t302 * t291 * t215 * t207 * t205 * t175 * t278 * t277 - 2 * t324 * t322 * t321 * t279 * t316 / t222 / t185 * t312 - t270 * t160 * t235 * t233 * t301 * t226 * t328 + 2 * t270 * t160 * t230 * t235 * t321 * t262 * t257 * t157 * t328 - t324 * t320 * t301 * t278 * t316 * t186 * t312;
  }

  real_type
  General::Fyf_D_1_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[39];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t8   = ModelPars[45] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[47];
    real_type t16  = ModelPars[6];
    real_type t18  = ModelPars[35];
    real_type t19  = Fzf__XO - t16;
    real_type t21  = t16 * ModelPars[33] + t19 * t18;
    real_type t24  = ModelPars[41] * t5 + 1;
    real_type t25  = 1.0 / t24;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t25 * t21;
    real_type t29  = 1.0 / t1;
    real_type t31  = 1.0 / Fzf__XO * t29;
    real_type t32  = Fzf__XO * t1;
    real_type t33  = 1.0 / t8;
    real_type t34  = t13 * t33;
    real_type t35  = ModelPars[37];
    real_type t36  = Fzf__XO * t35;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__f__XO * t36 + alpha__f__XO;
    real_type t45  = ModelPars[43] * phi__f__XO;
    real_type t48  = t24 * t37;
    real_type t51  = -t48 * t8 * t29 * t12 * t45 + t41 * t34 * t32;
    real_type t52  = t51 * t8;
    real_type t53  = t52 * t31;
    real_type t55  = atan(t53 * t28);
    real_type t56  = t55 * t14;
    real_type t57  = sin(t56);
    real_type t66  = 1.0 / t3;
    real_type t70  = t33 * t1;
    real_type t85  = t21 * t21;
    real_type t86  = 1.0 / t85;
    real_type t99  = t29 * t12;
    real_type t110 = t24 * t24;
    real_type t113 = t14 * t14;
    real_type t118 = t51 * t51;
    real_type t125 = cos(t56);
    real_type t132 = ModelPars[106];
    real_type t134 = ModelPars[98];
    real_type t137 = ModelPars[100] * ModelPars[100];
    real_type t138 = tan(alpha__f__XO);
    real_type t141 = pow(t138 - ModelPars[102], 2);
    real_type t143 = t141 * t137 + 1;
    real_type t144 = sqrt(t143);
    real_type t145 = 1.0 / t144;
    real_type t146 = t134 * t134;
    real_type t148 = 1.0 / t143 * t146;
    real_type t149 = ModelPars[110];
    real_type t150 = ModelPars[112];
    real_type t152 = 1.0 / t16;
    real_type t153 = t152 * t19 * t150;
    real_type t154 = lambda__f__XO + t149 + t153;
    real_type t155 = t154 * t154;
    real_type t157 = t155 * t148 + 1;
    real_type t158 = 1.0 / t157;
    real_type t160 = t145 * t134;
    real_type t162 = atan(t154 * t160);
    real_type t163 = t162 * t132;
    real_type t164 = sin(t163);
    real_type t165 = t149 + t153;
    real_type t167 = atan(t165 * t160);
    real_type t168 = t167 * t132;
    real_type t169 = cos(t168);
    real_type t170 = 1.0 / t169;
    real_type t171 = t170 * t164;
    real_type t177 = t8 * t99 * t45 + t57 * t12;
    real_type t185 = t157 * t157;
    real_type t186 = 1.0 / t185;
    real_type t192 = t132 * t132;
    real_type t193 = t192 * t177;
    real_type t196 = cos(t163);
    real_type t203 = t169 * t169;
    real_type t207 = t165 * t165;
    real_type t212 = sin(t168);
    return -t171 * t158 * t145 * t134 * t132 * (t10 * Fzf__XO * t2 * t57 * t13 + t125 / (t118 * t9 * t66 / t2 / t113 / t110 * t85 + 1) * (t53 * t27 * t25 * t18 - t52 * t66 * t29 * t28 + (t41 * t13 * t70 - t41 / t12 / t11 / t9 / t8 * t3 * t2 * t1 + (t18 * t24 * t86 * phi__f__XO * t36 - t48 * phi__f__XO * t35) * t34 * t32 - Fzf__XO * t24 * t37 * t33 * t1 * t13 * t45 + t18 * t24 * t86 * t8 * t99 * t45) * t8 * t31 * t28) * t14 * t12 + Fzf__XO * t70 * t13 * t45) + 2 * t154 * t171 * t186 * t152 * t150 / t144 / t143 * t146 * t134 * t132 * t177 - t170 * t196 * t186 * t152 * t150 * t148 * t193 - t212 / (t207 * t148 + 1) * t152 * t150 / t203 * t164 * t158 * t148 * t193;
  }

  real_type
  General::Fyf_D_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[39];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t6   = ModelPars[45];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[47];
    real_type t16  = ModelPars[6];
    real_type t19  = Fzf__XO - t16;
    real_type t21  = t16 * ModelPars[33] + t19 * ModelPars[35];
    real_type t22  = ModelPars[41];
    real_type t24  = t22 * t5 + 1;
    real_type t26  = 1.0 / t24 * t21;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t26;
    real_type t29  = 1.0 / t1;
    real_type t30  = 1.0 / Fzf__XO;
    real_type t31  = t30 * t29;
    real_type t32  = Fzf__XO * t1;
    real_type t34  = t13 / t8;
    real_type t36  = ModelPars[37] * Fzf__XO;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__f__XO * t36 + alpha__f__XO;
    real_type t44  = ModelPars[43];
    real_type t47  = t8 * t29;
    real_type t48  = t24 * t37;
    real_type t51  = -t48 * t47 * t12 * phi__f__XO * t44 + t41 * t34 * t32;
    real_type t55  = atan(t51 * t8 * t31 * t28);
    real_type t56  = t55 * t14;
    real_type t57  = sin(t56);
    real_type t63  = phi__f__XO * t6;
    real_type t68  = t24 * t24;
    real_type t69  = 1.0 / t68;
    real_type t71  = t29 * t27;
    real_type t93  = t9 * t9;
    real_type t110 = t12 * t44;
    real_type t115 = t5 * t44;
    real_type t124 = t12 * t115;
    real_type t138 = t21 * t21;
    real_type t140 = t14 * t14;
    real_type t146 = t51 * t51;
    real_type t153 = cos(t56);
    real_type t168 = ModelPars[106];
    real_type t171 = ModelPars[100] * ModelPars[100];
    real_type t172 = tan(alpha__f__XO);
    real_type t175 = pow(t172 - ModelPars[102], 2);
    real_type t178 = sqrt(t175 * t171 + 1);
    real_type t180 = 1.0 / t178 * ModelPars[98];
    real_type t181 = ModelPars[110];
    real_type t185 = 1.0 / t16 * t19 * ModelPars[112];
    real_type t188 = atan((lambda__f__XO + t181 + t185) * t180);
    real_type t190 = cos(t188 * t168);
    real_type t194 = atan((t181 + t185) * t180);
    real_type t196 = cos(t194 * t168);
    return 1.0 / t196 * t190 * (-2 * t63 / t9 / t8 * t3 * t2 * t57 * t13 + t153 / (t146 * t9 / t3 / t2 / t140 * t69 * t138 + 1) * (-2 * phi__f__XO * t22 * t51 * t8 * t30 * t71 * t69 * t21 + 2 * t51 * phi__f__XO * t6 * t30 * t71 * t26 + (-2 * t63 * t41 * t13 * t10 * t32 + 2 * t63 * t41 / t12 / t11 / t93 * t3 * Fzf__XO * t2 * t1 + (-2 * t22 * t37 * t5 * t36 - t48 * t36) * t34 * t32 - t24 * t37 * t8 * t29 * t110 + 2 * t6 * t3 * t24 * t37 * t10 * t1 * t13 * t115 - 2 * t48 * t6 * t29 * t124 - 2 * t22 * t37 * t47 * t124) * t8 * t31 * t28) * t14 * t12 + t47 * t110 - 2 * t6 * t3 * t10 * t1 * t13 * t115 + 2 * t6 * t29 * t12 * t115);
  }

  real_type
  General::Fyf_D_2_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[39];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t4   = t3 * t2;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t6   = ModelPars[45];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t4;
    real_type t12  = sqrt(t11);
    real_type t14  = 1.0 / t12 / t11;
    real_type t15  = ModelPars[47];
    real_type t17  = ModelPars[6];
    real_type t20  = Fzf__XO - t17;
    real_type t22  = t17 * ModelPars[33] + t20 * ModelPars[35];
    real_type t23  = ModelPars[41];
    real_type t25  = t23 * t5 + 1;
    real_type t27  = 1.0 / t25 * t22;
    real_type t28  = 1.0 / t15;
    real_type t29  = t28 * t27;
    real_type t30  = 1.0 / t1;
    real_type t31  = 1.0 / Fzf__XO;
    real_type t32  = t31 * t30;
    real_type t33  = Fzf__XO * t1;
    real_type t35  = 1.0 / t12;
    real_type t36  = t35 / t8;
    real_type t37  = ModelPars[37];
    real_type t38  = Fzf__XO * t37;
    real_type t39  = 1.0 / t22;
    real_type t43  = -t25 * t39 * phi__f__XO * t38 + alpha__f__XO;
    real_type t46  = ModelPars[43];
    real_type t49  = t8 * t30;
    real_type t50  = t25 * t39;
    real_type t53  = -t50 * t49 * t12 * phi__f__XO * t46 + t43 * t36 * t33;
    real_type t57  = atan(t53 * t8 * t32 * t29);
    real_type t58  = t57 * t15;
    real_type t59  = sin(t58);
    real_type t61  = t2 * t2;
    real_type t63  = t3 * t3;
    real_type t64  = t9 * t9;
    real_type t68  = t6 * t6;
    real_type t69  = t5 * t68;
    real_type t74  = t25 * t25;
    real_type t75  = 1.0 / t74;
    real_type t77  = t30 * t28;
    real_type t78  = t77 * t75 * t22;
    real_type t79  = t8 * t31;
    real_type t80  = t23 * t53;
    real_type t85  = t77 * t27;
    real_type t86  = t6 * t31;
    real_type t91  = t10 * t33;
    real_type t92  = t43 * t35;
    real_type t93  = phi__f__XO * t6;
    real_type t97  = t2 * t1;
    real_type t99  = t3 * Fzf__XO * t97;
    real_type t100 = 1.0 / t64;
    real_type t101 = t100 * t99;
    real_type t102 = t43 * t14;
    real_type t111 = -2 * t23 * t39 * t5 * t38 - t50 * t38;
    real_type t114 = t12 * t46;
    real_type t115 = t30 * t114;
    real_type t116 = t39 * t8;
    real_type t119 = t5 * t46;
    real_type t120 = t1 * t35;
    real_type t122 = t39 * t10;
    real_type t123 = t3 * t25;
    real_type t128 = t12 * t119;
    real_type t129 = t6 * t30;
    real_type t133 = t23 * t39;
    real_type t137 = 2 * t6 * t123 * t122 * t120 * t119 + 2 * t93 * t102 * t101 + t111 * t36 * t33 - t25 * t116 * t115 - 2 * t50 * t129 * t128 - 2 * t133 * t49 * t128 - 2 * t93 * t92 * t91;
    real_type t141 = t137 * t8 * t32 * t29 + 2 * t53 * phi__f__XO * t86 * t85 - 2 * phi__f__XO * t80 * t79 * t78;
    real_type t142 = t22 * t22;
    real_type t143 = t75 * t142;
    real_type t144 = t15 * t15;
    real_type t145 = 1.0 / t144;
    real_type t147 = 1.0 / t2;
    real_type t148 = 1.0 / t3;
    real_type t150 = t53 * t53;
    real_type t154 = t150 * t9 * t148 * t147 * t145 * t143 + 1;
    real_type t155 = 1.0 / t154;
    real_type t157 = cos(t58);
    real_type t160 = t9 * t8;
    real_type t161 = 1.0 / t160;
    real_type t168 = t2 * t59 * t35;
    real_type t177 = t15 * t12;
    real_type t179 = 1.0 / t74 / t25;
    real_type t182 = t23 * t23;
    real_type t214 = 1.0 / t64 / t8;
    real_type t250 = phi__f__XO * t23;
    real_type t254 = t35 * t46;
    real_type t269 = t5 * phi__f__XO * t46;
    real_type t278 = t120 * t269;
    real_type t293 = 8 * t69 * t92 * t161 * t33 - 20 * t69 * t102 * t214 * t99 - 4 * t93 * t111 * t35 * t91 - 2 * t6 * t92 * t91 + 12 * t69 * t43 / t12 / t100 / t64 / t160 * Fzf__XO * t1 + 4 * t93 * t111 * t14 * t101 + 2 * t6 * t102 * t101 - 6 * t250 * t39 * t37 * t36 * t3 * t1 + 6 * phi__f__XO * t6 * t3 * t50 * t10 * t1 * t254 - 6 * t50 * t93 * t115 - 6 * t250 * t116 * t115 + 4 * t68 * t63 * t25 * t39 * t214 * t97 * t14 * t269 - 4 * t68 * t123 * t39 * t161 * t278 + 8 * t6 * t3 * t23 * t122 * t278 - 8 * t133 * t129 * t12 * t269;
    real_type t302 = t154 * t154;
    real_type t303 = 1.0 / t302;
    real_type t306 = t147 * t145;
    real_type t308 = t9 * t148;
    real_type t314 = t306 * t143;
    real_type t329 = t141 * t141;
    real_type t353 = -4 * t69 / t64 / t9 * t63 * t61 * t59 * t14 - 4 * phi__f__XO * t6 * t161 * t4 * t157 * t155 * t141 * t15 * t35 + 12 * t69 * t100 * t3 * t168 - 2 * t6 * t161 * t3 * t168 + t157 * t155 * (8 * t5 * t182 * t53 * t79 * t77 * t179 * t22 - 4 * phi__f__XO * t23 * t137 * t79 * t78 - 8 * t23 * t53 * t5 * t86 * t78 + 4 * t137 * phi__f__XO * t86 * t85 + t293 * t8 * t32 * t29 + 2 * t53 * t6 * t32 * t29 - 2 * t80 * t79 * t78) * t177 - (-4 * phi__f__XO * t23 * t150 * t308 * t306 * t179 * t142 + 4 * phi__f__XO * t6 * t150 * t8 * t148 * t314 + 2 * t137 * t53 * t308 * t314) * t157 * t303 * t141 * t177 - t59 * t303 * t329 * t144 * t12 - 6 * t93 * t3 * t10 * t1 * t254 + 6 * phi__f__XO * t129 * t114 - 4 * t68 * t63 * t214 * t97 * t14 * t269 + 4 * t68 * t3 * t161 * t1 * t35 * t269;
    real_type t354 = ModelPars[106];
    real_type t357 = ModelPars[100] * ModelPars[100];
    real_type t358 = tan(alpha__f__XO);
    real_type t361 = pow(t358 - ModelPars[102], 2);
    real_type t364 = sqrt(t361 * t357 + 1);
    real_type t366 = 1.0 / t364 * ModelPars[98];
    real_type t367 = ModelPars[110];
    real_type t371 = 1.0 / t17 * t20 * ModelPars[112];
    real_type t374 = atan((lambda__f__XO + t367 + t371) * t366);
    real_type t376 = cos(t374 * t354);
    real_type t380 = atan((t367 + t371) * t366);
    real_type t382 = cos(t380 * t354);
    return 1.0 / t382 * t376 * t353;
  }

  real_type
  General::Fyf_D_2_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = phi__f__XO * phi__f__XO;
    real_type t2   = ModelPars[45];
    real_type t4   = t2 * t1 + 1;
    real_type t5   = 1.0 / t4;
    real_type t7   = ModelPars[6];
    real_type t10  = Fzf__XO - t7;
    real_type t12  = t10 * ModelPars[35] + t7 * ModelPars[33];
    real_type t14  = ModelPars[41];
    real_type t16  = t14 * t1 + 1;
    real_type t17  = 1.0 / t16;
    real_type t19  = t12 * t12;
    real_type t20  = t16 * t16;
    real_type t21  = 1.0 / t20;
    real_type t23  = ModelPars[47];
    real_type t24  = t23 * t23;
    real_type t27  = ModelPars[39];
    real_type t28  = t27 * t27;
    real_type t30  = Fzf__XO * Fzf__XO;
    real_type t33  = t4 * t4;
    real_type t34  = Fzf__XO * t27;
    real_type t36  = 1.0 / t33;
    real_type t37  = t36 * t30 * t28;
    real_type t38  = sqrt(t37);
    real_type t39  = 1.0 / t38;
    real_type t40  = t39 * t5;
    real_type t42  = ModelPars[37] * Fzf__XO;
    real_type t43  = 1.0 / t12;
    real_type t47  = -t16 * t43 * phi__f__XO * t42 + alpha__f__XO;
    real_type t50  = ModelPars[43];
    real_type t53  = 1.0 / t27;
    real_type t54  = t4 * t53;
    real_type t55  = t16 * t43;
    real_type t58  = -t55 * t54 * t38 * phi__f__XO * t50 + t47 * t40 * t34;
    real_type t59  = t58 * t58;
    real_type t63  = t59 * t33 / t30 / t28 / t24 * t21 * t19 + 1;
    real_type t64  = 1.0 / t63;
    real_type t65  = t17 * t12;
    real_type t66  = 1.0 / t23;
    real_type t67  = t66 * t65;
    real_type t68  = 1.0 / Fzf__XO;
    real_type t69  = t68 * t53;
    real_type t73  = atan(t58 * t4 * t69 * t67);
    real_type t74  = t73 * t23;
    real_type t75  = cos(t74);
    real_type t77  = phi__f__XO * t2;
    real_type t81  = t23 * t38;
    real_type t82  = t21 * t12;
    real_type t91  = t36 * t34;
    real_type t98  = t33 * t33;
    real_type t100 = 1.0 / t98 * t30 * Fzf__XO * t28 * t27;
    real_type t102 = 1.0 / t38 / t37;
    real_type t115 = t53 * t66;
    real_type t117 = t4 * t68;
    real_type t145 = t50 * t38;
    real_type t150 = t1 * t50;
    real_type t159 = t38 * t150;
    real_type t172 = -2 * phi__f__XO * t14 * t58 * t117 * t115 * t82 + 2 * t58 * phi__f__XO * t2 * t68 * t115 * t65 + (-2 * t77 * t47 * t39 * t91 + 2 * t77 * t47 * t102 * t100 + (-2 * t14 * t43 * t1 * t42 - t55 * t42) * t40 * t34 - t16 * t43 * t4 * t53 * t145 + 2 * t2 * t30 * t16 * t43 * t36 * t27 * t39 * t150 - 2 * t55 * t2 * t53 * t159 - 2 * t14 * t43 * t54 * t159) * t4 * t69 * t67;
    real_type t174 = t63 * t63;
    real_type t175 = 1.0 / t174;
    real_type t186 = sin(t74);
    real_type t190 = ModelPars[106];
    real_type t191 = ModelPars[98];
    real_type t193 = ModelPars[100] * ModelPars[100];
    real_type t194 = tan(alpha__f__XO);
    real_type t196 = t194 - ModelPars[102];
    real_type t197 = t196 * t196;
    real_type t199 = t197 * t193 + 1;
    real_type t200 = sqrt(t199);
    real_type t202 = 1.0 / t200 * t191;
    real_type t203 = ModelPars[110];
    real_type t207 = 1.0 / t7 * t10 * ModelPars[112];
    real_type t208 = lambda__f__XO + t203 + t207;
    real_type t210 = atan(t208 * t202);
    real_type t211 = t210 * t190;
    real_type t212 = cos(t211);
    real_type t214 = t203 + t207;
    real_type t216 = atan(t214 * t202);
    real_type t217 = t216 * t190;
    real_type t218 = cos(t217);
    real_type t219 = 1.0 / t218;
    real_type t243 = -2 * t77 / t33 / t4 * t30 * t28 * t186 * t39 + t75 * t64 * t172 * t81 + t54 * t145 - 2 * t2 * t30 * t36 * t27 * t39 * t150 + 2 * t2 * t53 * t38 * t150;
    real_type t246 = 1.0 / t200 / t199;
    real_type t251 = t194 * t194;
    real_type t252 = 1 + t251;
    real_type t254 = t191 * t191;
    real_type t256 = 1.0 / t199 * t254;
    real_type t257 = t208 * t208;
    real_type t261 = sin(t211);
    real_type t267 = t218 * t218;
    real_type t275 = t214 * t214;
    real_type t280 = sin(t217);
    return t219 * t212 * (-2 * t77 * t75 * t64 * t17 * t12 * t5 + t75 * t64 * (-2 * phi__f__XO * t14 * t39 * t66 * t82 + 2 * t40 * t77 * t67 + (2 * phi__f__XO * t2 * t102 * t100 - 2 * phi__f__XO * t2 * t39 * t91) * t4 * t69 * t67) * t81 - 2 * t58 * t117 * t53 * t21 * t19 * t75 * t175 * t172 * t66 - t186 * t65 * t175 * t172 * t23) + t219 * t261 / (t257 * t256 + 1) * t252 * t196 * t193 * t208 * t246 * t191 * t190 * t243 - t280 / (t275 * t256 + 1) * t252 * t196 * t193 * t214 * t246 * t191 * t190 / t267 * t212 * t243;
  }

  real_type
  General::Fyf_D_2_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[39];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t6   = ModelPars[45];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[47];
    real_type t16  = ModelPars[6];
    real_type t19  = Fzf__XO - t16;
    real_type t21  = t16 * ModelPars[33] + t19 * ModelPars[35];
    real_type t22  = ModelPars[41];
    real_type t24  = t22 * t5 + 1;
    real_type t26  = 1.0 / t24 * t21;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t26;
    real_type t29  = 1.0 / t1;
    real_type t30  = 1.0 / Fzf__XO;
    real_type t31  = t30 * t29;
    real_type t32  = Fzf__XO * t1;
    real_type t34  = t13 / t8;
    real_type t36  = ModelPars[37] * Fzf__XO;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__f__XO * t36 + alpha__f__XO;
    real_type t44  = ModelPars[43];
    real_type t47  = t8 * t29;
    real_type t48  = t24 * t37;
    real_type t51  = -t48 * t47 * t12 * phi__f__XO * t44 + t41 * t34 * t32;
    real_type t55  = atan(t51 * t8 * t31 * t28);
    real_type t56  = t55 * t14;
    real_type t57  = sin(t56);
    real_type t63  = phi__f__XO * t6;
    real_type t68  = t24 * t24;
    real_type t69  = 1.0 / t68;
    real_type t71  = t29 * t27;
    real_type t93  = t9 * t9;
    real_type t110 = t12 * t44;
    real_type t115 = t5 * t44;
    real_type t124 = t12 * t115;
    real_type t138 = t21 * t21;
    real_type t140 = t14 * t14;
    real_type t146 = t51 * t51;
    real_type t153 = cos(t56);
    real_type t168 = ModelPars[106];
    real_type t170 = ModelPars[98];
    real_type t173 = ModelPars[100] * ModelPars[100];
    real_type t174 = tan(alpha__f__XO);
    real_type t177 = pow(t174 - ModelPars[102], 2);
    real_type t179 = t177 * t173 + 1;
    real_type t180 = sqrt(t179);
    real_type t181 = 1.0 / t180;
    real_type t182 = t170 * t170;
    real_type t185 = ModelPars[110];
    real_type t189 = 1.0 / t16 * t19 * ModelPars[112];
    real_type t190 = lambda__f__XO + t185 + t189;
    real_type t191 = t190 * t190;
    real_type t196 = t181 * t170;
    real_type t198 = atan(t190 * t196);
    real_type t200 = sin(t198 * t168);
    real_type t203 = atan((t185 + t189) * t196);
    real_type t205 = cos(t203 * t168);
    return -1.0 / t205 * t200 / (t191 / t179 * t182 + 1) * t181 * t170 * t168 * (-2 * t63 / t9 / t8 * t3 * t2 * t57 * t13 + t153 / (t146 * t9 / t3 / t2 / t140 * t69 * t138 + 1) * (-2 * phi__f__XO * t22 * t51 * t8 * t30 * t71 * t69 * t21 + 2 * t51 * phi__f__XO * t6 * t30 * t71 * t26 + (-2 * t63 * t41 * t13 * t10 * t32 + 2 * t63 * t41 / t12 / t11 / t93 * t3 * Fzf__XO * t2 * t1 + (-2 * t22 * t37 * t5 * t36 - t48 * t36) * t34 * t32 - t24 * t37 * t8 * t29 * t110 + 2 * t6 * t3 * t24 * t37 * t10 * t1 * t13 * t115 - 2 * t48 * t6 * t29 * t124 - 2 * t22 * t37 * t47 * t124) * t8 * t31 * t28) * t14 * t12 + t47 * t110 - 2 * t6 * t3 * t10 * t1 * t13 * t115 + 2 * t6 * t29 * t12 * t115);
  }

  real_type
  General::Fyf_D_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t5   = Fzf__XO - t2;
    real_type t7   = t2 * ModelPars[33] + t5 * ModelPars[35];
    real_type t8   = phi__f__XO * phi__f__XO;
    real_type t11  = ModelPars[41] * t8 + 1;
    real_type t13  = 1.0 / t11 * t7;
    real_type t14  = t7 * t7;
    real_type t15  = t11 * t11;
    real_type t18  = ModelPars[47];
    real_type t19  = t18 * t18;
    real_type t22  = ModelPars[39];
    real_type t23  = t22 * t22;
    real_type t25  = Fzf__XO * Fzf__XO;
    real_type t30  = ModelPars[45] * t8 + 1;
    real_type t31  = t30 * t30;
    real_type t37  = sqrt(1.0 / t31 * t25 * t23);
    real_type t42  = 1.0 / t7;
    real_type t50  = ModelPars[43] * phi__f__XO;
    real_type t52  = 1.0 / t22;
    real_type t57  = (-t11 * t42 * phi__f__XO * ModelPars[37] * Fzf__XO + alpha__f__XO) / t37 / t30 * Fzf__XO * t22 - t11 * t42 * t30 * t52 * t37 * t50;
    real_type t58  = t57 * t57;
    real_type t72  = atan(t57 * t30 / Fzf__XO * t52 / t18 * t13);
    real_type t73  = t72 * t18;
    real_type t74  = cos(t73);
    real_type t75  = ModelPars[106];
    real_type t76  = ModelPars[98];
    real_type t78  = ModelPars[100] * ModelPars[100];
    real_type t79  = tan(alpha__f__XO);
    real_type t81  = t79 - ModelPars[102];
    real_type t82  = t81 * t81;
    real_type t84  = t82 * t78 + 1;
    real_type t85  = sqrt(t84);
    real_type t87  = 1.0 / t85 * t76;
    real_type t88  = ModelPars[110];
    real_type t92  = 1.0 / t2 * t5 * ModelPars[112];
    real_type t93  = lambda__f__XO + t88 + t92;
    real_type t95  = atan(t93 * t87);
    real_type t96  = t95 * t75;
    real_type t97  = cos(t96);
    real_type t99  = t88 + t92;
    real_type t101 = atan(t99 * t87);
    real_type t102 = t101 * t75;
    real_type t103 = cos(t102);
    real_type t104 = 1.0 / t103;
    real_type t107 = sin(t73);
    real_type t112 = t30 * t52 * t37 * t50 + t107 * t37;
    real_type t115 = 1.0 / t85 / t84;
    real_type t120 = t79 * t79;
    real_type t121 = 1 + t120;
    real_type t123 = t76 * t76;
    real_type t125 = 1.0 / t84 * t123;
    real_type t126 = t93 * t93;
    real_type t130 = sin(t96);
    real_type t136 = t103 * t103;
    real_type t144 = t99 * t99;
    real_type t149 = sin(t102);
    return t104 * t97 * t74 / (t58 * t31 / t25 / t23 / t19 / t15 * t14 + 1) * t13 + t104 * t130 / (t126 * t125 + 1) * t121 * t81 * t78 * t93 * t115 * t76 * t75 * t112 - t149 / (t144 * t125 + 1) * t121 * t81 * t78 * t99 * t115 * t76 * t75 / t136 * t97 * t112;
  }

  real_type
  General::Fyf_D_3_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t5   = Fzf__XO - t2;
    real_type t7   = t2 * ModelPars[33] + t5 * ModelPars[35];
    real_type t8   = t7 * t7;
    real_type t10  = phi__f__XO * phi__f__XO;
    real_type t13  = ModelPars[41] * t10 + 1;
    real_type t14  = t13 * t13;
    real_type t19  = 1.0 / t14 * t8;
    real_type t20  = ModelPars[47];
    real_type t21  = t20 * t20;
    real_type t22  = 1.0 / t21;
    real_type t24  = ModelPars[39];
    real_type t25  = t24 * t24;
    real_type t27  = Fzf__XO * Fzf__XO;
    real_type t32  = ModelPars[45] * t10 + 1;
    real_type t33  = t32 * t32;
    real_type t39  = sqrt(1.0 / t33 * t27 * t25);
    real_type t40  = 1.0 / t39;
    real_type t44  = 1.0 / t7;
    real_type t52  = ModelPars[43] * phi__f__XO;
    real_type t54  = 1.0 / t24;
    real_type t59  = (-t13 * t44 * phi__f__XO * ModelPars[37] * Fzf__XO + alpha__f__XO) * t40 / t32 * Fzf__XO * t24 - t13 * t44 * t32 * t54 * t39 * t52;
    real_type t60  = t59 * t59;
    real_type t64  = t60 * t33 / t27 / t25 * t22 * t19 + 1;
    real_type t65  = t64 * t64;
    real_type t66  = 1.0 / t65;
    real_type t69  = 1.0 / t13 * t7;
    real_type t72  = 1.0 / Fzf__XO;
    real_type t74  = t59 * t32;
    real_type t77  = atan(t74 * t72 * t54 / t20 * t69);
    real_type t78  = t77 * t20;
    real_type t79  = cos(t78);
    real_type t80  = ModelPars[106];
    real_type t81  = ModelPars[98];
    real_type t83  = ModelPars[100] * ModelPars[100];
    real_type t84  = tan(alpha__f__XO);
    real_type t86  = t84 - ModelPars[102];
    real_type t87  = t86 * t86;
    real_type t89  = t87 * t83 + 1;
    real_type t90  = sqrt(t89);
    real_type t92  = 1.0 / t90 * t81;
    real_type t93  = ModelPars[110];
    real_type t97  = 1.0 / t2 * t5 * ModelPars[112];
    real_type t98  = lambda__f__XO + t93 + t97;
    real_type t100 = atan(t98 * t92);
    real_type t101 = t100 * t80;
    real_type t102 = cos(t101);
    real_type t103 = t102 * t79;
    real_type t104 = t93 + t97;
    real_type t106 = atan(t104 * t92);
    real_type t107 = t106 * t80;
    real_type t108 = cos(t107);
    real_type t109 = 1.0 / t108;
    real_type t119 = sin(t78);
    real_type t125 = 1.0 / t64 * t69;
    real_type t128 = 1.0 / t90 / t89;
    real_type t129 = t128 * t81;
    real_type t134 = t84 * t84;
    real_type t135 = 1 + t134;
    real_type t136 = t81 * t81;
    real_type t138 = 1.0 / t89 * t136;
    real_type t139 = t98 * t98;
    real_type t141 = t139 * t138 + 1;
    real_type t142 = 1.0 / t141;
    real_type t144 = sin(t101);
    real_type t150 = t108 * t108;
    real_type t151 = 1.0 / t150;
    real_type t152 = t80 * t151;
    real_type t155 = t83 * t104;
    real_type t158 = t104 * t104;
    real_type t160 = t158 * t138 + 1;
    real_type t161 = 1.0 / t160;
    real_type t162 = sin(t107);
    real_type t163 = t162 * t161;
    real_type t172 = t32 * t54 * t39 * t52 + t119 * t39;
    real_type t173 = t80 * t172;
    real_type t174 = t89 * t89;
    real_type t176 = 1.0 / t90 / t174;
    real_type t180 = t83 * t83;
    real_type t181 = t87 * t180;
    real_type t182 = t135 * t135;
    real_type t183 = t182 * t181;
    real_type t185 = t109 * t144 * t142;
    real_type t203 = t136 * t81;
    real_type t204 = t174 * t89;
    real_type t206 = 1.0 / t90 / t204;
    real_type t211 = t141 * t141;
    real_type t212 = 1.0 / t211;
    real_type t218 = t80 * t80;
    real_type t219 = t218 * t172;
    real_type t220 = 1.0 / t204;
    real_type t239 = t102 * t172;
    real_type t248 = t160 * t160;
    real_type t249 = 1.0 / t248;
    real_type t250 = t249 * t182;
    real_type t251 = t162 * t162;
    real_type t256 = t151 * t239;
    real_type t257 = t81 * t80;
    real_type t263 = t162 * t161 * t182;
    return -2 * t40 * t74 * t72 * t54 * t22 * t109 * t103 * t66 / t14 / t13 * t8 * t7 - t109 * t102 * t119 * t40 * t66 * t19 + 2 * t109 * t144 * t142 * t135 * t86 * t83 * t98 * t129 * t80 * t79 * t125 - 2 * t163 * t135 * t86 * t155 * t129 * t152 * t103 * t125 - 3 * t185 * t183 * t98 * t176 * t81 * t173 + t185 * t182 * t83 * t98 * t129 * t173 + 2 * t185 * t135 * t84 * t86 * t83 * t98 * t128 * t81 * t173 + 2 * t109 * t144 * t212 * t183 * t139 * t98 * t206 * t203 * t173 - t109 * t102 * t212 * t183 * t139 * t220 * t136 * t219 - 2 * t163 * t104 * t151 * t144 * t142 * t182 * t181 * t98 * t220 * t136 * t219 + 2 * t251 * t250 * t87 * t180 * t158 * t220 * t136 * t218 / t150 / t108 * t239 + 3 * t263 * t87 * t180 * t104 * t176 * t257 * t256 - t263 * t83 * t104 * t128 * t81 * t152 * t239 - 2 * t163 * t135 * t84 * t86 * t155 * t128 * t257 * t256 - 2 * t162 * t250 * t87 * t180 * t158 * t104 * t206 * t203 * t80 * t256 + t249 * t182 * t87 * t180 * t158 * t220 * t136 * t218 * t109 * t239;
  }

  real_type
  General::Fyf_D_3_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t5   = Fzf__XO - t2;
    real_type t7   = t2 * ModelPars[33] + t5 * ModelPars[35];
    real_type t8   = phi__f__XO * phi__f__XO;
    real_type t11  = ModelPars[41] * t8 + 1;
    real_type t13  = 1.0 / t11 * t7;
    real_type t14  = t7 * t7;
    real_type t15  = t11 * t11;
    real_type t18  = ModelPars[47];
    real_type t19  = t18 * t18;
    real_type t22  = ModelPars[39];
    real_type t23  = t22 * t22;
    real_type t25  = Fzf__XO * Fzf__XO;
    real_type t30  = ModelPars[45] * t8 + 1;
    real_type t31  = t30 * t30;
    real_type t37  = sqrt(1.0 / t31 * t25 * t23);
    real_type t42  = 1.0 / t7;
    real_type t50  = ModelPars[43] * phi__f__XO;
    real_type t52  = 1.0 / t22;
    real_type t57  = (-t11 * t42 * phi__f__XO * ModelPars[37] * Fzf__XO + alpha__f__XO) / t37 / t30 * Fzf__XO * t22 - t11 * t42 * t30 * t52 * t37 * t50;
    real_type t58  = t57 * t57;
    real_type t71  = atan(t57 * t30 / Fzf__XO * t52 / t18 * t13);
    real_type t72  = t71 * t18;
    real_type t73  = cos(t72);
    real_type t75  = ModelPars[106];
    real_type t78  = ModelPars[98];
    real_type t80  = ModelPars[100] * ModelPars[100];
    real_type t81  = tan(alpha__f__XO);
    real_type t83  = t81 - ModelPars[102];
    real_type t84  = t83 * t83;
    real_type t86  = t84 * t80 + 1;
    real_type t87  = sqrt(t86);
    real_type t89  = 1.0 / t87 * t78;
    real_type t90  = t78 * t78;
    real_type t92  = 1.0 / t86 * t90;
    real_type t93  = ModelPars[110];
    real_type t97  = 1.0 / t2 * t5 * ModelPars[112];
    real_type t98  = lambda__f__XO + t93 + t97;
    real_type t99  = t98 * t98;
    real_type t101 = t99 * t92 + 1;
    real_type t102 = 1.0 / t101;
    real_type t104 = atan(t98 * t89);
    real_type t105 = t104 * t75;
    real_type t106 = sin(t105);
    real_type t108 = t93 + t97;
    real_type t110 = atan(t108 * t89);
    real_type t111 = t110 * t75;
    real_type t112 = cos(t111);
    real_type t113 = 1.0 / t112;
    real_type t114 = t113 * t106 * t102;
    real_type t117 = sin(t72);
    real_type t122 = t30 * t52 * t37 * t50 + t117 * t37;
    real_type t123 = t75 * t122;
    real_type t129 = t81 * t81;
    real_type t130 = 1 + t129;
    real_type t131 = t130 * t83;
    real_type t135 = t86 * t86;
    real_type t142 = t130 * t83 * t80;
    real_type t143 = t101 * t101;
    real_type t144 = 1.0 / t143;
    real_type t150 = t75 * t75;
    real_type t151 = t150 * t122;
    real_type t152 = 1.0 / t135;
    real_type t156 = cos(t105);
    real_type t165 = t112 * t112;
    real_type t169 = t108 * t108;
    real_type t173 = sin(t111);
    return -t114 * t89 * t75 * t73 / (t58 * t31 / t25 / t23 / t19 / t15 * t14 + 1) * t13 + t114 * t131 * t80 / t87 / t86 * t78 * t123 - 2 * t113 * t106 * t144 * t142 * t99 / t87 / t135 * t90 * t78 * t123 + t113 * t156 * t144 * t142 * t98 * t152 * t90 * t151 + t173 / (t169 * t92 + 1) * t131 * t80 * t108 / t165 * t106 * t102 * t152 * t90 * t151;
  }

  real_type
  General::Fyf_D_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[39];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t8   = ModelPars[45] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t12  = sqrt(1.0 / t9 * t3 * t2);
    real_type t13  = ModelPars[47];
    real_type t15  = ModelPars[6];
    real_type t18  = Fzf__XO - t15;
    real_type t20  = t15 * ModelPars[33] + t18 * ModelPars[35];
    real_type t23  = ModelPars[41] * t5 + 1;
    real_type t28  = 1.0 / t1;
    real_type t37  = 1.0 / t20;
    real_type t45  = ModelPars[43] * phi__f__XO;
    real_type t55  = atan(((-t23 * t37 * phi__f__XO * ModelPars[37] * Fzf__XO + alpha__f__XO) / t12 / t8 * Fzf__XO * t1 - t23 * t37 * t8 * t28 * t12 * t45) * t8 / Fzf__XO * t28 / t13 / t23 * t20);
    real_type t57  = sin(t55 * t13);
    real_type t63  = ModelPars[106];
    real_type t65  = ModelPars[98];
    real_type t68  = ModelPars[100] * ModelPars[100];
    real_type t69  = tan(alpha__f__XO);
    real_type t72  = pow(t69 - ModelPars[102], 2);
    real_type t74  = t72 * t68 + 1;
    real_type t75  = sqrt(t74);
    real_type t76  = 1.0 / t75;
    real_type t77  = t65 * t65;
    real_type t80  = ModelPars[110];
    real_type t84  = 1.0 / t15 * t18 * ModelPars[112];
    real_type t85  = lambda__f__XO + t80 + t84;
    real_type t86  = t85 * t85;
    real_type t91  = t76 * t65;
    real_type t93  = atan(t85 * t91);
    real_type t95  = sin(t93 * t63);
    real_type t98  = atan((t80 + t84) * t91);
    real_type t100 = cos(t98 * t63);
    return -1.0 / t100 * t95 / (t86 / t74 * t77 + 1) * t76 * t65 * t63 * (t8 * t28 * t12 * t45 + t57 * t12);
  }

  real_type
  General::Fyf_D_4_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[39];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t8   = ModelPars[45] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t12  = sqrt(1.0 / t9 * t3 * t2);
    real_type t13  = ModelPars[47];
    real_type t15  = ModelPars[6];
    real_type t18  = Fzf__XO - t15;
    real_type t20  = t15 * ModelPars[33] + t18 * ModelPars[35];
    real_type t23  = ModelPars[41] * t5 + 1;
    real_type t28  = 1.0 / t1;
    real_type t37  = 1.0 / t20;
    real_type t45  = ModelPars[43] * phi__f__XO;
    real_type t55  = atan(((-t23 * t37 * phi__f__XO * ModelPars[37] * Fzf__XO + alpha__f__XO) / t12 / t8 * Fzf__XO * t1 - t23 * t37 * t8 * t28 * t12 * t45) * t8 / Fzf__XO * t28 / t13 / t23 * t20);
    real_type t57  = sin(t55 * t13);
    real_type t62  = t8 * t28 * t12 * t45 + t57 * t12;
    real_type t63  = ModelPars[106];
    real_type t65  = ModelPars[98];
    real_type t66  = t65 * t65;
    real_type t69  = ModelPars[100] * ModelPars[100];
    real_type t70  = tan(alpha__f__XO);
    real_type t73  = pow(t70 - ModelPars[102], 2);
    real_type t75  = t73 * t69 + 1;
    real_type t76  = sqrt(t75);
    real_type t81  = 1.0 / t75;
    real_type t83  = ModelPars[110];
    real_type t87  = 1.0 / t15 * t18 * ModelPars[112];
    real_type t88  = lambda__f__XO + t83 + t87;
    real_type t89  = t88 * t88;
    real_type t92  = pow(t89 * t81 * t66 + 1, 2);
    real_type t93  = 1.0 / t92;
    real_type t95  = 1.0 / t76 * t65;
    real_type t97  = atan(t88 * t95);
    real_type t98  = t97 * t63;
    real_type t99  = sin(t98);
    real_type t103 = atan((t83 + t87) * t95);
    real_type t105 = cos(t103 * t63);
    real_type t106 = 1.0 / t105;
    real_type t111 = t63 * t63;
    real_type t115 = cos(t98);
    return 2 * t88 * t106 * t99 * t93 / t76 / t75 * t66 * t65 * t63 * t62 - t106 * t115 * t93 * t81 * t66 * t111 * t62;
  }

  real_type
  General::Fyr( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[40];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t8   = ModelPars[46] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t12  = sqrt(1.0 / t9 * t3 * t2);
    real_type t13  = ModelPars[47];
    real_type t15  = ModelPars[7];
    real_type t18  = Fzr__XO - t15;
    real_type t20  = t15 * ModelPars[34] + t18 * ModelPars[36];
    real_type t23  = ModelPars[42] * t5 + 1;
    real_type t28  = 1.0 / t1;
    real_type t37  = 1.0 / t20;
    real_type t45  = ModelPars[44] * phi__XO;
    real_type t55  = atan(((-t23 * t37 * phi__XO * ModelPars[38] * Fzr__XO + alpha__r__XO) / t12 / t8 * Fzr__XO * t1 - t23 * t37 * t8 * t28 * t12 * t45) * t8 / Fzr__XO * t28 / t13 / t23 * t20);
    real_type t57  = sin(t55 * t13);
    real_type t63  = ModelPars[107];
    real_type t66  = ModelPars[101] * ModelPars[101];
    real_type t67  = tan(alpha__r__XO);
    real_type t70  = pow(t67 - ModelPars[103], 2);
    real_type t73  = sqrt(t70 * t66 + 1);
    real_type t75  = 1.0 / t73 * ModelPars[99];
    real_type t76  = ModelPars[111];
    real_type t80  = 1.0 / t15 * t18 * ModelPars[113];
    real_type t83  = atan((lambda__r__XO + t76 + t80) * t75);
    real_type t85  = cos(t83 * t63);
    real_type t89  = atan((t76 + t80) * t75);
    real_type t91  = cos(t89 * t63);
    return 1.0 / t91 * t85 * (t8 * t28 * t12 * t45 + t57 * t12);
  }

  real_type
  General::Fyr_D_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[40];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t8   = ModelPars[46] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[47];
    real_type t16  = ModelPars[7];
    real_type t18  = ModelPars[36];
    real_type t19  = Fzr__XO - t16;
    real_type t21  = t16 * ModelPars[34] + t19 * t18;
    real_type t24  = ModelPars[42] * t5 + 1;
    real_type t25  = 1.0 / t24;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t25 * t21;
    real_type t29  = 1.0 / t1;
    real_type t31  = 1.0 / Fzr__XO * t29;
    real_type t32  = Fzr__XO * t1;
    real_type t33  = 1.0 / t8;
    real_type t34  = t13 * t33;
    real_type t35  = ModelPars[38];
    real_type t36  = Fzr__XO * t35;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__XO * t36 + alpha__r__XO;
    real_type t45  = ModelPars[44] * phi__XO;
    real_type t48  = t24 * t37;
    real_type t51  = -t48 * t8 * t29 * t12 * t45 + t41 * t34 * t32;
    real_type t52  = t51 * t8;
    real_type t53  = t52 * t31;
    real_type t55  = atan(t53 * t28);
    real_type t56  = t55 * t14;
    real_type t57  = sin(t56);
    real_type t66  = 1.0 / t3;
    real_type t70  = t33 * t1;
    real_type t85  = t21 * t21;
    real_type t86  = 1.0 / t85;
    real_type t99  = t29 * t12;
    real_type t110 = t24 * t24;
    real_type t113 = t14 * t14;
    real_type t118 = t51 * t51;
    real_type t125 = cos(t56);
    real_type t132 = ModelPars[107];
    real_type t133 = ModelPars[99];
    real_type t135 = ModelPars[101] * ModelPars[101];
    real_type t136 = tan(alpha__r__XO);
    real_type t139 = pow(t136 - ModelPars[103], 2);
    real_type t141 = t139 * t135 + 1;
    real_type t142 = sqrt(t141);
    real_type t143 = 1.0 / t142;
    real_type t144 = t143 * t133;
    real_type t145 = ModelPars[111];
    real_type t146 = ModelPars[113];
    real_type t148 = 1.0 / t16;
    real_type t149 = t148 * t19 * t146;
    real_type t150 = lambda__r__XO + t145 + t149;
    real_type t152 = atan(t150 * t144);
    real_type t153 = t152 * t132;
    real_type t154 = cos(t153);
    real_type t156 = t145 + t149;
    real_type t158 = atan(t156 * t144);
    real_type t159 = t158 * t132;
    real_type t160 = cos(t159);
    real_type t161 = 1.0 / t160;
    real_type t166 = t8 * t99 * t45 + t57 * t12;
    real_type t170 = t133 * t133;
    real_type t172 = 1.0 / t141 * t170;
    real_type t173 = t150 * t150;
    real_type t177 = sin(t153);
    real_type t183 = t160 * t160;
    real_type t189 = t156 * t156;
    real_type t194 = sin(t159);
    return t161 * t154 * (t10 * Fzr__XO * t2 * t57 * t13 + t125 / (t118 * t9 * t66 / t2 / t113 / t110 * t85 + 1) * (t53 * t27 * t25 * t18 - t52 * t66 * t29 * t28 + (t41 * t13 * t70 - t41 / t12 / t11 / t9 / t8 * t3 * t2 * t1 + (t18 * t24 * t86 * phi__XO * t36 - t48 * phi__XO * t35) * t34 * t32 - Fzr__XO * t24 * t37 * t33 * t1 * t13 * t45 + t18 * t24 * t86 * t8 * t99 * t45) * t8 * t31 * t28) * t14 * t12 + Fzr__XO * t70 * t13 * t45) - t161 * t177 / (t173 * t172 + 1) * t148 * t146 * t144 * t132 * t166 + t194 / (t189 * t172 + 1) * t148 * t146 * t143 * t133 * t132 / t183 * t154 * t166;
  }

  real_type
  General::Fyr_D_1_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[40];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t8   = ModelPars[46] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t14  = 1.0 / t12 / t11;
    real_type t15  = ModelPars[47];
    real_type t17  = ModelPars[7];
    real_type t19  = ModelPars[36];
    real_type t20  = Fzr__XO - t17;
    real_type t22  = t17 * ModelPars[34] + t20 * t19;
    real_type t25  = ModelPars[42] * t5 + 1;
    real_type t26  = 1.0 / t25;
    real_type t28  = 1.0 / t15;
    real_type t29  = t28 * t26 * t22;
    real_type t30  = 1.0 / t1;
    real_type t32  = 1.0 / Fzr__XO * t30;
    real_type t33  = Fzr__XO * t1;
    real_type t34  = 1.0 / t8;
    real_type t35  = 1.0 / t12;
    real_type t36  = t35 * t34;
    real_type t37  = ModelPars[38];
    real_type t38  = Fzr__XO * t37;
    real_type t39  = 1.0 / t22;
    real_type t43  = -t25 * t39 * phi__XO * t38 + alpha__r__XO;
    real_type t47  = ModelPars[44] * phi__XO;
    real_type t50  = t25 * t39;
    real_type t53  = -t50 * t8 * t30 * t12 * t47 + t43 * t36 * t33;
    real_type t54  = t53 * t8;
    real_type t55  = t54 * t32;
    real_type t57  = atan(t55 * t29);
    real_type t58  = t57 * t15;
    real_type t59  = sin(t58);
    real_type t61  = t2 * t2;
    real_type t63  = t9 * t9;
    real_type t64  = 1.0 / t63;
    real_type t69  = t28 * t26 * t19;
    real_type t71  = 1.0 / t3;
    real_type t72  = t71 * t30;
    real_type t73  = t54 * t72;
    real_type t75  = t34 * t1;
    real_type t78  = t2 * t1;
    real_type t79  = t3 * t78;
    real_type t81  = 1.0 / t9 / t8;
    real_type t82  = t14 * t81;
    real_type t85  = phi__XO * t37;
    real_type t87  = phi__XO * t38;
    real_type t88  = t22 * t22;
    real_type t89  = 1.0 / t88;
    real_type t91  = t19 * t25 * t89;
    real_type t93  = -t50 * t85 + t91 * t87;
    real_type t96  = t1 * t35;
    real_type t97  = t96 * t47;
    real_type t99  = Fzr__XO * t25;
    real_type t102 = t30 * t12;
    real_type t103 = t102 * t47;
    real_type t108 = t19 * t25 * t89 * t8 * t103 - t99 * t39 * t34 * t97 + t93 * t36 * t33 + t43 * t35 * t75 - t43 * t82 * t79;
    real_type t109 = t108 * t8;
    real_type t110 = t109 * t32;
    real_type t112 = t110 * t29 - t73 * t29 + t55 * t69;
    real_type t113 = t25 * t25;
    real_type t114 = 1.0 / t113;
    real_type t115 = t114 * t88;
    real_type t116 = t15 * t15;
    real_type t117 = 1.0 / t116;
    real_type t118 = t117 * t115;
    real_type t119 = 1.0 / t2;
    real_type t121 = t53 * t53;
    real_type t122 = t121 * t9;
    real_type t125 = t122 * t71 * t119 * t118 + 1;
    real_type t126 = 1.0 / t125;
    real_type t127 = t126 * t112;
    real_type t129 = cos(t58);
    real_type t135 = t59 * t35;
    real_type t138 = t15 * t12;
    real_type t143 = t3 * Fzr__XO;
    real_type t144 = 1.0 / t143;
    real_type t152 = t81 * t78;
    real_type t164 = t3 * t3;
    real_type t178 = 1.0 / t88 / t22;
    real_type t180 = t19 * t19;
    real_type t198 = t35 * t47;
    real_type t215 = t125 * t125;
    real_type t216 = 1.0 / t215;
    real_type t219 = t119 * t117;
    real_type t221 = t9 * t71;
    real_type t237 = t112 * t112;
    real_type t247 = ModelPars[107];
    real_type t248 = ModelPars[99];
    real_type t250 = ModelPars[101] * ModelPars[101];
    real_type t251 = tan(alpha__r__XO);
    real_type t254 = pow(t251 - ModelPars[103], 2);
    real_type t256 = t254 * t250 + 1;
    real_type t257 = sqrt(t256);
    real_type t258 = 1.0 / t257;
    real_type t259 = t258 * t248;
    real_type t260 = ModelPars[111];
    real_type t261 = ModelPars[113];
    real_type t263 = 1.0 / t17;
    real_type t264 = t263 * t20 * t261;
    real_type t265 = lambda__r__XO + t260 + t264;
    real_type t267 = atan(t265 * t259);
    real_type t268 = t267 * t247;
    real_type t269 = cos(t268);
    real_type t271 = t260 + t264;
    real_type t273 = atan(t271 * t259);
    real_type t274 = t273 * t247;
    real_type t275 = cos(t274);
    real_type t276 = 1.0 / t275;
    real_type t285 = t10 * Fzr__XO * t2 * t135 + Fzr__XO * t75 * t198 + t129 * t127 * t138;
    real_type t289 = t248 * t248;
    real_type t290 = 1.0 / t256;
    real_type t291 = t290 * t289;
    real_type t292 = t265 * t265;
    real_type t294 = t292 * t291 + 1;
    real_type t295 = 1.0 / t294;
    real_type t296 = sin(t268);
    real_type t303 = t275 * t275;
    real_type t304 = 1.0 / t303;
    real_type t305 = t247 * t304;
    real_type t309 = t271 * t271;
    real_type t311 = t309 * t291 + 1;
    real_type t312 = 1.0 / t311;
    real_type t314 = sin(t274);
    real_type t322 = t8 * t102 * t47 + t59 * t12;
    real_type t324 = t289 * t248;
    real_type t326 = 1.0 / t257 / t256;
    real_type t328 = t261 * t261;
    real_type t331 = t17 * t17;
    real_type t332 = 1.0 / t331;
    real_type t333 = t294 * t294;
    real_type t334 = 1.0 / t333;
    real_type t341 = t247 * t247;
    real_type t342 = t341 * t322;
    real_type t344 = t332 * t328;
    real_type t358 = t269 * t322;
    real_type t365 = t311 * t311;
    real_type t366 = 1.0 / t365;
    real_type t368 = t314 * t314;
    return t276 * t269 * (-t64 * t3 * t61 * t59 * t14 + 2 * t10 * Fzr__XO * t2 * t129 * t127 * t15 * t35 + t10 * t2 * t135 + t129 * t126 * (-2 * t73 * t69 + 2 * t110 * t69 + 2 * t54 * t144 * t30 * t29 - 2 * t109 * t72 * t29 + (-3 * Fzr__XO * t43 * t14 * t152 + 2 * t93 * t35 * t75 + 3 * t43 / t12 / t64 / t164 / t63 / t8 * t143 * t1 - 2 * t93 * t82 * t79 + (-2 * t180 * t25 * t178 * t87 + 2 * t85 * t91) * t36 * t33 + t3 * t25 * t39 * t81 * t78 * t14 * t47 + 2 * t19 * t99 * t89 * t34 * t97 - t50 * t75 * t198 - 2 * t180 * t25 * t178 * t8 * t103) * t8 * t32 * t29) * t138 - (2 * t19 * t121 * t221 * t219 * t114 * t22 + 2 * t108 * t53 * t221 * t219 * t115 - 2 * t122 * t144 * t119 * t118) * t129 * t216 * t112 * t138 - t59 * t216 * t237 * t116 * t12 - t3 * t152 * t14 * t47 + t34 * t96 * t47) - 2 * t276 * t296 * t295 * t263 * t261 * t259 * t247 * t285 + 2 * t314 * t312 * t263 * t261 * t258 * t248 * t305 * t269 * t285 + 2 * t265 * t276 * t296 * t334 * t332 * t328 * t326 * t324 * t247 * t322 - t276 * t269 * t334 * t344 * t291 * t342 - 2 * t314 * t312 * t304 * t296 * t295 * t332 * t328 * t291 * t342 + 2 * t368 * t366 * t332 * t328 * t290 * t289 * t341 / t303 / t275 * t358 - 2 * t271 * t314 * t366 * t332 * t328 * t326 * t324 * t305 * t358 + t366 * t344 * t291 * t341 * t276 * t358;
  }

  real_type
  General::Fyr_D_1_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[40];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t4   = t3 * t2;
    real_type t5   = phi__XO * phi__XO;
    real_type t6   = ModelPars[46];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t4;
    real_type t12  = sqrt(t11);
    real_type t14  = 1.0 / t12 / t11;
    real_type t15  = ModelPars[47];
    real_type t17  = ModelPars[7];
    real_type t19  = ModelPars[36];
    real_type t20  = Fzr__XO - t17;
    real_type t22  = t17 * ModelPars[34] + t20 * t19;
    real_type t23  = ModelPars[42];
    real_type t25  = t23 * t5 + 1;
    real_type t26  = 1.0 / t25;
    real_type t27  = t26 * t22;
    real_type t28  = 1.0 / t15;
    real_type t29  = t28 * t27;
    real_type t30  = 1.0 / t1;
    real_type t31  = 1.0 / Fzr__XO;
    real_type t32  = t31 * t30;
    real_type t33  = Fzr__XO * t1;
    real_type t34  = 1.0 / t8;
    real_type t35  = 1.0 / t12;
    real_type t36  = t35 * t34;
    real_type t37  = ModelPars[38];
    real_type t38  = Fzr__XO * t37;
    real_type t39  = 1.0 / t22;
    real_type t43  = -t25 * t39 * phi__XO * t38 + alpha__r__XO;
    real_type t46  = ModelPars[44];
    real_type t47  = phi__XO * t46;
    real_type t49  = t8 * t30;
    real_type t50  = t25 * t39;
    real_type t53  = -t50 * t49 * t12 * t47 + t43 * t36 * t33;
    real_type t54  = t53 * t8;
    real_type t55  = t54 * t32;
    real_type t57  = atan(t55 * t29);
    real_type t58  = t57 * t15;
    real_type t59  = sin(t58);
    real_type t61  = t2 * t2;
    real_type t63  = t3 * Fzr__XO;
    real_type t64  = t9 * t9;
    real_type t68  = phi__XO * t6;
    real_type t72  = t15 * t35;
    real_type t73  = t25 * t25;
    real_type t74  = 1.0 / t73;
    real_type t76  = t30 * t28;
    real_type t77  = t76 * t74 * t22;
    real_type t78  = t8 * t31;
    real_type t80  = phi__XO * t23 * t53;
    real_type t81  = t80 * t78;
    real_type t84  = t76 * t27;
    real_type t85  = t6 * t31;
    real_type t86  = t53 * phi__XO;
    real_type t87  = t86 * t85;
    real_type t90  = t10 * t33;
    real_type t91  = t43 * t35;
    real_type t95  = t2 * t1;
    real_type t97  = 1.0 / t64;
    real_type t98  = t97 * t63 * t95;
    real_type t108 = -2 * t23 * t39 * t5 * t38 - t50 * t38;
    real_type t111 = t12 * t46;
    real_type t112 = t30 * t111;
    real_type t116 = t5 * t46;
    real_type t117 = t1 * t35;
    real_type t118 = t117 * t116;
    real_type t119 = t39 * t10;
    real_type t125 = t12 * t116;
    real_type t130 = t23 * t39;
    real_type t134 = 2 * t6 * t3 * t25 * t119 * t118 - t25 * t39 * t8 * t112 - 2 * t50 * t6 * t30 * t125 + 2 * t68 * t43 * t14 * t98 + t108 * t36 * t33 - 2 * t130 * t49 * t125 - 2 * t68 * t91 * t90;
    real_type t135 = t134 * t8;
    real_type t136 = t135 * t32;
    real_type t138 = t136 * t29 - 2 * t81 * t77 + 2 * t87 * t84;
    real_type t139 = t22 * t22;
    real_type t140 = t74 * t139;
    real_type t141 = t15 * t15;
    real_type t142 = 1.0 / t141;
    real_type t144 = 1.0 / t2;
    real_type t145 = 1.0 / t3;
    real_type t147 = t53 * t53;
    real_type t151 = t147 * t9 * t145 * t144 * t142 * t140 + 1;
    real_type t152 = 1.0 / t151;
    real_type t153 = t152 * t138;
    real_type t155 = cos(t58);
    real_type t161 = t2 * t59 * t35;
    real_type t163 = 1.0 / t9 / t8;
    real_type t168 = t26 * t19;
    real_type t169 = t28 * t168;
    real_type t171 = t145 * t30;
    real_type t174 = t34 * t1;
    real_type t176 = t3 * t95;
    real_type t177 = t14 * t163;
    real_type t183 = 1.0 / t139;
    real_type t184 = t25 * t183;
    real_type t185 = t19 * t184;
    real_type t187 = t185 * phi__XO * t38 - t50 * phi__XO * t37;
    real_type t191 = t39 * t34;
    real_type t192 = Fzr__XO * t25;
    real_type t193 = t192 * t191;
    real_type t195 = t30 * t12;
    real_type t197 = t183 * t8;
    real_type t198 = t19 * t25;
    real_type t199 = t198 * t197;
    real_type t201 = -t193 * t117 * t47 - t43 * t177 * t176 + t187 * t36 * t33 + t199 * t195 * t47 + t91 * t174;
    real_type t205 = t201 * t8 * t32 * t29 - t54 * t171 * t29 + t55 * t169;
    real_type t214 = t15 * t12;
    real_type t223 = t8 * t145;
    real_type t242 = t10 * t1;
    real_type t243 = t35 * t242;
    real_type t248 = t97 * t95;
    real_type t257 = t3 * t3;
    real_type t294 = t35 * t46;
    real_type t320 = t195 * t116;
    real_type t329 = -2 * phi__XO * t6 * t43 * t243 + 8 * t68 * t3 * t43 * t14 * t248 + t108 * t35 * t174 - 6 * t68 * t43 / t12 / t97 / t64 / t9 * t1 - t108 * t177 * t176 - 2 * t68 * t187 * t35 * t90 + 2 * t68 * t187 * t14 * t98 + (2 * t19 * t23 * t183 * t5 * t38 - 2 * t130 * t5 * t37 - t25 * t39 * t37 + t185 * t38) * t36 * t33 - t193 * t1 * t294 - 2 * t6 * t63 * t25 * t39 * t97 * t95 * t14 * t116 + 2 * t6 * t192 * t119 * t118 - 2 * Fzr__XO * t23 * t191 * t118 + t199 * t112 - 2 * t6 * t3 * t19 * t184 * t243 * t116 + 2 * t198 * t183 * t6 * t320 + 2 * t19 * t23 * t197 * t320;
    real_type t338 = t151 * t151;
    real_type t339 = 1.0 / t338;
    real_type t344 = t144 * t142;
    real_type t346 = t9 * t145;
    real_type t352 = t344 * t140;
    real_type t377 = t35 * t116;
    real_type t383 = ModelPars[107];
    real_type t384 = ModelPars[99];
    real_type t386 = ModelPars[101] * ModelPars[101];
    real_type t387 = tan(alpha__r__XO);
    real_type t390 = pow(t387 - ModelPars[103], 2);
    real_type t392 = t390 * t386 + 1;
    real_type t393 = sqrt(t392);
    real_type t394 = 1.0 / t393;
    real_type t395 = t394 * t384;
    real_type t396 = ModelPars[111];
    real_type t397 = ModelPars[113];
    real_type t399 = 1.0 / t17;
    real_type t400 = t399 * t20 * t397;
    real_type t401 = lambda__r__XO + t396 + t400;
    real_type t403 = atan(t401 * t395);
    real_type t404 = t403 * t383;
    real_type t405 = cos(t404);
    real_type t407 = t396 + t400;
    real_type t409 = atan(t407 * t395);
    real_type t410 = t409 * t383;
    real_type t411 = cos(t410);
    real_type t412 = 1.0 / t411;
    real_type t428 = -2 * t68 * t163 * t3 * t161 - 2 * t6 * t3 * t242 * t377 + 2 * t6 * t195 * t116 + t155 * t153 * t214 + t49 * t111;
    real_type t432 = t384 * t384;
    real_type t434 = 1.0 / t392 * t432;
    real_type t435 = t401 * t401;
    real_type t439 = sin(t404);
    real_type t445 = t411 * t411;
    real_type t451 = t407 * t407;
    real_type t456 = sin(t410);
    return t412 * t405 * (2 * t68 / t64 / t8 * t63 * t61 * t59 * t14 + t10 * Fzr__XO * t2 * t155 * t153 * t72 - 4 * t68 * t163 * Fzr__XO * t161 - 2 * phi__XO * t6 * t163 * t4 * t155 * t152 * t205 * t72 + t155 * t152 * (-2 * phi__XO * t23 * t201 * t78 * t77 + 2 * t201 * phi__XO * t85 * t84 - 2 * t86 * t6 * t145 * t84 - 2 * t81 * t76 * t74 * t19 + t329 * t8 * t32 * t29 - t135 * t171 * t29 + 2 * t87 * t76 * t168 + 2 * t80 * t223 * t77 + t136 * t169) * t214 - (-4 * phi__XO * t23 * t147 * t346 * t344 / t73 / t25 * t139 + 4 * phi__XO * t6 * t147 * t223 * t352 + 2 * t134 * t53 * t346 * t352) * t155 * t339 * t205 * t214 - t59 * t138 * t339 * t205 * t141 * t12 + Fzr__XO * t174 * t294 + 2 * t6 * t63 * t248 * t14 * t116 - 2 * t6 * Fzr__XO * t242 * t377) - t412 * t439 / (t435 * t434 + 1) * t399 * t397 * t395 * t383 * t428 + t456 / (t451 * t434 + 1) * t399 * t397 * t394 * t384 * t383 / t445 * t405 * t428;
  }

  real_type
  General::Fyr_D_1_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = 1.0 / Fzr__XO;
    real_type t3   = ModelPars[7];
    real_type t5   = ModelPars[36];
    real_type t6   = Fzr__XO - t3;
    real_type t8   = t3 * ModelPars[34] + t6 * t5;
    real_type t10  = phi__XO * phi__XO;
    real_type t13  = ModelPars[42] * t10 + 1;
    real_type t14  = 1.0 / t13;
    real_type t15  = t8 * t8;
    real_type t16  = t13 * t13;
    real_type t17  = 1.0 / t16;
    real_type t19  = ModelPars[47];
    real_type t20  = t19 * t19;
    real_type t23  = ModelPars[40];
    real_type t24  = t23 * t23;
    real_type t26  = Fzr__XO * Fzr__XO;
    real_type t27  = 1.0 / t26;
    real_type t31  = ModelPars[46] * t10 + 1;
    real_type t32  = t31 * t31;
    real_type t33  = Fzr__XO * t23;
    real_type t34  = 1.0 / t31;
    real_type t36  = 1.0 / t32;
    real_type t37  = t36 * t26 * t24;
    real_type t38  = sqrt(t37);
    real_type t39  = 1.0 / t38;
    real_type t40  = t39 * t34;
    real_type t41  = ModelPars[38];
    real_type t42  = Fzr__XO * t41;
    real_type t43  = 1.0 / t8;
    real_type t47  = -t13 * t43 * phi__XO * t42 + alpha__r__XO;
    real_type t51  = ModelPars[44] * phi__XO;
    real_type t53  = 1.0 / t23;
    real_type t55  = t13 * t43;
    real_type t58  = -t55 * t31 * t53 * t38 * t51 + t47 * t40 * t33;
    real_type t59  = t58 * t58;
    real_type t63  = t59 * t32 * t27 / t24 / t20 * t17 * t15 + 1;
    real_type t64  = 1.0 / t63;
    real_type t66  = t14 * t8;
    real_type t67  = 1.0 / t19;
    real_type t68  = t67 * t66;
    real_type t69  = t1 * t53;
    real_type t70  = t58 * t31;
    real_type t71  = t70 * t69;
    real_type t73  = atan(t71 * t68);
    real_type t74  = t73 * t19;
    real_type t75  = cos(t74);
    real_type t78  = t19 * t38;
    real_type t79  = t14 * t5;
    real_type t85  = t34 * t23;
    real_type t88  = t26 * t24 * t23;
    real_type t93  = 1.0 / t38 / t37 / t32 / t31;
    real_type t115 = 1.0 / t15;
    real_type t128 = t53 * t38;
    real_type t138 = t71 * t67 * t79 - t70 * t27 * t53 * t68 + (t47 * t39 * t85 - t47 * t93 * t88 + (t5 * t13 * t115 * phi__XO * t42 - t55 * phi__XO * t41) * t40 * t33 - Fzr__XO * t13 * t43 * t34 * t23 * t39 * t51 + t5 * t13 * t115 * t31 * t128 * t51) * t31 * t69 * t68;
    real_type t140 = t63 * t63;
    real_type t141 = 1.0 / t140;
    real_type t153 = sin(t74);
    real_type t157 = ModelPars[107];
    real_type t158 = ModelPars[99];
    real_type t160 = ModelPars[101] * ModelPars[101];
    real_type t161 = tan(alpha__r__XO);
    real_type t163 = t161 - ModelPars[103];
    real_type t164 = t163 * t163;
    real_type t166 = t164 * t160 + 1;
    real_type t167 = sqrt(t166);
    real_type t168 = 1.0 / t167;
    real_type t169 = t168 * t158;
    real_type t170 = ModelPars[111];
    real_type t171 = ModelPars[113];
    real_type t173 = 1.0 / t3;
    real_type t174 = t173 * t6 * t171;
    real_type t175 = lambda__r__XO + t170 + t174;
    real_type t177 = atan(t175 * t169);
    real_type t178 = t177 * t157;
    real_type t179 = cos(t178);
    real_type t181 = t170 + t174;
    real_type t183 = atan(t181 * t169);
    real_type t184 = t183 * t157;
    real_type t185 = cos(t184);
    real_type t186 = 1.0 / t185;
    real_type t198 = t36 * Fzr__XO * t24 * t153 * t39 + Fzr__XO * t85 * t39 * t51 + t75 * t64 * t138 * t78;
    real_type t201 = 1.0 / t167 / t166;
    real_type t205 = t163 * t160;
    real_type t206 = t161 * t161;
    real_type t207 = 1 + t206;
    real_type t209 = t158 * t158;
    real_type t211 = 1.0 / t166 * t209;
    real_type t212 = t175 * t175;
    real_type t214 = t212 * t211 + 1;
    real_type t215 = 1.0 / t214;
    real_type t216 = sin(t178);
    real_type t218 = t186 * t216 * t215;
    real_type t219 = t218 * t207 * t205;
    real_type t222 = t185 * t185;
    real_type t223 = 1.0 / t222;
    real_type t225 = t158 * t157;
    real_type t226 = t201 * t225;
    real_type t228 = t160 * t181;
    real_type t230 = t181 * t181;
    real_type t232 = t230 * t211 + 1;
    real_type t233 = 1.0 / t232;
    real_type t235 = sin(t184);
    real_type t239 = t64 * t66;
    real_type t250 = t31 * t128 * t51 + t153 * t38;
    real_type t251 = t157 * t250;
    real_type t257 = t209 * t158;
    real_type t259 = t166 * t166;
    real_type t262 = t171 / t167 / t259;
    real_type t265 = t214 * t214;
    real_type t266 = 1.0 / t265;
    real_type t270 = t207 * t163;
    real_type t275 = t157 * t157;
    real_type t277 = t209 * t275 * t250;
    real_type t278 = 1.0 / t259;
    real_type t279 = t171 * t278;
    real_type t291 = t223 * t216;
    real_type t293 = t235 * t233;
    real_type t301 = t173 * t171;
    real_type t302 = t293 * t301;
    real_type t312 = t179 * t250;
    real_type t316 = t209 * t275;
    real_type t319 = t232 * t232;
    real_type t320 = 1.0 / t319;
    real_type t321 = t320 * t173;
    real_type t322 = t235 * t235;
    real_type t324 = t270 * t228;
    real_type t328 = t223 * t312;
    return t186 * t179 * (t75 * t64 * t14 * t8 * t1 + t75 * t64 * (t39 * t67 * t79 - t39 * t1 * t67 * t66 + (t39 * t85 - t93 * t88) * t31 * t69 * t68) * t78 - 2 * t58 * t31 * t1 * t53 * t17 * t15 * t75 * t141 * t138 * t67 - t153 * t66 * t141 * t138 * t19) + t219 * t175 * t201 * t158 * t157 * t198 - t235 * t233 * t207 * t163 * t228 * t226 * t223 * t179 * t198 - t218 * t173 * t171 * t168 * t158 * t157 * t75 * t239 + t219 * t173 * t171 * t201 * t158 * t251 - 2 * t270 * t160 * t212 * t186 * t216 * t266 * t173 * t262 * t257 * t251 + t186 * t179 * t270 * t160 * t175 * t266 * t173 * t279 * t277 + t293 * t270 * t228 * t291 * t215 * t173 * t279 * t277 + t302 * t168 * t225 * t223 * t179 * t75 * t239 + t302 * t291 * t215 * t207 * t205 * t175 * t278 * t277 - 2 * t324 * t322 * t321 * t279 * t316 / t222 / t185 * t312 - t270 * t160 * t235 * t233 * t301 * t226 * t328 + 2 * t270 * t160 * t230 * t235 * t321 * t262 * t257 * t157 * t328 - t324 * t320 * t301 * t278 * t316 * t186 * t312;
  }

  real_type
  General::Fyr_D_1_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[40];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t8   = ModelPars[46] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[47];
    real_type t16  = ModelPars[7];
    real_type t18  = ModelPars[36];
    real_type t19  = Fzr__XO - t16;
    real_type t21  = t16 * ModelPars[34] + t19 * t18;
    real_type t24  = ModelPars[42] * t5 + 1;
    real_type t25  = 1.0 / t24;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t25 * t21;
    real_type t29  = 1.0 / t1;
    real_type t31  = 1.0 / Fzr__XO * t29;
    real_type t32  = Fzr__XO * t1;
    real_type t33  = 1.0 / t8;
    real_type t34  = t13 * t33;
    real_type t35  = ModelPars[38];
    real_type t36  = Fzr__XO * t35;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__XO * t36 + alpha__r__XO;
    real_type t45  = ModelPars[44] * phi__XO;
    real_type t48  = t24 * t37;
    real_type t51  = -t48 * t8 * t29 * t12 * t45 + t41 * t34 * t32;
    real_type t52  = t51 * t8;
    real_type t53  = t52 * t31;
    real_type t55  = atan(t53 * t28);
    real_type t56  = t55 * t14;
    real_type t57  = sin(t56);
    real_type t66  = 1.0 / t3;
    real_type t70  = t33 * t1;
    real_type t85  = t21 * t21;
    real_type t86  = 1.0 / t85;
    real_type t99  = t29 * t12;
    real_type t110 = t24 * t24;
    real_type t113 = t14 * t14;
    real_type t118 = t51 * t51;
    real_type t125 = cos(t56);
    real_type t132 = ModelPars[107];
    real_type t134 = ModelPars[99];
    real_type t137 = ModelPars[101] * ModelPars[101];
    real_type t138 = tan(alpha__r__XO);
    real_type t141 = pow(t138 - ModelPars[103], 2);
    real_type t143 = t141 * t137 + 1;
    real_type t144 = sqrt(t143);
    real_type t145 = 1.0 / t144;
    real_type t146 = t134 * t134;
    real_type t148 = 1.0 / t143 * t146;
    real_type t149 = ModelPars[111];
    real_type t150 = ModelPars[113];
    real_type t152 = 1.0 / t16;
    real_type t153 = t152 * t19 * t150;
    real_type t154 = lambda__r__XO + t149 + t153;
    real_type t155 = t154 * t154;
    real_type t157 = t155 * t148 + 1;
    real_type t158 = 1.0 / t157;
    real_type t160 = t145 * t134;
    real_type t162 = atan(t154 * t160);
    real_type t163 = t162 * t132;
    real_type t164 = sin(t163);
    real_type t165 = t149 + t153;
    real_type t167 = atan(t165 * t160);
    real_type t168 = t167 * t132;
    real_type t169 = cos(t168);
    real_type t170 = 1.0 / t169;
    real_type t171 = t170 * t164;
    real_type t177 = t8 * t99 * t45 + t57 * t12;
    real_type t185 = t157 * t157;
    real_type t186 = 1.0 / t185;
    real_type t192 = t132 * t132;
    real_type t193 = t192 * t177;
    real_type t196 = cos(t163);
    real_type t203 = t169 * t169;
    real_type t207 = t165 * t165;
    real_type t212 = sin(t168);
    return -t171 * t158 * t145 * t134 * t132 * (t10 * Fzr__XO * t2 * t57 * t13 + t125 / (t118 * t9 * t66 / t2 / t113 / t110 * t85 + 1) * (t53 * t27 * t25 * t18 - t52 * t66 * t29 * t28 + (t41 * t13 * t70 - t41 / t12 / t11 / t9 / t8 * t3 * t2 * t1 + (t18 * t24 * t86 * phi__XO * t36 - t48 * phi__XO * t35) * t34 * t32 - Fzr__XO * t24 * t37 * t33 * t1 * t13 * t45 + t18 * t24 * t86 * t8 * t99 * t45) * t8 * t31 * t28) * t14 * t12 + Fzr__XO * t70 * t13 * t45) + 2 * t154 * t171 * t186 * t152 * t150 / t144 / t143 * t146 * t134 * t132 * t177 - t170 * t196 * t186 * t152 * t150 * t148 * t193 - t212 / (t207 * t148 + 1) * t152 * t150 / t203 * t164 * t158 * t148 * t193;
  }

  real_type
  General::Fyr_D_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[40];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t6   = ModelPars[46];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[47];
    real_type t16  = ModelPars[7];
    real_type t19  = Fzr__XO - t16;
    real_type t21  = t16 * ModelPars[34] + t19 * ModelPars[36];
    real_type t22  = ModelPars[42];
    real_type t24  = t22 * t5 + 1;
    real_type t26  = 1.0 / t24 * t21;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t26;
    real_type t29  = 1.0 / t1;
    real_type t30  = 1.0 / Fzr__XO;
    real_type t31  = t30 * t29;
    real_type t32  = Fzr__XO * t1;
    real_type t34  = t13 / t8;
    real_type t36  = ModelPars[38] * Fzr__XO;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__XO * t36 + alpha__r__XO;
    real_type t44  = ModelPars[44];
    real_type t47  = t8 * t29;
    real_type t48  = t24 * t37;
    real_type t51  = -t48 * t47 * t12 * phi__XO * t44 + t41 * t34 * t32;
    real_type t55  = atan(t51 * t8 * t31 * t28);
    real_type t56  = t55 * t14;
    real_type t57  = sin(t56);
    real_type t63  = phi__XO * t6;
    real_type t68  = t24 * t24;
    real_type t69  = 1.0 / t68;
    real_type t71  = t29 * t27;
    real_type t93  = t9 * t9;
    real_type t110 = t12 * t44;
    real_type t115 = t5 * t44;
    real_type t124 = t12 * t115;
    real_type t138 = t21 * t21;
    real_type t140 = t14 * t14;
    real_type t146 = t51 * t51;
    real_type t153 = cos(t56);
    real_type t168 = ModelPars[107];
    real_type t171 = ModelPars[101] * ModelPars[101];
    real_type t172 = tan(alpha__r__XO);
    real_type t175 = pow(t172 - ModelPars[103], 2);
    real_type t178 = sqrt(t175 * t171 + 1);
    real_type t180 = 1.0 / t178 * ModelPars[99];
    real_type t181 = ModelPars[111];
    real_type t185 = 1.0 / t16 * t19 * ModelPars[113];
    real_type t188 = atan((lambda__r__XO + t181 + t185) * t180);
    real_type t190 = cos(t188 * t168);
    real_type t194 = atan((t181 + t185) * t180);
    real_type t196 = cos(t194 * t168);
    return 1.0 / t196 * t190 * (-2 * t63 / t9 / t8 * t3 * t2 * t57 * t13 + t153 / (t146 * t9 / t3 / t2 / t140 * t69 * t138 + 1) * (-2 * phi__XO * t22 * t51 * t8 * t30 * t71 * t69 * t21 + 2 * t51 * phi__XO * t6 * t30 * t71 * t26 + (-2 * t63 * t41 * t13 * t10 * t32 + 2 * t63 * t41 / t12 / t11 / t93 * t3 * Fzr__XO * t2 * t1 + (-2 * t22 * t37 * t5 * t36 - t48 * t36) * t34 * t32 - t24 * t37 * t8 * t29 * t110 + 2 * t6 * t3 * t24 * t37 * t10 * t1 * t13 * t115 - 2 * t48 * t6 * t29 * t124 - 2 * t22 * t37 * t47 * t124) * t8 * t31 * t28) * t14 * t12 + t47 * t110 - 2 * t6 * t3 * t10 * t1 * t13 * t115 + 2 * t6 * t29 * t12 * t115);
  }

  real_type
  General::Fyr_D_2_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[40];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t4   = t3 * t2;
    real_type t5   = phi__XO * phi__XO;
    real_type t6   = ModelPars[46];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t4;
    real_type t12  = sqrt(t11);
    real_type t14  = 1.0 / t12 / t11;
    real_type t15  = ModelPars[47];
    real_type t17  = ModelPars[7];
    real_type t20  = Fzr__XO - t17;
    real_type t22  = t17 * ModelPars[34] + t20 * ModelPars[36];
    real_type t23  = ModelPars[42];
    real_type t25  = t23 * t5 + 1;
    real_type t27  = 1.0 / t25 * t22;
    real_type t28  = 1.0 / t15;
    real_type t29  = t28 * t27;
    real_type t30  = 1.0 / t1;
    real_type t31  = 1.0 / Fzr__XO;
    real_type t32  = t31 * t30;
    real_type t33  = Fzr__XO * t1;
    real_type t35  = 1.0 / t12;
    real_type t36  = t35 / t8;
    real_type t37  = ModelPars[38];
    real_type t38  = Fzr__XO * t37;
    real_type t39  = 1.0 / t22;
    real_type t43  = -t25 * t39 * phi__XO * t38 + alpha__r__XO;
    real_type t46  = ModelPars[44];
    real_type t49  = t8 * t30;
    real_type t50  = t25 * t39;
    real_type t53  = -t50 * t49 * t12 * phi__XO * t46 + t43 * t36 * t33;
    real_type t57  = atan(t53 * t8 * t32 * t29);
    real_type t58  = t57 * t15;
    real_type t59  = sin(t58);
    real_type t61  = t2 * t2;
    real_type t63  = t3 * t3;
    real_type t64  = t9 * t9;
    real_type t68  = t6 * t6;
    real_type t69  = t5 * t68;
    real_type t74  = t25 * t25;
    real_type t75  = 1.0 / t74;
    real_type t77  = t30 * t28;
    real_type t78  = t77 * t75 * t22;
    real_type t79  = t8 * t31;
    real_type t80  = t23 * t53;
    real_type t85  = t77 * t27;
    real_type t86  = t6 * t31;
    real_type t91  = t10 * t33;
    real_type t92  = t43 * t35;
    real_type t93  = phi__XO * t6;
    real_type t97  = t2 * t1;
    real_type t99  = t3 * Fzr__XO * t97;
    real_type t100 = 1.0 / t64;
    real_type t101 = t100 * t99;
    real_type t102 = t43 * t14;
    real_type t111 = -2 * t23 * t39 * t5 * t38 - t50 * t38;
    real_type t114 = t12 * t46;
    real_type t115 = t30 * t114;
    real_type t116 = t39 * t8;
    real_type t119 = t5 * t46;
    real_type t120 = t1 * t35;
    real_type t122 = t39 * t10;
    real_type t123 = t3 * t25;
    real_type t128 = t12 * t119;
    real_type t129 = t6 * t30;
    real_type t133 = t23 * t39;
    real_type t137 = 2 * t6 * t123 * t122 * t120 * t119 + 2 * t93 * t102 * t101 + t111 * t36 * t33 - t25 * t116 * t115 - 2 * t50 * t129 * t128 - 2 * t133 * t49 * t128 - 2 * t93 * t92 * t91;
    real_type t141 = 2 * t53 * phi__XO * t86 * t85 - 2 * phi__XO * t80 * t79 * t78 + t137 * t8 * t32 * t29;
    real_type t142 = t22 * t22;
    real_type t143 = t75 * t142;
    real_type t144 = t15 * t15;
    real_type t145 = 1.0 / t144;
    real_type t147 = 1.0 / t2;
    real_type t148 = 1.0 / t3;
    real_type t150 = t53 * t53;
    real_type t154 = t150 * t9 * t148 * t147 * t145 * t143 + 1;
    real_type t155 = 1.0 / t154;
    real_type t157 = cos(t58);
    real_type t160 = t9 * t8;
    real_type t161 = 1.0 / t160;
    real_type t168 = t2 * t59 * t35;
    real_type t177 = t15 * t12;
    real_type t179 = 1.0 / t74 / t25;
    real_type t182 = t23 * t23;
    real_type t214 = 1.0 / t64 / t8;
    real_type t250 = phi__XO * t23;
    real_type t254 = t35 * t46;
    real_type t269 = t5 * phi__XO * t46;
    real_type t278 = t120 * t269;
    real_type t293 = 8 * t69 * t92 * t161 * t33 - 20 * t69 * t102 * t214 * t99 - 4 * t93 * t111 * t35 * t91 - 2 * t6 * t92 * t91 + 12 * t69 * t43 / t12 / t100 / t64 / t160 * Fzr__XO * t1 + 4 * t93 * t111 * t14 * t101 + 2 * t6 * t102 * t101 - 6 * t250 * t39 * t37 * t36 * t3 * t1 + 6 * phi__XO * t6 * t3 * t50 * t10 * t1 * t254 - 6 * t50 * t93 * t115 - 6 * t250 * t116 * t115 + 4 * t68 * t63 * t25 * t39 * t214 * t97 * t14 * t269 - 4 * t68 * t123 * t39 * t161 * t278 + 8 * t6 * t3 * t23 * t122 * t278 - 8 * t133 * t129 * t12 * t269;
    real_type t302 = t154 * t154;
    real_type t303 = 1.0 / t302;
    real_type t306 = t147 * t145;
    real_type t308 = t9 * t148;
    real_type t314 = t306 * t143;
    real_type t329 = t141 * t141;
    real_type t353 = -4 * t69 / t64 / t9 * t63 * t61 * t59 * t14 - 4 * phi__XO * t6 * t161 * t4 * t157 * t155 * t141 * t15 * t35 + 12 * t69 * t100 * t3 * t168 - 2 * t6 * t161 * t3 * t168 + t157 * t155 * (8 * t5 * t182 * t53 * t79 * t77 * t179 * t22 - 4 * phi__XO * t23 * t137 * t79 * t78 - 8 * t23 * t53 * t5 * t86 * t78 + 4 * t137 * phi__XO * t86 * t85 + t293 * t8 * t32 * t29 + 2 * t53 * t6 * t32 * t29 - 2 * t80 * t79 * t78) * t177 - (-4 * phi__XO * t23 * t150 * t308 * t306 * t179 * t142 + 4 * phi__XO * t6 * t150 * t8 * t148 * t314 + 2 * t137 * t53 * t308 * t314) * t157 * t303 * t141 * t177 - t59 * t303 * t329 * t144 * t12 - 6 * t93 * t3 * t10 * t1 * t254 + 6 * phi__XO * t129 * t114 - 4 * t68 * t63 * t214 * t97 * t14 * t269 + 4 * t68 * t3 * t161 * t1 * t35 * t269;
    real_type t354 = ModelPars[107];
    real_type t357 = ModelPars[101] * ModelPars[101];
    real_type t358 = tan(alpha__r__XO);
    real_type t361 = pow(t358 - ModelPars[103], 2);
    real_type t364 = sqrt(t361 * t357 + 1);
    real_type t366 = 1.0 / t364 * ModelPars[99];
    real_type t367 = ModelPars[111];
    real_type t371 = 1.0 / t17 * t20 * ModelPars[113];
    real_type t374 = atan((lambda__r__XO + t367 + t371) * t366);
    real_type t376 = cos(t374 * t354);
    real_type t380 = atan((t367 + t371) * t366);
    real_type t382 = cos(t380 * t354);
    return 1.0 / t382 * t376 * t353;
  }

  real_type
  General::Fyr_D_2_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = phi__XO * phi__XO;
    real_type t2   = ModelPars[46];
    real_type t4   = t2 * t1 + 1;
    real_type t5   = 1.0 / t4;
    real_type t7   = ModelPars[7];
    real_type t10  = Fzr__XO - t7;
    real_type t12  = t10 * ModelPars[36] + t7 * ModelPars[34];
    real_type t14  = ModelPars[42];
    real_type t16  = t14 * t1 + 1;
    real_type t17  = 1.0 / t16;
    real_type t19  = t12 * t12;
    real_type t20  = t16 * t16;
    real_type t21  = 1.0 / t20;
    real_type t23  = ModelPars[47];
    real_type t24  = t23 * t23;
    real_type t27  = ModelPars[40];
    real_type t28  = t27 * t27;
    real_type t30  = Fzr__XO * Fzr__XO;
    real_type t33  = t4 * t4;
    real_type t34  = Fzr__XO * t27;
    real_type t36  = 1.0 / t33;
    real_type t37  = t36 * t30 * t28;
    real_type t38  = sqrt(t37);
    real_type t39  = 1.0 / t38;
    real_type t40  = t39 * t5;
    real_type t42  = ModelPars[38] * Fzr__XO;
    real_type t43  = 1.0 / t12;
    real_type t47  = -t16 * t43 * phi__XO * t42 + alpha__r__XO;
    real_type t50  = ModelPars[44];
    real_type t53  = 1.0 / t27;
    real_type t54  = t4 * t53;
    real_type t55  = t16 * t43;
    real_type t58  = -t55 * t54 * t38 * phi__XO * t50 + t47 * t40 * t34;
    real_type t59  = t58 * t58;
    real_type t63  = t59 * t33 / t30 / t28 / t24 * t21 * t19 + 1;
    real_type t64  = 1.0 / t63;
    real_type t65  = t17 * t12;
    real_type t66  = 1.0 / t23;
    real_type t67  = t66 * t65;
    real_type t68  = 1.0 / Fzr__XO;
    real_type t69  = t68 * t53;
    real_type t73  = atan(t58 * t4 * t69 * t67);
    real_type t74  = t73 * t23;
    real_type t75  = cos(t74);
    real_type t77  = phi__XO * t2;
    real_type t81  = t23 * t38;
    real_type t82  = t21 * t12;
    real_type t91  = t36 * t34;
    real_type t98  = t33 * t33;
    real_type t100 = 1.0 / t98 * t30 * Fzr__XO * t28 * t27;
    real_type t102 = 1.0 / t38 / t37;
    real_type t115 = t53 * t66;
    real_type t117 = t4 * t68;
    real_type t145 = t50 * t38;
    real_type t150 = t1 * t50;
    real_type t159 = t38 * t150;
    real_type t172 = -2 * phi__XO * t14 * t58 * t117 * t115 * t82 + 2 * t58 * phi__XO * t2 * t68 * t115 * t65 + (-2 * t77 * t47 * t39 * t91 + 2 * t77 * t47 * t102 * t100 + (-2 * t14 * t43 * t1 * t42 - t55 * t42) * t40 * t34 - t16 * t43 * t4 * t53 * t145 + 2 * t2 * t30 * t16 * t43 * t36 * t27 * t39 * t150 - 2 * t55 * t2 * t53 * t159 - 2 * t14 * t43 * t54 * t159) * t4 * t69 * t67;
    real_type t174 = t63 * t63;
    real_type t175 = 1.0 / t174;
    real_type t186 = sin(t74);
    real_type t190 = ModelPars[107];
    real_type t191 = ModelPars[99];
    real_type t193 = ModelPars[101] * ModelPars[101];
    real_type t194 = tan(alpha__r__XO);
    real_type t196 = t194 - ModelPars[103];
    real_type t197 = t196 * t196;
    real_type t199 = t197 * t193 + 1;
    real_type t200 = sqrt(t199);
    real_type t202 = 1.0 / t200 * t191;
    real_type t203 = ModelPars[111];
    real_type t207 = 1.0 / t7 * t10 * ModelPars[113];
    real_type t208 = lambda__r__XO + t203 + t207;
    real_type t210 = atan(t208 * t202);
    real_type t211 = t210 * t190;
    real_type t212 = cos(t211);
    real_type t214 = t203 + t207;
    real_type t216 = atan(t214 * t202);
    real_type t217 = t216 * t190;
    real_type t218 = cos(t217);
    real_type t219 = 1.0 / t218;
    real_type t243 = -2 * t77 / t33 / t4 * t30 * t28 * t186 * t39 + t75 * t64 * t172 * t81 + t54 * t145 - 2 * t2 * t30 * t36 * t27 * t39 * t150 + 2 * t2 * t53 * t38 * t150;
    real_type t246 = 1.0 / t200 / t199;
    real_type t251 = t194 * t194;
    real_type t252 = 1 + t251;
    real_type t254 = t191 * t191;
    real_type t256 = 1.0 / t199 * t254;
    real_type t257 = t208 * t208;
    real_type t261 = sin(t211);
    real_type t267 = t218 * t218;
    real_type t275 = t214 * t214;
    real_type t280 = sin(t217);
    return t219 * t212 * (-2 * t77 * t75 * t64 * t17 * t12 * t5 + t75 * t64 * (-2 * phi__XO * t14 * t39 * t66 * t82 + 2 * t40 * t77 * t67 + (2 * phi__XO * t2 * t102 * t100 - 2 * phi__XO * t2 * t39 * t91) * t4 * t69 * t67) * t81 - 2 * t58 * t117 * t53 * t21 * t19 * t75 * t175 * t172 * t66 - t186 * t65 * t175 * t172 * t23) + t219 * t261 / (t257 * t256 + 1) * t252 * t196 * t193 * t208 * t246 * t191 * t190 * t243 - t280 / (t275 * t256 + 1) * t252 * t196 * t193 * t214 * t246 * t191 * t190 / t267 * t212 * t243;
  }

  real_type
  General::Fyr_D_2_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[40];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t6   = ModelPars[46];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[47];
    real_type t16  = ModelPars[7];
    real_type t19  = Fzr__XO - t16;
    real_type t21  = t16 * ModelPars[34] + t19 * ModelPars[36];
    real_type t22  = ModelPars[42];
    real_type t24  = t22 * t5 + 1;
    real_type t26  = 1.0 / t24 * t21;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t26;
    real_type t29  = 1.0 / t1;
    real_type t30  = 1.0 / Fzr__XO;
    real_type t31  = t30 * t29;
    real_type t32  = Fzr__XO * t1;
    real_type t34  = t13 / t8;
    real_type t36  = ModelPars[38] * Fzr__XO;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__XO * t36 + alpha__r__XO;
    real_type t44  = ModelPars[44];
    real_type t47  = t8 * t29;
    real_type t48  = t24 * t37;
    real_type t51  = -t48 * t47 * t12 * phi__XO * t44 + t41 * t34 * t32;
    real_type t55  = atan(t51 * t8 * t31 * t28);
    real_type t56  = t55 * t14;
    real_type t57  = sin(t56);
    real_type t63  = phi__XO * t6;
    real_type t68  = t24 * t24;
    real_type t69  = 1.0 / t68;
    real_type t71  = t29 * t27;
    real_type t93  = t9 * t9;
    real_type t110 = t12 * t44;
    real_type t115 = t5 * t44;
    real_type t124 = t12 * t115;
    real_type t138 = t21 * t21;
    real_type t140 = t14 * t14;
    real_type t146 = t51 * t51;
    real_type t153 = cos(t56);
    real_type t168 = ModelPars[107];
    real_type t170 = ModelPars[99];
    real_type t173 = ModelPars[101] * ModelPars[101];
    real_type t174 = tan(alpha__r__XO);
    real_type t177 = pow(t174 - ModelPars[103], 2);
    real_type t179 = t177 * t173 + 1;
    real_type t180 = sqrt(t179);
    real_type t181 = 1.0 / t180;
    real_type t182 = t170 * t170;
    real_type t185 = ModelPars[111];
    real_type t189 = 1.0 / t16 * t19 * ModelPars[113];
    real_type t190 = lambda__r__XO + t185 + t189;
    real_type t191 = t190 * t190;
    real_type t196 = t181 * t170;
    real_type t198 = atan(t190 * t196);
    real_type t200 = sin(t198 * t168);
    real_type t203 = atan((t185 + t189) * t196);
    real_type t205 = cos(t203 * t168);
    return -1.0 / t205 * t200 / (t191 / t179 * t182 + 1) * t181 * t170 * t168 * (-2 * t63 / t9 / t8 * t3 * t2 * t57 * t13 + t153 / (t146 * t9 / t3 / t2 / t140 * t69 * t138 + 1) * (-2 * phi__XO * t22 * t51 * t8 * t30 * t71 * t69 * t21 + 2 * t51 * phi__XO * t6 * t30 * t71 * t26 + (-2 * t63 * t41 * t13 * t10 * t32 + 2 * t63 * t41 / t12 / t11 / t93 * t3 * Fzr__XO * t2 * t1 + (-2 * t22 * t37 * t5 * t36 - t48 * t36) * t34 * t32 - t24 * t37 * t8 * t29 * t110 + 2 * t6 * t3 * t24 * t37 * t10 * t1 * t13 * t115 - 2 * t48 * t6 * t29 * t124 - 2 * t22 * t37 * t47 * t124) * t8 * t31 * t28) * t14 * t12 + t47 * t110 - 2 * t6 * t3 * t10 * t1 * t13 * t115 + 2 * t6 * t29 * t12 * t115);
  }

  real_type
  General::Fyr_D_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[7];
    real_type t5   = Fzr__XO - t2;
    real_type t7   = t2 * ModelPars[34] + t5 * ModelPars[36];
    real_type t8   = phi__XO * phi__XO;
    real_type t11  = ModelPars[42] * t8 + 1;
    real_type t13  = 1.0 / t11 * t7;
    real_type t14  = t7 * t7;
    real_type t15  = t11 * t11;
    real_type t18  = ModelPars[47];
    real_type t19  = t18 * t18;
    real_type t22  = ModelPars[40];
    real_type t23  = t22 * t22;
    real_type t25  = Fzr__XO * Fzr__XO;
    real_type t30  = ModelPars[46] * t8 + 1;
    real_type t31  = t30 * t30;
    real_type t37  = sqrt(1.0 / t31 * t25 * t23);
    real_type t42  = 1.0 / t7;
    real_type t50  = ModelPars[44] * phi__XO;
    real_type t52  = 1.0 / t22;
    real_type t57  = (-t11 * t42 * phi__XO * ModelPars[38] * Fzr__XO + alpha__r__XO) / t37 / t30 * Fzr__XO * t22 - t11 * t42 * t30 * t52 * t37 * t50;
    real_type t58  = t57 * t57;
    real_type t72  = atan(t57 * t30 / Fzr__XO * t52 / t18 * t13);
    real_type t73  = t72 * t18;
    real_type t74  = cos(t73);
    real_type t75  = ModelPars[107];
    real_type t76  = ModelPars[99];
    real_type t78  = ModelPars[101] * ModelPars[101];
    real_type t79  = tan(alpha__r__XO);
    real_type t81  = t79 - ModelPars[103];
    real_type t82  = t81 * t81;
    real_type t84  = t82 * t78 + 1;
    real_type t85  = sqrt(t84);
    real_type t87  = 1.0 / t85 * t76;
    real_type t88  = ModelPars[111];
    real_type t92  = 1.0 / t2 * t5 * ModelPars[113];
    real_type t93  = lambda__r__XO + t88 + t92;
    real_type t95  = atan(t93 * t87);
    real_type t96  = t95 * t75;
    real_type t97  = cos(t96);
    real_type t99  = t88 + t92;
    real_type t101 = atan(t99 * t87);
    real_type t102 = t101 * t75;
    real_type t103 = cos(t102);
    real_type t104 = 1.0 / t103;
    real_type t107 = sin(t73);
    real_type t112 = t30 * t52 * t37 * t50 + t107 * t37;
    real_type t115 = 1.0 / t85 / t84;
    real_type t120 = t79 * t79;
    real_type t121 = 1 + t120;
    real_type t123 = t76 * t76;
    real_type t125 = 1.0 / t84 * t123;
    real_type t126 = t93 * t93;
    real_type t130 = sin(t96);
    real_type t136 = t103 * t103;
    real_type t144 = t99 * t99;
    real_type t149 = sin(t102);
    return t104 * t97 * t74 / (t58 * t31 / t25 / t23 / t19 / t15 * t14 + 1) * t13 + t104 * t130 / (t126 * t125 + 1) * t121 * t81 * t78 * t93 * t115 * t76 * t75 * t112 - t149 / (t144 * t125 + 1) * t121 * t81 * t78 * t99 * t115 * t76 * t75 / t136 * t97 * t112;
  }

  real_type
  General::Fyr_D_3_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[7];
    real_type t5   = Fzr__XO - t2;
    real_type t7   = t2 * ModelPars[34] + t5 * ModelPars[36];
    real_type t8   = t7 * t7;
    real_type t10  = phi__XO * phi__XO;
    real_type t13  = ModelPars[42] * t10 + 1;
    real_type t14  = t13 * t13;
    real_type t19  = 1.0 / t14 * t8;
    real_type t20  = ModelPars[47];
    real_type t21  = t20 * t20;
    real_type t22  = 1.0 / t21;
    real_type t24  = ModelPars[40];
    real_type t25  = t24 * t24;
    real_type t27  = Fzr__XO * Fzr__XO;
    real_type t32  = ModelPars[46] * t10 + 1;
    real_type t33  = t32 * t32;
    real_type t39  = sqrt(1.0 / t33 * t27 * t25);
    real_type t40  = 1.0 / t39;
    real_type t44  = 1.0 / t7;
    real_type t52  = ModelPars[44] * phi__XO;
    real_type t54  = 1.0 / t24;
    real_type t59  = (-t13 * t44 * phi__XO * ModelPars[38] * Fzr__XO + alpha__r__XO) * t40 / t32 * Fzr__XO * t24 - t13 * t44 * t32 * t54 * t39 * t52;
    real_type t60  = t59 * t59;
    real_type t64  = t60 * t33 / t27 / t25 * t22 * t19 + 1;
    real_type t65  = t64 * t64;
    real_type t66  = 1.0 / t65;
    real_type t69  = 1.0 / t13 * t7;
    real_type t72  = 1.0 / Fzr__XO;
    real_type t74  = t59 * t32;
    real_type t77  = atan(t74 * t72 * t54 / t20 * t69);
    real_type t78  = t77 * t20;
    real_type t79  = cos(t78);
    real_type t80  = ModelPars[107];
    real_type t81  = ModelPars[99];
    real_type t83  = ModelPars[101] * ModelPars[101];
    real_type t84  = tan(alpha__r__XO);
    real_type t86  = t84 - ModelPars[103];
    real_type t87  = t86 * t86;
    real_type t89  = t87 * t83 + 1;
    real_type t90  = sqrt(t89);
    real_type t92  = 1.0 / t90 * t81;
    real_type t93  = ModelPars[111];
    real_type t97  = 1.0 / t2 * t5 * ModelPars[113];
    real_type t98  = lambda__r__XO + t93 + t97;
    real_type t100 = atan(t98 * t92);
    real_type t101 = t100 * t80;
    real_type t102 = cos(t101);
    real_type t103 = t102 * t79;
    real_type t104 = t93 + t97;
    real_type t106 = atan(t104 * t92);
    real_type t107 = t106 * t80;
    real_type t108 = cos(t107);
    real_type t109 = 1.0 / t108;
    real_type t119 = sin(t78);
    real_type t125 = 1.0 / t64 * t69;
    real_type t128 = 1.0 / t90 / t89;
    real_type t129 = t128 * t81;
    real_type t134 = t84 * t84;
    real_type t135 = 1 + t134;
    real_type t136 = t81 * t81;
    real_type t138 = 1.0 / t89 * t136;
    real_type t139 = t98 * t98;
    real_type t141 = t139 * t138 + 1;
    real_type t142 = 1.0 / t141;
    real_type t144 = sin(t101);
    real_type t150 = t108 * t108;
    real_type t151 = 1.0 / t150;
    real_type t152 = t80 * t151;
    real_type t155 = t83 * t104;
    real_type t158 = t104 * t104;
    real_type t160 = t158 * t138 + 1;
    real_type t161 = 1.0 / t160;
    real_type t162 = sin(t107);
    real_type t163 = t162 * t161;
    real_type t172 = t32 * t54 * t39 * t52 + t119 * t39;
    real_type t173 = t80 * t172;
    real_type t174 = t89 * t89;
    real_type t176 = 1.0 / t90 / t174;
    real_type t180 = t83 * t83;
    real_type t181 = t87 * t180;
    real_type t182 = t135 * t135;
    real_type t183 = t182 * t181;
    real_type t185 = t109 * t144 * t142;
    real_type t203 = t136 * t81;
    real_type t204 = t174 * t89;
    real_type t206 = 1.0 / t90 / t204;
    real_type t211 = t141 * t141;
    real_type t212 = 1.0 / t211;
    real_type t218 = t80 * t80;
    real_type t219 = t218 * t172;
    real_type t220 = 1.0 / t204;
    real_type t239 = t102 * t172;
    real_type t248 = t160 * t160;
    real_type t249 = 1.0 / t248;
    real_type t250 = t249 * t182;
    real_type t251 = t162 * t162;
    real_type t256 = t151 * t239;
    real_type t257 = t81 * t80;
    real_type t263 = t162 * t161 * t182;
    return -2 * t40 * t74 * t72 * t54 * t22 * t109 * t103 * t66 / t14 / t13 * t8 * t7 - t109 * t102 * t119 * t40 * t66 * t19 + 2 * t109 * t144 * t142 * t135 * t86 * t83 * t98 * t129 * t80 * t79 * t125 - 2 * t163 * t135 * t86 * t155 * t129 * t152 * t103 * t125 - 3 * t185 * t183 * t98 * t176 * t81 * t173 + t185 * t182 * t83 * t98 * t129 * t173 + 2 * t185 * t135 * t84 * t86 * t83 * t98 * t128 * t81 * t173 + 2 * t109 * t144 * t212 * t183 * t139 * t98 * t206 * t203 * t173 - t109 * t102 * t212 * t183 * t139 * t220 * t136 * t219 - 2 * t163 * t104 * t151 * t144 * t142 * t182 * t181 * t98 * t220 * t136 * t219 + 2 * t251 * t250 * t87 * t180 * t158 * t220 * t136 * t218 / t150 / t108 * t239 + 3 * t263 * t87 * t180 * t104 * t176 * t257 * t256 - t263 * t83 * t104 * t128 * t81 * t152 * t239 - 2 * t163 * t135 * t84 * t86 * t155 * t128 * t257 * t256 - 2 * t162 * t250 * t87 * t180 * t158 * t104 * t206 * t203 * t80 * t256 + t249 * t182 * t87 * t180 * t158 * t220 * t136 * t218 * t109 * t239;
  }

  real_type
  General::Fyr_D_3_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[7];
    real_type t5   = Fzr__XO - t2;
    real_type t7   = t2 * ModelPars[34] + t5 * ModelPars[36];
    real_type t8   = phi__XO * phi__XO;
    real_type t11  = ModelPars[42] * t8 + 1;
    real_type t13  = 1.0 / t11 * t7;
    real_type t14  = t7 * t7;
    real_type t15  = t11 * t11;
    real_type t18  = ModelPars[47];
    real_type t19  = t18 * t18;
    real_type t22  = ModelPars[40];
    real_type t23  = t22 * t22;
    real_type t25  = Fzr__XO * Fzr__XO;
    real_type t30  = ModelPars[46] * t8 + 1;
    real_type t31  = t30 * t30;
    real_type t37  = sqrt(1.0 / t31 * t25 * t23);
    real_type t42  = 1.0 / t7;
    real_type t50  = ModelPars[44] * phi__XO;
    real_type t52  = 1.0 / t22;
    real_type t57  = (-t11 * t42 * phi__XO * ModelPars[38] * Fzr__XO + alpha__r__XO) / t37 / t30 * Fzr__XO * t22 - t11 * t42 * t30 * t52 * t37 * t50;
    real_type t58  = t57 * t57;
    real_type t71  = atan(t57 * t30 / Fzr__XO * t52 / t18 * t13);
    real_type t72  = t71 * t18;
    real_type t73  = cos(t72);
    real_type t75  = ModelPars[107];
    real_type t78  = ModelPars[99];
    real_type t80  = ModelPars[101] * ModelPars[101];
    real_type t81  = tan(alpha__r__XO);
    real_type t83  = t81 - ModelPars[103];
    real_type t84  = t83 * t83;
    real_type t86  = t84 * t80 + 1;
    real_type t87  = sqrt(t86);
    real_type t89  = 1.0 / t87 * t78;
    real_type t90  = t78 * t78;
    real_type t92  = 1.0 / t86 * t90;
    real_type t93  = ModelPars[111];
    real_type t97  = 1.0 / t2 * t5 * ModelPars[113];
    real_type t98  = lambda__r__XO + t93 + t97;
    real_type t99  = t98 * t98;
    real_type t101 = t99 * t92 + 1;
    real_type t102 = 1.0 / t101;
    real_type t104 = atan(t98 * t89);
    real_type t105 = t104 * t75;
    real_type t106 = sin(t105);
    real_type t108 = t93 + t97;
    real_type t110 = atan(t108 * t89);
    real_type t111 = t110 * t75;
    real_type t112 = cos(t111);
    real_type t113 = 1.0 / t112;
    real_type t114 = t113 * t106 * t102;
    real_type t117 = sin(t72);
    real_type t122 = t30 * t52 * t37 * t50 + t117 * t37;
    real_type t123 = t75 * t122;
    real_type t129 = t81 * t81;
    real_type t130 = 1 + t129;
    real_type t131 = t130 * t83;
    real_type t135 = t86 * t86;
    real_type t142 = t130 * t83 * t80;
    real_type t143 = t101 * t101;
    real_type t144 = 1.0 / t143;
    real_type t150 = t75 * t75;
    real_type t151 = t150 * t122;
    real_type t152 = 1.0 / t135;
    real_type t156 = cos(t105);
    real_type t165 = t112 * t112;
    real_type t169 = t108 * t108;
    real_type t173 = sin(t111);
    return -t114 * t89 * t75 * t73 / (t58 * t31 / t25 / t23 / t19 / t15 * t14 + 1) * t13 + t114 * t131 * t80 / t87 / t86 * t78 * t123 - 2 * t113 * t106 * t144 * t142 * t99 / t87 / t135 * t90 * t78 * t123 + t113 * t156 * t144 * t142 * t98 * t152 * t90 * t151 + t173 / (t169 * t92 + 1) * t131 * t80 * t108 / t165 * t106 * t102 * t152 * t90 * t151;
  }

  real_type
  General::Fyr_D_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[40];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t8   = ModelPars[46] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t12  = sqrt(1.0 / t9 * t3 * t2);
    real_type t13  = ModelPars[47];
    real_type t15  = ModelPars[7];
    real_type t18  = Fzr__XO - t15;
    real_type t20  = t15 * ModelPars[34] + t18 * ModelPars[36];
    real_type t23  = ModelPars[42] * t5 + 1;
    real_type t28  = 1.0 / t1;
    real_type t37  = 1.0 / t20;
    real_type t45  = ModelPars[44] * phi__XO;
    real_type t55  = atan(((-t23 * t37 * phi__XO * ModelPars[38] * Fzr__XO + alpha__r__XO) / t12 / t8 * Fzr__XO * t1 - t23 * t37 * t8 * t28 * t12 * t45) * t8 / Fzr__XO * t28 / t13 / t23 * t20);
    real_type t57  = sin(t55 * t13);
    real_type t63  = ModelPars[107];
    real_type t65  = ModelPars[99];
    real_type t68  = ModelPars[101] * ModelPars[101];
    real_type t69  = tan(alpha__r__XO);
    real_type t72  = pow(t69 - ModelPars[103], 2);
    real_type t74  = t72 * t68 + 1;
    real_type t75  = sqrt(t74);
    real_type t76  = 1.0 / t75;
    real_type t77  = t65 * t65;
    real_type t80  = ModelPars[111];
    real_type t84  = 1.0 / t15 * t18 * ModelPars[113];
    real_type t85  = lambda__r__XO + t80 + t84;
    real_type t86  = t85 * t85;
    real_type t91  = t76 * t65;
    real_type t93  = atan(t85 * t91);
    real_type t95  = sin(t93 * t63);
    real_type t98  = atan((t80 + t84) * t91);
    real_type t100 = cos(t98 * t63);
    return -1.0 / t100 * t95 / (t86 / t74 * t77 + 1) * t76 * t65 * t63 * (t8 * t28 * t12 * t45 + t57 * t12);
  }

  real_type
  General::Fyr_D_4_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[40];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t8   = ModelPars[46] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t12  = sqrt(1.0 / t9 * t3 * t2);
    real_type t13  = ModelPars[47];
    real_type t15  = ModelPars[7];
    real_type t18  = Fzr__XO - t15;
    real_type t20  = t15 * ModelPars[34] + t18 * ModelPars[36];
    real_type t23  = ModelPars[42] * t5 + 1;
    real_type t28  = 1.0 / t1;
    real_type t37  = 1.0 / t20;
    real_type t45  = ModelPars[44] * phi__XO;
    real_type t55  = atan(((-t23 * t37 * phi__XO * ModelPars[38] * Fzr__XO + alpha__r__XO) / t12 / t8 * Fzr__XO * t1 - t23 * t37 * t8 * t28 * t12 * t45) * t8 / Fzr__XO * t28 / t13 / t23 * t20);
    real_type t57  = sin(t55 * t13);
    real_type t62  = t8 * t28 * t12 * t45 + t57 * t12;
    real_type t63  = ModelPars[107];
    real_type t65  = ModelPars[99];
    real_type t66  = t65 * t65;
    real_type t69  = ModelPars[101] * ModelPars[101];
    real_type t70  = tan(alpha__r__XO);
    real_type t73  = pow(t70 - ModelPars[103], 2);
    real_type t75  = t73 * t69 + 1;
    real_type t76  = sqrt(t75);
    real_type t81  = 1.0 / t75;
    real_type t83  = ModelPars[111];
    real_type t87  = 1.0 / t15 * t18 * ModelPars[113];
    real_type t88  = lambda__r__XO + t83 + t87;
    real_type t89  = t88 * t88;
    real_type t92  = pow(t89 * t81 * t66 + 1, 2);
    real_type t93  = 1.0 / t92;
    real_type t95  = 1.0 / t76 * t65;
    real_type t97  = atan(t88 * t95);
    real_type t98  = t97 * t63;
    real_type t99  = sin(t98);
    real_type t103 = atan((t83 + t87) * t95);
    real_type t105 = cos(t103 * t63);
    real_type t106 = 1.0 / t105;
    real_type t111 = t63 * t63;
    real_type t115 = cos(t98);
    return 2 * t88 * t106 * t99 * t93 / t76 / t75 * t66 * t65 * t63 * t62 - t106 * t115 * t93 * t81 * t66 * t111 * t62;
  }

  real_type
  General::Mzf( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t4   = ModelPars[6];
    real_type t9   = t4 * ModelPars[33] + (Fzf__XO - t4) * ModelPars[35];
    real_type t13  = phi__f__XO * phi__f__XO;
    real_type t17  = 1.0 / (ModelPars[54] * t13 + 1);
    real_type t18  = t17 * ModelPars[48];
    real_type t20  = ModelPars[39];
    real_type t25  = ModelPars[45] * t13 + 1;
    real_type t26  = 1.0 / t25;
    real_type t27  = t20 * t20;
    real_type t28  = Fzf__XO * Fzf__XO;
    real_type t30  = t25 * t25;
    real_type t33  = sqrt(1.0 / t30 * t28 * t27);
    real_type t34  = 1.0 / t33;
    real_type t38  = atan(alpha__f__XO * t34 * t26 * Fzf__XO * t20 * ModelPars[56]);
    real_type t40  = cos(t38 * t18);
    real_type t42  = ModelPars[47];
    real_type t52  = atan(alpha__f__XO * t34 / t42 / (ModelPars[41] * t13 + 1) * t9);
    real_type t54  = sin(t52 * t42);
    real_type t60  = ModelPars[55];
    real_type t62  = atan(phi__f__XO * t60);
    real_type t76  = atan(alpha__f__XO * t34 * t26 * Fzf__XO * t20 / (ModelPars[53] * t13 + 1) * ModelPars[57]);
    real_type t78  = cos(t76 * t18);
    return -t54 * t33 * t17 * t40 / t9 * Fzf__XO * ModelPars[49] + t78 / t60 * t62 * ModelPars[51] * Fzf__XO;
  }

  real_type
  General::Mzf_D_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[49];
    real_type t3   = ModelPars[6];
    real_type t5   = ModelPars[35];
    real_type t8   = t3 * ModelPars[33] + (Fzf__XO - t3) * t5;
    real_type t9   = 1.0 / t8;
    real_type t11  = ModelPars[48];
    real_type t12  = phi__f__XO * phi__f__XO;
    real_type t15  = ModelPars[54] * t12 + 1;
    real_type t16  = 1.0 / t15;
    real_type t17  = t16 * t11;
    real_type t18  = ModelPars[56];
    real_type t19  = ModelPars[39];
    real_type t20  = t19 * t18;
    real_type t24  = ModelPars[45] * t12 + 1;
    real_type t25  = 1.0 / t24;
    real_type t26  = t19 * t19;
    real_type t27  = Fzf__XO * Fzf__XO;
    real_type t29  = t24 * t24;
    real_type t30  = 1.0 / t29;
    real_type t31  = t30 * t27 * t26;
    real_type t32  = sqrt(t31);
    real_type t33  = 1.0 / t32;
    real_type t35  = alpha__f__XO * t33 * t25;
    real_type t37  = atan(t35 * Fzf__XO * t20);
    real_type t38  = t37 * t17;
    real_type t39  = cos(t38);
    real_type t41  = t32 * t16;
    real_type t42  = ModelPars[47];
    real_type t45  = ModelPars[41] * t12 + 1;
    real_type t46  = 1.0 / t45;
    real_type t47  = t46 * t8;
    real_type t48  = 1.0 / t42;
    real_type t50  = alpha__f__XO * t33 * t48;
    real_type t52  = atan(t50 * t47);
    real_type t53  = t52 * t42;
    real_type t54  = sin(t53);
    real_type t57  = Fzf__XO * t1;
    real_type t58  = t8 * t8;
    real_type t66  = t15 * t15;
    real_type t71  = t26 * t19;
    real_type t75  = 1.0 / t29 / t24;
    real_type t77  = 1.0 / t32 / t31;
    real_type t82  = alpha__f__XO * alpha__f__XO;
    real_type t83  = t18 * t18;
    real_type t88  = sin(t38);
    real_type t94  = t39 * t9;
    real_type t113 = t45 * t45;
    real_type t116 = t42 * t42;
    real_type t128 = cos(t53);
    real_type t132 = ModelPars[51];
    real_type t133 = ModelPars[55];
    real_type t135 = atan(phi__f__XO * t133);
    real_type t137 = 1.0 / t133;
    real_type t138 = ModelPars[57];
    real_type t141 = ModelPars[53] * t12 + 1;
    real_type t143 = 1.0 / t141 * t138;
    real_type t144 = t19 * t143;
    real_type t149 = atan(alpha__f__XO * t33 * t25 * Fzf__XO * t144);
    real_type t150 = t149 * t17;
    real_type t151 = cos(t150);
    real_type t164 = t138 * t138;
    real_type t165 = t141 * t141;
    real_type t172 = sin(t150);
    return -t54 * t41 * t39 * t9 * t1 + t5 * t54 * t41 * t39 / t58 * t57 + t54 * t32 * t88 / (t83 * t82 + 1) * (-alpha__f__XO * t77 * t75 * t27 * t71 * t18 + t35 * t20) / t66 * t11 * t9 * t57 - t30 * t26 * t54 * t33 * t16 * t94 * t27 * t1 - t128 / (t82 * t29 / t27 / t26 / t116 / t113 * t58 + 1) * (-t30 * Fzf__XO * t26 * alpha__f__XO * t77 * t48 * t47 + t50 * t46 * t5) * t42 * t32 * t16 * t94 * t57 + t151 * t137 * t135 * t132 - t172 / (t82 / t165 * t164 + 1) * (-alpha__f__XO * t77 * t75 * t27 * t71 * t143 + t35 * t144) * t17 * t137 * t135 * Fzf__XO * t132;
  }

  real_type
  General::Mzf_D_1_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[49];
    real_type t2   = Fzf__XO * t1;
    real_type t4   = ModelPars[6];
    real_type t6   = ModelPars[35];
    real_type t9   = t4 * ModelPars[33] + (Fzf__XO - t4) * t6;
    real_type t10  = 1.0 / t9;
    real_type t11  = ModelPars[48];
    real_type t12  = phi__f__XO * phi__f__XO;
    real_type t15  = ModelPars[54] * t12 + 1;
    real_type t16  = 1.0 / t15;
    real_type t17  = t16 * t11;
    real_type t18  = ModelPars[56];
    real_type t19  = ModelPars[39];
    real_type t20  = t19 * t18;
    real_type t24  = ModelPars[45] * t12 + 1;
    real_type t25  = 1.0 / t24;
    real_type t26  = t19 * t19;
    real_type t27  = Fzf__XO * Fzf__XO;
    real_type t29  = t24 * t24;
    real_type t30  = 1.0 / t29;
    real_type t31  = t30 * t27 * t26;
    real_type t32  = sqrt(t31);
    real_type t33  = 1.0 / t32;
    real_type t35  = alpha__f__XO * t33 * t25;
    real_type t37  = atan(t35 * Fzf__XO * t20);
    real_type t38  = t37 * t17;
    real_type t39  = cos(t38);
    real_type t40  = t39 * t10;
    real_type t42  = t16 * t40 * t2;
    real_type t43  = ModelPars[47];
    real_type t44  = t43 * t32;
    real_type t47  = ModelPars[41] * t12 + 1;
    real_type t48  = 1.0 / t47;
    real_type t49  = t48 * t6;
    real_type t50  = 1.0 / t43;
    real_type t52  = alpha__f__XO * t33 * t50;
    real_type t54  = t48 * t9;
    real_type t56  = 1.0 / t32 / t31;
    real_type t57  = t56 * t50;
    real_type t61  = t30 * Fzf__XO * t26 * alpha__f__XO;
    real_type t63  = -t61 * t57 * t54 + t52 * t49;
    real_type t64  = t63 * t44;
    real_type t65  = t9 * t9;
    real_type t66  = t47 * t47;
    real_type t67  = 1.0 / t66;
    real_type t69  = t43 * t43;
    real_type t70  = 1.0 / t69;
    real_type t71  = t70 * t67 * t65;
    real_type t72  = 1.0 / t26;
    real_type t73  = 1.0 / t27;
    real_type t75  = alpha__f__XO * alpha__f__XO;
    real_type t76  = t75 * t29;
    real_type t79  = t76 * t73 * t72 * t71 + 1;
    real_type t80  = t79 * t79;
    real_type t81  = 1.0 / t80;
    real_type t83  = atan(t52 * t54);
    real_type t84  = t83 * t43;
    real_type t85  = cos(t84);
    real_type t94  = t27 * Fzf__XO;
    real_type t104 = ModelPars[51];
    real_type t105 = ModelPars[55];
    real_type t107 = atan(phi__f__XO * t105);
    real_type t109 = 1.0 / t105;
    real_type t112 = ModelPars[57];
    real_type t115 = ModelPars[53] * t12 + 1;
    real_type t117 = 1.0 / t115 * t112;
    real_type t118 = t19 * t117;
    real_type t120 = t26 * t19;
    real_type t121 = t120 * t117;
    real_type t123 = 1.0 / t29 / t24;
    real_type t125 = alpha__f__XO * t56;
    real_type t128 = -t125 * t123 * t27 * t121 + t35 * t118;
    real_type t130 = t112 * t112;
    real_type t131 = t115 * t115;
    real_type t135 = t75 / t131 * t130 + 1;
    real_type t136 = 1.0 / t135;
    real_type t141 = atan(alpha__f__XO * t33 * t25 * Fzf__XO * t118);
    real_type t142 = t141 * t17;
    real_type t143 = sin(t142);
    real_type t149 = t15 * t15;
    real_type t150 = 1.0 / t149;
    real_type t151 = t150 * t11;
    real_type t153 = t120 * t18;
    real_type t155 = t56 * t123;
    real_type t158 = -alpha__f__XO * t155 * t27 * t153 + t35 * t20;
    real_type t159 = t158 * t151;
    real_type t161 = t18 * t18;
    real_type t163 = t161 * t75 + 1;
    real_type t164 = 1.0 / t163;
    real_type t165 = sin(t38);
    real_type t166 = t165 * t164;
    real_type t168 = t63 * t43;
    real_type t169 = 1.0 / t79;
    real_type t170 = t85 * t169;
    real_type t177 = t109 * t107 * Fzf__XO * t104;
    real_type t178 = t11 * t11;
    real_type t180 = t128 * t128;
    real_type t181 = t135 * t135;
    real_type t184 = cos(t142);
    real_type t188 = t27 * t1;
    real_type t189 = 1.0 / t65;
    real_type t191 = t16 * t39 * t189;
    real_type t193 = sin(t84);
    real_type t194 = t193 * t33;
    real_type t206 = t26 * t26;
    real_type t207 = t206 * t19;
    real_type t210 = t29 * t29;
    real_type t212 = 1.0 / t210 / t24;
    real_type t213 = t27 * t27;
    real_type t215 = 1.0 / t210;
    real_type t218 = 1.0 / t32 / t215 / t213 / t206;
    real_type t226 = t193 * t32 * t165;
    real_type t249 = t63 * t63;
    real_type t268 = t10 * t1;
    real_type t270 = t164 * t158;
    real_type t274 = t16 * t39;
    real_type t275 = t274 * t268;
    real_type t290 = t32 * t16;
    real_type t291 = t6 * t6;
    real_type t308 = t158 * t158;
    real_type t309 = t163 * t163;
    real_type t330 = t10 * t188;
    return (2 * t6 * t75 * t29 * t73 * t72 * t70 * t67 * t9 - 2 * t76 / t94 * t72 * t71) * t85 * t81 * t64 * t42 - 2 * t143 * t136 * t128 * t16 * t11 * t109 * t107 * t104 + 2 * t170 * t168 * t32 * t166 * t159 * t10 * t2 - t184 / t181 * t180 * t150 * t178 * t177 + 2 * t30 * t26 * t6 * t194 * t191 * t188 + t226 * t164 * (3 * alpha__f__XO * t218 * t212 * t94 * t207 * t18 - 3 * Fzf__XO * t125 * t123 * t153) * t150 * t11 * t10 * t2 - t85 * t169 * (3 * t215 * t27 * t206 * alpha__f__XO * t218 * t50 * t54 - t30 * t26 * t125 * t50 * t54 - 2 * t61 * t57 * t49) * t44 * t42 + t193 * t81 * t249 * t69 * t32 * t42 - t143 * t136 * (3 * alpha__f__XO * t218 * t212 * t94 * t207 * t117 - 3 * alpha__f__XO * Fzf__XO * t155 * t121) * t17 * t177 + 2 * t226 * t270 * t151 * t268 - 3 * t30 * Fzf__XO * t26 * t194 * t275 - 2 * t85 * t169 * t63 * t44 * t275 - 2 * t291 * t193 * t290 * t39 / t65 / t9 * t2 + t215 * t206 * t193 * t56 * t16 * t40 * t94 * t1 + t193 * t32 * t39 / t309 * t308 / t149 / t15 * t178 * t10 * t2 - 2 * t6 * t193 * t32 * t165 * t270 * t150 * t11 * t189 * t2 + 2 * t6 * t170 * t64 * t191 * t2 - 2 * t30 * t26 * t85 * t169 * t168 * t33 * t274 * t330 + 2 * t30 * t26 * t193 * t33 * t166 * t159 * t330 + 2 * t6 * t193 * t290 * t39 * t189 * t1;
  }

  real_type
  General::Mzf_D_1_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[49];
    real_type t3   = ModelPars[6];
    real_type t5   = ModelPars[35];
    real_type t8   = t3 * ModelPars[33] + (Fzf__XO - t3) * t5;
    real_type t9   = 1.0 / t8;
    real_type t10  = t9 * t1;
    real_type t11  = ModelPars[48];
    real_type t12  = phi__f__XO * phi__f__XO;
    real_type t13  = ModelPars[54];
    real_type t15  = t13 * t12 + 1;
    real_type t16  = 1.0 / t15;
    real_type t17  = t16 * t11;
    real_type t18  = ModelPars[56];
    real_type t19  = ModelPars[39];
    real_type t20  = t19 * t18;
    real_type t22  = ModelPars[45];
    real_type t24  = t22 * t12 + 1;
    real_type t25  = 1.0 / t24;
    real_type t26  = t19 * t19;
    real_type t27  = Fzf__XO * Fzf__XO;
    real_type t28  = t27 * t26;
    real_type t29  = t24 * t24;
    real_type t30  = 1.0 / t29;
    real_type t31  = t30 * t28;
    real_type t32  = sqrt(t31);
    real_type t33  = 1.0 / t32;
    real_type t34  = t33 * t25;
    real_type t35  = alpha__f__XO * t34;
    real_type t37  = atan(t35 * Fzf__XO * t20);
    real_type t38  = t37 * t17;
    real_type t39  = cos(t38);
    real_type t40  = t15 * t15;
    real_type t41  = 1.0 / t40;
    real_type t42  = t41 * t39;
    real_type t44  = ModelPars[47];
    real_type t45  = ModelPars[41];
    real_type t47  = t45 * t12 + 1;
    real_type t48  = 1.0 / t47;
    real_type t49  = t48 * t8;
    real_type t50  = 1.0 / t44;
    real_type t52  = alpha__f__XO * t33 * t50;
    real_type t54  = atan(t52 * t49);
    real_type t55  = t54 * t44;
    real_type t56  = sin(t55);
    real_type t57  = t56 * t32;
    real_type t62  = ModelPars[51];
    real_type t63  = Fzf__XO * t62;
    real_type t64  = ModelPars[55];
    real_type t65  = t64 * t64;
    real_type t68  = 1.0 / (t65 * t12 + 1);
    real_type t71  = ModelPars[57];
    real_type t72  = ModelPars[53];
    real_type t74  = t72 * t12 + 1;
    real_type t76  = 1.0 / t74 * t71;
    real_type t77  = t19 * t76;
    real_type t79  = t26 * t19;
    real_type t82  = 1.0 / t29 / t24;
    real_type t85  = 1.0 / t32 / t31;
    real_type t86  = alpha__f__XO * t85;
    real_type t89  = -t86 * t82 * t27 * t79 * t76 + t35 * t77;
    real_type t90  = t89 * t16;
    real_type t91  = t71 * t71;
    real_type t92  = t74 * t74;
    real_type t93  = 1.0 / t92;
    real_type t95  = alpha__f__XO * alpha__f__XO;
    real_type t97  = t95 * t93 * t91 + 1;
    real_type t98  = 1.0 / t97;
    real_type t100 = alpha__f__XO * t33;
    real_type t103 = atan(t100 * t25 * Fzf__XO * t77);
    real_type t104 = t103 * t17;
    real_type t105 = sin(t104);
    real_type t109 = Fzf__XO * t1;
    real_type t110 = t39 * t9;
    real_type t111 = t16 * t110;
    real_type t112 = t111 * t109;
    real_type t113 = t44 * t32;
    real_type t114 = t47 * t47;
    real_type t115 = 1.0 / t114;
    real_type t119 = phi__f__XO * t45 * t100;
    real_type t122 = t48 * t5;
    real_type t123 = t85 * t50;
    real_type t124 = alpha__f__XO * t123;
    real_type t127 = phi__f__XO * t22 * t82;
    real_type t128 = t127 * t28;
    real_type t131 = t115 * t8;
    real_type t133 = Fzf__XO * t26;
    real_type t139 = t26 * t26;
    real_type t140 = t27 * t27;
    real_type t142 = t29 * t29;
    real_type t143 = 1.0 / t142;
    real_type t146 = 1.0 / t32 / t143 / t140 / t139;
    real_type t150 = t27 * Fzf__XO;
    real_type t155 = phi__f__XO * t22 / t142 / t24;
    real_type t159 = t124 * t49;
    real_type t164 = t8 * t8;
    real_type t165 = t115 * t164;
    real_type t166 = t44 * t44;
    real_type t167 = 1.0 / t166;
    real_type t169 = 1.0 / t26;
    real_type t170 = 1.0 / t27;
    real_type t175 = t95 * t29 * t170 * t169 * t167 * t165 + 1;
    real_type t176 = 1.0 / t175;
    real_type t178 = cos(t55);
    real_type t182 = 1.0 / t164;
    real_type t183 = t39 * t182;
    real_type t192 = atan(phi__f__XO * t64);
    real_type t193 = 1.0 / t64;
    real_type t194 = t193 * t192;
    real_type t196 = t11 * t194 * t63;
    real_type t197 = t41 * t11;
    real_type t202 = t93 * t71;
    real_type t203 = Fzf__XO * t19;
    real_type t206 = phi__f__XO * t72 * alpha__f__XO;
    real_type t212 = phi__f__XO * t22 * alpha__f__XO;
    real_type t217 = t85 * t143;
    real_type t224 = -2 * phi__f__XO * t13 * t103 * t197 + t98 * (2 * t212 * t217 * t150 * t79 * t76 - 2 * t212 * t33 * t30 * t203 * t76 - 2 * t206 * t34 * t203 * t202) * t17;
    real_type t226 = cos(t104);
    real_type t232 = t41 * t11 * t9 * t109;
    real_type t234 = phi__f__XO * t22;
    real_type t235 = t234 * t100;
    real_type t238 = t79 * t18;
    real_type t244 = t139 * t19;
    real_type t247 = 1.0 / t142 / t29;
    real_type t255 = t18 * t18;
    real_type t258 = 1.0 / (t255 * t95 + 1);
    real_type t260 = sin(t38);
    real_type t265 = t27 * t1;
    real_type t270 = t30 * Fzf__XO;
    real_type t281 = -2 * phi__f__XO * t13 * t37 * t197 + t258 * (2 * t234 * t86 * t143 * t150 * t238 - 2 * t235 * t270 * t20) * t17;
    real_type t283 = t260 * t281 * t9;
    real_type t286 = t26 * t56;
    real_type t312 = -2 * t119 * t50 * t131 + 2 * t128 * t159;
    real_type t314 = t178 * t176;
    real_type t326 = -t270 * t26 * alpha__f__XO * t123 * t49 + t52 * t122;
    real_type t328 = t178 * t176 * t326;
    real_type t333 = t175 * t175;
    real_type t334 = 1.0 / t333;
    real_type t344 = t169 * t167;
    real_type t369 = t16 * t39;
    real_type t370 = t33 * t369;
    real_type t376 = 2 * phi__f__XO * t13 * t57 * t42 * t10 - t105 * t98 * t90 * t11 * t68 * t63 - t178 * t176 * (-6 * t155 * t150 * t139 * alpha__f__XO * t146 * t50 * t49 + 2 * phi__f__XO * t45 * t30 * t133 * t124 * t131 - 2 * t119 * t50 * t115 * t5 + 2 * t128 * t124 * t122 + 4 * t127 * t133 * t159) * t113 * t112 - 2 * phi__f__XO * t13 * t5 * t57 * t41 * t183 * t109 - t226 * t224 * t98 * t90 * t196 + t56 * t32 * t260 * t258 * (-6 * t234 * alpha__f__XO * t146 * t247 * t140 * t244 * t18 + 8 * t234 * t27 * alpha__f__XO * t217 * t238 - 2 * t235 * t30 * t20) * t232 + t30 * t286 * t33 * t16 * t283 * t265 + 2 * phi__f__XO * t13 * t30 * t26 * t56 * t33 * t41 * t110 * t265 - 2 * t155 * t139 * t56 * t85 * t111 * t140 * t1 + t5 * t314 * t312 * t113 * t16 * t183 * t109 + t328 * t44 * t32 * t16 * t283 * t109 + t56 * t312 * t334 * t326 * t166 * t32 * t112 + (-4 * phi__f__XO * t45 * t95 * t29 * t170 * t344 / t114 / t47 * t164 + 4 * phi__f__XO * t22 * t95 * t24 * t170 * t344 * t165) * t178 * t334 * t326 * t113 * t112 + 2 * phi__f__XO * t13 * t105 * t98 * t89 * t41 * t196 + 6 * t127 * t27 * t286 * t370 * t10;
    real_type t379 = t85 * t82;
    real_type t382 = -alpha__f__XO * t379 * t27 * t238 + t35 * t20;
    real_type t383 = t258 * t382;
    real_type t393 = t97 * t97;
    real_type t400 = phi__f__XO * t72;
    real_type t405 = t150 * t1;
    real_type t406 = t9 * t405;
    real_type t412 = t234 * t82 * t26;
    real_type t422 = t9 * t109;
    real_type t426 = t32 * t260 * t258;
    real_type t427 = t312 * t44;
    real_type t499 = t16 * t260;
    real_type t517 = t56 * t32 * t39 * t281 * t383 * t232 - 4 * t400 * t95 / t92 / t74 * t91 * t105 / t393 * t90 * t11 * t193 * t192 * t63 - 2 * t412 * t56 * t33 * t260 * t383 * t197 * t406 + 2 * t412 * t328 * t44 * t33 * t369 * t406 + t314 * t427 * t426 * t382 * t197 * t422 + 2 * phi__f__XO * t13 * t178 * t176 * t326 * t44 * t32 * t42 * t422 - 2 * t127 * t26 * t5 * t56 * t370 * t182 * t405 - 4 * phi__f__XO * t13 * t56 * t426 * t382 / t40 / t15 * t11 * t422 - t30 * t26 * t178 * t176 * t427 * t370 * t9 * t265 - t105 * t98 * (8 * phi__f__XO * t22 * t27 * t86 * t143 * t79 * t76 - 6 * t212 * t146 * t247 * t140 * t244 * t76 - 2 * t400 * t100 * t25 * t19 * t202 + 2 * t206 * t379 * t27 * t79 * t202 - 2 * t235 * t30 * t19 * t76) * t17 * t194 * t63 - t5 * t57 * t499 * t281 * t182 * t109 - t178 * t176 * t312 * t113 * t369 * t10 + t57 * t499 * t281 * t10 + t226 * t68 * t62 - t105 * t224 * t193 * t192 * t62;
    return t376 + t517;
  }

  real_type
  General::Mzf_D_1_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[49];
    real_type t3   = ModelPars[6];
    real_type t5   = ModelPars[35];
    real_type t8   = t3 * ModelPars[33] + (Fzf__XO - t3) * t5;
    real_type t9   = 1.0 / t8;
    real_type t11  = ModelPars[48];
    real_type t12  = phi__f__XO * phi__f__XO;
    real_type t15  = ModelPars[54] * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t17  = 1.0 / t16;
    real_type t18  = t17 * t11;
    real_type t19  = ModelPars[56];
    real_type t20  = t19 * t18;
    real_type t22  = ModelPars[39];
    real_type t26  = ModelPars[45] * t12 + 1;
    real_type t27  = 1.0 / t26;
    real_type t29  = alpha__f__XO * alpha__f__XO;
    real_type t30  = t19 * t19;
    real_type t32  = t30 * t29 + 1;
    real_type t33  = 1.0 / t32;
    real_type t34  = 1.0 / t15;
    real_type t35  = t34 * t11;
    real_type t36  = t22 * t19;
    real_type t38  = t22 * t22;
    real_type t39  = Fzf__XO * Fzf__XO;
    real_type t41  = t26 * t26;
    real_type t42  = 1.0 / t41;
    real_type t43  = t42 * t39 * t38;
    real_type t44  = sqrt(t43);
    real_type t45  = 1.0 / t44;
    real_type t46  = t45 * t27;
    real_type t47  = alpha__f__XO * t46;
    real_type t49  = atan(t47 * Fzf__XO * t36);
    real_type t50  = t49 * t35;
    real_type t51  = sin(t50);
    real_type t52  = t51 * t33;
    real_type t53  = ModelPars[47];
    real_type t56  = ModelPars[41] * t12 + 1;
    real_type t57  = 1.0 / t56;
    real_type t58  = t57 * t8;
    real_type t59  = 1.0 / t53;
    real_type t60  = t45 * t59;
    real_type t61  = alpha__f__XO * t60;
    real_type t63  = atan(t61 * t58);
    real_type t64  = t63 * t53;
    real_type t65  = sin(t64);
    real_type t70  = cos(t50);
    real_type t73  = t8 * t8;
    real_type t74  = t56 * t56;
    real_type t75  = 1.0 / t74;
    real_type t77  = t53 * t53;
    real_type t80  = 1.0 / t38;
    real_type t86  = t29 * t41 / t39 * t80 / t77 * t75 * t73 + 1;
    real_type t87  = 1.0 / t86;
    real_type t89  = cos(t64);
    real_type t90  = t89 * t87 * t57;
    real_type t93  = t39 * t1;
    real_type t97  = t27 * t22;
    real_type t103 = Fzf__XO * t1;
    real_type t104 = t70 * t9;
    real_type t107 = t89 * t87;
    real_type t115 = t38 * t22;
    real_type t116 = t115 * t19;
    real_type t118 = 1.0 / t41 / t26;
    real_type t119 = t118 * t39;
    real_type t121 = 1.0 / t44 / t43;
    real_type t122 = t121 * t119;
    real_type t136 = -alpha__f__XO * t121 * t118 * t39 * t116 + t47 * t36;
    real_type t137 = t136 * t18;
    real_type t139 = t32 * t32;
    real_type t140 = 1.0 / t139;
    real_type t148 = t9 * t93;
    real_type t149 = t11 * t11;
    real_type t168 = t57 * t5;
    real_type t173 = t42 * Fzf__XO;
    real_type t176 = -t173 * t38 * alpha__f__XO * t121 * t59 * t58 + t61 * t168;
    real_type t177 = t176 * t53;
    real_type t197 = t34 * t70;
    real_type t201 = t86 * t86;
    real_type t202 = 1.0 / t201;
    real_type t216 = ModelPars[51];
    real_type t217 = ModelPars[55];
    real_type t219 = atan(phi__f__XO * t217);
    real_type t221 = 1.0 / t217;
    real_type t223 = ModelPars[57];
    real_type t228 = ModelPars[53] * t12 + 1;
    real_type t229 = 1.0 / t228;
    real_type t232 = t223 * t223;
    real_type t233 = t228 * t228;
    real_type t235 = 1.0 / t233 * t232;
    real_type t237 = t29 * t235 + 1;
    real_type t238 = 1.0 / t237;
    real_type t239 = t229 * t223;
    real_type t240 = t22 * t239;
    real_type t245 = atan(alpha__f__XO * t45 * t27 * Fzf__XO * t240);
    real_type t246 = t245 * t35;
    real_type t247 = sin(t246);
    real_type t252 = Fzf__XO * t216;
    real_type t257 = t115 * t239;
    real_type t272 = -alpha__f__XO * t121 * t119 * t257 + t47 * t240;
    real_type t273 = t237 * t237;
    real_type t274 = 1.0 / t273;
    real_type t289 = cos(t246);
    return 2 * t65 * t52 * t27 * Fzf__XO * t22 * t20 * t9 * t1 - 2 * t90 * t34 * t70 * t1 - t5 * t65 * t51 * t33 * t97 * t20 / t73 * t93 + t5 * t107 * t57 * t34 * t104 * t103 + t65 * t44 * t51 * t33 * (-t122 * t116 + t46 * t36) * t17 * t11 * t9 * t103 - 2 * alpha__f__XO * t30 * t65 * t44 * t51 * t140 * t137 * t9 * t103 + t65 * t70 * t27 * t22 * t19 * t140 * t136 / t16 / t15 * t149 * t148 + t90 * t52 * t137 * t103 + t107 * t177 * t51 * t33 * t27 * t36 * t18 * t148 - t89 * t87 * (-t173 * t38 * t121 * t59 * t58 + t60 * t168) * t53 * t44 * t34 * t104 * t103 + 2 * alpha__f__XO * t41 * t80 * t75 * t89 * t202 * t176 * t59 * t44 * t197 * t8 / Fzf__XO * t1 + t65 * t57 * t202 * t177 * t197 * t103 - t247 * t238 * t46 * Fzf__XO * t22 * t229 * t223 * t35 * t221 * t219 * t216 - t247 * t238 * (t45 * t97 * t239 - t122 * t257) * t35 * t221 * t219 * t252 + 2 * alpha__f__XO * t235 * t247 * t274 * t272 * t34 * t11 * t221 * t219 * t252 - t289 * t45 * t97 * t229 * t223 * t274 * t272 * t17 * t149 * t221 * t219 * t39 * t216;
  }

  real_type
  General::Mzf_D_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[49];
    real_type t2   = Fzf__XO * t1;
    real_type t4   = ModelPars[6];
    real_type t9   = t4 * ModelPars[33] + (Fzf__XO - t4) * ModelPars[35];
    real_type t10  = 1.0 / t9;
    real_type t11  = ModelPars[48];
    real_type t12  = phi__f__XO * phi__f__XO;
    real_type t13  = ModelPars[54];
    real_type t15  = t13 * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t17  = 1.0 / t16;
    real_type t18  = t17 * t11;
    real_type t19  = ModelPars[56];
    real_type t20  = ModelPars[39];
    real_type t21  = t20 * t19;
    real_type t23  = ModelPars[45];
    real_type t25  = t23 * t12 + 1;
    real_type t26  = 1.0 / t25;
    real_type t27  = t20 * t20;
    real_type t28  = Fzf__XO * Fzf__XO;
    real_type t29  = t28 * t27;
    real_type t30  = t25 * t25;
    real_type t31  = 1.0 / t30;
    real_type t32  = t31 * t29;
    real_type t33  = sqrt(t32);
    real_type t34  = 1.0 / t33;
    real_type t35  = t34 * t26;
    real_type t38  = atan(alpha__f__XO * t35 * Fzf__XO * t21);
    real_type t43  = 1.0 / t15;
    real_type t44  = t43 * t11;
    real_type t47  = alpha__f__XO * t34;
    real_type t48  = phi__f__XO * t23;
    real_type t51  = t27 * t20;
    real_type t53  = t28 * Fzf__XO;
    real_type t54  = t30 * t30;
    real_type t55  = 1.0 / t54;
    real_type t59  = 1.0 / t33 / t32;
    real_type t65  = alpha__f__XO * alpha__f__XO;
    real_type t66  = t19 * t19;
    real_type t75  = t38 * t44;
    real_type t76  = sin(t75);
    real_type t78  = ModelPars[47];
    real_type t79  = ModelPars[41];
    real_type t81  = t79 * t12 + 1;
    real_type t83  = 1.0 / t81 * t9;
    real_type t84  = 1.0 / t78;
    real_type t88  = atan(alpha__f__XO * t34 * t84 * t83);
    real_type t89  = t88 * t78;
    real_type t90  = sin(t89);
    real_type t94  = cos(t75);
    real_type t95  = t94 * t10;
    real_type t104 = t43 * t95;
    real_type t111 = phi__f__XO * t23 / t30 / t25;
    real_type t117 = t81 * t81;
    real_type t118 = 1.0 / t117;
    real_type t131 = t9 * t9;
    real_type t133 = t78 * t78;
    real_type t145 = cos(t89);
    real_type t150 = ModelPars[51] * Fzf__XO;
    real_type t151 = ModelPars[55];
    real_type t152 = t151 * t151;
    real_type t156 = ModelPars[57];
    real_type t157 = ModelPars[53];
    real_type t159 = t157 * t12 + 1;
    real_type t161 = 1.0 / t159 * t156;
    real_type t166 = atan(t47 * t26 * Fzf__XO * t20 * t161);
    real_type t167 = t166 * t44;
    real_type t168 = cos(t167);
    real_type t172 = atan(phi__f__XO * t151);
    real_type t179 = t159 * t159;
    real_type t180 = 1.0 / t179;
    real_type t182 = Fzf__XO * t20;
    real_type t191 = phi__f__XO * t23 * alpha__f__XO;
    real_type t201 = t156 * t156;
    real_type t210 = sin(t167);
    return t90 * t33 * t43 * t76 * (-2 * phi__f__XO * t13 * t38 * t18 + 1.0 / (t66 * t65 + 1) * (2 * t48 * alpha__f__XO * t59 * t55 * t53 * t51 * t19 - 2 * t48 * t47 * t31 * Fzf__XO * t21) * t44) * t10 * t2 + 2 * phi__f__XO * t13 * t90 * t33 * t17 * t95 * t2 + 2 * t111 * t27 * t90 * t34 * t104 * t53 * t1 - t145 / (t65 * t30 / t28 / t27 / t133 * t118 * t131 + 1) * (2 * t111 * t29 * alpha__f__XO * t59 * t84 * t83 - 2 * phi__f__XO * t79 * t47 * t84 * t118 * t9) * t78 * t33 * t104 * t2 + t168 / (t152 * t12 + 1) * t150 - t210 * (-2 * phi__f__XO * t13 * t166 * t18 + 1.0 / (t65 * t180 * t201 + 1) * (-2 * phi__f__XO * t157 * alpha__f__XO * t35 * t182 * t180 * t156 + 2 * t191 * t59 * t55 * t53 * t51 * t161 - 2 * t191 * t34 * t31 * t182 * t161) * t44) / t151 * t172 * t150;
  }

  real_type
  General::Mzf_D_2_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[49];
    real_type t2   = Fzf__XO * t1;
    real_type t4   = ModelPars[6];
    real_type t9   = t4 * ModelPars[33] + (Fzf__XO - t4) * ModelPars[35];
    real_type t10  = 1.0 / t9;
    real_type t11  = ModelPars[48];
    real_type t12  = phi__f__XO * phi__f__XO;
    real_type t13  = ModelPars[54];
    real_type t15  = t13 * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t18  = 1.0 / t16 / t15;
    real_type t19  = t18 * t11;
    real_type t20  = ModelPars[56];
    real_type t21  = ModelPars[39];
    real_type t22  = t21 * t20;
    real_type t23  = Fzf__XO * t22;
    real_type t24  = ModelPars[45];
    real_type t26  = t24 * t12 + 1;
    real_type t27  = 1.0 / t26;
    real_type t28  = t21 * t21;
    real_type t29  = Fzf__XO * Fzf__XO;
    real_type t30  = t29 * t28;
    real_type t31  = t26 * t26;
    real_type t32  = 1.0 / t31;
    real_type t33  = t32 * t30;
    real_type t34  = sqrt(t33);
    real_type t35  = 1.0 / t34;
    real_type t36  = t35 * t27;
    real_type t39  = atan(alpha__f__XO * t36 * t23);
    real_type t40  = t13 * t13;
    real_type t45  = 1.0 / t16;
    real_type t46  = t45 * t11;
    real_type t49  = alpha__f__XO * t35;
    real_type t50  = phi__f__XO * t24;
    real_type t53  = t28 * t21;
    real_type t54  = t53 * t20;
    real_type t55  = t29 * Fzf__XO;
    real_type t56  = t31 * t31;
    real_type t57  = 1.0 / t56;
    real_type t61  = 1.0 / t34 / t33;
    real_type t62  = alpha__f__XO * t61;
    real_type t66  = -2 * t50 * t49 * t32 * Fzf__XO * t22 + 2 * t50 * t62 * t57 * t55 * t54;
    real_type t68  = alpha__f__XO * alpha__f__XO;
    real_type t69  = t20 * t20;
    real_type t72  = 1.0 / (t69 * t68 + 1);
    real_type t77  = t13 * t39;
    real_type t80  = 1.0 / t15;
    real_type t81  = t80 * t11;
    real_type t82  = t31 * t26;
    real_type t83  = 1.0 / t82;
    real_type t86  = t24 * t24;
    real_type t87  = t12 * t86;
    real_type t92  = 1.0 / t56 / t26;
    real_type t98  = t35 * t32;
    real_type t99  = t24 * alpha__f__XO;
    real_type t100 = t99 * t98;
    real_type t103 = t28 * t28;
    real_type t104 = t103 * t21;
    real_type t106 = t29 * t29;
    real_type t107 = t106 * Fzf__XO;
    real_type t109 = 1.0 / t56 / t82;
    real_type t112 = t106 * t103;
    real_type t115 = 1.0 / t34 / t57 / t112;
    real_type t121 = t61 * t57;
    real_type t122 = t99 * t121;
    real_type t131 = t39 * t81;
    real_type t132 = sin(t131);
    real_type t134 = ModelPars[47];
    real_type t135 = ModelPars[41];
    real_type t136 = t135 * t12;
    real_type t137 = t136 + 1;
    real_type t139 = 1.0 / t137 * t9;
    real_type t140 = 1.0 / t134;
    real_type t144 = atan(alpha__f__XO * t35 * t140 * t139);
    real_type t145 = t144 * t134;
    real_type t146 = sin(t145);
    real_type t147 = t146 * t34;
    real_type t151 = ModelPars[51] * Fzf__XO;
    real_type t152 = ModelPars[55];
    real_type t153 = t152 * t152;
    real_type t155 = t153 * t12 + 1;
    real_type t157 = ModelPars[57];
    real_type t158 = ModelPars[53];
    real_type t159 = t158 * t12;
    real_type t160 = t159 + 1;
    real_type t162 = 1.0 / t160 * t157;
    real_type t167 = atan(t49 * t27 * Fzf__XO * t21 * t162);
    real_type t168 = t13 * t167;
    real_type t172 = t160 * t160;
    real_type t173 = 1.0 / t172;
    real_type t174 = t173 * t157;
    real_type t175 = Fzf__XO * t21;
    real_type t176 = t175 * t174;
    real_type t177 = t158 * alpha__f__XO;
    real_type t181 = t175 * t162;
    real_type t182 = phi__f__XO * t99;
    real_type t185 = t55 * t53;
    real_type t186 = t185 * t162;
    real_type t190 = -2 * phi__f__XO * t177 * t36 * t176 + 2 * t182 * t121 * t186 - 2 * t182 * t98 * t181;
    real_type t191 = t157 * t157;
    real_type t194 = t68 * t173 * t191 + 1;
    real_type t195 = 1.0 / t194;
    real_type t198 = -2 * phi__f__XO * t168 * t46 + t195 * t190 * t81;
    real_type t200 = t167 * t81;
    real_type t201 = sin(t200);
    real_type t205 = t155 * t155;
    real_type t208 = cos(t200);
    real_type t214 = atan(phi__f__XO * t152);
    real_type t215 = t214 * t151;
    real_type t216 = 1.0 / t152;
    real_type t229 = 1.0 / t172 / t160;
    real_type t232 = t158 * t158;
    real_type t240 = t24 * t159;
    real_type t254 = t12 * t86 * alpha__f__XO;
    real_type t275 = t194 * t194;
    real_type t294 = -2 * phi__f__XO * t77 * t46 + t72 * t66 * t81;
    real_type t297 = t132 * t294 * t10 * t2;
    real_type t298 = t34 * t45;
    real_type t299 = t13 * t146;
    real_type t304 = cos(t131);
    real_type t305 = t304 * t10;
    real_type t306 = t80 * t305;
    real_type t307 = t306 * t2;
    real_type t308 = t134 * t34;
    real_type t309 = t137 * t137;
    real_type t311 = 1.0 / t309 / t137;
    real_type t314 = t135 * t135;
    real_type t319 = 1.0 / t309;
    real_type t320 = t319 * t9;
    real_type t321 = t61 * t140;
    real_type t322 = alpha__f__XO * t321;
    real_type t326 = t24 * t83 * t29;
    real_type t330 = t140 * t320;
    real_type t340 = t12 * t86 / t56 / t31;
    real_type t344 = t322 * t139;
    real_type t346 = t12 * t86 * t57;
    real_type t356 = t9 * t9;
    real_type t357 = t319 * t356;
    real_type t358 = t134 * t134;
    real_type t359 = 1.0 / t358;
    real_type t361 = 1.0 / t28;
    real_type t362 = 1.0 / t29;
    real_type t367 = t68 * t31 * t362 * t361 * t359 * t357 + 1;
    real_type t368 = 1.0 / t367;
    real_type t370 = cos(t145);
    real_type t379 = phi__f__XO * t24 * t83;
    real_type t383 = -2 * phi__f__XO * t135 * t49 * t330 + 2 * t379 * t30 * t344;
    real_type t384 = t383 * t383;
    real_type t385 = t367 * t367;
    real_type t386 = 1.0 / t385;
    real_type t391 = t55 * t1;
    real_type t392 = t10 * t391;
    real_type t393 = t80 * t304;
    real_type t398 = t370 * t368 * t383;
    real_type t399 = t83 * t28;
    real_type t407 = t361 * t359;
    real_type t425 = t306 * t391;
    real_type t426 = t146 * t35;
    real_type t427 = t24 * t399;
    real_type t438 = t28 * t426;
    real_type t453 = t45 * t304;
    real_type t470 = t198 * t198;
    real_type t474 = t294 * t294;
    real_type t479 = t305 * t2;
    return t147 * t80 * t132 * (8 * t12 * t40 * t39 * t19 - 4 * phi__f__XO * t13 * t72 * t66 * t46 - 2 * t77 * t46 + t72 * (12 * t87 * alpha__f__XO * t115 * t109 * t107 * t104 * t20 + 8 * t87 * t49 * t83 * Fzf__XO * t22 - 20 * t87 * t62 * t92 * t55 * t54 + 2 * t122 * t55 * t54 - 2 * t100 * t23) * t81) * t10 * t2 - 2 * t201 * t198 / t155 * t151 - 2 * phi__f__XO * t153 * t208 / t205 * t151 - t201 * (8 * t12 * t40 * t167 * t19 - 4 * phi__f__XO * t13 * t195 * t190 * t46 - 2 * t168 * t46 + t195 * (8 * t12 * t232 * alpha__f__XO * t36 * t175 * t229 * t157 + 12 * t254 * t115 * t109 * t107 * t104 * t162 + 8 * t240 * t49 * t32 * t175 * t174 - 8 * t240 * t62 * t57 * t185 * t174 + 8 * t254 * t35 * t83 * t181 - 20 * t254 * t61 * t92 * t186 - 2 * t177 * t36 * t176 - 2 * t100 * t181 + 2 * t122 * t186) * t81 + 4 * phi__f__XO * t158 * t68 * t229 * t191 / t275 * t190 * t81) * t216 * t215 - 4 * phi__f__XO * t299 * t298 * t297 - t370 * t368 * (12 * t340 * t112 * alpha__f__XO * t115 * t140 * t139 + 8 * t12 * t314 * t49 * t140 * t311 * t9 - 8 * t326 * t28 * t136 * t322 * t320 + 2 * t326 * t28 * alpha__f__XO * t321 * t139 - 2 * t135 * t49 * t330 - 12 * t346 * t30 * t344) * t308 * t307 + t146 * t386 * t384 * t358 * t34 * t307 + 4 * t50 * t399 * t398 * t134 * t35 * t393 * t392 + (-4 * phi__f__XO * t135 * t68 * t31 * t362 * t407 * t311 * t356 + 4 * phi__f__XO * t24 * t68 * t26 * t362 * t407 * t357) * t370 * t386 * t383 * t308 * t307 + 2 * t427 * t426 * t425 + 4 * t340 * t103 * t146 * t61 * t306 * t107 * t1 - 12 * t346 * t438 * t425 + 2 * t398 * t134 * t34 * t80 * t297 - 4 * t379 * t438 * t80 * t132 * t294 * t392 - 8 * t427 * t12 * t299 * t35 * t453 * t392 + 4 * phi__f__XO * t13 * t370 * t368 * t383 * t134 * t34 * t453 * t10 * t2 - t208 * t470 * t216 * t215 + t147 * t393 * t474 * t10 * t2 + 2 * t299 * t298 * t479 - 8 * t12 * t40 * t146 * t34 * t18 * t479;
  }

  real_type
  General::Mzf_D_2_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[49];
    real_type t2   = Fzf__XO * t1;
    real_type t4   = ModelPars[6];
    real_type t9   = t4 * ModelPars[33] + (Fzf__XO - t4) * ModelPars[35];
    real_type t10  = 1.0 / t9;
    real_type t11  = ModelPars[48];
    real_type t12  = phi__f__XO * phi__f__XO;
    real_type t13  = ModelPars[54];
    real_type t15  = t13 * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t17  = 1.0 / t16;
    real_type t18  = t17 * t11;
    real_type t19  = ModelPars[56];
    real_type t20  = ModelPars[39];
    real_type t21  = t20 * t19;
    real_type t22  = Fzf__XO * t21;
    real_type t24  = ModelPars[45];
    real_type t26  = t24 * t12 + 1;
    real_type t27  = 1.0 / t26;
    real_type t28  = t20 * t20;
    real_type t29  = Fzf__XO * Fzf__XO;
    real_type t30  = t29 * t28;
    real_type t31  = t26 * t26;
    real_type t32  = 1.0 / t31;
    real_type t33  = t32 * t30;
    real_type t34  = sqrt(t33);
    real_type t35  = 1.0 / t34;
    real_type t36  = t35 * t27;
    real_type t37  = alpha__f__XO * alpha__f__XO;
    real_type t38  = t19 * t19;
    real_type t40  = t38 * t37 + 1;
    real_type t41  = 1.0 / t40;
    real_type t47  = 1.0 / t15;
    real_type t48  = t47 * t11;
    real_type t49  = t35 * t32;
    real_type t50  = phi__f__XO * t24;
    real_type t51  = t50 * t49;
    real_type t53  = t28 * t20;
    real_type t54  = t53 * t19;
    real_type t55  = t29 * Fzf__XO;
    real_type t57  = t31 * t31;
    real_type t58  = 1.0 / t57;
    real_type t60  = 1.0 / t34 / t33;
    real_type t61  = t60 * t58;
    real_type t62  = t50 * t61;
    real_type t70  = alpha__f__XO * t35;
    real_type t79  = 2 * t50 * alpha__f__XO * t60 * t58 * t55 * t54 - 2 * t50 * t70 * t32 * Fzf__XO * t21;
    real_type t81  = t40 * t40;
    real_type t92  = atan(alpha__f__XO * t36 * t22);
    real_type t93  = t92 * t48;
    real_type t94  = sin(t93);
    real_type t96  = ModelPars[47];
    real_type t97  = ModelPars[41];
    real_type t99  = t97 * t12 + 1;
    real_type t100 = 1.0 / t99;
    real_type t101 = t100 * t9;
    real_type t102 = 1.0 / t96;
    real_type t106 = atan(alpha__f__XO * t35 * t102 * t101);
    real_type t107 = t106 * t96;
    real_type t108 = sin(t107);
    real_type t113 = t10 * t29 * t1;
    real_type t120 = -2 * phi__f__XO * t13 * t92 * t18 + t41 * t79 * t48;
    real_type t125 = cos(t93);
    real_type t133 = t9 * t9;
    real_type t134 = t99 * t99;
    real_type t135 = 1.0 / t134;
    real_type t137 = t96 * t96;
    real_type t140 = 1.0 / t28;
    real_type t146 = t37 * t31 / t29 * t140 / t137 * t135 * t133 + 1;
    real_type t147 = 1.0 / t146;
    real_type t148 = cos(t107);
    real_type t149 = t148 * t147;
    real_type t159 = t108 * t94;
    real_type t167 = t147 * t100;
    real_type t181 = t47 * t125;
    real_type t194 = t102 * t135 * t9;
    real_type t198 = t60 * t102;
    real_type t205 = phi__f__XO * t24 / t31 / t26 * t30;
    real_type t208 = 2 * t205 * alpha__f__XO * t198 * t101 - 2 * phi__f__XO * t97 * t70 * t194;
    real_type t209 = t208 * t96;
    real_type t234 = t146 * t146;
    real_type t235 = 1.0 / t234;
    real_type t249 = ModelPars[51];
    real_type t250 = t29 * t249;
    real_type t251 = ModelPars[55];
    real_type t252 = t251 * t251;
    real_type t257 = ModelPars[57];
    real_type t260 = ModelPars[53];
    real_type t262 = t260 * t12 + 1;
    real_type t263 = 1.0 / t262;
    real_type t266 = t257 * t257;
    real_type t267 = t262 * t262;
    real_type t268 = 1.0 / t267;
    real_type t271 = t37 * t268 * t266 + 1;
    real_type t272 = 1.0 / t271;
    real_type t274 = t263 * t257;
    real_type t275 = t20 * t274;
    real_type t276 = t27 * Fzf__XO;
    real_type t279 = atan(t70 * t276 * t275);
    real_type t280 = t279 * t48;
    real_type t281 = sin(t280);
    real_type t287 = atan(phi__f__XO * t251);
    real_type t289 = 1.0 / t251;
    real_type t298 = Fzf__XO * t20;
    real_type t299 = t298 * t268 * t257;
    real_type t303 = t298 * t274;
    real_type t306 = t55 * t53 * t274;
    real_type t317 = phi__f__XO * t24 * alpha__f__XO;
    real_type t323 = -2 * phi__f__XO * t260 * alpha__f__XO * t36 * t299 - 2 * t317 * t49 * t303 + 2 * t317 * t61 * t306;
    real_type t325 = t271 * t271;
    real_type t347 = cos(t280);
    return t108 * t34 * t47 * t94 * (-2 * phi__f__XO * t13 * t41 * t36 * t22 * t18 + t41 * (2 * t62 * t55 * t54 - 2 * t51 * t22) * t48 - 2 * alpha__f__XO * t38 / t81 * t79 * t48) * t10 * t2 + t108 * t125 * t41 * t27 * t21 * t17 * t11 * t120 * t113 + t149 * t100 * t47 * t94 * t120 * t2 - 2 * phi__f__XO * t13 * t159 * t41 * t27 * t20 * t19 / t16 / t15 * t11 * t113 + 2 * phi__f__XO * t13 * t148 * t167 * t17 * t125 * t2 - 2 * t50 * t159 * t41 * t32 * t20 * t19 * t18 * t113 + 2 * phi__f__XO * t24 * t148 * t167 * t27 * t181 * t2 + t149 * t209 * t94 * t41 * t27 * t21 * t18 * t113 - t148 * t147 * (-2 * phi__f__XO * t97 * t35 * t194 + 2 * t205 * t198 * t101) * t96 * t34 * t47 * t125 * t10 * t2 + 2 * alpha__f__XO * t31 * t140 * t135 * t148 * t235 * t208 * t102 * t34 * t181 * t9 / Fzf__XO * t1 + t108 * t100 * t235 * t209 * t181 * t2 - t281 * t272 * t35 * t27 * t20 * t263 * t257 * t48 / (t252 * t12 + 1) * t250 - t281 * (-2 * phi__f__XO * t13 * t272 * t35 * t276 * t275 * t18 + t272 * (-2 * phi__f__XO * t260 * t36 * t299 - 2 * t51 * t303 + 2 * t62 * t306) * t48 - 2 * alpha__f__XO * t268 * t266 / t325 * t323 * t48) * t289 * t287 * Fzf__XO * t249 - t347 * t272 * t36 * t275 * t48 * (-2 * phi__f__XO * t13 * t279 * t18 + t272 * t323 * t48) * t289 * t287 * t250;
  }

  real_type
  General::Mzf_D_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[49];
    real_type t2   = Fzf__XO * Fzf__XO;
    real_type t5   = ModelPars[6];
    real_type t10  = t5 * ModelPars[33] + (Fzf__XO - t5) * ModelPars[35];
    real_type t12  = ModelPars[48];
    real_type t14  = phi__f__XO * phi__f__XO;
    real_type t17  = ModelPars[54] * t14 + 1;
    real_type t18  = t17 * t17;
    real_type t22  = ModelPars[56];
    real_type t23  = ModelPars[39];
    real_type t24  = t23 * t22;
    real_type t27  = ModelPars[45] * t14 + 1;
    real_type t28  = 1.0 / t27;
    real_type t30  = alpha__f__XO * alpha__f__XO;
    real_type t31  = t22 * t22;
    real_type t35  = 1.0 / t17;
    real_type t36  = t35 * t12;
    real_type t38  = t23 * t23;
    real_type t40  = t27 * t27;
    real_type t43  = sqrt(1.0 / t40 * t2 * t38);
    real_type t44  = 1.0 / t43;
    real_type t45  = t44 * t28;
    real_type t48  = atan(alpha__f__XO * t45 * Fzf__XO * t24);
    real_type t49  = t48 * t36;
    real_type t50  = sin(t49);
    real_type t52  = ModelPars[47];
    real_type t55  = ModelPars[41] * t14 + 1;
    real_type t56  = 1.0 / t55;
    real_type t62  = atan(alpha__f__XO * t44 / t52 * t56 * t10);
    real_type t63  = t62 * t52;
    real_type t64  = sin(t63);
    real_type t69  = cos(t49);
    real_type t72  = t10 * t10;
    real_type t73  = t55 * t55;
    real_type t76  = t52 * t52;
    real_type t87  = cos(t63);
    real_type t93  = ModelPars[55];
    real_type t95  = atan(phi__f__XO * t93);
    real_type t101 = ModelPars[57];
    real_type t104 = ModelPars[53] * t14 + 1;
    real_type t107 = t23 / t104 * t101;
    real_type t108 = t101 * t101;
    real_type t109 = t104 * t104;
    real_type t119 = atan(alpha__f__XO * t44 * t28 * Fzf__XO * t107);
    real_type t121 = sin(t119 * t36);
    return t64 * t50 / (t31 * t30 + 1) * t28 * t24 / t18 * t12 / t10 * t2 * t1 - t87 / (t30 * t40 / t2 / t38 / t76 / t73 * t72 + 1) * t56 * t35 * t69 * Fzf__XO * t1 - t121 / (t30 / t109 * t108 + 1) * t45 * t107 * t35 * t12 / t93 * t95 * t2 * ModelPars[51];
  }

  real_type
  General::Mzf_D_3_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[49];
    real_type t2   = Fzf__XO * Fzf__XO;
    real_type t3   = t2 * t1;
    real_type t5   = ModelPars[6];
    real_type t10  = t5 * ModelPars[33] + (Fzf__XO - t5) * ModelPars[35];
    real_type t11  = 1.0 / t10;
    real_type t13  = ModelPars[48];
    real_type t14  = phi__f__XO * phi__f__XO;
    real_type t17  = ModelPars[54] * t14 + 1;
    real_type t18  = t17 * t17;
    real_type t19  = 1.0 / t18;
    real_type t21  = ModelPars[56];
    real_type t22  = t21 * t21;
    real_type t26  = ModelPars[39];
    real_type t29  = ModelPars[45] * t14 + 1;
    real_type t30  = 1.0 / t29;
    real_type t32  = alpha__f__XO * alpha__f__XO;
    real_type t34  = t22 * t32 + 1;
    real_type t35  = t34 * t34;
    real_type t36  = 1.0 / t35;
    real_type t38  = 1.0 / t17;
    real_type t39  = t38 * t13;
    real_type t42  = t26 * t26;
    real_type t44  = t29 * t29;
    real_type t45  = 1.0 / t44;
    real_type t47  = sqrt(t45 * t2 * t42);
    real_type t48  = 1.0 / t47;
    real_type t52  = atan(alpha__f__XO * t48 * t30 * Fzf__XO * t26 * t21);
    real_type t53  = t52 * t39;
    real_type t54  = sin(t53);
    real_type t55  = ModelPars[47];
    real_type t58  = ModelPars[41] * t14 + 1;
    real_type t59  = 1.0 / t58;
    real_type t65  = atan(alpha__f__XO * t48 / t55 * t59 * t10);
    real_type t66  = t65 * t55;
    real_type t67  = sin(t66);
    real_type t76  = t13 * t13;
    real_type t84  = cos(t53);
    real_type t97  = t10 * t10;
    real_type t98  = t58 * t58;
    real_type t99  = 1.0 / t98;
    real_type t101 = t55 * t55;
    real_type t102 = 1.0 / t101;
    real_type t104 = 1.0 / t42;
    real_type t110 = t32 * t44 / t2 * t104 * t102 * t99 * t97 + 1;
    real_type t112 = cos(t66);
    real_type t124 = t110 * t110;
    real_type t125 = 1.0 / t124;
    real_type t143 = ModelPars[51];
    real_type t145 = ModelPars[55];
    real_type t147 = atan(phi__f__XO * t145);
    real_type t149 = 1.0 / t145;
    real_type t151 = ModelPars[57];
    real_type t152 = t151 * t151;
    real_type t159 = ModelPars[53] * t14 + 1;
    real_type t160 = t159 * t159;
    real_type t165 = 1.0 / t160;
    real_type t169 = pow(t32 * t165 * t152 + 1, 2);
    real_type t170 = 1.0 / t169;
    real_type t179 = atan(alpha__f__XO * t48 * t30 * Fzf__XO * t26 / t159 * t151);
    real_type t180 = t179 * t39;
    real_type t181 = sin(t180);
    real_type t193 = cos(t180);
    return -2 * alpha__f__XO * t67 * t54 * t36 * t30 * t26 * t22 * t21 * t19 * t13 * t11 * t3 + t67 * t84 * t48 * t36 * t45 * t42 * t22 / t18 / t17 * t76 * t11 * t2 * Fzf__XO * t1 + 2 * t112 / t110 * t48 * t59 * t54 / t34 * t30 * t26 * t21 * t19 * t13 * t3 + 2 * alpha__f__XO * t44 * t104 * t102 * t97 * t112 * t125 / t98 / t58 * t38 * t84 / Fzf__XO * t1 + t67 * t48 * t10 * t125 * t99 * t38 * t84 * Fzf__XO * t1 + 2 * alpha__f__XO * t181 * t170 * t48 * t30 * t26 / t160 / t159 * t152 * t151 * t38 * t13 * t149 * t147 * t2 * t143 - t193 * t170 * t165 * t152 * t19 * t76 * t149 * t147 * Fzf__XO * t143;
  }

  real_type
  General::Mzr( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t4   = ModelPars[7];
    real_type t9   = t4 * ModelPars[34] + (Fzr__XO - t4) * ModelPars[36];
    real_type t13  = phi__XO * phi__XO;
    real_type t17  = 1.0 / (ModelPars[54] * t13 + 1);
    real_type t18  = t17 * ModelPars[48];
    real_type t20  = ModelPars[40];
    real_type t25  = ModelPars[46] * t13 + 1;
    real_type t26  = 1.0 / t25;
    real_type t27  = t20 * t20;
    real_type t28  = Fzr__XO * Fzr__XO;
    real_type t30  = t25 * t25;
    real_type t33  = sqrt(1.0 / t30 * t28 * t27);
    real_type t34  = 1.0 / t33;
    real_type t38  = atan(alpha__r__XO * t34 * t26 * Fzr__XO * t20 * ModelPars[56]);
    real_type t40  = cos(t38 * t18);
    real_type t42  = ModelPars[47];
    real_type t52  = atan(alpha__r__XO * t34 / t42 / (ModelPars[42] * t13 + 1) * t9);
    real_type t54  = sin(t52 * t42);
    real_type t60  = ModelPars[55];
    real_type t62  = atan(phi__XO * t60);
    real_type t76  = atan(alpha__r__XO * t34 * t26 * Fzr__XO * t20 / (ModelPars[53] * t13 + 1) * ModelPars[57]);
    real_type t78  = cos(t76 * t18);
    return -t54 * t33 * t17 * t40 / t9 * Fzr__XO * ModelPars[50] + t78 / t60 * t62 * ModelPars[52] * Fzr__XO;
  }

  real_type
  General::Mzr_D_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[50];
    real_type t3   = ModelPars[7];
    real_type t5   = ModelPars[36];
    real_type t8   = t3 * ModelPars[34] + (Fzr__XO - t3) * t5;
    real_type t9   = 1.0 / t8;
    real_type t11  = ModelPars[48];
    real_type t12  = phi__XO * phi__XO;
    real_type t15  = ModelPars[54] * t12 + 1;
    real_type t16  = 1.0 / t15;
    real_type t17  = t16 * t11;
    real_type t18  = ModelPars[56];
    real_type t19  = ModelPars[40];
    real_type t20  = t19 * t18;
    real_type t24  = ModelPars[46] * t12 + 1;
    real_type t25  = 1.0 / t24;
    real_type t26  = t19 * t19;
    real_type t27  = Fzr__XO * Fzr__XO;
    real_type t29  = t24 * t24;
    real_type t30  = 1.0 / t29;
    real_type t31  = t30 * t27 * t26;
    real_type t32  = sqrt(t31);
    real_type t33  = 1.0 / t32;
    real_type t35  = alpha__r__XO * t33 * t25;
    real_type t37  = atan(t35 * Fzr__XO * t20);
    real_type t38  = t37 * t17;
    real_type t39  = cos(t38);
    real_type t41  = t32 * t16;
    real_type t42  = ModelPars[47];
    real_type t45  = ModelPars[42] * t12 + 1;
    real_type t46  = 1.0 / t45;
    real_type t47  = t46 * t8;
    real_type t48  = 1.0 / t42;
    real_type t50  = alpha__r__XO * t33 * t48;
    real_type t52  = atan(t50 * t47);
    real_type t53  = t52 * t42;
    real_type t54  = sin(t53);
    real_type t57  = Fzr__XO * t1;
    real_type t58  = t8 * t8;
    real_type t66  = t15 * t15;
    real_type t71  = t26 * t19;
    real_type t75  = 1.0 / t29 / t24;
    real_type t77  = 1.0 / t32 / t31;
    real_type t82  = alpha__r__XO * alpha__r__XO;
    real_type t83  = t18 * t18;
    real_type t88  = sin(t38);
    real_type t94  = t39 * t9;
    real_type t113 = t45 * t45;
    real_type t116 = t42 * t42;
    real_type t128 = cos(t53);
    real_type t132 = ModelPars[52];
    real_type t133 = ModelPars[55];
    real_type t135 = atan(phi__XO * t133);
    real_type t137 = 1.0 / t133;
    real_type t138 = ModelPars[57];
    real_type t141 = ModelPars[53] * t12 + 1;
    real_type t143 = 1.0 / t141 * t138;
    real_type t144 = t19 * t143;
    real_type t149 = atan(alpha__r__XO * t33 * t25 * Fzr__XO * t144);
    real_type t150 = t149 * t17;
    real_type t151 = cos(t150);
    real_type t164 = t138 * t138;
    real_type t165 = t141 * t141;
    real_type t172 = sin(t150);
    return -t54 * t41 * t39 * t9 * t1 + t5 * t54 * t41 * t39 / t58 * t57 + t54 * t32 * t88 / (t83 * t82 + 1) * (-alpha__r__XO * t77 * t75 * t27 * t71 * t18 + t35 * t20) / t66 * t11 * t9 * t57 - t30 * t26 * t54 * t33 * t16 * t94 * t27 * t1 - t128 / (t82 * t29 / t27 / t26 / t116 / t113 * t58 + 1) * (-t30 * Fzr__XO * t26 * alpha__r__XO * t77 * t48 * t47 + t50 * t46 * t5) * t42 * t32 * t16 * t94 * t57 + t151 * t137 * t135 * t132 - t172 / (t82 / t165 * t164 + 1) * (-alpha__r__XO * t77 * t75 * t27 * t71 * t143 + t35 * t144) * t17 * t137 * t135 * Fzr__XO * t132;
  }

  real_type
  General::Mzr_D_1_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[52];
    real_type t2   = ModelPars[55];
    real_type t4   = atan(phi__XO * t2);
    real_type t6   = 1.0 / t2;
    real_type t7   = ModelPars[48];
    real_type t10  = phi__XO * phi__XO;
    real_type t13  = ModelPars[54] * t10 + 1;
    real_type t14  = 1.0 / t13;
    real_type t15  = ModelPars[57];
    real_type t18  = ModelPars[53] * t10 + 1;
    real_type t20  = 1.0 / t18 * t15;
    real_type t21  = ModelPars[40];
    real_type t22  = t21 * t20;
    real_type t25  = ModelPars[46] * t10 + 1;
    real_type t26  = 1.0 / t25;
    real_type t27  = t21 * t21;
    real_type t28  = Fzr__XO * Fzr__XO;
    real_type t30  = t25 * t25;
    real_type t31  = 1.0 / t30;
    real_type t32  = t31 * t28 * t27;
    real_type t33  = sqrt(t32);
    real_type t34  = 1.0 / t33;
    real_type t36  = alpha__r__XO * t34 * t26;
    real_type t38  = t27 * t21;
    real_type t39  = t38 * t20;
    real_type t41  = 1.0 / t30 / t25;
    real_type t44  = 1.0 / t33 / t32;
    real_type t45  = alpha__r__XO * t44;
    real_type t48  = -t45 * t41 * t28 * t39 + t36 * t22;
    real_type t50  = t15 * t15;
    real_type t51  = t18 * t18;
    real_type t54  = alpha__r__XO * alpha__r__XO;
    real_type t56  = t54 / t51 * t50 + 1;
    real_type t57  = 1.0 / t56;
    real_type t58  = t14 * t7;
    real_type t63  = atan(alpha__r__XO * t34 * t26 * Fzr__XO * t22);
    real_type t64  = t63 * t58;
    real_type t65  = sin(t64);
    real_type t70  = ModelPars[50];
    real_type t71  = Fzr__XO * t70;
    real_type t73  = ModelPars[7];
    real_type t75  = ModelPars[36];
    real_type t78  = t73 * ModelPars[34] + (Fzr__XO - t73) * t75;
    real_type t79  = 1.0 / t78;
    real_type t81  = t13 * t13;
    real_type t82  = 1.0 / t81;
    real_type t83  = t82 * t7;
    real_type t84  = ModelPars[56];
    real_type t85  = t21 * t84;
    real_type t87  = t38 * t84;
    real_type t89  = t44 * t41;
    real_type t92  = -alpha__r__XO * t89 * t28 * t87 + t36 * t85;
    real_type t93  = t92 * t83;
    real_type t95  = t84 * t84;
    real_type t97  = t95 * t54 + 1;
    real_type t98  = 1.0 / t97;
    real_type t101 = atan(t36 * Fzr__XO * t85);
    real_type t102 = t101 * t58;
    real_type t103 = sin(t102);
    real_type t104 = t103 * t98;
    real_type t106 = ModelPars[47];
    real_type t109 = ModelPars[42] * t10 + 1;
    real_type t110 = 1.0 / t109;
    real_type t111 = t110 * t75;
    real_type t112 = 1.0 / t106;
    real_type t114 = alpha__r__XO * t34 * t112;
    real_type t116 = t110 * t78;
    real_type t117 = t44 * t112;
    real_type t121 = t31 * Fzr__XO * t27 * alpha__r__XO;
    real_type t123 = -t121 * t117 * t116 + t114 * t111;
    real_type t124 = t123 * t106;
    real_type t125 = t78 * t78;
    real_type t126 = t109 * t109;
    real_type t127 = 1.0 / t126;
    real_type t129 = t106 * t106;
    real_type t130 = 1.0 / t129;
    real_type t131 = t130 * t127 * t125;
    real_type t132 = 1.0 / t27;
    real_type t133 = 1.0 / t28;
    real_type t135 = t54 * t30;
    real_type t138 = t135 * t133 * t132 * t131 + 1;
    real_type t139 = 1.0 / t138;
    real_type t141 = atan(t114 * t116);
    real_type t142 = t141 * t106;
    real_type t143 = cos(t142);
    real_type t144 = t143 * t139;
    real_type t151 = t6 * t4 * Fzr__XO * t1;
    real_type t152 = t7 * t7;
    real_type t154 = t48 * t48;
    real_type t155 = t56 * t56;
    real_type t158 = cos(t64);
    real_type t162 = t28 * t70;
    real_type t163 = 1.0 / t125;
    real_type t164 = cos(t102);
    real_type t166 = t14 * t164 * t163;
    real_type t168 = sin(t142);
    real_type t169 = t168 * t34;
    real_type t181 = t27 * t27;
    real_type t182 = t181 * t21;
    real_type t184 = t28 * Fzr__XO;
    real_type t186 = t30 * t30;
    real_type t188 = 1.0 / t186 / t25;
    real_type t189 = t28 * t28;
    real_type t191 = 1.0 / t186;
    real_type t194 = 1.0 / t33 / t191 / t189 / t181;
    real_type t202 = t168 * t33 * t103;
    real_type t205 = t164 * t79;
    real_type t207 = t14 * t205 * t71;
    real_type t208 = t106 * t33;
    real_type t229 = t123 * t123;
    real_type t230 = t138 * t138;
    real_type t231 = 1.0 / t230;
    real_type t241 = t92 * t92;
    real_type t242 = t97 * t97;
    real_type t252 = t98 * t92;
    real_type t260 = t123 * t208;
    real_type t282 = t79 * t162;
    real_type t290 = t14 * t164;
    real_type t301 = t33 * t14;
    real_type t320 = t79 * t70;
    real_type t325 = t290 * t320;
    real_type t340 = t75 * t75;
    return -2 * t65 * t57 * t48 * t14 * t7 * t6 * t4 * t1 + 2 * t144 * t124 * t33 * t104 * t93 * t79 * t71 - t158 / t155 * t154 * t82 * t152 * t151 + 2 * t31 * t27 * t75 * t169 * t166 * t162 + t202 * t98 * (3 * alpha__r__XO * t194 * t188 * t184 * t182 * t84 - 3 * Fzr__XO * t45 * t41 * t87) * t82 * t7 * t79 * t71 - t143 * t139 * (3 * t191 * t28 * t181 * alpha__r__XO * t194 * t112 * t116 - t31 * t27 * t45 * t112 * t116 - 2 * t121 * t117 * t111) * t208 * t207 + t168 * t231 * t229 * t129 * t33 * t207 + t168 * t33 * t164 / t242 * t241 / t81 / t13 * t152 * t79 * t71 - 2 * t75 * t168 * t33 * t103 * t252 * t82 * t7 * t163 * t71 + 2 * t75 * t144 * t260 * t166 * t71 + (2 * t75 * t54 * t30 * t133 * t132 * t130 * t127 * t78 - 2 * t135 / t184 * t132 * t131) * t143 * t231 * t260 * t207 + 2 * t31 * t27 * t168 * t34 * t104 * t93 * t282 - 2 * t31 * t27 * t143 * t139 * t124 * t34 * t290 * t282 + 2 * t75 * t168 * t301 * t164 * t163 * t70 - t65 * t57 * (3 * alpha__r__XO * t194 * t188 * t184 * t182 * t20 - 3 * alpha__r__XO * Fzr__XO * t89 * t39) * t58 * t151 + 2 * t202 * t252 * t83 * t320 - 3 * t31 * Fzr__XO * t27 * t169 * t325 - 2 * t143 * t139 * t123 * t208 * t325 - 2 * t340 * t168 * t301 * t164 / t125 / t78 * t71 + t191 * t181 * t168 * t44 * t14 * t205 * t184 * t70;
  }

  real_type
  General::Mzr_D_1_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[50];
    real_type t2   = Fzr__XO * Fzr__XO;
    real_type t3   = t2 * Fzr__XO;
    real_type t4   = t3 * t1;
    real_type t6   = ModelPars[7];
    real_type t8   = ModelPars[36];
    real_type t11  = t6 * ModelPars[34] + (Fzr__XO - t6) * t8;
    real_type t12  = 1.0 / t11;
    real_type t13  = t12 * t4;
    real_type t14  = ModelPars[48];
    real_type t15  = phi__XO * phi__XO;
    real_type t16  = ModelPars[54];
    real_type t18  = t16 * t15 + 1;
    real_type t19  = t18 * t18;
    real_type t20  = 1.0 / t19;
    real_type t21  = t20 * t14;
    real_type t22  = ModelPars[56];
    real_type t23  = ModelPars[40];
    real_type t24  = t23 * t22;
    real_type t25  = ModelPars[46];
    real_type t27  = t25 * t15 + 1;
    real_type t28  = 1.0 / t27;
    real_type t29  = t23 * t23;
    real_type t30  = t2 * t29;
    real_type t31  = t27 * t27;
    real_type t32  = 1.0 / t31;
    real_type t33  = t32 * t30;
    real_type t34  = sqrt(t33);
    real_type t35  = 1.0 / t34;
    real_type t36  = t35 * t28;
    real_type t37  = alpha__r__XO * t36;
    real_type t39  = t29 * t23;
    real_type t40  = t39 * t22;
    real_type t43  = 1.0 / t31 / t27;
    real_type t45  = 1.0 / t34 / t33;
    real_type t46  = t45 * t43;
    real_type t49  = -alpha__r__XO * t46 * t2 * t40 + t37 * t24;
    real_type t50  = alpha__r__XO * alpha__r__XO;
    real_type t51  = t22 * t22;
    real_type t54  = 1.0 / (t51 * t50 + 1);
    real_type t55  = t54 * t49;
    real_type t58  = 1.0 / t18;
    real_type t59  = t58 * t14;
    real_type t62  = atan(t37 * Fzr__XO * t24);
    real_type t63  = t62 * t59;
    real_type t64  = sin(t63);
    real_type t66  = ModelPars[47];
    real_type t67  = ModelPars[42];
    real_type t69  = t67 * t15 + 1;
    real_type t70  = 1.0 / t69;
    real_type t71  = t70 * t11;
    real_type t72  = 1.0 / t66;
    real_type t74  = alpha__r__XO * t35 * t72;
    real_type t76  = atan(t74 * t71);
    real_type t77  = t76 * t66;
    real_type t78  = sin(t77);
    real_type t81  = phi__XO * t25;
    real_type t82  = t81 * t43 * t29;
    real_type t86  = cos(t63);
    real_type t87  = t58 * t86;
    real_type t91  = t70 * t8;
    real_type t93  = t45 * t72;
    real_type t96  = t32 * Fzr__XO;
    real_type t99  = -t96 * t29 * alpha__r__XO * t93 * t71 + t74 * t91;
    real_type t100 = t11 * t11;
    real_type t101 = t69 * t69;
    real_type t102 = 1.0 / t101;
    real_type t103 = t102 * t100;
    real_type t104 = t66 * t66;
    real_type t105 = 1.0 / t104;
    real_type t107 = 1.0 / t29;
    real_type t108 = 1.0 / t2;
    real_type t113 = t50 * t31 * t108 * t107 * t105 * t103 + 1;
    real_type t114 = 1.0 / t113;
    real_type t116 = cos(t77);
    real_type t117 = t116 * t114 * t99;
    real_type t121 = ModelPars[52];
    real_type t122 = Fzr__XO * t121;
    real_type t123 = ModelPars[55];
    real_type t125 = atan(phi__XO * t123);
    real_type t127 = 1.0 / t123;
    real_type t129 = ModelPars[57];
    real_type t130 = ModelPars[53];
    real_type t132 = t130 * t15 + 1;
    real_type t134 = 1.0 / t132 * t129;
    real_type t135 = t23 * t134;
    real_type t139 = alpha__r__XO * t45;
    real_type t142 = -t139 * t43 * t2 * t39 * t134 + t37 * t135;
    real_type t143 = t142 * t58;
    real_type t146 = t129 * t129;
    real_type t147 = t132 * t132;
    real_type t148 = 1.0 / t147;
    real_type t151 = t50 * t148 * t146 + 1;
    real_type t152 = t151 * t151;
    real_type t155 = alpha__r__XO * t35;
    real_type t158 = atan(t155 * t28 * Fzr__XO * t135);
    real_type t159 = t158 * t59;
    real_type t160 = sin(t159);
    real_type t166 = phi__XO * t130;
    real_type t171 = Fzr__XO * t1;
    real_type t177 = t81 * t155;
    real_type t179 = t31 * t31;
    real_type t180 = 1.0 / t179;
    real_type t189 = -2 * phi__XO * t16 * t62 * t21 + t54 * (2 * t81 * t139 * t180 * t3 * t40 - 2 * t177 * t96 * t24) * t59;
    real_type t191 = t64 * t189 * t12;
    real_type t197 = t12 * t86;
    real_type t198 = t58 * t197;
    real_type t199 = t198 * t171;
    real_type t202 = t113 * t113;
    real_type t203 = 1.0 / t202;
    real_type t204 = t102 * t11;
    real_type t207 = phi__XO * t67 * t155;
    real_type t209 = alpha__r__XO * t93;
    real_type t210 = t209 * t71;
    real_type t212 = phi__XO * t25 * t43;
    real_type t213 = t212 * t30;
    real_type t216 = -2 * t207 * t72 * t204 + 2 * t213 * t210;
    real_type t221 = t12 * t1;
    real_type t222 = t35 * t87;
    real_type t224 = t29 * t78;
    real_type t229 = t2 * t1;
    real_type t239 = t2 * t2;
    real_type t243 = t29 * t29;
    real_type t248 = phi__XO * t25 / t179 / t27;
    real_type t252 = 1.0 / t100;
    real_type t253 = t86 * t252;
    real_type t256 = t66 * t34;
    real_type t258 = t114 * t116;
    real_type t264 = t20 * t14 * t12 * t171;
    real_type t270 = t127 * t125;
    real_type t272 = t14 * t270 * t122;
    real_type t274 = 1.0 / t151;
    real_type t286 = t107 * t105;
    real_type t306 = t216 * t66;
    real_type t312 = t12 * t171;
    real_type t313 = t20 * t86;
    real_type t330 = -2 * t82 * t78 * t35 * t64 * t55 * t21 * t13 + 2 * t82 * t117 * t66 * t35 * t87 * t13 - 4 * t166 * t50 / t147 / t132 * t146 * t160 / t152 * t143 * t14 * t127 * t125 * t122 + t117 * t66 * t34 * t58 * t191 * t171 + t78 * t216 * t203 * t99 * t104 * t34 * t199 + 6 * t212 * t2 * t224 * t222 * t221 + 2 * phi__XO * t16 * t32 * t29 * t78 * t35 * t20 * t197 * t229 - 2 * t248 * t243 * t78 * t45 * t198 * t239 * t1 + t8 * t258 * t216 * t256 * t58 * t253 * t171 + t78 * t34 * t86 * t189 * t55 * t264 + 2 * phi__XO * t16 * t160 * t274 * t142 * t20 * t272 + (-4 * phi__XO * t67 * t50 * t31 * t108 * t286 / t101 / t69 * t100 + 4 * phi__XO * t25 * t50 * t27 * t108 * t286 * t103) * t116 * t203 * t99 * t256 * t199 - t32 * t29 * t116 * t114 * t306 * t222 * t12 * t229 + 2 * phi__XO * t16 * t116 * t114 * t99 * t66 * t34 * t313 * t312 - 2 * t212 * t29 * t8 * t78 * t222 * t252 * t4;
    real_type t337 = t34 * t64 * t54;
    real_type t356 = Fzr__XO * t29;
    real_type t365 = 1.0 / t34 / t180 / t239 / t243;
    real_type t385 = t148 * t129;
    real_type t386 = Fzr__XO * t23;
    real_type t389 = phi__XO * t130 * alpha__r__XO;
    real_type t395 = phi__XO * t25 * alpha__r__XO;
    real_type t400 = t45 * t180;
    real_type t407 = -2 * phi__XO * t16 * t158 * t21 + t274 * (2 * t395 * t400 * t3 * t39 * t134 - 2 * t395 * t35 * t32 * t386 * t134 - 2 * t389 * t36 * t386 * t385) * t59;
    real_type t409 = cos(t159);
    real_type t415 = t78 * t34;
    real_type t429 = t243 * t23;
    real_type t432 = 1.0 / t179 / t31;
    real_type t459 = t123 * t123;
    real_type t462 = 1.0 / (t459 * t15 + 1);
    real_type t469 = t58 * t64;
    real_type t517 = -4 * phi__XO * t16 * t78 * t337 * t49 / t19 / t18 * t14 * t312 + t258 * t306 * t337 * t49 * t21 * t312 - t116 * t114 * (-6 * t248 * t3 * t243 * alpha__r__XO * t365 * t72 * t71 + 2 * phi__XO * t67 * t32 * t356 * t209 * t204 - 2 * t207 * t72 * t102 * t8 + 2 * t213 * t209 * t91 + 4 * t212 * t356 * t210) * t256 * t199 - t409 * t407 * t274 * t143 * t272 - 2 * phi__XO * t16 * t8 * t415 * t20 * t253 * t171 + t78 * t34 * t64 * t54 * (-6 * t81 * alpha__r__XO * t365 * t432 * t239 * t429 * t22 + 8 * t81 * t2 * alpha__r__XO * t400 * t40 - 2 * t177 * t32 * t24) * t264 + t32 * t224 * t35 * t58 * t191 * t229 - t160 * t407 * t127 * t125 * t121 + 2 * phi__XO * t16 * t415 * t313 * t221 - t160 * t274 * t143 * t14 * t462 * t122 + t415 * t469 * t189 * t221 - t116 * t114 * t216 * t256 * t87 * t221 - t8 * t415 * t469 * t189 * t252 * t171 - t160 * t274 * (8 * phi__XO * t25 * t2 * t139 * t180 * t39 * t134 - 6 * t395 * t365 * t432 * t239 * t429 * t134 - 2 * t166 * t155 * t28 * t23 * t385 + 2 * t389 * t46 * t2 * t39 * t385 - 2 * t177 * t32 * t23 * t134) * t59 * t270 * t122 + t409 * t462 * t121;
    return t330 + t517;
  }

  real_type
  General::Mzr_D_1_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[50];
    real_type t3   = ModelPars[7];
    real_type t5   = ModelPars[36];
    real_type t8   = t3 * ModelPars[34] + (Fzr__XO - t3) * t5;
    real_type t9   = 1.0 / t8;
    real_type t11  = ModelPars[48];
    real_type t12  = phi__XO * phi__XO;
    real_type t15  = ModelPars[54] * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t17  = 1.0 / t16;
    real_type t18  = t17 * t11;
    real_type t19  = ModelPars[56];
    real_type t20  = t19 * t18;
    real_type t22  = ModelPars[40];
    real_type t26  = ModelPars[46] * t12 + 1;
    real_type t27  = 1.0 / t26;
    real_type t29  = alpha__r__XO * alpha__r__XO;
    real_type t30  = t19 * t19;
    real_type t32  = t30 * t29 + 1;
    real_type t33  = 1.0 / t32;
    real_type t34  = 1.0 / t15;
    real_type t35  = t34 * t11;
    real_type t36  = t22 * t19;
    real_type t38  = t22 * t22;
    real_type t39  = Fzr__XO * Fzr__XO;
    real_type t41  = t26 * t26;
    real_type t42  = 1.0 / t41;
    real_type t43  = t42 * t39 * t38;
    real_type t44  = sqrt(t43);
    real_type t45  = 1.0 / t44;
    real_type t46  = t45 * t27;
    real_type t47  = alpha__r__XO * t46;
    real_type t49  = atan(t47 * Fzr__XO * t36);
    real_type t50  = t49 * t35;
    real_type t51  = sin(t50);
    real_type t52  = t51 * t33;
    real_type t53  = ModelPars[47];
    real_type t56  = ModelPars[42] * t12 + 1;
    real_type t57  = 1.0 / t56;
    real_type t58  = t57 * t8;
    real_type t59  = 1.0 / t53;
    real_type t60  = t45 * t59;
    real_type t61  = alpha__r__XO * t60;
    real_type t63  = atan(t61 * t58);
    real_type t64  = t63 * t53;
    real_type t65  = sin(t64);
    real_type t70  = cos(t50);
    real_type t73  = t8 * t8;
    real_type t74  = t56 * t56;
    real_type t75  = 1.0 / t74;
    real_type t77  = t53 * t53;
    real_type t80  = 1.0 / t38;
    real_type t86  = t29 * t41 / t39 * t80 / t77 * t75 * t73 + 1;
    real_type t87  = 1.0 / t86;
    real_type t89  = cos(t64);
    real_type t90  = t89 * t87 * t57;
    real_type t93  = t39 * t1;
    real_type t97  = t27 * t22;
    real_type t103 = Fzr__XO * t1;
    real_type t104 = t70 * t9;
    real_type t107 = t89 * t87;
    real_type t115 = t38 * t22;
    real_type t116 = t115 * t19;
    real_type t118 = 1.0 / t41 / t26;
    real_type t119 = t118 * t39;
    real_type t121 = 1.0 / t44 / t43;
    real_type t122 = t121 * t119;
    real_type t136 = -alpha__r__XO * t121 * t118 * t39 * t116 + t47 * t36;
    real_type t137 = t136 * t18;
    real_type t139 = t32 * t32;
    real_type t140 = 1.0 / t139;
    real_type t148 = t9 * t93;
    real_type t149 = t11 * t11;
    real_type t168 = t57 * t5;
    real_type t173 = t42 * Fzr__XO;
    real_type t176 = -t173 * t38 * alpha__r__XO * t121 * t59 * t58 + t61 * t168;
    real_type t177 = t176 * t53;
    real_type t197 = t34 * t70;
    real_type t201 = t86 * t86;
    real_type t202 = 1.0 / t201;
    real_type t216 = ModelPars[52];
    real_type t217 = ModelPars[55];
    real_type t219 = atan(phi__XO * t217);
    real_type t221 = 1.0 / t217;
    real_type t223 = ModelPars[57];
    real_type t228 = ModelPars[53] * t12 + 1;
    real_type t229 = 1.0 / t228;
    real_type t232 = t223 * t223;
    real_type t233 = t228 * t228;
    real_type t235 = 1.0 / t233 * t232;
    real_type t237 = t29 * t235 + 1;
    real_type t238 = 1.0 / t237;
    real_type t239 = t229 * t223;
    real_type t240 = t22 * t239;
    real_type t245 = atan(alpha__r__XO * t45 * t27 * Fzr__XO * t240);
    real_type t246 = t245 * t35;
    real_type t247 = sin(t246);
    real_type t252 = Fzr__XO * t216;
    real_type t257 = t115 * t239;
    real_type t272 = -alpha__r__XO * t121 * t119 * t257 + t47 * t240;
    real_type t273 = t237 * t237;
    real_type t274 = 1.0 / t273;
    real_type t289 = cos(t246);
    return 2 * t65 * t52 * t27 * Fzr__XO * t22 * t20 * t9 * t1 - 2 * t90 * t34 * t70 * t1 - t5 * t65 * t51 * t33 * t97 * t20 / t73 * t93 + t5 * t107 * t57 * t34 * t104 * t103 + t65 * t44 * t51 * t33 * (-t122 * t116 + t46 * t36) * t17 * t11 * t9 * t103 - 2 * alpha__r__XO * t30 * t65 * t44 * t51 * t140 * t137 * t9 * t103 + t65 * t70 * t27 * t22 * t19 * t140 * t136 / t16 / t15 * t149 * t148 + t90 * t52 * t137 * t103 + t107 * t177 * t51 * t33 * t27 * t36 * t18 * t148 - t89 * t87 * (-t173 * t38 * t121 * t59 * t58 + t60 * t168) * t53 * t44 * t34 * t104 * t103 + 2 * alpha__r__XO * t41 * t80 * t75 * t89 * t202 * t176 * t59 * t44 * t197 * t8 / Fzr__XO * t1 + t65 * t57 * t202 * t177 * t197 * t103 - t247 * t238 * t46 * Fzr__XO * t22 * t229 * t223 * t35 * t221 * t219 * t216 - t247 * t238 * (t45 * t97 * t239 - t122 * t257) * t35 * t221 * t219 * t252 + 2 * alpha__r__XO * t235 * t247 * t274 * t272 * t34 * t11 * t221 * t219 * t252 - t289 * t45 * t97 * t229 * t223 * t274 * t272 * t17 * t149 * t221 * t219 * t39 * t216;
  }

  real_type
  General::Mzr_D_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[50];
    real_type t2   = Fzr__XO * t1;
    real_type t4   = ModelPars[7];
    real_type t9   = t4 * ModelPars[34] + (Fzr__XO - t4) * ModelPars[36];
    real_type t10  = 1.0 / t9;
    real_type t11  = ModelPars[48];
    real_type t12  = phi__XO * phi__XO;
    real_type t13  = ModelPars[54];
    real_type t15  = t13 * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t17  = 1.0 / t16;
    real_type t18  = t17 * t11;
    real_type t19  = ModelPars[56];
    real_type t20  = ModelPars[40];
    real_type t21  = t20 * t19;
    real_type t23  = ModelPars[46];
    real_type t25  = t23 * t12 + 1;
    real_type t26  = 1.0 / t25;
    real_type t27  = t20 * t20;
    real_type t28  = Fzr__XO * Fzr__XO;
    real_type t29  = t28 * t27;
    real_type t30  = t25 * t25;
    real_type t31  = 1.0 / t30;
    real_type t32  = t31 * t29;
    real_type t33  = sqrt(t32);
    real_type t34  = 1.0 / t33;
    real_type t35  = t34 * t26;
    real_type t38  = atan(alpha__r__XO * t35 * Fzr__XO * t21);
    real_type t43  = 1.0 / t15;
    real_type t44  = t43 * t11;
    real_type t47  = alpha__r__XO * t34;
    real_type t48  = phi__XO * t23;
    real_type t51  = t27 * t20;
    real_type t53  = t28 * Fzr__XO;
    real_type t54  = t30 * t30;
    real_type t55  = 1.0 / t54;
    real_type t59  = 1.0 / t33 / t32;
    real_type t65  = alpha__r__XO * alpha__r__XO;
    real_type t66  = t19 * t19;
    real_type t75  = t38 * t44;
    real_type t76  = sin(t75);
    real_type t78  = ModelPars[47];
    real_type t79  = ModelPars[42];
    real_type t81  = t79 * t12 + 1;
    real_type t83  = 1.0 / t81 * t9;
    real_type t84  = 1.0 / t78;
    real_type t88  = atan(alpha__r__XO * t34 * t84 * t83);
    real_type t89  = t88 * t78;
    real_type t90  = sin(t89);
    real_type t94  = cos(t75);
    real_type t95  = t94 * t10;
    real_type t104 = t43 * t95;
    real_type t111 = phi__XO * t23 / t30 / t25;
    real_type t117 = t81 * t81;
    real_type t118 = 1.0 / t117;
    real_type t131 = t9 * t9;
    real_type t133 = t78 * t78;
    real_type t145 = cos(t89);
    real_type t150 = ModelPars[52] * Fzr__XO;
    real_type t151 = ModelPars[55];
    real_type t152 = t151 * t151;
    real_type t156 = ModelPars[57];
    real_type t157 = ModelPars[53];
    real_type t159 = t157 * t12 + 1;
    real_type t161 = 1.0 / t159 * t156;
    real_type t166 = atan(t47 * t26 * Fzr__XO * t20 * t161);
    real_type t167 = t166 * t44;
    real_type t168 = cos(t167);
    real_type t172 = atan(phi__XO * t151);
    real_type t179 = t159 * t159;
    real_type t180 = 1.0 / t179;
    real_type t182 = Fzr__XO * t20;
    real_type t191 = phi__XO * t23 * alpha__r__XO;
    real_type t201 = t156 * t156;
    real_type t210 = sin(t167);
    return t90 * t33 * t43 * t76 * (-2 * phi__XO * t13 * t38 * t18 + 1.0 / (t66 * t65 + 1) * (2 * t48 * alpha__r__XO * t59 * t55 * t53 * t51 * t19 - 2 * t48 * t47 * t31 * Fzr__XO * t21) * t44) * t10 * t2 + 2 * phi__XO * t13 * t90 * t33 * t17 * t95 * t2 + 2 * t111 * t27 * t90 * t34 * t104 * t53 * t1 - t145 / (t65 * t30 / t28 / t27 / t133 * t118 * t131 + 1) * (-2 * phi__XO * t79 * t47 * t84 * t118 * t9 + 2 * t111 * t29 * alpha__r__XO * t59 * t84 * t83) * t78 * t33 * t104 * t2 + t168 / (t152 * t12 + 1) * t150 - t210 * (-2 * phi__XO * t13 * t166 * t18 + 1.0 / (t65 * t180 * t201 + 1) * (-2 * phi__XO * t157 * alpha__r__XO * t35 * t182 * t180 * t156 + 2 * t191 * t59 * t55 * t53 * t51 * t161 - 2 * t191 * t34 * t31 * t182 * t161) * t44) / t151 * t172 * t150;
  }

  real_type
  General::Mzr_D_2_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[50];
    real_type t2   = Fzr__XO * t1;
    real_type t4   = ModelPars[7];
    real_type t9   = t4 * ModelPars[34] + (Fzr__XO - t4) * ModelPars[36];
    real_type t10  = 1.0 / t9;
    real_type t11  = ModelPars[48];
    real_type t12  = phi__XO * phi__XO;
    real_type t13  = ModelPars[54];
    real_type t15  = t13 * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t18  = 1.0 / t16 / t15;
    real_type t19  = t18 * t11;
    real_type t20  = ModelPars[56];
    real_type t21  = ModelPars[40];
    real_type t22  = t21 * t20;
    real_type t23  = Fzr__XO * t22;
    real_type t24  = ModelPars[46];
    real_type t26  = t24 * t12 + 1;
    real_type t27  = 1.0 / t26;
    real_type t28  = t21 * t21;
    real_type t29  = Fzr__XO * Fzr__XO;
    real_type t30  = t29 * t28;
    real_type t31  = t26 * t26;
    real_type t32  = 1.0 / t31;
    real_type t33  = t32 * t30;
    real_type t34  = sqrt(t33);
    real_type t35  = 1.0 / t34;
    real_type t36  = t35 * t27;
    real_type t39  = atan(alpha__r__XO * t36 * t23);
    real_type t40  = t13 * t13;
    real_type t45  = 1.0 / t16;
    real_type t46  = t45 * t11;
    real_type t49  = alpha__r__XO * t35;
    real_type t50  = phi__XO * t24;
    real_type t53  = t28 * t21;
    real_type t54  = t53 * t20;
    real_type t55  = t29 * Fzr__XO;
    real_type t56  = t31 * t31;
    real_type t57  = 1.0 / t56;
    real_type t61  = 1.0 / t34 / t33;
    real_type t62  = alpha__r__XO * t61;
    real_type t66  = -2 * t50 * t49 * t32 * Fzr__XO * t22 + 2 * t50 * t62 * t57 * t55 * t54;
    real_type t68  = alpha__r__XO * alpha__r__XO;
    real_type t69  = t20 * t20;
    real_type t72  = 1.0 / (t69 * t68 + 1);
    real_type t77  = t13 * t39;
    real_type t80  = 1.0 / t15;
    real_type t81  = t80 * t11;
    real_type t82  = t31 * t26;
    real_type t83  = 1.0 / t82;
    real_type t86  = t24 * t24;
    real_type t87  = t12 * t86;
    real_type t92  = 1.0 / t56 / t26;
    real_type t98  = t35 * t32;
    real_type t99  = t24 * alpha__r__XO;
    real_type t100 = t99 * t98;
    real_type t103 = t28 * t28;
    real_type t104 = t103 * t21;
    real_type t106 = t29 * t29;
    real_type t107 = t106 * Fzr__XO;
    real_type t109 = 1.0 / t56 / t82;
    real_type t112 = t106 * t103;
    real_type t115 = 1.0 / t34 / t57 / t112;
    real_type t121 = t61 * t57;
    real_type t122 = t99 * t121;
    real_type t131 = t39 * t81;
    real_type t132 = sin(t131);
    real_type t134 = ModelPars[47];
    real_type t135 = ModelPars[42];
    real_type t136 = t135 * t12;
    real_type t137 = t136 + 1;
    real_type t139 = 1.0 / t137 * t9;
    real_type t140 = 1.0 / t134;
    real_type t144 = atan(alpha__r__XO * t35 * t140 * t139);
    real_type t145 = t144 * t134;
    real_type t146 = sin(t145);
    real_type t147 = t146 * t34;
    real_type t150 = t55 * t1;
    real_type t151 = t10 * t150;
    real_type t152 = cos(t131);
    real_type t153 = t80 * t152;
    real_type t157 = t137 * t137;
    real_type t158 = 1.0 / t157;
    real_type t159 = t158 * t9;
    real_type t160 = t140 * t159;
    real_type t164 = t61 * t140;
    real_type t165 = alpha__r__XO * t164;
    real_type t166 = t165 * t139;
    real_type t168 = phi__XO * t24 * t83;
    real_type t172 = -2 * phi__XO * t135 * t49 * t160 + 2 * t168 * t30 * t166;
    real_type t173 = t9 * t9;
    real_type t174 = t158 * t173;
    real_type t175 = t134 * t134;
    real_type t176 = 1.0 / t175;
    real_type t178 = 1.0 / t28;
    real_type t179 = 1.0 / t29;
    real_type t184 = t68 * t31 * t179 * t178 * t176 * t174 + 1;
    real_type t185 = 1.0 / t184;
    real_type t187 = cos(t145);
    real_type t188 = t187 * t185 * t172;
    real_type t189 = t83 * t28;
    real_type t195 = ModelPars[52] * Fzr__XO;
    real_type t196 = ModelPars[55];
    real_type t198 = atan(phi__XO * t196);
    real_type t199 = t198 * t195;
    real_type t200 = 1.0 / t196;
    real_type t201 = ModelPars[57];
    real_type t202 = ModelPars[53];
    real_type t203 = t202 * t12;
    real_type t204 = t203 + 1;
    real_type t206 = 1.0 / t204 * t201;
    real_type t211 = atan(t49 * t27 * Fzr__XO * t21 * t206);
    real_type t212 = t13 * t211;
    real_type t216 = t204 * t204;
    real_type t217 = 1.0 / t216;
    real_type t218 = t217 * t201;
    real_type t219 = Fzr__XO * t21;
    real_type t220 = t219 * t218;
    real_type t221 = t202 * alpha__r__XO;
    real_type t225 = t219 * t206;
    real_type t226 = phi__XO * t99;
    real_type t229 = t55 * t53;
    real_type t230 = t229 * t206;
    real_type t234 = -2 * phi__XO * t221 * t36 * t220 + 2 * t226 * t121 * t230 - 2 * t226 * t98 * t225;
    real_type t235 = t201 * t201;
    real_type t238 = t68 * t217 * t235 + 1;
    real_type t239 = 1.0 / t238;
    real_type t242 = -2 * phi__XO * t212 * t46 + t239 * t234 * t81;
    real_type t243 = t242 * t242;
    real_type t245 = t211 * t81;
    real_type t246 = cos(t245);
    real_type t249 = t196 * t196;
    real_type t251 = t249 * t12 + 1;
    real_type t254 = sin(t245);
    real_type t270 = 1.0 / t216 / t204;
    real_type t273 = t202 * t202;
    real_type t281 = t24 * t203;
    real_type t295 = t12 * t86 * alpha__r__XO;
    real_type t316 = t238 * t238;
    real_type t330 = t251 * t251;
    real_type t337 = t152 * t10;
    real_type t338 = t80 * t337;
    real_type t339 = t338 * t2;
    real_type t341 = t172 * t172;
    real_type t342 = t184 * t184;
    real_type t343 = 1.0 / t342;
    real_type t348 = t134 * t34;
    real_type t350 = 1.0 / t157 / t137;
    real_type t353 = t135 * t135;
    real_type t361 = t24 * t83 * t29;
    real_type t374 = t12 * t86 / t56 / t31;
    real_type t379 = t12 * t86 * t57;
    real_type t398 = -2 * phi__XO * t77 * t46 + t72 * t66 * t81;
    real_type t401 = t132 * t398 * t10 * t2;
    real_type t402 = t34 * t45;
    real_type t403 = t13 * t146;
    real_type t416 = t146 * t35;
    real_type t417 = t28 * t416;
    real_type t421 = t45 * t152;
    real_type t425 = t24 * t189;
    real_type t439 = t338 * t150;
    real_type t456 = t178 * t176;
    real_type t474 = t398 * t398;
    real_type t479 = t337 * t2;
    return t147 * t80 * t132 * (8 * t12 * t40 * t39 * t19 - 4 * phi__XO * t13 * t72 * t66 * t46 - 2 * t77 * t46 + t72 * (12 * t87 * alpha__r__XO * t115 * t109 * t107 * t104 * t20 + 8 * t87 * t49 * t83 * Fzr__XO * t22 - 20 * t87 * t62 * t92 * t55 * t54 + 2 * t122 * t55 * t54 - 2 * t100 * t23) * t81) * t10 * t2 + 4 * t50 * t189 * t188 * t134 * t35 * t153 * t151 - t246 * t243 * t200 * t199 - 2 * t254 * t242 / t251 * t195 - t254 * (8 * t12 * t40 * t211 * t19 - 4 * phi__XO * t13 * t239 * t234 * t46 - 2 * t212 * t46 + t239 * (8 * t12 * t273 * alpha__r__XO * t36 * t219 * t270 * t201 + 12 * t295 * t115 * t109 * t107 * t104 * t206 + 8 * t281 * t49 * t32 * t219 * t218 - 8 * t281 * t62 * t57 * t229 * t218 + 8 * t295 * t35 * t83 * t225 - 20 * t295 * t61 * t92 * t230 - 2 * t221 * t36 * t220 - 2 * t100 * t225 + 2 * t122 * t230) * t81 + 4 * phi__XO * t202 * t68 * t270 * t235 / t316 * t234 * t81) * t200 * t199 - 2 * phi__XO * t249 * t246 / t330 * t195 + t146 * t343 * t341 * t175 * t34 * t339 - t187 * t185 * (12 * t374 * t112 * alpha__r__XO * t115 * t140 * t139 + 8 * t12 * t353 * t49 * t140 * t350 * t9 - 8 * t361 * t28 * t136 * t165 * t159 + 2 * t361 * t28 * alpha__r__XO * t164 * t139 - 2 * t135 * t49 * t160 - 12 * t379 * t30 * t166) * t348 * t339 - 4 * phi__XO * t403 * t402 * t401 + 2 * t188 * t134 * t34 * t80 * t401 - 4 * t168 * t417 * t80 * t132 * t398 * t151 - 8 * t425 * t12 * t403 * t35 * t421 * t151 + 4 * phi__XO * t13 * t187 * t185 * t172 * t134 * t34 * t421 * t10 * t2 + 2 * t425 * t416 * t439 + 4 * t374 * t103 * t146 * t61 * t338 * t107 * t1 - 12 * t379 * t417 * t439 + (-4 * phi__XO * t135 * t68 * t31 * t179 * t456 * t350 * t173 + 4 * phi__XO * t24 * t68 * t26 * t179 * t456 * t174) * t187 * t343 * t172 * t348 * t339 + t147 * t153 * t474 * t10 * t2 + 2 * t403 * t402 * t479 - 8 * t12 * t40 * t146 * t34 * t18 * t479;
  }

  real_type
  General::Mzr_D_2_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[50];
    real_type t2   = Fzr__XO * t1;
    real_type t4   = ModelPars[7];
    real_type t9   = t4 * ModelPars[34] + (Fzr__XO - t4) * ModelPars[36];
    real_type t10  = 1.0 / t9;
    real_type t11  = ModelPars[48];
    real_type t12  = phi__XO * phi__XO;
    real_type t13  = ModelPars[54];
    real_type t15  = t13 * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t17  = 1.0 / t16;
    real_type t18  = t17 * t11;
    real_type t19  = ModelPars[56];
    real_type t20  = ModelPars[40];
    real_type t21  = t20 * t19;
    real_type t22  = Fzr__XO * t21;
    real_type t24  = ModelPars[46];
    real_type t26  = t24 * t12 + 1;
    real_type t27  = 1.0 / t26;
    real_type t28  = t20 * t20;
    real_type t29  = Fzr__XO * Fzr__XO;
    real_type t30  = t29 * t28;
    real_type t31  = t26 * t26;
    real_type t32  = 1.0 / t31;
    real_type t33  = t32 * t30;
    real_type t34  = sqrt(t33);
    real_type t35  = 1.0 / t34;
    real_type t36  = t35 * t27;
    real_type t37  = alpha__r__XO * alpha__r__XO;
    real_type t38  = t19 * t19;
    real_type t40  = t38 * t37 + 1;
    real_type t41  = 1.0 / t40;
    real_type t47  = 1.0 / t15;
    real_type t48  = t47 * t11;
    real_type t49  = t35 * t32;
    real_type t50  = phi__XO * t24;
    real_type t51  = t50 * t49;
    real_type t53  = t28 * t20;
    real_type t54  = t53 * t19;
    real_type t55  = t29 * Fzr__XO;
    real_type t57  = t31 * t31;
    real_type t58  = 1.0 / t57;
    real_type t60  = 1.0 / t34 / t33;
    real_type t61  = t60 * t58;
    real_type t62  = t50 * t61;
    real_type t70  = alpha__r__XO * t35;
    real_type t79  = 2 * t50 * alpha__r__XO * t60 * t58 * t55 * t54 - 2 * t50 * t70 * t32 * Fzr__XO * t21;
    real_type t81  = t40 * t40;
    real_type t92  = atan(alpha__r__XO * t36 * t22);
    real_type t93  = t92 * t48;
    real_type t94  = sin(t93);
    real_type t96  = ModelPars[47];
    real_type t97  = ModelPars[42];
    real_type t99  = t97 * t12 + 1;
    real_type t100 = 1.0 / t99;
    real_type t101 = t100 * t9;
    real_type t102 = 1.0 / t96;
    real_type t106 = atan(alpha__r__XO * t35 * t102 * t101);
    real_type t107 = t106 * t96;
    real_type t108 = sin(t107);
    real_type t113 = t10 * t29 * t1;
    real_type t120 = -2 * phi__XO * t13 * t92 * t18 + t41 * t79 * t48;
    real_type t125 = cos(t93);
    real_type t133 = t9 * t9;
    real_type t134 = t99 * t99;
    real_type t135 = 1.0 / t134;
    real_type t137 = t96 * t96;
    real_type t140 = 1.0 / t28;
    real_type t146 = t37 * t31 / t29 * t140 / t137 * t135 * t133 + 1;
    real_type t147 = 1.0 / t146;
    real_type t148 = cos(t107);
    real_type t149 = t148 * t147;
    real_type t159 = t108 * t94;
    real_type t167 = t147 * t100;
    real_type t181 = t47 * t125;
    real_type t194 = t102 * t135 * t9;
    real_type t198 = t60 * t102;
    real_type t205 = phi__XO * t24 / t31 / t26 * t30;
    real_type t208 = -2 * phi__XO * t97 * t70 * t194 + 2 * t205 * alpha__r__XO * t198 * t101;
    real_type t209 = t208 * t96;
    real_type t234 = t146 * t146;
    real_type t235 = 1.0 / t234;
    real_type t249 = ModelPars[52];
    real_type t250 = t29 * t249;
    real_type t251 = ModelPars[55];
    real_type t252 = t251 * t251;
    real_type t257 = ModelPars[57];
    real_type t260 = ModelPars[53];
    real_type t262 = t260 * t12 + 1;
    real_type t263 = 1.0 / t262;
    real_type t266 = t257 * t257;
    real_type t267 = t262 * t262;
    real_type t268 = 1.0 / t267;
    real_type t271 = t37 * t268 * t266 + 1;
    real_type t272 = 1.0 / t271;
    real_type t274 = t263 * t257;
    real_type t275 = t20 * t274;
    real_type t276 = t27 * Fzr__XO;
    real_type t279 = atan(t70 * t276 * t275);
    real_type t280 = t279 * t48;
    real_type t281 = sin(t280);
    real_type t287 = atan(phi__XO * t251);
    real_type t289 = 1.0 / t251;
    real_type t298 = Fzr__XO * t20;
    real_type t299 = t298 * t268 * t257;
    real_type t303 = t298 * t274;
    real_type t306 = t55 * t53 * t274;
    real_type t317 = phi__XO * t24 * alpha__r__XO;
    real_type t323 = -2 * phi__XO * t260 * alpha__r__XO * t36 * t299 - 2 * t317 * t49 * t303 + 2 * t317 * t61 * t306;
    real_type t325 = t271 * t271;
    real_type t347 = cos(t280);
    return t108 * t34 * t47 * t94 * (-2 * phi__XO * t13 * t41 * t36 * t22 * t18 + t41 * (2 * t62 * t55 * t54 - 2 * t51 * t22) * t48 - 2 * alpha__r__XO * t38 / t81 * t79 * t48) * t10 * t2 + t108 * t125 * t41 * t27 * t21 * t17 * t11 * t120 * t113 + t149 * t100 * t47 * t94 * t120 * t2 - 2 * phi__XO * t13 * t159 * t41 * t27 * t20 * t19 / t16 / t15 * t11 * t113 + 2 * phi__XO * t13 * t148 * t167 * t17 * t125 * t2 - 2 * t50 * t159 * t41 * t32 * t20 * t19 * t18 * t113 + 2 * phi__XO * t24 * t148 * t167 * t27 * t181 * t2 + t149 * t209 * t94 * t41 * t27 * t21 * t18 * t113 - t148 * t147 * (-2 * phi__XO * t97 * t35 * t194 + 2 * t205 * t198 * t101) * t96 * t34 * t47 * t125 * t10 * t2 + 2 * alpha__r__XO * t31 * t140 * t135 * t148 * t235 * t208 * t102 * t34 * t181 * t9 / Fzr__XO * t1 + t108 * t100 * t235 * t209 * t181 * t2 - t281 * t272 * t35 * t27 * t20 * t263 * t257 * t48 / (t252 * t12 + 1) * t250 - t281 * (-2 * phi__XO * t13 * t272 * t35 * t276 * t275 * t18 + t272 * (-2 * phi__XO * t260 * t36 * t299 - 2 * t51 * t303 + 2 * t62 * t306) * t48 - 2 * alpha__r__XO * t268 * t266 / t325 * t323 * t48) * t289 * t287 * Fzr__XO * t249 - t347 * t272 * t36 * t275 * t48 * (-2 * phi__XO * t13 * t279 * t18 + t272 * t323 * t48) * t289 * t287 * t250;
  }

  real_type
  General::Mzr_D_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[50];
    real_type t2   = Fzr__XO * Fzr__XO;
    real_type t5   = ModelPars[7];
    real_type t10  = t5 * ModelPars[34] + (Fzr__XO - t5) * ModelPars[36];
    real_type t12  = ModelPars[48];
    real_type t14  = phi__XO * phi__XO;
    real_type t17  = ModelPars[54] * t14 + 1;
    real_type t18  = t17 * t17;
    real_type t22  = ModelPars[56];
    real_type t23  = ModelPars[40];
    real_type t24  = t23 * t22;
    real_type t27  = ModelPars[46] * t14 + 1;
    real_type t28  = 1.0 / t27;
    real_type t30  = alpha__r__XO * alpha__r__XO;
    real_type t31  = t22 * t22;
    real_type t35  = 1.0 / t17;
    real_type t36  = t35 * t12;
    real_type t38  = t23 * t23;
    real_type t40  = t27 * t27;
    real_type t43  = sqrt(1.0 / t40 * t2 * t38);
    real_type t44  = 1.0 / t43;
    real_type t45  = t44 * t28;
    real_type t48  = atan(alpha__r__XO * t45 * Fzr__XO * t24);
    real_type t49  = t48 * t36;
    real_type t50  = sin(t49);
    real_type t52  = ModelPars[47];
    real_type t55  = ModelPars[42] * t14 + 1;
    real_type t56  = 1.0 / t55;
    real_type t62  = atan(alpha__r__XO * t44 / t52 * t56 * t10);
    real_type t63  = t62 * t52;
    real_type t64  = sin(t63);
    real_type t69  = cos(t49);
    real_type t72  = t10 * t10;
    real_type t73  = t55 * t55;
    real_type t76  = t52 * t52;
    real_type t87  = cos(t63);
    real_type t93  = ModelPars[55];
    real_type t95  = atan(phi__XO * t93);
    real_type t101 = ModelPars[57];
    real_type t104 = ModelPars[53] * t14 + 1;
    real_type t107 = t23 / t104 * t101;
    real_type t108 = t101 * t101;
    real_type t109 = t104 * t104;
    real_type t119 = atan(alpha__r__XO * t44 * t28 * Fzr__XO * t107);
    real_type t121 = sin(t119 * t36);
    return t64 * t50 / (t31 * t30 + 1) * t28 * t24 / t18 * t12 / t10 * t2 * t1 - t87 / (t30 * t40 / t2 / t38 / t76 / t73 * t72 + 1) * t56 * t35 * t69 * Fzr__XO * t1 - t121 / (t30 / t109 * t108 + 1) * t45 * t107 * t35 * t12 / t93 * t95 * t2 * ModelPars[52];
  }

  real_type
  General::Mzr_D_3_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[50];
    real_type t2   = Fzr__XO * Fzr__XO;
    real_type t3   = t2 * t1;
    real_type t5   = ModelPars[7];
    real_type t10  = t5 * ModelPars[34] + (Fzr__XO - t5) * ModelPars[36];
    real_type t11  = 1.0 / t10;
    real_type t13  = ModelPars[48];
    real_type t14  = phi__XO * phi__XO;
    real_type t17  = ModelPars[54] * t14 + 1;
    real_type t18  = t17 * t17;
    real_type t19  = 1.0 / t18;
    real_type t21  = ModelPars[56];
    real_type t22  = t21 * t21;
    real_type t26  = ModelPars[40];
    real_type t29  = ModelPars[46] * t14 + 1;
    real_type t30  = 1.0 / t29;
    real_type t32  = alpha__r__XO * alpha__r__XO;
    real_type t34  = t22 * t32 + 1;
    real_type t35  = t34 * t34;
    real_type t36  = 1.0 / t35;
    real_type t38  = 1.0 / t17;
    real_type t39  = t38 * t13;
    real_type t42  = t26 * t26;
    real_type t44  = t29 * t29;
    real_type t45  = 1.0 / t44;
    real_type t47  = sqrt(t45 * t2 * t42);
    real_type t48  = 1.0 / t47;
    real_type t52  = atan(alpha__r__XO * t48 * t30 * Fzr__XO * t26 * t21);
    real_type t53  = t52 * t39;
    real_type t54  = sin(t53);
    real_type t55  = ModelPars[47];
    real_type t58  = ModelPars[42] * t14 + 1;
    real_type t59  = 1.0 / t58;
    real_type t65  = atan(alpha__r__XO * t48 / t55 * t59 * t10);
    real_type t66  = t65 * t55;
    real_type t67  = sin(t66);
    real_type t76  = t13 * t13;
    real_type t84  = cos(t53);
    real_type t97  = t10 * t10;
    real_type t98  = t58 * t58;
    real_type t99  = 1.0 / t98;
    real_type t101 = t55 * t55;
    real_type t102 = 1.0 / t101;
    real_type t104 = 1.0 / t42;
    real_type t110 = t32 * t44 / t2 * t104 * t102 * t99 * t97 + 1;
    real_type t112 = cos(t66);
    real_type t124 = t110 * t110;
    real_type t125 = 1.0 / t124;
    real_type t143 = ModelPars[52];
    real_type t145 = ModelPars[55];
    real_type t147 = atan(phi__XO * t145);
    real_type t149 = 1.0 / t145;
    real_type t151 = ModelPars[57];
    real_type t152 = t151 * t151;
    real_type t159 = ModelPars[53] * t14 + 1;
    real_type t160 = t159 * t159;
    real_type t165 = 1.0 / t160;
    real_type t169 = pow(t32 * t165 * t152 + 1, 2);
    real_type t170 = 1.0 / t169;
    real_type t179 = atan(alpha__r__XO * t48 * t30 * Fzr__XO * t26 / t159 * t151);
    real_type t180 = t179 * t39;
    real_type t181 = sin(t180);
    real_type t193 = cos(t180);
    return -2 * alpha__r__XO * t67 * t54 * t36 * t30 * t26 * t22 * t21 * t19 * t13 * t11 * t3 + t67 * t84 * t48 * t36 * t45 * t42 * t22 / t18 / t17 * t76 * t11 * t2 * Fzr__XO * t1 + 2 * t112 / t110 * t48 * t59 * t54 / t34 * t30 * t26 * t21 * t19 * t13 * t3 + 2 * alpha__r__XO * t44 * t104 * t102 * t97 * t112 * t125 / t98 / t58 * t38 * t84 / Fzr__XO * t1 + t67 * t48 * t10 * t125 * t99 * t38 * t84 * Fzr__XO * t1 + 2 * alpha__r__XO * t181 * t170 * t48 * t30 * t26 / t160 / t159 * t152 * t151 * t38 * t13 * t149 * t147 * t2 * t143 - t193 * t170 * t165 * t152 * t19 * t76 * t149 * t147 * Fzr__XO * t143;
  }

  real_type
  General::Mxf( real_type t__XO ) const {
    return 0;
  }

  real_type
  General::Mxf_D( real_type t__XO ) const {
    return 0;
  }

  real_type
  General::Mxf_DD( real_type t__XO ) const {
    return 0;
  }

  real_type
  General::Mxr( real_type t__XO ) const {
    return 0;
  }

  real_type
  General::Mxr_D( real_type t__XO ) const {
    return 0;
  }

  real_type
  General::Mxr_DD( real_type t__XO ) const {
    return 0;
  }


  /*\
   |  _  _            _ _ _            _
   | | || |__ _ _ __ (_) | |_ ___ _ _ (_)__ _ _ _
   | | __ / _` | '  \| | |  _/ _ \ ' \| / _` | ' \
   | |_||_\__,_|_|_|_|_|_|\__\___/_||_|_\__,_|_||_|
   |
  \*/

  real_type
  General::H_eval(
    integer              i_segment,
    CellType const &     CELL__,
    P_const_pointer_type P__
  ) const {
    integer        i_cell = CELL__.i_cell;
    real_type const * Q__ = CELL__.qM;
    real_type const * X__ = CELL__.xM;
    real_type const * L__ = CELL__.lambdaM;
    real_type const * U__ = CELL__.uM;
    Road2D::SegmentClass const & segment = pRoad->getSegmentByIndex(i_segment);
    real_type t2   = X__[25];
    real_type t5   = X__[26];
    real_type t8   = X__[27];
    real_type t11  = X__[28];
    real_type t14  = X__[29];
    real_type t17  = X__[30];
    real_type t20  = X__[24];
    real_type t23  = X__[23];
    real_type t26  = X__[19];
    real_type t29  = X__[20];
    real_type t32  = X__[21];
    real_type t35  = X__[22];
    real_type t38  = ModelPars[23];
    real_type t39  = ModelPars[142];
    real_type t40  = ModelPars[171];
    real_type t41  = t40 * t39;
    real_type t42  = t23 - t29;
    real_type t43  = X__[7];
    real_type t44  = cos(t43);
    real_type t45  = t44 * t42;
    real_type t48  = sin(t43);
    real_type t49  = t23 - t29 - t40;
    real_type t51  = t23 - t29 + t40;
    real_type t55  = X__[4];
    real_type t57  = t55 + ModelPars[64];
    real_type t58  = cos(t57);
    real_type t62  = t39 * t48;
    real_type t68  = sin(t57);
    real_type t70  = ModelPars[119];
    real_type t71  = X__[14];
    real_type t72  = t70 + t71;
    real_type t73  = t26 * t26;
    real_type t75  = X__[13];
    real_type t78  = t40 * t39 * t75 + t14;
    real_type t81  = t40 * t40;
    real_type t87  = X__[3];
    real_type t88  = cos(t87);
    real_type t90  = t75 * t73;
    real_type t96  = 2 * t40 * t39 * t70 + 2 * t40 * t39 * t71 + 2 * t17;
    real_type t98  = t75 * t81;
    real_type t100 = 2 * t14 * t41;
    real_type t102 = sin(t87);
    real_type t106 = 2 * t40 * t39 * t32;
    real_type t107 = X__[5];
    real_type t108 = t81 * t107;
    real_type t114 = t2 * L__[25] + t5 * L__[26] + t8 * L__[27] + t11 * L__[28] + t14 * L__[29] + t17 * L__[30] + t20 * L__[24] + t23 * L__[23] + t26 * L__[19] + t29 * L__[20] + t32 * L__[21] + t35 * L__[22] + (t58 * (t51 * t49 * t48 - 2 * t45 * t41) * t38 - t68 * t38 * (2 * t42 * t40 * t62 + t44 * t51 * t49) - t88 * (-2 * t17 * t41 + 2 * t26 * t78 - t81 * t70 - t71 * t81 + t73 * t72) - t102 * (t26 * t96 + t100 - t90 + t98) - t106 - t108 - t81 * (-ModelPars[117] + t70)) * L__[18];
    real_type t138 = X__[31];
    real_type t141 = X__[32];
    real_type t144 = -t51;
    real_type t145 = -t49;
    real_type t148 = -t42;
    real_type t166 = X__[12];
    real_type t171 = t29 * t29;
    real_type t172 = ModelPars[120];
    real_type t173 = X__[8];
    real_type t174 = t172 - t173;
    real_type t176 = ModelPars[22];
    real_type t177 = ModelPars[140];
    real_type t178 = t176 + t177;
    real_type t179 = 2 * t178;
    real_type t181 = t29 * t40;
    real_type t183 = t173 * t81;
    real_type t184 = t81 * t172;
    real_type t186 = 2 * t20 * t41;
    real_type t189 = -t178;
    real_type t199 = t171 * t189 + t29 * (2 * t40 * t39 * t172 - 2 * t40 * t39 * t173 - 2 * t20) + t178 * t81;
    real_type t201 = ModelPars[116];
    real_type t202 = ModelPars[118];
    real_type t203 = t201 - t202;
    real_type t205 = -t203;
    real_type t206 = 2 * t205;
    real_type t209 = t141 * t141;
    real_type t210 = t209 * t203;
    real_type t211 = t81 * t205;
    real_type t212 = t26 * t141 * t206 + t73 * t203 + t210 + t211;
    real_type t213 = X__[16];
    real_type t214 = cos(t213);
    real_type t217 = t39 * t202;
    real_type t219 = -2 * t39 * t201 + 2 * t217;
    real_type t220 = t40 * t219;
    real_type t223 = -t40 * t219;
    real_type t226 = sin(t213);
    real_type t228 = X__[11];
    real_type t230 = t73 * (t202 + t228);
    real_type t231 = X__[10];
    real_type t236 = t26 * (2 * t40 * t39 * t231 + 2 * t5);
    real_type t237 = t228 * t81;
    real_type t238 = t202 * t81;
    real_type t241 = 2 * t40 * t39 * t8;
    real_type t249 = t231 * t73;
    real_type t254 = 2 * t40 * t39 * t228 + 2 * t40 * t217 + 2 * t8;
    real_type t256 = t231 * t81;
    real_type t259 = 2 * t40 * t39 * t5;
    real_type t265 = t26 - t141;
    real_type t266 = t265 * t203;
    real_type t270 = t26 - t141 - t40;
    real_type t271 = t26 - t141 + t40;
    real_type t272 = t271 * t270;
    real_type t287 = X__[6];
    real_type t289 = t39 * t35;
    real_type t296 = Fzf(t228, t8);
    real_type t297 = X__[2];
    real_type t298 = X__[0];
    real_type t299 = X__[1];
    real_type t300 = X__[9];
    real_type t301 = X__[15];
    real_type t302 = alpha__f(t297, t298, t299, t300, t231, t301, t2, t5);
    real_type t303 = X__[18];
    real_type t304 = lambda__f(t297, t213, t298, t299, t300, t231, t301, t303, t2, t5);
    real_type t305 = Fxf(t296, t213, t302, t304);
    real_type t306 = t305 * t228;
    real_type t307 = Fyf(t296, t213, t302, t304);
    real_type t308 = t307 * t228;
    real_type t310 = Fzr(t71, t17);
    real_type t311 = alpha__r(t297, t298, t299, t166, t75, t11, t14);
    real_type t312 = X__[17];
    real_type t313 = lambda__r(t297, t87, t298, t75, t312, t11);
    real_type t314 = Fxr(t310, t87, t311, t313);
    real_type t316 = t26 * t297;
    real_type t317 = t44 * t44;
    real_type t318 = ModelPars[73];
    real_type t319 = ModelPars[189];
    real_type t320 = ModelPars[199];
    real_type t321 = t38 - t319 + t320;
    real_type t323 = t38 - t319 - t320;
    real_type t325 = t38 * t38;
    real_type t326 = ModelPars[75];
    real_type t327 = t326 * t325;
    real_type t328 = ModelPars[14];
    real_type t329 = ModelPars[19];
    real_type t330 = -t323 * t321 * t318 - t327 + t328 - t329;
    real_type t332 = t48 * t44;
    real_type t333 = t38 - t319;
    real_type t334 = t333 * t318;
    real_type t336 = ModelPars[148];
    real_type t337 = t320 * t334 - t336;
    real_type t340 = ModelPars[65];
    real_type t341 = cos(t340);
    real_type t342 = t341 * t341;
    real_type t343 = -t330;
    real_type t344 = t343 * t342;
    real_type t345 = sin(t340);
    real_type t347 = t337 * t345 * t341;
    real_type t348 = 2 * t347;
    real_type t349 = ModelPars[186];
    real_type t350 = cos(t349);
    real_type t351 = t350 * t350;
    real_type t352 = ModelPars[72];
    real_type t353 = ModelPars[68];
    real_type t354 = t353 * t353;
    real_type t355 = t354 * t352;
    real_type t356 = ModelPars[13];
    real_type t357 = ModelPars[18];
    real_type t358 = -t355 - t356 + t357;
    real_type t359 = t358 * t351;
    real_type t360 = ModelPars[145];
    real_type t361 = t352 * t360;
    real_type t363 = t350 * t353 * t361;
    real_type t364 = ModelPars[141];
    real_type t366 = sin(t349);
    real_type t367 = t366 * t353;
    real_type t368 = t367 * t352 * t364;
    real_type t369 = ModelPars[74];
    real_type t370 = t173 * t173;
    real_type t371 = t370 * t369;
    real_type t372 = t371 / 2;
    real_type t374 = t369 * t172 * t173;
    real_type t375 = ModelPars[177];
    real_type t376 = t375 * t369;
    real_type t379 = (t172 - t375 / 2) * t376;
    real_type t380 = t352 * t353;
    real_type t381 = t360 * t380;
    real_type t382 = ModelPars[24];
    real_type t383 = ModelPars[27];
    real_type t384 = t383 * t383;
    real_type t385 = ModelPars[28];
    real_type t386 = t385 * t385;
    real_type t387 = -t384 + t386;
    real_type t389 = t387 * t382 / 2;
    real_type t390 = ModelPars[9];
    real_type t391 = t390 / 2;
    real_type t392 = ModelPars[11];
    real_type t393 = t392 / 2;
    real_type t394 = t330 * t317 + 2 * t337 * t332 + t344 - t348 + t355 + t356 - t357 + t359 - t363 + t368 + t372 - t374 + t379 + t381 + t389 + t391 - t393;
    real_type t395 = t88 * t88;
    real_type t399 = t337 * t42;
    real_type t400 = t317 * t399;
    real_type t401 = 2 * t400;
    real_type t402 = t48 * t42;
    real_type t403 = t44 * t343;
    real_type t404 = t403 * t402;
    real_type t405 = -t337;
    real_type t406 = 2 * t405;
    real_type t407 = t406 * t342;
    real_type t409 = t345 * t343 * t341;
    real_type t410 = -t358;
    real_type t412 = t364 * t380;
    real_type t414 = (t410 * t366 + t412) * t350;
    real_type t415 = t367 * t361;
    real_type t417 = t369 * t178 * t173;
    real_type t418 = t178 * t369;
    real_type t419 = t375 * t418;
    real_type t420 = t382 * t385;
    real_type t421 = t383 * t420;
    real_type t422 = ModelPars[0];
    real_type t423 = t407 - t409 + t414 + t415 - t417 + t419 - t421 - t412 - t422;
    real_type t424 = t29 * t423;
    real_type t425 = t23 * t337;
    real_type t427 = t20 * t369 * t174;
    real_type t429 = -t401 - t404 + t424 + t425 + t427 / 2;
    real_type t433 = -t406;
    real_type t436 = t433 * t317 + t48 * t403 + t407 - t409 - t412 + t414 + t415 - t417 + t419 - t421 - t422;
    real_type t437 = t297 - t26;
    real_type t439 = t297 + t26;
    real_type t445 = t317 * t343 * t42;
    real_type t448 = t44 * t337 * t402;
    real_type t451 = 2 * t342 * t330;
    real_type t452 = 4 * t347;
    real_type t453 = 2 * t410;
    real_type t454 = t351 * t453;
    real_type t455 = 2 * t363;
    real_type t456 = 2 * t368;
    real_type t457 = 2 * t374;
    real_type t458 = t375 * t172;
    real_type t460 = t375 * t375;
    real_type t462 = t369 * (-2 * t458 + t460);
    real_type t463 = 2 * t355;
    real_type t464 = 2 * t381;
    real_type t466 = -t387 * t382;
    real_type t467 = 2 * t356;
    real_type t468 = 2 * t357;
    real_type t469 = t451 + t452 + t454 + t455 - t456 - t371 + t457 + t462 - t463 - t464 + t466 - t467 + t468 - t390 + t392;
    real_type t470 = t29 * t469;
    real_type t471 = t23 * t343;
    real_type t473 = t369 * t178 * t20;
    real_type t474 = -2 * t445 + 4 * t448 + t470 + t471 + t473;
    real_type t479 = t58 * t58;
    real_type t497 = t44 * t26;
    real_type t498 = t318 * t107;
    real_type t499 = t320 * t498;
    real_type t502 = t38 * t326;
    real_type t503 = t334 + t502;
    real_type t504 = t26 * t503;
    real_type t509 = t320 * t318 * t341;
    real_type t510 = t503 * t345;
    real_type t511 = t350 * t380;
    real_type t512 = t369 * t173;
    real_type t513 = t509 + t510 + t511 - t380 + t376 - t512 - t420;
    real_type t514 = t513 * t107;
    real_type t515 = t26 * t514;
    real_type t517 = t177 * t418;
    real_type t518 = ModelPars[190];
    real_type t519 = t518 * t518;
    real_type t520 = ModelPars[168];
    real_type t521 = t520 * t519;
    real_type t523 = t520 * t518 * t176;
    real_type t524 = ModelPars[12];
    real_type t525 = ModelPars[16];
    real_type t526 = ModelPars[149];
    real_type t527 = ModelPars[150];
    real_type t528 = t517 + t521 + t523 - t524 + t525 - t526 + t527;
    real_type t529 = t287 * t528;
    real_type t531 = t177 * t369;
    real_type t533 = t20 * t287 * t531;
    real_type t536 = t177 * t172 * t369;
    real_type t537 = t518 * t520;
    real_type t539 = ModelPars[200] * t537;
    real_type t540 = ModelPars[147];
    real_type t541 = -t177 * t512 + t536 - t539 - t540;
    real_type t542 = t541 * t35;
    real_type t547 = t503 * t107;
    real_type t551 = t320 * t318;
    real_type t552 = t32 * t551;
    real_type t553 = -t107 * t503 * t23 + t29 * t547 - t552;
    real_type t554 = t553 / 2;
    real_type t557 = t29 * t320 * t498;
    real_type t559 = t23 * t320 * t498;
    real_type t560 = t503 * t32;
    real_type t561 = -t557 + t559 - t560;
    real_type t562 = t561 / 2;
    real_type t564 = t541 * t287;
    real_type t566 = t503 * t341;
    real_type t567 = t366 * t380;
    real_type t569 = t320 * t318 * t345;
    real_type t570 = t383 * t382;
    real_type t571 = t566 - t567 - t569 + t570;
    real_type t572 = t107 * t571;
    real_type t573 = t29 * t572;
    real_type t576 = t341 * t32 * t551;
    real_type t578 = t369 * t20;
    real_type t579 = t107 * t578;
    real_type t582 = (t510 + t511 + t376 - t512 - t420 - t380) * t32;
    real_type t588 = t297 * t297;
    real_type t590 = t297 * t299;
    real_type t591 = t551 * t590;
    real_type t592 = t23 - t26 - t29;
    real_type t594 = t23 + t26 - t29;
    real_type t597 = t594 * t107 * t592 * t503 + t588 * t547 - t591;
    real_type t599 = t107 * t588;
    real_type t600 = t551 * t599;
    real_type t601 = t299 * t503;
    real_type t602 = t297 * t601;
    real_type t606 = -t594 * t592 * t320 * t498 - t600 - t602;
    real_type t611 = t73 * t572;
    real_type t612 = t525 / 2;
    real_type t613 = t526 / 2;
    real_type t614 = t527 / 2;
    real_type t615 = ModelPars[151];
    real_type t616 = t615 / 2;
    real_type t622 = t20 * t29;
    real_type t625 = 2 * t369 * t107 * t622;
    real_type t626 = ModelPars[1];
    real_type t627 = t298 * t298;
    real_type t628 = t627 * t626;
    real_type t630 = ModelPars[144] * t628;
    real_type t635 = t48 * t26;
    real_type t637 = 2 * t552 * t635;
    real_type t639 = t287 * t528 * t588;
    real_type t641 = t571 * t32;
    real_type t643 = 2 * t26 * t641;
    real_type t644 = t29 * t528;
    real_type t645 = t20 * t531;
    real_type t646 = 2 * t645;
    real_type t647 = t303 * t525;
    real_type t653 = ModelPars[67];
    real_type t654 = t503 * t653;
    real_type t657 = t551 * t653 * t48;
    real_type t658 = t647 / 2;
    real_type t662 = 2 * t297 * t287 * (t644 + t645 - t658);
    real_type t663 = t571 * t653;
    real_type t671 = t26 * t572;
    real_type t673 = t29 * t564;
    real_type t695 = t73 * t514;
    real_type t697 = t525 * t287 * t303;
    real_type t703 = ModelPars[139] * t628;
    real_type t707 = 2 * t552 * t497;
    real_type t711 = t287 * t541 * t588;
    real_type t717 = t287 * t541 * t171;
    real_type t719 = t35 * (t524 - t612 + t613 - t614 + t616);
    real_type t723 = t525 * t303 * t35;
    real_type t728 = t48 * t654;
    real_type t730 = 2 * t673 - 2 * t719;
    real_type t732 = t513 * t653;
    real_type t737 = t343 * t26;
    real_type t739 = t48 * t337;
    real_type t742 = t107 * t107;
    real_type t743 = t742 * t382;
    real_type t744 = t384 * t382;
    real_type t745 = ModelPars[10];
    real_type t746 = t344 - t348 - t743 + t359 + t456 - t744 + t355 + t356 - t357 + t390 - t745;
    real_type t749 = t32 * t287;
    real_type t751 = t531 + t537;
    real_type t757 = t107 * t287;
    real_type t758 = t26 * t751;
    real_type t759 = t758 * t757;
    real_type t761 = t382 * t32;
    real_type t762 = t107 * t761;
    real_type t771 = t343 * t48;
    real_type t783 = t382 * t297 * t299 * t107;
    real_type t789 = t627 * t107 * t626;
    real_type t792 = t26 * t42;
    real_type t799 = t287 * t751;
    real_type t800 = t590 * t799;
    real_type t801 = t343 / 2;
    real_type t806 = t386 * t382;
    real_type t813 = t342 * t801 - t347 + t358 * t351 / 2 - t363 + t372 - t374 + t379 + t355 / 2 + t806 / 2 + t356 / 2 - t357 / 2 + t390 / 4 + t745 / 4 - t392 / 4 + t381;
    real_type t815 = t333 * t333;
    real_type t821 = ModelPars[17];
    real_type t823 = ModelPars[15];
    real_type t828 = t312 * t821;
    real_type t841 = Q__[0];
    real_type t842 = Mxf(t841);
    real_type t843 = t301 * t842;
    real_type t844 = t306 - t301 * t308 + t314 * t71 - t479 * (4 * t395 * t394 * t316 + t88 * (-4 * t102 * t429 * t297 + 2 * t439 * t437 * t436) + 2 * t26 * t102 * t474) - t58 * (t68 * (-4 * t395 * t26 * t297 * t436 + t88 * (2 * t102 * t474 * t297 + 2 * t439 * t437 * t394) + 4 * t26 * t102 * t429) - 2 * t395 * (-2 * t48 * t107 * t504 + t29 * t529 - 2 * t499 * t497 + 2 * t515 + t533 - t542) * t297 + t88 * (-4 * t102 * (t44 * t554 + t48 * t562 + t26 * t564 - t573 / 2 + t576 / 2 - t579 / 2 + t582 / 2) * t297 + t44 * t597 + t48 * t606 - t571 * t599 + t297 * t299 * t513 + t611 - 2 * t26 * (t517 + t521 + t612 - t613 + t614 + t616 + t523) * t35 - t171 * t572 - t625 + t630) + t102 * (-2 * t497 * t560 + t637 - t639 + t73 * t529 + t643 - t29 * t287 * (t644 + t646 - t647)) + t44 * t654 - t657 + t662 - t663) - t68 * (-2 * t395 * (2 * t44 * t107 * t504 + t35 * t528 - 2 * t499 * t635 - 2 * t671 + t673) * t297 + t88 * (4 * t102 * (t44 * t562 - t48 * t554 + t26 * t529 + t29 * t514 / 2 + t641 / 2) * t297 - t44 * t606 + t48 * t597 - t588 * t514 - t571 * t590 + t695 + t26 * (t697 - 2 * t542) - t171 * t514 + t703) + t102 * (2 * t26 * t513 * t32 + 2 * t29 * t719 - 2 * t635 * t560 + t73 * t564 - t707 - t711 - t717 + t723) + t551 * t653 * t44 + t728 + t297 * t730 - t732) - t296 * t300 + t310 * t166 + 2 * t395 * (-t317 * t737 + 2 * t497 * t739 + t26 * t746 + t751 * (t35 * t107 + t749)) * t297 - t88 * (2 * t102 * (-t401 - t404 + 2 * t759 + t424 + t762 + t425) * t297 - 2 * t317 * t439 * t437 * t337 - t44 * t439 * t437 * t771 + t588 * (t433 * t342 + t409 + (t358 * t366 - t412) * t350 + t417 - t415 - t419 + t421 + t412 + t422) - t783 + t73 * t423 + 2 * t29 * t174 * t578 + t789) - t102 * (2 * t317 * t343 * t792 - 4 * t44 * t635 * t399 - t800 + 4 * t26 * (t29 * t813 + t23 * (-t815 * t318 / 2 + t328 / 4 - t329 / 4 - t327 / 2 - t821 / 4 - t823 / 4) - t647 / 4 - t828 / 4)) - t297 * (t26 * (t390 - t745 + t392) + 2 * t751 * t749) + t843;
    real_type t847 = -t174;
    real_type t849 = -t179;
    real_type t857 = t39 * t206;
    real_type t878 = t40 * t289;
    real_type t879 = 2 * t878;
    real_type t884 = t40 * t39 * t287 + t35;
    real_type t904 = t26 * t141;
    real_type t910 = -t265;
    real_type t932 = t287 * t102;
    real_type t948 = -t271;
    real_type t950 = -t270;
    real_type t969 = t102 * t75;
    real_type t970 = t88 * t71;
    real_type t975 = t44 * t58;
    real_type t979 = Fyr(t310, t87, t311, t313);
    real_type t981 = t310 * t38;
    real_type t990 = t166 * t88 * t310;
    real_type t991 = Mzr(t310, t87, t311);
    real_type t992 = t102 * t991;
    real_type t994 = X__[33];
    real_type t995 = t994 * ModelPars[114];
    real_type t996 = X__[35];
    real_type t1000 = t102 * t58;
    real_type t1007 = t588 * t847;
    real_type t1009 = t178 * t26;
    real_type t1010 = t297 * t88;
    real_type t1034 = t73 * t107;
    real_type t1035 = t297 * t298;
    real_type t1037 = t35 * t177;
    real_type t1044 = t26 * t107;
    real_type t1048 = t26 * t287;
    real_type t1067 = t369 * t171;
    real_type t1074 = -t305 * (t301 * t1000 + t68) - t307 * (-t301 * t68 + t1000) - t479 * (t395 * t1007 + 2 * t1010 * t1009 - t847 * t73) * t369 - t58 * (-2 * t68 * (-t395 * t588 * t178 / 2 + t1010 * t847 * t26 + t178 * t73 / 2) * t369 - t296 * t88 + (t395 * t599 + t88 * (t932 * t588 * t177 + t653) - t599 - t1034 - t102 * t1035 + 2 * t26 * t1037) * t369) + 2 * t68 * t369 * (t88 * t297 * (t1044 - t1037) + t102 * t297 * (t177 * t1048 + t32) - t590 / 2) + 2 * t369 * t1010 * t1009 - 2 * t102 * t297 * t369 * t847 * t29 + t369 * t1007 - t173 * (-t1067 + ModelPars[70]) - t172 * t1067 - t20 * ModelPars[31];
    real_type t1081 = t588 * t395;
    real_type t1084 = t297 * t102;
    real_type t1085 = t107 * t1084;
    real_type t1086 = t23 * t107;
    real_type t1087 = t29 * t107;
    real_type t1088 = t298 / 2;
    real_type t1089 = t1085 + t1086 - t1087 + t1088;
    real_type t1096 = t107 * t102;
    real_type t1098 = t1086 - t1087 + t1088;
    real_type t1101 = t503 * t298;
    real_type t1113 = -t26 * t561;
    real_type t1120 = -t26 * t553;
    real_type t1134 = t1084 + 2 * t23 - 2 * t29;
    real_type t1135 = t1134 * t337;
    real_type t1138 = t44 * t1134;
    real_type t1146 = 2 * t412;
    real_type t1149 = 2 * t415;
    real_type t1152 = 2 * t421;
    real_type t1153 = 2 * t422;
    real_type t1155 = t29 * (4 * t405 * t342 - 2 * t409 + (t453 * t366 + t1146) * t350 + t1149 - 2 * t417 + 2 * t419 - t1152 - t1146 - t1153);
    real_type t1156 = t23 * t433;
    real_type t1162 = t344 - t348 + t359 - t363 + t368 + t372 - t374 + t379 + t355 + t381 + t389 - t357 + t391 - t393 + t356;
    real_type t1178 = -t333 * t318;
    real_type t1179 = t1178 - t502;
    real_type t1209 = t68 * (-2 * t88 * t297 * (-t102 * t297 * t423 - 2 * t317 * t1135 - t1138 * t771 + t1155 + t1156 + t427) - 4 * (t445 - 2 * t448 + t29 * t1162 - t23 * t801 - t473 / 2) * t26) - 2 * t564 * t1081 + 2 * t88 * (-t44 * t320 * t1089 * t318 + t1096 * (t1179 * t48 + t376 - t380 - t420 + t509 + t510 + t511 - t512) * t297 - t48 * t503 * t1098 - (t1087 - t1088) * t513) * t297 + t44 * (-t102 * t653 * t551 + 2 * t1120) + t102 * (-t297 * t730 - t728 + t732) + 2 * t48 * t1113 + t711 + t717 + t29 * (2 * t671 - 2 * t719) + t26 * (-2 * t576 + 2 * t579 - 2 * t582) - t723;
    real_type t1236 = t297 * t48;
    real_type t1242 = 4 * t368;
    real_type t1243 = 2 * t744;
    real_type t1244 = t451 + t452 + t454 - t1242 + t1243 - t463 - t467 + t468 - t390 + t745 + t392;
    real_type t1248 = t320 * t320;
    real_type t1251 = -2 * t1248 * t318 - t328 + t329 - t821 - t823;
    real_type t1256 = t653 * t799;
    real_type t1259 = t751 * t297;
    real_type t1267 = t107 * t799 * t1081;
    real_type t1277 = Mxr(t841);
    real_type t1278 = -t301 * t306 - 2 * t29 * t26 * t423 - t68 * (2 * t529 * t1081 + 2 * t88 * t297 * (t44 * t503 * t1089 - t1096 * t297 * (t320 * t318 * t48 + t566 - t567 - t569 + t570) - t48 * t1098 * t551 + t573 - t341 * t1101 / 2 + t579 - (-t567 - t569 + t570) * t298 / 2) + t44 * (t102 * t654 - 2 * t1113) + t102 * (-t657 + t662 - t663) + 2 * t48 * t1120 - t639 - t287 * t528 * t171 + t29 * (2 * t515 - 2 * t287 * (t645 - t658)) + t643) - t58 * t1209 - t479 * (-2 * t88 * (t102 * t1162 * t297 - t317 * t1134 * t343 + 2 * t1138 * t739 + t470 + t471 + t473) * t297 - 2 * (-4 * t400 - 2 * t404 + t1155 + t1156 + t427) * t26) - t26 * (2 * t762 + 2 * t425) - t308 - t979 * t71 + t296 * t231 + t310 * t75 - t88 * (-t317 * t1134 * t297 * t343 + 2 * t44 * t1135 * t1236 + t102 * t746 * t588 + t297 * (-t382 * t298 * t107 + t29 * t1244 + t23 * t1251 + 2 * t473 - t647 - t828) + t1256) - (-t382 * t653 * t107 - t298 * t287 * t1259) * t102 - 2 * t1267 + 2 * t44 * t737 * t402 + 4 * t317 * t337 * t792 + t757 * t751 * t588 + t842 + t1277;
    real_type t1280 = (-t88 * (-t26 * t96 - t100 + t90 - t98) + 2 * (-t73 * t72 / 2 - t26 * t78 + (t39 * t17 + t40 * t70 / 2 + t71 * t40 / 2) * t40) * t102) * L__[17] + t138 * L__[31] + t141 * L__[32] + (-t58 * (t44 * t145 * t144 - 2 * t148 * t40 * t62) * t38 - t68 * t38 * (t145 * t144 * t48 + 2 * t44 * t148 * t41) - 2 * t40 * t39 * t11 - t81 * t166) * L__[16] + (-t58 * (t181 * t39 * t179 + t174 * t171 + t183 - t184 + t186) - t68 * t199 - t88 * (t214 * t212 + t226 * (t141 * t223 + t26 * t220) + t230 + t236 - t237 - t238 - t241) - t102 * (t214 * (t141 * t220 + t26 * t223) + t226 * t212 - t249 + t26 * t254 + t256 + t259) - t108 - t106) * L__[15] + (-t88 * (-t226 * t203 * t272 - 2 * t214 * t266 * t41 - t26 * t254 + t249 - t256 - t259) - t102 * (t214 * t203 * t272 - 2 * t226 * t266 * t41 + t230 + t236 - t237 - t238 - t241) - (t40 * t287 + 2 * t289) * t40 * t177) * L__[14] + t844 * L__[4] + (-t68 * (t181 * t39 * t849 + t847 * t171 - t183 + t184 - t186) - t58 * t199 - t226 * (t301 * (t210 + t211) + t138 * t40 * t857) - t214 * (t301 * t141 * t40 * t857 + t141 * t138 * t206) + t81 * t300 + 2 * t40 * t39 * t2) * L__[13] + (-t58 * (t287 * (t171 - t81) - t879) - 2 * t68 * t884 * t29 - t88 * (t301 * (-t73 + t81) + 2 * t40 * t39 * t138) + 2 * (t40 * t39 * t301 + t138) * t26 * t102) * L__[11] + (-t68 * (t214 * (t88 * (t287 * (t73 - 2 * t904 + t171 + t209 - t81) - t879) - 2 * t884 * t910 * t102) + 2 * t226 * (t88 * t884 * t910 - (t287 * (-t171 / 2 - t209 / 2 + t904 + t81 / 2 - t73 / 2) + t878) * t102)) + 2 * t58 * (t214 * (t88 * t884 + t910 * t932) + t226 * (-t88 * t910 * t287 + t884 * t102)) * t29 - t214 * (-t950 * t948 * t102 - 2 * t88 * t910 * t41) - t226 * (-2 * t910 * t40 * t39 * t102 + t88 * t950 * t948)) * L__[12] + (-t314 * (-t38 * t44 * t68 + t38 * t48 * t58 + t107 + t969 - t970) - t979 * (-t38 * t48 * t68 - t38 * t975 + t166) * t102 - t88 * t975 * t981 - t68 * t88 * t48 * t981 - t88 * t821 * t316 + t990 + t992 + t995 + t996) * L__[10] + t1074 * L__[8] + t1278 * L__[3];
    real_type t1286 = X__[34];
    real_type t1297 = ALIAS_maxTorque(t312);
    real_type t1305 = X__[36];
    real_type t1309 = X__[38];
    real_type t1310 = cos(t1309);
    real_type t1312 = sin(t1309);
    real_type t1318 = -t301 * t300 + t231;
    real_type t1321 = -t301 * t231 - t300;
    real_type t1325 = Mzf(t296, t213, t302);
    real_type t1328 = -t320 * t319 + t320 * t38;
    real_type t1329 = 8 * t1328;
    real_type t1331 = 8 * t336;
    real_type t1332 = t318 * t1329 - t1331;
    real_type t1336 = -t318 * t1329 + t1331;
    real_type t1338 = t29 * t1332 + t23 * t1336;
    real_type t1339 = t297 * t1338;
    real_type t1342 = t319 * t38;
    real_type t1344 = t319 * t319;
    real_type t1347 = 4 * t325 - 8 * t1342 + 4 * t1344 - 4 * t1248;
    real_type t1349 = 4 * t327;
    real_type t1350 = 4 * t328;
    real_type t1351 = 4 * t329;
    real_type t1352 = t318 * t1347 + t1349 - t1350 + t1351;
    real_type t1356 = -t318 * t1347 - t1349 + t1350 - t1351;
    real_type t1364 = 4 * t410;
    real_type t1366 = 4 * t412;
    real_type t1369 = 4 * t189;
    real_type t1372 = 4 * t415;
    real_type t1375 = t375 * t176 + t375 * t177;
    real_type t1376 = 4 * t1375;
    real_type t1378 = 4 * t421;
    real_type t1379 = 4 * t422;
    real_type t1382 = 4 * t1328;
    real_type t1384 = 4 * t336;
    real_type t1385 = t318 * t1382 - t1384;
    real_type t1386 = t23 * t1385;
    real_type t1387 = t172 * t578;
    real_type t1388 = 2 * t1387;
    real_type t1390 = t369 * t20 * t173;
    real_type t1391 = 2 * t1390;
    real_type t1396 = t26 * t1356;
    real_type t1397 = t317 * t297;
    real_type t1400 = t44 * t1236;
    real_type t1405 = -t1364;
    real_type t1407 = 4 * t363;
    real_type t1408 = 2 * t371;
    real_type t1409 = 4 * t374;
    real_type t1412 = 4 * t458 - 2 * t460;
    real_type t1415 = 4 * t381;
    real_type t1420 = 2 * t390;
    real_type t1422 = t341 * t345 * t1336 + t342 * t1352 + t351 * t1405 + t369 * t1412 + 2 * t387 * t382 + t1242 - t1407 + t1408 - t1409 + t1415 + t1420 + 4 * t355 + 4 * t356 - 4 * t357 - 2 * t392;
    real_type t1429 = t23 * t1352 + t29 * t1356;
    real_type t1430 = t26 * t1429;
    real_type t1435 = 2 * t325;
    real_type t1436 = 4 * t1342;
    real_type t1437 = 2 * t1344;
    real_type t1439 = -t1435 + t1436 - t1437 + 2 * t1248;
    real_type t1441 = 2 * t327;
    real_type t1442 = 2 * t328;
    real_type t1443 = 2 * t329;
    real_type t1444 = t318 * t1439 - t1441 + t1442 - t1443;
    real_type t1445 = t23 * t1444;
    real_type t1448 = t20 * t369 * t849 + t29 * t1422 + t1445;
    real_type t1454 = -t318 * t1382 + t1384;
    real_type t1455 = t73 * t1454;
    real_type t1462 = -t318 * t1439 + t1441 - t1442 + t1443;
    real_type t1474 = t342 * t1385 + t341 * t345 * t1462 + (-t453 * t366 - t1146) * t350 + t173 * t369 * t179 - t1149 - 2 * t369 * t1375 + t1152 + t1146 + t1153;
    real_type t1500 = t342 * t1332 + t341 * t345 * t1352 + (t1405 * t366 - t1366) * t350 - t173 * t369 * t1369 - t1372 - t369 * t1376 + t1378 + t1366 + t1379;
    real_type t1520 = t342 * t1444;
    real_type t1522 = t341 * t345 * t1385;
    real_type t1523 = t1520 + t1522 + t454 + t455 - t371 + t457 - t456 + t462 - t463 - t464 + t466 - t467 + t468 - t390 + t392;
    real_type t1529 = 2 * t333;
    real_type t1531 = 2 * t502;
    real_type t1532 = t1529 * t318 + t1531;
    real_type t1537 = -t1529 * t318 - t1531;
    real_type t1538 = t23 * t1537;
    real_type t1540 = 2 * t552;
    real_type t1542 = t297 * (t29 * t107 * t1532 + t107 * t1538 - t1540);
    real_type t1544 = 2 * t557;
    real_type t1545 = 2 * t559;
    real_type t1546 = t32 * t1537;
    real_type t1554 = 2 * t569;
    real_type t1555 = 2 * t567;
    real_type t1556 = 2 * t570;
    real_type t1557 = t341 * t1537 + t1554 + t1555 - t1556;
    real_type t1560 = 2 * t579;
    real_type t1561 = 2 * t576;
    real_type t1562 = t32 * t1532;
    real_type t1566 = 2 * t350 * t32 * t380;
    real_type t1568 = -2 * t512 - 2 * t420 - 2 * t380 + 2 * t376;
    real_type t1574 = t44 * t316;
    real_type t1577 = 4 * t333;
    real_type t1579 = 4 * t502;
    real_type t1580 = t318 * t1577 + t1579;
    real_type t1582 = t48 * t316;
    real_type t1587 = -t318 * t1577 - t1579;
    real_type t1597 = t177 * t176;
    real_type t1598 = t177 * t177;
    real_type t1599 = -t1597 - t1598;
    real_type t1600 = 2 * t1599;
    real_type t1601 = t369 * t1600;
    real_type t1602 = 2 * t521;
    real_type t1603 = 2 * t523;
    real_type t1604 = 2 * t524;
    real_type t1605 = 2 * t525;
    real_type t1606 = 2 * t526;
    real_type t1607 = 2 * t527;
    real_type t1608 = t1601 - t1602 - t1603 + t1604 - t1605 + t1606 - t1607;
    real_type t1614 = 2 * t531 * t35 * t173;
    real_type t1616 = 2 * t536 - 2 * t539 - 2 * t540;
    real_type t1627 = t341 * t1546;
    real_type t1630 = 2 * t345 * t32 * t551;
    real_type t1632 = 2 * t567 - 2 * t570;
    real_type t1633 = t32 * t1632;
    real_type t1647 = t73 * t107 * t1179;
    real_type t1650 = t23 * t23;
    real_type t1651 = t1650 * t503;
    real_type t1653 = t107 * t1651 + t1087 * t1538 + t171 * t547 + t1647 - t591;
    real_type t1655 = t299 * t1179;
    real_type t1656 = t297 * t1655;
    real_type t1658 = t551 * t107 * t1650;
    real_type t1659 = t23 * t29;
    real_type t1661 = 2 * t499 * t1659;
    real_type t1662 = t551 * t1034;
    real_type t1664 = t551 * t107 * t171;
    real_type t1674 = -t420 - t380 + t376;
    real_type t1682 = t1179 * t341 + t567 + t569 - t570;
    real_type t1689 = t23 * t1532;
    real_type t1691 = t298 * t1179;
    real_type t1693 = t297 * (t29 * t107 * t1537 + t107 * t1689 + t1540 + t1691);
    real_type t1696 = t320 * t298 * t318;
    real_type t1700 = -t541;
    real_type t1702 = 2 * t287 * t1700;
    real_type t1705 = t341 * t1532 - t1554 - t1555 + t1556;
    real_type t1708 = -t1540 + t1101;
    real_type t1713 = -t32 * t1568;
    real_type t1737 = 2 * t509;
    real_type t1739 = 2 * t511;
    real_type t1740 = 2 * t380;
    real_type t1741 = 2 * t376;
    real_type t1742 = 2 * t512;
    real_type t1743 = 2 * t420;
    real_type t1744 = t1537 * t345 - t1737 - t1739 + t1740 - t1741 + t1742 + t1743;
    real_type t1774 = -t1604 + t525 - t526 + t527 - t615;
    real_type t1796 = t1179 * t345 - t376 + t380 + t420 - t509 - t511 + t512;
    real_type t1811 = t345 * t1532 + t1737 + t1739 - t1740 + t1741 - t1742 - t1743;
    real_type t1814 = t1562 + t1696;
    real_type t1829 = t29 * t1454 + t1386;
    real_type t1837 = -t751;
    real_type t1842 = 2 * t762;
    real_type t1844 = -2 * t1328;
    real_type t1846 = 2 * t336;
    real_type t1847 = t318 * t1844 + t1846;
    real_type t1859 = 2 * t743 + t1520 + t1522 + t454 - t1242 + t1243 - t463 - t467 + t468 - t1420 + 2 * t745;
    real_type t1861 = 2 * t1837;
    real_type t1862 = t35 * t1861;
    real_type t1879 = -t369 * t1412 + t1407 - t1408 + t1409 - t1415 + t1520 + t1522 - t390 + t392 + t454 - t463 - t467 + t468 - t745 - 2 * t806;
    real_type t1895 = t325 - 2 * t1342 + t1344 - t1248;
    real_type t1916 = -t1861;
    real_type t1917 = t287 * t1916;
    real_type t1921 = -t1318 * t305 - t1321 * t307 - t314 * t75 - t979 * t166 + t1325 + t991 - t479 * (t395 * (t317 * t1339 + t332 * t297 * (t29 * t1352 + t23 * t1356) + t297 * (t29 * (t342 * t1336 + t341 * t345 * t1356 + t350 * (t366 * t1364 + t1366) + t173 * t369 * t1369 + t1372 + t369 * t1376 - t1378 - t1366 - t1379) + t1386 + t1388 - t1391)) + t88 * (t102 * (t1400 * t26 * t1332 + t297 * t26 * t1422 + t1397 * t1396) + t317 * t1430 + t332 * t26 * t1338 + t26 * t1448) + t102 * (t332 * t73 * t1444 + t317 * t1455 + t73 * t1474)) - t58 * (t68 * (t395 * (t317 * t297 * t1429 + t332 * t1339 + t297 * t1448) + t88 * (t102 * (t1397 * t26 * t1336 + t297 * t26 * t1500 + t1400 * t1396) + t317 * t26 * (t23 * t1332 + t29 * t1336) + t332 * t1430 + t26 * (t23 * t1454 + t29 * t1500 - t1388 + t1391)) + t102 * (t317 * t73 * t1462 + t332 * t1455 + t73 * t1523)) + t395 * (t44 * t1542 + t48 * t297 * (-t1544 + t1545 + t1546) + t297 * (t29 * t107 * t1557 + 4 * t26 * t541 * t287 + t345 * t1562 + t32 * t1568 - t1560 + t1561 + t1566)) + t88 * (t102 * (4 * t499 * t1574 + t1582 * t107 * t1580 + t297 * (t26 * t107 * (t345 * t1587 - 4 * t376 + 4 * t380 + 4 * t420 - 4 * t509 - 4 * t511 + 4 * t512) + t29 * t287 * t1608 - 2 * t533 - t1614 + t35 * t1616)) + t497 * t1562 - t637 + t73 * t287 * (t369 * t1599 - t521 - t523 + t524 - t525 + t526 - t527) + t26 * (t1627 + t1630 + t1633) + t171 * t287 * (-t369 * t1599 + t521 + t523 - t524 + t525 - t526 + t527) + t29 * t287 * (t646 - t647)) + t102 * (t44 * t1653 + t48 * (t1656 - t1658 + t1661 + t1662 - t1664) + t297 * (-t369 * t299 * t173 + t551 * t299 * t341 + t380 * t299 * t350 + t299 * t1674 + t345 * t601) + t611 + t26 * t35 * (t1601 - t1602 - t1603 - t525 + t526 - t527 - t615) + t171 * t107 * t1682 - t625 + t630) + t44 * t1693 + t48 * t297 * (t1544 - t1545 + t1562 + t1696) + t297 * (t26 * t1702 + t29 * t107 * t1705 + t1560 + t341 * t1708 + t345 * (t1546 - t1696) - t1566 + t1713 - t380 * t298 * t366 + t383 * t298 * t382)) - t68 * (t395 * (t44 * t297 * (t1544 - t1545 + t1562) + t48 * t1542 + t297 * (t26 * t287 * (4 * t369 * t1599 - 4 * t521 - 4 * t523 + 4 * t524 - 4 * t525 + 4 * t526 - 4 * t527) + t29 * t107 * t1744 + t1627 + t1630 + t1633)) + t88 * (t102 * (t1574 * t107 * t1587 + 4 * t499 * t1582 + t297 * (t26 * t107 * (t341 * t1580 - 4 * t567 - 4 * t569 + 4 * t570) + t29 * t1702 + t35 * t1608)) + t707 + t635 * t1562 + t73 * t287 * t1700 + t26 * (t345 * t1546 - t1561 - t1566 + t1713) + t717 + t29 * t35 * t1774 - t723) + t102 * (t44 * (t602 + t1658 - t1661 - t1662 + t1664) + t48 * t1653 + t297 * (t345 * t299 * t551 + t366 * t299 * t380 - t382 * t299 * t383 + t341 * t1655) + t695 + t26 * (-t35 * t1616 + t1614 + t697) + t171 * t107 * t1796 + t703) + t44 * t297 * (-t1544 + t1545 + t1546 - t1696) + t48 * t1693 + t297 * (t26 * t287 * (-t369 * t1600 + t1602 + t1603 - t1604 + t1605 - t1606 + t1607) + t29 * t107 * t1811 + t341 * t1814 + t345 * t1708 + t380 * t298 * t350 - t32 * t1632 - t369 * t298 * t173 + t298 * t1674)) - t395 * (t317 * t297 * t1829 + t332 * t297 * (t29 * t1444 + t23 * t1462) + t297 * (4 * t1044 * t287 * t1837 + t29 * t1474 + t23 * t1847 - t1842)) - t88 * (t102 * (t1397 * t26 * t1462 + t1400 * t26 * t1454 + t297 * (t287 * t32 * t1861 + t107 * t1862 + t26 * t1859)) + t317 * t26 * (t29 * t1462 + t1445) + t332 * t26 * t1829 + t800 + t26 * (t29 * t1879 + t23 * (t318 * (t1435 - t1436 + t1437) + t1441 + t821 - t328 + t823 + t329) + t647 + t828)) - t102 * (t317 * t73 * (-t318 * t1844 - t1846) + t332 * t73 * (t318 * t1895 + t327 - t328 + t329) - t783 + t73 * (t342 * t1847 + t341 * t345 * (-t318 * t1895 - t327 + t328 - t329) + t414 + t173 * t369 * t189 + t415 + t369 * t1375 - t421 - t412 - t422) + t29 * (-2 * t1390 + 2 * t1387) + t789) - t297 * (t1044 * t1917 - t1388 + t1391 + t1842);
    real_type t1924 = t42 * t42;
    real_type t1925 = t73 + t1924;
    real_type t1928 = t1925 * t1179;
    real_type t1934 = t1179 * t42;
    real_type t1937 = t29 * t1682;
    real_type t1948 = 2 * t578;
    real_type t1962 = t107 * t382;
    real_type t1963 = t73 * t1962;
    real_type t1968 = t287 * t1837;
    real_type t1969 = t73 * t1968;
    real_type t1971 = 2 * t26 * t761;
    real_type t1977 = Q__[1];
    real_type t1978 = X__[37];
    real_type t1980 = t1978 * t1977 - 1;
    real_type t1981 = t298 * t1310;
    real_type t1982 = t299 * t1312;
    real_type t1984 = 1.0 / (t1981 - t1982);
    real_type t1985 = t1984 * t1980;
    real_type t1988 = roadLeftLateralBorder(t1978 + Q__[2]);
    real_type t1992 = roadRightLateralBorder(Q__[3] - t1978);
    real_type t1994 = MaxRollAngle(t87);
    real_type t1996 = MaxSteerAngle(t287);
    real_type t2008 = t588 * t551;
    real_type t2011 = t171 * t551;
    real_type t2013 = 2 * t1659 * t551;
    real_type t2014 = t1650 * t551;
    real_type t2015 = t320 * t318 * t73 + t2008 + t2011 - t2013 + t2014;
    real_type t2017 = t171 * t503;
    real_type t2018 = t29 * t1538;
    real_type t2020 = t588 * t503;
    real_type t2023 = t171 * t1796;
    real_type t2025 = t588 * t1796;
    real_type t2028 = t26 * t1532;
    real_type t2031 = t26 * t1538 + t29 * t2028;
    real_type t2038 = 2 * t551 * t23 * t26 - 2 * t551 * t29 * t26;
    real_type t2052 = 2 * t551 * t23 * t297 - 2 * t551 * t29 * t297;
    real_type t2060 = t297 * t29 * t1811;
    real_type t2066 = t588 * t1179;
    real_type t2073 = 2 * t369 * t622;
    real_type t2081 = t26 * t1744;
    real_type t2108 = (-U__[0] * ModelPars[153] - t1286) * L__[33] + (-U__[1] * ModelPars[154] - t996) * L__[35] + (t1297 * U__[2] - t995) * L__[34] + (U__[3] * ModelPars[185] - t1305) * L__[36] + (t299 * t1310 + t298 * t1312) * L__[37] + t1921 * L__[5] + (-t58 * (t88 * (-t44 * t1925 * t551 + t48 * t1928 + (t73 + t171) * t513) + 2 * t26 * (t44 * t1934 + t402 * t551 + t1937 - t578) * t102) - t68 * (t88 * (-t44 * t1928 - t48 * t1925 * t551 + t73 * t1682 + (t1937 - t1948) * t29) - 2 * t26 * t102 * (-t48 * t1934 + t29 * t513 + t45 * t551)) + t296 + t310 - t88 * (t26 * t35 * t1916 - t1963) - t102 * (t1969 - t1971) - t382 * t653) * L__[2] - t1988 * t1985 - t1992 * t1985 - t1994 * t1985 - t1996 * t1985 + 1.0 / t1980 * (t1977 * (t297 * t1978 + t1981 - t1982) - t297) * L__[38] + (t301 * t305 + t307 + t979 - t58 * (t102 * (t44 * t2015 + t48 * (t73 * t503 + t1651 + t2017 + t2018 + t2020) + t2023 + t73 * t1796 + t2025) + t88 * (t29 * t26 * t1557 - 2 * t20 * t26 * t369 + t44 * t2031 + t48 * t2038) + t44 * t2052 + t48 * (t29 * t297 * t1537 + t297 * t1689) + t2060) - t68 * (t102 * (t44 * (t1650 * t1179 + t171 * t1179 + t73 * t1179 + t29 * t1689 + t2066) + t48 * t2015 + t171 * t571 + t2073 + t73 * t571 + t588 * t571) + t88 * (t48 * t2031 - t44 * t2038 + t29 * t2081) + t44 * (t29 * t297 * t1532 + t297 * t1538) + t48 * t2052 + t29 * t297 * t1557 - 2 * t369 * t297 * t20) - t102 * (t26 * t1862 + t382 * t599 + t1963) - t88 * (t588 * t1968 + t1969 - t1971) - t382 * t1035) * L__[1];
    real_type t2110 = 1.0 / ModelPars[152];
    real_type t2112 = OnlyBrakingRear(-t996 * t2110);
    real_type t2114 = ModelPars[8];
    real_type t2115 = 1.0 / t2114;
    real_type t2118 = FrontWheelContact((t296 - t2114) * t2115);
    real_type t2120 = LongSlipRear(t313);
    real_type t2122 = LatSlipRear(t311);
    real_type t2126 = RearWheelContact((t310 - t2114) * t2115);
    real_type t2128 = LatSlipFront(t302);
    real_type t2130 = LongSlipFront(t304);
    real_type t2136 = t297 * (t29 * t1537 + t1689);
    real_type t2143 = -2 * t320 * t318 * t23 + 2 * t320 * t318 * t29;
    real_type t2154 = 2 * t88 * t316 * t551 + t1651 + t2017 + t2018 + t2020;
    real_type t2194 = t287 * t300;
    real_type t2196 = t178 * t301;
    real_type t2202 = t287 * t228;
    real_type t2203 = t847 * t301;
    real_type t2207 = t287 * t847;
    real_type t2208 = t301 * t228;
    real_type t2216 = t287 * t231;
    real_type t2235 = t525 * t316;
    real_type t2236 = t214 * t1286;
    real_type t2279 = t320 * t1178 + t336;
    real_type t2281 = t395 * t588 * t2279;
    real_type t2286 = -t323 * t321 * t318 - t327 + t328 - t329;
    real_type t2289 = t88 * t26 * t297 * t2286;
    real_type t2290 = t2279 * t73;
    real_type t2291 = -t2281 + t2289 + t2290;
    real_type t2295 = t395 * t588 * t2286;
    real_type t2297 = t88 * t2279 * t316;
    real_type t2298 = 4 * t2297;
    real_type t2299 = t73 * t2286;
    real_type t2302 = t44 * t48 * (t2295 + t2298 - t2299);
    real_type t2320 = t2066 * t107 * t395;
    real_type t2326 = 2 * t318 * t316 * t320 * t107 - t1179 * t653;
    real_type t2330 = t102 * (t1540 + t1691) * t297;
    real_type t2331 = t107 * t2066;
    real_type t2347 = t588 * t498 * t320 * t395 + t88 * (2 * t297 * t107 * t1179 * t26 + t318 * t320 * t653) - t102 * t1814 * t297 - t600 - t1656 - t1662;
    real_type t2365 = alpha__crw(t841);
    real_type t2366 = sin(t2365);
    real_type t2375 = -t314 * (t969 - t970 + t107) - t166 * t102 * t979 + t992 - t479 * (4 * t317 * t2291 + 2 * t2281 - 2 * t2289 - 2 * t2290 - 2 * t2302) - t58 * (t68 * (t317 * (2 * t2295 + 8 * t2297 - 2 * t2299) + 4 * t44 * t2291 * t48 - t2295 - t2298 + t2299) + t44 * (t88 * t2326 + t1647 - t2320 + t2330 + t2331 - t591) - t2347 * t48) - t68 * (t44 * t2347 - t48 * (-t88 * t2326 - t1647 + t2320 - t2330 - t2331 + t591)) + t990 + 2 * t317 * t2291 - t2302 + t2281 + t88 * t1251 * t316 - t2366 * t38 * t994 - t2290 + t995 - (ModelPars[32] * t23 + ModelPars[71] * t43) * ModelPars[29];
    real_type t2380 = pow(t43 - ModelPars[66], 2);
    real_type t2383 = pow(t107 - ModelPars[69], 2);
    real_type t2386 = pow(t32 - ModelPars[163], 2);
    real_type t2389 = pow(t1978 - ModelPars[76], 2);
    real_type t2392 = pow(t173 - ModelPars[179], 2);
    real_type t2395 = pow(t55 - ModelPars[188], 2);
    real_type t2398 = pow(t35 - ModelPars[157], 2);
    real_type t2401 = pow(t138 - ModelPars[205], 2);
    real_type t2404 = pow(t23 - ModelPars[162], 2);
    real_type t2407 = pow(t26 - ModelPars[174], 2);
    real_type t2410 = pow(t141 - ModelPars[175], 2);
    real_type t2413 = pow(t20 - ModelPars[178], 2);
    real_type t2416 = pow(t29 - ModelPars[187], 2);
    real_type t2419 = pow(t2 - ModelPars[191], 2);
    real_type t2422 = pow(t11 - ModelPars[193], 2);
    real_type t2425 = pow(t5 - ModelPars[195], 2);
    real_type t2428 = pow(t14 - ModelPars[197], 2);
    real_type t2431 = pow(t8 - ModelPars[201], 2);
    real_type t2434 = pow(t17 - ModelPars[203], 2);
    real_type t2435 = t2380 + t2383 + t2386 + t2389 + t2392 + t2395 + t2398 + t2401 + t2404 + t2407 + t2410 + t2413 + t2416 + t2419 + t2422 + t2425 + t2428 + t2431 + t2434;
    real_type t2448 = t68 * (t107 * t88 - t228);
    real_type t2449 = t88 * t176;
    real_type t2462 = t369 * t1597;
    real_type t2463 = t369 * t1598;
    real_type t2466 = t518 * (t176 + t518) * t520;
    real_type t2467 = t2462 + t2463 + t2466 - t526 + t527 - t524 + t525;
    real_type t2468 = t588 * t2467;
    real_type t2470 = t1700 * t26;
    real_type t2508 = t29 * (t2462 + t2463 + t2466 - t613 + t614 + t616 + t612) + t645 - t658;
    real_type t2544 = t29 * t2467;
    real_type t2562 = -t305 * (t58 * (t102 * t228 + t88 * t1318) + (t2448 + t2449) * t301) - t307 * (t58 * (-t102 * t2208 + t88 * t1321) + t2448 + t2449) + t88 * t1325 * t58 - t479 * (-2 * t1010 * t2470 - t2467 * t73 + t395 * t2468) * t287 - t58 * (-2 * t68 * (t588 * t1700 * t395 / 2 + t1010 * t2467 * t26 - t1700 * t73 / 2) * t287 - t300 * t102 * t296 - 2 * t88 * t297 * (-t102 * t1700 * t297 / 2 + t759 + t1700 * t29) + t102 * (t297 * t1774 * t26 - 2 * t799 * t32 * t297 + t843) + t800 - 2 * t2508 * t26) - t68 * (t296 * (t1096 + t231) - t1267 + t88 * (t588 * t2467 * t102 - 2 * t297 * t2508 - t1256) + t102 * t799 * t1035 + t799 * (t588 + t73) * t107 + 2 * t29 * t2470 + t842) - t102 * t296 * t176 + 2 * t1081 * (t2463 + t2462 / 2 + t518 * (t518 + t176 / 2) * t520 - t526 + t527 - t524 + t525) * t287 - t88 * (t1085 + t298) * t1259 - t102 * (-2 * t287 * (t2544 + t645 - t658) * t297 + t653 * t751) - t287 * (t2468 + t73 * t751 * t176 + t29 * (t2544 + t646 - t647)) + 2 * t32 * t758 - t35 * ModelPars[146] + t1305;
    real_type t2565 = OnlyTractionRear(t994 * t2110);
    real_type t2568 = OnlyBrakingFront(-t1286 * t2110);
    real_type t2570 = -t2112 * t1985 - t2118 * t1985 - t2120 * t1985 - t2122 * t1985 - t2126 * t1985 - t2128 * t1985 - t2130 * t1985 + (t305 - t301 * t307 + t314 - t58 * (t102 * (t44 * t2136 + t48 * t297 * t2143 + t297 * (t29 * t1705 + t1948)) + t44 * t2154 + t48 * (t1010 * t2028 - t2008 - t2011 + t2013 - t2014) + t1010 * t2081 + t588 * t1682 + t171 * t1682 - t2073) - t68 * (t102 * (-t44 * t297 * t2143 + t48 * t2136 + t2060) + t44 * (t1010 * t26 * t1537 + t2008 + t2011 - t2013 + t2014) + t48 * t2154 + t1010 * t26 * t1705 + t2025 + t2023) - t102 * t297 * (t26 * t1917 + 2 * t761) - t88 * t297 * (2 * t26 * t1962 + t1862) - t628 + t382 * t590) * L__[0] + (-t307 * (t68 * (t102 * t174 + t88 * t2194 - t2196) + t58 * (t107 * t88 * t287 + t102 * t178 - t2202 - t2203) + t102 * t1321 + t88 * (t2207 + t2208) - t301 * t107) - t305 * (t68 * (t102 * (-t2202 - t2203) - t88 * t2216 + t176 + t177) + t58 * (t102 * t2196 - t172 + t173) + t102 * t1318 - t88 * t228 + t107) - t1325 * (t287 * t68 * t88 - t102) - t68 * (t296 * (t102 * t2194 + t847 * t88) - t102 * (-t2235 + t2236) * t287 + t88 * (t1286 * t287 * t226 - t525 * t35 * t297) + t525 * t29 * t1048) - t58 * (t296 * (t107 * t932 + t88 * t189 + t2216) - t297 * t525 * t88 * t29 * t287 - t26 * t35 * t525 + t287 * t842) - t296 * (t102 * t2207 + t88 * t300) + t102 * t226 * t1286 + (-t2235 + t2236 + t843) * t88) * L__[9] + t2375 * L__[7] - t1984 * t1980 * (ModelPars[133] * t2435 + ModelPars[136]) + t2562 * L__[6] - t2565 * t1985 - t2568 * t1985;
    return t114 + t1280 + t2108 + t2570;
  }

  /*\
   |   ___               _ _   _
   |  | _ \___ _ _  __ _| | |_(_)___ ___
   |  |  _/ -_) ' \/ _` | |  _| / -_|_-<
   |  |_| \___|_||_\__,_|_|\__|_\___/__/
  \*/

  real_type
  General::penalties_eval(
    NodeType const     & NODE__,
    U_const_pointer_type U__,
    P_const_pointer_type P__
  ) const {
    integer     i_segment = NODE__.i_segment;
    real_type const * Q__ = NODE__.q;
    real_type const * X__ = NODE__.x;
    Road2D::SegmentClass const & segment = pRoad->getSegmentByIndex(i_segment);
    real_type t2   = X__[37];
    real_type t5   = X__[38];
    real_type t6   = cos(t5);
    real_type t7   = X__[0];
    real_type t9   = sin(t5);
    real_type t10  = X__[1];
    real_type t14  = 1.0 / (-t10 * t9 + t7 * t6) * (t2 * Q__[1] - 1);
    real_type t16  = 1.0 / ModelPars[152];
    real_type t19  = OnlyBrakingFront(-X__[34] * t16);
    real_type t23  = OnlyBrakingRear(-X__[35] * t16);
    real_type t27  = OnlyTractionRear(X__[33] * t16);
    real_type t29  = ModelPars[8];
    real_type t30  = 1.0 / t29;
    real_type t33  = Fzf(X__[11], X__[27]);
    real_type t36  = FrontWheelContact((t33 - t29) * t30);
    real_type t40  = Fzr(X__[14], X__[30]);
    real_type t43  = RearWheelContact((t40 - t29) * t30);
    real_type t45  = X__[2];
    real_type t47  = X__[9];
    real_type t48  = X__[10];
    real_type t49  = X__[15];
    real_type t51  = X__[25];
    real_type t52  = X__[26];
    real_type t53  = lambda__f(t45, X__[16], t7, t10, t47, t48, t49, X__[18], t51, t52);
    real_type t54  = LongSlipFront(t53);
    real_type t56  = X__[3];
    real_type t57  = X__[13];
    real_type t59  = X__[28];
    real_type t60  = lambda__r(t45, t56, t7, t57, X__[17], t59);
    real_type t61  = LongSlipRear(t60);
    real_type t63  = alpha__f(t45, t7, t10, t47, t48, t49, t51, t52);
    real_type t64  = LatSlipFront(t63);
    real_type t68  = alpha__r(t45, t7, t10, X__[12], t57, t59, X__[29]);
    real_type t69  = LatSlipRear(t68);
    real_type t72  = MaxSteerAngle(X__[6]);
    real_type t74  = MaxRollAngle(t56);
    real_type t78  = roadRightLateralBorder(Q__[3] - t2);
    real_type t82  = roadLeftLateralBorder(t2 + Q__[2]);
    return -t19 * t14 - t23 * t14 - t27 * t14 - t36 * t14 - t43 * t14 - t54 * t14 - t61 * t14 - t64 * t14 - t69 * t14 - t72 * t14 - t74 * t14 - t78 * t14 - t82 * t14;
  }

  real_type
  General::control_penalties_eval(
    NodeType const     & NODE__,
    U_const_pointer_type U__,
    P_const_pointer_type P__
  ) const {
    integer     i_segment = NODE__.i_segment;
    real_type const * Q__ = NODE__.q;
    real_type const * X__ = NODE__.x;
    Road2D::SegmentClass const & segment = pRoad->getSegmentByIndex(i_segment);
    real_type t5   = X__[38];
    real_type t6   = cos(t5);
    real_type t9   = sin(t5);
    real_type t14  = 1.0 / (X__[0] * t6 - X__[1] * t9) * (Q__[1] * X__[37] - 1);
    real_type t18  = t__oControl(U__[2], ModelPars[156], ModelPars[173]);
    real_type t21  = ModelPars[169];
    real_type t22  = ModelPars[30];
    real_type t23  = b__f__oControl(U__[0], t21, t22);
    real_type t26  = b__r__oControl(U__[1], t21, t22);
    real_type t29  = ModelPars[180];
    real_type t30  = tau__oControl(U__[3], -t29, t29);
    return -t18 * t14 - t23 * t14 - t26 * t14 - t30 * t14;
  }

  /*\
   |   _
   |  | |   __ _ __ _ _ _ __ _ _ _  __ _ ___
   |  | |__/ _` / _` | '_/ _` | ' \/ _` / -_)
   |  |____\__,_\__, |_| \__,_|_||_\__, \___|
   |            |___/              |___/
  \*/

  real_type
  General::lagrange_target(
    NodeType const     & NODE__,
    U_const_pointer_type U__,
    P_const_pointer_type P__
  ) const {
    integer     i_segment = NODE__.i_segment;
    real_type const * Q__ = NODE__.q;
    real_type const * X__ = NODE__.x;
    Road2D::SegmentClass const & segment = pRoad->getSegmentByIndex(i_segment);
    real_type t5   = pow(X__[7] - ModelPars[66], 2);
    real_type t9   = pow(X__[5] - ModelPars[69], 2);
    real_type t13  = pow(X__[21] - ModelPars[163], 2);
    real_type t14  = X__[37];
    real_type t17  = pow(t14 - ModelPars[76], 2);
    real_type t21  = pow(X__[8] - ModelPars[179], 2);
    real_type t25  = pow(X__[4] - ModelPars[188], 2);
    real_type t29  = pow(X__[22] - ModelPars[157], 2);
    real_type t33  = pow(X__[31] - ModelPars[205], 2);
    real_type t37  = pow(X__[23] - ModelPars[162], 2);
    real_type t41  = pow(X__[19] - ModelPars[174], 2);
    real_type t45  = pow(X__[32] - ModelPars[175], 2);
    real_type t49  = pow(X__[24] - ModelPars[178], 2);
    real_type t53  = pow(X__[20] - ModelPars[187], 2);
    real_type t57  = pow(X__[25] - ModelPars[191], 2);
    real_type t61  = pow(X__[28] - ModelPars[193], 2);
    real_type t65  = pow(X__[26] - ModelPars[195], 2);
    real_type t69  = pow(X__[29] - ModelPars[197], 2);
    real_type t73  = pow(X__[27] - ModelPars[201], 2);
    real_type t77  = pow(X__[30] - ModelPars[203], 2);
    real_type t78  = t5 + t9 + t13 + t17 + t21 + t25 + t29 + t33 + t37 + t41 + t45 + t49 + t53 + t57 + t61 + t65 + t69 + t73 + t77;
    real_type t86  = X__[38];
    real_type t87  = cos(t86);
    real_type t90  = sin(t86);
    return -1.0 / (X__[0] * t87 - X__[1] * t90) * (Q__[1] * t14 - 1) * (ModelPars[133] * t78 + ModelPars[136]);
  }

  /*\
   |   __  __
   |  |  \/  |__ _ _  _ ___ _ _
   |  | |\/| / _` | || / -_) '_|
   |  |_|  |_\__,_|\_, \___|_|
   |               |__/
  \*/

  real_type
  General::mayer_target(
    NodeType const     & LEFT__,
    NodeType const     & RIGHT__,
    P_const_pointer_type P__
  ) const {
    integer i_segment_left  = LEFT__.i_segment;
    real_type const * QL__  = LEFT__.q;
    real_type const * XL__  = LEFT__.x;
    integer i_segment_right = RIGHT__.i_segment;
    real_type const * QR__  = RIGHT__.q;
    real_type const * XR__  = RIGHT__.x;
    Road2D::SegmentClass const & segmentLeft  = pRoad->getSegmentByIndex(i_segment_left);
    Road2D::SegmentClass const & segmentRight = pRoad->getSegmentByIndex(i_segment_right);
    real_type t4   = pow(XL__[36] - ModelPars[121], 2);
    real_type t8   = pow(XL__[37] - ModelPars[76], 2);
    real_type t12  = pow(XL__[38] - ModelPars[143], 2);
    real_type t16  = pow(XL__[33] - ModelPars[4], 2);
    real_type t20  = pow(XL__[34] - ModelPars[25], 2);
    real_type t24  = pow(XL__[35] - ModelPars[26], 2);
    real_type t28  = pow(XL__[29] - ModelPars[197], 2);
    real_type t32  = pow(XL__[30] - ModelPars[203], 2);
    real_type t36  = pow(XL__[31] - ModelPars[205], 2);
    real_type t40  = pow(XL__[32] - ModelPars[175], 2);
    real_type t44  = pow(XL__[25] - ModelPars[191], 2);
    real_type t48  = pow(XL__[26] - ModelPars[195], 2);
    real_type t52  = pow(XL__[27] - ModelPars[201], 2);
    real_type t56  = pow(XL__[28] - ModelPars[193], 2);
    real_type t60  = pow(XL__[21] - ModelPars[163], 2);
    real_type t64  = pow(XL__[22] - ModelPars[157], 2);
    real_type t68  = pow(XL__[23] - ModelPars[162], 2);
    real_type t72  = pow(XL__[24] - ModelPars[178], 2);
    real_type t76  = pow(XL__[18] - ModelPars[170], 2);
    real_type t77  = t4 + t8 + t12 + t16 + t20 + t24 + t28 + t32 + t36 + t40 + t44 + t48 + t52 + t56 + t60 + t64 + t68 + t72 + t76;
    real_type t81  = pow(XL__[19] - ModelPars[174], 2);
    real_type t85  = pow(XL__[20] - ModelPars[187], 2);
    real_type t89  = pow(XL__[14] - ModelPars[204], 2);
    real_type t93  = pow(XL__[15] - ModelPars[158], 2);
    real_type t97  = pow(XL__[16] - ModelPars[176], 2);
    real_type t101 = pow(XL__[17] - ModelPars[172], 2);
    real_type t105 = pow(XL__[10] - ModelPars[196], 2);
    real_type t109 = pow(XL__[11] - ModelPars[202], 2);
    real_type t113 = pow(XL__[12] - ModelPars[194], 2);
    real_type t117 = pow(XL__[13] - ModelPars[198], 2);
    real_type t121 = pow(XL__[6] - ModelPars[159], 2);
    real_type t125 = pow(XL__[7] - ModelPars[66], 2);
    real_type t129 = pow(XL__[8] - ModelPars[179], 2);
    real_type t133 = pow(XL__[9] - ModelPars[192], 2);
    real_type t137 = pow(XL__[3] - ModelPars[91], 2);
    real_type t141 = pow(XL__[4] - ModelPars[188], 2);
    real_type t145 = pow(XL__[5] - ModelPars[69], 2);
    real_type t149 = pow(XL__[0] - ModelPars[126], 2);
    real_type t153 = pow(XL__[1] - ModelPars[127], 2);
    real_type t157 = pow(XL__[2] - ModelPars[155], 2);
    real_type t158 = t81 + t85 + t89 + t93 + t97 + t101 + t105 + t109 + t113 + t117 + t121 + t125 + t129 + t133 + t137 + t141 + t145 + t149 + t153 + t157;
    return ModelPars[130] * (t77 + t158);
  }

  /*\
   |    ___
   |   / _ \
   |  | (_) |
   |   \__\_\
  \*/

  integer
  General::q_numEqns() const
  { return 13; }

  void
  General::q_eval(
    integer        i_node,
    integer        i_segment,
    real_type      s,
    Q_pointer_type result__
  ) const {
    Road2D::SegmentClass const & segment = pRoad->getSegmentByIndex(i_segment);
    result__[ 0   ] = s;
    result__[ 1   ] = ALIAS_kappa(s);
    result__[ 2   ] = ALIAS_leftWidth(s);
    result__[ 3   ] = ALIAS_rightWidth(s);
    result__[ 4   ] = ALIAS_sectionSpeedLimit();
    result__[ 5   ] = ALIAS_adherence();
    result__[ 6   ] = ALIAS_xISOMidLane(s);
    result__[ 7   ] = ALIAS_yISOMidLane(s);
    result__[ 8   ] = ALIAS_xISOleft(s);
    result__[ 9   ] = ALIAS_yISOleft(s);
    result__[ 10  ] = ALIAS_xISOright(s);
    result__[ 11  ] = ALIAS_yISOright(s);
    result__[ 12  ] = ALIAS_ISOAngle(s);
    #ifdef MECHATRONIX_DEBUG
    CHECK_NAN(result__.pointer(),"q_eval",13);
    #endif
  }

  /*\
   |    ___
   |   / __|_  _ ___ ______
   |  | (_ | || / -_|_-<_-<
   |   \___|\_,_\___/__/__/
  \*/

  integer
  General::u_guess_numEqns() const
  { return 4; }

  void
  General::u_guess_eval(
    NodeType2 const    & NODE__,
    P_const_pointer_type P__,
    U_pointer_type       UGUESS__
  ) const {
    integer     i_segment = NODE__.i_segment;
    real_type const * Q__ = NODE__.q;
    real_type const * X__ = NODE__.x;
    real_type const * L__ = NODE__.lambda;
      Road2D::SegmentClass const & segment = pRoad->getSegmentByIndex(i_segment);
    std::fill_n( UGUESS__.pointer(), 4, 0 );
    UGUESS__[ iU_b__f__o ] = 0;
    UGUESS__[ iU_b__r__o ] = 0;
    UGUESS__[ iU_t__o    ] = 0;
    UGUESS__[ iU_tau__o  ] = 0;
    #ifdef MECHATRONIX_DEBUG
    CHECK_NAN(UGUESS__.pointer(),"u_guess_eval",4);
    #endif
  }

  void
  General::u_guess_eval(
    NodeType2 const    & LEFT__,
    NodeType2 const    & RIGHT__,
    P_const_pointer_type P__,
    U_pointer_type       UGUESS__
  ) const {
    NodeType2 NODE__;
    real_type Q__[13];
    real_type X__[39];
    real_type L__[39];
    NODE__.i_segment = LEFT__.i_segment;
    NODE__.q      = Q__;
    NODE__.x      = X__;
    NODE__.lambda = L__;
    // Qvars
    Q__[0] = (LEFT__.q[0]+RIGHT__.q[0])/2;
    Q__[1] = (LEFT__.q[1]+RIGHT__.q[1])/2;
    Q__[2] = (LEFT__.q[2]+RIGHT__.q[2])/2;
    Q__[3] = (LEFT__.q[3]+RIGHT__.q[3])/2;
    Q__[4] = (LEFT__.q[4]+RIGHT__.q[4])/2;
    Q__[5] = (LEFT__.q[5]+RIGHT__.q[5])/2;
    Q__[6] = (LEFT__.q[6]+RIGHT__.q[6])/2;
    Q__[7] = (LEFT__.q[7]+RIGHT__.q[7])/2;
    Q__[8] = (LEFT__.q[8]+RIGHT__.q[8])/2;
    Q__[9] = (LEFT__.q[9]+RIGHT__.q[9])/2;
    Q__[10] = (LEFT__.q[10]+RIGHT__.q[10])/2;
    Q__[11] = (LEFT__.q[11]+RIGHT__.q[11])/2;
    Q__[12] = (LEFT__.q[12]+RIGHT__.q[12])/2;
    // Xvars
    X__[0] = (LEFT__.x[0]+RIGHT__.x[0])/2;
    X__[1] = (LEFT__.x[1]+RIGHT__.x[1])/2;
    X__[2] = (LEFT__.x[2]+RIGHT__.x[2])/2;
    X__[3] = (LEFT__.x[3]+RIGHT__.x[3])/2;
    X__[4] = (LEFT__.x[4]+RIGHT__.x[4])/2;
    X__[5] = (LEFT__.x[5]+RIGHT__.x[5])/2;
    X__[6] = (LEFT__.x[6]+RIGHT__.x[6])/2;
    X__[7] = (LEFT__.x[7]+RIGHT__.x[7])/2;
    X__[8] = (LEFT__.x[8]+RIGHT__.x[8])/2;
    X__[9] = (LEFT__.x[9]+RIGHT__.x[9])/2;
    X__[10] = (LEFT__.x[10]+RIGHT__.x[10])/2;
    X__[11] = (LEFT__.x[11]+RIGHT__.x[11])/2;
    X__[12] = (LEFT__.x[12]+RIGHT__.x[12])/2;
    X__[13] = (LEFT__.x[13]+RIGHT__.x[13])/2;
    X__[14] = (LEFT__.x[14]+RIGHT__.x[14])/2;
    X__[15] = (LEFT__.x[15]+RIGHT__.x[15])/2;
    X__[16] = (LEFT__.x[16]+RIGHT__.x[16])/2;
    X__[17] = (LEFT__.x[17]+RIGHT__.x[17])/2;
    X__[18] = (LEFT__.x[18]+RIGHT__.x[18])/2;
    X__[19] = (LEFT__.x[19]+RIGHT__.x[19])/2;
    X__[20] = (LEFT__.x[20]+RIGHT__.x[20])/2;
    X__[21] = (LEFT__.x[21]+RIGHT__.x[21])/2;
    X__[22] = (LEFT__.x[22]+RIGHT__.x[22])/2;
    X__[23] = (LEFT__.x[23]+RIGHT__.x[23])/2;
    X__[24] = (LEFT__.x[24]+RIGHT__.x[24])/2;
    X__[25] = (LEFT__.x[25]+RIGHT__.x[25])/2;
    X__[26] = (LEFT__.x[26]+RIGHT__.x[26])/2;
    X__[27] = (LEFT__.x[27]+RIGHT__.x[27])/2;
    X__[28] = (LEFT__.x[28]+RIGHT__.x[28])/2;
    X__[29] = (LEFT__.x[29]+RIGHT__.x[29])/2;
    X__[30] = (LEFT__.x[30]+RIGHT__.x[30])/2;
    X__[31] = (LEFT__.x[31]+RIGHT__.x[31])/2;
    X__[32] = (LEFT__.x[32]+RIGHT__.x[32])/2;
    X__[33] = (LEFT__.x[33]+RIGHT__.x[33])/2;
    X__[34] = (LEFT__.x[34]+RIGHT__.x[34])/2;
    X__[35] = (LEFT__.x[35]+RIGHT__.x[35])/2;
    X__[36] = (LEFT__.x[36]+RIGHT__.x[36])/2;
    X__[37] = (LEFT__.x[37]+RIGHT__.x[37])/2;
    X__[38] = (LEFT__.x[38]+RIGHT__.x[38])/2;
    // Lvars
    L__[0] = (LEFT__.lambda[0]+RIGHT__.lambda[0])/2;
    L__[1] = (LEFT__.lambda[1]+RIGHT__.lambda[1])/2;
    L__[2] = (LEFT__.lambda[2]+RIGHT__.lambda[2])/2;
    L__[3] = (LEFT__.lambda[3]+RIGHT__.lambda[3])/2;
    L__[4] = (LEFT__.lambda[4]+RIGHT__.lambda[4])/2;
    L__[5] = (LEFT__.lambda[5]+RIGHT__.lambda[5])/2;
    L__[6] = (LEFT__.lambda[6]+RIGHT__.lambda[6])/2;
    L__[7] = (LEFT__.lambda[7]+RIGHT__.lambda[7])/2;
    L__[8] = (LEFT__.lambda[8]+RIGHT__.lambda[8])/2;
    L__[9] = (LEFT__.lambda[9]+RIGHT__.lambda[9])/2;
    L__[10] = (LEFT__.lambda[10]+RIGHT__.lambda[10])/2;
    L__[11] = (LEFT__.lambda[11]+RIGHT__.lambda[11])/2;
    L__[12] = (LEFT__.lambda[12]+RIGHT__.lambda[12])/2;
    L__[13] = (LEFT__.lambda[13]+RIGHT__.lambda[13])/2;
    L__[14] = (LEFT__.lambda[14]+RIGHT__.lambda[14])/2;
    L__[15] = (LEFT__.lambda[15]+RIGHT__.lambda[15])/2;
    L__[16] = (LEFT__.lambda[16]+RIGHT__.lambda[16])/2;
    L__[17] = (LEFT__.lambda[17]+RIGHT__.lambda[17])/2;
    L__[18] = (LEFT__.lambda[18]+RIGHT__.lambda[18])/2;
    L__[19] = (LEFT__.lambda[19]+RIGHT__.lambda[19])/2;
    L__[20] = (LEFT__.lambda[20]+RIGHT__.lambda[20])/2;
    L__[21] = (LEFT__.lambda[21]+RIGHT__.lambda[21])/2;
    L__[22] = (LEFT__.lambda[22]+RIGHT__.lambda[22])/2;
    L__[23] = (LEFT__.lambda[23]+RIGHT__.lambda[23])/2;
    L__[24] = (LEFT__.lambda[24]+RIGHT__.lambda[24])/2;
    L__[25] = (LEFT__.lambda[25]+RIGHT__.lambda[25])/2;
    L__[26] = (LEFT__.lambda[26]+RIGHT__.lambda[26])/2;
    L__[27] = (LEFT__.lambda[27]+RIGHT__.lambda[27])/2;
    L__[28] = (LEFT__.lambda[28]+RIGHT__.lambda[28])/2;
    L__[29] = (LEFT__.lambda[29]+RIGHT__.lambda[29])/2;
    L__[30] = (LEFT__.lambda[30]+RIGHT__.lambda[30])/2;
    L__[31] = (LEFT__.lambda[31]+RIGHT__.lambda[31])/2;
    L__[32] = (LEFT__.lambda[32]+RIGHT__.lambda[32])/2;
    L__[33] = (LEFT__.lambda[33]+RIGHT__.lambda[33])/2;
    L__[34] = (LEFT__.lambda[34]+RIGHT__.lambda[34])/2;
    L__[35] = (LEFT__.lambda[35]+RIGHT__.lambda[35])/2;
    L__[36] = (LEFT__.lambda[36]+RIGHT__.lambda[36])/2;
    L__[37] = (LEFT__.lambda[37]+RIGHT__.lambda[37])/2;
    L__[38] = (LEFT__.lambda[38]+RIGHT__.lambda[38])/2;
    this->u_guess_eval( NODE__, P__, UGUESS__ );
  }

  /*\
   |    ___ _           _
   |   / __| |_  ___ __| |__
   |  | (__| ' \/ -_) _| / /
   |   \___|_||_\___\__|_\_\
  \*/

  bool
  General::u_check_if_admissible(
    NodeType2 const    & NODE__,
    U_const_pointer_type U__,
    P_const_pointer_type P__
  ) const {
    bool ok = true;
    integer     i_segment = NODE__.i_segment;
    real_type const * Q__ = NODE__.q;
    real_type const * X__ = NODE__.x;
    real_type const * L__ = NODE__.lambda;
    Road2D::SegmentClass const & segment = pRoad->getSegmentByIndex(i_segment);
    ok = ok && t__oControl.check_range(U__[2], ModelPars[156], ModelPars[173]);
    real_type t5   = ModelPars[169];
    real_type t6   = ModelPars[30];
    ok = ok && b__f__oControl.check_range(U__[0], t5, t6);
    ok = ok && b__r__oControl.check_range(U__[1], t5, t6);
    real_type t9   = ModelPars[180];
    ok = ok && tau__oControl.check_range(U__[3], -t9, t9);
    return ok;
  }

  /*\
   |   ___        _     ___                       _
   |  | _ \___ __| |_  | _ \_ _ ___  __ ___ _____(_)_ _  __ _
   |  |  _/ _ (_-<  _| |  _/ '_/ _ \/ _/ -_|_-<_-< | ' \/ _` |
   |  |_| \___/__/\__| |_| |_| \___/\__\___/__/__/_|_||_\__, |
   |                                                    |___/
  \*/

  integer
  General::post_numEqns() const
  { return 54; }

  void
  General::post_eval(
    NodeType2 const    & NODE__,
    U_const_pointer_type U__,
    P_const_pointer_type P__,
    real_type            result__[]
  ) const {
    integer     i_segment = NODE__.i_segment;
    real_type const * Q__ = NODE__.q;
    real_type const * X__ = NODE__.x;
    real_type const * L__ = NODE__.lambda;
    Road2D::SegmentClass const & segment = pRoad->getSegmentByIndex(i_segment);
    result__[ 0   ] = t__oControl(U__[2], ModelPars[156], ModelPars[173]);
    real_type t5   = ModelPars[169];
    real_type t6   = ModelPars[30];
    result__[ 1   ] = b__f__oControl(U__[0], t5, t6);
    result__[ 2   ] = b__r__oControl(U__[1], t5, t6);
    real_type t9   = ModelPars[180];
    result__[ 3   ] = tau__oControl(U__[3], -t9, t9);
    real_type t11  = 1.0 / ModelPars[152];
    result__[ 4   ] = OnlyBrakingFront(-X__[34] * t11);
    result__[ 5   ] = OnlyBrakingRear(-X__[35] * t11);
    result__[ 6   ] = OnlyTractionRear(X__[33] * t11);
    real_type t18  = ModelPars[8];
    real_type t19  = 1.0 / t18;
    real_type t20  = X__[11];
    real_type t21  = X__[27];
    real_type t22  = Fzf(t20, t21);
    result__[ 7   ] = FrontWheelContact((t22 - t18) * t19);
    real_type t25  = X__[14];
    real_type t26  = X__[30];
    real_type t27  = Fzr(t25, t26);
    result__[ 8   ] = RearWheelContact((t27 - t18) * t19);
    real_type t30  = X__[2];
    real_type t31  = X__[16];
    real_type t32  = X__[0];
    real_type t33  = X__[1];
    real_type t34  = X__[9];
    real_type t35  = X__[10];
    real_type t36  = X__[15];
    real_type t38  = X__[25];
    real_type t39  = X__[26];
    real_type t40  = lambda__f(t30, t31, t32, t33, t34, t35, t36, X__[18], t38, t39);
    result__[ 9   ] = LongSlipFront(t40);
    real_type t41  = X__[3];
    real_type t42  = X__[13];
    real_type t43  = X__[17];
    real_type t44  = X__[28];
    real_type t45  = lambda__r(t30, t41, t32, t42, t43, t44);
    result__[ 10  ] = LongSlipRear(t45);
    real_type t46  = alpha__f(t30, t32, t33, t34, t35, t36, t38, t39);
    result__[ 11  ] = LatSlipFront(t46);
    real_type t47  = X__[12];
    real_type t48  = X__[29];
    real_type t49  = alpha__r(t30, t32, t33, t47, t42, t44, t48);
    result__[ 12  ] = LatSlipRear(t49);
    real_type t50  = X__[6];
    result__[ 13  ] = MaxSteerAngle(t50);
    result__[ 14  ] = MaxRollAngle(t41);
    real_type t52  = X__[37];
    result__[ 15  ] = roadRightLateralBorder(Q__[3] - t52);
    result__[ 16  ] = roadLeftLateralBorder(t52 + Q__[2]);
    result__[ 17  ] = t22;
    result__[ 18  ] = t27;
    result__[ 19  ] = t45;
    result__[ 20  ] = t40;
    result__[ 21  ] = t49;
    result__[ 22  ] = t46;
    result__[ 23  ] = Fxf(result__[17], t31, result__[22], result__[20]);
    result__[ 24  ] = Fxr(result__[18], t41, result__[21], result__[19]);
    result__[ 25  ] = Fyf(result__[17], t31, result__[22], result__[20]);
    result__[ 26  ] = Fyr(result__[18], t41, result__[21], result__[19]);
    result__[ 27  ] = Mzf(result__[17], t31, result__[22]);
    result__[ 28  ] = Mzr(result__[18], t41, result__[21]);
    real_type t56  = Q__[0];
    result__[ 29  ] = Mxf(t56);
    result__[ 30  ] = Mxr(t56);
    result__[ 31  ] = ALIAS_maxTorque(t43);
    result__[ 32  ] = Q__[10];
    result__[ 33  ] = Q__[11];
    result__[ 34  ] = Q__[8];
    result__[ 35  ] = Q__[9];
    real_type t58  = Q__[12];
    real_type t59  = sin(t58);
    result__[ 36  ] = t52 * t59 + Q__[6];
    real_type t62  = cos(t58);
    result__[ 37  ] = -t52 * t62 + Q__[7];
    real_type t66  = X__[4] + ModelPars[64];
    real_type t67  = cos(t66);
    real_type t69  = cos(t41);
    result__[ 38  ] = t36 * t69 - t67 * t50;
    real_type t71  = sin(t31);
    real_type t72  = t71 * t69;
    real_type t73  = sin(t41);
    real_type t74  = cos(t31);
    real_type t75  = t74 * t73;
    real_type t76  = t69 * t74;
    real_type t77  = t73 * t71;
    real_type t80  = sin(t66);
    result__[ 39  ] = -t72 + t75 - t80 * (t76 + t77) * t50;
    real_type t82  = ModelPars[22];
    real_type t83  = ModelPars[140];
    real_type t84  = t82 + t83;
    real_type t86  = ModelPars[120];
    real_type t87  = X__[8];
    real_type t88  = t86 - t87;
    real_type t90  = ModelPars[116];
    real_type t91  = ModelPars[118];
    real_type t92  = t90 - t91;
    real_type t93  = t71 * t92;
    result__[ 40  ] = -t36 * t93 + t84 * t67 + t80 * t88 - t34;
    real_type t95  = -t92;
    real_type t97  = t95 * t74 - t20 - t91;
    result__[ 41  ] = t97 * t73 + (t93 - t35) * t69 + t83 * t50;
    real_type t109 = X__[5];
    result__[ 42  ] = -t88 * t67 + t84 * t80 + t97 * t69 + t73 * (t95 * t71 + t35) + t109;
    real_type t110 = X__[7];
    real_type t111 = sin(t110);
    real_type t113 = ModelPars[23];
    real_type t115 = cos(t110);
    result__[ 43  ] = -t113 * t111 * t80 - t113 * t115 * t67 + t47;
    real_type t118 = ModelPars[119];
    real_type t119 = -t118 - t25;
    real_type t121 = t69 * t42;
    result__[ 44  ] = t119 * t73 - t121;
    real_type t127 = t73 * t42;
    result__[ 45  ] = t113 * t111 * t67 - t113 * t115 * t80 + t119 * t69 + t109 + t118 + t127 - ModelPars[117];
    real_type t129 = t80 * t50;
    real_type t130 = X__[20];
    real_type t132 = X__[22];
    real_type t134 = X__[31];
    real_type t136 = X__[19];
    real_type t137 = t136 * t73;
    result__[ 46  ] = t130 * t129 - t67 * t132 + t134 * t69 - t36 * t137;
    real_type t144 = X__[32];
    real_type t145 = t144 * t71;
    real_type t148 = t144 * t74;
    real_type t154 = t50 * t71;
    real_type t155 = t130 * t67;
    real_type t163 = t50 * t74;
    result__[ 47  ] = -t130 * t69 * t67 * t163 - t73 * t132 * t80 * t71 - t136 * t80 * t69 * t154 + t136 * t73 * t80 * t163 + t80 * t69 * t50 * t145 - t73 * t129 * t148 - t132 * t80 * t76 - t73 * t155 * t154 + t136 * t76 + t136 * t77 - t73 * t145 - t69 * t148;
    real_type t170 = X__[24];
    real_type t172 = t74 * t36;
    real_type t181 = t134 * t71;
    real_type t185 = t130 * t80;
    result__[ 48  ] = -t130 * t67 * t87 - t130 * t80 * t82 - t90 * t144 * t172 + t91 * t144 * t172 + t86 * t155 - t80 * t170 - t90 * t181 + t91 * t181 - t83 * t185 - t38;
    real_type t190 = t90 * t73;
    real_type t192 = t91 * t73;
    real_type t194 = t90 * t136;
    real_type t196 = t91 * t136;
    real_type t198 = t90 * t69;
    real_type t200 = t91 * t69;
    real_type t208 = t136 * t69;
    result__[ 49  ] = -t136 * t69 * t20 + t136 * t73 * t35 + t132 * t83 + t190 * t145 - t192 * t145 + t198 * t148 - t200 * t148 - t194 * t76 - t194 * t77 + t196 * t76 + t196 * t77 - t91 * t208 - t73 * t21 - t69 * t39;
    real_type t210 = X__[21];
    result__[ 50  ] = t130 * t67 * t82 - t130 * t80 * t87 + t136 * t73 * t20 + t136 * t69 * t35 + t91 * t137 + t198 * t145 - t200 * t145 - t190 * t148 + t192 * t148 + t83 * t155 + t67 * t170 + t86 * t185 - t194 * t72 + t194 * t75 + t196 * t72 - t196 * t75 - t69 * t21 + t73 * t39 + t210;
    real_type t233 = t111 * t113;
    real_type t234 = X__[23];
    real_type t235 = t67 * t234;
    real_type t238 = t115 * t113;
    real_type t239 = t80 * t234;
    result__[ 51  ] = -t155 * t233 + t185 * t238 + t235 * t233 - t239 * t238 + t44;
    result__[ 52  ] = -t136 * t69 * t25 - t118 * t208 + t136 * t127 - t26 * t73 - t69 * t48;
    result__[ 53  ] = t136 * t73 * t25 + t118 * t137 + t136 * t121 - t155 * t238 - t185 * t233 + t239 * t233 + t235 * t238 - t26 * t69 + t73 * t48 + t210;
    #ifdef MECHATRONIX_DEBUG
    CHECK_NAN(result__,"post_eval",54);
    #endif
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  integer
  General::integrated_post_numEqns() const
  { return 1; }

  void
  General::integrated_post_eval(
    NodeType2 const    & NODE__,
    U_const_pointer_type U__,
    P_const_pointer_type P__,
    real_type            result__[]
  ) const {
    integer     i_segment = NODE__.i_segment;
    real_type const * Q__ = NODE__.q;
    real_type const * X__ = NODE__.x;
    real_type const * L__ = NODE__.lambda;
    Road2D::SegmentClass const & segment = pRoad->getSegmentByIndex(i_segment);
    real_type t5   = X__[38];
    real_type t6   = cos(t5);
    real_type t9   = sin(t5);
    result__[ 0   ] = -1.0 / (X__[0] * t6 - X__[1] * t9) * (X__[37] * Q__[1] - 1);
    #ifdef MECHATRONIX_DEBUG
    CHECK_NAN(result__,"integrated_post_eval",1);
    #endif
  }

}

// EOF: General_Methods1.cc

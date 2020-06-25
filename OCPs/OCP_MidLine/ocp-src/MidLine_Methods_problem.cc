/*-----------------------------------------------------------------------*\
 |  file: MidLine_Methods1.cc                                            |
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


namespace MidLineDefine {
  /*\
   |   ___         _   _               _   _
   |  / __|___ _ _| |_(_)_ _ _  _ __ _| |_(_)___ _ _
   | | (__/ _ \ ' \  _| | ' \ || / _` |  _| / _ \ ' \
   |  \___\___/_||_\__|_|_||_\_,_\__,_|\__|_\___/_||_|
  \*/

  void
  MidLine::continuationStep0( real_type s ) {
    int msg_level = 3;
    pConsole->message(
      fmt::format( "\nContinuation step N.0 s = {}\n", s ),
      msg_level
    );
    real_type t1   = ModelPars[141];
    ModelPars[140] = t1 + (ModelPars[142] - t1) * s;
    real_type t5   = ModelPars[138];
    ModelPars[137] = t5 + (ModelPars[139] - t5) * s;
  }
  /*\
   |   ___         _   _               _   _
   |  / __|___ _ _| |_(_)_ _ _  _ __ _| |_(_)___ _ _
   | | (__/ _ \ ' \  _| | ' \ || / _` |  _| / _ \ ' \
   |  \___\___/_||_\__|_|_||_\_,_\__,_|\__|_\___/_||_|
  \*/

  void
  MidLine::continuationStep1( real_type s ) {
    int msg_level = 3;
    pConsole->message(
      fmt::format( "\nContinuation step N.1 s = {}\n", s ),
      msg_level
    );
    real_type t1   = ModelPars[133];
    ModelPars[132] = t1 + (ModelPars[134] - t1) * s;
  }
  /*\
   |   ___         _   _               _   _
   |  / __|___ _ _| |_(_)_ _ _  _ __ _| |_(_)___ _ _
   | | (__/ _ \ ' \  _| | ' \ || / _` |  _| / _ \ ' \
   |  \___\___/_||_\__|_|_||_\_,_\__,_|\__|_\___/_||_|
  \*/

  void
  MidLine::continuationStep2( real_type s ) {
    int msg_level = 3;
    pConsole->message(
      fmt::format( "\nContinuation step N.2 s = {}\n", s ),
      msg_level
    );
    real_type t1   = ModelPars[135];
    real_type w__ic = t1 + (ModelPars[136] - t1) * s;
  }
  /*\
   |   ___         _   _               _   _
   |  / __|___ _ _| |_(_)_ _ _  _ __ _| |_(_)___ _ _
   | | (__/ _ \ ' \  _| | ' \ || / _` |  _| / _ \ ' \
   |  \___\___/_||_\__|_|_||_\_,_\__,_|\__|_\___/_||_|
  \*/

  void
  MidLine::continuationStep3( real_type s ) {
    int msg_level = 3;
    pConsole->message(
      fmt::format( "\nContinuation step N.3 s = {}\n", s ),
      msg_level
    );
    real_type t1   = ModelPars[60];
    real_type t5   = t1 + (ModelPars[61] - t1) * s;
    b__f__oControl.update_epsilon(t5);
    real_type t6   = ModelPars[124];
    real_type t10  = t6 + (ModelPars[125] - t6) * s;
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
    real_type t16  = ModelPars[126];
    real_type t20  = t16 + (ModelPars[127] - t16) * s;
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
    real_type t26  = ModelPars[91];
    real_type t30  = t26 + (ModelPars[92] - t26) * s;
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
  MidLine::alpha__crw( real_type t__XO ) const {
    return asin(1.0 / ModelPars[23] * (ModelPars[116] - ModelPars[117]));
  }

  real_type
  MidLine::alpha__crw_D( real_type t__XO ) const {
    return 0;
  }

  real_type
  MidLine::alpha__crw_DD( real_type t__XO ) const {
    return 0;
  }

  real_type
  MidLine::alpha__pin( real_type eta__XO, real_type alpha__crw__XO ) const {
    return alpha__crw__XO + eta__XO;
  }

  real_type
  MidLine::alpha__pin_D_1( real_type eta__XO, real_type alpha__crw__XO ) const {
    return 1;
  }

  real_type
  MidLine::alpha__pin_D_1_1( real_type eta__XO, real_type alpha__crw__XO ) const {
    return 0;
  }

  real_type
  MidLine::alpha__pin_D_1_2( real_type eta__XO, real_type alpha__crw__XO ) const {
    return 0;
  }

  real_type
  MidLine::alpha__pin_D_2( real_type eta__XO, real_type alpha__crw__XO ) const {
    return 1;
  }

  real_type
  MidLine::alpha__pin_D_2_2( real_type eta__XO, real_type alpha__crw__XO ) const {
    return 0;
  }

  real_type
  MidLine::Fzf( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return -z__f__XO * ModelPars[20] - z__f__dot__XO * ModelPars[2];
  }

  real_type
  MidLine::Fzf_D_1( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return -ModelPars[20];
  }

  real_type
  MidLine::Fzf_D_1_1( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return 0;
  }

  real_type
  MidLine::Fzf_D_1_2( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return 0;
  }

  real_type
  MidLine::Fzf_D_2( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return -ModelPars[2];
  }

  real_type
  MidLine::Fzf_D_2_2( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return 0;
  }

  real_type
  MidLine::Fzr( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return -z__r__XO * ModelPars[21] - z__r__dot__XO * ModelPars[3];
  }

  real_type
  MidLine::Fzr_D_1( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return -ModelPars[21];
  }

  real_type
  MidLine::Fzr_D_1_1( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return 0;
  }

  real_type
  MidLine::Fzr_D_1_2( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return 0;
  }

  real_type
  MidLine::Fzr_D_2( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return -ModelPars[3];
  }

  real_type
  MidLine::Fzr_D_2_2( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return 0;
  }

  real_type
  MidLine::alpha__r( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t7   = atan((-Omega__XO * x__r__XO + v__XO + y__r__dot__XO) / (-y__r__XO * Omega__XO + u__XO - x__r__dot__XO));
    return -t7;
  }

  real_type
  MidLine::alpha__r_D_1( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t6   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t7   = t2 * t2;
    real_type t8   = 1.0 / t7;
    real_type t12  = t6 * t6;
    return -1.0 / (t8 * t12 + 1) * (-1.0 / t2 * x__r__XO + y__r__XO * t8 * t6);
  }

  real_type
  MidLine::alpha__r_D_1_1( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  MidLine::alpha__r_D_1_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  MidLine::alpha__r_D_1_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  MidLine::alpha__r_D_1_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  MidLine::alpha__r_D_1_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  MidLine::alpha__r_D_1_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  MidLine::alpha__r_D_1_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  MidLine::alpha__r_D_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t5   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t6   = 1.0 / t5;
    real_type t8   = t2 * t2;
    return 1.0 / (t6 * t8 + 1) * t6 * t2;
  }

  real_type
  MidLine::alpha__r_D_2_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  MidLine::alpha__r_D_2_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t7   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t9   = t4 * t7 + 1;
    real_type t12  = t3 * t3;
    real_type t15  = t9 * t9;
    return 1.0 / t9 * t4 - 2 / t15 / t12 * t7;
  }

  real_type
  MidLine::alpha__r_D_2_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t8   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t10  = t4 * t8 + 1;
    real_type t13  = t3 * t3;
    real_type t16  = t10 * t10;
    return -1.0 / t10 * t4 * Omega__XO + 2 * Omega__XO / t16 / t13 * t8;
  }

  real_type
  MidLine::alpha__r_D_2_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  MidLine::alpha__r_D_2_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  MidLine::alpha__r_D_2_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t7   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t9   = t4 * t7 + 1;
    real_type t12  = t3 * t3;
    real_type t15  = t9 * t9;
    return 1.0 / t9 * t4 - 2 / t15 / t12 * t7;
  }

  real_type
  MidLine::alpha__r_D_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t6   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t7   = t2 * t2;
    return -1.0 / (1.0 / t7 * t6 + 1) / t2;
  }

  real_type
  MidLine::alpha__r_D_3_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t12  = pow(1.0 / t3 * t8 + 1, 2);
    return 2 * t7 / t12 / t3 / t2;
  }

  real_type
  MidLine::alpha__r_D_3_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t12  = pow(1.0 / t3 * t8 + 1, 2);
    return -2 * Omega__XO * t7 / t12 / t3 / t2;
  }

  real_type
  MidLine::alpha__r_D_3_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t8   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t10  = t4 * t8 + 1;
    real_type t13  = t3 * t3;
    real_type t16  = t10 * t10;
    return -1.0 / t10 * t4 * Omega__XO + 2 * Omega__XO / t16 / t13 * t8;
  }

  real_type
  MidLine::alpha__r_D_3_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t7   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t9   = t4 * t7 + 1;
    real_type t12  = t3 * t3;
    real_type t15  = t9 * t9;
    return -1.0 / t9 * t4 + 2 / t15 / t12 * t7;
  }

  real_type
  MidLine::alpha__r_D_3_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t12  = pow(1.0 / t3 * t8 + 1, 2);
    return 2 * t7 / t12 / t3 / t2;
  }

  real_type
  MidLine::alpha__r_D_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t7   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t8   = t2 * t2;
    return 1.0 / (1.0 / t8 * t7 + 1) / t2 * Omega__XO;
  }

  real_type
  MidLine::alpha__r_D_4_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t1   = Omega__XO * Omega__XO;
    real_type t3   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t4   = t3 * t3;
    real_type t9   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t10  = t9 * t9;
    real_type t14  = pow(1.0 / t4 * t10 + 1, 2);
    return 2 * t9 / t14 / t4 / t3 * t1;
  }

  real_type
  MidLine::alpha__r_D_4_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  MidLine::alpha__r_D_4_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t8   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t10  = t4 * t8 + 1;
    real_type t13  = t3 * t3;
    real_type t16  = t10 * t10;
    return 1.0 / t10 * t4 * Omega__XO - 2 * Omega__XO / t16 / t13 * t8;
  }

  real_type
  MidLine::alpha__r_D_4_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t12  = pow(1.0 / t3 * t8 + 1, 2);
    return -2 * Omega__XO * t7 / t12 / t3 / t2;
  }

  real_type
  MidLine::alpha__r_D_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t5   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t6   = 1.0 / t5;
    real_type t8   = t2 * t2;
    return -1.0 / (t6 * t8 + 1) * Omega__XO * t6 * t2;
  }

  real_type
  MidLine::alpha__r_D_5_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  MidLine::alpha__r_D_5_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  MidLine::alpha__r_D_5_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t8   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t10  = t4 * t8 + 1;
    real_type t13  = t3 * t3;
    real_type t16  = t10 * t10;
    return -1.0 / t10 * t4 * Omega__XO + 2 * Omega__XO / t16 / t13 * t8;
  }

  real_type
  MidLine::alpha__r_D_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t5   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t6   = 1.0 / t5;
    real_type t8   = t2 * t2;
    return -1.0 / (t6 * t8 + 1) * t6 * t2;
  }

  real_type
  MidLine::alpha__r_D_6_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  MidLine::alpha__r_D_6_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t7   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t9   = t4 * t7 + 1;
    real_type t12  = t3 * t3;
    real_type t15  = t9 * t9;
    return -1.0 / t9 * t4 + 2 / t15 / t12 * t7;
  }

  real_type
  MidLine::alpha__r_D_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t6   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t7   = t2 * t2;
    return -1.0 / (1.0 / t7 * t6 + 1) / t2;
  }

  real_type
  MidLine::alpha__r_D_7_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t12  = pow(1.0 / t3 * t8 + 1, 2);
    return 2 * t7 / t12 / t3 / t2;
  }

  real_type
  MidLine::alpha__f( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t2   = y__f__XO * Omega__XO;
    real_type t5   = x__f__XO * Omega__XO;
    real_type t12  = atan(1.0 / (-t2 + u__XO + x__f__dot__XO + delta__f__XO * (t5 + v__XO + y__f__dot__XO)) * (-x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t2 - u__XO) * delta__f__XO + t5 + v__XO));
    return -t12;
  }

  real_type
  MidLine::alpha__f_D_1( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_1_1( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_1_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_1_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_1_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_1_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_1_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_1_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_1_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_2_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_2_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_2_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_2_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_2_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_2_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_2_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_3_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_3_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_3_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_3_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_3_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_3_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_4_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_4_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_4_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_4_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_4_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_5_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
    real_type t26  = Omega__XO * delta__f__XO;
    real_type t29  = t8 * t14;
    real_type t32  = t23 * t23;
    return -1.0 / t23 * (2 * t1 * t16 * t14 + 2 * delta__f__XO * t8 * t1) + (2 * Omega__XO * t16 * t21 + 2 * t26 * t29) / t32 * (1.0 / t6 * t26 + Omega__XO * t29);
  }

  real_type
  MidLine::alpha__f_D_5_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = y__f__XO * Omega__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO + y__f__dot__XO;
    real_type t5   = delta__f__XO * t3 - t1 + u__XO + x__f__dot__XO;
    real_type t6   = 1.0 / t5;
    real_type t8   = Omega__XO * delta__f__XO;
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
  MidLine::alpha__f_D_5_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_5_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
    real_type t30  = t7 * t15;
    real_type t33  = t25 * t25;
    return -1.0 / t25 * (-2 * t19 * t17 * t15 - Omega__XO * t7 * t9 + t7 * Omega__XO) + (-2 * delta__f__XO * t17 * t23 + 2 * t30) / t33 * (1.0 / t5 * t19 + Omega__XO * t30);
  }

  real_type
  MidLine::alpha__f_D_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_6_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_6_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_6_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_7_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_7_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::alpha__f_D_8_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  MidLine::lambda__r( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t4   = ModelPars[121];
    real_type t6   = cos(phi__XO);
    return (-x__r__dot__XO - t6 * omega__r__XO * t4 + (-ModelPars[119] + t4) * omega__r__XO - t1 + u__XO) / (t1 - u__XO + x__r__dot__XO);
  }

  real_type
  MidLine::lambda__r_D_1( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t5   = ModelPars[121];
    real_type t7   = cos(phi__XO);
    return -y__r__XO * (-x__r__dot__XO - t7 * omega__r__XO * t5 + (-ModelPars[119] + t5) * omega__r__XO - t1 + u__XO) / t3 - y__r__XO / t2;
  }

  real_type
  MidLine::lambda__r_D_1_1( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[121];
    real_type t8   = cos(phi__XO);
    real_type t15  = y__r__XO * y__r__XO;
    return 2 * t15 * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[119] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 * t15 / t3;
  }

  real_type
  MidLine::lambda__r_D_1_2( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t7   = sin(phi__XO);
    return -y__r__XO * t7 * omega__r__XO * ModelPars[121] / t3;
  }

  real_type
  MidLine::lambda__r_D_1_3( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[121];
    real_type t8   = cos(phi__XO);
    return -2 * y__r__XO * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[119] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 - 2 * y__r__XO / t3;
  }

  real_type
  MidLine::lambda__r_D_1_4( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[121];
    real_type t8   = cos(phi__XO);
    real_type t13  = -x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[119] + t6) * omega__r__XO - t1 + u__XO;
    real_type t17  = 1.0 / t3;
    return 2 * t1 * t13 / t3 / t2 + 2 * y__r__XO * Omega__XO * t17 - t13 * t17 - 1.0 / t2;
  }

  real_type
  MidLine::lambda__r_D_1_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t5   = ModelPars[121];
    real_type t6   = cos(phi__XO);
    return -y__r__XO * (-t6 * t5 + t5 - ModelPars[119]) / t3;
  }

  real_type
  MidLine::lambda__r_D_1_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[121];
    real_type t8   = cos(phi__XO);
    return 2 * y__r__XO * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[119] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 * y__r__XO / t3;
  }

  real_type
  MidLine::lambda__r_D_2( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t6   = sin(phi__XO);
    return t6 * omega__r__XO / (y__r__XO * Omega__XO - u__XO + x__r__dot__XO) * ModelPars[121];
  }

  real_type
  MidLine::lambda__r_D_2_2( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t6   = cos(phi__XO);
    return t6 * omega__r__XO / (y__r__XO * Omega__XO - u__XO + x__r__dot__XO) * ModelPars[121];
  }

  real_type
  MidLine::lambda__r_D_2_3( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t7   = sin(phi__XO);
    return t7 * omega__r__XO * ModelPars[121] / t3;
  }

  real_type
  MidLine::lambda__r_D_2_4( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t7   = sin(phi__XO);
    return -Omega__XO * t7 * omega__r__XO * ModelPars[121] / t3;
  }

  real_type
  MidLine::lambda__r_D_2_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t6   = sin(phi__XO);
    return t6 * ModelPars[121] / (y__r__XO * Omega__XO - u__XO + x__r__dot__XO);
  }

  real_type
  MidLine::lambda__r_D_2_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t7   = sin(phi__XO);
    return -t7 * omega__r__XO * ModelPars[121] / t3;
  }

  real_type
  MidLine::lambda__r_D_3( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t5   = ModelPars[121];
    real_type t7   = cos(phi__XO);
    return (-x__r__dot__XO - t7 * omega__r__XO * t5 + (-ModelPars[119] + t5) * omega__r__XO - t1 + u__XO) / t3 + 1.0 / t2;
  }

  real_type
  MidLine::lambda__r_D_3_3( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[121];
    real_type t8   = cos(phi__XO);
    return 2 * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[119] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 / t3;
  }

  real_type
  MidLine::lambda__r_D_3_4( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[121];
    real_type t8   = cos(phi__XO);
    return -2 * Omega__XO * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[119] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 - 2 * Omega__XO / t3;
  }

  real_type
  MidLine::lambda__r_D_3_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t5   = ModelPars[121];
    real_type t6   = cos(phi__XO);
    return (-t6 * t5 + t5 - ModelPars[119]) / t3;
  }

  real_type
  MidLine::lambda__r_D_3_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[121];
    real_type t8   = cos(phi__XO);
    return -2 * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[119] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 - 2 / t3;
  }

  real_type
  MidLine::lambda__r_D_4( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t5   = ModelPars[121];
    real_type t7   = cos(phi__XO);
    return -Omega__XO * (-x__r__dot__XO - t7 * omega__r__XO * t5 + (-ModelPars[119] + t5) * omega__r__XO - t1 + u__XO) / t3 - Omega__XO / t2;
  }

  real_type
  MidLine::lambda__r_D_4_4( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[121];
    real_type t8   = cos(phi__XO);
    real_type t15  = Omega__XO * Omega__XO;
    return 2 * t15 * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[119] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 * t15 / t3;
  }

  real_type
  MidLine::lambda__r_D_4_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t5   = ModelPars[121];
    real_type t6   = cos(phi__XO);
    return -Omega__XO * (-t6 * t5 + t5 - ModelPars[119]) / t3;
  }

  real_type
  MidLine::lambda__r_D_4_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[121];
    real_type t8   = cos(phi__XO);
    return 2 * Omega__XO * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[119] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 * Omega__XO / t3;
  }

  real_type
  MidLine::lambda__r_D_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t4   = ModelPars[121];
    real_type t5   = cos(phi__XO);
    return (-t5 * t4 + t4 - ModelPars[119]) / (y__r__XO * Omega__XO - u__XO + x__r__dot__XO);
  }

  real_type
  MidLine::lambda__r_D_5_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    return 0;
  }

  real_type
  MidLine::lambda__r_D_5_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t5   = ModelPars[121];
    real_type t6   = cos(phi__XO);
    return -(-t6 * t5 + t5 - ModelPars[119]) / t3;
  }

  real_type
  MidLine::lambda__r_D_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t5   = ModelPars[121];
    real_type t7   = cos(phi__XO);
    return -(-x__r__dot__XO - t7 * omega__r__XO * t5 + (-ModelPars[119] + t5) * omega__r__XO - t1 + u__XO) / t3 - 1.0 / t2;
  }

  real_type
  MidLine::lambda__r_D_6_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[121];
    real_type t8   = cos(phi__XO);
    return 2 * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[119] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 / t3;
  }

  real_type
  MidLine::lambda__f( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t8   = ModelPars[120];
    real_type t10  = cos(phi__f__XO);
    return (-x__f__dot__XO - t1 + t10 * omega__f__XO * t8 - t3 * delta__f__XO + t5 + (ModelPars[118] - t8) * omega__f__XO - u__XO) / (t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO);
  }

  real_type
  MidLine::lambda__f_D_1( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[120];
    real_type t11  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * x__f__XO - y__f__XO;
    return -t21 * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[118] - t9) * omega__f__XO - u__XO) / t7 - t21 / t6;
  }

  real_type
  MidLine::lambda__f_D_1_1( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t23  = t22 * t22;
    return 2 * t23 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * t22 / t7;
  }

  real_type
  MidLine::lambda__f_D_1_2( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return (delta__f__XO * x__f__XO - y__f__XO) * t11 * omega__f__XO * ModelPars[120] / t7;
  }

  real_type
  MidLine::lambda__f_D_1_3( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t25  = 1.0 / t7;
    return 2 * t22 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * t25;
  }

  real_type
  MidLine::lambda__f_D_1_4( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t26  = 1.0 / t7;
    return 2 * delta__f__XO * t22 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * delta__f__XO * t26;
  }

  real_type
  MidLine::lambda__f_D_1_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO;
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t27  = 1.0 / t7;
    return 2 * delta__f__XO * Omega__XO * t22 * t19 / t7 / t6 + 2 * t22 * delta__f__XO * Omega__XO * t27 - delta__f__XO * t19 * t27 - delta__f__XO / t6;
  }

  real_type
  MidLine::lambda__f_D_1_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO;
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t26  = 1.0 / t7;
    return -2 * Omega__XO * t22 * t19 / t7 / t6 - 2 * t22 * Omega__XO * t26 + t19 * t26 + 1.0 / t6;
  }

  real_type
  MidLine::lambda__f_D_1_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO;
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t23  = t2 + v__XO + y__f__dot__XO;
    real_type t27  = 1.0 / t7;
    return 2 * t23 * t22 * t19 / t7 / t6 + 2 * t22 * t23 * t27 - x__f__XO * t19 * t27 - x__f__XO / t6;
  }

  real_type
  MidLine::lambda__f_D_1_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[120];
    real_type t10  = cos(phi__f__XO);
    return -(delta__f__XO * x__f__XO - y__f__XO) * (t10 * t9 - t9 + ModelPars[118]) / t7;
  }

  real_type
  MidLine::lambda__f_D_1_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t25  = 1.0 / t7;
    return 2 * t22 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * t25;
  }

  real_type
  MidLine::lambda__f_D_1_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t26  = 1.0 / t7;
    return 2 * delta__f__XO * t22 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * delta__f__XO * t26;
  }

  real_type
  MidLine::lambda__f_D_2( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t10  = sin(phi__f__XO);
    return -t10 * omega__f__XO / (x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO) * ModelPars[120];
  }

  real_type
  MidLine::lambda__f_D_2_2( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t10  = cos(phi__f__XO);
    return -t10 * omega__f__XO / (x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO) * ModelPars[120];
  }

  real_type
  MidLine::lambda__f_D_2_3( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return t11 * omega__f__XO * ModelPars[120] / t7;
  }

  real_type
  MidLine::lambda__f_D_2_4( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return delta__f__XO * t11 * omega__f__XO * ModelPars[120] / t7;
  }

  real_type
  MidLine::lambda__f_D_2_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t12  = sin(phi__f__XO);
    return delta__f__XO * Omega__XO * t12 * omega__f__XO * ModelPars[120] / t7;
  }

  real_type
  MidLine::lambda__f_D_2_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return -Omega__XO * t11 * omega__f__XO * ModelPars[120] / t7;
  }

  real_type
  MidLine::lambda__f_D_2_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t2   = x__f__XO * Omega__XO;
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + (t2 + v__XO) * delta__f__XO - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return (t2 + v__XO + y__f__dot__XO) * t11 * omega__f__XO * ModelPars[120] / t7;
  }

  real_type
  MidLine::lambda__f_D_2_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t10  = sin(phi__f__XO);
    return -t10 * ModelPars[120] / (x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO);
  }

  real_type
  MidLine::lambda__f_D_2_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return t11 * omega__f__XO * ModelPars[120] / t7;
  }

  real_type
  MidLine::lambda__f_D_2_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return delta__f__XO * t11 * omega__f__XO * ModelPars[120] / t7;
  }

  real_type
  MidLine::lambda__f_D_3( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[120];
    real_type t11  = cos(phi__f__XO);
    return -(-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[118] - t9) * omega__f__XO - u__XO) / t7 - 1.0 / t6;
  }

  real_type
  MidLine::lambda__f_D_3_3( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    return 2 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 / t7;
  }

  real_type
  MidLine::lambda__f_D_3_4( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    return 2 * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO / t7;
  }

  real_type
  MidLine::lambda__f_D_3_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    return 2 * Omega__XO * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO * Omega__XO / t7;
  }

  real_type
  MidLine::lambda__f_D_3_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    return -2 * Omega__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * Omega__XO / t7;
  }

  real_type
  MidLine::lambda__f_D_3_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t24  = 1.0 / t7;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 * t24;
  }

  real_type
  MidLine::lambda__f_D_3_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[120];
    real_type t10  = cos(phi__f__XO);
    return -(t10 * t9 - t9 + ModelPars[118]) / t7;
  }

  real_type
  MidLine::lambda__f_D_3_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    return 2 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 / t7;
  }

  real_type
  MidLine::lambda__f_D_3_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    return 2 * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO / t7;
  }

  real_type
  MidLine::lambda__f_D_4( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[120];
    real_type t11  = cos(phi__f__XO);
    return -delta__f__XO * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[118] - t9) * omega__f__XO - u__XO) / t7 - delta__f__XO / t6;
  }

  real_type
  MidLine::lambda__f_D_4_4( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * delta__f__XO;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 / t7;
  }

  real_type
  MidLine::lambda__f_D_4_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * delta__f__XO;
    return 2 * Omega__XO * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 * Omega__XO / t7;
  }

  real_type
  MidLine::lambda__f_D_4_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    return -2 * Omega__XO * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * delta__f__XO * Omega__XO / t7;
  }

  real_type
  MidLine::lambda__f_D_4_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO;
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t25  = 1.0 / t7;
    return 2 * t21 * delta__f__XO * t19 / t7 / t6 + 2 * delta__f__XO * t21 * t25 - t19 * t25 - 1.0 / t6;
  }

  real_type
  MidLine::lambda__f_D_4_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[120];
    real_type t10  = cos(phi__f__XO);
    return -delta__f__XO * (t10 * t9 - t9 + ModelPars[118]) / t7;
  }

  real_type
  MidLine::lambda__f_D_4_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    return 2 * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO / t7;
  }

  real_type
  MidLine::lambda__f_D_4_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * delta__f__XO;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 / t7;
  }

  real_type
  MidLine::lambda__f_D_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[120];
    real_type t11  = cos(phi__f__XO);
    return -Omega__XO * delta__f__XO * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[118] - t9) * omega__f__XO - u__XO) / t7 - delta__f__XO * Omega__XO / t6;
  }

  real_type
  MidLine::lambda__f_D_5_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = Omega__XO * Omega__XO;
    real_type t22  = delta__f__XO * delta__f__XO;
    return 2 * t22 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * t21 / t7;
  }

  real_type
  MidLine::lambda__f_D_5_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = Omega__XO * Omega__XO;
    return -2 * delta__f__XO * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * delta__f__XO * t21 / t7;
  }

  real_type
  MidLine::lambda__f_D_5_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO;
    real_type t21  = delta__f__XO * Omega__XO;
    real_type t22  = t2 + v__XO + y__f__dot__XO;
    real_type t26  = 1.0 / t7;
    return 2 * t22 * t21 * t19 / t7 / t6 + t21 * t22 * t26 - Omega__XO * t19 * t26 + t22 * delta__f__XO * Omega__XO * t26 - Omega__XO / t6;
  }

  real_type
  MidLine::lambda__f_D_5_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[120];
    real_type t10  = cos(phi__f__XO);
    return -Omega__XO * delta__f__XO * (t10 * t9 - t9 + ModelPars[118]) / t7;
  }

  real_type
  MidLine::lambda__f_D_5_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    return 2 * Omega__XO * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO * Omega__XO / t7;
  }

  real_type
  MidLine::lambda__f_D_5_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * delta__f__XO;
    return 2 * Omega__XO * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 * Omega__XO / t7;
  }

  real_type
  MidLine::lambda__f_D_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[120];
    real_type t11  = cos(phi__f__XO);
    return Omega__XO * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[118] - t9) * omega__f__XO - u__XO) / t7 + Omega__XO / t6;
  }

  real_type
  MidLine::lambda__f_D_6_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = Omega__XO * Omega__XO;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 / t7;
  }

  real_type
  MidLine::lambda__f_D_6_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t25  = 1.0 / t7;
    return -2 * t21 * Omega__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * Omega__XO * t21 * t25;
  }

  real_type
  MidLine::lambda__f_D_6_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[120];
    real_type t10  = cos(phi__f__XO);
    return Omega__XO * (t10 * t9 - t9 + ModelPars[118]) / t7;
  }

  real_type
  MidLine::lambda__f_D_6_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    return -2 * Omega__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * Omega__XO / t7;
  }

  real_type
  MidLine::lambda__f_D_6_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    return -2 * Omega__XO * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * delta__f__XO * Omega__XO / t7;
  }

  real_type
  MidLine::lambda__f_D_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[120];
    real_type t11  = cos(phi__f__XO);
    real_type t20  = t2 + v__XO + y__f__dot__XO;
    return -t20 * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[118] - t9) * omega__f__XO - u__XO) / t7 - t20 / t6;
  }

  real_type
  MidLine::lambda__f_D_7_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t22  = t21 * t21;
    return 2 * t22 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 * t21 / t7;
  }

  real_type
  MidLine::lambda__f_D_7_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t2   = x__f__XO * Omega__XO;
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + (t2 + v__XO) * delta__f__XO - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[120];
    real_type t10  = cos(phi__f__XO);
    return -(t2 + v__XO + y__f__dot__XO) * (t10 * t9 - t9 + ModelPars[118]) / t7;
  }

  real_type
  MidLine::lambda__f_D_7_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t24  = 1.0 / t7;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 * t24;
  }

  real_type
  MidLine::lambda__f_D_7_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO;
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t25  = 1.0 / t7;
    return 2 * t21 * delta__f__XO * t19 / t7 / t6 + 2 * delta__f__XO * t21 * t25 - t19 * t25 - 1.0 / t6;
  }

  real_type
  MidLine::lambda__f_D_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t8   = ModelPars[120];
    real_type t9   = cos(phi__f__XO);
    return (t9 * t8 - t8 + ModelPars[118]) / (x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO);
  }

  real_type
  MidLine::lambda__f_D_8_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    return 0;
  }

  real_type
  MidLine::lambda__f_D_8_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[120];
    real_type t10  = cos(phi__f__XO);
    return -(t10 * t9 - t9 + ModelPars[118]) / t7;
  }

  real_type
  MidLine::lambda__f_D_8_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[120];
    real_type t10  = cos(phi__f__XO);
    return -delta__f__XO * (t10 * t9 - t9 + ModelPars[118]) / t7;
  }

  real_type
  MidLine::lambda__f_D_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[120];
    real_type t11  = cos(phi__f__XO);
    return -(-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[118] - t9) * omega__f__XO - u__XO) / t7 - 1.0 / t6;
  }

  real_type
  MidLine::lambda__f_D_9_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    return 2 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 / t7;
  }

  real_type
  MidLine::lambda__f_D_9_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    return 2 * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO / t7;
  }

  real_type
  MidLine::lambda__f_D_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[120];
    real_type t11  = cos(phi__f__XO);
    return -delta__f__XO * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[118] - t9) * omega__f__XO - u__XO) / t7 - delta__f__XO / t6;
  }

  real_type
  MidLine::lambda__f_D_10_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[120];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * delta__f__XO;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[118] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 / t7;
  }

  real_type
  MidLine::Fxf( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t11  = Fzf__XO * ModelPars[244] * (t5 * t3 * ModelPars[83] + ModelPars[81]);
    real_type t14  = ModelPars[79] * ModelPars[179];
    real_type t24  = exp(t5 * t3 * ModelPars[89]);
    real_type t34  = atan(lambda__f__XO / (t11 * t14 + ModelPars[172]) * ModelPars[181] * t24 * (t5 * t3 * ModelPars[87] + ModelPars[85]) * Fzf__XO);
    real_type t36  = sin(t34 * t14);
    real_type t37  = ModelPars[106];
    real_type t42  = lambda__f__XO * lambda__f__XO;
    real_type t44  = ModelPars[96] * ModelPars[96];
    real_type t47  = sqrt(t44 * t42 + 1);
    real_type t49  = 1.0 / t47 * (phi__f__XO * ModelPars[98] + ModelPars[94]);
    real_type t50  = ModelPars[110];
    real_type t53  = atan((alpha__f__XO + t50) * t49);
    real_type t55  = cos(t53 * t37);
    real_type t58  = atan(t50 * t49);
    real_type t60  = cos(t58 * t37);
    return 1.0 / t60 * t55 * t36 * t11;
  }

  real_type
  MidLine::Fxf_D_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[83];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[244];
    real_type t9   = ModelPars[79] * ModelPars[179];
    real_type t10  = ModelPars[87];
    real_type t11  = Fzf__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[85];
    real_type t16  = t15 * Fzf__XO;
    real_type t17  = ModelPars[89];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[181];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[81]);
    real_type t29  = Fzf__XO * t27 * t9;
    real_type t31  = t29 + ModelPars[172];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__f__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t40  = ModelPars[106];
    real_type t45  = lambda__f__XO * lambda__f__XO;
    real_type t47  = ModelPars[96] * ModelPars[96];
    real_type t50  = sqrt(t47 * t45 + 1);
    real_type t52  = 1.0 / t50 * (phi__f__XO * ModelPars[98] + ModelPars[94]);
    real_type t53  = ModelPars[110];
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
  MidLine::Fxf_D_1_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[83];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t4   = t3 * t1;
    real_type t5   = ModelPars[244];
    real_type t6   = t5 * t4;
    real_type t7   = ModelPars[79];
    real_type t8   = ModelPars[179];
    real_type t9   = t8 * t7;
    real_type t10  = ModelPars[87];
    real_type t11  = Fzf__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[85];
    real_type t16  = t15 * Fzf__XO;
    real_type t17  = ModelPars[89];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[181];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[81]);
    real_type t29  = Fzf__XO * t27 * t9;
    real_type t31  = t29 + ModelPars[172];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__f__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t39  = ModelPars[106];
    real_type t44  = lambda__f__XO * lambda__f__XO;
    real_type t46  = ModelPars[96] * ModelPars[96];
    real_type t49  = sqrt(t46 * t44 + 1);
    real_type t51  = 1.0 / t49 * (phi__f__XO * ModelPars[98] + ModelPars[94]);
    real_type t52  = ModelPars[110];
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
  MidLine::Fxf_D_1_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[83];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[244];
    real_type t6   = t5 * t3 * t1;
    real_type t7   = ModelPars[79];
    real_type t8   = ModelPars[179];
    real_type t9   = t8 * t7;
    real_type t10  = ModelPars[87];
    real_type t11  = Fzf__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[85];
    real_type t16  = t15 * Fzf__XO;
    real_type t17  = ModelPars[89];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[181];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[81]);
    real_type t28  = Fzf__XO * t27;
    real_type t31  = t28 * t9 + ModelPars[172];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__f__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t39  = t38 * Fzf__XO;
    real_type t40  = ModelPars[106];
    real_type t43  = ModelPars[98];
    real_type t44  = lambda__f__XO * lambda__f__XO;
    real_type t46  = ModelPars[96] * ModelPars[96];
    real_type t48  = t46 * t44 + 1;
    real_type t49  = sqrt(t48);
    real_type t50  = 1.0 / t49;
    real_type t51  = t50 * t43;
    real_type t52  = ModelPars[110];
    real_type t53  = alpha__f__XO + t52;
    real_type t57  = t43 * phi__f__XO + ModelPars[94];
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
  MidLine::Fxf_D_1_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[83];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[244];
    real_type t9   = ModelPars[79] * ModelPars[179];
    real_type t10  = ModelPars[87];
    real_type t11  = Fzf__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[85];
    real_type t16  = t15 * Fzf__XO;
    real_type t17  = ModelPars[89];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[181];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[81]);
    real_type t28  = Fzf__XO * t27;
    real_type t31  = t28 * t9 + ModelPars[172];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__f__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t41  = ModelPars[106];
    real_type t45  = phi__f__XO * ModelPars[98] + ModelPars[94];
    real_type t47  = lambda__f__XO * lambda__f__XO;
    real_type t49  = ModelPars[96] * ModelPars[96];
    real_type t51  = t49 * t47 + 1;
    real_type t52  = sqrt(t51);
    real_type t53  = 1.0 / t52;
    real_type t55  = t45 * t45;
    real_type t58  = ModelPars[110];
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
  MidLine::Fxf_D_1_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[83];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[244];
    real_type t6   = t5 * t3 * t1;
    real_type t7   = Fzf__XO * Fzf__XO;
    real_type t8   = ModelPars[79];
    real_type t10  = ModelPars[179];
    real_type t11  = ModelPars[87];
    real_type t12  = Fzf__XO - t2;
    real_type t16  = t3 * t12 * t11 + ModelPars[85];
    real_type t20  = ModelPars[89];
    real_type t23  = exp(t3 * t12 * t20);
    real_type t24  = ModelPars[181];
    real_type t25  = t24 * t23;
    real_type t26  = t10 * t8;
    real_type t31  = t5 * (t3 * t12 * t1 + ModelPars[81]);
    real_type t33  = Fzf__XO * t31 * t26;
    real_type t35  = t33 + ModelPars[172];
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
    real_type t60  = ModelPars[106];
    real_type t64  = phi__f__XO * ModelPars[98] + ModelPars[94];
    real_type t66  = ModelPars[96] * ModelPars[96];
    real_type t68  = t66 * t46 + 1;
    real_type t69  = sqrt(t68);
    real_type t71  = 1.0 / t69 * t64;
    real_type t72  = ModelPars[110];
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
  MidLine::Fxf_D_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[83] + ModelPars[81]);
    real_type t13  = ModelPars[79] * ModelPars[179];
    real_type t23  = exp(t5 * t3 * ModelPars[89]);
    real_type t26  = t10 * Fzf__XO;
    real_type t34  = atan(lambda__f__XO / (t26 * t13 + ModelPars[172]) * ModelPars[181] * t23 * (t5 * t3 * ModelPars[87] + ModelPars[85]) * Fzf__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[106];
    real_type t41  = ModelPars[98];
    real_type t42  = lambda__f__XO * lambda__f__XO;
    real_type t44  = ModelPars[96] * ModelPars[96];
    real_type t46  = t44 * t42 + 1;
    real_type t47  = sqrt(t46);
    real_type t48  = 1.0 / t47;
    real_type t50  = ModelPars[110];
    real_type t51  = alpha__f__XO + t50;
    real_type t55  = t41 * phi__f__XO + ModelPars[94];
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
  MidLine::Fxf_D_2_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[83] + ModelPars[81]);
    real_type t11  = t10 * Fzf__XO;
    real_type t14  = ModelPars[79] * ModelPars[179];
    real_type t24  = exp(t5 * t3 * ModelPars[89]);
    real_type t34  = atan(lambda__f__XO / (t11 * t14 + ModelPars[172]) * ModelPars[181] * t24 * (t5 * t3 * ModelPars[87] + ModelPars[85]) * Fzf__XO);
    real_type t36  = sin(t34 * t14);
    real_type t37  = ModelPars[106];
    real_type t39  = ModelPars[98];
    real_type t40  = t39 * t39;
    real_type t43  = lambda__f__XO * lambda__f__XO;
    real_type t45  = ModelPars[96] * ModelPars[96];
    real_type t47  = t45 * t43 + 1;
    real_type t48  = sqrt(t47);
    real_type t50  = 1.0 / t48 / t47;
    real_type t51  = ModelPars[110];
    real_type t52  = alpha__f__XO + t51;
    real_type t53  = t52 * t52;
    real_type t58  = t39 * phi__f__XO + ModelPars[94];
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
  MidLine::Fxf_D_2_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[83] + ModelPars[81]);
    real_type t13  = ModelPars[79] * ModelPars[179];
    real_type t23  = exp(t5 * t3 * ModelPars[89]);
    real_type t26  = t10 * Fzf__XO;
    real_type t34  = atan(lambda__f__XO / (t26 * t13 + ModelPars[172]) * ModelPars[181] * t23 * (t5 * t3 * ModelPars[87] + ModelPars[85]) * Fzf__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[106];
    real_type t41  = ModelPars[98];
    real_type t42  = lambda__f__XO * lambda__f__XO;
    real_type t44  = ModelPars[96] * ModelPars[96];
    real_type t46  = t44 * t42 + 1;
    real_type t47  = sqrt(t46);
    real_type t48  = 1.0 / t47;
    real_type t52  = t41 * phi__f__XO + ModelPars[94];
    real_type t53  = t52 * t52;
    real_type t54  = 1.0 / t46;
    real_type t55  = t54 * t53;
    real_type t56  = ModelPars[110];
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
  MidLine::Fxf_D_2_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[83] + ModelPars[81]);
    real_type t11  = Fzf__XO * Fzf__XO;
    real_type t12  = ModelPars[79];
    real_type t15  = ModelPars[179];
    real_type t20  = t5 * t3 * ModelPars[87] + ModelPars[85];
    real_type t25  = exp(t5 * t3 * ModelPars[89]);
    real_type t26  = ModelPars[181];
    real_type t28  = t15 * t12;
    real_type t29  = t10 * Fzf__XO;
    real_type t32  = t28 * t29 + ModelPars[172];
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
    real_type t59  = ModelPars[106];
    real_type t60  = ModelPars[98];
    real_type t64  = ModelPars[96] * ModelPars[96];
    real_type t66  = t45 * t64 + 1;
    real_type t67  = sqrt(t66);
    real_type t68  = 1.0 / t67;
    real_type t69  = ModelPars[110];
    real_type t70  = alpha__f__XO + t69;
    real_type t74  = t60 * phi__f__XO + ModelPars[94];
    real_type t75  = t74 * t74;
    real_type t77  = 1.0 / t66 * t75;
    real_type t78  = t70 * t70;
    real_type t80  = t77 * t78 + 1;
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
  MidLine::Fxf_D_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[83] + ModelPars[81]);
    real_type t13  = ModelPars[79] * ModelPars[179];
    real_type t23  = exp(t5 * t3 * ModelPars[89]);
    real_type t34  = atan(lambda__f__XO / (Fzf__XO * t10 * t13 + ModelPars[172]) * ModelPars[181] * t23 * (t5 * t3 * ModelPars[87] + ModelPars[85]) * Fzf__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[106];
    real_type t44  = phi__f__XO * ModelPars[98] + ModelPars[94];
    real_type t45  = lambda__f__XO * lambda__f__XO;
    real_type t47  = ModelPars[96] * ModelPars[96];
    real_type t49  = t47 * t45 + 1;
    real_type t50  = sqrt(t49);
    real_type t52  = 1.0 / t50 * t44;
    real_type t53  = t44 * t44;
    real_type t56  = ModelPars[110];
    real_type t57  = alpha__f__XO + t56;
    real_type t58  = t57 * t57;
    real_type t63  = atan(t57 * t52);
    real_type t65  = sin(t63 * t38);
    real_type t68  = atan(t56 * t52);
    real_type t70  = cos(t68 * t38);
    return -1.0 / t70 * t65 / (t58 / t49 * t53 + 1) * t52 * t38 * t36 * Fzf__XO * t10;
  }

  real_type
  MidLine::Fxf_D_3_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[83] + ModelPars[81]);
    real_type t13  = ModelPars[79] * ModelPars[179];
    real_type t23  = exp(t5 * t3 * ModelPars[89]);
    real_type t34  = atan(lambda__f__XO / (Fzf__XO * t10 * t13 + ModelPars[172]) * ModelPars[181] * t23 * (t5 * t3 * ModelPars[87] + ModelPars[85]) * Fzf__XO);
    real_type t36  = sin(t34 * t13);
    real_type t37  = t36 * Fzf__XO;
    real_type t38  = ModelPars[106];
    real_type t44  = phi__f__XO * ModelPars[98] + ModelPars[94];
    real_type t45  = t44 * t44;
    real_type t47  = lambda__f__XO * lambda__f__XO;
    real_type t49  = ModelPars[96] * ModelPars[96];
    real_type t51  = t49 * t47 + 1;
    real_type t52  = sqrt(t51);
    real_type t57  = 1.0 / t51 * t45;
    real_type t58  = ModelPars[110];
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
  MidLine::Fxf_D_3_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[83] + ModelPars[81]);
    real_type t11  = Fzf__XO * Fzf__XO;
    real_type t12  = ModelPars[79];
    real_type t15  = ModelPars[179];
    real_type t20  = t5 * t3 * ModelPars[87] + ModelPars[85];
    real_type t25  = exp(t5 * t3 * ModelPars[89]);
    real_type t26  = ModelPars[181];
    real_type t30  = t15 * t12;
    real_type t31  = t10 * Fzf__XO;
    real_type t34  = t31 * t30 + ModelPars[172];
    real_type t35  = 1.0 / t34;
    real_type t36  = t20 * t20;
    real_type t38  = t25 * t25;
    real_type t40  = t26 * t26;
    real_type t41  = t34 * t34;
    real_type t44  = lambda__f__XO * lambda__f__XO;
    real_type t55  = atan(lambda__f__XO * t35 * t26 * t25 * t20 * Fzf__XO);
    real_type t56  = t55 * t30;
    real_type t57  = cos(t56);
    real_type t58  = ModelPars[106];
    real_type t64  = phi__f__XO * ModelPars[98] + ModelPars[94];
    real_type t66  = ModelPars[96] * ModelPars[96];
    real_type t68  = t66 * t44 + 1;
    real_type t69  = sqrt(t68);
    real_type t71  = 1.0 / t69 * t64;
    real_type t72  = t64 * t64;
    real_type t74  = 1.0 / t68 * t72;
    real_type t75  = ModelPars[110];
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
  MidLine::Fxf_D_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[83] + ModelPars[81]);
    real_type t11  = Fzf__XO * Fzf__XO;
    real_type t15  = ModelPars[79] * ModelPars[179];
    real_type t20  = t5 * t3 * ModelPars[87] + ModelPars[85];
    real_type t26  = exp(t5 * t3 * ModelPars[89]);
    real_type t27  = ModelPars[181];
    real_type t29  = t10 * Fzf__XO;
    real_type t32  = t29 * t15 + ModelPars[172];
    real_type t33  = 1.0 / t32;
    real_type t35  = t20 * t20;
    real_type t37  = t26 * t26;
    real_type t39  = t27 * t27;
    real_type t40  = t32 * t32;
    real_type t43  = lambda__f__XO * lambda__f__XO;
    real_type t53  = atan(lambda__f__XO * t33 * t27 * t26 * t20 * Fzf__XO);
    real_type t54  = t53 * t15;
    real_type t55  = cos(t54);
    real_type t57  = ModelPars[106];
    real_type t61  = phi__f__XO * ModelPars[98] + ModelPars[94];
    real_type t63  = ModelPars[96] * ModelPars[96];
    real_type t65  = t63 * t43 + 1;
    real_type t66  = sqrt(t65);
    real_type t68  = 1.0 / t66 * t61;
    real_type t69  = ModelPars[110];
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
  MidLine::Fxf_D_4_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[83] + ModelPars[81]);
    real_type t11  = Fzf__XO * Fzf__XO;
    real_type t12  = t11 * t11;
    real_type t14  = ModelPars[79];
    real_type t15  = ModelPars[179];
    real_type t16  = t15 * t14;
    real_type t21  = t5 * t3 * ModelPars[87] + ModelPars[85];
    real_type t22  = t21 * t21;
    real_type t27  = exp(t5 * t3 * ModelPars[89]);
    real_type t28  = t27 * t27;
    real_type t33  = ModelPars[181];
    real_type t34  = t33 * t33;
    real_type t36  = t10 * Fzf__XO;
    real_type t39  = t36 * t16 + ModelPars[172];
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
    real_type t64  = ModelPars[106];
    real_type t68  = phi__f__XO * ModelPars[98] + ModelPars[94];
    real_type t70  = ModelPars[96] * ModelPars[96];
    real_type t72  = t70 * t48 + 1;
    real_type t73  = sqrt(t72);
    real_type t75  = 1.0 / t73 * t68;
    real_type t76  = ModelPars[110];
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
  MidLine::Fxr( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t11  = Fzr__XO * ModelPars[245] * (t5 * t3 * ModelPars[84] + ModelPars[82]);
    real_type t14  = ModelPars[80] * ModelPars[180];
    real_type t24  = exp(t5 * t3 * ModelPars[90]);
    real_type t34  = atan(lambda__r__XO / (t11 * t14 + ModelPars[173]) * ModelPars[182] * t24 * (t5 * t3 * ModelPars[88] + ModelPars[86]) * Fzr__XO);
    real_type t36  = sin(t34 * t14);
    real_type t37  = ModelPars[107];
    real_type t42  = lambda__r__XO * lambda__r__XO;
    real_type t44  = ModelPars[97] * ModelPars[97];
    real_type t47  = sqrt(t44 * t42 + 1);
    real_type t49  = 1.0 / t47 * (phi__XO * ModelPars[99] + ModelPars[95]);
    real_type t50  = ModelPars[111];
    real_type t53  = atan((alpha__r__XO + t50) * t49);
    real_type t55  = cos(t53 * t37);
    real_type t58  = atan(t50 * t49);
    real_type t60  = cos(t58 * t37);
    return 1.0 / t60 * t55 * t36 * t11;
  }

  real_type
  MidLine::Fxr_D_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[84];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[245];
    real_type t9   = ModelPars[80] * ModelPars[180];
    real_type t10  = ModelPars[88];
    real_type t11  = Fzr__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[86];
    real_type t16  = t15 * Fzr__XO;
    real_type t17  = ModelPars[90];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[182];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[82]);
    real_type t29  = Fzr__XO * t27 * t9;
    real_type t31  = t29 + ModelPars[173];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__r__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t40  = ModelPars[107];
    real_type t45  = lambda__r__XO * lambda__r__XO;
    real_type t47  = ModelPars[97] * ModelPars[97];
    real_type t50  = sqrt(t47 * t45 + 1);
    real_type t52  = 1.0 / t50 * (phi__XO * ModelPars[99] + ModelPars[95]);
    real_type t53  = ModelPars[111];
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
  MidLine::Fxr_D_1_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[84];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t4   = t3 * t1;
    real_type t5   = ModelPars[245];
    real_type t6   = t5 * t4;
    real_type t7   = ModelPars[80];
    real_type t8   = ModelPars[180];
    real_type t9   = t8 * t7;
    real_type t10  = ModelPars[88];
    real_type t11  = Fzr__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[86];
    real_type t16  = t15 * Fzr__XO;
    real_type t17  = ModelPars[90];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[182];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[82]);
    real_type t29  = Fzr__XO * t27 * t9;
    real_type t31  = t29 + ModelPars[173];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__r__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t39  = ModelPars[107];
    real_type t44  = lambda__r__XO * lambda__r__XO;
    real_type t46  = ModelPars[97] * ModelPars[97];
    real_type t49  = sqrt(t46 * t44 + 1);
    real_type t51  = 1.0 / t49 * (phi__XO * ModelPars[99] + ModelPars[95]);
    real_type t52  = ModelPars[111];
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
  MidLine::Fxr_D_1_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[84];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[245];
    real_type t6   = t5 * t3 * t1;
    real_type t7   = ModelPars[80];
    real_type t8   = ModelPars[180];
    real_type t9   = t8 * t7;
    real_type t10  = ModelPars[88];
    real_type t11  = Fzr__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[86];
    real_type t16  = t15 * Fzr__XO;
    real_type t17  = ModelPars[90];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[182];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[82]);
    real_type t28  = Fzr__XO * t27;
    real_type t31  = t28 * t9 + ModelPars[173];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__r__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t39  = t38 * Fzr__XO;
    real_type t40  = ModelPars[107];
    real_type t43  = ModelPars[99];
    real_type t44  = lambda__r__XO * lambda__r__XO;
    real_type t46  = ModelPars[97] * ModelPars[97];
    real_type t48  = t46 * t44 + 1;
    real_type t49  = sqrt(t48);
    real_type t50  = 1.0 / t49;
    real_type t51  = t50 * t43;
    real_type t52  = ModelPars[111];
    real_type t53  = alpha__r__XO + t52;
    real_type t57  = t43 * phi__XO + ModelPars[95];
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
  MidLine::Fxr_D_1_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[84];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[245];
    real_type t9   = ModelPars[80] * ModelPars[180];
    real_type t10  = ModelPars[88];
    real_type t11  = Fzr__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[86];
    real_type t16  = t15 * Fzr__XO;
    real_type t17  = ModelPars[90];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[182];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[82]);
    real_type t28  = Fzr__XO * t27;
    real_type t31  = t28 * t9 + ModelPars[173];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__r__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t41  = ModelPars[107];
    real_type t45  = phi__XO * ModelPars[99] + ModelPars[95];
    real_type t47  = lambda__r__XO * lambda__r__XO;
    real_type t49  = ModelPars[97] * ModelPars[97];
    real_type t51  = t49 * t47 + 1;
    real_type t52  = sqrt(t51);
    real_type t53  = 1.0 / t52;
    real_type t55  = t45 * t45;
    real_type t58  = ModelPars[111];
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
  MidLine::Fxr_D_1_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[84];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[245];
    real_type t6   = t5 * t3 * t1;
    real_type t7   = Fzr__XO * Fzr__XO;
    real_type t8   = ModelPars[80];
    real_type t10  = ModelPars[180];
    real_type t11  = ModelPars[88];
    real_type t12  = Fzr__XO - t2;
    real_type t16  = t3 * t12 * t11 + ModelPars[86];
    real_type t20  = ModelPars[90];
    real_type t23  = exp(t3 * t12 * t20);
    real_type t24  = ModelPars[182];
    real_type t25  = t24 * t23;
    real_type t26  = t10 * t8;
    real_type t31  = t5 * (t3 * t12 * t1 + ModelPars[82]);
    real_type t33  = Fzr__XO * t31 * t26;
    real_type t35  = t33 + ModelPars[173];
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
    real_type t60  = ModelPars[107];
    real_type t64  = phi__XO * ModelPars[99] + ModelPars[95];
    real_type t66  = ModelPars[97] * ModelPars[97];
    real_type t68  = t66 * t46 + 1;
    real_type t69  = sqrt(t68);
    real_type t71  = 1.0 / t69 * t64;
    real_type t72  = ModelPars[111];
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
  MidLine::Fxr_D_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[245] * (t5 * t3 * ModelPars[84] + ModelPars[82]);
    real_type t13  = ModelPars[80] * ModelPars[180];
    real_type t23  = exp(t5 * t3 * ModelPars[90]);
    real_type t26  = t10 * Fzr__XO;
    real_type t34  = atan(lambda__r__XO / (t26 * t13 + ModelPars[173]) * ModelPars[182] * t23 * (t5 * t3 * ModelPars[88] + ModelPars[86]) * Fzr__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[107];
    real_type t41  = ModelPars[99];
    real_type t42  = lambda__r__XO * lambda__r__XO;
    real_type t44  = ModelPars[97] * ModelPars[97];
    real_type t46  = t44 * t42 + 1;
    real_type t47  = sqrt(t46);
    real_type t48  = 1.0 / t47;
    real_type t50  = ModelPars[111];
    real_type t51  = alpha__r__XO + t50;
    real_type t55  = t41 * phi__XO + ModelPars[95];
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
  MidLine::Fxr_D_2_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[245] * (t5 * t3 * ModelPars[84] + ModelPars[82]);
    real_type t11  = t10 * Fzr__XO;
    real_type t14  = ModelPars[80] * ModelPars[180];
    real_type t24  = exp(t5 * t3 * ModelPars[90]);
    real_type t34  = atan(lambda__r__XO / (t11 * t14 + ModelPars[173]) * ModelPars[182] * t24 * (t5 * t3 * ModelPars[88] + ModelPars[86]) * Fzr__XO);
    real_type t36  = sin(t34 * t14);
    real_type t37  = ModelPars[107];
    real_type t39  = ModelPars[99];
    real_type t40  = t39 * t39;
    real_type t43  = lambda__r__XO * lambda__r__XO;
    real_type t45  = ModelPars[97] * ModelPars[97];
    real_type t47  = t45 * t43 + 1;
    real_type t48  = sqrt(t47);
    real_type t50  = 1.0 / t48 / t47;
    real_type t51  = ModelPars[111];
    real_type t52  = alpha__r__XO + t51;
    real_type t53  = t52 * t52;
    real_type t58  = t39 * phi__XO + ModelPars[95];
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
  MidLine::Fxr_D_2_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[245] * (t5 * t3 * ModelPars[84] + ModelPars[82]);
    real_type t13  = ModelPars[80] * ModelPars[180];
    real_type t23  = exp(t5 * t3 * ModelPars[90]);
    real_type t26  = t10 * Fzr__XO;
    real_type t34  = atan(lambda__r__XO / (t26 * t13 + ModelPars[173]) * ModelPars[182] * t23 * (t5 * t3 * ModelPars[88] + ModelPars[86]) * Fzr__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[107];
    real_type t41  = ModelPars[99];
    real_type t42  = lambda__r__XO * lambda__r__XO;
    real_type t44  = ModelPars[97] * ModelPars[97];
    real_type t46  = t44 * t42 + 1;
    real_type t47  = sqrt(t46);
    real_type t48  = 1.0 / t47;
    real_type t52  = t41 * phi__XO + ModelPars[95];
    real_type t53  = t52 * t52;
    real_type t54  = 1.0 / t46;
    real_type t55  = t54 * t53;
    real_type t56  = ModelPars[111];
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
  MidLine::Fxr_D_2_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[245] * (t5 * t3 * ModelPars[84] + ModelPars[82]);
    real_type t11  = Fzr__XO * Fzr__XO;
    real_type t12  = ModelPars[80];
    real_type t15  = ModelPars[180];
    real_type t20  = t5 * t3 * ModelPars[88] + ModelPars[86];
    real_type t25  = exp(t5 * t3 * ModelPars[90]);
    real_type t26  = ModelPars[182];
    real_type t28  = t15 * t12;
    real_type t29  = t10 * Fzr__XO;
    real_type t32  = t28 * t29 + ModelPars[173];
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
    real_type t59  = ModelPars[107];
    real_type t60  = ModelPars[99];
    real_type t64  = ModelPars[97] * ModelPars[97];
    real_type t66  = t45 * t64 + 1;
    real_type t67  = sqrt(t66);
    real_type t68  = 1.0 / t67;
    real_type t69  = ModelPars[111];
    real_type t70  = alpha__r__XO + t69;
    real_type t74  = t60 * phi__XO + ModelPars[95];
    real_type t75  = t74 * t74;
    real_type t77  = 1.0 / t66 * t75;
    real_type t78  = t70 * t70;
    real_type t80  = t77 * t78 + 1;
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
  MidLine::Fxr_D_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[245] * (t5 * t3 * ModelPars[84] + ModelPars[82]);
    real_type t13  = ModelPars[80] * ModelPars[180];
    real_type t23  = exp(t5 * t3 * ModelPars[90]);
    real_type t34  = atan(lambda__r__XO / (Fzr__XO * t10 * t13 + ModelPars[173]) * ModelPars[182] * t23 * (t5 * t3 * ModelPars[88] + ModelPars[86]) * Fzr__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[107];
    real_type t44  = phi__XO * ModelPars[99] + ModelPars[95];
    real_type t45  = lambda__r__XO * lambda__r__XO;
    real_type t47  = ModelPars[97] * ModelPars[97];
    real_type t49  = t47 * t45 + 1;
    real_type t50  = sqrt(t49);
    real_type t52  = 1.0 / t50 * t44;
    real_type t53  = t44 * t44;
    real_type t56  = ModelPars[111];
    real_type t57  = alpha__r__XO + t56;
    real_type t58  = t57 * t57;
    real_type t63  = atan(t57 * t52);
    real_type t65  = sin(t63 * t38);
    real_type t68  = atan(t56 * t52);
    real_type t70  = cos(t68 * t38);
    return -1.0 / t70 * t65 / (t58 / t49 * t53 + 1) * t52 * t38 * t36 * Fzr__XO * t10;
  }

  real_type
  MidLine::Fxr_D_3_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[245] * (t5 * t3 * ModelPars[84] + ModelPars[82]);
    real_type t13  = ModelPars[80] * ModelPars[180];
    real_type t23  = exp(t5 * t3 * ModelPars[90]);
    real_type t34  = atan(lambda__r__XO / (Fzr__XO * t10 * t13 + ModelPars[173]) * ModelPars[182] * t23 * (t5 * t3 * ModelPars[88] + ModelPars[86]) * Fzr__XO);
    real_type t36  = sin(t34 * t13);
    real_type t37  = t36 * Fzr__XO;
    real_type t38  = ModelPars[107];
    real_type t44  = phi__XO * ModelPars[99] + ModelPars[95];
    real_type t45  = t44 * t44;
    real_type t47  = lambda__r__XO * lambda__r__XO;
    real_type t49  = ModelPars[97] * ModelPars[97];
    real_type t51  = t49 * t47 + 1;
    real_type t52  = sqrt(t51);
    real_type t57  = 1.0 / t51 * t45;
    real_type t58  = ModelPars[111];
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
  MidLine::Fxr_D_3_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[245] * (t5 * t3 * ModelPars[84] + ModelPars[82]);
    real_type t11  = Fzr__XO * Fzr__XO;
    real_type t12  = ModelPars[80];
    real_type t15  = ModelPars[180];
    real_type t20  = t5 * t3 * ModelPars[88] + ModelPars[86];
    real_type t25  = exp(t5 * t3 * ModelPars[90]);
    real_type t26  = ModelPars[182];
    real_type t30  = t15 * t12;
    real_type t31  = t10 * Fzr__XO;
    real_type t34  = t31 * t30 + ModelPars[173];
    real_type t35  = 1.0 / t34;
    real_type t36  = t20 * t20;
    real_type t38  = t25 * t25;
    real_type t40  = t26 * t26;
    real_type t41  = t34 * t34;
    real_type t44  = lambda__r__XO * lambda__r__XO;
    real_type t55  = atan(lambda__r__XO * t35 * t26 * t25 * t20 * Fzr__XO);
    real_type t56  = t55 * t30;
    real_type t57  = cos(t56);
    real_type t58  = ModelPars[107];
    real_type t64  = phi__XO * ModelPars[99] + ModelPars[95];
    real_type t66  = ModelPars[97] * ModelPars[97];
    real_type t68  = t66 * t44 + 1;
    real_type t69  = sqrt(t68);
    real_type t71  = 1.0 / t69 * t64;
    real_type t72  = t64 * t64;
    real_type t74  = 1.0 / t68 * t72;
    real_type t75  = ModelPars[111];
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
  MidLine::Fxr_D_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[245] * (t5 * t3 * ModelPars[84] + ModelPars[82]);
    real_type t11  = Fzr__XO * Fzr__XO;
    real_type t15  = ModelPars[80] * ModelPars[180];
    real_type t20  = t5 * t3 * ModelPars[88] + ModelPars[86];
    real_type t26  = exp(t5 * t3 * ModelPars[90]);
    real_type t27  = ModelPars[182];
    real_type t29  = t10 * Fzr__XO;
    real_type t32  = t29 * t15 + ModelPars[173];
    real_type t33  = 1.0 / t32;
    real_type t35  = t20 * t20;
    real_type t37  = t26 * t26;
    real_type t39  = t27 * t27;
    real_type t40  = t32 * t32;
    real_type t43  = lambda__r__XO * lambda__r__XO;
    real_type t53  = atan(lambda__r__XO * t33 * t27 * t26 * t20 * Fzr__XO);
    real_type t54  = t53 * t15;
    real_type t55  = cos(t54);
    real_type t57  = ModelPars[107];
    real_type t61  = phi__XO * ModelPars[99] + ModelPars[95];
    real_type t63  = ModelPars[97] * ModelPars[97];
    real_type t65  = t63 * t43 + 1;
    real_type t66  = sqrt(t65);
    real_type t68  = 1.0 / t66 * t61;
    real_type t69  = ModelPars[111];
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
  MidLine::Fxr_D_4_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[245] * (t5 * t3 * ModelPars[84] + ModelPars[82]);
    real_type t11  = Fzr__XO * Fzr__XO;
    real_type t12  = t11 * t11;
    real_type t14  = ModelPars[80];
    real_type t15  = ModelPars[180];
    real_type t16  = t15 * t14;
    real_type t21  = t5 * t3 * ModelPars[88] + ModelPars[86];
    real_type t22  = t21 * t21;
    real_type t27  = exp(t5 * t3 * ModelPars[90]);
    real_type t28  = t27 * t27;
    real_type t33  = ModelPars[182];
    real_type t34  = t33 * t33;
    real_type t36  = t10 * Fzr__XO;
    real_type t39  = t36 * t16 + ModelPars[173];
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
    real_type t64  = ModelPars[107];
    real_type t68  = phi__XO * ModelPars[99] + ModelPars[95];
    real_type t70  = ModelPars[97] * ModelPars[97];
    real_type t72  = t70 * t48 + 1;
    real_type t73  = sqrt(t72);
    real_type t75  = 1.0 / t73 * t68;
    real_type t76  = ModelPars[111];
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
  MidLine::Fyf( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
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
    real_type t63  = ModelPars[108];
    real_type t66  = ModelPars[102] * ModelPars[102];
    real_type t67  = tan(alpha__f__XO);
    real_type t70  = pow(t67 - ModelPars[104], 2);
    real_type t73  = sqrt(t70 * t66 + 1);
    real_type t75  = 1.0 / t73 * ModelPars[100];
    real_type t76  = ModelPars[112];
    real_type t80  = 1.0 / t15 * t18 * ModelPars[114];
    real_type t83  = atan((lambda__f__XO + t76 + t80) * t75);
    real_type t85  = cos(t83 * t63);
    real_type t89  = atan((t76 + t80) * t75);
    real_type t91  = cos(t89 * t63);
    return 1.0 / t91 * t85 * (t8 * t28 * t12 * t45 + t57 * t12);
  }

  real_type
  MidLine::Fyf_D_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
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
    real_type t132 = ModelPars[108];
    real_type t133 = ModelPars[100];
    real_type t135 = ModelPars[102] * ModelPars[102];
    real_type t136 = tan(alpha__f__XO);
    real_type t139 = pow(t136 - ModelPars[104], 2);
    real_type t141 = t139 * t135 + 1;
    real_type t142 = sqrt(t141);
    real_type t143 = 1.0 / t142;
    real_type t144 = t143 * t133;
    real_type t145 = ModelPars[112];
    real_type t146 = ModelPars[114];
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
  MidLine::Fyf_D_1_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
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
    real_type t247 = ModelPars[108];
    real_type t248 = ModelPars[100];
    real_type t250 = ModelPars[102] * ModelPars[102];
    real_type t251 = tan(alpha__f__XO);
    real_type t254 = pow(t251 - ModelPars[104], 2);
    real_type t256 = t254 * t250 + 1;
    real_type t257 = sqrt(t256);
    real_type t258 = 1.0 / t257;
    real_type t259 = t258 * t248;
    real_type t260 = ModelPars[112];
    real_type t261 = ModelPars[114];
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
  MidLine::Fyf_D_1_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
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
    real_type t383 = ModelPars[108];
    real_type t384 = ModelPars[100];
    real_type t386 = ModelPars[102] * ModelPars[102];
    real_type t387 = tan(alpha__f__XO);
    real_type t390 = pow(t387 - ModelPars[104], 2);
    real_type t392 = t390 * t386 + 1;
    real_type t393 = sqrt(t392);
    real_type t394 = 1.0 / t393;
    real_type t395 = t394 * t384;
    real_type t396 = ModelPars[112];
    real_type t397 = ModelPars[114];
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
  MidLine::Fyf_D_1_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
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
    real_type t157 = ModelPars[108];
    real_type t158 = ModelPars[100];
    real_type t160 = ModelPars[102] * ModelPars[102];
    real_type t161 = tan(alpha__f__XO);
    real_type t163 = t161 - ModelPars[104];
    real_type t164 = t163 * t163;
    real_type t166 = t164 * t160 + 1;
    real_type t167 = sqrt(t166);
    real_type t168 = 1.0 / t167;
    real_type t169 = t168 * t158;
    real_type t170 = ModelPars[112];
    real_type t171 = ModelPars[114];
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
  MidLine::Fyf_D_1_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
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
    real_type t132 = ModelPars[108];
    real_type t134 = ModelPars[100];
    real_type t137 = ModelPars[102] * ModelPars[102];
    real_type t138 = tan(alpha__f__XO);
    real_type t141 = pow(t138 - ModelPars[104], 2);
    real_type t143 = t141 * t137 + 1;
    real_type t144 = sqrt(t143);
    real_type t145 = 1.0 / t144;
    real_type t146 = t134 * t134;
    real_type t148 = 1.0 / t143 * t146;
    real_type t149 = ModelPars[112];
    real_type t150 = ModelPars[114];
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
  MidLine::Fyf_D_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
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
    real_type t168 = ModelPars[108];
    real_type t171 = ModelPars[102] * ModelPars[102];
    real_type t172 = tan(alpha__f__XO);
    real_type t175 = pow(t172 - ModelPars[104], 2);
    real_type t178 = sqrt(t175 * t171 + 1);
    real_type t180 = 1.0 / t178 * ModelPars[100];
    real_type t181 = ModelPars[112];
    real_type t185 = 1.0 / t16 * t19 * ModelPars[114];
    real_type t188 = atan((lambda__f__XO + t181 + t185) * t180);
    real_type t190 = cos(t188 * t168);
    real_type t194 = atan((t181 + t185) * t180);
    real_type t196 = cos(t194 * t168);
    return 1.0 / t196 * t190 * (-2 * t63 / t9 / t8 * t3 * t2 * t57 * t13 + t153 / (t146 * t9 / t3 / t2 / t140 * t69 * t138 + 1) * (-2 * phi__f__XO * t22 * t51 * t8 * t30 * t71 * t69 * t21 + 2 * t51 * phi__f__XO * t6 * t30 * t71 * t26 + (-2 * t63 * t41 * t13 * t10 * t32 + 2 * t63 * t41 / t12 / t11 / t93 * t3 * Fzf__XO * t2 * t1 + (-2 * t22 * t37 * t5 * t36 - t48 * t36) * t34 * t32 - t24 * t37 * t8 * t29 * t110 + 2 * t6 * t3 * t24 * t37 * t10 * t1 * t13 * t115 - 2 * t48 * t6 * t29 * t124 - 2 * t22 * t37 * t47 * t124) * t8 * t31 * t28) * t14 * t12 + t47 * t110 - 2 * t6 * t3 * t10 * t1 * t13 * t115 + 2 * t6 * t29 * t12 * t115);
  }

  real_type
  MidLine::Fyf_D_2_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
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
    real_type t354 = ModelPars[108];
    real_type t357 = ModelPars[102] * ModelPars[102];
    real_type t358 = tan(alpha__f__XO);
    real_type t361 = pow(t358 - ModelPars[104], 2);
    real_type t364 = sqrt(t361 * t357 + 1);
    real_type t366 = 1.0 / t364 * ModelPars[100];
    real_type t367 = ModelPars[112];
    real_type t371 = 1.0 / t17 * t20 * ModelPars[114];
    real_type t374 = atan((lambda__f__XO + t367 + t371) * t366);
    real_type t376 = cos(t374 * t354);
    real_type t380 = atan((t367 + t371) * t366);
    real_type t382 = cos(t380 * t354);
    return 1.0 / t382 * t376 * t353;
  }

  real_type
  MidLine::Fyf_D_2_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
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
    real_type t145 = t38 * t50;
    real_type t150 = t1 * t50;
    real_type t159 = t38 * t150;
    real_type t172 = -2 * phi__f__XO * t14 * t58 * t117 * t115 * t82 + 2 * t58 * phi__f__XO * t2 * t68 * t115 * t65 + (-2 * t77 * t47 * t39 * t91 + 2 * t77 * t47 * t102 * t100 + (-2 * t14 * t43 * t1 * t42 - t55 * t42) * t40 * t34 - t16 * t43 * t4 * t53 * t145 + 2 * t2 * t30 * t16 * t43 * t36 * t27 * t39 * t150 - 2 * t55 * t2 * t53 * t159 - 2 * t14 * t43 * t54 * t159) * t4 * t69 * t67;
    real_type t174 = t63 * t63;
    real_type t175 = 1.0 / t174;
    real_type t186 = sin(t74);
    real_type t190 = ModelPars[108];
    real_type t191 = ModelPars[100];
    real_type t193 = ModelPars[102] * ModelPars[102];
    real_type t194 = tan(alpha__f__XO);
    real_type t196 = t194 - ModelPars[104];
    real_type t197 = t196 * t196;
    real_type t199 = t197 * t193 + 1;
    real_type t200 = sqrt(t199);
    real_type t202 = 1.0 / t200 * t191;
    real_type t203 = ModelPars[112];
    real_type t207 = 1.0 / t7 * t10 * ModelPars[114];
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
  MidLine::Fyf_D_2_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
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
    real_type t168 = ModelPars[108];
    real_type t170 = ModelPars[100];
    real_type t173 = ModelPars[102] * ModelPars[102];
    real_type t174 = tan(alpha__f__XO);
    real_type t177 = pow(t174 - ModelPars[104], 2);
    real_type t179 = t177 * t173 + 1;
    real_type t180 = sqrt(t179);
    real_type t181 = 1.0 / t180;
    real_type t182 = t170 * t170;
    real_type t185 = ModelPars[112];
    real_type t189 = 1.0 / t16 * t19 * ModelPars[114];
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
  MidLine::Fyf_D_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
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
    real_type t75  = ModelPars[108];
    real_type t76  = ModelPars[100];
    real_type t78  = ModelPars[102] * ModelPars[102];
    real_type t79  = tan(alpha__f__XO);
    real_type t81  = t79 - ModelPars[104];
    real_type t82  = t81 * t81;
    real_type t84  = t82 * t78 + 1;
    real_type t85  = sqrt(t84);
    real_type t87  = 1.0 / t85 * t76;
    real_type t88  = ModelPars[112];
    real_type t92  = 1.0 / t2 * t5 * ModelPars[114];
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
  MidLine::Fyf_D_3_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
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
    real_type t80  = ModelPars[108];
    real_type t81  = ModelPars[100];
    real_type t83  = ModelPars[102] * ModelPars[102];
    real_type t84  = tan(alpha__f__XO);
    real_type t86  = t84 - ModelPars[104];
    real_type t87  = t86 * t86;
    real_type t89  = t87 * t83 + 1;
    real_type t90  = sqrt(t89);
    real_type t92  = 1.0 / t90 * t81;
    real_type t93  = ModelPars[112];
    real_type t97  = 1.0 / t2 * t5 * ModelPars[114];
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
  MidLine::Fyf_D_3_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
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
    real_type t75  = ModelPars[108];
    real_type t78  = ModelPars[100];
    real_type t80  = ModelPars[102] * ModelPars[102];
    real_type t81  = tan(alpha__f__XO);
    real_type t83  = t81 - ModelPars[104];
    real_type t84  = t83 * t83;
    real_type t86  = t84 * t80 + 1;
    real_type t87  = sqrt(t86);
    real_type t89  = 1.0 / t87 * t78;
    real_type t90  = t78 * t78;
    real_type t92  = 1.0 / t86 * t90;
    real_type t93  = ModelPars[112];
    real_type t97  = 1.0 / t2 * t5 * ModelPars[114];
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
  MidLine::Fyf_D_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
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
    real_type t63  = ModelPars[108];
    real_type t65  = ModelPars[100];
    real_type t68  = ModelPars[102] * ModelPars[102];
    real_type t69  = tan(alpha__f__XO);
    real_type t72  = pow(t69 - ModelPars[104], 2);
    real_type t74  = t72 * t68 + 1;
    real_type t75  = sqrt(t74);
    real_type t76  = 1.0 / t75;
    real_type t77  = t65 * t65;
    real_type t80  = ModelPars[112];
    real_type t84  = 1.0 / t15 * t18 * ModelPars[114];
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
  MidLine::Fyf_D_4_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
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
    real_type t63  = ModelPars[108];
    real_type t65  = ModelPars[100];
    real_type t66  = t65 * t65;
    real_type t69  = ModelPars[102] * ModelPars[102];
    real_type t70  = tan(alpha__f__XO);
    real_type t73  = pow(t70 - ModelPars[104], 2);
    real_type t75  = t73 * t69 + 1;
    real_type t76  = sqrt(t75);
    real_type t81  = 1.0 / t75;
    real_type t83  = ModelPars[112];
    real_type t87  = 1.0 / t15 * t18 * ModelPars[114];
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
  MidLine::Fyr( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
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
    real_type t63  = ModelPars[109];
    real_type t66  = ModelPars[103] * ModelPars[103];
    real_type t67  = tan(alpha__r__XO);
    real_type t70  = pow(t67 - ModelPars[105], 2);
    real_type t73  = sqrt(t70 * t66 + 1);
    real_type t75  = 1.0 / t73 * ModelPars[101];
    real_type t76  = ModelPars[113];
    real_type t80  = 1.0 / t15 * t18 * ModelPars[115];
    real_type t83  = atan((lambda__r__XO + t76 + t80) * t75);
    real_type t85  = cos(t83 * t63);
    real_type t89  = atan((t76 + t80) * t75);
    real_type t91  = cos(t89 * t63);
    return 1.0 / t91 * t85 * (t8 * t28 * t12 * t45 + t57 * t12);
  }

  real_type
  MidLine::Fyr_D_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
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
    real_type t132 = ModelPars[109];
    real_type t133 = ModelPars[101];
    real_type t135 = ModelPars[103] * ModelPars[103];
    real_type t136 = tan(alpha__r__XO);
    real_type t139 = pow(t136 - ModelPars[105], 2);
    real_type t141 = t139 * t135 + 1;
    real_type t142 = sqrt(t141);
    real_type t143 = 1.0 / t142;
    real_type t144 = t143 * t133;
    real_type t145 = ModelPars[113];
    real_type t146 = ModelPars[115];
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
  MidLine::Fyr_D_1_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
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
    real_type t247 = ModelPars[109];
    real_type t248 = ModelPars[101];
    real_type t250 = ModelPars[103] * ModelPars[103];
    real_type t251 = tan(alpha__r__XO);
    real_type t254 = pow(t251 - ModelPars[105], 2);
    real_type t256 = t254 * t250 + 1;
    real_type t257 = sqrt(t256);
    real_type t258 = 1.0 / t257;
    real_type t259 = t258 * t248;
    real_type t260 = ModelPars[113];
    real_type t261 = ModelPars[115];
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
  MidLine::Fyr_D_1_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
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
    real_type t108 = -2 * t23 * t39 * t5 * t38 - t38 * t50;
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
    real_type t383 = ModelPars[109];
    real_type t384 = ModelPars[101];
    real_type t386 = ModelPars[103] * ModelPars[103];
    real_type t387 = tan(alpha__r__XO);
    real_type t390 = pow(t387 - ModelPars[105], 2);
    real_type t392 = t390 * t386 + 1;
    real_type t393 = sqrt(t392);
    real_type t394 = 1.0 / t393;
    real_type t395 = t394 * t384;
    real_type t396 = ModelPars[113];
    real_type t397 = ModelPars[115];
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
  MidLine::Fyr_D_1_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
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
    real_type t157 = ModelPars[109];
    real_type t158 = ModelPars[101];
    real_type t160 = ModelPars[103] * ModelPars[103];
    real_type t161 = tan(alpha__r__XO);
    real_type t163 = t161 - ModelPars[105];
    real_type t164 = t163 * t163;
    real_type t166 = t164 * t160 + 1;
    real_type t167 = sqrt(t166);
    real_type t168 = 1.0 / t167;
    real_type t169 = t168 * t158;
    real_type t170 = ModelPars[113];
    real_type t171 = ModelPars[115];
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
  MidLine::Fyr_D_1_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
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
    real_type t132 = ModelPars[109];
    real_type t134 = ModelPars[101];
    real_type t137 = ModelPars[103] * ModelPars[103];
    real_type t138 = tan(alpha__r__XO);
    real_type t141 = pow(t138 - ModelPars[105], 2);
    real_type t143 = t141 * t137 + 1;
    real_type t144 = sqrt(t143);
    real_type t145 = 1.0 / t144;
    real_type t146 = t134 * t134;
    real_type t148 = 1.0 / t143 * t146;
    real_type t149 = ModelPars[113];
    real_type t150 = ModelPars[115];
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
  MidLine::Fyr_D_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
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
    real_type t168 = ModelPars[109];
    real_type t171 = ModelPars[103] * ModelPars[103];
    real_type t172 = tan(alpha__r__XO);
    real_type t175 = pow(t172 - ModelPars[105], 2);
    real_type t178 = sqrt(t175 * t171 + 1);
    real_type t180 = 1.0 / t178 * ModelPars[101];
    real_type t181 = ModelPars[113];
    real_type t185 = 1.0 / t16 * t19 * ModelPars[115];
    real_type t188 = atan((lambda__r__XO + t181 + t185) * t180);
    real_type t190 = cos(t188 * t168);
    real_type t194 = atan((t181 + t185) * t180);
    real_type t196 = cos(t194 * t168);
    return 1.0 / t196 * t190 * (-2 * t63 / t9 / t8 * t3 * t2 * t57 * t13 + t153 / (t146 * t9 / t3 / t2 / t140 * t69 * t138 + 1) * (-2 * phi__XO * t22 * t51 * t8 * t30 * t71 * t69 * t21 + 2 * t51 * phi__XO * t6 * t30 * t71 * t26 + (-2 * t63 * t41 * t13 * t10 * t32 + 2 * t63 * t41 / t12 / t11 / t93 * t3 * Fzr__XO * t2 * t1 + (-2 * t22 * t37 * t5 * t36 - t48 * t36) * t34 * t32 - t24 * t37 * t8 * t29 * t110 + 2 * t6 * t3 * t24 * t37 * t10 * t1 * t13 * t115 - 2 * t48 * t6 * t29 * t124 - 2 * t22 * t37 * t47 * t124) * t8 * t31 * t28) * t14 * t12 + t47 * t110 - 2 * t6 * t3 * t10 * t1 * t13 * t115 + 2 * t6 * t29 * t12 * t115);
  }

  real_type
  MidLine::Fyr_D_2_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
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
    real_type t354 = ModelPars[109];
    real_type t357 = ModelPars[103] * ModelPars[103];
    real_type t358 = tan(alpha__r__XO);
    real_type t361 = pow(t358 - ModelPars[105], 2);
    real_type t364 = sqrt(t361 * t357 + 1);
    real_type t366 = 1.0 / t364 * ModelPars[101];
    real_type t367 = ModelPars[113];
    real_type t371 = 1.0 / t17 * t20 * ModelPars[115];
    real_type t374 = atan((lambda__r__XO + t367 + t371) * t366);
    real_type t376 = cos(t374 * t354);
    real_type t380 = atan((t367 + t371) * t366);
    real_type t382 = cos(t380 * t354);
    return 1.0 / t382 * t376 * t353;
  }

  real_type
  MidLine::Fyr_D_2_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
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
    real_type t190 = ModelPars[109];
    real_type t191 = ModelPars[101];
    real_type t193 = ModelPars[103] * ModelPars[103];
    real_type t194 = tan(alpha__r__XO);
    real_type t196 = t194 - ModelPars[105];
    real_type t197 = t196 * t196;
    real_type t199 = t197 * t193 + 1;
    real_type t200 = sqrt(t199);
    real_type t202 = 1.0 / t200 * t191;
    real_type t203 = ModelPars[113];
    real_type t207 = 1.0 / t7 * t10 * ModelPars[115];
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
  MidLine::Fyr_D_2_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
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
    real_type t168 = ModelPars[109];
    real_type t170 = ModelPars[101];
    real_type t173 = ModelPars[103] * ModelPars[103];
    real_type t174 = tan(alpha__r__XO);
    real_type t177 = pow(t174 - ModelPars[105], 2);
    real_type t179 = t177 * t173 + 1;
    real_type t180 = sqrt(t179);
    real_type t181 = 1.0 / t180;
    real_type t182 = t170 * t170;
    real_type t185 = ModelPars[113];
    real_type t189 = 1.0 / t16 * t19 * ModelPars[115];
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
  MidLine::Fyr_D_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
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
    real_type t75  = ModelPars[109];
    real_type t76  = ModelPars[101];
    real_type t78  = ModelPars[103] * ModelPars[103];
    real_type t79  = tan(alpha__r__XO);
    real_type t81  = t79 - ModelPars[105];
    real_type t82  = t81 * t81;
    real_type t84  = t82 * t78 + 1;
    real_type t85  = sqrt(t84);
    real_type t87  = 1.0 / t85 * t76;
    real_type t88  = ModelPars[113];
    real_type t92  = 1.0 / t2 * t5 * ModelPars[115];
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
  MidLine::Fyr_D_3_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
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
    real_type t80  = ModelPars[109];
    real_type t81  = ModelPars[101];
    real_type t83  = ModelPars[103] * ModelPars[103];
    real_type t84  = tan(alpha__r__XO);
    real_type t86  = t84 - ModelPars[105];
    real_type t87  = t86 * t86;
    real_type t89  = t87 * t83 + 1;
    real_type t90  = sqrt(t89);
    real_type t92  = 1.0 / t90 * t81;
    real_type t93  = ModelPars[113];
    real_type t97  = 1.0 / t2 * t5 * ModelPars[115];
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
  MidLine::Fyr_D_3_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
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
    real_type t75  = ModelPars[109];
    real_type t78  = ModelPars[101];
    real_type t80  = ModelPars[103] * ModelPars[103];
    real_type t81  = tan(alpha__r__XO);
    real_type t83  = t81 - ModelPars[105];
    real_type t84  = t83 * t83;
    real_type t86  = t84 * t80 + 1;
    real_type t87  = sqrt(t86);
    real_type t89  = 1.0 / t87 * t78;
    real_type t90  = t78 * t78;
    real_type t92  = 1.0 / t86 * t90;
    real_type t93  = ModelPars[113];
    real_type t97  = 1.0 / t2 * t5 * ModelPars[115];
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
  MidLine::Fyr_D_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
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
    real_type t63  = ModelPars[109];
    real_type t65  = ModelPars[101];
    real_type t68  = ModelPars[103] * ModelPars[103];
    real_type t69  = tan(alpha__r__XO);
    real_type t72  = pow(t69 - ModelPars[105], 2);
    real_type t74  = t72 * t68 + 1;
    real_type t75  = sqrt(t74);
    real_type t76  = 1.0 / t75;
    real_type t77  = t65 * t65;
    real_type t80  = ModelPars[113];
    real_type t84  = 1.0 / t15 * t18 * ModelPars[115];
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
  MidLine::Fyr_D_4_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
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
    real_type t63  = ModelPars[109];
    real_type t65  = ModelPars[101];
    real_type t66  = t65 * t65;
    real_type t69  = ModelPars[103] * ModelPars[103];
    real_type t70  = tan(alpha__r__XO);
    real_type t73  = pow(t70 - ModelPars[105], 2);
    real_type t75  = t73 * t69 + 1;
    real_type t76  = sqrt(t75);
    real_type t81  = 1.0 / t75;
    real_type t83  = ModelPars[113];
    real_type t87  = 1.0 / t15 * t18 * ModelPars[115];
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
  MidLine::Mzf( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
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
  MidLine::Mzf_D_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
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
  MidLine::Mzf_D_1_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[49];
    real_type t2   = Fzf__XO * t1;
    real_type t4   = ModelPars[6];
    real_type t6   = ModelPars[35];
    real_type t9   = t4 * ModelPars[33] + (Fzf__XO - t4) * t6;
    real_type t10  = 1.0 / t9;
    real_type t12  = ModelPars[48];
    real_type t13  = phi__f__XO * phi__f__XO;
    real_type t16  = ModelPars[54] * t13 + 1;
    real_type t17  = t16 * t16;
    real_type t18  = 1.0 / t17;
    real_type t19  = t18 * t12;
    real_type t20  = ModelPars[56];
    real_type t21  = ModelPars[39];
    real_type t22  = t21 * t20;
    real_type t25  = ModelPars[45] * t13 + 1;
    real_type t26  = 1.0 / t25;
    real_type t27  = t21 * t21;
    real_type t28  = Fzf__XO * Fzf__XO;
    real_type t30  = t25 * t25;
    real_type t31  = 1.0 / t30;
    real_type t32  = t31 * t28 * t27;
    real_type t33  = sqrt(t32);
    real_type t34  = 1.0 / t33;
    real_type t36  = alpha__f__XO * t34 * t26;
    real_type t38  = t27 * t21;
    real_type t39  = t38 * t20;
    real_type t42  = 1.0 / t30 / t25;
    real_type t44  = 1.0 / t33 / t32;
    real_type t45  = t44 * t42;
    real_type t48  = -alpha__f__XO * t45 * t28 * t39 + t36 * t22;
    real_type t49  = t48 * t19;
    real_type t51  = alpha__f__XO * alpha__f__XO;
    real_type t52  = t20 * t20;
    real_type t54  = t52 * t51 + 1;
    real_type t55  = 1.0 / t54;
    real_type t56  = 1.0 / t16;
    real_type t57  = t56 * t12;
    real_type t60  = atan(t36 * Fzf__XO * t22);
    real_type t61  = t60 * t57;
    real_type t62  = sin(t61);
    real_type t63  = t62 * t55;
    real_type t65  = ModelPars[47];
    real_type t68  = ModelPars[41] * t13 + 1;
    real_type t69  = 1.0 / t68;
    real_type t70  = t69 * t6;
    real_type t71  = 1.0 / t65;
    real_type t73  = alpha__f__XO * t34 * t71;
    real_type t75  = t69 * t9;
    real_type t76  = t44 * t71;
    real_type t80  = t31 * Fzf__XO * t27 * alpha__f__XO;
    real_type t82  = -t80 * t76 * t75 + t73 * t70;
    real_type t83  = t82 * t65;
    real_type t84  = t9 * t9;
    real_type t85  = t68 * t68;
    real_type t86  = 1.0 / t85;
    real_type t88  = t65 * t65;
    real_type t89  = 1.0 / t88;
    real_type t90  = t89 * t86 * t84;
    real_type t91  = 1.0 / t27;
    real_type t92  = 1.0 / t28;
    real_type t94  = t51 * t30;
    real_type t97  = t94 * t92 * t91 * t90 + 1;
    real_type t98  = 1.0 / t97;
    real_type t100 = atan(t73 * t75);
    real_type t101 = t100 * t65;
    real_type t102 = cos(t101);
    real_type t103 = t102 * t98;
    real_type t108 = ModelPars[51];
    real_type t110 = ModelPars[55];
    real_type t112 = atan(phi__f__XO * t110);
    real_type t113 = 1.0 / t110;
    real_type t115 = t113 * t112 * Fzf__XO * t108;
    real_type t116 = t12 * t12;
    real_type t118 = ModelPars[57];
    real_type t121 = ModelPars[53] * t13 + 1;
    real_type t123 = 1.0 / t121 * t118;
    real_type t124 = t21 * t123;
    real_type t126 = t38 * t123;
    real_type t128 = alpha__f__XO * t44;
    real_type t131 = -t128 * t42 * t28 * t126 + t36 * t124;
    real_type t132 = t131 * t131;
    real_type t133 = t118 * t118;
    real_type t134 = t121 * t121;
    real_type t138 = t51 / t134 * t133 + 1;
    real_type t139 = t138 * t138;
    real_type t146 = atan(alpha__f__XO * t34 * t26 * Fzf__XO * t124);
    real_type t147 = t146 * t57;
    real_type t148 = cos(t147);
    real_type t152 = 1.0 / t84;
    real_type t153 = cos(t61);
    real_type t155 = t56 * t153 * t152;
    real_type t157 = t65 * t33;
    real_type t158 = t82 * t157;
    real_type t163 = t153 * t10;
    real_type t165 = t56 * t163 * t2;
    real_type t166 = t97 * t97;
    real_type t167 = 1.0 / t166;
    real_type t176 = t28 * Fzf__XO;
    real_type t189 = t55 * t48;
    real_type t191 = sin(t101);
    real_type t201 = 1.0 / t138;
    real_type t202 = sin(t147);
    real_type t212 = t48 * t48;
    real_type t213 = t54 * t54;
    real_type t220 = t28 * t1;
    real_type t221 = t10 * t220;
    real_type t229 = t56 * t153;
    real_type t240 = t33 * t56;
    real_type t248 = t27 * t27;
    real_type t250 = t30 * t30;
    real_type t251 = 1.0 / t250;
    real_type t258 = t248 * t21;
    real_type t261 = 1.0 / t250 / t25;
    real_type t263 = t28 * t28;
    real_type t267 = 1.0 / t33 / t251 / t263 / t248;
    real_type t277 = t10 * t1;
    real_type t280 = t191 * t33 * t62;
    real_type t284 = t229 * t277;
    real_type t285 = t191 * t34;
    real_type t300 = t6 * t6;
    real_type t341 = t82 * t82;
    return 2 * t103 * t83 * t33 * t63 * t49 * t10 * t2 - t148 / t139 * t132 * t18 * t116 * t115 + 2 * t6 * t103 * t158 * t155 * t2 + (2 * t6 * t51 * t30 * t92 * t91 * t89 * t86 * t9 - 2 * t94 / t176 * t91 * t90) * t102 * t167 * t158 * t165 - 2 * t6 * t191 * t33 * t62 * t189 * t18 * t12 * t152 * t2 - 2 * t202 * t201 * t131 * t56 * t12 * t113 * t112 * t108 + t191 * t33 * t153 / t213 * t212 / t17 / t16 * t116 * t10 * t2 + 2 * t31 * t27 * t191 * t34 * t63 * t49 * t221 - 2 * t31 * t27 * t102 * t98 * t83 * t34 * t229 * t221 + 2 * t6 * t191 * t240 * t153 * t152 * t1 + t251 * t248 * t191 * t44 * t56 * t163 * t176 * t1 - t202 * t201 * (3 * alpha__f__XO * t267 * t261 * t176 * t258 * t123 - 3 * alpha__f__XO * Fzf__XO * t45 * t126) * t57 * t115 + 2 * t280 * t189 * t19 * t277 - 3 * t31 * Fzf__XO * t27 * t285 * t284 - 2 * t102 * t98 * t82 * t157 * t284 - 2 * t300 * t191 * t240 * t153 / t84 / t9 * t2 + t280 * t55 * (3 * alpha__f__XO * t267 * t261 * t176 * t258 * t20 - 3 * Fzf__XO * t128 * t42 * t39) * t18 * t12 * t10 * t2 - t102 * t98 * (3 * t251 * t28 * t248 * alpha__f__XO * t267 * t71 * t75 - t31 * t27 * t128 * t71 * t75 - 2 * t80 * t76 * t70) * t157 * t165 + t191 * t167 * t341 * t88 * t33 * t165 + 2 * t31 * t27 * t6 * t285 * t155 * t220;
  }

  real_type
  MidLine::Mzf_D_1_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[49];
    real_type t2   = Fzf__XO * t1;
    real_type t4   = ModelPars[6];
    real_type t6   = ModelPars[35];
    real_type t9   = t4 * ModelPars[33] + (Fzf__XO - t4) * t6;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t2;
    real_type t12  = ModelPars[48];
    real_type t13  = phi__f__XO * phi__f__XO;
    real_type t14  = ModelPars[54];
    real_type t16  = t14 * t13 + 1;
    real_type t17  = t16 * t16;
    real_type t18  = 1.0 / t17;
    real_type t19  = t18 * t12;
    real_type t20  = ModelPars[56];
    real_type t21  = ModelPars[39];
    real_type t22  = t21 * t20;
    real_type t23  = ModelPars[45];
    real_type t25  = t23 * t13 + 1;
    real_type t26  = 1.0 / t25;
    real_type t27  = t21 * t21;
    real_type t28  = Fzf__XO * Fzf__XO;
    real_type t29  = t28 * t27;
    real_type t30  = t25 * t25;
    real_type t31  = 1.0 / t30;
    real_type t32  = t31 * t29;
    real_type t33  = sqrt(t32);
    real_type t34  = 1.0 / t33;
    real_type t35  = t34 * t26;
    real_type t36  = alpha__f__XO * t35;
    real_type t38  = t27 * t21;
    real_type t39  = t38 * t20;
    real_type t42  = 1.0 / t30 / t25;
    real_type t44  = 1.0 / t33 / t32;
    real_type t45  = t44 * t42;
    real_type t48  = -alpha__f__XO * t45 * t28 * t39 + t36 * t22;
    real_type t51  = alpha__f__XO * alpha__f__XO;
    real_type t52  = t20 * t20;
    real_type t55  = 1.0 / (t52 * t51 + 1);
    real_type t56  = 1.0 / t16;
    real_type t57  = t56 * t12;
    real_type t60  = atan(t36 * Fzf__XO * t22);
    real_type t61  = t60 * t57;
    real_type t62  = sin(t61);
    real_type t64  = t33 * t62 * t55;
    real_type t65  = ModelPars[47];
    real_type t66  = ModelPars[41];
    real_type t68  = t66 * t13 + 1;
    real_type t69  = t68 * t68;
    real_type t70  = 1.0 / t69;
    real_type t71  = t70 * t9;
    real_type t72  = 1.0 / t65;
    real_type t74  = alpha__f__XO * t34;
    real_type t76  = phi__f__XO * t66 * t74;
    real_type t78  = 1.0 / t68;
    real_type t79  = t78 * t9;
    real_type t80  = t44 * t72;
    real_type t81  = alpha__f__XO * t80;
    real_type t82  = t81 * t79;
    real_type t84  = phi__f__XO * t23 * t42;
    real_type t85  = t84 * t29;
    real_type t88  = -2 * t76 * t72 * t71 + 2 * t85 * t82;
    real_type t89  = t88 * t65;
    real_type t90  = t9 * t9;
    real_type t91  = t70 * t90;
    real_type t92  = t65 * t65;
    real_type t93  = 1.0 / t92;
    real_type t95  = 1.0 / t27;
    real_type t96  = 1.0 / t28;
    real_type t101 = t51 * t30 * t96 * t95 * t93 * t91 + 1;
    real_type t102 = 1.0 / t101;
    real_type t104 = alpha__f__XO * t34 * t72;
    real_type t106 = atan(t104 * t79);
    real_type t107 = t106 * t65;
    real_type t108 = cos(t107);
    real_type t109 = t108 * t102;
    real_type t113 = ModelPars[51];
    real_type t114 = Fzf__XO * t113;
    real_type t115 = ModelPars[55];
    real_type t117 = atan(phi__f__XO * t115);
    real_type t118 = 1.0 / t115;
    real_type t119 = t118 * t117;
    real_type t121 = t12 * t119 * t114;
    real_type t122 = ModelPars[57];
    real_type t123 = ModelPars[53];
    real_type t125 = t123 * t13 + 1;
    real_type t127 = 1.0 / t125 * t122;
    real_type t128 = t21 * t127;
    real_type t132 = alpha__f__XO * t44;
    real_type t135 = -t132 * t42 * t28 * t38 * t127 + t36 * t128;
    real_type t136 = t135 * t56;
    real_type t137 = t122 * t122;
    real_type t138 = t125 * t125;
    real_type t139 = 1.0 / t138;
    real_type t142 = t51 * t139 * t137 + 1;
    real_type t143 = 1.0 / t142;
    real_type t147 = atan(t74 * t26 * Fzf__XO * t128);
    real_type t152 = t139 * t122;
    real_type t153 = Fzf__XO * t21;
    real_type t156 = phi__f__XO * t123 * alpha__f__XO;
    real_type t162 = phi__f__XO * t23 * alpha__f__XO;
    real_type t165 = t28 * Fzf__XO;
    real_type t168 = t30 * t30;
    real_type t169 = 1.0 / t168;
    real_type t170 = t44 * t169;
    real_type t177 = -2 * phi__f__XO * t14 * t147 * t19 + t143 * (-2 * t162 * t34 * t31 * t153 * t127 + 2 * t162 * t170 * t165 * t38 * t127 - 2 * t156 * t35 * t153 * t152) * t57;
    real_type t179 = t147 * t57;
    real_type t180 = cos(t179);
    real_type t184 = 1.0 / t90;
    real_type t185 = cos(t61);
    real_type t186 = t185 * t184;
    real_type t189 = sin(t107);
    real_type t190 = t189 * t33;
    real_type t198 = t18 * t12 * t10 * t2;
    real_type t200 = phi__f__XO * t23;
    real_type t201 = t200 * t74;
    real_type t209 = t27 * t27;
    real_type t210 = t209 * t21;
    real_type t212 = t28 * t28;
    real_type t214 = 1.0 / t168 / t30;
    real_type t220 = 1.0 / t33 / t169 / t212 / t209;
    real_type t231 = t28 * t1;
    real_type t236 = t31 * Fzf__XO;
    real_type t247 = -2 * phi__f__XO * t14 * t60 * t19 + t55 * (2 * t200 * t132 * t169 * t165 * t39 - 2 * t201 * t236 * t22) * t57;
    real_type t249 = t62 * t247 * t10;
    real_type t252 = t27 * t189;
    real_type t256 = t185 * t10;
    real_type t257 = t56 * t256;
    real_type t258 = t257 * t2;
    real_type t259 = t65 * t33;
    real_type t264 = t78 * t6;
    real_type t269 = Fzf__XO * t27;
    real_type t282 = phi__f__XO * t23 / t168 / t25;
    real_type t300 = t55 * t48;
    real_type t311 = -t236 * t27 * alpha__f__XO * t80 * t79 + t104 * t264;
    real_type t313 = t101 * t101;
    real_type t314 = 1.0 / t313;
    real_type t319 = t95 * t93;
    real_type t339 = sin(t179);
    real_type t349 = t108 * t102 * t311;
    real_type t374 = t10 * t1;
    real_type t375 = t56 * t185;
    real_type t376 = t34 * t375;
    real_type t382 = t109 * t89 * t64 * t48 * t19 * t11 - t180 * t177 * t143 * t136 * t121 - 2 * phi__f__XO * t14 * t6 * t190 * t18 * t186 * t2 + t189 * t33 * t62 * t55 * (-6 * t200 * alpha__f__XO * t220 * t214 * t212 * t210 * t20 + 8 * t200 * t28 * alpha__f__XO * t170 * t39 - 2 * t201 * t31 * t22) * t198 + t31 * t252 * t34 * t56 * t249 * t231 - t108 * t102 * (-6 * t282 * t165 * t209 * alpha__f__XO * t220 * t72 * t79 + 2 * phi__f__XO * t66 * t31 * t269 * t81 * t71 - 2 * t76 * t72 * t70 * t6 + 2 * t85 * t81 * t264 + 4 * t84 * t269 * t82) * t259 * t258 + t6 * t109 * t88 * t259 * t56 * t186 * t2 + t189 * t33 * t185 * t247 * t300 * t198 + (-4 * phi__f__XO * t66 * t51 * t30 * t96 * t319 / t69 / t68 * t90 + 4 * phi__f__XO * t23 * t51 * t25 * t96 * t319 * t91) * t108 * t314 * t311 * t259 * t258 + 2 * phi__f__XO * t14 * t339 * t143 * t135 * t18 * t121 + t349 * t65 * t33 * t56 * t249 * t2 + 2 * phi__f__XO * t14 * t31 * t27 * t189 * t34 * t18 * t256 * t231 + t189 * t88 * t314 * t311 * t92 * t33 * t258 - 2 * t282 * t209 * t189 * t44 * t257 * t212 * t1 + 6 * t84 * t28 * t252 * t376 * t374;
    real_type t393 = t18 * t185;
    real_type t410 = t165 * t1;
    real_type t425 = t56 * t62;
    real_type t432 = phi__f__XO * t123;
    real_type t463 = t10 * t410;
    real_type t469 = t200 * t42 * t27;
    real_type t483 = t142 * t142;
    real_type t498 = t115 * t115;
    real_type t501 = 1.0 / (t498 * t13 + 1);
    real_type t517 = -4 * phi__f__XO * t14 * t189 * t64 * t48 / t17 / t16 * t12 * t11 + 2 * phi__f__XO * t14 * t108 * t102 * t311 * t65 * t33 * t393 * t11 - t31 * t27 * t108 * t102 * t89 * t376 * t10 * t231 - 2 * t84 * t27 * t6 * t189 * t376 * t184 * t410 - t108 * t102 * t88 * t259 * t375 * t374 - t6 * t190 * t425 * t247 * t184 * t2 - t339 * t143 * (8 * phi__f__XO * t23 * t28 * t132 * t169 * t38 * t127 - 6 * t162 * t220 * t214 * t212 * t210 * t127 + 2 * t156 * t45 * t28 * t38 * t152 - 2 * t432 * t74 * t26 * t21 * t152 - 2 * t201 * t31 * t21 * t127) * t57 * t119 * t114 - 2 * t469 * t189 * t34 * t62 * t300 * t19 * t463 + 2 * t469 * t349 * t65 * t34 * t375 * t463 - 4 * t432 * t51 / t138 / t125 * t137 * t339 / t483 * t136 * t12 * t118 * t117 * t114 - t339 * t177 * t118 * t117 * t113 + t180 * t501 * t113 + t190 * t425 * t247 * t374 + 2 * phi__f__XO * t14 * t190 * t393 * t374 - t339 * t143 * t136 * t12 * t501 * t114;
    return t382 + t517;
  }

  real_type
  MidLine::Mzf_D_1_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
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
  MidLine::Mzf_D_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
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
  MidLine::Mzf_D_2_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t2   = ModelPars[51] * Fzf__XO;
    real_type t3   = ModelPars[55];
    real_type t5   = atan(phi__f__XO * t3);
    real_type t6   = t5 * t2;
    real_type t7   = 1.0 / t3;
    real_type t8   = ModelPars[48];
    real_type t9   = phi__f__XO * phi__f__XO;
    real_type t10  = ModelPars[54];
    real_type t12  = t10 * t9 + 1;
    real_type t13  = t12 * t12;
    real_type t14  = 1.0 / t13;
    real_type t15  = t14 * t8;
    real_type t16  = ModelPars[57];
    real_type t17  = ModelPars[53];
    real_type t18  = t17 * t9;
    real_type t19  = t18 + 1;
    real_type t21  = 1.0 / t19 * t16;
    real_type t22  = ModelPars[39];
    real_type t24  = ModelPars[45];
    real_type t26  = t24 * t9 + 1;
    real_type t27  = 1.0 / t26;
    real_type t29  = t22 * t22;
    real_type t30  = Fzf__XO * Fzf__XO;
    real_type t31  = t30 * t29;
    real_type t32  = t26 * t26;
    real_type t33  = 1.0 / t32;
    real_type t34  = t33 * t31;
    real_type t35  = sqrt(t34);
    real_type t36  = 1.0 / t35;
    real_type t37  = alpha__f__XO * t36;
    real_type t40  = atan(t37 * t27 * Fzf__XO * t22 * t21);
    real_type t41  = t10 * t40;
    real_type t45  = 1.0 / t12;
    real_type t46  = t45 * t8;
    real_type t47  = t19 * t19;
    real_type t48  = 1.0 / t47;
    real_type t49  = t48 * t16;
    real_type t50  = Fzf__XO * t22;
    real_type t51  = t50 * t49;
    real_type t52  = t36 * t27;
    real_type t53  = t17 * alpha__f__XO;
    real_type t57  = t50 * t21;
    real_type t58  = t36 * t33;
    real_type t59  = t24 * alpha__f__XO;
    real_type t60  = phi__f__XO * t59;
    real_type t63  = t29 * t22;
    real_type t64  = t30 * Fzf__XO;
    real_type t65  = t64 * t63;
    real_type t66  = t65 * t21;
    real_type t67  = t32 * t32;
    real_type t68  = 1.0 / t67;
    real_type t70  = 1.0 / t35 / t34;
    real_type t71  = t70 * t68;
    real_type t75  = -2 * phi__f__XO * t53 * t52 * t51 - 2 * t60 * t58 * t57 + 2 * t60 * t71 * t66;
    real_type t76  = t16 * t16;
    real_type t78  = alpha__f__XO * alpha__f__XO;
    real_type t80  = t78 * t48 * t76 + 1;
    real_type t81  = 1.0 / t80;
    real_type t84  = -2 * phi__f__XO * t41 * t15 + t81 * t75 * t46;
    real_type t85  = t84 * t84;
    real_type t87  = t40 * t46;
    real_type t88  = cos(t87);
    real_type t91  = ModelPars[49];
    real_type t92  = Fzf__XO * t91;
    real_type t94  = ModelPars[6];
    real_type t99  = t94 * ModelPars[33] + (Fzf__XO - t94) * ModelPars[35];
    real_type t100 = 1.0 / t99;
    real_type t101 = ModelPars[56];
    real_type t102 = t22 * t101;
    real_type t103 = Fzf__XO * t102;
    real_type t106 = atan(alpha__f__XO * t52 * t103);
    real_type t107 = t106 * t46;
    real_type t108 = cos(t107);
    real_type t109 = t108 * t100;
    real_type t110 = t45 * t109;
    real_type t111 = t110 * t92;
    real_type t112 = ModelPars[47];
    real_type t113 = t112 * t35;
    real_type t114 = ModelPars[41];
    real_type t115 = t114 * t9;
    real_type t116 = t115 + 1;
    real_type t117 = t116 * t116;
    real_type t118 = 1.0 / t117;
    real_type t119 = t118 * t99;
    real_type t120 = 1.0 / t112;
    real_type t121 = t120 * t119;
    real_type t126 = 1.0 / t116 * t99;
    real_type t127 = t70 * t120;
    real_type t128 = alpha__f__XO * t127;
    real_type t129 = t128 * t126;
    real_type t130 = t32 * t26;
    real_type t131 = 1.0 / t130;
    real_type t133 = phi__f__XO * t24 * t131;
    real_type t137 = -2 * phi__f__XO * t114 * t37 * t121 + 2 * t133 * t31 * t129;
    real_type t139 = t99 * t99;
    real_type t140 = t118 * t139;
    real_type t141 = t112 * t112;
    real_type t142 = 1.0 / t141;
    real_type t144 = 1.0 / t29;
    real_type t145 = 1.0 / t30;
    real_type t150 = t78 * t32 * t145 * t144 * t142 * t140 + 1;
    real_type t151 = t150 * t150;
    real_type t152 = 1.0 / t151;
    real_type t156 = atan(alpha__f__XO * t36 * t120 * t126);
    real_type t157 = t156 * t112;
    real_type t158 = cos(t157);
    real_type t161 = 1.0 / t117 / t116;
    real_type t163 = t144 * t142;
    real_type t181 = t10 * t106;
    real_type t187 = phi__f__XO * t24;
    real_type t190 = t63 * t101;
    real_type t193 = alpha__f__XO * t70;
    real_type t197 = -2 * t187 * t37 * t33 * Fzf__XO * t102 + 2 * t187 * t193 * t68 * t64 * t190;
    real_type t198 = t101 * t101;
    real_type t201 = 1.0 / (t198 * t78 + 1);
    real_type t204 = -2 * phi__f__XO * t181 * t15 + t201 * t197 * t46;
    real_type t206 = sin(t107);
    real_type t208 = t206 * t204 * t100 * t92;
    real_type t211 = 1.0 / t150;
    real_type t213 = t158 * t211 * t137;
    real_type t217 = t64 * t91;
    real_type t218 = t110 * t217;
    real_type t219 = sin(t157);
    real_type t220 = t219 * t36;
    real_type t221 = t131 * t29;
    real_type t222 = t24 * t221;
    real_type t226 = t30 * t30;
    real_type t227 = t226 * Fzf__XO;
    real_type t231 = t29 * t29;
    real_type t235 = t24 * t24;
    real_type t237 = t9 * t235 / t67 / t32;
    real_type t241 = t29 * t220;
    real_type t243 = t9 * t235 * t68;
    real_type t248 = t137 * t137;
    real_type t253 = t35 * t14;
    real_type t254 = t10 * t219;
    real_type t261 = t114 * t114;
    real_type t269 = t24 * t131 * t30;
    real_type t276 = t226 * t231;
    real_type t279 = 1.0 / t35 / t68 / t276;
    real_type t300 = 1.0 / t13 / t12;
    real_type t301 = t300 * t8;
    real_type t302 = t10 * t10;
    real_type t316 = t9 * t235;
    real_type t321 = 1.0 / t67 / t26;
    real_type t327 = t59 * t58;
    real_type t330 = t231 * t22;
    real_type t333 = 1.0 / t67 / t130;
    real_type t341 = t59 * t71;
    real_type t351 = t219 * t35;
    real_type t354 = t100 * t217;
    real_type t355 = t45 * t108;
    real_type t369 = t14 * t108;
    real_type t386 = t3 * t3;
    real_type t388 = t386 * t9 + 1;
    real_type t389 = t388 * t388;
    real_type t408 = 1.0 / t47 / t19;
    real_type t411 = t17 * t17;
    real_type t419 = t24 * t18;
    real_type t433 = t9 * t235 * alpha__f__XO;
    real_type t454 = t80 * t80;
    real_type t466 = sin(t87);
    real_type t474 = t109 * t92;
    real_type t484 = t204 * t204;
    return -t88 * t85 * t7 * t6 + (-4 * phi__f__XO * t114 * t78 * t32 * t145 * t163 * t161 * t139 + 4 * phi__f__XO * t24 * t78 * t26 * t145 * t163 * t140) * t158 * t152 * t137 * t113 * t111 + 2 * t213 * t112 * t35 * t45 * t208 + 2 * t222 * t220 * t218 + 4 * t237 * t231 * t219 * t70 * t110 * t227 * t91 - 12 * t243 * t241 * t218 + t219 * t152 * t248 * t141 * t35 * t111 - 4 * phi__f__XO * t254 * t253 * t208 - t158 * t211 * (12 * t237 * t276 * alpha__f__XO * t279 * t120 * t126 + 8 * t9 * t261 * t37 * t120 * t161 * t99 - 8 * t269 * t29 * t115 * t128 * t119 + 2 * t269 * t29 * alpha__f__XO * t127 * t126 - 2 * t114 * t37 * t121 - 12 * t243 * t31 * t129) * t113 * t111 + t351 * t45 * t206 * (8 * t9 * t302 * t106 * t301 - 4 * phi__f__XO * t10 * t201 * t197 * t15 - 2 * t181 * t15 + t201 * (12 * t316 * alpha__f__XO * t279 * t333 * t227 * t330 * t101 + 8 * t316 * t37 * t131 * Fzf__XO * t102 - 20 * t316 * t193 * t321 * t64 * t190 + 2 * t341 * t64 * t190 - 2 * t327 * t103) * t46) * t100 * t92 + 4 * t187 * t221 * t213 * t112 * t36 * t355 * t354 - 4 * t133 * t241 * t45 * t206 * t204 * t354 - 8 * t222 * t9 * t254 * t36 * t369 * t354 + 4 * phi__f__XO * t10 * t158 * t211 * t137 * t112 * t35 * t369 * t100 * t92 - 2 * phi__f__XO * t386 * t88 / t389 * t2 - t466 * (8 * t9 * t302 * t40 * t301 - 4 * phi__f__XO * t10 * t81 * t75 * t15 - 2 * t41 * t15 + t81 * (8 * t9 * t411 * alpha__f__XO * t52 * t50 * t408 * t16 + 12 * t433 * t279 * t333 * t227 * t330 * t21 - 8 * t419 * t193 * t68 * t65 * t49 + 8 * t419 * t37 * t33 * t50 * t49 + 8 * t433 * t36 * t131 * t57 - 20 * t433 * t70 * t321 * t66 - 2 * t53 * t52 * t51 - 2 * t327 * t57 + 2 * t341 * t66) * t46 + 4 * phi__f__XO * t17 * t78 * t408 * t76 / t454 * t75 * t46) * t7 * t6 - 2 * t466 * t84 / t388 * t2 + 2 * t254 * t253 * t474 - 8 * t9 * t302 * t219 * t35 * t300 * t474 + t351 * t355 * t484 * t100 * t92;
  }

  real_type
  MidLine::Mzf_D_2_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
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
  MidLine::Mzf_D_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
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
  MidLine::Mzf_D_3_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
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
  MidLine::Mzr( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
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
  MidLine::Mzr_D_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
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
  MidLine::Mzr_D_1_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[50];
    real_type t2   = Fzr__XO * t1;
    real_type t4   = ModelPars[7];
    real_type t6   = ModelPars[36];
    real_type t9   = t4 * ModelPars[34] + (Fzr__XO - t4) * t6;
    real_type t10  = 1.0 / t9;
    real_type t12  = ModelPars[48];
    real_type t13  = phi__XO * phi__XO;
    real_type t16  = ModelPars[54] * t13 + 1;
    real_type t17  = t16 * t16;
    real_type t18  = 1.0 / t17;
    real_type t19  = t18 * t12;
    real_type t20  = ModelPars[56];
    real_type t21  = ModelPars[40];
    real_type t22  = t21 * t20;
    real_type t25  = ModelPars[46] * t13 + 1;
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
    real_type t42  = 1.0 / t30 / t25;
    real_type t44  = 1.0 / t33 / t32;
    real_type t45  = t44 * t42;
    real_type t48  = -alpha__r__XO * t45 * t28 * t39 + t36 * t22;
    real_type t49  = t48 * t19;
    real_type t51  = alpha__r__XO * alpha__r__XO;
    real_type t52  = t20 * t20;
    real_type t54  = t52 * t51 + 1;
    real_type t55  = 1.0 / t54;
    real_type t56  = 1.0 / t16;
    real_type t57  = t56 * t12;
    real_type t60  = atan(t36 * Fzr__XO * t22);
    real_type t61  = t60 * t57;
    real_type t62  = sin(t61);
    real_type t63  = t62 * t55;
    real_type t65  = ModelPars[47];
    real_type t68  = ModelPars[42] * t13 + 1;
    real_type t69  = 1.0 / t68;
    real_type t70  = t69 * t6;
    real_type t71  = 1.0 / t65;
    real_type t73  = alpha__r__XO * t34 * t71;
    real_type t75  = t69 * t9;
    real_type t76  = t44 * t71;
    real_type t80  = t31 * Fzr__XO * t27 * alpha__r__XO;
    real_type t82  = -t80 * t76 * t75 + t73 * t70;
    real_type t83  = t82 * t65;
    real_type t84  = t9 * t9;
    real_type t85  = t68 * t68;
    real_type t86  = 1.0 / t85;
    real_type t88  = t65 * t65;
    real_type t89  = 1.0 / t88;
    real_type t90  = t89 * t86 * t84;
    real_type t91  = 1.0 / t27;
    real_type t92  = 1.0 / t28;
    real_type t94  = t51 * t30;
    real_type t97  = t94 * t92 * t91 * t90 + 1;
    real_type t98  = 1.0 / t97;
    real_type t100 = atan(t73 * t75);
    real_type t101 = t100 * t65;
    real_type t102 = cos(t101);
    real_type t103 = t102 * t98;
    real_type t108 = ModelPars[52];
    real_type t110 = ModelPars[55];
    real_type t112 = atan(phi__XO * t110);
    real_type t113 = 1.0 / t110;
    real_type t115 = t113 * t112 * Fzr__XO * t108;
    real_type t116 = t12 * t12;
    real_type t118 = ModelPars[57];
    real_type t121 = ModelPars[53] * t13 + 1;
    real_type t123 = 1.0 / t121 * t118;
    real_type t124 = t21 * t123;
    real_type t126 = t38 * t123;
    real_type t128 = alpha__r__XO * t44;
    real_type t131 = -t128 * t42 * t28 * t126 + t36 * t124;
    real_type t132 = t131 * t131;
    real_type t133 = t118 * t118;
    real_type t134 = t121 * t121;
    real_type t138 = t51 / t134 * t133 + 1;
    real_type t139 = t138 * t138;
    real_type t146 = atan(alpha__r__XO * t34 * t26 * Fzr__XO * t124);
    real_type t147 = t146 * t57;
    real_type t148 = cos(t147);
    real_type t155 = t27 * t27;
    real_type t156 = t155 * t21;
    real_type t158 = t28 * Fzr__XO;
    real_type t159 = t30 * t30;
    real_type t161 = 1.0 / t159 / t25;
    real_type t163 = t28 * t28;
    real_type t165 = 1.0 / t159;
    real_type t168 = 1.0 / t33 / t165 / t163 / t155;
    real_type t174 = 1.0 / t138;
    real_type t176 = sin(t147);
    real_type t180 = t10 * t1;
    real_type t182 = t55 * t48;
    real_type t184 = sin(t101);
    real_type t185 = t184 * t33 * t62;
    real_type t189 = cos(t61);
    real_type t190 = t56 * t189;
    real_type t191 = t190 * t180;
    real_type t192 = t184 * t34;
    real_type t198 = t65 * t33;
    real_type t208 = t33 * t56;
    real_type t209 = t6 * t6;
    real_type t215 = t189 * t10;
    real_type t222 = 1.0 / t84;
    real_type t239 = t56 * t189 * t222;
    real_type t241 = t82 * t198;
    real_type t247 = t56 * t215 * t2;
    real_type t248 = t97 * t97;
    real_type t249 = 1.0 / t248;
    real_type t303 = t82 * t82;
    real_type t308 = t28 * t1;
    real_type t328 = t48 * t48;
    real_type t329 = t54 * t54;
    real_type t336 = t10 * t308;
    return 2 * t103 * t83 * t33 * t63 * t49 * t10 * t2 - t148 / t139 * t132 * t18 * t116 * t115 - t176 * t174 * (3 * alpha__r__XO * t168 * t161 * t158 * t156 * t123 - 3 * alpha__r__XO * Fzr__XO * t45 * t126) * t57 * t115 + 2 * t185 * t182 * t19 * t180 - 3 * t31 * Fzr__XO * t27 * t192 * t191 - 2 * t102 * t98 * t82 * t198 * t191 - 2 * t209 * t184 * t208 * t189 / t84 / t9 * t2 + t165 * t155 * t184 * t44 * t56 * t215 * t158 * t1 + 2 * t6 * t184 * t208 * t189 * t222 * t1 - 2 * t6 * t184 * t33 * t62 * t182 * t18 * t12 * t222 * t2 + 2 * t6 * t103 * t241 * t239 * t2 + (2 * t6 * t51 * t30 * t92 * t91 * t89 * t86 * t9 - 2 * t94 / t158 * t91 * t90) * t102 * t249 * t241 * t247 + t185 * t55 * (3 * alpha__r__XO * t168 * t161 * t158 * t156 * t20 - 3 * Fzr__XO * t128 * t42 * t39) * t18 * t12 * t10 * t2 - t102 * t98 * (3 * t165 * t28 * t155 * alpha__r__XO * t168 * t71 * t75 - t31 * t27 * t128 * t71 * t75 - 2 * t80 * t76 * t70) * t198 * t247 + t184 * t249 * t303 * t88 * t33 * t247 + 2 * t31 * t27 * t6 * t192 * t239 * t308 - 2 * t176 * t174 * t131 * t56 * t12 * t113 * t112 * t108 + t184 * t33 * t189 / t329 * t328 / t17 / t16 * t116 * t10 * t2 + 2 * t31 * t27 * t184 * t34 * t63 * t49 * t336 - 2 * t31 * t27 * t102 * t98 * t83 * t34 * t190 * t336;
  }

  real_type
  MidLine::Mzr_D_1_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[50];
    real_type t2   = Fzr__XO * t1;
    real_type t4   = ModelPars[7];
    real_type t6   = ModelPars[36];
    real_type t9   = t4 * ModelPars[34] + (Fzr__XO - t4) * t6;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t2;
    real_type t12  = ModelPars[48];
    real_type t13  = phi__XO * phi__XO;
    real_type t14  = ModelPars[54];
    real_type t16  = t14 * t13 + 1;
    real_type t17  = t16 * t16;
    real_type t18  = 1.0 / t17;
    real_type t19  = t18 * t12;
    real_type t20  = ModelPars[56];
    real_type t21  = ModelPars[40];
    real_type t22  = t21 * t20;
    real_type t23  = ModelPars[46];
    real_type t25  = t23 * t13 + 1;
    real_type t26  = 1.0 / t25;
    real_type t27  = t21 * t21;
    real_type t28  = Fzr__XO * Fzr__XO;
    real_type t29  = t28 * t27;
    real_type t30  = t25 * t25;
    real_type t31  = 1.0 / t30;
    real_type t32  = t31 * t29;
    real_type t33  = sqrt(t32);
    real_type t34  = 1.0 / t33;
    real_type t35  = t34 * t26;
    real_type t36  = alpha__r__XO * t35;
    real_type t38  = t27 * t21;
    real_type t39  = t38 * t20;
    real_type t42  = 1.0 / t30 / t25;
    real_type t44  = 1.0 / t33 / t32;
    real_type t45  = t44 * t42;
    real_type t48  = -alpha__r__XO * t45 * t28 * t39 + t36 * t22;
    real_type t51  = alpha__r__XO * alpha__r__XO;
    real_type t52  = t20 * t20;
    real_type t55  = 1.0 / (t52 * t51 + 1);
    real_type t56  = 1.0 / t16;
    real_type t57  = t56 * t12;
    real_type t60  = atan(t36 * Fzr__XO * t22);
    real_type t61  = t60 * t57;
    real_type t62  = sin(t61);
    real_type t64  = t33 * t62 * t55;
    real_type t65  = ModelPars[47];
    real_type t66  = ModelPars[42];
    real_type t68  = t66 * t13 + 1;
    real_type t69  = t68 * t68;
    real_type t70  = 1.0 / t69;
    real_type t71  = t70 * t9;
    real_type t72  = 1.0 / t65;
    real_type t74  = alpha__r__XO * t34;
    real_type t76  = phi__XO * t66 * t74;
    real_type t78  = 1.0 / t68;
    real_type t79  = t78 * t9;
    real_type t80  = t44 * t72;
    real_type t81  = alpha__r__XO * t80;
    real_type t82  = t81 * t79;
    real_type t84  = phi__XO * t23 * t42;
    real_type t85  = t84 * t29;
    real_type t88  = -2 * t76 * t72 * t71 + 2 * t85 * t82;
    real_type t89  = t88 * t65;
    real_type t90  = t9 * t9;
    real_type t91  = t70 * t90;
    real_type t92  = t65 * t65;
    real_type t93  = 1.0 / t92;
    real_type t95  = 1.0 / t27;
    real_type t96  = 1.0 / t28;
    real_type t101 = t51 * t30 * t96 * t95 * t93 * t91 + 1;
    real_type t102 = 1.0 / t101;
    real_type t104 = alpha__r__XO * t34 * t72;
    real_type t106 = atan(t104 * t79);
    real_type t107 = t106 * t65;
    real_type t108 = cos(t107);
    real_type t109 = t108 * t102;
    real_type t113 = t28 * t28;
    real_type t115 = cos(t61);
    real_type t116 = t115 * t10;
    real_type t117 = t56 * t116;
    real_type t119 = sin(t107);
    real_type t121 = t27 * t27;
    real_type t123 = t30 * t30;
    real_type t127 = phi__XO * t23 / t123 / t25;
    real_type t131 = t117 * t2;
    real_type t132 = t65 * t33;
    real_type t133 = t78 * t6;
    real_type t137 = t31 * Fzr__XO;
    real_type t140 = -t137 * t27 * alpha__r__XO * t80 * t79 + t104 * t133;
    real_type t142 = t101 * t101;
    real_type t143 = 1.0 / t142;
    real_type t148 = t95 * t93;
    real_type t166 = ModelPars[52];
    real_type t167 = Fzr__XO * t166;
    real_type t168 = ModelPars[55];
    real_type t170 = atan(phi__XO * t168);
    real_type t171 = 1.0 / t168;
    real_type t172 = t171 * t170;
    real_type t174 = t12 * t172 * t167;
    real_type t175 = ModelPars[57];
    real_type t176 = ModelPars[53];
    real_type t178 = t176 * t13 + 1;
    real_type t180 = 1.0 / t178 * t175;
    real_type t181 = t21 * t180;
    real_type t185 = alpha__r__XO * t44;
    real_type t188 = -t185 * t42 * t28 * t38 * t180 + t36 * t181;
    real_type t190 = t175 * t175;
    real_type t191 = t178 * t178;
    real_type t192 = 1.0 / t191;
    real_type t195 = t51 * t192 * t190 + 1;
    real_type t196 = 1.0 / t195;
    real_type t201 = atan(t74 * t26 * Fzr__XO * t181);
    real_type t202 = t201 * t57;
    real_type t203 = sin(t202);
    real_type t214 = phi__XO * t23;
    real_type t215 = t214 * t74;
    real_type t217 = t28 * Fzr__XO;
    real_type t218 = 1.0 / t123;
    real_type t227 = -2 * phi__XO * t14 * t60 * t19 + t55 * (2 * t214 * t185 * t218 * t217 * t39 - 2 * t215 * t137 * t22) * t57;
    real_type t229 = t62 * t227 * t10;
    real_type t234 = t108 * t102 * t140;
    real_type t237 = t10 * t1;
    real_type t238 = t56 * t115;
    real_type t239 = t34 * t238;
    real_type t241 = t27 * t119;
    real_type t248 = t18 * t12 * t10 * t2;
    real_type t249 = t55 * t48;
    real_type t255 = t28 * t1;
    real_type t265 = 1.0 / t90;
    real_type t266 = t115 * t265;
    real_type t280 = t192 * t175;
    real_type t283 = phi__XO * t176;
    real_type t301 = phi__XO * t176 * alpha__r__XO;
    real_type t305 = t121 * t21;
    real_type t309 = 1.0 / t123 / t30;
    real_type t313 = 1.0 / t33 / t218 / t113 / t121;
    real_type t316 = phi__XO * t23 * alpha__r__XO;
    real_type t332 = t56 * t62;
    real_type t333 = t119 * t33;
    real_type t345 = Fzr__XO * t27;
    real_type t366 = t188 * t56;
    real_type t371 = Fzr__XO * t21;
    real_type t381 = t44 * t218;
    real_type t388 = -2 * phi__XO * t14 * t201 * t19 + t196 * (2 * t316 * t381 * t217 * t38 * t180 - 2 * t316 * t34 * t31 * t371 * t180 - 2 * t301 * t35 * t371 * t280) * t57;
    real_type t390 = cos(t202);
    real_type t394 = t109 * t89 * t64 * t48 * t19 * t11 - 2 * t127 * t121 * t119 * t44 * t117 * t113 * t1 + (-4 * phi__XO * t66 * t51 * t30 * t96 * t148 / t69 / t68 * t90 + 4 * phi__XO * t23 * t51 * t25 * t96 * t148 * t91) * t108 * t143 * t140 * t132 * t131 + 2 * phi__XO * t14 * t203 * t196 * t188 * t18 * t174 + t234 * t65 * t33 * t56 * t229 * t2 + 6 * t84 * t28 * t241 * t239 * t237 + t119 * t33 * t115 * t227 * t249 * t248 + 2 * phi__XO * t14 * t31 * t27 * t119 * t34 * t18 * t116 * t255 + t6 * t109 * t88 * t132 * t56 * t266 * t2 + t119 * t88 * t143 * t140 * t92 * t33 * t131 - t203 * t196 * (8 * phi__XO * t23 * t28 * t185 * t218 * t38 * t180 - 6 * t316 * t313 * t309 * t113 * t305 * t180 - 2 * t283 * t74 * t26 * t21 * t280 + 2 * t301 * t45 * t28 * t38 * t280 - 2 * t215 * t31 * t21 * t180) * t57 * t172 * t167 - t108 * t102 * t88 * t132 * t238 * t237 - t6 * t333 * t332 * t227 * t265 * t2 - t108 * t102 * (-6 * t127 * t217 * t121 * alpha__r__XO * t313 * t72 * t79 + 2 * phi__XO * t66 * t31 * t345 * t81 * t71 - 2 * t76 * t72 * t70 * t6 + 2 * t85 * t81 * t133 + 4 * t84 * t345 * t82) * t132 * t131 - t390 * t388 * t196 * t366 * t174;
    real_type t432 = t195 * t195;
    real_type t443 = t217 * t1;
    real_type t444 = t10 * t443;
    real_type t450 = t214 * t42 * t27;
    real_type t460 = t18 * t115;
    real_type t466 = t168 * t168;
    real_type t469 = 1.0 / (t466 * t13 + 1);
    real_type t517 = t31 * t241 * t34 * t56 * t229 * t255 + t119 * t33 * t62 * t55 * (-6 * t214 * alpha__r__XO * t313 * t309 * t113 * t305 * t20 + 8 * t214 * t28 * alpha__r__XO * t381 * t39 - 2 * t215 * t31 * t22) * t248 - 2 * phi__XO * t14 * t6 * t333 * t18 * t266 * t2 - 4 * t283 * t51 / t191 / t178 * t190 * t203 / t432 * t366 * t12 * t171 * t170 * t167 - 2 * t450 * t119 * t34 * t62 * t249 * t19 * t444 + 2 * t450 * t234 * t65 * t34 * t238 * t444 + 2 * phi__XO * t14 * t333 * t460 * t237 - t203 * t196 * t366 * t12 * t469 * t167 + t333 * t332 * t227 * t237 - t203 * t388 * t171 * t170 * t166 + 2 * phi__XO * t14 * t108 * t102 * t140 * t65 * t33 * t460 * t11 - 2 * t84 * t27 * t6 * t119 * t239 * t265 * t443 - 4 * phi__XO * t14 * t119 * t64 * t48 / t17 / t16 * t12 * t11 - t31 * t27 * t108 * t102 * t89 * t239 * t10 * t255 + t390 * t469 * t166;
    return t394 + t517;
  }

  real_type
  MidLine::Mzr_D_1_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
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
  MidLine::Mzr_D_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
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
  MidLine::Mzr_D_2_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
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
    real_type t38  = atan(alpha__r__XO * t35 * t22);
    real_type t39  = t38 * t13;
    real_type t43  = 1.0 / t15;
    real_type t44  = t43 * t11;
    real_type t47  = alpha__r__XO * t34;
    real_type t48  = phi__XO * t23;
    real_type t51  = t27 * t20;
    real_type t52  = t51 * t19;
    real_type t53  = t28 * Fzr__XO;
    real_type t54  = t30 * t30;
    real_type t55  = 1.0 / t54;
    real_type t59  = 1.0 / t33 / t32;
    real_type t60  = alpha__r__XO * t59;
    real_type t64  = -2 * t48 * t47 * t31 * Fzr__XO * t21 + 2 * t48 * t60 * t55 * t53 * t52;
    real_type t65  = alpha__r__XO * alpha__r__XO;
    real_type t66  = t19 * t19;
    real_type t69  = 1.0 / (t66 * t65 + 1);
    real_type t72  = -2 * phi__XO * t39 * t18 + t69 * t64 * t44;
    real_type t73  = t72 * t72;
    real_type t76  = t38 * t44;
    real_type t77  = cos(t76);
    real_type t78  = t43 * t77;
    real_type t79  = ModelPars[47];
    real_type t80  = ModelPars[42];
    real_type t81  = t80 * t12;
    real_type t82  = t81 + 1;
    real_type t84  = 1.0 / t82 * t9;
    real_type t85  = 1.0 / t79;
    real_type t89  = atan(alpha__r__XO * t34 * t85 * t84);
    real_type t90  = t89 * t79;
    real_type t91  = sin(t90);
    real_type t92  = t91 * t33;
    real_type t95  = t77 * t10;
    real_type t96  = t95 * t2;
    real_type t97  = t33 * t17;
    real_type t98  = t13 * t91;
    real_type t103 = 1.0 / t16 / t15;
    real_type t105 = t13 * t13;
    real_type t112 = ModelPars[52] * Fzr__XO;
    real_type t113 = ModelPars[55];
    real_type t115 = atan(phi__XO * t113);
    real_type t116 = t115 * t112;
    real_type t117 = 1.0 / t113;
    real_type t118 = ModelPars[57];
    real_type t119 = ModelPars[53];
    real_type t120 = t119 * t12;
    real_type t121 = t120 + 1;
    real_type t123 = 1.0 / t121 * t118;
    real_type t128 = atan(t47 * t26 * Fzr__XO * t20 * t123);
    real_type t129 = t13 * t128;
    real_type t133 = t121 * t121;
    real_type t134 = 1.0 / t133;
    real_type t135 = t134 * t118;
    real_type t136 = Fzr__XO * t20;
    real_type t137 = t136 * t135;
    real_type t138 = t119 * alpha__r__XO;
    real_type t142 = t136 * t123;
    real_type t143 = t34 * t31;
    real_type t144 = t23 * alpha__r__XO;
    real_type t145 = phi__XO * t144;
    real_type t148 = t53 * t51;
    real_type t149 = t148 * t123;
    real_type t150 = t59 * t55;
    real_type t154 = -2 * phi__XO * t138 * t35 * t137 - 2 * t145 * t143 * t142 + 2 * t145 * t150 * t149;
    real_type t155 = t118 * t118;
    real_type t158 = t65 * t134 * t155 + 1;
    real_type t159 = 1.0 / t158;
    real_type t162 = -2 * phi__XO * t129 * t18 + t159 * t154 * t44;
    real_type t163 = t162 * t162;
    real_type t165 = t128 * t44;
    real_type t166 = cos(t165);
    real_type t169 = t43 * t95;
    real_type t170 = t169 * t2;
    real_type t171 = t79 * t33;
    real_type t172 = t82 * t82;
    real_type t173 = 1.0 / t172;
    real_type t174 = t173 * t9;
    real_type t175 = t85 * t174;
    real_type t179 = t59 * t85;
    real_type t180 = alpha__r__XO * t179;
    real_type t181 = t180 * t84;
    real_type t182 = t30 * t25;
    real_type t183 = 1.0 / t182;
    real_type t185 = phi__XO * t23 * t183;
    real_type t189 = -2 * phi__XO * t80 * t47 * t175 + 2 * t185 * t29 * t181;
    real_type t191 = t9 * t9;
    real_type t192 = t173 * t191;
    real_type t193 = t79 * t79;
    real_type t194 = 1.0 / t193;
    real_type t196 = 1.0 / t27;
    real_type t197 = 1.0 / t28;
    real_type t202 = t65 * t30 * t197 * t196 * t194 * t192 + 1;
    real_type t203 = t202 * t202;
    real_type t204 = 1.0 / t203;
    real_type t205 = cos(t90);
    real_type t208 = 1.0 / t172 / t82;
    real_type t210 = t196 * t194;
    real_type t229 = sin(t76);
    real_type t231 = t229 * t72 * t10 * t2;
    real_type t234 = 1.0 / t202;
    real_type t236 = t205 * t234 * t189;
    real_type t240 = t53 * t1;
    real_type t241 = t169 * t240;
    real_type t242 = t91 * t34;
    real_type t243 = t183 * t27;
    real_type t244 = t23 * t243;
    real_type t248 = t28 * t28;
    real_type t249 = t248 * Fzr__XO;
    real_type t253 = t27 * t27;
    real_type t257 = t23 * t23;
    real_type t259 = t12 * t257 / t54 / t30;
    real_type t263 = t27 * t242;
    real_type t265 = t12 * t257 * t55;
    real_type t271 = t80 * t80;
    real_type t279 = t23 * t183 * t28;
    real_type t286 = t248 * t253;
    real_type t289 = 1.0 / t33 / t55 / t286;
    real_type t310 = t189 * t189;
    real_type t319 = t103 * t11;
    real_type t333 = t12 * t257;
    real_type t338 = 1.0 / t54 / t25;
    real_type t344 = t144 * t143;
    real_type t347 = t253 * t20;
    real_type t350 = 1.0 / t54 / t182;
    real_type t358 = t144 * t150;
    real_type t370 = t10 * t240;
    real_type t384 = t17 * t77;
    real_type t401 = t113 * t113;
    real_type t403 = t401 * t12 + 1;
    real_type t404 = t403 * t403;
    real_type t423 = 1.0 / t133 / t121;
    real_type t426 = t119 * t119;
    real_type t434 = t23 * t120;
    real_type t448 = t12 * t257 * alpha__r__XO;
    real_type t469 = t158 * t158;
    real_type t481 = sin(t165);
    return t92 * t78 * t73 * t10 * t2 + 2 * t98 * t97 * t96 - 8 * t12 * t105 * t91 * t33 * t103 * t96 - t166 * t163 * t117 * t116 + (-4 * phi__XO * t80 * t65 * t30 * t197 * t210 * t208 * t191 + 4 * phi__XO * t23 * t65 * t25 * t197 * t210 * t192) * t205 * t204 * t189 * t171 * t170 + 2 * t236 * t79 * t33 * t43 * t231 + 2 * t244 * t242 * t241 + 4 * t259 * t253 * t91 * t59 * t169 * t249 * t1 - 12 * t265 * t263 * t241 - t205 * t234 * (8 * t12 * t271 * t47 * t85 * t208 * t9 + 12 * t259 * t286 * alpha__r__XO * t289 * t85 * t84 - 8 * t279 * t27 * t81 * t180 * t174 + 2 * t279 * t27 * alpha__r__XO * t179 * t84 - 2 * t80 * t47 * t175 - 12 * t265 * t29 * t181) * t171 * t170 + t91 * t204 * t310 * t193 * t33 * t170 - 4 * phi__XO * t98 * t97 * t231 + t92 * t43 * t229 * (8 * t12 * t105 * t38 * t319 - 4 * phi__XO * t13 * t69 * t64 * t18 - 2 * t39 * t18 + t69 * (12 * t333 * alpha__r__XO * t289 * t350 * t249 * t347 * t19 + 8 * t333 * t47 * t183 * Fzr__XO * t21 - 20 * t333 * t60 * t338 * t53 * t52 + 2 * t358 * t53 * t52 - 2 * t344 * t22) * t44) * t10 * t2 + 4 * t48 * t243 * t236 * t79 * t34 * t78 * t370 - 4 * t185 * t263 * t43 * t229 * t72 * t370 - 8 * t244 * t12 * t98 * t34 * t384 * t370 + 4 * phi__XO * t13 * t205 * t234 * t189 * t79 * t33 * t384 * t10 * t2 - 2 * phi__XO * t401 * t166 / t404 * t112 - t481 * (8 * t12 * t105 * t128 * t319 - 4 * phi__XO * t13 * t159 * t154 * t18 - 2 * t129 * t18 + t159 * (8 * t12 * t426 * alpha__r__XO * t35 * t136 * t423 * t118 + 12 * t448 * t289 * t350 * t249 * t347 * t123 + 8 * t434 * t47 * t31 * t136 * t135 - 8 * t434 * t60 * t55 * t148 * t135 + 8 * t448 * t34 * t183 * t142 - 20 * t448 * t59 * t338 * t149 - 2 * t138 * t35 * t137 - 2 * t344 * t142 + 2 * t358 * t149) * t44 + 4 * phi__XO * t119 * t65 * t423 * t155 / t469 * t154 * t44) * t117 * t116 - 2 * t481 * t162 / t403 * t112;
  }

  real_type
  MidLine::Mzr_D_2_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
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
    real_type t276 = Fzr__XO * t27;
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
  MidLine::Mzr_D_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
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
  MidLine::Mzr_D_3_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
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
  MidLine::Mxf( real_type t__XO ) const {
    return 0;
  }

  real_type
  MidLine::Mxf_D( real_type t__XO ) const {
    return 0;
  }

  real_type
  MidLine::Mxf_DD( real_type t__XO ) const {
    return 0;
  }

  real_type
  MidLine::Mxr( real_type t__XO ) const {
    return 0;
  }

  real_type
  MidLine::Mxr_D( real_type t__XO ) const {
    return 0;
  }

  real_type
  MidLine::Mxr_DD( real_type t__XO ) const {
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
  MidLine::H_eval(
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
    real_type t2   = ModelPars[122];
    real_type t3   = X__[8];
    real_type t4   = t2 - t3;
    real_type t5   = X__[3];
    real_type t6   = sin(t5);
    real_type t8   = X__[9];
    real_type t9   = X__[6];
    real_type t10  = t9 * t8;
    real_type t11  = cos(t5);
    real_type t13  = X__[15];
    real_type t14  = ModelPars[22];
    real_type t15  = ModelPars[144];
    real_type t16  = t14 + t15;
    real_type t17  = t16 * t13;
    real_type t21  = X__[4] + ModelPars[64];
    real_type t22  = sin(t21);
    real_type t26  = X__[5];
    real_type t28  = X__[11];
    real_type t29  = t9 * t28;
    real_type t30  = -t4;
    real_type t31  = t30 * t13;
    real_type t33  = cos(t21);
    real_type t35  = X__[10];
    real_type t37  = -t35 * t13 - t8;
    real_type t39  = t9 * t30;
    real_type t40  = t13 * t28;
    real_type t45  = X__[27];
    real_type t46  = Fzf(t28, t45);
    real_type t47  = X__[16];
    real_type t48  = X__[2];
    real_type t49  = X__[0];
    real_type t50  = X__[1];
    real_type t51  = X__[25];
    real_type t52  = X__[26];
    real_type t53  = alpha__f(t48, t49, t50, t8, t35, t13, t51, t52);
    real_type t54  = X__[18];
    real_type t55  = lambda__f(t48, t47, t49, t50, t8, t35, t13, t54, t51, t52);
    real_type t56  = Fyf(t46, t47, t53, t55);
    real_type t60  = t9 * t35;
    real_type t68  = -t8 * t13 + t35;
    real_type t72  = Fxf(t46, t47, t53, t55);
    real_type t74  = t22 * t11;
    real_type t77  = Mzf(t46, t47, t53);
    real_type t83  = X__[19];
    real_type t84  = t48 * t83;
    real_type t85  = ModelPars[16];
    real_type t86  = t85 * t84;
    real_type t87  = X__[34];
    real_type t88  = cos(t47);
    real_type t89  = t88 * t87;
    real_type t93  = sin(t47);
    real_type t96  = X__[22];
    real_type t102 = X__[20];
    real_type t107 = t6 * t9;
    real_type t109 = -t16;
    real_type t113 = t102 * t9;
    real_type t119 = Q__[0];
    real_type t120 = Mxf(t119);
    real_type t130 = t13 * t120;
    real_type t136 = X__[13];
    real_type t137 = t6 * t136;
    real_type t138 = X__[14];
    real_type t139 = t138 * t11;
    real_type t141 = X__[30];
    real_type t142 = Fzr(t138, t141);
    real_type t143 = X__[12];
    real_type t144 = X__[28];
    real_type t145 = X__[29];
    real_type t146 = alpha__r(t48, t49, t50, t143, t136, t144, t145);
    real_type t147 = X__[17];
    real_type t148 = lambda__r(t48, t5, t49, t136, t147, t144);
    real_type t149 = Fxr(t142, t5, t146, t148);
    real_type t152 = Fyr(t142, t5, t146, t148);
    real_type t154 = Mzr(t142, t5, t146);
    real_type t155 = t154 * t6;
    real_type t156 = ModelPars[74];
    real_type t157 = ModelPars[23];
    real_type t158 = ModelPars[214];
    real_type t159 = -t157 + t158;
    real_type t160 = t159 * t156;
    real_type t161 = ModelPars[232];
    real_type t163 = ModelPars[153];
    real_type t164 = t161 * t160 + t163;
    real_type t165 = t48 * t48;
    real_type t167 = t11 * t11;
    real_type t168 = t167 * t165 * t164;
    real_type t169 = t158 + t161 - t157;
    real_type t171 = t158 - t161 - t157;
    real_type t173 = t157 * t157;
    real_type t174 = ModelPars[76];
    real_type t175 = t174 * t173;
    real_type t176 = ModelPars[14];
    real_type t177 = ModelPars[19];
    real_type t178 = -t171 * t169 * t156 - t175 + t176 - t177;
    real_type t181 = t11 * t83 * t178 * t48;
    real_type t182 = t83 * t83;
    real_type t183 = t164 * t182;
    real_type t184 = -t168 + t181 + t183;
    real_type t186 = X__[7];
    real_type t187 = cos(t186);
    real_type t188 = t187 * t187;
    real_type t190 = sin(t186);
    real_type t192 = t167 * t178 * t165;
    real_type t194 = t11 * t164 * t84;
    real_type t195 = 4 * t194;
    real_type t196 = t182 * t178;
    real_type t199 = t187 * (t192 + t195 - t196) * t190;
    real_type t205 = t33 * t33;
    real_type t218 = t157 * t174;
    real_type t219 = t160 - t218;
    real_type t220 = t165 * t219;
    real_type t221 = t220 * t26 * t167;
    real_type t226 = ModelPars[67];
    real_type t228 = 2 * t156 * t84 * t161 * t26 - t226 * t219;
    real_type t230 = t161 * t156;
    real_type t231 = X__[21];
    real_type t232 = t231 * t230;
    real_type t233 = 2 * t232;
    real_type t234 = t49 * t219;
    real_type t237 = t6 * (t233 + t234) * t48;
    real_type t238 = t26 * t220;
    real_type t239 = t48 * t50;
    real_type t240 = t230 * t239;
    real_type t242 = t182 * t26 * t219;
    real_type t246 = t26 * t156;
    real_type t250 = t48 * t26;
    real_type t257 = -t159;
    real_type t258 = 2 * t257;
    real_type t260 = 2 * t218;
    real_type t261 = t258 * t156 + t260;
    real_type t262 = t231 * t261;
    real_type t264 = t161 * t49 * t156;
    real_type t265 = t262 + t264;
    real_type t268 = t165 * t26;
    real_type t269 = t230 * t268;
    real_type t270 = t50 * t219;
    real_type t271 = t48 * t270;
    real_type t272 = t182 * t26;
    real_type t273 = t230 * t272;
    real_type t274 = t165 * t246 * t161 * t167 + t11 * (t156 * t161 * t226 + 2 * t250 * t83 * t219) - t6 * t265 * t48 - t269 - t271 - t273;
    real_type t286 = t142 * t143 * t11;
    real_type t290 = t161 * t161;
    real_type t293 = ModelPars[15];
    real_type t294 = ModelPars[17];
    real_type t295 = 2 * t290 * t156 + t176 - t177 + t293 + t294;
    real_type t298 = X__[33];
    real_type t300 = alpha__crw(t119);
    real_type t301 = sin(t300);
    real_type t304 = t298 * ModelPars[116];
    real_type t309 = X__[23];
    real_type t313 = -t149 * (t137 - t139 + t26) - t152 * t143 * t6 + t155 - t205 * (4 * t188 * t184 + 2 * t168 - 2 * t181 - 2 * t183 - 2 * t199) - t33 * (t22 * (t188 * (2 * t192 + 8 * t194 - 2 * t196) + 4 * t187 * t184 * t190 - t192 - t195 + t196) + t187 * (t11 * t228 - t221 + t237 + t238 - t240 + t242) - t274 * t190) - t22 * (t187 * t274 - (-t11 * t228 + t221 - t237 - t238 + t240 - t242) * t190) + t286 + 2 * t188 * t184 - t199 + t168 - t11 * t295 * t84 - t301 * t157 * t298 - t183 + t304 - (ModelPars[72] * t186 + t309 * ModelPars[32]) * ModelPars[29];
    real_type t322 = t22 * (t26 * t11 - t28);
    real_type t323 = t11 * t14;
    real_type t336 = t15 * t14;
    real_type t337 = ModelPars[75];
    real_type t338 = t337 * t336;
    real_type t339 = t15 * t15;
    real_type t340 = t337 * t339;
    real_type t341 = ModelPars[183];
    real_type t342 = ModelPars[215];
    real_type t345 = t342 * (t14 + t342) * t341;
    real_type t346 = ModelPars[155];
    real_type t347 = ModelPars[156];
    real_type t348 = ModelPars[12];
    real_type t349 = t338 + t340 + t345 - t346 + t347 - t348 + t85;
    real_type t350 = t165 * t349;
    real_type t355 = t337 * t2 * t15;
    real_type t358 = t341 * t342 * ModelPars[233];
    real_type t359 = ModelPars[152];
    real_type t360 = t337 * t3 * t15 - t355 + t358 + t359;
    real_type t361 = t360 * t83;
    real_type t362 = t48 * t11;
    real_type t385 = t26 * t9;
    real_type t386 = t15 * t337;
    real_type t388 = t342 * t341 + t386;
    real_type t389 = t83 * t388;
    real_type t390 = t389 * t385;
    real_type t397 = t9 * t388;
    real_type t400 = 2 * t348;
    real_type t401 = ModelPars[157];
    real_type t402 = t400 - t85 + t346 - t347 + t401;
    real_type t407 = t239 * t397;
    real_type t408 = t346 / 2;
    real_type t409 = t347 / 2;
    real_type t410 = t401 / 2;
    real_type t411 = t85 / 2;
    real_type t414 = X__[24];
    real_type t415 = t414 * t386;
    real_type t416 = t54 * t85;
    real_type t417 = t416 / 2;
    real_type t418 = t102 * (t338 + t340 + t345 - t408 + t409 + t410 + t411) + t415 - t417;
    real_type t426 = t165 * t167;
    real_type t428 = t26 * t397 * t426;
    real_type t431 = t226 * t397;
    real_type t436 = t48 * t49;
    real_type t457 = t388 * t48;
    real_type t458 = t48 * t6;
    real_type t459 = t26 * t458;
    real_type t463 = t102 * t349;
    real_type t473 = 2 * t415;
    real_type t482 = X__[36];
    real_type t483 = -t72 * (t33 * (t11 * t68 + t6 * t28) + (t322 + t323) * t13) - t56 * (t33 * (t11 * t37 - t6 * t40) + t322 + t323) + t77 * t11 * t33 - t205 * (t167 * t350 - t182 * t349 - 2 * t362 * t361) * t9 - t33 * (-2 * t22 * (t165 * t360 * t167 / 2 + t362 * t349 * t83 - t360 * t182 / 2) * t9 - t46 * t8 * t6 - 2 * t11 * t48 * (-t6 * t360 * t48 / 2 + t390 + t360 * t102) + t6 * (-2 * t397 * t231 * t48 - t48 * t402 * t83 + t130) + t407 - 2 * t83 * t418) - t22 * (t46 * (t26 * t6 + t35) - t428 + t11 * (t165 * t349 * t6 - 2 * t48 * t418 - t431) + t6 * t397 * t436 + t397 * (t165 + t182) * t26 + 2 * t102 * t361 + t120) - t46 * t6 * t14 + 2 * t426 * t9 * (t340 + t338 / 2 + t342 * (t342 + t14 / 2) * t341 - t346 + t347 - t348 + t85) - t11 * (t459 + t49) * t457 - t6 * (-2 * t9 * t48 * (t463 + t415 - t417) + t226 * t388) - t9 * (t350 + t182 * t388 * t14 + t102 * (t463 + t473 - t416)) + 2 * t231 * t389 - t96 * ModelPars[151] + t482;
    real_type t486 = t72 * t28;
    real_type t487 = t56 * t28;
    real_type t494 = -t171 * t169 * t156 - t175 + t176 - t177;
    real_type t496 = t190 * t187;
    real_type t497 = t257 * t156;
    real_type t499 = t161 * t497 - t163;
    real_type t500 = t499 * t496;
    real_type t502 = ModelPars[65];
    real_type t503 = cos(t502);
    real_type t504 = t503 * t503;
    real_type t505 = -t494;
    real_type t506 = t505 * t504;
    real_type t507 = sin(t502);
    real_type t509 = t499 * t507 * t503;
    real_type t510 = 2 * t509;
    real_type t511 = ModelPars[209];
    real_type t512 = cos(t511);
    real_type t513 = t512 * t512;
    real_type t514 = ModelPars[73];
    real_type t515 = ModelPars[69];
    real_type t516 = t515 * t515;
    real_type t517 = t516 * t514;
    real_type t518 = ModelPars[13];
    real_type t519 = ModelPars[18];
    real_type t520 = -t517 - t518 + t519;
    real_type t521 = t520 * t513;
    real_type t522 = ModelPars[150];
    real_type t523 = t514 * t522;
    real_type t525 = t512 * t515 * t523;
    real_type t526 = ModelPars[145];
    real_type t528 = sin(t511);
    real_type t529 = t528 * t515;
    real_type t530 = t529 * t514 * t526;
    real_type t531 = t3 * t3;
    real_type t532 = t531 * t337;
    real_type t533 = t532 / 2;
    real_type t535 = t337 * t2 * t3;
    real_type t536 = ModelPars[197];
    real_type t537 = t536 * t337;
    real_type t540 = (t2 - t536 / 2) * t537;
    real_type t541 = t514 * t515;
    real_type t542 = t522 * t541;
    real_type t543 = ModelPars[24];
    real_type t544 = ModelPars[27];
    real_type t545 = t544 * t544;
    real_type t546 = ModelPars[28];
    real_type t547 = t546 * t546;
    real_type t548 = -t545 + t547;
    real_type t550 = t548 * t543 / 2;
    real_type t551 = ModelPars[9];
    real_type t552 = t551 / 2;
    real_type t553 = ModelPars[11];
    real_type t554 = t553 / 2;
    real_type t555 = t494 * t188 + 2 * t500 + t506 - t510 + t517 + t518 - t519 + t521 - t525 + t530 + t533 - t535 + t540 + t542 + t550 + t552 - t554;
    real_type t557 = t167 * t48;
    real_type t560 = t309 - t102;
    real_type t561 = t499 * t560;
    real_type t562 = t188 * t561;
    real_type t563 = 2 * t562;
    real_type t564 = t560 * t505;
    real_type t565 = t496 * t564;
    real_type t566 = -t499;
    real_type t567 = 2 * t566;
    real_type t568 = t567 * t504;
    real_type t570 = t507 * t505 * t503;
    real_type t571 = -t520;
    real_type t573 = t526 * t541;
    real_type t575 = (t571 * t528 + t573) * t512;
    real_type t576 = t529 * t523;
    real_type t578 = t337 * t16 * t3;
    real_type t579 = t16 * t337;
    real_type t580 = t536 * t579;
    real_type t581 = t543 * t546;
    real_type t582 = t544 * t581;
    real_type t583 = ModelPars[0];
    real_type t584 = t568 - t570 + t575 + t576 - t578 + t580 - t582 - t573 - t583;
    real_type t585 = t102 * t584;
    real_type t586 = t499 * t309;
    real_type t588 = t414 * t337 * t4;
    real_type t590 = -t563 - t565 + t585 + t586 + t588 / 2;
    real_type t594 = t48 - t83;
    real_type t595 = t48 + t83;
    real_type t597 = -t567;
    real_type t601 = t190 * t505 * t187 + t597 * t188 + t568 - t570 - t573 + t575 + t576 - t578 + t580 - t582 - t583;
    real_type t606 = t6 * t83;
    real_type t607 = t188 * t564;
    real_type t609 = t496 * t561;
    real_type t612 = 2 * t504 * t494;
    real_type t613 = 4 * t509;
    real_type t614 = 2 * t571;
    real_type t615 = t513 * t614;
    real_type t616 = 2 * t525;
    real_type t617 = 2 * t530;
    real_type t618 = 2 * t535;
    real_type t619 = t536 * t2;
    real_type t621 = t536 * t536;
    real_type t623 = t337 * (-2 * t619 + t621);
    real_type t624 = 2 * t517;
    real_type t625 = 2 * t542;
    real_type t627 = -t548 * t543;
    real_type t628 = 2 * t518;
    real_type t629 = 2 * t519;
    real_type t630 = t612 + t613 + t615 + t616 - t617 - t532 + t618 + t623 - t624 - t625 + t627 - t628 + t629 - t551 + t553;
    real_type t631 = t102 * t630;
    real_type t632 = t309 * t505;
    real_type t634 = t337 * t16 * t414;
    real_type t635 = -2 * t607 + 4 * t609 + t631 + t632 + t634;
    real_type t655 = t83 * t26;
    real_type t656 = t156 * t187;
    real_type t657 = t161 * t656;
    real_type t660 = t497 + t218;
    real_type t661 = t190 * t660;
    real_type t665 = t161 * t156 * t503;
    real_type t666 = t660 * t507;
    real_type t667 = t512 * t541;
    real_type t668 = t337 * t3;
    real_type t669 = t665 + t666 + t667 - t541 + t537 - t668 - t581;
    real_type t670 = t669 * t26;
    real_type t671 = t83 * t670;
    real_type t673 = t15 * t579;
    real_type t674 = t342 * t342;
    real_type t675 = t341 * t674;
    real_type t677 = t341 * t342 * t14;
    real_type t678 = t673 + t675 + t677 - t348 + t85 - t346 + t347;
    real_type t679 = t678 * t9;
    real_type t682 = t414 * t9 * t386;
    real_type t683 = -t360;
    real_type t684 = t683 * t96;
    real_type t689 = t660 * t26;
    real_type t693 = -t26 * t660 * t309 + t102 * t689 - t232;
    real_type t694 = t693 / 2;
    real_type t697 = t102 * t161 * t246;
    real_type t699 = t309 * t161 * t246;
    real_type t701 = -t660 * t231 - t697 + t699;
    real_type t702 = t701 / 2;
    real_type t704 = t683 * t9;
    real_type t706 = t660 * t503;
    real_type t707 = t528 * t541;
    real_type t709 = t161 * t156 * t507;
    real_type t710 = t544 * t543;
    real_type t711 = t706 - t707 - t709 + t710;
    real_type t712 = t711 * t26;
    real_type t713 = t102 * t712;
    real_type t716 = t503 * t231 * t230;
    real_type t718 = t337 * t414;
    real_type t719 = t26 * t718;
    real_type t722 = (t666 + t667 + t537 - t668 - t581 - t541) * t231;
    real_type t729 = t309 - t83 - t102;
    real_type t731 = t309 + t83 - t102;
    real_type t734 = t660 * t731 * t729 * t26 + t165 * t689 - t240;
    real_type t736 = t660 * t50;
    real_type t737 = t48 * t736;
    real_type t741 = -t731 * t729 * t161 * t246 - t269 - t737;
    real_type t746 = t182 * t712;
    real_type t751 = t102 * t102;
    real_type t753 = t414 * t102;
    real_type t756 = 2 * t337 * t26 * t753;
    real_type t757 = ModelPars[1];
    real_type t758 = t49 * t49;
    real_type t759 = t758 * t757;
    real_type t761 = ModelPars[149] * t759;
    real_type t764 = t231 * t83;
    real_type t765 = t187 * t660;
    real_type t768 = t190 * t83;
    real_type t770 = 2 * t232 * t768;
    real_type t772 = t9 * t678 * t165;
    real_type t776 = 2 * t231 * t711 * t83;
    real_type t777 = t102 * t678;
    real_type t782 = t660 * t226;
    real_type t785 = t230 * t226 * t190;
    real_type t789 = 2 * t48 * t9 * (t777 + t415 - t417);
    real_type t790 = t226 * t711;
    real_type t795 = t161 * t246;
    real_type t798 = t83 * t712;
    real_type t800 = t102 * t704;
    real_type t823 = t182 * t670;
    real_type t825 = t85 * t9 * t54;
    real_type t831 = ModelPars[143] * t759;
    real_type t835 = 2 * t657 * t764;
    real_type t839 = t9 * t683 * t165;
    real_type t845 = t9 * t683 * t751;
    real_type t847 = (t348 - t411 + t408 - t409 + t410) * t96;
    real_type t851 = t85 * t96 * t54;
    real_type t856 = t190 * t782;
    real_type t858 = 2 * t800 - 2 * t847;
    real_type t860 = t226 * t669;
    real_type t870 = t26 * t26;
    real_type t871 = t870 * t543;
    real_type t872 = t545 * t543;
    real_type t873 = ModelPars[10];
    real_type t874 = t506 - t510 - t871 + t521 + t617 - t872 + t517 + t518 - t519 + t551 - t873;
    real_type t877 = t9 * t231;
    real_type t885 = t543 * t231;
    real_type t886 = t26 * t885;
    real_type t907 = t543 * t48 * t50 * t26;
    real_type t913 = t758 * t26 * t757;
    real_type t920 = t187 * t768;
    real_type t923 = t505 / 2;
    real_type t929 = t547 * t543;
    real_type t935 = t504 * t923 - t509 + t520 * t513 / 2 - t525 + t533 - t535 + t540 + t518 / 2 - t519 / 2 + t929 / 2 + t517 / 2 + t551 / 4 + t873 / 4 - t553 / 4 + t542;
    real_type t937 = t257 * t257;
    real_type t947 = t147 * t294;
    real_type t961 = t486 - t13 * t487 + t149 * t138 - t205 * (4 * t557 * t83 * t555 + t11 * (-4 * t6 * t48 * t590 + 2 * t601 * t595 * t594) + 2 * t635 * t606) - t33 * (t22 * (-4 * t557 * t601 * t83 + t11 * (2 * t6 * t635 * t48 + 2 * t595 * t555 * t594) + 4 * t6 * t83 * t590) - 2 * t167 * t48 * (t102 * t679 - 2 * t657 * t655 - 2 * t661 * t655 + 2 * t671 + t682 - t684) + t11 * (-4 * t6 * t48 * (t187 * t694 + t190 * t702 + t83 * t704 - t713 / 2 + t716 / 2 - t719 / 2 + t722 / 2) + t187 * t734 + t190 * t741 - t711 * t268 + t48 * t669 * t50 + t746 - 2 * t83 * (t673 + t675 + t411 - t408 + t409 + t410 + t677) * t96 - t751 * t712 - t756 + t761) + t6 * (-2 * t765 * t764 + t770 - t772 + t182 * t679 + t776 - (t777 + t473 - t416) * t113) + t187 * t782 - t785 + t789 - t790) - t22 * (-2 * t167 * t48 * (2 * t765 * t655 + t678 * t96 - 2 * t795 * t768 - 2 * t798 + t800) + t11 * (4 * t6 * t48 * (t187 * t702 - t190 * t694 + t83 * t679 + t102 * t670 / 2 + t231 * t711 / 2) - t187 * t741 + t190 * t734 - t165 * t670 - t711 * t239 + t823 + t83 * (t825 - 2 * t684) - t751 * t670 + t831) + t6 * (2 * t83 * t231 * t669 + 2 * t102 * t847 + t182 * t704 - 2 * t661 * t764 - t835 - t839 - t845 + t851) + t230 * t226 * t187 + t856 + t48 * t858 - t860) - t46 * t8 + t142 * t143 + 2 * t167 * t48 * (-t188 * t505 * t83 + 2 * t496 * t83 * t499 + t83 * t874 + t388 * (t26 * t96 + t877)) - t11 * (2 * t6 * (-t563 - t565 + 2 * t390 + t585 + t886 + t586) * t48 - 2 * t188 * t595 * t594 * t499 - t187 * t505 * t595 * t190 * t594 + t165 * (t597 * t504 + t570 + (t520 * t528 - t573) * t512 + t578 - t576 - t580 + t582 + t573 + t583) - t907 + t182 * t584 + 2 * t102 * t4 * t718 + t913) - t6 * (2 * t188 * t505 * t83 * t560 - 4 * t920 * t561 - t407 + 4 * (t102 * t935 + t309 * (-t937 * t156 / 2 - t294 / 4 - t293 / 4 + t176 / 4 - t177 / 4 - t175 / 2) - t947 / 4 - t416 / 4) * t83) - t48 * (t83 * (t551 - t873 + t553) + 2 * t388 * t877) + t130;
    real_type t981 = (-t56 * (t22 * (t11 * t10 + t6 * t4 - t17) + t33 * (t26 * t11 * t9 + t6 * t16 - t29 - t31) + t6 * t37 + t11 * (t39 + t40) - t13 * t26) - t72 * (t22 * (t6 * (-t29 - t31) - t11 * t60 + t14 + t15) + t33 * (t6 * t17 - t2 + t3) + t6 * t68 - t11 * t28 + t26) - t77 * (t9 * t74 - t6) - t22 * (t46 * (t6 * t10 + t30 * t11) - t6 * (-t86 + t89) * t9 + t11 * (-t85 * t48 * t96 + t87 * t9 * t93) + t85 * t102 * t83 * t9) - t33 * (t46 * (t26 * t107 + t11 * t109 + t60) - t48 * t85 * t11 * t113 - t83 * t96 * t85 + t9 * t120) - t46 * (t11 * t8 + t6 * t39) + t93 * t87 * t6 + (-t86 + t89 + t130) * t11) * L__[9] + t313 * L__[7] + t483 * L__[6] + t961 * L__[4] + t83 * L__[19] + t102 * L__[20] + t231 * L__[21] + t96 * L__[22] + t309 * L__[23] + t414 * L__[24] + t51 * L__[25] + t52 * L__[26] + t45 * L__[27];
    real_type t989 = X__[31];
    real_type t992 = X__[32];
    real_type t995 = X__[38];
    real_type t996 = cos(t995);
    real_type t998 = sin(t995);
    real_type t1012 = X__[35];
    real_type t1017 = ALIAS_maxTorque(t147);
    real_type t1034 = t458 + 2 * t309 - 2 * t102;
    real_type t1035 = t1034 * t505;
    real_type t1037 = t499 * t1034;
    real_type t1040 = t506 - t510 + t521 - t525 + t530 + t533 - t535 + t540 + t517 + t542 + t550 + t518 - t519 + t552 - t554;
    real_type t1052 = 2 * t573;
    real_type t1055 = 2 * t576;
    real_type t1058 = 2 * t582;
    real_type t1059 = 2 * t583;
    real_type t1061 = t102 * (4 * t566 * t504 - 2 * t570 + (t614 * t528 + t1052) * t512 + t1055 - 2 * t578 + 2 * t580 - t1058 - t1052 - t1059);
    real_type t1062 = t309 * t597;
    real_type t1089 = t309 * t26;
    real_type t1090 = t102 * t26;
    real_type t1091 = t49 / 2;
    real_type t1092 = t459 + t1089 - t1090 + t1091;
    real_type t1100 = t1089 - t1090 + t1091;
    real_type t1112 = -t83 * t693;
    real_type t1121 = -t701 * t83;
    real_type t1130 = t22 * (-2 * t11 * (-t6 * t584 * t48 - t496 * t1035 - 2 * t188 * t1037 + t1061 + t1062 + t588) * t48 - 4 * (t607 - 2 * t609 + t102 * t1040 - t309 * t923 - t634 / 2) * t83) - 2 * t704 * t426 + 2 * t11 * t48 * (-t187 * t161 * t1092 * t156 + t6 * (t219 * t190 + t537 - t541 - t581 + t665 + t666 + t667 - t668) * t250 - t190 * t660 * t1100 - (t1090 - t1091) * t669) + t187 * (-t6 * t226 * t230 + 2 * t1112) + t6 * (-t48 * t858 - t856 + t860) + 2 * t190 * t1121 + t839 + t845 + t102 * (2 * t798 - 2 * t847) + t83 * (-2 * t716 + 2 * t719 - 2 * t722) - t851;
    real_type t1136 = t156 * t190;
    real_type t1144 = t660 * t49;
    real_type t1182 = 4 * t530;
    real_type t1183 = 2 * t872;
    real_type t1184 = t612 + t613 + t615 - t1182 + t1183 - t624 - t628 + t629 - t551 + t873 + t553;
    real_type t1212 = Mxr(t119);
    real_type t1213 = -t13 * t486 - 2 * t102 * t83 * t584 - t205 * (-2 * t11 * (t6 * t1040 * t48 - t188 * t1035 + 2 * t496 * t1037 + t631 + t632 + t634) * t48 - 2 * (-4 * t562 - 2 * t565 + t1061 + t1062 + t588) * t83) - t33 * t1130 - t22 * (2 * t679 * t426 + 2 * t11 * (t187 * t660 * t1092 - t6 * (t161 * t1136 + t706 - t707 - t709 + t710) * t250 - t190 * t161 * t1100 * t156 + t713 - t503 * t1144 / 2 + t719 - (-t707 - t709 + t710) * t49 / 2) * t48 + t187 * (t6 * t782 - 2 * t1121) + t6 * (-t785 + t789 - t790) + 2 * t190 * t1112 - t772 - t9 * t678 * t751 + t102 * (2 * t671 - 2 * t9 * (t415 - t417)) + t776) - t487 - t152 * t138 + t46 * t35 + t142 * t136 - t11 * (-t188 * t1034 * t505 * t48 + 2 * t500 * t1034 * t48 + t6 * t874 * t165 + t48 * (-t543 * t49 * t26 + t102 * t1184 - t309 * t295 - t416 + 2 * t634 - t947) + t431) - (-t543 * t226 * t26 - t49 * t9 * t457) * t6 - t83 * (2 * t886 + 2 * t586) - 2 * t428 + 2 * t920 * t564 + 4 * t188 * t83 * t561 + t385 * t388 * t165 + t120 + t1212;
    real_type t1216 = t136 * t182;
    real_type t1217 = ModelPars[147];
    real_type t1218 = ModelPars[187];
    real_type t1219 = t1218 * t1217;
    real_type t1221 = ModelPars[121];
    real_type t1225 = -2 * t1218 * t1217 * t1221 - 2 * t138 * t1219 - 2 * t141;
    real_type t1228 = 2 * t145 * t1219;
    real_type t1229 = t1218 * t1218;
    real_type t1230 = t136 * t1229;
    real_type t1233 = -t1221 - t138;
    real_type t1237 = -t136 * t1219 - t145;
    real_type t1252 = t187 * t560;
    real_type t1255 = t309 - t102 - t1218;
    real_type t1257 = t309 - t102 + t1218;
    real_type t1264 = t1217 * t190;
    real_type t1288 = 2 * t1218 * t1217 * t231;
    real_type t1289 = t1229 * t26;
    real_type t1295 = t144 * L__[28] + t145 * L__[29] + t141 * L__[30] + t989 * L__[31] + t992 * L__[32] + (t49 * t998 + t50 * t996) * L__[37] + (-U__[0] * ModelPars[160] - t87) * L__[33] + (-U__[1] * ModelPars[162] - t1012) * L__[35] + (t1017 * U__[2] - t304) * L__[34] + (U__[3] * ModelPars[208] - t482) * L__[36] + t1213 * L__[3] + (-t11 * (t83 * t1225 + t1216 - t1228 - t1230) + 2 * t6 * (t182 * t1233 / 2 + t83 * t1237 + (t1217 * t141 + t1218 * t1221 / 2 + t138 * t1218 / 2) * t1218)) * L__[17] + (t33 * (t1257 * t1255 * t190 - 2 * t1252 * t1219) * t157 - t22 * t157 * (2 * t560 * t1218 * t1264 + t187 * t1257 * t1255) - t11 * (-2 * t141 * t1219 - t1229 * t1221 - t138 * t1229 - t182 * t1233 - 2 * t83 * t1237) - t6 * (-t83 * t1225 - t1216 + t1228 + t1230) - t1288 - t1289 - t1229 * (-ModelPars[119] + t1221)) * L__[18];
    real_type t1298 = -t1257;
    real_type t1299 = -t1255;
    real_type t1302 = -t560;
    real_type t1325 = 2 * t16;
    real_type t1327 = t102 * t1218;
    real_type t1329 = t3 * t1229;
    real_type t1330 = t1229 * t2;
    real_type t1332 = 2 * t414 * t1219;
    real_type t1344 = t751 * t109 + t102 * (2 * t1218 * t1217 * t2 - 2 * t1218 * t1217 * t3 - 2 * t414) + t16 * t1229;
    real_type t1346 = ModelPars[118];
    real_type t1347 = ModelPars[120];
    real_type t1348 = t1346 - t1347;
    real_type t1350 = -t1348;
    real_type t1351 = 2 * t1350;
    real_type t1354 = t992 * t992;
    real_type t1355 = t1354 * t1348;
    real_type t1356 = t1229 * t1350;
    real_type t1357 = t83 * t992 * t1351 + t182 * t1348 + t1355 + t1356;
    real_type t1360 = t1217 * t1347;
    real_type t1362 = -2 * t1217 * t1346 + 2 * t1360;
    real_type t1363 = t1218 * t1362;
    real_type t1366 = -t1218 * t1362;
    real_type t1371 = t182 * (t1347 + t28);
    real_type t1375 = t83 * (2 * t35 * t1219 + 2 * t52);
    real_type t1376 = t28 * t1229;
    real_type t1377 = t1347 * t1229;
    real_type t1380 = 2 * t1218 * t1217 * t45;
    real_type t1388 = t35 * t182;
    real_type t1392 = 2 * t1218 * t1360 + 2 * t28 * t1219 + 2 * t45;
    real_type t1394 = t35 * t1229;
    real_type t1397 = 2 * t1218 * t1217 * t52;
    real_type t1403 = t83 - t992;
    real_type t1404 = t1403 * t1348;
    real_type t1408 = t83 - t992 - t1218;
    real_type t1409 = t83 - t992 + t1218;
    real_type t1410 = t1409 * t1408;
    real_type t1426 = t96 * t1217;
    real_type t1434 = -t1325;
    real_type t1442 = t1217 * t1351;
    real_type t1469 = t187 * t33;
    real_type t1487 = t1218 * t1426;
    real_type t1488 = 2 * t1487;
    real_type t1492 = t9 * t1219 + t96;
    real_type t1510 = t83 * t992;
    real_type t1516 = -t1403;
    real_type t1553 = -t1409;
    real_type t1555 = -t1408;
    real_type t1578 = t165 * t30;
    real_type t1580 = t16 * t83;
    real_type t1600 = t9 * t15;
    real_type t1606 = t96 * t15;
    real_type t1634 = t337 * t751;
    real_type t1641 = -t72 * (t33 * t13 * t6 + t22) - t56 * (-t13 * t22 + t33 * t6) - t205 * (t167 * t1578 + 2 * t362 * t1580 - t30 * t182) * t337 - t33 * (-2 * t22 * t337 * (-t167 * t165 * t16 / 2 + t362 * t30 * t83 + t16 * t182 / 2) - t46 * t11 + t337 * (t167 * t268 + t11 * (t6 * t165 * t1600 + t226) - t268 - t272 - t6 * t436 + 2 * t83 * t1606)) + 2 * t22 * (t11 * t48 * (t655 - t1606) + t6 * t48 * (t83 * t1600 + t231) - t239 / 2) * t337 + 2 * t337 * t362 * t1580 - 2 * t6 * t48 * t337 * t30 * t102 + t337 * t1578 - t3 * (-t1634 + ModelPars[71]) - t2 * t1634 - t414 * ModelPars[31];
    real_type t1650 = t161 * t157 - t161 * t158;
    real_type t1651 = 8 * t1650;
    real_type t1653 = 8 * t163;
    real_type t1654 = t156 * t1651 - t1653;
    real_type t1658 = -t156 * t1651 + t1653;
    real_type t1660 = t102 * t1654 + t309 * t1658;
    real_type t1661 = t48 * t1660;
    real_type t1664 = t158 * t157;
    real_type t1666 = t158 * t158;
    real_type t1669 = 4 * t173 - 8 * t1664 + 4 * t1666 - 4 * t290;
    real_type t1671 = 4 * t175;
    real_type t1672 = 4 * t176;
    real_type t1673 = 4 * t177;
    real_type t1674 = t156 * t1669 + t1671 - t1672 + t1673;
    real_type t1678 = -t156 * t1669 - t1671 + t1672 - t1673;
    real_type t1686 = 4 * t571;
    real_type t1688 = 4 * t573;
    real_type t1691 = 4 * t109;
    real_type t1694 = 4 * t576;
    real_type t1697 = t536 * t14 + t536 * t15;
    real_type t1698 = 4 * t1697;
    real_type t1700 = 4 * t582;
    real_type t1701 = 4 * t583;
    real_type t1704 = 4 * t1650;
    real_type t1706 = 4 * t163;
    real_type t1707 = t156 * t1704 - t1706;
    real_type t1708 = t309 * t1707;
    real_type t1709 = t2 * t718;
    real_type t1710 = 2 * t1709;
    real_type t1712 = t337 * t414 * t3;
    real_type t1713 = 2 * t1712;
    real_type t1718 = t83 * t1678;
    real_type t1719 = t188 * t48;
    real_type t1723 = t187 * t190 * t48;
    real_type t1728 = -t1686;
    real_type t1730 = 4 * t525;
    real_type t1731 = 2 * t532;
    real_type t1732 = 4 * t535;
    real_type t1735 = 4 * t619 - 2 * t621;
    real_type t1738 = 4 * t542;
    real_type t1743 = 2 * t551;
    real_type t1745 = t503 * t507 * t1658 + t504 * t1674 + t513 * t1728 + t337 * t1735 + 2 * t548 * t543 + t1182 - t1730 + t1731 - t1732 + t1738 + t1743 + 4 * t517 + 4 * t518 - 4 * t519 - 2 * t553;
    real_type t1752 = t102 * t1678 + t309 * t1674;
    real_type t1753 = t83 * t1752;
    real_type t1758 = 2 * t173;
    real_type t1759 = 4 * t1664;
    real_type t1760 = 2 * t1666;
    real_type t1762 = -t1758 + t1759 - t1760 + 2 * t290;
    real_type t1764 = 2 * t175;
    real_type t1765 = 2 * t176;
    real_type t1766 = 2 * t177;
    real_type t1767 = t156 * t1762 - t1764 + t1765 - t1766;
    real_type t1768 = t309 * t1767;
    real_type t1771 = t414 * t337 * t1434 + t102 * t1745 + t1768;
    real_type t1777 = -t156 * t1704 + t1706;
    real_type t1778 = t182 * t1777;
    real_type t1785 = -t156 * t1762 + t1764 - t1765 + t1766;
    real_type t1797 = t504 * t1707 + t503 * t507 * t1785 + (-t614 * t528 - t1052) * t512 + t3 * t337 * t1325 - t1055 - 2 * t337 * t1697 + t1058 + t1052 + t1059;
    real_type t1823 = t504 * t1654 + t503 * t507 * t1674 + (t1728 * t528 - t1688) * t512 - t3 * t337 * t1691 - t1694 - t337 * t1698 + t1700 + t1688 + t1701;
    real_type t1843 = t504 * t1767;
    real_type t1845 = t503 * t507 * t1707;
    real_type t1846 = t1843 + t1845 + t615 + t616 - t532 + t618 - t617 + t623 - t624 - t625 + t627 - t628 + t629 - t551 + t553;
    real_type t1856 = -t258 * t156 - t260;
    real_type t1857 = t309 * t1856;
    real_type t1860 = t48 * (t102 * t26 * t261 + t26 * t1857 - t233);
    real_type t1862 = 2 * t697;
    real_type t1863 = 2 * t699;
    real_type t1864 = t231 * t1856;
    real_type t1872 = 2 * t709;
    real_type t1873 = 2 * t707;
    real_type t1874 = 2 * t710;
    real_type t1875 = t503 * t1856 + t1872 + t1873 - t1874;
    real_type t1878 = 2 * t719;
    real_type t1879 = 2 * t716;
    real_type t1883 = 2 * t512 * t231 * t541;
    real_type t1885 = -2 * t668 - 2 * t581 - 2 * t541 + 2 * t537;
    real_type t1894 = 4 * t257;
    real_type t1896 = 4 * t218;
    real_type t1897 = t156 * t1894 + t1896;
    real_type t1899 = t190 * t84;
    real_type t1904 = -t156 * t1894 - t1896;
    real_type t1914 = -t336 - t339;
    real_type t1915 = 2 * t1914;
    real_type t1916 = t337 * t1915;
    real_type t1917 = 2 * t675;
    real_type t1918 = 2 * t677;
    real_type t1919 = 2 * t85;
    real_type t1920 = 2 * t346;
    real_type t1921 = 2 * t347;
    real_type t1922 = t1916 - t1917 - t1918 + t400 - t1919 + t1920 - t1921;
    real_type t1928 = 2 * t386 * t96 * t3;
    real_type t1930 = 2 * t355 - 2 * t358 - 2 * t359;
    real_type t1942 = t503 * t1864;
    real_type t1945 = 2 * t507 * t231 * t230;
    real_type t1947 = 2 * t707 - 2 * t710;
    real_type t1948 = t231 * t1947;
    real_type t1963 = t309 * t309;
    real_type t1964 = t1963 * t660;
    real_type t1966 = t1090 * t1857 + t26 * t1964 + t751 * t689 - t240 + t242;
    real_type t1969 = t230 * t26 * t1963;
    real_type t1970 = t309 * t102;
    real_type t1972 = 2 * t795 * t1970;
    real_type t1974 = t230 * t26 * t751;
    real_type t1984 = -t581 - t541 + t537;
    real_type t1992 = t219 * t503 + t707 + t709 - t710;
    real_type t1999 = t309 * t261;
    real_type t2002 = t48 * (t102 * t26 * t1856 + t26 * t1999 + t233 + t234);
    real_type t2008 = 2 * t9 * t360;
    real_type t2011 = t503 * t261 - t1872 - t1873 + t1874;
    real_type t2014 = -t233 + t1144;
    real_type t2019 = -t231 * t1885;
    real_type t2043 = 2 * t665;
    real_type t2045 = 2 * t667;
    real_type t2046 = 2 * t541;
    real_type t2047 = 2 * t537;
    real_type t2048 = 2 * t668;
    real_type t2049 = 2 * t581;
    real_type t2050 = t1856 * t507 - t2043 - t2045 + t2046 - t2047 + t2048 + t2049;
    real_type t2103 = t219 * t507 - t537 + t541 + t581 - t665 - t667 + t668;
    real_type t2118 = t507 * t261 + t2043 + t2045 - t2046 + t2047 - t2048 - t2049;
    real_type t2135 = t102 * t1777 + t1708;
    real_type t2143 = -t388;
    real_type t2148 = 2 * t886;
    real_type t2150 = -2 * t1650;
    real_type t2152 = 2 * t163;
    real_type t2153 = t156 * t2150 + t2152;
    real_type t2165 = 2 * t871 + t1843 + t1845 + t615 - t1182 + t1183 - t624 - t628 + t629 - t1743 + 2 * t873;
    real_type t2167 = 2 * t2143;
    real_type t2168 = t96 * t2167;
    real_type t2185 = -t337 * t1735 + t1730 - t1731 + t1732 - t1738 + t1843 + t1845 - t551 + t553 + t615 - t624 - t628 + t629 - t873 - 2 * t929;
    real_type t2201 = t173 - 2 * t1664 + t1666 - t290;
    real_type t2222 = -t2167;
    real_type t2223 = t9 * t2222;
    real_type t2227 = -t68 * t72 - t37 * t56 - t149 * t136 - t152 * t143 + t77 + t154 - t205 * (t167 * (t188 * t1661 + t496 * t48 * (t102 * t1674 + t309 * t1678) + t48 * (t102 * (t504 * t1658 + t503 * t507 * t1678 + t512 * (t528 * t1686 + t1688) + t3 * t337 * t1691 + t1694 + t337 * t1698 - t1700 - t1688 - t1701) + t1708 + t1710 - t1713)) + t11 * (t6 * (t1723 * t83 * t1654 + t48 * t83 * t1745 + t1719 * t1718) + t188 * t1753 + t496 * t83 * t1660 + t83 * t1771) + t6 * (t496 * t182 * t1767 + t188 * t1778 + t182 * t1797)) - t33 * (t22 * (t167 * (t188 * t48 * t1752 + t496 * t1661 + t48 * t1771) + t11 * (t6 * (t1719 * t83 * t1658 + t48 * t83 * t1823 + t1723 * t1718) + t188 * t83 * (t102 * t1658 + t309 * t1654) + t496 * t1753 + t83 * (t102 * t1823 + t309 * t1777 - t1710 + t1713)) + t6 * (t188 * t182 * t1785 + t496 * t1778 + t182 * t1846)) + t167 * (t187 * t1860 + t190 * t48 * (-t1862 + t1863 + t1864) + t48 * (t102 * t26 * t1875 + 4 * t83 * t683 * t9 + t231 * t1885 + t507 * t262 - t1878 + t1879 + t1883)) + t11 * (t6 * (4 * t657 * t26 * t84 + t1899 * t26 * t1897 + t48 * (t83 * t26 * (t507 * t1904 - 4 * t537 + 4 * t541 + 4 * t581 - 4 * t665 - 4 * t667 + 4 * t668) + t102 * t9 * t1922 - 2 * t682 - t1928 + t96 * t1930)) + t187 * t83 * t262 - t770 + t182 * t9 * (t337 * t1914 + t346 - t347 + t348 - t675 - t677 - t85) + t83 * (t1942 + t1945 + t1948) + t751 * t9 * (-t337 * t1914 - t346 + t347 - t348 + t675 + t677 + t85) + t102 * t9 * (t473 - t416)) + t6 * (t187 * t1966 + t190 * (t271 - t1969 + t1972 + t273 - t1974) + t48 * (t230 * t50 * t503 - t337 * t50 * t3 + t541 * t50 * t512 + t50 * t1984 + t507 * t736) + t746 + t83 * t96 * (t1916 - t1917 - t1918 - t85 + t346 - t347 - t401) + t751 * t26 * t1992 - t756 + t761) + t187 * t2002 + t190 * t48 * (t1862 - t1863 + t262 + t264) + t48 * (t83 * t2008 + t102 * t26 * t2011 + t1878 + t503 * t2014 + t507 * (t1864 - t264) - t1883 + t2019 - t541 * t49 * t528 + t544 * t49 * t543)) - t22 * (t167 * (t187 * t48 * (t1862 - t1863 + t262) + t190 * t1860 + t48 * (t83 * t9 * (4 * t337 * t1914 + 4 * t346 - 4 * t347 + 4 * t348 - 4 * t675 - 4 * t677 - 4 * t85) + t102 * t26 * t2050 + t1942 + t1945 + t1948)) + t11 * (t6 * (t187 * t84 * t26 * t1904 + 4 * t795 * t1899 + t48 * (t83 * t26 * (t503 * t1897 - 4 * t707 - 4 * t709 + 4 * t710) + t102 * t2008 + t96 * t1922)) + t835 + t768 * t262 + t182 * t9 * t360 + t83 * (t507 * t1864 - t1879 - t1883 + t2019) + t845 - t102 * t96 * t402 - t851) + t6 * (t187 * (t737 + t1969 - t1972 - t273 + t1974) + t190 * t1966 + t48 * (t507 * t50 * t230 + t528 * t50 * t541 - t543 * t50 * t544 + t503 * t270) + t823 + t83 * (-t96 * t1930 + t1928 + t825) + t751 * t26 * t2103 + t831) + t187 * t48 * (-t1862 + t1863 + t1864 - t264) + t190 * t2002 + t48 * (t83 * t9 * (-t337 * t1915 + t1917 + t1918 + t1919 - t1920 + t1921 - t400) + t102 * t26 * t2118 + t503 * t265 + t507 * t2014 + t541 * t49 * t512 - t231 * t1947 - t337 * t49 * t3 + t49 * t1984)) - t167 * (t188 * t48 * t2135 + t496 * t48 * (t102 * t1767 + t309 * t1785) + t48 * (4 * t655 * t9 * t2143 + t102 * t1797 + t309 * t2153 - t2148)) - t11 * (t6 * (t1719 * t83 * t1785 + t1723 * t83 * t1777 + t48 * (t9 * t231 * t2167 + t83 * t2165 + t26 * t2168)) + t188 * t83 * (t102 * t1785 + t1768) + t496 * t83 * t2135 + t407 + t83 * (t102 * t2185 + t309 * (t156 * (t1758 - t1759 + t1760) + t1764 + t294 - t176 + t293 + t177) + t416 + t947)) - t6 * (t188 * t182 * (-t156 * t2150 - t2152) + t496 * t182 * (t156 * t2201 + t175 - t176 + t177) - t907 + t182 * (t504 * t2153 + t503 * t507 * (-t156 * t2201 - t175 + t176 - t177) + t575 + t3 * t337 * t109 + t576 + t337 * t1697 - t582 - t573 - t583) + t102 * (-2 * t1712 + 2 * t1709) + t913) - t48 * (t655 * t2223 - t1710 + t1713 + t2148);
    real_type t2230 = t560 * t560;
    real_type t2231 = t182 + t2230;
    real_type t2232 = t2231 * t161;
    real_type t2234 = t2231 * t219;
    real_type t2240 = t560 * t219;
    real_type t2244 = t102 * t1992;
    real_type t2254 = 2 * t718;
    real_type t2268 = t26 * t543;
    real_type t2269 = t182 * t2268;
    real_type t2274 = t9 * t2143;
    real_type t2275 = t182 * t2274;
    real_type t2277 = 2 * t83 * t885;
    real_type t2285 = t165 * t230;
    real_type t2287 = t751 * t230;
    real_type t2289 = 2 * t1970 * t230;
    real_type t2290 = t1963 * t230;
    real_type t2291 = t182 * t230 + t2285 + t2287 - t2289 + t2290;
    real_type t2293 = t751 * t660;
    real_type t2294 = t102 * t1857;
    real_type t2296 = t165 * t660;
    real_type t2299 = t751 * t2103;
    real_type t2301 = t165 * t2103;
    real_type t2304 = t83 * t261;
    real_type t2307 = t102 * t2304 + t83 * t1857;
    real_type t2314 = -2 * t102 * t83 * t230 + 2 * t83 * t309 * t230;
    real_type t2328 = -2 * t102 * t48 * t230 + 2 * t309 * t48 * t230;
    real_type t2336 = t48 * t102 * t2118;
    real_type t2348 = 2 * t337 * t753;
    real_type t2356 = t83 * t2050;
    real_type t2387 = t48 * (t102 * t1856 + t1999);
    real_type t2392 = 2 * t102 * t230 - 2 * t309 * t230;
    real_type t2403 = 2 * t11 * t84 * t230 + t1964 + t2293 + t2294 + t2296;
    real_type t2441 = X__[37];
    real_type t2442 = Q__[1];
    real_type t2444 = t2442 * t2441 - 1;
    real_type t2445 = t49 * t996;
    real_type t2446 = t50 * t998;
    real_type t2448 = 1.0 / (t2445 - t2446);
    real_type t2449 = t2448 * t2444;
    real_type t2450 = LatSlipRear(t146);
    real_type t2452 = (-t33 * (-2 * t1302 * t1218 * t1264 + t187 * t1299 * t1298) * t157 - t22 * (2 * t187 * t1302 * t1219 + t1299 * t1298 * t190) * t157 - 2 * t1218 * t1217 * t144 - t1229 * t143) * L__[16] + (-t33 * (t1327 * t1217 * t1325 + t4 * t751 + t1329 - t1330 + t1332) - t22 * t1344 - t11 * (t88 * t1357 + t93 * (t83 * t1363 + t992 * t1366) + t1371 + t1375 - t1376 - t1377 - t1380) - t6 * (t88 * (t992 * t1363 + t83 * t1366) + t93 * t1357 - t1388 + t83 * t1392 + t1394 + t1397) - t1289 - t1288) * L__[15] + (-t11 * (-2 * t88 * t1404 * t1219 - t93 * t1348 * t1410 - t83 * t1392 + t1388 - t1394 - t1397) - t6 * (-2 * t93 * t1404 * t1219 + t88 * t1348 * t1410 + t1371 + t1375 - t1376 - t1377 - t1380) - (t1218 * t9 + 2 * t1426) * t1218 * t15) * L__[14] + (-t22 * (t1327 * t1217 * t1434 + t30 * t751 - t1329 + t1330 - t1332) - t33 * t1344 - t93 * (t13 * (t1355 + t1356) + t989 * t1218 * t1442) - t88 * (t13 * t992 * t1218 * t1442 + t992 * t989 * t1351) + t1229 * t8 + 2 * t1218 * t1217 * t51) * L__[13] + (-t149 * (-t157 * t187 * t22 + t157 * t190 * t33 + t137 - t139 + t26) - t152 * (-t157 * t190 * t22 - t157 * t1469 + t143) * t6 - t142 * t1469 * t11 * t157 - t142 * t74 * t190 * t157 - t11 * t294 * t84 + t286 + t155 + t304 + t1012) * L__[10] + (-t33 * (t9 * (t751 - t1229) - t1488) - 2 * t22 * t1492 * t102 - t11 * (t13 * (-t182 + t1229) + 2 * t1218 * t1217 * t989) + 2 * (t13 * t1219 + t989) * t606) * L__[11] + (-t22 * (t88 * (t11 * (t9 * (t182 - 2 * t1510 + t751 + t1354 - t1229) - t1488) - 2 * t1492 * t1516 * t6) + 2 * (t11 * t1492 * t1516 - t6 * (t9 * (-t751 / 2 - t1354 / 2 + t1510 + t1229 / 2 - t182 / 2) + t1487)) * t93) + 2 * t33 * t102 * (t88 * (t1516 * t107 + t11 * t1492) + (-t11 * t1516 * t9 + t1492 * t6) * t93) - t88 * (-2 * t11 * t1516 * t1219 - t1555 * t1553 * t6) - (-2 * t1516 * t1218 * t1217 * t6 + t11 * t1555 * t1553) * t93) * L__[12] + t1641 * L__[8] + t2227 * L__[5] + (-t33 * (t11 * (-t656 * t2232 + t190 * t2234 + (t182 + t751) * t669) + 2 * t6 * (t190 * t560 * t230 + t187 * t2240 + t2244 - t718) * t83) - t22 * (t11 * (-t187 * t2234 - t1136 * t2232 + t182 * t1992 + (t2244 - t2254) * t102) - 2 * t6 * (t102 * t669 + t1252 * t230 - t190 * t2240) * t83) + t46 + t142 - t11 * (t83 * t96 * t2222 - t2269) - t6 * (t2275 - t2277) - t543 * t226) * L__[2] + (t13 * t72 + t56 + t152 - t33 * (t6 * (t187 * t2291 + t190 * (t182 * t660 + t1964 + t2293 + t2294 + t2296) + t2299 + t182 * t2103 + t2301) + t11 * (t102 * t83 * t1875 - 2 * t414 * t83 * t337 + t187 * t2307 + t190 * t2314) + t187 * t2328 + t190 * (t102 * t48 * t1856 + t48 * t1999) + t2336) - t22 * (t6 * (t187 * (t102 * t1999 + t182 * t219 + t1963 * t219 + t751 * t219 + t220) + t190 * t2291 + t751 * t711 + t2348 + t182 * t711 + t165 * t711) + t11 * (t102 * t2356 - t187 * t2314 + t190 * t2307) + t187 * (t102 * t48 * t261 + t48 * t1857) + t190 * t2328 + t102 * t48 * t1875 - 2 * t337 * t48 * t414) - t6 * (t83 * t2168 + t543 * t268 + t2269) - t11 * (t165 * t2274 + t2275 - t2277) - t543 * t436) * L__[1] + (t72 - t13 * t56 + t149 - t33 * (t6 * (t187 * t2387 + t190 * t48 * t2392 + t48 * (t102 * t2011 + t2254)) + t187 * t2403 + t190 * (t362 * t2304 - t2285 - t2287 + t2289 - t2290) + t362 * t2356 + t165 * t1992 + t751 * t1992 - t2348) - t22 * (t6 * (-t187 * t48 * t2392 + t190 * t2387 + t2336) + t187 * (t362 * t83 * t1856 + t2285 + t2287 - t2289 + t2290) + t190 * t2403 + t362 * t83 * t2011 + t2301 + t2299) - t6 * t48 * (t83 * t2223 + 2 * t885) - t11 * t48 * (2 * t83 * t2268 + t2168) - t759 + t543 * t239) * L__[0] - t2450 * t2449;
    real_type t2453 = ModelPars[8];
    real_type t2454 = 1.0 / t2453;
    real_type t2457 = RearWheelContact((t142 - t2453) * t2454);
    real_type t2460 = 1.0 / ModelPars[158];
    real_type t2462 = OnlyTractionRear(t298 * t2460);
    real_type t2465 = OnlyBrakingFront(-t87 * t2460);
    real_type t2468 = OnlyBrakingRear(-t1012 * t2460);
    real_type t2480 = roadRightLateralBorder(Q__[3] - t2441);
    real_type t2482 = MaxRollAngle(t5);
    real_type t2484 = MaxSteerAngle(t9);
    real_type t2486 = LatSlipFront(t53);
    real_type t2488 = LongSlipFront(t55);
    real_type t2492 = FrontWheelContact((t46 - t2453) * t2454);
    real_type t2494 = LongSlipRear(t148);
    real_type t2498 = roadLeftLateralBorder(t2441 + Q__[2]);
    real_type t2502 = ModelPars[177] * ModelPars[177];
    real_type t2506 = pow(t231 - ModelPars[178], 2);
    real_type t2509 = ModelPars[166] * ModelPars[166];
    real_type t2513 = pow(t96 - ModelPars[167], 2);
    real_type t2516 = ModelPars[242] * ModelPars[242];
    real_type t2520 = pow(t989 - ModelPars[243], 2);
    real_type t2523 = ModelPars[174] * ModelPars[174];
    real_type t2527 = pow(t309 - ModelPars[175], 2);
    real_type t2530 = ModelPars[191] * ModelPars[191];
    real_type t2534 = pow(t83 - ModelPars[192], 2);
    real_type t2537 = ModelPars[246] * ModelPars[246];
    real_type t2541 = pow(t992 - ModelPars[194], 2);
    real_type t2544 = ModelPars[198] * ModelPars[198];
    real_type t2548 = pow(t414 - ModelPars[199], 2);
    real_type t2551 = ModelPars[210] * ModelPars[210];
    real_type t2555 = pow(t102 - ModelPars[211], 2);
    real_type t2558 = ModelPars[216] * ModelPars[216];
    real_type t2562 = pow(t51 - ModelPars[217], 2);
    real_type t2565 = ModelPars[220] * ModelPars[220];
    real_type t2569 = pow(t144 - ModelPars[221], 2);
    real_type t2572 = ModelPars[224] * ModelPars[224];
    real_type t2576 = pow(t52 - ModelPars[225], 2);
    real_type t2579 = ModelPars[228] * ModelPars[228];
    real_type t2583 = pow(t145 - ModelPars[229], 2);
    real_type t2586 = ModelPars[234] * ModelPars[234];
    real_type t2590 = pow(t45 - ModelPars[235], 2);
    real_type t2593 = ModelPars[238] * ModelPars[238];
    real_type t2597 = pow(t141 - ModelPars[239], 2);
    real_type t2599 = t2506 / t2502 + t2513 / t2509 + t2520 / t2516 + t2527 / t2523 + t2534 / t2530 + t2541 / t2537 + t2548 / t2544 + t2555 / t2551 + t2562 / t2558 + t2569 / t2565 + t2576 / t2572 + t2583 / t2579 + t2590 / t2586 + t2597 / t2593;
    real_type t2603 = ModelPars[77] * ModelPars[77];
    real_type t2605 = t2441 * t2441;
    real_type t2610 = -t2457 * t2449 - t2462 * t2449 - t2465 * t2449 - t2468 * t2449 + 1.0 / t2444 * (t2442 * (t48 * t2441 + t2445 - t2446) - t48) * L__[38] - t2480 * t2449 - t2482 * t2449 - t2484 * t2449 - t2486 * t2449 - t2488 * t2449 - t2492 * t2449 - t2494 * t2449 - t2498 * t2449 - t2448 * t2444 * (ModelPars[140] + ModelPars[137] * t2599 + t2605 / t2603);
    return t981 + t1295 + t2452 + t2610;
  }

  /*\
   |   ___               _ _   _
   |  | _ \___ _ _  __ _| | |_(_)___ ___
   |  |  _/ -_) ' \/ _` | |  _| / -_|_-<
   |  |_| \___|_||_\__,_|_|\__|_\___/__/
  \*/

  real_type
  MidLine::penalties_eval(
    NodeType const     & NODE__,
    U_const_pointer_type U__,
    P_const_pointer_type P__
  ) const {
    integer     i_segment = NODE__.i_segment;
    real_type const * Q__ = NODE__.q;
    real_type const * X__ = NODE__.x;
    Road2D::SegmentClass const & segment = pRoad->getSegmentByIndex(i_segment);
    real_type t1   = X__[37];
    real_type t5   = X__[38];
    real_type t6   = cos(t5);
    real_type t7   = X__[0];
    real_type t9   = sin(t5);
    real_type t10  = X__[1];
    real_type t14  = 1.0 / (-t10 * t9 + t7 * t6) * (Q__[1] * t1 - 1);
    real_type t16  = 1.0 / ModelPars[158];
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
    real_type t78  = roadRightLateralBorder(Q__[3] - t1);
    real_type t82  = roadLeftLateralBorder(t1 + Q__[2]);
    return -t19 * t14 - t23 * t14 - t27 * t14 - t36 * t14 - t43 * t14 - t54 * t14 - t61 * t14 - t64 * t14 - t69 * t14 - t72 * t14 - t74 * t14 - t78 * t14 - t82 * t14;
  }

  real_type
  MidLine::control_penalties_eval(
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
    real_type t14  = 1.0 / (X__[0] * t6 - X__[1] * t9) * (X__[37] * Q__[1] - 1);
    real_type t18  = t__oControl(U__[2], ModelPars[165], ModelPars[190]);
    real_type t21  = ModelPars[184];
    real_type t22  = ModelPars[30];
    real_type t23  = b__f__oControl(U__[0], t21, t22);
    real_type t26  = b__r__oControl(U__[1], t21, t22);
    real_type t29  = ModelPars[202];
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
  MidLine::lagrange_target(
    NodeType const     & NODE__,
    U_const_pointer_type U__,
    P_const_pointer_type P__
  ) const {
    integer     i_segment = NODE__.i_segment;
    real_type const * Q__ = NODE__.q;
    real_type const * X__ = NODE__.x;
    Road2D::SegmentClass const & segment = pRoad->getSegmentByIndex(i_segment);
    real_type t3   = ModelPars[177] * ModelPars[177];
    real_type t8   = pow(X__[21] - ModelPars[178], 2);
    real_type t11  = ModelPars[166] * ModelPars[166];
    real_type t16  = pow(X__[22] - ModelPars[167], 2);
    real_type t19  = ModelPars[242] * ModelPars[242];
    real_type t24  = pow(X__[31] - ModelPars[243], 2);
    real_type t27  = ModelPars[174] * ModelPars[174];
    real_type t32  = pow(X__[23] - ModelPars[175], 2);
    real_type t35  = ModelPars[191] * ModelPars[191];
    real_type t40  = pow(X__[19] - ModelPars[192], 2);
    real_type t43  = ModelPars[246] * ModelPars[246];
    real_type t48  = pow(X__[32] - ModelPars[194], 2);
    real_type t51  = ModelPars[198] * ModelPars[198];
    real_type t56  = pow(X__[24] - ModelPars[199], 2);
    real_type t59  = ModelPars[210] * ModelPars[210];
    real_type t64  = pow(X__[20] - ModelPars[211], 2);
    real_type t67  = ModelPars[216] * ModelPars[216];
    real_type t72  = pow(X__[25] - ModelPars[217], 2);
    real_type t75  = ModelPars[220] * ModelPars[220];
    real_type t80  = pow(X__[28] - ModelPars[221], 2);
    real_type t83  = ModelPars[224] * ModelPars[224];
    real_type t88  = pow(X__[26] - ModelPars[225], 2);
    real_type t91  = ModelPars[228] * ModelPars[228];
    real_type t96  = pow(X__[29] - ModelPars[229], 2);
    real_type t99  = ModelPars[234] * ModelPars[234];
    real_type t104 = pow(X__[27] - ModelPars[235], 2);
    real_type t107 = ModelPars[238] * ModelPars[238];
    real_type t112 = pow(X__[30] - ModelPars[239], 2);
    real_type t114 = t8 / t3 + t16 / t11 + t24 / t19 + t32 / t27 + t40 / t35 + t48 / t43 + t56 / t51 + t64 / t59 + t72 / t67 + t80 / t75 + t88 / t83 + t96 / t91 + t104 / t99 + t112 / t107;
    real_type t118 = ModelPars[77] * ModelPars[77];
    real_type t120 = X__[37];
    real_type t121 = t120 * t120;
    real_type t128 = X__[38];
    real_type t129 = cos(t128);
    real_type t132 = sin(t128);
    return -1.0 / (X__[0] * t129 - X__[1] * t132) * (Q__[1] * t120 - 1) * (ModelPars[140] + ModelPars[137] * t114 + t121 / t118);
  }

  /*\
   |   __  __
   |  |  \/  |__ _ _  _ ___ _ _
   |  | |\/| / _` | || / -_) '_|
   |  |_|  |_\__,_|\_, \___|_|
   |               |__/
  \*/

  real_type
  MidLine::mayer_target(
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
    real_type t2   = ModelPars[174] * ModelPars[174];
    real_type t7   = pow(XL__[23] - XR__[23], 2);
    real_type t10  = ModelPars[198] * ModelPars[198];
    real_type t15  = pow(XL__[24] - XR__[24], 2);
    real_type t18  = ModelPars[216] * ModelPars[216];
    real_type t23  = pow(XL__[25] - XR__[25], 2);
    real_type t26  = ModelPars[224] * ModelPars[224];
    real_type t31  = pow(XL__[26] - XR__[26], 2);
    real_type t34  = ModelPars[128] * ModelPars[128];
    real_type t39  = pow(XL__[0] - XR__[0], 2);
    real_type t42  = ModelPars[130] * ModelPars[130];
    real_type t47  = pow(XL__[1] - XR__[1], 2);
    real_type t50  = ModelPars[163] * ModelPars[163];
    real_type t55  = pow(XL__[2] - XR__[2], 2);
    real_type t58  = ModelPars[193] * ModelPars[193];
    real_type t63  = pow(XL__[3] - XR__[3], 2);
    real_type t66  = ModelPars[212] * ModelPars[212];
    real_type t71  = pow(XL__[4] - XR__[4], 2);
    real_type t74  = ModelPars[68] * ModelPars[68];
    real_type t79  = pow(XL__[5] - XR__[5], 2);
    real_type t82  = ModelPars[168] * ModelPars[168];
    real_type t87  = pow(XL__[6] - XR__[6], 2);
    real_type t90  = ModelPars[176] * ModelPars[176];
    real_type t95  = pow(XL__[7] - XR__[7], 2);
    real_type t98  = ModelPars[200] * ModelPars[200];
    real_type t103 = pow(XL__[8] - XR__[8], 2);
    real_type t106 = ModelPars[195] * ModelPars[195];
    real_type t111 = pow(XL__[16] - XR__[16], 2);
    real_type t114 = ModelPars[188] * ModelPars[188];
    real_type t119 = pow(XL__[17] - XR__[17], 2);
    real_type t122 = ModelPars[185] * ModelPars[185];
    real_type t127 = pow(XL__[18] - XR__[18], 2);
    real_type t130 = ModelPars[191] * ModelPars[191];
    real_type t135 = pow(XL__[19] - XR__[19], 2);
    real_type t138 = ModelPars[210] * ModelPars[210];
    real_type t143 = pow(XL__[20] - XR__[20], 2);
    real_type t146 = ModelPars[177] * ModelPars[177];
    real_type t151 = pow(XL__[21] - XR__[21], 2);
    real_type t153 = t7 / t2 + t15 / t10 + t23 / t18 + t31 / t26 + t39 / t34 + t47 / t42 + t55 / t50 + t63 / t58 + t71 / t66 + t79 / t74 + t87 / t82 + t95 / t90 + t103 / t98 + t111 / t106 + t119 / t114 + t127 / t122 + t135 / t130 + t143 / t138 + t151 / t146;
    real_type t155 = ModelPars[166] * ModelPars[166];
    real_type t160 = pow(XL__[22] - XR__[22], 2);
    real_type t163 = ModelPars[234] * ModelPars[234];
    real_type t168 = pow(XL__[27] - XR__[27], 2);
    real_type t171 = ModelPars[220] * ModelPars[220];
    real_type t176 = pow(XL__[28] - XR__[28], 2);
    real_type t179 = ModelPars[228] * ModelPars[228];
    real_type t184 = pow(XL__[29] - XR__[29], 2);
    real_type t187 = ModelPars[238] * ModelPars[238];
    real_type t192 = pow(XL__[30] - XR__[30], 2);
    real_type t195 = ModelPars[242] * ModelPars[242];
    real_type t200 = pow(XL__[31] - XR__[31], 2);
    real_type t203 = ModelPars[246] * ModelPars[246];
    real_type t208 = pow(XL__[32] - XR__[32], 2);
    real_type t211 = ModelPars[154] * ModelPars[154];
    real_type t216 = pow(XL__[33] - XR__[33], 2);
    real_type t219 = ModelPars[159] * ModelPars[159];
    real_type t224 = pow(XL__[34] - XR__[34], 2);
    real_type t227 = ModelPars[218] * ModelPars[218];
    real_type t232 = pow(XL__[9] - XR__[9], 2);
    real_type t235 = ModelPars[226] * ModelPars[226];
    real_type t240 = pow(XL__[10] - XR__[10], 2);
    real_type t243 = ModelPars[236] * ModelPars[236];
    real_type t248 = pow(XL__[11] - XR__[11], 2);
    real_type t251 = ModelPars[222] * ModelPars[222];
    real_type t256 = pow(XL__[12] - XR__[12], 2);
    real_type t259 = ModelPars[230] * ModelPars[230];
    real_type t264 = pow(XL__[13] - XR__[13], 2);
    real_type t267 = ModelPars[240] * ModelPars[240];
    real_type t272 = pow(XL__[14] - XR__[14], 2);
    real_type t275 = ModelPars[169] * ModelPars[169];
    real_type t280 = pow(XL__[15] - XR__[15], 2);
    real_type t283 = ModelPars[203] * ModelPars[203];
    real_type t288 = pow(XL__[36] - XR__[36], 2);
    real_type t291 = ModelPars[77] * ModelPars[77];
    real_type t296 = pow(XL__[37] - XR__[37], 2);
    real_type t299 = ModelPars[146] * ModelPars[146];
    real_type t304 = pow(XL__[38] - XR__[38], 2);
    real_type t307 = ModelPars[161] * ModelPars[161];
    real_type t312 = pow(XL__[35] - XR__[35], 2);
    real_type t314 = t160 / t155 + t168 / t163 + t176 / t171 + t184 / t179 + t192 / t187 + t200 / t195 + t208 / t203 + t216 / t211 + t224 / t219 + t232 / t227 + t240 / t235 + t248 / t243 + t256 / t251 + t264 / t259 + t272 / t267 + t280 / t275 + t288 / t283 + t296 / t291 + t304 / t299 + t312 / t307;
    return ModelPars[132] * (t153 + t314);
  }

  /*\
   |    ___
   |   / _ \
   |  | (_) |
   |   \__\_\
  \*/

  integer
  MidLine::q_numEqns() const
  { return 13; }

  void
  MidLine::q_eval(
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
  MidLine::u_guess_numEqns() const
  { return 4; }

  void
  MidLine::u_guess_eval(
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
  MidLine::u_guess_eval(
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
  MidLine::u_check_if_admissible(
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
    ok = ok && t__oControl.check_range(U__[2], ModelPars[165], ModelPars[190]);
    real_type t5   = ModelPars[184];
    real_type t6   = ModelPars[30];
    ok = ok && b__f__oControl.check_range(U__[0], t5, t6);
    ok = ok && b__r__oControl.check_range(U__[1], t5, t6);
    real_type t9   = ModelPars[202];
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
  MidLine::post_numEqns() const
  { return 54; }

  void
  MidLine::post_eval(
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
    result__[ 0   ] = t__oControl(U__[2], ModelPars[165], ModelPars[190]);
    real_type t5   = ModelPars[184];
    real_type t6   = ModelPars[30];
    result__[ 1   ] = b__f__oControl(U__[0], t5, t6);
    result__[ 2   ] = b__r__oControl(U__[1], t5, t6);
    real_type t9   = ModelPars[202];
    result__[ 3   ] = tau__oControl(U__[3], -t9, t9);
    real_type t11  = 1.0 / ModelPars[158];
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
    real_type t83  = ModelPars[144];
    real_type t84  = t82 + t83;
    real_type t86  = ModelPars[122];
    real_type t87  = X__[8];
    real_type t88  = t86 - t87;
    real_type t90  = ModelPars[118];
    real_type t91  = ModelPars[120];
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
    real_type t118 = ModelPars[121];
    real_type t119 = -t118 - t25;
    real_type t121 = t69 * t42;
    result__[ 44  ] = t119 * t73 - t121;
    real_type t127 = t73 * t42;
    result__[ 45  ] = t113 * t111 * t67 - t113 * t115 * t80 + t119 * t69 + t109 + t118 + t127 - ModelPars[119];
    real_type t129 = t80 * t50;
    real_type t130 = X__[20];
    real_type t132 = X__[22];
    real_type t134 = X__[31];
    real_type t136 = X__[19];
    real_type t137 = t136 * t73;
    result__[ 46  ] = t130 * t129 - t67 * t132 + t134 * t69 - t36 * t137;
    real_type t139 = X__[32];
    real_type t140 = t139 * t74;
    real_type t143 = t139 * t71;
    real_type t154 = t50 * t71;
    real_type t155 = t130 * t67;
    real_type t163 = t50 * t74;
    result__[ 47  ] = -t130 * t69 * t67 * t163 - t73 * t132 * t80 * t71 - t136 * t80 * t69 * t154 + t136 * t73 * t80 * t163 + t80 * t69 * t50 * t143 - t73 * t129 * t140 - t132 * t80 * t76 - t73 * t155 * t154 + t136 * t76 + t136 * t77 - t69 * t140 - t73 * t143;
    real_type t170 = X__[24];
    real_type t172 = t134 * t71;
    real_type t176 = t130 * t80;
    result__[ 48  ] = -t130 * t67 * t87 - t130 * t80 * t82 - t90 * t36 * t140 + t91 * t36 * t140 + t86 * t155 - t80 * t170 - t90 * t172 + t91 * t172 - t83 * t176 - t38;
    real_type t193 = t136 * t69;
    real_type t195 = t90 * t73;
    real_type t197 = t91 * t73;
    real_type t199 = t90 * t136;
    real_type t201 = t91 * t136;
    real_type t203 = t90 * t69;
    real_type t205 = t91 * t69;
    result__[ 49  ] = -t136 * t69 * t20 + t136 * t73 * t35 + t132 * t83 + t203 * t140 - t205 * t140 + t195 * t143 - t197 * t143 - t91 * t193 - t199 * t76 - t199 * t77 + t201 * t76 + t201 * t77 - t73 * t21 - t69 * t39;
    real_type t212 = X__[21];
    result__[ 50  ] = t130 * t67 * t82 - t130 * t80 * t87 + t136 * t73 * t20 + t136 * t69 * t35 + t91 * t137 - t195 * t140 + t197 * t140 + t203 * t143 - t205 * t143 + t83 * t155 + t67 * t170 + t86 * t176 - t199 * t72 + t199 * t75 + t201 * t72 - t201 * t75 - t69 * t21 + t73 * t39 + t212;
    real_type t232 = t111 * t113;
    real_type t233 = X__[23];
    real_type t234 = t67 * t233;
    real_type t237 = t115 * t113;
    real_type t238 = t80 * t233;
    result__[ 51  ] = -t155 * t232 + t176 * t237 + t234 * t232 - t238 * t237 + t44;
    result__[ 52  ] = -t136 * t69 * t25 - t118 * t193 + t136 * t127 - t26 * t73 - t69 * t48;
    result__[ 53  ] = t136 * t73 * t25 + t118 * t137 + t136 * t121 - t155 * t237 - t176 * t232 + t238 * t232 + t234 * t237 - t26 * t69 + t73 * t48 + t212;
    #ifdef MECHATRONIX_DEBUG
    CHECK_NAN(result__,"post_eval",54);
    #endif
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  integer
  MidLine::integrated_post_numEqns() const
  { return 1; }

  void
  MidLine::integrated_post_eval(
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

// EOF: MidLine_Methods1.cc

/*-----------------------------------------------------------------------*\
 |  file: Straight_Methods1.cc                                           |
 |                                                                       |
 |  version: 1.0   date 23/6/2020                                        |
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


#include "Straight.hh"
#include "Straight_Pars.hh"

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


namespace StraightDefine {
  /*\
   |   ___         _   _               _   _
   |  / __|___ _ _| |_(_)_ _ _  _ __ _| |_(_)___ _ _
   | | (__/ _ \ ' \  _| | ' \ || / _` |  _| / _ \ ' \
   |  \___\___/_||_\__|_|_||_\_,_\__,_|\__|_\___/_||_|
  \*/

  void
  Straight::continuationStep0( real_type s ) {
    int msg_level = 3;
    pConsole->message(
      fmt::format( "\nContinuation step N.0 s = {}\n", s ),
      msg_level
    );
    real_type t1   = ModelPars[145];
    ModelPars[144] = t1 + (ModelPars[146] - t1) * s;
    real_type t5   = ModelPars[142];
    ModelPars[141] = t5 + (ModelPars[143] - t5) * s;
  }
  /*\
   |   ___         _   _               _   _
   |  / __|___ _ _| |_(_)_ _ _  _ __ _| |_(_)___ _ _
   | | (__/ _ \ ' \  _| | ' \ || / _` |  _| / _ \ ' \
   |  \___\___/_||_\__|_|_||_\_,_\__,_|\__|_\___/_||_|
  \*/

  void
  Straight::continuationStep1( real_type s ) {
    int msg_level = 3;
    pConsole->message(
      fmt::format( "\nContinuation step N.1 s = {}\n", s ),
      msg_level
    );
    real_type t1   = ModelPars[61];
    real_type t5   = t1 + (ModelPars[62] - t1) * s;
    b__f__oControl.update_epsilon(t5);
    real_type t6   = ModelPars[130];
    real_type t10  = t6 + (ModelPars[131] - t6) * s;
    b__f__oControl.update_tolerance(t10);
    t__oControl.update_epsilon(t5);
    t__oControl.update_tolerance(t10);
    b__r__oControl.update_epsilon(t5);
    b__r__oControl.update_tolerance(t10);
    tau__oControl.update_epsilon(t5);
    tau__oControl.update_tolerance(t10);
    real_type t11  = ModelPars[63];
    real_type t15  = t11 + (ModelPars[64] - t11) * s;
    FrontWheelContact.update_epsilon(t15);
    real_type t16  = ModelPars[132];
    real_type t20  = t16 + (ModelPars[133] - t16) * s;
    FrontWheelContact.update_tolerance(t20);
    RearWheelContact.update_epsilon(t15);
    RearWheelContact.update_tolerance(t20);
    LongSlipFront.update_epsilon(t15);
    LongSlipFront.update_tolerance(t20);
    LatSlipFront.update_epsilon(t15);
    LatSlipFront.update_tolerance(t20);
    MaxSteerAngle.update_epsilon(t15);
    MaxSteerAngle.update_tolerance(t20);
    MaxRollAngle.update_epsilon(t15);
    MaxRollAngle.update_tolerance(t20);
    roadRightLateralBorder.update_epsilon(t15);
    roadRightLateralBorder.update_tolerance(t20);
    roadLeftLateralBorder.update_epsilon(t15);
    roadLeftLateralBorder.update_tolerance(t20);
  }

  /*\
   |  _   _               ___             _   _
   | | | | |___ ___ _ _  | __|  _ _ _  __| |_(_)___ _ _  ___
   | | |_| (_-</ -_) '_| | _| || | ' \/ _|  _| / _ \ ' \(_-<
   |  \___//__/\___|_|   |_| \_,_|_||_\__|\__|_\___/_||_/__/
  \*/
  // user defined functions which has a body defined in MAPLE
  real_type
  Straight::alpha__crw( real_type t__XO ) const {
    return asin(1.0 / ModelPars[24] * (ModelPars[120] - ModelPars[121]));
  }

  real_type
  Straight::alpha__crw_D( real_type t__XO ) const {
    return 0;
  }

  real_type
  Straight::alpha__crw_DD( real_type t__XO ) const {
    return 0;
  }

  real_type
  Straight::alpha__pin( real_type eta__XO, real_type alpha__crw__XO ) const {
    return alpha__crw__XO + eta__XO;
  }

  real_type
  Straight::alpha__pin_D_1( real_type eta__XO, real_type alpha__crw__XO ) const {
    return 1;
  }

  real_type
  Straight::alpha__pin_D_1_1( real_type eta__XO, real_type alpha__crw__XO ) const {
    return 0;
  }

  real_type
  Straight::alpha__pin_D_1_2( real_type eta__XO, real_type alpha__crw__XO ) const {
    return 0;
  }

  real_type
  Straight::alpha__pin_D_2( real_type eta__XO, real_type alpha__crw__XO ) const {
    return 1;
  }

  real_type
  Straight::alpha__pin_D_2_2( real_type eta__XO, real_type alpha__crw__XO ) const {
    return 0;
  }

  real_type
  Straight::Fzf( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return -z__f__XO * ModelPars[21] - z__f__dot__XO * ModelPars[2];
  }

  real_type
  Straight::Fzf_D_1( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return -ModelPars[21];
  }

  real_type
  Straight::Fzf_D_1_1( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return 0;
  }

  real_type
  Straight::Fzf_D_1_2( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return 0;
  }

  real_type
  Straight::Fzf_D_2( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return -ModelPars[2];
  }

  real_type
  Straight::Fzf_D_2_2( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return 0;
  }

  real_type
  Straight::Fzr( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return -z__r__XO * ModelPars[22] - z__r__dot__XO * ModelPars[3];
  }

  real_type
  Straight::Fzr_D_1( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return -ModelPars[22];
  }

  real_type
  Straight::Fzr_D_1_1( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return 0;
  }

  real_type
  Straight::Fzr_D_1_2( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return 0;
  }

  real_type
  Straight::Fzr_D_2( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return -ModelPars[3];
  }

  real_type
  Straight::Fzr_D_2_2( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return 0;
  }

  real_type
  Straight::alpha__r( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t7   = atan((-Omega__XO * x__r__XO + v__XO + y__r__dot__XO) / (-y__r__XO * Omega__XO + u__XO - x__r__dot__XO));
    return -t7;
  }

  real_type
  Straight::alpha__r_D_1( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t6   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t7   = t2 * t2;
    real_type t8   = 1.0 / t7;
    real_type t12  = t6 * t6;
    return -1.0 / (t8 * t12 + 1) * (-1.0 / t2 * x__r__XO + y__r__XO * t8 * t6);
  }

  real_type
  Straight::alpha__r_D_1_1( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Straight::alpha__r_D_1_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Straight::alpha__r_D_1_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Straight::alpha__r_D_1_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Straight::alpha__r_D_1_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Straight::alpha__r_D_1_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Straight::alpha__r_D_1_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Straight::alpha__r_D_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t5   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t6   = 1.0 / t5;
    real_type t8   = t2 * t2;
    return 1.0 / (t6 * t8 + 1) * t6 * t2;
  }

  real_type
  Straight::alpha__r_D_2_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Straight::alpha__r_D_2_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t7   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t9   = t4 * t7 + 1;
    real_type t12  = t3 * t3;
    real_type t15  = t9 * t9;
    return 1.0 / t9 * t4 - 2 / t15 / t12 * t7;
  }

  real_type
  Straight::alpha__r_D_2_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t8   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t10  = t4 * t8 + 1;
    real_type t13  = t3 * t3;
    real_type t16  = t10 * t10;
    return -1.0 / t10 * t4 * Omega__XO + 2 * Omega__XO / t16 / t13 * t8;
  }

  real_type
  Straight::alpha__r_D_2_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Straight::alpha__r_D_2_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Straight::alpha__r_D_2_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t7   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t9   = t4 * t7 + 1;
    real_type t12  = t3 * t3;
    real_type t15  = t9 * t9;
    return 1.0 / t9 * t4 - 2 / t15 / t12 * t7;
  }

  real_type
  Straight::alpha__r_D_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t6   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t7   = t2 * t2;
    return -1.0 / (1.0 / t7 * t6 + 1) / t2;
  }

  real_type
  Straight::alpha__r_D_3_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t12  = pow(1.0 / t3 * t8 + 1, 2);
    return 2 * t7 / t12 / t3 / t2;
  }

  real_type
  Straight::alpha__r_D_3_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t12  = pow(1.0 / t3 * t8 + 1, 2);
    return -2 * Omega__XO * t7 / t12 / t3 / t2;
  }

  real_type
  Straight::alpha__r_D_3_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t8   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t10  = t4 * t8 + 1;
    real_type t13  = t3 * t3;
    real_type t16  = t10 * t10;
    return -1.0 / t10 * t4 * Omega__XO + 2 * Omega__XO / t16 / t13 * t8;
  }

  real_type
  Straight::alpha__r_D_3_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t7   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t9   = t4 * t7 + 1;
    real_type t12  = t3 * t3;
    real_type t15  = t9 * t9;
    return -1.0 / t9 * t4 + 2 / t15 / t12 * t7;
  }

  real_type
  Straight::alpha__r_D_3_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t12  = pow(1.0 / t3 * t8 + 1, 2);
    return 2 * t7 / t12 / t3 / t2;
  }

  real_type
  Straight::alpha__r_D_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t7   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t8   = t2 * t2;
    return 1.0 / (1.0 / t8 * t7 + 1) / t2 * Omega__XO;
  }

  real_type
  Straight::alpha__r_D_4_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t1   = Omega__XO * Omega__XO;
    real_type t3   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t4   = t3 * t3;
    real_type t9   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t10  = t9 * t9;
    real_type t14  = pow(1.0 / t4 * t10 + 1, 2);
    return 2 * t9 / t14 / t4 / t3 * t1;
  }

  real_type
  Straight::alpha__r_D_4_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Straight::alpha__r_D_4_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t8   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t10  = t4 * t8 + 1;
    real_type t13  = t3 * t3;
    real_type t16  = t10 * t10;
    return 1.0 / t10 * t4 * Omega__XO - 2 * Omega__XO / t16 / t13 * t8;
  }

  real_type
  Straight::alpha__r_D_4_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t12  = pow(1.0 / t3 * t8 + 1, 2);
    return -2 * Omega__XO * t7 / t12 / t3 / t2;
  }

  real_type
  Straight::alpha__r_D_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t5   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t6   = 1.0 / t5;
    real_type t8   = t2 * t2;
    return -1.0 / (t6 * t8 + 1) * Omega__XO * t6 * t2;
  }

  real_type
  Straight::alpha__r_D_5_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Straight::alpha__r_D_5_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Straight::alpha__r_D_5_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t8   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t10  = t4 * t8 + 1;
    real_type t13  = t3 * t3;
    real_type t16  = t10 * t10;
    return -1.0 / t10 * t4 * Omega__XO + 2 * Omega__XO / t16 / t13 * t8;
  }

  real_type
  Straight::alpha__r_D_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t5   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t6   = 1.0 / t5;
    real_type t8   = t2 * t2;
    return -1.0 / (t6 * t8 + 1) * t6 * t2;
  }

  real_type
  Straight::alpha__r_D_6_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Straight::alpha__r_D_6_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t7   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t9   = t4 * t7 + 1;
    real_type t12  = t3 * t3;
    real_type t15  = t9 * t9;
    return -1.0 / t9 * t4 + 2 / t15 / t12 * t7;
  }

  real_type
  Straight::alpha__r_D_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t6   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t7   = t2 * t2;
    return -1.0 / (1.0 / t7 * t6 + 1) / t2;
  }

  real_type
  Straight::alpha__r_D_7_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t12  = pow(1.0 / t3 * t8 + 1, 2);
    return 2 * t7 / t12 / t3 / t2;
  }

  real_type
  Straight::alpha__f( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t2   = y__f__XO * Omega__XO;
    real_type t5   = x__f__XO * Omega__XO;
    real_type t12  = atan(1.0 / (-t2 + u__XO + x__f__dot__XO + delta__f__XO * (t5 + v__XO + y__f__dot__XO)) * (-x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t2 - u__XO) * delta__f__XO + t5 + v__XO));
    return -t12;
  }

  real_type
  Straight::alpha__f_D_1( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_1_1( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_1_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_1_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_1_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_1_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_1_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_1_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_1_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_2_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_2_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_2_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_2_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_2_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_2_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_2_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_3_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_3_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_3_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_3_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_3_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_3_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_4_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_4_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_4_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_4_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_4_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_5_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_5_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_5_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_5_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_6_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_6_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_6_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_7_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_7_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::alpha__f_D_8_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Straight::lambda__r( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t4   = ModelPars[125];
    real_type t6   = cos(phi__XO);
    return (-x__r__dot__XO - t6 * omega__r__XO * t4 + (-ModelPars[123] + t4) * omega__r__XO - t1 + u__XO) / (t1 - u__XO + x__r__dot__XO);
  }

  real_type
  Straight::lambda__r_D_1( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t5   = ModelPars[125];
    real_type t7   = cos(phi__XO);
    return -y__r__XO * (-x__r__dot__XO - t7 * omega__r__XO * t5 + (-ModelPars[123] + t5) * omega__r__XO - t1 + u__XO) / t3 - y__r__XO / t2;
  }

  real_type
  Straight::lambda__r_D_1_1( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[125];
    real_type t8   = cos(phi__XO);
    real_type t15  = y__r__XO * y__r__XO;
    return 2 * t15 * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[123] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 * t15 / t3;
  }

  real_type
  Straight::lambda__r_D_1_2( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t7   = sin(phi__XO);
    return -y__r__XO * t7 * omega__r__XO * ModelPars[125] / t3;
  }

  real_type
  Straight::lambda__r_D_1_3( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[125];
    real_type t8   = cos(phi__XO);
    return -2 * y__r__XO * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[123] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 - 2 * y__r__XO / t3;
  }

  real_type
  Straight::lambda__r_D_1_4( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[125];
    real_type t8   = cos(phi__XO);
    real_type t13  = -x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[123] + t6) * omega__r__XO - t1 + u__XO;
    real_type t17  = 1.0 / t3;
    return 2 * t1 * t13 / t3 / t2 + 2 * y__r__XO * Omega__XO * t17 - t13 * t17 - 1.0 / t2;
  }

  real_type
  Straight::lambda__r_D_1_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t5   = ModelPars[125];
    real_type t6   = cos(phi__XO);
    return -y__r__XO * (-t6 * t5 + t5 - ModelPars[123]) / t3;
  }

  real_type
  Straight::lambda__r_D_1_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[125];
    real_type t8   = cos(phi__XO);
    return 2 * y__r__XO * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[123] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 * y__r__XO / t3;
  }

  real_type
  Straight::lambda__r_D_2( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t6   = sin(phi__XO);
    return t6 * omega__r__XO / (y__r__XO * Omega__XO - u__XO + x__r__dot__XO) * ModelPars[125];
  }

  real_type
  Straight::lambda__r_D_2_2( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t6   = cos(phi__XO);
    return t6 * omega__r__XO / (y__r__XO * Omega__XO - u__XO + x__r__dot__XO) * ModelPars[125];
  }

  real_type
  Straight::lambda__r_D_2_3( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t7   = sin(phi__XO);
    return t7 * omega__r__XO * ModelPars[125] / t3;
  }

  real_type
  Straight::lambda__r_D_2_4( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t7   = sin(phi__XO);
    return -Omega__XO * t7 * omega__r__XO * ModelPars[125] / t3;
  }

  real_type
  Straight::lambda__r_D_2_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t6   = sin(phi__XO);
    return t6 * ModelPars[125] / (y__r__XO * Omega__XO - u__XO + x__r__dot__XO);
  }

  real_type
  Straight::lambda__r_D_2_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t7   = sin(phi__XO);
    return -t7 * omega__r__XO * ModelPars[125] / t3;
  }

  real_type
  Straight::lambda__r_D_3( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t5   = ModelPars[125];
    real_type t7   = cos(phi__XO);
    return (-x__r__dot__XO - t7 * omega__r__XO * t5 + (-ModelPars[123] + t5) * omega__r__XO - t1 + u__XO) / t3 + 1.0 / t2;
  }

  real_type
  Straight::lambda__r_D_3_3( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[125];
    real_type t8   = cos(phi__XO);
    return 2 * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[123] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 / t3;
  }

  real_type
  Straight::lambda__r_D_3_4( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[125];
    real_type t8   = cos(phi__XO);
    return -2 * Omega__XO * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[123] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 - 2 * Omega__XO / t3;
  }

  real_type
  Straight::lambda__r_D_3_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t5   = ModelPars[125];
    real_type t6   = cos(phi__XO);
    return (-t6 * t5 + t5 - ModelPars[123]) / t3;
  }

  real_type
  Straight::lambda__r_D_3_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[125];
    real_type t8   = cos(phi__XO);
    return -2 * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[123] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 - 2 / t3;
  }

  real_type
  Straight::lambda__r_D_4( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t5   = ModelPars[125];
    real_type t7   = cos(phi__XO);
    return -Omega__XO * (-x__r__dot__XO - t7 * omega__r__XO * t5 + (-ModelPars[123] + t5) * omega__r__XO - t1 + u__XO) / t3 - Omega__XO / t2;
  }

  real_type
  Straight::lambda__r_D_4_4( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[125];
    real_type t8   = cos(phi__XO);
    real_type t15  = Omega__XO * Omega__XO;
    return 2 * t15 * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[123] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 * t15 / t3;
  }

  real_type
  Straight::lambda__r_D_4_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t5   = ModelPars[125];
    real_type t6   = cos(phi__XO);
    return -Omega__XO * (-t6 * t5 + t5 - ModelPars[123]) / t3;
  }

  real_type
  Straight::lambda__r_D_4_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[125];
    real_type t8   = cos(phi__XO);
    return 2 * Omega__XO * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[123] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 * Omega__XO / t3;
  }

  real_type
  Straight::lambda__r_D_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t4   = ModelPars[125];
    real_type t5   = cos(phi__XO);
    return (-t5 * t4 + t4 - ModelPars[123]) / (y__r__XO * Omega__XO - u__XO + x__r__dot__XO);
  }

  real_type
  Straight::lambda__r_D_5_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    return 0;
  }

  real_type
  Straight::lambda__r_D_5_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t5   = ModelPars[125];
    real_type t6   = cos(phi__XO);
    return -(-t6 * t5 + t5 - ModelPars[123]) / t3;
  }

  real_type
  Straight::lambda__r_D_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t5   = ModelPars[125];
    real_type t7   = cos(phi__XO);
    return -(-x__r__dot__XO - t7 * omega__r__XO * t5 + (-ModelPars[123] + t5) * omega__r__XO - t1 + u__XO) / t3 - 1.0 / t2;
  }

  real_type
  Straight::lambda__r_D_6_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[125];
    real_type t8   = cos(phi__XO);
    return 2 * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[123] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 / t3;
  }

  real_type
  Straight::lambda__f( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t8   = ModelPars[124];
    real_type t10  = cos(phi__f__XO);
    return (-x__f__dot__XO - t1 + t10 * omega__f__XO * t8 - t3 * delta__f__XO + t5 + (ModelPars[122] - t8) * omega__f__XO - u__XO) / (t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO);
  }

  real_type
  Straight::lambda__f_D_1( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[124];
    real_type t11  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * x__f__XO - y__f__XO;
    return -t21 * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[122] - t9) * omega__f__XO - u__XO) / t7 - t21 / t6;
  }

  real_type
  Straight::lambda__f_D_1_1( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t23  = t22 * t22;
    return 2 * t23 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * t22 / t7;
  }

  real_type
  Straight::lambda__f_D_1_2( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return (delta__f__XO * x__f__XO - y__f__XO) * t11 * omega__f__XO * ModelPars[124] / t7;
  }

  real_type
  Straight::lambda__f_D_1_3( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t25  = 1.0 / t7;
    return 2 * t22 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * t25;
  }

  real_type
  Straight::lambda__f_D_1_4( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t26  = 1.0 / t7;
    return 2 * delta__f__XO * t22 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * delta__f__XO * t26;
  }

  real_type
  Straight::lambda__f_D_1_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO;
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t27  = 1.0 / t7;
    return 2 * delta__f__XO * Omega__XO * t22 * t19 / t7 / t6 + 2 * t22 * delta__f__XO * Omega__XO * t27 - delta__f__XO * t19 * t27 - delta__f__XO / t6;
  }

  real_type
  Straight::lambda__f_D_1_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO;
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t26  = 1.0 / t7;
    return -2 * Omega__XO * t22 * t19 / t7 / t6 - 2 * t22 * Omega__XO * t26 + t19 * t26 + 1.0 / t6;
  }

  real_type
  Straight::lambda__f_D_1_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO;
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t23  = t2 + v__XO + y__f__dot__XO;
    real_type t27  = 1.0 / t7;
    return 2 * t23 * t22 * t19 / t7 / t6 + 2 * t22 * t23 * t27 - x__f__XO * t19 * t27 - x__f__XO / t6;
  }

  real_type
  Straight::lambda__f_D_1_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[124];
    real_type t10  = cos(phi__f__XO);
    return -(delta__f__XO * x__f__XO - y__f__XO) * (t10 * t9 - t9 + ModelPars[122]) / t7;
  }

  real_type
  Straight::lambda__f_D_1_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t25  = 1.0 / t7;
    return 2 * t22 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * t25;
  }

  real_type
  Straight::lambda__f_D_1_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t26  = 1.0 / t7;
    return 2 * delta__f__XO * t22 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * delta__f__XO * t26;
  }

  real_type
  Straight::lambda__f_D_2( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t10  = sin(phi__f__XO);
    return -t10 * omega__f__XO / (x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO) * ModelPars[124];
  }

  real_type
  Straight::lambda__f_D_2_2( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t10  = cos(phi__f__XO);
    return -t10 * omega__f__XO / (x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO) * ModelPars[124];
  }

  real_type
  Straight::lambda__f_D_2_3( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return t11 * omega__f__XO * ModelPars[124] / t7;
  }

  real_type
  Straight::lambda__f_D_2_4( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return delta__f__XO * t11 * omega__f__XO * ModelPars[124] / t7;
  }

  real_type
  Straight::lambda__f_D_2_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t12  = sin(phi__f__XO);
    return delta__f__XO * Omega__XO * t12 * omega__f__XO * ModelPars[124] / t7;
  }

  real_type
  Straight::lambda__f_D_2_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return -Omega__XO * t11 * omega__f__XO * ModelPars[124] / t7;
  }

  real_type
  Straight::lambda__f_D_2_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t2   = x__f__XO * Omega__XO;
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + (t2 + v__XO) * delta__f__XO - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return (t2 + v__XO + y__f__dot__XO) * t11 * omega__f__XO * ModelPars[124] / t7;
  }

  real_type
  Straight::lambda__f_D_2_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t10  = sin(phi__f__XO);
    return -t10 * ModelPars[124] / (x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO);
  }

  real_type
  Straight::lambda__f_D_2_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return t11 * omega__f__XO * ModelPars[124] / t7;
  }

  real_type
  Straight::lambda__f_D_2_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return delta__f__XO * t11 * omega__f__XO * ModelPars[124] / t7;
  }

  real_type
  Straight::lambda__f_D_3( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[124];
    real_type t11  = cos(phi__f__XO);
    return -(-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[122] - t9) * omega__f__XO - u__XO) / t7 - 1.0 / t6;
  }

  real_type
  Straight::lambda__f_D_3_3( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    return 2 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 / t7;
  }

  real_type
  Straight::lambda__f_D_3_4( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    return 2 * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO / t7;
  }

  real_type
  Straight::lambda__f_D_3_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    return 2 * Omega__XO * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO * Omega__XO / t7;
  }

  real_type
  Straight::lambda__f_D_3_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    return -2 * Omega__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * Omega__XO / t7;
  }

  real_type
  Straight::lambda__f_D_3_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t24  = 1.0 / t7;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 * t24;
  }

  real_type
  Straight::lambda__f_D_3_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[124];
    real_type t10  = cos(phi__f__XO);
    return -(t10 * t9 - t9 + ModelPars[122]) / t7;
  }

  real_type
  Straight::lambda__f_D_3_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    return 2 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 / t7;
  }

  real_type
  Straight::lambda__f_D_3_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    return 2 * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO / t7;
  }

  real_type
  Straight::lambda__f_D_4( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[124];
    real_type t11  = cos(phi__f__XO);
    return -delta__f__XO * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[122] - t9) * omega__f__XO - u__XO) / t7 - delta__f__XO / t6;
  }

  real_type
  Straight::lambda__f_D_4_4( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * delta__f__XO;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 / t7;
  }

  real_type
  Straight::lambda__f_D_4_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * delta__f__XO;
    return 2 * Omega__XO * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 * Omega__XO / t7;
  }

  real_type
  Straight::lambda__f_D_4_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    return -2 * Omega__XO * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * delta__f__XO * Omega__XO / t7;
  }

  real_type
  Straight::lambda__f_D_4_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO;
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t25  = 1.0 / t7;
    return 2 * t21 * delta__f__XO * t19 / t7 / t6 + 2 * delta__f__XO * t21 * t25 - t19 * t25 - 1.0 / t6;
  }

  real_type
  Straight::lambda__f_D_4_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[124];
    real_type t10  = cos(phi__f__XO);
    return -delta__f__XO * (t10 * t9 - t9 + ModelPars[122]) / t7;
  }

  real_type
  Straight::lambda__f_D_4_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    return 2 * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO / t7;
  }

  real_type
  Straight::lambda__f_D_4_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * delta__f__XO;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 / t7;
  }

  real_type
  Straight::lambda__f_D_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[124];
    real_type t11  = cos(phi__f__XO);
    return -Omega__XO * delta__f__XO * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[122] - t9) * omega__f__XO - u__XO) / t7 - delta__f__XO * Omega__XO / t6;
  }

  real_type
  Straight::lambda__f_D_5_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = Omega__XO * Omega__XO;
    real_type t22  = delta__f__XO * delta__f__XO;
    return 2 * t22 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * t21 / t7;
  }

  real_type
  Straight::lambda__f_D_5_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = Omega__XO * Omega__XO;
    return -2 * delta__f__XO * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * delta__f__XO * t21 / t7;
  }

  real_type
  Straight::lambda__f_D_5_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO;
    real_type t21  = delta__f__XO * Omega__XO;
    real_type t22  = t2 + v__XO + y__f__dot__XO;
    real_type t26  = 1.0 / t7;
    return 2 * t22 * t21 * t19 / t7 / t6 + t21 * t22 * t26 - Omega__XO * t19 * t26 + t22 * delta__f__XO * Omega__XO * t26 - Omega__XO / t6;
  }

  real_type
  Straight::lambda__f_D_5_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[124];
    real_type t10  = cos(phi__f__XO);
    return -Omega__XO * delta__f__XO * (t10 * t9 - t9 + ModelPars[122]) / t7;
  }

  real_type
  Straight::lambda__f_D_5_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    return 2 * Omega__XO * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO * Omega__XO / t7;
  }

  real_type
  Straight::lambda__f_D_5_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * delta__f__XO;
    return 2 * Omega__XO * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 * Omega__XO / t7;
  }

  real_type
  Straight::lambda__f_D_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[124];
    real_type t11  = cos(phi__f__XO);
    return Omega__XO * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[122] - t9) * omega__f__XO - u__XO) / t7 + Omega__XO / t6;
  }

  real_type
  Straight::lambda__f_D_6_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = Omega__XO * Omega__XO;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 / t7;
  }

  real_type
  Straight::lambda__f_D_6_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t25  = 1.0 / t7;
    return -2 * t21 * Omega__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * Omega__XO * t21 * t25;
  }

  real_type
  Straight::lambda__f_D_6_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[124];
    real_type t10  = cos(phi__f__XO);
    return Omega__XO * (t10 * t9 - t9 + ModelPars[122]) / t7;
  }

  real_type
  Straight::lambda__f_D_6_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    return -2 * Omega__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * Omega__XO / t7;
  }

  real_type
  Straight::lambda__f_D_6_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    return -2 * Omega__XO * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * delta__f__XO * Omega__XO / t7;
  }

  real_type
  Straight::lambda__f_D_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[124];
    real_type t11  = cos(phi__f__XO);
    real_type t20  = t2 + v__XO + y__f__dot__XO;
    return -t20 * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[122] - t9) * omega__f__XO - u__XO) / t7 - t20 / t6;
  }

  real_type
  Straight::lambda__f_D_7_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t22  = t21 * t21;
    return 2 * t22 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 * t21 / t7;
  }

  real_type
  Straight::lambda__f_D_7_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t2   = x__f__XO * Omega__XO;
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + (t2 + v__XO) * delta__f__XO - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[124];
    real_type t10  = cos(phi__f__XO);
    return -(t2 + v__XO + y__f__dot__XO) * (t10 * t9 - t9 + ModelPars[122]) / t7;
  }

  real_type
  Straight::lambda__f_D_7_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t24  = 1.0 / t7;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 * t24;
  }

  real_type
  Straight::lambda__f_D_7_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO;
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t25  = 1.0 / t7;
    return 2 * t21 * delta__f__XO * t19 / t7 / t6 + 2 * delta__f__XO * t21 * t25 - t19 * t25 - 1.0 / t6;
  }

  real_type
  Straight::lambda__f_D_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t8   = ModelPars[124];
    real_type t9   = cos(phi__f__XO);
    return (t9 * t8 - t8 + ModelPars[122]) / (x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO);
  }

  real_type
  Straight::lambda__f_D_8_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    return 0;
  }

  real_type
  Straight::lambda__f_D_8_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[124];
    real_type t10  = cos(phi__f__XO);
    return -(t10 * t9 - t9 + ModelPars[122]) / t7;
  }

  real_type
  Straight::lambda__f_D_8_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[124];
    real_type t10  = cos(phi__f__XO);
    return -delta__f__XO * (t10 * t9 - t9 + ModelPars[122]) / t7;
  }

  real_type
  Straight::lambda__f_D_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[124];
    real_type t11  = cos(phi__f__XO);
    return -(-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[122] - t9) * omega__f__XO - u__XO) / t7 - 1.0 / t6;
  }

  real_type
  Straight::lambda__f_D_9_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    return 2 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 / t7;
  }

  real_type
  Straight::lambda__f_D_9_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    return 2 * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO / t7;
  }

  real_type
  Straight::lambda__f_D_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[124];
    real_type t11  = cos(phi__f__XO);
    return -delta__f__XO * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[122] - t9) * omega__f__XO - u__XO) / t7 - delta__f__XO / t6;
  }

  real_type
  Straight::lambda__f_D_10_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[124];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * delta__f__XO;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[122] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 / t7;
  }

  real_type
  Straight::Fxf( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t11  = Fzf__XO * ModelPars[243] * (t5 * t3 * ModelPars[87] + ModelPars[85]);
    real_type t14  = ModelPars[83] * ModelPars[186];
    real_type t24  = exp(t5 * t3 * ModelPars[93]);
    real_type t34  = atan(lambda__f__XO / (t11 * t14 + ModelPars[180]) * ModelPars[188] * t24 * (t5 * t3 * ModelPars[91] + ModelPars[89]) * Fzf__XO);
    real_type t36  = sin(t34 * t14);
    real_type t37  = ModelPars[110];
    real_type t42  = lambda__f__XO * lambda__f__XO;
    real_type t44  = ModelPars[100] * ModelPars[100];
    real_type t47  = sqrt(t44 * t42 + 1);
    real_type t49  = 1.0 / t47 * (phi__f__XO * ModelPars[102] + ModelPars[98]);
    real_type t50  = ModelPars[114];
    real_type t53  = atan((alpha__f__XO + t50) * t49);
    real_type t55  = cos(t53 * t37);
    real_type t58  = atan(t50 * t49);
    real_type t60  = cos(t58 * t37);
    return 1.0 / t60 * t55 * t36 * t11;
  }

  real_type
  Straight::Fxf_D_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[87];
    real_type t2   = ModelPars[6];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[243];
    real_type t9   = ModelPars[83] * ModelPars[186];
    real_type t10  = ModelPars[91];
    real_type t11  = Fzf__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[89];
    real_type t16  = t15 * Fzf__XO;
    real_type t17  = ModelPars[93];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[188];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[85]);
    real_type t29  = Fzf__XO * t27 * t9;
    real_type t31  = t29 + ModelPars[180];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__f__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t40  = ModelPars[110];
    real_type t45  = lambda__f__XO * lambda__f__XO;
    real_type t47  = ModelPars[100] * ModelPars[100];
    real_type t50  = sqrt(t47 * t45 + 1);
    real_type t52  = 1.0 / t50 * (phi__f__XO * ModelPars[102] + ModelPars[98]);
    real_type t53  = ModelPars[114];
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
  Straight::Fxf_D_1_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[87];
    real_type t2   = ModelPars[6];
    real_type t3   = 1.0 / t2;
    real_type t4   = t3 * t1;
    real_type t5   = ModelPars[243];
    real_type t6   = t5 * t4;
    real_type t7   = ModelPars[83];
    real_type t8   = ModelPars[186];
    real_type t9   = t8 * t7;
    real_type t10  = ModelPars[91];
    real_type t11  = Fzf__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[89];
    real_type t16  = t15 * Fzf__XO;
    real_type t17  = ModelPars[93];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[188];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[85]);
    real_type t29  = Fzf__XO * t27 * t9;
    real_type t31  = t29 + ModelPars[180];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__f__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t39  = ModelPars[110];
    real_type t44  = lambda__f__XO * lambda__f__XO;
    real_type t46  = ModelPars[100] * ModelPars[100];
    real_type t49  = sqrt(t46 * t44 + 1);
    real_type t51  = 1.0 / t49 * (phi__f__XO * ModelPars[102] + ModelPars[98]);
    real_type t52  = ModelPars[114];
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
  Straight::Fxf_D_1_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[87];
    real_type t2   = ModelPars[6];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[243];
    real_type t6   = t5 * t3 * t1;
    real_type t7   = ModelPars[83];
    real_type t8   = ModelPars[186];
    real_type t9   = t8 * t7;
    real_type t10  = ModelPars[91];
    real_type t11  = Fzf__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[89];
    real_type t16  = t15 * Fzf__XO;
    real_type t17  = ModelPars[93];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[188];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[85]);
    real_type t28  = Fzf__XO * t27;
    real_type t31  = t28 * t9 + ModelPars[180];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__f__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t39  = t38 * Fzf__XO;
    real_type t40  = ModelPars[110];
    real_type t43  = ModelPars[102];
    real_type t44  = lambda__f__XO * lambda__f__XO;
    real_type t46  = ModelPars[100] * ModelPars[100];
    real_type t48  = t46 * t44 + 1;
    real_type t49  = sqrt(t48);
    real_type t50  = 1.0 / t49;
    real_type t51  = t50 * t43;
    real_type t52  = ModelPars[114];
    real_type t53  = alpha__f__XO + t52;
    real_type t57  = t43 * phi__f__XO + ModelPars[98];
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
  Straight::Fxf_D_1_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[87];
    real_type t2   = ModelPars[6];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[243];
    real_type t9   = ModelPars[83] * ModelPars[186];
    real_type t10  = ModelPars[91];
    real_type t11  = Fzf__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[89];
    real_type t16  = t15 * Fzf__XO;
    real_type t17  = ModelPars[93];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[188];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[85]);
    real_type t28  = Fzf__XO * t27;
    real_type t31  = t28 * t9 + ModelPars[180];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__f__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t41  = ModelPars[110];
    real_type t45  = phi__f__XO * ModelPars[102] + ModelPars[98];
    real_type t47  = lambda__f__XO * lambda__f__XO;
    real_type t49  = ModelPars[100] * ModelPars[100];
    real_type t51  = t49 * t47 + 1;
    real_type t52  = sqrt(t51);
    real_type t53  = 1.0 / t52;
    real_type t55  = t45 * t45;
    real_type t58  = ModelPars[114];
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
  Straight::Fxf_D_1_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[87];
    real_type t2   = ModelPars[6];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[243];
    real_type t6   = t5 * t3 * t1;
    real_type t7   = Fzf__XO * Fzf__XO;
    real_type t8   = ModelPars[83];
    real_type t10  = ModelPars[186];
    real_type t11  = ModelPars[91];
    real_type t12  = Fzf__XO - t2;
    real_type t16  = t3 * t12 * t11 + ModelPars[89];
    real_type t20  = ModelPars[93];
    real_type t23  = exp(t3 * t12 * t20);
    real_type t24  = ModelPars[188];
    real_type t25  = t24 * t23;
    real_type t26  = t10 * t8;
    real_type t31  = t5 * (t3 * t12 * t1 + ModelPars[85]);
    real_type t33  = Fzf__XO * t31 * t26;
    real_type t35  = t33 + ModelPars[180];
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
    real_type t60  = ModelPars[110];
    real_type t64  = phi__f__XO * ModelPars[102] + ModelPars[98];
    real_type t66  = ModelPars[100] * ModelPars[100];
    real_type t68  = t66 * t46 + 1;
    real_type t69  = sqrt(t68);
    real_type t71  = 1.0 / t69 * t64;
    real_type t72  = ModelPars[114];
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
  Straight::Fxf_D_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[243] * (t5 * t3 * ModelPars[87] + ModelPars[85]);
    real_type t13  = ModelPars[83] * ModelPars[186];
    real_type t23  = exp(t5 * t3 * ModelPars[93]);
    real_type t26  = t10 * Fzf__XO;
    real_type t34  = atan(lambda__f__XO / (t26 * t13 + ModelPars[180]) * ModelPars[188] * t23 * (t5 * t3 * ModelPars[91] + ModelPars[89]) * Fzf__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[110];
    real_type t41  = ModelPars[102];
    real_type t42  = lambda__f__XO * lambda__f__XO;
    real_type t44  = ModelPars[100] * ModelPars[100];
    real_type t46  = t44 * t42 + 1;
    real_type t47  = sqrt(t46);
    real_type t48  = 1.0 / t47;
    real_type t50  = ModelPars[114];
    real_type t51  = alpha__f__XO + t50;
    real_type t55  = t41 * phi__f__XO + ModelPars[98];
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
  Straight::Fxf_D_2_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[243] * (t5 * t3 * ModelPars[87] + ModelPars[85]);
    real_type t11  = t10 * Fzf__XO;
    real_type t14  = ModelPars[83] * ModelPars[186];
    real_type t24  = exp(t5 * t3 * ModelPars[93]);
    real_type t34  = atan(lambda__f__XO / (t11 * t14 + ModelPars[180]) * ModelPars[188] * t24 * (t5 * t3 * ModelPars[91] + ModelPars[89]) * Fzf__XO);
    real_type t36  = sin(t34 * t14);
    real_type t37  = ModelPars[110];
    real_type t39  = ModelPars[102];
    real_type t40  = t39 * t39;
    real_type t43  = lambda__f__XO * lambda__f__XO;
    real_type t45  = ModelPars[100] * ModelPars[100];
    real_type t47  = t45 * t43 + 1;
    real_type t48  = sqrt(t47);
    real_type t50  = 1.0 / t48 / t47;
    real_type t51  = ModelPars[114];
    real_type t52  = alpha__f__XO + t51;
    real_type t53  = t52 * t52;
    real_type t58  = t39 * phi__f__XO + ModelPars[98];
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
  Straight::Fxf_D_2_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[243] * (t5 * t3 * ModelPars[87] + ModelPars[85]);
    real_type t13  = ModelPars[83] * ModelPars[186];
    real_type t23  = exp(t5 * t3 * ModelPars[93]);
    real_type t26  = t10 * Fzf__XO;
    real_type t34  = atan(lambda__f__XO / (t26 * t13 + ModelPars[180]) * ModelPars[188] * t23 * (t5 * t3 * ModelPars[91] + ModelPars[89]) * Fzf__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[110];
    real_type t41  = ModelPars[102];
    real_type t42  = lambda__f__XO * lambda__f__XO;
    real_type t44  = ModelPars[100] * ModelPars[100];
    real_type t46  = t44 * t42 + 1;
    real_type t47  = sqrt(t46);
    real_type t48  = 1.0 / t47;
    real_type t52  = t41 * phi__f__XO + ModelPars[98];
    real_type t53  = t52 * t52;
    real_type t54  = 1.0 / t46;
    real_type t55  = t54 * t53;
    real_type t56  = ModelPars[114];
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
  Straight::Fxf_D_2_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[243] * (t5 * t3 * ModelPars[87] + ModelPars[85]);
    real_type t11  = Fzf__XO * Fzf__XO;
    real_type t12  = ModelPars[83];
    real_type t15  = ModelPars[186];
    real_type t20  = t5 * t3 * ModelPars[91] + ModelPars[89];
    real_type t25  = exp(t5 * t3 * ModelPars[93]);
    real_type t26  = ModelPars[188];
    real_type t28  = t15 * t12;
    real_type t29  = t10 * Fzf__XO;
    real_type t32  = t29 * t28 + ModelPars[180];
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
    real_type t59  = ModelPars[110];
    real_type t60  = ModelPars[102];
    real_type t64  = ModelPars[100] * ModelPars[100];
    real_type t66  = t64 * t45 + 1;
    real_type t67  = sqrt(t66);
    real_type t68  = 1.0 / t67;
    real_type t69  = ModelPars[114];
    real_type t70  = alpha__f__XO + t69;
    real_type t74  = t60 * phi__f__XO + ModelPars[98];
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
  Straight::Fxf_D_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[243] * (t5 * t3 * ModelPars[87] + ModelPars[85]);
    real_type t13  = ModelPars[83] * ModelPars[186];
    real_type t23  = exp(t5 * t3 * ModelPars[93]);
    real_type t34  = atan(lambda__f__XO / (Fzf__XO * t10 * t13 + ModelPars[180]) * ModelPars[188] * t23 * (t5 * t3 * ModelPars[91] + ModelPars[89]) * Fzf__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[110];
    real_type t44  = phi__f__XO * ModelPars[102] + ModelPars[98];
    real_type t45  = lambda__f__XO * lambda__f__XO;
    real_type t47  = ModelPars[100] * ModelPars[100];
    real_type t49  = t47 * t45 + 1;
    real_type t50  = sqrt(t49);
    real_type t52  = 1.0 / t50 * t44;
    real_type t53  = t44 * t44;
    real_type t56  = ModelPars[114];
    real_type t57  = alpha__f__XO + t56;
    real_type t58  = t57 * t57;
    real_type t63  = atan(t57 * t52);
    real_type t65  = sin(t63 * t38);
    real_type t68  = atan(t56 * t52);
    real_type t70  = cos(t68 * t38);
    return -1.0 / t70 * t65 / (t58 / t49 * t53 + 1) * t52 * t38 * t36 * Fzf__XO * t10;
  }

  real_type
  Straight::Fxf_D_3_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[243] * (t5 * t3 * ModelPars[87] + ModelPars[85]);
    real_type t13  = ModelPars[83] * ModelPars[186];
    real_type t23  = exp(t5 * t3 * ModelPars[93]);
    real_type t34  = atan(lambda__f__XO / (Fzf__XO * t10 * t13 + ModelPars[180]) * ModelPars[188] * t23 * (t5 * t3 * ModelPars[91] + ModelPars[89]) * Fzf__XO);
    real_type t36  = sin(t34 * t13);
    real_type t37  = t36 * Fzf__XO;
    real_type t38  = ModelPars[110];
    real_type t44  = phi__f__XO * ModelPars[102] + ModelPars[98];
    real_type t45  = t44 * t44;
    real_type t47  = lambda__f__XO * lambda__f__XO;
    real_type t49  = ModelPars[100] * ModelPars[100];
    real_type t51  = t49 * t47 + 1;
    real_type t52  = sqrt(t51);
    real_type t57  = 1.0 / t51 * t45;
    real_type t58  = ModelPars[114];
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
  Straight::Fxf_D_3_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[243] * (t5 * t3 * ModelPars[87] + ModelPars[85]);
    real_type t11  = Fzf__XO * Fzf__XO;
    real_type t12  = ModelPars[83];
    real_type t15  = ModelPars[186];
    real_type t20  = t5 * t3 * ModelPars[91] + ModelPars[89];
    real_type t25  = exp(t5 * t3 * ModelPars[93]);
    real_type t26  = ModelPars[188];
    real_type t30  = t15 * t12;
    real_type t31  = Fzf__XO * t10;
    real_type t34  = t31 * t30 + ModelPars[180];
    real_type t35  = 1.0 / t34;
    real_type t36  = t20 * t20;
    real_type t38  = t25 * t25;
    real_type t40  = t26 * t26;
    real_type t41  = t34 * t34;
    real_type t44  = lambda__f__XO * lambda__f__XO;
    real_type t55  = atan(lambda__f__XO * t35 * t26 * t25 * t20 * Fzf__XO);
    real_type t56  = t55 * t30;
    real_type t57  = cos(t56);
    real_type t58  = ModelPars[110];
    real_type t64  = phi__f__XO * ModelPars[102] + ModelPars[98];
    real_type t66  = ModelPars[100] * ModelPars[100];
    real_type t68  = t66 * t44 + 1;
    real_type t69  = sqrt(t68);
    real_type t71  = 1.0 / t69 * t64;
    real_type t72  = t64 * t64;
    real_type t74  = 1.0 / t68 * t72;
    real_type t75  = ModelPars[114];
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
  Straight::Fxf_D_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[243] * (t5 * t3 * ModelPars[87] + ModelPars[85]);
    real_type t11  = Fzf__XO * Fzf__XO;
    real_type t15  = ModelPars[83] * ModelPars[186];
    real_type t20  = t5 * t3 * ModelPars[91] + ModelPars[89];
    real_type t26  = exp(t5 * t3 * ModelPars[93]);
    real_type t27  = ModelPars[188];
    real_type t29  = Fzf__XO * t10;
    real_type t32  = t29 * t15 + ModelPars[180];
    real_type t33  = 1.0 / t32;
    real_type t35  = t20 * t20;
    real_type t37  = t26 * t26;
    real_type t39  = t27 * t27;
    real_type t40  = t32 * t32;
    real_type t43  = lambda__f__XO * lambda__f__XO;
    real_type t53  = atan(lambda__f__XO * t33 * t27 * t26 * t20 * Fzf__XO);
    real_type t54  = t53 * t15;
    real_type t55  = cos(t54);
    real_type t57  = ModelPars[110];
    real_type t61  = phi__f__XO * ModelPars[102] + ModelPars[98];
    real_type t63  = ModelPars[100] * ModelPars[100];
    real_type t65  = t63 * t43 + 1;
    real_type t66  = sqrt(t65);
    real_type t68  = 1.0 / t66 * t61;
    real_type t69  = ModelPars[114];
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
  Straight::Fxf_D_4_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[243] * (t5 * t3 * ModelPars[87] + ModelPars[85]);
    real_type t11  = Fzf__XO * Fzf__XO;
    real_type t12  = t11 * t11;
    real_type t14  = ModelPars[83];
    real_type t15  = ModelPars[186];
    real_type t16  = t15 * t14;
    real_type t21  = t5 * t3 * ModelPars[91] + ModelPars[89];
    real_type t22  = t21 * t21;
    real_type t27  = exp(t5 * t3 * ModelPars[93]);
    real_type t28  = t27 * t27;
    real_type t33  = ModelPars[188];
    real_type t34  = t33 * t33;
    real_type t36  = Fzf__XO * t10;
    real_type t39  = t36 * t16 + ModelPars[180];
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
    real_type t64  = ModelPars[110];
    real_type t68  = phi__f__XO * ModelPars[102] + ModelPars[98];
    real_type t70  = ModelPars[100] * ModelPars[100];
    real_type t72  = t70 * t48 + 1;
    real_type t73  = sqrt(t72);
    real_type t75  = 1.0 / t73 * t68;
    real_type t76  = ModelPars[114];
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
  Straight::Fxr( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t11  = Fzr__XO * ModelPars[244] * (t5 * t3 * ModelPars[88] + ModelPars[86]);
    real_type t14  = ModelPars[84] * ModelPars[187];
    real_type t24  = exp(t5 * t3 * ModelPars[94]);
    real_type t34  = atan(lambda__r__XO / (t11 * t14 + ModelPars[181]) * ModelPars[189] * t24 * (t5 * t3 * ModelPars[92] + ModelPars[90]) * Fzr__XO);
    real_type t36  = sin(t34 * t14);
    real_type t37  = ModelPars[111];
    real_type t42  = lambda__r__XO * lambda__r__XO;
    real_type t44  = ModelPars[101] * ModelPars[101];
    real_type t47  = sqrt(t44 * t42 + 1);
    real_type t49  = 1.0 / t47 * (phi__XO * ModelPars[103] + ModelPars[99]);
    real_type t50  = ModelPars[115];
    real_type t53  = atan((alpha__r__XO + t50) * t49);
    real_type t55  = cos(t53 * t37);
    real_type t58  = atan(t50 * t49);
    real_type t60  = cos(t58 * t37);
    return 1.0 / t60 * t55 * t36 * t11;
  }

  real_type
  Straight::Fxr_D_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[88];
    real_type t2   = ModelPars[6];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[244];
    real_type t9   = ModelPars[84] * ModelPars[187];
    real_type t10  = ModelPars[92];
    real_type t11  = Fzr__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[90];
    real_type t16  = t15 * Fzr__XO;
    real_type t17  = ModelPars[94];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[189];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[86]);
    real_type t29  = Fzr__XO * t27 * t9;
    real_type t31  = t29 + ModelPars[181];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__r__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t40  = ModelPars[111];
    real_type t45  = lambda__r__XO * lambda__r__XO;
    real_type t47  = ModelPars[101] * ModelPars[101];
    real_type t50  = sqrt(t47 * t45 + 1);
    real_type t52  = 1.0 / t50 * (phi__XO * ModelPars[103] + ModelPars[99]);
    real_type t53  = ModelPars[115];
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
  Straight::Fxr_D_1_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[88];
    real_type t2   = ModelPars[6];
    real_type t3   = 1.0 / t2;
    real_type t4   = t3 * t1;
    real_type t5   = ModelPars[244];
    real_type t6   = t5 * t4;
    real_type t7   = ModelPars[84];
    real_type t8   = ModelPars[187];
    real_type t9   = t8 * t7;
    real_type t10  = ModelPars[92];
    real_type t11  = Fzr__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[90];
    real_type t16  = t15 * Fzr__XO;
    real_type t17  = ModelPars[94];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[189];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[86]);
    real_type t29  = Fzr__XO * t27 * t9;
    real_type t31  = t29 + ModelPars[181];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__r__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t39  = ModelPars[111];
    real_type t44  = lambda__r__XO * lambda__r__XO;
    real_type t46  = ModelPars[101] * ModelPars[101];
    real_type t49  = sqrt(t46 * t44 + 1);
    real_type t51  = 1.0 / t49 * (phi__XO * ModelPars[103] + ModelPars[99]);
    real_type t52  = ModelPars[115];
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
  Straight::Fxr_D_1_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[88];
    real_type t2   = ModelPars[6];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[244];
    real_type t6   = t5 * t3 * t1;
    real_type t7   = ModelPars[84];
    real_type t8   = ModelPars[187];
    real_type t9   = t8 * t7;
    real_type t10  = ModelPars[92];
    real_type t11  = Fzr__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[90];
    real_type t16  = t15 * Fzr__XO;
    real_type t17  = ModelPars[94];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[189];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[86]);
    real_type t28  = Fzr__XO * t27;
    real_type t31  = t28 * t9 + ModelPars[181];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__r__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t39  = t38 * Fzr__XO;
    real_type t40  = ModelPars[111];
    real_type t43  = ModelPars[103];
    real_type t44  = lambda__r__XO * lambda__r__XO;
    real_type t46  = ModelPars[101] * ModelPars[101];
    real_type t48  = t46 * t44 + 1;
    real_type t49  = sqrt(t48);
    real_type t50  = 1.0 / t49;
    real_type t51  = t50 * t43;
    real_type t52  = ModelPars[115];
    real_type t53  = alpha__r__XO + t52;
    real_type t57  = t43 * phi__XO + ModelPars[99];
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
  Straight::Fxr_D_1_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[88];
    real_type t2   = ModelPars[6];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[244];
    real_type t9   = ModelPars[84] * ModelPars[187];
    real_type t10  = ModelPars[92];
    real_type t11  = Fzr__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[90];
    real_type t16  = t15 * Fzr__XO;
    real_type t17  = ModelPars[94];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[189];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[86]);
    real_type t28  = Fzr__XO * t27;
    real_type t31  = t28 * t9 + ModelPars[181];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__r__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t41  = ModelPars[111];
    real_type t45  = phi__XO * ModelPars[103] + ModelPars[99];
    real_type t47  = lambda__r__XO * lambda__r__XO;
    real_type t49  = ModelPars[101] * ModelPars[101];
    real_type t51  = t49 * t47 + 1;
    real_type t52  = sqrt(t51);
    real_type t53  = 1.0 / t52;
    real_type t55  = t45 * t45;
    real_type t58  = ModelPars[115];
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
  Straight::Fxr_D_1_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[88];
    real_type t2   = ModelPars[6];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[244];
    real_type t6   = t5 * t3 * t1;
    real_type t7   = Fzr__XO * Fzr__XO;
    real_type t8   = ModelPars[84];
    real_type t10  = ModelPars[187];
    real_type t11  = ModelPars[92];
    real_type t12  = Fzr__XO - t2;
    real_type t16  = t3 * t12 * t11 + ModelPars[90];
    real_type t20  = ModelPars[94];
    real_type t23  = exp(t3 * t12 * t20);
    real_type t24  = ModelPars[189];
    real_type t25  = t24 * t23;
    real_type t26  = t10 * t8;
    real_type t31  = t5 * (t3 * t12 * t1 + ModelPars[86]);
    real_type t33  = Fzr__XO * t31 * t26;
    real_type t35  = t33 + ModelPars[181];
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
    real_type t60  = ModelPars[111];
    real_type t64  = phi__XO * ModelPars[103] + ModelPars[99];
    real_type t66  = ModelPars[101] * ModelPars[101];
    real_type t68  = t66 * t46 + 1;
    real_type t69  = sqrt(t68);
    real_type t71  = 1.0 / t69 * t64;
    real_type t72  = ModelPars[115];
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
  Straight::Fxr_D_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[88] + ModelPars[86]);
    real_type t13  = ModelPars[84] * ModelPars[187];
    real_type t23  = exp(t5 * t3 * ModelPars[94]);
    real_type t26  = t10 * Fzr__XO;
    real_type t34  = atan(lambda__r__XO / (t26 * t13 + ModelPars[181]) * ModelPars[189] * t23 * (t5 * t3 * ModelPars[92] + ModelPars[90]) * Fzr__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[111];
    real_type t41  = ModelPars[103];
    real_type t42  = lambda__r__XO * lambda__r__XO;
    real_type t44  = ModelPars[101] * ModelPars[101];
    real_type t46  = t44 * t42 + 1;
    real_type t47  = sqrt(t46);
    real_type t48  = 1.0 / t47;
    real_type t50  = ModelPars[115];
    real_type t51  = alpha__r__XO + t50;
    real_type t55  = t41 * phi__XO + ModelPars[99];
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
  Straight::Fxr_D_2_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[88] + ModelPars[86]);
    real_type t11  = Fzr__XO * t10;
    real_type t14  = ModelPars[84] * ModelPars[187];
    real_type t24  = exp(t5 * t3 * ModelPars[94]);
    real_type t34  = atan(lambda__r__XO / (t11 * t14 + ModelPars[181]) * ModelPars[189] * t24 * (t5 * t3 * ModelPars[92] + ModelPars[90]) * Fzr__XO);
    real_type t36  = sin(t34 * t14);
    real_type t37  = ModelPars[111];
    real_type t39  = ModelPars[103];
    real_type t40  = t39 * t39;
    real_type t43  = lambda__r__XO * lambda__r__XO;
    real_type t45  = ModelPars[101] * ModelPars[101];
    real_type t47  = t45 * t43 + 1;
    real_type t48  = sqrt(t47);
    real_type t50  = 1.0 / t48 / t47;
    real_type t51  = ModelPars[115];
    real_type t52  = alpha__r__XO + t51;
    real_type t53  = t52 * t52;
    real_type t58  = t39 * phi__XO + ModelPars[99];
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
  Straight::Fxr_D_2_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[88] + ModelPars[86]);
    real_type t13  = ModelPars[84] * ModelPars[187];
    real_type t23  = exp(t5 * t3 * ModelPars[94]);
    real_type t26  = Fzr__XO * t10;
    real_type t34  = atan(lambda__r__XO / (t26 * t13 + ModelPars[181]) * ModelPars[189] * t23 * (t5 * t3 * ModelPars[92] + ModelPars[90]) * Fzr__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[111];
    real_type t41  = ModelPars[103];
    real_type t42  = lambda__r__XO * lambda__r__XO;
    real_type t44  = ModelPars[101] * ModelPars[101];
    real_type t46  = t44 * t42 + 1;
    real_type t47  = sqrt(t46);
    real_type t48  = 1.0 / t47;
    real_type t52  = t41 * phi__XO + ModelPars[99];
    real_type t53  = t52 * t52;
    real_type t54  = 1.0 / t46;
    real_type t55  = t54 * t53;
    real_type t56  = ModelPars[115];
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
  Straight::Fxr_D_2_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[88] + ModelPars[86]);
    real_type t11  = Fzr__XO * Fzr__XO;
    real_type t12  = ModelPars[84];
    real_type t15  = ModelPars[187];
    real_type t20  = t5 * t3 * ModelPars[92] + ModelPars[90];
    real_type t25  = exp(t5 * t3 * ModelPars[94]);
    real_type t26  = ModelPars[189];
    real_type t28  = t15 * t12;
    real_type t29  = Fzr__XO * t10;
    real_type t32  = t29 * t28 + ModelPars[181];
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
    real_type t59  = ModelPars[111];
    real_type t60  = ModelPars[103];
    real_type t64  = ModelPars[101] * ModelPars[101];
    real_type t66  = t64 * t45 + 1;
    real_type t67  = sqrt(t66);
    real_type t68  = 1.0 / t67;
    real_type t69  = ModelPars[115];
    real_type t70  = alpha__r__XO + t69;
    real_type t74  = t60 * phi__XO + ModelPars[99];
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
  Straight::Fxr_D_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[88] + ModelPars[86]);
    real_type t13  = ModelPars[84] * ModelPars[187];
    real_type t23  = exp(t5 * t3 * ModelPars[94]);
    real_type t34  = atan(lambda__r__XO / (Fzr__XO * t10 * t13 + ModelPars[181]) * ModelPars[189] * t23 * (t5 * t3 * ModelPars[92] + ModelPars[90]) * Fzr__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[111];
    real_type t44  = phi__XO * ModelPars[103] + ModelPars[99];
    real_type t45  = lambda__r__XO * lambda__r__XO;
    real_type t47  = ModelPars[101] * ModelPars[101];
    real_type t49  = t47 * t45 + 1;
    real_type t50  = sqrt(t49);
    real_type t52  = 1.0 / t50 * t44;
    real_type t53  = t44 * t44;
    real_type t56  = ModelPars[115];
    real_type t57  = alpha__r__XO + t56;
    real_type t58  = t57 * t57;
    real_type t63  = atan(t57 * t52);
    real_type t65  = sin(t63 * t38);
    real_type t68  = atan(t56 * t52);
    real_type t70  = cos(t68 * t38);
    return -1.0 / t70 * t65 / (t58 / t49 * t53 + 1) * t52 * t38 * t36 * Fzr__XO * t10;
  }

  real_type
  Straight::Fxr_D_3_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[88] + ModelPars[86]);
    real_type t13  = ModelPars[84] * ModelPars[187];
    real_type t23  = exp(t5 * t3 * ModelPars[94]);
    real_type t34  = atan(lambda__r__XO / (Fzr__XO * t10 * t13 + ModelPars[181]) * ModelPars[189] * t23 * (t5 * t3 * ModelPars[92] + ModelPars[90]) * Fzr__XO);
    real_type t36  = sin(t34 * t13);
    real_type t37  = t36 * Fzr__XO;
    real_type t38  = ModelPars[111];
    real_type t44  = phi__XO * ModelPars[103] + ModelPars[99];
    real_type t45  = t44 * t44;
    real_type t47  = lambda__r__XO * lambda__r__XO;
    real_type t49  = ModelPars[101] * ModelPars[101];
    real_type t51  = t49 * t47 + 1;
    real_type t52  = sqrt(t51);
    real_type t57  = 1.0 / t51 * t45;
    real_type t58  = ModelPars[115];
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
  Straight::Fxr_D_3_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[88] + ModelPars[86]);
    real_type t11  = Fzr__XO * Fzr__XO;
    real_type t12  = ModelPars[84];
    real_type t15  = ModelPars[187];
    real_type t20  = t5 * t3 * ModelPars[92] + ModelPars[90];
    real_type t25  = exp(t5 * t3 * ModelPars[94]);
    real_type t26  = ModelPars[189];
    real_type t30  = t15 * t12;
    real_type t31  = Fzr__XO * t10;
    real_type t34  = t31 * t30 + ModelPars[181];
    real_type t35  = 1.0 / t34;
    real_type t36  = t20 * t20;
    real_type t38  = t25 * t25;
    real_type t40  = t26 * t26;
    real_type t41  = t34 * t34;
    real_type t44  = lambda__r__XO * lambda__r__XO;
    real_type t55  = atan(lambda__r__XO * t35 * t26 * t25 * t20 * Fzr__XO);
    real_type t56  = t55 * t30;
    real_type t57  = cos(t56);
    real_type t58  = ModelPars[111];
    real_type t64  = phi__XO * ModelPars[103] + ModelPars[99];
    real_type t66  = ModelPars[101] * ModelPars[101];
    real_type t68  = t66 * t44 + 1;
    real_type t69  = sqrt(t68);
    real_type t71  = 1.0 / t69 * t64;
    real_type t72  = t64 * t64;
    real_type t74  = 1.0 / t68 * t72;
    real_type t75  = ModelPars[115];
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
  Straight::Fxr_D_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[88] + ModelPars[86]);
    real_type t11  = Fzr__XO * Fzr__XO;
    real_type t15  = ModelPars[84] * ModelPars[187];
    real_type t20  = t5 * t3 * ModelPars[92] + ModelPars[90];
    real_type t26  = exp(t5 * t3 * ModelPars[94]);
    real_type t27  = ModelPars[189];
    real_type t29  = Fzr__XO * t10;
    real_type t32  = t29 * t15 + ModelPars[181];
    real_type t33  = 1.0 / t32;
    real_type t35  = t20 * t20;
    real_type t37  = t26 * t26;
    real_type t39  = t27 * t27;
    real_type t40  = t32 * t32;
    real_type t43  = lambda__r__XO * lambda__r__XO;
    real_type t53  = atan(lambda__r__XO * t33 * t27 * t26 * t20 * Fzr__XO);
    real_type t54  = t53 * t15;
    real_type t55  = cos(t54);
    real_type t57  = ModelPars[111];
    real_type t61  = phi__XO * ModelPars[103] + ModelPars[99];
    real_type t63  = ModelPars[101] * ModelPars[101];
    real_type t65  = t63 * t43 + 1;
    real_type t66  = sqrt(t65);
    real_type t68  = 1.0 / t66 * t61;
    real_type t69  = ModelPars[115];
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
  Straight::Fxr_D_4_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[88] + ModelPars[86]);
    real_type t11  = Fzr__XO * Fzr__XO;
    real_type t12  = t11 * t11;
    real_type t14  = ModelPars[84];
    real_type t15  = ModelPars[187];
    real_type t16  = t15 * t14;
    real_type t21  = t5 * t3 * ModelPars[92] + ModelPars[90];
    real_type t22  = t21 * t21;
    real_type t27  = exp(t5 * t3 * ModelPars[94]);
    real_type t28  = t27 * t27;
    real_type t33  = ModelPars[189];
    real_type t34  = t33 * t33;
    real_type t36  = Fzr__XO * t10;
    real_type t39  = t36 * t16 + ModelPars[181];
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
    real_type t64  = ModelPars[111];
    real_type t68  = phi__XO * ModelPars[103] + ModelPars[99];
    real_type t70  = ModelPars[101] * ModelPars[101];
    real_type t72  = t70 * t48 + 1;
    real_type t73  = sqrt(t72);
    real_type t75  = 1.0 / t73 * t68;
    real_type t76  = ModelPars[115];
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
  Straight::Fyf( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[42];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t8   = ModelPars[48] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t12  = sqrt(1.0 / t9 * t3 * t2);
    real_type t13  = ModelPars[50];
    real_type t15  = ModelPars[7];
    real_type t18  = Fzf__XO - t15;
    real_type t20  = t15 * ModelPars[36] + t18 * ModelPars[38];
    real_type t23  = ModelPars[44] * t5 + 1;
    real_type t28  = 1.0 / t1;
    real_type t37  = 1.0 / t20;
    real_type t45  = ModelPars[46] * phi__f__XO;
    real_type t55  = atan(((-t23 * t37 * phi__f__XO * ModelPars[40] * Fzf__XO + alpha__f__XO) / t12 / t8 * Fzf__XO * t1 - t23 * t37 * t8 * t28 * t12 * t45) * t8 / Fzf__XO * t28 / t13 / t23 * t20);
    real_type t57  = sin(t55 * t13);
    real_type t63  = ModelPars[112];
    real_type t66  = ModelPars[106] * ModelPars[106];
    real_type t67  = tan(alpha__f__XO);
    real_type t70  = pow(t67 - ModelPars[108], 2);
    real_type t73  = sqrt(t70 * t66 + 1);
    real_type t75  = 1.0 / t73 * ModelPars[104];
    real_type t76  = ModelPars[116];
    real_type t80  = 1.0 / t15 * t18 * ModelPars[118];
    real_type t83  = atan((lambda__f__XO + t76 + t80) * t75);
    real_type t85  = cos(t83 * t63);
    real_type t89  = atan((t76 + t80) * t75);
    real_type t91  = cos(t89 * t63);
    return 1.0 / t91 * t85 * (t8 * t28 * t12 * t45 + t57 * t12);
  }

  real_type
  Straight::Fyf_D_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[42];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t8   = ModelPars[48] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[50];
    real_type t16  = ModelPars[7];
    real_type t18  = ModelPars[38];
    real_type t19  = Fzf__XO - t16;
    real_type t21  = t16 * ModelPars[36] + t19 * t18;
    real_type t24  = ModelPars[44] * t5 + 1;
    real_type t25  = 1.0 / t24;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t25 * t21;
    real_type t29  = 1.0 / t1;
    real_type t31  = 1.0 / Fzf__XO * t29;
    real_type t32  = Fzf__XO * t1;
    real_type t33  = 1.0 / t8;
    real_type t34  = t13 * t33;
    real_type t35  = ModelPars[40];
    real_type t36  = Fzf__XO * t35;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__f__XO * t36 + alpha__f__XO;
    real_type t45  = ModelPars[46] * phi__f__XO;
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
    real_type t132 = ModelPars[112];
    real_type t133 = ModelPars[104];
    real_type t135 = ModelPars[106] * ModelPars[106];
    real_type t136 = tan(alpha__f__XO);
    real_type t139 = pow(t136 - ModelPars[108], 2);
    real_type t141 = t139 * t135 + 1;
    real_type t142 = sqrt(t141);
    real_type t143 = 1.0 / t142;
    real_type t144 = t143 * t133;
    real_type t145 = ModelPars[116];
    real_type t146 = ModelPars[118];
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
  Straight::Fyf_D_1_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[42];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t8   = ModelPars[48] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t14  = 1.0 / t12 / t11;
    real_type t15  = ModelPars[50];
    real_type t17  = ModelPars[7];
    real_type t19  = ModelPars[38];
    real_type t20  = Fzf__XO - t17;
    real_type t22  = t17 * ModelPars[36] + t20 * t19;
    real_type t25  = ModelPars[44] * t5 + 1;
    real_type t26  = 1.0 / t25;
    real_type t28  = 1.0 / t15;
    real_type t29  = t28 * t26 * t22;
    real_type t30  = 1.0 / t1;
    real_type t32  = 1.0 / Fzf__XO * t30;
    real_type t33  = Fzf__XO * t1;
    real_type t34  = 1.0 / t8;
    real_type t35  = 1.0 / t12;
    real_type t36  = t35 * t34;
    real_type t37  = ModelPars[40];
    real_type t38  = Fzf__XO * t37;
    real_type t39  = 1.0 / t22;
    real_type t43  = -t25 * t39 * phi__f__XO * t38 + alpha__f__XO;
    real_type t47  = ModelPars[46] * phi__f__XO;
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
    real_type t247 = ModelPars[112];
    real_type t248 = ModelPars[104];
    real_type t250 = ModelPars[106] * ModelPars[106];
    real_type t251 = tan(alpha__f__XO);
    real_type t254 = pow(t251 - ModelPars[108], 2);
    real_type t256 = t254 * t250 + 1;
    real_type t257 = sqrt(t256);
    real_type t258 = 1.0 / t257;
    real_type t259 = t258 * t248;
    real_type t260 = ModelPars[116];
    real_type t261 = ModelPars[118];
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
  Straight::Fyf_D_1_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[42];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t4   = t3 * t2;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t6   = ModelPars[48];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t4;
    real_type t12  = sqrt(t11);
    real_type t14  = 1.0 / t12 / t11;
    real_type t15  = ModelPars[50];
    real_type t17  = ModelPars[7];
    real_type t19  = ModelPars[38];
    real_type t20  = Fzf__XO - t17;
    real_type t22  = t17 * ModelPars[36] + t20 * t19;
    real_type t23  = ModelPars[44];
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
    real_type t37  = ModelPars[40];
    real_type t38  = Fzf__XO * t37;
    real_type t39  = 1.0 / t22;
    real_type t43  = -t25 * t39 * phi__f__XO * t38 + alpha__f__XO;
    real_type t46  = ModelPars[46];
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
    real_type t383 = ModelPars[112];
    real_type t384 = ModelPars[104];
    real_type t386 = ModelPars[106] * ModelPars[106];
    real_type t387 = tan(alpha__f__XO);
    real_type t390 = pow(t387 - ModelPars[108], 2);
    real_type t392 = t390 * t386 + 1;
    real_type t393 = sqrt(t392);
    real_type t394 = 1.0 / t393;
    real_type t395 = t394 * t384;
    real_type t396 = ModelPars[116];
    real_type t397 = ModelPars[118];
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
  Straight::Fyf_D_1_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = 1.0 / Fzf__XO;
    real_type t3   = ModelPars[7];
    real_type t5   = ModelPars[38];
    real_type t6   = Fzf__XO - t3;
    real_type t8   = t3 * ModelPars[36] + t6 * t5;
    real_type t10  = phi__f__XO * phi__f__XO;
    real_type t13  = ModelPars[44] * t10 + 1;
    real_type t14  = 1.0 / t13;
    real_type t15  = t8 * t8;
    real_type t16  = t13 * t13;
    real_type t17  = 1.0 / t16;
    real_type t19  = ModelPars[50];
    real_type t20  = t19 * t19;
    real_type t23  = ModelPars[42];
    real_type t24  = t23 * t23;
    real_type t26  = Fzf__XO * Fzf__XO;
    real_type t27  = 1.0 / t26;
    real_type t31  = ModelPars[48] * t10 + 1;
    real_type t32  = t31 * t31;
    real_type t33  = Fzf__XO * t23;
    real_type t34  = 1.0 / t31;
    real_type t36  = 1.0 / t32;
    real_type t37  = t36 * t26 * t24;
    real_type t38  = sqrt(t37);
    real_type t39  = 1.0 / t38;
    real_type t40  = t39 * t34;
    real_type t41  = ModelPars[40];
    real_type t42  = Fzf__XO * t41;
    real_type t43  = 1.0 / t8;
    real_type t47  = -t13 * t43 * phi__f__XO * t42 + alpha__f__XO;
    real_type t51  = ModelPars[46] * phi__f__XO;
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
    real_type t157 = ModelPars[112];
    real_type t158 = ModelPars[104];
    real_type t160 = ModelPars[106] * ModelPars[106];
    real_type t161 = tan(alpha__f__XO);
    real_type t163 = t161 - ModelPars[108];
    real_type t164 = t163 * t163;
    real_type t166 = t164 * t160 + 1;
    real_type t167 = sqrt(t166);
    real_type t168 = 1.0 / t167;
    real_type t169 = t168 * t158;
    real_type t170 = ModelPars[116];
    real_type t171 = ModelPars[118];
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
  Straight::Fyf_D_1_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[42];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t8   = ModelPars[48] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[50];
    real_type t16  = ModelPars[7];
    real_type t18  = ModelPars[38];
    real_type t19  = Fzf__XO - t16;
    real_type t21  = t16 * ModelPars[36] + t19 * t18;
    real_type t24  = ModelPars[44] * t5 + 1;
    real_type t25  = 1.0 / t24;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t25 * t21;
    real_type t29  = 1.0 / t1;
    real_type t31  = 1.0 / Fzf__XO * t29;
    real_type t32  = Fzf__XO * t1;
    real_type t33  = 1.0 / t8;
    real_type t34  = t13 * t33;
    real_type t35  = ModelPars[40];
    real_type t36  = Fzf__XO * t35;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__f__XO * t36 + alpha__f__XO;
    real_type t45  = ModelPars[46] * phi__f__XO;
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
    real_type t132 = ModelPars[112];
    real_type t134 = ModelPars[104];
    real_type t137 = ModelPars[106] * ModelPars[106];
    real_type t138 = tan(alpha__f__XO);
    real_type t141 = pow(t138 - ModelPars[108], 2);
    real_type t143 = t141 * t137 + 1;
    real_type t144 = sqrt(t143);
    real_type t145 = 1.0 / t144;
    real_type t146 = t134 * t134;
    real_type t148 = 1.0 / t143 * t146;
    real_type t149 = ModelPars[116];
    real_type t150 = ModelPars[118];
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
  Straight::Fyf_D_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[42];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t6   = ModelPars[48];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[50];
    real_type t16  = ModelPars[7];
    real_type t19  = Fzf__XO - t16;
    real_type t21  = t16 * ModelPars[36] + t19 * ModelPars[38];
    real_type t22  = ModelPars[44];
    real_type t24  = t22 * t5 + 1;
    real_type t26  = 1.0 / t24 * t21;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t26;
    real_type t29  = 1.0 / t1;
    real_type t30  = 1.0 / Fzf__XO;
    real_type t31  = t30 * t29;
    real_type t32  = Fzf__XO * t1;
    real_type t34  = t13 / t8;
    real_type t36  = ModelPars[40] * Fzf__XO;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__f__XO * t36 + alpha__f__XO;
    real_type t44  = ModelPars[46];
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
    real_type t168 = ModelPars[112];
    real_type t171 = ModelPars[106] * ModelPars[106];
    real_type t172 = tan(alpha__f__XO);
    real_type t175 = pow(t172 - ModelPars[108], 2);
    real_type t178 = sqrt(t175 * t171 + 1);
    real_type t180 = 1.0 / t178 * ModelPars[104];
    real_type t181 = ModelPars[116];
    real_type t185 = 1.0 / t16 * t19 * ModelPars[118];
    real_type t188 = atan((lambda__f__XO + t181 + t185) * t180);
    real_type t190 = cos(t188 * t168);
    real_type t194 = atan((t181 + t185) * t180);
    real_type t196 = cos(t194 * t168);
    return 1.0 / t196 * t190 * (-2 * t63 / t9 / t8 * t3 * t2 * t57 * t13 + t153 / (t146 * t9 / t3 / t2 / t140 * t69 * t138 + 1) * (-2 * phi__f__XO * t22 * t51 * t8 * t30 * t71 * t69 * t21 + 2 * t51 * phi__f__XO * t6 * t30 * t71 * t26 + (-2 * t63 * t41 * t13 * t10 * t32 + 2 * t63 * t41 / t12 / t11 / t93 * t3 * Fzf__XO * t2 * t1 + (-2 * t22 * t37 * t5 * t36 - t48 * t36) * t34 * t32 - t24 * t37 * t8 * t29 * t110 + 2 * t6 * t3 * t24 * t37 * t10 * t1 * t13 * t115 - 2 * t48 * t6 * t29 * t124 - 2 * t22 * t37 * t47 * t124) * t8 * t31 * t28) * t14 * t12 + t47 * t110 - 2 * t6 * t3 * t10 * t1 * t13 * t115 + 2 * t6 * t29 * t12 * t115);
  }

  real_type
  Straight::Fyf_D_2_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[42];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t4   = t3 * t2;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t6   = ModelPars[48];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t4;
    real_type t12  = sqrt(t11);
    real_type t14  = 1.0 / t12 / t11;
    real_type t15  = ModelPars[50];
    real_type t17  = ModelPars[7];
    real_type t20  = Fzf__XO - t17;
    real_type t22  = t17 * ModelPars[36] + t20 * ModelPars[38];
    real_type t23  = ModelPars[44];
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
    real_type t37  = ModelPars[40];
    real_type t38  = Fzf__XO * t37;
    real_type t39  = 1.0 / t22;
    real_type t43  = -t25 * t39 * phi__f__XO * t38 + alpha__f__XO;
    real_type t46  = ModelPars[46];
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
    real_type t354 = ModelPars[112];
    real_type t357 = ModelPars[106] * ModelPars[106];
    real_type t358 = tan(alpha__f__XO);
    real_type t361 = pow(t358 - ModelPars[108], 2);
    real_type t364 = sqrt(t361 * t357 + 1);
    real_type t366 = 1.0 / t364 * ModelPars[104];
    real_type t367 = ModelPars[116];
    real_type t371 = 1.0 / t17 * t20 * ModelPars[118];
    real_type t374 = atan((lambda__f__XO + t367 + t371) * t366);
    real_type t376 = cos(t374 * t354);
    real_type t380 = atan((t367 + t371) * t366);
    real_type t382 = cos(t380 * t354);
    return 1.0 / t382 * t376 * t353;
  }

  real_type
  Straight::Fyf_D_2_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = phi__f__XO * phi__f__XO;
    real_type t2   = ModelPars[48];
    real_type t4   = t2 * t1 + 1;
    real_type t5   = 1.0 / t4;
    real_type t7   = ModelPars[7];
    real_type t10  = Fzf__XO - t7;
    real_type t12  = t10 * ModelPars[38] + t7 * ModelPars[36];
    real_type t14  = ModelPars[44];
    real_type t16  = t14 * t1 + 1;
    real_type t17  = 1.0 / t16;
    real_type t19  = t12 * t12;
    real_type t20  = t16 * t16;
    real_type t21  = 1.0 / t20;
    real_type t23  = ModelPars[50];
    real_type t24  = t23 * t23;
    real_type t27  = ModelPars[42];
    real_type t28  = t27 * t27;
    real_type t30  = Fzf__XO * Fzf__XO;
    real_type t33  = t4 * t4;
    real_type t34  = Fzf__XO * t27;
    real_type t36  = 1.0 / t33;
    real_type t37  = t36 * t30 * t28;
    real_type t38  = sqrt(t37);
    real_type t39  = 1.0 / t38;
    real_type t40  = t39 * t5;
    real_type t42  = ModelPars[40] * Fzf__XO;
    real_type t43  = 1.0 / t12;
    real_type t47  = -t16 * t43 * phi__f__XO * t42 + alpha__f__XO;
    real_type t50  = ModelPars[46];
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
    real_type t190 = ModelPars[112];
    real_type t191 = ModelPars[104];
    real_type t193 = ModelPars[106] * ModelPars[106];
    real_type t194 = tan(alpha__f__XO);
    real_type t196 = t194 - ModelPars[108];
    real_type t197 = t196 * t196;
    real_type t199 = t197 * t193 + 1;
    real_type t200 = sqrt(t199);
    real_type t202 = 1.0 / t200 * t191;
    real_type t203 = ModelPars[116];
    real_type t207 = 1.0 / t7 * t10 * ModelPars[118];
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
  Straight::Fyf_D_2_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[42];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t6   = ModelPars[48];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[50];
    real_type t16  = ModelPars[7];
    real_type t19  = Fzf__XO - t16;
    real_type t21  = t16 * ModelPars[36] + t19 * ModelPars[38];
    real_type t22  = ModelPars[44];
    real_type t24  = t22 * t5 + 1;
    real_type t26  = 1.0 / t24 * t21;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t26;
    real_type t29  = 1.0 / t1;
    real_type t30  = 1.0 / Fzf__XO;
    real_type t31  = t30 * t29;
    real_type t32  = Fzf__XO * t1;
    real_type t34  = t13 / t8;
    real_type t36  = ModelPars[40] * Fzf__XO;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__f__XO * t36 + alpha__f__XO;
    real_type t44  = ModelPars[46];
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
    real_type t168 = ModelPars[112];
    real_type t170 = ModelPars[104];
    real_type t173 = ModelPars[106] * ModelPars[106];
    real_type t174 = tan(alpha__f__XO);
    real_type t177 = pow(t174 - ModelPars[108], 2);
    real_type t179 = t177 * t173 + 1;
    real_type t180 = sqrt(t179);
    real_type t181 = 1.0 / t180;
    real_type t182 = t170 * t170;
    real_type t185 = ModelPars[116];
    real_type t189 = 1.0 / t16 * t19 * ModelPars[118];
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
  Straight::Fyf_D_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[7];
    real_type t5   = Fzf__XO - t2;
    real_type t7   = t2 * ModelPars[36] + t5 * ModelPars[38];
    real_type t8   = phi__f__XO * phi__f__XO;
    real_type t11  = ModelPars[44] * t8 + 1;
    real_type t13  = 1.0 / t11 * t7;
    real_type t14  = t7 * t7;
    real_type t15  = t11 * t11;
    real_type t18  = ModelPars[50];
    real_type t19  = t18 * t18;
    real_type t22  = ModelPars[42];
    real_type t23  = t22 * t22;
    real_type t25  = Fzf__XO * Fzf__XO;
    real_type t30  = ModelPars[48] * t8 + 1;
    real_type t31  = t30 * t30;
    real_type t37  = sqrt(1.0 / t31 * t25 * t23);
    real_type t42  = 1.0 / t7;
    real_type t50  = ModelPars[46] * phi__f__XO;
    real_type t52  = 1.0 / t22;
    real_type t57  = (-t11 * t42 * phi__f__XO * ModelPars[40] * Fzf__XO + alpha__f__XO) / t37 / t30 * Fzf__XO * t22 - t11 * t42 * t30 * t52 * t37 * t50;
    real_type t58  = t57 * t57;
    real_type t72  = atan(t57 * t30 / Fzf__XO * t52 / t18 * t13);
    real_type t73  = t72 * t18;
    real_type t74  = cos(t73);
    real_type t75  = ModelPars[112];
    real_type t76  = ModelPars[104];
    real_type t78  = ModelPars[106] * ModelPars[106];
    real_type t79  = tan(alpha__f__XO);
    real_type t81  = t79 - ModelPars[108];
    real_type t82  = t81 * t81;
    real_type t84  = t82 * t78 + 1;
    real_type t85  = sqrt(t84);
    real_type t87  = 1.0 / t85 * t76;
    real_type t88  = ModelPars[116];
    real_type t92  = 1.0 / t2 * t5 * ModelPars[118];
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
  Straight::Fyf_D_3_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[7];
    real_type t5   = Fzf__XO - t2;
    real_type t7   = t2 * ModelPars[36] + t5 * ModelPars[38];
    real_type t8   = t7 * t7;
    real_type t10  = phi__f__XO * phi__f__XO;
    real_type t13  = ModelPars[44] * t10 + 1;
    real_type t14  = t13 * t13;
    real_type t19  = 1.0 / t14 * t8;
    real_type t20  = ModelPars[50];
    real_type t21  = t20 * t20;
    real_type t22  = 1.0 / t21;
    real_type t24  = ModelPars[42];
    real_type t25  = t24 * t24;
    real_type t27  = Fzf__XO * Fzf__XO;
    real_type t32  = ModelPars[48] * t10 + 1;
    real_type t33  = t32 * t32;
    real_type t39  = sqrt(1.0 / t33 * t27 * t25);
    real_type t40  = 1.0 / t39;
    real_type t44  = 1.0 / t7;
    real_type t52  = ModelPars[46] * phi__f__XO;
    real_type t54  = 1.0 / t24;
    real_type t59  = (-t13 * t44 * phi__f__XO * ModelPars[40] * Fzf__XO + alpha__f__XO) * t40 / t32 * Fzf__XO * t24 - t13 * t44 * t32 * t54 * t39 * t52;
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
    real_type t80  = ModelPars[112];
    real_type t81  = ModelPars[104];
    real_type t83  = ModelPars[106] * ModelPars[106];
    real_type t84  = tan(alpha__f__XO);
    real_type t86  = t84 - ModelPars[108];
    real_type t87  = t86 * t86;
    real_type t89  = t87 * t83 + 1;
    real_type t90  = sqrt(t89);
    real_type t92  = 1.0 / t90 * t81;
    real_type t93  = ModelPars[116];
    real_type t97  = 1.0 / t2 * t5 * ModelPars[118];
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
  Straight::Fyf_D_3_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[7];
    real_type t5   = Fzf__XO - t2;
    real_type t7   = t2 * ModelPars[36] + t5 * ModelPars[38];
    real_type t8   = phi__f__XO * phi__f__XO;
    real_type t11  = ModelPars[44] * t8 + 1;
    real_type t13  = 1.0 / t11 * t7;
    real_type t14  = t7 * t7;
    real_type t15  = t11 * t11;
    real_type t18  = ModelPars[50];
    real_type t19  = t18 * t18;
    real_type t22  = ModelPars[42];
    real_type t23  = t22 * t22;
    real_type t25  = Fzf__XO * Fzf__XO;
    real_type t30  = ModelPars[48] * t8 + 1;
    real_type t31  = t30 * t30;
    real_type t37  = sqrt(1.0 / t31 * t25 * t23);
    real_type t42  = 1.0 / t7;
    real_type t50  = ModelPars[46] * phi__f__XO;
    real_type t52  = 1.0 / t22;
    real_type t57  = (-t11 * t42 * phi__f__XO * ModelPars[40] * Fzf__XO + alpha__f__XO) / t37 / t30 * Fzf__XO * t22 - t11 * t42 * t30 * t52 * t37 * t50;
    real_type t58  = t57 * t57;
    real_type t71  = atan(t57 * t30 / Fzf__XO * t52 / t18 * t13);
    real_type t72  = t71 * t18;
    real_type t73  = cos(t72);
    real_type t75  = ModelPars[112];
    real_type t78  = ModelPars[104];
    real_type t80  = ModelPars[106] * ModelPars[106];
    real_type t81  = tan(alpha__f__XO);
    real_type t83  = t81 - ModelPars[108];
    real_type t84  = t83 * t83;
    real_type t86  = t84 * t80 + 1;
    real_type t87  = sqrt(t86);
    real_type t89  = 1.0 / t87 * t78;
    real_type t90  = t78 * t78;
    real_type t92  = 1.0 / t86 * t90;
    real_type t93  = ModelPars[116];
    real_type t97  = 1.0 / t2 * t5 * ModelPars[118];
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
  Straight::Fyf_D_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[42];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t8   = ModelPars[48] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t12  = sqrt(1.0 / t9 * t3 * t2);
    real_type t13  = ModelPars[50];
    real_type t15  = ModelPars[7];
    real_type t18  = Fzf__XO - t15;
    real_type t20  = t15 * ModelPars[36] + t18 * ModelPars[38];
    real_type t23  = ModelPars[44] * t5 + 1;
    real_type t28  = 1.0 / t1;
    real_type t37  = 1.0 / t20;
    real_type t45  = ModelPars[46] * phi__f__XO;
    real_type t55  = atan(((-t23 * t37 * phi__f__XO * ModelPars[40] * Fzf__XO + alpha__f__XO) / t12 / t8 * Fzf__XO * t1 - t23 * t37 * t8 * t28 * t12 * t45) * t8 / Fzf__XO * t28 / t13 / t23 * t20);
    real_type t57  = sin(t55 * t13);
    real_type t63  = ModelPars[112];
    real_type t65  = ModelPars[104];
    real_type t68  = ModelPars[106] * ModelPars[106];
    real_type t69  = tan(alpha__f__XO);
    real_type t72  = pow(t69 - ModelPars[108], 2);
    real_type t74  = t72 * t68 + 1;
    real_type t75  = sqrt(t74);
    real_type t76  = 1.0 / t75;
    real_type t77  = t65 * t65;
    real_type t80  = ModelPars[116];
    real_type t84  = 1.0 / t15 * t18 * ModelPars[118];
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
  Straight::Fyf_D_4_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[42];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t8   = ModelPars[48] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t12  = sqrt(1.0 / t9 * t3 * t2);
    real_type t13  = ModelPars[50];
    real_type t15  = ModelPars[7];
    real_type t18  = Fzf__XO - t15;
    real_type t20  = t15 * ModelPars[36] + t18 * ModelPars[38];
    real_type t23  = ModelPars[44] * t5 + 1;
    real_type t28  = 1.0 / t1;
    real_type t37  = 1.0 / t20;
    real_type t45  = ModelPars[46] * phi__f__XO;
    real_type t55  = atan(((-t23 * t37 * phi__f__XO * ModelPars[40] * Fzf__XO + alpha__f__XO) / t12 / t8 * Fzf__XO * t1 - t23 * t37 * t8 * t28 * t12 * t45) * t8 / Fzf__XO * t28 / t13 / t23 * t20);
    real_type t57  = sin(t55 * t13);
    real_type t62  = t8 * t28 * t12 * t45 + t57 * t12;
    real_type t63  = ModelPars[112];
    real_type t65  = ModelPars[104];
    real_type t66  = t65 * t65;
    real_type t69  = ModelPars[106] * ModelPars[106];
    real_type t70  = tan(alpha__f__XO);
    real_type t73  = pow(t70 - ModelPars[108], 2);
    real_type t75  = t73 * t69 + 1;
    real_type t76  = sqrt(t75);
    real_type t81  = 1.0 / t75;
    real_type t83  = ModelPars[116];
    real_type t87  = 1.0 / t15 * t18 * ModelPars[118];
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
  Straight::Fyr( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[43];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t8   = ModelPars[49] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t12  = sqrt(1.0 / t9 * t3 * t2);
    real_type t13  = ModelPars[50];
    real_type t15  = ModelPars[8];
    real_type t18  = Fzr__XO - t15;
    real_type t20  = t15 * ModelPars[37] + t18 * ModelPars[39];
    real_type t23  = ModelPars[45] * t5 + 1;
    real_type t28  = 1.0 / t1;
    real_type t37  = 1.0 / t20;
    real_type t45  = ModelPars[47] * phi__XO;
    real_type t55  = atan(((-t23 * t37 * phi__XO * ModelPars[41] * Fzr__XO + alpha__r__XO) / t12 / t8 * Fzr__XO * t1 - t23 * t37 * t8 * t28 * t12 * t45) * t8 / Fzr__XO * t28 / t13 / t23 * t20);
    real_type t57  = sin(t55 * t13);
    real_type t63  = ModelPars[113];
    real_type t66  = ModelPars[107] * ModelPars[107];
    real_type t67  = tan(alpha__r__XO);
    real_type t70  = pow(t67 - ModelPars[109], 2);
    real_type t73  = sqrt(t70 * t66 + 1);
    real_type t75  = 1.0 / t73 * ModelPars[105];
    real_type t76  = ModelPars[117];
    real_type t80  = 1.0 / t15 * t18 * ModelPars[119];
    real_type t83  = atan((lambda__r__XO + t76 + t80) * t75);
    real_type t85  = cos(t83 * t63);
    real_type t89  = atan((t76 + t80) * t75);
    real_type t91  = cos(t89 * t63);
    return 1.0 / t91 * t85 * (t8 * t28 * t12 * t45 + t57 * t12);
  }

  real_type
  Straight::Fyr_D_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[43];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t8   = ModelPars[49] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[50];
    real_type t16  = ModelPars[8];
    real_type t18  = ModelPars[39];
    real_type t19  = Fzr__XO - t16;
    real_type t21  = t16 * ModelPars[37] + t19 * t18;
    real_type t24  = ModelPars[45] * t5 + 1;
    real_type t25  = 1.0 / t24;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t25 * t21;
    real_type t29  = 1.0 / t1;
    real_type t31  = 1.0 / Fzr__XO * t29;
    real_type t32  = Fzr__XO * t1;
    real_type t33  = 1.0 / t8;
    real_type t34  = t13 * t33;
    real_type t35  = ModelPars[41];
    real_type t36  = Fzr__XO * t35;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__XO * t36 + alpha__r__XO;
    real_type t45  = ModelPars[47] * phi__XO;
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
    real_type t132 = ModelPars[113];
    real_type t133 = ModelPars[105];
    real_type t135 = ModelPars[107] * ModelPars[107];
    real_type t136 = tan(alpha__r__XO);
    real_type t139 = pow(t136 - ModelPars[109], 2);
    real_type t141 = t139 * t135 + 1;
    real_type t142 = sqrt(t141);
    real_type t143 = 1.0 / t142;
    real_type t144 = t143 * t133;
    real_type t145 = ModelPars[117];
    real_type t146 = ModelPars[119];
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
  Straight::Fyr_D_1_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[43];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t8   = ModelPars[49] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t14  = 1.0 / t12 / t11;
    real_type t15  = ModelPars[50];
    real_type t17  = ModelPars[8];
    real_type t19  = ModelPars[39];
    real_type t20  = Fzr__XO - t17;
    real_type t22  = t17 * ModelPars[37] + t20 * t19;
    real_type t25  = ModelPars[45] * t5 + 1;
    real_type t26  = 1.0 / t25;
    real_type t28  = 1.0 / t15;
    real_type t29  = t28 * t26 * t22;
    real_type t30  = 1.0 / t1;
    real_type t32  = 1.0 / Fzr__XO * t30;
    real_type t33  = Fzr__XO * t1;
    real_type t34  = 1.0 / t8;
    real_type t35  = 1.0 / t12;
    real_type t36  = t35 * t34;
    real_type t37  = ModelPars[41];
    real_type t38  = Fzr__XO * t37;
    real_type t39  = 1.0 / t22;
    real_type t43  = -t25 * t39 * phi__XO * t38 + alpha__r__XO;
    real_type t47  = ModelPars[47] * phi__XO;
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
    real_type t247 = ModelPars[113];
    real_type t248 = ModelPars[105];
    real_type t250 = ModelPars[107] * ModelPars[107];
    real_type t251 = tan(alpha__r__XO);
    real_type t254 = pow(t251 - ModelPars[109], 2);
    real_type t256 = t254 * t250 + 1;
    real_type t257 = sqrt(t256);
    real_type t258 = 1.0 / t257;
    real_type t259 = t258 * t248;
    real_type t260 = ModelPars[117];
    real_type t261 = ModelPars[119];
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
    return t276 * t269 * (-t64 * t3 * t61 * t59 * t14 + 2 * t10 * Fzr__XO * t2 * t129 * t127 * t15 * t35 + t10 * t2 * t135 + t129 * t126 * (-2 * t73 * t69 + 2 * t110 * t69 + 2 * t54 * t144 * t30 * t29 - 2 * t109 * t72 * t29 + (-3 * Fzr__XO * t43 * t14 * t152 + 2 * t93 * t35 * t75 + 3 * t43 / t12 / t64 / t164 / t63 / t8 * t143 * t1 - 2 * t93 * t82 * t79 + (-2 * t180 * t25 * t178 * t87 + 2 * t91 * t85) * t36 * t33 + t3 * t25 * t39 * t81 * t78 * t14 * t47 + 2 * t19 * t99 * t89 * t34 * t97 - t50 * t75 * t198 - 2 * t180 * t25 * t178 * t8 * t103) * t8 * t32 * t29) * t138 - (2 * t19 * t121 * t221 * t219 * t114 * t22 + 2 * t108 * t53 * t221 * t219 * t115 - 2 * t122 * t144 * t119 * t118) * t129 * t216 * t112 * t138 - t59 * t216 * t237 * t116 * t12 - t3 * t152 * t14 * t47 + t34 * t96 * t47) - 2 * t276 * t296 * t295 * t263 * t261 * t259 * t247 * t285 + 2 * t314 * t312 * t263 * t261 * t258 * t248 * t305 * t269 * t285 + 2 * t265 * t276 * t296 * t334 * t332 * t328 * t326 * t324 * t247 * t322 - t276 * t269 * t334 * t344 * t291 * t342 - 2 * t314 * t312 * t304 * t296 * t295 * t332 * t328 * t291 * t342 + 2 * t368 * t366 * t332 * t328 * t290 * t289 * t341 / t303 / t275 * t358 - 2 * t271 * t314 * t366 * t332 * t328 * t326 * t324 * t305 * t358 + t366 * t344 * t291 * t341 * t276 * t358;
  }

  real_type
  Straight::Fyr_D_1_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[43];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t4   = t3 * t2;
    real_type t5   = phi__XO * phi__XO;
    real_type t6   = ModelPars[49];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t4;
    real_type t12  = sqrt(t11);
    real_type t14  = 1.0 / t12 / t11;
    real_type t15  = ModelPars[50];
    real_type t17  = ModelPars[8];
    real_type t19  = ModelPars[39];
    real_type t20  = Fzr__XO - t17;
    real_type t22  = t17 * ModelPars[37] + t20 * t19;
    real_type t23  = ModelPars[45];
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
    real_type t37  = ModelPars[41];
    real_type t38  = Fzr__XO * t37;
    real_type t39  = 1.0 / t22;
    real_type t43  = -t25 * t39 * phi__XO * t38 + alpha__r__XO;
    real_type t46  = ModelPars[47];
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
    real_type t383 = ModelPars[113];
    real_type t384 = ModelPars[105];
    real_type t386 = ModelPars[107] * ModelPars[107];
    real_type t387 = tan(alpha__r__XO);
    real_type t390 = pow(t387 - ModelPars[109], 2);
    real_type t392 = t390 * t386 + 1;
    real_type t393 = sqrt(t392);
    real_type t394 = 1.0 / t393;
    real_type t395 = t394 * t384;
    real_type t396 = ModelPars[117];
    real_type t397 = ModelPars[119];
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
  Straight::Fyr_D_1_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = 1.0 / Fzr__XO;
    real_type t3   = ModelPars[8];
    real_type t5   = ModelPars[39];
    real_type t6   = Fzr__XO - t3;
    real_type t8   = t3 * ModelPars[37] + t6 * t5;
    real_type t10  = phi__XO * phi__XO;
    real_type t13  = ModelPars[45] * t10 + 1;
    real_type t14  = 1.0 / t13;
    real_type t15  = t8 * t8;
    real_type t16  = t13 * t13;
    real_type t17  = 1.0 / t16;
    real_type t19  = ModelPars[50];
    real_type t20  = t19 * t19;
    real_type t23  = ModelPars[43];
    real_type t24  = t23 * t23;
    real_type t26  = Fzr__XO * Fzr__XO;
    real_type t27  = 1.0 / t26;
    real_type t31  = ModelPars[49] * t10 + 1;
    real_type t32  = t31 * t31;
    real_type t33  = Fzr__XO * t23;
    real_type t34  = 1.0 / t31;
    real_type t36  = 1.0 / t32;
    real_type t37  = t36 * t26 * t24;
    real_type t38  = sqrt(t37);
    real_type t39  = 1.0 / t38;
    real_type t40  = t39 * t34;
    real_type t41  = ModelPars[41];
    real_type t42  = Fzr__XO * t41;
    real_type t43  = 1.0 / t8;
    real_type t47  = -t13 * t43 * phi__XO * t42 + alpha__r__XO;
    real_type t51  = ModelPars[47] * phi__XO;
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
    real_type t157 = ModelPars[113];
    real_type t158 = ModelPars[105];
    real_type t160 = ModelPars[107] * ModelPars[107];
    real_type t161 = tan(alpha__r__XO);
    real_type t163 = t161 - ModelPars[109];
    real_type t164 = t163 * t163;
    real_type t166 = t164 * t160 + 1;
    real_type t167 = sqrt(t166);
    real_type t168 = 1.0 / t167;
    real_type t169 = t168 * t158;
    real_type t170 = ModelPars[117];
    real_type t171 = ModelPars[119];
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
  Straight::Fyr_D_1_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[43];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t8   = ModelPars[49] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[50];
    real_type t16  = ModelPars[8];
    real_type t18  = ModelPars[39];
    real_type t19  = Fzr__XO - t16;
    real_type t21  = t16 * ModelPars[37] + t19 * t18;
    real_type t24  = ModelPars[45] * t5 + 1;
    real_type t25  = 1.0 / t24;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t25 * t21;
    real_type t29  = 1.0 / t1;
    real_type t31  = 1.0 / Fzr__XO * t29;
    real_type t32  = Fzr__XO * t1;
    real_type t33  = 1.0 / t8;
    real_type t34  = t13 * t33;
    real_type t35  = ModelPars[41];
    real_type t36  = Fzr__XO * t35;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__XO * t36 + alpha__r__XO;
    real_type t45  = ModelPars[47] * phi__XO;
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
    real_type t132 = ModelPars[113];
    real_type t134 = ModelPars[105];
    real_type t137 = ModelPars[107] * ModelPars[107];
    real_type t138 = tan(alpha__r__XO);
    real_type t141 = pow(t138 - ModelPars[109], 2);
    real_type t143 = t141 * t137 + 1;
    real_type t144 = sqrt(t143);
    real_type t145 = 1.0 / t144;
    real_type t146 = t134 * t134;
    real_type t148 = 1.0 / t143 * t146;
    real_type t149 = ModelPars[117];
    real_type t150 = ModelPars[119];
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
  Straight::Fyr_D_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[43];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t6   = ModelPars[49];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[50];
    real_type t16  = ModelPars[8];
    real_type t19  = Fzr__XO - t16;
    real_type t21  = t16 * ModelPars[37] + t19 * ModelPars[39];
    real_type t22  = ModelPars[45];
    real_type t24  = t22 * t5 + 1;
    real_type t26  = 1.0 / t24 * t21;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t26;
    real_type t29  = 1.0 / t1;
    real_type t30  = 1.0 / Fzr__XO;
    real_type t31  = t30 * t29;
    real_type t32  = Fzr__XO * t1;
    real_type t34  = t13 / t8;
    real_type t36  = ModelPars[41] * Fzr__XO;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__XO * t36 + alpha__r__XO;
    real_type t44  = ModelPars[47];
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
    real_type t168 = ModelPars[113];
    real_type t171 = ModelPars[107] * ModelPars[107];
    real_type t172 = tan(alpha__r__XO);
    real_type t175 = pow(t172 - ModelPars[109], 2);
    real_type t178 = sqrt(t175 * t171 + 1);
    real_type t180 = 1.0 / t178 * ModelPars[105];
    real_type t181 = ModelPars[117];
    real_type t185 = 1.0 / t16 * t19 * ModelPars[119];
    real_type t188 = atan((lambda__r__XO + t181 + t185) * t180);
    real_type t190 = cos(t188 * t168);
    real_type t194 = atan((t181 + t185) * t180);
    real_type t196 = cos(t194 * t168);
    return 1.0 / t196 * t190 * (-2 * t63 / t9 / t8 * t3 * t2 * t57 * t13 + t153 / (t146 * t9 / t3 / t2 / t140 * t69 * t138 + 1) * (-2 * phi__XO * t22 * t51 * t8 * t30 * t71 * t69 * t21 + 2 * t51 * phi__XO * t6 * t30 * t71 * t26 + (-2 * t63 * t41 * t13 * t10 * t32 + 2 * t63 * t41 / t12 / t11 / t93 * t3 * Fzr__XO * t2 * t1 + (-2 * t22 * t37 * t5 * t36 - t48 * t36) * t34 * t32 - t24 * t37 * t8 * t29 * t110 + 2 * t6 * t3 * t24 * t37 * t10 * t1 * t13 * t115 - 2 * t48 * t6 * t29 * t124 - 2 * t22 * t37 * t47 * t124) * t8 * t31 * t28) * t14 * t12 + t47 * t110 - 2 * t6 * t3 * t10 * t1 * t13 * t115 + 2 * t6 * t29 * t12 * t115);
  }

  real_type
  Straight::Fyr_D_2_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[43];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t4   = t3 * t2;
    real_type t5   = phi__XO * phi__XO;
    real_type t6   = ModelPars[49];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t4;
    real_type t12  = sqrt(t11);
    real_type t14  = 1.0 / t12 / t11;
    real_type t15  = ModelPars[50];
    real_type t17  = ModelPars[8];
    real_type t20  = Fzr__XO - t17;
    real_type t22  = t17 * ModelPars[37] + t20 * ModelPars[39];
    real_type t23  = ModelPars[45];
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
    real_type t37  = ModelPars[41];
    real_type t38  = Fzr__XO * t37;
    real_type t39  = 1.0 / t22;
    real_type t43  = -t25 * t39 * phi__XO * t38 + alpha__r__XO;
    real_type t46  = ModelPars[47];
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
    real_type t354 = ModelPars[113];
    real_type t357 = ModelPars[107] * ModelPars[107];
    real_type t358 = tan(alpha__r__XO);
    real_type t361 = pow(t358 - ModelPars[109], 2);
    real_type t364 = sqrt(t361 * t357 + 1);
    real_type t366 = 1.0 / t364 * ModelPars[105];
    real_type t367 = ModelPars[117];
    real_type t371 = 1.0 / t17 * t20 * ModelPars[119];
    real_type t374 = atan((lambda__r__XO + t367 + t371) * t366);
    real_type t376 = cos(t374 * t354);
    real_type t380 = atan((t367 + t371) * t366);
    real_type t382 = cos(t380 * t354);
    return 1.0 / t382 * t376 * t353;
  }

  real_type
  Straight::Fyr_D_2_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = phi__XO * phi__XO;
    real_type t2   = ModelPars[49];
    real_type t4   = t2 * t1 + 1;
    real_type t5   = 1.0 / t4;
    real_type t7   = ModelPars[8];
    real_type t10  = Fzr__XO - t7;
    real_type t12  = t10 * ModelPars[39] + t7 * ModelPars[37];
    real_type t14  = ModelPars[45];
    real_type t16  = t14 * t1 + 1;
    real_type t17  = 1.0 / t16;
    real_type t19  = t12 * t12;
    real_type t20  = t16 * t16;
    real_type t21  = 1.0 / t20;
    real_type t23  = ModelPars[50];
    real_type t24  = t23 * t23;
    real_type t27  = ModelPars[43];
    real_type t28  = t27 * t27;
    real_type t30  = Fzr__XO * Fzr__XO;
    real_type t33  = t4 * t4;
    real_type t34  = Fzr__XO * t27;
    real_type t36  = 1.0 / t33;
    real_type t37  = t36 * t30 * t28;
    real_type t38  = sqrt(t37);
    real_type t39  = 1.0 / t38;
    real_type t40  = t39 * t5;
    real_type t42  = ModelPars[41] * Fzr__XO;
    real_type t43  = 1.0 / t12;
    real_type t47  = -t16 * t43 * phi__XO * t42 + alpha__r__XO;
    real_type t50  = ModelPars[47];
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
    real_type t190 = ModelPars[113];
    real_type t191 = ModelPars[105];
    real_type t193 = ModelPars[107] * ModelPars[107];
    real_type t194 = tan(alpha__r__XO);
    real_type t196 = t194 - ModelPars[109];
    real_type t197 = t196 * t196;
    real_type t199 = t197 * t193 + 1;
    real_type t200 = sqrt(t199);
    real_type t202 = 1.0 / t200 * t191;
    real_type t203 = ModelPars[117];
    real_type t207 = 1.0 / t7 * t10 * ModelPars[119];
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
  Straight::Fyr_D_2_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[43];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t6   = ModelPars[49];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[50];
    real_type t16  = ModelPars[8];
    real_type t19  = Fzr__XO - t16;
    real_type t21  = t16 * ModelPars[37] + t19 * ModelPars[39];
    real_type t22  = ModelPars[45];
    real_type t24  = t22 * t5 + 1;
    real_type t26  = 1.0 / t24 * t21;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t26;
    real_type t29  = 1.0 / t1;
    real_type t30  = 1.0 / Fzr__XO;
    real_type t31  = t30 * t29;
    real_type t32  = Fzr__XO * t1;
    real_type t34  = t13 / t8;
    real_type t36  = ModelPars[41] * Fzr__XO;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__XO * t36 + alpha__r__XO;
    real_type t44  = ModelPars[47];
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
    real_type t168 = ModelPars[113];
    real_type t170 = ModelPars[105];
    real_type t173 = ModelPars[107] * ModelPars[107];
    real_type t174 = tan(alpha__r__XO);
    real_type t177 = pow(t174 - ModelPars[109], 2);
    real_type t179 = t177 * t173 + 1;
    real_type t180 = sqrt(t179);
    real_type t181 = 1.0 / t180;
    real_type t182 = t170 * t170;
    real_type t185 = ModelPars[117];
    real_type t189 = 1.0 / t16 * t19 * ModelPars[119];
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
  Straight::Fyr_D_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[8];
    real_type t5   = Fzr__XO - t2;
    real_type t7   = t2 * ModelPars[37] + t5 * ModelPars[39];
    real_type t8   = phi__XO * phi__XO;
    real_type t11  = ModelPars[45] * t8 + 1;
    real_type t13  = 1.0 / t11 * t7;
    real_type t14  = t7 * t7;
    real_type t15  = t11 * t11;
    real_type t18  = ModelPars[50];
    real_type t19  = t18 * t18;
    real_type t22  = ModelPars[43];
    real_type t23  = t22 * t22;
    real_type t25  = Fzr__XO * Fzr__XO;
    real_type t30  = ModelPars[49] * t8 + 1;
    real_type t31  = t30 * t30;
    real_type t37  = sqrt(1.0 / t31 * t25 * t23);
    real_type t42  = 1.0 / t7;
    real_type t50  = ModelPars[47] * phi__XO;
    real_type t52  = 1.0 / t22;
    real_type t57  = (-t11 * t42 * phi__XO * ModelPars[41] * Fzr__XO + alpha__r__XO) / t37 / t30 * Fzr__XO * t22 - t11 * t42 * t30 * t52 * t37 * t50;
    real_type t58  = t57 * t57;
    real_type t72  = atan(t57 * t30 / Fzr__XO * t52 / t18 * t13);
    real_type t73  = t72 * t18;
    real_type t74  = cos(t73);
    real_type t75  = ModelPars[113];
    real_type t76  = ModelPars[105];
    real_type t78  = ModelPars[107] * ModelPars[107];
    real_type t79  = tan(alpha__r__XO);
    real_type t81  = t79 - ModelPars[109];
    real_type t82  = t81 * t81;
    real_type t84  = t82 * t78 + 1;
    real_type t85  = sqrt(t84);
    real_type t87  = 1.0 / t85 * t76;
    real_type t88  = ModelPars[117];
    real_type t92  = 1.0 / t2 * t5 * ModelPars[119];
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
  Straight::Fyr_D_3_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[8];
    real_type t5   = Fzr__XO - t2;
    real_type t7   = t2 * ModelPars[37] + t5 * ModelPars[39];
    real_type t8   = t7 * t7;
    real_type t10  = phi__XO * phi__XO;
    real_type t13  = ModelPars[45] * t10 + 1;
    real_type t14  = t13 * t13;
    real_type t19  = 1.0 / t14 * t8;
    real_type t20  = ModelPars[50];
    real_type t21  = t20 * t20;
    real_type t22  = 1.0 / t21;
    real_type t24  = ModelPars[43];
    real_type t25  = t24 * t24;
    real_type t27  = Fzr__XO * Fzr__XO;
    real_type t32  = ModelPars[49] * t10 + 1;
    real_type t33  = t32 * t32;
    real_type t39  = sqrt(1.0 / t33 * t27 * t25);
    real_type t40  = 1.0 / t39;
    real_type t44  = 1.0 / t7;
    real_type t52  = ModelPars[47] * phi__XO;
    real_type t54  = 1.0 / t24;
    real_type t59  = (-t13 * t44 * phi__XO * ModelPars[41] * Fzr__XO + alpha__r__XO) * t40 / t32 * Fzr__XO * t24 - t13 * t44 * t32 * t54 * t39 * t52;
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
    real_type t80  = ModelPars[113];
    real_type t81  = ModelPars[105];
    real_type t83  = ModelPars[107] * ModelPars[107];
    real_type t84  = tan(alpha__r__XO);
    real_type t86  = t84 - ModelPars[109];
    real_type t87  = t86 * t86;
    real_type t89  = t87 * t83 + 1;
    real_type t90  = sqrt(t89);
    real_type t92  = 1.0 / t90 * t81;
    real_type t93  = ModelPars[117];
    real_type t97  = 1.0 / t2 * t5 * ModelPars[119];
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
  Straight::Fyr_D_3_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[8];
    real_type t5   = Fzr__XO - t2;
    real_type t7   = t2 * ModelPars[37] + t5 * ModelPars[39];
    real_type t8   = phi__XO * phi__XO;
    real_type t11  = ModelPars[45] * t8 + 1;
    real_type t13  = 1.0 / t11 * t7;
    real_type t14  = t7 * t7;
    real_type t15  = t11 * t11;
    real_type t18  = ModelPars[50];
    real_type t19  = t18 * t18;
    real_type t22  = ModelPars[43];
    real_type t23  = t22 * t22;
    real_type t25  = Fzr__XO * Fzr__XO;
    real_type t30  = ModelPars[49] * t8 + 1;
    real_type t31  = t30 * t30;
    real_type t37  = sqrt(1.0 / t31 * t25 * t23);
    real_type t42  = 1.0 / t7;
    real_type t50  = ModelPars[47] * phi__XO;
    real_type t52  = 1.0 / t22;
    real_type t57  = (-t11 * t42 * phi__XO * ModelPars[41] * Fzr__XO + alpha__r__XO) / t37 / t30 * Fzr__XO * t22 - t11 * t42 * t30 * t52 * t37 * t50;
    real_type t58  = t57 * t57;
    real_type t71  = atan(t57 * t30 / Fzr__XO * t52 / t18 * t13);
    real_type t72  = t71 * t18;
    real_type t73  = cos(t72);
    real_type t75  = ModelPars[113];
    real_type t78  = ModelPars[105];
    real_type t80  = ModelPars[107] * ModelPars[107];
    real_type t81  = tan(alpha__r__XO);
    real_type t83  = t81 - ModelPars[109];
    real_type t84  = t83 * t83;
    real_type t86  = t84 * t80 + 1;
    real_type t87  = sqrt(t86);
    real_type t89  = 1.0 / t87 * t78;
    real_type t90  = t78 * t78;
    real_type t92  = 1.0 / t86 * t90;
    real_type t93  = ModelPars[117];
    real_type t97  = 1.0 / t2 * t5 * ModelPars[119];
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
  Straight::Fyr_D_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[43];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t8   = ModelPars[49] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t12  = sqrt(1.0 / t9 * t3 * t2);
    real_type t13  = ModelPars[50];
    real_type t15  = ModelPars[8];
    real_type t18  = Fzr__XO - t15;
    real_type t20  = t15 * ModelPars[37] + t18 * ModelPars[39];
    real_type t23  = ModelPars[45] * t5 + 1;
    real_type t28  = 1.0 / t1;
    real_type t37  = 1.0 / t20;
    real_type t45  = ModelPars[47] * phi__XO;
    real_type t55  = atan(((-t23 * t37 * phi__XO * ModelPars[41] * Fzr__XO + alpha__r__XO) / t12 / t8 * Fzr__XO * t1 - t23 * t37 * t8 * t28 * t12 * t45) * t8 / Fzr__XO * t28 / t13 / t23 * t20);
    real_type t57  = sin(t55 * t13);
    real_type t63  = ModelPars[113];
    real_type t65  = ModelPars[105];
    real_type t68  = ModelPars[107] * ModelPars[107];
    real_type t69  = tan(alpha__r__XO);
    real_type t72  = pow(t69 - ModelPars[109], 2);
    real_type t74  = t72 * t68 + 1;
    real_type t75  = sqrt(t74);
    real_type t76  = 1.0 / t75;
    real_type t77  = t65 * t65;
    real_type t80  = ModelPars[117];
    real_type t84  = 1.0 / t15 * t18 * ModelPars[119];
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
  Straight::Fyr_D_4_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[43];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t8   = ModelPars[49] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t12  = sqrt(1.0 / t9 * t3 * t2);
    real_type t13  = ModelPars[50];
    real_type t15  = ModelPars[8];
    real_type t18  = Fzr__XO - t15;
    real_type t20  = t15 * ModelPars[37] + t18 * ModelPars[39];
    real_type t23  = ModelPars[45] * t5 + 1;
    real_type t28  = 1.0 / t1;
    real_type t37  = 1.0 / t20;
    real_type t45  = ModelPars[47] * phi__XO;
    real_type t55  = atan(((-t23 * t37 * phi__XO * ModelPars[41] * Fzr__XO + alpha__r__XO) / t12 / t8 * Fzr__XO * t1 - t23 * t37 * t8 * t28 * t12 * t45) * t8 / Fzr__XO * t28 / t13 / t23 * t20);
    real_type t57  = sin(t55 * t13);
    real_type t62  = t8 * t28 * t12 * t45 + t57 * t12;
    real_type t63  = ModelPars[113];
    real_type t65  = ModelPars[105];
    real_type t66  = t65 * t65;
    real_type t69  = ModelPars[107] * ModelPars[107];
    real_type t70  = tan(alpha__r__XO);
    real_type t73  = pow(t70 - ModelPars[109], 2);
    real_type t75  = t73 * t69 + 1;
    real_type t76  = sqrt(t75);
    real_type t81  = 1.0 / t75;
    real_type t83  = ModelPars[117];
    real_type t87  = 1.0 / t15 * t18 * ModelPars[119];
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
  Straight::Mzf( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t4   = ModelPars[7];
    real_type t9   = t4 * ModelPars[36] + (Fzf__XO - t4) * ModelPars[38];
    real_type t13  = phi__f__XO * phi__f__XO;
    real_type t17  = 1.0 / (ModelPars[57] * t13 + 1);
    real_type t18  = t17 * ModelPars[51];
    real_type t20  = ModelPars[42];
    real_type t25  = ModelPars[48] * t13 + 1;
    real_type t26  = 1.0 / t25;
    real_type t27  = t20 * t20;
    real_type t28  = Fzf__XO * Fzf__XO;
    real_type t30  = t25 * t25;
    real_type t33  = sqrt(1.0 / t30 * t28 * t27);
    real_type t34  = 1.0 / t33;
    real_type t38  = atan(alpha__f__XO * t34 * t26 * Fzf__XO * t20 * ModelPars[59]);
    real_type t40  = cos(t38 * t18);
    real_type t42  = ModelPars[50];
    real_type t52  = atan(alpha__f__XO * t34 / t42 / (ModelPars[44] * t13 + 1) * t9);
    real_type t54  = sin(t52 * t42);
    real_type t60  = ModelPars[58];
    real_type t62  = atan(phi__f__XO * t60);
    real_type t76  = atan(alpha__f__XO * t34 * t26 * Fzf__XO * t20 / (ModelPars[56] * t13 + 1) * ModelPars[60]);
    real_type t78  = cos(t76 * t18);
    return -t54 * t33 * t17 * t40 / t9 * Fzf__XO * ModelPars[52] + t78 / t60 * t62 * ModelPars[54] * Fzf__XO;
  }

  real_type
  Straight::Mzf_D_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[52];
    real_type t3   = ModelPars[7];
    real_type t5   = ModelPars[38];
    real_type t8   = t3 * ModelPars[36] + (Fzf__XO - t3) * t5;
    real_type t9   = 1.0 / t8;
    real_type t11  = ModelPars[51];
    real_type t12  = phi__f__XO * phi__f__XO;
    real_type t15  = ModelPars[57] * t12 + 1;
    real_type t16  = 1.0 / t15;
    real_type t17  = t16 * t11;
    real_type t18  = ModelPars[59];
    real_type t19  = ModelPars[42];
    real_type t20  = t19 * t18;
    real_type t24  = ModelPars[48] * t12 + 1;
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
    real_type t42  = ModelPars[50];
    real_type t45  = ModelPars[44] * t12 + 1;
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
    real_type t132 = ModelPars[54];
    real_type t133 = ModelPars[58];
    real_type t135 = atan(phi__f__XO * t133);
    real_type t137 = 1.0 / t133;
    real_type t138 = ModelPars[60];
    real_type t141 = ModelPars[56] * t12 + 1;
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
  Straight::Mzf_D_1_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[52];
    real_type t2   = Fzf__XO * t1;
    real_type t4   = ModelPars[7];
    real_type t6   = ModelPars[38];
    real_type t9   = t4 * ModelPars[36] + (Fzf__XO - t4) * t6;
    real_type t10  = 1.0 / t9;
    real_type t12  = ModelPars[51];
    real_type t13  = phi__f__XO * phi__f__XO;
    real_type t16  = ModelPars[57] * t13 + 1;
    real_type t17  = t16 * t16;
    real_type t18  = 1.0 / t17;
    real_type t19  = t18 * t12;
    real_type t20  = ModelPars[59];
    real_type t21  = ModelPars[42];
    real_type t22  = t21 * t20;
    real_type t25  = ModelPars[48] * t13 + 1;
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
    real_type t65  = ModelPars[50];
    real_type t68  = ModelPars[44] * t13 + 1;
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
    real_type t108 = ModelPars[54];
    real_type t110 = ModelPars[58];
    real_type t112 = atan(phi__f__XO * t110);
    real_type t113 = 1.0 / t110;
    real_type t115 = t113 * t112 * Fzf__XO * t108;
    real_type t116 = t12 * t12;
    real_type t118 = ModelPars[60];
    real_type t121 = ModelPars[56] * t13 + 1;
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
    real_type t155 = t27 * t27;
    real_type t156 = t155 * t21;
    real_type t158 = t28 * Fzf__XO;
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
    real_type t238 = t56 * t215 * t2;
    real_type t259 = t82 * t82;
    real_type t260 = t97 * t97;
    real_type t261 = 1.0 / t260;
    real_type t266 = t28 * t1;
    real_type t268 = t56 * t189 * t222;
    real_type t296 = t48 * t48;
    real_type t297 = t54 * t54;
    real_type t314 = t82 * t198;
    real_type t336 = t10 * t266;
    return 2 * t103 * t83 * t33 * t63 * t49 * t10 * t2 - t148 / t139 * t132 * t18 * t116 * t115 - t176 * t174 * (3 * alpha__f__XO * t168 * t161 * t158 * t156 * t123 - 3 * alpha__f__XO * Fzf__XO * t45 * t126) * t57 * t115 + 2 * t185 * t182 * t19 * t180 - 3 * t31 * Fzf__XO * t27 * t192 * t191 - 2 * t102 * t98 * t82 * t198 * t191 - 2 * t209 * t184 * t208 * t189 / t84 / t9 * t2 + t165 * t155 * t184 * t44 * t56 * t215 * t158 * t1 + 2 * t6 * t184 * t208 * t189 * t222 * t1 - 2 * t176 * t174 * t131 * t56 * t12 * t113 * t112 * t108 - t102 * t98 * (3 * t165 * t28 * t155 * alpha__f__XO * t168 * t71 * t75 - t31 * t27 * t128 * t71 * t75 - 2 * t80 * t76 * t70) * t198 * t238 + t184 * t261 * t259 * t88 * t33 * t238 + 2 * t31 * t27 * t6 * t192 * t268 * t266 + t185 * t55 * (3 * alpha__f__XO * t168 * t161 * t158 * t156 * t20 - 3 * Fzf__XO * t128 * t42 * t39) * t18 * t12 * t10 * t2 + t184 * t33 * t189 / t297 * t296 / t17 / t16 * t116 * t10 * t2 - 2 * t6 * t184 * t33 * t62 * t182 * t18 * t12 * t222 * t2 + 2 * t6 * t103 * t314 * t268 * t2 + (2 * t6 * t51 * t30 * t92 * t91 * t89 * t86 * t9 - 2 * t94 / t158 * t91 * t90) * t102 * t261 * t314 * t238 + 2 * t31 * t27 * t184 * t34 * t63 * t49 * t336 - 2 * t31 * t27 * t102 * t98 * t83 * t34 * t190 * t336;
  }

  real_type
  Straight::Mzf_D_1_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[54];
    real_type t2   = Fzf__XO * t1;
    real_type t3   = ModelPars[58];
    real_type t5   = atan(phi__f__XO * t3);
    real_type t6   = 1.0 / t3;
    real_type t7   = t6 * t5;
    real_type t8   = ModelPars[51];
    real_type t10  = t8 * t7 * t2;
    real_type t11  = phi__f__XO * phi__f__XO;
    real_type t12  = ModelPars[57];
    real_type t14  = t12 * t11 + 1;
    real_type t15  = 1.0 / t14;
    real_type t16  = ModelPars[60];
    real_type t17  = ModelPars[56];
    real_type t19  = t17 * t11 + 1;
    real_type t21  = 1.0 / t19 * t16;
    real_type t22  = ModelPars[42];
    real_type t23  = t22 * t21;
    real_type t24  = ModelPars[48];
    real_type t26  = t24 * t11 + 1;
    real_type t27  = 1.0 / t26;
    real_type t28  = t22 * t22;
    real_type t29  = Fzf__XO * Fzf__XO;
    real_type t30  = t29 * t28;
    real_type t31  = t26 * t26;
    real_type t32  = 1.0 / t31;
    real_type t33  = t32 * t30;
    real_type t34  = sqrt(t33);
    real_type t35  = 1.0 / t34;
    real_type t36  = t35 * t27;
    real_type t37  = alpha__f__XO * t36;
    real_type t39  = t28 * t22;
    real_type t42  = 1.0 / t31 / t26;
    real_type t45  = 1.0 / t34 / t33;
    real_type t46  = alpha__f__XO * t45;
    real_type t49  = -t46 * t42 * t29 * t39 * t21 + t37 * t23;
    real_type t50  = t49 * t15;
    real_type t51  = t16 * t16;
    real_type t52  = t19 * t19;
    real_type t53  = 1.0 / t52;
    real_type t55  = alpha__f__XO * alpha__f__XO;
    real_type t57  = t55 * t53 * t51 + 1;
    real_type t58  = 1.0 / t57;
    real_type t59  = t14 * t14;
    real_type t60  = 1.0 / t59;
    real_type t61  = t60 * t8;
    real_type t63  = alpha__f__XO * t35;
    real_type t66  = atan(t63 * t27 * Fzf__XO * t23);
    real_type t71  = t15 * t8;
    real_type t72  = t53 * t16;
    real_type t73  = Fzf__XO * t22;
    real_type t76  = phi__f__XO * t17 * alpha__f__XO;
    real_type t82  = phi__f__XO * t24 * alpha__f__XO;
    real_type t85  = t29 * Fzf__XO;
    real_type t88  = t31 * t31;
    real_type t89  = 1.0 / t88;
    real_type t90  = t45 * t89;
    real_type t97  = -2 * phi__f__XO * t12 * t66 * t61 + t58 * (-2 * t82 * t35 * t32 * t73 * t21 + 2 * t82 * t90 * t85 * t39 * t21 - 2 * t76 * t36 * t73 * t72) * t71;
    real_type t99  = t66 * t71;
    real_type t100 = cos(t99);
    real_type t104 = ModelPars[52];
    real_type t105 = Fzf__XO * t104;
    real_type t107 = ModelPars[7];
    real_type t109 = ModelPars[38];
    real_type t112 = t107 * ModelPars[36] + (Fzf__XO - t107) * t109;
    real_type t113 = 1.0 / t112;
    real_type t114 = ModelPars[59];
    real_type t115 = t22 * t114;
    real_type t118 = atan(t37 * Fzf__XO * t115);
    real_type t119 = t118 * t71;
    real_type t120 = cos(t119);
    real_type t121 = t120 * t113;
    real_type t122 = t15 * t121;
    real_type t123 = t122 * t105;
    real_type t124 = ModelPars[50];
    real_type t125 = t124 * t34;
    real_type t126 = ModelPars[44];
    real_type t128 = t126 * t11 + 1;
    real_type t129 = t128 * t128;
    real_type t130 = 1.0 / t129;
    real_type t132 = 1.0 / t124;
    real_type t135 = phi__f__XO * t126 * t63;
    real_type t138 = 1.0 / t128;
    real_type t139 = t138 * t109;
    real_type t140 = t45 * t132;
    real_type t141 = alpha__f__XO * t140;
    real_type t144 = phi__f__XO * t24 * t42;
    real_type t145 = t144 * t30;
    real_type t148 = t130 * t112;
    real_type t150 = t28 * Fzf__XO;
    real_type t156 = t138 * t112;
    real_type t157 = t28 * t28;
    real_type t158 = t29 * t29;
    real_type t162 = 1.0 / t34 / t89 / t158 / t157;
    real_type t170 = phi__f__XO * t24 / t88 / t26;
    real_type t174 = t141 * t156;
    real_type t179 = t112 * t112;
    real_type t180 = t130 * t179;
    real_type t181 = t124 * t124;
    real_type t182 = 1.0 / t181;
    real_type t184 = 1.0 / t28;
    real_type t185 = 1.0 / t29;
    real_type t190 = t55 * t31 * t185 * t184 * t182 * t180 + 1;
    real_type t191 = 1.0 / t190;
    real_type t194 = alpha__f__XO * t35 * t132;
    real_type t196 = atan(t194 * t156);
    real_type t197 = t196 * t124;
    real_type t198 = cos(t197);
    real_type t202 = 1.0 / t179;
    real_type t203 = t120 * t202;
    real_type t206 = sin(t197);
    real_type t207 = t206 * t34;
    real_type t215 = t60 * t8 * t113 * t105;
    real_type t217 = phi__f__XO * t24;
    real_type t218 = t217 * t63;
    real_type t221 = t39 * t114;
    real_type t227 = t157 * t22;
    real_type t230 = 1.0 / t88 / t31;
    real_type t238 = t114 * t114;
    real_type t241 = 1.0 / (t238 * t55 + 1);
    real_type t243 = sin(t119);
    real_type t248 = t29 * t104;
    real_type t253 = t32 * Fzf__XO;
    real_type t264 = -2 * phi__f__XO * t12 * t118 * t61 + t241 * (2 * t217 * t46 * t89 * t85 * t221 - 2 * t218 * t253 * t115) * t71;
    real_type t266 = t243 * t264 * t113;
    real_type t269 = t28 * t206;
    real_type t273 = t113 * t104;
    real_type t275 = t15 * t243;
    real_type t278 = t15 * t120;
    real_type t279 = t35 * t278;
    real_type t291 = -2 * t135 * t132 * t148 + 2 * t145 * t174;
    real_type t293 = t198 * t191;
    real_type t299 = t45 * t42;
    real_type t302 = -alpha__f__XO * t299 * t29 * t221 + t37 * t115;
    real_type t303 = t241 * t302;
    real_type t330 = -t253 * t28 * alpha__f__XO * t140 * t156 + t194 * t139;
    real_type t332 = t190 * t190;
    real_type t333 = 1.0 / t332;
    real_type t338 = t184 * t182;
    real_type t360 = t198 * t191 * t330;
    real_type t371 = sin(t99);
    real_type t377 = -t100 * t97 * t58 * t50 * t10 - t198 * t191 * (-6 * t170 * t85 * t157 * alpha__f__XO * t162 * t132 * t156 + 2 * phi__f__XO * t126 * t32 * t150 * t141 * t148 - 2 * t135 * t132 * t130 * t109 + 2 * t145 * t141 * t139 + 4 * t144 * t150 * t174) * t125 * t123 - 2 * phi__f__XO * t12 * t109 * t207 * t60 * t203 * t105 + t206 * t34 * t243 * t241 * (-6 * t217 * alpha__f__XO * t162 * t230 * t158 * t227 * t114 + 8 * t217 * t29 * alpha__f__XO * t90 * t221 - 2 * t218 * t32 * t115) * t215 + t32 * t269 * t35 * t15 * t266 * t248 + t207 * t275 * t264 * t273 + 6 * t144 * t29 * t269 * t279 * t273 + t109 * t293 * t291 * t125 * t15 * t203 * t105 + t206 * t34 * t120 * t264 * t303 * t215 + 2 * phi__f__XO * t12 * t32 * t28 * t206 * t35 * t60 * t121 * t248 - 2 * t170 * t157 * t206 * t45 * t122 * t158 * t104 + (-4 * phi__f__XO * t126 * t55 * t31 * t185 * t338 / t129 / t128 * t179 + 4 * phi__f__XO * t24 * t55 * t26 * t185 * t338 * t180) * t198 * t333 * t330 * t125 * t123 + t360 * t124 * t34 * t15 * t266 * t105 + t206 * t291 * t333 * t330 * t181 * t34 * t123 + 2 * phi__f__XO * t12 * t371 * t58 * t49 * t60 * t10;
    real_type t378 = t113 * t105;
    real_type t379 = t60 * t120;
    real_type t389 = t85 * t104;
    real_type t403 = t34 * t243 * t241;
    real_type t411 = t291 * t124;
    real_type t421 = t3 * t3;
    real_type t424 = 1.0 / (t421 * t11 + 1);
    real_type t442 = t113 * t389;
    real_type t448 = t217 * t42 * t28;
    real_type t462 = t57 * t57;
    real_type t469 = phi__f__XO * t17;
    real_type t517 = 2 * phi__f__XO * t12 * t198 * t191 * t330 * t124 * t34 * t379 * t378 - 2 * t144 * t28 * t109 * t206 * t279 * t202 * t389 - 4 * phi__f__XO * t12 * t206 * t403 * t302 / t59 / t14 * t8 * t378 - t32 * t28 * t198 * t191 * t411 * t279 * t113 * t248 - t371 * t97 * t6 * t5 * t1 + t100 * t424 * t1 + 2 * phi__f__XO * t12 * t207 * t379 * t273 - t371 * t58 * t50 * t8 * t424 * t2 + t293 * t411 * t403 * t302 * t61 * t378 - 2 * t448 * t206 * t35 * t243 * t303 * t61 * t442 + 2 * t448 * t360 * t124 * t35 * t278 * t442 - 4 * t469 * t55 / t52 / t19 * t51 * t371 / t462 * t50 * t8 * t6 * t5 * t2 - t198 * t191 * t291 * t125 * t278 * t273 - t109 * t207 * t275 * t264 * t202 * t105 - t371 * t58 * (8 * phi__f__XO * t24 * t29 * t46 * t89 * t39 * t21 - 6 * t82 * t162 * t230 * t158 * t227 * t21 - 2 * t469 * t63 * t27 * t22 * t72 + 2 * t76 * t299 * t29 * t39 * t72 - 2 * t218 * t32 * t22 * t21) * t71 * t7 * t2;
    return t377 + t517;
  }

  real_type
  Straight::Mzf_D_1_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[52];
    real_type t3   = ModelPars[7];
    real_type t5   = ModelPars[38];
    real_type t8   = t3 * ModelPars[36] + (Fzf__XO - t3) * t5;
    real_type t9   = 1.0 / t8;
    real_type t11  = ModelPars[51];
    real_type t12  = phi__f__XO * phi__f__XO;
    real_type t15  = ModelPars[57] * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t17  = 1.0 / t16;
    real_type t18  = t17 * t11;
    real_type t19  = ModelPars[59];
    real_type t20  = t19 * t18;
    real_type t22  = ModelPars[42];
    real_type t26  = ModelPars[48] * t12 + 1;
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
    real_type t53  = ModelPars[50];
    real_type t56  = ModelPars[44] * t12 + 1;
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
    real_type t216 = ModelPars[54];
    real_type t217 = ModelPars[58];
    real_type t219 = atan(phi__f__XO * t217);
    real_type t221 = 1.0 / t217;
    real_type t223 = ModelPars[60];
    real_type t228 = ModelPars[56] * t12 + 1;
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
  Straight::Mzf_D_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[52];
    real_type t2   = Fzf__XO * t1;
    real_type t4   = ModelPars[7];
    real_type t9   = t4 * ModelPars[36] + (Fzf__XO - t4) * ModelPars[38];
    real_type t10  = 1.0 / t9;
    real_type t11  = ModelPars[51];
    real_type t12  = phi__f__XO * phi__f__XO;
    real_type t13  = ModelPars[57];
    real_type t15  = t13 * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t17  = 1.0 / t16;
    real_type t18  = t17 * t11;
    real_type t19  = ModelPars[59];
    real_type t20  = ModelPars[42];
    real_type t21  = t20 * t19;
    real_type t23  = ModelPars[48];
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
    real_type t78  = ModelPars[50];
    real_type t79  = ModelPars[44];
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
    real_type t150 = ModelPars[54] * Fzf__XO;
    real_type t151 = ModelPars[58];
    real_type t152 = t151 * t151;
    real_type t156 = ModelPars[60];
    real_type t157 = ModelPars[56];
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
  Straight::Mzf_D_2_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[52];
    real_type t2   = Fzf__XO * t1;
    real_type t4   = ModelPars[7];
    real_type t9   = t4 * ModelPars[36] + (Fzf__XO - t4) * ModelPars[38];
    real_type t10  = 1.0 / t9;
    real_type t11  = ModelPars[51];
    real_type t12  = phi__f__XO * phi__f__XO;
    real_type t13  = ModelPars[57];
    real_type t15  = t13 * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t17  = 1.0 / t16;
    real_type t18  = t17 * t11;
    real_type t19  = ModelPars[59];
    real_type t20  = ModelPars[42];
    real_type t21  = t20 * t19;
    real_type t22  = Fzf__XO * t21;
    real_type t23  = ModelPars[48];
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
    real_type t38  = atan(alpha__f__XO * t35 * t22);
    real_type t39  = t13 * t38;
    real_type t43  = 1.0 / t15;
    real_type t44  = t43 * t11;
    real_type t47  = alpha__f__XO * t34;
    real_type t48  = phi__f__XO * t23;
    real_type t51  = t27 * t20;
    real_type t52  = t51 * t19;
    real_type t53  = t28 * Fzf__XO;
    real_type t54  = t30 * t30;
    real_type t55  = 1.0 / t54;
    real_type t59  = 1.0 / t33 / t32;
    real_type t60  = alpha__f__XO * t59;
    real_type t64  = -2 * t48 * t47 * t31 * Fzf__XO * t21 + 2 * t48 * t60 * t55 * t53 * t52;
    real_type t65  = alpha__f__XO * alpha__f__XO;
    real_type t66  = t19 * t19;
    real_type t69  = 1.0 / (t66 * t65 + 1);
    real_type t72  = -2 * phi__f__XO * t39 * t18 + t69 * t64 * t44;
    real_type t73  = t72 * t72;
    real_type t76  = t38 * t44;
    real_type t77  = cos(t76);
    real_type t78  = t43 * t77;
    real_type t79  = ModelPars[50];
    real_type t80  = ModelPars[44];
    real_type t81  = t80 * t12;
    real_type t82  = t81 + 1;
    real_type t84  = 1.0 / t82 * t9;
    real_type t85  = 1.0 / t79;
    real_type t89  = atan(alpha__f__XO * t34 * t85 * t84);
    real_type t90  = t89 * t79;
    real_type t91  = sin(t90);
    real_type t92  = t91 * t33;
    real_type t95  = t77 * t10;
    real_type t96  = t95 * t2;
    real_type t97  = t33 * t17;
    real_type t98  = t13 * t91;
    real_type t103 = 1.0 / t16 / t15;
    real_type t105 = t13 * t13;
    real_type t112 = ModelPars[54] * Fzf__XO;
    real_type t113 = ModelPars[58];
    real_type t115 = atan(phi__f__XO * t113);
    real_type t116 = t115 * t112;
    real_type t117 = 1.0 / t113;
    real_type t118 = ModelPars[60];
    real_type t119 = ModelPars[56];
    real_type t120 = t119 * t12;
    real_type t121 = t120 + 1;
    real_type t123 = 1.0 / t121 * t118;
    real_type t128 = atan(t47 * t26 * Fzf__XO * t20 * t123);
    real_type t129 = t13 * t128;
    real_type t133 = t121 * t121;
    real_type t134 = 1.0 / t133;
    real_type t135 = t134 * t118;
    real_type t136 = Fzf__XO * t20;
    real_type t137 = t136 * t135;
    real_type t138 = t119 * alpha__f__XO;
    real_type t142 = t136 * t123;
    real_type t143 = t34 * t31;
    real_type t144 = t23 * alpha__f__XO;
    real_type t145 = phi__f__XO * t144;
    real_type t148 = t53 * t51;
    real_type t149 = t148 * t123;
    real_type t150 = t59 * t55;
    real_type t154 = -2 * phi__f__XO * t138 * t35 * t137 - 2 * t145 * t143 * t142 + 2 * t145 * t150 * t149;
    real_type t155 = t118 * t118;
    real_type t158 = t65 * t134 * t155 + 1;
    real_type t159 = 1.0 / t158;
    real_type t162 = -2 * phi__f__XO * t129 * t18 + t159 * t154 * t44;
    real_type t163 = t162 * t162;
    real_type t165 = t128 * t44;
    real_type t166 = cos(t165);
    real_type t169 = t103 * t11;
    real_type t181 = t30 * t25;
    real_type t182 = 1.0 / t181;
    real_type t185 = t23 * t23;
    real_type t186 = t12 * t185;
    real_type t191 = 1.0 / t54 / t25;
    real_type t197 = t144 * t143;
    real_type t200 = t27 * t27;
    real_type t201 = t200 * t20;
    real_type t203 = t28 * t28;
    real_type t204 = t203 * Fzf__XO;
    real_type t206 = 1.0 / t54 / t181;
    real_type t209 = t203 * t200;
    real_type t212 = 1.0 / t33 / t55 / t209;
    real_type t218 = t144 * t150;
    real_type t227 = sin(t76);
    real_type t233 = t227 * t72 * t10 * t2;
    real_type t238 = t43 * t95;
    real_type t239 = t238 * t2;
    real_type t240 = t79 * t33;
    real_type t241 = t82 * t82;
    real_type t243 = 1.0 / t241 / t82;
    real_type t246 = t80 * t80;
    real_type t251 = 1.0 / t241;
    real_type t252 = t251 * t9;
    real_type t253 = t59 * t85;
    real_type t254 = alpha__f__XO * t253;
    real_type t258 = t23 * t182 * t28;
    real_type t262 = t85 * t252;
    real_type t272 = t12 * t185 / t54 / t30;
    real_type t276 = t254 * t84;
    real_type t278 = t12 * t185 * t55;
    real_type t288 = t9 * t9;
    real_type t289 = t251 * t288;
    real_type t290 = t79 * t79;
    real_type t291 = 1.0 / t290;
    real_type t293 = 1.0 / t27;
    real_type t294 = 1.0 / t28;
    real_type t299 = t65 * t30 * t294 * t293 * t291 * t289 + 1;
    real_type t300 = 1.0 / t299;
    real_type t302 = cos(t90);
    real_type t311 = phi__f__XO * t23 * t182;
    real_type t315 = -2 * phi__f__XO * t80 * t47 * t262 + 2 * t311 * t29 * t276;
    real_type t316 = t315 * t315;
    real_type t317 = t299 * t299;
    real_type t318 = 1.0 / t317;
    real_type t323 = t113 * t113;
    real_type t325 = t323 * t12 + 1;
    real_type t328 = sin(t165);
    real_type t332 = t53 * t1;
    real_type t333 = t10 * t332;
    real_type t338 = t302 * t300 * t315;
    real_type t339 = t182 * t27;
    real_type t344 = t325 * t325;
    real_type t363 = 1.0 / t133 / t121;
    real_type t366 = t119 * t119;
    real_type t374 = t23 * t120;
    real_type t388 = t12 * t185 * alpha__f__XO;
    real_type t409 = t158 * t158;
    real_type t423 = t238 * t332;
    real_type t424 = t91 * t34;
    real_type t425 = t23 * t339;
    real_type t436 = t27 * t424;
    real_type t443 = t293 * t291;
    real_type t472 = t17 * t77;
    return t92 * t78 * t73 * t10 * t2 + 2 * t98 * t97 * t96 - 8 * t12 * t105 * t91 * t33 * t103 * t96 - t166 * t163 * t117 * t116 + t92 * t43 * t227 * (8 * t12 * t105 * t38 * t169 - 4 * phi__f__XO * t13 * t69 * t64 * t18 - 2 * t39 * t18 + t69 * (12 * t186 * alpha__f__XO * t212 * t206 * t204 * t201 * t19 + 8 * t186 * t47 * t182 * Fzf__XO * t21 - 20 * t186 * t60 * t191 * t53 * t52 + 2 * t218 * t53 * t52 - 2 * t197 * t22) * t44) * t10 * t2 - 4 * phi__f__XO * t98 * t97 * t233 - t302 * t300 * (8 * t12 * t246 * t47 * t85 * t243 * t9 + 12 * t272 * t209 * alpha__f__XO * t212 * t85 * t84 - 8 * t258 * t27 * t81 * t254 * t252 + 2 * t258 * t27 * alpha__f__XO * t253 * t84 - 2 * t80 * t47 * t262 - 12 * t278 * t29 * t276) * t240 * t239 + t91 * t318 * t316 * t290 * t33 * t239 - 2 * t328 * t162 / t325 * t112 + 4 * t48 * t339 * t338 * t79 * t34 * t78 * t333 - 2 * phi__f__XO * t323 * t166 / t344 * t112 - t328 * (8 * t12 * t105 * t128 * t169 - 4 * phi__f__XO * t13 * t159 * t154 * t18 - 2 * t129 * t18 + t159 * (8 * t12 * t366 * alpha__f__XO * t35 * t136 * t363 * t118 + 12 * t388 * t212 * t206 * t204 * t201 * t123 + 8 * t374 * t47 * t31 * t136 * t135 - 8 * t374 * t60 * t55 * t148 * t135 + 8 * t388 * t34 * t182 * t142 - 20 * t388 * t59 * t191 * t149 - 2 * t138 * t35 * t137 - 2 * t197 * t142 + 2 * t218 * t149) * t44 + 4 * phi__f__XO * t119 * t65 * t363 * t155 / t409 * t154 * t44) * t117 * t116 + 2 * t425 * t424 * t423 + 4 * t272 * t200 * t91 * t59 * t238 * t204 * t1 - 12 * t278 * t436 * t423 + (-4 * phi__f__XO * t80 * t65 * t30 * t294 * t443 * t243 * t288 + 4 * phi__f__XO * t23 * t65 * t25 * t294 * t443 * t289) * t302 * t318 * t315 * t240 * t239 + 2 * t338 * t79 * t33 * t43 * t233 - 4 * t311 * t436 * t43 * t227 * t72 * t333 - 8 * t425 * t12 * t98 * t34 * t472 * t333 + 4 * phi__f__XO * t13 * t302 * t300 * t315 * t79 * t33 * t472 * t10 * t2;
  }

  real_type
  Straight::Mzf_D_2_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[52];
    real_type t2   = Fzf__XO * t1;
    real_type t4   = ModelPars[7];
    real_type t9   = t4 * ModelPars[36] + (Fzf__XO - t4) * ModelPars[38];
    real_type t10  = 1.0 / t9;
    real_type t11  = ModelPars[51];
    real_type t12  = phi__f__XO * phi__f__XO;
    real_type t13  = ModelPars[57];
    real_type t15  = t13 * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t17  = 1.0 / t16;
    real_type t18  = t17 * t11;
    real_type t19  = ModelPars[59];
    real_type t20  = ModelPars[42];
    real_type t21  = t20 * t19;
    real_type t22  = Fzf__XO * t21;
    real_type t24  = ModelPars[48];
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
    real_type t96  = ModelPars[50];
    real_type t97  = ModelPars[44];
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
    real_type t249 = ModelPars[54];
    real_type t250 = t29 * t249;
    real_type t251 = ModelPars[58];
    real_type t252 = t251 * t251;
    real_type t257 = ModelPars[60];
    real_type t260 = ModelPars[56];
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
  Straight::Mzf_D_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[52];
    real_type t2   = Fzf__XO * Fzf__XO;
    real_type t5   = ModelPars[7];
    real_type t10  = t5 * ModelPars[36] + (Fzf__XO - t5) * ModelPars[38];
    real_type t12  = ModelPars[51];
    real_type t14  = phi__f__XO * phi__f__XO;
    real_type t17  = ModelPars[57] * t14 + 1;
    real_type t18  = t17 * t17;
    real_type t22  = ModelPars[59];
    real_type t23  = ModelPars[42];
    real_type t24  = t23 * t22;
    real_type t27  = ModelPars[48] * t14 + 1;
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
    real_type t52  = ModelPars[50];
    real_type t55  = ModelPars[44] * t14 + 1;
    real_type t56  = 1.0 / t55;
    real_type t62  = atan(alpha__f__XO * t44 / t52 * t56 * t10);
    real_type t63  = t62 * t52;
    real_type t64  = sin(t63);
    real_type t69  = cos(t49);
    real_type t72  = t10 * t10;
    real_type t73  = t55 * t55;
    real_type t76  = t52 * t52;
    real_type t87  = cos(t63);
    real_type t93  = ModelPars[58];
    real_type t95  = atan(phi__f__XO * t93);
    real_type t101 = ModelPars[60];
    real_type t104 = ModelPars[56] * t14 + 1;
    real_type t107 = t23 / t104 * t101;
    real_type t108 = t101 * t101;
    real_type t109 = t104 * t104;
    real_type t119 = atan(alpha__f__XO * t44 * t28 * Fzf__XO * t107);
    real_type t121 = sin(t119 * t36);
    return t64 * t50 / (t31 * t30 + 1) * t28 * t24 / t18 * t12 / t10 * t2 * t1 - t87 / (t30 * t40 / t2 / t38 / t76 / t73 * t72 + 1) * t56 * t35 * t69 * Fzf__XO * t1 - t121 / (t30 / t109 * t108 + 1) * t45 * t107 * t35 * t12 / t93 * t95 * t2 * ModelPars[54];
  }

  real_type
  Straight::Mzf_D_3_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[52];
    real_type t2   = Fzf__XO * Fzf__XO;
    real_type t3   = t2 * t1;
    real_type t5   = ModelPars[7];
    real_type t10  = t5 * ModelPars[36] + (Fzf__XO - t5) * ModelPars[38];
    real_type t11  = 1.0 / t10;
    real_type t13  = ModelPars[51];
    real_type t14  = phi__f__XO * phi__f__XO;
    real_type t17  = ModelPars[57] * t14 + 1;
    real_type t18  = t17 * t17;
    real_type t19  = 1.0 / t18;
    real_type t21  = ModelPars[59];
    real_type t22  = t21 * t21;
    real_type t26  = ModelPars[42];
    real_type t29  = ModelPars[48] * t14 + 1;
    real_type t30  = 1.0 / t29;
    real_type t32  = alpha__f__XO * alpha__f__XO;
    real_type t34  = t22 * t32 + 1;
    real_type t35  = t34 * t34;
    real_type t36  = 1.0 / t35;
    real_type t38  = 1.0 / t17;
    real_type t39  = t13 * t38;
    real_type t42  = t26 * t26;
    real_type t44  = t29 * t29;
    real_type t45  = 1.0 / t44;
    real_type t47  = sqrt(t45 * t2 * t42);
    real_type t48  = 1.0 / t47;
    real_type t52  = atan(alpha__f__XO * t48 * t30 * Fzf__XO * t26 * t21);
    real_type t53  = t52 * t39;
    real_type t54  = sin(t53);
    real_type t55  = ModelPars[50];
    real_type t58  = ModelPars[44] * t14 + 1;
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
    real_type t143 = ModelPars[54];
    real_type t145 = ModelPars[58];
    real_type t147 = atan(phi__f__XO * t145);
    real_type t149 = 1.0 / t145;
    real_type t151 = ModelPars[60];
    real_type t152 = t151 * t151;
    real_type t159 = ModelPars[56] * t14 + 1;
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
  Straight::Mzr( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t4   = ModelPars[8];
    real_type t9   = t4 * ModelPars[37] + (Fzr__XO - t4) * ModelPars[39];
    real_type t13  = phi__XO * phi__XO;
    real_type t17  = 1.0 / (ModelPars[57] * t13 + 1);
    real_type t18  = t17 * ModelPars[51];
    real_type t20  = ModelPars[43];
    real_type t25  = ModelPars[49] * t13 + 1;
    real_type t26  = 1.0 / t25;
    real_type t27  = t20 * t20;
    real_type t28  = Fzr__XO * Fzr__XO;
    real_type t30  = t25 * t25;
    real_type t33  = sqrt(1.0 / t30 * t28 * t27);
    real_type t34  = 1.0 / t33;
    real_type t38  = atan(alpha__r__XO * t34 * t26 * Fzr__XO * t20 * ModelPars[59]);
    real_type t40  = cos(t38 * t18);
    real_type t42  = ModelPars[50];
    real_type t52  = atan(alpha__r__XO * t34 / t42 / (ModelPars[45] * t13 + 1) * t9);
    real_type t54  = sin(t52 * t42);
    real_type t60  = ModelPars[58];
    real_type t62  = atan(phi__XO * t60);
    real_type t76  = atan(alpha__r__XO * t34 * t26 * Fzr__XO * t20 / (ModelPars[56] * t13 + 1) * ModelPars[60]);
    real_type t78  = cos(t76 * t18);
    return -t54 * t33 * t17 * t40 / t9 * Fzr__XO * ModelPars[53] + t78 / t60 * t62 * ModelPars[55] * Fzr__XO;
  }

  real_type
  Straight::Mzr_D_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[53];
    real_type t3   = ModelPars[8];
    real_type t5   = ModelPars[39];
    real_type t8   = t3 * ModelPars[37] + (Fzr__XO - t3) * t5;
    real_type t9   = 1.0 / t8;
    real_type t11  = ModelPars[51];
    real_type t12  = phi__XO * phi__XO;
    real_type t15  = ModelPars[57] * t12 + 1;
    real_type t16  = 1.0 / t15;
    real_type t17  = t16 * t11;
    real_type t18  = ModelPars[59];
    real_type t19  = ModelPars[43];
    real_type t20  = t19 * t18;
    real_type t24  = ModelPars[49] * t12 + 1;
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
    real_type t42  = ModelPars[50];
    real_type t45  = ModelPars[45] * t12 + 1;
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
    real_type t132 = ModelPars[55];
    real_type t133 = ModelPars[58];
    real_type t135 = atan(phi__XO * t133);
    real_type t137 = 1.0 / t133;
    real_type t138 = ModelPars[60];
    real_type t141 = ModelPars[56] * t12 + 1;
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
  Straight::Mzr_D_1_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[53];
    real_type t2   = Fzr__XO * t1;
    real_type t4   = ModelPars[8];
    real_type t6   = ModelPars[39];
    real_type t9   = t4 * ModelPars[37] + (Fzr__XO - t4) * t6;
    real_type t10  = 1.0 / t9;
    real_type t11  = ModelPars[51];
    real_type t12  = t11 * t11;
    real_type t14  = phi__XO * phi__XO;
    real_type t17  = ModelPars[57] * t14 + 1;
    real_type t18  = t17 * t17;
    real_type t23  = ModelPars[59];
    real_type t24  = ModelPars[43];
    real_type t25  = t24 * t23;
    real_type t28  = ModelPars[49] * t14 + 1;
    real_type t29  = 1.0 / t28;
    real_type t30  = t24 * t24;
    real_type t31  = Fzr__XO * Fzr__XO;
    real_type t33  = t28 * t28;
    real_type t34  = 1.0 / t33;
    real_type t35  = t34 * t31 * t30;
    real_type t36  = sqrt(t35);
    real_type t37  = 1.0 / t36;
    real_type t39  = alpha__r__XO * t37 * t29;
    real_type t41  = t30 * t24;
    real_type t42  = t41 * t23;
    real_type t45  = 1.0 / t33 / t28;
    real_type t47  = 1.0 / t36 / t35;
    real_type t48  = t47 * t45;
    real_type t51  = -alpha__r__XO * t48 * t31 * t42 + t39 * t25;
    real_type t52  = t51 * t51;
    real_type t53  = alpha__r__XO * alpha__r__XO;
    real_type t54  = t23 * t23;
    real_type t56  = t54 * t53 + 1;
    real_type t57  = t56 * t56;
    real_type t60  = 1.0 / t17;
    real_type t61  = t60 * t11;
    real_type t64  = atan(t39 * Fzr__XO * t25);
    real_type t65  = t64 * t61;
    real_type t66  = cos(t65);
    real_type t68  = ModelPars[50];
    real_type t71  = ModelPars[45] * t14 + 1;
    real_type t72  = 1.0 / t71;
    real_type t73  = t72 * t9;
    real_type t74  = 1.0 / t68;
    real_type t76  = alpha__r__XO * t37 * t74;
    real_type t78  = atan(t76 * t73);
    real_type t79  = t78 * t68;
    real_type t80  = sin(t79);
    real_type t85  = 1.0 / t18;
    real_type t86  = t85 * t11;
    real_type t87  = t51 * t86;
    real_type t89  = 1.0 / t56;
    real_type t90  = sin(t65);
    real_type t91  = t90 * t89;
    real_type t93  = t72 * t6;
    real_type t95  = t47 * t74;
    real_type t99  = t34 * Fzr__XO * t30 * alpha__r__XO;
    real_type t101 = -t99 * t95 * t73 + t76 * t93;
    real_type t102 = t101 * t68;
    real_type t103 = t9 * t9;
    real_type t104 = t71 * t71;
    real_type t105 = 1.0 / t104;
    real_type t107 = t68 * t68;
    real_type t108 = 1.0 / t107;
    real_type t109 = t108 * t105 * t103;
    real_type t110 = 1.0 / t30;
    real_type t111 = 1.0 / t31;
    real_type t113 = t53 * t33;
    real_type t116 = t113 * t111 * t110 * t109 + 1;
    real_type t117 = 1.0 / t116;
    real_type t118 = cos(t79);
    real_type t119 = t118 * t117;
    real_type t124 = ModelPars[55];
    real_type t126 = ModelPars[58];
    real_type t128 = atan(phi__XO * t126);
    real_type t129 = 1.0 / t126;
    real_type t131 = t129 * t128 * Fzr__XO * t124;
    real_type t133 = ModelPars[60];
    real_type t136 = ModelPars[56] * t14 + 1;
    real_type t138 = 1.0 / t136 * t133;
    real_type t139 = t24 * t138;
    real_type t141 = t41 * t138;
    real_type t143 = alpha__r__XO * t47;
    real_type t146 = -t143 * t45 * t31 * t141 + t39 * t139;
    real_type t147 = t146 * t146;
    real_type t148 = t133 * t133;
    real_type t149 = t136 * t136;
    real_type t153 = t53 / t149 * t148 + 1;
    real_type t154 = t153 * t153;
    real_type t161 = atan(alpha__r__XO * t37 * t29 * Fzr__XO * t139);
    real_type t162 = t161 * t61;
    real_type t163 = cos(t162);
    real_type t167 = t31 * Fzr__XO;
    real_type t169 = t66 * t10;
    real_type t172 = t30 * t30;
    real_type t174 = t33 * t33;
    real_type t175 = 1.0 / t174;
    real_type t182 = t172 * t24;
    real_type t185 = 1.0 / t174 / t28;
    real_type t187 = t31 * t31;
    real_type t191 = 1.0 / t36 / t175 / t187 / t172;
    real_type t197 = 1.0 / t153;
    real_type t199 = sin(t162);
    real_type t203 = t10 * t1;
    real_type t205 = t89 * t51;
    real_type t207 = t80 * t36 * t90;
    real_type t211 = t60 * t66;
    real_type t212 = t211 * t203;
    real_type t213 = t80 * t37;
    real_type t219 = t68 * t36;
    real_type t229 = t36 * t60;
    real_type t230 = t6 * t6;
    real_type t235 = 1.0 / t103;
    real_type t251 = t60 * t169 * t2;
    real_type t272 = t101 * t101;
    real_type t273 = t116 * t116;
    real_type t274 = 1.0 / t273;
    real_type t279 = t31 * t1;
    real_type t281 = t60 * t66 * t235;
    real_type t305 = t101 * t219;
    real_type t336 = t10 * t279;
    return t80 * t36 * t66 / t57 * t52 / t18 / t17 * t12 * t10 * t2 + 2 * t119 * t102 * t36 * t91 * t87 * t10 * t2 - t163 / t154 * t147 * t85 * t12 * t131 + t175 * t172 * t80 * t47 * t60 * t169 * t167 * t1 - t199 * t197 * (3 * alpha__r__XO * t191 * t185 * t167 * t182 * t138 - 3 * alpha__r__XO * Fzr__XO * t48 * t141) * t61 * t131 + 2 * t207 * t205 * t86 * t203 - 3 * t34 * Fzr__XO * t30 * t213 * t212 - 2 * t118 * t117 * t101 * t219 * t212 - 2 * t230 * t80 * t229 * t66 / t103 / t9 * t2 + 2 * t6 * t80 * t229 * t66 * t235 * t1 - 2 * t199 * t197 * t146 * t60 * t11 * t129 * t128 * t124 - t118 * t117 * (3 * t175 * t31 * t172 * alpha__r__XO * t191 * t74 * t73 - t34 * t30 * t143 * t74 * t73 - 2 * t99 * t95 * t93) * t219 * t251 + t80 * t274 * t272 * t107 * t36 * t251 + 2 * t34 * t30 * t6 * t213 * t281 * t279 + t207 * t89 * (3 * alpha__r__XO * t191 * t185 * t167 * t182 * t23 - 3 * Fzr__XO * t143 * t45 * t42) * t85 * t11 * t10 * t2 + 2 * t6 * t119 * t305 * t281 * t2 + (2 * t6 * t53 * t33 * t111 * t110 * t108 * t105 * t9 - 2 * t113 / t167 * t110 * t109) * t118 * t274 * t305 * t251 - 2 * t6 * t80 * t36 * t90 * t205 * t85 * t11 * t235 * t2 - 2 * t34 * t30 * t118 * t117 * t102 * t37 * t211 * t336 + 2 * t34 * t30 * t80 * t37 * t91 * t87 * t336;
  }

  real_type
  Straight::Mzr_D_1_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[53];
    real_type t2   = Fzr__XO * t1;
    real_type t4   = ModelPars[8];
    real_type t6   = ModelPars[39];
    real_type t9   = t4 * ModelPars[37] + (Fzr__XO - t4) * t6;
    real_type t10  = 1.0 / t9;
    real_type t11  = ModelPars[51];
    real_type t13  = phi__XO * phi__XO;
    real_type t14  = ModelPars[57];
    real_type t16  = t14 * t13 + 1;
    real_type t17  = t16 * t16;
    real_type t18  = 1.0 / t17;
    real_type t20  = t18 * t11 * t10 * t2;
    real_type t21  = ModelPars[59];
    real_type t22  = ModelPars[43];
    real_type t23  = t22 * t21;
    real_type t24  = ModelPars[49];
    real_type t26  = t24 * t13 + 1;
    real_type t27  = 1.0 / t26;
    real_type t28  = t22 * t22;
    real_type t29  = Fzr__XO * Fzr__XO;
    real_type t30  = t29 * t28;
    real_type t31  = t26 * t26;
    real_type t32  = 1.0 / t31;
    real_type t33  = t32 * t30;
    real_type t34  = sqrt(t33);
    real_type t35  = 1.0 / t34;
    real_type t36  = t35 * t27;
    real_type t37  = alpha__r__XO * t36;
    real_type t39  = t28 * t22;
    real_type t40  = t39 * t21;
    real_type t43  = 1.0 / t31 / t26;
    real_type t45  = 1.0 / t34 / t33;
    real_type t46  = t45 * t43;
    real_type t49  = -alpha__r__XO * t46 * t29 * t40 + t37 * t23;
    real_type t50  = alpha__r__XO * alpha__r__XO;
    real_type t51  = t21 * t21;
    real_type t54  = 1.0 / (t51 * t50 + 1);
    real_type t55  = t54 * t49;
    real_type t56  = t18 * t11;
    real_type t59  = atan(t37 * Fzr__XO * t23);
    real_type t64  = 1.0 / t16;
    real_type t65  = t64 * t11;
    real_type t66  = t32 * Fzr__XO;
    real_type t68  = alpha__r__XO * t35;
    real_type t69  = phi__XO * t24;
    real_type t70  = t69 * t68;
    real_type t72  = t29 * Fzr__XO;
    real_type t73  = t31 * t31;
    real_type t74  = 1.0 / t73;
    real_type t77  = alpha__r__XO * t45;
    real_type t84  = -2 * phi__XO * t14 * t59 * t56 + t54 * (2 * t69 * t77 * t74 * t72 * t40 - 2 * t70 * t66 * t23) * t65;
    real_type t86  = t59 * t65;
    real_type t87  = cos(t86);
    real_type t89  = ModelPars[50];
    real_type t90  = ModelPars[45];
    real_type t92  = t90 * t13 + 1;
    real_type t93  = 1.0 / t92;
    real_type t94  = t93 * t9;
    real_type t95  = 1.0 / t89;
    real_type t97  = alpha__r__XO * t35 * t95;
    real_type t99  = atan(t97 * t94);
    real_type t100 = t99 * t89;
    real_type t101 = sin(t100);
    real_type t105 = t29 * t1;
    real_type t106 = t87 * t10;
    real_type t116 = t29 * t29;
    real_type t118 = t64 * t106;
    real_type t121 = t28 * t28;
    real_type t126 = phi__XO * t24 / t73 / t26;
    real_type t130 = t118 * t2;
    real_type t131 = t89 * t34;
    real_type t132 = t93 * t6;
    real_type t134 = t45 * t95;
    real_type t139 = -t66 * t28 * alpha__r__XO * t134 * t94 + t97 * t132;
    real_type t141 = t9 * t9;
    real_type t142 = t92 * t92;
    real_type t143 = 1.0 / t142;
    real_type t144 = t143 * t141;
    real_type t145 = t89 * t89;
    real_type t146 = 1.0 / t145;
    real_type t148 = 1.0 / t28;
    real_type t149 = 1.0 / t29;
    real_type t154 = t50 * t31 * t149 * t148 * t146 * t144 + 1;
    real_type t155 = t154 * t154;
    real_type t156 = 1.0 / t155;
    real_type t157 = cos(t100);
    real_type t162 = t148 * t146;
    real_type t180 = t10 * t1;
    real_type t181 = t64 * t87;
    real_type t182 = t35 * t181;
    real_type t184 = t28 * t101;
    real_type t187 = phi__XO * t24 * t43;
    real_type t191 = t10 * t2;
    real_type t194 = sin(t86);
    real_type t196 = t34 * t194 * t54;
    real_type t197 = t143 * t9;
    real_type t200 = phi__XO * t90 * t68;
    real_type t202 = alpha__r__XO * t134;
    real_type t203 = t202 * t94;
    real_type t204 = t187 * t30;
    real_type t207 = -2 * t200 * t95 * t197 + 2 * t204 * t203;
    real_type t208 = t207 * t89;
    real_type t209 = 1.0 / t154;
    real_type t210 = t157 * t209;
    real_type t214 = 1.0 / t141;
    real_type t215 = t87 * t214;
    real_type t223 = t194 * t84 * t10;
    real_type t228 = t157 * t209 * t139;
    real_type t237 = ModelPars[55];
    real_type t238 = Fzr__XO * t237;
    real_type t239 = ModelPars[58];
    real_type t241 = atan(phi__XO * t239);
    real_type t242 = 1.0 / t239;
    real_type t243 = t242 * t241;
    real_type t245 = t11 * t243 * t238;
    real_type t246 = ModelPars[60];
    real_type t247 = ModelPars[56];
    real_type t249 = t247 * t13 + 1;
    real_type t251 = 1.0 / t249 * t246;
    real_type t252 = t22 * t251;
    real_type t258 = -t77 * t43 * t29 * t39 * t251 + t37 * t252;
    real_type t260 = t246 * t246;
    real_type t261 = t249 * t249;
    real_type t262 = 1.0 / t261;
    real_type t265 = t50 * t262 * t260 + 1;
    real_type t266 = 1.0 / t265;
    real_type t271 = atan(t68 * t27 * Fzr__XO * t252);
    real_type t272 = t271 * t65;
    real_type t273 = sin(t272);
    real_type t280 = t64 * t194;
    real_type t281 = t101 * t34;
    real_type t285 = t262 * t246;
    real_type t288 = phi__XO * t247;
    real_type t306 = phi__XO * t247 * alpha__r__XO;
    real_type t310 = t121 * t22;
    real_type t314 = 1.0 / t73 / t31;
    real_type t318 = 1.0 / t34 / t74 / t116 / t121;
    real_type t321 = phi__XO * t24 * alpha__r__XO;
    real_type t340 = t18 * t87;
    real_type t350 = t101 * t34 * t87 * t84 * t55 * t20 + 2 * phi__XO * t14 * t32 * t28 * t101 * t35 * t18 * t106 * t105 - 2 * t126 * t121 * t101 * t45 * t118 * t116 * t1 + (-4 * phi__XO * t90 * t50 * t31 * t149 * t162 / t142 / t92 * t141 + 4 * phi__XO * t24 * t50 * t26 * t149 * t162 * t144) * t157 * t156 * t139 * t131 * t130 + 6 * t187 * t29 * t184 * t182 * t180 + t210 * t208 * t196 * t49 * t56 * t191 + t6 * t210 * t207 * t131 * t64 * t215 * t2 + t228 * t89 * t34 * t64 * t223 * t2 + t101 * t207 * t156 * t139 * t145 * t34 * t130 + 2 * phi__XO * t14 * t273 * t266 * t258 * t18 * t245 + t281 * t280 * t84 * t180 - t273 * t266 * (8 * phi__XO * t24 * t29 * t77 * t74 * t39 * t251 - 6 * t321 * t318 * t314 * t116 * t310 * t251 - 2 * t288 * t68 * t27 * t22 * t285 + 2 * t306 * t46 * t29 * t39 * t285 - 2 * t70 * t32 * t22 * t251) * t65 * t243 * t238 - t6 * t281 * t280 * t84 * t214 * t2 - t157 * t209 * t207 * t131 * t181 * t180 + 2 * phi__XO * t14 * t157 * t209 * t139 * t89 * t34 * t340 * t191;
    real_type t351 = t72 * t1;
    real_type t376 = t239 * t239;
    real_type t379 = 1.0 / (t376 * t13 + 1);
    real_type t381 = cos(t272);
    real_type t385 = t258 * t64;
    real_type t388 = t265 * t265;
    real_type t399 = t10 * t351;
    real_type t405 = t69 * t43 * t28;
    real_type t420 = Fzr__XO * t22;
    real_type t430 = t45 * t74;
    real_type t437 = -2 * phi__XO * t14 * t271 * t56 + t266 * (-2 * t321 * t35 * t32 * t420 * t251 + 2 * t321 * t430 * t72 * t39 * t251 - 2 * t306 * t36 * t420 * t285) * t65;
    real_type t485 = Fzr__XO * t28;
    real_type t517 = -2 * t187 * t28 * t6 * t101 * t182 * t214 * t351 - 4 * phi__XO * t14 * t101 * t196 * t49 / t17 / t16 * t11 * t191 - t32 * t28 * t157 * t209 * t208 * t182 * t10 * t105 + t381 * t379 * t237 - 4 * t288 * t50 / t261 / t249 * t260 * t273 / t388 * t385 * t11 * t242 * t241 * t238 - 2 * t405 * t101 * t35 * t194 * t55 * t56 * t399 + 2 * t405 * t228 * t89 * t35 * t181 * t399 - t273 * t437 * t242 * t241 * t237 + 2 * phi__XO * t14 * t281 * t340 * t180 - t273 * t266 * t385 * t11 * t379 * t238 + t101 * t34 * t194 * t54 * (-6 * t69 * alpha__r__XO * t318 * t314 * t116 * t310 * t21 + 8 * t69 * t29 * alpha__r__XO * t430 * t40 - 2 * t70 * t32 * t23) * t20 + t32 * t184 * t35 * t64 * t223 * t105 - t157 * t209 * (-6 * t126 * t72 * t121 * alpha__r__XO * t318 * t95 * t94 + 2 * phi__XO * t90 * t32 * t485 * t202 * t197 - 2 * t200 * t95 * t143 * t6 + 2 * t204 * t202 * t132 + 4 * t187 * t485 * t203) * t131 * t130 - 2 * phi__XO * t14 * t6 * t281 * t18 * t215 * t2 - t381 * t437 * t266 * t385 * t245;
    return t350 + t517;
  }

  real_type
  Straight::Mzr_D_1_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[53];
    real_type t3   = ModelPars[8];
    real_type t5   = ModelPars[39];
    real_type t8   = t3 * ModelPars[37] + (Fzr__XO - t3) * t5;
    real_type t9   = 1.0 / t8;
    real_type t11  = ModelPars[51];
    real_type t12  = phi__XO * phi__XO;
    real_type t15  = ModelPars[57] * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t17  = 1.0 / t16;
    real_type t18  = t17 * t11;
    real_type t19  = ModelPars[59];
    real_type t20  = t19 * t18;
    real_type t22  = ModelPars[43];
    real_type t26  = ModelPars[49] * t12 + 1;
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
    real_type t53  = ModelPars[50];
    real_type t56  = ModelPars[45] * t12 + 1;
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
    real_type t148 = t93 * t9;
    real_type t149 = t11 * t11;
    real_type t168 = t57 * t5;
    real_type t173 = t42 * Fzr__XO;
    real_type t176 = -t173 * t38 * alpha__r__XO * t121 * t59 * t58 + t61 * t168;
    real_type t177 = t176 * t53;
    real_type t197 = t34 * t70;
    real_type t201 = t86 * t86;
    real_type t202 = 1.0 / t201;
    real_type t216 = ModelPars[55];
    real_type t217 = ModelPars[58];
    real_type t219 = atan(phi__XO * t217);
    real_type t221 = 1.0 / t217;
    real_type t223 = ModelPars[60];
    real_type t228 = ModelPars[56] * t12 + 1;
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
  Straight::Mzr_D_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[53];
    real_type t2   = Fzr__XO * t1;
    real_type t4   = ModelPars[8];
    real_type t9   = t4 * ModelPars[37] + (Fzr__XO - t4) * ModelPars[39];
    real_type t10  = 1.0 / t9;
    real_type t11  = ModelPars[51];
    real_type t12  = phi__XO * phi__XO;
    real_type t13  = ModelPars[57];
    real_type t15  = t13 * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t17  = 1.0 / t16;
    real_type t18  = t17 * t11;
    real_type t19  = ModelPars[59];
    real_type t20  = ModelPars[43];
    real_type t21  = t20 * t19;
    real_type t23  = ModelPars[49];
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
    real_type t53  = Fzr__XO * t28;
    real_type t54  = t30 * t30;
    real_type t55  = 1.0 / t54;
    real_type t59  = 1.0 / t33 / t32;
    real_type t65  = alpha__r__XO * alpha__r__XO;
    real_type t66  = t19 * t19;
    real_type t75  = t38 * t44;
    real_type t76  = sin(t75);
    real_type t78  = ModelPars[50];
    real_type t79  = ModelPars[45];
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
    real_type t150 = ModelPars[55] * Fzr__XO;
    real_type t151 = ModelPars[58];
    real_type t152 = t151 * t151;
    real_type t156 = ModelPars[60];
    real_type t157 = ModelPars[56];
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
  Straight::Mzr_D_2_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t2   = ModelPars[55] * Fzr__XO;
    real_type t3   = phi__XO * phi__XO;
    real_type t4   = ModelPars[58];
    real_type t5   = t4 * t4;
    real_type t7   = t5 * t3 + 1;
    real_type t9   = ModelPars[51];
    real_type t10  = ModelPars[57];
    real_type t12  = t10 * t3 + 1;
    real_type t13  = t12 * t12;
    real_type t14  = 1.0 / t13;
    real_type t15  = t14 * t9;
    real_type t16  = ModelPars[60];
    real_type t17  = ModelPars[56];
    real_type t18  = t17 * t3;
    real_type t19  = t18 + 1;
    real_type t21  = 1.0 / t19 * t16;
    real_type t22  = ModelPars[43];
    real_type t24  = ModelPars[49];
    real_type t26  = t24 * t3 + 1;
    real_type t27  = 1.0 / t26;
    real_type t29  = t22 * t22;
    real_type t30  = Fzr__XO * Fzr__XO;
    real_type t31  = t30 * t29;
    real_type t32  = t26 * t26;
    real_type t33  = 1.0 / t32;
    real_type t34  = t33 * t31;
    real_type t35  = sqrt(t34);
    real_type t36  = 1.0 / t35;
    real_type t37  = alpha__r__XO * t36;
    real_type t40  = atan(t37 * t27 * Fzr__XO * t22 * t21);
    real_type t41  = t10 * t40;
    real_type t45  = 1.0 / t12;
    real_type t46  = t45 * t9;
    real_type t47  = t19 * t19;
    real_type t48  = 1.0 / t47;
    real_type t49  = t48 * t16;
    real_type t50  = Fzr__XO * t22;
    real_type t51  = t50 * t49;
    real_type t52  = t36 * t27;
    real_type t53  = t17 * alpha__r__XO;
    real_type t57  = t50 * t21;
    real_type t58  = t36 * t33;
    real_type t59  = t24 * alpha__r__XO;
    real_type t60  = phi__XO * t59;
    real_type t63  = t29 * t22;
    real_type t64  = t30 * Fzr__XO;
    real_type t65  = t64 * t63;
    real_type t66  = t65 * t21;
    real_type t67  = t32 * t32;
    real_type t68  = 1.0 / t67;
    real_type t70  = 1.0 / t35 / t34;
    real_type t71  = t70 * t68;
    real_type t75  = -2 * phi__XO * t53 * t52 * t51 - 2 * t60 * t58 * t57 + 2 * t60 * t71 * t66;
    real_type t76  = t16 * t16;
    real_type t78  = alpha__r__XO * alpha__r__XO;
    real_type t80  = t78 * t48 * t76 + 1;
    real_type t81  = 1.0 / t80;
    real_type t84  = -2 * phi__XO * t41 * t15 + t81 * t75 * t46;
    real_type t86  = t40 * t46;
    real_type t87  = sin(t86);
    real_type t91  = ModelPars[53];
    real_type t92  = t64 * t91;
    real_type t94  = ModelPars[8];
    real_type t99  = t94 * ModelPars[37] + (Fzr__XO - t94) * ModelPars[39];
    real_type t100 = 1.0 / t99;
    real_type t101 = t100 * t92;
    real_type t102 = ModelPars[59];
    real_type t103 = t22 * t102;
    real_type t104 = Fzr__XO * t103;
    real_type t107 = atan(alpha__r__XO * t52 * t104);
    real_type t108 = t107 * t46;
    real_type t109 = cos(t108);
    real_type t110 = t45 * t109;
    real_type t111 = ModelPars[50];
    real_type t115 = ModelPars[45];
    real_type t116 = t115 * t3;
    real_type t117 = t116 + 1;
    real_type t118 = t117 * t117;
    real_type t119 = 1.0 / t118;
    real_type t120 = t119 * t99;
    real_type t121 = 1.0 / t111;
    real_type t122 = t121 * t120;
    real_type t127 = 1.0 / t117 * t99;
    real_type t128 = t70 * t121;
    real_type t129 = alpha__r__XO * t128;
    real_type t130 = t129 * t127;
    real_type t131 = t32 * t26;
    real_type t132 = 1.0 / t131;
    real_type t134 = phi__XO * t24 * t132;
    real_type t138 = -2 * phi__XO * t115 * t37 * t122 + 2 * t134 * t31 * t130;
    real_type t139 = t99 * t99;
    real_type t140 = t119 * t139;
    real_type t141 = t111 * t111;
    real_type t142 = 1.0 / t141;
    real_type t144 = 1.0 / t29;
    real_type t145 = 1.0 / t30;
    real_type t150 = t78 * t32 * t145 * t144 * t142 * t140 + 1;
    real_type t151 = 1.0 / t150;
    real_type t156 = atan(alpha__r__XO * t36 * t121 * t127);
    real_type t157 = t156 * t111;
    real_type t158 = cos(t157);
    real_type t159 = t158 * t151 * t138;
    real_type t160 = t132 * t29;
    real_type t161 = phi__XO * t24;
    real_type t166 = t7 * t7;
    real_type t169 = cos(t86);
    real_type t175 = atan(phi__XO * t4);
    real_type t176 = t175 * t2;
    real_type t177 = 1.0 / t4;
    real_type t179 = 1.0 / t13 / t12;
    real_type t180 = t179 * t9;
    real_type t181 = t10 * t10;
    real_type t194 = 1.0 / t47 / t19;
    real_type t197 = t17 * t17;
    real_type t205 = t24 * t18;
    real_type t211 = alpha__r__XO * t70;
    real_type t219 = t24 * t24;
    real_type t221 = t3 * t219 * alpha__r__XO;
    real_type t226 = 1.0 / t67 / t26;
    real_type t231 = t59 * t58;
    real_type t234 = t29 * t29;
    real_type t235 = t234 * t22;
    real_type t236 = t30 * t30;
    real_type t237 = t236 * Fzr__XO;
    real_type t241 = 1.0 / t67 / t131;
    real_type t242 = t236 * t234;
    real_type t245 = 1.0 / t35 / t68 / t242;
    real_type t250 = t59 * t71;
    real_type t256 = t80 * t80;
    real_type t270 = Fzr__XO * t91;
    real_type t271 = t109 * t100;
    real_type t272 = t271 * t270;
    real_type t273 = t35 * t14;
    real_type t274 = sin(t157);
    real_type t275 = t10 * t274;
    real_type t285 = t10 * t107;
    real_type t293 = t63 * t102;
    real_type t299 = -2 * t161 * t37 * t33 * Fzr__XO * t103 + 2 * t161 * t211 * t68 * t64 * t293;
    real_type t300 = t102 * t102;
    real_type t303 = 1.0 / (t300 * t78 + 1);
    real_type t306 = -2 * phi__XO * t285 * t15 + t303 * t299 * t46;
    real_type t307 = t306 * t306;
    real_type t310 = t274 * t35;
    real_type t313 = t84 * t84;
    real_type t330 = t3 * t219;
    real_type t357 = sin(t108);
    real_type t361 = t45 * t271;
    real_type t362 = t361 * t270;
    real_type t363 = t111 * t35;
    real_type t365 = 1.0 / t118 / t117;
    real_type t368 = t115 * t115;
    real_type t376 = t24 * t132 * t30;
    real_type t389 = t3 * t219 / t67 / t32;
    real_type t394 = t3 * t219 * t68;
    real_type t409 = t138 * t138;
    real_type t410 = t150 * t150;
    real_type t411 = 1.0 / t410;
    real_type t418 = t357 * t306 * t100 * t270;
    real_type t426 = t144 * t142;
    real_type t449 = t361 * t92;
    real_type t450 = t274 * t36;
    real_type t451 = t24 * t160;
    real_type t462 = t29 * t450;
    real_type t472 = t14 * t109;
    return -2 * t87 * t84 / t7 * t2 + 4 * t161 * t160 * t159 * t111 * t36 * t110 * t101 - 2 * phi__XO * t5 * t169 / t166 * t2 - t87 * (8 * t3 * t181 * t40 * t180 - 4 * phi__XO * t10 * t81 * t75 * t15 - 2 * t41 * t15 + t81 * (8 * t3 * t197 * alpha__r__XO * t52 * t50 * t194 * t16 + 12 * t221 * t245 * t241 * t237 * t235 * t21 - 8 * t205 * t211 * t68 * t65 * t49 + 8 * t205 * t37 * t33 * t50 * t49 + 8 * t221 * t36 * t132 * t57 - 20 * t221 * t70 * t226 * t66 - 2 * t53 * t52 * t51 - 2 * t231 * t57 + 2 * t250 * t66) * t46 + 4 * phi__XO * t17 * t78 * t194 * t76 / t256 * t75 * t46) * t177 * t176 + 2 * t275 * t273 * t272 - 8 * t3 * t181 * t274 * t35 * t179 * t272 + t310 * t110 * t307 * t100 * t270 - t169 * t313 * t177 * t176 + t310 * t45 * t357 * (8 * t3 * t181 * t107 * t180 - 4 * phi__XO * t10 * t303 * t299 * t15 - 2 * t285 * t15 + t303 * (12 * t330 * alpha__r__XO * t245 * t241 * t237 * t235 * t102 + 8 * t330 * t37 * t132 * Fzr__XO * t103 - 20 * t330 * t211 * t226 * t64 * t293 + 2 * t250 * t64 * t293 - 2 * t231 * t104) * t46) * t100 * t270 - t158 * t151 * (12 * t389 * t242 * alpha__r__XO * t245 * t121 * t127 + 8 * t3 * t368 * t37 * t121 * t365 * t99 - 8 * t376 * t29 * t116 * t129 * t120 + 2 * t376 * t29 * alpha__r__XO * t128 * t127 - 2 * t115 * t37 * t122 - 12 * t394 * t31 * t130) * t363 * t362 + t274 * t411 * t409 * t141 * t35 * t362 - 4 * phi__XO * t275 * t273 * t418 + (-4 * phi__XO * t115 * t78 * t32 * t145 * t426 * t365 * t139 + 4 * phi__XO * t24 * t78 * t26 * t145 * t426 * t140) * t158 * t411 * t138 * t363 * t362 + 2 * t159 * t111 * t35 * t45 * t418 + 2 * t451 * t450 * t449 + 4 * t389 * t234 * t274 * t70 * t361 * t237 * t91 - 12 * t394 * t462 * t449 - 4 * t134 * t462 * t45 * t357 * t306 * t101 - 8 * t451 * t3 * t275 * t36 * t472 * t101 + 4 * phi__XO * t10 * t158 * t151 * t138 * t111 * t35 * t472 * t100 * t270;
  }

  real_type
  Straight::Mzr_D_2_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[53];
    real_type t2   = Fzr__XO * t1;
    real_type t4   = ModelPars[8];
    real_type t9   = t4 * ModelPars[37] + (Fzr__XO - t4) * ModelPars[39];
    real_type t10  = 1.0 / t9;
    real_type t11  = ModelPars[51];
    real_type t12  = phi__XO * phi__XO;
    real_type t13  = ModelPars[57];
    real_type t15  = t13 * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t17  = 1.0 / t16;
    real_type t18  = t17 * t11;
    real_type t19  = ModelPars[59];
    real_type t20  = ModelPars[43];
    real_type t21  = t20 * t19;
    real_type t22  = Fzr__XO * t21;
    real_type t24  = ModelPars[49];
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
    real_type t96  = ModelPars[50];
    real_type t97  = ModelPars[45];
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
    real_type t249 = ModelPars[55];
    real_type t250 = t29 * t249;
    real_type t251 = ModelPars[58];
    real_type t252 = t251 * t251;
    real_type t257 = ModelPars[60];
    real_type t260 = ModelPars[56];
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
  Straight::Mzr_D_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[53];
    real_type t2   = Fzr__XO * Fzr__XO;
    real_type t5   = ModelPars[8];
    real_type t10  = t5 * ModelPars[37] + (Fzr__XO - t5) * ModelPars[39];
    real_type t12  = ModelPars[51];
    real_type t14  = phi__XO * phi__XO;
    real_type t17  = ModelPars[57] * t14 + 1;
    real_type t18  = t17 * t17;
    real_type t22  = ModelPars[59];
    real_type t23  = ModelPars[43];
    real_type t24  = t23 * t22;
    real_type t27  = ModelPars[49] * t14 + 1;
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
    real_type t52  = ModelPars[50];
    real_type t55  = ModelPars[45] * t14 + 1;
    real_type t56  = 1.0 / t55;
    real_type t62  = atan(alpha__r__XO * t44 / t52 * t56 * t10);
    real_type t63  = t62 * t52;
    real_type t64  = sin(t63);
    real_type t69  = cos(t49);
    real_type t72  = t10 * t10;
    real_type t73  = t55 * t55;
    real_type t76  = t52 * t52;
    real_type t87  = cos(t63);
    real_type t93  = ModelPars[58];
    real_type t95  = atan(phi__XO * t93);
    real_type t101 = ModelPars[60];
    real_type t104 = ModelPars[56] * t14 + 1;
    real_type t107 = t23 / t104 * t101;
    real_type t108 = t101 * t101;
    real_type t109 = t104 * t104;
    real_type t119 = atan(alpha__r__XO * t44 * t28 * Fzr__XO * t107);
    real_type t121 = sin(t119 * t36);
    return t64 * t50 / (t31 * t30 + 1) * t28 * t24 / t18 * t12 / t10 * t2 * t1 - t87 / (t30 * t40 / t2 / t38 / t76 / t73 * t72 + 1) * t56 * t35 * t69 * Fzr__XO * t1 - t121 / (t30 / t109 * t108 + 1) * t45 * t107 * t35 * t12 / t93 * t95 * t2 * ModelPars[55];
  }

  real_type
  Straight::Mzr_D_3_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[53];
    real_type t2   = Fzr__XO * Fzr__XO;
    real_type t3   = t2 * t1;
    real_type t5   = ModelPars[8];
    real_type t10  = t5 * ModelPars[37] + (Fzr__XO - t5) * ModelPars[39];
    real_type t11  = 1.0 / t10;
    real_type t13  = ModelPars[51];
    real_type t14  = phi__XO * phi__XO;
    real_type t17  = ModelPars[57] * t14 + 1;
    real_type t18  = t17 * t17;
    real_type t19  = 1.0 / t18;
    real_type t21  = ModelPars[59];
    real_type t22  = t21 * t21;
    real_type t26  = ModelPars[43];
    real_type t29  = ModelPars[49] * t14 + 1;
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
    real_type t55  = ModelPars[50];
    real_type t58  = ModelPars[45] * t14 + 1;
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
    real_type t143 = ModelPars[55];
    real_type t145 = ModelPars[58];
    real_type t147 = atan(phi__XO * t145);
    real_type t149 = 1.0 / t145;
    real_type t151 = ModelPars[60];
    real_type t152 = t151 * t151;
    real_type t159 = ModelPars[56] * t14 + 1;
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
  Straight::Mxf( real_type t__XO ) const {
    return 0;
  }

  real_type
  Straight::Mxf_D( real_type t__XO ) const {
    return 0;
  }

  real_type
  Straight::Mxf_DD( real_type t__XO ) const {
    return 0;
  }

  real_type
  Straight::Mxr( real_type t__XO ) const {
    return 0;
  }

  real_type
  Straight::Mxr_D( real_type t__XO ) const {
    return 0;
  }

  real_type
  Straight::Mxr_DD( real_type t__XO ) const {
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
  Straight::H_eval(
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
    real_type t2   = X__[19];
    real_type t5   = X__[20];
    real_type t8   = X__[21];
    real_type t11  = X__[22];
    real_type t14  = X__[23];
    real_type t17  = X__[24];
    real_type t20  = X__[25];
    real_type t23  = X__[26];
    real_type t26  = X__[27];
    real_type t29  = X__[28];
    real_type t32  = X__[29];
    real_type t35  = X__[30];
    real_type t38  = X__[31];
    real_type t40  = t11 * L__[22] + t14 * L__[23] + t17 * L__[24] + t2 * L__[19] + t20 * L__[25] + t23 * L__[26] + t26 * L__[27] + t29 * L__[28] + t32 * L__[29] + t35 * L__[30] + t38 * L__[31] + t5 * L__[20] + t8 * L__[21];
    real_type t42  = X__[32];
    real_type t45  = X__[38];
    real_type t46  = cos(t45);
    real_type t47  = X__[1];
    real_type t49  = sin(t45);
    real_type t50  = X__[0];
    real_type t58  = X__[34];
    real_type t65  = X__[35];
    real_type t70  = X__[17];
    real_type t71  = ALIAS_maxTorque(t70);
    real_type t74  = X__[33];
    real_type t75  = t74 * ModelPars[120];
    real_type t82  = X__[36];
    real_type t86  = X__[3];
    real_type t87  = cos(t86);
    real_type t88  = t87 * t87;
    real_type t89  = X__[2];
    real_type t90  = t89 * t89;
    real_type t91  = t90 * t88;
    real_type t92  = ModelPars[77];
    real_type t93  = ModelPars[149];
    real_type t94  = t93 * t92;
    real_type t95  = ModelPars[190];
    real_type t96  = ModelPars[220];
    real_type t98  = t96 * t95 + t94;
    real_type t99  = X__[6];
    real_type t100 = t99 * t98;
    real_type t101 = X__[5];
    real_type t103 = t101 * t100 * t91;
    real_type t105 = X__[7];
    real_type t106 = sin(t105);
    real_type t107 = t14 - t5;
    real_type t108 = t107 * t106;
    real_type t109 = ModelPars[76];
    real_type t110 = ModelPars[24];
    real_type t111 = ModelPars[219];
    real_type t112 = ModelPars[233];
    real_type t113 = t110 - t111 + t112;
    real_type t115 = t110 - t111 - t112;
    real_type t117 = t110 * t110;
    real_type t118 = ModelPars[78];
    real_type t119 = t118 * t117;
    real_type t120 = ModelPars[15];
    real_type t121 = ModelPars[20];
    real_type t122 = t115 * t113 * t109 + t119 - t120 + t121;
    real_type t123 = t122 * t2;
    real_type t124 = cos(t105);
    real_type t128 = X__[11];
    real_type t129 = Fzf(t128, t26);
    real_type t130 = X__[16];
    real_type t131 = X__[9];
    real_type t132 = X__[10];
    real_type t133 = X__[15];
    real_type t134 = alpha__f(t89, t50, t47, t131, t132, t133, t20, t23);
    real_type t135 = X__[18];
    real_type t136 = lambda__f(t89, t130, t50, t47, t131, t132, t133, t135, t20, t23);
    real_type t137 = Fxf(t129, t130, t134, t136);
    real_type t138 = t137 * t128;
    real_type t140 = ModelPars[66];
    real_type t141 = cos(t140);
    real_type t142 = t141 * t141;
    real_type t143 = t110 - t111;
    real_type t144 = t143 * t109;
    real_type t146 = ModelPars[164];
    real_type t147 = -t112 * t144 + t146;
    real_type t148 = 2 * t147;
    real_type t149 = t148 * t142;
    real_type t151 = sin(t140);
    real_type t152 = t151 * t122 * t141;
    real_type t153 = ModelPars[214];
    real_type t154 = cos(t153);
    real_type t155 = sin(t153);
    real_type t156 = ModelPars[75];
    real_type t157 = ModelPars[71];
    real_type t158 = t157 * t157;
    real_type t159 = t158 * t156;
    real_type t160 = ModelPars[14];
    real_type t161 = ModelPars[19];
    real_type t162 = t159 + t160 - t161;
    real_type t164 = t156 * t157;
    real_type t165 = ModelPars[151];
    real_type t166 = t165 * t164;
    real_type t168 = (t162 * t155 + t166) * t154;
    real_type t169 = ModelPars[161];
    real_type t170 = t156 * t169;
    real_type t171 = t155 * t157;
    real_type t172 = t171 * t170;
    real_type t173 = X__[8];
    real_type t174 = ModelPars[23];
    real_type t175 = t174 + t93;
    real_type t177 = t92 * t175 * t173;
    real_type t178 = t175 * t92;
    real_type t179 = ModelPars[204];
    real_type t180 = t179 * t178;
    real_type t181 = ModelPars[31];
    real_type t182 = ModelPars[25];
    real_type t183 = t182 * t181;
    real_type t184 = ModelPars[30];
    real_type t185 = t184 * t183;
    real_type t186 = ModelPars[0];
    real_type t187 = t149 - t152 + t168 + t172 - t177 + t180 - t185 - t166 - t186;
    real_type t191 = Q__[0];
    real_type t192 = Mxf(t191);
    real_type t193 = Mxr(t191);
    real_type t194 = Fyf(t129, t130, t134, t136);
    real_type t195 = t194 * t128;
    real_type t196 = X__[14];
    real_type t197 = Fzr(t196, t35);
    real_type t198 = X__[12];
    real_type t199 = X__[13];
    real_type t200 = alpha__r(t89, t50, t47, t198, t199, t29, t32);
    real_type t201 = lambda__r(t89, t86, t50, t199, t70, t29);
    real_type t202 = Fyr(t197, t86, t200, t201);
    real_type t207 = sin(t86);
    real_type t208 = t207 * t89;
    real_type t211 = t208 + 2 * t14 - 2 * t5;
    real_type t212 = t124 * t124;
    real_type t215 = t106 * t89;
    real_type t216 = -t147;
    real_type t217 = t211 * t216;
    real_type t221 = t122 * t142;
    real_type t223 = t216 * t151 * t141;
    real_type t224 = 2 * t223;
    real_type t225 = t101 * t101;
    real_type t226 = t225 * t182;
    real_type t227 = t154 * t154;
    real_type t228 = -t162;
    real_type t229 = t228 * t227;
    real_type t231 = t171 * t156 * t165;
    real_type t232 = 2 * t231;
    real_type t233 = t184 * t184;
    real_type t234 = t233 * t182;
    real_type t235 = ModelPars[10];
    real_type t236 = ModelPars[11];
    real_type t237 = t221 - t224 - t226 + t229 + t232 - t234 + t159 + t160 - t161 + t235 - t236;
    real_type t240 = -t122;
    real_type t242 = 2 * t142 * t240;
    real_type t243 = 4 * t223;
    real_type t244 = 2 * t162;
    real_type t245 = t227 * t244;
    real_type t246 = 4 * t231;
    real_type t247 = 2 * t234;
    real_type t248 = 2 * t159;
    real_type t249 = 2 * t160;
    real_type t250 = 2 * t161;
    real_type t251 = ModelPars[12];
    real_type t252 = t242 + t243 + t245 - t246 + t247 - t248 - t249 + t250 - t235 + t236 + t251;
    real_type t256 = t112 * t112;
    real_type t259 = ModelPars[16];
    real_type t260 = ModelPars[18];
    real_type t261 = -2 * t256 * t109 - t120 + t121 - t259 - t260;
    real_type t264 = t92 * t175 * t17;
    real_type t266 = ModelPars[17];
    real_type t267 = t135 * t266;
    real_type t268 = t70 * t260;
    real_type t271 = ModelPars[69];
    real_type t272 = t271 * t100;
    real_type t275 = t98 * t89;
    real_type t282 = t182 * t8;
    real_type t283 = t101 * t282;
    real_type t284 = t216 * t14;
    real_type t291 = t124 * t211;
    real_type t299 = 2 * t166;
    real_type t302 = 2 * t172;
    real_type t305 = 2 * t185;
    real_type t306 = 2 * t186;
    real_type t308 = t5 * (4 * t147 * t142 - 2 * t152 + (t244 * t155 + t299) * t154 + t302 - 2 * t177 + 2 * t180 - t305 - t299 - t306);
    real_type t309 = -t148;
    real_type t310 = t14 * t309;
    real_type t311 = ModelPars[127];
    real_type t312 = t311 - t173;
    real_type t314 = t17 * t92 * t312;
    real_type t320 = t212 * t122 * t107;
    real_type t321 = t216 * t106;
    real_type t322 = t124 * t107;
    real_type t323 = t322 * t321;
    real_type t326 = t154 * t157 * t170;
    real_type t327 = t173 * t173;
    real_type t328 = t327 * t92;
    real_type t329 = t328 / 2;
    real_type t331 = t92 * t311 * t173;
    real_type t332 = t179 * t92;
    real_type t335 = (t311 - t179 / 2) * t332;
    real_type t336 = t169 * t164;
    real_type t337 = t181 * t181;
    real_type t338 = -t233 + t337;
    real_type t340 = t338 * t182 / 2;
    real_type t341 = t235 / 2;
    real_type t342 = t251 / 2;
    real_type t343 = t221 - t224 + t229 - t326 + t231 + t329 - t331 + t335 + t159 + t336 + t340 + t160 - t161 + t341 - t342;
    real_type t345 = t240 / 2;
    real_type t354 = X__[4] + ModelPars[65];
    real_type t355 = sin(t354);
    real_type t360 = t92 * t311 * t93;
    real_type t363 = t95 * t96 * ModelPars[234];
    real_type t364 = ModelPars[163];
    real_type t365 = -t92 * t173 * t93 + t360 - t363 - t364;
    real_type t366 = t99 * t365;
    real_type t369 = t101 * t208;
    real_type t370 = t14 * t101;
    real_type t371 = t5 * t101;
    real_type t372 = t50 / 2;
    real_type t373 = t369 + t370 - t371 + t372;
    real_type t375 = t124 * t112;
    real_type t378 = -t143;
    real_type t379 = t109 * t378;
    real_type t380 = t110 * t118;
    real_type t381 = t379 - t380;
    real_type t384 = t112 * t109 * t141;
    real_type t385 = t144 + t380;
    real_type t386 = t385 * t151;
    real_type t387 = t154 * t164;
    real_type t388 = t92 * t173;
    real_type t392 = t370 - t371 + t372;
    real_type t395 = t384 + t386 + t387 - t164 + t332 - t388 - t183;
    real_type t402 = t109 * t112;
    real_type t405 = t385 * t101;
    real_type t409 = t8 * t402;
    real_type t410 = t101 * t385 * t14 - t5 * t405 + t409;
    real_type t411 = t2 * t410;
    real_type t415 = t385 * t271;
    real_type t416 = t106 * t415;
    real_type t417 = t5 * t366;
    real_type t418 = ModelPars[13];
    real_type t419 = t266 / 2;
    real_type t420 = ModelPars[165];
    real_type t421 = t420 / 2;
    real_type t422 = ModelPars[166];
    real_type t423 = t422 / 2;
    real_type t424 = ModelPars[167];
    real_type t425 = t424 / 2;
    real_type t427 = (t418 - t419 + t421 - t423 + t425) * t11;
    real_type t429 = -2 * t417 + 2 * t427;
    real_type t431 = t395 * t271;
    real_type t434 = t101 * t109;
    real_type t436 = t5 * t112 * t434;
    real_type t438 = t14 * t112 * t434;
    real_type t440 = t385 * t8 + t436 - t438;
    real_type t441 = t2 * t440;
    real_type t445 = t99 * t365 * t90;
    real_type t446 = t5 * t5;
    real_type t448 = t99 * t365 * t446;
    real_type t449 = t385 * t141;
    real_type t450 = t155 * t164;
    real_type t452 = t112 * t109 * t151;
    real_type t453 = t184 * t182;
    real_type t454 = t449 - t450 - t452 + t453;
    real_type t455 = t101 * t454;
    real_type t456 = t2 * t455;
    real_type t461 = t141 * t8 * t402;
    real_type t462 = t92 * t17;
    real_type t463 = t101 * t462;
    real_type t465 = (t386 + t387 + t332 - t388 - t183 - t164) * t8;
    real_type t470 = t266 * t11 * t135;
    real_type t471 = t355 * (-2 * t87 * (-t291 * t122 * t106 - t207 * t89 * t187 - 2 * t212 * t217 + t308 + t310 + t314) * t89 - 4 * (t320 - 2 * t323 + t5 * t343 + t14 * t345 - t264 / 2) * t2) - 2 * t366 * t91 + 2 * t87 * (-t375 * t109 * t373 + t207 * (t381 * t106 - t164 - t183 + t332 + t384 + t386 + t387 - t388) * t89 * t101 - t106 * t385 * t392 - (t371 - t372) * t395) * t89 + t124 * (-t207 * t271 * t402 + 2 * t411) + t207 * (t89 * t429 - t416 + t431) + 2 * t106 * t441 + t445 + t448 + t5 * (2 * t456 - 2 * t427) + t2 * (-2 * t461 + 2 * t463 - 2 * t465) - t470;
    real_type t472 = cos(t354);
    real_type t474 = t93 * t178;
    real_type t475 = t96 * t96;
    real_type t476 = t95 * t475;
    real_type t478 = t95 * t96 * t174;
    real_type t479 = t474 + t476 + t478 - t418 + t266 - t420 + t422;
    real_type t480 = t99 * t479;
    real_type t491 = t106 * t112;
    real_type t493 = t5 * t455;
    real_type t494 = t385 * t50;
    real_type t509 = t402 * t271 * t106;
    real_type t510 = t5 * t479;
    real_type t511 = t17 * t94;
    real_type t512 = t267 / 2;
    real_type t516 = 2 * t89 * (t510 + t511 - t512) * t99;
    real_type t517 = t454 * t271;
    real_type t523 = t99 * t479 * t90;
    real_type t526 = t395 * t101;
    real_type t527 = t2 * t526;
    real_type t533 = t454 * t8;
    real_type t535 = 2 * t2 * t533;
    real_type t544 = 2 * t326;
    real_type t545 = 2 * t331;
    real_type t546 = t179 * t311;
    real_type t548 = t179 * t179;
    real_type t550 = t92 * (-2 * t546 + t548);
    real_type t551 = 2 * t336;
    real_type t553 = -t338 * t182;
    real_type t554 = t242 + t243 + t245 + t544 - t232 - t328 + t545 + t550 - t248 - t551 + t553 - t249 + t250 - t235 + t251;
    real_type t555 = t5 * t554;
    real_type t556 = t14 * t122;
    real_type t560 = t107 * t216;
    real_type t561 = t212 * t560;
    real_type t563 = t124 * t122;
    real_type t564 = t563 * t108;
    real_type t570 = t472 * t472;
    real_type t576 = t101 * t99;
    real_type t578 = -2 * t103 + 2 * t124 * t123 * t108 - t133 * t138 - 2 * t5 * t2 * t187 + t192 + t193 - t195 - t202 * t196 + t129 * t132 + t197 * t199 - t87 * (-t212 * t211 * t122 * t89 + 2 * t124 * t217 * t215 + t207 * t237 * t90 + t89 * (-t182 * t50 * t101 + t14 * t261 + t5 * t252 + 2 * t264 - t267 - t268) + t272) - (-t182 * t271 * t101 - t50 * t99 * t275) * t207 - t2 * (2 * t283 + 2 * t284) - t472 * t471 - t355 * (2 * t480 * t91 + 2 * t87 * (t124 * t385 * t373 - t208 * t101 * (t112 * t109 * t106 + t449 - t450 - t452 + t453) - t491 * t392 * t109 + t493 - t141 * t494 / 2 + t463 - (-t450 - t452 + t453) * t50 / 2) * t89 + t124 * (t207 * t415 - 2 * t441) + t207 * (-t509 + t516 - t517) + 2 * t106 * t411 - t523 - t99 * t479 * t446 + t5 * (2 * t527 - 2 * t99 * (t511 - t512)) + t535) - t570 * (-2 * t87 * (-t212 * t211 * t122 + t207 * t343 * t89 + 2 * t291 * t321 + t264 + t555 + t556) * t89 - 2 * t2 * (-4 * t561 - 2 * t564 + t308 + t310 + t314)) + 4 * t212 * t2 * t560 + t576 * t98 * t90;
    real_type t581 = ModelPars[194];
    real_type t582 = t5 - t581 - t14;
    real_type t583 = t5 + t581 - t14;
    real_type t586 = ModelPars[154];
    real_type t587 = t586 * t106;
    real_type t588 = -t107;
    real_type t595 = t581 * t586;
    real_type t607 = t581 * t581;
    real_type t612 = t2 * t2;
    real_type t613 = t199 * t612;
    real_type t615 = ModelPars[125];
    real_type t619 = -2 * t581 * t586 * t615 - 2 * t196 * t595 - 2 * t35;
    real_type t622 = 2 * t32 * t595;
    real_type t623 = t199 * t607;
    real_type t626 = -t615 - t196;
    real_type t630 = -t199 * t595 - t32;
    real_type t647 = -t583;
    real_type t649 = -t582;
    real_type t679 = 2 * t581 * t586 * t8;
    real_type t680 = t607 * t101;
    real_type t688 = Fxr(t197, t86, t200, t201);
    real_type t689 = 2 * t378;
    real_type t691 = 2 * t380;
    real_type t692 = t689 * t109 - t691;
    real_type t696 = -t689 * t109 + t691;
    real_type t697 = t14 * t696;
    real_type t699 = t89 * (t5 * t692 + t697);
    real_type t704 = -2 * t14 * t402 + 2 * t5 * t402;
    real_type t708 = 2 * t450;
    real_type t709 = 2 * t452;
    real_type t710 = 2 * t453;
    real_type t711 = t141 * t696 - t708 - t709 + t710;
    real_type t713 = 2 * t462;
    real_type t718 = t2 * t89;
    real_type t722 = t90 * t385;
    real_type t723 = t446 * t385;
    real_type t724 = t14 * t692;
    real_type t725 = t5 * t724;
    real_type t726 = t14 * t14;
    real_type t727 = t726 * t385;
    real_type t728 = 2 * t87 * t718 * t402 + t722 + t723 + t725 + t727;
    real_type t730 = t2 * t696;
    real_type t731 = t87 * t89;
    real_type t733 = t90 * t402;
    real_type t734 = t726 * t402;
    real_type t735 = t5 * t14;
    real_type t737 = 2 * t735 * t402;
    real_type t738 = t446 * t402;
    real_type t741 = 2 * t384;
    real_type t743 = 2 * t387;
    real_type t744 = 2 * t164;
    real_type t745 = 2 * t332;
    real_type t746 = 2 * t388;
    real_type t747 = 2 * t183;
    real_type t748 = t692 * t151 - t741 - t743 + t744 - t745 + t746 + t747;
    real_type t749 = t2 * t748;
    real_type t752 = t381 * t141 + t450 + t452 - t453;
    real_type t755 = t17 * t5;
    real_type t757 = 2 * t92 * t755;
    real_type t765 = t151 * t696 + t741 + t743 - t744 + t745 - t746 - t747;
    real_type t767 = t89 * t5 * t765;
    real_type t778 = t381 * t151 + t164 + t183 - t332 - t384 - t387 + t388;
    real_type t779 = t90 * t778;
    real_type t780 = t446 * t778;
    real_type t783 = 2 * t98;
    real_type t784 = t99 * t783;
    real_type t790 = t182 * t101;
    real_type t793 = -t783;
    real_type t794 = t11 * t793;
    real_type t798 = ModelPars[1];
    real_type t799 = t50 * t50;
    real_type t800 = t799 * t798;
    real_type t801 = t89 * t47;
    real_type t806 = t207 * t472;
    real_type t813 = -t312;
    real_type t814 = t90 * t813;
    real_type t816 = t175 * t2;
    real_type t835 = t101 * t90;
    real_type t838 = t207 * t99;
    real_type t842 = t612 * t101;
    real_type t843 = t89 * t50;
    real_type t845 = t11 * t93;
    real_type t852 = t2 * t101;
    real_type t870 = t89 * t92;
    real_type t875 = t92 * t446;
    real_type t882 = -t137 * (t133 * t806 + t355) - t194 * (-t133 * t355 + t806) - t570 * t92 * (-t813 * t612 + 2 * t731 * t816 + t88 * t814) - t472 * (-2 * t355 * t92 * (-t88 * t90 * t175 / 2 + t731 * t813 * t2 + t175 * t612 / 2) - t129 * t87 + t92 * (t88 * t835 + t87 * (t838 * t90 * t93 + t271) - t835 - t842 - t207 * t843 + 2 * t2 * t845)) + 2 * t355 * t92 * (t87 * t89 * (t852 - t845) + t207 * t89 * (t2 * t93 * t99 + t8) - t801 / 2) + 2 * t92 * t731 * t816 - 2 * t207 * t870 * t813 * t5 + t92 * t814 - t173 * (-t875 + ModelPars[73]) - t311 * t875 - t17 * ModelPars[34];
    real_type t886 = t99 * t131;
    real_type t888 = t175 * t133;
    real_type t894 = t99 * t128;
    real_type t895 = t813 * t133;
    real_type t899 = -t132 * t133 - t131;
    real_type t901 = t99 * t813;
    real_type t902 = t133 * t128;
    real_type t910 = t99 * t132;
    real_type t918 = -t131 * t133 + t132;
    real_type t926 = Mzf(t129, t130, t134);
    real_type t932 = t266 * t718;
    real_type t933 = cos(t130);
    real_type t934 = t933 * t58;
    real_type t938 = sin(t130);
    real_type t951 = -t175;
    real_type t970 = t133 * t192;
    real_type t975 = t42 * L__[32] + (t47 * t46 + t50 * t49) * L__[37] + (-U__[0] * ModelPars[169] - t58) * L__[33] + (-U__[1] * ModelPars[170] - t65) * L__[35] + (t71 * U__[2] - t75) * L__[34] + (U__[3] * ModelPars[213] - t82) * L__[36] + t578 * L__[3] + (-t472 * (t124 * t583 * t582 - 2 * t588 * t581 * t587) * t110 - t355 * (t583 * t582 * t106 + 2 * t124 * t588 * t595) * t110 - 2 * t581 * t586 * t29 - t607 * t198) * L__[16] + (-t87 * (t2 * t619 + t613 - t622 - t623) + 2 * (t612 * t626 / 2 + t2 * t630 + (t586 * t35 + t581 * t615 / 2 + t196 * t581 / 2) * t581) * t207) * L__[17] + (t472 * t110 * (t649 * t647 * t106 - 2 * t322 * t595) - t355 * t110 * (2 * t107 * t581 * t587 + t124 * t649 * t647) - t87 * (-t196 * t607 - 2 * t2 * t630 - 2 * t35 * t595 - t607 * t615 - t612 * t626) - t207 * (-t2 * t619 - t613 + t622 + t623) - t679 - t680 - t607 * (-ModelPars[123] + t615)) * L__[18] + (t137 - t133 * t194 + t688 - t472 * (t207 * (t124 * t699 + t106 * t89 * t704 + t89 * (t5 * t711 + t713)) + t124 * t728 + t106 * (t731 * t730 - t733 - t734 + t737 - t738) + t731 * t749 + t90 * t752 + t446 * t752 - t757) - t355 * (t207 * (-t124 * t89 * t704 + t106 * t699 + t767) + t124 * (t731 * t2 * t692 + t733 + t734 - t737 + t738) + t106 * t728 + t731 * t2 * t711 + t779 + t780) - t207 * t89 * (t2 * t784 + 2 * t282) - t87 * t89 * (2 * t2 * t790 + t794) - t800 + t182 * t801) * L__[0] + t882 * L__[8] + (-t194 * (t355 * (t207 * t312 + t87 * t886 - t888) + t472 * (t101 * t87 * t99 + t207 * t175 - t894 - t895) + t207 * t899 + t87 * (t901 + t902) - t133 * t101) - t137 * (t355 * (t207 * (-t894 - t895) - t87 * t910 + t174 + t93) + t472 * (t207 * t888 + t173 - t311) + t207 * t918 - t87 * t128 + t101) - t926 * (t99 * t355 * t87 - t207) - t355 * (t129 * (t207 * t886 + t813 * t87) - t207 * (-t932 + t934) * t99 + t87 * (-t266 * t11 * t89 + t58 * t99 * t938) + t266 * t5 * t2 * t99) - t472 * (t129 * (t101 * t838 + t87 * t951 + t910) - t89 * t266 * t87 * t5 * t99 - t2 * t11 * t266 + t99 * t192) - t129 * (t87 * t131 + t207 * t901) + t207 * t938 * t58 + (-t932 + t934 + t970) * t87) * L__[9];
    real_type t979 = 2 * t951;
    real_type t981 = t5 * t581;
    real_type t983 = t173 * t607;
    real_type t985 = 2 * t17 * t595;
    real_type t986 = t607 * t311;
    real_type t998 = t446 * t951 + t5 * (-2 * t581 * t586 * t173 + 2 * t581 * t586 * t311 - 2 * t17) + t175 * t607;
    real_type t1000 = ModelPars[122];
    real_type t1001 = ModelPars[124];
    real_type t1002 = t1000 - t1001;
    real_type t1003 = t42 * t42;
    real_type t1004 = t1003 * t1002;
    real_type t1005 = -t1002;
    real_type t1006 = t607 * t1005;
    real_type t1009 = 2 * t1005;
    real_type t1010 = t586 * t1009;
    real_type t1032 = t202 * t198;
    real_type t1033 = Mzr(t197, t86, t200);
    real_type t1036 = t112 * t110 - t112 * t111;
    real_type t1037 = 8 * t1036;
    real_type t1039 = 8 * t146;
    real_type t1040 = t109 * t1037 - t1039;
    real_type t1044 = -t109 * t1037 + t1039;
    real_type t1046 = t5 * t1040 + t14 * t1044;
    real_type t1047 = t89 * t1046;
    real_type t1050 = t111 * t110;
    real_type t1052 = t111 * t111;
    real_type t1055 = 4 * t117 - 8 * t1050 + 4 * t1052 - 4 * t256;
    real_type t1057 = 4 * t119;
    real_type t1058 = 4 * t120;
    real_type t1059 = 4 * t121;
    real_type t1060 = t109 * t1055 + t1057 - t1058 + t1059;
    real_type t1064 = -t109 * t1055 - t1057 + t1058 - t1059;
    real_type t1068 = t124 * t106;
    real_type t1073 = 4 * t162;
    real_type t1075 = 4 * t166;
    real_type t1078 = 4 * t951;
    real_type t1081 = 4 * t172;
    real_type t1084 = t179 * t174 + t179 * t93;
    real_type t1085 = 4 * t1084;
    real_type t1087 = 4 * t185;
    real_type t1088 = 4 * t186;
    real_type t1091 = 4 * t1036;
    real_type t1093 = 4 * t146;
    real_type t1094 = t109 * t1091 - t1093;
    real_type t1095 = t14 * t1094;
    real_type t1096 = t311 * t462;
    real_type t1097 = 2 * t1096;
    real_type t1098 = t173 * t462;
    real_type t1099 = 2 * t1098;
    real_type t1104 = t2 * t1064;
    real_type t1105 = t212 * t89;
    real_type t1108 = t124 * t215;
    real_type t1113 = -t1073;
    real_type t1115 = 4 * t326;
    real_type t1116 = 2 * t328;
    real_type t1117 = 4 * t331;
    real_type t1120 = 4 * t546 - 2 * t548;
    real_type t1123 = 4 * t336;
    real_type t1128 = 2 * t235;
    real_type t1130 = t141 * t151 * t1044 + t142 * t1060 + t227 * t1113 + t92 * t1120 + 2 * t338 * t182 - t1115 + t1116 - t1117 + t1123 + t1128 + 4 * t159 + 4 * t160 - 4 * t161 + t246 - 2 * t251;
    real_type t1137 = t14 * t1060 + t5 * t1064;
    real_type t1138 = t2 * t1137;
    real_type t1143 = 2 * t117;
    real_type t1144 = 4 * t1050;
    real_type t1145 = 2 * t1052;
    real_type t1147 = -t1143 + t1144 - t1145 + 2 * t256;
    real_type t1149 = 2 * t119;
    real_type t1150 = 2 * t120;
    real_type t1151 = 2 * t121;
    real_type t1152 = t109 * t1147 - t1149 + t1150 - t1151;
    real_type t1153 = t14 * t1152;
    real_type t1156 = t17 * t92 * t979 + t5 * t1130 + t1153;
    real_type t1162 = -t109 * t1091 + t1093;
    real_type t1163 = t612 * t1162;
    real_type t1170 = -t109 * t1147 + t1149 - t1150 + t1151;
    real_type t1177 = -t979;
    real_type t1183 = t142 * t1094 + t141 * t151 * t1170 + (-t244 * t155 - t299) * t154 + t173 * t92 * t1177 - t302 - 2 * t92 * t1084 + t305 + t299 + t306;
    real_type t1209 = t142 * t1040 + t141 * t151 * t1060 + (t1113 * t155 - t1075) * t154 - t173 * t92 * t1078 - t1081 - t92 * t1085 + t1087 + t1075 + t1088;
    real_type t1229 = t142 * t1152;
    real_type t1231 = t141 * t151 * t1094;
    real_type t1232 = t1229 + t1231 + t245 + t544 - t328 + t545 - t232 + t550 - t248 - t551 + t553 - t249 + t250 - t235 + t251;
    real_type t1241 = 2 * t409;
    real_type t1243 = t89 * (t5 * t101 * t696 + t101 * t724 - t1241);
    real_type t1245 = 2 * t436;
    real_type t1246 = 2 * t438;
    real_type t1247 = t8 * t692;
    real_type t1255 = t141 * t692 + t708 + t709 - t710;
    real_type t1258 = 2 * t463;
    real_type t1259 = 2 * t461;
    real_type t1260 = t8 * t696;
    real_type t1264 = 2 * t154 * t8 * t164;
    real_type t1266 = -2 * t388 - 2 * t183 - 2 * t164 + 2 * t332;
    real_type t1272 = t124 * t718;
    real_type t1273 = t112 * t434;
    real_type t1276 = 4 * t143;
    real_type t1278 = 4 * t380;
    real_type t1279 = t109 * t1276 + t1278;
    real_type t1281 = t106 * t718;
    real_type t1286 = -t109 * t1276 - t1278;
    real_type t1296 = t93 * t174;
    real_type t1297 = t93 * t93;
    real_type t1298 = -t1296 - t1297;
    real_type t1299 = 2 * t1298;
    real_type t1300 = t92 * t1299;
    real_type t1301 = 2 * t476;
    real_type t1302 = 2 * t478;
    real_type t1303 = 2 * t418;
    real_type t1304 = 2 * t266;
    real_type t1305 = 2 * t420;
    real_type t1306 = 2 * t422;
    real_type t1307 = t1300 - t1301 - t1302 + t1303 - t1304 + t1305 - t1306;
    real_type t1311 = t17 * t99 * t94;
    real_type t1315 = 2 * t94 * t11 * t173;
    real_type t1317 = 2 * t360 - 2 * t363 - 2 * t364;
    real_type t1323 = t124 * t2;
    real_type t1325 = t106 * t2;
    real_type t1327 = 2 * t409 * t1325;
    real_type t1332 = t141 * t1247;
    real_type t1335 = 2 * t151 * t8 * t402;
    real_type t1337 = 2 * t450 - 2 * t453;
    real_type t1338 = t8 * t1337;
    real_type t1346 = 2 * t511;
    real_type t1352 = t402 * t801;
    real_type t1353 = t101 * t381;
    real_type t1354 = t612 * t1353;
    real_type t1358 = t101 * t727 + t371 * t724 + t446 * t405 - t1352 + t1354;
    real_type t1360 = t47 * t381;
    real_type t1361 = t89 * t1360;
    real_type t1363 = t402 * t101 * t726;
    real_type t1365 = 2 * t1273 * t735;
    real_type t1366 = t402 * t842;
    real_type t1368 = t402 * t101 * t446;
    real_type t1373 = t47 * t385;
    real_type t1379 = -t183 - t164 + t332;
    real_type t1383 = t612 * t455;
    real_type t1391 = 2 * t92 * t101 * t755;
    real_type t1393 = ModelPars[158] * t800;
    real_type t1399 = t50 * t381;
    real_type t1401 = t89 * (t5 * t101 * t692 + t101 * t697 + t1241 + t1399);
    real_type t1404 = t112 * t50 * t109;
    real_type t1408 = -t365;
    real_type t1410 = 2 * t99 * t1408;
    real_type t1414 = -t1241 + t494;
    real_type t1419 = -t8 * t1266;
    real_type t1467 = 2 * t409 * t1323;
    real_type t1474 = -t1303 + t266 - t420 + t422 - t424;
    real_type t1479 = t89 * t1373;
    real_type t1492 = t612 * t526;
    real_type t1494 = t266 * t99 * t135;
    real_type t1502 = ModelPars[147] * t800;
    real_type t1516 = t1260 + t1404;
    real_type t1531 = t5 * t1162 + t1095;
    real_type t1539 = -t98;
    real_type t1544 = 2 * t283;
    real_type t1546 = -2 * t1036;
    real_type t1548 = 2 * t146;
    real_type t1549 = t109 * t1546 + t1548;
    real_type t1561 = 2 * t226 + t1229 + t1231 + t245 - t246 + t247 - t248 - t249 + t250 - t1128 + 2 * t236;
    real_type t1576 = t801 * t100;
    real_type t1579 = t337 * t182;
    real_type t1581 = -t92 * t1120 + t1115 - t1116 + t1117 - t1123 + t1229 + t1231 - 2 * t1579 - t235 - t236 + t245 - t248 - t249 + t250 + t251;
    real_type t1597 = t117 - 2 * t1050 + t1052 - t256;
    real_type t1604 = t182 * t89 * t47 * t101;
    real_type t1620 = t799 * t101 * t798;
    real_type t1626 = -t918 * t137 - t899 * t194 - t688 * t199 - t1032 + t926 + t1033 - t570 * (t88 * (t212 * t1047 + t1068 * t89 * (t5 * t1060 + t14 * t1064) + t89 * (t5 * (t142 * t1044 + t141 * t151 * t1064 + t154 * (t155 * t1073 + t1075) + t173 * t92 * t1078 + t1081 + t92 * t1085 - t1087 - t1075 - t1088) + t1095 + t1097 - t1099)) + t87 * (t207 * (t1108 * t2 * t1040 + t89 * t2 * t1130 + t1105 * t1104) + t212 * t1138 + t1068 * t2 * t1046 + t2 * t1156) + t207 * (t1068 * t612 * t1152 + t212 * t1163 + t612 * t1183)) - t472 * (t355 * (t88 * (t212 * t89 * t1137 + t1068 * t1047 + t89 * t1156) + t87 * (t207 * (t1105 * t2 * t1044 + t89 * t2 * t1209 + t1108 * t1104) + t212 * t2 * (t14 * t1040 + t5 * t1044) + t1068 * t1138 + t2 * (t14 * t1162 + t5 * t1209 - t1097 + t1099)) + t207 * (t212 * t612 * t1170 + t1068 * t1163 + t612 * t1232)) + t88 * (t124 * t1243 + t106 * t89 * (-t1245 + t1246 + t1247) + t89 * (t5 * t101 * t1255 + 4 * t2 * t99 * t365 + t151 * t1260 + t8 * t1266 - t1258 + t1259 + t1264)) + t87 * (t207 * (4 * t1273 * t1272 + t1281 * t101 * t1279 + t89 * (t2 * t101 * (t151 * t1286 + 4 * t164 + 4 * t183 - 4 * t332 - 4 * t384 - 4 * t387 + 4 * t388) + t5 * t99 * t1307 - 2 * t1311 - t1315 + t11 * t1317)) + t1323 * t1260 - t1327 + t612 * t99 * (t92 * t1298 - t266 + t418 + t420 - t422 - t476 - t478) + t2 * (t1332 + t1335 + t1338) + t446 * t99 * (-t92 * t1298 + t266 - t418 - t420 + t422 + t476 + t478) + t5 * t99 * (t1346 - t267)) + t207 * (t124 * t1358 + t106 * (t1361 - t1363 + t1365 + t1366 - t1368) + t89 * (t402 * t47 * t141 + t164 * t47 * t154 - t92 * t47 * t173 + t151 * t1373 + t47 * t1379) + t1383 + t2 * t11 * (t1300 - t1301 - t1302 - t266 + t420 - t422 - t424) + t446 * t101 * t752 - t1391 + t1393) + t124 * t1401 + t106 * t89 * (t1245 - t1246 + t1260 + t1404) + t89 * (t2 * t1410 + t5 * t101 * t711 + t1258 + t141 * t1414 + t151 * (t1247 - t1404) - t1264 + t1419 - t164 * t50 * t155 + t184 * t50 * t182)) - t355 * (t88 * (t124 * t89 * (t1245 - t1246 + t1260) + t106 * t1243 + t89 * (t2 * t99 * (4 * t92 * t1298 - 4 * t266 + 4 * t418 + 4 * t420 - 4 * t422 - 4 * t476 - 4 * t478) + t5 * t101 * t748 + t1332 + t1335 + t1338)) + t87 * (t207 * (t1272 * t101 * t1286 + 4 * t1273 * t1281 + t89 * (t2 * t101 * (t141 * t1279 - 4 * t450 - 4 * t452 + 4 * t453) + t5 * t1410 + t11 * t1307)) + t1467 + t1325 * t1260 + t612 * t99 * t1408 + t2 * (t151 * t1247 - t1259 - t1264 + t1419) + t448 + t5 * t11 * t1474 - t470) + t207 * (t124 * (t1479 + t1363 - t1365 - t1366 + t1368) + t106 * t1358 + t89 * (t151 * t47 * t402 + t155 * t47 * t164 - t182 * t47 * t184 + t141 * t1360) + t1492 + t2 * (-t11 * t1317 + t1315 + t1494) + t446 * t101 * t778 + t1502) + t124 * t89 * (-t1245 + t1246 + t1247 - t1404) + t106 * t1401 + t89 * (t2 * t99 * (-t92 * t1299 + t1301 + t1302 - t1303 + t1304 - t1305 + t1306) + t5 * t101 * t765 + t141 * t1516 + t151 * t1414 + t164 * t50 * t154 - t8 * t1337 - t92 * t50 * t173 + t50 * t1379)) - t88 * (t212 * t89 * t1531 + t1068 * t89 * (t5 * t1152 + t14 * t1170) + t89 * (4 * t852 * t99 * t1539 + t5 * t1183 + t14 * t1549 - t1544)) - t87 * (t207 * (t1105 * t2 * t1170 + t1108 * t2 * t1162 + t89 * (t99 * t8 * t793 + t101 * t794 + t2 * t1561)) + t212 * t2 * (t5 * t1170 + t1153) + t1068 * t2 * t1531 + t1576 + t2 * (t5 * t1581 + t14 * (t109 * (t1143 - t1144 + t1145) + t1149 + t260 - t120 + t259 + t121) + t267 + t268)) - t207 * (t212 * t612 * (-t109 * t1546 - t1548) + t1068 * t612 * (t109 * t1597 + t119 - t120 + t121) - t1604 + t612 * (t142 * t1549 + t141 * t151 * (-t109 * t1597 - t119 + t120 - t121) + t168 + t173 * t92 * t951 + t172 + t92 * t1084 - t185 - t166 - t186) + t5 * (-2 * t1098 + 2 * t1096) + t1620) - t89 * (t852 * t784 - t1097 + t1099 + t1544);
    real_type t1635 = t355 * (t101 * t87 - t128);
    real_type t1636 = t87 * t174;
    real_type t1649 = t92 * t1296;
    real_type t1650 = t92 * t1297;
    real_type t1653 = t96 * (t174 + t96) * t95;
    real_type t1654 = t1649 + t1650 + t1653 - t420 + t422 - t418 + t266;
    real_type t1655 = t90 * t1654;
    real_type t1657 = t1408 * t2;
    real_type t1675 = t129 * t131;
    real_type t1680 = t2 * t98;
    real_type t1681 = t1680 * t576;
    real_type t1697 = t5 * (t1649 + t1650 + t1653 - t421 + t423 + t425 + t419) + t511 - t512;
    real_type t1735 = t5 * t1654;
    real_type t1753 = -t137 * (t472 * (t207 * t128 + t87 * t918) + t133 * (t1635 + t1636)) - t194 * (t472 * (-t207 * t902 + t87 * t899) + t1635 + t1636) + t87 * t926 * t472 - t570 * t99 * (-t1654 * t612 + t88 * t1655 - 2 * t731 * t1657) - t472 * (-2 * t355 * t99 * (t90 * t1408 * t88 / 2 + t731 * t1654 * t2 - t1408 * t612 / 2) - t207 * t1675 - 2 * t87 * (-t207 * t1408 * t89 / 2 + t1681 + t1408 * t5) * t89 + t207 * (-2 * t100 * t8 * t89 + t89 * t1474 * t2 + t970) + t1576 - 2 * t1697 * t2) - t355 * (t129 * (t101 * t207 + t132) - t103 + t87 * (t90 * t1654 * t207 - 2 * t1697 * t89 - t272) + t207 * t100 * t843 + t100 * (t90 + t612) * t101 + 2 * t5 * t1657 + t192) - t207 * t129 * t174 + 2 * t88 * (t1650 + t1649 / 2 + t96 * (t96 + t174 / 2) * t95 - t420 + t422 - t418 + t266) * t99 * t90 - t87 * (t369 + t50) * t275 - t207 * (-2 * t99 * (t1735 + t511 - t512) * t89 + t271 * t98) - t99 * (t1655 + t612 * t98 * t174 + t5 * (t1735 + t1346 - t267)) + 2 * t8 * t1680 - t11 * ModelPars[162] + t82;
    real_type t1756 = t2 - t42;
    real_type t1757 = t1756 * t1002;
    real_type t1761 = t2 - t42 - t581;
    real_type t1762 = t2 - t42 + t581;
    real_type t1763 = t1762 * t1761;
    real_type t1766 = t132 * t612;
    real_type t1768 = t586 * t1001;
    real_type t1771 = -2 * t128 * t595 - 2 * t581 * t1768 - 2 * t26;
    real_type t1775 = 2 * t581 * t586 * t23;
    real_type t1776 = t132 * t607;
    real_type t1785 = t612 * (t1001 + t128);
    real_type t1789 = t2 * (2 * t132 * t595 + 2 * t23);
    real_type t1792 = 2 * t581 * t586 * t26;
    real_type t1793 = t1001 * t607;
    real_type t1794 = t128 * t607;
    real_type t1799 = t11 * t586;
    real_type t1815 = t2 * t42 * t1009 + t612 * t1002 + t1004 + t1006;
    real_type t1819 = -2 * t586 * t1000 + 2 * t1768;
    real_type t1820 = t581 * t1819;
    real_type t1823 = -t581 * t1819;
    real_type t1845 = t207 * t199;
    real_type t1846 = t196 * t87;
    real_type t1851 = t124 * t472;
    real_type t1856 = t197 * t110;
    real_type t1864 = t197 * t198;
    real_type t1865 = t87 * t1864;
    real_type t1866 = t207 * t1033;
    real_type t1872 = t581 * t1799;
    real_type t1873 = 2 * t1872;
    real_type t1877 = t99 * t595 + t11;
    real_type t1888 = t2 * t207;
    real_type t1896 = t2 * t42;
    real_type t1902 = -t1756;
    real_type t1939 = -t1762;
    real_type t1941 = -t1761;
    real_type t1958 = t612 * t402 + t733 + t734 - t737 + t738;
    real_type t1968 = t2 * t724 + t5 * t730;
    real_type t1975 = 2 * t2 * t14 * t402 - 2 * t5 * t2 * t402;
    real_type t1989 = 2 * t14 * t89 * t402 - 2 * t5 * t89 * t402;
    real_type t2001 = t90 * t381;
    real_type t2029 = t612 * t790;
    real_type t2034 = t99 * t1539;
    real_type t2035 = t612 * t2034;
    real_type t2037 = 2 * t2 * t282;
    real_type t2050 = 2 * t216 * t1068 + t240 * t212 + t159 + t160 - t161 + t221 - t224 + t229 + t231 - t326 + t329 - t331 + t335 + t336 + t340 + t341 - t342;
    real_type t2052 = t88 * t2;
    real_type t2055 = 2 * t561;
    real_type t2056 = t5 * t187;
    real_type t2058 = -t2055 - t564 + t2056 + t284 + t314 / 2;
    real_type t2062 = t89 + t2;
    real_type t2065 = t106 * t563 + t309 * t212 + t149 - t152 - t166 + t168 + t172 - t177 + t180 - t185 - t186;
    real_type t2067 = t89 - t2;
    real_type t2074 = -2 * t320 + 4 * t323 + t555 + t556 + t264;
    real_type t2096 = t2 * t385;
    real_type t2102 = t365 * t11;
    real_type t2108 = -t410 / 2;
    real_type t2111 = -t440 / 2;
    real_type t2123 = t14 + t2 - t5;
    real_type t2125 = t14 - t2 - t5;
    real_type t2128 = t2125 * t101 * t2123 * t385 + t90 * t405 - t1352;
    real_type t2130 = t402 * t835;
    real_type t2134 = -t2123 * t2125 * t112 * t434 - t1479 - t2130;
    real_type t2213 = t99 * t8;
    real_type t2263 = -t142 * t345 - t223 + t228 * t227 / 2 - t326 + t329 - t331 + t335 + t1579 / 2 + t159 / 2 + t336 + t235 / 4 + t236 / 4 - t251 / 4 + t160 / 2 - t161 / 2;
    real_type t2265 = t143 * t143;
    real_type t2288 = t138 - t133 * t195 + t688 * t196 - t570 * (4 * t2052 * t89 * t2050 + t87 * (-4 * t207 * t2058 * t89 + 2 * t2067 * t2065 * t2062) + 2 * t2074 * t1888) - t472 * (t355 * (-4 * t2052 * t89 * t2065 + t87 * (2 * t2067 * t2062 * t2050 + 2 * t207 * t2074 * t89) + 4 * t207 * t2 * t2058) - 2 * t88 * (-2 * t106 * t101 * t2096 - 2 * t1273 * t1323 + t5 * t480 + t1311 - t2102 + 2 * t527) * t89 + t87 * (-4 * t207 * (t124 * t2108 + t106 * t2111 + t2 * t366 - t493 / 2 + t461 / 2 - t463 / 2 + t465 / 2) * t89 + t124 * t2128 + t106 * t2134 - t454 * t835 + t89 * t47 * t395 + t1383 - 2 * t2 * (t474 + t476 + t419 - t421 + t423 + t425 + t478) * t11 - t446 * t455 - t1391 + t1393) + t207 * (-2 * t124 * t8 * t2096 + t1327 - t523 + t612 * t480 + t535 - t5 * t99 * (t510 + t1346 - t267)) + t124 * t415 - t509 + t516 - t517) - t355 * (-2 * t88 * (2 * t124 * t101 * t2096 + t479 * t11 - 2 * t1273 * t1325 + t417 - 2 * t456) * t89 + t87 * (4 * t207 * (t124 * t2111 - t106 * t2108 + t2 * t480 + t5 * t526 / 2 + t533 / 2) * t89 - t124 * t2134 + t106 * t2128 - t90 * t526 - t454 * t801 + t1492 + t2 * (t1494 - 2 * t2102) - t446 * t526 + t1502) + t207 * (-2 * t106 * t8 * t2096 + 2 * t2 * t8 * t395 + t612 * t366 + 2 * t5 * t427 - t1467 - t445 - t448 + t470) + t402 * t271 * t124 + t416 - t89 * t429 - t431) - t1675 + t1864 + 2 * t88 * (-t212 * t123 + 2 * t1323 * t321 + t2 * t237 + t98 * (t101 * t11 + t2213)) * t89 - t87 * (2 * t207 * (-t2055 - t564 + 2 * t1681 + t2056 + t283 + t284) * t89 - 2 * t212 * t2067 * t2062 * t216 - t124 * t2067 * t106 * t122 * t2062 + t90 * (t309 * t142 + t152 + (t228 * t155 - t166) * t154 + t177 - t172 - t180 + t185 + t166 + t186) - t1604 + t612 * t187 + 2 * t5 * t312 * t462 + t1620) - t207 * (2 * t212 * t107 * t123 - 4 * t124 * t107 * t2 * t321 - t1576 + 4 * t2 * (t5 * t2263 + t14 * (-t2265 * t109 / 2 - t260 / 4 - t259 / 4 - t119 / 2 + t120 / 4 - t121 / 4) - t268 / 4 - t267 / 4)) - t89 * (t2 * (t235 - t236 + t251) + 2 * t98 * t2213) + t970;
    real_type t2295 = t112 * t379 + t146;
    real_type t2297 = t88 * t90 * t2295;
    real_type t2302 = -t115 * t113 * t109 - t119 + t120 - t121;
    real_type t2304 = t87 * t2;
    real_type t2305 = t2304 * t89 * t2302;
    real_type t2306 = t612 * t2295;
    real_type t2307 = -t2297 + t2305 + t2306;
    real_type t2311 = t88 * t90 * t2302;
    real_type t2313 = t2304 * t2295 * t89;
    real_type t2314 = 4 * t2313;
    real_type t2315 = t612 * t2302;
    real_type t2318 = t124 * (t2311 + t2314 - t2315) * t106;
    real_type t2336 = t2001 * t101 * t88;
    real_type t2342 = 2 * t109 * t718 * t112 * t101 - t271 * t381;
    real_type t2346 = t207 * (t1241 + t1399) * t89;
    real_type t2347 = t90 * t1353;
    real_type t2361 = t90 * t434 * t112 * t88 + t87 * (t109 * t112 * t271 + 2 * t718 * t1353) - t207 * t89 * t1516 - t2130 - t1361 - t1366;
    real_type t2379 = alpha__crw(t191);
    real_type t2380 = sin(t2379);
    real_type t2389 = -t688 * (t1845 - t1846 + t101) - t207 * t1032 + t1866 - t570 * (4 * t212 * t2307 + 2 * t2297 - 2 * t2305 - 2 * t2306 - 2 * t2318) - t472 * (t355 * (t212 * (2 * t2311 + 8 * t2313 - 2 * t2315) + 4 * t124 * t106 * t2307 - t2311 - t2314 + t2315) + t124 * (t87 * t2342 - t1352 + t1354 - t2336 + t2346 + t2347) - t2361 * t106) - t355 * (t124 * t2361 - (-t87 * t2342 + t1352 - t1354 + t2336 - t2346 - t2347) * t106) + t1865 + 2 * t212 * t2307 - t2318 + t2297 + t87 * t261 * t718 - t2380 * t110 * t74 - t2306 + t75 - (ModelPars[74] * t105 + t14 * ModelPars[35]) * ModelPars[32];
    real_type t2392 = t107 * t107;
    real_type t2393 = t612 + t2392;
    real_type t2394 = t109 * t2393;
    real_type t2396 = t381 * t2393;
    real_type t2402 = t381 * t107;
    real_type t2405 = t5 * t752;
    real_type t2435 = X__[37];
    real_type t2436 = Q__[1];
    real_type t2438 = t2436 * t2435 - 1;
    real_type t2439 = t50 * t46;
    real_type t2440 = t47 * t49;
    real_type t2442 = 1.0 / (t2439 - t2440);
    real_type t2443 = t2442 * t2438;
    real_type t2446 = roadRightLateralBorder(Q__[3] - t2435);
    real_type t2448 = (-t355 * (t981 * t586 * t979 + t813 * t446 - t983 - t985 + t986) - t472 * t998 - t938 * (t133 * (t1004 + t1006) + t38 * t581 * t1010) - t933 * (t133 * t42 * t581 * t1010 + t42 * t38 * t1009) + t607 * t131 + 2 * t581 * t586 * t20) * L__[13] + t1626 * L__[5] + t1753 * L__[6] + (-t87 * (-t938 * t1002 * t1763 - 2 * t933 * t1757 * t595 + t2 * t1771 + t1766 - t1775 - t1776) - t207 * (t933 * t1002 * t1763 - 2 * t938 * t1757 * t595 + t1785 + t1789 - t1792 - t1793 - t1794) - (t581 * t99 + 2 * t1799) * t581 * t93) * L__[14] + (-t472 * (t981 * t586 * t1177 + t312 * t446 + t983 + t985 - t986) - t355 * t998 - t87 * (t933 * t1815 + t938 * (t2 * t1820 + t42 * t1823) + t1785 + t1789 - t1794 - t1793 - t1792) - t207 * (t933 * (t42 * t1820 + t2 * t1823) + t938 * t1815 - t1766 - t2 * t1771 + t1776 + t1775) - t680 - t679) * L__[15] + (-t688 * (t110 * t106 * t472 - t110 * t124 * t355 + t101 + t1845 - t1846) - t202 * (-t110 * t106 * t355 - t110 * t1851 + t198) * t207 - t87 * t1851 * t1856 - t355 * t87 * t106 * t1856 - t87 * t260 * t718 + t1865 + t1866 + t75 + t65) * L__[10] + (-t472 * (t99 * (t446 - t607) - t1873) - 2 * t355 * t1877 * t5 - t87 * (t133 * (-t612 + t607) + 2 * t581 * t586 * t38) + 2 * (t133 * t595 + t38) * t1888) * L__[11] + (-t355 * (t933 * (t87 * (t99 * (t612 - 2 * t1896 + t446 + t1003 - t607) - t1873) - 2 * t1877 * t1902 * t207) + 2 * t938 * (t87 * t1877 * t1902 - t207 * (t99 * (-t446 / 2 - t1003 / 2 + t1896 + t607 / 2 - t612 / 2) + t1872))) + 2 * t472 * (t933 * (t87 * t1877 + t1902 * t838) + (-t87 * t1902 * t99 + t1877 * t207) * t938) * t5 - t933 * (-2 * t87 * t1902 * t595 - t1941 * t1939 * t207) - t938 * (-2 * t1902 * t581 * t586 * t207 + t87 * t1941 * t1939)) * L__[12] + (t133 * t137 + t194 + t202 - t472 * (t207 * (t124 * t1958 + t106 * (t612 * t385 + t722 + t723 + t725 + t727) + t780 + t612 * t778 + t779) + t87 * (t5 * t2 * t1255 - 2 * t17 * t2 * t92 + t106 * t1975 + t124 * t1968) + t124 * t1989 + t106 * (t5 * t89 * t692 + t89 * t697) + t767) - t355 * (t207 * (t124 * (t446 * t381 + t612 * t381 + t726 * t381 + t5 * t697 + t2001) + t106 * t1958 + t446 * t454 + t757 + t612 * t454 + t90 * t454) + t87 * (t106 * t1968 - t124 * t1975 + t5 * t749) + t124 * (t5 * t89 * t696 + t89 * t724) + t106 * t1989 + t5 * t89 * t1255 - 2 * t17 * t870) - t207 * (t182 * t835 + t2 * t794 + t2029) - t87 * (t90 * t2034 + t2035 - t2037) - t182 * t843) * L__[1] + t2288 * L__[4] + t2389 * L__[7] + (-t472 * (t87 * (-t375 * t2394 + t106 * t2396 + (t612 + t446) * t395) + 2 * (t108 * t402 + t124 * t2402 + t2405 - t462) * t1888) - t355 * (t87 * (-t124 * t2396 - t491 * t2394 + t612 * t752 + t5 * (t2405 - t713)) - 2 * (-t106 * t2402 + t322 * t402 + t395 * t5) * t1888) + t129 + t197 - t87 * (t2 * t11 * t783 - t2029) - t207 * (t2035 - t2037) - t182 * t271) * L__[2] - t2446 * t2443;
    real_type t2452 = pow(t2435 - ModelPars[81], 2);
    real_type t2457 = pow(t50 - ModelPars[136], 2);
    real_type t2460 = pow(t47 - ModelPars[138], 2);
    real_type t2463 = pow(t89 - ModelPars[172], 2);
    real_type t2466 = pow(t86 - ModelPars[97], 2);
    real_type t2481 = MaxRollAngle(t86);
    real_type t2483 = MaxSteerAngle(t99);
    real_type t2486 = 1.0 / ModelPars[168];
    real_type t2488 = OnlyTractionRear(t74 * t2486);
    real_type t2491 = OnlyBrakingFront(-t58 * t2486);
    real_type t2494 = OnlyBrakingRear(-t65 * t2486);
    real_type t2496 = LatSlipFront(t134);
    real_type t2498 = LongSlipFront(t136);
    real_type t2500 = ModelPars[9];
    real_type t2501 = 1.0 / t2500;
    real_type t2504 = FrontWheelContact((t129 - t2500) * t2501);
    real_type t2506 = LongSlipRear(t201);
    real_type t2508 = LatSlipRear(t200);
    real_type t2512 = RearWheelContact((t197 - t2500) * t2501);
    real_type t2516 = roadLeftLateralBorder(t2435 + Q__[2]);
    real_type t2518 = -t2442 * t2438 * (ModelPars[144] + ModelPars[140] * t2452 + ModelPars[141] * (t2457 + t2460 + t2463 + t2466)) + 1.0 / t2438 * (t2436 * (t89 * t2435 + t2439 - t2440) - t89) * L__[38] - t2481 * t2443 - t2483 * t2443 - t2488 * t2443 - t2491 * t2443 - t2494 * t2443 - t2496 * t2443 - t2498 * t2443 - t2504 * t2443 - t2506 * t2443 - t2508 * t2443 - t2512 * t2443 - t2516 * t2443;
    return t40 + t975 + t2448 + t2518;
  }

  /*\
   |   ___               _ _   _
   |  | _ \___ _ _  __ _| | |_(_)___ ___
   |  |  _/ -_) ' \/ _` | |  _| / -_|_-<
   |  |_| \___|_||_\__,_|_|\__|_\___/__/
  \*/

  real_type
  Straight::penalties_eval(
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
    real_type t16  = 1.0 / ModelPars[168];
    real_type t19  = OnlyBrakingFront(-X__[34] * t16);
    real_type t23  = OnlyBrakingRear(-X__[35] * t16);
    real_type t27  = OnlyTractionRear(X__[33] * t16);
    real_type t29  = ModelPars[9];
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
  Straight::control_penalties_eval(
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
    real_type t18  = t__oControl(U__[2], ModelPars[173], ModelPars[197]);
    real_type t21  = ModelPars[191];
    real_type t22  = ModelPars[33];
    real_type t23  = b__f__oControl(U__[0], t21, t22);
    real_type t26  = b__r__oControl(U__[1], t21, t22);
    real_type t29  = ModelPars[208];
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
  Straight::lagrange_target(
    NodeType const     & NODE__,
    U_const_pointer_type U__,
    P_const_pointer_type P__
  ) const {
    integer     i_segment = NODE__.i_segment;
    real_type const * Q__ = NODE__.q;
    real_type const * X__ = NODE__.x;
    Road2D::SegmentClass const & segment = pRoad->getSegmentByIndex(i_segment);
    real_type t2   = X__[37];
    real_type t5   = pow(t2 - ModelPars[81], 2);
    real_type t8   = X__[0];
    real_type t11  = pow(t8 - ModelPars[136], 2);
    real_type t12  = X__[1];
    real_type t15  = pow(t12 - ModelPars[138], 2);
    real_type t19  = pow(X__[2] - ModelPars[172], 2);
    real_type t23  = pow(X__[3] - ModelPars[97], 2);
    real_type t32  = X__[38];
    real_type t33  = cos(t32);
    real_type t35  = sin(t32);
    return -1.0 / (-t12 * t35 + t8 * t33) * (Q__[1] * t2 - 1) * (ModelPars[144] + ModelPars[140] * t5 + ModelPars[141] * (t11 + t15 + t19 + t23));
  }

  /*\
   |   __  __
   |  |  \/  |__ _ _  _ ___ _ _
   |  | |\/| / _` | || / -_) '_|
   |  |_|  |_\__,_|\_, \___|_|
   |               |__/
  \*/

  real_type
  Straight::mayer_target(
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
    real_type t4   = pow(XL__[33] - ModelPars[5], 2);
    real_type t8   = pow(XL__[34] - ModelPars[27], 2);
    real_type t12  = pow(XL__[35] - ModelPars[29], 2);
    real_type t14  = XL__[36] * XL__[36];
    return ModelPars[139] * (t4 + t8 + t12 + t14);
  }

  /*\
   |    ___
   |   / _ \
   |  | (_) |
   |   \__\_\
  \*/

  integer
  Straight::q_numEqns() const
  { return 13; }

  void
  Straight::q_eval(
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
  Straight::u_guess_numEqns() const
  { return 4; }

  void
  Straight::u_guess_eval(
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
  Straight::u_guess_eval(
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
  Straight::u_check_if_admissible(
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
    ok = ok && t__oControl.check_range(U__[2], ModelPars[173], ModelPars[197]);
    real_type t5   = ModelPars[191];
    real_type t6   = ModelPars[33];
    ok = ok && b__f__oControl.check_range(U__[0], t5, t6);
    ok = ok && b__r__oControl.check_range(U__[1], t5, t6);
    real_type t9   = ModelPars[208];
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
  Straight::post_numEqns() const
  { return 38; }

  void
  Straight::post_eval(
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
    result__[ 0   ] = t__oControl(U__[2], ModelPars[173], ModelPars[197]);
    real_type t5   = ModelPars[191];
    real_type t6   = ModelPars[33];
    result__[ 1   ] = b__f__oControl(U__[0], t5, t6);
    result__[ 2   ] = b__r__oControl(U__[1], t5, t6);
    real_type t9   = ModelPars[208];
    result__[ 3   ] = tau__oControl(U__[3], -t9, t9);
    real_type t11  = 1.0 / ModelPars[168];
    result__[ 4   ] = OnlyBrakingFront(-X__[34] * t11);
    result__[ 5   ] = OnlyBrakingRear(-X__[35] * t11);
    result__[ 6   ] = OnlyTractionRear(X__[33] * t11);
    real_type t18  = ModelPars[9];
    real_type t19  = 1.0 / t18;
    real_type t22  = Fzf(X__[11], X__[27]);
    result__[ 7   ] = FrontWheelContact((t22 - t18) * t19);
    real_type t27  = Fzr(X__[14], X__[30]);
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
    real_type t49  = alpha__r(t30, t32, t33, X__[12], t42, t44, X__[29]);
    result__[ 12  ] = LatSlipRear(t49);
    result__[ 13  ] = MaxSteerAngle(X__[6]);
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
    #ifdef MECHATRONIX_DEBUG
    CHECK_NAN(result__,"post_eval",38);
    #endif
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  integer
  Straight::integrated_post_numEqns() const
  { return 1; }

  void
  Straight::integrated_post_eval(
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

// EOF: Straight_Methods1.cc

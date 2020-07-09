/*-----------------------------------------------------------------------*\
 |  file: General_Methods1.cc                                            |
 |                                                                       |
 |  version: 1.0   date 29/6/2020                                        |
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
    real_type t1   = ModelPars[138];
    ModelPars[137] = t1 + (ModelPars[139] - t1) * s;
    real_type t5   = ModelPars[135];
    ModelPars[134] = t5 + (ModelPars[136] - t5) * s;
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
    real_type t1   = ModelPars[129];
    ModelPars[128] = t1 + (ModelPars[130] - t1) * s;
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
    real_type t1   = ModelPars[132];
    ModelPars[131] = t1 + (ModelPars[133] - t1) * s;
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
    real_type t26  = Omega__XO * delta__f__XO;
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
    real_type t19  = Omega__XO * delta__f__XO;
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
    real_type t11  = Fzf__XO * ModelPars[239] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t14  = ModelPars[77] * ModelPars[165];
    real_type t24  = exp(t5 * t3 * ModelPars[87]);
    real_type t34  = atan(lambda__f__XO / (t11 * t14 + ModelPars[161]) * ModelPars[167] * t24 * (t5 * t3 * ModelPars[85] + ModelPars[83]) * Fzf__XO);
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
    real_type t5   = ModelPars[239];
    real_type t9   = ModelPars[77] * ModelPars[165];
    real_type t10  = ModelPars[85];
    real_type t11  = Fzf__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[83];
    real_type t16  = t15 * Fzf__XO;
    real_type t17  = ModelPars[87];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[167];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[79]);
    real_type t29  = Fzf__XO * t27 * t9;
    real_type t31  = t29 + ModelPars[161];
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
    real_type t5   = ModelPars[239];
    real_type t6   = t5 * t4;
    real_type t7   = ModelPars[77];
    real_type t8   = ModelPars[165];
    real_type t9   = t8 * t7;
    real_type t10  = ModelPars[85];
    real_type t11  = Fzf__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[83];
    real_type t16  = t15 * Fzf__XO;
    real_type t17  = ModelPars[87];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[167];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[79]);
    real_type t29  = Fzf__XO * t27 * t9;
    real_type t31  = t29 + ModelPars[161];
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
    real_type t5   = ModelPars[239];
    real_type t6   = t5 * t3 * t1;
    real_type t7   = ModelPars[77];
    real_type t8   = ModelPars[165];
    real_type t9   = t8 * t7;
    real_type t10  = ModelPars[85];
    real_type t11  = Fzf__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[83];
    real_type t16  = t15 * Fzf__XO;
    real_type t17  = ModelPars[87];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[167];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[79]);
    real_type t28  = Fzf__XO * t27;
    real_type t31  = t28 * t9 + ModelPars[161];
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
    real_type t5   = ModelPars[239];
    real_type t9   = ModelPars[77] * ModelPars[165];
    real_type t10  = ModelPars[85];
    real_type t11  = Fzf__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[83];
    real_type t16  = t15 * Fzf__XO;
    real_type t17  = ModelPars[87];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[167];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[79]);
    real_type t28  = Fzf__XO * t27;
    real_type t31  = t28 * t9 + ModelPars[161];
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
    real_type t5   = ModelPars[239];
    real_type t6   = t5 * t3 * t1;
    real_type t7   = Fzf__XO * Fzf__XO;
    real_type t8   = ModelPars[77];
    real_type t10  = ModelPars[165];
    real_type t11  = ModelPars[85];
    real_type t12  = Fzf__XO - t2;
    real_type t16  = t3 * t12 * t11 + ModelPars[83];
    real_type t20  = ModelPars[87];
    real_type t23  = exp(t3 * t12 * t20);
    real_type t24  = ModelPars[167];
    real_type t25  = t24 * t23;
    real_type t26  = t10 * t8;
    real_type t31  = t5 * (t3 * t12 * t1 + ModelPars[79]);
    real_type t33  = Fzf__XO * t31 * t26;
    real_type t35  = t33 + ModelPars[161];
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
    real_type t10  = ModelPars[239] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t13  = ModelPars[77] * ModelPars[165];
    real_type t23  = exp(t5 * t3 * ModelPars[87]);
    real_type t26  = t10 * Fzf__XO;
    real_type t34  = atan(lambda__f__XO / (t26 * t13 + ModelPars[161]) * ModelPars[167] * t23 * (t5 * t3 * ModelPars[85] + ModelPars[83]) * Fzf__XO);
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
    real_type t10  = ModelPars[239] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t11  = Fzf__XO * t10;
    real_type t14  = ModelPars[77] * ModelPars[165];
    real_type t24  = exp(t5 * t3 * ModelPars[87]);
    real_type t34  = atan(lambda__f__XO / (t11 * t14 + ModelPars[161]) * ModelPars[167] * t24 * (t5 * t3 * ModelPars[85] + ModelPars[83]) * Fzf__XO);
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
    real_type t10  = ModelPars[239] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t13  = ModelPars[77] * ModelPars[165];
    real_type t23  = exp(t5 * t3 * ModelPars[87]);
    real_type t26  = Fzf__XO * t10;
    real_type t34  = atan(lambda__f__XO / (t26 * t13 + ModelPars[161]) * ModelPars[167] * t23 * (t5 * t3 * ModelPars[85] + ModelPars[83]) * Fzf__XO);
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
    real_type t10  = ModelPars[239] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t11  = Fzf__XO * Fzf__XO;
    real_type t12  = ModelPars[77];
    real_type t15  = ModelPars[165];
    real_type t20  = t5 * t3 * ModelPars[85] + ModelPars[83];
    real_type t25  = exp(t5 * t3 * ModelPars[87]);
    real_type t26  = ModelPars[167];
    real_type t28  = t15 * t12;
    real_type t29  = Fzf__XO * t10;
    real_type t32  = t29 * t28 + ModelPars[161];
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
    real_type t10  = ModelPars[239] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t13  = ModelPars[77] * ModelPars[165];
    real_type t23  = exp(t5 * t3 * ModelPars[87]);
    real_type t34  = atan(lambda__f__XO / (Fzf__XO * t10 * t13 + ModelPars[161]) * ModelPars[167] * t23 * (t5 * t3 * ModelPars[85] + ModelPars[83]) * Fzf__XO);
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
    real_type t10  = ModelPars[239] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t13  = ModelPars[77] * ModelPars[165];
    real_type t23  = exp(t5 * t3 * ModelPars[87]);
    real_type t34  = atan(lambda__f__XO / (Fzf__XO * t10 * t13 + ModelPars[161]) * ModelPars[167] * t23 * (t5 * t3 * ModelPars[85] + ModelPars[83]) * Fzf__XO);
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
    real_type t10  = ModelPars[239] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t11  = Fzf__XO * Fzf__XO;
    real_type t12  = ModelPars[77];
    real_type t15  = ModelPars[165];
    real_type t20  = t5 * t3 * ModelPars[85] + ModelPars[83];
    real_type t25  = exp(t5 * t3 * ModelPars[87]);
    real_type t26  = ModelPars[167];
    real_type t30  = t15 * t12;
    real_type t31  = Fzf__XO * t10;
    real_type t34  = t31 * t30 + ModelPars[161];
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
    real_type t10  = ModelPars[239] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t11  = Fzf__XO * Fzf__XO;
    real_type t15  = ModelPars[77] * ModelPars[165];
    real_type t20  = t5 * t3 * ModelPars[85] + ModelPars[83];
    real_type t26  = exp(t5 * t3 * ModelPars[87]);
    real_type t27  = ModelPars[167];
    real_type t29  = Fzf__XO * t10;
    real_type t32  = t29 * t15 + ModelPars[161];
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
    real_type t10  = ModelPars[239] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t11  = Fzf__XO * Fzf__XO;
    real_type t12  = t11 * t11;
    real_type t14  = ModelPars[77];
    real_type t15  = ModelPars[165];
    real_type t16  = t15 * t14;
    real_type t21  = t5 * t3 * ModelPars[85] + ModelPars[83];
    real_type t22  = t21 * t21;
    real_type t27  = exp(t5 * t3 * ModelPars[87]);
    real_type t28  = t27 * t27;
    real_type t33  = ModelPars[167];
    real_type t34  = t33 * t33;
    real_type t36  = Fzf__XO * t10;
    real_type t39  = t36 * t16 + ModelPars[161];
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
    real_type t11  = Fzr__XO * ModelPars[240] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t14  = ModelPars[78] * ModelPars[166];
    real_type t24  = exp(t5 * t3 * ModelPars[88]);
    real_type t34  = atan(lambda__r__XO / (t11 * t14 + ModelPars[162]) * ModelPars[168] * t24 * (t5 * t3 * ModelPars[86] + ModelPars[84]) * Fzr__XO);
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
    real_type t5   = ModelPars[240];
    real_type t9   = ModelPars[78] * ModelPars[166];
    real_type t10  = ModelPars[86];
    real_type t11  = Fzr__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[84];
    real_type t16  = t15 * Fzr__XO;
    real_type t17  = ModelPars[88];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[168];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[80]);
    real_type t29  = Fzr__XO * t27 * t9;
    real_type t31  = t29 + ModelPars[162];
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
    real_type t5   = ModelPars[240];
    real_type t6   = t5 * t4;
    real_type t7   = ModelPars[78];
    real_type t8   = ModelPars[166];
    real_type t9   = t8 * t7;
    real_type t10  = ModelPars[86];
    real_type t11  = Fzr__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[84];
    real_type t16  = t15 * Fzr__XO;
    real_type t17  = ModelPars[88];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[168];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[80]);
    real_type t29  = Fzr__XO * t27 * t9;
    real_type t31  = t29 + ModelPars[162];
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
    real_type t5   = ModelPars[240];
    real_type t6   = t5 * t3 * t1;
    real_type t7   = ModelPars[78];
    real_type t8   = ModelPars[166];
    real_type t9   = t8 * t7;
    real_type t10  = ModelPars[86];
    real_type t11  = Fzr__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[84];
    real_type t16  = t15 * Fzr__XO;
    real_type t17  = ModelPars[88];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[168];
    real_type t27  = t5 * (t3 * t11 * t1 + ModelPars[80]);
    real_type t28  = Fzr__XO * t27;
    real_type t31  = t28 * t9 + ModelPars[162];
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
    real_type t5   = ModelPars[240];
    real_type t9   = ModelPars[78] * ModelPars[166];
    real_type t10  = ModelPars[86];
    real_type t11  = Fzr__XO - t2;
    real_type t15  = t3 * t11 * t10 + ModelPars[84];
    real_type t16  = t15 * Fzr__XO;
    real_type t17  = ModelPars[88];
    real_type t20  = exp(t3 * t11 * t17);
    real_type t21  = t20 * t16;
    real_type t22  = ModelPars[168];
    real_type t27  = t5 * (t1 * t11 * t3 + ModelPars[80]);
    real_type t28  = Fzr__XO * t27;
    real_type t31  = t28 * t9 + ModelPars[162];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__r__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t41  = ModelPars[105];
    real_type t45  = phi__XO * ModelPars[97] + ModelPars[93];
    real_type t47  = lambda__r__XO * lambda__r__XO;
    real_type t49  = ModelPars[95] * ModelPars[95];
    real_type t51  = t47 * t49 + 1;
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
    return -t75 * t53 * t45 * t41 * t38 * Fzr__XO * t5 * t3 * t1 - t75 * t64 * t41 * t38 * t27 - t74 * t68 * t63 * t53 * t45 * t41 * t120 / (t106 * t107 * t109 * t111 * t47 * t94 + 1) * (t34 * t20 * t15 + t88 * t3 * t10 * Fzr__XO + t88 * t3 * t17 * t16 - (Fzr__XO * t1 * t3 * t5 * t9 + t27 * t9) * lambda__r__XO * t94 * t22 * t21) * t9 * t28;
  }

  real_type
  General::Fxr_D_1_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[82];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[240];
    real_type t6   = t5 * t3 * t1;
    real_type t7   = Fzr__XO * Fzr__XO;
    real_type t8   = ModelPars[78];
    real_type t10  = ModelPars[166];
    real_type t11  = ModelPars[86];
    real_type t12  = Fzr__XO - t2;
    real_type t16  = t3 * t12 * t11 + ModelPars[84];
    real_type t20  = ModelPars[88];
    real_type t23  = exp(t3 * t12 * t20);
    real_type t24  = ModelPars[168];
    real_type t25  = t24 * t23;
    real_type t26  = t10 * t8;
    real_type t31  = t5 * (t3 * t12 * t1 + ModelPars[80]);
    real_type t33  = Fzr__XO * t31 * t26;
    real_type t35  = t33 + ModelPars[162];
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
    real_type t10  = ModelPars[240] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t13  = ModelPars[78] * ModelPars[166];
    real_type t23  = exp(t5 * t3 * ModelPars[88]);
    real_type t26  = Fzr__XO * t10;
    real_type t34  = atan(lambda__r__XO / (t26 * t13 + ModelPars[162]) * ModelPars[168] * t23 * (t5 * t3 * ModelPars[86] + ModelPars[84]) * Fzr__XO);
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
    real_type t10  = ModelPars[240] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t11  = Fzr__XO * t10;
    real_type t14  = ModelPars[78] * ModelPars[166];
    real_type t24  = exp(t5 * t3 * ModelPars[88]);
    real_type t34  = atan(lambda__r__XO / (t11 * t14 + ModelPars[162]) * ModelPars[168] * t24 * (t5 * t3 * ModelPars[86] + ModelPars[84]) * Fzr__XO);
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
    real_type t10  = ModelPars[240] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t13  = ModelPars[78] * ModelPars[166];
    real_type t23  = exp(t5 * t3 * ModelPars[88]);
    real_type t26  = Fzr__XO * t10;
    real_type t34  = atan(lambda__r__XO / (t26 * t13 + ModelPars[162]) * ModelPars[168] * t23 * (t5 * t3 * ModelPars[86] + ModelPars[84]) * Fzr__XO);
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
    real_type t10  = ModelPars[240] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t11  = Fzr__XO * Fzr__XO;
    real_type t12  = ModelPars[78];
    real_type t15  = ModelPars[166];
    real_type t20  = t5 * t3 * ModelPars[86] + ModelPars[84];
    real_type t25  = exp(t5 * t3 * ModelPars[88]);
    real_type t26  = ModelPars[168];
    real_type t28  = t15 * t12;
    real_type t29  = Fzr__XO * t10;
    real_type t32  = t29 * t28 + ModelPars[162];
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
    real_type t10  = ModelPars[240] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t13  = ModelPars[78] * ModelPars[166];
    real_type t23  = exp(t5 * t3 * ModelPars[88]);
    real_type t34  = atan(lambda__r__XO / (Fzr__XO * t10 * t13 + ModelPars[162]) * ModelPars[168] * t23 * (t5 * t3 * ModelPars[86] + ModelPars[84]) * Fzr__XO);
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
    real_type t10  = ModelPars[240] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t13  = ModelPars[78] * ModelPars[166];
    real_type t23  = exp(t5 * t3 * ModelPars[88]);
    real_type t34  = atan(lambda__r__XO / (Fzr__XO * t10 * t13 + ModelPars[162]) * ModelPars[168] * t23 * (t5 * t3 * ModelPars[86] + ModelPars[84]) * Fzr__XO);
    real_type t36  = sin(t34 * t13);
    real_type t37  = t36 * Fzr__XO;
    real_type t38  = ModelPars[105];
    real_type t44  = phi__XO * ModelPars[97] + ModelPars[93];
    real_type t45  = t44 * t44;
    real_type t47  = lambda__r__XO * lambda__r__XO;
    real_type t49  = ModelPars[95] * ModelPars[95];
    real_type t51  = t47 * t49 + 1;
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
    real_type t10  = ModelPars[240] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t11  = Fzr__XO * Fzr__XO;
    real_type t12  = ModelPars[78];
    real_type t15  = ModelPars[166];
    real_type t20  = t5 * t3 * ModelPars[86] + ModelPars[84];
    real_type t25  = exp(t5 * t3 * ModelPars[88]);
    real_type t26  = ModelPars[168];
    real_type t30  = t15 * t12;
    real_type t31  = Fzr__XO * t10;
    real_type t34  = t31 * t30 + ModelPars[162];
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
    real_type t10  = ModelPars[240] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t11  = Fzr__XO * Fzr__XO;
    real_type t15  = ModelPars[78] * ModelPars[166];
    real_type t20  = t5 * t3 * ModelPars[86] + ModelPars[84];
    real_type t26  = exp(t5 * t3 * ModelPars[88]);
    real_type t27  = ModelPars[168];
    real_type t29  = Fzr__XO * t10;
    real_type t32  = t29 * t15 + ModelPars[162];
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
    real_type t10  = ModelPars[240] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t11  = Fzr__XO * Fzr__XO;
    real_type t12  = t11 * t11;
    real_type t14  = ModelPars[78];
    real_type t15  = ModelPars[166];
    real_type t16  = t15 * t14;
    real_type t21  = t5 * t3 * ModelPars[86] + ModelPars[84];
    real_type t22  = t21 * t21;
    real_type t27  = exp(t5 * t3 * ModelPars[88]);
    real_type t28  = t27 * t27;
    real_type t33  = ModelPars[168];
    real_type t34  = t33 * t33;
    real_type t36  = Fzr__XO * t10;
    real_type t39  = t36 * t16 + ModelPars[162];
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
    real_type t141 = t135 * t139 + 1;
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
    real_type t166 = t45 * t8 * t99 + t12 * t57;
    real_type t170 = t133 * t133;
    real_type t172 = 1.0 / t141 * t170;
    real_type t173 = t150 * t150;
    real_type t177 = sin(t153);
    real_type t183 = t160 * t160;
    real_type t189 = t156 * t156;
    real_type t194 = sin(t159);
    return t161 * t154 * (t10 * Fzf__XO * t2 * t57 * t13 + t125 / (t118 * t9 * t66 / t2 / t113 / t110 * t85 + 1) * (t53 * t27 * t25 * t18 - t52 * t66 * t29 * t28 + (t41 * t13 * t70 - t41 / t12 / t11 / t9 / t8 * t3 * t2 * t1 + (t18 * t24 * t36 * t86 * phi__f__XO - t35 * t48 * phi__f__XO) * t34 * t32 - Fzf__XO * t24 * t37 * t33 * t1 * t13 * t45 + t18 * t24 * t86 * t8 * t99 * t45) * t8 * t31 * t28) * t14 * t12 + Fzf__XO * t70 * t13 * t45) - t161 * t177 / (t172 * t173 + 1) * t148 * t146 * t144 * t132 * t166 + t194 / (t172 * t189 + 1) * t148 * t146 * t143 * t133 * t132 / t183 * t154 * t166;
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
    real_type t177 = t45 * t8 * t99 + t12 * t57;
    real_type t185 = t157 * t157;
    real_type t186 = 1.0 / t185;
    real_type t192 = t132 * t132;
    real_type t193 = t192 * t177;
    real_type t196 = cos(t163);
    real_type t203 = t169 * t169;
    real_type t207 = t165 * t165;
    real_type t212 = sin(t168);
    return -t171 * t158 * t145 * t134 * t132 * (t10 * Fzf__XO * t2 * t57 * t13 + t125 / (t118 * t9 * t66 / t2 / t113 / t110 * t85 + 1) * (t53 * t27 * t25 * t18 - t52 * t66 * t29 * t28 + (t41 * t13 * t70 - t41 / t12 / t11 / t9 / t8 * t3 * t2 * t1 + (t18 * t24 * t36 * t86 * phi__f__XO - t35 * t48 * phi__f__XO) * t34 * t32 - Fzf__XO * t24 * t37 * t33 * t1 * t13 * t45 + t18 * t24 * t86 * t8 * t99 * t45) * t8 * t31 * t28) * t14 * t12 + Fzf__XO * t70 * t13 * t45) + 2 * t154 * t171 * t186 * t152 * t150 / t144 / t143 * t146 * t134 * t132 * t177 - t170 * t196 * t186 * t152 * t150 * t148 * t193 - t212 / (t207 * t148 + 1) * t152 * t150 / t203 * t164 * t158 * t148 * t193;
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
    real_type t188 = t10 * t1;
    real_type t190 = t164 * t158;
    real_type t192 = sin(t84);
    real_type t193 = t192 * t32 * t165;
    real_type t197 = t16 * t39;
    real_type t198 = t197 * t188;
    real_type t199 = t192 * t33;
    real_type t214 = t32 * t16;
    real_type t215 = t6 * t6;
    real_type t223 = t26 * t26;
    real_type t225 = t29 * t29;
    real_type t226 = 1.0 / t225;
    real_type t230 = t27 * t1;
    real_type t231 = 1.0 / t65;
    real_type t233 = t16 * t39 * t231;
    real_type t246 = t223 * t19;
    real_type t250 = 1.0 / t225 / t24;
    real_type t251 = t27 * t27;
    real_type t255 = 1.0 / t32 / t226 / t251 / t223;
    real_type t284 = t63 * t63;
    real_type t294 = t158 * t158;
    real_type t295 = t163 * t163;
    real_type t316 = t10 * t230;
    return (2 * t6 * t75 * t29 * t73 * t72 * t70 * t67 * t9 - 2 * t76 / t94 * t72 * t71) * t85 * t81 * t64 * t42 - 2 * t143 * t136 * t128 * t16 * t11 * t109 * t107 * t104 + 2 * t170 * t168 * t32 * t166 * t159 * t10 * t2 - t184 / t181 * t180 * t150 * t178 * t177 + 2 * t193 * t190 * t151 * t188 - 3 * t30 * Fzf__XO * t26 * t199 * t198 - 2 * t85 * t169 * t63 * t44 * t198 - 2 * t215 * t192 * t214 * t39 / t65 / t9 * t2 + t226 * t223 * t192 * t56 * t16 * t40 * t94 * t1 + 2 * t30 * t26 * t6 * t199 * t233 * t230 + t193 * t164 * (3 * alpha__f__XO * t255 * t250 * t94 * t246 * t18 - 3 * Fzf__XO * t125 * t123 * t153) * t150 * t11 * t10 * t2 - t85 * t169 * (3 * t226 * t27 * t223 * alpha__f__XO * t255 * t50 * t54 - t30 * t26 * t125 * t50 * t54 - 2 * t61 * t57 * t49) * t44 * t42 + t192 * t81 * t284 * t69 * t32 * t42 + t192 * t32 * t39 / t295 * t294 / t149 / t15 * t178 * t10 * t2 - 2 * t6 * t192 * t32 * t165 * t190 * t150 * t11 * t231 * t2 + 2 * t6 * t170 * t64 * t233 * t2 - 2 * t30 * t26 * t85 * t169 * t168 * t33 * t197 * t316 + 2 * t30 * t26 * t192 * t33 * t166 * t159 * t316 + 2 * t6 * t192 * t214 * t39 * t231 * t1 - t143 * t136 * (3 * alpha__f__XO * t255 * t250 * t94 * t246 * t117 - 3 * alpha__f__XO * Fzf__XO * t155 * t121) * t17 * t177;
  }

  real_type
  General::Mzf_D_1_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[49];
    real_type t2   = Fzf__XO * t1;
    real_type t4   = ModelPars[6];
    real_type t6   = ModelPars[35];
    real_type t9   = t4 * ModelPars[33] + (Fzf__XO - t4) * t6;
    real_type t10  = 1.0 / t9;
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
    real_type t40  = t39 * t10;
    real_type t41  = t16 * t40;
    real_type t42  = t41 * t2;
    real_type t43  = ModelPars[47];
    real_type t44  = t43 * t32;
    real_type t45  = ModelPars[41];
    real_type t47  = t45 * t12 + 1;
    real_type t48  = t47 * t47;
    real_type t49  = 1.0 / t48;
    real_type t51  = 1.0 / t43;
    real_type t53  = alpha__f__XO * t33;
    real_type t55  = phi__f__XO * t45 * t53;
    real_type t58  = 1.0 / t47;
    real_type t59  = t58 * t6;
    real_type t61  = 1.0 / t32 / t31;
    real_type t62  = t61 * t51;
    real_type t63  = alpha__f__XO * t62;
    real_type t66  = 1.0 / t29 / t24;
    real_type t68  = phi__f__XO * t22 * t66;
    real_type t69  = t68 * t28;
    real_type t72  = t49 * t9;
    real_type t74  = Fzf__XO * t26;
    real_type t80  = t58 * t9;
    real_type t81  = t26 * t26;
    real_type t82  = t27 * t27;
    real_type t84  = t29 * t29;
    real_type t85  = 1.0 / t84;
    real_type t88  = 1.0 / t32 / t85 / t82 / t81;
    real_type t92  = t27 * Fzf__XO;
    real_type t97  = phi__f__XO * t22 / t84 / t24;
    real_type t101 = t63 * t80;
    real_type t106 = t9 * t9;
    real_type t107 = t49 * t106;
    real_type t108 = t43 * t43;
    real_type t109 = 1.0 / t108;
    real_type t111 = 1.0 / t26;
    real_type t112 = 1.0 / t27;
    real_type t114 = alpha__f__XO * alpha__f__XO;
    real_type t118 = t114 * t29 * t112 * t111 * t109 * t107 + 1;
    real_type t119 = 1.0 / t118;
    real_type t122 = alpha__f__XO * t33 * t51;
    real_type t124 = atan(t122 * t80);
    real_type t125 = t124 * t43;
    real_type t126 = cos(t125);
    real_type t130 = ModelPars[51];
    real_type t131 = Fzf__XO * t130;
    real_type t132 = ModelPars[55];
    real_type t134 = atan(phi__f__XO * t132);
    real_type t135 = 1.0 / t132;
    real_type t136 = t135 * t134;
    real_type t138 = t11 * t136 * t131;
    real_type t139 = ModelPars[57];
    real_type t140 = ModelPars[53];
    real_type t142 = t140 * t12 + 1;
    real_type t144 = 1.0 / t142 * t139;
    real_type t145 = t19 * t144;
    real_type t147 = t26 * t19;
    real_type t150 = alpha__f__XO * t61;
    real_type t153 = -t150 * t66 * t27 * t147 * t144 + t35 * t145;
    real_type t154 = t153 * t16;
    real_type t155 = t139 * t139;
    real_type t156 = t142 * t142;
    real_type t157 = 1.0 / t156;
    real_type t160 = t114 * t157 * t155 + 1;
    real_type t161 = 1.0 / t160;
    real_type t162 = t15 * t15;
    real_type t163 = 1.0 / t162;
    real_type t164 = t163 * t11;
    real_type t168 = atan(t53 * t25 * Fzf__XO * t145);
    real_type t173 = t157 * t139;
    real_type t174 = Fzf__XO * t19;
    real_type t177 = phi__f__XO * t140 * alpha__f__XO;
    real_type t183 = phi__f__XO * t22 * alpha__f__XO;
    real_type t188 = t61 * t85;
    real_type t195 = -2 * phi__f__XO * t13 * t168 * t164 + t161 * (2 * t183 * t188 * t92 * t147 * t144 - 2 * t183 * t33 * t30 * t174 * t144 - 2 * t177 * t34 * t174 * t173) * t17;
    real_type t197 = t168 * t17;
    real_type t198 = cos(t197);
    real_type t202 = 1.0 / t106;
    real_type t203 = t39 * t202;
    real_type t206 = sin(t125);
    real_type t207 = t206 * t32;
    real_type t215 = t163 * t11 * t10 * t2;
    real_type t217 = phi__f__XO * t22;
    real_type t218 = t217 * t53;
    real_type t221 = t147 * t18;
    real_type t227 = t81 * t19;
    real_type t230 = 1.0 / t84 / t29;
    real_type t238 = t18 * t18;
    real_type t241 = 1.0 / (t238 * t114 + 1);
    real_type t243 = sin(t38);
    real_type t248 = t27 * t1;
    real_type t253 = t30 * Fzf__XO;
    real_type t264 = -2 * phi__f__XO * t13 * t37 * t164 + t241 * (2 * t217 * t150 * t85 * t92 * t221 - 2 * t218 * t253 * t20) * t17;
    real_type t266 = t243 * t264 * t10;
    real_type t269 = t26 * t206;
    real_type t273 = t10 * t2;
    real_type t276 = t61 * t66;
    real_type t279 = -alpha__f__XO * t276 * t27 * t221 + t35 * t20;
    real_type t283 = t32 * t243 * t241;
    real_type t288 = -2 * t55 * t51 * t72 + 2 * t69 * t101;
    real_type t289 = t288 * t43;
    real_type t290 = t126 * t119;
    real_type t296 = sin(t197);
    real_type t323 = -t253 * t26 * alpha__f__XO * t62 * t80 + t122 * t59;
    real_type t325 = t118 * t118;
    real_type t326 = 1.0 / t325;
    real_type t331 = t111 * t109;
    real_type t353 = t126 * t119 * t323;
    real_type t362 = t10 * t1;
    real_type t363 = t16 * t39;
    real_type t364 = t33 * t363;
    real_type t376 = t241 * t279;
    real_type t382 = -t126 * t119 * (-6 * t97 * t92 * t81 * alpha__f__XO * t88 * t51 * t80 + 2 * phi__f__XO * t45 * t30 * t74 * t63 * t72 - 2 * t55 * t51 * t49 * t6 + 4 * t68 * t74 * t101 + 2 * t69 * t63 * t59) * t44 * t42 - t198 * t195 * t161 * t154 * t138 - 2 * phi__f__XO * t13 * t6 * t207 * t163 * t203 * t2 + t206 * t32 * t243 * t241 * (-6 * t217 * alpha__f__XO * t88 * t230 * t82 * t227 * t18 + 8 * t217 * t27 * alpha__f__XO * t188 * t221 - 2 * t218 * t30 * t20) * t215 + t30 * t269 * t33 * t16 * t266 * t248 + t290 * t289 * t283 * t279 * t164 * t273 + 2 * phi__f__XO * t13 * t296 * t161 * t153 * t163 * t138 + 2 * phi__f__XO * t13 * t30 * t26 * t206 * t33 * t163 * t40 * t248 - 2 * t97 * t81 * t206 * t61 * t41 * t82 * t1 + (-4 * phi__f__XO * t45 * t114 * t29 * t112 * t331 / t48 / t47 * t106 + 4 * phi__f__XO * t22 * t114 * t24 * t112 * t331 * t107) * t126 * t326 * t323 * t44 * t42 + t353 * t43 * t32 * t16 * t266 * t2 + t206 * t288 * t326 * t323 * t108 * t32 * t42 + 6 * t68 * t27 * t269 * t364 * t362 + t6 * t290 * t288 * t44 * t16 * t203 * t2 + t206 * t32 * t39 * t264 * t376 * t215;
    real_type t383 = t92 * t1;
    real_type t384 = t10 * t383;
    real_type t390 = t217 * t66 * t26;
    real_type t404 = t160 * t160;
    real_type t411 = phi__f__XO * t140;
    real_type t420 = t163 * t39;
    real_type t455 = t16 * t243;
    real_type t458 = t132 * t132;
    real_type t461 = 1.0 / (t458 * t12 + 1);
    real_type t517 = -2 * t390 * t206 * t33 * t243 * t376 * t164 * t384 + 2 * t390 * t353 * t43 * t33 * t363 * t384 - 4 * t411 * t114 / t156 / t142 * t155 * t296 / t404 * t154 * t11 * t135 * t134 * t131 - t296 * t195 * t135 * t134 * t130 + 2 * phi__f__XO * t13 * t126 * t119 * t323 * t43 * t32 * t420 * t273 - 2 * t68 * t26 * t6 * t206 * t364 * t202 * t383 - 4 * phi__f__XO * t13 * t206 * t283 * t279 / t162 / t15 * t11 * t273 - t30 * t26 * t126 * t119 * t289 * t364 * t10 * t248 + t207 * t455 * t264 * t362 + t198 * t461 * t130 - t126 * t119 * t288 * t44 * t363 * t362 - t6 * t207 * t455 * t264 * t202 * t2 - t296 * t161 * (8 * phi__f__XO * t22 * t27 * t150 * t85 * t147 * t144 - 6 * t183 * t88 * t230 * t82 * t227 * t144 + 2 * t177 * t276 * t27 * t147 * t173 - 2 * t411 * t53 * t25 * t19 * t173 - 2 * t218 * t30 * t19 * t144) * t17 * t136 * t131 + 2 * phi__f__XO * t13 * t207 * t420 * t362 - t296 * t161 * t154 * t11 * t461 * t131;
    return t382 + t517;
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
    real_type t327 = t79 * t70;
    real_type t332 = t290 * t327;
    real_type t347 = t75 * t75;
    return -2 * t65 * t57 * t48 * t14 * t7 * t6 * t4 * t1 + 2 * t144 * t124 * t33 * t104 * t93 * t79 * t71 - t158 / t155 * t154 * t82 * t152 * t151 + 2 * t31 * t27 * t75 * t169 * t166 * t162 + t202 * t98 * (3 * alpha__r__XO * t194 * t188 * t184 * t182 * t84 - 3 * Fzr__XO * t45 * t41 * t87) * t82 * t7 * t79 * t71 - t143 * t139 * (3 * t191 * t28 * t181 * alpha__r__XO * t194 * t112 * t116 - t31 * t27 * t45 * t112 * t116 - 2 * t121 * t117 * t111) * t208 * t207 + t168 * t231 * t229 * t129 * t33 * t207 + t168 * t33 * t164 / t242 * t241 / t81 / t13 * t152 * t79 * t71 - 2 * t75 * t168 * t33 * t103 * t252 * t82 * t7 * t163 * t71 + 2 * t75 * t144 * t260 * t166 * t71 + (2 * t75 * t54 * t30 * t133 * t132 * t130 * t127 * t78 - 2 * t135 / t184 * t132 * t131) * t143 * t231 * t260 * t207 + 2 * t31 * t27 * t168 * t34 * t104 * t93 * t282 - 2 * t31 * t27 * t143 * t139 * t124 * t34 * t290 * t282 + 2 * t75 * t168 * t301 * t164 * t163 * t70 + t191 * t181 * t168 * t44 * t14 * t205 * t184 * t70 - t65 * t57 * (3 * alpha__r__XO * t194 * t188 * t184 * t182 * t20 - 3 * alpha__r__XO * Fzr__XO * t89 * t39) * t58 * t151 + 2 * t202 * t252 * t83 * t327 - 3 * t31 * Fzr__XO * t27 * t169 * t332 - 2 * t143 * t139 * t123 * t208 * t332 - 2 * t347 * t168 * t301 * t164 / t125 / t78 * t71;
  }

  real_type
  General::Mzr_D_1_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
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
    real_type t113 = 1.0 / t90;
    real_type t114 = cos(t61);
    real_type t115 = t114 * t113;
    real_type t118 = sin(t107);
    real_type t119 = t118 * t33;
    real_type t125 = ModelPars[52];
    real_type t126 = Fzr__XO * t125;
    real_type t127 = ModelPars[55];
    real_type t129 = atan(phi__XO * t127);
    real_type t130 = 1.0 / t127;
    real_type t131 = t130 * t129;
    real_type t133 = ModelPars[57];
    real_type t134 = ModelPars[53];
    real_type t136 = t134 * t13 + 1;
    real_type t137 = t136 * t136;
    real_type t138 = 1.0 / t137;
    real_type t139 = t138 * t133;
    real_type t142 = phi__XO * t134;
    real_type t147 = 1.0 / t136 * t133;
    real_type t150 = phi__XO * t23;
    real_type t151 = t150 * t74;
    real_type t154 = t30 * t30;
    real_type t155 = 1.0 / t154;
    real_type t158 = alpha__r__XO * t44;
    real_type t167 = phi__XO * t134 * alpha__r__XO;
    real_type t171 = t27 * t27;
    real_type t172 = t171 * t21;
    real_type t173 = t28 * t28;
    real_type t177 = 1.0 / t154 / t30;
    real_type t181 = 1.0 / t33 / t155 / t173 / t171;
    real_type t184 = phi__XO * t23 * alpha__r__XO;
    real_type t189 = t133 * t133;
    real_type t192 = t51 * t138 * t189 + 1;
    real_type t193 = 1.0 / t192;
    real_type t195 = t21 * t147;
    real_type t199 = atan(t74 * t26 * Fzr__XO * t195);
    real_type t200 = t199 * t57;
    real_type t201 = sin(t200);
    real_type t209 = t31 * Fzr__XO;
    real_type t212 = t28 * Fzr__XO;
    real_type t221 = -2 * phi__XO * t14 * t60 * t19 + t55 * (2 * t150 * t158 * t155 * t212 * t39 - 2 * t151 * t209 * t22) * t57;
    real_type t224 = t56 * t62;
    real_type t228 = t10 * t1;
    real_type t229 = t56 * t114;
    real_type t231 = t65 * t33;
    real_type t236 = t114 * t10;
    real_type t237 = t56 * t236;
    real_type t238 = t237 * t2;
    real_type t243 = t78 * t6;
    real_type t248 = Fzr__XO * t27;
    real_type t261 = phi__XO * t23 / t154 / t25;
    real_type t274 = t12 * t131 * t126;
    real_type t280 = -t158 * t42 * t28 * t38 * t147 + t36 * t195;
    real_type t281 = t280 * t56;
    real_type t286 = Fzr__XO * t21;
    real_type t296 = t44 * t155;
    real_type t303 = -2 * phi__XO * t14 * t199 * t19 + t193 * (2 * t184 * t296 * t212 * t38 * t147 - 2 * t184 * t34 * t31 * t286 * t147 - 2 * t167 * t35 * t286 * t139) * t57;
    real_type t305 = cos(t200);
    real_type t311 = t18 * t12 * t10 * t2;
    real_type t333 = t28 * t1;
    real_type t335 = t62 * t221 * t10;
    real_type t338 = t27 * t118;
    real_type t346 = t55 * t48;
    real_type t359 = t34 * t229;
    real_type t379 = -t209 * t27 * alpha__r__XO * t80 * t79 + t104 * t243;
    real_type t381 = t108 * t102 * t379;
    real_type t384 = t109 * t89 * t64 * t48 * t19 * t11 - 2 * phi__XO * t14 * t6 * t119 * t18 * t115 * t2 - t201 * t193 * (8 * phi__XO * t23 * t28 * t158 * t155 * t38 * t147 - 6 * t184 * t181 * t177 * t173 * t172 * t147 - 2 * t142 * t74 * t26 * t21 * t139 + 2 * t167 * t45 * t28 * t38 * t139 - 2 * t151 * t31 * t21 * t147) * t57 * t131 * t126 - t6 * t119 * t224 * t221 * t113 * t2 - t108 * t102 * t88 * t231 * t229 * t228 - t108 * t102 * (-6 * t261 * t212 * t171 * alpha__r__XO * t181 * t72 * t79 + 2 * phi__XO * t66 * t31 * t248 * t81 * t71 - 2 * t76 * t72 * t70 * t6 + 2 * t85 * t81 * t243 + 4 * t84 * t248 * t82) * t231 * t238 - t305 * t303 * t193 * t281 * t274 + t118 * t33 * t62 * t55 * (-6 * t150 * alpha__r__XO * t181 * t177 * t173 * t172 * t20 + 8 * t150 * t28 * alpha__r__XO * t296 * t39 - 2 * t151 * t31 * t22) * t311 + t31 * t338 * t34 * t56 * t335 * t333 - t201 * t303 * t130 * t129 * t125 + t118 * t33 * t114 * t221 * t346 * t311 + 2 * phi__XO * t14 * t201 * t193 * t280 * t18 * t274 + 6 * t84 * t28 * t338 * t359 * t228 + t6 * t109 * t88 * t231 * t56 * t115 * t2 + t381 * t65 * t33 * t56 * t335 * t2;
    real_type t387 = t101 * t101;
    real_type t388 = 1.0 / t387;
    real_type t414 = t95 * t93;
    real_type t432 = t127 * t127;
    real_type t435 = 1.0 / (t432 * t13 + 1);
    real_type t441 = t18 * t114;
    real_type t461 = t212 * t1;
    real_type t486 = t10 * t461;
    real_type t491 = t150 * t42 * t27;
    real_type t506 = t192 * t192;
    real_type t517 = t118 * t88 * t388 * t379 * t92 * t33 * t238 + 2 * phi__XO * t14 * t31 * t27 * t118 * t34 * t18 * t236 * t333 - 2 * t261 * t171 * t118 * t44 * t237 * t173 * t1 + (-4 * phi__XO * t66 * t51 * t30 * t96 * t414 / t69 / t68 * t90 + 4 * phi__XO * t23 * t51 * t25 * t96 * t414 * t91) * t108 * t388 * t379 * t231 * t238 + t305 * t435 * t125 + t119 * t224 * t221 * t228 + 2 * phi__XO * t14 * t119 * t441 * t228 - t201 * t193 * t281 * t12 * t435 * t126 + 2 * phi__XO * t14 * t108 * t102 * t379 * t65 * t33 * t441 * t11 - 2 * t84 * t27 * t6 * t118 * t359 * t113 * t461 - 4 * phi__XO * t14 * t118 * t64 * t48 / t17 / t16 * t12 * t11 - t31 * t27 * t108 * t102 * t89 * t359 * t10 * t333 + 2 * t491 * t381 * t65 * t34 * t229 * t486 - 2 * t491 * t118 * t34 * t62 * t346 * t19 * t486 - 4 * t142 * t51 / t137 / t136 * t189 * t201 / t506 * t281 * t12 * t130 * t129 * t126;
    return t384 + t517;
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
    real_type t197 = t196 * t196;
    real_type t199 = t197 * t12 + 1;
    real_type t200 = t199 * t199;
    real_type t203 = ModelPars[57];
    real_type t204 = ModelPars[53];
    real_type t205 = t204 * t12;
    real_type t206 = t205 + 1;
    real_type t208 = 1.0 / t206 * t203;
    real_type t213 = atan(t49 * t27 * Fzr__XO * t21 * t208);
    real_type t214 = t213 * t81;
    real_type t215 = cos(t214);
    real_type t221 = atan(phi__XO * t196);
    real_type t222 = t221 * t195;
    real_type t223 = 1.0 / t196;
    real_type t224 = t13 * t213;
    real_type t228 = t206 * t206;
    real_type t229 = 1.0 / t228;
    real_type t230 = t229 * t203;
    real_type t231 = Fzr__XO * t21;
    real_type t232 = t231 * t230;
    real_type t233 = t204 * alpha__r__XO;
    real_type t237 = t231 * t208;
    real_type t238 = phi__XO * t99;
    real_type t241 = t55 * t53;
    real_type t242 = t241 * t208;
    real_type t246 = -2 * phi__XO * t233 * t36 * t232 + 2 * t238 * t121 * t242 - 2 * t238 * t98 * t237;
    real_type t247 = t203 * t203;
    real_type t250 = t68 * t229 * t247 + 1;
    real_type t251 = 1.0 / t250;
    real_type t254 = -2 * phi__XO * t224 * t46 + t251 * t246 * t81;
    real_type t255 = t254 * t254;
    real_type t261 = sin(t214);
    real_type t277 = 1.0 / t228 / t206;
    real_type t280 = t204 * t204;
    real_type t288 = t24 * t205;
    real_type t302 = t12 * t86 * alpha__r__XO;
    real_type t323 = t250 * t250;
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
    real_type t411 = t146 * t35;
    real_type t412 = t28 * t411;
    real_type t416 = t45 * t152;
    real_type t420 = t24 * t189;
    real_type t437 = t178 * t176;
    real_type t460 = t338 * t150;
    real_type t474 = t398 * t398;
    real_type t479 = t337 * t2;
    return t147 * t80 * t132 * (8 * t12 * t40 * t39 * t19 - 4 * phi__XO * t13 * t72 * t66 * t46 - 2 * t77 * t46 + t72 * (12 * t87 * alpha__r__XO * t115 * t109 * t107 * t104 * t20 + 8 * t87 * t49 * t83 * Fzr__XO * t22 - 20 * t87 * t62 * t92 * t55 * t54 + 2 * t122 * t55 * t54 - 2 * t100 * t23) * t81) * t10 * t2 + 4 * t50 * t189 * t188 * t134 * t35 * t153 * t151 - 2 * phi__XO * t197 * t215 / t200 * t195 - t215 * t255 * t223 * t222 - 2 * t261 * t254 / t199 * t195 - t261 * (8 * t12 * t40 * t213 * t19 - 4 * phi__XO * t13 * t251 * t246 * t46 - 2 * t224 * t46 + t251 * (8 * t12 * t280 * alpha__r__XO * t36 * t231 * t277 * t203 + 12 * t302 * t115 * t109 * t107 * t104 * t208 + 8 * t288 * t49 * t32 * t231 * t230 - 8 * t288 * t62 * t57 * t241 * t230 + 8 * t302 * t35 * t83 * t237 - 20 * t302 * t61 * t92 * t242 - 2 * t233 * t36 * t232 - 2 * t100 * t237 + 2 * t122 * t242) * t81 + 4 * phi__XO * t204 * t68 * t277 * t247 / t323 * t246 * t81) * t223 * t222 + t146 * t343 * t341 * t175 * t34 * t339 - t187 * t185 * (12 * t374 * t112 * alpha__r__XO * t115 * t140 * t139 + 8 * t12 * t353 * t49 * t140 * t350 * t9 - 8 * t361 * t28 * t136 * t165 * t159 + 2 * t361 * t28 * alpha__r__XO * t164 * t139 - 2 * t135 * t49 * t160 - 12 * t379 * t30 * t166) * t348 * t339 - 4 * phi__XO * t403 * t402 * t401 - 4 * t168 * t412 * t80 * t132 * t398 * t151 - 8 * t420 * t12 * t403 * t35 * t416 * t151 + 4 * phi__XO * t13 * t187 * t185 * t172 * t134 * t34 * t416 * t10 * t2 + (-4 * phi__XO * t135 * t68 * t31 * t179 * t437 * t350 * t173 + 4 * phi__XO * t24 * t68 * t26 * t179 * t437 * t174) * t187 * t343 * t172 * t348 * t339 + 2 * t188 * t134 * t34 * t80 * t401 + 2 * t420 * t411 * t460 + 4 * t374 * t103 * t146 * t61 * t338 * t107 * t1 - 12 * t379 * t412 * t460 + t147 * t153 * t474 * t10 * t2 + 2 * t403 * t402 * t479 - 8 * t12 * t40 * t146 * t34 * t18 * t479;
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
    real_type t2   = X__[19];
    real_type t3   = t2 * t2;
    real_type t4   = X__[13];
    real_type t5   = t4 * t3;
    real_type t6   = ModelPars[143];
    real_type t7   = ModelPars[172];
    real_type t8   = t7 * t6;
    real_type t9   = X__[14];
    real_type t11  = ModelPars[119];
    real_type t14  = X__[30];
    real_type t16  = -2 * t7 * t6 * t11 - 2 * t9 * t8 - 2 * t14;
    real_type t18  = X__[29];
    real_type t20  = 2 * t18 * t8;
    real_type t21  = t7 * t7;
    real_type t22  = t4 * t21;
    real_type t24  = X__[3];
    real_type t25  = cos(t24);
    real_type t27  = sin(t24);
    real_type t28  = -t11 - t9;
    real_type t32  = -t4 * t8 - t18;
    real_type t49  = X__[20];
    real_type t52  = X__[21];
    real_type t55  = X__[22];
    real_type t58  = X__[23];
    real_type t61  = X__[24];
    real_type t64  = X__[25];
    real_type t67  = X__[26];
    real_type t70  = X__[27];
    real_type t73  = X__[28];
    real_type t79  = (-t25 * (t2 * t16 - t20 - t22 + t5) + 2 * (t3 * t28 / 2 + t2 * t32 + (t6 * t14 + t7 * t11 / 2 + t9 * t7 / 2) * t7) * t27) * L__[17] + t2 * L__[19] + t49 * L__[20] + t52 * L__[21] + t55 * L__[22] + t58 * L__[23] + t61 * L__[24] + t64 * L__[25] + t67 * L__[26] + t70 * L__[27] + t73 * L__[28] + t18 * L__[29] + t14 * L__[30];
    real_type t81  = X__[31];
    real_type t84  = X__[32];
    real_type t87  = X__[38];
    real_type t88  = cos(t87);
    real_type t89  = X__[1];
    real_type t91  = sin(t87);
    real_type t92  = X__[0];
    real_type t100 = X__[34];
    real_type t107 = X__[35];
    real_type t112 = X__[17];
    real_type t113 = ALIAS_maxTorque(t112);
    real_type t116 = X__[33];
    real_type t117 = t116 * ModelPars[114];
    real_type t124 = X__[36];
    real_type t128 = ModelPars[116];
    real_type t129 = ModelPars[118];
    real_type t130 = t128 - t129;
    real_type t131 = t2 - t84;
    real_type t132 = t131 * t130;
    real_type t133 = X__[16];
    real_type t134 = cos(t133);
    real_type t138 = t2 - t84 - t7;
    real_type t139 = t2 - t84 + t7;
    real_type t140 = t139 * t138;
    real_type t141 = sin(t133);
    real_type t144 = X__[10];
    real_type t145 = t144 * t3;
    real_type t146 = X__[11];
    real_type t148 = t6 * t129;
    real_type t151 = -2 * t146 * t8 - 2 * t7 * t148 - 2 * t70;
    real_type t155 = 2 * t7 * t6 * t67;
    real_type t156 = t144 * t21;
    real_type t165 = t3 * (t129 + t146);
    real_type t169 = t2 * (2 * t144 * t8 + 2 * t67);
    real_type t172 = 2 * t7 * t6 * t70;
    real_type t173 = t129 * t21;
    real_type t174 = t146 * t21;
    real_type t177 = ModelPars[141];
    real_type t179 = X__[6];
    real_type t181 = t55 * t6;
    real_type t188 = t49 * t49;
    real_type t189 = ModelPars[120];
    real_type t190 = X__[8];
    real_type t191 = t189 - t190;
    real_type t193 = ModelPars[22];
    real_type t194 = t193 + t177;
    real_type t195 = 2 * t194;
    real_type t197 = t49 * t7;
    real_type t199 = t190 * t21;
    real_type t200 = t21 * t189;
    real_type t202 = 2 * t61 * t8;
    real_type t204 = X__[4];
    real_type t206 = t204 + ModelPars[64];
    real_type t207 = cos(t206);
    real_type t209 = -t194;
    real_type t219 = t188 * t209 + t49 * (2 * t7 * t6 * t189 - 2 * t7 * t6 * t190 - 2 * t61) + t194 * t21;
    real_type t220 = sin(t206);
    real_type t223 = -t130;
    real_type t224 = 2 * t223;
    real_type t227 = t84 * t84;
    real_type t228 = t227 * t130;
    real_type t229 = t21 * t223;
    real_type t230 = t2 * t84 * t224 + t3 * t130 + t228 + t229;
    real_type t234 = -2 * t6 * t128 + 2 * t148;
    real_type t235 = t7 * t234;
    real_type t238 = -t7 * t234;
    real_type t253 = X__[5];
    real_type t254 = t21 * t253;
    real_type t257 = 2 * t7 * t6 * t52;
    real_type t261 = ModelPars[23];
    real_type t262 = t49 - t7 - t58;
    real_type t263 = t49 + t7 - t58;
    real_type t265 = X__[7];
    real_type t266 = cos(t265);
    real_type t268 = sin(t265);
    real_type t269 = t6 * t268;
    real_type t270 = t49 - t58;
    real_type t288 = X__[12];
    real_type t293 = -t191;
    real_type t295 = -t195;
    real_type t302 = X__[15];
    real_type t304 = t6 * t224;
    real_type t316 = X__[9];
    real_type t324 = -t270;
    real_type t325 = t266 * t324;
    real_type t328 = -t263;
    real_type t330 = -t262;
    real_type t364 = t2 * t84;
    real_type t368 = t7 * t181;
    real_type t369 = 2 * t368;
    real_type t372 = -t131;
    real_type t375 = t179 * t8 + t55;
    real_type t396 = t179 * t27;
    real_type t412 = -t139;
    real_type t414 = -t138;
    real_type t428 = t81 * L__[31] + t84 * L__[32] + (t89 * t88 + t92 * t91) * L__[37] + (-U__[0] * ModelPars[154] - t100) * L__[33] + (-U__[1] * ModelPars[155] - t107) * L__[35] + (t113 * U__[2] - t117) * L__[34] + (U__[3] * ModelPars[186] - t124) * L__[36] + (-t25 * (-t141 * t130 * t140 - 2 * t134 * t132 * t8 + t2 * t151 + t145 - t155 - t156) - t27 * (t134 * t130 * t140 - 2 * t141 * t132 * t8 + t165 + t169 - t172 - t173 - t174) - (t7 * t179 + 2 * t181) * t7 * t177) * L__[14] + (-t207 * (t197 * t6 * t195 + t191 * t188 + t199 - t200 + t202) - t220 * t219 - t25 * (t134 * t230 + t141 * (t2 * t235 + t84 * t238) + t165 + t169 - t174 - t173 - t172) - t27 * (t134 * (t2 * t238 + t84 * t235) + t141 * t230 - t145 - t2 * t151 + t156 + t155) - t254 - t257) * L__[15] + (-t207 * (t266 * t263 * t262 - 2 * t270 * t7 * t269) * t261 - t220 * (t263 * t262 * t268 + 2 * t266 * t270 * t8) * t261 - 2 * t7 * t6 * t73 - t21 * t288) * L__[16] + (-t220 * (t197 * t6 * t295 + t293 * t188 - t199 + t200 - t202) - t207 * t219 - t141 * (t302 * (t228 + t229) + t81 * t7 * t304) - t134 * (t302 * t84 * t7 * t304 + t84 * t81 * t224) + t21 * t316 + 2 * t7 * t6 * t64) * L__[13] + (t207 * (t330 * t328 * t268 - 2 * t325 * t8) * t261 - t220 * (t266 * t330 * t328 + 2 * t324 * t7 * t269) * t261 - t25 * (-t21 * t11 - 2 * t14 * t8 - 2 * t2 * t32 - t9 * t21 - t3 * t28) - t27 * (-t2 * t16 + t20 + t22 - t5) - t257 - t254 - t21 * (-ModelPars[117] + t11)) * L__[18] + (-t220 * (t134 * (t25 * (t179 * (t3 - 2 * t364 + t188 + t227 - t21) - t369) - 2 * t375 * t372 * t27) + 2 * t141 * (t25 * t375 * t372 - (t179 * (-t188 / 2 - t227 / 2 + t364 + t21 / 2 - t3 / 2) + t368) * t27)) + 2 * t207 * (t134 * (t25 * t375 + t372 * t396) + (-t25 * t372 * t179 + t375 * t27) * t141) * t49 - t134 * (-2 * t25 * t372 * t8 - t414 * t412 * t27) - t141 * (-2 * t372 * t7 * t6 * t27 + t25 * t414 * t412)) * L__[12];
    real_type t431 = t27 * t207;
    real_type t434 = Fzf(t146, t70);
    real_type t435 = X__[2];
    real_type t436 = alpha__f(t435, t92, t89, t316, t144, t302, t64, t67);
    real_type t437 = X__[18];
    real_type t438 = lambda__f(t435, t133, t92, t89, t316, t144, t302, t437, t64, t67);
    real_type t439 = Fxf(t434, t133, t436, t438);
    real_type t443 = Fyf(t434, t133, t436, t438);
    real_type t445 = ModelPars[74];
    real_type t446 = t435 * t435;
    real_type t447 = t446 * t293;
    real_type t448 = t25 * t25;
    real_type t450 = t194 * t2;
    real_type t451 = t435 * t25;
    real_type t457 = t207 * t207;
    real_type t471 = t253 * t446;
    real_type t475 = ModelPars[67];
    real_type t478 = t3 * t253;
    real_type t479 = t92 * t435;
    real_type t481 = t55 * t177;
    real_type t488 = t2 * t253;
    real_type t497 = t435 * t89;
    real_type t512 = t445 * t188;
    real_type t519 = -t439 * (t302 * t431 + t220) - t443 * (-t302 * t220 + t431) - t457 * (-t293 * t3 + t448 * t447 + 2 * t451 * t450) * t445 - t207 * (-2 * t220 * (-t448 * t446 * t194 / 2 + t451 * t293 * t2 + t194 * t3 / 2) * t445 - t434 * t25 + t445 * (t448 * t471 + t25 * (t396 * t446 * t177 + t475) - t471 - t478 - t27 * t479 + 2 * t2 * t481)) + 2 * t220 * (t25 * t435 * (t488 - t481) + t27 * t435 * (t2 * t177 * t179 + t52) - t497 / 2) * t445 + 2 * t445 * t451 * t450 - 2 * t27 * t435 * t445 * t293 * t49 + t445 * t447 - t190 * (-t512 + ModelPars[70]) - t189 * t512 - t61 * ModelPars[31];
    real_type t526 = t27 * t4;
    real_type t527 = t25 * t9;
    real_type t529 = Fzr(t9, t14);
    real_type t530 = alpha__r(t435, t92, t89, t288, t4, t73, t18);
    real_type t531 = lambda__r(t435, t24, t92, t4, t112, t73);
    real_type t532 = Fxr(t529, t24, t530, t531);
    real_type t536 = t266 * t207;
    real_type t540 = Fyr(t529, t24, t530, t531);
    real_type t542 = t529 * t261;
    real_type t548 = t435 * t2;
    real_type t549 = ModelPars[17];
    real_type t553 = t288 * t25 * t529;
    real_type t554 = Mzr(t529, t24, t530);
    real_type t555 = t27 * t554;
    real_type t582 = -t316 * t302 + t144;
    real_type t589 = t220 * (t253 * t25 - t146);
    real_type t590 = t25 * t193;
    real_type t596 = -t144 * t302 - t316;
    real_type t598 = t302 * t146;
    real_type t604 = Mzf(t434, t133, t436);
    real_type t607 = t177 * t193;
    real_type t608 = t445 * t607;
    real_type t609 = t177 * t177;
    real_type t610 = t445 * t609;
    real_type t611 = ModelPars[169];
    real_type t612 = ModelPars[223];
    real_type t615 = t612 * (t193 + t612) * t611;
    real_type t616 = ModelPars[150];
    real_type t617 = ModelPars[151];
    real_type t618 = ModelPars[12];
    real_type t619 = ModelPars[16];
    real_type t620 = t608 + t610 + t615 - t616 + t617 - t618 + t619;
    real_type t621 = t446 * t620;
    real_type t626 = t445 * t189 * t177;
    real_type t629 = t611 * t612 * ModelPars[233];
    real_type t630 = ModelPars[148];
    real_type t631 = t445 * t190 * t177 - t626 + t629 + t630;
    real_type t632 = t631 * t2;
    real_type t655 = t253 * t179;
    real_type t656 = t177 * t445;
    real_type t658 = t612 * t611 + t656;
    real_type t659 = t2 * t658;
    real_type t660 = t659 * t655;
    real_type t667 = t179 * t658;
    real_type t670 = 2 * t618;
    real_type t671 = ModelPars[152];
    real_type t672 = t670 - t619 + t616 - t617 + t671;
    real_type t675 = Q__[0];
    real_type t676 = Mxf(t675);
    real_type t677 = t302 * t676;
    real_type t680 = t497 * t667;
    real_type t681 = t616 / 2;
    real_type t682 = t617 / 2;
    real_type t683 = t671 / 2;
    real_type t684 = t619 / 2;
    real_type t687 = t61 * t656;
    real_type t688 = t437 * t619;
    real_type t689 = t688 / 2;
    real_type t690 = t49 * (t608 + t610 + t615 - t681 + t682 + t683 + t684) + t687 - t689;
    real_type t695 = t253 * t27;
    real_type t698 = t446 * t448;
    real_type t700 = t253 * t667 * t698;
    real_type t703 = t475 * t667;
    real_type t729 = t658 * t435;
    real_type t730 = t435 * t27;
    real_type t731 = t253 * t730;
    real_type t735 = t49 * t620;
    real_type t745 = 2 * t687;
    real_type t754 = -t439 * (t207 * (t27 * t146 + t25 * t582) + (t589 + t590) * t302) - t443 * (t207 * (t25 * t596 - t27 * t598) + t589 + t590) + t25 * t604 * t207 - t457 * t179 * (-t3 * t620 + t448 * t621 - 2 * t451 * t632) - t207 * (-2 * t220 * t179 * (t446 * t631 * t448 / 2 + t451 * t620 * t2 - t631 * t3 / 2) - t316 * t27 * t434 - 2 * t25 * (-t27 * t631 * t435 / 2 + t660 + t631 * t49) * t435 + t27 * (-t435 * t672 * t2 - 2 * t667 * t52 * t435 + t677) + t680 - 2 * t2 * t690) - t220 * (t434 * (t695 + t144) - t700 + t25 * (t446 * t620 * t27 - 2 * t690 * t435 - t703) + t27 * t667 * t479 + t667 * (t446 + t3) * t253 + 2 * t49 * t632 + t676) - t27 * t434 * t193 + 2 * t448 * t179 * (t610 + t608 / 2 + t612 * (t612 + t193 / 2) * t611 - t616 + t617 - t618 + t619) * t446 - t25 * (t731 + t92) * t729 - t27 * (-2 * t179 * (t735 + t687 - t689) * t435 + t475 * t658) - t179 * (t621 + t3 * t658 * t193 + t49 * (t735 + t745 - t688)) + 2 * t52 * t659 - t55 * ModelPars[147] + t124;
    real_type t761 = ModelPars[73];
    real_type t762 = ModelPars[222];
    real_type t763 = -t261 + t762;
    real_type t764 = t763 * t761;
    real_type t765 = ModelPars[232];
    real_type t767 = ModelPars[149];
    real_type t768 = t765 * t764 + t767;
    real_type t770 = t448 * t446 * t768;
    real_type t771 = t762 + t765 - t261;
    real_type t773 = t762 - t765 - t261;
    real_type t775 = t261 * t261;
    real_type t776 = ModelPars[75];
    real_type t777 = t776 * t775;
    real_type t778 = ModelPars[14];
    real_type t779 = ModelPars[19];
    real_type t780 = -t773 * t771 * t761 - t777 + t778 - t779;
    real_type t782 = t451 * t780 * t2;
    real_type t783 = t768 * t3;
    real_type t784 = -t770 + t782 + t783;
    real_type t786 = t266 * t266;
    real_type t789 = t448 * t446 * t780;
    real_type t791 = t451 * t768 * t2;
    real_type t792 = 4 * t791;
    real_type t793 = t780 * t3;
    real_type t796 = t266 * t268 * (t789 + t792 - t793);
    real_type t814 = t261 * t776;
    real_type t815 = t764 - t814;
    real_type t816 = t446 * t815;
    real_type t817 = t816 * t253 * t448;
    real_type t823 = 2 * t761 * t548 * t765 * t253 - t475 * t815;
    real_type t825 = t765 * t761;
    real_type t826 = t52 * t825;
    real_type t827 = 2 * t826;
    real_type t828 = t92 * t815;
    real_type t831 = t27 * t435 * (t827 + t828);
    real_type t832 = t253 * t815;
    real_type t833 = t446 * t832;
    real_type t834 = t825 * t497;
    real_type t835 = t3 * t832;
    real_type t839 = t253 * t761;
    real_type t843 = t435 * t253;
    real_type t850 = -t763;
    real_type t851 = 2 * t850;
    real_type t853 = 2 * t814;
    real_type t854 = t851 * t761 + t853;
    real_type t855 = t52 * t854;
    real_type t857 = t765 * t92 * t761;
    real_type t858 = t855 + t857;
    real_type t861 = t825 * t471;
    real_type t862 = t89 * t815;
    real_type t863 = t435 * t862;
    real_type t864 = t825 * t478;
    real_type t865 = t446 * t839 * t765 * t448 + t25 * (2 * t843 * t2 * t815 + t761 * t765 * t475) - t27 * t435 * t858 - t861 - t863 - t864;
    real_type t879 = t765 * t765;
    real_type t882 = ModelPars[15];
    real_type t883 = 2 * t879 * t761 + t549 + t778 - t779 + t882;
    real_type t887 = alpha__crw(t675);
    real_type t888 = sin(t887);
    real_type t897 = -t532 * (t526 - t527 + t253) - t288 * t27 * t540 + t555 - t457 * (4 * t786 * t784 + 2 * t770 - 2 * t782 - 2 * t783 - 2 * t796) - t207 * (t220 * (t786 * (2 * t789 + 8 * t791 - 2 * t793) + 4 * t266 * t784 * t268 - t789 - t792 + t793) + t266 * (t25 * t823 - t817 + t831 + t833 - t834 + t835) - t268 * t865) - t220 * (t266 * t865 - t268 * (-t25 * t823 + t817 - t831 - t833 + t834 - t835)) + t553 + 2 * t786 * t784 - t796 + t770 - t25 * t883 * t548 - t888 * t261 * t116 - t783 + t117 - (ModelPars[71] * t265 + t58 * ModelPars[32]) * ModelPars[29];
    real_type t906 = t765 * t261 - t765 * t762;
    real_type t907 = 8 * t906;
    real_type t909 = 8 * t767;
    real_type t910 = t761 * t907 - t909;
    real_type t914 = -t761 * t907 + t909;
    real_type t916 = t49 * t910 + t58 * t914;
    real_type t917 = t435 * t916;
    real_type t920 = t762 * t261;
    real_type t922 = t762 * t762;
    real_type t925 = 4 * t775 - 8 * t920 + 4 * t922 - 4 * t879;
    real_type t927 = 4 * t777;
    real_type t928 = 4 * t778;
    real_type t929 = 4 * t779;
    real_type t930 = t761 * t925 + t927 - t928 + t929;
    real_type t934 = -t761 * t925 - t927 + t928 - t929;
    real_type t938 = t266 * t268;
    real_type t940 = ModelPars[65];
    real_type t941 = cos(t940);
    real_type t942 = t941 * t941;
    real_type t944 = sin(t940);
    real_type t947 = ModelPars[72];
    real_type t948 = ModelPars[68];
    real_type t949 = t948 * t948;
    real_type t950 = t949 * t947;
    real_type t951 = ModelPars[13];
    real_type t952 = ModelPars[18];
    real_type t953 = t950 + t951 - t952;
    real_type t954 = 4 * t953;
    real_type t955 = ModelPars[187];
    real_type t956 = sin(t955);
    real_type t958 = t947 * t948;
    real_type t959 = ModelPars[142];
    real_type t960 = t959 * t958;
    real_type t961 = 4 * t960;
    real_type t963 = cos(t955);
    real_type t965 = 4 * t209;
    real_type t968 = ModelPars[146];
    real_type t969 = t947 * t968;
    real_type t970 = t956 * t948;
    real_type t971 = t970 * t969;
    real_type t972 = 4 * t971;
    real_type t973 = ModelPars[178];
    real_type t976 = t973 * t177 + t973 * t193;
    real_type t977 = 4 * t976;
    real_type t979 = ModelPars[28];
    real_type t980 = ModelPars[24];
    real_type t981 = t980 * t979;
    real_type t982 = ModelPars[27];
    real_type t983 = t982 * t981;
    real_type t984 = 4 * t983;
    real_type t985 = ModelPars[0];
    real_type t986 = 4 * t985;
    real_type t989 = 4 * t906;
    real_type t991 = 4 * t767;
    real_type t992 = t761 * t989 - t991;
    real_type t993 = t58 * t992;
    real_type t994 = t445 * t61;
    real_type t995 = t189 * t994;
    real_type t996 = 2 * t995;
    real_type t998 = t445 * t61 * t190;
    real_type t999 = 2 * t998;
    real_type t1004 = t2 * t934;
    real_type t1005 = t786 * t435;
    real_type t1008 = t268 * t435;
    real_type t1009 = t266 * t1008;
    real_type t1014 = -t954;
    real_type t1015 = t963 * t963;
    real_type t1018 = t963 * t948 * t969;
    real_type t1019 = 4 * t1018;
    real_type t1020 = t190 * t190;
    real_type t1021 = t1020 * t445;
    real_type t1022 = 2 * t1021;
    real_type t1024 = t445 * t189 * t190;
    real_type t1025 = 4 * t1024;
    real_type t1027 = t970 * t947 * t959;
    real_type t1028 = 4 * t1027;
    real_type t1029 = t973 * t189;
    real_type t1031 = t973 * t973;
    real_type t1033 = 4 * t1029 - 2 * t1031;
    real_type t1036 = t968 * t958;
    real_type t1037 = 4 * t1036;
    real_type t1038 = t982 * t982;
    real_type t1039 = t979 * t979;
    real_type t1040 = -t1038 + t1039;
    real_type t1045 = ModelPars[9];
    real_type t1046 = 2 * t1045;
    real_type t1047 = ModelPars[11];
    real_type t1049 = t941 * t944 * t914 + t1015 * t1014 + t445 * t1033 + 2 * t980 * t1040 + t942 * t930 - t1019 + t1022 - t1025 + t1028 + t1037 + t1046 - 2 * t1047 + 4 * t950 + 4 * t951 - 4 * t952;
    real_type t1056 = t49 * t934 + t58 * t930;
    real_type t1057 = t2 * t1056;
    real_type t1062 = 2 * t775;
    real_type t1063 = 4 * t920;
    real_type t1064 = 2 * t922;
    real_type t1066 = -t1062 + t1063 - t1064 + 2 * t879;
    real_type t1068 = 2 * t777;
    real_type t1069 = 2 * t778;
    real_type t1070 = 2 * t779;
    real_type t1071 = t761 * t1066 - t1068 + t1069 - t1070;
    real_type t1072 = t58 * t1071;
    real_type t1075 = t61 * t445 * t295 + t49 * t1049 + t1072;
    real_type t1081 = -t761 * t989 + t991;
    real_type t1082 = t3 * t1081;
    real_type t1089 = -t761 * t1066 + t1068 - t1069 + t1070;
    real_type t1092 = -t953;
    real_type t1093 = 2 * t1092;
    real_type t1095 = 2 * t960;
    real_type t1100 = 2 * t971;
    real_type t1104 = 2 * t983;
    real_type t1105 = 2 * t985;
    real_type t1106 = t942 * t992 + t941 * t944 * t1089 + (t1093 * t956 - t1095) * t963 + t190 * t445 * t195 - t1100 - 2 * t445 * t976 + t1104 + t1095 + t1105;
    real_type t1132 = t942 * t910 + t941 * t944 * t930 + (t1014 * t956 - t961) * t963 - t190 * t445 * t965 - t972 - t445 * t977 + t984 + t961 + t986;
    real_type t1152 = t942 * t1071;
    real_type t1154 = t941 * t944 * t992;
    real_type t1155 = -t1093;
    real_type t1156 = t1015 * t1155;
    real_type t1157 = 2 * t1018;
    real_type t1158 = 2 * t1024;
    real_type t1159 = 2 * t1027;
    real_type t1162 = t445 * (-2 * t1029 + t1031);
    real_type t1163 = 2 * t950;
    real_type t1164 = 2 * t1036;
    real_type t1166 = -t980 * t1040;
    real_type t1167 = 2 * t951;
    real_type t1168 = 2 * t952;
    real_type t1169 = t1152 + t1154 + t1156 + t1157 - t1021 + t1158 - t1159 + t1162 - t1163 - t1164 + t1166 - t1167 + t1168 - t1045 + t1047;
    real_type t1179 = -t851 * t761 - t853;
    real_type t1180 = t58 * t1179;
    real_type t1183 = t435 * (t49 * t253 * t854 + t253 * t1180 - t827);
    real_type t1186 = t49 * t765 * t839;
    real_type t1187 = 2 * t1186;
    real_type t1189 = t58 * t765 * t839;
    real_type t1190 = 2 * t1189;
    real_type t1191 = t52 * t1179;
    real_type t1195 = -t631;
    real_type t1201 = t765 * t761 * t944;
    real_type t1202 = 2 * t1201;
    real_type t1203 = t956 * t958;
    real_type t1204 = 2 * t1203;
    real_type t1205 = t982 * t980;
    real_type t1206 = 2 * t1205;
    real_type t1207 = t941 * t1179 + t1202 + t1204 - t1206;
    real_type t1210 = t253 * t994;
    real_type t1211 = 2 * t1210;
    real_type t1213 = t941 * t52 * t825;
    real_type t1214 = 2 * t1213;
    real_type t1218 = 2 * t963 * t52 * t958;
    real_type t1219 = t445 * t190;
    real_type t1220 = t445 * t973;
    real_type t1222 = -2 * t1219 - 2 * t981 - 2 * t958 + 2 * t1220;
    real_type t1228 = t266 * t548;
    real_type t1229 = t765 * t839;
    real_type t1232 = 4 * t850;
    real_type t1234 = 4 * t814;
    real_type t1235 = t761 * t1232 + t1234;
    real_type t1237 = t268 * t548;
    real_type t1240 = t765 * t761 * t941;
    real_type t1244 = -t761 * t1232 - t1234;
    real_type t1246 = t963 * t958;
    real_type t1255 = -t607 - t609;
    real_type t1256 = 2 * t1255;
    real_type t1257 = t445 * t1256;
    real_type t1258 = t612 * t612;
    real_type t1259 = t611 * t1258;
    real_type t1260 = 2 * t1259;
    real_type t1262 = t611 * t612 * t193;
    real_type t1263 = 2 * t1262;
    real_type t1264 = 2 * t619;
    real_type t1265 = 2 * t616;
    real_type t1266 = 2 * t617;
    real_type t1267 = t1257 - t1260 - t1263 + t670 - t1264 + t1265 - t1266;
    real_type t1271 = t61 * t179 * t656;
    real_type t1275 = 2 * t656 * t55 * t190;
    real_type t1277 = 2 * t626 - 2 * t629 - 2 * t630;
    real_type t1283 = t266 * t2;
    real_type t1285 = t268 * t2;
    real_type t1287 = 2 * t826 * t1285;
    real_type t1292 = t941 * t1191;
    real_type t1295 = 2 * t944 * t52 * t825;
    real_type t1297 = 2 * t1203 - 2 * t1205;
    real_type t1298 = t52 * t1297;
    real_type t1311 = t850 * t761;
    real_type t1312 = t1311 + t814;
    real_type t1313 = t253 * t1312;
    real_type t1315 = t49 * t253;
    real_type t1317 = t58 * t58;
    real_type t1318 = t1317 * t1312;
    real_type t1320 = t1315 * t1180 + t188 * t1313 + t253 * t1318 - t834 + t835;
    real_type t1323 = t825 * t253 * t1317;
    real_type t1324 = t58 * t49;
    real_type t1326 = 2 * t1229 * t1324;
    real_type t1328 = t825 * t253 * t188;
    real_type t1333 = t89 * t1312;
    real_type t1339 = -t981 - t958 + t1220;
    real_type t1343 = t1312 * t941;
    real_type t1344 = t1343 - t1203 - t1201 + t1205;
    real_type t1345 = t1344 * t253;
    real_type t1346 = t3 * t1345;
    real_type t1351 = t815 * t941 + t1201 + t1203 - t1205;
    real_type t1354 = t61 * t49;
    real_type t1357 = 2 * t445 * t253 * t1354;
    real_type t1358 = ModelPars[1];
    real_type t1359 = t92 * t92;
    real_type t1360 = t1359 * t1358;
    real_type t1362 = ModelPars[145] * t1360;
    real_type t1367 = t58 * t854;
    real_type t1370 = t435 * (t49 * t253 * t1179 + t253 * t1367 + t827 + t828);
    real_type t1376 = 2 * t179 * t631;
    real_type t1379 = t941 * t854 - t1202 - t1204 + t1206;
    real_type t1382 = t92 * t1312;
    real_type t1383 = -t827 + t1382;
    real_type t1388 = -t52 * t1222;
    real_type t1412 = 2 * t1240;
    real_type t1414 = 2 * t1246;
    real_type t1415 = 2 * t958;
    real_type t1416 = 2 * t1220;
    real_type t1417 = 2 * t1219;
    real_type t1418 = 2 * t981;
    real_type t1419 = t1179 * t944 - t1412 - t1414 + t1415 - t1416 + t1417 + t1418;
    real_type t1444 = 2 * t826 * t1283;
    real_type t1452 = t179 * t1195 * t188;
    real_type t1457 = t619 * t437 * t55;
    real_type t1460 = t435 * t1333;
    real_type t1473 = t1312 * t944;
    real_type t1474 = t1240 + t1473 + t1246 - t958 + t1220 - t1219 - t981;
    real_type t1475 = t253 * t1474;
    real_type t1476 = t3 * t1475;
    real_type t1478 = t619 * t179 * t437;
    real_type t1484 = t815 * t944 + t1219 - t1220 - t1240 - t1246 + t958 + t981;
    real_type t1488 = ModelPars[140] * t1360;
    real_type t1501 = t944 * t854 + t1412 + t1414 - t1415 + t1416 - t1417 - t1418;
    real_type t1518 = t49 * t1081 + t993;
    real_type t1526 = -t658;
    real_type t1531 = t980 * t52;
    real_type t1532 = t253 * t1531;
    real_type t1533 = 2 * t1532;
    real_type t1535 = -2 * t906;
    real_type t1537 = 2 * t767;
    real_type t1538 = t761 * t1535 + t1537;
    real_type t1548 = t253 * t253;
    real_type t1549 = t1548 * t980;
    real_type t1551 = t1038 * t980;
    real_type t1552 = 2 * t1551;
    real_type t1553 = ModelPars[10];
    real_type t1555 = 2 * t1549 + t1152 + t1154 + t1156 - t1028 + t1552 - t1163 - t1167 + t1168 - t1046 + 2 * t1553;
    real_type t1557 = 2 * t1526;
    real_type t1558 = t55 * t1557;
    real_type t1574 = t1039 * t980;
    real_type t1576 = -t445 * t1033 + t1019 - t1022 + t1025 - t1037 - t1045 + t1047 + t1152 + t1154 + t1156 - t1163 - t1167 + t1168 - t1553 - 2 * t1574;
    real_type t1582 = t112 * t549;
    real_type t1593 = t775 - 2 * t920 + t922 - t879;
    real_type t1600 = t980 * t435 * t89 * t253;
    real_type t1609 = (t953 * t956 + t960) * t963;
    real_type t1619 = t1359 * t253 * t1358;
    real_type t1622 = -t1557;
    real_type t1623 = t179 * t1622;
    real_type t1627 = -t582 * t439 - t596 * t443 - t532 * t4 - t540 * t288 + t604 + t554 - t457 * (t448 * (t786 * t917 + t938 * t435 * (t49 * t930 + t58 * t934) + t435 * (t49 * (t942 * t914 + t941 * t944 * t934 + t963 * (t956 * t954 + t961) + t190 * t445 * t965 + t972 + t445 * t977 - t984 - t961 - t986) + t993 + t996 - t999)) + t25 * (t27 * (t1009 * t2 * t910 + t435 * t2 * t1049 + t1005 * t1004) + t786 * t1057 + t938 * t2 * t916 + t2 * t1075) + t27 * (t938 * t3 * t1071 + t786 * t1082 + t3 * t1106)) - t207 * (t220 * (t448 * (t786 * t435 * t1056 + t435 * t1075 + t938 * t917) + t25 * (t27 * (t1005 * t2 * t914 + t435 * t2 * t1132 + t1009 * t1004) + t786 * t2 * (t49 * t914 + t58 * t910) + t938 * t1057 + t2 * (t58 * t1081 + t49 * t1132 - t996 + t999)) + t27 * (t786 * t3 * t1089 + t938 * t1082 + t3 * t1169)) + t448 * (t266 * t1183 + t268 * t435 * (-t1187 + t1190 + t1191) + t435 * (4 * t2 * t1195 * t179 + t49 * t253 * t1207 + t52 * t1222 + t944 * t855 - t1211 + t1214 + t1218)) + t25 * (t27 * (4 * t1229 * t1228 + t1237 * t253 * t1235 + t435 * (t2 * t253 * (t944 * t1244 + 4 * t1219 - 4 * t1220 - 4 * t1240 - 4 * t1246 + 4 * t958 + 4 * t981) + t49 * t179 * t1267 - 2 * t1271 - t1275 + t55 * t1277)) + t1283 * t855 - t1287 + t3 * t179 * (t445 * t1255 - t1259 - t1262 + t616 - t617 + t618 - t619) + t2 * (t1292 + t1295 + t1298) + t188 * t179 * (-t445 * t1255 + t1259 + t1262 - t616 + t617 - t618 + t619) + t49 * t179 * (t745 - t688)) + t27 * (t266 * t1320 + t268 * (t863 - t1323 + t1326 + t864 - t1328) + t435 * (-t445 * t89 * t190 + t825 * t89 * t941 + t958 * t89 * t963 + t944 * t1333 + t89 * t1339) + t1346 + t2 * t55 * (t1257 - t1260 - t1263 - t619 + t616 - t617 - t671) + t188 * t253 * t1351 - t1357 + t1362) + t266 * t1370 + t268 * t435 * (t1187 - t1190 + t855 + t857) + t435 * (t2 * t1376 + t49 * t253 * t1379 + t1211 + t941 * t1383 + t944 * (t1191 - t857) - t1218 + t1388 - t958 * t92 * t956 + t982 * t92 * t980)) - t220 * (t448 * (t266 * t435 * (t1187 - t1190 + t855) + t268 * t1183 + t435 * (t2 * t179 * (4 * t445 * t1255 - 4 * t1259 - 4 * t1262 + 4 * t616 - 4 * t617 + 4 * t618 - 4 * t619) + t49 * t253 * t1419 + t1292 + t1295 + t1298)) + t25 * (t27 * (t1228 * t253 * t1244 + 4 * t1229 * t1237 + t435 * (t2 * t253 * (t941 * t1235 - 4 * t1201 - 4 * t1203 + 4 * t1205) + t49 * t1376 + t55 * t1267)) + t1444 + t1285 * t855 + t3 * t179 * t631 + t2 * (t944 * t1191 - t1214 - t1218 + t1388) + t1452 - t49 * t55 * t672 - t1457) + t27 * (t266 * (t1460 + t1323 - t1326 - t864 + t1328) + t268 * t1320 + t435 * (t944 * t89 * t825 + t956 * t89 * t958 - t980 * t89 * t982 + t941 * t862) + t1476 + t2 * (-t55 * t1277 + t1275 + t1478) + t188 * t253 * t1484 + t1488) + t266 * t435 * (-t1187 + t1190 + t1191 - t857) + t268 * t1370 + t435 * (t2 * t179 * (-t445 * t1256 + t1260 + t1263 + t1264 - t1265 + t1266 - t670) + t49 * t253 * t1501 + t941 * t858 + t944 * t1383 + t958 * t92 * t963 - t52 * t1297 - t445 * t92 * t190 + t92 * t1339)) - t448 * (t786 * t435 * t1518 + t938 * t435 * (t49 * t1071 + t58 * t1089) + t435 * (4 * t488 * t179 * t1526 + t49 * t1106 + t58 * t1538 - t1533)) - t25 * (t27 * (t1005 * t2 * t1089 + t1009 * t2 * t1081 + t435 * (t179 * t52 * t1557 + t2 * t1555 + t253 * t1558)) + t786 * t2 * (t49 * t1089 + t1072) + t938 * t2 * t1518 + t680 + t2 * (t49 * t1576 + t58 * (t761 * (t1062 - t1063 + t1064) + t1068 + t549 - t778 + t882 + t779) + t688 + t1582)) - t27 * (t786 * t3 * (-t761 * t1535 - t1537) + t938 * t3 * (t761 * t1593 + t777 - t778 + t779) - t1600 + t3 * (t942 * t1538 + t941 * t944 * (-t761 * t1593 - t777 + t778 - t779) + t1609 + t190 * t445 * t209 + t971 + t445 * t976 - t983 - t960 - t985) + t49 * (-2 * t998 + 2 * t995) + t1619) - t435 * (t488 * t1623 + t1533 - t996 + t999);
    real_type t1630 = t324 * t324;
    real_type t1631 = t3 + t1630;
    real_type t1632 = t1631 * t761;
    real_type t1635 = t1631 * t815;
    real_type t1641 = t815 * t324;
    real_type t1643 = t268 * t324;
    real_type t1645 = t49 * t1351;
    real_type t1656 = 2 * t994;
    real_type t1670 = t253 * t980;
    real_type t1671 = t3 * t1670;
    real_type t1676 = t179 * t1526;
    real_type t1677 = t3 * t1676;
    real_type t1679 = 2 * t2 * t1531;
    real_type t1687 = t446 * t825;
    real_type t1689 = t188 * t825;
    real_type t1691 = 2 * t1324 * t825;
    real_type t1692 = t1317 * t825;
    real_type t1693 = t3 * t825 + t1687 + t1689 - t1691 + t1692;
    real_type t1695 = t188 * t1312;
    real_type t1696 = t49 * t1180;
    real_type t1698 = t446 * t1312;
    real_type t1701 = t188 * t1484;
    real_type t1703 = t446 * t1484;
    real_type t1706 = t2 * t854;
    real_type t1709 = t2 * t1180 + t49 * t1706;
    real_type t1716 = -2 * t49 * t2 * t825 + 2 * t2 * t58 * t825;
    real_type t1730 = -2 * t49 * t435 * t825 + 2 * t58 * t435 * t825;
    real_type t1738 = t435 * t49 * t1501;
    real_type t1750 = 2 * t445 * t1354;
    real_type t1758 = t2 * t1419;
    real_type t1789 = t435 * (t49 * t1179 + t1367);
    real_type t1794 = 2 * t49 * t825 - 2 * t58 * t825;
    real_type t1805 = 2 * t25 * t548 * t825 + t1318 + t1695 + t1696 + t1698;
    real_type t1844 = t439 * t146;
    real_type t1845 = t443 * t146;
    real_type t1852 = -t773 * t771 * t761 - t777 + t778 - t779;
    real_type t1855 = t765 * t1311 - t767;
    real_type t1858 = -t1852;
    real_type t1859 = t1858 * t942;
    real_type t1861 = t1855 * t944 * t941;
    real_type t1862 = 2 * t1861;
    real_type t1863 = t1092 * t1015;
    real_type t1864 = t1021 / 2;
    real_type t1867 = (t189 - t973 / 2) * t1220;
    real_type t1869 = t980 * t1040 / 2;
    real_type t1870 = t1045 / 2;
    real_type t1871 = t1047 / 2;
    real_type t1872 = t1852 * t786 + 2 * t1855 * t938 - t1018 - t1024 + t1027 + t1036 + t1859 - t1862 + t1863 + t1864 + t1867 + t1869 + t1870 - t1871 + t950 + t951 - t952;
    real_type t1874 = t448 * t2;
    real_type t1877 = t1855 * t324;
    real_type t1878 = t786 * t1877;
    real_type t1879 = 2 * t1878;
    real_type t1880 = t1858 * t324;
    real_type t1881 = t938 * t1880;
    real_type t1882 = -t1855;
    real_type t1883 = 2 * t1882;
    real_type t1884 = t1883 * t942;
    real_type t1886 = t944 * t1858 * t941;
    real_type t1888 = t445 * t194 * t190;
    real_type t1889 = t194 * t445;
    real_type t1890 = t973 * t1889;
    real_type t1891 = t1884 - t1886 + t1609 + t971 - t1888 + t1890 - t983 - t960 - t985;
    real_type t1892 = t49 * t1891;
    real_type t1893 = t1855 * t58;
    real_type t1895 = t61 * t445 * t191;
    real_type t1897 = -t1879 - t1881 + t1892 + t1893 + t1895 / 2;
    real_type t1901 = t435 + t2;
    real_type t1902 = t435 - t2;
    real_type t1903 = t1902 * t1901;
    real_type t1904 = -t1883;
    real_type t1908 = t268 * t1858 * t266 + t1904 * t786 + t1609 + t1884 - t1886 - t1888 + t1890 - t960 + t971 - t983 - t985;
    real_type t1913 = t786 * t1880;
    real_type t1916 = t266 * t1855 * t1643;
    real_type t1919 = 2 * t942 * t1852;
    real_type t1920 = 4 * t1861;
    real_type t1921 = t1919 + t1920 + t1156 + t1157 - t1159 - t1021 + t1158 + t1162 - t1163 - t1164 + t1166 - t1167 + t1168 - t1045 + t1047;
    real_type t1922 = t49 * t1921;
    real_type t1923 = t58 * t1858;
    real_type t1925 = t445 * t194 * t61;
    real_type t1926 = -2 * t1913 + 4 * t1916 + t1922 + t1923 + t1925;
    real_type t1951 = t2 * t1475;
    real_type t1953 = t177 * t1889;
    real_type t1954 = t1953 + t1259 + t1262 - t618 + t619 - t616 + t617;
    real_type t1955 = t179 * t1954;
    real_type t1957 = t1195 * t55;
    real_type t1965 = -t253 * t58 * t1312 + t49 * t1313 - t826;
    real_type t1966 = t1965 / 2;
    real_type t1968 = t1312 * t52;
    real_type t1969 = -t1186 + t1189 - t1968;
    real_type t1970 = t1969 / 2;
    real_type t1972 = t1195 * t179;
    real_type t1974 = t49 * t1345;
    real_type t1979 = (t1473 + t1246 + t1220 - t1219 - t981 - t958) * t52;
    real_type t1986 = t58 - t2 - t49;
    real_type t1987 = t58 + t2 - t49;
    real_type t1990 = t1987 * t1986 * t1313 + t446 * t1313 - t834;
    real_type t1995 = -t1987 * t1986 * t765 * t839 - t1460 - t861;
    real_type t2010 = t179 * t1954 * t446;
    real_type t2012 = t1344 * t52;
    real_type t2014 = 2 * t2 * t2012;
    real_type t2015 = t49 * t1954;
    real_type t2021 = t475 * t1312;
    real_type t2024 = t825 * t475 * t268;
    real_type t2028 = 2 * t435 * (t2015 + t687 - t689) * t179;
    real_type t2029 = t475 * t1344;
    real_type t2036 = t2 * t1345;
    real_type t2038 = t49 * t1972;
    real_type t2069 = t179 * t1195 * t446;
    real_type t2075 = (t618 - t684 + t681 - t682 + t683) * t55;
    real_type t2082 = t268 * t2021;
    real_type t2084 = 2 * t2038 - 2 * t2075;
    real_type t2086 = t475 * t1474;
    real_type t2096 = t1859 - t1862 - t1549 + t1863 + t1159 - t1551 + t950 + t951 - t952 + t1045 - t1553;
    real_type t2099 = t179 * t52;
    real_type t2130 = t786 * t2;
    real_type t2133 = t266 * t1285;
    real_type t2136 = t1858 / 2;
    real_type t2147 = t942 * t2136 - t1861 + t1092 * t1015 / 2 - t1018 + t1864 - t1024 + t1867 + t1574 / 2 + t950 / 2 + t1036 + t1045 / 4 + t1553 / 4 - t1047 / 4 + t951 / 2 - t952 / 2;
    real_type t2149 = t850 * t850;
    real_type t2172 = t1844 - t302 * t1845 + t532 * t9 - t457 * (4 * t1874 * t1872 * t435 + t25 * (-4 * t27 * t435 * t1897 + 2 * t1908 * t1903) + 2 * t2 * t27 * t1926) - t207 * (t220 * (-4 * t1874 * t1908 * t435 + t25 * (2 * t1902 * t1872 * t1901 + 2 * t27 * t435 * t1926) + 4 * t2 * t27 * t1897) - 2 * t448 * (-2 * t1229 * t1283 - 2 * t1285 * t1313 + t49 * t1955 + t1271 + 2 * t1951 - t1957) * t435 + t25 * (-4 * t27 * (t266 * t1966 + t268 * t1970 + t2 * t1972 - t1974 / 2 + t1213 / 2 - t1210 / 2 + t1979 / 2) * t435 + t266 * t1990 + t268 * t1995 - t1344 * t471 + t435 * t89 * t1474 + t1346 - 2 * t2 * (t1953 + t1259 + t684 - t681 + t682 + t683 + t1262) * t55 - t188 * t1345 - t1357 + t1362) + t27 * (-2 * t1283 * t1968 + t1287 - t2010 + t3 * t1955 + t2014 - t179 * (t2015 + t745 - t688) * t49) + t266 * t2021 - t2024 + t2028 - t2029) - t220 * (-2 * t448 * t435 * (-2 * t1229 * t1285 + 2 * t1283 * t1313 + t55 * t1954 - 2 * t2036 + t2038) + t25 * (4 * t27 * t435 * (t266 * t1970 - t268 * t1966 + t2 * t1955 + t49 * t1475 / 2 + t2012 / 2) - t266 * t1995 + t268 * t1990 - t446 * t1475 - t1344 * t497 + t1476 + t2 * (t1478 - 2 * t1957) - t188 * t1475 + t1488) + t27 * (2 * t2 * t52 * t1474 - 2 * t1285 * t1968 + t3 * t1972 + 2 * t49 * t2075 - t1444 - t1452 + t1457 - t2069) + t825 * t475 * t266 + t2082 + t435 * t2084 - t2086) - t434 * t316 + t529 * t288 + 2 * t448 * (-t786 * t2 * t1858 + 2 * t938 * t2 * t1855 + t2 * t2096 + t658 * (t253 * t55 + t2099)) * t435 - t25 * (2 * t27 * t435 * (-t1879 - t1881 + 2 * t660 + t1892 + t1532 + t1893) - 2 * t786 * t1855 * t1903 - t266 * t268 * t1902 * t1858 * t1901 + t446 * (t1904 * t942 + t1886 + (t1092 * t956 - t960) * t963 + t1888 - t971 - t1890 + t983 + t960 + t985) - t1600 + t3 * t1891 + 2 * t49 * t191 * t994 + t1619) - t27 * (2 * t2130 * t1880 - 4 * t2133 * t1877 - t680 + 4 * (t49 * t2147 + t58 * (-t2149 * t761 / 2 - t882 / 4 - t549 / 4 + t778 / 4 - t779 / 4 - t777 / 2) - t1582 / 4 - t688 / 4) * t2) - t435 * (t2 * (t1045 - t1553 + t1047) + 2 * t658 * t2099) + t677;
    real_type t2181 = t58 * t253;
    real_type t2182 = t92 / 2;
    real_type t2183 = t731 + t2181 - t1315 + t2182;
    real_type t2186 = t761 * t268;
    real_type t2191 = t2181 - t1315 + t2182;
    real_type t2205 = -t2 * t1969;
    real_type t2212 = -t2 * t1965;
    real_type t2230 = t730 + 2 * t58 - 2 * t49;
    real_type t2233 = t2230 * t1855;
    real_type t2239 = t1919 + t1920 + t1156 - t1028 + t1552 - t1163 - t1167 + t1168 - t1045 + t1553 + t1047;
    real_type t2262 = t266 * t2230;
    real_type t2275 = t49 * (4 * t1882 * t942 - 2 * t1886 + (t1155 * t956 + t1095) * t963 + t1100 - 2 * t1888 + 2 * t1890 - t1104 - t1095 - t1105);
    real_type t2276 = t58 * t1904;
    real_type t2282 = t1859 - t1862 + t1863 - t1018 + t1027 + t1864 - t1024 + t1867 + t950 + t1036 + t1869 + t951 - t952 + t1870 - t1871;
    real_type t2326 = t220 * (-2 * t25 * (-t2262 * t268 * t1858 - t27 * t435 * t1891 - 2 * t786 * t2233 + t1895 + t2275 + t2276) * t435 - 4 * (t1913 - 2 * t1916 + t49 * t2282 - t58 * t2136 - t1925 / 2) * t2) - 2 * t1972 * t698 + 2 * t25 * t435 * (-t266 * t761 * t2183 * t765 + t27 * (t815 * t268 - t1219 + t1220 + t1240 + t1246 + t1473 - t958 - t981) * t843 - t268 * t1312 * t2191 - (t1315 - t2182) * t1474) + t266 * (-t27 * t475 * t825 + 2 * t2212) + t27 * (-t435 * t2084 - t2082 + t2086) + 2 * t268 * t2205 + t2069 + t1452 + t49 * (2 * t2036 - 2 * t2075) + t2 * (-2 * t1213 + 2 * t1210 - 2 * t1979) - t1457;
    real_type t2352 = Mxr(t675);
    real_type t2353 = -t302 * t1844 - 2 * t49 * t2 * t1891 - t220 * (2 * t1955 * t698 + 2 * t25 * t435 * (t266 * t1312 * t2183 - t695 * t435 * (t765 * t2186 - t1201 - t1203 + t1205 + t1343) - t2186 * t2191 * t765 + t1974 - t941 * t1382 / 2 + t1210 - (-t1203 - t1201 + t1205) * t92 / 2) + t266 * (t27 * t2021 - 2 * t2205) + t27 * (-t2024 + t2028 - t2029) + 2 * t268 * t2212 - t2010 - t179 * t1954 * t188 + t49 * (2 * t1951 - 2 * (t687 - t689) * t179) + t2014) - t1845 - t540 * t9 + t434 * t144 + t529 * t4 - t25 * (-t786 * t2230 * t1858 * t435 + 2 * t266 * t2233 * t1008 + t27 * t446 * t2096 + t435 * (-t980 * t92 * t253 + t49 * t2239 - t58 * t883 - t1582 + 2 * t1925 - t688) + t703) - (-t92 * t179 * t729 - t980 * t475 * t253) * t27 - t2 * (2 * t1532 + 2 * t1893) - t207 * t2326 - t457 * (-2 * t25 * t435 * (2 * t2262 * t1855 * t268 - t786 * t2230 * t1858 + t27 * t2282 * t435 + t1922 + t1923 + t1925) - 2 * t2 * (-4 * t1878 - 2 * t1881 + t2275 + t2276 + t1895)) - 2 * t700 + 2 * t2133 * t1880 + t655 * t658 * t446 + 4 * t2130 * t1877 + t676 + t2352;
    real_type t2357 = t179 * t316;
    real_type t2359 = t194 * t302;
    real_type t2365 = t179 * t146;
    real_type t2366 = t293 * t302;
    real_type t2370 = t179 * t293;
    real_type t2378 = t179 * t144;
    real_type t2397 = t619 * t548;
    real_type t2398 = t134 * t100;
    real_type t2439 = pow(t265 - ModelPars[66], 2);
    real_type t2444 = pow(t253 - ModelPars[69], 2);
    real_type t2449 = pow(t52 - ModelPars[164], 2);
    real_type t2454 = pow(t190 - ModelPars[180], 2);
    real_type t2459 = pow(t204 - ModelPars[189], 2);
    real_type t2464 = pow(t55 - ModelPars[158], 2);
    real_type t2469 = pow(t81 - ModelPars[238], 2);
    real_type t2474 = pow(t58 - ModelPars[163], 2);
    real_type t2479 = pow(t2 - ModelPars[175], 2);
    real_type t2484 = pow(t84 - ModelPars[176], 2);
    real_type t2489 = pow(t61 - ModelPars[179], 2);
    real_type t2494 = pow(t49 - ModelPars[188], 2);
    real_type t2499 = pow(t64 - ModelPars[224], 2);
    real_type t2504 = pow(t73 - ModelPars[226], 2);
    real_type t2509 = pow(t67 - ModelPars[228], 2);
    real_type t2514 = pow(t18 - ModelPars[230], 2);
    real_type t2519 = pow(t70 - ModelPars[234], 2);
    real_type t2524 = pow(t14 - ModelPars[236], 2);
    real_type t2527 = ModelPars[198] * t2439 + ModelPars[201] * t2444 + ModelPars[246] * t2449 + ModelPars[209] * t2454 + ModelPars[212] * t2459 + ModelPars[242] * t2464 + ModelPars[244] * t2469 + ModelPars[245] * t2474 + ModelPars[247] * t2479 + ModelPars[249] * t2484 + ModelPars[251] * t2489 + ModelPars[253] * t2494 + ModelPars[255] * t2499 + ModelPars[257] * t2504 + ModelPars[259] * t2509 + ModelPars[261] * t2514 + ModelPars[263] * t2519 + ModelPars[265] * t2524;
    real_type t2531 = X__[37];
    real_type t2532 = Q__[1];
    real_type t2534 = t2532 * t2531 - 1;
    real_type t2536 = t92 * t88;
    real_type t2537 = t89 * t91;
    real_type t2539 = 1.0 / (t2536 - t2537);
    real_type t2541 = t519 * L__[8] + (-t532 * (t261 * t268 * t207 - t261 * t266 * t220 + t253 + t526 - t527) - t540 * (-t261 * t268 * t220 - t261 * t536 + t288) * t27 - t25 * t536 * t542 - t220 * t25 * t268 * t542 - t25 * t549 * t548 + t553 + t555 + t117 + t107) * L__[10] + (-t207 * (t179 * (t188 - t21) - t369) - 2 * t220 * t375 * t49 - t25 * (t302 * (-t3 + t21) + 2 * t7 * t6 * t81) + 2 * (t302 * t8 + t81) * t2 * t27) * L__[11] + t754 * L__[6] + t897 * L__[7] + t1627 * L__[5] + (-t207 * (t25 * (-t266 * t765 * t1632 + t268 * t1635 + (t3 + t188) * t1474) + 2 * t27 * t2 * (t266 * t1641 + t1643 * t825 + t1645 - t994)) - t220 * (t25 * (-t266 * t1635 - t268 * t765 * t1632 + t3 * t1351 + (t1645 - t1656) * t49) - 2 * t27 * t2 * (t1474 * t49 - t268 * t1641 + t325 * t825)) + t434 + t529 - t25 * (t2 * t55 * t1622 - t1671) - t27 * (t1677 - t1679) - t980 * t475) * L__[2] + (t302 * t439 + t443 + t540 - t207 * (t27 * (t266 * t1693 + t268 * (t3 * t1312 + t1318 + t1695 + t1696 + t1698) + t1701 + t3 * t1484 + t1703) + t25 * (t49 * t2 * t1207 - 2 * t61 * t2 * t445 + t266 * t1709 + t268 * t1716) + t266 * t1730 + t268 * (t49 * t435 * t1179 + t435 * t1367) + t1738) - t220 * (t27 * (t266 * (t1317 * t815 + t49 * t1367 + t188 * t815 + t3 * t815 + t816) + t268 * t1693 + t188 * t1344 + t1750 + t3 * t1344 + t446 * t1344) + t25 * (t268 * t1709 - t266 * t1716 + t49 * t1758) + t266 * (t49 * t435 * t854 + t435 * t1180) + t268 * t1730 + t49 * t435 * t1207 - 2 * t445 * t435 * t61) - t27 * (t2 * t1558 + t980 * t471 + t1671) - t25 * (t446 * t1676 + t1677 - t1679) - t980 * t479) * L__[1] + (t439 - t302 * t443 + t532 - t207 * (t27 * (t266 * t1789 + t268 * t435 * t1794 + t435 * (t49 * t1379 + t1656)) + t266 * t1805 + t268 * (t451 * t1706 - t1687 - t1689 + t1691 - t1692) + t451 * t1758 + t446 * t1351 + t188 * t1351 - t1750) - t220 * (t27 * (-t266 * t435 * t1794 + t268 * t1789 + t1738) + t266 * (t451 * t2 * t1179 + t1687 + t1689 - t1691 + t1692) + t268 * t1805 + t451 * t2 * t1379 + t1703 + t1701) - t27 * t435 * (t2 * t1623 + 2 * t1531) - t25 * t435 * (2 * t2 * t1670 + t1558) - t1360 + t980 * t497) * L__[0] + t2172 * L__[4] + t2353 * L__[3] + (-t443 * (t220 * (t27 * t191 + t25 * t2357 - t2359) + t207 * (t253 * t25 * t179 + t27 * t194 - t2365 - t2366) + t27 * t596 + t25 * (t2370 + t598) - t302 * t253) - t439 * (t220 * (t27 * (-t2365 - t2366) - t25 * t2378 + t193 + t177) + t207 * (t27 * t2359 - t189 + t190) + t27 * t582 - t25 * t146 + t253) - t604 * (t179 * t220 * t25 - t27) - t220 * (t434 * (t27 * t2357 + t293 * t25) - t27 * (-t2397 + t2398) * t179 + t25 * (t100 * t179 * t141 - t619 * t55 * t435) + t619 * t49 * t2 * t179) - t207 * (t434 * (t25 * t209 + t253 * t396 + t2378) - t435 * t619 * t25 * t49 * t179 - t2 * t55 * t619 + t179 * t676) - t434 * (t27 * t2370 + t25 * t316) + t27 * t141 * t100 + (-t2397 + t2398 + t677) * t25) * L__[9] - t2539 * t2534 * (ModelPars[134] * t2527 + ModelPars[137]);
    real_type t2542 = t2539 * t2534;
    real_type t2544 = 1.0 / ModelPars[153];
    real_type t2546 = OnlyBrakingRear(-t107 * t2544);
    real_type t2550 = roadLeftLateralBorder(t2531 + Q__[2]);
    real_type t2554 = roadRightLateralBorder(Q__[3] - t2531);
    real_type t2556 = MaxRollAngle(t24);
    real_type t2558 = MaxSteerAngle(t179);
    real_type t2568 = LatSlipFront(t436);
    real_type t2570 = LongSlipFront(t438);
    real_type t2572 = ModelPars[8];
    real_type t2573 = 1.0 / t2572;
    real_type t2576 = FrontWheelContact((t434 - t2572) * t2573);
    real_type t2578 = LongSlipRear(t531);
    real_type t2580 = LatSlipRear(t530);
    real_type t2584 = RearWheelContact((t529 - t2572) * t2573);
    real_type t2587 = OnlyTractionRear(t116 * t2544);
    real_type t2590 = OnlyBrakingFront(-t100 * t2544);
    real_type t2592 = -t2546 * t2542 - t2550 * t2542 - t2554 * t2542 - t2556 * t2542 - t2558 * t2542 + 1.0 / t2534 * (t2532 * (t435 * t2531 + t2536 - t2537) - t435) * L__[38] - t2568 * t2542 - t2570 * t2542 - t2576 * t2542 - t2578 * t2542 - t2580 * t2542 - t2584 * t2542 - t2587 * t2542 - t2590 * t2542;
    return t79 + t428 + t2541 + t2592;
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
    real_type t1   = X__[37];
    real_type t5   = X__[38];
    real_type t6   = cos(t5);
    real_type t7   = X__[0];
    real_type t9   = sin(t5);
    real_type t10  = X__[1];
    real_type t14  = 1.0 / (-t10 * t9 + t7 * t6) * (Q__[1] * t1 - 1);
    real_type t16  = 1.0 / ModelPars[153];
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
    real_type t14  = 1.0 / (X__[0] * t6 - X__[1] * t9) * (X__[37] * Q__[1] - 1);
    real_type t18  = t__oControl(U__[2], ModelPars[157], ModelPars[174]);
    real_type t21  = ModelPars[170];
    real_type t22  = ModelPars[30];
    real_type t23  = b__f__oControl(U__[0], t21, t22);
    real_type t26  = b__r__oControl(U__[1], t21, t22);
    real_type t29  = ModelPars[181];
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
    real_type t11  = pow(X__[5] - ModelPars[69], 2);
    real_type t17  = pow(X__[21] - ModelPars[164], 2);
    real_type t23  = pow(X__[8] - ModelPars[180], 2);
    real_type t29  = pow(X__[4] - ModelPars[189], 2);
    real_type t35  = pow(X__[22] - ModelPars[158], 2);
    real_type t41  = pow(X__[31] - ModelPars[238], 2);
    real_type t47  = pow(X__[23] - ModelPars[163], 2);
    real_type t53  = pow(X__[19] - ModelPars[175], 2);
    real_type t59  = pow(X__[32] - ModelPars[176], 2);
    real_type t65  = pow(X__[24] - ModelPars[179], 2);
    real_type t71  = pow(X__[20] - ModelPars[188], 2);
    real_type t77  = pow(X__[25] - ModelPars[224], 2);
    real_type t83  = pow(X__[28] - ModelPars[226], 2);
    real_type t89  = pow(X__[26] - ModelPars[228], 2);
    real_type t95  = pow(X__[29] - ModelPars[230], 2);
    real_type t101 = pow(X__[27] - ModelPars[234], 2);
    real_type t107 = pow(X__[30] - ModelPars[236], 2);
    real_type t110 = ModelPars[198] * t5 + ModelPars[201] * t11 + ModelPars[246] * t17 + ModelPars[209] * t23 + ModelPars[212] * t29 + ModelPars[242] * t35 + ModelPars[244] * t41 + ModelPars[245] * t47 + ModelPars[247] * t53 + ModelPars[249] * t59 + ModelPars[251] * t65 + ModelPars[253] * t71 + ModelPars[255] * t77 + ModelPars[257] * t83 + ModelPars[259] * t89 + ModelPars[261] * t95 + ModelPars[263] * t101 + ModelPars[265] * t107;
    real_type t119 = X__[38];
    real_type t120 = cos(t119);
    real_type t123 = sin(t119);
    return -1.0 / (X__[0] * t120 - X__[1] * t123) * (X__[37] * Q__[1] - 1) * (ModelPars[134] * t110 + ModelPars[137]);
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
    real_type t1   = XL__[25];
    real_type t4   = pow(t1 - ModelPars[224], 2);
    real_type t5   = ModelPars[254];
    real_type t7   = XL__[26];
    real_type t10  = pow(t7 - ModelPars[228], 2);
    real_type t11  = ModelPars[258];
    real_type t13  = XL__[0];
    real_type t16  = pow(t13 - ModelPars[126], 2);
    real_type t17  = ModelPars[213];
    real_type t19  = XL__[1];
    real_type t22  = pow(t19 - ModelPars[127], 2);
    real_type t23  = ModelPars[214];
    real_type t25  = XL__[2];
    real_type t28  = pow(t25 - ModelPars[156], 2);
    real_type t29  = ModelPars[193];
    real_type t31  = XL__[3];
    real_type t34  = pow(t31 - ModelPars[91], 2);
    real_type t35  = ModelPars[205];
    real_type t37  = XL__[4];
    real_type t40  = pow(t37 - ModelPars[189], 2);
    real_type t41  = ModelPars[211];
    real_type t43  = XL__[5];
    real_type t46  = pow(t43 - ModelPars[69], 2);
    real_type t47  = ModelPars[199];
    real_type t49  = XL__[6];
    real_type t52  = pow(t49 - ModelPars[160], 2);
    real_type t53  = ModelPars[194];
    real_type t55  = XL__[7];
    real_type t58  = pow(t55 - ModelPars[66], 2);
    real_type t59  = ModelPars[196];
    real_type t61  = XL__[8];
    real_type t64  = pow(t61 - ModelPars[180], 2);
    real_type t65  = ModelPars[208];
    real_type t67  = XL__[9];
    real_type t70  = pow(t67 - ModelPars[225], 2);
    real_type t71  = ModelPars[215];
    real_type t73  = XL__[10];
    real_type t76  = pow(t73 - ModelPars[229], 2);
    real_type t77  = ModelPars[218];
    real_type t79  = XL__[11];
    real_type t82  = pow(t79 - ModelPars[235], 2);
    real_type t83  = ModelPars[220];
    real_type t85  = XL__[12];
    real_type t88  = pow(t85 - ModelPars[227], 2);
    real_type t89  = ModelPars[216];
    real_type t91  = XL__[23];
    real_type t94  = pow(t91 - ModelPars[163], 2);
    real_type t95  = ModelPars[197];
    real_type t97  = XL__[24];
    real_type t100 = pow(t97 - ModelPars[179], 2);
    real_type t101 = ModelPars[250];
    real_type t103 = XL__[13];
    real_type t106 = pow(t103 - ModelPars[231], 2);
    real_type t107 = ModelPars[219];
    real_type t109 = XL__[14];
    real_type t112 = pow(t109 - ModelPars[237], 2);
    real_type t113 = ModelPars[221];
    real_type t115 = t5 * t4 + t11 * t10 + t17 * t16 + t23 * t22 + t29 * t28 + t35 * t34 + t41 * t40 + t47 * t46 + t53 * t52 + t59 * t58 + t65 * t64 + t71 * t70 + t77 * t76 + t83 * t82 + t89 * t88 + t95 * t94 + t101 * t100 + t107 * t106 + t113 * t112;
    real_type t116 = XL__[15];
    real_type t119 = pow(t116 - ModelPars[159], 2);
    real_type t120 = ModelPars[195];
    real_type t122 = XL__[16];
    real_type t125 = pow(t122 - ModelPars[177], 2);
    real_type t126 = ModelPars[207];
    real_type t128 = XL__[17];
    real_type t131 = pow(t128 - ModelPars[173], 2);
    real_type t132 = ModelPars[204];
    real_type t134 = XL__[18];
    real_type t137 = pow(t134 - ModelPars[171], 2);
    real_type t138 = ModelPars[203];
    real_type t140 = XL__[19];
    real_type t143 = pow(t140 - ModelPars[175], 2);
    real_type t144 = ModelPars[206];
    real_type t146 = XL__[20];
    real_type t149 = pow(t146 - ModelPars[188], 2);
    real_type t150 = ModelPars[252];
    real_type t152 = XL__[33];
    real_type t155 = pow(t152 - ModelPars[4], 2);
    real_type t156 = ModelPars[190];
    real_type t158 = XL__[21];
    real_type t161 = pow(t158 - ModelPars[164], 2);
    real_type t162 = ModelPars[200];
    real_type t164 = XL__[22];
    real_type t167 = pow(t164 - ModelPars[158], 2);
    real_type t168 = ModelPars[241];
    real_type t170 = XL__[34];
    real_type t173 = pow(t170 - ModelPars[25], 2);
    real_type t174 = ModelPars[191];
    real_type t176 = XL__[35];
    real_type t179 = pow(t176 - ModelPars[26], 2);
    real_type t180 = ModelPars[192];
    real_type t182 = XL__[36];
    real_type t185 = pow(t182 - ModelPars[121], 2);
    real_type t186 = ModelPars[210];
    real_type t188 = XL__[37];
    real_type t191 = pow(t188 - ModelPars[76], 2);
    real_type t192 = ModelPars[202];
    real_type t194 = XL__[38];
    real_type t197 = pow(t194 - ModelPars[144], 2);
    real_type t198 = ModelPars[217];
    real_type t200 = XL__[27];
    real_type t203 = pow(t200 - ModelPars[234], 2);
    real_type t204 = ModelPars[262];
    real_type t206 = XL__[28];
    real_type t209 = pow(t206 - ModelPars[226], 2);
    real_type t210 = ModelPars[256];
    real_type t212 = XL__[29];
    real_type t215 = pow(t212 - ModelPars[230], 2);
    real_type t216 = ModelPars[260];
    real_type t218 = XL__[30];
    real_type t221 = pow(t218 - ModelPars[236], 2);
    real_type t222 = ModelPars[264];
    real_type t224 = XL__[31];
    real_type t227 = pow(t224 - ModelPars[238], 2);
    real_type t228 = ModelPars[243];
    real_type t230 = XL__[32];
    real_type t233 = pow(t230 - ModelPars[176], 2);
    real_type t234 = ModelPars[248];
    real_type t236 = t120 * t119 + t126 * t125 + t132 * t131 + t138 * t137 + t144 * t143 + t150 * t149 + t156 * t155 + t162 * t161 + t168 * t167 + t174 * t173 + t180 * t179 + t186 * t185 + t192 * t191 + t198 * t197 + t204 * t203 + t210 * t209 + t216 * t215 + t222 * t221 + t228 * t227 + t234 * t233;
    real_type t241 = t158 * t158;
    real_type t243 = t170 * t170;
    real_type t245 = t176 * t176;
    real_type t247 = t182 * t182;
    real_type t249 = t188 * t188;
    real_type t251 = t194 * t194;
    real_type t253 = XR__[0];
    real_type t254 = t253 * t253;
    real_type t256 = XR__[29];
    real_type t257 = t256 * t256;
    real_type t259 = XR__[30];
    real_type t260 = t259 * t259;
    real_type t262 = XR__[31];
    real_type t263 = t262 * t262;
    real_type t265 = XR__[35];
    real_type t266 = t265 * t265;
    real_type t268 = XR__[36];
    real_type t269 = t268 * t268;
    real_type t271 = XR__[37];
    real_type t272 = t271 * t271;
    real_type t274 = XR__[38];
    real_type t275 = t274 * t274;
    real_type t277 = t241 * t162 + t254 * t17 + t243 * t174 + t245 * t180 + t266 * t180 + t247 * t186 + t269 * t186 + t249 * t192 + t272 * t192 + t251 * t198 + t275 * t198 + t257 * t216 + t260 * t222 + t263 * t228;
    real_type t278 = XR__[1];
    real_type t279 = t278 * t278;
    real_type t281 = XR__[2];
    real_type t282 = t281 * t281;
    real_type t284 = XR__[3];
    real_type t285 = t284 * t284;
    real_type t287 = XR__[4];
    real_type t288 = t287 * t287;
    real_type t290 = XR__[5];
    real_type t291 = t290 * t290;
    real_type t293 = XR__[6];
    real_type t294 = t293 * t293;
    real_type t296 = XR__[7];
    real_type t297 = t296 * t296;
    real_type t299 = XR__[8];
    real_type t300 = t299 * t299;
    real_type t302 = XR__[9];
    real_type t303 = t302 * t302;
    real_type t307 = pow(t73 - XR__[10], 2);
    real_type t309 = XR__[11];
    real_type t310 = t309 * t309;
    real_type t312 = XR__[12];
    real_type t313 = t312 * t312;
    real_type t315 = XR__[13];
    real_type t316 = t315 * t315;
    real_type t318 = XR__[14];
    real_type t319 = t318 * t318;
    real_type t321 = t316 * t107 + t319 * t113 + t279 * t23 + t282 * t29 + t285 * t35 + t288 * t41 + t291 * t47 + t294 * t53 + t297 * t59 + t300 * t65 + t303 * t71 + t77 * t307 + t310 * t83 + t313 * t89;
    real_type t323 = XR__[15];
    real_type t324 = t323 * t323;
    real_type t326 = XR__[16];
    real_type t327 = t326 * t326;
    real_type t329 = XR__[17];
    real_type t330 = t329 * t329;
    real_type t332 = XR__[18];
    real_type t333 = t332 * t332;
    real_type t335 = XR__[19];
    real_type t336 = t335 * t335;
    real_type t338 = XR__[20];
    real_type t339 = t338 * t338;
    real_type t341 = XR__[21];
    real_type t342 = t341 * t341;
    real_type t344 = XR__[22];
    real_type t345 = t344 * t344;
    real_type t347 = XR__[23];
    real_type t348 = t347 * t347;
    real_type t350 = XR__[24];
    real_type t351 = t350 * t350;
    real_type t353 = XR__[25];
    real_type t354 = t353 * t353;
    real_type t356 = XR__[26];
    real_type t357 = t356 * t356;
    real_type t359 = XR__[27];
    real_type t360 = t359 * t359;
    real_type t362 = XR__[28];
    real_type t363 = t362 * t362;
    real_type t365 = t351 * t101 + t357 * t11 + t324 * t120 + t327 * t126 + t330 * t132 + t333 * t138 + t336 * t144 + t339 * t150 + t342 * t162 + t345 * t168 + t360 * t204 + t363 * t210 + t348 * t95 + t354 * t5;
    real_type t366 = t13 * t13;
    real_type t368 = t19 * t19;
    real_type t370 = t25 * t25;
    real_type t372 = t31 * t31;
    real_type t374 = t37 * t37;
    real_type t376 = t164 * t164;
    real_type t378 = t91 * t91;
    real_type t380 = t97 * t97;
    real_type t382 = t1 * t1;
    real_type t384 = t7 * t7;
    real_type t386 = t200 * t200;
    real_type t388 = t206 * t206;
    real_type t390 = t212 * t212;
    real_type t392 = t218 * t218;
    real_type t394 = t224 * t224;
    real_type t396 = t380 * t101 + t384 * t11 + t376 * t168 + t366 * t17 + t386 * t204 + t388 * t210 + t390 * t216 + t392 * t222 + t394 * t228 + t368 * t23 + t370 * t29 + t372 * t35 + t374 * t41 + t378 * t95 + t382 * t5;
    real_type t399 = t43 * t43;
    real_type t401 = t49 * t49;
    real_type t403 = t55 * t55;
    real_type t405 = t61 * t61;
    real_type t407 = t67 * t67;
    real_type t409 = t79 * t79;
    real_type t411 = t85 * t85;
    real_type t413 = t103 * t103;
    real_type t415 = t109 * t109;
    real_type t417 = t230 * t230;
    real_type t419 = t152 * t152;
    real_type t421 = XR__[32];
    real_type t422 = t421 * t421;
    real_type t424 = XR__[33];
    real_type t425 = t424 * t424;
    real_type t427 = XR__[34];
    real_type t428 = t427 * t427;
    real_type t430 = t413 * t107 + t415 * t113 + t419 * t156 + t425 * t156 + t428 * t174 + t417 * t234 + t422 * t234 + t399 * t47 + t401 * t53 + t403 * t59 + t405 * t65 + t407 * t71 + t409 * t83 + t411 * t89;
    real_type t431 = t116 * t116;
    real_type t433 = t122 * t122;
    real_type t435 = t128 * t128;
    real_type t437 = t134 * t134;
    real_type t439 = t140 * t140;
    real_type t441 = t146 * t146;
    real_type t470 = t431 * t120 + t433 * t126 + t435 * t132 + t437 * t138 + t439 * t144 + t441 * t150 - 2 * t344 * t164 * t168 - 2 * t347 * t91 * t95 - 2 * t350 * t97 * t101 - 2 * t353 * t1 * t5 - 2 * t356 * t7 * t11 - 2 * t359 * t200 * t204 - 2 * t362 * t206 * t210 - 2 * t256 * t212 * t216 - 2 * t259 * t218 * t222;
    real_type t500 = -t262 * t224 * t228 - t421 * t230 * t234 - t424 * t152 * t156 - t427 * t170 * t174 - t265 * t176 * t180 - t268 * t182 * t186 - t271 * t188 * t192 - t274 * t194 * t198 - t253 * t13 * t17 - t278 * t19 * t23 - t281 * t25 * t29 - t284 * t31 * t35 - t287 * t37 * t41 - t290 * t43 * t47;
    real_type t532 = -t293 * t49 * t53 - t296 * t55 * t59 - t299 * t61 * t65 - t302 * t67 * t71 - t309 * t79 * t83 - t312 * t85 * t89 - t315 * t103 * t107 - t318 * t109 * t113 - t323 * t116 * t120 - t326 * t122 * t126 - t329 * t128 * t132 - t332 * t134 * t138 - t335 * t140 * t144 - t338 * t146 * t150 - t341 * t158 * t162;
    return ModelPars[131] * (t115 + t236) + (t277 + t321 + t365 + t396 + t430 + t470 + 2 * t500 + 2 * t532) * ModelPars[128];
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
    ok = ok && t__oControl.check_range(U__[2], ModelPars[157], ModelPars[174]);
    real_type t5   = ModelPars[170];
    real_type t6   = ModelPars[30];
    ok = ok && b__f__oControl.check_range(U__[0], t5, t6);
    ok = ok && b__r__oControl.check_range(U__[1], t5, t6);
    real_type t9   = ModelPars[181];
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
    result__[ 0   ] = t__oControl(U__[2], ModelPars[157], ModelPars[174]);
    real_type t5   = ModelPars[170];
    real_type t6   = ModelPars[30];
    result__[ 1   ] = b__f__oControl(U__[0], t5, t6);
    result__[ 2   ] = b__r__oControl(U__[1], t5, t6);
    real_type t9   = ModelPars[181];
    result__[ 3   ] = tau__oControl(U__[3], -t9, t9);
    real_type t11  = 1.0 / ModelPars[153];
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
    real_type t83  = ModelPars[141];
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
    real_type t139 = X__[32];
    real_type t140 = t139 * t71;
    real_type t143 = t139 * t74;
    real_type t154 = t50 * t71;
    real_type t155 = t130 * t67;
    real_type t163 = t50 * t74;
    result__[ 47  ] = -t130 * t69 * t67 * t163 - t73 * t132 * t80 * t71 - t136 * t80 * t69 * t154 + t136 * t73 * t80 * t163 + t80 * t69 * t50 * t140 - t73 * t129 * t143 - t132 * t80 * t76 - t73 * t155 * t154 + t136 * t76 + t136 * t77 - t73 * t140 - t69 * t143;
    real_type t170 = X__[24];
    real_type t172 = t134 * t71;
    real_type t176 = t130 * t80;
    real_type t182 = t74 * t36;
    result__[ 48  ] = -t130 * t67 * t87 - t130 * t80 * t82 - t90 * t139 * t182 + t91 * t139 * t182 + t86 * t155 - t80 * t170 - t90 * t172 + t91 * t172 - t83 * t176 - t38;
    real_type t194 = t136 * t69;
    real_type t196 = t90 * t73;
    real_type t198 = t91 * t73;
    real_type t200 = t90 * t136;
    real_type t202 = t91 * t136;
    real_type t204 = t90 * t69;
    real_type t206 = t91 * t69;
    result__[ 49  ] = -t136 * t69 * t20 + t136 * t73 * t35 + t132 * t83 + t196 * t140 - t198 * t140 + t204 * t143 - t206 * t143 - t91 * t194 - t200 * t76 - t200 * t77 + t202 * t76 + t202 * t77 - t73 * t21 - t69 * t39;
    real_type t210 = X__[21];
    result__[ 50  ] = t130 * t67 * t82 - t130 * t80 * t87 + t136 * t73 * t20 + t136 * t69 * t35 + t91 * t137 + t204 * t140 - t206 * t140 - t196 * t143 + t198 * t143 + t83 * t155 + t67 * t170 + t86 * t176 - t200 * t72 + t200 * t75 + t202 * t72 - t202 * t75 - t69 * t21 + t73 * t39 + t210;
    real_type t233 = t111 * t113;
    real_type t234 = X__[23];
    real_type t235 = t67 * t234;
    real_type t238 = t115 * t113;
    real_type t239 = t80 * t234;
    result__[ 51  ] = -t155 * t233 + t176 * t238 + t235 * t233 - t239 * t238 + t44;
    result__[ 52  ] = -t136 * t69 * t25 - t118 * t194 + t136 * t127 - t26 * t73 - t69 * t48;
    result__[ 53  ] = t136 * t73 * t25 + t118 * t137 + t136 * t121 - t155 * t238 - t176 * t233 + t239 * t233 + t235 * t238 - t26 * t69 + t73 * t48 + t210;
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

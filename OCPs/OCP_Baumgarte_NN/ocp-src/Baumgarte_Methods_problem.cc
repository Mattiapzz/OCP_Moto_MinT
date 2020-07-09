/*-----------------------------------------------------------------------*\
 |  file: Baumgarte_Methods1.cc                                          |
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


#include "Baumgarte.hh"
#include "Baumgarte_Pars.hh"

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
#define ALIAS_MaxBetaAngle_DD(__t1) MaxBetaAngle.DD( __t1)
#define ALIAS_MaxBetaAngle_D(__t1) MaxBetaAngle.D( __t1)
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


namespace BaumgarteDefine {
  /*\
   |   ___         _   _               _   _
   |  / __|___ _ _| |_(_)_ _ _  _ __ _| |_(_)___ _ _
   | | (__/ _ \ ' \  _| | ' \ || / _` |  _| / _ \ ' \
   |  \___\___/_||_\__|_|_||_\_,_\__,_|\__|_\___/_||_|
  \*/

  void
  Baumgarte::continuationStep0( real_type s ) {
    int msg_level = 3;
    pConsole->message(
      fmt::format( "\nContinuation step N.0 s = {}\n", s ),
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
  Baumgarte::continuationStep1( real_type s ) {
    int msg_level = 3;
    pConsole->message(
      fmt::format( "\nContinuation step N.1 s = {}\n", s ),
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
  Baumgarte::continuationStep2( real_type s ) {
    int msg_level = 3;
    pConsole->message(
      fmt::format( "\nContinuation step N.2 s = {}\n", s ),
      msg_level
    );
    real_type t1   = ModelPars[246];
    ModelPars[171] = t1 + (ModelPars[247] - t1) * s;
    real_type t5   = ModelPars[248];
    ModelPars[172] = t5 + (ModelPars[249] - t5) * s;
  }
  /*\
   |   ___         _   _               _   _
   |  / __|___ _ _| |_(_)_ _ _  _ __ _| |_(_)___ _ _
   | | (__/ _ \ ' \  _| | ' \ || / _` |  _| / _ \ ' \
   |  \___\___/_||_\__|_|_||_\_,_\__,_|\__|_\___/_||_|
  \*/

  void
  Baumgarte::continuationStep3( real_type s ) {
    int msg_level = 3;
    pConsole->message(
      fmt::format( "\nContinuation step N.3 s = {}\n", s ),
      msg_level
    );
    real_type t1   = ModelPars[128];
    ModelPars[127] = t1 + (ModelPars[129] - t1) * s;
  }
  /*\
   |   ___         _   _               _   _
   |  / __|___ _ _| |_(_)_ _ _  _ __ _| |_(_)___ _ _
   | | (__/ _ \ ' \  _| | ' \ || / _` |  _| / _ \ ' \
   |  \___\___/_||_\__|_|_||_\_,_\__,_|\__|_\___/_||_|
  \*/

  void
  Baumgarte::continuationStep4( real_type s ) {
    int msg_level = 3;
    pConsole->message(
      fmt::format( "\nContinuation step N.4 s = {}\n", s ),
      msg_level
    );
    real_type t1   = ModelPars[59];
    real_type t5   = t1 + (ModelPars[60] - t1) * s;
    b__f__oControl.update_epsilon(t5);
    real_type t6   = ModelPars[120];
    real_type t10  = t6 + (ModelPars[121] - t6) * s;
    b__f__oControl.update_tolerance(t10);
    t__oControl.update_epsilon(t5);
    t__oControl.update_tolerance(t10);
    b__r__oControl.update_epsilon(t5);
    b__r__oControl.update_tolerance(t10);
    tau__oControl.update_epsilon(t5);
    tau__oControl.update_tolerance(t10);
    real_type t11  = ModelPars[61];
    real_type t15  = t11 + (ModelPars[62] - t11) * s;
    FrontWheelContact.update_epsilon(t15);
    real_type t16  = ModelPars[122];
    real_type t20  = t16 + (ModelPars[123] - t16) * s;
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
  }

  /*\
   |  _   _               ___             _   _
   | | | | |___ ___ _ _  | __|  _ _ _  __| |_(_)___ _ _  ___
   | | |_| (_-</ -_) '_| | _| || | ' \/ _|  _| / _ \ ' \(_-<
   |  \___//__/\___|_|   |_| \_,_|_||_\__|\__|_\___/_||_/__/
  \*/
  // user defined functions which has a body defined in MAPLE
  real_type
  Baumgarte::alpha__crw( real_type t__XO ) const {
    return asin(1.0 / ModelPars[24] * (ModelPars[112] - ModelPars[113]));
  }

  real_type
  Baumgarte::alpha__crw_D( real_type t__XO ) const {
    return 0;
  }

  real_type
  Baumgarte::alpha__crw_DD( real_type t__XO ) const {
    return 0;
  }

  real_type
  Baumgarte::alpha__pin( real_type eta__XO, real_type alpha__crw__XO ) const {
    return alpha__crw__XO + eta__XO;
  }

  real_type
  Baumgarte::alpha__pin_D_1( real_type eta__XO, real_type alpha__crw__XO ) const {
    return 1;
  }

  real_type
  Baumgarte::alpha__pin_D_1_1( real_type eta__XO, real_type alpha__crw__XO ) const {
    return 0;
  }

  real_type
  Baumgarte::alpha__pin_D_1_2( real_type eta__XO, real_type alpha__crw__XO ) const {
    return 0;
  }

  real_type
  Baumgarte::alpha__pin_D_2( real_type eta__XO, real_type alpha__crw__XO ) const {
    return 1;
  }

  real_type
  Baumgarte::alpha__pin_D_2_2( real_type eta__XO, real_type alpha__crw__XO ) const {
    return 0;
  }

  real_type
  Baumgarte::Fzf( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return -z__f__XO * ModelPars[21] - z__f__dot__XO * ModelPars[2];
  }

  real_type
  Baumgarte::Fzf_D_1( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return -ModelPars[21];
  }

  real_type
  Baumgarte::Fzf_D_1_1( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return 0;
  }

  real_type
  Baumgarte::Fzf_D_1_2( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return 0;
  }

  real_type
  Baumgarte::Fzf_D_2( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return -ModelPars[2];
  }

  real_type
  Baumgarte::Fzf_D_2_2( real_type z__f__XO, real_type z__f__dot__XO ) const {
    return 0;
  }

  real_type
  Baumgarte::Fzr( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return -z__r__XO * ModelPars[22] - z__r__dot__XO * ModelPars[3];
  }

  real_type
  Baumgarte::Fzr_D_1( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return -ModelPars[22];
  }

  real_type
  Baumgarte::Fzr_D_1_1( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return 0;
  }

  real_type
  Baumgarte::Fzr_D_1_2( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return 0;
  }

  real_type
  Baumgarte::Fzr_D_2( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return -ModelPars[3];
  }

  real_type
  Baumgarte::Fzr_D_2_2( real_type z__r__XO, real_type z__r__dot__XO ) const {
    return 0;
  }

  real_type
  Baumgarte::alpha__r( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t7   = atan((-Omega__XO * x__r__XO + v__XO + y__r__dot__XO) / (-y__r__XO * Omega__XO + u__XO - x__r__dot__XO));
    return -t7;
  }

  real_type
  Baumgarte::alpha__r_D_1( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t6   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t7   = t2 * t2;
    real_type t8   = 1.0 / t7;
    real_type t12  = t6 * t6;
    return -1.0 / (t8 * t12 + 1) * (-1.0 / t2 * x__r__XO + y__r__XO * t8 * t6);
  }

  real_type
  Baumgarte::alpha__r_D_1_1( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Baumgarte::alpha__r_D_1_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Baumgarte::alpha__r_D_1_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Baumgarte::alpha__r_D_1_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Baumgarte::alpha__r_D_1_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Baumgarte::alpha__r_D_1_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Baumgarte::alpha__r_D_1_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Baumgarte::alpha__r_D_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t5   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t6   = 1.0 / t5;
    real_type t8   = t2 * t2;
    return 1.0 / (t6 * t8 + 1) * t6 * t2;
  }

  real_type
  Baumgarte::alpha__r_D_2_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Baumgarte::alpha__r_D_2_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t7   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t9   = t4 * t7 + 1;
    real_type t12  = t3 * t3;
    real_type t15  = t9 * t9;
    return 1.0 / t9 * t4 - 2 / t15 / t12 * t7;
  }

  real_type
  Baumgarte::alpha__r_D_2_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t8   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t10  = t4 * t8 + 1;
    real_type t13  = t3 * t3;
    real_type t16  = t10 * t10;
    return -1.0 / t10 * t4 * Omega__XO + 2 * Omega__XO / t16 / t13 * t8;
  }

  real_type
  Baumgarte::alpha__r_D_2_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Baumgarte::alpha__r_D_2_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Baumgarte::alpha__r_D_2_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t7   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t9   = t4 * t7 + 1;
    real_type t12  = t3 * t3;
    real_type t15  = t9 * t9;
    return 1.0 / t9 * t4 - 2 / t15 / t12 * t7;
  }

  real_type
  Baumgarte::alpha__r_D_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t6   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t7   = t2 * t2;
    return -1.0 / (1.0 / t7 * t6 + 1) / t2;
  }

  real_type
  Baumgarte::alpha__r_D_3_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t12  = pow(1.0 / t3 * t8 + 1, 2);
    return 2 * t7 / t12 / t3 / t2;
  }

  real_type
  Baumgarte::alpha__r_D_3_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t12  = pow(1.0 / t3 * t8 + 1, 2);
    return -2 * Omega__XO * t7 / t12 / t3 / t2;
  }

  real_type
  Baumgarte::alpha__r_D_3_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t8   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t10  = t4 * t8 + 1;
    real_type t13  = t3 * t3;
    real_type t16  = t10 * t10;
    return -1.0 / t10 * t4 * Omega__XO + 2 * Omega__XO / t16 / t13 * t8;
  }

  real_type
  Baumgarte::alpha__r_D_3_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t7   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t9   = t4 * t7 + 1;
    real_type t12  = t3 * t3;
    real_type t15  = t9 * t9;
    return -1.0 / t9 * t4 + 2 / t15 / t12 * t7;
  }

  real_type
  Baumgarte::alpha__r_D_3_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t12  = pow(1.0 / t3 * t8 + 1, 2);
    return 2 * t7 / t12 / t3 / t2;
  }

  real_type
  Baumgarte::alpha__r_D_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t7   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t8   = t2 * t2;
    return 1.0 / (1.0 / t8 * t7 + 1) / t2 * Omega__XO;
  }

  real_type
  Baumgarte::alpha__r_D_4_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t1   = Omega__XO * Omega__XO;
    real_type t3   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t4   = t3 * t3;
    real_type t9   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t10  = t9 * t9;
    real_type t14  = pow(1.0 / t4 * t10 + 1, 2);
    return 2 * t9 / t14 / t4 / t3 * t1;
  }

  real_type
  Baumgarte::alpha__r_D_4_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Baumgarte::alpha__r_D_4_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t8   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t10  = t4 * t8 + 1;
    real_type t13  = t3 * t3;
    real_type t16  = t10 * t10;
    return 1.0 / t10 * t4 * Omega__XO - 2 * Omega__XO / t16 / t13 * t8;
  }

  real_type
  Baumgarte::alpha__r_D_4_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t12  = pow(1.0 / t3 * t8 + 1, 2);
    return -2 * Omega__XO * t7 / t12 / t3 / t2;
  }

  real_type
  Baumgarte::alpha__r_D_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t5   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t6   = 1.0 / t5;
    real_type t8   = t2 * t2;
    return -1.0 / (t6 * t8 + 1) * Omega__XO * t6 * t2;
  }

  real_type
  Baumgarte::alpha__r_D_5_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Baumgarte::alpha__r_D_5_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Baumgarte::alpha__r_D_5_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t8   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t10  = t4 * t8 + 1;
    real_type t13  = t3 * t3;
    real_type t16  = t10 * t10;
    return -1.0 / t10 * t4 * Omega__XO + 2 * Omega__XO / t16 / t13 * t8;
  }

  real_type
  Baumgarte::alpha__r_D_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t5   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t6   = 1.0 / t5;
    real_type t8   = t2 * t2;
    return -1.0 / (t6 * t8 + 1) * t6 * t2;
  }

  real_type
  Baumgarte::alpha__r_D_6_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
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
  Baumgarte::alpha__r_D_6_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t3   = pow(-y__r__XO * Omega__XO + u__XO - x__r__dot__XO, 2);
    real_type t4   = 1.0 / t3;
    real_type t7   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t9   = t4 * t7 + 1;
    real_type t12  = t3 * t3;
    real_type t15  = t9 * t9;
    return -1.0 / t9 * t4 + 2 / t15 / t12 * t7;
  }

  real_type
  Baumgarte::alpha__r_D_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t6   = pow(-Omega__XO * x__r__XO + v__XO + y__r__dot__XO, 2);
    real_type t7   = t2 * t2;
    return -1.0 / (1.0 / t7 * t6 + 1) / t2;
  }

  real_type
  Baumgarte::alpha__r_D_7_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const {
    real_type t2   = -y__r__XO * Omega__XO + u__XO - x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t7   = -Omega__XO * x__r__XO + v__XO + y__r__dot__XO;
    real_type t8   = t7 * t7;
    real_type t12  = pow(1.0 / t3 * t8 + 1, 2);
    return 2 * t7 / t12 / t3 / t2;
  }

  real_type
  Baumgarte::alpha__f( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t2   = y__f__XO * Omega__XO;
    real_type t5   = x__f__XO * Omega__XO;
    real_type t12  = atan(1.0 / (-t2 + u__XO + x__f__dot__XO + delta__f__XO * (t5 + v__XO + y__f__dot__XO)) * (-x__f__dot__XO * delta__f__XO + y__f__dot__XO + (t2 - u__XO) * delta__f__XO + t5 + v__XO));
    return -t12;
  }

  real_type
  Baumgarte::alpha__f_D_1( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_1_1( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_1_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_1_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_1_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_1_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_1_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_1_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_1_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_2_2( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_2_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_2_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_2_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_2_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_2_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_2_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_3_3( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_3_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_3_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_3_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_3_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_3_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_4_4( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_4_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_4_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_4_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_4_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_5_5( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_5_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_5_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_5_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_6_6( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_6_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_6_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_7_7( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_7_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::alpha__f_D_8_8( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
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
  Baumgarte::lambda__r( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t4   = ModelPars[117];
    real_type t6   = cos(phi__XO);
    return (-x__r__dot__XO - t6 * omega__r__XO * t4 + (-ModelPars[115] + t4) * omega__r__XO - t1 + u__XO) / (t1 - u__XO + x__r__dot__XO);
  }

  real_type
  Baumgarte::lambda__r_D_1( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t5   = ModelPars[117];
    real_type t7   = cos(phi__XO);
    return -y__r__XO * (-x__r__dot__XO - t7 * omega__r__XO * t5 + (-ModelPars[115] + t5) * omega__r__XO - t1 + u__XO) / t3 - y__r__XO / t2;
  }

  real_type
  Baumgarte::lambda__r_D_1_1( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[117];
    real_type t8   = cos(phi__XO);
    real_type t15  = y__r__XO * y__r__XO;
    return 2 * t15 * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[115] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 * t15 / t3;
  }

  real_type
  Baumgarte::lambda__r_D_1_2( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t7   = sin(phi__XO);
    return -y__r__XO * t7 * omega__r__XO * ModelPars[117] / t3;
  }

  real_type
  Baumgarte::lambda__r_D_1_3( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[117];
    real_type t8   = cos(phi__XO);
    return -2 * y__r__XO * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[115] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 - 2 * y__r__XO / t3;
  }

  real_type
  Baumgarte::lambda__r_D_1_4( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[117];
    real_type t8   = cos(phi__XO);
    real_type t13  = -x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[115] + t6) * omega__r__XO - t1 + u__XO;
    real_type t17  = 1.0 / t3;
    return 2 * t1 * t13 / t3 / t2 + 2 * y__r__XO * Omega__XO * t17 - t13 * t17 - 1.0 / t2;
  }

  real_type
  Baumgarte::lambda__r_D_1_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t5   = ModelPars[117];
    real_type t6   = cos(phi__XO);
    return -y__r__XO * (-t6 * t5 + t5 - ModelPars[115]) / t3;
  }

  real_type
  Baumgarte::lambda__r_D_1_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[117];
    real_type t8   = cos(phi__XO);
    return 2 * y__r__XO * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[115] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 * y__r__XO / t3;
  }

  real_type
  Baumgarte::lambda__r_D_2( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t6   = sin(phi__XO);
    return t6 * omega__r__XO / (y__r__XO * Omega__XO - u__XO + x__r__dot__XO) * ModelPars[117];
  }

  real_type
  Baumgarte::lambda__r_D_2_2( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t6   = cos(phi__XO);
    return t6 * omega__r__XO / (y__r__XO * Omega__XO - u__XO + x__r__dot__XO) * ModelPars[117];
  }

  real_type
  Baumgarte::lambda__r_D_2_3( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t7   = sin(phi__XO);
    return t7 * omega__r__XO * ModelPars[117] / t3;
  }

  real_type
  Baumgarte::lambda__r_D_2_4( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t7   = sin(phi__XO);
    return -Omega__XO * t7 * omega__r__XO * ModelPars[117] / t3;
  }

  real_type
  Baumgarte::lambda__r_D_2_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t6   = sin(phi__XO);
    return t6 * ModelPars[117] / (y__r__XO * Omega__XO - u__XO + x__r__dot__XO);
  }

  real_type
  Baumgarte::lambda__r_D_2_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t7   = sin(phi__XO);
    return -t7 * omega__r__XO * ModelPars[117] / t3;
  }

  real_type
  Baumgarte::lambda__r_D_3( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t5   = ModelPars[117];
    real_type t7   = cos(phi__XO);
    return (-x__r__dot__XO - t7 * omega__r__XO * t5 + (-ModelPars[115] + t5) * omega__r__XO - t1 + u__XO) / t3 + 1.0 / t2;
  }

  real_type
  Baumgarte::lambda__r_D_3_3( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[117];
    real_type t8   = cos(phi__XO);
    return 2 * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[115] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 / t3;
  }

  real_type
  Baumgarte::lambda__r_D_3_4( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[117];
    real_type t8   = cos(phi__XO);
    return -2 * Omega__XO * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[115] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 - 2 * Omega__XO / t3;
  }

  real_type
  Baumgarte::lambda__r_D_3_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t5   = ModelPars[117];
    real_type t6   = cos(phi__XO);
    return (-t6 * t5 + t5 - ModelPars[115]) / t3;
  }

  real_type
  Baumgarte::lambda__r_D_3_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[117];
    real_type t8   = cos(phi__XO);
    return -2 * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[115] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 - 2 / t3;
  }

  real_type
  Baumgarte::lambda__r_D_4( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t5   = ModelPars[117];
    real_type t7   = cos(phi__XO);
    return -Omega__XO * (-x__r__dot__XO - t7 * omega__r__XO * t5 + (-ModelPars[115] + t5) * omega__r__XO - t1 + u__XO) / t3 - Omega__XO / t2;
  }

  real_type
  Baumgarte::lambda__r_D_4_4( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[117];
    real_type t8   = cos(phi__XO);
    real_type t15  = Omega__XO * Omega__XO;
    return 2 * t15 * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[115] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 * t15 / t3;
  }

  real_type
  Baumgarte::lambda__r_D_4_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t5   = ModelPars[117];
    real_type t6   = cos(phi__XO);
    return -Omega__XO * (-t6 * t5 + t5 - ModelPars[115]) / t3;
  }

  real_type
  Baumgarte::lambda__r_D_4_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[117];
    real_type t8   = cos(phi__XO);
    return 2 * Omega__XO * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[115] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 * Omega__XO / t3;
  }

  real_type
  Baumgarte::lambda__r_D_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t4   = ModelPars[117];
    real_type t5   = cos(phi__XO);
    return (-t5 * t4 + t4 - ModelPars[115]) / (y__r__XO * Omega__XO - u__XO + x__r__dot__XO);
  }

  real_type
  Baumgarte::lambda__r_D_5_5( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    return 0;
  }

  real_type
  Baumgarte::lambda__r_D_5_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t3   = pow(y__r__XO * Omega__XO - u__XO + x__r__dot__XO, 2);
    real_type t5   = ModelPars[117];
    real_type t6   = cos(phi__XO);
    return -(-t6 * t5 + t5 - ModelPars[115]) / t3;
  }

  real_type
  Baumgarte::lambda__r_D_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t5   = ModelPars[117];
    real_type t7   = cos(phi__XO);
    return -(-x__r__dot__XO - t7 * omega__r__XO * t5 + (-ModelPars[115] + t5) * omega__r__XO - t1 + u__XO) / t3 - 1.0 / t2;
  }

  real_type
  Baumgarte::lambda__r_D_6_6( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const {
    real_type t1   = y__r__XO * Omega__XO;
    real_type t2   = t1 - u__XO + x__r__dot__XO;
    real_type t3   = t2 * t2;
    real_type t6   = ModelPars[117];
    real_type t8   = cos(phi__XO);
    return 2 * (-x__r__dot__XO - t8 * omega__r__XO * t6 + (-ModelPars[115] + t6) * omega__r__XO - t1 + u__XO) / t3 / t2 + 2 / t3;
  }

  real_type
  Baumgarte::lambda__f( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t8   = ModelPars[116];
    real_type t10  = cos(phi__f__XO);
    return (-x__f__dot__XO - t1 + t10 * omega__f__XO * t8 - t3 * delta__f__XO + t5 + (ModelPars[114] - t8) * omega__f__XO - u__XO) / (t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO);
  }

  real_type
  Baumgarte::lambda__f_D_1( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[116];
    real_type t11  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * x__f__XO - y__f__XO;
    return -t21 * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[114] - t9) * omega__f__XO - u__XO) / t7 - t21 / t6;
  }

  real_type
  Baumgarte::lambda__f_D_1_1( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t23  = t22 * t22;
    return 2 * t23 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * t22 / t7;
  }

  real_type
  Baumgarte::lambda__f_D_1_2( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return (delta__f__XO * x__f__XO - y__f__XO) * t11 * omega__f__XO * ModelPars[116] / t7;
  }

  real_type
  Baumgarte::lambda__f_D_1_3( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t25  = 1.0 / t7;
    return 2 * t22 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * t25;
  }

  real_type
  Baumgarte::lambda__f_D_1_4( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t26  = 1.0 / t7;
    return 2 * delta__f__XO * t22 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * delta__f__XO * t26;
  }

  real_type
  Baumgarte::lambda__f_D_1_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO;
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t27  = 1.0 / t7;
    return 2 * delta__f__XO * Omega__XO * t22 * t19 / t7 / t6 + 2 * t22 * delta__f__XO * Omega__XO * t27 - delta__f__XO * t19 * t27 - delta__f__XO / t6;
  }

  real_type
  Baumgarte::lambda__f_D_1_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO;
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t26  = 1.0 / t7;
    return -2 * Omega__XO * t22 * t19 / t7 / t6 - 2 * t22 * Omega__XO * t26 + t19 * t26 + 1.0 / t6;
  }

  real_type
  Baumgarte::lambda__f_D_1_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO;
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t23  = t2 + v__XO + y__f__dot__XO;
    real_type t27  = 1.0 / t7;
    return 2 * t23 * t22 * t19 / t7 / t6 + 2 * t22 * t23 * t27 - x__f__XO * t19 * t27 - x__f__XO / t6;
  }

  real_type
  Baumgarte::lambda__f_D_1_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[116];
    real_type t10  = cos(phi__f__XO);
    return -(delta__f__XO * x__f__XO - y__f__XO) * (t10 * t9 - t9 + ModelPars[114]) / t7;
  }

  real_type
  Baumgarte::lambda__f_D_1_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t25  = 1.0 / t7;
    return 2 * t22 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * t25;
  }

  real_type
  Baumgarte::lambda__f_D_1_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t22  = delta__f__XO * x__f__XO - y__f__XO;
    real_type t26  = 1.0 / t7;
    return 2 * delta__f__XO * t22 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * delta__f__XO * t26;
  }

  real_type
  Baumgarte::lambda__f_D_2( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t10  = sin(phi__f__XO);
    return -t10 * omega__f__XO / (x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO) * ModelPars[116];
  }

  real_type
  Baumgarte::lambda__f_D_2_2( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t10  = cos(phi__f__XO);
    return -t10 * omega__f__XO / (x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO) * ModelPars[116];
  }

  real_type
  Baumgarte::lambda__f_D_2_3( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return t11 * omega__f__XO * ModelPars[116] / t7;
  }

  real_type
  Baumgarte::lambda__f_D_2_4( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return delta__f__XO * t11 * omega__f__XO * ModelPars[116] / t7;
  }

  real_type
  Baumgarte::lambda__f_D_2_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t12  = sin(phi__f__XO);
    return delta__f__XO * Omega__XO * t12 * omega__f__XO * ModelPars[116] / t7;
  }

  real_type
  Baumgarte::lambda__f_D_2_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return -Omega__XO * t11 * omega__f__XO * ModelPars[116] / t7;
  }

  real_type
  Baumgarte::lambda__f_D_2_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t2   = x__f__XO * Omega__XO;
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + (t2 + v__XO) * delta__f__XO - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return (t2 + v__XO + y__f__dot__XO) * t11 * omega__f__XO * ModelPars[116] / t7;
  }

  real_type
  Baumgarte::lambda__f_D_2_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t10  = sin(phi__f__XO);
    return -t10 * ModelPars[116] / (x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO);
  }

  real_type
  Baumgarte::lambda__f_D_2_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return t11 * omega__f__XO * ModelPars[116] / t7;
  }

  real_type
  Baumgarte::lambda__f_D_2_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t11  = sin(phi__f__XO);
    return delta__f__XO * t11 * omega__f__XO * ModelPars[116] / t7;
  }

  real_type
  Baumgarte::lambda__f_D_3( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[116];
    real_type t11  = cos(phi__f__XO);
    return -(-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[114] - t9) * omega__f__XO - u__XO) / t7 - 1.0 / t6;
  }

  real_type
  Baumgarte::lambda__f_D_3_3( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    return 2 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 / t7;
  }

  real_type
  Baumgarte::lambda__f_D_3_4( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    return 2 * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO / t7;
  }

  real_type
  Baumgarte::lambda__f_D_3_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    return 2 * Omega__XO * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO * Omega__XO / t7;
  }

  real_type
  Baumgarte::lambda__f_D_3_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    return -2 * Omega__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * Omega__XO / t7;
  }

  real_type
  Baumgarte::lambda__f_D_3_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t24  = 1.0 / t7;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 * t24;
  }

  real_type
  Baumgarte::lambda__f_D_3_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[116];
    real_type t10  = cos(phi__f__XO);
    return -(t10 * t9 - t9 + ModelPars[114]) / t7;
  }

  real_type
  Baumgarte::lambda__f_D_3_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    return 2 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 / t7;
  }

  real_type
  Baumgarte::lambda__f_D_3_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    return 2 * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO / t7;
  }

  real_type
  Baumgarte::lambda__f_D_4( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[116];
    real_type t11  = cos(phi__f__XO);
    return -delta__f__XO * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[114] - t9) * omega__f__XO - u__XO) / t7 - delta__f__XO / t6;
  }

  real_type
  Baumgarte::lambda__f_D_4_4( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * delta__f__XO;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 / t7;
  }

  real_type
  Baumgarte::lambda__f_D_4_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * delta__f__XO;
    return 2 * Omega__XO * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 * Omega__XO / t7;
  }

  real_type
  Baumgarte::lambda__f_D_4_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    return -2 * Omega__XO * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * delta__f__XO * Omega__XO / t7;
  }

  real_type
  Baumgarte::lambda__f_D_4_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO;
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t25  = 1.0 / t7;
    return 2 * t21 * delta__f__XO * t19 / t7 / t6 + 2 * delta__f__XO * t21 * t25 - t19 * t25 - 1.0 / t6;
  }

  real_type
  Baumgarte::lambda__f_D_4_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[116];
    real_type t10  = cos(phi__f__XO);
    return -delta__f__XO * (t10 * t9 - t9 + ModelPars[114]) / t7;
  }

  real_type
  Baumgarte::lambda__f_D_4_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    return 2 * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO / t7;
  }

  real_type
  Baumgarte::lambda__f_D_4_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * delta__f__XO;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 / t7;
  }

  real_type
  Baumgarte::lambda__f_D_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[116];
    real_type t11  = cos(phi__f__XO);
    return -Omega__XO * delta__f__XO * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[114] - t9) * omega__f__XO - u__XO) / t7 - delta__f__XO * Omega__XO / t6;
  }

  real_type
  Baumgarte::lambda__f_D_5_5( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = Omega__XO * Omega__XO;
    real_type t22  = delta__f__XO * delta__f__XO;
    return 2 * t22 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t22 * t21 / t7;
  }

  real_type
  Baumgarte::lambda__f_D_5_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = Omega__XO * Omega__XO;
    return -2 * delta__f__XO * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * delta__f__XO * t21 / t7;
  }

  real_type
  Baumgarte::lambda__f_D_5_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO;
    real_type t21  = delta__f__XO * Omega__XO;
    real_type t22  = t2 + v__XO + y__f__dot__XO;
    real_type t26  = 1.0 / t7;
    return 2 * t22 * t21 * t19 / t7 / t6 + t21 * t22 * t26 - Omega__XO * t19 * t26 + t22 * delta__f__XO * Omega__XO * t26 - Omega__XO / t6;
  }

  real_type
  Baumgarte::lambda__f_D_5_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[116];
    real_type t10  = cos(phi__f__XO);
    return -Omega__XO * delta__f__XO * (t10 * t9 - t9 + ModelPars[114]) / t7;
  }

  real_type
  Baumgarte::lambda__f_D_5_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    return 2 * Omega__XO * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO * Omega__XO / t7;
  }

  real_type
  Baumgarte::lambda__f_D_5_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * delta__f__XO;
    return 2 * Omega__XO * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 * Omega__XO / t7;
  }

  real_type
  Baumgarte::lambda__f_D_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[116];
    real_type t11  = cos(phi__f__XO);
    return Omega__XO * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[114] - t9) * omega__f__XO - u__XO) / t7 + Omega__XO / t6;
  }

  real_type
  Baumgarte::lambda__f_D_6_6( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = Omega__XO * Omega__XO;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 / t7;
  }

  real_type
  Baumgarte::lambda__f_D_6_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t25  = 1.0 / t7;
    return -2 * Omega__XO * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * Omega__XO * t21 * t25;
  }

  real_type
  Baumgarte::lambda__f_D_6_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[116];
    real_type t10  = cos(phi__f__XO);
    return Omega__XO * (t10 * t9 - t9 + ModelPars[114]) / t7;
  }

  real_type
  Baumgarte::lambda__f_D_6_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    return -2 * Omega__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * Omega__XO / t7;
  }

  real_type
  Baumgarte::lambda__f_D_6_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    return -2 * Omega__XO * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 - 2 * delta__f__XO * Omega__XO / t7;
  }

  real_type
  Baumgarte::lambda__f_D_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[116];
    real_type t11  = cos(phi__f__XO);
    real_type t20  = t2 + v__XO + y__f__dot__XO;
    return -t20 * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[114] - t9) * omega__f__XO - u__XO) / t7 - t20 / t6;
  }

  real_type
  Baumgarte::lambda__f_D_7_7( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t22  = t21 * t21;
    return 2 * t22 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 * t21 / t7;
  }

  real_type
  Baumgarte::lambda__f_D_7_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t2   = x__f__XO * Omega__XO;
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + (t2 + v__XO) * delta__f__XO - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[116];
    real_type t10  = cos(phi__f__XO);
    return -(t2 + v__XO + y__f__dot__XO) * (t10 * t9 - t9 + ModelPars[114]) / t7;
  }

  real_type
  Baumgarte::lambda__f_D_7_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t24  = 1.0 / t7;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 * t24;
  }

  real_type
  Baumgarte::lambda__f_D_7_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t2   = x__f__XO * Omega__XO;
    real_type t3   = t2 + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t19  = -x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO;
    real_type t21  = t2 + v__XO + y__f__dot__XO;
    real_type t25  = 1.0 / t7;
    return 2 * t21 * delta__f__XO * t19 / t7 / t6 + 2 * delta__f__XO * t21 * t25 - t19 * t25 - 1.0 / t6;
  }

  real_type
  Baumgarte::lambda__f_D_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t8   = ModelPars[116];
    real_type t9   = cos(phi__f__XO);
    return (t9 * t8 - t8 + ModelPars[114]) / (x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO);
  }

  real_type
  Baumgarte::lambda__f_D_8_8( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    return 0;
  }

  real_type
  Baumgarte::lambda__f_D_8_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[116];
    real_type t10  = cos(phi__f__XO);
    return -(t10 * t9 - t9 + ModelPars[114]) / t7;
  }

  real_type
  Baumgarte::lambda__f_D_8_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t7   = pow(x__f__dot__XO + delta__f__XO * y__f__dot__XO + delta__f__XO * (x__f__XO * Omega__XO + v__XO) - y__f__XO * Omega__XO + u__XO, 2);
    real_type t9   = ModelPars[116];
    real_type t10  = cos(phi__f__XO);
    return -delta__f__XO * (t10 * t9 - t9 + ModelPars[114]) / t7;
  }

  real_type
  Baumgarte::lambda__f_D_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[116];
    real_type t11  = cos(phi__f__XO);
    return -(-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[114] - t9) * omega__f__XO - u__XO) / t7 - 1.0 / t6;
  }

  real_type
  Baumgarte::lambda__f_D_9_9( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    return 2 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 / t7;
  }

  real_type
  Baumgarte::lambda__f_D_9_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    return 2 * delta__f__XO * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * delta__f__XO / t7;
  }

  real_type
  Baumgarte::lambda__f_D_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t9   = ModelPars[116];
    real_type t11  = cos(phi__f__XO);
    return -delta__f__XO * (-x__f__dot__XO - t1 + t11 * omega__f__XO * t9 - t3 * delta__f__XO + t5 + (ModelPars[114] - t9) * omega__f__XO - u__XO) / t7 - delta__f__XO / t6;
  }

  real_type
  Baumgarte::lambda__f_D_10_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const {
    real_type t1   = delta__f__XO * y__f__dot__XO;
    real_type t3   = x__f__XO * Omega__XO + v__XO;
    real_type t5   = y__f__XO * Omega__XO;
    real_type t6   = t3 * delta__f__XO + t1 - t5 + u__XO + x__f__dot__XO;
    real_type t7   = t6 * t6;
    real_type t10  = ModelPars[116];
    real_type t12  = cos(phi__f__XO);
    real_type t21  = delta__f__XO * delta__f__XO;
    return 2 * t21 * (-x__f__dot__XO - t1 + t12 * omega__f__XO * t10 - t3 * delta__f__XO + t5 + (ModelPars[114] - t10) * omega__f__XO - u__XO) / t7 / t6 + 2 * t21 / t7;
  }

  real_type
  Baumgarte::Fxf( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t11  = Fzf__XO * ModelPars[244] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t14  = ModelPars[77] * ModelPars[165];
    real_type t24  = exp(t5 * t3 * ModelPars[87]);
    real_type t34  = atan(lambda__f__XO / (t11 * t14 + ModelPars[161]) * ModelPars[167] * t24 * (t5 * t3 * ModelPars[85] + ModelPars[83]) * Fzf__XO);
    real_type t36  = sin(t34 * t14);
    real_type t37  = ModelPars[102];
    real_type t42  = lambda__f__XO * lambda__f__XO;
    real_type t44  = ModelPars[92] * ModelPars[92];
    real_type t47  = sqrt(t44 * t42 + 1);
    real_type t49  = 1.0 / t47 * (phi__f__XO * ModelPars[94] + ModelPars[90]);
    real_type t50  = ModelPars[106];
    real_type t53  = atan((alpha__f__XO + t50) * t49);
    real_type t55  = cos(t53 * t37);
    real_type t58  = atan(t50 * t49);
    real_type t60  = cos(t58 * t37);
    return 1.0 / t60 * t55 * t36 * t11;
  }

  real_type
  Baumgarte::Fxf_D_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[81];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[244];
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
    real_type t40  = ModelPars[102];
    real_type t45  = lambda__f__XO * lambda__f__XO;
    real_type t47  = ModelPars[92] * ModelPars[92];
    real_type t50  = sqrt(t47 * t45 + 1);
    real_type t52  = 1.0 / t50 * (phi__f__XO * ModelPars[94] + ModelPars[90]);
    real_type t53  = ModelPars[106];
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
  Baumgarte::Fxf_D_1_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[81];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t4   = t3 * t1;
    real_type t5   = ModelPars[244];
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
    real_type t39  = ModelPars[102];
    real_type t44  = lambda__f__XO * lambda__f__XO;
    real_type t46  = ModelPars[92] * ModelPars[92];
    real_type t49  = sqrt(t46 * t44 + 1);
    real_type t51  = 1.0 / t49 * (phi__f__XO * ModelPars[94] + ModelPars[90]);
    real_type t52  = ModelPars[106];
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
  Baumgarte::Fxf_D_1_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[81];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[244];
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
    real_type t40  = ModelPars[102];
    real_type t43  = ModelPars[94];
    real_type t44  = lambda__f__XO * lambda__f__XO;
    real_type t46  = ModelPars[92] * ModelPars[92];
    real_type t48  = t46 * t44 + 1;
    real_type t49  = sqrt(t48);
    real_type t50  = 1.0 / t49;
    real_type t51  = t50 * t43;
    real_type t52  = ModelPars[106];
    real_type t53  = alpha__f__XO + t52;
    real_type t57  = t43 * phi__f__XO + ModelPars[90];
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
  Baumgarte::Fxf_D_1_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[81];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[244];
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
    real_type t41  = ModelPars[102];
    real_type t45  = phi__f__XO * ModelPars[94] + ModelPars[90];
    real_type t47  = lambda__f__XO * lambda__f__XO;
    real_type t49  = ModelPars[92] * ModelPars[92];
    real_type t51  = t49 * t47 + 1;
    real_type t52  = sqrt(t51);
    real_type t53  = 1.0 / t52;
    real_type t55  = t45 * t45;
    real_type t58  = ModelPars[106];
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
  Baumgarte::Fxf_D_1_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[81];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[244];
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
    real_type t60  = ModelPars[102];
    real_type t64  = phi__f__XO * ModelPars[94] + ModelPars[90];
    real_type t66  = ModelPars[92] * ModelPars[92];
    real_type t68  = t66 * t46 + 1;
    real_type t69  = sqrt(t68);
    real_type t71  = 1.0 / t69 * t64;
    real_type t72  = ModelPars[106];
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
  Baumgarte::Fxf_D_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t13  = ModelPars[77] * ModelPars[165];
    real_type t23  = exp(t5 * t3 * ModelPars[87]);
    real_type t26  = t10 * Fzf__XO;
    real_type t34  = atan(lambda__f__XO / (t26 * t13 + ModelPars[161]) * ModelPars[167] * t23 * (t5 * t3 * ModelPars[85] + ModelPars[83]) * Fzf__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[102];
    real_type t41  = ModelPars[94];
    real_type t42  = lambda__f__XO * lambda__f__XO;
    real_type t44  = ModelPars[92] * ModelPars[92];
    real_type t46  = t44 * t42 + 1;
    real_type t47  = sqrt(t46);
    real_type t48  = 1.0 / t47;
    real_type t50  = ModelPars[106];
    real_type t51  = alpha__f__XO + t50;
    real_type t55  = t41 * phi__f__XO + ModelPars[90];
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
  Baumgarte::Fxf_D_2_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t11  = t10 * Fzf__XO;
    real_type t14  = ModelPars[77] * ModelPars[165];
    real_type t24  = exp(t5 * t3 * ModelPars[87]);
    real_type t34  = atan(lambda__f__XO / (t11 * t14 + ModelPars[161]) * ModelPars[167] * t24 * (t5 * t3 * ModelPars[85] + ModelPars[83]) * Fzf__XO);
    real_type t36  = sin(t34 * t14);
    real_type t37  = ModelPars[102];
    real_type t39  = ModelPars[94];
    real_type t40  = t39 * t39;
    real_type t43  = lambda__f__XO * lambda__f__XO;
    real_type t45  = ModelPars[92] * ModelPars[92];
    real_type t47  = t45 * t43 + 1;
    real_type t48  = sqrt(t47);
    real_type t50  = 1.0 / t48 / t47;
    real_type t51  = ModelPars[106];
    real_type t52  = alpha__f__XO + t51;
    real_type t53  = t52 * t52;
    real_type t58  = t39 * phi__f__XO + ModelPars[90];
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
  Baumgarte::Fxf_D_2_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t13  = ModelPars[77] * ModelPars[165];
    real_type t23  = exp(t5 * t3 * ModelPars[87]);
    real_type t26  = t10 * Fzf__XO;
    real_type t34  = atan(lambda__f__XO / (t26 * t13 + ModelPars[161]) * ModelPars[167] * t23 * (t5 * t3 * ModelPars[85] + ModelPars[83]) * Fzf__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[102];
    real_type t41  = ModelPars[94];
    real_type t42  = lambda__f__XO * lambda__f__XO;
    real_type t44  = ModelPars[92] * ModelPars[92];
    real_type t46  = t44 * t42 + 1;
    real_type t47  = sqrt(t46);
    real_type t48  = 1.0 / t47;
    real_type t52  = t41 * phi__f__XO + ModelPars[90];
    real_type t53  = t52 * t52;
    real_type t54  = 1.0 / t46;
    real_type t55  = t54 * t53;
    real_type t56  = ModelPars[106];
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
  Baumgarte::Fxf_D_2_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t11  = Fzf__XO * Fzf__XO;
    real_type t12  = ModelPars[77];
    real_type t15  = ModelPars[165];
    real_type t20  = t5 * t3 * ModelPars[85] + ModelPars[83];
    real_type t25  = exp(t5 * t3 * ModelPars[87]);
    real_type t26  = ModelPars[167];
    real_type t28  = t15 * t12;
    real_type t29  = t10 * Fzf__XO;
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
    real_type t59  = ModelPars[102];
    real_type t60  = ModelPars[94];
    real_type t64  = ModelPars[92] * ModelPars[92];
    real_type t66  = t64 * t45 + 1;
    real_type t67  = sqrt(t66);
    real_type t68  = 1.0 / t67;
    real_type t69  = ModelPars[106];
    real_type t70  = alpha__f__XO + t69;
    real_type t74  = t60 * phi__f__XO + ModelPars[90];
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
  Baumgarte::Fxf_D_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t13  = ModelPars[77] * ModelPars[165];
    real_type t23  = exp(t5 * t3 * ModelPars[87]);
    real_type t34  = atan(lambda__f__XO / (Fzf__XO * t10 * t13 + ModelPars[161]) * ModelPars[167] * t23 * (t5 * t3 * ModelPars[85] + ModelPars[83]) * Fzf__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[102];
    real_type t44  = phi__f__XO * ModelPars[94] + ModelPars[90];
    real_type t45  = lambda__f__XO * lambda__f__XO;
    real_type t47  = ModelPars[92] * ModelPars[92];
    real_type t49  = t47 * t45 + 1;
    real_type t50  = sqrt(t49);
    real_type t52  = 1.0 / t50 * t44;
    real_type t53  = t44 * t44;
    real_type t56  = ModelPars[106];
    real_type t57  = alpha__f__XO + t56;
    real_type t58  = t57 * t57;
    real_type t63  = atan(t57 * t52);
    real_type t65  = sin(t63 * t38);
    real_type t68  = atan(t56 * t52);
    real_type t70  = cos(t68 * t38);
    return -1.0 / t70 * t65 / (t58 / t49 * t53 + 1) * t52 * t38 * t36 * Fzf__XO * t10;
  }

  real_type
  Baumgarte::Fxf_D_3_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t13  = ModelPars[77] * ModelPars[165];
    real_type t23  = exp(t5 * t3 * ModelPars[87]);
    real_type t34  = atan(lambda__f__XO / (Fzf__XO * t10 * t13 + ModelPars[161]) * ModelPars[167] * t23 * (t5 * t3 * ModelPars[85] + ModelPars[83]) * Fzf__XO);
    real_type t36  = sin(t34 * t13);
    real_type t37  = t36 * Fzf__XO;
    real_type t38  = ModelPars[102];
    real_type t44  = phi__f__XO * ModelPars[94] + ModelPars[90];
    real_type t45  = t44 * t44;
    real_type t47  = lambda__f__XO * lambda__f__XO;
    real_type t49  = ModelPars[92] * ModelPars[92];
    real_type t51  = t49 * t47 + 1;
    real_type t52  = sqrt(t51);
    real_type t57  = 1.0 / t51 * t45;
    real_type t58  = ModelPars[106];
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
  Baumgarte::Fxf_D_3_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
    real_type t11  = Fzf__XO * Fzf__XO;
    real_type t12  = ModelPars[77];
    real_type t15  = ModelPars[165];
    real_type t20  = t5 * t3 * ModelPars[85] + ModelPars[83];
    real_type t25  = exp(t5 * t3 * ModelPars[87]);
    real_type t26  = ModelPars[167];
    real_type t30  = t15 * t12;
    real_type t31  = t10 * Fzf__XO;
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
    real_type t58  = ModelPars[102];
    real_type t64  = phi__f__XO * ModelPars[94] + ModelPars[90];
    real_type t66  = ModelPars[92] * ModelPars[92];
    real_type t68  = t66 * t44 + 1;
    real_type t69  = sqrt(t68);
    real_type t71  = 1.0 / t69 * t64;
    real_type t72  = t64 * t64;
    real_type t74  = 1.0 / t68 * t72;
    real_type t75  = ModelPars[106];
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
  Baumgarte::Fxf_D_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
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
    real_type t57  = ModelPars[102];
    real_type t61  = phi__f__XO * ModelPars[94] + ModelPars[90];
    real_type t63  = ModelPars[92] * ModelPars[92];
    real_type t65  = t63 * t43 + 1;
    real_type t66  = sqrt(t65);
    real_type t68  = 1.0 / t66 * t61;
    real_type t69  = ModelPars[106];
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
  Baumgarte::Fxf_D_4_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzf__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[244] * (t5 * t3 * ModelPars[81] + ModelPars[79]);
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
    real_type t64  = ModelPars[102];
    real_type t68  = phi__f__XO * ModelPars[94] + ModelPars[90];
    real_type t70  = ModelPars[92] * ModelPars[92];
    real_type t72  = t70 * t48 + 1;
    real_type t73  = sqrt(t72);
    real_type t75  = 1.0 / t73 * t68;
    real_type t76  = ModelPars[106];
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
  Baumgarte::Fxr( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t11  = Fzr__XO * ModelPars[245] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t14  = ModelPars[78] * ModelPars[166];
    real_type t24  = exp(t5 * t3 * ModelPars[88]);
    real_type t34  = atan(lambda__r__XO / (t11 * t14 + ModelPars[162]) * ModelPars[168] * t24 * (t5 * t3 * ModelPars[86] + ModelPars[84]) * Fzr__XO);
    real_type t36  = sin(t34 * t14);
    real_type t37  = ModelPars[103];
    real_type t42  = lambda__r__XO * lambda__r__XO;
    real_type t44  = ModelPars[93] * ModelPars[93];
    real_type t47  = sqrt(t44 * t42 + 1);
    real_type t49  = 1.0 / t47 * (phi__XO * ModelPars[95] + ModelPars[91]);
    real_type t50  = ModelPars[107];
    real_type t53  = atan((alpha__r__XO + t50) * t49);
    real_type t55  = cos(t53 * t37);
    real_type t58  = atan(t50 * t49);
    real_type t60  = cos(t58 * t37);
    return 1.0 / t60 * t55 * t36 * t11;
  }

  real_type
  Baumgarte::Fxr_D_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[82];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[245];
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
    real_type t40  = ModelPars[103];
    real_type t45  = lambda__r__XO * lambda__r__XO;
    real_type t47  = ModelPars[93] * ModelPars[93];
    real_type t50  = sqrt(t47 * t45 + 1);
    real_type t52  = 1.0 / t50 * (phi__XO * ModelPars[95] + ModelPars[91]);
    real_type t53  = ModelPars[107];
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
  Baumgarte::Fxr_D_1_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[82];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t4   = t3 * t1;
    real_type t5   = ModelPars[245];
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
    real_type t39  = ModelPars[103];
    real_type t44  = lambda__r__XO * lambda__r__XO;
    real_type t46  = ModelPars[93] * ModelPars[93];
    real_type t49  = sqrt(t46 * t44 + 1);
    real_type t51  = 1.0 / t49 * (phi__XO * ModelPars[95] + ModelPars[91]);
    real_type t52  = ModelPars[107];
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
  Baumgarte::Fxr_D_1_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[82];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[245];
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
    real_type t40  = ModelPars[103];
    real_type t43  = ModelPars[95];
    real_type t44  = lambda__r__XO * lambda__r__XO;
    real_type t46  = ModelPars[93] * ModelPars[93];
    real_type t48  = t46 * t44 + 1;
    real_type t49  = sqrt(t48);
    real_type t50  = 1.0 / t49;
    real_type t51  = t50 * t43;
    real_type t52  = ModelPars[107];
    real_type t53  = alpha__r__XO + t52;
    real_type t57  = t43 * phi__XO + ModelPars[91];
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
  Baumgarte::Fxr_D_1_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[82];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[245];
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
    real_type t28  = Fzr__XO * t27;
    real_type t31  = t28 * t9 + ModelPars[162];
    real_type t32  = 1.0 / t31;
    real_type t34  = lambda__r__XO * t32 * t22;
    real_type t36  = atan(t34 * t21);
    real_type t37  = t36 * t9;
    real_type t38  = sin(t37);
    real_type t41  = ModelPars[103];
    real_type t45  = phi__XO * ModelPars[95] + ModelPars[91];
    real_type t47  = lambda__r__XO * lambda__r__XO;
    real_type t49  = ModelPars[93] * ModelPars[93];
    real_type t51  = t49 * t47 + 1;
    real_type t52  = sqrt(t51);
    real_type t53  = 1.0 / t52;
    real_type t55  = t45 * t45;
    real_type t58  = ModelPars[107];
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
  Baumgarte::Fxr_D_1_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[82];
    real_type t2   = ModelPars[5];
    real_type t3   = 1.0 / t2;
    real_type t5   = ModelPars[245];
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
    real_type t60  = ModelPars[103];
    real_type t64  = phi__XO * ModelPars[95] + ModelPars[91];
    real_type t66  = ModelPars[93] * ModelPars[93];
    real_type t68  = t66 * t46 + 1;
    real_type t69  = sqrt(t68);
    real_type t71  = 1.0 / t69 * t64;
    real_type t72  = ModelPars[107];
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
  Baumgarte::Fxr_D_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[245] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t13  = ModelPars[78] * ModelPars[166];
    real_type t23  = exp(t5 * t3 * ModelPars[88]);
    real_type t26  = t10 * Fzr__XO;
    real_type t34  = atan(lambda__r__XO / (t26 * t13 + ModelPars[162]) * ModelPars[168] * t23 * (t5 * t3 * ModelPars[86] + ModelPars[84]) * Fzr__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[103];
    real_type t41  = ModelPars[95];
    real_type t42  = lambda__r__XO * lambda__r__XO;
    real_type t44  = ModelPars[93] * ModelPars[93];
    real_type t46  = t44 * t42 + 1;
    real_type t47  = sqrt(t46);
    real_type t48  = 1.0 / t47;
    real_type t50  = ModelPars[107];
    real_type t51  = alpha__r__XO + t50;
    real_type t55  = t41 * phi__XO + ModelPars[91];
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
  Baumgarte::Fxr_D_2_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[245] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t11  = t10 * Fzr__XO;
    real_type t14  = ModelPars[78] * ModelPars[166];
    real_type t24  = exp(t5 * t3 * ModelPars[88]);
    real_type t34  = atan(lambda__r__XO / (t11 * t14 + ModelPars[162]) * ModelPars[168] * t24 * (t5 * t3 * ModelPars[86] + ModelPars[84]) * Fzr__XO);
    real_type t36  = sin(t34 * t14);
    real_type t37  = ModelPars[103];
    real_type t39  = ModelPars[95];
    real_type t40  = t39 * t39;
    real_type t43  = lambda__r__XO * lambda__r__XO;
    real_type t45  = ModelPars[93] * ModelPars[93];
    real_type t47  = t45 * t43 + 1;
    real_type t48  = sqrt(t47);
    real_type t50  = 1.0 / t48 / t47;
    real_type t51  = ModelPars[107];
    real_type t52  = alpha__r__XO + t51;
    real_type t53  = t52 * t52;
    real_type t58  = t39 * phi__XO + ModelPars[91];
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
  Baumgarte::Fxr_D_2_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[245] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t13  = ModelPars[78] * ModelPars[166];
    real_type t23  = exp(t5 * t3 * ModelPars[88]);
    real_type t26  = t10 * Fzr__XO;
    real_type t34  = atan(lambda__r__XO / (t26 * t13 + ModelPars[162]) * ModelPars[168] * t23 * (t5 * t3 * ModelPars[86] + ModelPars[84]) * Fzr__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[103];
    real_type t41  = ModelPars[95];
    real_type t42  = lambda__r__XO * lambda__r__XO;
    real_type t44  = ModelPars[93] * ModelPars[93];
    real_type t46  = t44 * t42 + 1;
    real_type t47  = sqrt(t46);
    real_type t48  = 1.0 / t47;
    real_type t52  = t41 * phi__XO + ModelPars[91];
    real_type t53  = t52 * t52;
    real_type t54  = 1.0 / t46;
    real_type t55  = t54 * t53;
    real_type t56  = ModelPars[107];
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
  Baumgarte::Fxr_D_2_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[245] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t11  = Fzr__XO * Fzr__XO;
    real_type t12  = ModelPars[78];
    real_type t15  = ModelPars[166];
    real_type t20  = t5 * t3 * ModelPars[86] + ModelPars[84];
    real_type t25  = exp(t5 * t3 * ModelPars[88]);
    real_type t26  = ModelPars[168];
    real_type t28  = t15 * t12;
    real_type t29  = t10 * Fzr__XO;
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
    real_type t59  = ModelPars[103];
    real_type t60  = ModelPars[95];
    real_type t64  = ModelPars[93] * ModelPars[93];
    real_type t66  = t64 * t45 + 1;
    real_type t67  = sqrt(t66);
    real_type t68  = 1.0 / t67;
    real_type t69  = ModelPars[107];
    real_type t70  = alpha__r__XO + t69;
    real_type t74  = t60 * phi__XO + ModelPars[91];
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
  Baumgarte::Fxr_D_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[245] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t13  = ModelPars[78] * ModelPars[166];
    real_type t23  = exp(t5 * t3 * ModelPars[88]);
    real_type t34  = atan(lambda__r__XO / (Fzr__XO * t10 * t13 + ModelPars[162]) * ModelPars[168] * t23 * (t5 * t3 * ModelPars[86] + ModelPars[84]) * Fzr__XO);
    real_type t36  = sin(t34 * t13);
    real_type t38  = ModelPars[103];
    real_type t44  = phi__XO * ModelPars[95] + ModelPars[91];
    real_type t45  = lambda__r__XO * lambda__r__XO;
    real_type t47  = ModelPars[93] * ModelPars[93];
    real_type t49  = t47 * t45 + 1;
    real_type t50  = sqrt(t49);
    real_type t52  = 1.0 / t50 * t44;
    real_type t53  = t44 * t44;
    real_type t56  = ModelPars[107];
    real_type t57  = alpha__r__XO + t56;
    real_type t58  = t57 * t57;
    real_type t63  = atan(t57 * t52);
    real_type t65  = sin(t63 * t38);
    real_type t68  = atan(t56 * t52);
    real_type t70  = cos(t68 * t38);
    return -1.0 / t70 * t65 / (t58 / t49 * t53 + 1) * t52 * t38 * t36 * Fzr__XO * t10;
  }

  real_type
  Baumgarte::Fxr_D_3_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[245] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
    real_type t13  = ModelPars[78] * ModelPars[166];
    real_type t23  = exp(t5 * t3 * ModelPars[88]);
    real_type t34  = atan(lambda__r__XO / (Fzr__XO * t10 * t13 + ModelPars[162]) * ModelPars[168] * t23 * (t5 * t3 * ModelPars[86] + ModelPars[84]) * Fzr__XO);
    real_type t36  = sin(t34 * t13);
    real_type t37  = t36 * Fzr__XO;
    real_type t38  = ModelPars[103];
    real_type t44  = phi__XO * ModelPars[95] + ModelPars[91];
    real_type t45  = t44 * t44;
    real_type t47  = lambda__r__XO * lambda__r__XO;
    real_type t49  = ModelPars[93] * ModelPars[93];
    real_type t51  = t49 * t47 + 1;
    real_type t52  = sqrt(t51);
    real_type t57  = 1.0 / t51 * t45;
    real_type t58  = ModelPars[107];
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
  Baumgarte::Fxr_D_3_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[245] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
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
    real_type t58  = ModelPars[103];
    real_type t64  = phi__XO * ModelPars[95] + ModelPars[91];
    real_type t66  = ModelPars[93] * ModelPars[93];
    real_type t68  = t66 * t44 + 1;
    real_type t69  = sqrt(t68);
    real_type t71  = 1.0 / t69 * t64;
    real_type t72  = t64 * t64;
    real_type t74  = 1.0 / t68 * t72;
    real_type t75  = ModelPars[107];
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
  Baumgarte::Fxr_D_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[245] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
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
    real_type t57  = ModelPars[103];
    real_type t61  = phi__XO * ModelPars[95] + ModelPars[91];
    real_type t63  = ModelPars[93] * ModelPars[93];
    real_type t65  = t63 * t43 + 1;
    real_type t66  = sqrt(t65);
    real_type t68  = 1.0 / t66 * t61;
    real_type t69  = ModelPars[107];
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
  Baumgarte::Fxr_D_4_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[5];
    real_type t3   = Fzr__XO - t2;
    real_type t5   = 1.0 / t2;
    real_type t10  = ModelPars[245] * (t5 * t3 * ModelPars[82] + ModelPars[80]);
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
    real_type t64  = ModelPars[103];
    real_type t68  = phi__XO * ModelPars[95] + ModelPars[91];
    real_type t70  = ModelPars[93] * ModelPars[93];
    real_type t72  = t70 * t48 + 1;
    real_type t73  = sqrt(t72);
    real_type t75  = 1.0 / t73 * t68;
    real_type t76  = ModelPars[107];
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
  Baumgarte::Fyf( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[40];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t8   = ModelPars[46] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t12  = sqrt(1.0 / t9 * t3 * t2);
    real_type t13  = ModelPars[48];
    real_type t15  = ModelPars[6];
    real_type t18  = Fzf__XO - t15;
    real_type t20  = t15 * ModelPars[34] + t18 * ModelPars[36];
    real_type t23  = ModelPars[42] * t5 + 1;
    real_type t28  = 1.0 / t1;
    real_type t37  = 1.0 / t20;
    real_type t45  = ModelPars[44] * phi__f__XO;
    real_type t55  = atan(((-t23 * t37 * phi__f__XO * ModelPars[38] * Fzf__XO + alpha__f__XO) / t12 / t8 * Fzf__XO * t1 - t23 * t37 * t8 * t28 * t12 * t45) * t8 / Fzf__XO * t28 / t13 / t23 * t20);
    real_type t57  = sin(t55 * t13);
    real_type t63  = ModelPars[104];
    real_type t66  = ModelPars[98] * ModelPars[98];
    real_type t67  = tan(alpha__f__XO);
    real_type t70  = pow(t67 - ModelPars[100], 2);
    real_type t73  = sqrt(t70 * t66 + 1);
    real_type t75  = 1.0 / t73 * ModelPars[96];
    real_type t76  = ModelPars[108];
    real_type t80  = 1.0 / t15 * t18 * ModelPars[110];
    real_type t83  = atan((lambda__f__XO + t76 + t80) * t75);
    real_type t85  = cos(t83 * t63);
    real_type t89  = atan((t76 + t80) * t75);
    real_type t91  = cos(t89 * t63);
    return 1.0 / t91 * t85 * (t8 * t28 * t12 * t45 + t57 * t12);
  }

  real_type
  Baumgarte::Fyf_D_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[40];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t8   = ModelPars[46] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[48];
    real_type t16  = ModelPars[6];
    real_type t18  = ModelPars[36];
    real_type t19  = Fzf__XO - t16;
    real_type t21  = t16 * ModelPars[34] + t19 * t18;
    real_type t24  = ModelPars[42] * t5 + 1;
    real_type t25  = 1.0 / t24;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t25 * t21;
    real_type t29  = 1.0 / t1;
    real_type t31  = 1.0 / Fzf__XO * t29;
    real_type t32  = Fzf__XO * t1;
    real_type t33  = 1.0 / t8;
    real_type t34  = t13 * t33;
    real_type t35  = ModelPars[38];
    real_type t36  = Fzf__XO * t35;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__f__XO * t36 + alpha__f__XO;
    real_type t45  = ModelPars[44] * phi__f__XO;
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
    real_type t132 = ModelPars[104];
    real_type t133 = ModelPars[96];
    real_type t135 = ModelPars[98] * ModelPars[98];
    real_type t136 = tan(alpha__f__XO);
    real_type t139 = pow(t136 - ModelPars[100], 2);
    real_type t141 = t139 * t135 + 1;
    real_type t142 = sqrt(t141);
    real_type t143 = 1.0 / t142;
    real_type t144 = t143 * t133;
    real_type t145 = ModelPars[108];
    real_type t146 = ModelPars[110];
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
  Baumgarte::Fyf_D_1_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[40];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t8   = ModelPars[46] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t14  = 1.0 / t12 / t11;
    real_type t15  = ModelPars[48];
    real_type t17  = ModelPars[6];
    real_type t19  = ModelPars[36];
    real_type t20  = Fzf__XO - t17;
    real_type t22  = t17 * ModelPars[34] + t20 * t19;
    real_type t25  = ModelPars[42] * t5 + 1;
    real_type t26  = 1.0 / t25;
    real_type t28  = 1.0 / t15;
    real_type t29  = t28 * t26 * t22;
    real_type t30  = 1.0 / t1;
    real_type t32  = 1.0 / Fzf__XO * t30;
    real_type t33  = Fzf__XO * t1;
    real_type t34  = 1.0 / t8;
    real_type t35  = 1.0 / t12;
    real_type t36  = t35 * t34;
    real_type t37  = ModelPars[38];
    real_type t38  = Fzf__XO * t37;
    real_type t39  = 1.0 / t22;
    real_type t43  = -t25 * t39 * phi__f__XO * t38 + alpha__f__XO;
    real_type t47  = ModelPars[44] * phi__f__XO;
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
    real_type t247 = ModelPars[104];
    real_type t248 = ModelPars[96];
    real_type t250 = ModelPars[98] * ModelPars[98];
    real_type t251 = tan(alpha__f__XO);
    real_type t254 = pow(t251 - ModelPars[100], 2);
    real_type t256 = t254 * t250 + 1;
    real_type t257 = sqrt(t256);
    real_type t258 = 1.0 / t257;
    real_type t259 = t258 * t248;
    real_type t260 = ModelPars[108];
    real_type t261 = ModelPars[110];
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
  Baumgarte::Fyf_D_1_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[40];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t4   = t3 * t2;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t6   = ModelPars[46];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t4;
    real_type t12  = sqrt(t11);
    real_type t14  = 1.0 / t12 / t11;
    real_type t15  = ModelPars[48];
    real_type t17  = ModelPars[6];
    real_type t19  = ModelPars[36];
    real_type t20  = Fzf__XO - t17;
    real_type t22  = t17 * ModelPars[34] + t20 * t19;
    real_type t23  = ModelPars[42];
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
    real_type t37  = ModelPars[38];
    real_type t38  = Fzf__XO * t37;
    real_type t39  = 1.0 / t22;
    real_type t43  = -t25 * t39 * phi__f__XO * t38 + alpha__f__XO;
    real_type t46  = ModelPars[44];
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
    real_type t383 = ModelPars[104];
    real_type t384 = ModelPars[96];
    real_type t386 = ModelPars[98] * ModelPars[98];
    real_type t387 = tan(alpha__f__XO);
    real_type t390 = pow(t387 - ModelPars[100], 2);
    real_type t392 = t390 * t386 + 1;
    real_type t393 = sqrt(t392);
    real_type t394 = 1.0 / t393;
    real_type t395 = t394 * t384;
    real_type t396 = ModelPars[108];
    real_type t397 = ModelPars[110];
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
  Baumgarte::Fyf_D_1_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = 1.0 / Fzf__XO;
    real_type t3   = ModelPars[6];
    real_type t5   = ModelPars[36];
    real_type t6   = Fzf__XO - t3;
    real_type t8   = t3 * ModelPars[34] + t6 * t5;
    real_type t10  = phi__f__XO * phi__f__XO;
    real_type t13  = ModelPars[42] * t10 + 1;
    real_type t14  = 1.0 / t13;
    real_type t15  = t8 * t8;
    real_type t16  = t13 * t13;
    real_type t17  = 1.0 / t16;
    real_type t19  = ModelPars[48];
    real_type t20  = t19 * t19;
    real_type t23  = ModelPars[40];
    real_type t24  = t23 * t23;
    real_type t26  = Fzf__XO * Fzf__XO;
    real_type t27  = 1.0 / t26;
    real_type t31  = ModelPars[46] * t10 + 1;
    real_type t32  = t31 * t31;
    real_type t33  = Fzf__XO * t23;
    real_type t34  = 1.0 / t31;
    real_type t36  = 1.0 / t32;
    real_type t37  = t36 * t26 * t24;
    real_type t38  = sqrt(t37);
    real_type t39  = 1.0 / t38;
    real_type t40  = t39 * t34;
    real_type t41  = ModelPars[38];
    real_type t42  = Fzf__XO * t41;
    real_type t43  = 1.0 / t8;
    real_type t47  = -t13 * t43 * phi__f__XO * t42 + alpha__f__XO;
    real_type t51  = ModelPars[44] * phi__f__XO;
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
    real_type t157 = ModelPars[104];
    real_type t158 = ModelPars[96];
    real_type t160 = ModelPars[98] * ModelPars[98];
    real_type t161 = tan(alpha__f__XO);
    real_type t163 = t161 - ModelPars[100];
    real_type t164 = t163 * t163;
    real_type t166 = t164 * t160 + 1;
    real_type t167 = sqrt(t166);
    real_type t168 = 1.0 / t167;
    real_type t169 = t168 * t158;
    real_type t170 = ModelPars[108];
    real_type t171 = ModelPars[110];
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
  Baumgarte::Fyf_D_1_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[40];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t8   = ModelPars[46] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[48];
    real_type t16  = ModelPars[6];
    real_type t18  = ModelPars[36];
    real_type t19  = Fzf__XO - t16;
    real_type t21  = t16 * ModelPars[34] + t19 * t18;
    real_type t24  = ModelPars[42] * t5 + 1;
    real_type t25  = 1.0 / t24;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t25 * t21;
    real_type t29  = 1.0 / t1;
    real_type t31  = 1.0 / Fzf__XO * t29;
    real_type t32  = Fzf__XO * t1;
    real_type t33  = 1.0 / t8;
    real_type t34  = t13 * t33;
    real_type t35  = ModelPars[38];
    real_type t36  = Fzf__XO * t35;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__f__XO * t36 + alpha__f__XO;
    real_type t45  = ModelPars[44] * phi__f__XO;
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
    real_type t132 = ModelPars[104];
    real_type t134 = ModelPars[96];
    real_type t137 = ModelPars[98] * ModelPars[98];
    real_type t138 = tan(alpha__f__XO);
    real_type t141 = pow(t138 - ModelPars[100], 2);
    real_type t143 = t141 * t137 + 1;
    real_type t144 = sqrt(t143);
    real_type t145 = 1.0 / t144;
    real_type t146 = t134 * t134;
    real_type t148 = 1.0 / t143 * t146;
    real_type t149 = ModelPars[108];
    real_type t150 = ModelPars[110];
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
  Baumgarte::Fyf_D_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[40];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t6   = ModelPars[46];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[48];
    real_type t16  = ModelPars[6];
    real_type t19  = Fzf__XO - t16;
    real_type t21  = t16 * ModelPars[34] + t19 * ModelPars[36];
    real_type t22  = ModelPars[42];
    real_type t24  = t22 * t5 + 1;
    real_type t26  = 1.0 / t24 * t21;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t26;
    real_type t29  = 1.0 / t1;
    real_type t30  = 1.0 / Fzf__XO;
    real_type t31  = t30 * t29;
    real_type t32  = Fzf__XO * t1;
    real_type t34  = t13 / t8;
    real_type t36  = ModelPars[38] * Fzf__XO;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__f__XO * t36 + alpha__f__XO;
    real_type t44  = ModelPars[44];
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
    real_type t168 = ModelPars[104];
    real_type t171 = ModelPars[98] * ModelPars[98];
    real_type t172 = tan(alpha__f__XO);
    real_type t175 = pow(t172 - ModelPars[100], 2);
    real_type t178 = sqrt(t175 * t171 + 1);
    real_type t180 = 1.0 / t178 * ModelPars[96];
    real_type t181 = ModelPars[108];
    real_type t185 = 1.0 / t16 * t19 * ModelPars[110];
    real_type t188 = atan((lambda__f__XO + t181 + t185) * t180);
    real_type t190 = cos(t188 * t168);
    real_type t194 = atan((t181 + t185) * t180);
    real_type t196 = cos(t194 * t168);
    return 1.0 / t196 * t190 * (-2 * t63 / t9 / t8 * t3 * t2 * t57 * t13 + t153 / (t146 * t9 / t3 / t2 / t140 * t69 * t138 + 1) * (-2 * phi__f__XO * t22 * t51 * t8 * t30 * t71 * t69 * t21 + 2 * t51 * phi__f__XO * t6 * t30 * t71 * t26 + (-2 * t63 * t41 * t13 * t10 * t32 + 2 * t63 * t41 / t12 / t11 / t93 * t3 * Fzf__XO * t2 * t1 + (-2 * t22 * t37 * t5 * t36 - t48 * t36) * t34 * t32 - t24 * t37 * t8 * t29 * t110 + 2 * t6 * t3 * t24 * t37 * t10 * t1 * t13 * t115 - 2 * t48 * t6 * t29 * t124 - 2 * t22 * t37 * t47 * t124) * t8 * t31 * t28) * t14 * t12 + t47 * t110 - 2 * t6 * t3 * t10 * t1 * t13 * t115 + 2 * t6 * t29 * t12 * t115);
  }

  real_type
  Baumgarte::Fyf_D_2_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[40];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t4   = t3 * t2;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t6   = ModelPars[46];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t4;
    real_type t12  = sqrt(t11);
    real_type t14  = 1.0 / t12 / t11;
    real_type t15  = ModelPars[48];
    real_type t17  = ModelPars[6];
    real_type t20  = Fzf__XO - t17;
    real_type t22  = t17 * ModelPars[34] + t20 * ModelPars[36];
    real_type t23  = ModelPars[42];
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
    real_type t37  = ModelPars[38];
    real_type t38  = Fzf__XO * t37;
    real_type t39  = 1.0 / t22;
    real_type t43  = -t25 * t39 * phi__f__XO * t38 + alpha__f__XO;
    real_type t46  = ModelPars[44];
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
    real_type t353 = -4 * t69 / t64 / t9 * t63 * t61 * t59 * t14 - 4 * phi__f__XO * t6 * t161 * t4 * t157 * t155 * t141 * t15 * t35 + 12 * t69 * t100 * t3 * t168 - 2 * t6 * t161 * t3 * t168 + t157 * t155 * (8 * t179 * t182 * t22 * t5 * t53 * t77 * t79 - 4 * t137 * t23 * t78 * t79 * phi__f__XO - 8 * t23 * t5 * t53 * t78 * t86 + 4 * t137 * t85 * t86 * phi__f__XO + t29 * t293 * t32 * t8 + 2 * t29 * t32 * t53 * t6 - 2 * t78 * t79 * t80) * t177 - (-4 * t142 * t150 * t179 * t23 * t306 * t308 * phi__f__XO + 4 * t148 * t150 * t314 * t6 * t8 * phi__f__XO + 2 * t137 * t308 * t314 * t53) * t157 * t303 * t141 * t177 - t59 * t303 * t329 * t144 * t12 - 6 * t93 * t3 * t10 * t1 * t254 + 6 * phi__f__XO * t129 * t114 - 4 * t68 * t63 * t214 * t97 * t14 * t269 + 4 * t68 * t3 * t161 * t1 * t35 * t269;
    real_type t354 = ModelPars[104];
    real_type t357 = ModelPars[98] * ModelPars[98];
    real_type t358 = tan(alpha__f__XO);
    real_type t361 = pow(t358 - ModelPars[100], 2);
    real_type t364 = sqrt(t357 * t361 + 1);
    real_type t366 = 1.0 / t364 * ModelPars[96];
    real_type t367 = ModelPars[108];
    real_type t371 = 1.0 / t17 * t20 * ModelPars[110];
    real_type t374 = atan((lambda__f__XO + t367 + t371) * t366);
    real_type t376 = cos(t374 * t354);
    real_type t380 = atan((t367 + t371) * t366);
    real_type t382 = cos(t380 * t354);
    return 1.0 / t382 * t376 * t353;
  }

  real_type
  Baumgarte::Fyf_D_2_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = phi__f__XO * phi__f__XO;
    real_type t2   = ModelPars[46];
    real_type t4   = t2 * t1 + 1;
    real_type t5   = 1.0 / t4;
    real_type t7   = ModelPars[6];
    real_type t10  = Fzf__XO - t7;
    real_type t12  = t10 * ModelPars[36] + t7 * ModelPars[34];
    real_type t14  = ModelPars[42];
    real_type t16  = t14 * t1 + 1;
    real_type t17  = 1.0 / t16;
    real_type t19  = t12 * t12;
    real_type t20  = t16 * t16;
    real_type t21  = 1.0 / t20;
    real_type t23  = ModelPars[48];
    real_type t24  = t23 * t23;
    real_type t27  = ModelPars[40];
    real_type t28  = t27 * t27;
    real_type t30  = Fzf__XO * Fzf__XO;
    real_type t33  = t4 * t4;
    real_type t34  = Fzf__XO * t27;
    real_type t36  = 1.0 / t33;
    real_type t37  = t36 * t30 * t28;
    real_type t38  = sqrt(t37);
    real_type t39  = 1.0 / t38;
    real_type t40  = t39 * t5;
    real_type t42  = ModelPars[38] * Fzf__XO;
    real_type t43  = 1.0 / t12;
    real_type t47  = -t16 * t43 * phi__f__XO * t42 + alpha__f__XO;
    real_type t50  = ModelPars[44];
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
    real_type t190 = ModelPars[104];
    real_type t191 = ModelPars[96];
    real_type t193 = ModelPars[98] * ModelPars[98];
    real_type t194 = tan(alpha__f__XO);
    real_type t196 = t194 - ModelPars[100];
    real_type t197 = t196 * t196;
    real_type t199 = t197 * t193 + 1;
    real_type t200 = sqrt(t199);
    real_type t202 = 1.0 / t200 * t191;
    real_type t203 = ModelPars[108];
    real_type t207 = 1.0 / t7 * t10 * ModelPars[110];
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
  Baumgarte::Fyf_D_2_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[40];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t6   = ModelPars[46];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[48];
    real_type t16  = ModelPars[6];
    real_type t19  = Fzf__XO - t16;
    real_type t21  = t16 * ModelPars[34] + t19 * ModelPars[36];
    real_type t22  = ModelPars[42];
    real_type t24  = t22 * t5 + 1;
    real_type t26  = 1.0 / t24 * t21;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t26;
    real_type t29  = 1.0 / t1;
    real_type t30  = 1.0 / Fzf__XO;
    real_type t31  = t30 * t29;
    real_type t32  = Fzf__XO * t1;
    real_type t34  = t13 / t8;
    real_type t36  = ModelPars[38] * Fzf__XO;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__f__XO * t36 + alpha__f__XO;
    real_type t44  = ModelPars[44];
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
    real_type t168 = ModelPars[104];
    real_type t170 = ModelPars[96];
    real_type t173 = ModelPars[98] * ModelPars[98];
    real_type t174 = tan(alpha__f__XO);
    real_type t177 = pow(t174 - ModelPars[100], 2);
    real_type t179 = t177 * t173 + 1;
    real_type t180 = sqrt(t179);
    real_type t181 = 1.0 / t180;
    real_type t182 = t170 * t170;
    real_type t185 = ModelPars[108];
    real_type t189 = 1.0 / t16 * t19 * ModelPars[110];
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
  Baumgarte::Fyf_D_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t5   = Fzf__XO - t2;
    real_type t7   = t2 * ModelPars[34] + t5 * ModelPars[36];
    real_type t8   = phi__f__XO * phi__f__XO;
    real_type t11  = ModelPars[42] * t8 + 1;
    real_type t13  = 1.0 / t11 * t7;
    real_type t14  = t7 * t7;
    real_type t15  = t11 * t11;
    real_type t18  = ModelPars[48];
    real_type t19  = t18 * t18;
    real_type t22  = ModelPars[40];
    real_type t23  = t22 * t22;
    real_type t25  = Fzf__XO * Fzf__XO;
    real_type t30  = ModelPars[46] * t8 + 1;
    real_type t31  = t30 * t30;
    real_type t37  = sqrt(1.0 / t31 * t25 * t23);
    real_type t42  = 1.0 / t7;
    real_type t50  = ModelPars[44] * phi__f__XO;
    real_type t52  = 1.0 / t22;
    real_type t57  = (-t11 * t42 * phi__f__XO * ModelPars[38] * Fzf__XO + alpha__f__XO) / t37 / t30 * Fzf__XO * t22 - t11 * t42 * t30 * t52 * t37 * t50;
    real_type t58  = t57 * t57;
    real_type t72  = atan(t57 * t30 / Fzf__XO * t52 / t18 * t13);
    real_type t73  = t72 * t18;
    real_type t74  = cos(t73);
    real_type t75  = ModelPars[104];
    real_type t76  = ModelPars[96];
    real_type t78  = ModelPars[98] * ModelPars[98];
    real_type t79  = tan(alpha__f__XO);
    real_type t81  = t79 - ModelPars[100];
    real_type t82  = t81 * t81;
    real_type t84  = t82 * t78 + 1;
    real_type t85  = sqrt(t84);
    real_type t87  = 1.0 / t85 * t76;
    real_type t88  = ModelPars[108];
    real_type t92  = 1.0 / t2 * t5 * ModelPars[110];
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
  Baumgarte::Fyf_D_3_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t5   = Fzf__XO - t2;
    real_type t7   = t2 * ModelPars[34] + t5 * ModelPars[36];
    real_type t8   = t7 * t7;
    real_type t10  = phi__f__XO * phi__f__XO;
    real_type t13  = ModelPars[42] * t10 + 1;
    real_type t14  = t13 * t13;
    real_type t19  = 1.0 / t14 * t8;
    real_type t20  = ModelPars[48];
    real_type t21  = t20 * t20;
    real_type t22  = 1.0 / t21;
    real_type t24  = ModelPars[40];
    real_type t25  = t24 * t24;
    real_type t27  = Fzf__XO * Fzf__XO;
    real_type t32  = ModelPars[46] * t10 + 1;
    real_type t33  = t32 * t32;
    real_type t39  = sqrt(1.0 / t33 * t27 * t25);
    real_type t40  = 1.0 / t39;
    real_type t44  = 1.0 / t7;
    real_type t52  = ModelPars[44] * phi__f__XO;
    real_type t54  = 1.0 / t24;
    real_type t59  = (-t13 * t44 * phi__f__XO * ModelPars[38] * Fzf__XO + alpha__f__XO) * t40 / t32 * Fzf__XO * t24 - t13 * t44 * t32 * t54 * t39 * t52;
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
    real_type t80  = ModelPars[104];
    real_type t81  = ModelPars[96];
    real_type t83  = ModelPars[98] * ModelPars[98];
    real_type t84  = tan(alpha__f__XO);
    real_type t86  = t84 - ModelPars[100];
    real_type t87  = t86 * t86;
    real_type t89  = t87 * t83 + 1;
    real_type t90  = sqrt(t89);
    real_type t92  = 1.0 / t90 * t81;
    real_type t93  = ModelPars[108];
    real_type t97  = 1.0 / t2 * t5 * ModelPars[110];
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
  Baumgarte::Fyf_D_3_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t2   = ModelPars[6];
    real_type t5   = Fzf__XO - t2;
    real_type t7   = t2 * ModelPars[34] + t5 * ModelPars[36];
    real_type t8   = phi__f__XO * phi__f__XO;
    real_type t11  = ModelPars[42] * t8 + 1;
    real_type t13  = 1.0 / t11 * t7;
    real_type t14  = t7 * t7;
    real_type t15  = t11 * t11;
    real_type t18  = ModelPars[48];
    real_type t19  = t18 * t18;
    real_type t22  = ModelPars[40];
    real_type t23  = t22 * t22;
    real_type t25  = Fzf__XO * Fzf__XO;
    real_type t30  = ModelPars[46] * t8 + 1;
    real_type t31  = t30 * t30;
    real_type t37  = sqrt(1.0 / t31 * t25 * t23);
    real_type t42  = 1.0 / t7;
    real_type t50  = ModelPars[44] * phi__f__XO;
    real_type t52  = 1.0 / t22;
    real_type t57  = (-t11 * t42 * phi__f__XO * ModelPars[38] * Fzf__XO + alpha__f__XO) / t37 / t30 * Fzf__XO * t22 - t11 * t42 * t30 * t52 * t37 * t50;
    real_type t58  = t57 * t57;
    real_type t71  = atan(t57 * t30 / Fzf__XO * t52 / t18 * t13);
    real_type t72  = t71 * t18;
    real_type t73  = cos(t72);
    real_type t75  = ModelPars[104];
    real_type t78  = ModelPars[96];
    real_type t80  = ModelPars[98] * ModelPars[98];
    real_type t81  = tan(alpha__f__XO);
    real_type t83  = t81 - ModelPars[100];
    real_type t84  = t83 * t83;
    real_type t86  = t84 * t80 + 1;
    real_type t87  = sqrt(t86);
    real_type t89  = 1.0 / t87 * t78;
    real_type t90  = t78 * t78;
    real_type t92  = 1.0 / t86 * t90;
    real_type t93  = ModelPars[108];
    real_type t97  = 1.0 / t2 * t5 * ModelPars[110];
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
  Baumgarte::Fyf_D_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[40];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t8   = ModelPars[46] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t12  = sqrt(1.0 / t9 * t3 * t2);
    real_type t13  = ModelPars[48];
    real_type t15  = ModelPars[6];
    real_type t18  = Fzf__XO - t15;
    real_type t20  = t15 * ModelPars[34] + t18 * ModelPars[36];
    real_type t23  = ModelPars[42] * t5 + 1;
    real_type t28  = 1.0 / t1;
    real_type t37  = 1.0 / t20;
    real_type t45  = ModelPars[44] * phi__f__XO;
    real_type t55  = atan(((-t23 * t37 * phi__f__XO * ModelPars[38] * Fzf__XO + alpha__f__XO) / t12 / t8 * Fzf__XO * t1 - t23 * t37 * t8 * t28 * t12 * t45) * t8 / Fzf__XO * t28 / t13 / t23 * t20);
    real_type t57  = sin(t55 * t13);
    real_type t63  = ModelPars[104];
    real_type t65  = ModelPars[96];
    real_type t68  = ModelPars[98] * ModelPars[98];
    real_type t69  = tan(alpha__f__XO);
    real_type t72  = pow(t69 - ModelPars[100], 2);
    real_type t74  = t72 * t68 + 1;
    real_type t75  = sqrt(t74);
    real_type t76  = 1.0 / t75;
    real_type t77  = t65 * t65;
    real_type t80  = ModelPars[108];
    real_type t84  = 1.0 / t15 * t18 * ModelPars[110];
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
  Baumgarte::Fyf_D_4_4( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const {
    real_type t1   = ModelPars[40];
    real_type t2   = t1 * t1;
    real_type t3   = Fzf__XO * Fzf__XO;
    real_type t5   = phi__f__XO * phi__f__XO;
    real_type t8   = ModelPars[46] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t12  = sqrt(1.0 / t9 * t3 * t2);
    real_type t13  = ModelPars[48];
    real_type t15  = ModelPars[6];
    real_type t18  = Fzf__XO - t15;
    real_type t20  = t15 * ModelPars[34] + t18 * ModelPars[36];
    real_type t23  = ModelPars[42] * t5 + 1;
    real_type t28  = 1.0 / t1;
    real_type t37  = 1.0 / t20;
    real_type t45  = ModelPars[44] * phi__f__XO;
    real_type t55  = atan(((-t23 * t37 * phi__f__XO * ModelPars[38] * Fzf__XO + alpha__f__XO) / t12 / t8 * Fzf__XO * t1 - t23 * t37 * t8 * t28 * t12 * t45) * t8 / Fzf__XO * t28 / t13 / t23 * t20);
    real_type t57  = sin(t55 * t13);
    real_type t62  = t8 * t28 * t12 * t45 + t57 * t12;
    real_type t63  = ModelPars[104];
    real_type t65  = ModelPars[96];
    real_type t66  = t65 * t65;
    real_type t69  = ModelPars[98] * ModelPars[98];
    real_type t70  = tan(alpha__f__XO);
    real_type t73  = pow(t70 - ModelPars[100], 2);
    real_type t75  = t73 * t69 + 1;
    real_type t76  = sqrt(t75);
    real_type t81  = 1.0 / t75;
    real_type t83  = ModelPars[108];
    real_type t87  = 1.0 / t15 * t18 * ModelPars[110];
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
  Baumgarte::Fyr( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[41];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t8   = ModelPars[47] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t12  = sqrt(1.0 / t9 * t3 * t2);
    real_type t13  = ModelPars[48];
    real_type t15  = ModelPars[7];
    real_type t18  = Fzr__XO - t15;
    real_type t20  = t15 * ModelPars[35] + t18 * ModelPars[37];
    real_type t23  = ModelPars[43] * t5 + 1;
    real_type t28  = 1.0 / t1;
    real_type t37  = 1.0 / t20;
    real_type t45  = ModelPars[45] * phi__XO;
    real_type t55  = atan(((-t23 * t37 * phi__XO * ModelPars[39] * Fzr__XO + alpha__r__XO) / t12 / t8 * Fzr__XO * t1 - t23 * t37 * t8 * t28 * t12 * t45) * t8 / Fzr__XO * t28 / t13 / t23 * t20);
    real_type t57  = sin(t55 * t13);
    real_type t63  = ModelPars[105];
    real_type t66  = ModelPars[99] * ModelPars[99];
    real_type t67  = tan(alpha__r__XO);
    real_type t70  = pow(t67 - ModelPars[101], 2);
    real_type t73  = sqrt(t70 * t66 + 1);
    real_type t75  = 1.0 / t73 * ModelPars[97];
    real_type t76  = ModelPars[109];
    real_type t80  = 1.0 / t15 * t18 * ModelPars[111];
    real_type t83  = atan((lambda__r__XO + t76 + t80) * t75);
    real_type t85  = cos(t83 * t63);
    real_type t89  = atan((t76 + t80) * t75);
    real_type t91  = cos(t89 * t63);
    return 1.0 / t91 * t85 * (t8 * t28 * t12 * t45 + t57 * t12);
  }

  real_type
  Baumgarte::Fyr_D_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[41];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t8   = ModelPars[47] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[48];
    real_type t16  = ModelPars[7];
    real_type t18  = ModelPars[37];
    real_type t19  = Fzr__XO - t16;
    real_type t21  = t16 * ModelPars[35] + t19 * t18;
    real_type t24  = ModelPars[43] * t5 + 1;
    real_type t25  = 1.0 / t24;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t25 * t21;
    real_type t29  = 1.0 / t1;
    real_type t31  = 1.0 / Fzr__XO * t29;
    real_type t32  = Fzr__XO * t1;
    real_type t33  = 1.0 / t8;
    real_type t34  = t13 * t33;
    real_type t35  = ModelPars[39];
    real_type t36  = Fzr__XO * t35;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__XO * t36 + alpha__r__XO;
    real_type t45  = ModelPars[45] * phi__XO;
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
    real_type t132 = ModelPars[105];
    real_type t133 = ModelPars[97];
    real_type t135 = ModelPars[99] * ModelPars[99];
    real_type t136 = tan(alpha__r__XO);
    real_type t139 = pow(t136 - ModelPars[101], 2);
    real_type t141 = t139 * t135 + 1;
    real_type t142 = sqrt(t141);
    real_type t143 = 1.0 / t142;
    real_type t144 = t143 * t133;
    real_type t145 = ModelPars[109];
    real_type t146 = ModelPars[111];
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
  Baumgarte::Fyr_D_1_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[41];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t8   = ModelPars[47] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t14  = 1.0 / t12 / t11;
    real_type t15  = ModelPars[48];
    real_type t17  = ModelPars[7];
    real_type t19  = ModelPars[37];
    real_type t20  = Fzr__XO - t17;
    real_type t22  = t17 * ModelPars[35] + t20 * t19;
    real_type t25  = ModelPars[43] * t5 + 1;
    real_type t26  = 1.0 / t25;
    real_type t28  = 1.0 / t15;
    real_type t29  = t28 * t26 * t22;
    real_type t30  = 1.0 / t1;
    real_type t32  = 1.0 / Fzr__XO * t30;
    real_type t33  = Fzr__XO * t1;
    real_type t34  = 1.0 / t8;
    real_type t35  = 1.0 / t12;
    real_type t36  = t35 * t34;
    real_type t37  = ModelPars[39];
    real_type t38  = Fzr__XO * t37;
    real_type t39  = 1.0 / t22;
    real_type t43  = -t25 * t39 * phi__XO * t38 + alpha__r__XO;
    real_type t47  = ModelPars[45] * phi__XO;
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
    real_type t247 = ModelPars[105];
    real_type t248 = ModelPars[97];
    real_type t250 = ModelPars[99] * ModelPars[99];
    real_type t251 = tan(alpha__r__XO);
    real_type t254 = pow(t251 - ModelPars[101], 2);
    real_type t256 = t254 * t250 + 1;
    real_type t257 = sqrt(t256);
    real_type t258 = 1.0 / t257;
    real_type t259 = t258 * t248;
    real_type t260 = ModelPars[109];
    real_type t261 = ModelPars[111];
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
  Baumgarte::Fyr_D_1_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[41];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t4   = t3 * t2;
    real_type t5   = phi__XO * phi__XO;
    real_type t6   = ModelPars[47];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t4;
    real_type t12  = sqrt(t11);
    real_type t14  = 1.0 / t12 / t11;
    real_type t15  = ModelPars[48];
    real_type t17  = ModelPars[7];
    real_type t19  = ModelPars[37];
    real_type t20  = Fzr__XO - t17;
    real_type t22  = t17 * ModelPars[35] + t20 * t19;
    real_type t23  = ModelPars[43];
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
    real_type t37  = ModelPars[39];
    real_type t38  = Fzr__XO * t37;
    real_type t39  = 1.0 / t22;
    real_type t43  = -t25 * t39 * phi__XO * t38 + alpha__r__XO;
    real_type t46  = ModelPars[45];
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
    real_type t383 = ModelPars[105];
    real_type t384 = ModelPars[97];
    real_type t386 = ModelPars[99] * ModelPars[99];
    real_type t387 = tan(alpha__r__XO);
    real_type t390 = pow(t387 - ModelPars[101], 2);
    real_type t392 = t390 * t386 + 1;
    real_type t393 = sqrt(t392);
    real_type t394 = 1.0 / t393;
    real_type t395 = t394 * t384;
    real_type t396 = ModelPars[109];
    real_type t397 = ModelPars[111];
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
  Baumgarte::Fyr_D_1_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = 1.0 / Fzr__XO;
    real_type t3   = ModelPars[7];
    real_type t5   = ModelPars[37];
    real_type t6   = Fzr__XO - t3;
    real_type t8   = t3 * ModelPars[35] + t6 * t5;
    real_type t10  = phi__XO * phi__XO;
    real_type t13  = ModelPars[43] * t10 + 1;
    real_type t14  = 1.0 / t13;
    real_type t15  = t8 * t8;
    real_type t16  = t13 * t13;
    real_type t17  = 1.0 / t16;
    real_type t19  = ModelPars[48];
    real_type t20  = t19 * t19;
    real_type t23  = ModelPars[41];
    real_type t24  = t23 * t23;
    real_type t26  = Fzr__XO * Fzr__XO;
    real_type t27  = 1.0 / t26;
    real_type t31  = ModelPars[47] * t10 + 1;
    real_type t32  = t31 * t31;
    real_type t33  = Fzr__XO * t23;
    real_type t34  = 1.0 / t31;
    real_type t36  = 1.0 / t32;
    real_type t37  = t36 * t26 * t24;
    real_type t38  = sqrt(t37);
    real_type t39  = 1.0 / t38;
    real_type t40  = t39 * t34;
    real_type t41  = ModelPars[39];
    real_type t42  = Fzr__XO * t41;
    real_type t43  = 1.0 / t8;
    real_type t47  = -t13 * t43 * phi__XO * t42 + alpha__r__XO;
    real_type t51  = ModelPars[45] * phi__XO;
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
    real_type t138 = t71 * t67 * t79 - t70 * t27 * t53 * t68 + (t47 * t39 * t85 - t47 * t93 * t88 + (phi__XO * t115 * t13 * t42 * t5 - phi__XO * t41 * t55) * t40 * t33 - Fzr__XO * t13 * t43 * t34 * t23 * t39 * t51 + t5 * t13 * t115 * t31 * t128 * t51) * t31 * t69 * t68;
    real_type t140 = t63 * t63;
    real_type t141 = 1.0 / t140;
    real_type t153 = sin(t74);
    real_type t157 = ModelPars[105];
    real_type t158 = ModelPars[97];
    real_type t160 = ModelPars[99] * ModelPars[99];
    real_type t161 = tan(alpha__r__XO);
    real_type t163 = t161 - ModelPars[101];
    real_type t164 = t163 * t163;
    real_type t166 = t160 * t164 + 1;
    real_type t167 = sqrt(t166);
    real_type t168 = 1.0 / t167;
    real_type t169 = t168 * t158;
    real_type t170 = ModelPars[109];
    real_type t171 = ModelPars[111];
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
    real_type t198 = Fzr__XO * t153 * t24 * t36 * t39 + Fzr__XO * t39 * t51 * t85 + t138 * t64 * t75 * t78;
    real_type t201 = 1.0 / t167 / t166;
    real_type t205 = t163 * t160;
    real_type t206 = t161 * t161;
    real_type t207 = 1 + t206;
    real_type t209 = t158 * t158;
    real_type t211 = 1.0 / t166 * t209;
    real_type t212 = t175 * t175;
    real_type t214 = t211 * t212 + 1;
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
    real_type t232 = t211 * t230 + 1;
    real_type t233 = 1.0 / t232;
    real_type t235 = sin(t184);
    real_type t239 = t64 * t66;
    real_type t250 = t128 * t31 * t51 + t153 * t38;
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
    return t186 * t179 * (t75 * t64 * t14 * t8 * t1 + t75 * t64 * (t39 * t67 * t79 - t39 * t1 * t67 * t66 + (t39 * t85 - t88 * t93) * t31 * t69 * t68) * t78 - 2 * t58 * t31 * t1 * t53 * t17 * t15 * t75 * t141 * t138 * t67 - t153 * t66 * t141 * t138 * t19) + t219 * t175 * t201 * t158 * t157 * t198 - t235 * t233 * t207 * t163 * t228 * t226 * t223 * t179 * t198 - t218 * t173 * t171 * t168 * t158 * t157 * t75 * t239 + t219 * t173 * t171 * t201 * t158 * t251 - 2 * t270 * t160 * t212 * t186 * t216 * t266 * t173 * t262 * t257 * t251 + t186 * t179 * t270 * t160 * t175 * t266 * t173 * t279 * t277 + t293 * t270 * t228 * t291 * t215 * t173 * t279 * t277 + t302 * t168 * t225 * t223 * t179 * t75 * t239 + t302 * t291 * t215 * t207 * t205 * t175 * t278 * t277 - 2 * t324 * t322 * t321 * t279 * t316 / t222 / t185 * t312 - t270 * t160 * t235 * t233 * t301 * t226 * t328 + 2 * t270 * t160 * t230 * t235 * t321 * t262 * t257 * t157 * t328 - t324 * t320 * t301 * t278 * t316 * t186 * t312;
  }

  real_type
  Baumgarte::Fyr_D_1_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[41];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t8   = ModelPars[47] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[48];
    real_type t16  = ModelPars[7];
    real_type t18  = ModelPars[37];
    real_type t19  = Fzr__XO - t16;
    real_type t21  = t16 * ModelPars[35] + t19 * t18;
    real_type t24  = ModelPars[43] * t5 + 1;
    real_type t25  = 1.0 / t24;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t25 * t21;
    real_type t29  = 1.0 / t1;
    real_type t31  = 1.0 / Fzr__XO * t29;
    real_type t32  = Fzr__XO * t1;
    real_type t33  = 1.0 / t8;
    real_type t34  = t13 * t33;
    real_type t35  = ModelPars[39];
    real_type t36  = Fzr__XO * t35;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__XO * t36 + alpha__r__XO;
    real_type t45  = ModelPars[45] * phi__XO;
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
    real_type t132 = ModelPars[105];
    real_type t134 = ModelPars[97];
    real_type t137 = ModelPars[99] * ModelPars[99];
    real_type t138 = tan(alpha__r__XO);
    real_type t141 = pow(t138 - ModelPars[101], 2);
    real_type t143 = t141 * t137 + 1;
    real_type t144 = sqrt(t143);
    real_type t145 = 1.0 / t144;
    real_type t146 = t134 * t134;
    real_type t148 = 1.0 / t143 * t146;
    real_type t149 = ModelPars[109];
    real_type t150 = ModelPars[111];
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
  Baumgarte::Fyr_D_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[41];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t6   = ModelPars[47];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[48];
    real_type t16  = ModelPars[7];
    real_type t19  = Fzr__XO - t16;
    real_type t21  = t16 * ModelPars[35] + t19 * ModelPars[37];
    real_type t22  = ModelPars[43];
    real_type t24  = t22 * t5 + 1;
    real_type t26  = 1.0 / t24 * t21;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t26;
    real_type t29  = 1.0 / t1;
    real_type t30  = 1.0 / Fzr__XO;
    real_type t31  = t30 * t29;
    real_type t32  = Fzr__XO * t1;
    real_type t34  = t13 / t8;
    real_type t36  = ModelPars[39] * Fzr__XO;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__XO * t36 + alpha__r__XO;
    real_type t44  = ModelPars[45];
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
    real_type t168 = ModelPars[105];
    real_type t171 = ModelPars[99] * ModelPars[99];
    real_type t172 = tan(alpha__r__XO);
    real_type t175 = pow(t172 - ModelPars[101], 2);
    real_type t178 = sqrt(t175 * t171 + 1);
    real_type t180 = 1.0 / t178 * ModelPars[97];
    real_type t181 = ModelPars[109];
    real_type t185 = 1.0 / t16 * t19 * ModelPars[111];
    real_type t188 = atan((lambda__r__XO + t181 + t185) * t180);
    real_type t190 = cos(t188 * t168);
    real_type t194 = atan((t181 + t185) * t180);
    real_type t196 = cos(t194 * t168);
    return 1.0 / t196 * t190 * (-2 * t63 / t9 / t8 * t3 * t2 * t57 * t13 + t153 / (t146 * t9 / t3 / t2 / t140 * t69 * t138 + 1) * (-2 * phi__XO * t22 * t51 * t8 * t30 * t71 * t69 * t21 + 2 * t51 * phi__XO * t6 * t30 * t71 * t26 + (-2 * t63 * t41 * t13 * t10 * t32 + 2 * t63 * t41 / t12 / t11 / t93 * t3 * Fzr__XO * t2 * t1 + (-2 * t22 * t37 * t5 * t36 - t48 * t36) * t34 * t32 - t24 * t37 * t8 * t29 * t110 + 2 * t6 * t3 * t24 * t37 * t10 * t1 * t13 * t115 - 2 * t48 * t6 * t29 * t124 - 2 * t22 * t37 * t47 * t124) * t8 * t31 * t28) * t14 * t12 + t47 * t110 - 2 * t6 * t3 * t10 * t1 * t13 * t115 + 2 * t6 * t29 * t12 * t115);
  }

  real_type
  Baumgarte::Fyr_D_2_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[41];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t4   = t3 * t2;
    real_type t5   = phi__XO * phi__XO;
    real_type t6   = ModelPars[47];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t4;
    real_type t12  = sqrt(t11);
    real_type t14  = 1.0 / t12 / t11;
    real_type t15  = ModelPars[48];
    real_type t17  = ModelPars[7];
    real_type t20  = Fzr__XO - t17;
    real_type t22  = t17 * ModelPars[35] + t20 * ModelPars[37];
    real_type t23  = ModelPars[43];
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
    real_type t37  = ModelPars[39];
    real_type t38  = Fzr__XO * t37;
    real_type t39  = 1.0 / t22;
    real_type t43  = -t25 * t39 * phi__XO * t38 + alpha__r__XO;
    real_type t46  = ModelPars[45];
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
    real_type t354 = ModelPars[105];
    real_type t357 = ModelPars[99] * ModelPars[99];
    real_type t358 = tan(alpha__r__XO);
    real_type t361 = pow(t358 - ModelPars[101], 2);
    real_type t364 = sqrt(t361 * t357 + 1);
    real_type t366 = 1.0 / t364 * ModelPars[97];
    real_type t367 = ModelPars[109];
    real_type t371 = 1.0 / t17 * t20 * ModelPars[111];
    real_type t374 = atan((lambda__r__XO + t367 + t371) * t366);
    real_type t376 = cos(t374 * t354);
    real_type t380 = atan((t367 + t371) * t366);
    real_type t382 = cos(t380 * t354);
    return 1.0 / t382 * t376 * t353;
  }

  real_type
  Baumgarte::Fyr_D_2_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = phi__XO * phi__XO;
    real_type t2   = ModelPars[47];
    real_type t4   = t2 * t1 + 1;
    real_type t5   = 1.0 / t4;
    real_type t7   = ModelPars[7];
    real_type t10  = Fzr__XO - t7;
    real_type t12  = t10 * ModelPars[37] + t7 * ModelPars[35];
    real_type t14  = ModelPars[43];
    real_type t16  = t14 * t1 + 1;
    real_type t17  = 1.0 / t16;
    real_type t19  = t12 * t12;
    real_type t20  = t16 * t16;
    real_type t21  = 1.0 / t20;
    real_type t23  = ModelPars[48];
    real_type t24  = t23 * t23;
    real_type t27  = ModelPars[41];
    real_type t28  = t27 * t27;
    real_type t30  = Fzr__XO * Fzr__XO;
    real_type t33  = t4 * t4;
    real_type t34  = Fzr__XO * t27;
    real_type t36  = 1.0 / t33;
    real_type t37  = t36 * t30 * t28;
    real_type t38  = sqrt(t37);
    real_type t39  = 1.0 / t38;
    real_type t40  = t39 * t5;
    real_type t42  = ModelPars[39] * Fzr__XO;
    real_type t43  = 1.0 / t12;
    real_type t47  = -t16 * t43 * phi__XO * t42 + alpha__r__XO;
    real_type t50  = ModelPars[45];
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
    real_type t190 = ModelPars[105];
    real_type t191 = ModelPars[97];
    real_type t193 = ModelPars[99] * ModelPars[99];
    real_type t194 = tan(alpha__r__XO);
    real_type t196 = t194 - ModelPars[101];
    real_type t197 = t196 * t196;
    real_type t199 = t197 * t193 + 1;
    real_type t200 = sqrt(t199);
    real_type t202 = 1.0 / t200 * t191;
    real_type t203 = ModelPars[109];
    real_type t207 = 1.0 / t7 * t10 * ModelPars[111];
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
  Baumgarte::Fyr_D_2_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[41];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t6   = ModelPars[47];
    real_type t8   = t6 * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t3 * t2;
    real_type t12  = sqrt(t11);
    real_type t13  = 1.0 / t12;
    real_type t14  = ModelPars[48];
    real_type t16  = ModelPars[7];
    real_type t19  = Fzr__XO - t16;
    real_type t21  = t16 * ModelPars[35] + t19 * ModelPars[37];
    real_type t22  = ModelPars[43];
    real_type t24  = t22 * t5 + 1;
    real_type t26  = 1.0 / t24 * t21;
    real_type t27  = 1.0 / t14;
    real_type t28  = t27 * t26;
    real_type t29  = 1.0 / t1;
    real_type t30  = 1.0 / Fzr__XO;
    real_type t31  = t30 * t29;
    real_type t32  = Fzr__XO * t1;
    real_type t34  = t13 / t8;
    real_type t36  = ModelPars[39] * Fzr__XO;
    real_type t37  = 1.0 / t21;
    real_type t41  = -t24 * t37 * phi__XO * t36 + alpha__r__XO;
    real_type t44  = ModelPars[45];
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
    real_type t168 = ModelPars[105];
    real_type t170 = ModelPars[97];
    real_type t173 = ModelPars[99] * ModelPars[99];
    real_type t174 = tan(alpha__r__XO);
    real_type t177 = pow(t174 - ModelPars[101], 2);
    real_type t179 = t177 * t173 + 1;
    real_type t180 = sqrt(t179);
    real_type t181 = 1.0 / t180;
    real_type t182 = t170 * t170;
    real_type t185 = ModelPars[109];
    real_type t189 = 1.0 / t16 * t19 * ModelPars[111];
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
  Baumgarte::Fyr_D_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[7];
    real_type t5   = Fzr__XO - t2;
    real_type t7   = t2 * ModelPars[35] + t5 * ModelPars[37];
    real_type t8   = phi__XO * phi__XO;
    real_type t11  = ModelPars[43] * t8 + 1;
    real_type t13  = 1.0 / t11 * t7;
    real_type t14  = t7 * t7;
    real_type t15  = t11 * t11;
    real_type t18  = ModelPars[48];
    real_type t19  = t18 * t18;
    real_type t22  = ModelPars[41];
    real_type t23  = t22 * t22;
    real_type t25  = Fzr__XO * Fzr__XO;
    real_type t30  = ModelPars[47] * t8 + 1;
    real_type t31  = t30 * t30;
    real_type t37  = sqrt(1.0 / t31 * t25 * t23);
    real_type t42  = 1.0 / t7;
    real_type t50  = ModelPars[45] * phi__XO;
    real_type t52  = 1.0 / t22;
    real_type t57  = (-t11 * t42 * phi__XO * ModelPars[39] * Fzr__XO + alpha__r__XO) / t37 / t30 * Fzr__XO * t22 - t11 * t42 * t30 * t52 * t37 * t50;
    real_type t58  = t57 * t57;
    real_type t72  = atan(t57 * t30 / Fzr__XO * t52 / t18 * t13);
    real_type t73  = t72 * t18;
    real_type t74  = cos(t73);
    real_type t75  = ModelPars[105];
    real_type t76  = ModelPars[97];
    real_type t78  = ModelPars[99] * ModelPars[99];
    real_type t79  = tan(alpha__r__XO);
    real_type t81  = t79 - ModelPars[101];
    real_type t82  = t81 * t81;
    real_type t84  = t82 * t78 + 1;
    real_type t85  = sqrt(t84);
    real_type t87  = 1.0 / t85 * t76;
    real_type t88  = ModelPars[109];
    real_type t92  = 1.0 / t2 * t5 * ModelPars[111];
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
  Baumgarte::Fyr_D_3_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[7];
    real_type t5   = Fzr__XO - t2;
    real_type t7   = t2 * ModelPars[35] + t5 * ModelPars[37];
    real_type t8   = t7 * t7;
    real_type t10  = phi__XO * phi__XO;
    real_type t13  = ModelPars[43] * t10 + 1;
    real_type t14  = t13 * t13;
    real_type t19  = 1.0 / t14 * t8;
    real_type t20  = ModelPars[48];
    real_type t21  = t20 * t20;
    real_type t22  = 1.0 / t21;
    real_type t24  = ModelPars[41];
    real_type t25  = t24 * t24;
    real_type t27  = Fzr__XO * Fzr__XO;
    real_type t32  = ModelPars[47] * t10 + 1;
    real_type t33  = t32 * t32;
    real_type t39  = sqrt(1.0 / t33 * t27 * t25);
    real_type t40  = 1.0 / t39;
    real_type t44  = 1.0 / t7;
    real_type t52  = ModelPars[45] * phi__XO;
    real_type t54  = 1.0 / t24;
    real_type t59  = (-t13 * t44 * phi__XO * ModelPars[39] * Fzr__XO + alpha__r__XO) * t40 / t32 * Fzr__XO * t24 - t13 * t44 * t32 * t54 * t39 * t52;
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
    real_type t80  = ModelPars[105];
    real_type t81  = ModelPars[97];
    real_type t83  = ModelPars[99] * ModelPars[99];
    real_type t84  = tan(alpha__r__XO);
    real_type t86  = t84 - ModelPars[101];
    real_type t87  = t86 * t86;
    real_type t89  = t87 * t83 + 1;
    real_type t90  = sqrt(t89);
    real_type t92  = 1.0 / t90 * t81;
    real_type t93  = ModelPars[109];
    real_type t97  = 1.0 / t2 * t5 * ModelPars[111];
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
  Baumgarte::Fyr_D_3_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t2   = ModelPars[7];
    real_type t5   = Fzr__XO - t2;
    real_type t7   = t2 * ModelPars[35] + t5 * ModelPars[37];
    real_type t8   = phi__XO * phi__XO;
    real_type t11  = ModelPars[43] * t8 + 1;
    real_type t13  = 1.0 / t11 * t7;
    real_type t14  = t7 * t7;
    real_type t15  = t11 * t11;
    real_type t18  = ModelPars[48];
    real_type t19  = t18 * t18;
    real_type t22  = ModelPars[41];
    real_type t23  = t22 * t22;
    real_type t25  = Fzr__XO * Fzr__XO;
    real_type t30  = ModelPars[47] * t8 + 1;
    real_type t31  = t30 * t30;
    real_type t37  = sqrt(1.0 / t31 * t25 * t23);
    real_type t42  = 1.0 / t7;
    real_type t50  = ModelPars[45] * phi__XO;
    real_type t52  = 1.0 / t22;
    real_type t57  = (-t11 * t42 * phi__XO * ModelPars[39] * Fzr__XO + alpha__r__XO) / t37 / t30 * Fzr__XO * t22 - t11 * t42 * t30 * t52 * t37 * t50;
    real_type t58  = t57 * t57;
    real_type t71  = atan(t57 * t30 / Fzr__XO * t52 / t18 * t13);
    real_type t72  = t71 * t18;
    real_type t73  = cos(t72);
    real_type t75  = ModelPars[105];
    real_type t78  = ModelPars[97];
    real_type t80  = ModelPars[99] * ModelPars[99];
    real_type t81  = tan(alpha__r__XO);
    real_type t83  = t81 - ModelPars[101];
    real_type t84  = t83 * t83;
    real_type t86  = t84 * t80 + 1;
    real_type t87  = sqrt(t86);
    real_type t89  = 1.0 / t87 * t78;
    real_type t90  = t78 * t78;
    real_type t92  = 1.0 / t86 * t90;
    real_type t93  = ModelPars[109];
    real_type t97  = 1.0 / t2 * t5 * ModelPars[111];
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
  Baumgarte::Fyr_D_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[41];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t8   = ModelPars[47] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t12  = sqrt(1.0 / t9 * t3 * t2);
    real_type t13  = ModelPars[48];
    real_type t15  = ModelPars[7];
    real_type t18  = Fzr__XO - t15;
    real_type t20  = t15 * ModelPars[35] + t18 * ModelPars[37];
    real_type t23  = ModelPars[43] * t5 + 1;
    real_type t28  = 1.0 / t1;
    real_type t37  = 1.0 / t20;
    real_type t45  = ModelPars[45] * phi__XO;
    real_type t55  = atan(((-t23 * t37 * phi__XO * ModelPars[39] * Fzr__XO + alpha__r__XO) / t12 / t8 * Fzr__XO * t1 - t23 * t37 * t8 * t28 * t12 * t45) * t8 / Fzr__XO * t28 / t13 / t23 * t20);
    real_type t57  = sin(t55 * t13);
    real_type t63  = ModelPars[105];
    real_type t65  = ModelPars[97];
    real_type t68  = ModelPars[99] * ModelPars[99];
    real_type t69  = tan(alpha__r__XO);
    real_type t72  = pow(t69 - ModelPars[101], 2);
    real_type t74  = t72 * t68 + 1;
    real_type t75  = sqrt(t74);
    real_type t76  = 1.0 / t75;
    real_type t77  = t65 * t65;
    real_type t80  = ModelPars[109];
    real_type t84  = 1.0 / t15 * t18 * ModelPars[111];
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
  Baumgarte::Fyr_D_4_4( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const {
    real_type t1   = ModelPars[41];
    real_type t2   = t1 * t1;
    real_type t3   = Fzr__XO * Fzr__XO;
    real_type t5   = phi__XO * phi__XO;
    real_type t8   = ModelPars[47] * t5 + 1;
    real_type t9   = t8 * t8;
    real_type t12  = sqrt(1.0 / t9 * t3 * t2);
    real_type t13  = ModelPars[48];
    real_type t15  = ModelPars[7];
    real_type t18  = Fzr__XO - t15;
    real_type t20  = t15 * ModelPars[35] + t18 * ModelPars[37];
    real_type t23  = ModelPars[43] * t5 + 1;
    real_type t28  = 1.0 / t1;
    real_type t37  = 1.0 / t20;
    real_type t45  = ModelPars[45] * phi__XO;
    real_type t55  = atan(((-t23 * t37 * phi__XO * ModelPars[39] * Fzr__XO + alpha__r__XO) / t12 / t8 * Fzr__XO * t1 - t23 * t37 * t8 * t28 * t12 * t45) * t8 / Fzr__XO * t28 / t13 / t23 * t20);
    real_type t57  = sin(t55 * t13);
    real_type t62  = t8 * t28 * t12 * t45 + t57 * t12;
    real_type t63  = ModelPars[105];
    real_type t65  = ModelPars[97];
    real_type t66  = t65 * t65;
    real_type t69  = ModelPars[99] * ModelPars[99];
    real_type t70  = tan(alpha__r__XO);
    real_type t73  = pow(t70 - ModelPars[101], 2);
    real_type t75  = t73 * t69 + 1;
    real_type t76  = sqrt(t75);
    real_type t81  = 1.0 / t75;
    real_type t83  = ModelPars[109];
    real_type t87  = 1.0 / t15 * t18 * ModelPars[111];
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
  Baumgarte::Mzf( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t4   = ModelPars[6];
    real_type t9   = t4 * ModelPars[34] + (Fzf__XO - t4) * ModelPars[36];
    real_type t13  = phi__f__XO * phi__f__XO;
    real_type t17  = 1.0 / (ModelPars[55] * t13 + 1);
    real_type t18  = t17 * ModelPars[49];
    real_type t20  = ModelPars[40];
    real_type t25  = ModelPars[46] * t13 + 1;
    real_type t26  = 1.0 / t25;
    real_type t27  = t20 * t20;
    real_type t28  = Fzf__XO * Fzf__XO;
    real_type t30  = t25 * t25;
    real_type t33  = sqrt(1.0 / t30 * t28 * t27);
    real_type t34  = 1.0 / t33;
    real_type t38  = atan(alpha__f__XO * t34 * t26 * Fzf__XO * t20 * ModelPars[57]);
    real_type t40  = cos(t38 * t18);
    real_type t42  = ModelPars[48];
    real_type t52  = atan(alpha__f__XO * t34 / t42 / (ModelPars[42] * t13 + 1) * t9);
    real_type t54  = sin(t52 * t42);
    real_type t60  = ModelPars[56];
    real_type t62  = atan(phi__f__XO * t60);
    real_type t76  = atan(alpha__f__XO * t34 * t26 * Fzf__XO * t20 / (ModelPars[54] * t13 + 1) * ModelPars[58]);
    real_type t78  = cos(t76 * t18);
    return -t54 * t33 * t17 * t40 / t9 * Fzf__XO * ModelPars[50] + t78 / t60 * t62 * ModelPars[52] * Fzf__XO;
  }

  real_type
  Baumgarte::Mzf_D_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[50];
    real_type t3   = ModelPars[6];
    real_type t5   = ModelPars[36];
    real_type t8   = t3 * ModelPars[34] + (Fzf__XO - t3) * t5;
    real_type t9   = 1.0 / t8;
    real_type t11  = ModelPars[49];
    real_type t12  = phi__f__XO * phi__f__XO;
    real_type t15  = ModelPars[55] * t12 + 1;
    real_type t16  = 1.0 / t15;
    real_type t17  = t16 * t11;
    real_type t18  = ModelPars[57];
    real_type t19  = ModelPars[40];
    real_type t20  = t19 * t18;
    real_type t24  = ModelPars[46] * t12 + 1;
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
    real_type t42  = ModelPars[48];
    real_type t45  = ModelPars[42] * t12 + 1;
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
    real_type t132 = ModelPars[52];
    real_type t133 = ModelPars[56];
    real_type t135 = atan(phi__f__XO * t133);
    real_type t137 = 1.0 / t133;
    real_type t138 = ModelPars[58];
    real_type t141 = ModelPars[54] * t12 + 1;
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
  Baumgarte::Mzf_D_1_1( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[50];
    real_type t2   = Fzf__XO * t1;
    real_type t4   = ModelPars[6];
    real_type t6   = ModelPars[36];
    real_type t9   = t4 * ModelPars[34] + (Fzf__XO - t4) * t6;
    real_type t10  = t9 * t9;
    real_type t11  = 1.0 / t10;
    real_type t12  = ModelPars[49];
    real_type t14  = phi__f__XO * phi__f__XO;
    real_type t17  = ModelPars[55] * t14 + 1;
    real_type t18  = t17 * t17;
    real_type t19  = 1.0 / t18;
    real_type t22  = ModelPars[57];
    real_type t23  = ModelPars[40];
    real_type t24  = t23 * t22;
    real_type t27  = ModelPars[46] * t14 + 1;
    real_type t28  = 1.0 / t27;
    real_type t29  = t23 * t23;
    real_type t30  = Fzf__XO * Fzf__XO;
    real_type t32  = t27 * t27;
    real_type t33  = 1.0 / t32;
    real_type t34  = t33 * t30 * t29;
    real_type t35  = sqrt(t34);
    real_type t36  = 1.0 / t35;
    real_type t38  = alpha__f__XO * t36 * t28;
    real_type t40  = t29 * t23;
    real_type t41  = t40 * t22;
    real_type t44  = 1.0 / t32 / t27;
    real_type t46  = 1.0 / t35 / t34;
    real_type t47  = t46 * t44;
    real_type t50  = -alpha__f__XO * t47 * t30 * t41 + t38 * t24;
    real_type t51  = alpha__f__XO * alpha__f__XO;
    real_type t52  = t22 * t22;
    real_type t54  = t52 * t51 + 1;
    real_type t55  = 1.0 / t54;
    real_type t56  = t55 * t50;
    real_type t57  = 1.0 / t17;
    real_type t58  = t57 * t12;
    real_type t61  = atan(t38 * Fzf__XO * t24);
    real_type t62  = t61 * t58;
    real_type t63  = sin(t62);
    real_type t65  = ModelPars[48];
    real_type t68  = ModelPars[42] * t14 + 1;
    real_type t69  = 1.0 / t68;
    real_type t70  = t69 * t9;
    real_type t71  = 1.0 / t65;
    real_type t73  = alpha__f__XO * t36 * t71;
    real_type t75  = atan(t73 * t70);
    real_type t76  = t75 * t65;
    real_type t77  = sin(t76);
    real_type t83  = cos(t62);
    real_type t85  = t57 * t83 * t11;
    real_type t87  = t65 * t35;
    real_type t88  = t69 * t6;
    real_type t90  = t46 * t71;
    real_type t94  = t33 * Fzf__XO * t29 * alpha__f__XO;
    real_type t96  = -t94 * t90 * t70 + t73 * t88;
    real_type t97  = t96 * t87;
    real_type t98  = t68 * t68;
    real_type t99  = 1.0 / t98;
    real_type t101 = t65 * t65;
    real_type t102 = 1.0 / t101;
    real_type t103 = t102 * t99 * t10;
    real_type t104 = 1.0 / t29;
    real_type t105 = 1.0 / t30;
    real_type t107 = t51 * t32;
    real_type t110 = t107 * t105 * t104 * t103 + 1;
    real_type t111 = 1.0 / t110;
    real_type t112 = cos(t76);
    real_type t113 = t112 * t111;
    real_type t118 = 1.0 / t9;
    real_type t119 = t83 * t118;
    real_type t121 = t57 * t119 * t2;
    real_type t122 = t110 * t110;
    real_type t123 = 1.0 / t122;
    real_type t132 = t30 * Fzf__XO;
    real_type t143 = t19 * t12;
    real_type t144 = t50 * t143;
    real_type t146 = t63 * t55;
    real_type t148 = t96 * t65;
    real_type t153 = ModelPars[52];
    real_type t155 = ModelPars[56];
    real_type t157 = atan(phi__f__XO * t155);
    real_type t158 = 1.0 / t155;
    real_type t160 = t158 * t157 * Fzf__XO * t153;
    real_type t161 = t12 * t12;
    real_type t163 = ModelPars[58];
    real_type t166 = ModelPars[54] * t14 + 1;
    real_type t168 = 1.0 / t166 * t163;
    real_type t169 = t23 * t168;
    real_type t171 = t40 * t168;
    real_type t173 = alpha__f__XO * t46;
    real_type t176 = -t173 * t44 * t30 * t171 + t38 * t169;
    real_type t177 = t176 * t176;
    real_type t178 = t163 * t163;
    real_type t179 = t166 * t166;
    real_type t183 = t51 / t179 * t178 + 1;
    real_type t184 = t183 * t183;
    real_type t191 = atan(alpha__f__XO * t36 * t28 * Fzf__XO * t169);
    real_type t192 = t191 * t58;
    real_type t193 = cos(t192);
    real_type t197 = t30 * t1;
    real_type t198 = t118 * t197;
    real_type t206 = t57 * t83;
    real_type t219 = 1.0 / t183;
    real_type t220 = sin(t192);
    real_type t230 = t50 * t50;
    real_type t231 = t54 * t54;
    real_type t241 = t29 * t29;
    real_type t242 = t241 * t23;
    real_type t244 = t32 * t32;
    real_type t246 = 1.0 / t244 / t27;
    real_type t248 = t30 * t30;
    real_type t250 = 1.0 / t244;
    real_type t253 = 1.0 / t35 / t250 / t248 / t241;
    real_type t263 = t118 * t1;
    real_type t266 = t77 * t35 * t63;
    real_type t270 = t206 * t263;
    real_type t271 = t77 * t36;
    real_type t286 = t35 * t57;
    real_type t287 = t6 * t6;
    real_type t319 = t96 * t96;
    return -2 * t6 * t77 * t35 * t63 * t56 * t19 * t12 * t11 * t2 + 2 * t6 * t113 * t97 * t85 * t2 + (2 * t6 * t51 * t32 * t105 * t104 * t102 * t99 * t9 - 2 * t107 / t132 * t104 * t103) * t112 * t123 * t97 * t121 + 2 * t113 * t148 * t35 * t146 * t144 * t118 * t2 - t193 / t184 * t177 * t19 * t161 * t160 + 2 * t33 * t29 * t77 * t36 * t146 * t144 * t198 - 2 * t33 * t29 * t112 * t111 * t148 * t36 * t206 * t198 - 2 * t220 * t219 * t176 * t57 * t12 * t158 * t157 * t153 + t77 * t35 * t83 / t231 * t230 / t18 / t17 * t161 * t118 * t2 - t220 * t219 * (3 * alpha__f__XO * t253 * t246 * t132 * t242 * t168 - 3 * alpha__f__XO * Fzf__XO * t47 * t171) * t58 * t160 + 2 * t266 * t56 * t143 * t263 - 3 * t33 * Fzf__XO * t29 * t271 * t270 - 2 * t112 * t111 * t96 * t87 * t270 - 2 * t287 * t77 * t286 * t83 / t10 / t9 * t2 + t250 * t241 * t77 * t46 * t57 * t119 * t132 * t1 - t112 * t111 * (3 * t250 * t30 * t241 * alpha__f__XO * t253 * t71 * t70 - t33 * t29 * t173 * t71 * t70 - 2 * t94 * t90 * t88) * t87 * t121 + t77 * t123 * t319 * t101 * t35 * t121 + 2 * t33 * t29 * t6 * t271 * t85 * t197 + t266 * t55 * (3 * alpha__f__XO * t253 * t246 * t132 * t242 * t22 - 3 * Fzf__XO * t173 * t44 * t41) * t19 * t12 * t118 * t2 + 2 * t6 * t77 * t286 * t83 * t11 * t1;
  }

  real_type
  Baumgarte::Mzf_D_1_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[50];
    real_type t2   = Fzf__XO * Fzf__XO;
    real_type t3   = t2 * Fzf__XO;
    real_type t4   = t3 * t1;
    real_type t6   = ModelPars[6];
    real_type t8   = ModelPars[36];
    real_type t11  = t6 * ModelPars[34] + (Fzf__XO - t6) * t8;
    real_type t12  = 1.0 / t11;
    real_type t13  = t12 * t4;
    real_type t14  = ModelPars[49];
    real_type t15  = phi__f__XO * phi__f__XO;
    real_type t16  = ModelPars[55];
    real_type t18  = t16 * t15 + 1;
    real_type t19  = t18 * t18;
    real_type t20  = 1.0 / t19;
    real_type t21  = t20 * t14;
    real_type t22  = ModelPars[57];
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
    real_type t37  = alpha__f__XO * t36;
    real_type t39  = t29 * t23;
    real_type t40  = t39 * t22;
    real_type t43  = 1.0 / t31 / t27;
    real_type t45  = 1.0 / t34 / t33;
    real_type t46  = t45 * t43;
    real_type t49  = -alpha__f__XO * t46 * t2 * t40 + t37 * t24;
    real_type t50  = alpha__f__XO * alpha__f__XO;
    real_type t51  = t22 * t22;
    real_type t54  = 1.0 / (t51 * t50 + 1);
    real_type t55  = t54 * t49;
    real_type t58  = 1.0 / t18;
    real_type t59  = t58 * t14;
    real_type t62  = atan(t37 * Fzf__XO * t24);
    real_type t63  = t62 * t59;
    real_type t64  = sin(t63);
    real_type t66  = ModelPars[48];
    real_type t67  = ModelPars[42];
    real_type t69  = t67 * t15 + 1;
    real_type t70  = 1.0 / t69;
    real_type t71  = t70 * t11;
    real_type t72  = 1.0 / t66;
    real_type t74  = alpha__f__XO * t35 * t72;
    real_type t76  = atan(t74 * t71);
    real_type t77  = t76 * t66;
    real_type t78  = sin(t77);
    real_type t81  = phi__f__XO * t25;
    real_type t82  = t81 * t43 * t29;
    real_type t86  = ModelPars[52];
    real_type t87  = Fzf__XO * t86;
    real_type t88  = ModelPars[56];
    real_type t90  = atan(phi__f__XO * t88);
    real_type t92  = 1.0 / t88;
    real_type t94  = ModelPars[58];
    real_type t95  = ModelPars[54];
    real_type t97  = t95 * t15 + 1;
    real_type t99  = 1.0 / t97 * t94;
    real_type t100 = t23 * t99;
    real_type t104 = alpha__f__XO * t45;
    real_type t107 = -t104 * t43 * t2 * t39 * t99 + t37 * t100;
    real_type t108 = t107 * t58;
    real_type t111 = t94 * t94;
    real_type t112 = t97 * t97;
    real_type t113 = 1.0 / t112;
    real_type t116 = t50 * t113 * t111 + 1;
    real_type t117 = t116 * t116;
    real_type t120 = alpha__f__XO * t35;
    real_type t123 = atan(t120 * t28 * Fzf__XO * t100);
    real_type t124 = t123 * t59;
    real_type t125 = sin(t124);
    real_type t131 = phi__f__XO * t95;
    real_type t136 = cos(t63);
    real_type t137 = t58 * t136;
    real_type t141 = t70 * t8;
    real_type t143 = t45 * t72;
    real_type t146 = t32 * Fzf__XO;
    real_type t149 = -t146 * t29 * alpha__f__XO * t143 * t71 + t74 * t141;
    real_type t150 = t11 * t11;
    real_type t151 = t69 * t69;
    real_type t152 = 1.0 / t151;
    real_type t153 = t152 * t150;
    real_type t154 = t66 * t66;
    real_type t155 = 1.0 / t154;
    real_type t157 = 1.0 / t29;
    real_type t158 = 1.0 / t2;
    real_type t163 = t50 * t31 * t158 * t157 * t155 * t153 + 1;
    real_type t164 = 1.0 / t163;
    real_type t166 = cos(t77);
    real_type t167 = t166 * t164 * t149;
    real_type t171 = Fzf__XO * t1;
    real_type t172 = t136 * t12;
    real_type t173 = t58 * t172;
    real_type t174 = t173 * t171;
    real_type t175 = t66 * t34;
    real_type t179 = phi__f__XO * t67 * t120;
    real_type t182 = alpha__f__XO * t143;
    real_type t185 = phi__f__XO * t25 * t43;
    real_type t186 = t185 * t30;
    real_type t189 = t152 * t11;
    real_type t191 = Fzf__XO * t29;
    real_type t197 = t29 * t29;
    real_type t198 = t2 * t2;
    real_type t200 = t31 * t31;
    real_type t201 = 1.0 / t200;
    real_type t204 = 1.0 / t34 / t201 / t198 / t197;
    real_type t212 = phi__f__XO * t25 / t200 / t27;
    real_type t216 = t182 * t71;
    real_type t225 = t92 * t90;
    real_type t227 = t14 * t225 * t87;
    real_type t228 = 1.0 / t116;
    real_type t233 = t113 * t94;
    real_type t234 = Fzf__XO * t23;
    real_type t237 = phi__f__XO * t95 * alpha__f__XO;
    real_type t243 = phi__f__XO * t25 * alpha__f__XO;
    real_type t248 = t45 * t201;
    real_type t255 = -2 * phi__f__XO * t16 * t123 * t21 + t228 * (-2 * t243 * t35 * t32 * t234 * t99 + 2 * t243 * t248 * t3 * t39 * t99 - 2 * t237 * t36 * t234 * t233) * t59;
    real_type t257 = cos(t124);
    real_type t263 = t20 * t14 * t12 * t171;
    real_type t265 = t81 * t120;
    real_type t273 = t197 * t23;
    real_type t276 = 1.0 / t200 / t31;
    real_type t289 = t2 * t1;
    real_type t304 = -2 * phi__f__XO * t16 * t62 * t21 + t54 * (2 * t81 * t104 * t201 * t3 * t40 - 2 * t265 * t146 * t24) * t59;
    real_type t306 = t64 * t304 * t12;
    real_type t309 = t29 * t78;
    real_type t313 = 1.0 / t150;
    real_type t314 = t136 * t313;
    real_type t317 = t78 * t34;
    real_type t324 = t35 * t137;
    real_type t330 = -2 * t179 * t72 * t189 + 2 * t186 * t216;
    real_type t331 = t330 * t66;
    real_type t337 = t12 * t171;
    real_type t344 = t34 * t64 * t54;
    real_type t357 = t20 * t136;
    real_type t374 = t163 * t163;
    real_type t375 = 1.0 / t374;
    real_type t380 = t12 * t1;
    real_type t386 = -2 * t82 * t78 * t35 * t64 * t55 * t21 * t13 - 4 * t131 * t50 / t112 / t97 * t111 * t125 / t117 * t108 * t14 * t92 * t90 * t87 + 2 * t82 * t167 * t66 * t35 * t137 * t13 - t166 * t164 * (-6 * t212 * t3 * t197 * alpha__f__XO * t204 * t72 * t71 + 2 * phi__f__XO * t67 * t32 * t191 * t182 * t189 - 2 * t179 * t72 * t152 * t8 + 2 * t186 * t182 * t141 + 4 * t185 * t191 * t216) * t175 * t174 - t257 * t255 * t228 * t108 * t227 + t78 * t34 * t64 * t54 * (-6 * t81 * alpha__f__XO * t204 * t276 * t198 * t273 * t22 + 8 * t81 * t2 * alpha__f__XO * t248 * t40 - 2 * t265 * t32 * t24) * t263 + t32 * t309 * t35 * t58 * t306 * t289 - 2 * phi__f__XO * t16 * t8 * t317 * t20 * t314 * t171 - t32 * t29 * t166 * t164 * t331 * t324 * t12 * t289 - 4 * phi__f__XO * t16 * t78 * t344 * t49 / t19 / t18 * t14 * t337 - 2 * t185 * t29 * t8 * t78 * t324 * t313 * t4 + 2 * phi__f__XO * t16 * t166 * t164 * t149 * t66 * t34 * t357 * t337 + t167 * t66 * t34 * t58 * t306 * t171 + t78 * t330 * t375 * t149 * t154 * t34 * t174 + 6 * t185 * t2 * t309 * t324 * t380;
    real_type t390 = t166 * t164;
    real_type t427 = t157 * t155;
    real_type t446 = t58 * t64;
    real_type t458 = t88 * t88;
    real_type t461 = 1.0 / (t458 * t15 + 1);
    real_type t517 = t8 * t390 * t330 * t175 * t58 * t314 * t171 + t78 * t34 * t136 * t304 * t55 * t263 + 2 * phi__f__XO * t16 * t32 * t29 * t78 * t35 * t20 * t172 * t289 - 2 * t212 * t197 * t78 * t45 * t173 * t198 * t1 + 2 * phi__f__XO * t16 * t125 * t228 * t107 * t20 * t227 + (-4 * phi__f__XO * t67 * t50 * t31 * t158 * t427 / t151 / t69 * t150 + 4 * phi__f__XO * t25 * t50 * t27 * t158 * t427 * t153) * t166 * t375 * t149 * t175 * t174 + t317 * t446 * t304 * t380 - t125 * t255 * t92 * t90 * t86 + t390 * t331 * t344 * t49 * t21 * t337 + t257 * t461 * t86 + 2 * phi__f__XO * t16 * t317 * t357 * t380 - t125 * t228 * t108 * t14 * t461 * t87 - t8 * t317 * t446 * t304 * t313 * t171 - t166 * t164 * t330 * t175 * t137 * t380 - t125 * t228 * (8 * phi__f__XO * t25 * t2 * t104 * t201 * t39 * t99 - 6 * t243 * t204 * t276 * t198 * t273 * t99 - 2 * t131 * t120 * t28 * t23 * t233 + 2 * t237 * t46 * t2 * t39 * t233 - 2 * t265 * t32 * t23 * t99) * t59 * t225 * t87;
    return t386 + t517;
  }

  real_type
  Baumgarte::Mzf_D_1_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[50];
    real_type t3   = ModelPars[6];
    real_type t5   = ModelPars[36];
    real_type t8   = t3 * ModelPars[34] + (Fzf__XO - t3) * t5;
    real_type t9   = 1.0 / t8;
    real_type t11  = ModelPars[49];
    real_type t12  = phi__f__XO * phi__f__XO;
    real_type t15  = ModelPars[55] * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t17  = 1.0 / t16;
    real_type t18  = t17 * t11;
    real_type t19  = ModelPars[57];
    real_type t20  = t19 * t18;
    real_type t22  = ModelPars[40];
    real_type t26  = ModelPars[46] * t12 + 1;
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
    real_type t53  = ModelPars[48];
    real_type t56  = ModelPars[42] * t12 + 1;
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
    real_type t216 = ModelPars[52];
    real_type t217 = ModelPars[56];
    real_type t219 = atan(phi__f__XO * t217);
    real_type t221 = 1.0 / t217;
    real_type t223 = ModelPars[58];
    real_type t228 = ModelPars[54] * t12 + 1;
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
  Baumgarte::Mzf_D_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[50];
    real_type t2   = Fzf__XO * t1;
    real_type t4   = ModelPars[6];
    real_type t9   = t4 * ModelPars[34] + (Fzf__XO - t4) * ModelPars[36];
    real_type t10  = 1.0 / t9;
    real_type t11  = ModelPars[49];
    real_type t12  = phi__f__XO * phi__f__XO;
    real_type t13  = ModelPars[55];
    real_type t15  = t13 * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t17  = 1.0 / t16;
    real_type t18  = t17 * t11;
    real_type t19  = ModelPars[57];
    real_type t20  = ModelPars[40];
    real_type t21  = t20 * t19;
    real_type t23  = ModelPars[46];
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
    real_type t78  = ModelPars[48];
    real_type t79  = ModelPars[42];
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
    real_type t150 = ModelPars[52] * Fzf__XO;
    real_type t151 = ModelPars[56];
    real_type t152 = t151 * t151;
    real_type t156 = ModelPars[58];
    real_type t157 = ModelPars[54];
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
  Baumgarte::Mzf_D_2_2( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t2   = ModelPars[52] * Fzf__XO;
    real_type t3   = phi__f__XO * phi__f__XO;
    real_type t4   = ModelPars[56];
    real_type t5   = t4 * t4;
    real_type t7   = t5 * t3 + 1;
    real_type t9   = ModelPars[49];
    real_type t10  = ModelPars[55];
    real_type t12  = t10 * t3 + 1;
    real_type t13  = t12 * t12;
    real_type t14  = 1.0 / t13;
    real_type t15  = t14 * t9;
    real_type t16  = ModelPars[58];
    real_type t17  = ModelPars[54];
    real_type t18  = t17 * t3;
    real_type t19  = t18 + 1;
    real_type t21  = 1.0 / t19 * t16;
    real_type t22  = ModelPars[40];
    real_type t24  = ModelPars[46];
    real_type t26  = t24 * t3 + 1;
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
    real_type t46  = t45 * t9;
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
    real_type t86  = t40 * t46;
    real_type t87  = sin(t86);
    real_type t92  = atan(phi__f__XO * t4);
    real_type t93  = t92 * t2;
    real_type t94  = 1.0 / t4;
    real_type t96  = 1.0 / t13 / t12;
    real_type t97  = t96 * t9;
    real_type t98  = t10 * t10;
    real_type t111 = 1.0 / t47 / t19;
    real_type t114 = t17 * t17;
    real_type t122 = t24 * t18;
    real_type t128 = alpha__f__XO * t70;
    real_type t135 = t32 * t26;
    real_type t136 = 1.0 / t135;
    real_type t138 = t24 * t24;
    real_type t140 = t3 * t138 * alpha__f__XO;
    real_type t145 = 1.0 / t67 / t26;
    real_type t150 = t59 * t58;
    real_type t153 = t29 * t29;
    real_type t154 = t153 * t22;
    real_type t155 = t30 * t30;
    real_type t156 = t155 * Fzf__XO;
    real_type t160 = 1.0 / t67 / t135;
    real_type t161 = t155 * t153;
    real_type t164 = 1.0 / t35 / t68 / t161;
    real_type t169 = t59 * t71;
    real_type t175 = t80 * t80;
    real_type t189 = t7 * t7;
    real_type t192 = cos(t86);
    real_type t197 = ModelPars[50];
    real_type t198 = t64 * t197;
    real_type t200 = ModelPars[6];
    real_type t205 = t200 * ModelPars[34] + (Fzf__XO - t200) * ModelPars[36];
    real_type t206 = 1.0 / t205;
    real_type t207 = ModelPars[57];
    real_type t208 = t22 * t207;
    real_type t209 = Fzf__XO * t208;
    real_type t212 = atan(alpha__f__XO * t52 * t209);
    real_type t213 = t212 * t46;
    real_type t214 = cos(t213);
    real_type t215 = t214 * t206;
    real_type t216 = t45 * t215;
    real_type t217 = t216 * t198;
    real_type t218 = ModelPars[48];
    real_type t219 = ModelPars[42];
    real_type t220 = t219 * t3;
    real_type t221 = t220 + 1;
    real_type t223 = 1.0 / t221 * t205;
    real_type t224 = 1.0 / t218;
    real_type t228 = atan(alpha__f__XO * t36 * t224 * t223);
    real_type t229 = t228 * t218;
    real_type t230 = sin(t229);
    real_type t231 = t230 * t36;
    real_type t232 = t136 * t29;
    real_type t233 = t24 * t232;
    real_type t244 = t3 * t138 / t67 / t32;
    real_type t248 = t29 * t231;
    real_type t250 = t3 * t138 * t68;
    real_type t254 = Fzf__XO * t197;
    real_type t255 = t216 * t254;
    real_type t256 = t218 * t35;
    real_type t257 = t221 * t221;
    real_type t258 = 1.0 / t257;
    real_type t259 = t258 * t205;
    real_type t260 = t224 * t259;
    real_type t264 = t70 * t224;
    real_type t265 = alpha__f__XO * t264;
    real_type t266 = t265 * t223;
    real_type t268 = phi__f__XO * t24 * t136;
    real_type t272 = -2 * phi__f__XO * t219 * t37 * t260 + 2 * t268 * t31 * t266;
    real_type t274 = t205 * t205;
    real_type t275 = t258 * t274;
    real_type t276 = t218 * t218;
    real_type t277 = 1.0 / t276;
    real_type t279 = 1.0 / t29;
    real_type t280 = 1.0 / t30;
    real_type t285 = t78 * t32 * t280 * t279 * t277 * t275 + 1;
    real_type t286 = t285 * t285;
    real_type t287 = 1.0 / t286;
    real_type t288 = cos(t229);
    real_type t291 = 1.0 / t257 / t221;
    real_type t293 = t279 * t277;
    real_type t311 = t10 * t212;
    real_type t317 = phi__f__XO * t24;
    real_type t320 = t63 * t207;
    real_type t326 = -2 * t317 * t37 * t33 * Fzf__XO * t208 + 2 * t317 * t128 * t68 * t64 * t320;
    real_type t327 = t207 * t207;
    real_type t330 = 1.0 / (t327 * t78 + 1);
    real_type t333 = -2 * phi__f__XO * t311 * t15 + t330 * t326 * t46;
    real_type t335 = sin(t213);
    real_type t337 = t335 * t333 * t206 * t254;
    real_type t340 = 1.0 / t285;
    real_type t342 = t288 * t340 * t272;
    real_type t346 = t206 * t198;
    real_type t353 = t14 * t214;
    real_type t356 = t10 * t230;
    real_type t384 = t3 * t138;
    real_type t412 = t230 * t35;
    real_type t415 = t45 * t214;
    real_type t423 = t333 * t333;
    real_type t428 = t215 * t254;
    real_type t429 = t35 * t14;
    real_type t441 = t219 * t219;
    real_type t449 = t24 * t136 * t30;
    real_type t476 = t272 * t272;
    real_type t485 = t84 * t84;
    return -2 * t87 * t84 / t7 * t2 - t87 * (8 * t3 * t98 * t40 * t97 - 4 * phi__f__XO * t10 * t81 * t75 * t15 - 2 * t41 * t15 + t81 * (8 * t3 * t114 * alpha__f__XO * t52 * t50 * t111 * t16 + 12 * t140 * t164 * t160 * t156 * t154 * t21 - 8 * t122 * t128 * t68 * t65 * t49 + 8 * t122 * t37 * t33 * t50 * t49 + 8 * t140 * t36 * t136 * t57 - 20 * t140 * t70 * t145 * t66 - 2 * t53 * t52 * t51 - 2 * t150 * t57 + 2 * t169 * t66) * t46 + 4 * phi__f__XO * t17 * t78 * t111 * t76 / t175 * t75 * t46) * t94 * t93 - 2 * phi__f__XO * t5 * t192 / t189 * t2 + 2 * t233 * t231 * t217 + 4 * t244 * t153 * t230 * t70 * t216 * t156 * t197 - 12 * t250 * t248 * t217 + (-4 * phi__f__XO * t219 * t78 * t32 * t280 * t293 * t291 * t274 + 4 * phi__f__XO * t24 * t78 * t26 * t280 * t293 * t275) * t288 * t287 * t272 * t256 * t255 + 2 * t342 * t218 * t35 * t45 * t337 - 4 * t268 * t248 * t45 * t335 * t333 * t346 - 8 * t233 * t3 * t356 * t36 * t353 * t346 + 4 * phi__f__XO * t10 * t288 * t340 * t272 * t218 * t35 * t353 * t206 * t254 + t412 * t45 * t335 * (8 * t3 * t98 * t212 * t97 - 4 * phi__f__XO * t10 * t330 * t326 * t15 - 2 * t311 * t15 + t330 * (12 * t384 * alpha__f__XO * t164 * t160 * t156 * t154 * t207 + 8 * t384 * t37 * t136 * Fzf__XO * t208 - 20 * t384 * t128 * t145 * t64 * t320 + 2 * t169 * t64 * t320 - 2 * t150 * t209) * t46) * t206 * t254 + 4 * t317 * t232 * t342 * t218 * t36 * t415 * t346 + t412 * t415 * t423 * t206 * t254 + 2 * t356 * t429 * t428 - 8 * t3 * t98 * t230 * t35 * t96 * t428 - t288 * t340 * (12 * t244 * t161 * alpha__f__XO * t164 * t224 * t223 + 8 * t3 * t441 * t37 * t224 * t291 * t205 - 8 * t449 * t29 * t220 * t265 * t259 + 2 * t449 * t29 * alpha__f__XO * t264 * t223 - 2 * t219 * t37 * t260 - 12 * t250 * t31 * t266) * t256 * t255 + t230 * t287 * t476 * t276 * t35 * t255 - 4 * phi__f__XO * t356 * t429 * t337 - t192 * t485 * t94 * t93;
  }

  real_type
  Baumgarte::Mzf_D_2_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[50];
    real_type t2   = Fzf__XO * t1;
    real_type t4   = ModelPars[6];
    real_type t9   = t4 * ModelPars[34] + (Fzf__XO - t4) * ModelPars[36];
    real_type t10  = 1.0 / t9;
    real_type t11  = ModelPars[49];
    real_type t12  = phi__f__XO * phi__f__XO;
    real_type t13  = ModelPars[55];
    real_type t15  = t13 * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t17  = 1.0 / t16;
    real_type t18  = t17 * t11;
    real_type t19  = ModelPars[57];
    real_type t20  = ModelPars[40];
    real_type t21  = t20 * t19;
    real_type t22  = Fzf__XO * t21;
    real_type t24  = ModelPars[46];
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
    real_type t55  = Fzf__XO * t29;
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
    real_type t96  = ModelPars[48];
    real_type t97  = ModelPars[42];
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
    real_type t249 = ModelPars[52];
    real_type t250 = t29 * t249;
    real_type t251 = ModelPars[56];
    real_type t252 = t251 * t251;
    real_type t257 = ModelPars[58];
    real_type t260 = ModelPars[54];
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
  Baumgarte::Mzf_D_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[50];
    real_type t2   = Fzf__XO * Fzf__XO;
    real_type t5   = ModelPars[6];
    real_type t10  = t5 * ModelPars[34] + (Fzf__XO - t5) * ModelPars[36];
    real_type t12  = ModelPars[49];
    real_type t14  = phi__f__XO * phi__f__XO;
    real_type t17  = ModelPars[55] * t14 + 1;
    real_type t18  = t17 * t17;
    real_type t22  = ModelPars[57];
    real_type t23  = ModelPars[40];
    real_type t24  = t23 * t22;
    real_type t27  = ModelPars[46] * t14 + 1;
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
    real_type t52  = ModelPars[48];
    real_type t55  = ModelPars[42] * t14 + 1;
    real_type t56  = 1.0 / t55;
    real_type t62  = atan(alpha__f__XO * t44 / t52 * t56 * t10);
    real_type t63  = t62 * t52;
    real_type t64  = sin(t63);
    real_type t69  = cos(t49);
    real_type t72  = t10 * t10;
    real_type t73  = t55 * t55;
    real_type t76  = t52 * t52;
    real_type t87  = cos(t63);
    real_type t93  = ModelPars[56];
    real_type t95  = atan(phi__f__XO * t93);
    real_type t101 = ModelPars[58];
    real_type t104 = ModelPars[54] * t14 + 1;
    real_type t107 = t23 / t104 * t101;
    real_type t108 = t101 * t101;
    real_type t109 = t104 * t104;
    real_type t119 = atan(alpha__f__XO * t44 * t28 * Fzf__XO * t107);
    real_type t121 = sin(t119 * t36);
    return t64 * t50 / (t31 * t30 + 1) * t28 * t24 / t18 * t12 / t10 * t2 * t1 - t87 / (t30 * t40 / t2 / t38 / t76 / t73 * t72 + 1) * t56 * t35 * t69 * Fzf__XO * t1 - t121 / (t30 / t109 * t108 + 1) * t45 * t107 * t35 * t12 / t93 * t95 * t2 * ModelPars[52];
  }

  real_type
  Baumgarte::Mzf_D_3_3( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const {
    real_type t1   = ModelPars[50];
    real_type t2   = Fzf__XO * Fzf__XO;
    real_type t3   = t2 * t1;
    real_type t5   = ModelPars[6];
    real_type t10  = t5 * ModelPars[34] + (Fzf__XO - t5) * ModelPars[36];
    real_type t11  = 1.0 / t10;
    real_type t13  = ModelPars[49];
    real_type t14  = phi__f__XO * phi__f__XO;
    real_type t17  = ModelPars[55] * t14 + 1;
    real_type t18  = t17 * t17;
    real_type t19  = 1.0 / t18;
    real_type t21  = ModelPars[57];
    real_type t22  = t21 * t21;
    real_type t26  = ModelPars[40];
    real_type t29  = ModelPars[46] * t14 + 1;
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
    real_type t55  = ModelPars[48];
    real_type t58  = ModelPars[42] * t14 + 1;
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
    real_type t143 = ModelPars[52];
    real_type t145 = ModelPars[56];
    real_type t147 = atan(phi__f__XO * t145);
    real_type t149 = 1.0 / t145;
    real_type t151 = ModelPars[58];
    real_type t152 = t151 * t151;
    real_type t159 = ModelPars[54] * t14 + 1;
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
  Baumgarte::Mzr( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t4   = ModelPars[7];
    real_type t9   = t4 * ModelPars[35] + (Fzr__XO - t4) * ModelPars[37];
    real_type t13  = phi__XO * phi__XO;
    real_type t17  = 1.0 / (ModelPars[55] * t13 + 1);
    real_type t18  = t17 * ModelPars[49];
    real_type t20  = ModelPars[41];
    real_type t25  = ModelPars[47] * t13 + 1;
    real_type t26  = 1.0 / t25;
    real_type t27  = t20 * t20;
    real_type t28  = Fzr__XO * Fzr__XO;
    real_type t30  = t25 * t25;
    real_type t33  = sqrt(1.0 / t30 * t28 * t27);
    real_type t34  = 1.0 / t33;
    real_type t38  = atan(alpha__r__XO * t34 * t26 * Fzr__XO * t20 * ModelPars[57]);
    real_type t40  = cos(t38 * t18);
    real_type t42  = ModelPars[48];
    real_type t52  = atan(alpha__r__XO * t34 / t42 / (ModelPars[43] * t13 + 1) * t9);
    real_type t54  = sin(t52 * t42);
    real_type t60  = ModelPars[56];
    real_type t62  = atan(phi__XO * t60);
    real_type t76  = atan(alpha__r__XO * t34 * t26 * Fzr__XO * t20 / (ModelPars[54] * t13 + 1) * ModelPars[58]);
    real_type t78  = cos(t76 * t18);
    return -t54 * t33 * t17 * t40 / t9 * Fzr__XO * ModelPars[51] + t78 / t60 * t62 * ModelPars[53] * Fzr__XO;
  }

  real_type
  Baumgarte::Mzr_D_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[51];
    real_type t3   = ModelPars[7];
    real_type t5   = ModelPars[37];
    real_type t8   = t3 * ModelPars[35] + (Fzr__XO - t3) * t5;
    real_type t9   = 1.0 / t8;
    real_type t11  = ModelPars[49];
    real_type t12  = phi__XO * phi__XO;
    real_type t15  = ModelPars[55] * t12 + 1;
    real_type t16  = 1.0 / t15;
    real_type t17  = t16 * t11;
    real_type t18  = ModelPars[57];
    real_type t19  = ModelPars[41];
    real_type t20  = t19 * t18;
    real_type t24  = ModelPars[47] * t12 + 1;
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
    real_type t42  = ModelPars[48];
    real_type t45  = ModelPars[43] * t12 + 1;
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
    real_type t132 = ModelPars[53];
    real_type t133 = ModelPars[56];
    real_type t135 = atan(phi__XO * t133);
    real_type t137 = 1.0 / t133;
    real_type t138 = ModelPars[58];
    real_type t141 = ModelPars[54] * t12 + 1;
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
  Baumgarte::Mzr_D_1_1( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[51];
    real_type t2   = Fzr__XO * t1;
    real_type t4   = ModelPars[7];
    real_type t6   = ModelPars[37];
    real_type t9   = t4 * ModelPars[35] + (Fzr__XO - t4) * t6;
    real_type t10  = 1.0 / t9;
    real_type t12  = ModelPars[49];
    real_type t13  = phi__XO * phi__XO;
    real_type t16  = ModelPars[55] * t13 + 1;
    real_type t17  = t16 * t16;
    real_type t18  = 1.0 / t17;
    real_type t19  = t18 * t12;
    real_type t20  = ModelPars[57];
    real_type t21  = ModelPars[41];
    real_type t22  = t21 * t20;
    real_type t25  = ModelPars[47] * t13 + 1;
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
    real_type t65  = ModelPars[48];
    real_type t68  = ModelPars[43] * t13 + 1;
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
    real_type t108 = ModelPars[53];
    real_type t110 = ModelPars[56];
    real_type t112 = atan(phi__XO * t110);
    real_type t113 = 1.0 / t110;
    real_type t115 = t113 * t112 * Fzr__XO * t108;
    real_type t116 = t12 * t12;
    real_type t118 = ModelPars[58];
    real_type t121 = ModelPars[54] * t13 + 1;
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
    real_type t152 = t28 * t1;
    real_type t153 = t10 * t152;
    real_type t156 = sin(t101);
    real_type t162 = cos(t61);
    real_type t163 = t56 * t162;
    real_type t172 = 1.0 / t84;
    real_type t176 = t55 * t48;
    real_type t184 = t56 * t162 * t172;
    real_type t186 = t65 * t33;
    real_type t187 = t82 * t186;
    real_type t192 = t162 * t10;
    real_type t194 = t56 * t192 * t2;
    real_type t195 = t97 * t97;
    real_type t196 = 1.0 / t195;
    real_type t205 = t28 * Fzr__XO;
    real_type t219 = 1.0 / t138;
    real_type t220 = sin(t147);
    real_type t230 = t48 * t48;
    real_type t231 = t54 * t54;
    real_type t241 = t27 * t27;
    real_type t242 = t241 * t21;
    real_type t244 = t30 * t30;
    real_type t246 = 1.0 / t244 / t25;
    real_type t248 = t28 * t28;
    real_type t250 = 1.0 / t244;
    real_type t253 = 1.0 / t33 / t250 / t248 / t241;
    real_type t263 = t10 * t1;
    real_type t266 = t156 * t33 * t62;
    real_type t270 = t163 * t263;
    real_type t271 = t156 * t34;
    real_type t286 = t33 * t56;
    real_type t287 = t6 * t6;
    real_type t319 = t82 * t82;
    return 2 * t103 * t83 * t33 * t63 * t49 * t10 * t2 - t148 / t139 * t132 * t18 * t116 * t115 + 2 * t31 * t27 * t156 * t34 * t63 * t49 * t153 - 2 * t31 * t27 * t102 * t98 * t83 * t34 * t163 * t153 - 2 * t6 * t156 * t33 * t62 * t176 * t18 * t12 * t172 * t2 + 2 * t6 * t103 * t187 * t184 * t2 + (2 * t6 * t51 * t30 * t92 * t91 * t89 * t86 * t9 - 2 * t94 / t205 * t91 * t90) * t102 * t196 * t187 * t194 - 2 * t220 * t219 * t131 * t56 * t12 * t113 * t112 * t108 + t156 * t33 * t162 / t231 * t230 / t17 / t16 * t116 * t10 * t2 - t220 * t219 * (3 * alpha__r__XO * t253 * t246 * t205 * t242 * t123 - 3 * alpha__r__XO * Fzr__XO * t45 * t126) * t57 * t115 + 2 * t266 * t176 * t19 * t263 - 3 * t31 * Fzr__XO * t27 * t271 * t270 - 2 * t102 * t98 * t82 * t186 * t270 - 2 * t287 * t156 * t286 * t162 / t84 / t9 * t2 + t250 * t241 * t156 * t44 * t56 * t192 * t205 * t1 - t102 * t98 * (3 * t250 * t28 * t241 * alpha__r__XO * t253 * t71 * t75 - t31 * t27 * t128 * t71 * t75 - 2 * t80 * t76 * t70) * t186 * t194 + t156 * t196 * t319 * t88 * t33 * t194 + 2 * t31 * t27 * t6 * t271 * t184 * t152 + t266 * t55 * (3 * alpha__r__XO * t253 * t246 * t205 * t242 * t20 - 3 * Fzr__XO * t128 * t42 * t39) * t18 * t12 * t10 * t2 + 2 * t6 * t156 * t286 * t162 * t172 * t1;
  }

  real_type
  Baumgarte::Mzr_D_1_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[51];
    real_type t2   = Fzr__XO * t1;
    real_type t4   = ModelPars[7];
    real_type t6   = ModelPars[37];
    real_type t9   = t4 * ModelPars[35] + (Fzr__XO - t4) * t6;
    real_type t10  = 1.0 / t9;
    real_type t11  = t10 * t2;
    real_type t12  = ModelPars[49];
    real_type t13  = phi__XO * phi__XO;
    real_type t14  = ModelPars[55];
    real_type t16  = t13 * t14 + 1;
    real_type t17  = t16 * t16;
    real_type t18  = 1.0 / t17;
    real_type t19  = t18 * t12;
    real_type t20  = ModelPars[57];
    real_type t21  = ModelPars[41];
    real_type t22  = t21 * t20;
    real_type t23  = ModelPars[47];
    real_type t25  = t13 * t23 + 1;
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
    real_type t48  = -t28 * t39 * t45 * alpha__r__XO + t22 * t36;
    real_type t51  = alpha__r__XO * alpha__r__XO;
    real_type t52  = t20 * t20;
    real_type t55  = 1.0 / (t51 * t52 + 1);
    real_type t56  = 1.0 / t16;
    real_type t57  = t56 * t12;
    real_type t60  = atan(t36 * Fzr__XO * t22);
    real_type t61  = t60 * t57;
    real_type t62  = sin(t61);
    real_type t64  = t33 * t62 * t55;
    real_type t65  = ModelPars[48];
    real_type t66  = ModelPars[43];
    real_type t68  = t13 * t66 + 1;
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
    real_type t88  = -2 * t71 * t72 * t76 + 2 * t82 * t85;
    real_type t89  = t88 * t65;
    real_type t90  = t9 * t9;
    real_type t91  = t70 * t90;
    real_type t92  = t65 * t65;
    real_type t93  = 1.0 / t92;
    real_type t95  = 1.0 / t27;
    real_type t96  = 1.0 / t28;
    real_type t101 = t30 * t51 * t91 * t93 * t95 * t96 + 1;
    real_type t102 = 1.0 / t101;
    real_type t104 = alpha__r__XO * t34 * t72;
    real_type t106 = atan(t104 * t79);
    real_type t107 = t106 * t65;
    real_type t108 = cos(t107);
    real_type t109 = t108 * t102;
    real_type t113 = t28 * t1;
    real_type t114 = cos(t61);
    real_type t115 = t114 * t10;
    real_type t118 = sin(t107);
    real_type t126 = t28 * t28;
    real_type t128 = t56 * t115;
    real_type t131 = t27 * t27;
    real_type t133 = t30 * t30;
    real_type t137 = phi__XO * t23 / t133 / t25;
    real_type t141 = ModelPars[53];
    real_type t142 = Fzr__XO * t141;
    real_type t143 = ModelPars[56];
    real_type t145 = atan(phi__XO * t143);
    real_type t146 = 1.0 / t143;
    real_type t147 = t146 * t145;
    real_type t149 = t12 * t147 * t142;
    real_type t150 = ModelPars[58];
    real_type t151 = ModelPars[54];
    real_type t153 = t13 * t151 + 1;
    real_type t155 = 1.0 / t153 * t150;
    real_type t156 = t21 * t155;
    real_type t160 = alpha__r__XO * t44;
    real_type t163 = -t155 * t160 * t28 * t38 * t42 + t156 * t36;
    real_type t165 = t150 * t150;
    real_type t166 = t153 * t153;
    real_type t167 = 1.0 / t166;
    real_type t170 = t165 * t167 * t51 + 1;
    real_type t171 = 1.0 / t170;
    real_type t176 = atan(t74 * t26 * Fzr__XO * t156);
    real_type t177 = t176 * t57;
    real_type t178 = sin(t177);
    real_type t184 = 1.0 / t90;
    real_type t185 = t114 * t184;
    real_type t188 = t65 * t33;
    real_type t195 = t18 * t12 * t10 * t2;
    real_type t196 = t55 * t48;
    real_type t201 = t31 * Fzr__XO;
    real_type t203 = phi__XO * t23;
    real_type t204 = t203 * t74;
    real_type t206 = t28 * Fzr__XO;
    real_type t207 = 1.0 / t133;
    real_type t216 = -2 * phi__XO * t14 * t60 * t19 + t55 * (2 * t160 * t203 * t206 * t207 * t39 - 2 * t201 * t204 * t22) * t57;
    real_type t222 = t128 * t2;
    real_type t224 = t78 * t6;
    real_type t230 = -t201 * t27 * t79 * t80 * alpha__r__XO + t104 * t224;
    real_type t232 = t101 * t101;
    real_type t233 = 1.0 / t232;
    real_type t238 = t10 * t1;
    real_type t239 = t56 * t114;
    real_type t240 = t34 * t239;
    real_type t242 = t27 * t118;
    real_type t252 = t95 * t93;
    real_type t271 = t62 * t216 * t10;
    real_type t276 = t108 * t102 * t230;
    real_type t279 = t206 * t1;
    real_type t280 = t10 * t279;
    real_type t285 = t203 * t42 * t27;
    real_type t298 = t163 * t56;
    real_type t301 = t170 * t170;
    real_type t308 = phi__XO * t151;
    real_type t313 = t18 * t114;
    real_type t315 = t118 * t33;
    real_type t320 = t143 * t143;
    real_type t323 = 1.0 / (t13 * t320 + 1);
    real_type t329 = t109 * t89 * t64 * t48 * t19 * t11 + 2 * phi__XO * t14 * t31 * t27 * t118 * t34 * t18 * t115 * t113 - 2 * t137 * t131 * t118 * t44 * t128 * t126 * t1 + 2 * phi__XO * t14 * t178 * t171 * t163 * t18 * t149 + t6 * t109 * t88 * t188 * t56 * t185 * t2 + t118 * t33 * t114 * t216 * t196 * t195 + t118 * t88 * t233 * t230 * t92 * t33 * t222 + 6 * t84 * t28 * t242 * t240 * t238 + (-4 * phi__XO * t66 * t51 * t30 * t96 * t252 / t69 / t68 * t90 + 4 * phi__XO * t23 * t51 * t25 * t96 * t252 * t91) * t108 * t233 * t230 * t188 * t222 + t276 * t65 * t33 * t56 * t271 * t2 + 2 * t285 * t276 * t65 * t34 * t239 * t280 - 2 * t285 * t118 * t34 * t62 * t196 * t19 * t280 - 4 * t308 * t51 / t166 / t153 * t165 * t178 / t301 * t298 * t12 * t146 * t145 * t142 + 2 * phi__XO * t14 * t315 * t313 * t238 - t178 * t171 * t298 * t12 * t323 * t142;
    real_type t331 = t167 * t150;
    real_type t351 = phi__XO * t151 * alpha__r__XO;
    real_type t355 = t131 * t21;
    real_type t359 = 1.0 / t133 / t30;
    real_type t363 = 1.0 / t33 / t207 / t126 / t131;
    real_type t366 = phi__XO * t23 * alpha__r__XO;
    real_type t377 = t56 * t62;
    real_type t432 = t44 * t207;
    real_type t464 = Fzr__XO * t27;
    real_type t489 = Fzr__XO * t21;
    real_type t505 = -2 * phi__XO * t14 * t176 * t19 + t171 * (2 * t155 * t206 * t366 * t38 * t432 - 2 * t155 * t31 * t34 * t366 * t489 - 2 * t331 * t35 * t351 * t489) * t57;
    real_type t507 = cos(t177);
    real_type t517 = -t178 * t171 * (8 * phi__XO * t155 * t160 * t207 * t23 * t28 * t38 - 6 * t126 * t155 * t355 * t359 * t363 * t366 - 2 * t21 * t26 * t308 * t331 * t74 + 2 * t28 * t331 * t351 * t38 * t45 - 2 * t155 * t204 * t21 * t31) * t57 * t147 * t142 - t6 * t315 * t377 * t216 * t184 * t2 - t108 * t102 * t88 * t188 * t239 * t238 + 2 * phi__XO * t14 * t108 * t102 * t230 * t65 * t33 * t313 * t11 - 2 * t84 * t27 * t6 * t118 * t240 * t184 * t279 - 4 * phi__XO * t14 * t118 * t64 * t48 / t17 / t16 * t12 * t11 - t31 * t27 * t108 * t102 * t89 * t240 * t10 * t113 + t315 * t377 * t216 * t238 - 2 * phi__XO * t14 * t6 * t315 * t18 * t185 * t2 + t118 * t33 * t62 * t55 * (-6 * t126 * t20 * t203 * t355 * t359 * t363 * alpha__r__XO + 8 * t203 * t28 * t39 * t432 * alpha__r__XO - 2 * t204 * t22 * t31) * t195 + t31 * t242 * t34 * t56 * t271 * t113 - t108 * t102 * (-6 * t131 * t137 * t206 * t363 * t72 * t79 * alpha__r__XO + 2 * phi__XO * t31 * t464 * t66 * t71 * t81 - 2 * t6 * t70 * t72 * t76 + 2 * t224 * t81 * t85 + 4 * t464 * t82 * t84) * t188 * t222 - t507 * t505 * t171 * t298 * t149 - t178 * t505 * t146 * t145 * t141 + t507 * t323 * t141;
    return t329 + t517;
  }

  real_type
  Baumgarte::Mzr_D_1_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[51];
    real_type t3   = ModelPars[7];
    real_type t5   = ModelPars[37];
    real_type t8   = t3 * ModelPars[35] + (Fzr__XO - t3) * t5;
    real_type t9   = 1.0 / t8;
    real_type t11  = ModelPars[49];
    real_type t12  = phi__XO * phi__XO;
    real_type t15  = ModelPars[55] * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t17  = 1.0 / t16;
    real_type t18  = t17 * t11;
    real_type t19  = ModelPars[57];
    real_type t20  = t19 * t18;
    real_type t22  = ModelPars[41];
    real_type t26  = ModelPars[47] * t12 + 1;
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
    real_type t53  = ModelPars[48];
    real_type t56  = ModelPars[43] * t12 + 1;
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
    real_type t216 = ModelPars[53];
    real_type t217 = ModelPars[56];
    real_type t219 = atan(phi__XO * t217);
    real_type t221 = 1.0 / t217;
    real_type t223 = ModelPars[58];
    real_type t228 = ModelPars[54] * t12 + 1;
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
  Baumgarte::Mzr_D_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[51];
    real_type t2   = Fzr__XO * t1;
    real_type t4   = ModelPars[7];
    real_type t9   = t4 * ModelPars[35] + (Fzr__XO - t4) * ModelPars[37];
    real_type t10  = 1.0 / t9;
    real_type t11  = ModelPars[49];
    real_type t12  = phi__XO * phi__XO;
    real_type t13  = ModelPars[55];
    real_type t15  = t13 * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t17  = 1.0 / t16;
    real_type t18  = t17 * t11;
    real_type t19  = ModelPars[57];
    real_type t20  = ModelPars[41];
    real_type t21  = t20 * t19;
    real_type t23  = ModelPars[47];
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
    real_type t78  = ModelPars[48];
    real_type t79  = ModelPars[43];
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
    real_type t150 = ModelPars[53] * Fzr__XO;
    real_type t151 = ModelPars[56];
    real_type t152 = t151 * t151;
    real_type t156 = ModelPars[58];
    real_type t157 = ModelPars[54];
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
  Baumgarte::Mzr_D_2_2( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t2   = ModelPars[53] * Fzr__XO;
    real_type t3   = phi__XO * phi__XO;
    real_type t4   = ModelPars[56];
    real_type t5   = t4 * t4;
    real_type t7   = t5 * t3 + 1;
    real_type t8   = t7 * t7;
    real_type t11  = ModelPars[49];
    real_type t12  = ModelPars[55];
    real_type t14  = t12 * t3 + 1;
    real_type t15  = 1.0 / t14;
    real_type t16  = t15 * t11;
    real_type t17  = ModelPars[58];
    real_type t18  = ModelPars[54];
    real_type t19  = t18 * t3;
    real_type t20  = t19 + 1;
    real_type t22  = 1.0 / t20 * t17;
    real_type t23  = ModelPars[41];
    real_type t25  = ModelPars[47];
    real_type t27  = t25 * t3 + 1;
    real_type t28  = 1.0 / t27;
    real_type t30  = t23 * t23;
    real_type t31  = Fzr__XO * Fzr__XO;
    real_type t32  = t31 * t30;
    real_type t33  = t27 * t27;
    real_type t34  = 1.0 / t33;
    real_type t35  = t34 * t32;
    real_type t36  = sqrt(t35);
    real_type t37  = 1.0 / t36;
    real_type t38  = alpha__r__XO * t37;
    real_type t41  = atan(t38 * t28 * Fzr__XO * t23 * t22);
    real_type t42  = t41 * t16;
    real_type t43  = cos(t42);
    real_type t49  = atan(phi__XO * t4);
    real_type t50  = t49 * t2;
    real_type t51  = 1.0 / t4;
    real_type t52  = t14 * t14;
    real_type t54  = 1.0 / t52 / t14;
    real_type t55  = t54 * t11;
    real_type t56  = t12 * t12;
    real_type t61  = 1.0 / t52;
    real_type t62  = t61 * t11;
    real_type t63  = t20 * t20;
    real_type t64  = 1.0 / t63;
    real_type t65  = t64 * t17;
    real_type t66  = Fzr__XO * t23;
    real_type t67  = t66 * t65;
    real_type t68  = t37 * t28;
    real_type t69  = t18 * alpha__r__XO;
    real_type t73  = t66 * t22;
    real_type t74  = t37 * t34;
    real_type t75  = t25 * alpha__r__XO;
    real_type t76  = phi__XO * t75;
    real_type t79  = t30 * t23;
    real_type t80  = t31 * Fzr__XO;
    real_type t81  = t80 * t79;
    real_type t82  = t81 * t22;
    real_type t83  = t33 * t33;
    real_type t84  = 1.0 / t83;
    real_type t86  = 1.0 / t36 / t35;
    real_type t87  = t86 * t84;
    real_type t91  = -2 * phi__XO * t69 * t68 * t67 - 2 * t76 * t74 * t73 + 2 * t76 * t87 * t82;
    real_type t93  = t17 * t17;
    real_type t95  = alpha__r__XO * alpha__r__XO;
    real_type t97  = t95 * t64 * t93 + 1;
    real_type t98  = 1.0 / t97;
    real_type t103 = t12 * t41;
    real_type t107 = 1.0 / t63 / t20;
    real_type t110 = t18 * t18;
    real_type t118 = t25 * t19;
    real_type t124 = alpha__r__XO * t86;
    real_type t131 = t33 * t27;
    real_type t132 = 1.0 / t131;
    real_type t134 = t25 * t25;
    real_type t136 = t3 * t134 * alpha__r__XO;
    real_type t141 = 1.0 / t83 / t27;
    real_type t146 = t75 * t74;
    real_type t149 = t30 * t30;
    real_type t150 = t149 * t23;
    real_type t151 = t31 * t31;
    real_type t152 = t151 * Fzr__XO;
    real_type t156 = 1.0 / t83 / t131;
    real_type t157 = t151 * t149;
    real_type t160 = 1.0 / t36 / t84 / t157;
    real_type t165 = t75 * t87;
    real_type t171 = t97 * t97;
    real_type t183 = sin(t42);
    real_type t186 = ModelPars[51];
    real_type t187 = t80 * t186;
    real_type t189 = ModelPars[7];
    real_type t194 = t189 * ModelPars[35] + (Fzr__XO - t189) * ModelPars[37];
    real_type t195 = 1.0 / t194;
    real_type t196 = t195 * t187;
    real_type t197 = ModelPars[57];
    real_type t198 = t23 * t197;
    real_type t199 = Fzr__XO * t198;
    real_type t202 = atan(alpha__r__XO * t68 * t199);
    real_type t203 = t12 * t202;
    real_type t209 = phi__XO * t25;
    real_type t212 = t79 * t197;
    real_type t218 = -2 * t209 * t38 * t34 * Fzr__XO * t198 + 2 * t209 * t124 * t84 * t80 * t212;
    real_type t219 = t197 * t197;
    real_type t222 = 1.0 / (t219 * t95 + 1);
    real_type t225 = -2 * phi__XO * t203 * t62 + t222 * t218 * t16;
    real_type t226 = t202 * t16;
    real_type t227 = sin(t226);
    real_type t231 = ModelPars[48];
    real_type t232 = ModelPars[43];
    real_type t233 = t232 * t3;
    real_type t234 = t233 + 1;
    real_type t236 = 1.0 / t234 * t194;
    real_type t237 = 1.0 / t231;
    real_type t241 = atan(alpha__r__XO * t37 * t237 * t236);
    real_type t242 = t241 * t231;
    real_type t243 = sin(t242);
    real_type t244 = t243 * t37;
    real_type t245 = t30 * t244;
    real_type t247 = phi__XO * t25 * t132;
    real_type t251 = cos(t226);
    real_type t252 = t61 * t251;
    real_type t255 = t12 * t243;
    real_type t257 = t132 * t30;
    real_type t258 = t25 * t257;
    real_type t262 = Fzr__XO * t186;
    real_type t266 = t234 * t234;
    real_type t267 = 1.0 / t266;
    real_type t268 = t267 * t194;
    real_type t269 = t237 * t268;
    real_type t273 = t86 * t237;
    real_type t274 = alpha__r__XO * t273;
    real_type t275 = t274 * t236;
    real_type t279 = -2 * phi__XO * t232 * t38 * t269 + 2 * t247 * t32 * t275;
    real_type t281 = t194 * t194;
    real_type t282 = t267 * t281;
    real_type t283 = t231 * t231;
    real_type t284 = 1.0 / t283;
    real_type t286 = 1.0 / t30;
    real_type t287 = 1.0 / t31;
    real_type t292 = t95 * t33 * t287 * t286 * t284 * t282 + 1;
    real_type t293 = 1.0 / t292;
    real_type t295 = cos(t242);
    real_type t301 = t251 * t195;
    real_type t302 = t15 * t301;
    real_type t303 = t302 * t262;
    real_type t304 = t231 * t36;
    real_type t306 = t292 * t292;
    real_type t307 = 1.0 / t306;
    real_type t310 = 1.0 / t266 / t234;
    real_type t312 = t286 * t284;
    real_type t332 = t227 * t225 * t195 * t262;
    real_type t336 = t295 * t293 * t279;
    real_type t340 = t302 * t187;
    real_type t351 = t3 * t134 / t83 / t33;
    real_type t356 = t3 * t134 * t84;
    real_type t366 = -2 * phi__XO * t103 * t62 + t98 * t91 * t16;
    real_type t384 = t3 * t134;
    real_type t412 = t243 * t36;
    real_type t415 = t15 * t251;
    real_type t423 = t225 * t225;
    real_type t428 = t301 * t262;
    real_type t429 = t36 * t61;
    real_type t441 = t232 * t232;
    real_type t449 = t25 * t132 * t31;
    real_type t476 = t279 * t279;
    real_type t485 = t366 * t366;
    return -2 * phi__XO * t5 * t43 / t8 * t2 - t183 * (8 * t3 * t56 * t41 * t55 - 4 * phi__XO * t12 * t98 * t91 * t62 - 2 * t103 * t62 + t98 * (8 * t3 * t110 * alpha__r__XO * t68 * t66 * t107 * t17 + 12 * t136 * t160 * t156 * t152 * t150 * t22 - 8 * t118 * t124 * t84 * t81 * t65 + 8 * t118 * t38 * t34 * t66 * t65 + 8 * t136 * t37 * t132 * t73 - 20 * t136 * t86 * t141 * t82 - 2 * t69 * t68 * t67 - 2 * t146 * t73 + 2 * t165 * t82) * t16 + 4 * phi__XO * t18 * t95 * t107 * t93 / t171 * t91 * t16) * t51 * t50 - 4 * t247 * t245 * t15 * t227 * t225 * t196 - 8 * t258 * t3 * t255 * t37 * t252 * t196 + 4 * phi__XO * t12 * t295 * t293 * t279 * t231 * t36 * t252 * t195 * t262 + (-4 * phi__XO * t232 * t95 * t33 * t287 * t312 * t310 * t281 + 4 * phi__XO * t25 * t95 * t27 * t287 * t312 * t282) * t295 * t307 * t279 * t304 * t303 + 2 * t336 * t231 * t36 * t15 * t332 + 2 * t258 * t244 * t340 + 4 * t351 * t149 * t243 * t86 * t302 * t152 * t186 - 12 * t356 * t245 * t340 - 2 * t183 * t366 / t7 * t2 + t412 * t15 * t227 * (8 * t3 * t56 * t202 * t55 - 4 * phi__XO * t12 * t222 * t218 * t62 - 2 * t203 * t62 + t222 * (12 * t384 * alpha__r__XO * t160 * t156 * t152 * t150 * t197 + 8 * t384 * t38 * t132 * Fzr__XO * t198 - 20 * t384 * t124 * t141 * t80 * t212 + 2 * t165 * t80 * t212 - 2 * t146 * t199) * t16) * t195 * t262 + 4 * t209 * t257 * t336 * t231 * t37 * t415 * t196 + t412 * t415 * t423 * t195 * t262 + 2 * t255 * t429 * t428 - 8 * t3 * t56 * t243 * t36 * t54 * t428 - t295 * t293 * (12 * t351 * t157 * alpha__r__XO * t160 * t237 * t236 + 8 * t3 * t441 * t38 * t237 * t310 * t194 - 8 * t449 * t30 * t233 * t274 * t268 + 2 * t449 * t30 * alpha__r__XO * t273 * t236 - 2 * t232 * t38 * t269 - 12 * t356 * t32 * t275) * t304 * t303 + t243 * t307 * t476 * t283 * t36 * t303 - 4 * phi__XO * t255 * t429 * t332 - t43 * t485 * t51 * t50;
  }

  real_type
  Baumgarte::Mzr_D_2_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[51];
    real_type t2   = Fzr__XO * t1;
    real_type t4   = ModelPars[7];
    real_type t9   = t4 * ModelPars[35] + (Fzr__XO - t4) * ModelPars[37];
    real_type t10  = 1.0 / t9;
    real_type t11  = ModelPars[49];
    real_type t12  = phi__XO * phi__XO;
    real_type t13  = ModelPars[55];
    real_type t15  = t13 * t12 + 1;
    real_type t16  = t15 * t15;
    real_type t17  = 1.0 / t16;
    real_type t18  = t17 * t11;
    real_type t19  = ModelPars[57];
    real_type t20  = ModelPars[41];
    real_type t21  = t20 * t19;
    real_type t22  = Fzr__XO * t21;
    real_type t24  = ModelPars[47];
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
    real_type t96  = ModelPars[48];
    real_type t97  = ModelPars[43];
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
    real_type t249 = ModelPars[53];
    real_type t250 = t29 * t249;
    real_type t251 = ModelPars[56];
    real_type t252 = t251 * t251;
    real_type t257 = ModelPars[58];
    real_type t260 = ModelPars[54];
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
  Baumgarte::Mzr_D_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[51];
    real_type t2   = Fzr__XO * Fzr__XO;
    real_type t5   = ModelPars[7];
    real_type t10  = t5 * ModelPars[35] + (Fzr__XO - t5) * ModelPars[37];
    real_type t12  = ModelPars[49];
    real_type t14  = phi__XO * phi__XO;
    real_type t17  = ModelPars[55] * t14 + 1;
    real_type t18  = t17 * t17;
    real_type t22  = ModelPars[57];
    real_type t23  = ModelPars[41];
    real_type t24  = t23 * t22;
    real_type t27  = ModelPars[47] * t14 + 1;
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
    real_type t52  = ModelPars[48];
    real_type t55  = ModelPars[43] * t14 + 1;
    real_type t56  = 1.0 / t55;
    real_type t62  = atan(alpha__r__XO * t44 / t52 * t56 * t10);
    real_type t63  = t62 * t52;
    real_type t64  = sin(t63);
    real_type t69  = cos(t49);
    real_type t72  = t10 * t10;
    real_type t73  = t55 * t55;
    real_type t76  = t52 * t52;
    real_type t87  = cos(t63);
    real_type t93  = ModelPars[56];
    real_type t95  = atan(phi__XO * t93);
    real_type t101 = ModelPars[58];
    real_type t104 = ModelPars[54] * t14 + 1;
    real_type t107 = t23 / t104 * t101;
    real_type t108 = t101 * t101;
    real_type t109 = t104 * t104;
    real_type t119 = atan(alpha__r__XO * t44 * t28 * Fzr__XO * t107);
    real_type t121 = sin(t119 * t36);
    return t64 * t50 / (t31 * t30 + 1) * t28 * t24 / t18 * t12 / t10 * t2 * t1 - t87 / (t30 * t40 / t2 / t38 / t76 / t73 * t72 + 1) * t56 * t35 * t69 * Fzr__XO * t1 - t121 / (t30 / t109 * t108 + 1) * t45 * t107 * t35 * t12 / t93 * t95 * t2 * ModelPars[53];
  }

  real_type
  Baumgarte::Mzr_D_3_3( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const {
    real_type t1   = ModelPars[51];
    real_type t2   = Fzr__XO * Fzr__XO;
    real_type t3   = t2 * t1;
    real_type t5   = ModelPars[7];
    real_type t10  = t5 * ModelPars[35] + (Fzr__XO - t5) * ModelPars[37];
    real_type t11  = 1.0 / t10;
    real_type t13  = ModelPars[49];
    real_type t14  = phi__XO * phi__XO;
    real_type t17  = ModelPars[55] * t14 + 1;
    real_type t18  = t17 * t17;
    real_type t19  = 1.0 / t18;
    real_type t21  = ModelPars[57];
    real_type t22  = t21 * t21;
    real_type t26  = ModelPars[41];
    real_type t29  = ModelPars[47] * t14 + 1;
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
    real_type t55  = ModelPars[48];
    real_type t58  = ModelPars[43] * t14 + 1;
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
    real_type t143 = ModelPars[53];
    real_type t145 = ModelPars[56];
    real_type t147 = atan(phi__XO * t145);
    real_type t149 = 1.0 / t145;
    real_type t151 = ModelPars[58];
    real_type t152 = t151 * t151;
    real_type t159 = ModelPars[54] * t14 + 1;
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
  Baumgarte::Mxf( real_type t__XO ) const {
    return 0;
  }

  real_type
  Baumgarte::Mxf_D( real_type t__XO ) const {
    return 0;
  }

  real_type
  Baumgarte::Mxf_DD( real_type t__XO ) const {
    return 0;
  }

  real_type
  Baumgarte::Mxr( real_type t__XO ) const {
    return 0;
  }

  real_type
  Baumgarte::Mxr_D( real_type t__XO ) const {
    return 0;
  }

  real_type
  Baumgarte::Mxr_DD( real_type t__XO ) const {
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
  Baumgarte::H_eval(
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
    real_type t2   = X__[15];
    real_type t3   = X__[4];
    real_type t5   = t3 + ModelPars[63];
    real_type t6   = cos(t5);
    real_type t8   = X__[3];
    real_type t9   = sin(t8);
    real_type t11  = sin(t5);
    real_type t13  = X__[11];
    real_type t14  = X__[27];
    real_type t15  = Fzf(t13, t14);
    real_type t16  = X__[16];
    real_type t17  = X__[2];
    real_type t18  = X__[0];
    real_type t19  = X__[1];
    real_type t20  = X__[9];
    real_type t21  = X__[10];
    real_type t22  = X__[25];
    real_type t23  = X__[26];
    real_type t24  = alpha__f(t17, t18, t19, t20, t21, t2, t22, t23);
    real_type t25  = X__[18];
    real_type t26  = lambda__f(t17, t16, t18, t19, t20, t21, t2, t25, t22, t23);
    real_type t27  = Fxf(t15, t16, t24, t26);
    real_type t32  = Fyf(t15, t16, t24, t26);
    real_type t34  = ModelPars[118];
    real_type t35  = X__[8];
    real_type t36  = -t34 + t35;
    real_type t37  = t17 * t17;
    real_type t38  = t37 * t36;
    real_type t39  = cos(t8);
    real_type t40  = t39 * t39;
    real_type t42  = X__[19];
    real_type t43  = ModelPars[23];
    real_type t44  = ModelPars[140];
    real_type t45  = t43 + t44;
    real_type t46  = t45 * t42;
    real_type t47  = t17 * t39;
    real_type t50  = t42 * t42;
    real_type t53  = ModelPars[74];
    real_type t55  = t6 * t6;
    real_type t69  = X__[5];
    real_type t70  = t37 * t69;
    real_type t72  = X__[6];
    real_type t73  = t72 * t44;
    real_type t76  = ModelPars[66];
    real_type t79  = t50 * t69;
    real_type t80  = t17 * t18;
    real_type t82  = X__[22];
    real_type t83  = t82 * t44;
    real_type t90  = t42 * t69;
    real_type t95  = X__[21];
    real_type t99  = t17 * t19;
    real_type t108 = X__[20];
    real_type t115 = t108 * t108;
    real_type t116 = t53 * t115;
    real_type t122 = X__[24];
    real_type t124 = -t27 * (t9 * t6 * t2 + t11) - t32 * (-t11 * t2 + t6 * t9) - t55 * t53 * (-t36 * t50 + t40 * t38 + 2 * t47 * t46) - t6 * (-2 * t11 * (-t40 * t37 * t45 / 2 + t47 * t36 * t42 + t45 * t50 / 2) * t53 - t15 * t39 + t53 * (t40 * t70 + t39 * (t9 * t37 * t73 + t76) - t70 - t79 - t9 * t80 + 2 * t42 * t83)) + 2 * t11 * (t39 * t17 * (t90 - t83) + t9 * t17 * (t42 * t73 + t95) - t99 / 2) * t53 + 2 * t53 * t47 * t46 - 2 * t9 * t17 * t53 * t36 * t108 + t53 * t38 - t35 * (-t116 + ModelPars[70]) - t34 * t116 - t122 * ModelPars[32];
    real_type t131 = X__[28];
    real_type t134 = X__[29];
    real_type t137 = X__[30];
    real_type t140 = X__[31];
    real_type t143 = X__[32];
    real_type t146 = X__[38];
    real_type t147 = cos(t146);
    real_type t149 = sin(t146);
    real_type t157 = X__[34];
    real_type t162 = X__[14];
    real_type t163 = Fzr(t162, t137);
    real_type t164 = X__[12];
    real_type t165 = X__[13];
    real_type t166 = alpha__r(t17, t18, t19, t164, t165, t131, t134);
    real_type t167 = X__[17];
    real_type t168 = lambda__r(t17, t8, t18, t165, t167, t131);
    real_type t169 = Fxr(t163, t8, t166, t168);
    real_type t170 = ModelPars[73];
    real_type t171 = ModelPars[24];
    real_type t172 = ModelPars[227];
    real_type t173 = -t171 + t172;
    real_type t174 = 2 * t173;
    real_type t176 = ModelPars[75];
    real_type t177 = t171 * t176;
    real_type t178 = 2 * t177;
    real_type t179 = t174 * t170 - t178;
    real_type t183 = -t174 * t170 + t178;
    real_type t184 = X__[23];
    real_type t185 = t184 * t183;
    real_type t187 = t17 * (t108 * t179 + t185);
    real_type t188 = X__[7];
    real_type t189 = cos(t188);
    real_type t191 = ModelPars[237];
    real_type t192 = t191 * t170;
    real_type t196 = 2 * t108 * t192 - 2 * t184 * t192;
    real_type t198 = sin(t188);
    real_type t200 = ModelPars[64];
    real_type t201 = cos(t200);
    real_type t203 = ModelPars[72];
    real_type t204 = ModelPars[67];
    real_type t205 = t204 * t203;
    real_type t206 = ModelPars[192];
    real_type t207 = sin(t206);
    real_type t208 = t207 * t205;
    real_type t209 = 2 * t208;
    real_type t210 = sin(t200);
    real_type t212 = t191 * t170 * t210;
    real_type t213 = 2 * t212;
    real_type t214 = ModelPars[25];
    real_type t215 = ModelPars[28];
    real_type t216 = t215 * t214;
    real_type t217 = 2 * t216;
    real_type t218 = t201 * t183 - t209 - t213 + t217;
    real_type t220 = t122 * t53;
    real_type t221 = 2 * t220;
    real_type t226 = t42 * t17;
    real_type t230 = -t173;
    real_type t231 = t230 * t170;
    real_type t232 = t231 + t177;
    real_type t233 = t37 * t232;
    real_type t234 = t115 * t232;
    real_type t235 = t184 * t179;
    real_type t236 = t108 * t235;
    real_type t237 = t184 * t184;
    real_type t238 = t237 * t232;
    real_type t239 = 2 * t39 * t226 * t192 + t233 + t234 + t236 + t238;
    real_type t241 = t42 * t183;
    real_type t243 = t37 * t192;
    real_type t244 = t237 * t192;
    real_type t245 = t108 * t184;
    real_type t247 = 2 * t245 * t192;
    real_type t248 = t115 * t192;
    real_type t252 = t191 * t170 * t201;
    real_type t253 = 2 * t252;
    real_type t255 = cos(t206);
    real_type t256 = t255 * t205;
    real_type t257 = 2 * t256;
    real_type t258 = 2 * t205;
    real_type t259 = ModelPars[183];
    real_type t260 = t53 * t259;
    real_type t261 = 2 * t260;
    real_type t262 = t53 * t35;
    real_type t263 = 2 * t262;
    real_type t264 = ModelPars[29];
    real_type t265 = t264 * t214;
    real_type t266 = 2 * t265;
    real_type t267 = t179 * t210 - t253 - t257 + t258 - t261 + t263 + t266;
    real_type t268 = t42 * t267;
    real_type t270 = t170 * t173;
    real_type t271 = t270 - t177;
    real_type t273 = t271 * t201 + t208 + t212 - t216;
    real_type t276 = t122 * t108;
    real_type t278 = 2 * t53 * t276;
    real_type t286 = t210 * t183 + t253 + t257 - t258 + t261 - t263 - t266;
    real_type t288 = t17 * t108 * t286;
    real_type t299 = t271 * t210 + t205 - t252 - t256 - t260 + t262 + t265;
    real_type t300 = t37 * t299;
    real_type t301 = t115 * t299;
    real_type t304 = t44 * t53;
    real_type t305 = ModelPars[173];
    real_type t306 = ModelPars[228];
    real_type t308 = t306 * t305 + t304;
    real_type t309 = 2 * t308;
    real_type t310 = t72 * t309;
    real_type t312 = t214 * t95;
    real_type t317 = t214 * t69;
    real_type t320 = -t309;
    real_type t321 = t82 * t320;
    real_type t325 = ModelPars[1];
    real_type t326 = t18 * t18;
    real_type t327 = t326 * t325;
    real_type t333 = -t20 * t2 + t21;
    real_type t340 = t11 * (t69 * t39 - t13);
    real_type t341 = t39 * t43;
    real_type t347 = -t21 * t2 - t20;
    real_type t349 = t2 * t13;
    real_type t356 = Mzf(t15, t16, t24);
    real_type t358 = t44 * t43;
    real_type t359 = t53 * t358;
    real_type t360 = t44 * t44;
    real_type t361 = t53 * t360;
    real_type t364 = t306 * (t43 + t306) * t305;
    real_type t365 = ModelPars[149];
    real_type t366 = ModelPars[150];
    real_type t367 = ModelPars[13];
    real_type t368 = ModelPars[17];
    real_type t369 = t359 + t361 + t364 - t365 + t366 - t367 + t368;
    real_type t370 = t37 * t369;
    real_type t375 = t53 * t34 * t44;
    real_type t378 = t305 * t306 * ModelPars[238];
    real_type t379 = ModelPars[147];
    real_type t380 = t53 * t35 * t44 - t375 + t378 + t379;
    real_type t381 = t380 * t42;
    real_type t404 = t69 * t72;
    real_type t405 = t42 * t308;
    real_type t406 = t405 * t404;
    real_type t413 = t72 * t308;
    real_type t416 = 2 * t367;
    real_type t417 = ModelPars[151];
    real_type t418 = t416 - t368 + t365 - t366 + t417;
    real_type t421 = Q__[0];
    real_type t422 = Mxf(t421);
    real_type t423 = t2 * t422;
    real_type t426 = t99 * t413;
    real_type t427 = t365 / 2;
    real_type t428 = t366 / 2;
    real_type t429 = t417 / 2;
    real_type t430 = t368 / 2;
    real_type t433 = t122 * t304;
    real_type t434 = t25 * t368;
    real_type t435 = t434 / 2;
    real_type t436 = t108 * (t359 + t361 + t364 - t427 + t428 + t429 + t430) + t433 - t435;
    real_type t441 = t69 * t9;
    real_type t444 = t37 * t40;
    real_type t446 = t69 * t413 * t444;
    real_type t449 = t76 * t413;
    real_type t475 = t308 * t17;
    real_type t476 = t17 * t9;
    real_type t477 = t69 * t476;
    real_type t481 = t108 * t369;
    real_type t491 = 2 * t433;
    real_type t500 = X__[36];
    real_type t501 = -t27 * (t6 * (t9 * t13 + t39 * t333) + (t340 + t341) * t2) - t32 * (t6 * (t39 * t347 - t9 * t349) + t340 + t341) + t356 * t39 * t6 - t55 * (-t369 * t50 + t40 * t370 - 2 * t47 * t381) * t72 - t6 * (-2 * t11 * (t37 * t380 * t40 / 2 + t47 * t369 * t42 - t380 * t50 / 2) * t72 - t15 * t20 * t9 - 2 * t39 * (-t9 * t380 * t17 / 2 + t406 + t380 * t108) * t17 + t9 * (-2 * t413 * t95 * t17 - t17 * t418 * t42 + t423) + t426 - 2 * t436 * t42) - t11 * (t15 * (t441 + t21) - t446 + t39 * (t37 * t369 * t9 - 2 * t436 * t17 - t449) + t9 * t413 * t80 + t413 * (t37 + t50) * t69 + 2 * t108 * t381 + t422) - t15 * t9 * t43 + 2 * t40 * t72 * t37 * (t361 + t359 / 2 + t306 * (t306 + t43 / 2) * t305 - t365 + t366 - t367 + t368) - t39 * (t477 + t18) * t475 - t9 * (-2 * t72 * (t481 + t433 - t435) * t17 + t76 * t308) - t72 * (t370 + t50 * t308 * t43 + t108 * (t481 + t491 - t434)) + 2 * t95 * t405 - t82 * ModelPars[146] + t500;
    real_type t503 = t124 * L__[8] + t23 * L__[26] + t14 * L__[27] + t131 * L__[28] + t134 * L__[29] + t137 * L__[30] + t140 * L__[31] + t143 * L__[32] + (t19 * t147 + t18 * t149) * L__[37] + (-U__[0] * ModelPars[152] - t157) * L__[33] + (t27 - t2 * t32 + t169 - t6 * (t9 * (t189 * t187 + t198 * t17 * t196 + t17 * (t108 * t218 + t221)) + t189 * t239 + t198 * (t47 * t241 - t243 - t244 + t247 - t248) + t47 * t268 + t37 * t273 + t115 * t273 - t278) - t11 * (t9 * (-t189 * t17 * t196 + t198 * t187 + t288) + t189 * (t47 * t42 * t179 + t243 + t244 - t247 + t248) + t198 * t239 + t47 * t42 * t218 + t300 + t301) - t9 * t17 * (t42 * t310 + 2 * t312) - t39 * t17 * (2 * t42 * t317 + t321) - t327 + t214 * t99) * L__[0] + t501 * L__[6];
    real_type t505 = t9 * t165;
    real_type t506 = t162 * t39;
    real_type t510 = Fyr(t163, t8, t166, t168);
    real_type t512 = Mzr(t163, t8, t166);
    real_type t513 = t512 * t9;
    real_type t515 = ModelPars[148];
    real_type t516 = t191 * t270 + t515;
    real_type t518 = t40 * t37 * t516;
    real_type t519 = t172 + t191 - t171;
    real_type t521 = t172 - t191 - t171;
    real_type t523 = t171 * t171;
    real_type t524 = t176 * t523;
    real_type t525 = ModelPars[15];
    real_type t526 = ModelPars[20];
    real_type t527 = -t521 * t519 * t170 - t524 + t525 - t526;
    real_type t529 = t39 * t42;
    real_type t530 = t529 * t17 * t527;
    real_type t531 = t50 * t516;
    real_type t532 = -t518 + t530 + t531;
    real_type t534 = t189 * t189;
    real_type t537 = t40 * t37 * t527;
    real_type t539 = t529 * t516 * t17;
    real_type t540 = 4 * t539;
    real_type t541 = t50 * t527;
    real_type t544 = t189 * (t537 + t540 - t541) * t198;
    real_type t562 = t37 * t271;
    real_type t563 = t562 * t69 * t40;
    real_type t569 = 2 * t170 * t226 * t191 * t69 - t271 * t76;
    real_type t571 = t95 * t192;
    real_type t572 = 2 * t571;
    real_type t573 = t18 * t271;
    real_type t576 = t9 * t17 * (t572 + t573);
    real_type t577 = t271 * t69;
    real_type t578 = t37 * t577;
    real_type t579 = t192 * t99;
    real_type t580 = t50 * t577;
    real_type t584 = t69 * t170;
    real_type t593 = t95 * t183;
    real_type t595 = t191 * t18 * t170;
    real_type t596 = t593 + t595;
    real_type t599 = t192 * t70;
    real_type t600 = t19 * t271;
    real_type t601 = t17 * t600;
    real_type t602 = t192 * t79;
    real_type t603 = t37 * t584 * t191 * t40 + t39 * (t170 * t191 * t76 + 2 * t226 * t577) - t9 * t596 * t17 - t599 - t601 - t602;
    real_type t615 = t163 * t164 * t39;
    real_type t619 = t191 * t191;
    real_type t622 = ModelPars[16];
    real_type t623 = ModelPars[18];
    real_type t624 = 2 * t619 * t170 + t525 - t526 + t622 + t623;
    real_type t627 = X__[33];
    real_type t629 = alpha__crw(t421);
    real_type t630 = sin(t629);
    real_type t633 = t627 * ModelPars[112];
    real_type t641 = -t169 * (t505 - t506 + t69) - t510 * t164 * t9 + t513 - t55 * (4 * t534 * t532 + 2 * t518 - 2 * t530 - 2 * t531 - 2 * t544) - t6 * (t11 * (t534 * (2 * t537 + 8 * t539 - 2 * t541) + 4 * t189 * t532 * t198 - t537 - t540 + t541) + t189 * (t39 * t569 - t563 + t576 + t578 - t579 + t580) - t603 * t198) - t11 * (t189 * t603 - (-t39 * t569 + t563 - t576 - t578 + t579 - t580) * t198) + t615 + 2 * t534 * t532 - t544 + t518 - t39 * t624 * t226 - t630 * t171 * t627 - t531 + t633 - (t184 * ModelPars[33] + ModelPars[71] * t188) * ModelPars[30];
    real_type t644 = t184 - t108;
    real_type t645 = t644 * t644;
    real_type t646 = t50 + t645;
    real_type t647 = t646 * t191;
    real_type t650 = t646 * t271;
    real_type t652 = t232 * t210;
    real_type t653 = t252 + t652 + t256 - t205 + t260 - t262 - t265;
    real_type t658 = t644 * t271;
    real_type t662 = t108 * t273;
    real_type t670 = t198 * t170;
    real_type t677 = t189 * t644;
    real_type t687 = t50 * t317;
    real_type t692 = -t308;
    real_type t693 = t72 * t692;
    real_type t694 = t50 * t693;
    real_type t696 = 2 * t42 * t312;
    real_type t703 = t27 * t13;
    real_type t704 = t32 * t13;
    real_type t711 = -t521 * t519 * t170 - t524 + t525 - t526;
    real_type t713 = t198 * t189;
    real_type t715 = t191 * t231 - t515;
    real_type t716 = t715 * t713;
    real_type t718 = t201 * t201;
    real_type t719 = -t711;
    real_type t720 = t719 * t718;
    real_type t722 = t715 * t210 * t201;
    real_type t723 = 2 * t722;
    real_type t724 = t255 * t255;
    real_type t725 = t204 * t204;
    real_type t726 = t725 * t203;
    real_type t727 = ModelPars[14];
    real_type t728 = ModelPars[19];
    real_type t729 = -t726 - t727 + t728;
    real_type t730 = t729 * t724;
    real_type t731 = ModelPars[145];
    real_type t732 = t203 * t731;
    real_type t734 = t255 * t204 * t732;
    real_type t735 = ModelPars[141];
    real_type t737 = t207 * t204;
    real_type t738 = t737 * t203 * t735;
    real_type t739 = t35 * t35;
    real_type t740 = t739 * t53;
    real_type t741 = t740 / 2;
    real_type t743 = t53 * t34 * t35;
    real_type t746 = (t34 - t259 / 2) * t260;
    real_type t747 = t731 * t205;
    real_type t748 = t215 * t215;
    real_type t749 = t264 * t264;
    real_type t750 = -t748 + t749;
    real_type t752 = t750 * t214 / 2;
    real_type t753 = ModelPars[10];
    real_type t754 = t753 / 2;
    real_type t755 = ModelPars[12];
    real_type t756 = t755 / 2;
    real_type t757 = t711 * t534 + 2 * t716 + t720 - t723 + t726 + t727 - t728 + t730 - t734 + t738 + t741 - t743 + t746 + t747 + t752 + t754 - t756;
    real_type t759 = t40 * t42;
    real_type t762 = t644 * t715;
    real_type t763 = t534 * t762;
    real_type t764 = 2 * t763;
    real_type t765 = t719 * t198;
    real_type t766 = t677 * t765;
    real_type t767 = -t715;
    real_type t768 = 2 * t767;
    real_type t769 = t768 * t718;
    real_type t771 = t210 * t719 * t201;
    real_type t772 = -t729;
    real_type t774 = t735 * t205;
    real_type t776 = (t772 * t207 + t774) * t255;
    real_type t777 = t737 * t732;
    real_type t779 = t53 * t45 * t35;
    real_type t780 = t45 * t53;
    real_type t781 = t259 * t780;
    real_type t782 = t215 * t265;
    real_type t783 = ModelPars[0];
    real_type t784 = t769 - t771 + t776 + t777 - t779 + t781 - t782 - t774 - t783;
    real_type t785 = t108 * t784;
    real_type t786 = t715 * t184;
    real_type t787 = -t36;
    real_type t789 = t122 * t53 * t787;
    real_type t791 = -t764 - t766 + t785 + t786 + t789 / 2;
    real_type t797 = (t17 - t42) * (t17 + t42);
    real_type t798 = -t768;
    real_type t800 = t719 * t189;
    real_type t801 = t198 * t800;
    real_type t802 = t798 * t534 + t769 - t771 - t774 + t776 + t777 - t779 + t781 - t782 - t783 + t801;
    real_type t807 = t644 * t719;
    real_type t808 = t534 * t807;
    real_type t810 = t715 * t198;
    real_type t811 = t677 * t810;
    real_type t814 = 2 * t718 * t711;
    real_type t815 = 4 * t722;
    real_type t816 = 2 * t772;
    real_type t817 = t724 * t816;
    real_type t818 = 2 * t734;
    real_type t819 = 2 * t738;
    real_type t820 = 2 * t743;
    real_type t821 = t259 * t34;
    real_type t823 = t259 * t259;
    real_type t825 = t53 * (-2 * t821 + t823);
    real_type t826 = 2 * t726;
    real_type t827 = 2 * t747;
    real_type t829 = -t750 * t214;
    real_type t830 = 2 * t727;
    real_type t831 = 2 * t728;
    real_type t832 = t814 + t815 + t817 + t818 - t819 - t740 + t820 + t825 - t826 - t827 + t829 - t830 + t831 - t753 + t755;
    real_type t833 = t108 * t832;
    real_type t834 = t184 * t719;
    real_type t836 = t53 * t45 * t122;
    real_type t837 = -2 * t808 + 4 * t811 + t833 + t834 + t836;
    real_type t857 = t189 * t42;
    real_type t858 = t191 * t584;
    real_type t861 = t232 * t69;
    real_type t862 = t198 * t42;
    real_type t865 = t653 * t69;
    real_type t866 = t42 * t865;
    real_type t868 = t44 * t780;
    real_type t869 = t306 * t306;
    real_type t870 = t305 * t869;
    real_type t872 = t305 * t306 * t43;
    real_type t873 = t868 + t870 + t872 - t367 + t368 - t365 + t366;
    real_type t874 = t72 * t873;
    real_type t877 = t122 * t72 * t304;
    real_type t878 = -t380;
    real_type t879 = t878 * t82;
    real_type t887 = -t69 * t232 * t184 + t108 * t861 - t571;
    real_type t888 = t887 / 2;
    real_type t891 = t108 * t191 * t584;
    real_type t893 = t184 * t191 * t584;
    real_type t894 = t232 * t95;
    real_type t895 = -t891 + t893 - t894;
    real_type t896 = t895 / 2;
    real_type t898 = t878 * t72;
    real_type t900 = t232 * t201;
    real_type t901 = t900 - t208 - t212 + t216;
    real_type t902 = t69 * t901;
    real_type t903 = t108 * t902;
    real_type t906 = t201 * t95 * t192;
    real_type t908 = t69 * t220;
    real_type t911 = (t652 + t256 + t260 - t262 - t265 - t205) * t95;
    real_type t918 = t184 + t42 - t108;
    real_type t919 = t184 - t42 - t108;
    real_type t922 = t861 * t919 * t918 + t37 * t861 - t579;
    real_type t924 = t232 * t19;
    real_type t925 = t17 * t924;
    real_type t929 = -t918 * t919 * t191 * t584 - t599 - t925;
    real_type t934 = t50 * t902;
    real_type t942 = 2 * t53 * t69 * t276;
    real_type t944 = ModelPars[144] * t327;
    real_type t950 = 2 * t571 * t862;
    real_type t952 = t72 * t873 * t37;
    real_type t954 = t901 * t95;
    real_type t956 = 2 * t42 * t954;
    real_type t957 = t108 * t873;
    real_type t963 = t232 * t76;
    real_type t966 = t192 * t76 * t198;
    real_type t970 = 2 * t17 * t72 * (t957 + t433 - t435);
    real_type t971 = t76 * t901;
    real_type t978 = t42 * t902;
    real_type t980 = t108 * t898;
    real_type t1002 = t50 * t865;
    real_type t1004 = t368 * t72 * t25;
    real_type t1010 = ModelPars[139] * t327;
    real_type t1014 = 2 * t571 * t857;
    real_type t1018 = t72 * t878 * t37;
    real_type t1024 = t72 * t878 * t115;
    real_type t1026 = t82 * (t367 - t430 + t427 - t428 + t429);
    real_type t1030 = t368 * t82 * t25;
    real_type t1035 = t198 * t963;
    real_type t1037 = 2 * t980 - 2 * t1026;
    real_type t1039 = t653 * t76;
    real_type t1048 = t69 * t69;
    real_type t1049 = t1048 * t214;
    real_type t1050 = t748 * t214;
    real_type t1051 = ModelPars[11];
    real_type t1052 = t720 - t723 - t1049 + t730 + t819 - t1050 + t726 + t727 - t728 + t753 - t1051;
    real_type t1055 = t72 * t95;
    real_type t1063 = t69 * t312;
    real_type t1080 = t214 * t17 * t19 * t69;
    real_type t1086 = t326 * t69 * t325;
    real_type t1089 = t534 * t42;
    real_type t1093 = t189 * t42 * t644;
    real_type t1096 = t719 / 2;
    real_type t1101 = t749 * t214;
    real_type t1108 = t718 * t1096 - t722 + t729 * t724 / 2 - t734 + t741 - t743 + t746 + t726 / 2 + t1101 / 2 + t753 / 4 + t1051 / 4 - t755 / 4 + t727 / 2 - t728 / 2 + t747;
    real_type t1110 = t230 * t230;
    real_type t1121 = t167 * t623;
    real_type t1134 = t703 - t2 * t704 + t169 * t162 - t55 * (4 * t759 * t757 * t17 + t39 * (-4 * t9 * t791 * t17 + 2 * t802 * t797) + 2 * t42 * t9 * t837) - t6 * (t11 * (-4 * t759 * t802 * t17 + t39 * (2 * t9 * t837 * t17 + 2 * t757 * t797) + 4 * t42 * t791 * t9) - 2 * t40 * (t108 * t874 - 2 * t858 * t857 - 2 * t862 * t861 + 2 * t866 + t877 - t879) * t17 + t39 * (-4 * t9 * (t189 * t888 + t198 * t896 + t42 * t898 - t903 / 2 + t906 / 2 - t908 / 2 + t911 / 2) * t17 + t189 * t922 + t198 * t929 - t901 * t70 + t17 * t653 * t19 + t934 - 2 * t42 * (t868 + t870 + t430 - t427 + t428 + t429 + t872) * t82 - t115 * t902 - t942 + t944) + t9 * (-2 * t857 * t894 + t950 - t952 + t50 * t874 + t956 - t108 * t72 * (t957 + t491 - t434)) + t189 * t963 - t966 + t970 - t971) - t11 * (-2 * t40 * (t873 * t82 + 2 * t857 * t861 - 2 * t858 * t862 - 2 * t978 + t980) * t17 + t39 * (4 * t9 * (t189 * t896 - t198 * t888 + t42 * t874 + t108 * t865 / 2 + t954 / 2) * t17 - t189 * t929 + t198 * t922 - t37 * t865 - t901 * t99 + t1002 + t42 * (t1004 - 2 * t879) - t115 * t865 + t1010) + t9 * (2 * t42 * t653 * t95 + 2 * t108 * t1026 + t50 * t898 - 2 * t862 * t894 - t1014 - t1018 - t1024 + t1030) + t192 * t76 * t189 + t1035 + t17 * t1037 - t1039) - t15 * t20 + t163 * t164 + 2 * t40 * t17 * (-t534 * t42 * t719 + 2 * t857 * t810 + t42 * t1052 + t308 * (t69 * t82 + t1055)) - t39 * (2 * t9 * (-t764 - t766 + 2 * t406 + t785 + t1063 + t786) * t17 - 2 * t534 * t715 * t797 - t801 * t797 + t37 * (t798 * t718 + t771 + (t729 * t207 - t774) * t255 + t779 - t777 - t781 + t782 + t774 + t783) - t1080 + t50 * t784 + 2 * t108 * t787 * t220 + t1086) - t9 * (2 * t1089 * t807 - 4 * t1093 * t810 - t426 + 4 * t42 * (t108 * t1108 + t184 * (-t1110 * t170 / 2 - t623 / 4 - t622 / 4 - t524 / 2 + t525 / 4 - t526 / 4) - t434 / 4 - t1121 / 4)) - t17 * (t42 * (t753 - t1051 + t755) + 2 * t308 * t1055) + t423;
    real_type t1138 = t72 * t20;
    real_type t1140 = t45 * t2;
    real_type t1146 = t72 * t13;
    real_type t1147 = t36 * t2;
    real_type t1151 = t72 * t36;
    real_type t1159 = t72 * t21;
    real_type t1170 = t11 * t39;
    real_type t1178 = t368 * t226;
    real_type t1179 = cos(t16);
    real_type t1180 = t1179 * t157;
    real_type t1184 = sin(t16);
    real_type t1196 = t9 * t72;
    real_type t1198 = -t45;
    real_type t1228 = t191 * t171 - t191 * t172;
    real_type t1229 = 8 * t1228;
    real_type t1231 = 8 * t515;
    real_type t1232 = t170 * t1229 - t1231;
    real_type t1236 = -t170 * t1229 + t1231;
    real_type t1238 = t108 * t1232 + t184 * t1236;
    real_type t1239 = t17 * t1238;
    real_type t1242 = t172 * t171;
    real_type t1244 = t172 * t172;
    real_type t1247 = 4 * t523 - 8 * t1242 + 4 * t1244 - 4 * t619;
    real_type t1249 = 4 * t524;
    real_type t1250 = 4 * t525;
    real_type t1251 = 4 * t526;
    real_type t1252 = t170 * t1247 + t1249 - t1250 + t1251;
    real_type t1256 = -t170 * t1247 - t1249 + t1250 - t1251;
    real_type t1264 = 4 * t772;
    real_type t1266 = 4 * t774;
    real_type t1269 = 4 * t1198;
    real_type t1272 = 4 * t777;
    real_type t1275 = t259 * t43 + t259 * t44;
    real_type t1276 = 4 * t1275;
    real_type t1278 = 4 * t782;
    real_type t1279 = 4 * t783;
    real_type t1282 = 4 * t1228;
    real_type t1284 = 4 * t515;
    real_type t1285 = t170 * t1282 - t1284;
    real_type t1286 = t184 * t1285;
    real_type t1287 = t34 * t220;
    real_type t1288 = 2 * t1287;
    real_type t1289 = t35 * t220;
    real_type t1290 = 2 * t1289;
    real_type t1295 = t42 * t1256;
    real_type t1296 = t534 * t17;
    real_type t1300 = t189 * t198 * t17;
    real_type t1305 = -t1264;
    real_type t1307 = 4 * t734;
    real_type t1308 = 2 * t740;
    real_type t1309 = 4 * t743;
    real_type t1310 = 4 * t738;
    real_type t1313 = 4 * t821 - 2 * t823;
    real_type t1316 = 4 * t747;
    real_type t1321 = 2 * t753;
    real_type t1323 = t201 * t210 * t1236 + t718 * t1252 + t724 * t1305 + t53 * t1313 + 2 * t750 * t214 - t1307 + t1308 - t1309 + t1310 + t1316 + t1321 + 4 * t726 + 4 * t727 - 4 * t728 - 2 * t755;
    real_type t1330 = t108 * t1256 + t184 * t1252;
    real_type t1331 = t42 * t1330;
    real_type t1336 = 2 * t523;
    real_type t1337 = 4 * t1242;
    real_type t1338 = 2 * t1244;
    real_type t1340 = -t1336 + t1337 - t1338 + 2 * t619;
    real_type t1342 = 2 * t524;
    real_type t1343 = 2 * t525;
    real_type t1344 = 2 * t526;
    real_type t1345 = t170 * t1340 - t1342 + t1343 - t1344;
    real_type t1346 = t184 * t1345;
    real_type t1347 = 2 * t1198;
    real_type t1350 = t122 * t53 * t1347 + t108 * t1323 + t1346;
    real_type t1356 = -t170 * t1282 + t1284;
    real_type t1357 = t50 * t1356;
    real_type t1364 = -t170 * t1340 + t1342 - t1343 + t1344;
    real_type t1369 = 2 * t774;
    real_type t1372 = -t1347;
    real_type t1375 = 2 * t777;
    real_type t1379 = 2 * t782;
    real_type t1380 = 2 * t783;
    real_type t1381 = t718 * t1285 + t201 * t210 * t1364 + (-t816 * t207 - t1369) * t255 + t35 * t53 * t1372 - t1375 - 2 * t53 * t1275 + t1379 + t1369 + t1380;
    real_type t1407 = t718 * t1232 + t201 * t210 * t1252 + (t1305 * t207 - t1266) * t255 - t35 * t53 * t1269 - t1272 - t53 * t1276 + t1278 + t1266 + t1279;
    real_type t1427 = t718 * t1345;
    real_type t1429 = t201 * t210 * t1285;
    real_type t1430 = t1427 + t1429 + t817 + t818 - t740 + t820 - t819 + t825 - t826 - t827 + t829 - t830 + t831 - t753 + t755;
    real_type t1440 = t17 * (t108 * t69 * t183 + t69 * t235 - t572);
    real_type t1442 = 2 * t891;
    real_type t1443 = 2 * t893;
    real_type t1444 = t95 * t179;
    real_type t1452 = t201 * t179 + t209 + t213 - t217;
    real_type t1455 = 2 * t908;
    real_type t1456 = 2 * t906;
    real_type t1460 = 2 * t255 * t95 * t205;
    real_type t1462 = -2 * t262 - 2 * t265 - 2 * t205 + 2 * t260;
    real_type t1468 = t189 * t226;
    real_type t1471 = 4 * t230;
    real_type t1473 = 4 * t177;
    real_type t1474 = t170 * t1471 + t1473;
    real_type t1476 = t198 * t226;
    real_type t1481 = -t170 * t1471 - t1473;
    real_type t1491 = -t358 - t360;
    real_type t1492 = 2 * t1491;
    real_type t1493 = t53 * t1492;
    real_type t1494 = 2 * t870;
    real_type t1495 = 2 * t872;
    real_type t1496 = 2 * t368;
    real_type t1497 = 2 * t365;
    real_type t1498 = 2 * t366;
    real_type t1499 = t1493 - t1494 - t1495 + t416 - t1496 + t1497 - t1498;
    real_type t1505 = 2 * t304 * t82 * t35;
    real_type t1507 = 2 * t375 - 2 * t378 - 2 * t379;
    real_type t1518 = t201 * t1444;
    real_type t1521 = 2 * t210 * t95 * t192;
    real_type t1523 = 2 * t208 - 2 * t216;
    real_type t1524 = t95 * t1523;
    real_type t1538 = t108 * t69;
    real_type t1541 = t115 * t861 + t1538 * t235 + t69 * t238 - t579 + t580;
    real_type t1544 = t192 * t69 * t237;
    real_type t1546 = 2 * t858 * t245;
    real_type t1548 = t192 * t69 * t115;
    real_type t1558 = -t265 - t205 + t260;
    real_type t1573 = t17 * (t108 * t69 * t179 + t69 * t185 + t572 + t573);
    real_type t1579 = 2 * t72 * t380;
    real_type t1583 = t18 * t232;
    real_type t1584 = -t572 + t1583;
    real_type t1589 = -t95 * t1462;
    real_type t1692 = t108 * t1356 + t1286;
    real_type t1704 = 2 * t1063;
    real_type t1706 = -2 * t1228;
    real_type t1708 = 2 * t515;
    real_type t1709 = t170 * t1706 + t1708;
    real_type t1720 = 2 * t1050;
    real_type t1722 = 2 * t1049 + t1427 + t1429 + t817 - t1310 + t1720 - t826 - t830 + t831 - t1321 + 2 * t1051;
    real_type t1740 = -t53 * t1313 - t1051 - 2 * t1101 + t1307 - t1308 + t1309 - t1316 + t1427 + t1429 - t753 + t755 + t817 - t826 - t830 + t831;
    real_type t1756 = t523 - 2 * t1242 + t1244 - t619;
    real_type t1780 = -t333 * t27 - t347 * t32 - t169 * t165 - t510 * t164 + t356 + t512 - t55 * (t40 * (t534 * t1239 + t713 * t17 * (t108 * t1252 + t184 * t1256) + t17 * (t108 * (t718 * t1236 + t201 * t210 * t1256 + t255 * (t207 * t1264 + t1266) + t35 * t53 * t1269 + t1272 + t53 * t1276 - t1278 - t1266 - t1279) + t1286 + t1288 - t1290)) + t39 * (t9 * (t1300 * t42 * t1232 + t17 * t42 * t1323 + t1296 * t1295) + t534 * t1331 + t713 * t42 * t1238 + t42 * t1350) + t9 * (t713 * t50 * t1345 + t534 * t1357 + t50 * t1381)) - t6 * (t11 * (t40 * (t534 * t17 * t1330 + t713 * t1239 + t17 * t1350) + t39 * (t9 * (t1296 * t42 * t1236 + t17 * t42 * t1407 + t1300 * t1295) + t534 * t42 * (t108 * t1236 + t184 * t1232) + t713 * t1331 + t42 * (t108 * t1407 + t184 * t1356 - t1288 + t1290)) + t9 * (t534 * t50 * t1364 + t713 * t1357 + t50 * t1430)) + t40 * (t189 * t1440 + t198 * t17 * (-t1442 + t1443 + t1444) + t17 * (t108 * t69 * t1452 + 4 * t42 * t878 * t72 + t95 * t1462 + t210 * t593 - t1455 + t1456 + t1460)) + t39 * (t9 * (4 * t858 * t1468 + t1476 * t69 * t1474 + t17 * (t42 * t69 * (t210 * t1481 + 4 * t205 - 4 * t252 - 4 * t256 - 4 * t260 + 4 * t262 + 4 * t265) + t108 * t72 * t1499 - 2 * t877 - t1505 + t82 * t1507)) + t857 * t593 - t950 + t50 * t72 * (t53 * t1491 + t365 - t366 + t367 - t368 - t870 - t872) + t42 * (t1518 + t1521 + t1524) + t115 * t72 * (-t53 * t1491 - t365 + t366 - t367 + t368 + t870 + t872) + t108 * t72 * (t491 - t434)) + t9 * (t189 * t1541 + t198 * (t601 - t1544 + t1546 + t602 - t1548) + t17 * (t192 * t201 * t19 + t205 * t255 * t19 - t53 * t35 * t19 + t19 * t1558 + t210 * t924) + t934 + t42 * t82 * (t1493 - t1494 - t1495 - t368 + t365 - t366 - t417) + t115 * t69 * t273 - t942 + t944) + t189 * t1573 + t198 * t17 * (t1442 - t1443 + t593 + t595) + t17 * (t42 * t1579 + t108 * t69 * t218 + t1455 + t201 * t1584 + t210 * (t1444 - t595) - t1460 + t1589 - t205 * t18 * t207 + t215 * t18 * t214)) - t11 * (t40 * (t189 * t17 * (t1442 - t1443 + t593) + t198 * t1440 + t17 * (t42 * t72 * (4 * t53 * t1491 + 4 * t365 - 4 * t366 + 4 * t367 - 4 * t368 - 4 * t870 - 4 * t872) + t108 * t69 * t267 + t1518 + t1521 + t1524)) + t39 * (t9 * (t1468 * t69 * t1481 + 4 * t858 * t1476 + t17 * (t42 * t69 * (t201 * t1474 - 4 * t208 - 4 * t212 + 4 * t216) + t108 * t1579 + t82 * t1499)) + t1014 + t862 * t593 + t50 * t72 * t380 + t42 * (t210 * t1444 - t1456 - t1460 + t1589) + t1024 - t108 * t82 * t418 - t1030) + t9 * (t189 * (t925 + t1544 - t1546 - t602 + t1548) + t198 * t1541 + t17 * (t192 * t210 * t19 + t205 * t207 * t19 - t215 * t19 * t214 + t201 * t600) + t1002 + t42 * (-t82 * t1507 + t1004 + t1505) + t115 * t69 * t299 + t1010) + t189 * t17 * (-t1442 + t1443 + t1444 - t595) + t198 * t1573 + t17 * (t42 * t72 * (-t53 * t1492 + t1494 + t1495 + t1496 - t1497 + t1498 - t416) + t108 * t69 * t286 + t201 * t596 + t210 * t1584 + t205 * t18 * t255 - t95 * t1523 - t53 * t18 * t35 + t18 * t1558)) - t40 * (t534 * t17 * t1692 + t713 * t17 * (t108 * t1345 + t184 * t1364) + t17 * (4 * t90 * t72 * t692 + t108 * t1381 + t184 * t1709 - t1704)) - t39 * (t9 * (t1296 * t42 * t1364 + t1300 * t42 * t1356 + t17 * (t72 * t95 * t320 + t42 * t1722 + t69 * t321)) + t534 * t42 * (t108 * t1364 + t1346) + t713 * t42 * t1692 + t426 + t42 * (t108 * t1740 + t184 * (t170 * (t1336 - t1337 + t1338) + t1342 + t623 - t525 + t622 + t526) + t434 + t1121)) - t9 * (t534 * t50 * (-t170 * t1706 - t1708) + t713 * t50 * (t170 * t1756 + t524 - t525 + t526) - t1080 + t50 * (t718 * t1709 + t201 * t210 * (-t170 * t1756 - t524 + t525 - t526) + t776 + t35 * t53 * t1198 + t777 + t53 * t1275 - t782 - t774 - t783) + t108 * (-2 * t1289 + 2 * t1287) + t1086) - t17 * (t90 * t310 - t1288 + t1290 + t1704);
    real_type t1784 = ModelPars[142];
    real_type t1786 = ModelPars[176];
    real_type t1787 = t108 * t1786;
    real_type t1789 = t1786 * t1786;
    real_type t1790 = t35 * t1789;
    real_type t1791 = t1786 * t1784;
    real_type t1793 = 2 * t122 * t1791;
    real_type t1794 = t1789 * t34;
    real_type t1806 = t115 * t1198 + t108 * (2 * t1786 * t1784 * t34 - 2 * t1786 * t1784 * t35 - 2 * t122) + t45 * t1789;
    real_type t1808 = ModelPars[114];
    real_type t1809 = ModelPars[116];
    real_type t1810 = t1808 - t1809;
    real_type t1811 = t143 * t143;
    real_type t1812 = t1811 * t1810;
    real_type t1813 = -t1810;
    real_type t1814 = t1789 * t1813;
    real_type t1817 = 2 * t1813;
    real_type t1818 = t1784 * t1817;
    real_type t1859 = X__[35];
    real_type t1865 = t1784 * t82;
    real_type t1866 = t1786 * t1865;
    real_type t1867 = 2 * t1866;
    real_type t1871 = t72 * t1791 + t82;
    real_type t1890 = t42 * t143;
    real_type t1896 = t143 - t42;
    real_type t1933 = t143 - t1786 - t42;
    real_type t1935 = t143 + t1786 - t42;
    real_type t1957 = ALIAS_maxTorque(t167);
    real_type t1963 = -t1896 * t1810;
    real_type t1969 = t1935 * t1933;
    real_type t1972 = t21 * t50;
    real_type t1974 = t1784 * t1809;
    real_type t1977 = -2 * t13 * t1791 - 2 * t1786 * t1974 - 2 * t14;
    real_type t1981 = 2 * t1786 * t1784 * t23;
    real_type t1982 = t21 * t1789;
    real_type t1991 = t50 * (t1809 + t13);
    real_type t1995 = t42 * (2 * t21 * t1791 + 2 * t23);
    real_type t1998 = 2 * t1786 * t1784 * t14;
    real_type t1999 = t1809 * t1789;
    real_type t2000 = t13 * t1789;
    real_type t2020 = t42 * t143 * t1817 + t50 * t1810 + t1812 + t1814;
    real_type t2024 = -2 * t1784 * t1808 + 2 * t1974;
    real_type t2025 = t1786 * t2024;
    real_type t2028 = -t1786 * t2024;
    real_type t2043 = t1789 * t69;
    real_type t2046 = 2 * t1786 * t1784 * t95;
    real_type t2049 = t641 * L__[7] + (-t6 * (t39 * (-t189 * t170 * t647 + t198 * t650 + (t50 + t115) * t653) + 2 * t42 * (t198 * t644 * t192 + t189 * t658 - t220 + t662) * t9) - t11 * (t39 * (-t189 * t650 - t670 * t647 + t50 * t273 + t108 * (t662 - t221)) - 2 * t42 * (t653 * t108 + t677 * t192 - t198 * t658) * t9) + t15 + t163 - t39 * (t42 * t82 * t309 - t687) - t9 * (t694 - t696) - t214 * t76) * L__[2] + t1134 * L__[4] + (-t32 * (t11 * (t39 * t1138 + t9 * t787 - t1140) + t6 * (t69 * t39 * t72 + t9 * t45 - t1146 - t1147) + t9 * t347 + t39 * (t1151 + t349) - t69 * t2) - t27 * (t11 * (t9 * (-t1146 - t1147) - t39 * t1159 + t43 + t44) + t6 * (t9 * t1140 - t34 + t35) + t9 * t333 - t39 * t13 + t69) - t356 * (t72 * t1170 - t9) - t11 * (t15 * (t9 * t1138 + t36 * t39) - t9 * (-t1178 + t1180) * t72 + t39 * (t157 * t72 * t1184 - t368 * t17 * t82) + t368 * t108 * t42 * t72) - t6 * (t15 * (t69 * t1196 + t39 * t1198 + t1159) - t17 * t368 * t39 * t108 * t72 - t42 * t82 * t368 + t72 * t422) - t15 * (t9 * t1151 + t39 * t20) + t1184 * t157 * t9 + (-t1178 + t1180 + t423) * t39) * L__[9] + t1780 * L__[5] + (-t11 * (t1787 * t1784 * t1347 + t36 * t115 - t1790 - t1793 + t1794) - t6 * t1806 - t1184 * (t2 * (t1812 + t1814) + t140 * t1786 * t1818) - t1179 * (t2 * t143 * t1786 * t1818 + t143 * t140 * t1817) + t1789 * t20 + 2 * t1786 * t1784 * t22) * L__[13] + (-t169 * (-t171 * t189 * t11 + t171 * t198 * t6 + t505 - t506 + t69) - t510 * (-t171 * t198 * t11 - t171 * t189 * t6 + t164) * t9 - t163 * t39 * t189 * t6 * t171 - t163 * t1170 * t198 * t171 - t39 * t623 * t226 + t615 + t513 + t633 + t1859) * L__[10] + (-t6 * (t72 * (t115 - t1789) - t1867) - 2 * t11 * t1871 * t108 - t39 * (t2 * (-t50 + t1789) + 2 * t1786 * t1784 * t140) + 2 * (t2 * t1791 + t140) * t42 * t9) * L__[11] + (-t11 * (t1179 * (t39 * (t72 * (t50 - 2 * t1890 + t115 + t1811 - t1789) - t1867) - 2 * t1871 * t1896 * t9) + 2 * (t39 * t1871 * t1896 - (t72 * (-t115 / 2 - t1811 / 2 + t1890 + t1789 / 2 - t50 / 2) + t1866) * t9) * t1184) + 2 * t6 * (t1179 * (t1896 * t1196 + t39 * t1871) + t1184 * (-t39 * t1896 * t72 + t1871 * t9)) * t108 - t1179 * (-2 * t39 * t1896 * t1791 - t1935 * t1933 * t9) - (-2 * t1896 * t1786 * t1784 * t9 + t39 * t1935 * t1933) * t1184) * L__[12] + (-U__[1] * ModelPars[153] - t1859) * L__[35] + (t1957 * U__[2] - t633) * L__[34] + (-t39 * (-2 * t1179 * t1963 * t1791 - t1184 * t1810 * t1969 + t42 * t1977 + t1972 - t1981 - t1982) - t9 * (t1179 * t1810 * t1969 - 2 * t1184 * t1963 * t1791 + t1991 + t1995 - t1998 - t1999 - t2000) - (t1786 * t72 + 2 * t1865) * t1786 * t44) * L__[14] + (-t6 * (t1787 * t1784 * t1372 + t787 * t115 + t1790 + t1793 - t1794) - t11 * t1806 - t39 * (t1179 * t2020 + t1184 * (t143 * t2028 + t42 * t2025) + t1991 + t1995 - t2000 - t1999 - t1998) - t9 * (t1179 * (t143 * t2025 + t42 * t2028) + t1184 * t2020 - t1972 - t42 * t1977 + t1982 + t1981) - t2043 - t2046) * L__[15];
    real_type t2051 = t19 * t149;
    real_type t2052 = t18 * t147;
    real_type t2054 = 1.0 / (t2051 - t2052);
    real_type t2055 = X__[37];
    real_type t2056 = Q__[1];
    real_type t2058 = t2056 * t2055 - 1;
    real_type t2059 = t2058 * t2054;
    real_type t2064 = roadLeftLateralBorder(t2055 / Q__[2] + 1);
    real_type t2070 = roadRightLateralBorder(1 - t2055 / Q__[3]);
    real_type t2075 = MaxRollAngle(t8 / ModelPars[182]);
    real_type t2078 = t108 - t1786 - t184;
    real_type t2079 = t108 + t1786 - t184;
    real_type t2082 = t1784 * t198;
    real_type t2083 = -t644;
    real_type t2105 = t165 * t50;
    real_type t2107 = ModelPars[117];
    real_type t2111 = -2 * t1786 * t1784 * t2107 - 2 * t162 * t1791 - 2 * t137;
    real_type t2114 = 2 * t134 * t1791;
    real_type t2115 = t165 * t1789;
    real_type t2118 = -t2107 - t162;
    real_type t2122 = -t165 * t1791 - t134;
    real_type t2139 = -t2079;
    real_type t2141 = -t2078;
    real_type t2188 = t2064 * t2059 + t2070 * t2059 + t2075 * t2059 + (-t6 * (-2 * t2083 * t1786 * t2082 + t189 * t2079 * t2078) * t171 - t11 * (2 * t189 * t2083 * t1791 + t2079 * t2078 * t198) * t171 - 2 * t1786 * t1784 * t131 - t1789 * t164) * L__[16] + (-t39 * (t42 * t2111 + t2105 - t2114 - t2115) + 2 * (t50 * t2118 / 2 + t42 * t2122 + (t1784 * t137 + t1786 * t2107 / 2 + t162 * t1786 / 2) * t1786) * t9) * L__[17] + (t6 * (t2141 * t2139 * t198 - 2 * t677 * t1791) * t171 - t11 * (2 * t644 * t1786 * t2082 + t189 * t2141 * t2139) * t171 - t39 * (-2 * t137 * t1791 - t162 * t1789 - t1789 * t2107 - t50 * t2118 - 2 * t42 * t2122) - t9 * (-t42 * t2111 - t2105 + t2114 + t2115) - t2046 - t2043 - t1789 * (-ModelPars[115] + t2107)) * L__[18] + t42 * L__[19] + t108 * L__[20] + t95 * L__[21] + t82 * L__[22] + t184 * L__[23] + t122 * L__[24] + t22 * L__[25];
    real_type t2192 = LatSlipFront(t24 / ModelPars[169]);
    real_type t2197 = LongSlipFront(t26 / ModelPars[171]);
    real_type t2202 = MaxSteerAngle(t72 / ModelPars[159]);
    real_type t2207 = LongSlipRear(t168 / ModelPars[172]);
    real_type t2212 = LatSlipRear(t166 / ModelPars[170]);
    real_type t2215 = 1.0 / ModelPars[9];
    real_type t2216 = ModelPars[8];
    real_type t2219 = RearWheelContact((t163 - t2216) * t2215);
    real_type t2224 = t50 * t192 + t243 + t244 - t247 + t248;
    real_type t2234 = t108 * t241 + t42 * t235;
    real_type t2241 = -2 * t108 * t42 * t192 + 2 * t42 * t184 * t192;
    real_type t2255 = -2 * t108 * t17 * t192 + 2 * t184 * t17 * t192;
    real_type t2312 = Mxr(t421);
    real_type t2322 = t184 * t69;
    real_type t2323 = t18 / 2;
    real_type t2324 = t477 + t2322 - t1538 + t2323;
    real_type t2331 = t2322 - t1538 + t2323;
    real_type t2346 = -t42 * t895;
    real_type t2353 = -t42 * t887;
    real_type t2367 = t476 + 2 * t184 - 2 * t108;
    real_type t2371 = t198 * t2367;
    real_type t2384 = t108 * (4 * t767 * t718 - 2 * t771 + (t816 * t207 + t1369) * t255 + t1375 - 2 * t779 + 2 * t781 - t1379 - t1369 - t1380);
    real_type t2385 = t184 * t798;
    real_type t2391 = t720 - t723 + t730 - t734 + t738 + t741 - t743 + t746 + t726 + t747 + t752 + t727 - t728 + t754 - t756;
    real_type t2436 = t11 * (-2 * t39 * (-t9 * t784 * t17 - 2 * t534 * t715 * t2367 - t800 * t2371 + t2384 + t2385 + t789) * t17 - 4 * t42 * (t808 - 2 * t811 + t108 * t2391 - t184 * t1096 - t836 / 2)) - 2 * t898 * t444 + 2 * t39 * (-t189 * t191 * t2324 * t170 + t9 * (t271 * t198 - t205 + t252 + t256 + t260 - t262 - t265 + t652) * t69 * t17 - t198 * t2331 * t232 - t653 * (t1538 - t2323)) * t17 + t189 * (-t9 * t76 * t192 + 2 * t2353) + t9 * (-t17 * t1037 - t1035 + t1039) + 2 * t198 * t2346 + t1018 + t1024 + t108 * (2 * t978 - 2 * t1026) + t42 * (-2 * t906 + 2 * t908 - 2 * t911) - t1030;
    real_type t2457 = t17 * t2367;
    real_type t2464 = t814 + t815 + t817 - t1310 + t1720 - t826 - t830 + t831 - t753 + t1051 + t755;
    real_type t2489 = t422 + t2312 - 2 * t446 + 2 * t1093 * t765 + 4 * t1089 * t762 + t404 * t308 * t37 - t11 * (2 * t874 * t444 + 2 * t39 * (t189 * t2324 * t232 - t441 * t17 * (t191 * t670 - t208 - t212 + t216 + t900) - t198 * t191 * t170 * t2331 + t903 - t201 * t1583 / 2 + t908 - (-t208 - t212 + t216) * t18 / 2) * t17 + t189 * (t9 * t963 - 2 * t2346) + t9 * (-t966 + t970 - t971) + 2 * t198 * t2353 - t952 - t72 * t873 * t115 + t108 * (2 * t866 - 2 * (t433 - t435) * t72) + t956) - t6 * t2436 - t55 * (-2 * t39 * (t9 * t17 * t2391 + 2 * t189 * t715 * t2371 - t534 * t719 * t2367 + t833 + t834 + t836) * t17 - 2 * t42 * (-4 * t763 - 2 * t766 + t2384 + t2385 + t789)) + t15 * t21 + t163 * t165 - t39 * (-t534 * t719 * t2457 + 2 * t716 * t2457 + t9 * t1052 * t37 + t17 * (-t214 * t18 * t69 + t108 * t2464 - t184 * t624 - t1121 - t434 + 2 * t836) + t449) - (-t18 * t72 * t475 - t214 * t76 * t69) * t9 - t42 * (2 * t1063 + 2 * t786) - t704 - t510 * t162 - t2 * t703 - 2 * t108 * t42 * t784;
    real_type t2493 = FrontWheelContact((t15 - t2216) * t2215);
    real_type t2499 = atan(t19 / t18);
    real_type t2501 = MaxBetaAngle(t2499 / ModelPars[155]);
    real_type t2514 = pow(t188 - ModelPars[65], 2);
    real_type t2519 = pow(t69 - ModelPars[68], 2);
    real_type t2524 = pow(t95 - ModelPars[164], 2);
    real_type t2529 = pow(t35 - ModelPars[185], 2);
    real_type t2534 = pow(t3 - ModelPars[194], 2);
    real_type t2539 = pow(t82 - ModelPars[157], 2);
    real_type t2544 = pow(t140 - ModelPars[243], 2);
    real_type t2549 = pow(t184 - ModelPars[163], 2);
    real_type t2554 = pow(t42 - ModelPars[179], 2);
    real_type t2559 = pow(t143 - ModelPars[180], 2);
    real_type t2564 = pow(t122 - ModelPars[184], 2);
    real_type t2569 = pow(t108 - ModelPars[193], 2);
    real_type t2574 = pow(t22 - ModelPars[229], 2);
    real_type t2579 = pow(t131 - ModelPars[231], 2);
    real_type t2584 = pow(t23 - ModelPars[233], 2);
    real_type t2589 = pow(t134 - ModelPars[235], 2);
    real_type t2594 = pow(t14 - ModelPars[239], 2);
    real_type t2599 = pow(t137 - ModelPars[241], 2);
    real_type t2602 = ModelPars[203] * t2514 + ModelPars[206] * t2519 + ModelPars[255] * t2524 + ModelPars[214] * t2529 + ModelPars[217] * t2534 + ModelPars[251] * t2539 + ModelPars[253] * t2544 + ModelPars[254] * t2549 + ModelPars[256] * t2554 + ModelPars[258] * t2559 + ModelPars[260] * t2564 + ModelPars[262] * t2569 + ModelPars[264] * t2574 + ModelPars[266] * t2579 + ModelPars[268] * t2584 + ModelPars[270] * t2589 + ModelPars[272] * t2594 + ModelPars[274] * t2599;
    real_type t2605 = t2055 * t2055;
    real_type t2611 = t2192 * t2059 + t2197 * t2059 + t2202 * t2059 + t2207 * t2059 + t2212 * t2059 + t2219 * t2059 + (t2 * t27 + t32 + t510 - t6 * (t9 * (t189 * t2224 + t198 * (t50 * t232 + t233 + t234 + t236 + t238) + t301 + t50 * t299 + t300) + t39 * (t108 * t42 * t1452 - 2 * t122 * t42 * t53 + t189 * t2234 + t198 * t2241) + t189 * t2255 + t198 * (t108 * t17 * t179 + t17 * t185) + t288) - t11 * (t9 * (t189 * (t108 * t185 + t115 * t271 + t237 * t271 + t50 * t271 + t562) + t198 * t2224 + t115 * t901 + t278 + t50 * t901 + t37 * t901) + t39 * (t108 * t268 - t189 * t2241 + t198 * t2234) + t189 * (t108 * t17 * t183 + t17 * t235) + t198 * t2255 + t108 * t17 * t1452 - 2 * t53 * t17 * t122) - t9 * (t214 * t70 + t42 * t321 + t687) - t39 * (t37 * t693 + t694 - t696) - t214 * t80) * L__[1] + (U__[3] * ModelPars[191] - t500) * L__[36] + t2489 * L__[3] + t2493 * t2059 + t2501 * t2059 + 1.0 / t2058 * (t2056 * (t17 * t2055 - t2051 + t2052) - t17) * L__[38] + t2058 * t2054 * (ModelPars[133] * t2602 + ModelPars[130] * t2605 + ModelPars[136]);
    return t503 + t2049 + t2188 + t2611;
  }

  /*\
   |   ___               _ _   _
   |  | _ \___ _ _  __ _| | |_(_)___ ___
   |  |  _/ -_) ' \/ _` | |  _| / -_|_-<
   |  |_| \___|_||_\__,_|_|\__|_\___/__/
  \*/

  real_type
  Baumgarte::penalties_eval(
    NodeType const     & NODE__,
    U_const_pointer_type U__,
    P_const_pointer_type P__
  ) const {
    integer     i_segment = NODE__.i_segment;
    real_type const * Q__ = NODE__.q;
    real_type const * X__ = NODE__.x;
    Road2D::SegmentClass const & segment = pRoad->getSegmentByIndex(i_segment);
    real_type t1   = X__[38];
    real_type t2   = sin(t1);
    real_type t3   = X__[1];
    real_type t5   = cos(t1);
    real_type t6   = X__[0];
    real_type t10  = X__[37];
    real_type t14  = (Q__[1] * t10 - 1) / (t3 * t2 - t6 * t5);
    real_type t16  = 1.0 / ModelPars[9];
    real_type t19  = Fzf(X__[11], X__[27]);
    real_type t20  = ModelPars[8];
    real_type t23  = FrontWheelContact((t19 - t20) * t16);
    real_type t27  = Fzr(X__[14], X__[30]);
    real_type t30  = RearWheelContact((t27 - t20) * t16);
    real_type t34  = X__[2];
    real_type t36  = X__[9];
    real_type t37  = X__[10];
    real_type t38  = X__[15];
    real_type t40  = X__[25];
    real_type t41  = X__[26];
    real_type t42  = lambda__f(t34, X__[16], t6, t3, t36, t37, t38, X__[18], t40, t41);
    real_type t44  = LongSlipFront(t42 / ModelPars[171]);
    real_type t48  = X__[3];
    real_type t49  = X__[13];
    real_type t51  = X__[28];
    real_type t52  = lambda__r(t34, t48, t6, t49, X__[17], t51);
    real_type t54  = LongSlipRear(t52 / ModelPars[172]);
    real_type t58  = alpha__f(t34, t6, t3, t36, t37, t38, t40, t41);
    real_type t60  = LatSlipFront(t58 / ModelPars[169]);
    real_type t66  = alpha__r(t34, t6, t3, X__[12], t49, t51, X__[29]);
    real_type t68  = LatSlipRear(t66 / ModelPars[170]);
    real_type t74  = atan(t3 / t6);
    real_type t76  = MaxBetaAngle(t74 / ModelPars[155]);
    real_type t82  = MaxSteerAngle(1.0 / ModelPars[159] * X__[6]);
    real_type t87  = MaxRollAngle(t48 / ModelPars[182]);
    real_type t93  = roadRightLateralBorder(1 - t10 / Q__[3]);
    real_type t99  = roadLeftLateralBorder(t10 / Q__[2] + 1);
    return t23 * t14 + t30 * t14 + t44 * t14 + t54 * t14 + t60 * t14 + t68 * t14 + t76 * t14 + t82 * t14 + t87 * t14 + t93 * t14 + t99 * t14;
  }

  real_type
  Baumgarte::control_penalties_eval(
    NodeType const     & NODE__,
    U_const_pointer_type U__,
    P_const_pointer_type P__
  ) const {
    integer     i_segment = NODE__.i_segment;
    real_type const * Q__ = NODE__.q;
    real_type const * X__ = NODE__.x;
    Road2D::SegmentClass const & segment = pRoad->getSegmentByIndex(i_segment);
    real_type t1   = X__[38];
    real_type t2   = sin(t1);
    real_type t5   = cos(t1);
    real_type t14  = (X__[37] * Q__[1] - 1) / (X__[1] * t2 - X__[0] * t5);
    real_type t17  = ModelPars[69];
    real_type t21  = t__oControl(U__[2], ModelPars[156] - t17, ModelPars[178] + t17);
    real_type t25  = ModelPars[174] - t17;
    real_type t27  = ModelPars[31] + t17;
    real_type t28  = b__f__oControl(U__[0], t25, t27);
    real_type t31  = b__r__oControl(U__[1], t25, t27);
    real_type t35  = -ModelPars[186] - t17;
    real_type t37  = tau__oControl(U__[3], t35, -t35);
    return t21 * t14 + t28 * t14 + t31 * t14 + t37 * t14;
  }

  /*\
   |   _
   |  | |   __ _ __ _ _ _ __ _ _ _  __ _ ___
   |  | |__/ _` / _` | '_/ _` | ' \/ _` / -_)
   |  |____\__,_\__, |_| \__,_|_||_\__, \___|
   |            |___/              |___/
  \*/

  real_type
  Baumgarte::lagrange_target(
    NodeType const     & NODE__,
    U_const_pointer_type U__,
    P_const_pointer_type P__
  ) const {
    integer     i_segment = NODE__.i_segment;
    real_type const * Q__ = NODE__.q;
    real_type const * X__ = NODE__.x;
    Road2D::SegmentClass const & segment = pRoad->getSegmentByIndex(i_segment);
    real_type t5   = pow(X__[7] - ModelPars[65], 2);
    real_type t11  = pow(X__[5] - ModelPars[68], 2);
    real_type t17  = pow(X__[21] - ModelPars[164], 2);
    real_type t23  = pow(X__[8] - ModelPars[185], 2);
    real_type t29  = pow(X__[4] - ModelPars[194], 2);
    real_type t35  = pow(X__[22] - ModelPars[157], 2);
    real_type t41  = pow(X__[31] - ModelPars[243], 2);
    real_type t47  = pow(X__[23] - ModelPars[163], 2);
    real_type t53  = pow(X__[19] - ModelPars[179], 2);
    real_type t59  = pow(X__[32] - ModelPars[180], 2);
    real_type t65  = pow(X__[24] - ModelPars[184], 2);
    real_type t71  = pow(X__[20] - ModelPars[193], 2);
    real_type t77  = pow(X__[25] - ModelPars[229], 2);
    real_type t83  = pow(X__[28] - ModelPars[231], 2);
    real_type t89  = pow(X__[26] - ModelPars[233], 2);
    real_type t95  = pow(X__[29] - ModelPars[235], 2);
    real_type t101 = pow(X__[27] - ModelPars[239], 2);
    real_type t107 = pow(X__[30] - ModelPars[241], 2);
    real_type t110 = ModelPars[203] * t5 + ModelPars[206] * t11 + ModelPars[255] * t17 + ModelPars[214] * t23 + ModelPars[217] * t29 + ModelPars[251] * t35 + ModelPars[253] * t41 + ModelPars[254] * t47 + ModelPars[256] * t53 + ModelPars[258] * t59 + ModelPars[260] * t65 + ModelPars[262] * t71 + ModelPars[264] * t77 + ModelPars[266] * t83 + ModelPars[268] * t89 + ModelPars[270] * t95 + ModelPars[272] * t101 + ModelPars[274] * t107;
    real_type t113 = X__[37];
    real_type t114 = t113 * t113;
    real_type t118 = X__[38];
    real_type t119 = sin(t118);
    real_type t122 = cos(t118);
    return (Q__[1] * t113 - 1) / (X__[1] * t119 - X__[0] * t122) * (ModelPars[133] * t110 + ModelPars[130] * t114 + ModelPars[136]);
  }

  /*\
   |   __  __
   |  |  \/  |__ _ _  _ ___ _ _
   |  | |\/| / _` | || / -_) '_|
   |  |_|  |_\__,_|\_, \___|_|
   |               |__/
  \*/

  real_type
  Baumgarte::mayer_target(
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
    real_type t1   = XL__[7];
    real_type t4   = pow(t1 - ModelPars[65], 2);
    real_type t5   = ModelPars[201];
    real_type t7   = XL__[8];
    real_type t10  = pow(t7 - ModelPars[185], 2);
    real_type t11  = ModelPars[213];
    real_type t13  = XL__[26];
    real_type t16  = pow(t13 - ModelPars[233], 2);
    real_type t17  = ModelPars[267];
    real_type t19  = XL__[27];
    real_type t22  = pow(t19 - ModelPars[239], 2);
    real_type t23  = ModelPars[271];
    real_type t25  = XL__[28];
    real_type t28  = pow(t25 - ModelPars[231], 2);
    real_type t29  = ModelPars[265];
    real_type t31  = XL__[29];
    real_type t34  = pow(t31 - ModelPars[235], 2);
    real_type t35  = ModelPars[269];
    real_type t37  = XL__[30];
    real_type t40  = pow(t37 - ModelPars[241], 2);
    real_type t41  = ModelPars[273];
    real_type t43  = XL__[31];
    real_type t46  = pow(t43 - ModelPars[243], 2);
    real_type t47  = ModelPars[252];
    real_type t49  = XL__[32];
    real_type t52  = pow(t49 - ModelPars[180], 2);
    real_type t53  = ModelPars[257];
    real_type t55  = XL__[33];
    real_type t58  = pow(t55 - ModelPars[4], 2);
    real_type t59  = ModelPars[195];
    real_type t61  = XL__[9];
    real_type t64  = pow(t61 - ModelPars[230], 2);
    real_type t65  = ModelPars[220];
    real_type t67  = XL__[10];
    real_type t70  = pow(t67 - ModelPars[234], 2);
    real_type t71  = ModelPars[223];
    real_type t73  = XL__[11];
    real_type t76  = pow(t73 - ModelPars[240], 2);
    real_type t77  = ModelPars[225];
    real_type t79  = XL__[12];
    real_type t82  = pow(t79 - ModelPars[232], 2);
    real_type t83  = ModelPars[221];
    real_type t85  = XL__[34];
    real_type t88  = pow(t85 - ModelPars[26], 2);
    real_type t89  = ModelPars[196];
    real_type t91  = XL__[35];
    real_type t94  = pow(t91 - ModelPars[27], 2);
    real_type t95  = ModelPars[197];
    real_type t97  = XL__[36];
    real_type t100 = pow(t97 - ModelPars[119], 2);
    real_type t101 = ModelPars[215];
    real_type t103 = XL__[37];
    real_type t106 = pow(t103 - ModelPars[76], 2);
    real_type t107 = ModelPars[207];
    real_type t109 = XL__[38];
    real_type t112 = pow(t109 - ModelPars[143], 2);
    real_type t113 = ModelPars[222];
    real_type t115 = t5 * t4 + t11 * t10 + t17 * t16 + t23 * t22 + t29 * t28 + t35 * t34 + t41 * t40 + t47 * t46 + t53 * t52 + t59 * t58 + t65 * t64 + t71 * t70 + t77 * t76 + t83 * t82 + t89 * t88 + t95 * t94 + t101 * t100 + t107 * t106 + t113 * t112;
    real_type t116 = XL__[13];
    real_type t119 = pow(t116 - ModelPars[236], 2);
    real_type t120 = ModelPars[224];
    real_type t122 = XL__[14];
    real_type t125 = pow(t122 - ModelPars[242], 2);
    real_type t126 = ModelPars[226];
    real_type t128 = XL__[15];
    real_type t131 = pow(t128 - ModelPars[158], 2);
    real_type t132 = ModelPars[200];
    real_type t134 = XL__[16];
    real_type t137 = pow(t134 - ModelPars[181], 2);
    real_type t138 = ModelPars[212];
    real_type t140 = XL__[17];
    real_type t143 = pow(t140 - ModelPars[177], 2);
    real_type t144 = ModelPars[209];
    real_type t146 = XL__[18];
    real_type t149 = pow(t146 - ModelPars[175], 2);
    real_type t150 = ModelPars[208];
    real_type t152 = XL__[19];
    real_type t155 = pow(t152 - ModelPars[179], 2);
    real_type t156 = ModelPars[211];
    real_type t158 = XL__[0];
    real_type t161 = pow(t158 - ModelPars[124], 2);
    real_type t162 = ModelPars[218];
    real_type t164 = XL__[1];
    real_type t167 = pow(t164 - ModelPars[125], 2);
    real_type t168 = ModelPars[219];
    real_type t170 = XL__[2];
    real_type t173 = pow(t170 - ModelPars[154], 2);
    real_type t174 = ModelPars[198];
    real_type t176 = XL__[3];
    real_type t179 = pow(t176 - ModelPars[89], 2);
    real_type t180 = ModelPars[210];
    real_type t182 = XL__[4];
    real_type t185 = pow(t182 - ModelPars[194], 2);
    real_type t186 = ModelPars[216];
    real_type t188 = XL__[5];
    real_type t191 = pow(t188 - ModelPars[68], 2);
    real_type t192 = ModelPars[204];
    real_type t194 = XL__[6];
    real_type t197 = pow(t194 - ModelPars[160], 2);
    real_type t198 = ModelPars[199];
    real_type t200 = XL__[20];
    real_type t203 = pow(t200 - ModelPars[193], 2);
    real_type t204 = ModelPars[261];
    real_type t206 = XL__[21];
    real_type t209 = pow(t206 - ModelPars[164], 2);
    real_type t210 = ModelPars[205];
    real_type t212 = XL__[22];
    real_type t215 = pow(t212 - ModelPars[157], 2);
    real_type t216 = ModelPars[250];
    real_type t218 = XL__[23];
    real_type t221 = pow(t218 - ModelPars[163], 2);
    real_type t222 = ModelPars[202];
    real_type t224 = XL__[24];
    real_type t227 = pow(t224 - ModelPars[184], 2);
    real_type t228 = ModelPars[259];
    real_type t230 = XL__[25];
    real_type t233 = pow(t230 - ModelPars[229], 2);
    real_type t234 = ModelPars[263];
    real_type t236 = t120 * t119 + t126 * t125 + t132 * t131 + t138 * t137 + t144 * t143 + t150 * t149 + t156 * t155 + t162 * t161 + t168 * t167 + t174 * t173 + t180 * t179 + t186 * t185 + t192 * t191 + t198 * t197 + t204 * t203 + t210 * t209 + t216 * t215 + t222 * t221 + t228 * t227 + t234 * t233;
    real_type t240 = XR__[19];
    real_type t241 = t240 * t240;
    real_type t243 = XR__[20];
    real_type t244 = t243 * t243;
    real_type t246 = XR__[21];
    real_type t247 = t246 * t246;
    real_type t249 = XR__[22];
    real_type t250 = t249 * t249;
    real_type t252 = XR__[23];
    real_type t253 = t252 * t252;
    real_type t255 = XR__[24];
    real_type t256 = t255 * t255;
    real_type t258 = XR__[25];
    real_type t259 = t258 * t258;
    real_type t261 = XR__[26];
    real_type t262 = t261 * t261;
    real_type t264 = XR__[27];
    real_type t265 = t264 * t264;
    real_type t267 = XR__[28];
    real_type t268 = t267 * t267;
    real_type t270 = XR__[29];
    real_type t271 = t270 * t270;
    real_type t273 = XR__[36];
    real_type t274 = t273 * t273;
    real_type t276 = XR__[37];
    real_type t277 = t276 * t276;
    real_type t279 = XR__[38];
    real_type t280 = t279 * t279;
    real_type t282 = t274 * t101 + t277 * t107 + t280 * t113 + t241 * t156 + t262 * t17 + t244 * t204 + t247 * t210 + t250 * t216 + t253 * t222 + t256 * t228 + t265 * t23 + t259 * t234 + t268 * t29 + t271 * t35;
    real_type t283 = t158 * t158;
    real_type t285 = t164 * t164;
    real_type t287 = t170 * t170;
    real_type t289 = t176 * t176;
    real_type t291 = t79 * t79;
    real_type t293 = t116 * t116;
    real_type t295 = t122 * t122;
    real_type t297 = t128 * t128;
    real_type t299 = t134 * t134;
    real_type t301 = t140 * t140;
    real_type t303 = t146 * t146;
    real_type t305 = XR__[30];
    real_type t306 = t305 * t305;
    real_type t308 = XR__[31];
    real_type t309 = t308 * t308;
    real_type t311 = XR__[32];
    real_type t312 = t311 * t311;
    real_type t314 = t293 * t120 + t295 * t126 + t297 * t132 + t299 * t138 + t301 * t144 + t303 * t150 + t283 * t162 + t285 * t168 + t287 * t174 + t289 * t180 + t291 * t83 + t306 * t41 + t309 * t47 + t312 * t53;
    real_type t316 = t182 * t182;
    real_type t318 = t188 * t188;
    real_type t320 = t194 * t194;
    real_type t322 = t1 * t1;
    real_type t331 = XR__[34];
    real_type t335 = XR__[35];
    real_type t342 = XR__[33];
    real_type t357 = t316 * t186 + t318 * t192 + t320 * t198 + t322 * t5 - 2 * t276 * t103 * t107 - 2 * t279 * t109 * t113 - 2 * t331 * t85 * t89 - 2 * t335 * t91 * t95 - 2 * t273 * t97 * t101 - 2 * t342 * t55 * t59 - 2 * t270 * t31 * t35 - 2 * t305 * t37 * t41 - 2 * t308 * t43 * t47 - 2 * t311 * t49 * t53;
    real_type t373 = XR__[16];
    real_type t376 = XR__[17];
    real_type t379 = XR__[18];
    real_type t388 = XR__[15];
    real_type t391 = XR__[0];
    real_type t393 = -t267 * t25 * t29 - t261 * t13 * t17 - t264 * t19 * t23 - t255 * t224 * t228 - t258 * t230 * t234 - t249 * t212 * t216 - t252 * t218 * t222 - t373 * t134 * t138 - t376 * t140 * t144 - t379 * t146 * t150 - t240 * t152 * t156 - t243 * t200 * t204 - t246 * t206 * t210 - t388 * t128 * t132 - t391 * t158 * t162;
    real_type t398 = XR__[1];
    real_type t402 = XR__[2];
    real_type t406 = XR__[3];
    real_type t410 = XR__[4];
    real_type t414 = XR__[5];
    real_type t418 = XR__[6];
    real_type t422 = XR__[7];
    real_type t426 = XR__[8];
    real_type t430 = XR__[9];
    real_type t434 = XR__[11];
    real_type t438 = XR__[12];
    real_type t442 = XR__[13];
    real_type t446 = XR__[14];
    real_type t449 = t152 * t152;
    real_type t451 = -2 * t398 * t164 * t168 - 2 * t402 * t170 * t174 - 2 * t406 * t176 * t180 - 2 * t410 * t182 * t186 - 2 * t414 * t188 * t192 - 2 * t418 * t194 * t198 - 2 * t422 * t1 * t5 - 2 * t426 * t7 * t11 - 2 * t430 * t61 * t65 - 2 * t434 * t73 * t77 - 2 * t438 * t79 * t83 - 2 * t442 * t116 * t120 - 2 * t446 * t122 * t126 + t449 * t156;
    real_type t452 = t7 * t7;
    real_type t454 = t61 * t61;
    real_type t456 = t200 * t200;
    real_type t458 = t206 * t206;
    real_type t460 = t212 * t212;
    real_type t462 = t218 * t218;
    real_type t464 = t224 * t224;
    real_type t466 = t230 * t230;
    real_type t468 = t13 * t13;
    real_type t470 = t19 * t19;
    real_type t472 = t25 * t25;
    real_type t474 = t31 * t31;
    real_type t476 = t37 * t37;
    real_type t478 = t43 * t43;
    real_type t480 = t49 * t49;
    real_type t482 = t452 * t11 + t468 * t17 + t456 * t204 + t458 * t210 + t460 * t216 + t462 * t222 + t464 * t228 + t470 * t23 + t466 * t234 + t472 * t29 + t474 * t35 + t476 * t41 + t454 * t65 + t478 * t47 + t480 * t53;
    real_type t484 = t73 * t73;
    real_type t486 = t55 * t55;
    real_type t488 = t85 * t85;
    real_type t490 = t91 * t91;
    real_type t492 = t97 * t97;
    real_type t494 = t103 * t103;
    real_type t496 = t109 * t109;
    real_type t498 = t391 * t391;
    real_type t500 = t398 * t398;
    real_type t502 = t402 * t402;
    real_type t504 = t406 * t406;
    real_type t506 = t410 * t410;
    real_type t508 = t414 * t414;
    real_type t510 = t418 * t418;
    real_type t512 = t492 * t101 + t494 * t107 + t496 * t113 + t498 * t162 + t500 * t168 + t502 * t174 + t504 * t180 + t506 * t186 + t508 * t192 + t510 * t198 + t484 * t77 + t486 * t59 + t488 * t89 + t490 * t95;
    real_type t513 = t422 * t422;
    real_type t515 = t426 * t426;
    real_type t517 = t430 * t430;
    real_type t521 = pow(t67 - XR__[10], 2);
    real_type t523 = t434 * t434;
    real_type t525 = t438 * t438;
    real_type t527 = t442 * t442;
    real_type t529 = t446 * t446;
    real_type t531 = t388 * t388;
    real_type t533 = t373 * t373;
    real_type t535 = t376 * t376;
    real_type t537 = t379 * t379;
    real_type t539 = t342 * t342;
    real_type t541 = t331 * t331;
    real_type t543 = t335 * t335;
    real_type t545 = t515 * t11 + t527 * t120 + t529 * t126 + t531 * t132 + t533 * t138 + t535 * t144 + t537 * t150 + t513 * t5 + t517 * t65 + t71 * t521 + t523 * t77 + t525 * t83 + t539 * t59 + t541 * t89 + t543 * t95;
    return ModelPars[127] * (t115 + t236) + ModelPars[126] * (t282 + t314 + t357 + 2 * t393 + t451 + t482 + t512 + t545);
  }

  /*\
   |    ___
   |   / _ \
   |  | (_) |
   |   \__\_\
  \*/

  integer
  Baumgarte::q_numEqns() const
  { return 13; }

  void
  Baumgarte::q_eval(
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
  Baumgarte::u_guess_numEqns() const
  { return 4; }

  void
  Baumgarte::u_guess_eval(
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
  Baumgarte::u_guess_eval(
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
  Baumgarte::u_check_if_admissible(
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
    real_type t3   = ModelPars[69];
    ok = ok && t__oControl.check_range(U__[2], ModelPars[156] - t3, ModelPars[178] + t3);
    real_type t9   = ModelPars[174] - t3;
    real_type t11  = ModelPars[31] + t3;
    ok = ok && b__f__oControl.check_range(U__[0], t9, t11);
    ok = ok && b__r__oControl.check_range(U__[1], t9, t11);
    real_type t15  = -ModelPars[186] - t3;
    ok = ok && tau__oControl.check_range(U__[3], t15, -t15);
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
  Baumgarte::post_numEqns() const
  { return 52; }

  void
  Baumgarte::post_eval(
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
    real_type t3   = ModelPars[69];
    result__[ 0   ] = t__oControl(U__[2], ModelPars[156] - t3, ModelPars[178] + t3);
    real_type t9   = ModelPars[174] - t3;
    real_type t11  = ModelPars[31] + t3;
    result__[ 1   ] = b__f__oControl(U__[0], t9, t11);
    result__[ 2   ] = b__r__oControl(U__[1], t9, t11);
    real_type t15  = -ModelPars[186] - t3;
    result__[ 3   ] = tau__oControl(U__[3], t15, -t15);
    real_type t18  = 1.0 / ModelPars[9];
    real_type t19  = X__[11];
    real_type t20  = X__[27];
    real_type t21  = Fzf(t19, t20);
    real_type t22  = ModelPars[8];
    result__[ 4   ] = FrontWheelContact((t21 - t22) * t18);
    real_type t25  = X__[14];
    real_type t26  = X__[30];
    real_type t27  = Fzr(t25, t26);
    result__[ 5   ] = RearWheelContact((t27 - t22) * t18);
    real_type t32  = X__[2];
    real_type t33  = X__[16];
    real_type t34  = X__[0];
    real_type t35  = X__[1];
    real_type t36  = X__[9];
    real_type t37  = X__[10];
    real_type t38  = X__[15];
    real_type t40  = X__[25];
    real_type t41  = X__[26];
    real_type t42  = lambda__f(t32, t33, t34, t35, t36, t37, t38, X__[18], t40, t41);
    result__[ 6   ] = LongSlipFront(t42 / ModelPars[171]);
    real_type t46  = X__[3];
    real_type t47  = X__[13];
    real_type t48  = X__[17];
    real_type t49  = X__[28];
    real_type t50  = lambda__r(t32, t46, t34, t47, t48, t49);
    result__[ 7   ] = LongSlipRear(t50 / ModelPars[172]);
    real_type t54  = alpha__f(t32, t34, t35, t36, t37, t38, t40, t41);
    result__[ 8   ] = LatSlipFront(t54 / ModelPars[169]);
    real_type t58  = X__[12];
    real_type t59  = X__[29];
    real_type t60  = alpha__r(t32, t34, t35, t58, t47, t49, t59);
    result__[ 9   ] = LatSlipRear(t60 / ModelPars[170]);
    real_type t66  = atan(t35 / t34);
    result__[ 10  ] = MaxBetaAngle(t66 / ModelPars[155]);
    real_type t70  = X__[6];
    result__[ 11  ] = MaxSteerAngle(t70 / ModelPars[159]);
    result__[ 12  ] = MaxRollAngle(t46 / ModelPars[182]);
    real_type t77  = X__[37];
    result__[ 13  ] = roadRightLateralBorder(1 - t77 / Q__[3]);
    result__[ 14  ] = roadLeftLateralBorder(t77 / Q__[2] + 1);
    result__[ 15  ] = t21;
    result__[ 16  ] = t27;
    result__[ 17  ] = t50;
    result__[ 18  ] = t42;
    result__[ 19  ] = t60;
    result__[ 20  ] = t54;
    result__[ 21  ] = Fxf(result__[15], t33, result__[20], result__[18]);
    result__[ 22  ] = Fxr(result__[16], t46, result__[19], result__[17]);
    result__[ 23  ] = Fyf(result__[15], t33, result__[20], result__[18]);
    result__[ 24  ] = Fyr(result__[16], t46, result__[19], result__[17]);
    result__[ 25  ] = Mzf(result__[15], t33, result__[20]);
    result__[ 26  ] = Mzr(result__[16], t46, result__[19]);
    real_type t84  = Q__[0];
    result__[ 27  ] = Mxf(t84);
    result__[ 28  ] = Mxr(t84);
    result__[ 29  ] = ALIAS_maxTorque(t48);
    result__[ 30  ] = Q__[10];
    result__[ 31  ] = Q__[11];
    result__[ 32  ] = Q__[8];
    result__[ 33  ] = Q__[9];
    real_type t86  = Q__[12];
    real_type t87  = sin(t86);
    result__[ 34  ] = t77 * t87 + Q__[6];
    real_type t90  = cos(t86);
    result__[ 35  ] = -t77 * t90 + Q__[7];
    real_type t94  = X__[4] + ModelPars[63];
    real_type t95  = cos(t94);
    real_type t97  = cos(t46);
    result__[ 36  ] = t38 * t97 - t70 * t95;
    real_type t99  = sin(t33);
    real_type t100 = t99 * t97;
    real_type t101 = sin(t46);
    real_type t102 = cos(t33);
    real_type t103 = t102 * t101;
    real_type t104 = t97 * t102;
    real_type t105 = t101 * t99;
    real_type t108 = sin(t94);
    result__[ 37  ] = -t100 + t103 - t108 * (t104 + t105) * t70;
    real_type t110 = ModelPars[23];
    real_type t111 = ModelPars[140];
    real_type t112 = t110 + t111;
    real_type t114 = ModelPars[118];
    real_type t115 = X__[8];
    real_type t116 = t114 - t115;
    real_type t118 = ModelPars[114];
    real_type t119 = ModelPars[116];
    real_type t120 = t118 - t119;
    real_type t121 = t99 * t120;
    result__[ 38  ] = t108 * t116 + t112 * t95 - t121 * t38 - t36;
    real_type t123 = -t120;
    real_type t125 = t102 * t123 - t119 - t19;
    result__[ 39  ] = t125 * t101 + (t121 - t37) * t97 + t111 * t70;
    real_type t137 = X__[5];
    result__[ 40  ] = -t116 * t95 + t112 * t108 + t125 * t97 + t101 * (t123 * t99 + t37) + t137;
    real_type t138 = X__[7];
    real_type t139 = sin(t138);
    real_type t141 = ModelPars[24];
    real_type t143 = cos(t138);
    result__[ 41  ] = -t108 * t139 * t141 - t141 * t143 * t95 + t58;
    real_type t146 = ModelPars[117];
    real_type t147 = -t146 - t25;
    real_type t149 = t97 * t47;
    result__[ 42  ] = t101 * t147 - t149;
    real_type t155 = t101 * t47;
    result__[ 43  ] = -t108 * t141 * t143 + t139 * t141 * t95 + t147 * t97 + t137 + t146 + t155 - ModelPars[115];
    real_type t157 = t108 * t70;
    real_type t158 = X__[20];
    real_type t160 = X__[22];
    real_type t162 = X__[31];
    real_type t164 = X__[19];
    real_type t165 = t164 * t101;
    result__[ 44  ] = t157 * t158 - t160 * t95 + t162 * t97 - t165 * t38;
    real_type t168 = X__[32];
    real_type t169 = t168 * t102;
    real_type t172 = t168 * t99;
    real_type t182 = t70 * t99;
    real_type t183 = t158 * t95;
    real_type t191 = t70 * t102;
    result__[ 45  ] = -t101 * t108 * t160 * t99 + t101 * t108 * t164 * t191 - t108 * t164 * t182 * t97 + t108 * t172 * t70 * t97 - t158 * t191 * t95 * t97 - t101 * t157 * t169 - t101 * t182 * t183 - t104 * t108 * t160 - t101 * t172 + t104 * t164 + t105 * t164 - t169 * t97;
    real_type t199 = t158 * t108;
    real_type t205 = t162 * t99;
    real_type t212 = X__[24];
    result__[ 46  ] = -t108 * t110 * t158 - t115 * t158 * t95 - t118 * t169 * t38 + t119 * t169 * t38 - t108 * t212 - t111 * t199 + t114 * t183 - t118 * t205 + t119 * t205 - t40;
    real_type t218 = t164 * t97;
    real_type t220 = t119 * t101;
    real_type t222 = t118 * t164;
    real_type t224 = t119 * t164;
    real_type t226 = t118 * t97;
    real_type t228 = t119 * t97;
    real_type t232 = t118 * t101;
    result__[ 47  ] = t101 * t164 * t37 - t164 * t19 * t97 - t101 * t20 - t104 * t222 + t104 * t224 - t105 * t222 + t105 * t224 + t111 * t160 - t119 * t218 + t169 * t226 - t169 * t228 - t172 * t220 + t172 * t232 - t41 * t97;
    real_type t259 = X__[21];
    result__[ 48  ] = t101 * t164 * t19 - t108 * t115 * t158 + t110 * t158 * t95 + t164 * t37 * t97 - t100 * t222 + t100 * t224 + t101 * t41 + t103 * t222 - t103 * t224 + t111 * t183 + t114 * t199 + t119 * t165 + t169 * t220 - t169 * t232 + t172 * t226 - t172 * t228 - t20 * t97 + t212 * t95 + t259;
    real_type t260 = t139 * t141;
    real_type t261 = X__[23];
    real_type t262 = t95 * t261;
    real_type t265 = t143 * t141;
    real_type t266 = t108 * t261;
    result__[ 49  ] = -t183 * t260 + t199 * t265 + t260 * t262 - t265 * t266 + t49;
    result__[ 50  ] = -t164 * t25 * t97 - t101 * t26 - t146 * t218 + t155 * t164 - t59 * t97;
    result__[ 51  ] = t101 * t164 * t25 + t101 * t59 + t146 * t165 + t149 * t164 - t183 * t265 - t199 * t260 - t26 * t97 + t260 * t266 + t262 * t265 + t259;
    #ifdef MECHATRONIX_DEBUG
    CHECK_NAN(result__,"post_eval",52);
    #endif
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  integer
  Baumgarte::integrated_post_numEqns() const
  { return 1; }

  void
  Baumgarte::integrated_post_eval(
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
    real_type t1   = X__[38];
    real_type t2   = sin(t1);
    real_type t5   = cos(t1);
    result__[ 0   ] = (X__[37] * Q__[1] - 1) / (X__[1] * t2 - X__[0] * t5);
    #ifdef MECHATRONIX_DEBUG
    CHECK_NAN(result__,"integrated_post_eval",1);
    #endif
  }

}

// EOF: Baumgarte_Methods1.cc

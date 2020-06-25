/*-----------------------------------------------------------------------*\
 |  file: Straight_Guess.cc                                              |
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

  using namespace std;

  /*\
   |    ____
   |   / ___|_   _  ___  ___ ___
   |  | |  _| | | |/ _ \/ __/ __|
   |  | |_| | |_| |  __/\__ \__ \
   |   \____|\__,_|\___||___/___/
  \*/

  void
  Straight::p_guess_eval( P_pointer_type P__ ) const {
  }

  void
  Straight::xlambda_guess_eval(
    integer              i_segment,
    Q_const_pointer_type Q__,
    P_const_pointer_type P__,
    X_pointer_type       X__,
    L_pointer_type       L__
  ) const {
    Road2D::SegmentClass const & segment = pRoad->getSegmentByIndex(i_segment);
    X__[ iX_u             ] = ModelPars[136];
    X__[ iX_v             ] = ModelPars[138];
    X__[ iX_Omega         ] = ModelPars[172];
    X__[ iX_phi           ] = ModelPars[97];
    X__[ iX_theta         ] = ModelPars[218];
    X__[ iX_h             ] = ModelPars[72];
    X__[ iX_delta         ] = ModelPars[179];
    X__[ iX_eta           ] = ModelPars[68];
    X__[ iX_s__f          ] = ModelPars[207];
    X__[ iX_x__f          ] = ModelPars[223];
    X__[ iX_y__f          ] = ModelPars[229];
    X__[ iX_z__f          ] = ModelPars[237];
    X__[ iX_x__r          ] = ModelPars[226];
    X__[ iX_y__r          ] = ModelPars[232];
    X__[ iX_z__r          ] = ModelPars[240];
    X__[ iX_delta__f      ] = ModelPars[177];
    X__[ iX_phi__f        ] = ModelPars[203];
    X__[ iX_omega__r      ] = ModelPars[196];
    X__[ iX_omega__f      ] = ModelPars[193];
    X__[ iX_phi__dot      ] = ModelPars[199];
    X__[ iX_theta__dot    ] = ModelPars[216];
    X__[ iX_h__dot        ] = ModelPars[185];
    X__[ iX_delta__dot    ] = ModelPars[175];
    X__[ iX_eta__dot      ] = ModelPars[183];
    X__[ iX_s__f__dot     ] = ModelPars[206];
    X__[ iX_x__f__dot     ] = ModelPars[222];
    X__[ iX_y__f__dot     ] = ModelPars[228];
    X__[ iX_z__f__dot     ] = ModelPars[236];
    X__[ iX_x__r__dot     ] = ModelPars[225];
    X__[ iX_y__r__dot     ] = ModelPars[231];
    X__[ iX_z__r__dot     ] = ModelPars[239];
    X__[ iX_delta__f__dot ] = ModelPars[242];
    X__[ iX_phi__f__dot   ] = ModelPars[201];
    X__[ iX_Ftr           ] = ModelPars[5];
    X__[ iX_Mbf           ] = ModelPars[27];
    X__[ iX_Mbr           ] = ModelPars[29];
    X__[ iX_tau           ] = ModelPars[129];
    X__[ iX_n             ] = ModelPars[82];
    X__[ iX_xi            ] = ModelPars[155];

  }

  /*\
   |    ____ _               _
   |   / ___| |__   ___  ___| | __
   |  | |   | '_ \ / _ \/ __| |/ /
   |  | |___| | | |  __/ (__|   <
   |   \____|_| |_|\___|\___|_|\_\
  \*/

  #define Xoptima__check__lt(A,B) ( (A) <  (B) )
  #define Xoptima__check__le(A,B) ( (A) <= (B) )

  // Node check strings
  #define __message_node_check_2 "2 < u(zeta)"
  #define __message_node_check_1 "5 < omega__f(zeta)"
  #define __message_node_check_0 "5 < omega__r(zeta)"
  #define __message_node_check_3 ".2 < h(zeta)"
  #define __message_node_check_4 "-1/2*Pi < phi(zeta)"
  #define __message_node_check_6 "-1/2*Pi < phi__f(zeta)"
  #define __message_node_check_8 "-1/4*Pi < delta(zeta)"
  #define __message_node_check_10 "-1/4*Pi < delta__f(zeta)"
  #define __message_node_check_9 "delta(zeta) < 1/4*Pi"
  #define __message_node_check_5 "phi(zeta) < 1/2*Pi"
  #define __message_node_check_7 "phi__f(zeta) < 1/2*Pi"
  #define __message_node_check_11 "delta__f(zeta) < 1/4*Pi"

  bool
  Straight::p_check( P_const_pointer_type P__ ) const {
    bool ok = true;

    return ok;
  }

  bool
  Straight::xlambda_check_node(
    integer              ipos,
    NodeType2 const    & NODE__,
    P_const_pointer_type P__
  ) const {
    bool ok = true;
    integer     i_segment = NODE__.i_segment;
    real_type const * Q__ = NODE__.q;
    real_type const * X__ = NODE__.x;
    real_type const * L__ = NODE__.lambda;
    Road2D::SegmentClass const & segment = pRoad->getSegmentByIndex(i_segment);
    ok = ok && Xoptima__check__lt(2, X__[0]);
    ok = ok && Xoptima__check__lt(5, X__[18]);
    ok = ok && Xoptima__check__lt(5, X__[17]);
    ok = ok && Xoptima__check__lt(0.2e0, X__[5]);
    real_type t5   = 0.3141592654e1 / 2;
    real_type t6   = X__[3];
    ok = ok && Xoptima__check__lt(-t5, t6);
    real_type t7   = X__[16];
    ok = ok && Xoptima__check__lt(-t5, t7);
    real_type t8   = 0.3141592654e1 / 4;
    real_type t9   = X__[6];
    ok = ok && Xoptima__check__lt(-t8, t9);
    real_type t10  = X__[15];
    ok = ok && Xoptima__check__lt(-t8, t10);
    ok = ok && Xoptima__check__lt(t9, t8);
    ok = ok && Xoptima__check__lt(t6, t5);
    ok = ok && Xoptima__check__lt(t7, t5);
    ok = ok && Xoptima__check__lt(t10, t8);
    return ok;
  }

  bool
  Straight::xlambda_check_cell(
    integer              icell,
    NodeType2 const    & NODE__,
    P_const_pointer_type P__
  ) const {
    return true;
  }

  bool
  Straight::xlambda_check_cell(
    integer              icell,
    NodeType2 const    & LEFT__,
    NodeType2 const    & RIGHT__,
    P_const_pointer_type P__
  ) const {
    return true;
  }
}

// EOF: Straight_Guess.cc

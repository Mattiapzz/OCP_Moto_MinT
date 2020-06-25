/*-----------------------------------------------------------------------*\
 |  file: Straight.hh                                                    |
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


#ifndef STRAIGHT_HH
#define STRAIGHT_HH

// Mechatronix Headers
#include <MechatronixCore/MechatronixCore.hh>
#include <MechatronixSolver/MechatronixSolver.hh>
#include <MechatronixInterfaceMruby/MechatronixInterfaceMruby.hh>

// user headers
#include <MechatronixVehicle/MechatronixVehicle.hh>
#include <MechatronixRoad/MechatronixRoad.hh>


#ifdef MECHATRONIX_OS_WINDOWS
  #ifndef STRAIGHT_API_DLL
    #ifdef STRAIGHT_EXPORT
      #define STRAIGHT_API_DLL __declspec(dllexport)
    #elif defined(STRAIGHT_IMPORT)
      #define STRAIGHT_API_DLL __declspec(dllimport)
    #else
      #define STRAIGHT_API_DLL
    #endif
  #endif
  #ifndef _SCL_SECURE_NO_WARNINGS
    #define _SCL_SECURE_NO_WARNINGS 1
  #endif
#else
  #define STRAIGHT_API_DLL
#endif

#define OCP_VIRTUAL          MECHATRONIX_OVERRIDE
#define INDIRECT_OCP_VIRTUAL MECHATRONIX_OVERRIDE

namespace StraightDefine {

  using namespace MechatronixLoad;

  using namespace std;
  using Mechatronix::real_type;
  using Mechatronix::integer;
  using Mechatronix::ostream_type;

  // user class in namespaces
  using Mechatronix::Engine;
  using Mechatronix::Road2D;


  extern char const *namesBc[];
  extern char const *namesXvars[];
  extern char const *namesLvars[];
  extern char const *namesUvars[];
  extern char const *namesQvars[];
  extern char const *namesPvars[];
  extern char const *namesOMEGAvars[];

  extern char const *namesModelPars[];

  extern char const *namesPostProcess[];
  extern char const *namesIntegratedPostProcess[];
  extern char const *namesConstraint1D[];
  extern char const *namesConstraint2D[];
  extern char const *namesConstraintU[];

  using Mechatronix::X_pointer_type;
  using Mechatronix::L_pointer_type;
  using Mechatronix::Z_pointer_type;
  using Mechatronix::U_pointer_type;
  using Mechatronix::V_pointer_type;
  using Mechatronix::Q_pointer_type;
  using Mechatronix::P_pointer_type;
  using Mechatronix::OMEGA_pointer_type;
  using Mechatronix::OMEGA_full_pointer_type;

  using Mechatronix::X_const_pointer_type;
  using Mechatronix::L_const_pointer_type;
  using Mechatronix::Z_const_pointer_type;
  using Mechatronix::U_const_pointer_type;
  using Mechatronix::V_const_pointer_type;
  using Mechatronix::Q_const_pointer_type;
  using Mechatronix::P_const_pointer_type;
  using Mechatronix::OMEGA_const_pointer_type;
  using Mechatronix::OMEGA_full_const_pointer_type;

  using Mechatronix::MatrixWrapper;

  using GenericContainerNamespace::GenericContainer;

  class Straight : public Mechatronix::Discretized_Indirect_OCP {

    // redirect output to a string in GenericContainer - - - - - - - - - - - - -
    stringstream ss_redirected_stream;

    // Model Paramaters  - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    real_type ModelPars[245];

    // Controls  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    Mechatronix::PenaltyBarrierU t__oControl;
    Mechatronix::PenaltyBarrierU b__f__oControl;
    Mechatronix::PenaltyBarrierU b__r__oControl;
    Mechatronix::PenaltyBarrierU tau__oControl;

    // Constraints 1D  - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    Mechatronix::PenaltyBarrier1DGreaterThan OnlyBrakingFront;
    Mechatronix::PenaltyBarrier1DGreaterThan OnlyBrakingRear;
    Mechatronix::PenaltyBarrier1DGreaterThan OnlyTractionRear;
    Mechatronix::PenaltyBarrier1DGreaterThan FrontWheelContact;
    Mechatronix::PenaltyBarrier1DGreaterThan RearWheelContact;
    Mechatronix::PenaltyBarrier1DInterval LongSlipFront;
    Mechatronix::PenaltyBarrier1DInterval LongSlipRear;
    Mechatronix::PenaltyBarrier1DInterval LatSlipFront;
    Mechatronix::PenaltyBarrier1DInterval LatSlipRear;
    Mechatronix::PenaltyBarrier1DInterval MaxSteerAngle;
    Mechatronix::PenaltyBarrier1DInterval MaxRollAngle;
    Mechatronix::PenaltyBarrier1DGreaterThan roadRightLateralBorder;
    Mechatronix::PenaltyBarrier1DGreaterThan roadLeftLateralBorder;

    // Constraints 2D  - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // User mapped functions - - - - - - - - - - - - - - - - - - - - - - - - - -

    // User classes (internal) - - - - - - - - - - - - - - - - - - - - - - - - -

    // User classes (external) - - - - - - - - - - - - - - - - - - - - - - - - -
    Mechatronix::Engine * pEngine;
    Mechatronix::Road2D * pRoad;

    // block copy constructor  - - - - - - - - - - - - - - - - - - - - - - - - -
    Straight( Straight const & );
    Straight const & operator = ( Straight const & );

    // subclass for continuation - - - - - - - - - - - - - - - - - - - - - - - -
    void continuationStep0( real_type s );
    void continuationStep1( real_type s );

  public:

    using Mechatronix::Discretized_Indirect_OCP::setup;
    using Mechatronix::Discretized_Indirect_OCP::guess;

    using Mechatronix::Discretized_Indirect_OCP::numOMEGA;

    using Mechatronix::Discretized_Indirect_OCP::bcInvMap;
    using Mechatronix::Discretized_Indirect_OCP::bcMap;
    using Mechatronix::Discretized_Indirect_OCP::numBC;

    using Mechatronix::Discretized_Indirect_OCP::dim_Q;
    using Mechatronix::Discretized_Indirect_OCP::dim_X;
    using Mechatronix::Discretized_Indirect_OCP::dim_U;
    using Mechatronix::Discretized_Indirect_OCP::dim_Pars;
    using Mechatronix::Discretized_Indirect_OCP::dim_Omega;
    using Mechatronix::Discretized_Indirect_OCP::dim_BC;
    using Mechatronix::Discretized_Indirect_OCP::nNodes;

    using Mechatronix::Discretized_Indirect_OCP::numEquations;
    using Mechatronix::Discretized_Indirect_OCP::eval_F;
    using Mechatronix::Discretized_Indirect_OCP::eval_JF_nnz;
    using Mechatronix::Discretized_Indirect_OCP::eval_JF_pattern;
    using Mechatronix::Discretized_Indirect_OCP::eval_JF_values;
    using Mechatronix::Discretized_Indirect_OCP::eval_JF;

    using Mechatronix::Discretized_Indirect_OCP::get_solution;
    using Mechatronix::Discretized_Indirect_OCP::get_solution_as_spline;
    using Mechatronix::Discretized_Indirect_OCP::get_solution_as_guess;

    using Mechatronix::Indirect_OCP::setupBC;

    STRAIGHT_API_DLL
    explicit
    Straight(
      string const & name,
      ThreadPool   * _TP,
      Console      * _pConsole
    );

    STRAIGHT_API_DLL virtual
    ~Straight() MECHATRONIX_OVERRIDE;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    STRAIGHT_API_DLL virtual
    char const * model_name() const MECHATRONIX_OVERRIDE
    { return "Straight"; }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    STRAIGHT_API_DLL
    void
    infoClasses() const;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // C++ initializer (raccolti in setup( gc ))
    STRAIGHT_API_DLL
    void
    setupParameters( GenericContainer const & gc );

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    STRAIGHT_API_DLL
    void
    setupParameters( real_type const Pars[] );

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    STRAIGHT_API_DLL
    void
    updateParameter( real_type val, integer idx )
    { ModelPars[idx] = val; }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    STRAIGHT_API_DLL
    void
    setupClasses( GenericContainer const & gc );

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    STRAIGHT_API_DLL
    void
    setupUserClasses( GenericContainer const & gc );

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    STRAIGHT_API_DLL
    void
    setupUserMappedFunctions( GenericContainer const & gc );

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    STRAIGHT_API_DLL
    void
    setupControls( GenericContainer const & gc );

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    STRAIGHT_API_DLL
    void
    setupPointers( GenericContainer const & gc );

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // user functions prototype (with derivative)
    STRAIGHT_API_DLL real_type alpha__crw      ( real_type t__XO ) const;
    STRAIGHT_API_DLL real_type alpha__crw_D    ( real_type t__XO ) const;
    STRAIGHT_API_DLL real_type alpha__crw_DD   ( real_type t__XO ) const;
    STRAIGHT_API_DLL real_type alpha__pin      ( real_type eta__XO, real_type alpha__crw__XO ) const;
    STRAIGHT_API_DLL real_type alpha__pin_D_1  ( real_type eta__XO, real_type alpha__crw__XO ) const;
    STRAIGHT_API_DLL real_type alpha__pin_D_1_1( real_type eta__XO, real_type alpha__crw__XO ) const;
    STRAIGHT_API_DLL real_type alpha__pin_D_1_2( real_type eta__XO, real_type alpha__crw__XO ) const;
    STRAIGHT_API_DLL real_type alpha__pin_D_2  ( real_type eta__XO, real_type alpha__crw__XO ) const;
    STRAIGHT_API_DLL real_type alpha__pin_D_2_2( real_type eta__XO, real_type alpha__crw__XO ) const;
    STRAIGHT_API_DLL real_type Fzf             ( real_type z__f__XO, real_type z__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type Fzf_D_1         ( real_type z__f__XO, real_type z__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type Fzf_D_1_1       ( real_type z__f__XO, real_type z__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type Fzf_D_1_2       ( real_type z__f__XO, real_type z__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type Fzf_D_2         ( real_type z__f__XO, real_type z__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type Fzf_D_2_2       ( real_type z__f__XO, real_type z__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type Fzr             ( real_type z__r__XO, real_type z__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type Fzr_D_1         ( real_type z__r__XO, real_type z__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type Fzr_D_1_1       ( real_type z__r__XO, real_type z__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type Fzr_D_1_2       ( real_type z__r__XO, real_type z__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type Fzr_D_2         ( real_type z__r__XO, real_type z__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type Fzr_D_2_2       ( real_type z__r__XO, real_type z__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r        ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_1    ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_1_1  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_1_2  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_1_3  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_1_4  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_1_5  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_1_6  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_1_7  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_2    ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_2_2  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_2_3  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_2_4  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_2_5  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_2_6  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_2_7  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_3    ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_3_3  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_3_4  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_3_5  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_3_6  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_3_7  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_4    ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_4_4  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_4_5  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_4_6  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_4_7  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_5    ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_5_5  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_5_6  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_5_7  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_6    ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_6_6  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_6_7  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_7    ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__r_D_7_7  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__r__XO, real_type y__r__XO, real_type x__r__dot__XO, real_type y__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f        ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_1    ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_1_1  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_1_2  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_1_3  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_1_4  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_1_5  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_1_6  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_1_7  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_1_8  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_2    ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_2_2  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_2_3  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_2_4  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_2_5  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_2_6  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_2_7  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_2_8  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_3    ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_3_3  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_3_4  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_3_5  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_3_6  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_3_7  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_3_8  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_4    ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_4_4  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_4_5  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_4_6  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_4_7  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_4_8  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_5    ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_5_5  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_5_6  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_5_7  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_5_8  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_6    ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_6_6  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_6_7  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_6_8  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_7    ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_7_7  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_7_8  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_8    ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type alpha__f_D_8_8  ( real_type Omega__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r       ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_1   ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_1_1 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_1_2 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_1_3 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_1_4 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_1_5 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_1_6 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_2   ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_2_2 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_2_3 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_2_4 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_2_5 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_2_6 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_3   ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_3_3 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_3_4 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_3_5 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_3_6 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_4   ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_4_4 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_4_5 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_4_6 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_5   ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_5_5 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_5_6 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_6   ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__r_D_6_6 ( real_type Omega__XO, real_type phi__XO, real_type u__XO, real_type y__r__XO, real_type omega__r__XO, real_type x__r__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f       ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_1   ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_1_1 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_1_2 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_1_3 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_1_4 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_1_5 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_1_6 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_1_7 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_1_8 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_1_9 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_1_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_2   ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_2_2 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_2_3 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_2_4 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_2_5 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_2_6 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_2_7 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_2_8 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_2_9 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_2_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_3   ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_3_3 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_3_4 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_3_5 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_3_6 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_3_7 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_3_8 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_3_9 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_3_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_4   ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_4_4 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_4_5 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_4_6 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_4_7 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_4_8 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_4_9 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_4_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_5   ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_5_5 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_5_6 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_5_7 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_5_8 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_5_9 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_5_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_6   ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_6_6 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_6_7 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_6_8 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_6_9 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_6_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_7   ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_7_7 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_7_8 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_7_9 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_7_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_8   ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_8_8 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_8_9 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_8_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_9   ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_9_9 ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_9_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_10  ( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type lambda__f_D_10_10( real_type Omega__XO, real_type phi__f__XO, real_type u__XO, real_type v__XO, real_type x__f__XO, real_type y__f__XO, real_type delta__f__XO, real_type omega__f__XO, real_type x__f__dot__XO, real_type y__f__dot__XO ) const;
    STRAIGHT_API_DLL real_type Fxf             ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fxf_D_1         ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fxf_D_1_1       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fxf_D_1_2       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fxf_D_1_3       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fxf_D_1_4       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fxf_D_2         ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fxf_D_2_2       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fxf_D_2_3       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fxf_D_2_4       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fxf_D_3         ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fxf_D_3_3       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fxf_D_3_4       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fxf_D_4         ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fxf_D_4_4       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fxr             ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fxr_D_1         ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fxr_D_1_1       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fxr_D_1_2       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fxr_D_1_3       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fxr_D_1_4       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fxr_D_2         ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fxr_D_2_2       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fxr_D_2_3       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fxr_D_2_4       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fxr_D_3         ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fxr_D_3_3       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fxr_D_3_4       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fxr_D_4         ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fxr_D_4_4       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fyf             ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fyf_D_1         ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fyf_D_1_1       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fyf_D_1_2       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fyf_D_1_3       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fyf_D_1_4       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fyf_D_2         ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fyf_D_2_2       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fyf_D_2_3       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fyf_D_2_4       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fyf_D_3         ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fyf_D_3_3       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fyf_D_3_4       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fyf_D_4         ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fyf_D_4_4       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO, real_type lambda__f__XO ) const;
    STRAIGHT_API_DLL real_type Fyr             ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fyr_D_1         ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fyr_D_1_1       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fyr_D_1_2       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fyr_D_1_3       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fyr_D_1_4       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fyr_D_2         ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fyr_D_2_2       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fyr_D_2_3       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fyr_D_2_4       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fyr_D_3         ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fyr_D_3_3       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fyr_D_3_4       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fyr_D_4         ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Fyr_D_4_4       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO, real_type lambda__r__XO ) const;
    STRAIGHT_API_DLL real_type Mzf             ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const;
    STRAIGHT_API_DLL real_type Mzf_D_1         ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const;
    STRAIGHT_API_DLL real_type Mzf_D_1_1       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const;
    STRAIGHT_API_DLL real_type Mzf_D_1_2       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const;
    STRAIGHT_API_DLL real_type Mzf_D_1_3       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const;
    STRAIGHT_API_DLL real_type Mzf_D_2         ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const;
    STRAIGHT_API_DLL real_type Mzf_D_2_2       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const;
    STRAIGHT_API_DLL real_type Mzf_D_2_3       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const;
    STRAIGHT_API_DLL real_type Mzf_D_3         ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const;
    STRAIGHT_API_DLL real_type Mzf_D_3_3       ( real_type Fzf__XO, real_type phi__f__XO, real_type alpha__f__XO ) const;
    STRAIGHT_API_DLL real_type Mzr             ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const;
    STRAIGHT_API_DLL real_type Mzr_D_1         ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const;
    STRAIGHT_API_DLL real_type Mzr_D_1_1       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const;
    STRAIGHT_API_DLL real_type Mzr_D_1_2       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const;
    STRAIGHT_API_DLL real_type Mzr_D_1_3       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const;
    STRAIGHT_API_DLL real_type Mzr_D_2         ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const;
    STRAIGHT_API_DLL real_type Mzr_D_2_2       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const;
    STRAIGHT_API_DLL real_type Mzr_D_2_3       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const;
    STRAIGHT_API_DLL real_type Mzr_D_3         ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const;
    STRAIGHT_API_DLL real_type Mzr_D_3_3       ( real_type Fzr__XO, real_type phi__XO, real_type alpha__r__XO ) const;
    STRAIGHT_API_DLL real_type Mxf             ( real_type t__XO ) const;
    STRAIGHT_API_DLL real_type Mxf_D           ( real_type t__XO ) const;
    STRAIGHT_API_DLL real_type Mxf_DD          ( real_type t__XO ) const;
    STRAIGHT_API_DLL real_type Mxr             ( real_type t__XO ) const;
    STRAIGHT_API_DLL real_type Mxr_D           ( real_type t__XO ) const;
    STRAIGHT_API_DLL real_type Mxr_DD          ( real_type t__XO ) const;

    #include <MechatronixSolver/OCP_methods.hxx>
    #include <MechatronixSolver/Indirect_OCP_methods.hxx>

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // C++ initializer (all in one)
    STRAIGHT_API_DLL
    void
    setup( GenericContainer const & gc );

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    void
    get_names( GenericContainer & out ) const;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // do some check on the computed solution
    STRAIGHT_API_DLL
    void
    diagnostic( GenericContainer const & gc_solution );

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Continuation phase update
    STRAIGHT_API_DLL
    void
    updateContinuation( integer phase, real_type s ) MECHATRONIX_OVERRIDE;

    // save model parameters
    STRAIGHT_API_DLL virtual
    void
    save_OCP_info( GenericContainer & gc ) const MECHATRONIX_OVERRIDE;

  };
}

namespace StraightLoad {
  using StraightDefine::Straight;

}

#endif

// EOF: Straight.hh

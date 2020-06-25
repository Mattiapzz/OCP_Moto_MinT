--[[
/*-----------------------------------------------------------------------*\
 |  file: MidLine_Data.lua                                               |
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

--]]

-- Auxiliary values
stering_norm    = 1
delta__max      = 1/18*Pi
phi__max        = 7/18*Pi
not_braking     = 0
w__LR0          = 0
tol_c0          = 0.1
tol_p0          = 0.1
w__LR           = w__LR0
long_peak_front = 0.11
w__ss0          = 1
w__ss           = w__ss0
eps_c0          = 0.1
eps_c           = eps_c0
eps_p0          = 0.1
lat_peak_rear   = 1
lat_peak_front  = 1
e__b0           = 0.1
e__b            = e__b0
tol_c           = tol_c0
closed_gas      = 0
tol_p           = tol_p0
w__ic0          = 1
braking         = 1
w__ic           = w__ic0
long_peak_rear  = 0.11
w__t0           = 1/100000
w__t            = w__t0
eps_p           = eps_p0
p__b0           = 0.1
p__b            = p__b0
open_gas        = 1

content = {

  -- Level of message
  InfoLevel = 4,

  -- maximum number of threads used for linear algebra and various solvers
  N_threads   = 4,
  U_threaded  = true,
  F_threaded  = true,
  JF_threaded = true,
  LU_threaded = true,

  -- Enable doctor
  Doctor = false,

  -- Enable check jacobian
  JacobianCheck            = false,
  JacobianCheckFull        = false,
  JacobianCheck_epsilon    = 1e-4,
  FiniteDifferenceJacobian = false,

  -- Redirect output to GenericContainer["stream_output"]
  RedirectStreamToString = false,

  -- Dump Function and Jacobian if uncommented
  -- DumpFile = "MidLine_dump",

  -- spline output (all values as function of "s")
  -- OutputSplines = [0],

  -- Redirect output to GenericContainer["stream_output"]
  RedirectStreamToString = false,

  ControlSolver = {
    -- "LU", "LUPQ", "QR", "QRP", "SVD", "LSS", "LSY", "MINIMIZATION"
    factorization = "LU",
    MaxIter       = 50,
    Tolerance     = 1e-9,
    Iterative     = false,
    InfoLevel     = -1 -- suppress all messages
  },

  -- setup solver
  Solver = {
    -- Linear algebra factorization selection:
    -- "LU", "QR", "QRP", "SUPERLU"
    factorization = "LU",

    -- Last Block selection:
    -- "LU", "LUPQ", "QR", "QRP", "SVD", "LSS", "LSY"
    last_factorization = "LU",

    -- choose solves: Hyness, NewtonDumped
    solver = "Hyness",

    -- solver parameters
    max_iter             = 300,
    max_step_iter        = 40,
    max_accumulated_iter = 800,
    tolerance            = 9.999999999999999e-10,

    -- continuation parameters
    ns_continuation_begin = 0,
    ns_continuation_end   = 4,
    continuation = {
      initial_step   = 0.2,   -- initial step for continuation
      min_step       = 0.001, -- minimum accepted step for continuation
      reduce_factor  = 0.5,   -- if continuation step fails, reduce step by this factor
      augment_factor = 1.5,   -- if step successful in less than few_iteration augment step by this factor
      few_iterations = 8
    }
  },

  -- Boundary Conditions (SET/FREE)
  BoundaryConditions = {
  },

  -- Guess
  Guess = {
    -- possible value: zero, default, none, warm
    initialize = "zero",
    -- possible value: default, none, warm, spline, table
    guess_type = "default"
  },

  Parameters = {

    -- Model Parameters
    CXZ                = 0.38736092039245,
    Ca                 = 0.3,
    Fzmin              = 200,
    IX                 = 42.6268656266919,
    IY                 = 79.5529675764739,
    IZ                 = 39.0041019529552,
    Id__wf             = 0.216,
    Ix__rdr            = 4.946,
    Ix__swa            = 0.02,
    Iy__swa            = 0.8,
    Iy__wf             = 0.4683,
    Iy__wr             = 0.664,
    Iz__rdr            = 3.304,
    Iz__swa            = 0.8,
    L__b               = 0.73,
    L__swa             = 0.535,
    M__tot             = 269.85,
    XG                 = 0.22776564612299,
    ZG                 = 0.198334243175045,
    a__1               = 0.13526,
    braking            = braking,
    c__fs              = 1679.7,
    c__rs              = 12000,
    epsilon            = 0.428,
    eta__00            = 0.029581668127468,
    g                  = 9.807,
    h__rdr             = 0.2,
    k__fs              = 19620,
    k__rs              = 147360,
    m__rdr             = 68.5,
    m__swa             = 10,
    m__wf              = 12,
    m__wr              = 16.2,
    n__err             = 1,
    r__crw             = 0.1,
    rf                 = 0.292,
    rr                 = 0.317,
    rtf                = 0.0624,
    rtr                = 0.097,
    s__fs              = 0.49,
    w__ss              = w__ss,
    w__t               = w__t,
    x__a               = 0,
    x__off             = 0.034,
    x__rdr             = 0.313,
    xi__n              = 1,
    z__a               = 0.8776,
    z__rdr             = 0.504,
    C__delta           = 10,
    Cxz__delta         = 0,
    Cxz__swa           = 0,
    Ix__delta          = 0.287,
    Iy__delta          = 0.143,
    Iz__delta          = 0.2063,
    Mb__norm           = 10,
    Mbf__max           = 800,
    Mbr__max           = 800,
    closed_gas         = closed_gas,
    delta__dot__err    = 1,
    delta__dot__ss     = 0,
    eta__dot__err      = 1,
    eta__dot__ss       = 0,
    h__dot__err        = 1,
    h__dot__ss         = 0,
    m__delta           = 8.75,
    not_braking        = not_braking,
    omega__n           = 10,
    open_gas           = open_gas,
    phi__dot__err      = 1,
    phi__dot__ss       = 0,
    phi__f__dot__ss    = 0,
    s__f__00           = 0.0568145809445757,
    s__f__dot__err     = 1,
    s__f__dot__ss      = 0,
    stering_norm       = stering_norm,
    tau__m__f          = 0.1,
    tau__m__r          = 0.1,
    tau__m__s          = 0.1,
    tau__m__t          = 0.1,
    tau__max           = 50,
    theta__d__00       = 0,
    theta__dot__err    = 1,
    theta__dot__ss     = 0,
    x__Swing           = 0.275,
    x__delta           = 0.023,
    x__f__dot__err     = 1,
    x__f__dot__ss      = 0,
    x__r__dot__err     = 1,
    x__r__dot__ss      = 0,
    y__f__dot__err     = 1,
    y__f__dot__ss      = 0,
    y__r__dot__err     = 1,
    y__r__dot__ss      = 0,
    z__Swing           = 0.052,
    z__delta           = -0.098,
    z__f__dot__err     = 1,
    z__f__dot__ss      = 0,
    z__r__dot__err     = 1,
    z__r__dot__ss      = 0,
    delta__f__dot__err = 1,
    delta__f__dot__ss  = 0,
    phi__f__dot__err   = 1,

    -- Guess Parameters
    Ftr__ss      = 380.399999969783,
    Mbf__ss      = 0,
    Mbr__ss      = 0,
    eta__ss      = 0.0307542604438726,
    h__ss        = 0.45769645033686,
    n__ss        = 0,
    phi__ss      = 0,
    tau__ss      = -7.00288564829757e-11,
    u__ss        = 20,
    v__ss        = -2.16104835964975e-10,
    xi__ss       = 0,
    Omega__ss    = -5.24305880064494e-14,
    delta__f__ss = -1.06378620631546e-11,
    delta__ss    = -1.14755170317492e-11,
    omega__f__ss = 68.4931507012337,
    omega__r__ss = 63.2921421213881,
    phi__f__ss   = 3.59906220911141e-12,
    s__f__ss     = 0.0508979277750632,
    theta__ss    = -0.109051230119918,
    x__f__ss     = 0.86315668743341,
    x__r__ss     = 0.512935849630048,
    y__f__ss     = -1.20129903987853e-13,
    y__r__ss     = 1.66702482026682e-13,
    z__f__ss     = -0.011693410617639,
    z__r__ss     = -0.0113621376018157,

    -- Boundary Conditions
    h__err        = 1,
    u__err        = 1,
    v__err        = 1,
    w__LR         = w__LR,
    xi__err       = 1,
    Ftr__err      = 1,
    Mbf__err      = 1,
    Mbr__err      = 1,
    Omega__err    = 1,
    delta__err    = 1,
    delta__f__err = 1,
    eta__err      = 1,
    omega__f__err = 1,
    omega__r__err = 1,
    phi__err      = 1,
    phi__f__err   = 1,
    s__f__err     = 1,
    tau__err      = 1,
    theta__err    = 1,
    x__f__err     = 1,
    x__r__err     = 1,
    y__f__err     = 1,
    y__r__err     = 1,
    z__f__err     = 1,
    z__r__err     = 1,

    -- Post Processing Parameters

    -- User Function Parameters
    Cp__f            = 70,
    Cp__r            = 100,
    Fz0              = 1600,
    Fz0__f           = 1700,
    Fz0__r           = 2000,
    Kp__f            = 100000,
    Kp__r            = 130000,
    d1__f            = 14,
    d1__r            = 13,
    d2__f            = 9,
    d2__r            = 4,
    d3__f            = 0.8,
    d3__r            = 0.8,
    d4__f            = 1.2,
    d4__r            = 1.2,
    d5__f            = 0.15,
    d5__r            = 0.4,
    d6__f            = 0.1,
    d6__r            = 0.1,
    d7__f            = 0.15,
    d7__r            = 0.15,
    d8               = 1.6,
    e10              = 1,
    e1__f            = 0.4,
    e1__r            = 0.4,
    e2__f            = 0.04,
    e2__r            = 0.07,
    e4               = 10,
    e5               = 2,
    e6               = 1.5,
    e7               = 50,
    e9               = 20,
    pCx1__f          = 1.6064,
    pCx1__r          = 1.6064,
    pDx1__f          = 1.2017,
    pDx1__r          = 1,
    pDx2__f          = -0.0922,
    pDx2__r          = 0,
    pKx1__f          = 25.94,
    pKx1__r          = 25.94,
    pKx2__f          = -4.233,
    pKx2__r          = -4.233,
    pKx3__f          = 0.3369,
    pKx3__r          = 0.3369,
    rBx1__f          = 10.27,
    rBx1__r          = 16.95,
    rBx2__f          = 7.72,
    rBx2__r          = 13.34,
    rBx3__f          = -0.44,
    rBx3__r          = 0.24,
    rBy1__f          = 4.67,
    rBy1__r          = 10.5,
    rBy2__f          = 7.18,
    rBy2__r          = 11.4,
    rBy3__f          = 0,
    rBy3__r          = 0,
    rCx1__f          = 1.26,
    rCx1__r          = 1.12,
    rCy1__f          = 1,
    rCy1__r          = 1,
    rHx1__f          = 0.016,
    rHx1__r          = 0.011,
    rHy1__f          = 0,
    rHy1__r          = 0,
    rHy2__f          = 0,
    rHy2__r          = 0,
    r__pin           = 0.05,
    epsilon__x__f    = 0,
    epsilon__x__r    = 0,
    lambda__C__x__f  = 1,
    lambda__C__x__r  = 1,
    lambda__K__x__f  = 1,
    lambda__K__x__r  = 1,
    lambda__mu__x__f = 1,
    lambda__mu__x__r = 1,

    -- Continuation Parameters
    e__b0  = e__b0,
    e__b1  = 0.01,
    eps_c0 = eps_c0,
    eps_c1 = 0.01,
    eps_p0 = eps_p0,
    eps_p1 = 0.01,
    p__b0  = p__b0,
    p__b1  = 0.01,
    tol_c0 = tol_c0,
    tol_c1 = 0.01,
    tol_p0 = tol_p0,
    tol_p1 = 0.01,
    w__LR0 = w__LR0,
    w__LR1 = 1,
    w__ic0 = w__ic0,
    w__ic1 = 0,
    w__ss0 = w__ss0,
    w__ss1 = 0,
    w__t0  = w__t0,
    w__t1  = 1,

    -- Constraints Parameters
  },

  -- functions mapped objects
  MappedObjects = {
  },

  -- Controls
  -- Penalty type controls: 'QUADRATIC', 'QUADRATIC2', 'PARABOLA', 'CUBIC'
  -- Barrier type controls: 'LOGARITHMIC', 'COS_LOGARITHMIC', 'TAN2', HYPERBOLIC'

  Controls = {
    t__oControl = {
      type      = 'COS_LOGARITHMIC',
      epsilon   = eps_c,
      tolerance = tol_c,
    },
    b__f__oControl = {
      type      = 'COS_LOGARITHMIC',
      epsilon   = eps_c,
      tolerance = tol_c,
    },
    b__r__oControl = {
      type      = 'COS_LOGARITHMIC',
      epsilon   = eps_c,
      tolerance = tol_c,
    },
    tau__oControl = {
      type      = 'COS_LOGARITHMIC',
      epsilon   = eps_c,
      tolerance = tol_c,
    },
  },

  Constraints = {
  -- Constraint1D
  -- Penalty subtype: "PENALTY_REGULAR", "PENALTY_SMOOTH", "PENALTY_PIECEWISE"
  -- Barrier subtype: "BARRIER_LOG", "BARRIER_LOG_EXP", "BARRIER_LOG0"
    -- PenaltyBarrier1DGreaterThan
    OnlyBrakingFrontsubType   = "PENALTY_REGULAR",
    OnlyBrakingFrontepsilon   = e__b,
    OnlyBrakingFronttolerance = p__b,
    OnlyBrakingFrontactive    = true

    -- PenaltyBarrier1DGreaterThan
    OnlyBrakingRearsubType   = "PENALTY_REGULAR",
    OnlyBrakingRearepsilon   = e__b,
    OnlyBrakingReartolerance = p__b,
    OnlyBrakingRearactive    = true

    -- PenaltyBarrier1DGreaterThan
    OnlyTractionRearsubType   = "PENALTY_REGULAR",
    OnlyTractionRearepsilon   = e__b,
    OnlyTractionReartolerance = p__b,
    OnlyTractionRearactive    = true

    -- PenaltyBarrier1DGreaterThan
    FrontWheelContactsubType   = "PENALTY_REGULAR",
    FrontWheelContactepsilon   = eps_p,
    FrontWheelContacttolerance = tol_p,
    FrontWheelContactactive    = true

    -- PenaltyBarrier1DGreaterThan
    RearWheelContactsubType   = "PENALTY_REGULAR",
    RearWheelContactepsilon   = eps_p,
    RearWheelContacttolerance = tol_p,
    RearWheelContactactive    = true

    -- PenaltyBarrier1DInterval
    LongSlipFrontsubType   = "PENALTY_REGULAR",
    LongSlipFrontepsilon   = eps_p,
    LongSlipFronttolerance = tol_p,
    LongSlipFrontmin       = -long_peak_front,
    LongSlipFrontmax       = long_peak_front,
    LongSlipFrontactive    = true

    -- PenaltyBarrier1DInterval
    LongSlipRearsubType   = "PENALTY_REGULAR",
    LongSlipRearepsilon   = eps_p,
    LongSlipReartolerance = tol_p,
    LongSlipRearmin       = -long_peak_rear,
    LongSlipRearmax       = long_peak_rear,
    LongSlipRearactive    = true

    -- PenaltyBarrier1DInterval
    LatSlipFrontsubType   = "PENALTY_REGULAR",
    LatSlipFrontepsilon   = eps_p,
    LatSlipFronttolerance = tol_p,
    LatSlipFrontmin       = -lat_peak_front,
    LatSlipFrontmax       = lat_peak_front,
    LatSlipFrontactive    = true

    -- PenaltyBarrier1DInterval
    LatSlipRearsubType   = "PENALTY_REGULAR",
    LatSlipRearepsilon   = eps_p,
    LatSlipReartolerance = tol_p,
    LatSlipRearmin       = -lat_peak_rear,
    LatSlipRearmax       = lat_peak_rear,
    LatSlipRearactive    = true

    -- PenaltyBarrier1DInterval
    MaxSteerAnglesubType   = "PENALTY_REGULAR",
    MaxSteerAngleepsilon   = eps_p,
    MaxSteerAngletolerance = tol_p,
    MaxSteerAnglemin       = -delta__max,
    MaxSteerAnglemax       = delta__max,
    MaxSteerAngleactive    = true

    -- PenaltyBarrier1DInterval
    MaxRollAnglesubType   = "PENALTY_REGULAR",
    MaxRollAngleepsilon   = eps_p,
    MaxRollAngletolerance = tol_p,
    MaxRollAnglemin       = -phi__max,
    MaxRollAnglemax       = phi__max,
    MaxRollAngleactive    = true

    -- PenaltyBarrier1DGreaterThan
    roadRightLateralBordersubType   = "PENALTY_REGULAR",
    roadRightLateralBorderepsilon   = eps_p,
    roadRightLateralBordertolerance = tol_p,
    roadRightLateralBorderactive    = true

    -- PenaltyBarrier1DGreaterThan
    roadLeftLateralBordersubType   = "PENALTY_REGULAR",
    roadLeftLateralBorderepsilon   = eps_p,
    roadLeftLateralBordertolerance = tol_p,
    roadLeftLateralBorderactive    = true

  -- Constraint2D: none defined
  },

  -- User defined classes initialization
  -- User defined classes: E N G I N E
  Engine = 
  {
    Gear_ratio = {
            4.82,
            2.5,
            1.75,
            1.368,
            1.09,
            0.956,
            0.851,
    },
    Rpm        = {
            0,
            3000,
            3500,
            4000,
            4500,
            5000,
            5500,
            6000,
            6500,
            7000,
            7500,
            8000,
            8500,
            9000,
            9500,
            10000,
            10500,
            11000,
            11500,
            12000,
            12500,
            13000,
    },
    Torque     = {
            2,
            47.088,
            62.784,
            70.632,
            78.48,
            84.366,
            85.347,
            83.385,
            81.423,
            90.252,
            92.214,
            87.309,
            85.347,
            84.366,
            80.442,
            79.461,
            73.575,
            63.765,
            49.05,
            29.43,
            19.62,
            2,
    },
  },
  -- User defined classes: R O A D
  dofile('../../../Custom_Tracks/Adria_Track/circuit-Adria.rb')


}

-- EOF

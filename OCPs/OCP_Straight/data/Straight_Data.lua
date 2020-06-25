--[[
/*-----------------------------------------------------------------------*\
 |  file: Straight_Data.lua                                              |
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

--]]

-- Auxiliary values
y__f__dot__ss     = 0
delta__f__ss      = -1.031762170965e-11
y__f__dot__e      = y__f__dot__ss
p__tb0            = 0.1
h__b0             = 0.1
u__ss             = 10
eta__ss           = 0.0307542604438375
braking           = 1
lat_peak_rear     = 1
phi__dot__ss      = 0
y__f__ss          = -1.01023535838235e-12
lat_peak_front    = 1
long_peak_front   = 0.11
tol_c0            = 0.1
tol_c             = tol_c0
p__tb             = p__tb0
phi__ss           = 0
eta__i            = eta__ss
s__f__ss          = 0.0508979277750826
s__f__i           = s__f__ss
s__f__e           = s__f__ss
theta__dot__ss    = 0
x__r__ss          = 0.512935849629851
delta__f__i       = delta__f__ss
e__tb0            = 0.1
x__f__ss          = 0.86315668743402
x__f__e           = x__f__ss
Ftr__ss           = 380.399999983247
Ftr__i            = Ftr__ss
p__b0             = 0.1
h__tb0            = 0.1
h__tb             = h__tb0
y__f__e           = y__f__ss
Mbf__ss           = 0
Mbf__i            = Mbf__ss
Mbf__e            = Mbf__ss
delta__max        = 1/18*Pi
closed_gas        = 0
Omega__ss         = 6.48425313027618e-13
Omega__e          = Omega__ss
phi__f__ss        = 3.18923345618669e-12
h__dot__ss        = 0
x__r__dot__ss     = 0
eps_c0            = 0.1
eps_c             = eps_c0
z__f__dot__ss     = 0
z__f__dot__e      = z__f__dot__ss
phi__f__e         = phi__f__ss
delta__ss         = -1.08966694619339e-11
delta__i          = delta__ss
phi__f__i         = phi__f__ss
phi__i            = phi__ss
Mbr__ss           = 0
Mbr__e            = Mbr__ss
open_gas          = 1
x__r__dot__e      = x__r__dot__ss
theta__ss         = -0.109051230120231
theta__i          = theta__ss
z__r__ss          = -0.0113621376018203
z__r__i           = z__r__ss
w__ss0            = 1
w__ss             = w__ss0
e__tb             = e__tb0
delta__dot__ss    = 0
delta__dot__i     = delta__dot__ss
delta__dot__e     = delta__dot__ss
x__r__i           = x__r__ss
x__r__e           = x__r__ss
n__ss             = 0
n__e              = n__ss
n__i              = n__ss
z__f__dot__i      = z__f__dot__ss
not_braking       = 0
eta__e            = eta__ss
h__dot__e         = h__dot__ss
tol_p0            = 0.1
u__i              = u__ss
h__dot__i         = h__dot__ss
y__r__dot__ss     = 0
y__r__dot__i      = y__r__dot__ss
e__b0             = 0.1
e__b              = e__b0
xi__ss            = 0
xi__e             = xi__ss
delta__f__e       = delta__f__ss
v__ss             = -2.11768865109627e-10
v__i              = v__ss
p__b              = p__b0
long_peak_rear    = 0.11
z__f__ss          = -0.0116934106176325
z__f__e           = z__f__ss
z__f__i           = z__f__ss
xi__i             = xi__ss
tol_p             = tol_p0
x__r__dot__i      = x__r__dot__ss
eta__dot__ss      = 0
eta__dot__i       = eta__dot__ss
eta__dot__e       = eta__dot__ss
y__r__dot__e      = y__r__dot__ss
y__f__dot__i      = y__f__dot__ss
u__e              = u__ss
delta__f__dot__ss = 0
delta__f__dot__e  = delta__f__dot__ss
delta__f__dot__i  = delta__f__dot__ss
theta__e          = theta__ss
w__t0             = 1e-05
s__f__dot__ss     = 0
s__f__dot__i      = s__f__dot__ss
s__f__dot__e      = s__f__dot__ss
phi__f__dot__ss   = 0
phi__f__dot__i    = phi__f__dot__ss
phi__f__dot__e    = phi__f__dot__ss
theta__dot__e     = theta__dot__ss
omega__r__ss      = 63.2921417857407
omega__r__e       = omega__r__ss
omega__r__i       = omega__r__ss
v__e              = v__ss
y__r__ss          = -1.02824615618752e-12
y__r__i           = y__r__ss
y__r__e           = y__r__ss
Mbr__i            = Mbr__ss
z__r__e           = z__r__ss
y__f__i           = y__f__ss
theta__dot__i     = theta__dot__ss
tau__ss           = -1.80900757436901e-10
tau__e            = tau__ss
delta__e          = delta__ss
x__f__dot__ss     = 0
x__f__dot__i      = x__f__dot__ss
phi__max          = 7/18*Pi
w__t              = w__t0
phi__e            = phi__ss
eps_p0            = 0.1
eps_p             = eps_p0
x__f__dot__e      = x__f__dot__ss
phi__dot__i       = phi__dot__ss
h__b              = h__b0
x__f__i           = x__f__ss
Ftr__e            = Ftr__ss
h__ss             = 0.457696450337007
h__i              = h__ss
h__e              = h__ss
z__r__dot__ss     = 0
z__r__dot__i      = z__r__dot__ss
z__r__dot__e      = z__r__dot__ss
Omega__i          = Omega__ss
tau__i            = tau__ss
stering_norm      = 1
phi__dot__e       = phi__dot__ss
omega__f__ss      = 68.4931506791026
omega__f__e       = omega__f__ss
omega__f__i       = omega__f__ss

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
  -- DumpFile = "Straight_dump",

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
    ns_continuation_end   = 2,
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
    initial_u             = SET,
    initial_v             = SET,
    initial_Omega         = SET,
    initial_phi           = SET,
    initial_theta         = SET,
    initial_h             = SET,
    initial_delta         = SET,
    initial_eta           = SET,
    initial_s__f          = SET,
    initial_x__f          = SET,
    initial_y__f          = SET,
    initial_z__f          = SET,
    initial_x__r          = SET,
    initial_y__r          = SET,
    initial_z__r          = SET,
    initial_delta__f      = SET,
    initial_phi__f        = SET,
    initial_omega__r      = SET,
    initial_omega__f      = SET,
    initial_phi__dot      = SET,
    initial_theta__dot    = SET,
    initial_h__dot        = SET,
    initial_delta__dot    = SET,
    initial_eta__dot      = SET,
    initial_s__f__dot     = SET,
    initial_x__f__dot     = SET,
    initial_y__f__dot     = SET,
    initial_z__f__dot     = SET,
    initial_x__r__dot     = SET,
    initial_y__r__dot     = SET,
    initial_z__r__dot     = SET,
    initial_delta__f__dot = SET,
    initial_phi__f__dot   = SET,
    initial_Ftr           = SET,
    initial_Mbf           = SET,
    initial_Mbr           = SET,
    initial_tau           = SET,
    initial_n             = SET,
    initial_xi            = SET,
    final_u               = SET,
    final_phi             = SET,
    final_n               = SET,
    final_xi              = SET,
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
    CXZ          = 0.38736092039245,
    Ca           = 0.3,
    Fzmin        = 200,
    IX           = 42.6268656266919,
    IY           = 79.5529675764739,
    IZ           = 39.0041019529552,
    Id__wf       = 0.216,
    Ix__rdr      = 4.946,
    Ix__swa      = 0.02,
    Iy__swa      = 0.8,
    Iy__wf       = 0.4683,
    Iy__wr       = 0.664,
    Iz__rdr      = 3.304,
    Iz__swa      = 0.8,
    L__b         = 0.73,
    L__swa       = 0.535,
    M__tot       = 269.85,
    XG           = 0.22776564612299,
    ZG           = 0.198334243175045,
    a__1         = 0.13526,
    braking      = braking,
    c__fs        = 1679.7,
    c__rs        = 12000,
    epsilon      = 0.428,
    eta__00      = 0.029581668127468,
    g            = 9.807,
    h__rdr       = 0.2,
    k__fs        = 19620,
    k__rs        = 147360,
    m__rdr       = 68.5,
    m__swa       = 10,
    m__wf        = 12,
    m__wr        = 16.2,
    n__ref       = 0,
    phi__ss      = phi__ss,
    r__crw       = 0.1,
    rf           = 0.292,
    rr           = 0.317,
    rtf          = 0.0624,
    rtr          = 0.097,
    s__fs        = 0.49,
    u__ss        = u__ss,
    v__ss        = v__ss,
    w__n         = 0,
    w__ss        = w__ss,
    w__t         = w__t,
    x__a         = 0,
    x__off       = 0.034,
    x__rdr       = 0.313,
    xi__n        = 1,
    z__a         = 0.8776,
    z__rdr       = 0.504,
    C__delta     = 10,
    Cxz__delta   = 0,
    Cxz__swa     = 0,
    Ix__delta    = 0.287,
    Iy__delta    = 0.143,
    Iz__delta    = 0.2063,
    Mb__norm     = 10,
    Mbf__max     = 800,
    Mbr__max     = 800,
    Omega__ss    = Omega__ss,
    closed_gas   = closed_gas,
    m__delta     = 8.75,
    not_braking  = not_braking,
    omega__n     = 10,
    open_gas     = open_gas,
    s__f__00     = 0.0568145809445759,
    stering_norm = stering_norm,
    tau__m__f    = 0.1,
    tau__m__r    = 0.1,
    tau__m__s    = 0.1,
    tau__m__t    = 0.1,
    tau__max     = 50,
    theta__d__00 = 0,
    x__Swing     = 0.275,
    x__delta     = 0.023,
    z__Swing     = 0.052,
    z__delta     = -0.098,

    -- Guess Parameters
    eta__ss           = eta__ss,
    h__ss             = h__ss,
    n__ss             = n__ss,
    tau__ss           = tau__ss,
    xi__ss            = xi__ss,
    delta__dot__ss    = delta__dot__ss,
    delta__f__ss      = delta__f__ss,
    delta__ss         = delta__ss,
    eta__dot__ss      = eta__dot__ss,
    h__dot__ss        = h__dot__ss,
    omega__f__ss      = omega__f__ss,
    omega__r__ss      = omega__r__ss,
    phi__dot__ss      = phi__dot__ss,
    phi__f__dot__ss   = phi__f__dot__ss,
    phi__f__ss        = phi__f__ss,
    s__f__dot__ss     = s__f__dot__ss,
    s__f__ss          = s__f__ss,
    theta__dot__ss    = theta__dot__ss,
    theta__ss         = theta__ss,
    x__f__dot__ss     = x__f__dot__ss,
    x__f__ss          = x__f__ss,
    x__r__dot__ss     = x__r__dot__ss,
    x__r__ss          = x__r__ss,
    y__f__dot__ss     = y__f__dot__ss,
    y__f__ss          = y__f__ss,
    y__r__dot__ss     = y__r__dot__ss,
    y__r__ss          = y__r__ss,
    z__f__dot__ss     = z__f__dot__ss,
    z__f__ss          = z__f__ss,
    z__r__dot__ss     = z__r__dot__ss,
    z__r__ss          = z__r__ss,
    delta__f__dot__ss = delta__f__dot__ss,

    -- Boundary Conditions
    Ftr__i           = Ftr__i,
    Ftr__ss          = Ftr__ss,
    Mbf__i           = Mbf__i,
    Mbf__ss          = Mbf__ss,
    Mbr__i           = Mbr__i,
    Mbr__ss          = Mbr__ss,
    eta__i           = eta__i,
    h__i             = h__i,
    n__e             = n__e,
    n__i             = n__i,
    phi__e           = phi__e,
    phi__i           = phi__i,
    s__f__i          = s__f__i,
    tau__i           = tau__i,
    u__e             = u__e,
    u__i             = u__i,
    v__i             = v__i,
    w__may           = 1,
    x__f__i          = x__f__i,
    x__r__i          = x__r__i,
    xi__e            = xi__e,
    xi__i            = xi__i,
    y__f__i          = y__f__i,
    y__r__i          = y__r__i,
    z__f__i          = z__f__i,
    z__r__i          = z__r__i,
    Omega__i         = Omega__i,
    delta__dot__i    = delta__dot__i,
    delta__f__i      = delta__f__i,
    delta__i         = delta__i,
    eta__dot__i      = eta__dot__i,
    h__dot__i        = h__dot__i,
    omega__f__i      = omega__f__i,
    omega__r__i      = omega__r__i,
    phi__dot__i      = phi__dot__i,
    phi__f__dot__i   = phi__f__dot__i,
    phi__f__i        = phi__f__i,
    s__f__dot__i     = s__f__dot__i,
    theta__dot__i    = theta__dot__i,
    theta__i         = theta__i,
    x__f__dot__i     = x__f__dot__i,
    x__r__dot__i     = x__r__dot__i,
    y__f__dot__i     = y__f__dot__i,
    y__r__dot__i     = y__r__dot__i,
    z__f__dot__i     = z__f__dot__i,
    z__r__dot__i     = z__r__dot__i,
    delta__f__dot__i = delta__f__dot__i,

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
    eps_c0 = eps_c0,
    eps_c1 = 0.01,
    eps_p0 = eps_p0,
    eps_p1 = 0.01,
    tol_c0 = tol_c0,
    tol_c1 = 0.01,
    tol_p0 = tol_p0,
    tol_p1 = 0.01,
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
  Road = 
  {
    theta0   = 0,
    s0       = 0,
    x0       = 0,
    y0       = 0,
    is_SAE   = false,
    segments = {
      
      {
        leftWidth  = 3,
        gridSize   = 0.5,
        rightWidth = 3,
        length     = 300,
        curvature  = 0,
      },
    },
  },


}

-- EOF

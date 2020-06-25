#-----------------------------------------------------------------------#
#  file: Straight_Data.rb                                               #
#                                                                       #
#  version: 1.0   date 23/6/2020                                        #
#                                                                       #
#  Copyright (C) 2020                                                   #
#                                                                       #
#      Enrico Bertolazzi, Francesco Biral and Paolo Bosetti             #
#      Dipartimento di Ingegneria Industriale                           #
#      Universita` degli Studi di Trento                                #
#      Via Sommarive 9, I-38123, Trento, Italy                          #
#      email: enrico.bertolazzi@unitn.it                                #
#             francesco.biral@unitn.it                                  #
#             paolo.bosetti@unitn.it                                    #
#-----------------------------------------------------------------------#


include Mechatronix

# Auxiliary values
y__f__dot__ss     = 0
delta__f__ss      = -1.031762170965e-11
y__f__dot__e      = y__f__dot__ss
p__tb0            = 0.1
h__b0             = 0.1
u__ss             = 20
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
delta__max        = 1/18.0*Math::PI
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
w__ss0            = 0
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
w__t0             = 1e-01
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
phi__max          = 7/18.0*Math::PI
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

mechatronix do |data|

  # Level of message
  data.InfoLevel = 4

  # maximum number of threads used for linear algebra and various solvers
  data.N_threads   = 4
  data.U_threaded  = true
  data.F_threaded  = true
  data.JF_threaded = true
  data.LU_threaded = true

  # Enable doctor
  data.Doctor = false

  # Enable check jacobian
  data.JacobianCheck            = false
  data.JacobianCheckFull        = false
  data.JacobianCheck_epsilon    = 1e-4
  data.FiniteDifferenceJacobian = false

  # Redirect output to GenericContainer["stream_output"]
  data.RedirectStreamToString = false

  # Dump Function and Jacobian if uncommented
  #data.DumpFile = "Straight_dump"

  # spline output (all values as function of "s")
  data.OutputSplines = [:s]

  # setup solver for controls
  data.ControlSolver = {
    # ==============================================================
    # 'LU', 'LUPQ', 'QR', 'QRP', 'SVD', 'LSS', 'LSY', 'MINIMIZATION'
    :factorization => 'LU',
    # ==============================================================
    :Rcond     => 1e-14,  # reciprocal condition number threshold for QR, SVD, LSS, LSY
    :MaxIter   => 50,
    :Tolerance => 1e-9,
    :Iterative => false,
    :InfoLevel => -1,     # suppress all messages
  }

  # setup solver
  data.Solver = {
    # Linear algebra factorization selection:
    # 'LU', 'QR', 'QRP', 'SUPERLU'
    # =================
    :factorization => 'LU',
    # =================

    # Last Block selection:
    # 'LU', 'LUPQ', 'QR', 'QRP', 'SVD', 'LSS', 'LSY'
    # ==============================================
    :last_factorization => 'LU',
    # ==============================================

    # choose solves: Hyness, NewtonDumped
    # ===================================
    :solver => "Hyness",
    # ===================================

    # solver parameters
    :max_iter             => 300,
    :max_step_iter        => 40,
    :max_accumulated_iter => 800,
    :tolerance            => 9.999999999999999e-10,

    # continuation parameters
    :ns_continuation_begin => 0,
    :ns_continuation_end   => 2,
    :continuation => {
      :initial_step   => 0.2,   # initial step for continuation
      :min_step       => 0.001, # minimum accepted step for continuation
      :reduce_factor  => 0.5,   # p fails, reduce step by this factor
      :augment_factor => 1.5,   # if step successful in less than few_iteration augment step by this factor
      :few_iterations => 8,     #
    }
  }

  # Boundary Conditions
  data.BoundaryConditions = {
    :initial_u             => SET,
    :initial_v             => SET,
    :initial_Omega         => SET,
    :initial_phi           => SET,
    :initial_theta         => SET,
    :initial_h             => SET,
    :initial_delta         => SET,
    :initial_eta           => SET,
    :initial_s__f          => SET,
    :initial_x__f          => SET,
    :initial_y__f          => SET,
    :initial_z__f          => SET,
    :initial_x__r          => SET,
    :initial_y__r          => SET,
    :initial_z__r          => SET,
    :initial_delta__f      => SET,
    :initial_phi__f        => SET,
    :initial_omega__r      => SET,
    :initial_omega__f      => SET,
    :initial_phi__dot      => SET,
    :initial_theta__dot    => SET,
    :initial_h__dot        => SET,
    :initial_delta__dot    => SET,
    :initial_eta__dot      => SET,
    :initial_s__f__dot     => SET,
    :initial_x__f__dot     => SET,
    :initial_y__f__dot     => SET,
    :initial_z__f__dot     => SET,
    :initial_x__r__dot     => SET,
    :initial_y__r__dot     => SET,
    :initial_z__r__dot     => SET,
    :initial_delta__f__dot => SET,
    :initial_phi__f__dot   => SET,
    :initial_Ftr           => SET,
    :initial_Mbf           => SET,
    :initial_Mbr           => SET,
    :initial_tau           => SET,
    :initial_n             => SET,
    :initial_xi            => SET,
    :final_u               => SET,
    :final_phi             => SET,
    :final_n               => SET,
    :final_xi              => SET,
  }

  # Guess
  data.Guess = {
    # possible value: zero, default, none, warm
    :initialize => 'zero',
    # possible value: default, none, warm, spline, table
    :guess_type => 'default',
  }

  data.Parameters = {}; # more than 127 keys!
  # Model Parameters
  data.Parameters[:CXZ         ] = 0.38736092039245
  data.Parameters[:Ca          ] = 0.3
  data.Parameters[:Fzmin       ] = 200
  data.Parameters[:IX          ] = 42.6268656266919
  data.Parameters[:IY          ] = 79.5529675764739
  data.Parameters[:IZ          ] = 39.0041019529552
  data.Parameters[:Id__wf      ] = 0.216
  data.Parameters[:Ix__rdr     ] = 4.946
  data.Parameters[:Ix__swa     ] = 0.02
  data.Parameters[:Iy__swa     ] = 0.8
  data.Parameters[:Iy__wf      ] = 0.4683
  data.Parameters[:Iy__wr      ] = 0.664
  data.Parameters[:Iz__rdr     ] = 3.304
  data.Parameters[:Iz__swa     ] = 0.8
  data.Parameters[:L__b        ] = 0.73
  data.Parameters[:L__swa      ] = 0.535
  data.Parameters[:M__tot      ] = 269.85
  data.Parameters[:XG          ] = 0.22776564612299
  data.Parameters[:ZG          ] = 0.198334243175045
  data.Parameters[:a__1        ] = 0.13526
  data.Parameters[:braking     ] = braking
  data.Parameters[:c__fs       ] = 1679.7
  data.Parameters[:c__rs       ] = 12000
  data.Parameters[:epsilon     ] = 0.428
  data.Parameters[:eta__00     ] = 0.029581668127468
  data.Parameters[:g           ] = 9.807
  data.Parameters[:h__rdr      ] = 0.2
  data.Parameters[:k__fs       ] = 19620
  data.Parameters[:k__rs       ] = 147360
  data.Parameters[:m__rdr      ] = 68.5
  data.Parameters[:m__swa      ] = 10
  data.Parameters[:m__wf       ] = 12
  data.Parameters[:m__wr       ] = 16.2
  data.Parameters[:n__ref      ] = 0
  data.Parameters[:phi__ss     ] = phi__ss
  data.Parameters[:r__crw      ] = 0.1
  data.Parameters[:rf          ] = 0.292
  data.Parameters[:rr          ] = 0.317
  data.Parameters[:rtf         ] = 0.0624
  data.Parameters[:rtr         ] = 0.097
  data.Parameters[:s__fs       ] = 0.49
  data.Parameters[:u__ss       ] = u__ss
  data.Parameters[:v__ss       ] = v__ss
  data.Parameters[:w__n        ] = 1
  data.Parameters[:w__ss       ] = w__ss
  data.Parameters[:w__t        ] = w__t
  data.Parameters[:x__a        ] = 0
  data.Parameters[:x__off      ] = 0.034
  data.Parameters[:x__rdr      ] = 0.313
  data.Parameters[:xi__n       ] = 1
  data.Parameters[:z__a        ] = 0.8776
  data.Parameters[:z__rdr      ] = 0.504
  data.Parameters[:C__delta    ] = 10
  data.Parameters[:Cxz__delta  ] = 0
  data.Parameters[:Cxz__swa    ] = 0
  data.Parameters[:Ix__delta   ] = 0.287
  data.Parameters[:Iy__delta   ] = 0.143
  data.Parameters[:Iz__delta   ] = 0.2063
  data.Parameters[:Mb__norm    ] = 10
  data.Parameters[:Mbf__max    ] = 800
  data.Parameters[:Mbr__max    ] = 800
  data.Parameters[:Omega__ss   ] = Omega__ss
  data.Parameters[:closed_gas  ] = closed_gas
  data.Parameters[:m__delta    ] = 8.75
  data.Parameters[:not_braking ] = not_braking
  data.Parameters[:omega__n    ] = 10
  data.Parameters[:open_gas    ] = open_gas
  data.Parameters[:s__f__00    ] = 0.0568145809445759
  data.Parameters[:stering_norm] = stering_norm
  data.Parameters[:tau__m__f   ] = 0.1
  data.Parameters[:tau__m__r   ] = 0.1
  data.Parameters[:tau__m__s   ] = 0.1
  data.Parameters[:tau__m__t   ] = 0.1
  data.Parameters[:tau__max    ] = 50
  data.Parameters[:theta__d__00] = 0
  data.Parameters[:x__Swing    ] = 0.275
  data.Parameters[:x__delta    ] = 0.023
  data.Parameters[:z__Swing    ] = 0.052
  data.Parameters[:z__delta    ] = -0.098

  # Guess Parameters
  data.Parameters[:eta__ss          ] = eta__ss
  data.Parameters[:h__ss            ] = h__ss
  data.Parameters[:n__ss            ] = n__ss
  data.Parameters[:tau__ss          ] = tau__ss
  data.Parameters[:xi__ss           ] = xi__ss
  data.Parameters[:delta__dot__ss   ] = delta__dot__ss
  data.Parameters[:delta__f__ss     ] = delta__f__ss
  data.Parameters[:delta__ss        ] = delta__ss
  data.Parameters[:eta__dot__ss     ] = eta__dot__ss
  data.Parameters[:h__dot__ss       ] = h__dot__ss
  data.Parameters[:omega__f__ss     ] = omega__f__ss
  data.Parameters[:omega__r__ss     ] = omega__r__ss
  data.Parameters[:phi__dot__ss     ] = phi__dot__ss
  data.Parameters[:phi__f__dot__ss  ] = phi__f__dot__ss
  data.Parameters[:phi__f__ss       ] = phi__f__ss
  data.Parameters[:s__f__dot__ss    ] = s__f__dot__ss
  data.Parameters[:s__f__ss         ] = s__f__ss
  data.Parameters[:theta__dot__ss   ] = theta__dot__ss
  data.Parameters[:theta__ss        ] = theta__ss
  data.Parameters[:x__f__dot__ss    ] = x__f__dot__ss
  data.Parameters[:x__f__ss         ] = x__f__ss
  data.Parameters[:x__r__dot__ss    ] = x__r__dot__ss
  data.Parameters[:x__r__ss         ] = x__r__ss
  data.Parameters[:y__f__dot__ss    ] = y__f__dot__ss
  data.Parameters[:y__f__ss         ] = y__f__ss
  data.Parameters[:y__r__dot__ss    ] = y__r__dot__ss
  data.Parameters[:y__r__ss         ] = y__r__ss
  data.Parameters[:z__f__dot__ss    ] = z__f__dot__ss
  data.Parameters[:z__f__ss         ] = z__f__ss
  data.Parameters[:z__r__dot__ss    ] = z__r__dot__ss
  data.Parameters[:z__r__ss         ] = z__r__ss
  data.Parameters[:delta__f__dot__ss] = delta__f__dot__ss

  # Boundary Conditions
  data.Parameters[:Ftr__i          ] = Ftr__i
  data.Parameters[:Ftr__ss         ] = Ftr__ss
  data.Parameters[:Mbf__i          ] = Mbf__i
  data.Parameters[:Mbf__ss         ] = Mbf__ss
  data.Parameters[:Mbr__i          ] = Mbr__i
  data.Parameters[:Mbr__ss         ] = Mbr__ss
  data.Parameters[:eta__i          ] = eta__i
  data.Parameters[:h__i            ] = h__i
  data.Parameters[:n__e            ] = n__e
  data.Parameters[:n__i            ] = n__i
  data.Parameters[:phi__e          ] = phi__e
  data.Parameters[:phi__i          ] = phi__i
  data.Parameters[:s__f__i         ] = s__f__i
  data.Parameters[:tau__i          ] = tau__i
  data.Parameters[:u__e            ] = u__e
  data.Parameters[:u__i            ] = u__i
  data.Parameters[:v__i            ] = v__i
  data.Parameters[:w__may          ] = 1
  data.Parameters[:x__f__i         ] = x__f__i
  data.Parameters[:x__r__i         ] = x__r__i
  data.Parameters[:xi__e           ] = xi__e
  data.Parameters[:xi__i           ] = xi__i
  data.Parameters[:y__f__i         ] = y__f__i
  data.Parameters[:y__r__i         ] = y__r__i
  data.Parameters[:z__f__i         ] = z__f__i
  data.Parameters[:z__r__i         ] = z__r__i
  data.Parameters[:Omega__i        ] = Omega__i
  data.Parameters[:delta__dot__i   ] = delta__dot__i
  data.Parameters[:delta__f__i     ] = delta__f__i
  data.Parameters[:delta__i        ] = delta__i
  data.Parameters[:eta__dot__i     ] = eta__dot__i
  data.Parameters[:h__dot__i       ] = h__dot__i
  data.Parameters[:omega__f__i     ] = omega__f__i
  data.Parameters[:omega__r__i     ] = omega__r__i
  data.Parameters[:phi__dot__i     ] = phi__dot__i
  data.Parameters[:phi__f__dot__i  ] = phi__f__dot__i
  data.Parameters[:phi__f__i       ] = phi__f__i
  data.Parameters[:s__f__dot__i    ] = s__f__dot__i
  data.Parameters[:theta__dot__i   ] = theta__dot__i
  data.Parameters[:theta__i        ] = theta__i
  data.Parameters[:x__f__dot__i    ] = x__f__dot__i
  data.Parameters[:x__r__dot__i    ] = x__r__dot__i
  data.Parameters[:y__f__dot__i    ] = y__f__dot__i
  data.Parameters[:y__r__dot__i    ] = y__r__dot__i
  data.Parameters[:z__f__dot__i    ] = z__f__dot__i
  data.Parameters[:z__r__dot__i    ] = z__r__dot__i
  data.Parameters[:delta__f__dot__i] = delta__f__dot__i

  # Post Processing Parameters

  # User Function Parameters
  data.Parameters[:Cp__f           ] = 70
  data.Parameters[:Cp__r           ] = 100
  data.Parameters[:Fz0             ] = 1600
  data.Parameters[:Fz0__f          ] = 1700
  data.Parameters[:Fz0__r          ] = 2000
  data.Parameters[:Kp__f           ] = 100000
  data.Parameters[:Kp__r           ] = 130000
  data.Parameters[:d1__f           ] = 14
  data.Parameters[:d1__r           ] = 13
  data.Parameters[:d2__f           ] = 9
  data.Parameters[:d2__r           ] = 4
  data.Parameters[:d3__f           ] = 0.8
  data.Parameters[:d3__r           ] = 0.8
  data.Parameters[:d4__f           ] = 1.2
  data.Parameters[:d4__r           ] = 1.2
  data.Parameters[:d5__f           ] = 0.15
  data.Parameters[:d5__r           ] = 0.4
  data.Parameters[:d6__f           ] = 0.1
  data.Parameters[:d6__r           ] = 0.1
  data.Parameters[:d7__f           ] = 0.15
  data.Parameters[:d7__r           ] = 0.15
  data.Parameters[:d8              ] = 1.6
  data.Parameters[:e10             ] = 1
  data.Parameters[:e1__f           ] = 0.4
  data.Parameters[:e1__r           ] = 0.4
  data.Parameters[:e2__f           ] = 0.04
  data.Parameters[:e2__r           ] = 0.07
  data.Parameters[:e4              ] = 10
  data.Parameters[:e5              ] = 2
  data.Parameters[:e6              ] = 1.5
  data.Parameters[:e7              ] = 50
  data.Parameters[:e9              ] = 20
  data.Parameters[:pCx1__f         ] = 1.6064
  data.Parameters[:pCx1__r         ] = 1.6064
  data.Parameters[:pDx1__f         ] = 1.2017
  data.Parameters[:pDx1__r         ] = 1
  data.Parameters[:pDx2__f         ] = -0.0922
  data.Parameters[:pDx2__r         ] = 0
  data.Parameters[:pKx1__f         ] = 25.94
  data.Parameters[:pKx1__r         ] = 25.94
  data.Parameters[:pKx2__f         ] = -4.233
  data.Parameters[:pKx2__r         ] = -4.233
  data.Parameters[:pKx3__f         ] = 0.3369
  data.Parameters[:pKx3__r         ] = 0.3369
  data.Parameters[:rBx1__f         ] = 10.27
  data.Parameters[:rBx1__r         ] = 16.95
  data.Parameters[:rBx2__f         ] = 7.72
  data.Parameters[:rBx2__r         ] = 13.34
  data.Parameters[:rBx3__f         ] = -0.44
  data.Parameters[:rBx3__r         ] = 0.24
  data.Parameters[:rBy1__f         ] = 4.67
  data.Parameters[:rBy1__r         ] = 10.5
  data.Parameters[:rBy2__f         ] = 7.18
  data.Parameters[:rBy2__r         ] = 11.4
  data.Parameters[:rBy3__f         ] = 0
  data.Parameters[:rBy3__r         ] = 0
  data.Parameters[:rCx1__f         ] = 1.26
  data.Parameters[:rCx1__r         ] = 1.12
  data.Parameters[:rCy1__f         ] = 1
  data.Parameters[:rCy1__r         ] = 1
  data.Parameters[:rHx1__f         ] = 0.016
  data.Parameters[:rHx1__r         ] = 0.011
  data.Parameters[:rHy1__f         ] = 0
  data.Parameters[:rHy1__r         ] = 0
  data.Parameters[:rHy2__f         ] = 0
  data.Parameters[:rHy2__r         ] = 0
  data.Parameters[:r__pin          ] = 0.05
  data.Parameters[:epsilon__x__f   ] = 0
  data.Parameters[:epsilon__x__r   ] = 0
  data.Parameters[:lambda__C__x__f ] = 1
  data.Parameters[:lambda__C__x__r ] = 1
  data.Parameters[:lambda__K__x__f ] = 1
  data.Parameters[:lambda__K__x__r ] = 1
  data.Parameters[:lambda__mu__x__f] = 1
  data.Parameters[:lambda__mu__x__r] = 1

  # Continuation Parameters
  data.Parameters[:eps_c0] = eps_c0
  data.Parameters[:eps_c1] = 0.01
  data.Parameters[:eps_p0] = eps_p0
  data.Parameters[:eps_p1] = 0.01
  data.Parameters[:tol_c0] = tol_c0
  data.Parameters[:tol_c1] = 0.01
  data.Parameters[:tol_p0] = tol_p0
  data.Parameters[:tol_p1] = 0.01
  data.Parameters[:w__ss0] = w__ss0
  data.Parameters[:w__ss1] = 0
  data.Parameters[:w__t0 ] = w__t0
  data.Parameters[:w__t1 ] = 1

  # Constraints Parameters


  # functions mapped on objects
  data.MappedObjects = {}

  # Controls
  # Penalty type controls: "QUADRATIC", "QUADRATIC2", "PARABOLA", "CUBIC"
  # Barrier type controls: "LOGARITHMIC", "COS_LOGARITHMIC", "TAN2", "HYPERBOLIC"

  data.Controls = {}
  data.Controls[:t__oControl] = {
    :type      => 'COS_LOGARITHMIC',
    :epsilon   => eps_c,
    :tolerance => tol_c
  }

  data.Controls[:b__f__oControl] = {
    :type      => 'COS_LOGARITHMIC',
    :epsilon   => eps_c,
    :tolerance => tol_c
  }

  data.Controls[:b__r__oControl] = {
    :type      => 'COS_LOGARITHMIC',
    :epsilon   => eps_c,
    :tolerance => tol_c
  }

  data.Controls[:tau__oControl] = {
    :type      => 'COS_LOGARITHMIC',
    :epsilon   => eps_c,
    :tolerance => tol_c
  }


  data.Constraints = {}
  # Constraint1D
  # Penalty subtype: 'PENALTY_REGULAR', 'PENALTY_SMOOTH', 'PENALTY_PIECEWISE'
  # Barrier subtype: 'BARRIER_LOG', 'BARRIER_LOG_EXP', 'BARRIER_LOG0'
  # PenaltyBarrier1DGreaterThan
  data.Constraints[:OnlyBrakingFront] = {
    :subType   => 'PENALTY_REGULAR',
    :epsilon   => e__b,
    :tolerance => p__b,
    :active    => true
  }
  # PenaltyBarrier1DGreaterThan
  data.Constraints[:OnlyBrakingRear] = {
    :subType   => 'PENALTY_REGULAR',
    :epsilon   => e__b,
    :tolerance => p__b,
    :active    => true
  }
  # PenaltyBarrier1DGreaterThan
  data.Constraints[:OnlyTractionRear] = {
    :subType   => 'PENALTY_REGULAR',
    :epsilon   => e__b,
    :tolerance => p__b,
    :active    => true
  }
  # PenaltyBarrier1DGreaterThan
  data.Constraints[:FrontWheelContact] = {
    :subType   => 'PENALTY_REGULAR',
    :epsilon   => eps_p,
    :tolerance => tol_p,
    :active    => true
  }
  # PenaltyBarrier1DGreaterThan
  data.Constraints[:RearWheelContact] = {
    :subType   => 'PENALTY_REGULAR',
    :epsilon   => eps_p,
    :tolerance => tol_p,
    :active    => true
  }
  # PenaltyBarrier1DInterval
  data.Constraints[:LongSlipFront] = {
    :subType   => 'PENALTY_REGULAR',
    :epsilon   => eps_p,
    :tolerance => tol_p,
    :min       => -long_peak_front,
    :max       => long_peak_front,
    :active    => true
  }
  # PenaltyBarrier1DInterval
  data.Constraints[:LongSlipRear] = {
    :subType   => 'PENALTY_REGULAR',
    :epsilon   => eps_p,
    :tolerance => tol_p,
    :min       => -long_peak_rear,
    :max       => long_peak_rear,
    :active    => true
  }
  # PenaltyBarrier1DInterval
  data.Constraints[:LatSlipFront] = {
    :subType   => 'PENALTY_REGULAR',
    :epsilon   => eps_p,
    :tolerance => tol_p,
    :min       => -lat_peak_front,
    :max       => lat_peak_front,
    :active    => true
  }
  # PenaltyBarrier1DInterval
  data.Constraints[:LatSlipRear] = {
    :subType   => 'PENALTY_REGULAR',
    :epsilon   => eps_p,
    :tolerance => tol_p,
    :min       => -lat_peak_rear,
    :max       => lat_peak_rear,
    :active    => true
  }
  # PenaltyBarrier1DInterval
  data.Constraints[:MaxSteerAngle] = {
    :subType   => 'PENALTY_REGULAR',
    :epsilon   => eps_p,
    :tolerance => tol_p,
    :min       => -delta__max,
    :max       => delta__max,
    :active    => true
  }
  # PenaltyBarrier1DInterval
  data.Constraints[:MaxRollAngle] = {
    :subType   => 'PENALTY_REGULAR',
    :epsilon   => eps_p,
    :tolerance => tol_p,
    :min       => -phi__max,
    :max       => phi__max,
    :active    => true
  }
  # PenaltyBarrier1DGreaterThan
  data.Constraints[:roadRightLateralBorder] = {
    :subType   => 'PENALTY_REGULAR',
    :epsilon   => eps_p,
    :tolerance => tol_p,
    :active    => true
  }
  # PenaltyBarrier1DGreaterThan
  data.Constraints[:roadLeftLateralBorder] = {
    :subType   => 'PENALTY_REGULAR',
    :epsilon   => eps_p,
    :tolerance => tol_p,
    :active    => true
  }
  # Constraint2D: none defined

  # User defined classes initialization
  # User defined classes: E N G I N E
  data.Engine =
  {
    :Gear_ratio => [
            4.82,
            2.5,
            1.75,
            1.368,
            1.09,
            0.956,
            0.851,
    ],
    :Rpm        => [
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
    ],
    :Torque     => [
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
    ],
  };
  # User defined classes: R O A D
  data.Road =
  {
    :theta0   => 0,
    :s0       => 0,
    :x0       => 0,
    :y0       => 0,
    :is_SAE   => false,
    :segments => [
      {
        :leftWidth  => 3,
        :gridSize   => 0.5,
        :rightWidth => 3,
        :length     => 300,
        :curvature  => 0,
      },
    ],
  };


end

# EOF

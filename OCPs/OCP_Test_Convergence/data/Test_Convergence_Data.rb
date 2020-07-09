#-----------------------------------------------------------------------#
#  file: Test_Convergence_Data.rb                                       #
#                                                                       #
#  version: 1.0   date 30/6/2020                                        #
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
w__ss0       = 1
w__ss        = w__ss0
w__LR0       = 100
not_braking  = 0
stering_norm = 1
e__b0        = 0.1
tol_c0       = 0.1
tol_c        = tol_c0
open_gas     = 1
e__b         = e__b0
w__t0        = 1/100000.0
w__LR        = w__LR0
w__n00       = 1
w__n0        = w__n00
eps_c0       = 0.1
eps_c        = eps_c0
p__b0        = 0.1
p__b         = p__b0
tol_p0       = 0.1
h_c0         = 0.01
h_c          = h_c0
tol_p        = tol_p0
baum_flag    = 1
xi__n        = 0.9*baum_flag
w__pen       = 1-baum_flag
omega__n     = 20*baum_flag
w__t         = w__t0
braking      = 1
closed_gas   = 0
eps_p0       = 0.1
eps_p        = eps_p0
w__ic0       = 1
w__ic        = w__ic0

mechatronix do |data|

  # Level of message
  data.InfoLevel = 4

  # maximum number of threads used for linear algebra and various solvers
  data.N_threads   = 6
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
  #data.DumpFile = "Test_Convergence_dump"

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
    :ns_continuation_end   => 3,
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
  data.Parameters[:CXZ                    ] = 0.387358773110395
  data.Parameters[:Ca                     ] = 0.3
  data.Parameters[:Fzmin                  ] = 100
  data.Parameters[:Fznorm                 ] = 1200
  data.Parameters[:IX                     ] = 42.6268722360008
  data.Parameters[:IY                     ] = 79.5529742502298
  data.Parameters[:IZ                     ] = 39.0041020174022
  data.Parameters[:Id__wf                 ] = 0.216
  data.Parameters[:Ix__rdr                ] = 4.946
  data.Parameters[:Ix__swa                ] = 0.02
  data.Parameters[:Iy__swa                ] = 0.8
  data.Parameters[:Iy__wf                 ] = 0.4683
  data.Parameters[:Iy__wr                 ] = 0.664
  data.Parameters[:Iz__rdr                ] = 3.304
  data.Parameters[:Iz__swa                ] = 0.8
  data.Parameters[:L__b                   ] = 0.73
  data.Parameters[:L__swa                 ] = 0.535
  data.Parameters[:M__tot                 ] = 269.85
  data.Parameters[:XG                     ] = 0.227765645922705
  data.Parameters[:ZG                     ] = 0.198334221855122
  data.Parameters[:a__1                   ] = 0.13526
  data.Parameters[:braking                ] = braking
  data.Parameters[:c__fs                  ] = 1679.7
  data.Parameters[:c__rs                  ] = 12000
  data.Parameters[:epsilon                ] = 0.428
  data.Parameters[:eta__00                ] = 0.0295816047682125
  data.Parameters[:eta__ss                ] = 0.0299034407512161
  data.Parameters[:g                      ] = 9.807
  data.Parameters[:h__rdr                 ] = 0.2
  data.Parameters[:h__ss                  ] = 0.45640876323828
  data.Parameters[:h_c                    ] = h_c
  data.Parameters[:k__fs                  ] = 19620
  data.Parameters[:k__rs                  ] = 147360
  data.Parameters[:m__rdr                 ] = 68.5
  data.Parameters[:m__swa                 ] = 10
  data.Parameters[:m__wf                  ] = 12
  data.Parameters[:m__wr                  ] = 16.2
  data.Parameters[:r__crw                 ] = 0.1
  data.Parameters[:rf                     ] = 0.292
  data.Parameters[:rr                     ] = 0.317
  data.Parameters[:rtf                    ] = 0.0624
  data.Parameters[:rtr                    ] = 0.097
  data.Parameters[:s__fs                  ] = 0.49
  data.Parameters[:w__n0                  ] = w__n0
  data.Parameters[:w__ss                  ] = w__ss
  data.Parameters[:w__t                   ] = w__t
  data.Parameters[:x__a                   ] = 0
  data.Parameters[:x__off                 ] = 0.034
  data.Parameters[:x__rdr                 ] = 0.313
  data.Parameters[:xi__n                  ] = xi__n
  data.Parameters[:z__a                   ] = 0.8776
  data.Parameters[:z__rdr                 ] = 0.504
  data.Parameters[:C__delta               ] = 10
  data.Parameters[:Cxz__delta             ] = 0
  data.Parameters[:Cxz__swa               ] = 0
  data.Parameters[:Ix__delta              ] = 0.287
  data.Parameters[:Iy__delta              ] = 0.143
  data.Parameters[:Iz__delta              ] = 0.2063
  data.Parameters[:Mb__norm               ] = 10
  data.Parameters[:Mbf__max               ] = 800
  data.Parameters[:Mbr__max               ] = 800
  data.Parameters[:closed_gas             ] = closed_gas
  data.Parameters[:delta__dot__ss         ] = 0
  data.Parameters[:delta__max             ] = 1.5/18.00*Math::PI
  data.Parameters[:eta__dot__ss           ] = 0
  data.Parameters[:h__dot__ss             ] = 0
  data.Parameters[:lat_peak_front         ] = 1
  data.Parameters[:lat_peak_rear          ] = 1
  data.Parameters[:long_peak_front        ] = 0.06
  data.Parameters[:long_peak_rear         ] = 0.06
  data.Parameters[:m__delta               ] = 8.75
  data.Parameters[:not_braking            ] = not_braking
  data.Parameters[:omega__n               ] = omega__n
  data.Parameters[:open_gas               ] = open_gas
  data.Parameters[:phi__dot__ss           ] = 0
  data.Parameters[:phi__f__dot__ss        ] = 0
  data.Parameters[:phi__max               ] = 7/18.00*Math::PI
  data.Parameters[:s__f__00               ] = 0.0568141608946355
  data.Parameters[:s__f__dot__ss          ] = 0
  data.Parameters[:s__f__ss               ] = 0.0554922544768378
  data.Parameters[:stering_norm           ] = stering_norm
  data.Parameters[:tau__m__f              ] = 0.1
  data.Parameters[:tau__m__r              ] = 0.1
  data.Parameters[:tau__m__s              ] = 0.1
  data.Parameters[:tau__m__t              ] = 0.1
  data.Parameters[:tau__max               ] = 50
  data.Parameters[:theta__d__00           ] = 0
  data.Parameters[:theta__dot__ss         ] = 0
  data.Parameters[:theta__ss              ] = -0.113743662374938
  data.Parameters[:w__eta__sserr          ] = 1
  data.Parameters[:w__h__sserr            ] = 1
  data.Parameters[:w__s__f__sserr         ] = 1
  data.Parameters[:w__theta__sserr        ] = 1
  data.Parameters[:x__Swing               ] = 0.275
  data.Parameters[:x__delta               ] = 0.023
  data.Parameters[:x__f__dot__ss          ] = 0
  data.Parameters[:x__r__dot__ss          ] = 0
  data.Parameters[:y__f__dot__ss          ] = 0
  data.Parameters[:y__r__dot__ss          ] = 0
  data.Parameters[:z__Swing               ] = 0.052
  data.Parameters[:z__delta               ] = -0.098
  data.Parameters[:z__f__dot__ss          ] = 0
  data.Parameters[:z__r__dot__ss          ] = 0
  data.Parameters[:delta__f__dot__ss      ] = 0
  data.Parameters[:w__delta__dot__sserr   ] = 1
  data.Parameters[:w__delta__f__dot__sserr] = 1
  data.Parameters[:w__eta__dot__sserr     ] = 1
  data.Parameters[:w__h__dot__sserr       ] = 1
  data.Parameters[:w__phi__dot__sserr     ] = 1
  data.Parameters[:w__phi__f__dot__sserr  ] = 1
  data.Parameters[:w__s__f__dot__sserr    ] = 1
  data.Parameters[:w__theta__dot__sserr   ] = 1
  data.Parameters[:w__x__f__dot__sserr    ] = 1
  data.Parameters[:w__x__r__dot__sserr    ] = 1
  data.Parameters[:w__y__f__dot__sserr    ] = 1
  data.Parameters[:w__y__r__dot__sserr    ] = 1
  data.Parameters[:w__z__f__dot__sserr    ] = 1
  data.Parameters[:w__z__r__dot__sserr    ] = 1

  # Guess Parameters

  # Boundary Conditions
  data.Parameters[:Ftr__ss             ] = 52.7258148279645
  data.Parameters[:Mbf__ss             ] = 0
  data.Parameters[:Mbr__ss             ] = 0
  data.Parameters[:n__ss               ] = 0
  data.Parameters[:phi__ss             ] = 0
  data.Parameters[:tau__ss             ] = -0.0835883342939634
  data.Parameters[:u__ss               ] = 10
  data.Parameters[:v__ss               ] = -1.43375156862825e-05
  data.Parameters[:w__LR               ] = w__LR
  data.Parameters[:w__ic               ] = w__ic
  data.Parameters[:xi__ss              ] = 0
  data.Parameters[:Omega__ss           ] = 0.000273532055385562
  data.Parameters[:delta__f__ss        ] = 3.72585770969475e-05
  data.Parameters[:delta__ss           ] = 4.92092837253778e-05
  data.Parameters[:omega__f__ss        ] = 34.2465757357341
  data.Parameters[:omega__r__ss        ] = 31.5727410597457
  data.Parameters[:phi__f__ss          ] = -7.0332277538887e-05
  data.Parameters[:w__Ftr__bc          ] = 0.0003597107358
  data.Parameters[:w__Mbf__bc          ] = 1
  data.Parameters[:w__Mbr__bc          ] = 1
  data.Parameters[:w__Omega__bc        ] = 1
  data.Parameters[:w__delta__bc        ] = 1
  data.Parameters[:w__delta__f__bc     ] = 1
  data.Parameters[:w__eta__bc          ] = 1
  data.Parameters[:w__eta__dot__bc     ] = 1
  data.Parameters[:w__h__bc            ] = 1
  data.Parameters[:w__h__dot__bc       ] = 1
  data.Parameters[:w__n__bc            ] = 1
  data.Parameters[:w__omega__f__bc     ] = 0.0008526399802
  data.Parameters[:w__omega__r__bc     ] = 0.001003172052
  data.Parameters[:w__phi__bc          ] = 1
  data.Parameters[:w__phi__dot__bc     ] = 1
  data.Parameters[:w__phi__f__bc       ] = 1
  data.Parameters[:w__s__f__bc         ] = 1
  data.Parameters[:w__tau__bc          ] = 1
  data.Parameters[:w__theta__bc        ] = 1
  data.Parameters[:w__u__bc            ] = 1/100.00
  data.Parameters[:w__v__bc            ] = 1
  data.Parameters[:w__x__f__bc         ] = 1
  data.Parameters[:w__x__r__bc         ] = 1
  data.Parameters[:w__xi__bc           ] = 1
  data.Parameters[:w__y__f__bc         ] = 1
  data.Parameters[:w__y__r__bc         ] = 1
  data.Parameters[:w__z__f__bc         ] = 1
  data.Parameters[:w__z__r__bc         ] = 1
  data.Parameters[:x__f__ss            ] = 0.860894602131645
  data.Parameters[:x__r__ss            ] = 0.516577362225953
  data.Parameters[:y__f__ss            ] = -0.000302588874971793
  data.Parameters[:y__r__ss            ] = -0.000118829564865676
  data.Parameters[:z__f__ss            ] = -0.0126250813674318
  data.Parameters[:z__r__ss            ] = -0.0106454677881745
  data.Parameters[:w__delta__dot__bc   ] = 1
  data.Parameters[:w__delta__f__dot__bc] = 1
  data.Parameters[:w__phi__f__dot__bc  ] = 1
  data.Parameters[:w__s__f__dot__bc    ] = 1
  data.Parameters[:w__theta__dot__bc   ] = 1
  data.Parameters[:w__x__f__dot__bc    ] = 1
  data.Parameters[:w__x__r__dot__bc    ] = 1
  data.Parameters[:w__y__f__dot__bc    ] = 1
  data.Parameters[:w__y__r__dot__bc    ] = 1
  data.Parameters[:w__z__f__dot__bc    ] = 1
  data.Parameters[:w__z__r__dot__bc    ] = 1

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
  data.Parameters[:e__b0 ] = e__b0
  data.Parameters[:e__b1 ] = 0.05
  data.Parameters[:eps_c0] = eps_c0
  data.Parameters[:eps_c1] = 10E-4
  data.Parameters[:eps_p0] = eps_p0
  data.Parameters[:eps_p1] = 10E-4
  data.Parameters[:h_c0  ] = h_c0
  data.Parameters[:h_c1  ] = 0.01
  data.Parameters[:p__b0 ] = p__b0
  data.Parameters[:p__b1 ] = 0.05
  data.Parameters[:tol_c0] = tol_c0
  data.Parameters[:tol_c1] = 0.05
  data.Parameters[:tol_p0] = tol_p0
  data.Parameters[:tol_p1] = 0.01
  data.Parameters[:w__ic0] = w__ic0
  data.Parameters[:w__ic1] = 0
  data.Parameters[:w__n00] = w__n00
  data.Parameters[:w__n01] = 0
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
    :active    => false
  }
  # PenaltyBarrier1DGreaterThan
  data.Constraints[:OnlyBrakingRear] = {
    :subType   => 'PENALTY_REGULAR',
    :epsilon   => e__b,
    :tolerance => p__b,
    :active    => false
  }
  # PenaltyBarrier1DGreaterThan
  data.Constraints[:OnlyTractionRear] = {
    :subType   => 'PENALTY_REGULAR',
    :epsilon   => e__b,
    :tolerance => p__b,
    :active    => false
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
    :min       => -1,
    :max       => 1,
    :active    => true
  }
  # PenaltyBarrier1DInterval
  data.Constraints[:LongSlipRear] = {
    :subType   => 'PENALTY_REGULAR',
    :epsilon   => eps_p,
    :tolerance => tol_p,
    :min       => -1,
    :max       => 1,
    :active    => true
  }
  # PenaltyBarrier1DInterval
  data.Constraints[:LatSlipFront] = {
    :subType   => 'PENALTY_REGULAR',
    :epsilon   => eps_p,
    :tolerance => tol_p,
    :min       => -1,
    :max       => 1,
    :active    => true
  }
  # PenaltyBarrier1DInterval
  data.Constraints[:LatSlipRear] = {
    :subType   => 'PENALTY_REGULAR',
    :epsilon   => eps_p,
    :tolerance => tol_p,
    :min       => -1,
    :max       => 1,
    :active    => true
  }
  # PenaltyBarrier1DInterval
  data.Constraints[:MaxSteerAngle] = {
    :subType   => 'PENALTY_REGULAR',
    :epsilon   => eps_p,
    :tolerance => tol_p,
    :min       => -1,
    :max       => 1,
    :active    => true
  }
  # PenaltyBarrier1DInterval
  data.Constraints[:MaxRollAngle] = {
    :subType   => 'PENALTY_REGULAR',
    :epsilon   => eps_p,
    :tolerance => tol_p,
    :min       => -1,
    :max       => 1,
    :active    => false
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
  require_relative('../../../Custom_Tracks/Adria_Track/circuit-Adria.rb',__FILE__)


end

# EOF

#!/usr/bin/env ruby
############################################################################
#                                                                          #
#  file: Straight_ffi_structs.rb                                           #
#                                                                          #
#  version: 1.0   date 23/6/2020                                           #
#                                                                          #
#  Copyright (C) 2020                                                      #
#                                                                          #
#      Enrico Bertolazzi and Francesco Biral and Paolo Bosetti             #
#      Dipartimento di Ingegneria Industriale                              #
#      Universita` degli Studi di Trento                                   #
#      Via Mesiano 77, I-38050 Trento, Italy                               #
#      email: enrico.bertolazzi@ing.unitn.it                               #
#             francesco.biral@ing.unitn.it                                 #
#             paolo.bosetti@ing.unitn.it                                   #
#                                                                          #
############################################################################

module Straight

  typedef :double, :data_t
  typedef :uint32, :index_t
  typedef :int,    :retcode

  class Straight_solver_params < FFI::Struct
    layout(
      :max_iter,             :index_t,
      :max_step_iter,        :index_t,
      :max_accumulated_iter, :index_t,
      :tolerance,            :data_t,
    )
    def initialize
      self[:max_iter]      = 500
      self[:max_step_iter] = 20
      self[:tolerance]     = 1e-10
    end
  end

  class Straight_model_params < FFI::Struct
    layout(

      :CXZ,               :data_t,

      :Ca,                :data_t,

      :Cp__f,             :data_t,

      :Cp__r,             :data_t,

      :Ftr__i,            :data_t,

      :Ftr__ss,           :data_t,

      :Fz0,               :data_t,

      :Fz0__f,            :data_t,

      :Fz0__r,            :data_t,

      :Fzmin,             :data_t,

      :IX,                :data_t,

      :IY,                :data_t,

      :IZ,                :data_t,

      :Id__wf,            :data_t,

      :Ix__rdr,           :data_t,

      :Ix__swa,           :data_t,

      :Iy__swa,           :data_t,

      :Iy__wf,            :data_t,

      :Iy__wr,            :data_t,

      :Iz__rdr,           :data_t,

      :Iz__swa,           :data_t,

      :Kp__f,             :data_t,

      :Kp__r,             :data_t,

      :L__b,              :data_t,

      :L__swa,            :data_t,

      :M__tot,            :data_t,

      :Mbf__i,            :data_t,

      :Mbf__ss,           :data_t,

      :Mbr__i,            :data_t,

      :Mbr__ss,           :data_t,

      :XG,                :data_t,

      :ZG,                :data_t,

      :a__1,              :data_t,

      :braking,           :data_t,

      :c__fs,             :data_t,

      :c__rs,             :data_t,

      :d1__f,             :data_t,

      :d1__r,             :data_t,

      :d2__f,             :data_t,

      :d2__r,             :data_t,

      :d3__f,             :data_t,

      :d3__r,             :data_t,

      :d4__f,             :data_t,

      :d4__r,             :data_t,

      :d5__f,             :data_t,

      :d5__r,             :data_t,

      :d6__f,             :data_t,

      :d6__r,             :data_t,

      :d7__f,             :data_t,

      :d7__r,             :data_t,

      :d8,                :data_t,

      :e10,               :data_t,

      :e1__f,             :data_t,

      :e1__r,             :data_t,

      :e2__f,             :data_t,

      :e2__r,             :data_t,

      :e4,                :data_t,

      :e5,                :data_t,

      :e6,                :data_t,

      :e7,                :data_t,

      :e9,                :data_t,

      :eps_c0,            :data_t,

      :eps_c1,            :data_t,

      :eps_p0,            :data_t,

      :eps_p1,            :data_t,

      :epsilon,           :data_t,

      :eta__00,           :data_t,

      :eta__i,            :data_t,

      :eta__ss,           :data_t,

      :g,                 :data_t,

      :h__i,              :data_t,

      :h__rdr,            :data_t,

      :h__ss,             :data_t,

      :k__fs,             :data_t,

      :k__rs,             :data_t,

      :m__rdr,            :data_t,

      :m__swa,            :data_t,

      :m__wf,             :data_t,

      :m__wr,             :data_t,

      :n__e,              :data_t,

      :n__i,              :data_t,

      :n__ref,            :data_t,

      :n__ss,             :data_t,

      :pCx1__f,           :data_t,

      :pCx1__r,           :data_t,

      :pDx1__f,           :data_t,

      :pDx1__r,           :data_t,

      :pDx2__f,           :data_t,

      :pDx2__r,           :data_t,

      :pKx1__f,           :data_t,

      :pKx1__r,           :data_t,

      :pKx2__f,           :data_t,

      :pKx2__r,           :data_t,

      :pKx3__f,           :data_t,

      :pKx3__r,           :data_t,

      :phi__e,            :data_t,

      :phi__i,            :data_t,

      :phi__ss,           :data_t,

      :rBx1__f,           :data_t,

      :rBx1__r,           :data_t,

      :rBx2__f,           :data_t,

      :rBx2__r,           :data_t,

      :rBx3__f,           :data_t,

      :rBx3__r,           :data_t,

      :rBy1__f,           :data_t,

      :rBy1__r,           :data_t,

      :rBy2__f,           :data_t,

      :rBy2__r,           :data_t,

      :rBy3__f,           :data_t,

      :rBy3__r,           :data_t,

      :rCx1__f,           :data_t,

      :rCx1__r,           :data_t,

      :rCy1__f,           :data_t,

      :rCy1__r,           :data_t,

      :rHx1__f,           :data_t,

      :rHx1__r,           :data_t,

      :rHy1__f,           :data_t,

      :rHy1__r,           :data_t,

      :rHy2__f,           :data_t,

      :rHy2__r,           :data_t,

      :r__crw,            :data_t,

      :r__pin,            :data_t,

      :rf,                :data_t,

      :rr,                :data_t,

      :rtf,               :data_t,

      :rtr,               :data_t,

      :s__f__i,           :data_t,

      :s__fs,             :data_t,

      :tau__i,            :data_t,

      :tau__ss,           :data_t,

      :tol_c0,            :data_t,

      :tol_c1,            :data_t,

      :tol_p0,            :data_t,

      :tol_p1,            :data_t,

      :u__e,              :data_t,

      :u__i,              :data_t,

      :u__ss,             :data_t,

      :v__i,              :data_t,

      :v__ss,             :data_t,

      :w__may,            :data_t,

      :w__n,              :data_t,

      :w__ss,             :data_t,

      :w__ss0,            :data_t,

      :w__ss1,            :data_t,

      :w__t,              :data_t,

      :w__t0,             :data_t,

      :w__t1,             :data_t,

      :x__a,              :data_t,

      :x__f__i,           :data_t,

      :x__off,            :data_t,

      :x__r__i,           :data_t,

      :x__rdr,            :data_t,

      :xi__e,             :data_t,

      :xi__i,             :data_t,

      :xi__n,             :data_t,

      :xi__ss,            :data_t,

      :y__f__i,           :data_t,

      :y__r__i,           :data_t,

      :z__a,              :data_t,

      :z__f__i,           :data_t,

      :z__r__i,           :data_t,

      :z__rdr,            :data_t,

      :C__delta,          :data_t,

      :Cxz__delta,        :data_t,

      :Cxz__swa,          :data_t,

      :Ix__delta,         :data_t,

      :Iy__delta,         :data_t,

      :Iz__delta,         :data_t,

      :Mb__norm,          :data_t,

      :Mbf__max,          :data_t,

      :Mbr__max,          :data_t,

      :Omega__i,          :data_t,

      :Omega__ss,         :data_t,

      :closed_gas,        :data_t,

      :delta__dot__i,     :data_t,

      :delta__dot__ss,    :data_t,

      :delta__f__i,       :data_t,

      :delta__f__ss,      :data_t,

      :delta__i,          :data_t,

      :delta__ss,         :data_t,

      :epsilon__x__f,     :data_t,

      :epsilon__x__r,     :data_t,

      :eta__dot__i,       :data_t,

      :eta__dot__ss,      :data_t,

      :h__dot__i,         :data_t,

      :h__dot__ss,        :data_t,

      :lambda__C__x__f,   :data_t,

      :lambda__C__x__r,   :data_t,

      :lambda__K__x__f,   :data_t,

      :lambda__K__x__r,   :data_t,

      :m__delta,          :data_t,

      :not_braking,       :data_t,

      :omega__f__i,       :data_t,

      :omega__f__ss,      :data_t,

      :omega__n,          :data_t,

      :omega__r__i,       :data_t,

      :omega__r__ss,      :data_t,

      :open_gas,          :data_t,

      :phi__dot__i,       :data_t,

      :phi__dot__ss,      :data_t,

      :phi__f__dot__i,    :data_t,

      :phi__f__dot__ss,   :data_t,

      :phi__f__i,         :data_t,

      :phi__f__ss,        :data_t,

      :s__f__00,          :data_t,

      :s__f__dot__i,      :data_t,

      :s__f__dot__ss,     :data_t,

      :s__f__ss,          :data_t,

      :stering_norm,      :data_t,

      :tau__m__f,         :data_t,

      :tau__m__r,         :data_t,

      :tau__m__s,         :data_t,

      :tau__m__t,         :data_t,

      :tau__max,          :data_t,

      :theta__d__00,      :data_t,

      :theta__dot__i,     :data_t,

      :theta__dot__ss,    :data_t,

      :theta__i,          :data_t,

      :theta__ss,         :data_t,

      :x__Swing,          :data_t,

      :x__delta,          :data_t,

      :x__f__dot__i,      :data_t,

      :x__f__dot__ss,     :data_t,

      :x__f__ss,          :data_t,

      :x__r__dot__i,      :data_t,

      :x__r__dot__ss,     :data_t,

      :x__r__ss,          :data_t,

      :y__f__dot__i,      :data_t,

      :y__f__dot__ss,     :data_t,

      :y__f__ss,          :data_t,

      :y__r__dot__i,      :data_t,

      :y__r__dot__ss,     :data_t,

      :y__r__ss,          :data_t,

      :z__Swing,          :data_t,

      :z__delta,          :data_t,

      :z__f__dot__i,      :data_t,

      :z__f__dot__ss,     :data_t,

      :z__f__ss,          :data_t,

      :z__r__dot__i,      :data_t,

      :z__r__dot__ss,     :data_t,

      :z__r__ss,          :data_t,

      :delta__f__dot__i,  :data_t,

      :delta__f__dot__ss, :data_t,

      :lambda__mu__x__f,  :data_t,

      :lambda__mu__x__r,  :data_t,

    )

    def initialize
      members.each { |m| self[m] = Float::NAN }
      # Custom initializations go here:
      # self[:key] = value
    end
  end

  class Straight_BC_params < FFI::Struct
    layout(

      :initial_u,             :bool,

      :initial_v,             :bool,

      :initial_Omega,         :bool,

      :initial_phi,           :bool,

      :initial_theta,         :bool,

      :initial_h,             :bool,

      :initial_delta,         :bool,

      :initial_eta,           :bool,

      :initial_s__f,          :bool,

      :initial_x__f,          :bool,

      :initial_y__f,          :bool,

      :initial_z__f,          :bool,

      :initial_x__r,          :bool,

      :initial_y__r,          :bool,

      :initial_z__r,          :bool,

      :initial_delta__f,      :bool,

      :initial_phi__f,        :bool,

      :initial_omega__r,      :bool,

      :initial_omega__f,      :bool,

      :initial_phi__dot,      :bool,

      :initial_theta__dot,    :bool,

      :initial_h__dot,        :bool,

      :initial_delta__dot,    :bool,

      :initial_eta__dot,      :bool,

      :initial_s__f__dot,     :bool,

      :initial_x__f__dot,     :bool,

      :initial_y__f__dot,     :bool,

      :initial_z__f__dot,     :bool,

      :initial_x__r__dot,     :bool,

      :initial_y__r__dot,     :bool,

      :initial_z__r__dot,     :bool,

      :initial_delta__f__dot, :bool,

      :initial_phi__f__dot,   :bool,

      :initial_Ftr,           :bool,

      :initial_Mbf,           :bool,

      :initial_Mbr,           :bool,

      :initial_tau,           :bool,

      :initial_n,             :bool,

      :initial_xi,            :bool,

      :final_u,               :bool,

      :final_phi,             :bool,

      :final_n,               :bool,

      :final_xi,              :bool,

    )

    def initialize
      members.each { |m| self[m] = true }
      # Custom initializations go here:
      # self[:key] = value
    end
  end

  class Straight_constraints_params < FFI::Struct
    layout(
      # 1D constraints
      :OnlyBrakingFrontSubType,         :index_t,
      :OnlyBrakingFrontEpsilon,         :data_t,
      :OnlyBrakingFrontTolerance,       :data_t,
      :OnlyBrakingRearSubType,          :index_t,
      :OnlyBrakingRearEpsilon,          :data_t,
      :OnlyBrakingRearTolerance,        :data_t,
      :OnlyTractionRearSubType,         :index_t,
      :OnlyTractionRearEpsilon,         :data_t,
      :OnlyTractionRearTolerance,       :data_t,
      :FrontWheelContactSubType,        :index_t,
      :FrontWheelContactEpsilon,        :data_t,
      :FrontWheelContactTolerance,      :data_t,
      :RearWheelContactSubType,         :index_t,
      :RearWheelContactEpsilon,         :data_t,
      :RearWheelContactTolerance,       :data_t,
      :LongSlipFrontSubType,            :index_t,
      :LongSlipFrontEpsilon,            :data_t,
      :LongSlipFrontTolerance,          :data_t,
      :LongSlipFrontMinValue,           :data_t,
      :LongSlipFrontMaxValue,           :data_t,
      :LongSlipRearSubType,             :index_t,
      :LongSlipRearEpsilon,             :data_t,
      :LongSlipRearTolerance,           :data_t,
      :LongSlipRearMinValue,            :data_t,
      :LongSlipRearMaxValue,            :data_t,
      :LatSlipFrontSubType,             :index_t,
      :LatSlipFrontEpsilon,             :data_t,
      :LatSlipFrontTolerance,           :data_t,
      :LatSlipFrontMinValue,            :data_t,
      :LatSlipFrontMaxValue,            :data_t,
      :LatSlipRearSubType,              :index_t,
      :LatSlipRearEpsilon,              :data_t,
      :LatSlipRearTolerance,            :data_t,
      :LatSlipRearMinValue,             :data_t,
      :LatSlipRearMaxValue,             :data_t,
      :MaxSteerAngleSubType,            :index_t,
      :MaxSteerAngleEpsilon,            :data_t,
      :MaxSteerAngleTolerance,          :data_t,
      :MaxSteerAngleMinValue,           :data_t,
      :MaxSteerAngleMaxValue,           :data_t,
      :MaxRollAngleSubType,             :index_t,
      :MaxRollAngleEpsilon,             :data_t,
      :MaxRollAngleTolerance,           :data_t,
      :MaxRollAngleMinValue,            :data_t,
      :MaxRollAngleMaxValue,            :data_t,
      :roadRightLateralBorderSubType,   :index_t,
      :roadRightLateralBorderEpsilon,   :data_t,
      :roadRightLateralBorderTolerance, :data_t,
      :roadLeftLateralBorderSubType,    :index_t,
      :roadLeftLateralBorderEpsilon,    :data_t,
      :roadLeftLateralBorderTolerance,  :data_t,

      # 2D constraints

      # Controls
      :t__oControlType,         :index_t,
      :t__oControlEpsilon,      :data_t,
      :t__oControlTolerance,    :data_t,
      :b__f__oControlType,      :index_t,
      :b__f__oControlEpsilon,   :data_t,
      :b__f__oControlTolerance, :data_t,
      :b__r__oControlType,      :index_t,
      :b__r__oControlEpsilon,   :data_t,
      :b__r__oControlTolerance, :data_t,
      :tau__oControlType,       :index_t,
      :tau__oControlEpsilon,    :data_t,
      :tau__oControlTolerance,  :data_t,
    )

    def initialize
      members.each do |m|
        case self[m]
        when Float
          self[m] = Float::NAN
        when Fixnum
          self[m] = 0
        when FFI::Pointer
          self[m] = nil
        else
          warn "Unmanaged initialization type in struct (for field #{m} of type #{self[m].class})"
        end
      end
      # Custom initializations go here:
      # self[:key] = value
    end
  end

  attach_function :setup_model,                          # ruby name
                  :Straight_setup_model,        # C name
                  [:pointer ,:pointer ,:pointer ],
                  :void

  attach_function :setup_solver,                         # ruby name
                  :Straight_setup_solver,       # C name
                  [:pointer ],
                  :void

  attach_function :write_solution_to_file,                   # ruby name
                  :Straight_write_solution_to_file, # C name
                  [:string],
                  :void

  attach_function :printout_enabled?,                       # ruby name
                  :Straight_printout_is_enabled,   # C name
                  [],
                  :int

  attach_function :enable_printout,                      # ruby name
                  :Straight_enable_printout,    # C name
                  [],
                  :void

  attach_function :disable_printout,                     # ruby name
                  :Straight_disable_printout,   # C name
                  [],
                  :void

  attach_function :reset_multiplier,                     # ruby name
                  :Straight_reset_multiplier,   # C name
                  [],
                  :void

  attach_function :reset_BC_multiplier,                     # ruby name
                  :Straight_reset_BC_multiplier,   # C name
                  [],
                  :void

  attach_function :set_internal_guess,                     # ruby name
                  :Straight_set_internal_guess,   # C name
                  [],
                  :void

end

# EOF: Straight_ffi_stucts.rb

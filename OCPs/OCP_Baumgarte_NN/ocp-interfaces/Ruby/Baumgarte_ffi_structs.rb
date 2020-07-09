#!/usr/bin/env ruby
############################################################################
#                                                                          #
#  file: Baumgarte_ffi_structs.rb                                          #
#                                                                          #
#  version: 1.0   date 2/7/2020                                            #
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

module Baumgarte

  typedef :double, :data_t
  typedef :uint32, :index_t
  typedef :int,    :retcode

  class Baumgarte_solver_params < FFI::Struct
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

  class Baumgarte_model_params < FFI::Struct
    layout(

      :CXZ,                     :data_t,

      :Ca,                      :data_t,

      :Cp__f,                   :data_t,

      :Cp__r,                   :data_t,

      :Ftr__ss,                 :data_t,

      :Fz0,                     :data_t,

      :Fz0__f,                  :data_t,

      :Fz0__r,                  :data_t,

      :Fzmin,                   :data_t,

      :Fznorm,                  :data_t,

      :IX,                      :data_t,

      :IY,                      :data_t,

      :IZ,                      :data_t,

      :Id__wf,                  :data_t,

      :Ix__rdr,                 :data_t,

      :Ix__swa,                 :data_t,

      :Iy__swa,                 :data_t,

      :Iy__wf,                  :data_t,

      :Iy__wr,                  :data_t,

      :Iz__rdr,                 :data_t,

      :Iz__swa,                 :data_t,

      :Kp__f,                   :data_t,

      :Kp__r,                   :data_t,

      :L__b,                    :data_t,

      :L__swa,                  :data_t,

      :M__tot,                  :data_t,

      :Mbf__ss,                 :data_t,

      :Mbr__ss,                 :data_t,

      :XG,                      :data_t,

      :ZG,                      :data_t,

      :a__1,                    :data_t,

      :braking,                 :data_t,

      :c__fs,                   :data_t,

      :c__rs,                   :data_t,

      :d1__f,                   :data_t,

      :d1__r,                   :data_t,

      :d2__f,                   :data_t,

      :d2__r,                   :data_t,

      :d3__f,                   :data_t,

      :d3__r,                   :data_t,

      :d4__f,                   :data_t,

      :d4__r,                   :data_t,

      :d5__f,                   :data_t,

      :d5__r,                   :data_t,

      :d6__f,                   :data_t,

      :d6__r,                   :data_t,

      :d7__f,                   :data_t,

      :d7__r,                   :data_t,

      :d8,                      :data_t,

      :e10,                     :data_t,

      :e1__f,                   :data_t,

      :e1__r,                   :data_t,

      :e2__f,                   :data_t,

      :e2__r,                   :data_t,

      :e4,                      :data_t,

      :e5,                      :data_t,

      :e6,                      :data_t,

      :e7,                      :data_t,

      :e9,                      :data_t,

      :eps_c0,                  :data_t,

      :eps_c1,                  :data_t,

      :eps_p0,                  :data_t,

      :eps_p1,                  :data_t,

      :epsilon,                 :data_t,

      :eta__00,                 :data_t,

      :eta__ss,                 :data_t,

      :g,                       :data_t,

      :h__rdr,                  :data_t,

      :h__ss,                   :data_t,

      :h_c,                     :data_t,

      :k__fs,                   :data_t,

      :k__rs,                   :data_t,

      :m__rdr,                  :data_t,

      :m__swa,                  :data_t,

      :m__wf,                   :data_t,

      :m__wr,                   :data_t,

      :n__ss,                   :data_t,

      :pCx1__f,                 :data_t,

      :pCx1__r,                 :data_t,

      :pDx1__f,                 :data_t,

      :pDx1__r,                 :data_t,

      :pDx2__f,                 :data_t,

      :pDx2__r,                 :data_t,

      :pKx1__f,                 :data_t,

      :pKx1__r,                 :data_t,

      :pKx2__f,                 :data_t,

      :pKx2__r,                 :data_t,

      :pKx3__f,                 :data_t,

      :pKx3__r,                 :data_t,

      :phi__ss,                 :data_t,

      :rBx1__f,                 :data_t,

      :rBx1__r,                 :data_t,

      :rBx2__f,                 :data_t,

      :rBx2__r,                 :data_t,

      :rBx3__f,                 :data_t,

      :rBx3__r,                 :data_t,

      :rBy1__f,                 :data_t,

      :rBy1__r,                 :data_t,

      :rBy2__f,                 :data_t,

      :rBy2__r,                 :data_t,

      :rBy3__f,                 :data_t,

      :rBy3__r,                 :data_t,

      :rCx1__f,                 :data_t,

      :rCx1__r,                 :data_t,

      :rCy1__f,                 :data_t,

      :rCy1__r,                 :data_t,

      :rHx1__f,                 :data_t,

      :rHx1__r,                 :data_t,

      :rHy1__f,                 :data_t,

      :rHy1__r,                 :data_t,

      :rHy2__f,                 :data_t,

      :rHy2__r,                 :data_t,

      :r__crw,                  :data_t,

      :r__pin,                  :data_t,

      :rf,                      :data_t,

      :rr,                      :data_t,

      :rtf,                     :data_t,

      :rtr,                     :data_t,

      :s__fs,                   :data_t,

      :tau__ss,                 :data_t,

      :tol_c0,                  :data_t,

      :tol_c1,                  :data_t,

      :tol_p0,                  :data_t,

      :tol_p1,                  :data_t,

      :u__ss,                   :data_t,

      :v__ss,                   :data_t,

      :w__LR,                   :data_t,

      :w__ic,                   :data_t,

      :w__ic0,                  :data_t,

      :w__ic1,                  :data_t,

      :w__n0,                   :data_t,

      :w__n00,                  :data_t,

      :w__n01,                  :data_t,

      :w__ss,                   :data_t,

      :w__ss0,                  :data_t,

      :w__ss1,                  :data_t,

      :w__t,                    :data_t,

      :w__t0,                   :data_t,

      :w__t1,                   :data_t,

      :x__a,                    :data_t,

      :x__off,                  :data_t,

      :x__rdr,                  :data_t,

      :xi__n,                   :data_t,

      :xi__ss,                  :data_t,

      :z__a,                    :data_t,

      :z__rdr,                  :data_t,

      :C__delta,                :data_t,

      :Cxz__delta,              :data_t,

      :Cxz__swa,                :data_t,

      :Ix__delta,               :data_t,

      :Iy__delta,               :data_t,

      :Iz__delta,               :data_t,

      :Mbf__max,                :data_t,

      :Mbr__max,                :data_t,

      :Omega__ss,               :data_t,

      :beta__max,               :data_t,

      :closed_gas,              :data_t,

      :delta__dot__ss,          :data_t,

      :delta__f__ss,            :data_t,

      :delta__max,              :data_t,

      :delta__ss,               :data_t,

      :epsilon__x__f,           :data_t,

      :epsilon__x__r,           :data_t,

      :eta__dot__ss,            :data_t,

      :h__dot__ss,              :data_t,

      :lambda__C__x__f,         :data_t,

      :lambda__C__x__r,         :data_t,

      :lambda__K__x__f,         :data_t,

      :lambda__K__x__r,         :data_t,

      :lat_peak_front,          :data_t,

      :lat_peak_rear,           :data_t,

      :long_peak_front,         :data_t,

      :long_peak_rear,          :data_t,

      :m__delta,                :data_t,

      :not_braking,             :data_t,

      :omega__f__ss,            :data_t,

      :omega__n,                :data_t,

      :omega__r__ss,            :data_t,

      :open_gas,                :data_t,

      :phi__dot__ss,            :data_t,

      :phi__f__dot__ss,         :data_t,

      :phi__f__ss,              :data_t,

      :phi__max,                :data_t,

      :s__f__00,                :data_t,

      :s__f__dot__ss,           :data_t,

      :s__f__ss,                :data_t,

      :stering_norm,            :data_t,

      :tau__m__f,               :data_t,

      :tau__m__r,               :data_t,

      :tau__m__s,               :data_t,

      :tau__m__t,               :data_t,

      :tau__max,                :data_t,

      :theta__d__00,            :data_t,

      :theta__dot__ss,          :data_t,

      :theta__ss,               :data_t,

      :w__Ftr__bc,              :data_t,

      :w__Mbf__bc,              :data_t,

      :w__Mbr__bc,              :data_t,

      :w__Omega__bc,            :data_t,

      :w__delta__bc,            :data_t,

      :w__delta__f__bc,         :data_t,

      :w__eta__bc,              :data_t,

      :w__eta__dot__bc,         :data_t,

      :w__eta__sserr,           :data_t,

      :w__h__bc,                :data_t,

      :w__h__dot__bc,           :data_t,

      :w__h__sserr,             :data_t,

      :w__n__bc,                :data_t,

      :w__omega__f__bc,         :data_t,

      :w__omega__r__bc,         :data_t,

      :w__phi__bc,              :data_t,

      :w__phi__dot__bc,         :data_t,

      :w__phi__f__bc,           :data_t,

      :w__s__f__bc,             :data_t,

      :w__s__f__sserr,          :data_t,

      :w__tau__bc,              :data_t,

      :w__theta__bc,            :data_t,

      :w__theta__sserr,         :data_t,

      :w__u__bc,                :data_t,

      :w__v__bc,                :data_t,

      :w__x__f__bc,             :data_t,

      :w__x__r__bc,             :data_t,

      :w__xi__bc,               :data_t,

      :w__y__f__bc,             :data_t,

      :w__y__r__bc,             :data_t,

      :w__z__f__bc,             :data_t,

      :w__z__r__bc,             :data_t,

      :x__Swing,                :data_t,

      :x__delta,                :data_t,

      :x__f__dot__ss,           :data_t,

      :x__f__ss,                :data_t,

      :x__r__dot__ss,           :data_t,

      :x__r__ss,                :data_t,

      :y__f__dot__ss,           :data_t,

      :y__f__ss,                :data_t,

      :y__r__dot__ss,           :data_t,

      :y__r__ss,                :data_t,

      :z__Swing,                :data_t,

      :z__delta,                :data_t,

      :z__f__dot__ss,           :data_t,

      :z__f__ss,                :data_t,

      :z__r__dot__ss,           :data_t,

      :z__r__ss,                :data_t,

      :delta__f__dot__ss,       :data_t,

      :lambda__mu__x__f,        :data_t,

      :lambda__mu__x__r,        :data_t,

      :long_peak_front_0,       :data_t,

      :long_peak_front_1,       :data_t,

      :long_peak_rear_0,        :data_t,

      :long_peak_rear_1,        :data_t,

      :w__delta__dot__bc,       :data_t,

      :w__delta__dot__sserr,    :data_t,

      :w__delta__f__dot__bc,    :data_t,

      :w__delta__f__dot__sserr, :data_t,

      :w__eta__dot__sserr,      :data_t,

      :w__h__dot__sserr,        :data_t,

      :w__phi__dot__sserr,      :data_t,

      :w__phi__f__dot__bc,      :data_t,

      :w__phi__f__dot__sserr,   :data_t,

      :w__s__f__dot__bc,        :data_t,

      :w__s__f__dot__sserr,     :data_t,

      :w__theta__dot__bc,       :data_t,

      :w__theta__dot__sserr,    :data_t,

      :w__x__f__dot__bc,        :data_t,

      :w__x__f__dot__sserr,     :data_t,

      :w__x__r__dot__bc,        :data_t,

      :w__x__r__dot__sserr,     :data_t,

      :w__y__f__dot__bc,        :data_t,

      :w__y__f__dot__sserr,     :data_t,

      :w__y__r__dot__bc,        :data_t,

      :w__y__r__dot__sserr,     :data_t,

      :w__z__f__dot__bc,        :data_t,

      :w__z__f__dot__sserr,     :data_t,

      :w__z__r__dot__bc,        :data_t,

      :w__z__r__dot__sserr,     :data_t,

    )

    def initialize
      members.each { |m| self[m] = Float::NAN }
      # Custom initializations go here:
      # self[:key] = value
    end
  end

  class Baumgarte_BC_params < FFI::Struct
    layout(

    )

    def initialize
      members.each { |m| self[m] = true }
      # Custom initializations go here:
      # self[:key] = value
    end
  end

  class Baumgarte_constraints_params < FFI::Struct
    layout(
      # 1D constraints
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
      :MaxBetaAngleSubType,             :index_t,
      :MaxBetaAngleEpsilon,             :data_t,
      :MaxBetaAngleTolerance,           :data_t,
      :MaxBetaAngleMinValue,            :data_t,
      :MaxBetaAngleMaxValue,            :data_t,
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
                  :Baumgarte_setup_model,        # C name
                  [:pointer ,:pointer ,:pointer ],
                  :void

  attach_function :setup_solver,                         # ruby name
                  :Baumgarte_setup_solver,       # C name
                  [:pointer ],
                  :void

  attach_function :write_solution_to_file,                   # ruby name
                  :Baumgarte_write_solution_to_file, # C name
                  [:string],
                  :void

  attach_function :printout_enabled?,                       # ruby name
                  :Baumgarte_printout_is_enabled,   # C name
                  [],
                  :int

  attach_function :enable_printout,                      # ruby name
                  :Baumgarte_enable_printout,    # C name
                  [],
                  :void

  attach_function :disable_printout,                     # ruby name
                  :Baumgarte_disable_printout,   # C name
                  [],
                  :void

  attach_function :reset_multiplier,                     # ruby name
                  :Baumgarte_reset_multiplier,   # C name
                  [],
                  :void

  attach_function :reset_BC_multiplier,                     # ruby name
                  :Baumgarte_reset_BC_multiplier,   # C name
                  [],
                  :void

  attach_function :set_internal_guess,                     # ruby name
                  :Baumgarte_set_internal_guess,   # C name
                  [],
                  :void

end

# EOF: Baumgarte_ffi_stucts.rb

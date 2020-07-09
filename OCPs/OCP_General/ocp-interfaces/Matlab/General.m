%-----------------------------------------------------------------------%
%  file: General.m                                                      %
%                                                                       %
%  version: 1.0   date 29/6/2020                                        %
%                                                                       %
%  Copyright (C) 2020                                                   %
%                                                                       %
%      Enrico Bertolazzi, Francesco Biral and Paolo Bosetti             %
%      Dipartimento di Ingegneria Industriale                           %
%      Universita` degli Studi di Trento                                %
%      Via Sommarive 9, I-38123, Trento, Italy                          %
%      email: enrico.bertolazzi@unitn.it                                %
%             francesco.biral@unitn.it                                  %
%             paolo.bosetti@unitn.it                                    %
%-----------------------------------------------------------------------%


classdef General < handle
  properties (SetAccess = private, Hidden = true)
    objectHandle; % Handle to the underlying C++ class instance
  end

  methods

    function self = General( name )
      self.objectHandle = General_Mex( 'new', name );
    end

    function delete( self )
      %% Destroy the C++ class instance
      General_Mex( 'delete', self.objectHandle );
    end

    function help( self )
      General_Mex('help');
    end

    % -------------------------------------------------------------------------
    % INITIALIZATION
    % -------------------------------------------------------------------------
    function data = read( self, fname )
      data = General_Mex( 'read', self.objectHandle, fname );
    end

    function setup( self, fname_or_struct )
      General_Mex( 'setup', self.objectHandle, fname_or_struct );
    end

    function n = names( self )
      n = General_Mex( 'names', self.objectHandle );
    end

    function res = dims( self )
      res = General_Mex( 'dims', self.objectHandle );
    end

    function res = get_ocp_data( self )
      res = General_Mex( 'get_ocp_data', self.objectHandle );
    end

    % -------------------------------------------------------------------------
    % INFO LEVEL
    % -------------------------------------------------------------------------
    function infoLevel( self, infoLvl )
      General_Mex( 'infoLevel', self.objectHandle, infoLvl );
    end

    % -------------------------------------------------------------------------
    % GUESS
    % -------------------------------------------------------------------------
    function set_guess( self, varargin )
      General_Mex( 'set_guess', self.objectHandle, varargin{:} );
    end
    function guess = get_guess( self )
      guess = General_Mex( 'get_guess', self.objectHandle );
    end
    function guess = get_solution_as_guess( self )
      guess = General_Mex( 'get_solution_as_guess', self.objectHandle );
    end

    % -------------------------------------------------------------------------
    % SOLVE
    % -------------------------------------------------------------------------
    function ok = solve( self )
      % ok = false if computation failed
      % ok = true if computation is succesfull
      ok = General_Mex( 'solve', self.objectHandle );
    end

    function updateContinuation( self, n, s )
      General_Mex( 'updateContinuation', self.objectHandle, n, s );
    end

    % -------------------------------------------------------------------------
    % GET SOLUTION
    % -------------------------------------------------------------------------
    function sol = solution( self, varargin )
      sol = General_Mex( 'get_solution', self.objectHandle, varargin{:} );
    end

    function sol = pack( self, X, Lambda, Pars, Omega )
      sol = General_Mex( 'pack', self.objectHandle, X, Lambda, Pars, Omega );
    end

    function [X, Lambda, Pars, Omega] = unpack( self, sol )
      [X, Lambda, Pars, Omega] = General_Mex( 'unpack', self.objectHandle, sol );
    end

    % -------------------------------------------------------------------------
    % ZETA
    % -------------------------------------------------------------------------
    function res = zeta( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'zeta' );
    end

    % -------------------------------------------------------------------------
    % STATES
    % -------------------------------------------------------------------------
    function res = u( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'u', varargin{:} );
    end
    function res = v( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'v', varargin{:} );
    end
    function res = Omega( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'Omega', varargin{:} );
    end
    function res = phi( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'phi', varargin{:} );
    end
    function res = theta( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'theta', varargin{:} );
    end
    function res = h( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'h', varargin{:} );
    end
    function res = delta( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'delta', varargin{:} );
    end
    function res = eta( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'eta', varargin{:} );
    end
    function res = s__f( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 's__f', varargin{:} );
    end
    function res = x__f( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'x__f', varargin{:} );
    end
    function res = y__f( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'y__f', varargin{:} );
    end
    function res = z__f( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'z__f', varargin{:} );
    end
    function res = x__r( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'x__r', varargin{:} );
    end
    function res = y__r( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'y__r', varargin{:} );
    end
    function res = z__r( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'z__r', varargin{:} );
    end
    function res = delta__f( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'delta__f', varargin{:} );
    end
    function res = phi__f( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'phi__f', varargin{:} );
    end
    function res = omega__r( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'omega__r', varargin{:} );
    end
    function res = omega__f( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'omega__f', varargin{:} );
    end
    function res = phi__dot( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'phi__dot', varargin{:} );
    end
    function res = theta__dot( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'theta__dot', varargin{:} );
    end
    function res = h__dot( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'h__dot', varargin{:} );
    end
    function res = delta__dot( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'delta__dot', varargin{:} );
    end
    function res = eta__dot( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'eta__dot', varargin{:} );
    end
    function res = s__f__dot( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 's__f__dot', varargin{:} );
    end
    function res = x__f__dot( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'x__f__dot', varargin{:} );
    end
    function res = y__f__dot( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'y__f__dot', varargin{:} );
    end
    function res = z__f__dot( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'z__f__dot', varargin{:} );
    end
    function res = x__r__dot( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'x__r__dot', varargin{:} );
    end
    function res = y__r__dot( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'y__r__dot', varargin{:} );
    end
    function res = z__r__dot( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'z__r__dot', varargin{:} );
    end
    function res = delta__f__dot( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'delta__f__dot', varargin{:} );
    end
    function res = phi__f__dot( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'phi__f__dot', varargin{:} );
    end
    function res = Ftr( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'Ftr', varargin{:} );
    end
    function res = Mbf( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'Mbf', varargin{:} );
    end
    function res = Mbr( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'Mbr', varargin{:} );
    end
    function res = tau( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'tau', varargin{:} );
    end
    function res = n( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'n', varargin{:} );
    end
    function res = xi( self, varargin  )
      res = General_Mex( 'get_solution', self.objectHandle, 'xi', varargin{:} );
    end

    % -------------------------------------------------------------------------
    % MULTIPLIER
    % -------------------------------------------------------------------------
    function res = lambda1( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda1__xo', varargin{:} );
    end
    function res = lambda2( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda2__xo', varargin{:} );
    end
    function res = lambda3( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda3__xo', varargin{:} );
    end
    function res = lambda4( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda4__xo', varargin{:} );
    end
    function res = lambda5( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda5__xo', varargin{:} );
    end
    function res = lambda6( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda6__xo', varargin{:} );
    end
    function res = lambda7( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda7__xo', varargin{:} );
    end
    function res = lambda8( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda8__xo', varargin{:} );
    end
    function res = lambda9( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda9__xo', varargin{:} );
    end
    function res = lambda10( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda10__xo', varargin{:} );
    end
    function res = lambda11( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda11__xo', varargin{:} );
    end
    function res = lambda12( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda12__xo', varargin{:} );
    end
    function res = lambda13( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda13__xo', varargin{:} );
    end
    function res = lambda14( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda14__xo', varargin{:} );
    end
    function res = lambda15( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda15__xo', varargin{:} );
    end
    function res = lambda16( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda16__xo', varargin{:} );
    end
    function res = lambda17( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda17__xo', varargin{:} );
    end
    function res = lambda18( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda18__xo', varargin{:} );
    end
    function res = lambda19( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda19__xo', varargin{:} );
    end
    function res = lambda20( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda20__xo', varargin{:} );
    end
    function res = lambda21( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda21__xo', varargin{:} );
    end
    function res = lambda22( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda22__xo', varargin{:} );
    end
    function res = lambda23( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda23__xo', varargin{:} );
    end
    function res = lambda24( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda24__xo', varargin{:} );
    end
    function res = lambda25( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda25__xo', varargin{:} );
    end
    function res = lambda26( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda26__xo', varargin{:} );
    end
    function res = lambda27( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda27__xo', varargin{:} );
    end
    function res = lambda28( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda28__xo', varargin{:} );
    end
    function res = lambda29( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda29__xo', varargin{:} );
    end
    function res = lambda30( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda30__xo', varargin{:} );
    end
    function res = lambda31( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda31__xo', varargin{:} );
    end
    function res = lambda32( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda32__xo', varargin{:} );
    end
    function res = lambda33( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda33__xo', varargin{:} );
    end
    function res = lambda34( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda34__xo', varargin{:} );
    end
    function res = lambda35( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda35__xo', varargin{:} );
    end
    function res = lambda36( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda36__xo', varargin{:} );
    end
    function res = lambda37( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda37__xo', varargin{:} );
    end
    function res = lambda38( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda38__xo', varargin{:} );
    end
    function res = lambda39( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda39__xo', varargin{:} );
    end

    % -------------------------------------------------------------------------
    % CONTROLS
    % -------------------------------------------------------------------------
    function res = b__f__o( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'b__f__o', varargin{:} );
    end
    function res = b__r__o( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'b__r__o', varargin{:} );
    end
    function res = t__o( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 't__o', varargin{:} );
    end
    function res = tau__o( self, varargin )
      res = General_Mex( 'get_solution', self.objectHandle, 'tau__o', varargin{:} );
    end

    % -------------------------------------------------------------------------
    % POSTPROCESSING
    % -------------------------------------------------------------------------
    function res = t__oControl( self )
      res = General_Mex( 'get_solution', self.objectHandle, 't__oControl' );
    end
    function res = b__f__oControl( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'b__f__oControl' );
    end
    function res = b__r__oControl( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'b__r__oControl' );
    end
    function res = tau__oControl( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'tau__oControl' );
    end
    function res = OnlyBrakingFront( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'OnlyBrakingFront' );
    end
    function res = OnlyBrakingRear( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'OnlyBrakingRear' );
    end
    function res = OnlyTractionRear( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'OnlyTractionRear' );
    end
    function res = FrontWheelContact( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'FrontWheelContact' );
    end
    function res = RearWheelContact( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'RearWheelContact' );
    end
    function res = LongSlipFront( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'LongSlipFront' );
    end
    function res = LongSlipRear( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'LongSlipRear' );
    end
    function res = LatSlipFront( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'LatSlipFront' );
    end
    function res = LatSlipRear( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'LatSlipRear' );
    end
    function res = MaxSteerAngle( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'MaxSteerAngle' );
    end
    function res = MaxRollAngle( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'MaxRollAngle' );
    end
    function res = roadRightLateralBorder( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'roadRightLateralBorder' );
    end
    function res = roadLeftLateralBorder( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'roadLeftLateralBorder' );
    end
    function res = Fzf( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'Fzf' );
    end
    function res = Fzr( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'Fzr' );
    end
    function res = lambda__r( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda__r' );
    end
    function res = lambda__f( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'lambda__f' );
    end
    function res = alpha__r( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'alpha__r' );
    end
    function res = alpha__f( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'alpha__f' );
    end
    function res = Fxf( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'Fxf' );
    end
    function res = Fxr( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'Fxr' );
    end
    function res = Fyf( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'Fyf' );
    end
    function res = Fyr( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'Fyr' );
    end
    function res = Mzf( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'Mzf' );
    end
    function res = Mzr( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'Mzr' );
    end
    function res = Mxf( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'Mxf' );
    end
    function res = Mxr( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'Mxr' );
    end
    function res = max_torque_at_wheel( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'max_torque_at_wheel' );
    end
    function res = xISORight( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'xISORight' );
    end
    function res = yISORight( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'yISORight' );
    end
    function res = xISOLeft( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'xISOLeft' );
    end
    function res = yISOLeft( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'yISOLeft' );
    end
    function res = xISOTrajectory( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'xISOTrajectory' );
    end
    function res = yISOTrajectory( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'yISOTrajectory' );
    end
    function res = ALG__11( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'ALG__11' );
    end
    function res = ALG__12( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'ALG__12' );
    end
    function res = ALG__13( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'ALG__13' );
    end
    function res = ALG__14( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'ALG__14' );
    end
    function res = ALG__15( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'ALG__15' );
    end
    function res = ALG__16( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'ALG__16' );
    end
    function res = ALG__17( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'ALG__17' );
    end
    function res = ALG__18( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'ALG__18' );
    end
    function res = ALG__21( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'ALG__21' );
    end
    function res = ALG__22( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'ALG__22' );
    end
    function res = ALG__23( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'ALG__23' );
    end
    function res = ALG__24( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'ALG__24' );
    end
    function res = ALG__25( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'ALG__25' );
    end
    function res = ALG__26( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'ALG__26' );
    end
    function res = ALG__27( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'ALG__27' );
    end
    function res = ALG__28( self )
      res = General_Mex( 'get_solution', self.objectHandle, 'ALG__28' );
    end

    % -------------------------------------------------------------------------
    % NONLINEAR SYSTEM
    % -------------------------------------------------------------------------
    function F = eval_F( self, x )
      F = General_Mex( 'eval_F', self.objectHandle, x );
    end

    function JF = eval_JF( self, x )
      JF = General_Mex( 'eval_JF', self.objectHandle, x );
    end

    function JF = eval_JF_pattern( self )
      JF = General_Mex( 'eval_JF_pattern', self.objectHandle );
    end

    function x = get_raw_solution( self )
      x = General_Mex( 'get_raw_solution', self.objectHandle );
    end

    function set_raw_solution( self, x )
      General_Mex( 'set_raw_solution', self.objectHandle, x );
    end

    function ok = check_raw_solution( self, x )
      ok = General_Mex( 'check_raw_solution', self.objectHandle, x );
    end

    function check_jacobian( self, x, epsi )
      General_Mex( 'check_jacobian', self.objectHandle, x, epsi );
    end

    % -------------------------------------------------------------------------
    % PLOT SOLUTION
    % -------------------------------------------------------------------------
    function plot_states( self )
      plot(...
        self.zeta(), self.u(), ...
        self.zeta(), self.v(), ...
        self.zeta(), self.Omega(), ...
        self.zeta(), self.phi(), ...
        self.zeta(), self.theta(), ...
        self.zeta(), self.h(), ...
        self.zeta(), self.delta(), ...
        self.zeta(), self.eta(), ...
        self.zeta(), self.s__f(), ...
        self.zeta(), self.x__f(), ...
        self.zeta(), self.y__f(), ...
        self.zeta(), self.z__f(), ...
        self.zeta(), self.x__r(), ...
        self.zeta(), self.y__r(), ...
        self.zeta(), self.z__r(), ...
        self.zeta(), self.delta__f(), ...
        self.zeta(), self.phi__f(), ...
        self.zeta(), self.omega__r(), ...
        self.zeta(), self.omega__f(), ...
        self.zeta(), self.phi__dot(), ...
        self.zeta(), self.theta__dot(), ...
        self.zeta(), self.h__dot(), ...
        self.zeta(), self.delta__dot(), ...
        self.zeta(), self.eta__dot(), ...
        self.zeta(), self.s__f__dot(), ...
        self.zeta(), self.x__f__dot(), ...
        self.zeta(), self.y__f__dot(), ...
        self.zeta(), self.z__f__dot(), ...
        self.zeta(), self.x__r__dot(), ...
        self.zeta(), self.y__r__dot(), ...
        self.zeta(), self.z__r__dot(), ...
        self.zeta(), self.delta__f__dot(), ...
        self.zeta(), self.phi__f__dot(), ...
        self.zeta(), self.Ftr(), ...
        self.zeta(), self.Mbf(), ...
        self.zeta(), self.Mbr(), ...
        self.zeta(), self.tau(), ...
        self.zeta(), self.n(), ...
        self.zeta(), self.xi(), ...
        'Linewidth', 2 ...
      );
      title('states');
      legend( 'u', 'v', '\Omega', '\phi', 'th\eta', 'h', '\delta', '\eta', 's\_f', 'x\_f', 'y\_f', 'z\_f', 'x\_r', 'y\_r', 'z\_r', '\delta\_f', '\phi\_f', '\omega\_r', '\omega\_f', '\phi\_dot', 'th\eta\_dot', 'h\_dot', '\delta\_dot', '\eta\_dot', 's\_f\_dot', 'x\_f\_dot', 'y\_f\_dot', 'z\_f\_dot', 'x\_r\_dot', 'y\_r\_dot', 'z\_r\_dot', '\delta\_f\_dot', '\phi\_f\_dot', 'Ftr', 'Mbf', 'Mbr', '\tau', 'n', '\xi' );
    end

    function plot_multipliers( self )
      plot(...
        self.zeta(), self.lambda1(), ...
        self.zeta(), self.lambda2(), ...
        self.zeta(), self.lambda3(), ...
        self.zeta(), self.lambda4(), ...
        self.zeta(), self.lambda5(), ...
        self.zeta(), self.lambda6(), ...
        self.zeta(), self.lambda7(), ...
        self.zeta(), self.lambda8(), ...
        self.zeta(), self.lambda9(), ...
        self.zeta(), self.lambda10(), ...
        self.zeta(), self.lambda11(), ...
        self.zeta(), self.lambda12(), ...
        self.zeta(), self.lambda13(), ...
        self.zeta(), self.lambda14(), ...
        self.zeta(), self.lambda15(), ...
        self.zeta(), self.lambda16(), ...
        self.zeta(), self.lambda17(), ...
        self.zeta(), self.lambda18(), ...
        self.zeta(), self.lambda19(), ...
        self.zeta(), self.lambda20(), ...
        self.zeta(), self.lambda21(), ...
        self.zeta(), self.lambda22(), ...
        self.zeta(), self.lambda23(), ...
        self.zeta(), self.lambda24(), ...
        self.zeta(), self.lambda25(), ...
        self.zeta(), self.lambda26(), ...
        self.zeta(), self.lambda27(), ...
        self.zeta(), self.lambda28(), ...
        self.zeta(), self.lambda29(), ...
        self.zeta(), self.lambda30(), ...
        self.zeta(), self.lambda31(), ...
        self.zeta(), self.lambda32(), ...
        self.zeta(), self.lambda33(), ...
        self.zeta(), self.lambda34(), ...
        self.zeta(), self.lambda35(), ...
        self.zeta(), self.lambda36(), ...
        self.zeta(), self.lambda37(), ...
        self.zeta(), self.lambda38(), ...
        self.zeta(), self.lambda39(), ...
        'Linewidth', 2 ...
      );
      title('multipliers');
      legend( '\lambda1', '\lambda2', '\lambda3', '\lambda4', '\lambda5', '\lambda6', '\lambda7', '\lambda8', '\lambda9', '\lambda10', '\lambda11', '\lambda12', '\lambda13', '\lambda14', '\lambda15', '\lambda16', '\lambda17', '\lambda18', '\lambda19', '\lambda20', '\lambda21', '\lambda22', '\lambda23', '\lambda24', '\lambda25', '\lambda26', '\lambda27', '\lambda28', '\lambda29', '\lambda30', '\lambda31', '\lambda32', '\lambda33', '\lambda34', '\lambda35', '\lambda36', '\lambda37', '\lambda38', '\lambda39' );
    end

    function plot_controls( self )
      plot(...
        self.zeta(), self.b__f__o(), ...
        self.zeta(), self.b__r__o(), ...
        self.zeta(), self.t__o(), ...
        self.zeta(), self.tau__o(), ...
        'Linewidth', 2 ...
      );
      title('controls');
      legend( 'b\_f\_o', 'b\_r\_o', 't\_o', '\tau\_o' );
    end

  end

end

% EOF: General.m

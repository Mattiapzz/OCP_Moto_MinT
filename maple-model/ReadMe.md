# MOTORCYCLE MINIMUM LAP TIME PROBLEM #

## What is this repository for? ##

This repository aims to solve a minimum lap time problem of a motorcycle.
This is a constrained optimisation problem where the target function to be minimised is the elapsed time.

The optimal control problem is divided in two main parts:
1. Model development
2. Optimal control solution 

### Model ###

The model is derived in two steps. In the first maple file [Model.mw](.\Model.mw) the dynamic model is derived exploiting the anti-body method. All the procedure is well described and commented inside the file. In a second step, in the file [Static_SteadyState.mw](.\Static_SteadyState.mw), some unknown parameters are computed with a minimisation problem. Moreover, this file yields the states in static conditions and in straight steady state running. This states will be used in a second time as guess for the OCP. 

### Optimal control problem ###

In file [OCP_*.mw](.\OCP_*.mw) the OCP is built using XOptima and PINS. Once created the problem can be compiled and ran directly from console, [Maple](https://www.maplesoft.com/products/Maple/), [MATLAB](https://it.mathworks.com/products/matlab.html) or [Octave](https://www.gnu.org/software/octave/).
All parameters and data can be changed without recompiling each time.





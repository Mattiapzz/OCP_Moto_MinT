- [1. TO DO LIST](#1-to-do-list)
  - [1.1. Model](#11-model)
  - [1.2. Parameter computation](#12-parameter-computation)
  - [1.3. First order system](#13-first-order-system)
  - [1.4. OCP](#14-ocp)
- [2. QA](#2-qa)

# 1. TO DO LIST

## 1.1. Model

- Chain tension
  - force:
    - map tension to engine torque
    - use direct torque instead of force
  - angle:
    - full algebraic constraint
    - substitution and simplification
- Pacejka forces: add rolling resistance

  
## 1.2. Parameter computation

- Compute Static-State
  - Evaluate the internal DoF frozen
  - Evaluate CoM
  - Evaluate masses and inertias
- Compute Steady-State
  - SS in straight running as a minimisation problem
  - SS in static cornering situation

## 1.3. First order system

- Build FOS

## 1.4. OCP 

- Build OCP structure:
  - check user defined functions
  - check every constraint structure
- Simple straight running
- Corner
- Double corner
- Track
 

# 2. QA

- Chain tension force
  - use the torque of the motor or the force?
  - Inertia of shafts as an equivalent shaft?
- Check chain tension angle $\alpha_{crw}(t)$
  - Use it as an algebraic variable or as a constraint?
  - not sure of the position of the pinion wrt chassis
    - use fixed angle? 
    - use same angle as the swing arm? 
- Vertical forces direct substitution?
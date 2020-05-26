# MINIMUM LAP-TIME OPTIMAL CONTROL PROBLEM OF A RACING MOTORCYCLE

The project is divided in two parts. The first is the derivation of the kinematic and dynamic model of the motorcycle. The second part describe the formulation of the OCP with particular focus on minimum time application. 

## MODEL

The model is derived using [Maple](https://www.maplesoft.com/products/Maple/) a commercial software for symbolic manipulation of expressions in combination with [MBSymba](http://www.multibody.net/mbsymba/) custom library. THe procedure is well described inside the maple worksheets and in the readme file in the model folder.

## OCP 

The optimal control problem is defined inside [Maple](https://www.maplesoft.com/products/Maple/) using a custom library (XOptima) and PINS (PINS Is Not a Solver) which is a software free for academic purposes developed at the University of Trento (Italy) by Prof. Enrico Bertolazzi, Prof. Francesco Biral and Prof. Paolo Bosetti. 

The optimal control problem is solved using the indirect method inside PINS.

THe procedure is well described inside the maple worksheets and in the readme file in the OCP folder.

/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "optimization.h"

//==============================================================================
//
// LagrangianProblem
//
// we define an unconstraint optimization problem from a constrained one
// that can include lagrange terms, penalties, log barriers, and augmented lagrangian terms
//

struct LagrangianProblem : ScalarFunction { //TODO: rename: UnconstrainedLagrangianProblem
  ConstrainedProblem& P;
  
  //-- parameters of the unconstrained (Lagrangian) scalar function
  double muLB;       ///< log barrier weight
  double mu;         ///< penalty parameter for inequalities g
  double nu;         ///< penalty parameter for equalities h
  arr lambda;        ///< lagrange multipliers for inequalities g and equalities h
  
  //-- buffers to avoid re-evaluating points
  arr x;               ///< point where P was last evaluated
  arr phi_x, J_x, H_x; ///< features at x
  ObjectiveTypeA tt_x; ///< feature types at x

  ostream *logFile=NULL;  ///< file for logging

  LagrangianProblem(ConstrainedProblem &P, OptOptions opt=NOOPT, arr& lambdaInit=NoArr);
  
  double lagrangian(arr& dL, arr& HL, const arr& x); ///< CORE METHOD: the unconstrained scalar function F
  
  double get_costs();            ///< info on the terms from last call
  double get_sumOfGviolations(); ///< info on the terms from last call
  double get_sumOfHviolations(); ///< info on the terms from last call
  uint get_dimOfType(const ObjectiveType& tt); ///< info on the terms from last call
  
  void aulaUpdate(bool anyTimeVariant, double lambdaStepsize=1., double muInc=1., double *L_x=NULL, arr &dL_x=NoArr, arr &HL_x=NoArr);
  
  //private: used gpenalty function
  double gpenalty(double g);
  double gpenalty_d(double g);
  double gpenalty_dd(double g);
  
  double hpenalty(double h);
  double hpenalty_d(double h);
  double hpenalty_dd(double h);
};


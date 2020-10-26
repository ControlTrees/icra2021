/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "lagrangian.h"

//==============================================================================
//
// LagrangianProblem
//

double I_lambda_x(uint i, arr& lambda, arr& g) {
  if(g(i)>0. || (lambda.N && lambda(i)>0.)) return 1.;
  return 0.;
}

//==============================================================================

LagrangianProblem::LagrangianProblem(ConstrainedProblem& P, OptOptions opt, arr& lambdaInit)
  : P(P), muLB(0.), mu(0.), nu(0.) {
  
  ScalarFunction::operator=([this](arr& dL, arr& HL, const arr& x) -> double {
    return this->lagrangian(dL, HL, x);
  });
  
  //switch on penalty terms
  nu=opt.muInit;
  switch(opt.constrainedMethod) {
    case squaredPenalty: mu=opt.muInit;  break;
    case augmentedLag:   mu=opt.muInit;  break;
    case anyTimeAula:    mu=opt.muInit;  break;
    case logBarrier:     muLB=opt.muLBInit;  break;
    case squaredPenaltyFixed: mu=opt.muInit;  break;
    case noMethod: HALT("need to set method before");  break;
  }
  
  if(!!lambdaInit) lambda = lambdaInit;
}

double LagrangianProblem::lagrangian(arr& dL, arr& HL, const arr& _x) {
  //-- evaluate constrained problem and buffer
  if(_x!=x) {
    x=_x;
    P.phi(phi_x, J_x, H_x, tt_x, x, lambda);
  } else { //we evaluated this before - use buffered values; the meta F is still recomputed as (dual) parameters might have changed
  }
  CHECK(x.N, "zero-dim optimization variables!");
  if(!isSparseMatrix(J_x)){
    CHECK_EQ(phi_x.N, J_x.d0, "Jacobian size inconsistent");
  }
  CHECK_EQ(phi_x.N, tt_x.N, "termType array size inconsistent");
  
  //-- construct unconstrained problem
  //precompute I_lambda_x
  boolA I_lambda_x(phi_x.N);
  if(phi_x.N) I_lambda_x = false;
  if(mu)      for(uint i=0; i<phi_x.N; i++) if(tt_x.p[i]==OT_ineq) I_lambda_x.p[i] = (phi_x.p[i]>0. || (lambda.N && lambda.p[i]>0.));
  
  double L=0.; //L value
  for(uint i=0; i<phi_x.N; i++) {
    if(tt_x.p[i]==OT_f) L += phi_x.p[i];                                                  // direct cost term
    if(tt_x.p[i]==OT_sos) L += rai::sqr(phi_x.p[i]);                                 // sumOfSqr term
    if(muLB     && tt_x.p[i]==OT_ineq) { if(phi_x.p[i]>0.) return NAN;  L -= muLB * ::log(-phi_x.p[i]); }                   //log barrier, check feasibility
    if(mu       && tt_x.p[i]==OT_ineq && I_lambda_x.p[i]) L += gpenalty(phi_x.p[i]);      //g-penalty
    if(lambda.N && tt_x.p[i]==OT_ineq && lambda.p[i]>0.) L += lambda.p[i] * phi_x.p[i];   //g-lagrange terms
    if(nu       && tt_x.p[i]==OT_eq) L += hpenalty(phi_x.p[i]);                           //h-penalty
    if(lambda.N && tt_x.p[i]==OT_eq) L += lambda.p[i] * phi_x.p[i];                       //h-lagrange terms
  }
  
  if(!!dL) { //L gradient
    arr coeff=zeros(phi_x.N);
    for(uint i=0; i<phi_x.N; i++) {
      if(tt_x.p[i]==OT_f) coeff.p[i] += 1.;                                                  // direct cost term
      if(tt_x.p[i]==OT_sos) coeff.p[i] += 2.* phi_x.p[i];                               // sumOfSqr terms
      if(muLB     && tt_x.p[i]==OT_ineq) coeff.p[i] -= (muLB/phi_x.p[i]);                    //log barrier, check feasibility
      if(mu       && tt_x.p[i]==OT_ineq && I_lambda_x.p[i]) coeff.p[i] += gpenalty_d(phi_x.p[i]);  //g-penalty
      if(lambda.N && tt_x.p[i]==OT_ineq && lambda.p[i]>0.) coeff.p[i] += lambda.p[i];              //g-lagrange terms
      if(nu       && tt_x.p[i]==OT_eq) coeff.p[i] += hpenalty_d(phi_x.p[i]);                       //h-penalty
      if(lambda.N && tt_x.p[i]==OT_eq) coeff.p[i] += lambda.p[i];                                  //h-lagrange terms
    }
    dL = comp_At_x(J_x, coeff);
    dL.reshape(x.N);
  }
  
  if(!!HL) { //L hessian: Most terms are of the form   "J^T  diag(coeffs)  J"
    arr coeff=zeros(phi_x.N);
    int fterm=-1;
    for(uint i=0; i<phi_x.N; i++) {
      if(tt_x.p[i]==OT_f) { if(fterm!=-1) HALT("There must only be 1 f-term (in the current implementation)");  fterm=i; }
      if(tt_x.p[i]==OT_sos) coeff.p[i] += 2.;                                 // sumOfSqr terms
      if(muLB     && tt_x.p[i]==OT_ineq) coeff.p[i] += (muLB/rai::sqr(phi_x.p[i]));                     //log barrier, check feasibility
      if(mu       && tt_x.p[i]==OT_ineq && I_lambda_x.p[i]) coeff.p[i] += gpenalty_dd(phi_x.p[i]);   //g-penalty
      if(nu       && tt_x.p[i]==OT_eq) coeff.p[i] += hpenalty_dd(phi_x.p[i]);                        //h-penalty
    }
    arr tmp = J_x;
    if(!isSparseMatrix(tmp)){
      for(uint i=0; i<phi_x.N; i++) tmp[i]() *= sqrt(coeff.p[i]);
    }else{
      arr sqrtCoeff = sqrt(coeff);
      tmp.sparse().rowWiseMult(sqrtCoeff);
    }
#if 1
    HL = comp_At_A(tmp); //Gauss-Newton type!
#else
    arr tmpt = comp_At(tmp);
    HL = comp_A_At(tmpt); //Gauss-Newton type!
#endif
    
    if(fterm!=-1) { //For f-terms, the Hessian must be given explicitly, and is not \propto J^T J
      HL += H_x;
    }
    
    if(!HL.special) HL.reshape(x.N,x.N);
  }

  if(logFile) (*logFile) <<"{ lagrangianQuery: True, errors: [" <<get_costs() <<", " <<get_sumOfGviolations() <<", " <<get_sumOfHviolations() <<"] }," <<endl;
  
  return L;
}

double LagrangianProblem::get_costs() {
  double S=0.;
  for(uint i=0; i<phi_x.N; i++) {
    if(tt_x(i)==OT_f) S += phi_x(i);
    if(tt_x(i)==OT_sos) S += rai::sqr(phi_x(i));
  }
  return S;
}

double LagrangianProblem::get_sumOfGviolations() {
  double S=0.;
  for(uint i=0; i<phi_x.N; i++) {
    if(tt_x(i)==OT_ineq && phi_x(i)>0.) {
      S += phi_x(i);
//      cout <<"g violation" <<i << phi_x(i) <<' '<<(lambda.N?lambda(i):-1.) <<endl;
    }
  }
  return S;
}

double LagrangianProblem::get_sumOfHviolations() {
  double S=0.;
  for(uint i=0; i<phi_x.N; i++) {
    if(tt_x(i)==OT_eq) S += fabs(phi_x(i));
  }
  return S;
}

uint LagrangianProblem::get_dimOfType(const ObjectiveType& tt) {
  uint d=0;
  for(uint i=0; i<tt_x.N; i++) if(tt_x(i)==tt) d++;
  return d;
}

void LagrangianProblem::aulaUpdate(bool anyTimeVariant, double lambdaStepsize, double muInc, double *L_x, arr& dL_x, arr& HL_x) {
  if(!lambda.N) lambda=zeros(phi_x.N);
  
  //-- lambda update
  if(lambdaStepsize>0.) {
    for(uint i=0; i<lambda.N; i++) {
      if(tt_x(i)==OT_eq)  lambda(i) += lambdaStepsize * hpenalty_d(phi_x(i));
      if(tt_x(i)==OT_ineq)  lambda(i) += lambdaStepsize * gpenalty_d(phi_x(i));
      if(tt_x(i)==OT_ineq && lambda(i)<0.) lambda(i)=0.;  //bound clipping
    }
  }
  
  if(anyTimeVariant) {
    //collect gradients of active constraints
    arr A;
    RowShifted *Aaux=NULL, *Jaux=NULL;
    if(isRowShifted(J_x)) {
      Aaux = makeRowShifted(A, 0, J_x.d1, x.N);
      Jaux = castRowShifted(J_x);
    }
    //append rows of J_x to A if constraint is active
    for(uint i=0; i<lambda.N; i++) {
      if((tt_x(i)==OT_eq) ||
          (tt_x(i)==OT_ineq && (phi_x(i)>0. || lambda(i)>0.))) {
        A.append(J_x[i]);
        A.reshape(A.N/J_x.d1,J_x.d1);
        if(isRowShifted(J_x))
          Aaux->rowShift.append(Jaux->rowShift(i));
      }
    }
    if(A.d0>0) {
      arr tmp = comp_A_At(A);
      addDiag(tmp, 1e-6);
      //    if(isRowShifted(J_x)){
      //      CHECK_EQ(castRowShifted(tmp)->symmetric, true, "");
      //      for(uint i=0;i<tmp.d0;i++) tmp(i,0) += 1e-6;
      //    }else{
      //      for(uint i=0;i<tmp.d0;i++) tmp(i,i) += 1e-6;
      //    }
      
      arr AdL = comp_A_x(A, dL_x);
      arr beta;
      bool worked=true;
      try {
        beta = lapack_Ainv_b_sym(tmp, AdL);
      } catch(...) {
        arr sig, eig;
        lapack_EigenDecomp(tmp, sig, eig);
        cout <<endl <<"** hessian inversion failed AulaAnyTimeUpdate: " <<sig <<" -- revert to standard update" <<endl;
        worked=false;
      }
      
      if(worked) {
        //reinsert zero rows
        for(uint i=0; i<lambda.N; i++) {
          if(!((tt_x(i)==OT_eq) ||
               (tt_x(i)==OT_ineq && (phi_x(i)>0. || lambda(i)>0.)))) {
            beta.insert(i,0.);
          }
        }
        lambda -= lambdaStepsize * beta;
        //bound clipping
        for(uint i=0; i<lambda.N; i++) if(lambda(i)<0.) lambda(i)=0.;
      }
    }
  }
  
  //-- adapt mu as well?
  if(muInc>1. && mu<1e6) mu *= muInc;
  if(muInc>1. && nu<1e6) nu *= muInc;
  
  //-- recompute the Lagrangian with the new parameters (its current value, gradient & hessian)
  if(L_x || !!dL_x || !!HL_x) {
    double L = lagrangian(dL_x, HL_x, x); //reevaluate gradients and hessian (using buffered info)
    if(L_x) *L_x = L;
  }
}

#if 0
double LagrangianProblem::gpenalty(double g) { return mu*g*g; }
double LagrangianProblem::gpenalty_d(double g) { return 2.*mu*g; }
double LagrangianProblem::gpenalty_dd(double g) { return 2.*mu; }
#else
double LagrangianProblem::gpenalty(double g) { g*=mu;  if(g>0.) return (g*g + g*g*g);  return g*g; }
double LagrangianProblem::gpenalty_d(double g) { g*=mu; if(g>0.) return mu*(2.*g + 3.*g*g);  return 2.*mu*g; }
double LagrangianProblem::gpenalty_dd(double g) { g*=mu; if(g>0.) return mu*mu*(2. + 6.*g);  return 2.*mu*mu; }
#endif

#if 0
double LagrangianProblem::hpenalty(double h) { return nu*h*h;  }
double LagrangianProblem::hpenalty_d(double h) { return 2.*nu*h;  }
double LagrangianProblem::hpenalty_dd(double h) { return 2.*nu;  }
#else
double LagrangianProblem::hpenalty(double h) { h*=nu;  return (h*h + fabs(h)*h*h);  }
double LagrangianProblem::hpenalty_d(double h) { h*=nu; return nu*(2.*h + 3.*fabs(h)*h);  }
double LagrangianProblem::hpenalty_dd(double h) { h*=nu; return nu*nu*(2. + 6.*fabs(h));  }
#endif


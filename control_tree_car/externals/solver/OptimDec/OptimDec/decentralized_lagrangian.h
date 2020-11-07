#pragma once

#include <Optim/newton.h>
#include <Optim/constrained.h>

using CallBackType = std::function<void()>;

enum Mode
{
  SEQUENTIAL = 0,
  PARALLEL,
  FIRST_ITERATION_SEQUENTIAL_THEN_PARALLEL // doesn't seem to perform very well on komo problems
};

struct DecOptConfig
{
  DecOptConfig(const Mode& scheduling, bool compressed, OptOptions opt=NOOPT, bool checkGradients=false, CallBackType callback = CallBackType(), ostream *logFile=nullptr)
    : scheduling(scheduling)
    , compressed(compressed)
    , opt(opt)
    , checkGradients(checkGradients)
    , callback(callback)
    , logFile(logFile)
  {
  }

  Mode scheduling;

  bool compressed; // wether xs.d0 == x.d0, if true subproblem optimizers act on smaller (local) x
  OptOptions opt;  // for newton and aula

  bool checkGradients;
  CallBackType callback; // called after each step() (for debugging)
  ostream *logFile;

  double muInit{1.0};   // initial mu after first step
  double muInc{1.0}; // mu increase
};

template <typename T>
struct DecLagrangianProblem : ScalarFunction {
  T& L;

  //-- parameters of the ADMM
  double mu;         ///< ADMM square penalty
  arr z;             ///< external ADMM reference (global)
  arr lambda;        ///< ADMM lagrange multiplier
  intA var;          ///< indicate where an index of x (local) contributes on z (global)
  intA admmVar;      ///< indices where on x do we have ADMM multipliers
  arr admmMask;      ///< 0 or 1 depending on whether we have an addm mutl at this index
  const DecOptConfig& config_; ///< optimization config

  DecLagrangianProblem(T& L, const arr & z, const intA & _var, const intA & _admmVar, const DecOptConfig& config)
    : L(L)
    , mu(0.0) // first step done with 0 (to avoid fitting to a unset reference)
    , z(z)
    , var(_var)
    , admmVar(_admmVar)
    , config_(config)
  {
    lambda = zeros(var.d0);
    admmMask = zeros(var.d0);

    for(auto i: admmVar)
    {
      admmMask(i) = 1.0;
    }

    ScalarFunction::operator=([this](arr& dL, arr& HL, const arr& x) -> double {
      return this->decLagrangian(dL, HL, x);
    });
  }

  double decLagrangian(arr& dL, arr& HL, const arr& x) const;

  arr deltaZ(const arr& x) const;

  void updateADMM(const arr& x, const arr& z);
};

#include <OptimDec/decentralized_lagrangian.tpp>

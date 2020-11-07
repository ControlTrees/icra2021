#pragma once

#include <OptimDec/decentralized_lagrangian.h>

#include <Optim/newton.h>
#include <Optim/constrained.h>

#include <functional>
#include <memory>


template< typename T>
struct LagrangianTypes
{
};

template<>
struct LagrangianTypes<ConstrainedProblem>
{
  typedef LagrangianProblem Lagrangian;
  typedef DecLagrangianProblem<Lagrangian> DecLagrangian;
};

struct DualState
{
  std::vector<arr> duals;
  std::vector<arr> admmDuals;
};

struct AverageUpdater
{
  void checkApplicability(const arr& contribs) const {/*always applicable*/}

  template<typename V>
  void updateZ(arr& z,
               const std::vector<arr>& xs,
               const std::vector<std::unique_ptr<V>>& DLs,
               const std::vector<intA>& vars,
               const arr& contribs) const;
};

struct BeliefState
{
  BeliefState(const arr& bs)
    : beliefState(bs)
  {}

  arr beliefState;

  void checkApplicability(const arr& contribs) const
  {
    bool valid = true;
    for(const auto& c: contribs)
    {
      valid = valid && (c == 1 || c == beliefState.d0);
    }

    CHECK(valid, "Beliefstate z-updater not applicable");

    double sum = 0;
    for(const auto s: beliefState) sum += s;

    CHECK(fabs(sum-1.0) < 0.0001, "Corrupted belief state");
  }

  template<typename V>
  void updateZ(arr& z,
               const std::vector<arr>& xs,
               const std::vector<std::unique_ptr<V>>& DLs,
               const std::vector<intA>& vars,
               const arr& contribs) const;
};

template< typename T, typename U>
struct DecOptConstrained
{
  typedef typename LagrangianTypes<T>::Lagrangian LagrangianType;
  typedef typename LagrangianTypes<T>::DecLagrangian DecLagrangianType;

  arr&z_final; ///< init and solution
  const uint N; ///< number of subproblems
  std::vector<std::unique_ptr<LagrangianType>> Ls;
  std::vector<std::unique_ptr<OptNewton>> newtons;
  std::vector<std::unique_ptr<DecLagrangianType>> DLs;
  std::vector<intA> vars; ///< how local xs maps to global opt variable
  std::vector<intA> admmVars; ///< where each subproblem needs admm lagrange terms
  arr contribs; ///< amount of contribution on x (on each index) - caching (deduced directly from masks)
  uint m; ///< number of z indices where problems overlap
  arr z_prev, z; ///< internal admm reference variable (z_prev is the one of the previous step)
  const U zUpdater;
  std::vector<arr> xs; ///< subproblems solutions
  std::vector<arr> duals; ///< duals of the subproblems
  bool subProblemsSolved{false}; ///< indicates whether the stopping criterion of each sub-problem is met (necessary for overall stopping condition)
  uint its{0}; ///< number of ADMM iterations

  DecOptConfig config;

public:
  /**
   * @brief DecOptConstrained
   * @param _z optimization variable
   * @param Ps subproblems
   * @param masks where each subproblem contributes on optimization variable
   * @param _config
   */
  DecOptConstrained(arr&_z, std::vector<std::shared_ptr<T>> & Ps, const std::vector<arr> & masks, const U & _zUpdater, DecOptConfig _config); //bool compressed = false, int verbose=-1, OptOptions _opt=NOOPT, ostream* _logFile=0);

  std::vector<uint> run();

  DualState getDualState() const; // for enabling warm start
  void setDualState(const DualState& state);

private:
  void initVars(const std::vector<arr> & xmasks);
  void initXs();  // init xs based on z (typicaly called once at the start)
  void initLagrangians(const std::vector<std::shared_ptr<T>> & Ps);

  bool step(); // outer step
  void stepSequential(); // step each subproblem and update Z in sequence
  void stepParallel();   // step each subproblem and update Z in //

  bool step(DecLagrangianType& DL, OptNewton& newton, arr& dual, uint i) const; // inner step

  //void updateZSequential(uint i); // updating z usinmg last result of subproblem i
  void updateZ(); // update z given the xs

  void updateADMM();
  bool stoppingCriterion() const;

  double primalResidual() const;
  double dualResidual() const;

  bool primalFeasibility(double r) const;
  bool dualFeasibility(double s) const;

  void checkGradients() const;
};

#include <OptimDec/decentralized_optimizer.tpp>

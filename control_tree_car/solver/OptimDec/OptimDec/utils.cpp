#include <OptimDec/utils.h>

void add(arr& HL, double mu, const intA& admmVar, const arr& admmMask)
{
  if(isSparseMatrix(HL))
  {
    const auto Hs = dynamic_cast<rai::SparseMatrix*>(HL.special);
    const auto& elems = Hs->elems;
    const auto& Z = Hs->Z;

    intA incremented; incremented.reserve(admmVar.d0);

    for(uint k=0; k < elems.d0; k++)
    {
      const auto i = elems.p[2*k];
      const auto j = elems.p[2*k+1];
      if(i == j && admmMask(i))
      {
        Z.elem(k) += mu;
        incremented.append(i);
      }
    }

    if(incremented.d0 != admmVar.d0)
    {
      // slow, quadratic, should happen rarely!!
      for(auto i: admmVar)
      {
        bool found = false;
        for(auto j: incremented)
        {
          if(i==j) found = true;
        }
        if(!found) Hs->addEntry(i, i) = mu;
      }
    }
  }
  else // not sparse
  {
    for(auto i: admmVar)
    {
      HL(i, i) += mu;
    }
  }
}

double sparsity(arr & H)
{
  if(isSparseMatrix(H))
  {
    auto Hs = dynamic_cast<rai::SparseMatrix*>(H.special);

    return double(Hs->elems.d0) / (H.d0 * H.d0);
  }
  else
  {
    return H.sparsity();
  }

  return 0.0;
}

bool checkGradients(const std::shared_ptr<ConstrainedProblem>& cp, const arr & x)
{
  const double tolerance=1e-4;

  VectorFunction F = [cp](arr& phi, arr& J, const arr& x) {
    return cp->phi(phi, J, NoArr, NoTermTypeA, x, NoArr);
  };

  arr J;
  arr JJ=finiteDifferenceJacobian(F, x, J);

  bool succ=true;
  double mmd=0.;
  for(uint i=0; i<J.d0; i++) {
    uint j;
    double md=maxDiff(J[i], JJ[i], &j);
    if(md>mmd) mmd=md;
    if(md>tolerance && md>fabs(J(i,j))*tolerance) {
//      if(!dense){
//        LOG(-1) <<"FAILURE in line " <<i <<" t=" <<CP_komo.featureTimes(i) <<' ' <<komo_problem.featureNames(i) <<" -- max diff=" <<md <<" |"<<J(i,j)<<'-'<<JJ(i,j)<<"| (stored in files z.J_*)";
//      }else{
//        LOG(-1) <<"FAILURE in line " <<i <<" t=" <</*CP_komo.featureTimes(i) <<' ' <<komo_problem.featureNames(i) <<*/" -- max diff=" <<md <<" |"<<J(i,j)<<'-'<<JJ(i,j)<<"| (stored in files z.J_*)";
//      }
      J[i] >>FILE("z.J_analytical");
      JJ[i] >>FILE("z.J_empirical");
      //cout <<"\nmeasured grad=" <<JJ <<"\ncomputed grad=" <<J <<endl;
      //HALT("");
//        return;
      rai::wait();
      succ=false;
    }
  }
  if(succ) std::cout <<"jacobianCheck -- SUCCESS (max diff error=" <<mmd <<")" <<std::endl;

  return true;
}

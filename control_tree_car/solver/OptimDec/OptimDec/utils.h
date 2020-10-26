#pragma once

#include <Core/array.h>
#include <Optim/constrained.h>

/**
 * @brief Add mu to the diagonal elements specified by var and mask
 * @param HL
 * @param mu
 * @param admmVar
 * @param admmMask
 */
void add(arr& HL, double mu, const intA& admmVar, const arr& admmMask);

double sparsity(arr & H);

bool checkGradients(const std::shared_ptr<ConstrainedProblem>& cp, const arr & x);

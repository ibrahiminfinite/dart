/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <fstream>

#include "dart/dart.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::optimization;
using dart::optimization::Function;
using dart::optimization::FunctionPtr;
using dart::optimization::UniqueFunctionPtr;

//==============================================================================
static int dimension = 10;
static Eigen::VectorXd lowerLimits = Eigen::VectorXd::Zero(dimension);
static Eigen::VectorXd upperLimits = Eigen::VectorXd::Constant(dimension, 1.0);

class ZDT1 : public MultiObjectiveProblem
{
public:
  ZDT1() : MultiObjectiveProblem(dimension)
  {
    // Do nothing
  }

  std::size_t getObjectiveDimension() const override
  {
    return 2u;
  }

  Eigen::VectorXd evaluateObjectives(const Eigen::VectorXd& x) const override
  {
    Eigen::VectorXd ret(getObjectiveDimension());

    ret[0] = x[0];

    const double g = 1.0 + 9 * (x.sum() - x[0]) / double(dimension - 1);
    ret[1] = g * (1.0 - std::sqrt(x[0] / g));

    return ret;
  }

protected:
};

//==============================================================================
class Func1 : public Function
{
public:
  Func1() = default;

  double eval(const Eigen::VectorXd& x) override
  {
    return x[0];
  }

  UniqueFunctionPtr clone() const
  {
    return std::make_unique<Func1>(*this);
  }

  std::size_t getParameterDimension() const
  {
    return 1u;
  }
};

//==============================================================================
class Func2 : public Function
{
public:
  Func2() = default;

  double eval(const Eigen::VectorXd& x) override
  {
    double g = 1.0 + 9 * (x.sum() - x[0]) / double(dimension - 1);
    return g * (1.0 - std::sqrt(x[0] / g));
  }

  UniqueFunctionPtr clone() const
  {
    return std::make_unique<Func2>(*this);
  }

  std::size_t getParameterDimension() const
  {
    return 1u;
  }
};

//==============================================================================
void testZDT1(MultiObjectiveSolver& solver)
{
#ifdef NDEBUG // release mode
  std::size_t numSolutions = 50;
#else
  std::size_t numSolutions = 10;
#endif
#ifdef NDEBUG // release mode
  std::size_t iterationNum = 1000;
#else
  std::size_t iterationNum = 200;
#endif

  auto problem = std::make_shared<ZDT1>();
  problem->setLowerBounds(lowerLimits);
  problem->setUpperBounds(upperLimits);

  solver.setProblem(problem);
  solver.setPopulationSize(numSolutions);
  solver.setNumPopulations(100);
  solver.setNumIterationsPerEvolution(iterationNum);
  solver.solve(100);

  EXPECT_TRUE(solver.getPopulation(0).getSize() == numSolutions);
  EXPECT_TRUE(solver.getPopulation(1).getSize() == numSolutions);
  EXPECT_TRUE(solver.getNumPopulations() == 100);
}

//==============================================================================
void testZDT1Generic(MultiObjectiveSolver& solver)
{
  auto pFunc1 = std::make_shared<Func1>();
  auto pFunc2 = std::make_shared<Func2>();

  std::vector<FunctionPtr> pFuncs;
  pFuncs.push_back(pFunc1);
  pFuncs.push_back(pFunc2);

#ifdef NDEBUG // release mode
  std::size_t numSolutions = 50;
#else
  std::size_t numSolutions = 10;
#endif
#ifdef NDEBUG // release mode
  std::size_t iterationNum = 1000;
#else
  std::size_t iterationNum = 200;
#endif

  auto problem
      = std::make_shared<optimization::GenericMultiObjectiveProblem>(dimension);
  problem->setObjectiveFunctions(pFuncs);
  problem->setLowerBounds(lowerLimits);
  problem->setUpperBounds(upperLimits);

  solver.setProblem(problem);
  solver.setPopulationSize(numSolutions);
  solver.setNumPopulations(100);
  solver.setNumIterationsPerEvolution(iterationNum);
  solver.solve(100);

  EXPECT_TRUE(solver.getPopulation(0).getSize() == numSolutions);
  EXPECT_TRUE(solver.getPopulation(1).getSize() == numSolutions);
  EXPECT_TRUE(solver.getNumPopulations() == 100);
}

//==============================================================================
TEST(ZDT1, Basic)
{
#if DART_HAVE_PAGMO
  PagmoMultiObjectiveSolver pagmoSolver;
  testZDT1(pagmoSolver);
  testZDT1Generic(pagmoSolver);
#endif
}

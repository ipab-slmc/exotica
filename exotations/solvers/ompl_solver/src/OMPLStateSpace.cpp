/*
 * OMPLStateSpace.cpp
 *
 *  Created on: 9 Jul 2014
 *      Author: s0972326
 */

#include "ompl_solver/OMPLStateSpace.h"

namespace exotica
{

  OMPLStateSpace::OMPLStateSpace(unsigned int dim)
      : ompl::base::RealVectorStateSpace(dim)
  {
    setName("OMPLStateSpace");
  }

  boost::shared_ptr<OMPLStateSpace> OMPLStateSpace::FromProblem(
      OMPLProblem_ptr prob)
  {
    int n = prob->getSpaceDim();
    boost::shared_ptr<OMPLStateSpace> ret;
    if (n <= 0)
    {
      ERROR("State space size error!");
    }
    else
    {
      ret.reset(new OMPLStateSpace(n));
      ompl::base::RealVectorBounds bounds(n);
      if (prob->getBounds().size() == 2 * n)
      {
        for (int i = 0; i < n; i++)
        {
          bounds.setHigh(i, prob->getBounds()[i + n]);
          bounds.setLow(i, prob->getBounds()[i]);
        }
      }
      else
      {
        WARNING(
            "State space bounds were not specified!\n"<< prob->getBounds().size() << " " << n);
      }
      ret->setBounds(bounds);
    }
    return ret;
  }

  OMPLStateSpace::~OMPLStateSpace()
  {
    // TODO Auto-generated destructor stub
  }

  EReturn OMPLStateSpace::copyToOMPLState(ompl::base::State *state,
      Eigen::VectorXd q) const
  {
    if (!state)
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    if (q.rows() != (int) getDimension())
    {
      ERROR(
          "State vector ("<<q.rows()<<") and internal state ("<<(int)getDimension()<<") dimension disagree");
      return FAILURE;
    }
    memcpy(state->as<ompl::base::RealVectorStateSpace::StateType>()->values,
        q.data(), sizeof(double) * q.rows());
    return SUCCESS;
  }

  EReturn OMPLStateSpace::copyFromOMPLState(const ompl::base::State *state,
      Eigen::VectorXd& q) const
  {
    if (!state)
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    if (q.rows() != (int) getDimension())
    {
      ERROR(
          "State vector ("<<q.rows()<<") and internal state ("<<(int)getDimension()<<") dimension disagree");
      return FAILURE;
    }
    memcpy(q.data(),
        state->as<ompl::base::RealVectorStateSpace::StateType>()->values,
        sizeof(double) * q.rows());
    return SUCCESS;
  }

} /* namespace exotica */

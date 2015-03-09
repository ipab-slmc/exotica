#ifndef TESTING_POSITION_SOLVER_TYPE_1_H
#define TESTING_POSITION_SOLVER_TYPE_1_H

#include <exotica/PositionSolver.h>
#include <exotica/PosSolverFactory.h>

#include "testing_pkg/TestingTools.h"

class PositionSolverType_1 : public exotica::PositionSolver
{
  public:
  
    PositionSolverType_1(const exotica::OptimisationParameters_t & params);
    
    virtual bool solve(const Eigen::VectorXd & init_conf, Eigen::VectorXd & solution_vector);
    
    void clearFlags() { derived_called = false; }
    
    bool                                          derived_called;
    const std::string                             name;
    std::string                                   string_element;
    const exotica::OptimisationParameters_t &     params_ref_; //!< Reference to the underlying paramaters (for testing)
    boost::shared_ptr<exotica::VelocitySolver> &  vel_solv_ref_;
    
  protected:
  
    virtual bool initDerived(tinyxml2::XMLHandle & derived_element);
};

#endif

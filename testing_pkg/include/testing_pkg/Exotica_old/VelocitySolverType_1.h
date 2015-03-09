#ifndef TESTING_VELOCITY_SOLVER_TYPE_1
#define TESTING_VELOCITY_SOLVER_TYPE_1

#include <exotica/VelocitySolver.h>
#include <exotica/VelSolverFactory.h>
 
class VelocitySolverType_1 : public exotica::VelocitySolver
{
  public :
    //!< Member Functions
    VelocitySolverType_1(const exotica::OptimisationParameters_t & params);
    virtual bool getInverse(const Eigen::MatrixXd & big_jacobian, const Eigen::MatrixXd & config_weights, const Eigen::MatrixXd & task_weights, Eigen::MatrixXd & inv_jacobian);
    void clearFlags() { derived_called = inverse_called = false; }
    
    //!< Member Variables
    std::string string_element;
    bool derived_called;
    bool inverse_called;
    const std::string name;
    
  protected : 
    virtual bool initDerived(tinyxml2::XMLHandle & derived_element);
    
};

#endif

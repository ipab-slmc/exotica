#ifndef EXOTICA_TASK_1_H
#define EXOTICA_TASK_1_H

#include <exotica/EXOTica.hpp>
#include <Eigen/Eigen>

//!< An Empty ExoticaTask
class ExoticaTaskTest_1 : public exotica::TaskDefinition
{
  public:
  
    ExoticaTaskTest_1(const exotica::OptimisationParameters_t & optimisation_params);  //!< Constructor
    
    virtual bool updateTask(const Eigen::VectorXd & configuration, int index=0);  //!< Task Updator
    
    virtual bool initDerived(tinyxml2::XMLHandle & derived_element);  //!< The Initialiser
    
    void clearFlags() { derived_called = update_called = false; }
    
    std::string name;
    bool derived_called;
    bool update_called;

};

#endif

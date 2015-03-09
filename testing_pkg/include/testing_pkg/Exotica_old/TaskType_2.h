#ifndef EXOTICA_TASK_2_H
#define EXOTICA_TASK_2_H

#include <exotica/EXOTica.hpp>
#include <Eigen/Eigen>

//!< An Empty ExoticaTask
class ExoticaTaskTest_2 : public exotica::TaskDefinition
{
  public:
  
    ExoticaTaskTest_2(const exotica::OptimisationParameters_t & optimisation_params);  //!< Constructor
    
    virtual bool updateTask(const Eigen::VectorXd & configuration, int index=0);  //!< Task Updator
    
    virtual bool initDerived(tinyxml2::XMLHandle & derived_element);  //!< The Initialiser
    
    void clearFlags() { derived_called = update_called = false; }
    
    std::string name;
    
    int         int_element;
    std::string string_element;
    bool        derived_called;
    bool        update_called;
    
};

#endif

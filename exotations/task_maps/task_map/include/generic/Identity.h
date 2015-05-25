#ifndef EXOTICA_GENERIC_IDENTITY_H
#define EXOTICA_GENERIC_IDENTITY_H

#include <exotica/TaskMap.h>
#include <exotica/Factory.h>
#include <exotica/Test.h>
#include <tinyxml2/tinyxml2.h>
#include <Eigen/Dense>

namespace exotica
{
  class Identity : public TaskMap
  {
    public:
      /**
       * \brief Default constructor
       */
      Identity();
      virtual ~Identity(){}
      
      /**
       * \brief Concrete implementation of the update method
       */
      virtual EReturn update(Eigen::VectorXdRefConst x, const int t);
      
      /**
       * \brief Concrete implementation of the task-space size
       */
      virtual EReturn taskSpaceDim(int & task_dim);

      virtual EReturn initialise(const rapidjson::Value& a);

      bool useRef;
      std::vector<int> jointMap;
      Eigen::VectorXd jointRef;
      
    protected:
      /**
       * \brief Concrete implementation of the initialisation method
       */
      virtual EReturn initDerived(tinyxml2::XMLHandle & handle);

      int getJointIDexternal(std::string& name);
      int getJointID(std::string& name);


  };
}
#endif

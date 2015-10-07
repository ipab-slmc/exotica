#ifndef EXOTICA_TASKMAP_DISTANCE_H
#define EXOTICA_TASKMAP_DISTANCE_H

#include <exotica/TaskMap.h>
#include <exotica/Factory.h>
#include <exotica/Test.h>
#include <tinyxml2/tinyxml2.h>
#include <Eigen/Dense>
#include <boost/thread/mutex.hpp>

namespace exotica //!< Since this is part of the core library, it will be within the same namespace
{
  class Distance: public TaskMap
  {
    public:
      /**
       * \brief Default constructor
       */
      Distance();
      virtual ~Distance()
      {
      }

      /**
       * \brief Concrete implementation of the update method
       */
      virtual EReturn update(Eigen::VectorXdRefConst x, const int t);

      /**
       * \brief Concrete implementation of the task-space size
       */
      virtual EReturn taskSpaceDim(int & task_dim);

      virtual EReturn initialise(const rapidjson::Value& a);

      KDL::Frame ref_pose_;

    protected:
      /**
       * \brief Concrete implementation of TaskMap::initDerived()
       * @return  Always returns success
       */
      virtual EReturn initDerived(tinyxml2::XMLHandle & handle);
  };
  typedef boost::shared_ptr<Distance> Distance_Ptr;
}

#endif

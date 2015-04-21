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
  class Distance : public TaskMap
  {
    public:
      /**
       * \brief Default constructor
       */
      Distance();
      virtual ~Distance(){};

      /**
       * \brief Concrete implementation of the update method
       */
      virtual EReturn update(const Eigen::VectorXd & x, const int t);

      /**
       * \brief Concrete implementation of the task-space size
       */
      virtual EReturn taskSpaceDim(int & task_dim);

    protected:
      /**
       * \brief Concrete implementation of TaskMap::initDerived()
       * @return  Always returns success
       */
      virtual EReturn initDerived(tinyxml2::XMLHandle & handle);
  private:
      Eigen::VectorXd tmp_phi_;
      Eigen::MatrixXd tmp_jac_;
      Eigen::VectorXd ret_phi_;
      Eigen::MatrixXd ret_jac_;
  };
}

#endif

/*
 * DrakeIKFilter.h
 *
 *  Created on: 27 Nov 2015
 *      Author: yiming
 */

#ifndef EXOTICA_TOOLS_DRAKE_IK_FILTER_INCLUDE_DRAKE_IK_FILTER_DRAKEIKFILTER_H_
#define EXOTICA_TOOLS_DRAKE_IK_FILTER_INCLUDE_DRAKE_IK_FILTER_DRAKEIKFILTER_H_
#include <drake/RigidBodyIK.h>
#include <drake/RigidBodyManipulator.h>
#include <drake/RigidBodyConstraint.h>
#include <drake/IKoptions.h>
#include <exotica/EXOTica.hpp>

namespace exotica
{
  /*
   * \brief This class converts OMPL's SE3+RN state space to drake state via DrakeIK
   * TODO Specifically for Valkyrie
   */
  class DrakeIKFilter
  {
    public:
      DrakeIKFilter();
      ~DrakeIKFilter();
      EReturn initialise(const std::string &urdf);
      EReturn convert(const Eigen::VectorXd &state, Eigen::VectorXd &drake);
    private:
      RigidBodyManipulator* model_;
      std::vector<RigidBodyConstraint*> constraints_;
      std::vector<std::string> joints_;
      std::vector<int> joints_idx_;
      Eigen::VectorXd reach_start_;
      IKoptions* ik_options_;
  };
}

#endif /* EXOTICA_TOOLS_DRAKE_IK_FILTER_INCLUDE_DRAKE_IK_FILTER_DRAKEIKFILTER_H_ */

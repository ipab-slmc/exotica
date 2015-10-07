/*
 * DRMSpaceLoader.h
 *
 *  Created on: 21 Sep 2015
 *      Author: yiming
 */

#ifndef EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMSPACELOADER_H_
#define EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMSPACELOADER_H_

#include "dynamic_reachability_map/DRMSpace.h"
#include "dynamic_reachability_map/tinyxml2.h"
namespace dynamic_reachability_map
{
  class DRMSpaceLoader
  {
    public:
      DRMSpaceLoader();

      ~DRMSpaceLoader();

      bool loadSpace(const std::string &path, DRMSpace_ptr &space,
          const robot_model::RobotModelConstPtr &model);
    private:
      class MultiThreadsSpaceOccupationLoader
      {
        public:
          MultiThreadsSpaceOccupationLoader(const std::string & path,
              DRMSpace_ptr &space);
          ~MultiThreadsSpaceOccupationLoader();
          void loadSpaceOccupation();
        private:
          void load(int id);
          std::string path_;
          DRMSpace_ptr space_;
      };
  };
}

#endif /* EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMSPACELOADER_H_ */

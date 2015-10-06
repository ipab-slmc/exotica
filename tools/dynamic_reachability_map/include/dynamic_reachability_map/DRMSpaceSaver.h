/*
 * DRMSpaceSaver.h
 *
 *  Created on: 20 Sep 2015
 *      Author: yiming
 */

#ifndef EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMSPACESAVER_H_
#define EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMSPACESAVER_H_

#include "dynamic_reachability_map/DRMSpace.h"
namespace dynamic_reachability_map
{
class DRMSpaceSaver
{
public:
  DRMSpaceSaver();

  ~DRMSpaceSaver();

  bool saveSpace(const std::string &path, DRMSpace_ptr &space, std::string &savepath);
};
}

#endif /* EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMSPACESAVER_H_ */

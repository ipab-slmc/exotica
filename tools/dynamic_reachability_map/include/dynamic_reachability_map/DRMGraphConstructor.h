/*
 * DRMGraphConstructor.h
 *
 *  Created on: 3 Dec 2015
 *      Author: yiming
 */

#ifndef EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMGRAPHCONSTRUCTOR_H_
#define EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMGRAPHCONSTRUCTOR_H_

#include "dynamic_reachability_map/DRMSpace.h"

namespace dynamic_reachability_map
{
  class DRMGraphConstructor
  {
    public:
      DRMGraphConstructor();
      virtual ~DRMGraphConstructor();
      bool createGraph(DRMSpace_ptr &space);
      void getKNN(const unsigned long int index,
          std::map<double, unsigned long int> &knn);
    private:
      DRMSpace_ptr space_;
  };
}

#endif /* EXOTICA_TOOLS_DYNAMIC_REACHABILITY_MAP_INCLUDE_DYNAMIC_REACHABILITY_MAP_DRMGRAPHCONSTRUCTOR_H_ */

/*
 * RRTConnect.h
 *
 *  Created on: 10 Oct 2017
 *      Author: yiming
 */

#ifndef INCLUDE_OMPL_SOLVER_RRTCONNECT_H_
#define INCLUDE_OMPL_SOLVER_RRTCONNECT_H_

#include <ompl_solver/ompl_solver.h>

namespace exotica
{
class RRTConnect : public OMPLsolver, Instantiable<RRTConnectInitializer>
{
public:
    RRTConnect();

    virtual ~RRTConnect();

    virtual void Instantiate(RRTConnectInitializer& init);
};
}

#endif /* INCLUDE_OMPL_SOLVER_RRTCONNECT_H_ */

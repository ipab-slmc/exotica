/*
 * ompl_native_solvers.h
 *
 *  Created on: 10 Oct 2017
 *      Author: yiming
 */

#ifndef INCLUDE_OMPL_SOLVER_OMPL_NATIVE_SOLVERS_H_
#define INCLUDE_OMPL_SOLVER_OMPL_NATIVE_SOLVERS_H_

#include <ompl_solver/ompl_solver.h>

namespace exotica
{
class RRT : public OMPLsolver, Instantiable<OMPLsolverInitializer>
{
public:
    RRT();
    virtual void Instantiate(OMPLsolverInitializer& init);
};

class RRTConnect : public OMPLsolver, Instantiable<OMPLsolverInitializer>
{
public:
    RRTConnect();
    virtual void Instantiate(OMPLsolverInitializer& init);
};

class PRM : public OMPLsolver, Instantiable<OMPLsolverInitializer>
{
public:
    PRM();
    virtual void Instantiate(OMPLsolverInitializer& init);
};
}

#endif /* INCLUDE_OMPL_SOLVER_OMPL_NATIVE_SOLVERS_H_ */

/*
 * ompl_solver.h
 *
 *  Created on: 10 Oct 2017
 *      Author: yiming
 */

#ifndef INCLUDE_OMPL_OMPL_SOLVER_H_
#define INCLUDE_OMPL_OMPL_SOLVER_H_

#include <exotica/MotionSolver.h>
#include <ompl_solver/ompl_exo.h>

typedef boost::function<
    ompl::base::PlannerPtr(const ompl::base::SpaceInformationPtr &si,
                           const std::string &name)>
    ConfiguredPlannerAllocator;

namespace exotica
{
class OMPLsolver : public MotionSolver
{
public:
    OMPLsolver();

    virtual ~OMPLsolver();

    virtual void Solve(Eigen::MatrixXd &solution);
    virtual void specifyProblem(PlanningProblem_ptr pointer);

protected:
    template <typename T>
    static ompl::base::PlannerPtr allocatePlanner(
        const ompl::base::SpaceInformationPtr &si,
        const std::string &new_name)
    {
        ompl::base::PlannerPtr planner(new T(si));
        if (!new_name.empty())
            planner->setName(new_name);
        return planner;
    }

    void setGoalState(const Eigen::VectorXd &qT, const double eps = 0);
    void preSolve();
    void postSolve();
    void getPath(Eigen::MatrixXd &traj, ompl::base::PlannerTerminationCondition &ptc);
    OMPLsolverInitializer init_;
    SamplingProblem_ptr prob_;
    ompl::geometric::SimpleSetupPtr ompl_simple_setup_;
    ompl::base::StateSpacePtr state_space_;
    ConfiguredPlannerAllocator planner_allocator_;
    std::string algorithm_;
};
}

#endif /* INCLUDE_OMPL_OMPL_SOLVER_H_ */

/*
 * OMPLProblem.h
 *
 *  Created on: 19 Jun 2014
 *      Author: Vladimir Ivan
 */

#ifndef OMPLPROBLEM_H_
#define OMPLPROBLEM_H_

#include <exotica/PlanningProblem.h>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include "task_definition/TaskTerminationCriterion.h"


namespace exotica
{

    class TaskBias : public TaskSqrError
    {
    public:
        TaskBias();
    };

    typedef boost::shared_ptr<exotica::TaskBias> TaskBias_ptr;

    enum OMPLProblem_Type
    {
        OMPL_PROBLEM_GOAL=0,
        OMPL_PROBLEM_COSTS,
        OMPL_PROBLEM_GOAL_BIAS,
        OMPL_PROBLEM_SAMPLING_BIAS
    };

	class OMPLProblem : public PlanningProblem
	{
		public:
			OMPLProblem ();
			virtual
			~OMPLProblem ();

			int getSpaceDim();

			boost::mutex& getLock()
			{
				return update_lock_;
			}

			std::vector<TaskTerminationCriterion_ptr>& getGoals();
            std::vector<TaskSqrError_ptr>& getCosts();
            std::vector<TaskBias_ptr>& getGoalBias();
            std::vector<TaskBias_ptr>& getSamplingBias();
			std::vector<double>& getBounds();

            virtual EReturn reinitialise(rapidjson::Document& document, boost::shared_ptr<PlanningProblem> problem);
			std::string local_planner_config_;
            EParam<std_msgs::Bool> full_body_plan_;

            virtual void clear(bool keepOriginals=true);
		protected:
			/**
			 * \brief Derived Initialiser (from XML): PURE VIRTUAL
			 * @param handle The handle to the XML-element describing the Problem Definition
			 * @return Indication of success/failure
			 */
			virtual EReturn initDerived(tinyxml2::XMLHandle & handle);
		private:
			boost::mutex update_lock_;
			std::vector<TaskTerminationCriterion_ptr> goals_;
            std::vector<TaskSqrError_ptr> costs_;
            std::vector<TaskBias_ptr> goalBias_;
            std::vector<TaskBias_ptr> samplingBias_;
			std::vector<double> bounds_;
			int space_dim_;
            OMPLProblem_Type problemType;

            std::vector<TaskTerminationCriterion_ptr> originalGoals_;
            std::vector<TaskSqrError_ptr> originalCosts_;
            std::vector<TaskBias_ptr> originalGoalBias_;
            std::vector<TaskBias_ptr> originalSamplingBias_;

	};

	typedef boost::shared_ptr<exotica::OMPLProblem> OMPLProblem_ptr;

} /* namespace exotica */

#endif /* OMPLPROBLEM_H_ */

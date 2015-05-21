/*
 * OMPLProblem.cpp
 *
 *  Created on: 19 Jun 2014
 *      Author: Vladimir Ivan
 */

#include "ompl_solver/OMPLProblem.h"

REGISTER_PROBLEM_TYPE("OMPLProblem", exotica::OMPLProblem);

#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) {INDICATE_FAILURE; return PAR_ERR;}}
#define XML_OK(x) if(!ok(x)){INDICATE_FAILURE; return PAR_ERR;}

namespace exotica
{

	OMPLProblem::OMPLProblem() :
			space_dim_(0)
	{
		// TODO Auto-generated constructor stub

	}

	OMPLProblem::~OMPLProblem()
	{
		// TODO Auto-generated destructor stub
	}

	std::vector<double>& OMPLProblem::getBounds()
	{
		return bounds_;
	}

    EReturn OMPLProblem::reinitialise(rapidjson::Document& document, boost::shared_ptr<PlanningProblem> problem)
    {
        task_defs_.clear();
        task_maps_.clear();
        goals_.clear();
        if(document.IsArray())
        {
            for (rapidjson::SizeType i=0;i<document.Size();i++)
            {
                rapidjson::Value& obj = document[i];
                if(obj.IsObject())
                {
                    std::string constraintClass;
                    if(ok(getJSON(obj["class"],constraintClass)))
                    {
                        if(knownMaps_.find(constraintClass)!=knownMaps_.end())
                        {
                            TaskMap_ptr taskmap;
                            if(ok(TaskMap_fac::Instance().createObject(knownMaps_[constraintClass],taskmap)))
                            {
                                EReturn ret = taskmap->initialise(obj,server_,scenes_,problem);
                                if(ok(ret))
                                {
                                    if(ret!=CANCELLED)
                                    {
                                        std::string name=taskmap->getObjectName();
                                        task_maps_[name]=taskmap;
                                        TaskDefinition_ptr task;
                                        if(ok(TaskDefinition_fac::Instance().createObject("TaskTerminationCriterion",task)))
                                        {
                                            TaskTerminationCriterion_ptr sqr = boost::static_pointer_cast<TaskTerminationCriterion>(task);
                                            sqr->setTaskMap(taskmap);
                                            int dim;
                                            taskmap->taskSpaceDim(dim);
                                            sqr->y_star0_.resize(dim);
                                            sqr->rho0_(0)=1.0;
                                            sqr->threshold0_(0)=1e-1;
                                            sqr->object_name_=name+std::to_string((unsigned long)sqr.get());

                                            // TODO: Better implementation of stting goals from JSON
                                            sqr->y_star0_.setZero();

                                            sqr->setTimeSteps(1);
                                            sqr->wasFullyInitialised_=true;
                                            task_defs_[name]=task;
                                            goals_.push_back(sqr);
                                        }
                                        else
                                        {
                                            INDICATE_FAILURE;
                                            return FAILURE;
                                        }
                                    }
                                    else
                                    {
                                        ROS_WARN_STREAM("Creation of '"<<constraintClass<<"' cancelled!");
                                    }
                                }
                                else
                                {
                                    INDICATE_FAILURE;
                                    return FAILURE;
                                }
                            }
                            else
                            {
                                INDICATE_FAILURE;
                                return FAILURE;
                            }

                        }
                        else
                        {
                            WARNING("Ignoring unknown constraint '"<<constraintClass<<"'");
                        }
                    }
                    else
                    {
                        INDICATE_FAILURE;
                        return FAILURE;
                    }
                }
                else
                {
                    INDICATE_FAILURE;
                    return FAILURE;
                }
            }
        }
        else
        {
            INDICATE_FAILURE;
            return FAILURE;
        }

        robot_model::RobotModelPtr model = scenes_.begin()->second->getRobotModel();
        std::vector<std::string> joints = scenes_.begin()->second->getSolver().getJointNames();
        int n=joints.size();
        bounds_.resize(n*2);

        for(int i=0;i<n;i++)
        {
            boost::shared_ptr<urdf::JointLimits> lim=model->getURDF()->getJoint(joints[i])->limits;
            bounds_[i]=lim->lower;
            bounds_[i+n]=lim->upper;
        }
        return SUCCESS;

    }

	EReturn OMPLProblem::initDerived(tinyxml2::XMLHandle & handle)
	{
		for (auto goal : task_defs_)
		{
			if (goal.second->type().compare("exotica::TaskTerminationCriterion") == 0)
			{
				goals_.push_back(boost::static_pointer_cast<exotica::TaskTerminationCriterion>(goal.second));
			}
		}
		tinyxml2::XMLHandle tmp_handle = handle.FirstChildElement("LocalPlannerConfig");
		if (tmp_handle.ToElement())
		{
			local_planner_config_ = tmp_handle.ToElement()->GetText();
		}

		for (auto scene : scenes_)
		{
			int nn = scene.second->getNumJoints();
			if (space_dim_ == 0)
			{
				space_dim_ = nn;
				continue;
			}
			else
			{
				if (space_dim_ != nn)
				{
					ERROR("Kinematic scenes have different joint space sizes!");
					return FAILURE;
				}
				else
				{
					continue;
				}
			}
		}

		std::vector<std::string> jnts;
		scenes_.begin()->second->getJointNames(jnts);
		robot_model::RobotModelConstPtr model=server_->getModel("robot_description");
		getBounds().resize(jnts.size() * 2);
		for (int i = 0; i < jnts.size(); i++)
		{
			getBounds()[i] = model->getJointModel(jnts[i])->getVariableBounds()[0].min_position_;
			getBounds()[i + jnts.size()] = model->getJointModel(jnts[i])->getVariableBounds()[0].max_position_;
		}
		return SUCCESS;
	}

	int OMPLProblem::getSpaceDim()
	{
		return space_dim_;
	}

	std::vector<TaskTerminationCriterion_ptr>& OMPLProblem::getGoals()
	{
		return goals_;
	}

} /* namespace exotica */

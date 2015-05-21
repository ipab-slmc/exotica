#include "generic/Identity.h"

REGISTER_TASKMAP_TYPE("Identity", exotica::Identity);
REGISTER_FOR_XML_TEST("Identity", "Identity.xml");

namespace exotica
{
    Identity::Identity() : useRef(false)
    {

    }

    int Identity::getJointID(std::string& name)
    {
        for(int j=0;j<scene_->getSolver().getJointNames().size();j++)
        {
            if(scene_->getSolver().getJointNames()[j].compare(name)==0)
            {
                return j;
            }
        }
        return -1;
    }

    int Identity::getJointIDexternal(std::string& name)
    {
        for(int j=0;j<posesJointNames->size();j++)
        {
            if((*posesJointNames)[j].compare(name)==0)
            {
                return j;
            }
        }
        return -1;
    }

    EReturn Identity::initialise(const rapidjson::Value& a)
    {
        if(!poses || !posesJointNames)
        {
            ERROR("Poses have not been set!");
            return FAILURE;
        }
        std::string postureName;
        if(ok(getJSON(a["postureName"],postureName)))
        {
            std::map<std::string,Eigen::VectorXd>::const_iterator pose = poses->find(postureName);
            if(pose!=poses->end())
            {
                std::vector<std::string> joints;
                if(ok(getJSON(a["joints"],joints)))
                {
                    useRef=true;
                    jointMap.resize(0);
                    jointRef.resize(0);

                    for(int i=0;i<joints.size();i++)
                    {
                        int idext=getJointIDexternal(joints[i]);
                        if(idext>=0)
                        {
                            int id=getJointID(joints[i]);
                            if(id>=0)
                            {
                                jointMap.push_back(id);
                                jointRef.conservativeResize(jointRef.rows()+1);
                                jointRef(jointRef.rows()-1)=pose->second(idext);
                                continue;
                            }
                        }
                        else
                        {
                            ERROR("Requesting unknown joint '"<<joints[i]<<"'");
                            return FAILURE;
                        }
                    }
                    if(jointMap.size()==0)
                    {
                        return CANCELLED;
                    }
                    else
                    {
                        return SUCCESS;
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
                ERROR("Can't find pose '"<<postureName<<"'");
                return FAILURE;
            }
        }
        else
        {
            INDICATE_FAILURE;
            return FAILURE;
        }
    }

    EReturn Identity::update(Eigen::VectorXdRefConst x, const int t)
    {
      if(!isRegistered(t)||!getEffReferences()) {INDICATE_FAILURE; return FAILURE;}
      if(x.rows()==PHI.rows())
      {
        if(useRef)
        {
            if(updateJacobian_) JAC.setZero();
            for(int i=0;i<jointMap.size();i++)
            {
                PHI(i)=x(jointMap[i])-jointRef(i);
                if(updateJacobian_)
                {
                    JAC(i,jointMap[i])=1.0;
                }
            }
        }
        else
        {
            PHI=x;
            if(updateJacobian_)
            {
                JAC=Eigen::MatrixXd::Identity(x.rows(), x.rows());
            }
        }
      }
      else
      {
          INDICATE_FAILURE;
          return FAILURE;
      }
      return SUCCESS;
    }

    EReturn Identity::initDerived(tinyxml2::XMLHandle & handle)
    {
      return SUCCESS;
    }

    EReturn Identity::taskSpaceDim(int & task_dim)
    {
      if(useRef)
      {
          task_dim=jointMap.size();
      }
      else
      {
          task_dim = scene_->getNumJoints();
      }
      return SUCCESS;
    }
}

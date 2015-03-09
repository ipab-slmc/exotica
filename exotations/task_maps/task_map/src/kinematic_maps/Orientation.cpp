#include "kinematic_maps/Orientation.h"

#define P1 i*4*3
#define P2 i*4*3+3
#define P3 i*4*3+6
#define P4 i*4*3+9
#define X
#define Y +1
#define Z +2

REGISTER_TASKMAP_TYPE("Orientation", exotica::Orientation);

exotica::Orientation::Orientation()
{
  //!< Empty constructor
}

exotica::EReturn exotica::Orientation::update(const Eigen::VectorXd & x, const int t)
{
  //!< Prepare
  invalidate();
  LOCK(scene_lock_);

  //!< Check
  if (scene_ == nullptr) { INDICATE_FAILURE; return MMB_NIN; }

  //!< Temporaries
  std::vector<std::string> temp_vector;
  bool success = true;
  EReturn tmp_rtn = FAILURE;

  success = scene_->getForwardMap(tmp_phi_, temp_vector);

  if(!success) { INDICATE_FAILURE; return FAILURE; }
  else { success = scene_->getJacobian(tmp_jac_); }
  if(!success) { INDICATE_FAILURE; return FAILURE; }

  ret_jac_.setZero();
  for(int i=0;i<scene_->getMapSize()/4;i++)
  {
      ret_phi_(i)=sqrt((tmp_phi_(P2 X)-tmp_phi_(P1 X)+tmp_phi_(P4 X)-tmp_phi_(P3 X))*
                       (tmp_phi_(P2 X)-tmp_phi_(P1 X)+tmp_phi_(P4 X)-tmp_phi_(P3 X))+
                       (tmp_phi_(P2 Y)-tmp_phi_(P1 Y)+tmp_phi_(P4 Y)-tmp_phi_(P3 Y))*
                       (tmp_phi_(P2 Y)-tmp_phi_(P1 Y)+tmp_phi_(P4 Y)-tmp_phi_(P3 Y))+
                       (tmp_phi_(P2 Z)-tmp_phi_(P1 Z)+tmp_phi_(P4 Z)-tmp_phi_(P3 Z))*
                       (tmp_phi_(P2 Z)-tmp_phi_(P1 Z)+tmp_phi_(P4 Z)-tmp_phi_(P3 Z)));


      if(ret_phi_(i)>1e-50)
      {
          for(int j=0;j<scene_->getNumJoints();j++)
          {
            ret_jac_(i,j)=( (tmp_phi_(P2 X)-tmp_phi_(P1 X)+tmp_phi_(P4 X)-tmp_phi_(P3 X))*
                            (tmp_jac_(P2 X,j)-tmp_jac_(P1 X,j)+tmp_jac_(P4 X,j)-tmp_jac_(P3 X,j))+
                            (tmp_phi_(P2 Y)-tmp_phi_(P1 Y)+tmp_phi_(P4 Y)-tmp_phi_(P3 Y))*
                            (tmp_jac_(P2 Y,j)-tmp_jac_(P1 Y,j)+tmp_jac_(P4 Y,j)-tmp_jac_(P3 Y,j))+
                            (tmp_phi_(P2 Z)-tmp_phi_(P1 Z)+tmp_phi_(P4 Z)-tmp_phi_(P3 Z))*
                            (tmp_jac_(P2 Z,j)-tmp_jac_(P1 Z,j)+tmp_jac_(P4 Z,j)-tmp_jac_(P3 Z,j)) )/
                            ret_phi_(i);
          }
      }
  }
  { tmp_rtn = setPhi(ret_phi_,t); }
  if(!success) { INDICATE_FAILURE; return FAILURE; }
  if (ok(tmp_rtn))  { tmp_rtn = setJacobian(ret_jac_,t); }

  return tmp_rtn;
}

exotica::EReturn exotica::Orientation::initDerived(tinyxml2::XMLHandle & handle)
{
  if(scene_->getMapSize()%4!=0)
  {
    ERROR("Kinematic scene must have even number of end-effectors!");
    return FAILURE;
  }
  else
  {
    tmp_phi_.resize(scene_->getMapSize()*3);
    tmp_jac_.resize(scene_->getMapSize()*3,scene_->getNumJoints());
    ret_phi_.resize(scene_->getMapSize()/4);
    ret_jac_.resize(scene_->getMapSize()/4,scene_->getNumJoints());
    return SUCCESS;
  }
}

exotica::EReturn exotica::Orientation::taskSpaceDim(int & task_dim)
{
  if(!scene_)
  {
    task_dim = -1;
    ERROR("Kinematic scene has not been initialized!");
    return exotica::MMB_NIN;
  }
  else
  {
    task_dim = scene_->getMapSize()/4;
  }
  return exotica::SUCCESS;
}

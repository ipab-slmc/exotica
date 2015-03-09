#include "task_definition/TaskTerminationCriterion.h"

exotica::TaskTerminationCriterion::TaskTerminationCriterion()
{
  strength_ = CONTINUE;
}

exotica::EReturn exotica::TaskTerminationCriterion::initBase(tinyxml2::XMLHandle & handle)
{  
  if (!handle.ToElement()) { return PAR_ERR; }
  std::string temp_string(handle.ToElement()->Attribute("strength"));
  if (!temp_string.size()) { return PAR_ERR; }
  
  setStrength(temp_string.compare("soft") ? HARD_END : SOFT_END);
  
  return initDerived(handle); 
}


exotica::EReturn exotica::TaskTerminationCriterion::getStrength(ETerminate & strength)
{
  LOCK(strength_lock_);
  
  if (strength_)  //!< If set to something other than continue which is invalid!
  {
    strength = strength_;
    return SUCCESS;
  }
  else
  {
    return MMB_NIN;
  }
}

void exotica::TaskTerminationCriterion::setStrength(const ETerminate & strength)
{
  LOCK(strength_lock_);
  strength_ = strength;
} 

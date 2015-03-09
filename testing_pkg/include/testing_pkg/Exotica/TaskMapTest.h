#ifndef TESTING_TASK_MAP_TEST_H
#define TESTING_TASK_MAP_TEST_H

#include <exotica/EXOTica.hpp>
#include <kinematic_scene/kinematic_scene.h>
#include <tinyxml2/tinyxml2.h>
#include <gtest/gtest.h>
#include <Eigen/Eigen>
#include <cmath>

#include "testing_pkg/TestingTools.h"
#include "testing_pkg/Exotica/DummyClasses.h"

class TaskMapTest : public ::testing::Test  //!< Testing class for the exotica library
{
  public: 
  
    virtual void SetUp()
    {
      ASSERT_EQ(exotica::SUCCESS, (exotica::TaskMap_fac::Instance().listImplementations(registered_types_)));  //!< Ensure that the object loads
      std::string package_path;
      ASSERT_TRUE(findPackagePath("testing_pkg", package_path));  //!< Removes dependency on ros (in the future)
      resource_path_ = package_path.append("/resource/Exotica/");
    };
    
    virtual void TearDown() {};
  
    //!< Member Variables
    std::vector<std::string>                registered_types_;
    std::string                             resource_path_;
    boost::shared_ptr<exotica::TaskMap>     base_ptr_;
    kinematica::KinematicScene_map          kin_scenes_;
    
};

#endif

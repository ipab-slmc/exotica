#ifndef TESTING_PROBLEM_COMPONENT_TEST_H
#define TESTING_PROBLEM_COMPONENT_TEST_H

#include <exotica/EXOTica.hpp>
#include <tinyxml2/tinyxml2.h>
#include <gtest/gtest.h>
#include <Eigen/Eigen>

#include "testing_pkg/TestingTools.h"
#include "testing_pkg/Exotica/DummyClasses.h"

class TaskDefinitionTest : public ::testing::Test
{
  public:
    virtual void SetUp()
    {
      exotica::TaskMap_ptr task_ptr;
      ASSERT_EQ(exotica::SUCCESS, (exotica::TaskMap_fac::Instance().createObject("DTaskMap", task_ptr))) << "No Default TaskMap class";  //!< Ensure that there is a default TaskMap for testing
      task_maps_["Map1"] = task_ptr;
      
      std::string package_path;
      ASSERT_TRUE(findPackagePath("testing_pkg", package_path));  //!< Removes dependency on ros (in the future)
      resource_path_ = package_path.append("/resource/Exotica/");
    };
    
    virtual void TearDown() {};
  
    //!< Member Variables
    std::string                     resource_path_;
    exotica::TaskMap_map            task_maps_;
    exotica::TaskMap_map            empty_map_;
    testing::DTaskDefinition        problem_comp_;
};

#endif

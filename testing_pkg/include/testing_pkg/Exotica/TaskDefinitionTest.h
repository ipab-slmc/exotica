/*
 *      Author: Michael Camilleri
 * 
 * Copyright (c) 2016, University Of Edinburgh 
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 * 
 *  * Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 *  * Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  * Neither the name of  nor the names of its contributors may be used to 
 *    endorse or promote products derived from this software without specific 
 *    prior written permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE. 
 *
 */

#ifndef TESTING_PROBLEM_COMPONENT_TEST_H
#define TESTING_PROBLEM_COMPONENT_TEST_H

#include <exotica/EXOTica.hpp>
#include <tinyxml2/tinyxml2.h>
#include <gtest/gtest.h>
#include <Eigen/Eigen>

#include "testing_pkg/TestingTools.h"
#include "testing_pkg/Exotica/DummyClasses.h"

class TaskDefinitionTest: public ::testing::Test
{
  public:
    virtual void SetUp()
    {
      exotica::TaskMap_ptr task_ptr;
      ASSERT_EQ(exotica::SUCCESS, (exotica::TaskMap_fac::Instance().createObject("DTaskMap", task_ptr)))<< "No Default TaskMap class"; //!< Ensure that there is a default TaskMap for testing
      task_maps_["Map1"] = task_ptr;

      std::string package_path;
      ASSERT_TRUE(findPackagePath("testing_pkg", package_path)); //!< Removes dependency on ros (in the future)
      resource_path_ = package_path.append("/resource/Exotica/");
    }
    ;

    virtual void TearDown()
    {
    }
    ;

    //!< Member Variables
    std::string resource_path_;
    exotica::TaskMap_map task_maps_;
    exotica::TaskMap_map empty_map_;
    testing::DTaskDefinition problem_comp_;
};

#endif

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

#include "testing_pkg/Exotica/TaskDefinitionTest.h"

using namespace testing;

TEST_F(TaskDefinitionTest, DIT)  //!< Default Initialisation Test
{
  Eigen::VectorXd y;
  Eigen::MatrixXd y_dot;

  //!< Attempt access without Initialising
  EXPECT_EQ(exotica::MEM_ERR, problem_comp_.phi(y));
  EXPECT_EQ(exotica::MEM_ERR, problem_comp_.jacobian(y_dot));
}

TEST_F(TaskDefinitionTest, XIT) //!< XML Initialisation Test
{
  Eigen::VectorXd y, y2;
  Eigen::MatrixXd y_dot, y_dot2;
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLHandle handle(doc);

  //!< Attempt initialisation with invalid handle
  EXPECT_EQ(exotica::WARNING, problem_comp_.initBase(handle, empty_map_));
  problem_comp_.clearFlags();

  //!< Open valid handle and point to the first Task Tag
  ASSERT_EQ(tinyxml2::XML_NO_ERROR,
      doc.LoadFile(
          (resource_path_ + std::string("DTaskDefinition.xml")).c_str()));
  handle = tinyxml2::XMLHandle(doc.RootElement());
  handle = handle.FirstChildElement("Task");

  //!< Now try initialisation with valid handle but no mapping
  EXPECT_EQ(exotica::WARNING, problem_comp_.initBase(handle, task_maps_));
  EXPECT_TRUE(problem_comp_.derived_called);
  problem_comp_.clearFlags();
  EXPECT_EQ(exotica::MEM_ERR, problem_comp_.phi(y));
  EXPECT_EQ(exotica::MEM_ERR, problem_comp_.jacobian(y_dot));

  //!< Now move to next tag with invalid map definition
  handle = handle.NextSiblingElement("Task");

  //!< Retry initialisation
  EXPECT_EQ(exotica::PAR_ERR, problem_comp_.initBase(handle, task_maps_));
  EXPECT_FALSE(problem_comp_.derived_called);
  problem_comp_.clearFlags();
  EXPECT_EQ(exotica::MEM_ERR, problem_comp_.phi(y));
  EXPECT_EQ(exotica::MEM_ERR, problem_comp_.jacobian(y_dot));

  //!< Finally move to a valid initialiser
  handle = handle.NextSiblingElement("Task");

  //!< First try initialisation with unfound map
  EXPECT_EQ(exotica::PAR_ERR, problem_comp_.initBase(handle, empty_map_));
  EXPECT_FALSE(problem_comp_.derived_called);
  problem_comp_.clearFlags();
  EXPECT_EQ(exotica::MEM_ERR, problem_comp_.phi(y));
  EXPECT_EQ(exotica::MEM_ERR, problem_comp_.jacobian(y_dot));

  //!< Now retry with map found but do not update the TaskMap
  EXPECT_EQ(exotica::SUCCESS, problem_comp_.initBase(handle, task_maps_));
  EXPECT_TRUE(problem_comp_.derived_called);
  problem_comp_.clearFlags();
  EXPECT_EQ(exotica::MMB_NIN, problem_comp_.phi(y));
  EXPECT_EQ(exotica::MMB_NIN, problem_comp_.jacobian(y_dot));

  //!< Attempt initialisattion and call update
  EXPECT_EQ(exotica::SUCCESS,
      task_maps_["Map1"]->update(Eigen::VectorXd::Zero(3)));
  ASSERT_EQ(exotica::SUCCESS, problem_comp_.phi(y));
  ASSERT_EQ(exotica::SUCCESS, problem_comp_.jacobian(y_dot));
  ASSERT_EQ(exotica::SUCCESS, task_maps_["Map1"]->phi(y2));
  ASSERT_EQ(exotica::SUCCESS, task_maps_["Map1"]->jacobian(y_dot2));
  EXPECT_TRUE(compareVectors(y, y2));
  EXPECT_TRUE(compareMatrices(y_dot, y_dot2));
}

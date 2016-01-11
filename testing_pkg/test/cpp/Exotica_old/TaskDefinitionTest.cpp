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

TEST_F(ExoticaTaskTest, RCT) //!< Registration and Creation Test
{
  //!< Ensure that at least the two test task types are present
  std::string task1_name("ExoticaTask_1");
  std::string task2_name("ExoticaTask_2");
  bool task1_ok, task2_ok;
  task1_ok = task2_ok = false;
  for (int i = 0; i < registered_types_.size(); i++)
  {
    if (task1_name.compare(registered_types_[i]) == 0)
    {
      task1_ok = true;
    }
    if (task2_name.compare(registered_types_[i]) == 0)
    {
      task2_ok = true;
    }
  }
  if (!task1_ok)
  {
    FAIL()<< "Test Task 1 not registered";
  }
  if (!task2_ok)
  {
    FAIL()<< "Test Task 2 not registered";
  }
  task1_p_ = boost::dynamic_pointer_cast<ExoticaTaskTest_1>(
      exotica::TaskCreator::Instance()->createObject("ExoticaTask_3", params_));
  ASSERT_TRUE(task1_p_ == nullptr);
  task1_p_ = boost::dynamic_pointer_cast<ExoticaTaskTest_1>(
      exotica::TaskCreator::Instance()->createObject("ExoticaTask_1", params_));
  ASSERT_TRUE(task1_p_ != nullptr)<<" : Could not initialise object 1";
  task2_p_ = boost::dynamic_pointer_cast<ExoticaTaskTest_2>(
      exotica::TaskCreator::Instance()->createObject("ExoticaTask_2", params_));
  ASSERT_TRUE(task2_p_ != nullptr)<<" : Could not initialise object 2";

  std::string testing;
  EXPECT_TRUE(exotica::TestRegistrar::Instance()->findXML("ExoticaTask_1", testing))
      << " : Tests for Task 1 not registered";
  EXPECT_TRUE(exotica::TestRegistrar::Instance()->findXML("ExoticaTask_2", testing))
      << " : Tests for Task 2 not registered";
  EXPECT_FALSE(exotica::TestRegistrar::Instance()->findXML("ExoticaTask_3", testing))
      << " : Somehow registered for incorrect name";

  EXPECT_EQ(std::string("ExoticaTask_1"), task1_p_->name);
  EXPECT_EQ(std::string("ExoticaTask_2"), task2_p_->name);
}

TEST_F(ExoticaTaskTest, MIT) //!< Manual Initialisation Test
{
  params_.optimisation_window = -1;
  task1_p_ = boost::dynamic_pointer_cast<ExoticaTaskTest_1>(
      exotica::TaskCreator::Instance()->createObject("ExoticaTask_1", params_));
  ASSERT_TRUE(task1_p_ != nullptr)<<" : Could not initialise object";
  EXPECT_EQ(1, task1_p_->getParameters().optimisation_window)
      << " : Incorrect initialisation";

  //!< Initialisation of Goal Weights
  Eigen::MatrixXd goal_weights = Eigen::MatrixXd::Random(5, 5); //!< Create a random 5x5 matrix
  EXPECT_FALSE(task1_p_->setGoalWeights(goal_weights, 100));
  EXPECT_EQ(0, task1_p_->getGoalWeights(0).size());
  EXPECT_TRUE(task1_p_->setGoalWeights(goal_weights, 0));
  EXPECT_EQ(goal_weights, task1_p_->getGoalWeights(0));

  //!< Initialisation of Task Weight
  double task_weight = -0.2;
  EXPECT_FALSE(task1_p_->setTaskWeight(task_weight, 100));
  EXPECT_EQ(0, task1_p_->getTaskWeight(0));
  EXPECT_TRUE(task1_p_->setTaskWeight(task_weight, 0));
  EXPECT_EQ(task1_p_->getTaskWeight(0), task_weight);

  //!< Goal Param
  Eigen::VectorXd ones5 = Eigen::VectorXd::Ones(5);
  EXPECT_FALSE(task1_p_->setGoal(ones5, -1, 100));
  EXPECT_EQ(0, task1_p_->getGoal(0).size());
  EXPECT_TRUE(task1_p_->setGoal(ones5, -1, 0));
  EXPECT_EQ(ones5, task1_p_->getGoal(0));

  //!< Re0initialise with new size...
  params_.optimisation_window = 3;
  task1_p_ = boost::dynamic_pointer_cast<ExoticaTaskTest_1>(
      exotica::TaskCreator::Instance()->createObject("ExoticaTask_1", params_));
  ASSERT_TRUE(task1_p_ != nullptr)<<" : Could not initialise object";
  EXPECT_EQ(3, task1_p_->getParameters().optimisation_window)
      << " : Incorrect initialisation";
  EXPECT_TRUE(task1_p_->setGoalWeights(goal_weights, 2));
  EXPECT_EQ(goal_weights, task1_p_->getGoalWeights(2));
}

TEST_F(ExoticaTaskTest, XIT) //!< XML Initialisation Test
{
  boost::shared_ptr<tinyxml2::XMLHandle> xml_handle;
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(tinyxml2::XML_NO_ERROR,
      doc.LoadFile((resource_path_ + std::string("ex_task_1.xml")).c_str()));
  xml_handle.reset(new tinyxml2::XMLHandle(*(doc.RootElement())));
  ASSERT_NE(nullptr, xml_handle)<< " : Aborting: could  not initialise";

  params_.optimisation_window = 2;  //!< Test with 2 instances
  task1_p_ = boost::dynamic_pointer_cast<ExoticaTaskTest_1>(
      exotica::TaskCreator::Instance()->createObject("ExoticaTask_1", params_));
  ASSERT_TRUE(task1_p_ != nullptr)<<" : Could not initialise object";
  EXPECT_EQ(2, task1_p_->getParameters().optimisation_window)
      << " : Incorrect initialisation";
  EXPECT_TRUE(task1_p_->initBase(*xml_handle));
  EXPECT_TRUE(task1_p_->derived_called);
  task1_p_->clearFlags();

  //!< Goal Weights
  EXPECT_EQ(Eigen::MatrixXd::Identity(3, 3), task1_p_->getGoalWeights(0));
  EXPECT_EQ(Eigen::MatrixXd::Ones(3, 3), task1_p_->getGoalWeights(1));
  EXPECT_EQ(Eigen::MatrixXd::Zero(1, 1), task1_p_->getGoalWeights(2));

  //!< Task Weight
  EXPECT_FLOAT_EQ(0.5, task1_p_->getTaskWeight(0));
  EXPECT_FLOAT_EQ(0.2, task1_p_->getTaskWeight(1));
  EXPECT_GT(0, task1_p_->getTaskWeight(2));

  //!< Goal
  Eigen::VectorXd goal_0(3);
  Eigen::VectorXd goal_1(3);
  goal_0 << 0.2, 0.4, 0.6;
  goal_1 << 0.5, 0.5, 0.5;
  ASSERT_EQ(goal_0.size(), task1_p_->getGoal(0).size());
  EXPECT_EQ(goal_0, task1_p_->getGoal(0));
  EXPECT_FLOAT_EQ(0.2, task1_p_->getTolerance()); //!< Should give the zeroth one..
  ASSERT_EQ(goal_1.size(), task1_p_->getGoal(1).size());
  EXPECT_EQ(goal_1, task1_p_->getGoal(1));
  EXPECT_FLOAT_EQ(0.01, task1_p_->getTolerance(1));
  EXPECT_EQ(Eigen::VectorXd::Zero(1), task1_p_->getGoal(2));
  EXPECT_GT(0, task1_p_->getTolerance(2));

  //!< Private members (check that they are passed ok from base) (task of type 2)
  ASSERT_EQ(tinyxml2::XML_NO_ERROR,
      doc.LoadFile(
          (resource_path_ + std::string("ExoticaTask2_default.xml")).c_str()));
  xml_handle.reset(new tinyxml2::XMLHandle(*(doc.RootElement())));
  ASSERT_NE(nullptr, xml_handle)<< " : Aborting: could  not initialise";

  params_.optimisation_window = 1;  //!< Test with 1 instances
  task2_p_ = boost::dynamic_pointer_cast<ExoticaTaskTest_2>(
      exotica::TaskCreator::Instance()->createObject("ExoticaTask_2", params_));
  ASSERT_TRUE(task2_p_ != nullptr)<<" : Could not initialise object";
  EXPECT_EQ(1, task2_p_->getParameters().optimisation_window)
      << " : Incorrect initialisation";
  EXPECT_TRUE(task2_p_->initBase(*xml_handle));
  EXPECT_TRUE(task2_p_->derived_called);
  task2_p_->clearFlags();
  EXPECT_EQ(6, task2_p_->int_element);
  EXPECT_EQ(0, task2_p_->string_element.compare("blabla"));
}

TEST_F(ExoticaTaskTest, BXT) //!< Broken XML file Test
{
  boost::shared_ptr<tinyxml2::XMLHandle> xml_handle;
  tinyxml2::XMLDocument doc;
  task1_p_ = boost::dynamic_pointer_cast<ExoticaTaskTest_1>(
      exotica::TaskCreator::Instance()->createObject("ExoticaTask_1", params_));
  ASSERT_TRUE(task1_p_ != nullptr)<<" : Could not initialise object";
  EXPECT_EQ(1, task1_p_->getParameters().optimisation_window)
      << " : Incorrect initialisation";

  //!< Wrong Entries -- GoalWeights of incorrect size
  ASSERT_EQ(tinyxml2::XML_NO_ERROR,
      doc.LoadFile((resource_path_ + std::string("ex_task_2.xml")).c_str()));
  xml_handle.reset(new tinyxml2::XMLHandle(*(doc.RootElement())));
  ASSERT_NE(nullptr, xml_handle)<< " : Aborting: could  not initialise";
  EXPECT_FALSE(task1_p_->initBase(*xml_handle));
  EXPECT_FALSE(task1_p_->derived_called);
  task1_p_->clearFlags();

  //!< Wrong Entries -- GoalWeights with incorrect dimension (x)
  ASSERT_EQ(tinyxml2::XML_NO_ERROR,
      doc.LoadFile((resource_path_ + std::string("ex_task_10.xml")).c_str()));
  xml_handle.reset(new tinyxml2::XMLHandle(*(doc.RootElement())));
  ASSERT_NE(nullptr, xml_handle)<< " : Aborting: could  not initialise";
  EXPECT_FALSE(task1_p_->initBase(*xml_handle));
  EXPECT_FALSE(task1_p_->derived_called);
  task1_p_->clearFlags();

  //!< Wrong Entries -- GoalWeights with no dimension
  ASSERT_EQ(tinyxml2::XML_NO_ERROR,
      doc.LoadFile((resource_path_ + std::string("ex_task_11.xml")).c_str()));
  xml_handle.reset(new tinyxml2::XMLHandle(*(doc.RootElement())));
  ASSERT_NE(nullptr, xml_handle)<< " : Aborting: could  not initialise";
  EXPECT_FALSE(task1_p_->initBase(*xml_handle));
  EXPECT_FALSE(task1_p_->derived_called);
  task1_p_->clearFlags();

  //!< Wrong Entries -- Task Weight specified as string...
  ASSERT_EQ(tinyxml2::XML_NO_ERROR,
      doc.LoadFile((resource_path_ + std::string("ex_task_3.xml")).c_str()));
  xml_handle.reset(new tinyxml2::XMLHandle(*(doc.RootElement())));
  ASSERT_NE(nullptr, xml_handle)<< " : Aborting: could  not initialise";
  EXPECT_FALSE(task1_p_->initBase(*xml_handle));
  EXPECT_FALSE(task1_p_->derived_called);
  task1_p_->clearFlags();

  //!< Wrong Entries -- Goal Tolerance specified with invalid character
  ASSERT_EQ(tinyxml2::XML_NO_ERROR,
      doc.LoadFile((resource_path_ + std::string("ex_task_12.xml")).c_str()));
  xml_handle.reset(new tinyxml2::XMLHandle(*(doc.RootElement())));
  ASSERT_NE(nullptr, xml_handle)<< " : Aborting: could  not initialise";
  EXPECT_FALSE(task1_p_->initBase(*xml_handle));
  EXPECT_FALSE(task1_p_->derived_called);
  task1_p_->clearFlags();

  //!< Wrong Entries -- Goal specified with invalid character: will not fail since we do not know how much to read
  /*ASSERT_EQ(tinyxml2::XML_NO_ERROR, doc.LoadFile(resource_path_ + std::string("ex_task_13.xml").c_str()));
   xml_handle.reset(new tinyxml2::XMLHandle(*(doc.RootElement())));
   ASSERT_NE(nullptr, xml_handle) << " : Aborting: could  not initialise";
   EXPECT_FALSE(task1_p_->initBase(*xml_handle));
   EXPECT_FALSE(task1_p_->derived_called);
   task1_p_->clearFlags();*/

  //!< Wrong Entries -- No Goal Tolerance specified
  ASSERT_EQ(tinyxml2::XML_NO_ERROR,
      doc.LoadFile((resource_path_ + std::string("ex_task_14.xml")).c_str()));
  xml_handle.reset(new tinyxml2::XMLHandle(*(doc.RootElement())));
  ASSERT_NE(nullptr, xml_handle)<< " : Aborting: could  not initialise";
  EXPECT_FALSE(task1_p_->initBase(*xml_handle));
  EXPECT_FALSE(task1_p_->derived_called);
  task1_p_->clearFlags();

  //!< Incorrect Number of entries: 1 less
  ASSERT_EQ(tinyxml2::XML_NO_ERROR,
      doc.LoadFile((resource_path_ + std::string("ex_task_4.xml")).c_str()));
  xml_handle.reset(new tinyxml2::XMLHandle(*(doc.RootElement())));
  params_.optimisation_window = 2;
  task1_p_ = boost::dynamic_pointer_cast<ExoticaTaskTest_1>(
      exotica::TaskCreator::Instance()->createObject("ExoticaTask_1", params_));
  ASSERT_NE(nullptr, xml_handle)<< " : Aborting: could  not initialise";
  EXPECT_FALSE(task1_p_->initBase(*xml_handle));
  EXPECT_FALSE(task1_p_->derived_called);
  task1_p_->clearFlags();

  //!< No Time Element
  task1_p_ = boost::dynamic_pointer_cast<ExoticaTaskTest_1>(
      exotica::TaskCreator::Instance()->createObject("ExoticaTask_1", params_));
  ASSERT_TRUE(task1_p_ != nullptr)<<" : Could not initialise object";
  EXPECT_EQ(2, task1_p_->getParameters().optimisation_window)
      << " : Incorrect initialisation";
  ASSERT_EQ(tinyxml2::XML_NO_ERROR,
      doc.LoadFile((resource_path_ + std::string("ex_task_8.xml")).c_str()));
  xml_handle.reset(new tinyxml2::XMLHandle(*(doc.RootElement())));
  ASSERT_NE(nullptr, xml_handle)<< " : Aborting: could  not initialise";
  EXPECT_FALSE(task1_p_->initBase(*xml_handle));
  EXPECT_FALSE(task1_p_->derived_called);
  task1_p_->clearFlags();

  //!< No Task Paramaters
  task1_p_ = boost::dynamic_pointer_cast<ExoticaTaskTest_1>(
      exotica::TaskCreator::Instance()->createObject("ExoticaTask_1", params_));
  ASSERT_TRUE(task1_p_ != nullptr)<<" : Could not initialise object";
  EXPECT_EQ(2, task1_p_->getParameters().optimisation_window)
      << " : Incorrect initialisation";
  ASSERT_EQ(tinyxml2::XML_NO_ERROR,
      doc.LoadFile((resource_path_ + std::string("ex_task_9.xml")).c_str()));
  xml_handle.reset(new tinyxml2::XMLHandle(*(doc.RootElement())));
  ASSERT_NE(nullptr, xml_handle)<< " : Aborting: could  not initialise";
  EXPECT_FALSE(task1_p_->initBase(*xml_handle));
  EXPECT_FALSE(task1_p_->derived_called);
  task1_p_->clearFlags();
}

TEST_F(ExoticaTaskTest, IXT)  //!< Incomplete XML Test (or reordering)
{
  boost::shared_ptr<tinyxml2::XMLHandle> xml_handle;
  tinyxml2::XMLDocument doc;
  task1_p_ = boost::dynamic_pointer_cast<ExoticaTaskTest_1>(
      exotica::TaskCreator::Instance()->createObject("ExoticaTask_1", params_));
  ASSERT_TRUE(task1_p_ != nullptr)<<" : Could not initialise object";
  EXPECT_EQ(1, task1_p_->getParameters().optimisation_window)
      << " : Incorrect initialisation";

  //!< Incorrect Number of entries: 1 more: should not fail
  ASSERT_EQ(tinyxml2::XML_NO_ERROR,
      doc.LoadFile((resource_path_ + std::string("ex_task_1.xml")).c_str()));
  xml_handle.reset(new tinyxml2::XMLHandle(*(doc.RootElement())));
  ASSERT_NE(nullptr, xml_handle)<< " : Aborting: could  not initialise";
  EXPECT_TRUE(task1_p_->initBase(*xml_handle));
  EXPECT_TRUE(task1_p_->derived_called);
  task1_p_->clearFlags();
  EXPECT_EQ(Eigen::MatrixXd::Identity(3, 3), task1_p_->getGoalWeights(0));

  //!< Goal Weights not Specified
  task1_p_ = boost::dynamic_pointer_cast<ExoticaTaskTest_1>(
      exotica::TaskCreator::Instance()->createObject("ExoticaTask_1", params_));
  ASSERT_TRUE(task1_p_ != nullptr)<<" : Could not initialise object";
  EXPECT_EQ(1, task1_p_->getParameters().optimisation_window)
      << " : Incorrect initialisation";
  ASSERT_EQ(tinyxml2::XML_NO_ERROR,
      doc.LoadFile((resource_path_ + std::string("ex_task_5.xml")).c_str()));
  xml_handle.reset(new tinyxml2::XMLHandle(*(doc.RootElement())));
  ASSERT_NE(nullptr, xml_handle)<< " : Aborting: could  not initialise";
  EXPECT_TRUE(task1_p_->initBase(*xml_handle));
  EXPECT_TRUE(task1_p_->derived_called);
  task1_p_->clearFlags();
  EXPECT_EQ(0, task1_p_->getGoalWeights().size());

  //!< Task Weight not specified
  task1_p_ = boost::dynamic_pointer_cast<ExoticaTaskTest_1>(
      exotica::TaskCreator::Instance()->createObject("ExoticaTask_1", params_));
  ASSERT_TRUE(task1_p_ != nullptr)<<" : Could not initialise object";
  EXPECT_EQ(1, task1_p_->getParameters().optimisation_window)
      << " : Incorrect initialisation";
  ASSERT_EQ(tinyxml2::XML_NO_ERROR,
      doc.LoadFile((resource_path_ + std::string("ex_task_6.xml")).c_str()));
  xml_handle.reset(new tinyxml2::XMLHandle(*(doc.RootElement())));
  ASSERT_NE(nullptr, xml_handle)<< " : Aborting: could  not initialise";
  EXPECT_TRUE(task1_p_->initBase(*xml_handle));
  EXPECT_TRUE(task1_p_->derived_called);
  task1_p_->clearFlags();
  EXPECT_EQ(0, task1_p_->getTaskWeight());

  //!< Goal Not specified
  task1_p_ = boost::dynamic_pointer_cast<ExoticaTaskTest_1>(
      exotica::TaskCreator::Instance()->createObject("ExoticaTask_1", params_));
  ASSERT_TRUE(task1_p_ != nullptr)<<" : Could not initialise object";
  EXPECT_EQ(1, task1_p_->getParameters().optimisation_window)
      << " : Incorrect initialisation";
  ASSERT_EQ(tinyxml2::XML_NO_ERROR,
      doc.LoadFile((resource_path_ + std::string("ex_task_7.xml")).c_str()));
  xml_handle.reset(new tinyxml2::XMLHandle(*(doc.RootElement())));
  ASSERT_NE(nullptr, xml_handle)<< " : Aborting: could  not initialise";
  EXPECT_TRUE(task1_p_->initBase(*xml_handle));
  EXPECT_TRUE(task1_p_->derived_called);
  task1_p_->clearFlags();
  EXPECT_EQ(0, task1_p_->getGoal().size());
}

TEST_F(ExoticaTaskTest, DTT) //!< Derived-task Jacobian Tests: will contain tests to check whether the jacobian is correctly computed through finite-differences method
{
  boost::shared_ptr<exotica::TaskDefinition> task_ptr;
  for (int i = 0; i < registered_types_.size(); i++) //!< Iterate through the registered tasks
  {
    std::string xml_path;
    tinyxml2::XMLDocument document;
    boost::shared_ptr<tinyxml2::XMLHandle> element_ptr;
    if (exotica::TestRegistrar::Instance()->findXML(registered_types_[i],
        xml_path))  //!< If it is wished to be tested...
    {
      if (document.LoadFile((resource_path_ + xml_path).c_str())
          != tinyxml2::XML_NO_ERROR) //!< Attempt to load file
      {
        ADD_FAILURE()<< " : Could not Load initialiser for " << registered_types_[i] << " (file: "<<resource_path_ + xml_path <<")."; //!< Have to use this method to add failure since I do not want to continue for this object but do not wish to abort for all the others...
        continue;//!< Go to next object
      }
      //!< Initialise
      if (!(task_ptr = exotica::TaskCreator::Instance()->createObject(registered_types_[i], params_)))//!< If we could not create 
      {
        ADD_FAILURE() << " : Could not create object of type " << registered_types_[i]; //!< Have to use this method to add failure since I do not want to continue for this object but do not wish to abort for all the others...
        continue;//!< Go to next object
      }
      tinyxml2::XMLHandle xml_handle(document.RootElement()); //!< Get a handle to the root element
      if (!task_ptr->initBase(xml_handle))
      {
        ADD_FAILURE() << " : XML Initialiser is malformed for " << registered_types_[i];
        continue;
      }

      //!< Now our testing stuff...
      element_ptr.reset(new tinyxml2::XMLHandle(xml_handle.FirstChildElement("TestPoint")));
      if (!element_ptr)
      {
        ADD_FAILURE() << " : XML Tester is malformed for " << registered_types_[i];
        continue;
      }
      int j=0;
      while (element_ptr->ToElement()) //!< Iterate through all possible test cases
      {
        Eigen::VectorXd conf_space; //!< Vector containing the configuration point for testing
        Eigen::VectorXd task_space;//!< Vector with the task-space co-ordinate of the forward map
        Eigen::VectorXd phi;//!< The computed phi
        Eigen::MatrixXd jacobian;//!< The Jacobian computation
        if (!element_ptr->FirstChildElement("ConfSpace").ToElement())//!< If inexistent
        {
          ADD_FAILURE() << " : Missing Config for " << registered_types_[i] << " @ iteration " << j;
          element_ptr.reset(new tinyxml2::XMLHandle(element_ptr->NextSiblingElement("TestPoint")));
          j++;
          continue; //!< With the inner while loop...
        }
        if (!exotica::getVector(*(element_ptr->FirstChildElement("ConfSpace").ToElement()), conf_space))
        {
          ADD_FAILURE() << " : Could not parse Config Vector for " << registered_types_[i] << " @ iteration " << j;
          element_ptr.reset(new tinyxml2::XMLHandle(element_ptr->NextSiblingElement("TestPoint")));
          j++;
          continue; //!< With the inner while loop...
        }
        if (!element_ptr->FirstChildElement("TaskSpace").ToElement()) //!< If inexistent
        {
          ADD_FAILURE() << " : Missing Task Space point for " << registered_types_[i] << " @ iteration " << j;
          element_ptr.reset(new tinyxml2::XMLHandle(element_ptr->NextSiblingElement("TestPoint")));
          j++;
          continue; //!< With the inner while loop...
        }
        if (!exotica::getVector(*(element_ptr->FirstChildElement("TaskSpace").ToElement()), task_space))
        {
          ADD_FAILURE() << " : Could not parse task vector for " << registered_types_[i] << " @ iteration " << j;
          element_ptr.reset(new tinyxml2::XMLHandle(element_ptr->NextSiblingElement("TestPoint")));
          j++;
          continue; //!< With the inner while loop...
        }

        //!< First test forward mapping
        if (!task_ptr->updateTask(conf_space, 0))
        {
          ADD_FAILURE() << " for " << registered_types_[i] << " @ iteration " << j;
          element_ptr.reset(new tinyxml2::XMLHandle(element_ptr->NextSiblingElement("TestPoint")));
          j++;
          continue;
        }
        phi = task_ptr->getPhi(0);
        jacobian = task_ptr->getJacobian(0);
        if (phi.size() != task_space.size())
        {
          ADD_FAILURE() << " Task-size mismatch for " << registered_types_[i] << " @ iteration " << j;
          element_ptr.reset(new tinyxml2::XMLHandle(element_ptr->NextSiblingElement("TestPoint")));
          j++;
          continue;
        }
        EXPECT_TRUE(compareVectors(phi, task_space, TOLERANCE)) << " : Phi mismatch for " << registered_types_[i] << " @ iteration " << j;

        //!< Now the Jacobian (finite differences method)
        for (int k=0; k<conf_space.size(); k++)
        {
          Eigen::VectorXd conf_perturb = conf_space;
          conf_perturb(k) += EPSILON;
          if (!task_ptr->updateTask(conf_perturb, 0))
          { ADD_FAILURE() << " for " << registered_types_[i] << " @ iteration " << j << " in config-dimension " << k; continue;}
          if (task_ptr->getPhi(0).size() != phi.size())
          { ADD_FAILURE() << " for " << registered_types_[i] << " @ iteration " << j << " in config-dimension " << k; continue;}
          Eigen::VectorXd task_diff = (task_ptr->getPhi(0) - phi)/EPSILON; //!< Final - initial
          Eigen::VectorXd jac_column = jacobian.col(k);//!< Get the desired column (for the current joint)
          EXPECT_TRUE(compareVectors(task_diff, jac_column, TOLERANCE)) << " Incorrect for " << registered_types_[i] << " @ iteration " << j << " in config-dimension " << k << task_diff << "\n" << jac_column;
        }
        element_ptr.reset(new tinyxml2::XMLHandle(element_ptr->NextSiblingElement("TestPoint")));
        j++;
      } //!< Iteration through j
    } //check on testing
  } //!< Iteration through i
}


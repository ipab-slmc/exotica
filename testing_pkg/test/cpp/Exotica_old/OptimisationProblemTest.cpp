#include "testing_pkg/Exotica/OptimisationProblemTest.h"
#include "testing_pkg/Exotica/TaskType_1.h"
#include "testing_pkg/Exotica/TaskType_2.h"

TEST_F(ExoticaOptProbTest, XIT) //!< XML Initialisation Test
{
  //!< First attempt to open the Document and parse the optimisation problem
  tinyxml2::XMLDocument document;
  ASSERT_EQ(tinyxml2::XML_NO_ERROR, document.LoadFile((resource_path_ + std::string("OptimFunc_1.xml")).c_str()));//!< Attempt to open the file for parsing
  tinyxml2::XMLHandle xml_handle(document.RootElement());
  ASSERT_TRUE(problem.initBase(xml_handle, params_));

  //!< Now check for correct construction (ordering/Task types)
  ASSERT_EQ(2, problem.problem_.size());//!< Ensure that both levels are initialised
  EXPECT_TRUE(problem.problem_[0].getName().compare("TestFunction") == 0);//!< Ensure that the functions are correctly named
  EXPECT_TRUE(problem.problem_[1].getName().compare("Problem_1") == 0);
  EXPECT_EQ(2, problem.problem_[0].getNumTasks());
  EXPECT_EQ(1, problem.problem_[1].getNumTasks());

  if(!problem.problem_[0].access("Task0").lock())
  {
    ADD_FAILURE() << "First task of incorrect name";
  }
  else
  {
    EXPECT_EQ(0, boost::dynamic_pointer_cast<ExoticaTaskTest_1>(problem.problem_[0].access("Task0").lock())->name.compare("ExoticaTask_1")) << "First task is of incorrect type";
    EXPECT_TRUE(boost::dynamic_pointer_cast<ExoticaTaskTest_1>(problem.problem_[0].access("Task0").lock())->derived_called);
  }
  if(!problem.problem_[0].access("Task_1").lock())
  {
    ADD_FAILURE() << "Second task of incorrect (auto) name";
  }
  else
  {
    EXPECT_EQ(0, boost::dynamic_pointer_cast<ExoticaTaskTest_2>(problem.problem_[0].access("Task_1").lock())->name.compare("ExoticaTask_2")) << "Second task is of incorrect type";
    EXPECT_TRUE(boost::dynamic_pointer_cast<ExoticaTaskTest_2>(problem.problem_[0].access("Task_1").lock())->derived_called);
  }

  if(problem.problem_[1].access("Task2").lock() == nullptr)
  {
    ADD_FAILURE() << "Third task of incorrect name";
  }
  else
  {

    EXPECT_EQ(0, boost::dynamic_pointer_cast<ExoticaTaskTest_1>(problem.problem_[1].access("Task2").lock())->name.compare("ExoticaTask_1")) << "Third task is of incorrect type";
    EXPECT_TRUE(boost::dynamic_pointer_cast<ExoticaTaskTest_1>(problem.problem_[1].access("Task2").lock())->derived_called);
  }

  //!< Now check for correct Weighting Initialisation (in so doing also testing the getWeights() function)
  Eigen::MatrixXd tasks_0 = Eigen::MatrixXd::Zero(6,6);
  tasks_0.block(0,0,3,3) = Eigen::MatrixXd::Identity(3,3)*0.5;
  tasks_0.block(3,3,3,3) = Eigen::MatrixXd::Identity(3,3)*0.8;
  Eigen::MatrixXd config, task;
  if (problem.problem_[0].getWeights(config, task))//!< If got weights
  {
    EXPECT_TRUE(compareMatrices(Eigen::MatrixXd::Identity(3,3)*0.001, config)) << "\n" << config << "\n";
    EXPECT_TRUE(compareMatrices(tasks_0, task));
  }
  else
  {
    ADD_FAILURE() << "Could not compute weights for Function 0";
  }
  Eigen::MatrixXd tasks_1 = Eigen::MatrixXd::Zero(2,2);
  tasks_1(0,0) = 1.0;
  tasks_1(1,1) = 0.5;

  if (problem.problem_[1].getWeights(config, task)) //!< If got weights
  {
    EXPECT_TRUE(compareMatrices(Eigen::MatrixXd::Ones(1,1)*0.001, config));
    EXPECT_TRUE(compareMatrices(tasks_1, task));
  }
  else
  {
    ADD_FAILURE()<< "Could not compute weights for Function 1";
  }
}

TEST_F(ExoticaOptProbTest, BXT) //!< Broken Xml Test
{
  //!< Missing Task Definition for the first optimisation problem
  tinyxml2::XMLDocument document;
  ASSERT_EQ(tinyxml2::XML_NO_ERROR, document.LoadFile((resource_path_ + std::string("OptimFunc_2.xml")).c_str()));//!< Attempt to open the file for parsing
  {
    tinyxml2::XMLHandle xml_handle(document.RootElement());
    ASSERT_FALSE(problem.initBase(xml_handle, params_));
    ASSERT_EQ(0, problem.problem_.size());  //!< Problem should be uninitialised
  }

  //!< Unknown Task Type
  problem.problem_.clear();//!< Re-initialise
  ASSERT_EQ(tinyxml2::XML_NO_ERROR, document.LoadFile((resource_path_ + std::string("OptimFunc_3.xml")).c_str()));//!< Attempt to open the file for parsing
  {
    tinyxml2::XMLHandle xml_handle(document.RootElement());
    ASSERT_FALSE(problem.initBase(xml_handle, params_));
    ASSERT_EQ(0, problem.problem_.size());  //!< Problem should be uninitialised
  }

  //!< Two Tasks with same name at same level
  problem.problem_.clear();//!< Re-initialise
  ASSERT_EQ(tinyxml2::XML_NO_ERROR, document.LoadFile((resource_path_ + std::string("OptimFunc_4.xml")).c_str()));//!< Attempt to open the file for parsing
  {
    tinyxml2::XMLHandle xml_handle(document.RootElement());
    ASSERT_FALSE(problem.initBase(xml_handle, params_));
    ASSERT_EQ(0, problem.problem_.size());  //!< Problem should be uninitialised
  }

  //!< Failure at second level
  problem.problem_.clear();//!< Re-initialise
  ASSERT_EQ(tinyxml2::XML_NO_ERROR, document.LoadFile((resource_path_ + std::string("OptimFunc_5.xml")).c_str()));//!< Attempt to open the file for parsing
  {
    tinyxml2::XMLHandle xml_handle(document.RootElement());
    ASSERT_FALSE(problem.initBase(xml_handle, params_));
    ASSERT_EQ(0, problem.problem_.size());  //!< Problem should be uninitialised
  }
}

TEST_F(ExoticaOptProbTest, WFT) //!< Wrapper Functions Test
{
  //!< Temporaries
  tinyxml2::XMLDocument document;
  boost::shared_ptr<tinyxml2::XMLHandle> xml_handle_p;
  Eigen::MatrixXd temp_mat_1, temp_mat_2;
  Eigen::VectorXd temp_vect_1, temp_vect_2;
  bool ok;

  //!< Task Deletion
  ASSERT_EQ(tinyxml2::XML_NO_ERROR, document.LoadFile((resource_path_ + std::string("OptimFunc_6.xml")).c_str()));//!< Attempt to open the file for parsing
  xml_handle_p.reset(new tinyxml2::XMLHandle(document.RootElement()));
  ASSERT_TRUE(problem.initBase(*xml_handle_p, params_));
  EXPECT_TRUE(problem.problem_[0].deleteTask("Task0"));
  EXPECT_EQ(nullptr, problem.problem_[0].access("Task0").lock());//!< Should not be there anymore
  problem.problem_.clear();

  //!< State update
  ASSERT_TRUE(problem.initBase(*xml_handle_p, params_));
  EXPECT_FALSE(problem.problem_[0].updateState(Eigen::VectorXd::Zero(2)));//!< Incorrect vector size
  EXPECT_FALSE(problem.problem_[0].updateState(Eigen::VectorXd::Zero(3), 1));//!< Incorrect index
  ASSERT_TRUE(problem.problem_[0].updateState(Eigen::VectorXd::Ones(3), 0));//!< Correct...
  if(!problem.problem_[0].access("Task0").lock())
  {
    ADD_FAILURE() << "Something wrong in First Task";
  }
  else
  {
    EXPECT_TRUE(boost::dynamic_pointer_cast<ExoticaTaskTest_1>(problem.problem_[0].access("Task0").lock())->update_called);
  }
  if(!problem.problem_[0].access("Task_1").lock())
  {
    ADD_FAILURE() << "Something wrong in First Task";
  }
  else
  {
    EXPECT_TRUE(boost::dynamic_pointer_cast<ExoticaTaskTest_2>(problem.problem_[0].access("Task_1").lock())->update_called);
  }

  //!< Now get Jacobian (from same configuration)
  ASSERT_TRUE(problem.problem_[0].getJacobian(temp_mat_1));
  ASSERT_EQ(2, temp_mat_1.rows());
  ASSERT_EQ(3, temp_mat_1.cols());
  temp_mat_2.resize(2,3);
  temp_mat_2 << 2, 2, 1, 3, 4, -4;
  EXPECT_EQ(temp_mat_2, temp_mat_1);

  //!< Now get Task Error (from same configuration)
  temp_vect_2.resize(2);
  temp_vect_2 << -3.8, -1.2;
  ASSERT_TRUE(problem.problem_[0].getTaskError(temp_vect_1, ok));
  EXPECT_FALSE(ok);
  ASSERT_EQ(2, temp_vect_1.size());
  EXPECT_EQ(temp_vect_2, temp_vect_1);

  //!< Change Goal
  temp_vect_2.resize(1);
  temp_vect_2 << 3.9;
  ASSERT_TRUE(problem.problem_[0].access("Task0").lock() != nullptr);
  EXPECT_TRUE(problem.problem_[0].access("Task0").lock()->setGoal(temp_vect_2, 0.2));
  temp_vect_2 << 2;
  ASSERT_TRUE(problem.problem_[0].access("Task_1").lock() != nullptr);
  EXPECT_TRUE(problem.problem_[0].access("Task_1").lock()->setGoal(temp_vect_2, 0.01));
  temp_vect_2.resize(2);
  temp_vect_2 << -0.1, 0;
  ASSERT_TRUE(problem.problem_[0].getTaskError(temp_vect_1, ok));
  EXPECT_TRUE(ok);
  ASSERT_EQ(2, temp_vect_1.size());
  EXPECT_TRUE(compareVectors(temp_vect_2, temp_vect_1, TOLERANCE));

  //!< Missing Goal
  ASSERT_TRUE(problem.problem_[1].updateState(Eigen::VectorXd::Ones(3), 0));//!< Correct...
  ASSERT_TRUE(problem.problem_[1].access("Task2").lock() != nullptr);
  ASSERT_TRUE(boost::dynamic_pointer_cast<ExoticaTaskTest_1>(problem.problem_[1].access("Task2").lock())->update_called);
  EXPECT_EQ(2, problem.problem_[1].access("Task2").lock()->getTaskDimension());
  EXPECT_FALSE(problem.problem_[1].getTaskError(temp_vect_1, ok));
}


#include "testing_pkg/Exotica/PlanningProblemTest.h"

TEST_F(PlanningProblemTest, DIT)  //!< Default Initialisation Test
{
  ASSERT_EQ(exotica::SUCCESS, exotica::PlanningProblem_fac::Instance().createObject("DPlanningProblem", base_ptr_));  
  EXPECT_EQ(0, base_ptr_->type().compare("testing::DPlanningProblem"));
  EXPECT_EQ(exotica::WARNING, base_ptr_->update(Eigen::VectorXd::Zero(3)));
}

TEST_F(PlanningProblemTest, KXT) //!< KinematicScene XML initialisation Test
{
  //!< Temporaries
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLHandle   root(doc);
  
  //!< Initialise
  ASSERT_EQ(exotica::SUCCESS, exotica::PlanningProblem_fac::Instance().createObject("DPlanningProblem", base_ptr_));  //!< Create object
  ASSERT_EQ(tinyxml2::XML_NO_ERROR, doc.LoadFile((resource_path_ + std::string("DPlanningProblem_1.xml")).c_str()));
  root = tinyxml2::XMLHandle(doc.RootElement());
  
  //!<No Name specified
  root = root.FirstChildElement("Problem");
  EXPECT_EQ(exotica::PAR_ERR, base_ptr_->initBase(root));
  EXPECT_TRUE(boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->checkInvalid());
  boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->refresh();
  
  //!< Multiple KScene's with same name
  root = root.NextSiblingElement("Problem");
  EXPECT_EQ(exotica::PAR_ERR, base_ptr_->initBase(root));
  EXPECT_TRUE(boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->checkInvalid());
  boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->refresh();
  
  //!< Incorrectly specified KScene...
  root = root.NextSiblingElement("Problem");
  EXPECT_EQ(exotica::FAILURE, base_ptr_->initBase(root));
  EXPECT_TRUE(boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->checkInvalid());
  boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->refresh();
}


TEST_F(PlanningProblemTest, MXT) //!< TaskMap XML initialisation Test
{
  //!< Temporaries
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLHandle   root(doc);
  
  //!< Initialise
  ASSERT_EQ(exotica::SUCCESS, exotica::PlanningProblem_fac::Instance().createObject("DPlanningProblem", base_ptr_));  //!< Create object
  ASSERT_EQ(tinyxml2::XML_NO_ERROR, doc.LoadFile((resource_path_ + std::string("DPlanningProblem_2.xml")).c_str()));
  root = tinyxml2::XMLHandle(doc.RootElement());
  
  //!<No Type Specified
  root = root.FirstChildElement("Problem");
  EXPECT_EQ(exotica::PAR_ERR, base_ptr_->initBase(root));
  EXPECT_TRUE(boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->checkInvalid());
  boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->refresh();
  
  //!< Incorrect Type specified
  root = root.NextSiblingElement("Problem");
  EXPECT_EQ(exotica::PAR_ERR, base_ptr_->initBase(root));
  EXPECT_TRUE(boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->checkInvalid());
  boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->refresh();
  
  //!< No Name specified
  root = root.NextSiblingElement("Problem");
  EXPECT_EQ(exotica::PAR_ERR, base_ptr_->initBase(root));
  EXPECT_TRUE(boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->checkInvalid());
  boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->refresh();
  
  //!< Two maps with same name...
  root = root.NextSiblingElement("Problem");
  EXPECT_EQ(exotica::PAR_ERR, base_ptr_->initBase(root));
  EXPECT_TRUE(boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->checkInvalid());
  boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->refresh();
}

TEST_F(PlanningProblemTest, DXT) //!< TaskDefinition XML initialisation Test
{
  //!< Temporaries
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLHandle   root(doc);
  
  //!< Initialise
  ASSERT_EQ(exotica::SUCCESS, exotica::PlanningProblem_fac::Instance().createObject("DPlanningProblem", base_ptr_));  //!< Create object
  ASSERT_EQ(tinyxml2::XML_NO_ERROR, doc.LoadFile((resource_path_ + std::string("DPlanningProblem_3.xml")).c_str()));
  root = tinyxml2::XMLHandle(doc.RootElement());
  
  //!<No Type Specified
  root = root.FirstChildElement("Problem");
  EXPECT_EQ(exotica::PAR_ERR, base_ptr_->initBase(root));
  EXPECT_TRUE(boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->checkInvalid());
  boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->refresh();
  
  //!< Incorrect Type specified
  root = root.NextSiblingElement("Problem");
  EXPECT_EQ(exotica::PAR_ERR, base_ptr_->initBase(root));
  EXPECT_TRUE(boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->checkInvalid());
  boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->refresh();
  
  //!< No Name specified
  root = root.NextSiblingElement("Problem");
  EXPECT_EQ(exotica::PAR_ERR, base_ptr_->initBase(root));
  EXPECT_TRUE(boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->checkInvalid());
  boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->refresh();
  
  //!< Two Definitions with same name...
  root = root.NextSiblingElement("Problem");
  EXPECT_EQ(exotica::PAR_ERR, base_ptr_->initBase(root));
  EXPECT_TRUE(boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->checkInvalid());
  boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->refresh();
}

TEST_F(PlanningProblemTest, GCT) //!< Global Component XML initialisation Test
{
  //!< Temporaries
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLHandle   root(doc);
  
  //!< Initialise
  ASSERT_EQ(exotica::SUCCESS, exotica::PlanningProblem_fac::Instance().createObject("DPlanningProblem", base_ptr_));  //!< Create object
  ASSERT_EQ(tinyxml2::XML_NO_ERROR, doc.LoadFile((resource_path_ + std::string("DPlanningProblem_4.xml")).c_str()));
  root = tinyxml2::XMLHandle(doc.RootElement());
  
  //!<No KScene specified
  root = root.FirstChildElement("Problem");
  EXPECT_EQ(exotica::WARNING, base_ptr_->initBase(root));
  EXPECT_FALSE(boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->checkInvalid());
  boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->refresh();
  
  //!< No Map specified
  root = root.NextSiblingElement("Problem");
  EXPECT_EQ(exotica::WARNING, base_ptr_->initBase(root));
  EXPECT_FALSE(boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->checkInvalid());
  boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->refresh();
  
  //!< No Task Definiton Specified
  root = root.NextSiblingElement("Problem");
  EXPECT_EQ(exotica::WARNING, base_ptr_->initBase(root));
  EXPECT_FALSE(boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->checkInvalid());
  boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->refresh();
}

TEST_F(PlanningProblemTest, CIT) //!< Correct Instantiation Test
{
  //!< Temporaries
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLHandle   root(doc);
  
  //!< Initialise
  ASSERT_EQ(exotica::SUCCESS, exotica::PlanningProblem_fac::Instance().createObject("DPlanningProblem", base_ptr_));  //!< Create object
  ASSERT_EQ(tinyxml2::XML_NO_ERROR, doc.LoadFile((resource_path_ + std::string("DPlanningProblem.xml")).c_str()));
  root = tinyxml2::XMLHandle(doc.RootElement());
  root = root.FirstChildElement("Problem");
  ASSERT_TRUE(exotica::ok(base_ptr_->initBase(root)));
  EXPECT_TRUE(boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->derived_called);
  
  //!< Check the kinematic scenes array
  EXPECT_EQ(1, boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->scenes_ref_.size());
  EXPECT_EQ(0, boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->scenes_ref_.begin()->first.compare("kscene1"));
  
  //!< Check the Maps
  EXPECT_EQ(2, boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->maps_ref_.size());
  auto map_it = boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->maps_ref_.begin();
  for (int i=0; i<2; i++, map_it++)
  {
    std::string name(std::to_string(i+1));
    EXPECT_EQ(0, map_it->first.compare(std::string("map").append(name)));
    EXPECT_EQ(0, (boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->maps_ref_.begin()->second)->type().compare("testing::DTaskMap"));
  }
  
  //!< Check the TaskDefinitions
  EXPECT_EQ(3, boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->defs_ref_.size());
  auto def_it = boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->defs_ref_.begin();
  for (int i=0; i<3; i++, def_it++)
  {
    std::string name(std::to_string(i+1));
    EXPECT_EQ(0, def_it->first.compare(std::string("task").append(name)));
    EXPECT_EQ(0, boost::dynamic_pointer_cast<testing::DPlanningProblem>(base_ptr_)->defs_ref_.begin()->second->type().compare("testing::DTaskDefinition"));
  }
}


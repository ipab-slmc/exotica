#include "testing_pkg/Exotica/TaskMapTest.h"


TEST_F(TaskMapTest, DIT)  //!< Default Initialisation Test
{
  Eigen::VectorXd phi;
  Eigen::MatrixXd jac;
  
  ASSERT_EQ(exotica::SUCCESS, (exotica::TaskMap_fac::Instance().createObject("DTaskMap", base_ptr_)));
  EXPECT_EQ(0, base_ptr_->type().compare("testing::DTaskMap"));
  EXPECT_EQ(exotica::MMB_NIN, base_ptr_->phi(phi));
  EXPECT_EQ(exotica::MMB_NIN, base_ptr_->jacobian(jac));
}

TEST_F(TaskMapTest, XIT)  //!< XML Initialisation Test
{
  //!< Pre-requisites
  tinyxml2::XMLDocument doc;  
  ASSERT_EQ(exotica::SUCCESS, (exotica::TaskMap_fac::Instance().createObject("DTaskMap", base_ptr_)));
  ASSERT_EQ(tinyxml2::XML_NO_ERROR, doc.LoadFile((resource_path_ + std::string("DTaskMap.xml")).c_str()));
  tinyxml2::XMLHandle handle(doc.RootElement());
  
  //!< Set Handle to first Element 
  tinyxml2::XMLHandle map_handle(handle.FirstChildElement("Map"));
  
  //!< Test with default parameter
  EXPECT_EQ(exotica::SUCCESS, base_ptr_->initBase(map_handle));
  EXPECT_TRUE(boost::dynamic_pointer_cast<testing::DTaskMap>(base_ptr_)->derived_called);
  EXPECT_EQ(nullptr, boost::dynamic_pointer_cast<testing::DTaskMap>(base_ptr_)->scene_ptr_);
  
  //!< Test with Empty map
  boost::dynamic_pointer_cast<testing::DTaskMap>(base_ptr_)->clearFlags();
  EXPECT_EQ(exotica::SUCCESS, base_ptr_->initBase(map_handle, kin_scenes_));
  EXPECT_TRUE(boost::dynamic_pointer_cast<testing::DTaskMap>(base_ptr_)->derived_called);
  EXPECT_EQ(nullptr, boost::dynamic_pointer_cast<testing::DTaskMap>(base_ptr_)->scene_ptr_);
  
  //!< Move to next handle which requests a KScene
  map_handle = map_handle.NextSiblingElement("Map");
  
  //!< Test with default parameter
  boost::dynamic_pointer_cast<testing::DTaskMap>(base_ptr_)->clearFlags();
  EXPECT_EQ(exotica::PAR_ERR, base_ptr_->initBase(map_handle));
  EXPECT_FALSE(boost::dynamic_pointer_cast<testing::DTaskMap>(base_ptr_)->derived_called);
  EXPECT_EQ(nullptr, boost::dynamic_pointer_cast<testing::DTaskMap>(base_ptr_)->scene_ptr_);
  
  //!< Test with Empty map
  boost::dynamic_pointer_cast<testing::DTaskMap>(base_ptr_)->clearFlags();
  EXPECT_EQ(exotica::PAR_ERR, base_ptr_->initBase(map_handle, kin_scenes_));
  EXPECT_FALSE(boost::dynamic_pointer_cast<testing::DTaskMap>(base_ptr_)->derived_called);
  EXPECT_EQ(nullptr, boost::dynamic_pointer_cast<testing::DTaskMap>(base_ptr_)->scene_ptr_);
  
  //!< Test with correctly defined kscene pointer
  kin_scenes_["kscene1"] = kinematica::KinematicScene_ptr();
  boost::dynamic_pointer_cast<testing::DTaskMap>(base_ptr_)->clearFlags();
  EXPECT_EQ(exotica::SUCCESS, base_ptr_->initBase(map_handle, kin_scenes_));
  EXPECT_TRUE(boost::dynamic_pointer_cast<testing::DTaskMap>(base_ptr_)->derived_called);
  EXPECT_EQ(nullptr, boost::dynamic_pointer_cast<testing::DTaskMap>(base_ptr_)->scene_ptr_);
  
  //!< Move to next handle which requests a KScene but does not define it
  map_handle = map_handle.NextSiblingElement("Map");
  boost::dynamic_pointer_cast<testing::DTaskMap>(base_ptr_)->clearFlags();
  EXPECT_EQ(exotica::PAR_ERR, base_ptr_->initBase(map_handle, kin_scenes_));
  EXPECT_FALSE(boost::dynamic_pointer_cast<testing::DTaskMap>(base_ptr_)->derived_called);
  EXPECT_EQ(nullptr, boost::dynamic_pointer_cast<testing::DTaskMap>(base_ptr_)->scene_ptr_);
}

TEST_F(TaskMapTest, DMT)  //!< Derived-task Mapping Tests: will contain tests to check whether the jacobian is correctly computed through finite-differences method
{
  for (int i=0; i<registered_types_.size(); i++)  //!< Iterate through the registered tasks
  {
    std::string             xml_path;
    tinyxml2::XMLDocument   xml_document;
    tinyxml2::XMLHandle     xml_handle(xml_document);
    
    kin_scenes_.clear();  //!< Erase all scenes: by definition, since it is a shared_ptr, as soon as they go out of scope they will be deleted
    if (exotica::XMLTester::Instance().getTest(registered_types_[i], xml_path) == exotica::SUCCESS)  //!< If it is wished to be tested...
    {
      //!< Prepare
      std::string name;
      
      if (xml_document.LoadFile((resource_path_ + xml_path).c_str()) != tinyxml2::XML_NO_ERROR) //!< Attempt to load file
      {
        ADD_FAILURE() << " : Initialiser for " << registered_types_[i] << " (file: "<<resource_path_ + xml_path <<") is invalid."; //!< Have to use this method to add failure since I do not want to continue for this object but do not wish to abort for all the others...
        continue; //!< Go to next object
      }
      
      if (exotica::TaskMap_fac::Instance().createObject(registered_types_[i], base_ptr_))  //!< Could not create
      {
        ADD_FAILURE() << " : Could not create object of type " << registered_types_[i];
        continue;
      }
      
      xml_handle = tinyxml2::XMLHandle(xml_document.RootElement());
     
      //!< First initialise a Kinematic Scene if needed
      tinyxml2::XMLHandle scene_handle(xml_handle.FirstChildElement("KScene"));
      CHECK_OK;
      if (scene_handle.ToElement()) //!< If one is needed
      {
        name = std::string(scene_handle.ToElement()->Attribute("name"));
        if (name.empty())
        {
          ADD_FAILURE() << " : No name specified for scene";
          continue;
        }
        CHECK_OK;
        kin_scenes_[name].reset(new kinematica::KinematicScene(name));
        CHECK_OK;
        if (kin_scenes_[name] == nullptr)
        {
          ADD_FAILURE() << " : Could not create " << name << " KinematicScene!";
          continue;
        }
        CHECK_OK;
        if (!kin_scenes_[name]->initKinematicScene(scene_handle))
        {
          ADD_FAILURE() << " : Could not initialise " << name << " KinematicScene!";
          continue;
        }
        CHECK_OK;
      }
      
      tinyxml2::XMLHandle map_handle(xml_handle.FirstChildElement("Map"));
      if (base_ptr_->initBase(map_handle, kin_scenes_))  //!< Something wrong in initialisation
      {
        
        ADD_FAILURE() << " : XML Initialiser is malformed for " << registered_types_[i] << "\nFile name: " << xml_path;
        continue;
      }
      CHECK_OK;
      //!< Test
      tinyxml2::XMLHandle test_handle(xml_handle.FirstChildElement("TestPoint"));  //!< Locate the first test-point
      CHECK_OK;
      if (!test_handle.ToElement())
      {
        ADD_FAILURE() << " : XML Tester is malformed for " << registered_types_[i];
        continue;
      }
      CHECK_OK;
      int j=0;
      while (test_handle.ToElement())
      {
        CHECK_OK;
        Eigen::VectorXd conf_space; //!< Vector containing the configuration point for testing
        Eigen::VectorXd task_space; //!< Vector with the task-space co-ordinate of the forward map
        Eigen::VectorXd phi;        //!< The computed phi
        Eigen::MatrixXd jacobian;   //!< The Jacobian computation
        
        //!< Prepare
        if (!test_handle.FirstChildElement("ConfSpace").ToElement()) //!< If inexistent
        {
          ADD_FAILURE() << " : Missing Config for " << registered_types_[i] << " @ iteration " << j;
          test_handle = tinyxml2::XMLHandle(test_handle.NextSiblingElement("TestPoint"));
          j++;
          continue; //!< With the inner while loop...
        }
        CHECK_OK;
        if (exotica::getVector(*(test_handle.FirstChildElement("ConfSpace").ToElement()), conf_space) != exotica::SUCCESS)
        {
          ADD_FAILURE() << " : Could not parse Config Vector for " << registered_types_[i] << " @ iteration " << j;
          test_handle = tinyxml2::XMLHandle(test_handle.NextSiblingElement("TestPoint"));
          j++;
          continue; //!< With the inner while loop...
        }
        CHECK_OK;
        if (!test_handle.FirstChildElement("TaskSpace").ToElement()) //!< If inexistent
        {
          ADD_FAILURE() << " : Missing Task Space point for " << registered_types_[i] << " @ iteration " << j;
          test_handle = tinyxml2::XMLHandle(test_handle.NextSiblingElement("TestPoint"));
          j++;
          continue; //!< With the inner while loop...
        }
        CHECK_OK;
        if (exotica::getVector(*(test_handle.FirstChildElement("TaskSpace").ToElement()), task_space) != exotica::SUCCESS)
        {
          ADD_FAILURE() << " : Could not parse task vector for " << registered_types_[i] << " @ iteration " << j;
          test_handle = tinyxml2::XMLHandle(test_handle.NextSiblingElement("TestPoint"));
          j++;
          continue; //!< With the inner while loop...
        }
        CHECK_OK;
        if (!kin_scenes_.empty())
        {
          if (!kin_scenes_[name]->update(conf_space))
          {
            ADD_FAILURE() << " for " << registered_types_[i] << " @ iteration " << j;
            test_handle = tinyxml2::XMLHandle(test_handle.NextSiblingElement("TestPoint"));
            j++;
            continue; //!< With the inner while loop...
          }
        }
        CHECK_OK;
        if (base_ptr_->update(conf_space) != exotica::SUCCESS)
        {
          ADD_FAILURE() << " for " << registered_types_[i] << " @ iteration " << j;
          test_handle = tinyxml2::XMLHandle(test_handle.NextSiblingElement("TestPoint"));
          j++;
          continue; //!< With the inner while loop...
        }
              CHECK_OK;
        if (base_ptr_->phi(phi) != exotica::SUCCESS)
        {
          ADD_FAILURE() << " for " << registered_types_[i] << " @ iteration " << j;
          test_handle = tinyxml2::XMLHandle(test_handle.NextSiblingElement("TestPoint"));
          j++;
          continue; //!< With the inner while loop...
        }
              CHECK_OK;
        if (base_ptr_->jacobian(jacobian) != exotica::SUCCESS)
        {
          ADD_FAILURE() << " for " << registered_types_[i] << " @ iteration " << j;
          test_handle = tinyxml2::XMLHandle(test_handle.NextSiblingElement("TestPoint"));
          j++;
          continue; //!< With the inner while loop...
        }
              CHECK_OK;
        //!< Check the Forward mapping
        EXPECT_TRUE(compareVectors(phi, task_space, TOLERANCE)) << " : Phi mismatch for " << registered_types_[i] << " @ iteration " << j;
        
        //!< Now the Jacobian (finite differences method)
        for (int k=0; k<conf_space.size(); k++)
        {
          
          Eigen::VectorXd temp_phi;
          Eigen::VectorXd conf_perturb = conf_space;
          conf_perturb(k) += EPSILON;
          
          if (base_ptr_->update(conf_perturb) != exotica::SUCCESS) { ADD_FAILURE() << " for " << registered_types_[i] << " @ iteration " << j << " in config-dimension " << k; continue; }
          
          if (base_ptr_->phi(temp_phi) != exotica::SUCCESS) { ADD_FAILURE() << " for " << registered_types_[i] << " @ iteration " << j << " in config-dimension " << k; continue; }
          
          if (temp_phi.size() != phi.size()) { ADD_FAILURE() << " for " << registered_types_[i] << " @ iteration " << j << " in config-dimension " << k; continue; }
          
          Eigen::VectorXd task_diff = (temp_phi - phi)/EPSILON;  //!< Final - initial
          
          Eigen::VectorXd jac_column = jacobian.col(k); //!< Get the desired column (for the current joint)
          
          EXPECT_TRUE(compareVectors(task_diff, jac_column, TOLERANCE)) << " Incorrect for " << registered_types_[i] << " @ iteration " << j << " in config-dimension " << k << task_diff << "\n" << jac_column;
        }
        
        test_handle = tinyxml2::XMLHandle(test_handle.NextSiblingElement("TestPoint"));
        j++;
        
      }
      
    }
    
  }
  
}

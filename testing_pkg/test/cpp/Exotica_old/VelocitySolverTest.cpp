#include "testing_pkg/Exotica/VelocitySolverTest.h"

TEST_F(ExoticaVelSolverTest, RIT) //!< Registration & Initialisation Test
{
  std::string solver_name("VelocitySolverType_1");
  bool vel_solv_ok = false;
  for (int i=0; i<registered_types_.size(); i++)
  {
    if (solver_name.compare(registered_types_[i]) == 0) {vel_solv_ok = true;}
  }
  ASSERT_TRUE(vel_solv_ok) << "Test Velocity Solver not registered";
  
  EXPECT_EQ(nullptr, vel_solv_ptr_ = exotica::VelocitySolverCreator::Instance()->createObject("VelocitySolverType_x", params_)) << "Incorrect registration";
  EXPECT_NE(nullptr, vel_solv_ptr_ = exotica::VelocitySolverCreator::Instance()->createObject("VelocitySolverType_1", params_)) << "Incorrect registration";
  std::string testing;
  EXPECT_TRUE(exotica::TestRegistrar::Instance()->findXML("VelocitySolverType_1", testing)) << " : Tests for Test Velocity Solver not registered";
  EXPECT_EQ(0, solver_name.compare((boost::dynamic_pointer_cast<VelocitySolverType_1>(vel_solv_ptr_))->name));  //!< Name is correct?
  
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(tinyxml2::XML_NO_ERROR, doc.LoadFile((resource_path_ + std::string("VelocitySolver_1.xml")).c_str()));
  tinyxml2::XMLHandle xml_handle(*(doc.RootElement()));
  EXPECT_TRUE(vel_solv_ptr_->initBase(xml_handle));
  EXPECT_TRUE((boost::dynamic_pointer_cast<VelocitySolverType_1>(vel_solv_ptr_))->derived_called);
  EXPECT_EQ(0, (boost::dynamic_pointer_cast<VelocitySolverType_1>(vel_solv_ptr_))->string_element.compare("testing"));
}

TEST_F(ExoticaVelSolverTest, IFT) //!< Inverse-computation Function Test
{
  //!< Temporaries
  Eigen::MatrixXd                        test_matrix;
  tinyxml2::XMLDocument                  xml_document;
  boost::shared_ptr<tinyxml2::XMLHandle> xml_handle_ptr;
  
  //!< Load the Matrix for inversion testing
  ASSERT_EQ(tinyxml2::XML_NO_ERROR, xml_document.LoadFile((resource_path_ + std::string("../storage.xml")).c_str()));
  xml_handle_ptr.reset(new tinyxml2::XMLHandle(xml_document.RootElement()));
  ASSERT_NE(nullptr, xml_handle_ptr->FirstChildElement("InvertibleMatrix").ToElement());
  ASSERT_TRUE(exotica::getMatrix(*(xml_handle_ptr->FirstChildElement("InvertibleMatrix").ToElement()), test_matrix));
  
  //!< Now iterate through all the registered velocity solvers
  for (int i=0; i<registered_types_.size(); i++)  //!< Iterate through the registered tasks
  {
    std::string xml_path;
    if (exotica::TestRegistrar::Instance()->findXML(registered_types_[i], xml_path))  //!< If it is wished to be tested...
    {
      if (xml_document.LoadFile((resource_path_ + xml_path).c_str()) != tinyxml2::XML_NO_ERROR) //!< Attempt to load file
      {
        ADD_FAILURE() << " : Could not Load initialiser for " << registered_types_[i] << " (file: "<<resource_path_ + xml_path <<")."; //!< Have to use this method to add failure since I do not want to continue for this object but do not wish to abort for all the others...
        continue; //!< Go to next object
      }
      if (!(vel_solv_ptr_ = exotica::VelocitySolverCreator::Instance()->createObject(registered_types_[i], params_)))  //!< If we could not create 
      {
        ADD_FAILURE() << " : Could not create object of type " << registered_types_[i]; //!< Have to use this method to add failure since I do not want to continue for this object but do not wish to abort for all the others...
        continue; //!< Go to next object
      }
      xml_handle_ptr.reset(new tinyxml2::XMLHandle(xml_document.RootElement()));  //!< Get handle to root element
      *xml_handle_ptr = xml_handle_ptr->FirstChildElement("VelocitySolver");      //!< Locate the child
      if (!vel_solv_ptr_->initBase(*xml_handle_ptr))
      {
        ADD_FAILURE() << " : Could not initialise " << registered_types_[i];
        continue;
      }
      Eigen::MatrixXd temp_inverse;
      if (!vel_solv_ptr_->getInverse(test_matrix, Eigen::MatrixXd::Identity(30,30)*0.0000001, Eigen::MatrixXd::Identity(30,30), temp_inverse))  //!< Since we know matrix is perfectly invertible
      {
        ADD_FAILURE() << " : Could not compute inverse for " << registered_types_[i];
        continue;
      }
      if (temp_inverse.rows() == 30 and temp_inverse.cols() == 30)
      {
        EXPECT_TRUE(compareMatrices(Eigen::MatrixXd::Identity(30,30), test_matrix*temp_inverse, TOLERANCE_L));
      }
      else
      {
        ADD_FAILURE() << " : Jacobian computation for " << registered_types_[i] << " incorrect";
        continue;
      }
    }
  }
}

/*TEST_F(ExoticaVelSolverTest, SET) //!< Solve Error Test (that the solve function decreases the error) : currently seems unnecessary
{

}*/

#include "testing_pkg/Exotica/PositionSolverTest.h"

TEST_F(ExoticaPosSolverTest, RCT) //Registration and Creation Test
{
  std::string solver_name("PositionSolverType_1");
  exotica::OptimisationParameters_t params_;
  params_.optimisation_window = 1;
  bool pos_solv_ok = false;
  for (int i=0; i<registered_types_.size(); i++)
  {
    if (solver_name.compare(registered_types_[i]) == 0)
    { pos_solv_ok = true;}
  }
  ASSERT_TRUE(pos_solv_ok) << "Test Position Solver not registered";

  EXPECT_EQ(nullptr, pos_solv_ptr_ = exotica::PositionSolverCreator::Instance()->createObject("PositionSolverType_x", params_)) << "Incorrect registration";
  EXPECT_NE(nullptr, pos_solv_ptr_ = exotica::PositionSolverCreator::Instance()->createObject("PositionSolverType_1", params_)) << "Incorrect registration";
  std::string testing;
  EXPECT_EQ(0, solver_name.compare((boost::dynamic_pointer_cast<PositionSolverType_1>(pos_solv_ptr_))->name)); //!< Name is correct?
}

TEST_F(ExoticaPosSolverTest, XIT) //Xml-based Initialisation Test
{
  std::string pos_solver("PositionSolverType_1");
  std::string vel_solver("VelocitySolverType_1");
  std::string opt_name_1("optional");
  std::string opt_name_2("Problem_1");
  tinyxml2::XMLDocument document;
  ASSERT_EQ(tinyxml2::XML_NO_ERROR, document.LoadFile((resource_path_ + std::string("FullSpecification_1.xml")).c_str())); //!< Attempt to open the file for parsing
  tinyxml2::XMLHandle xml_handle(document);
  ASSERT_TRUE(exotica::initialiseSolver(xml_handle, pos_solv_ptr_));

  EXPECT_EQ(1, boost::dynamic_pointer_cast<PositionSolverType_1>(pos_solv_ptr_)->params_ref_.optimisation_window);
  EXPECT_EQ(0, pos_solver.compare((boost::dynamic_pointer_cast<PositionSolverType_1>(pos_solv_ptr_))->name));//!< Name is correct?
  EXPECT_EQ(0, vel_solver.compare(boost::dynamic_pointer_cast<VelocitySolverType_1>((boost::dynamic_pointer_cast<PositionSolverType_1>(pos_solv_ptr_))->vel_solv_ref_)->name));//!< Name is correct?
  EXPECT_EQ(2, pos_solv_ptr_->problem().size());
  EXPECT_EQ(0, opt_name_1.compare(pos_solv_ptr_->problem()[0].getName()));
  EXPECT_EQ(0, opt_name_2.compare(pos_solv_ptr_->problem()[1].getName()));
}

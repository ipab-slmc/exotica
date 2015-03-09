#include <exotica/EXOTica.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>

int main(int argc, char **argv)
{
  //!< Initialise ROS
	testing::InitGoogleTest(&argc, argv);           //!< Testing initialisation
	ros::init(argc, argv, "EXOTica_Test_Suite");    //!< name this node
	
	std::cout << "================================\n Starting ROS_Testing framework \n================================\n" << std::endl;
	
	std::vector<std::string>  registered_taskMaps_;
	std::vector<std::string>  registered_error_func_;
	
	if (exotica::TaskMap_fac::Instance().listImplementations(registered_taskMaps_)){ std::cout << "OOPS! Could not find registry for " << exotica::TaskMap_fac::Instance().type() << std::endl; return -1; };
	
	std::cout << "Registered Task Map Types [" << exotica::TaskMap_fac::Instance().type() << "]: (t indicates testing requested)";
	for (int i=0; i<registered_taskMaps_.size(); i++)
	{
	  std::cout << "\n  " << i+1 << ") " << registered_taskMaps_[i];
	  std::string temp_string;
	  if (exotica::XMLTester::Instance().getTest(registered_taskMaps_[i], temp_string) == exotica::SUCCESS)
	  {
	    std::cout << " (t)";
	  }
	}
	std::cout << "\n---------------------\n" << std::endl;
	
	if (exotica::TaskDefinition_fac::Instance().listImplementations(registered_error_func_)){ std::cout << "OOPS! Could not find registry for " << exotica::TaskDefinition_fac::Instance().type() << std::endl; return -1; };
	
	std::cout << "Registered Task Definition Types [" << exotica::TaskDefinition_fac::Instance().type() << "]: (t indicates testing requested)";
	for (int i=0; i<registered_error_func_.size(); i++)
	{
	  std::cout << "\n  " << i+1 << ") " << registered_error_func_[i];
	  std::string temp_string;
	  if (exotica::XMLTester::Instance().getTest(registered_error_func_[i], temp_string) == exotica::SUCCESS)
	  {
	    std::cout << " (t)";
	  }
	}
	std::cout << "\n---------------------\n" << std::endl;
	
	//!< Run Tests
  return RUN_ALL_TESTS();
}

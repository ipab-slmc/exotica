#include <ros/ros.h>
#include <gtest/gtest.h>

int main(int argc, char **argv)
{
  //!< Initialise ROS
	testing::InitGoogleTest(&argc, argv);           //!< Testing initialisation
	ros::init(argc, argv, "Kinematica_Test_Suite"); //!< name this node
	
	std::cout << "Starting ROS_Testing framework" << std::endl;
	
	//!< Run Tests
  return RUN_ALL_TESTS();
}

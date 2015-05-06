#ifndef TESTING_KINEMATICA_YOUBOT_TEST_H
#define TESTING_KINEMATICA_YOUBOT_TEST_H

#include <kinematica/KinematicTree.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/Eigen>
#include <cstdlib>
#include <ctime>
#include <gtest/gtest.h>
#include <boost/bind.hpp>
#include <cmath>        
#include "testing_pkg/TestingTools.h"

#define NJOINTS     8

#define ROOT_FRAME  "base_footprint"                 //!< The default root frame
#define ROOT_ALTER  "arm_link_1"                 //!< A different root from ROOT_FRAME

#define SEGMENT_1   "base_footprint"                 //!< 4 random segments
#define SEGMENT_2   "base_link"
#define SEGMENT_3   "arm_link_0"
#define SEGMENT_4   "arm_link_5"

#define ROBOT_JNTS "dummy_prismatic_x_joint", "dummy_prismatic_y_joint", "dummy_revolute_joint", "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"
#define ROBOT_CONFIG_1 0.5, 0.9, 1.2, 3.0, 1.2, -4.3, 2.9, 0.4

#define ROBOT_URDF   "/resource/Kinematica/youbot.urdf"   //!< Default

#define EPSILON     0.00001     //!< Change in joint angles 

typedef std::map<std::string, KDL::ChainFkSolverPos_recursive*> solver_map_t;
class KinematicaYoubotTest : public ::testing::Test
{
  public:
    
    virtual void SetUp()
    {            
      //!< Initialise the Solution Structure
      robot_solution_.root_segment       = ROOT_FRAME;
      robot_solution_.root_seg_off       = KDL::Frame::Identity();
      robot_solution_.joints_update      = {ROBOT_JNTS};
      robot_solution_.zero_other_joints  = true;
      robot_solution_.ignore_unused_segs = false;
      robot_solution_.end_effector_segs   = {SEGMENT_1, SEGMENT_2, SEGMENT_3, SEGMENT_4};
      robot_solution_.end_effector_offs   = {KDL::Frame::Identity(), KDL::Frame::Identity(), KDL::Frame::Identity(), KDL::Frame::Identity()};
      
      //!< The location of the urdf file
      std::string package_path;
      ASSERT_TRUE(findPackagePath("testing_pkg", package_path));  //!< Removes dependency on ros
      robot_urdf_ = package_path.append(ROBOT_URDF);
      
      //!< KDL System for cross-checking - This part will need to be modified if the robot model changes
      KDL::Tree   kdl_tree;
      ASSERT_TRUE (kdl_parser::treeFromFile(robot_urdf_, kdl_tree)) << "Something wrong with ATLAS urdf specification: not Kinematica Problem";
      KDL::Chain  temp_chain;
      ASSERT_TRUE (kdl_tree.getChain(ROOT_FRAME, SEGMENT_1, temp_chain)) << "Non-Kinematica Error: could not generate chain for " << SEGMENT_1;
      EXPECT_EQ(0, temp_chain.getNrOfJoints());
      kdl_solver[SEGMENT_1] = new KDL::ChainFkSolverPos_recursive(temp_chain);
      ASSERT_TRUE (kdl_tree.getChain(ROOT_FRAME, SEGMENT_2, temp_chain)) << "Non-Kinematica Error: could not generate chain for " << SEGMENT_2;
      EXPECT_EQ(3, temp_chain.getNrOfJoints());
      kdl_solver[SEGMENT_2] = new KDL::ChainFkSolverPos_recursive(temp_chain);
      ASSERT_TRUE (kdl_tree.getChain(ROOT_FRAME, SEGMENT_3, temp_chain)) << "Non-Kinematica Error: could not generate chain for " << SEGMENT_3;
      EXPECT_EQ(3, temp_chain.getNrOfJoints());
      kdl_solver[SEGMENT_3] = new KDL::ChainFkSolverPos_recursive(temp_chain);
      ASSERT_TRUE (kdl_tree.getChain(ROOT_FRAME, SEGMENT_4, temp_chain)) << "Non-Kinematica Error: could not generate chain for " << SEGMENT_4;
      EXPECT_EQ(8, temp_chain.getNrOfJoints());
      kdl_solver[SEGMENT_4] = new KDL::ChainFkSolverPos_recursive(temp_chain);
      
      //!< Joints...
      joint_config_1_.resize(NJOINTS);
      joint_config_1_ << ROBOT_CONFIG_1;
      
      joint_mapping_seg_1_.resize(0); //!< No joints here
      
      joint_mapping_seg_2_.resize(3);
      joint_mapping_seg_2_ << 0, 1, 2;
      
      joint_mapping_seg_3_.resize(3);
      joint_mapping_seg_3_ << 0, 1, 2;
      
      joint_mapping_seg_4_.resize(8);
      joint_mapping_seg_4_ << 0, 1, 2, 3, 4, 5, 6, 7;
      
      setUpVector(joint_mapping_seg_1_, joint_config_1_, joint_config_1_1_);
      setUpVector(joint_mapping_seg_2_, joint_config_1_, joint_config_1_2_);
      setUpVector(joint_mapping_seg_3_, joint_config_1_, joint_config_1_3_);
      setUpVector(joint_mapping_seg_4_, joint_config_1_, joint_config_1_4_);
      
      joint_config_0_8_.resize(8);
      KDL::SetToZero(joint_config_0_8_);
      
      joint_config_0_3_.resize(3);
      KDL::SetToZero(joint_config_0_3_);
      
      joint_config_0_0_.resize(0);  //!< Empty KDL Jnt Array
    }
    
    virtual void TearDown()
    {
      //!< Clean up
      if (kdl_solver.size() > 0)
      {
        for (solver_map_t::iterator it=kdl_solver.begin(); it != kdl_solver.end(); it++)
        {
          delete(it->second);
        }
      }
    }
    
    void setUpVector(Eigen::VectorXdRefConst mapping, Eigen::VectorXdRefConst full_config, KDL::JntArray & sub_config)
    {
      sub_config.resize(mapping.size());
      for (int i=0; i<mapping.size(); i++)
      {
        sub_config(i) = full_config(mapping(i));
      }
    }
    
    kinematica::SolutionForm_t  robot_solution_;
    
    kinematica::KinematicTree   robot_tree_;
    std::string                 robot_urdf_;
    
    Eigen::VectorXd             joint_config_1_;
    
    Eigen::VectorXd             joint_mapping_seg_1_;
    Eigen::VectorXd             joint_mapping_seg_2_;
    Eigen::VectorXd             joint_mapping_seg_3_;
    Eigen::VectorXd             joint_mapping_seg_4_;
    KDL::JntArray               joint_config_1_1_;  //!< Joint Config 1 for segment 1
    KDL::JntArray               joint_config_1_2_;
    KDL::JntArray               joint_config_1_3_;
    KDL::JntArray               joint_config_1_4_;
    
    KDL::JntArray               joint_config_0_8_;  //!< Joint config array of size 9, all 0s
    
    KDL::JntArray               joint_config_0_3_;  //!< Joint config array of size 6, all 0s
    KDL::JntArray               joint_config_0_0_;  //!< Empty Joint Config Array
    solver_map_t                kdl_solver;
    
};

#endif

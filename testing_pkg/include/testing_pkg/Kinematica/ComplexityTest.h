#ifndef TESTING_KINEMATICA_ATLAS_TEST_H
#define TESTING_KINEMATICA_ATLAS_TEST_H

#include <exotica/KinematicTree.h>
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

#define NJOINTS     28

#define ROOT_FRAME  "pelvis"                 //!< The default root frame
#define ROOT_ALTER  "utorso"                 //!< A different root from ROOT_FRAME

#define SEGMENT_1   "l_hand"                 //!< 4 random segments, tips, on different chains w.r.t. the original ROOT_FRAME
#define SEGMENT_2   "r_hand"
#define SEGMENT_3   "l_foot"
#define SEGMENT_4   "r_foot"

#define ROBOT_JNTS "back_bkz", "back_bky", "back_bkx", "neck_ry", "l_leg_hpz", "l_leg_hpx", "l_leg_hpy", "l_leg_kny", "l_leg_aky", "l_leg_akx", "r_leg_hpz", "r_leg_hpx", "r_leg_hpy", "r_leg_kny", "r_leg_aky", "r_leg_akx", "l_arm_shy", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_wry", "l_arm_wrx", "r_arm_shy", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_wry", "r_arm_wrx" //!< Ordering from https://bitbucket.org/osrf/drcsim/src/default/drcsim_gazebo_ros_plugins/src/AtlasPlugin.cpp
#define ROBOT_CONFIG_1 -0.4, 0.3, 0.0, 1.0, 0.2, -0.3, -1.2, 2.1, -0.8, 0.25, -0.6, 0.3, -1.6, 1.1, -0.3, 0.0, 0.7, 0.4, 2.8, 0.1, 1.5, -0.9, 0.6, -1.5, 3.0, -1.0, 2.1, 0.0

#define ROBOT_URDF   "/resource/Kinematica/atlas.urdf"   //!< Default

#define EPSILON     0.00001     //!< Change in joint angles 

typedef std::map<std::string, KDL::ChainFkSolverPos_recursive*> solver_map_t;
class KinematicaAtlasTest: public ::testing::Test
{
  public:

    virtual void SetUp()
    {
      //!< Initialise the Solution Structure
      robot_solution_.root_segment = ROOT_FRAME;
      robot_solution_.root_seg_off = KDL::Frame::Identity();
      robot_solution_.joints_update =
      { ROBOT_JNTS};
      robot_solution_.zero_other_joints = true;
      robot_solution_.ignore_unused_segs = false;
      robot_solution_.end_effector_segs =
      { SEGMENT_1, SEGMENT_2, SEGMENT_3, SEGMENT_4};
      robot_solution_.end_effector_offs =
      { KDL::Frame::Identity(), KDL::Frame::Identity(), KDL::Frame::Identity(), KDL::Frame::Identity()};

      //!< The location of the urdf file
      std::string package_path;
      ASSERT_TRUE(findPackagePath("testing_pkg", package_path)); //!< Removes dependency on ros
      robot_urdf_ = package_path.append(ROBOT_URDF);

      //!< KDL System for cross-checking - This part will need to be modified if the robot model changes
      KDL::Tree kdl_tree;
      ASSERT_TRUE (kdl_parser::treeFromFile(robot_urdf_, kdl_tree))<< "Something wrong with ATLAS urdf specification: not Kinematica Problem";
      KDL::Chain temp_chain;
      ASSERT_TRUE (kdl_tree.getChain(ROOT_FRAME, SEGMENT_1, temp_chain))<< "Non-Kinematica Error: could not generate chain for " << SEGMENT_1;
      EXPECT_EQ(9, temp_chain.getNrOfJoints());
      kdl_solver[SEGMENT_1] = new KDL::ChainFkSolverPos_recursive(temp_chain);
      ASSERT_TRUE (kdl_tree.getChain(ROOT_FRAME, SEGMENT_2, temp_chain))<< "Non-Kinematica Error: could not generate chain for " << SEGMENT_2;
      EXPECT_EQ(9, temp_chain.getNrOfJoints());
      kdl_solver[SEGMENT_2] = new KDL::ChainFkSolverPos_recursive(temp_chain);
      ASSERT_TRUE (kdl_tree.getChain(ROOT_FRAME, SEGMENT_3, temp_chain))<< "Non-Kinematica Error: could not generate chain for " << SEGMENT_3;
      EXPECT_EQ(6, temp_chain.getNrOfJoints());
      kdl_solver[SEGMENT_3] = new KDL::ChainFkSolverPos_recursive(temp_chain);
      ASSERT_TRUE (kdl_tree.getChain(ROOT_FRAME, SEGMENT_4, temp_chain))<< "Non-Kinematica Error: could not generate chain for " << SEGMENT_4;
      EXPECT_EQ(6, temp_chain.getNrOfJoints());
      kdl_solver[SEGMENT_4] = new KDL::ChainFkSolverPos_recursive(temp_chain);

      //!< Joints...
      joint_config_1_.resize(NJOINTS);
      joint_config_1_ << ROBOT_CONFIG_1;

      joint_mapping_seg_1_.resize(9);
      joint_mapping_seg_1_ << 0,
      1, 2, 16, 17, 18, 19, 20, 21;

      joint_mapping_seg_2_.resize(9);
      joint_mapping_seg_2_ << 0, 1, 2, 22, 23, 24, 25, 26, 27;

      joint_mapping_seg_3_.resize(6);
      joint_mapping_seg_3_ << 4, 5, 6, 7, 8, 9;

      joint_mapping_seg_4_.resize(6);
      joint_mapping_seg_4_ << 10, 11, 12, 13, 14, 15;

      setUpVector(joint_mapping_seg_1_, joint_config_1_, joint_config_1_1_);
      setUpVector(joint_mapping_seg_2_, joint_config_1_, joint_config_1_2_);
      setUpVector(joint_mapping_seg_3_, joint_config_1_, joint_config_1_3_);
      setUpVector(joint_mapping_seg_4_, joint_config_1_, joint_config_1_4_);

      joint_config_0_9_.resize(9);
      KDL::SetToZero(joint_config_0_9_);

      joint_config_0_6_.resize(6);
      KDL::SetToZero(joint_config_0_6_);
    }

    virtual void TearDown()
    {
      //!< Clean up
      if (kdl_solver.size() > 0)
      {
        for (solver_map_t::iterator it = kdl_solver.begin();
            it != kdl_solver.end(); it++)
        {
          delete (it->second);
        }
      }
    }

    void setUpVector(Eigen::VectorXdRefConst mapping,
        Eigen::VectorXdRefConst full_config, KDL::JntArray & sub_config)
    {
      sub_config.resize(mapping.size());
      for (int i = 0; i < mapping.size(); i++)
      {
        sub_config(i) = full_config(mapping(i));
      }
    }

    exotica::SolutionForm_t robot_solution_;

    exotica::KinematicTree robot_tree_;
    std::string robot_urdf_;

    Eigen::VectorXd joint_config_1_;

    Eigen::VectorXd joint_mapping_seg_1_;
    Eigen::VectorXd joint_mapping_seg_2_;
    Eigen::VectorXd joint_mapping_seg_3_;
    Eigen::VectorXd joint_mapping_seg_4_;
    KDL::JntArray joint_config_1_1_;  //!< Joint Config 1 for segment 1
    KDL::JntArray joint_config_1_2_;
    KDL::JntArray joint_config_1_3_;
    KDL::JntArray joint_config_1_4_;

    KDL::JntArray joint_config_0_9_;  //!< Joint config array of size 9, all 0s

    KDL::JntArray joint_config_0_6_;  //!< Joint config array of size 6, all 0s
    solver_map_t kdl_solver;

};

#endif

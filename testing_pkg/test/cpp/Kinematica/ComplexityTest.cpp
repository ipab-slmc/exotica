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

#include "testing_pkg/Kinematica/ComplexityTest.h"

TEST_F(KinematicaAtlasTest, DefaultInitialisation)
{
  std::map<std::string, int> segmap;
  KDL::Frame root_pose;
  ASSERT_TRUE(robot_tree_.initKinematics(robot_urdf_, robot_solution_))<< "Could Not Initialise Robot";
  EXPECT_TRUE(robot_tree_.getSegmentMap(segmap)) << "Could Not get Segment Map";
  ASSERT_FALSE(segmap.find(ROOT_FRAME) == segmap.end())<< "Root frame not found";
  EXPECT_EQ(0, segmap[ROOT_FRAME]) << "Root frame not correct";
  ASSERT_TRUE(robot_tree_.getPose(ROOT_FRAME, root_pose))<< "Not initialised correctly";
  EXPECT_EQ(KDL::Frame::Identity(), root_pose);
}

TEST_F(KinematicaAtlasTest, InvalidUrdf)
{
  ASSERT_FALSE(robot_tree_.initKinematics("blabla", robot_solution_))<< "Does not recognise Invalid URDF";
}

TEST_F(KinematicaAtlasTest, WrongRoot)
{
  robot_solution_.root_segment = "blabla";
  ASSERT_FALSE(robot_tree_.initKinematics(robot_urdf_, robot_solution_))<< "Does not recognise Invalid Root";
}

TEST_F(KinematicaAtlasTest, InvalidJoints)
{
  robot_solution_.zero_other_joints = false;
  robot_solution_.joints_update.pop_back(); //!< Reduce the size to invalidate number
  EXPECT_FALSE(robot_tree_.initKinematics(robot_urdf_, robot_solution_))
      << "Ignores incorrect sizes of joints even if zero_other_joints is false";
  robot_solution_.zero_other_joints = true;
  EXPECT_TRUE(robot_tree_.initKinematics(robot_urdf_, robot_solution_))
      << "Does not initialise without correct number of joints even with zeroing flag being true";
  robot_solution_.joints_update =
  { "blabla"};
  EXPECT_FALSE(robot_tree_.initKinematics(robot_urdf_, robot_solution_))
      << "Does not recognise invalid joints";
}

TEST_F(KinematicaAtlasTest, UpdatingKinematics)
{
  //!< Temporaries
  KDL::Frame global_pose;
  KDL::Frame relative_pose;
  KDL::Frame incorrect_pose;

  //!< Test for catching unitialised queries
  EXPECT_FALSE(robot_tree_.updateConfiguration(Eigen::VectorXd::Zero(NJOINTS)))
      << "Does not recognise unitialisation";

  //!< Now initialise robot
  robot_solution_.root_seg_off.p = KDL::Vector(0.5, 0.4, 0.2); //!< Give random pose
  ASSERT_TRUE(robot_tree_.initKinematics(robot_urdf_, robot_solution_))<< "Cannot proceed with Test since could Not Initialise Robot";

  //!< Attempt update of kinematics
  EXPECT_FALSE(robot_tree_.updateConfiguration(Eigen::VectorXd::Zero(NJOINTS-1)))
      << "Does not fail when Incorrect number of joints specified";

  ASSERT_TRUE(robot_tree_.updateConfiguration(joint_config_1_))<< "Could not update kinematics";
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_1, global_pose));
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_1, ROOT_FRAME, relative_pose));
  EXPECT_EQ(KDL::Vector(0.5, 0.4, 0.2), relative_pose.p - global_pose.p)
      << "Did Not transform FK : " << global_pose.p.x() << "/"
      << relative_pose.p.x() << " " << global_pose.p.y() << "/"
      << relative_pose.p.y() << " " << global_pose.p.z() << "/"
      << relative_pose.p.z();

  robot_solution_.root_segment =
  { SEGMENT_2};  //!< Use another base
  robot_solution_.ignore_unused_segs = true;   //!< so now will not update
  ASSERT_TRUE(robot_tree_.initKinematics(robot_urdf_, robot_solution_))<< "Cannot proceed with Test since could Not Initialise Robot";
  ASSERT_TRUE(robot_tree_.updateConfiguration(joint_config_1_))<< "Could not update kinematics";
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_1, incorrect_pose));
  EXPECT_NE(global_pose, incorrect_pose)
      << "ignore_unused_segs flag is having no effect";
}

TEST_F(KinematicaAtlasTest, CheckingKinematicsInternally)
{
  //!< Temporaries & Initialisation
  KDL::Frame pose_1, pose_2, pose_3, pose_4;
  double x_1, y_1, z_1, w_1, x_2, y_2, z_2, w_2, x_3, y_3, z_3, w_3, x_4, y_4,
      z_4, w_4;
  ASSERT_TRUE(robot_tree_.initKinematics(robot_urdf_, robot_solution_))<< "Cannot proceed with Test since could Not Initialise Robot";
  ASSERT_TRUE(robot_tree_.updateConfiguration(joint_config_1_))<< "Could not update kinematics";

  //!< Get Poses and check that inversion is obeyed
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_1, SEGMENT_2, pose_1));
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_2, SEGMENT_1, pose_2));
  pose_3 = pose_1.Inverse();  //!< Get the inverse
  pose_4 = pose_2;
  pose_3.M.GetQuaternion(x_3, y_3, z_3, w_3);
  pose_4.M.GetQuaternion(x_4, y_4, z_4, w_4);
  EXPECT_TRUE(KDL::Equal(pose_3, pose_4, 0.001)) << "Inversion not correct: x("
      << std::setprecision(4) << pose_3.p.x() << "/" << pose_4.p.x() << ") : y("
      << pose_3.p.y() << "/" << pose_4.p.y() << ") : z(" << pose_3.p.z() << "/"
      << pose_4.p.z() << ") : q_x(" << x_3 << "/" << x_4 << ") : q_y(" << y_3
      << "/" << y_4 << ") : q_z(" << z_3 << "/" << z_4 << ") : q_w(" << w_3
      << "/" << w_4 << ")";

  //!< Get Poses and check that the triangle law is obeyed
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_1, ROOT_FRAME, pose_1));
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_2, ROOT_FRAME, pose_2));
  pose_3 = pose_1.Inverse() * pose_2;
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_2, SEGMENT_1, pose_4));
  pose_3.M.GetQuaternion(x_3, y_3, z_3, w_3);
  pose_4.M.GetQuaternion(x_4, y_4, z_4, w_4);
  EXPECT_TRUE(KDL::Equal(pose_3, pose_4, 0.0001))
      << "Triangulation not correct: x(" << std::setprecision(4) << pose_3.p.x()
      << "/" << pose_4.p.x() << ") : y(" << pose_3.p.y() << "/" << pose_4.p.y()
      << ") : z(" << pose_3.p.z() << "/" << pose_4.p.z() << ") : q_x(" << x_3
      << "/" << x_4 << ") : q_y(" << y_3 << "/" << y_4 << ") : q_z(" << z_3
      << "/" << z_4 << ") : q_w(" << w_3 << "/" << w_4 << ")";

  //!< Now repeat with another root
  robot_solution_.root_segment =
  { ROOT_ALTER};
  ASSERT_TRUE(robot_tree_.initKinematics(robot_urdf_, robot_solution_))<< "Cannot proceed with Test since could Not Initialise Robot with new root";
  ASSERT_TRUE(robot_tree_.updateConfiguration(joint_config_1_))<< "Could not update kinematics";

  //!< Get Poses and check that inversion is obeyed
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_1, SEGMENT_2, pose_1));
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_2, SEGMENT_1, pose_2));
  pose_3 = pose_1.Inverse();  //!< Get the inverse
  pose_4 = pose_2;
  pose_3.M.GetQuaternion(x_3, y_3, z_3, w_3);
  pose_4.M.GetQuaternion(x_4, y_4, z_4, w_4);
  EXPECT_TRUE(KDL::Equal(pose_3, pose_4, 0.001))
      << "Inversion not correct with changed root: x(" << std::setprecision(4)
      << pose_3.p.x() << "/" << pose_4.p.x() << ") : y(" << pose_3.p.y() << "/"
      << pose_4.p.y() << ") : z(" << pose_3.p.z() << "/" << pose_4.p.z()
      << ") : q_x(" << x_3 << "/" << x_4 << ") : q_y(" << y_3 << "/" << y_4
      << ") : q_z(" << z_3 << "/" << z_4 << ") : q_w(" << w_3 << "/" << w_4
      << ")";

  //!< Get Poses and check that the triangle law is obeyed
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_1, ROOT_FRAME, pose_1));
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_2, ROOT_FRAME, pose_2));
  pose_3 = pose_1.Inverse() * pose_2;
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_2, SEGMENT_1, pose_4));
  pose_3.M.GetQuaternion(x_3, y_3, z_3, w_3);
  pose_4.M.GetQuaternion(x_4, y_4, z_4, w_4);
  EXPECT_TRUE(KDL::Equal(pose_3, pose_4, 0.0001))
      << "Triangulation not correct with changed root: x("
      << std::setprecision(4) << pose_3.p.x() << "/" << pose_4.p.x() << ") : y("
      << pose_3.p.y() << "/" << pose_4.p.y() << ") : z(" << pose_3.p.z() << "/"
      << pose_4.p.z() << ") : q_x(" << x_3 << "/" << x_4 << ") : q_y(" << y_3
      << "/" << y_4 << ") : q_z(" << z_3 << "/" << z_4 << ") : q_w(" << w_3
      << "/" << w_4 << ")";
}

TEST_F(KinematicaAtlasTest, CheckingKinematicsAgainstKDLNormalRoot)
{
  KDL::Frame kdl_pose, kinematica_pose;
  double kdl_x, kdl_y, kdl_z, kdl_w, kin_x, kin_y, kin_z, kin_w;

  ASSERT_TRUE(robot_tree_.initKinematics(robot_urdf_, robot_solution_))<< "Cannot proceed with Test since could Not Initialise Robot";

  //!< First test with zeroed out joint angles
  ASSERT_TRUE(robot_tree_.updateConfiguration(Eigen::VectorXd::Zero(NJOINTS)))<< "Could not update kinematics";

  //!< Segment 1
  ASSERT_GE(0, kdl_solver[SEGMENT_1]->JntToCart(joint_config_0_9_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_1, kinematica_pose));
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  //std::cout << kdl_pose.p.x() << " " << kdl_pose.p.y() << " " << kdl_pose.p.z() << std::endl;
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_1 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Segment 2
  ASSERT_GE(0, kdl_solver[SEGMENT_2]->JntToCart(joint_config_0_9_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_2, kinematica_pose));
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_2 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Segment 3
  ASSERT_GE(0, kdl_solver[SEGMENT_3]->JntToCart(joint_config_0_6_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_3, kinematica_pose));
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_3 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Segment 4
  ASSERT_GE(0, kdl_solver[SEGMENT_4]->JntToCart(joint_config_0_6_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_4, kinematica_pose));
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_4 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Now test with random joint configuration
  ASSERT_TRUE(robot_tree_.updateConfiguration(joint_config_1_))<< "Could not update kinematics";

  //!< Segment 1
  ASSERT_GE(0, kdl_solver[SEGMENT_1]->JntToCart(joint_config_1_1_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_1, kinematica_pose));
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  //std::cout << kdl_pose.p.x() << " " << kdl_pose.p.y() << " " << kdl_pose.p.z() << std::endl; getchar();
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_1 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Segment 2
  ASSERT_GE(0, kdl_solver[SEGMENT_2]->JntToCart(joint_config_1_2_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_2, kinematica_pose));
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_2 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Segment 3
  ASSERT_GE(0, kdl_solver[SEGMENT_3]->JntToCart(joint_config_1_3_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_3, kinematica_pose));
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_3 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Segment 4
  ASSERT_GE(0, kdl_solver[SEGMENT_4]->JntToCart(joint_config_1_4_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_4, kinematica_pose));
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_4 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";
}

TEST_F(KinematicaAtlasTest, CheckingKinematicsAgainstKDLAlternateRoot)
{
  KDL::Frame kdl_pose, kdl_pose_2, kinematica_pose;
  double kdl_x, kdl_y, kdl_z, kdl_w, kin_x, kin_y, kin_z, kin_w;

  robot_solution_.root_segment = ROOT_ALTER;
  ASSERT_TRUE(robot_tree_.initKinematics(robot_urdf_, robot_solution_))<< "Cannot proceed with Test since could Not Initialise Robot";

  //!< First test with zeroed out joint angles
  ASSERT_TRUE(robot_tree_.updateConfiguration(Eigen::VectorXd::Zero(NJOINTS)))<< "Could not update kinematics";

  //!< Segment 1
  ASSERT_GE(0, kdl_solver[SEGMENT_1]->JntToCart(joint_config_0_9_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_GE(0, kdl_solver[SEGMENT_1]->JntToCart(joint_config_0_9_, kdl_pose_2, 0))<< "Non-Kinematica Error: Could not compute FK from KDL";
  kdl_pose = kdl_pose_2.Inverse() * kdl_pose; //!< Get the actual position in terms of the tip of the root, which is what Kinematica is able to get us...
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_1, ROOT_FRAME, kinematica_pose)); //!< Specify that we want it w.r.t. the original root frame
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_1 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Segment 2
  ASSERT_GE(0, kdl_solver[SEGMENT_2]->JntToCart(joint_config_0_9_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_GE(0, kdl_solver[SEGMENT_2]->JntToCart(joint_config_0_9_, kdl_pose_2, 0))<< "Non-Kinematica Error: Could not compute FK from KDL";
  kdl_pose = kdl_pose_2.Inverse() * kdl_pose; //!< Get the actual position in terms of the tip of the root, which is what Kinematica is able to get us...
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_2, ROOT_FRAME, kinematica_pose)); //!< Specify that we want it w.r.t. the original root frame
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_2 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Segment 3
  ASSERT_GE(0, kdl_solver[SEGMENT_3]->JntToCart(joint_config_0_6_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_GE(0, kdl_solver[SEGMENT_3]->JntToCart(joint_config_0_6_, kdl_pose_2, 0))<< "Non-Kinematica Error: Could not compute FK from KDL";
  kdl_pose = kdl_pose_2.Inverse() * kdl_pose; //!< Get the actual position in terms of the tip of the root, which is what Kinematica is able to get us...
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_3, ROOT_FRAME, kinematica_pose)); //!< Specify that we want it w.r.t. the original root frame
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_3 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Segment 4
  ASSERT_GE(0, kdl_solver[SEGMENT_4]->JntToCart(joint_config_0_6_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_GE(0, kdl_solver[SEGMENT_4]->JntToCart(joint_config_0_6_, kdl_pose_2, 0))<< "Non-Kinematica Error: Could not compute FK from KDL";
  kdl_pose = kdl_pose_2.Inverse() * kdl_pose; //!< Get the actual position in terms of the tip of the root, which is what Kinematica is able to get us...
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_4, ROOT_FRAME, kinematica_pose)); //!< Specify that we want it w.r.t. the original root frame
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_4 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Now test with random joint configuration
  ASSERT_TRUE(robot_tree_.updateConfiguration(joint_config_1_))<< "Could not update kinematics";

  //!< Segment 1
  ASSERT_GE(0, kdl_solver[SEGMENT_1]->JntToCart(joint_config_1_1_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_GE(0, kdl_solver[SEGMENT_1]->JntToCart(joint_config_1_1_, kdl_pose_2, 0))<< "Non-Kinematica Error: Could not compute FK from KDL";
  kdl_pose = kdl_pose_2.Inverse() * kdl_pose; //!< Get the actual position in terms of the tip of the root, which is what Kinematica is able to get us...
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_1, ROOT_FRAME, kinematica_pose)); //!< Specify that we want it w.r.t. the original root frame
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_1 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Segment 2
  ASSERT_GE(0, kdl_solver[SEGMENT_2]->JntToCart(joint_config_1_2_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_GE(0, kdl_solver[SEGMENT_2]->JntToCart(joint_config_1_2_, kdl_pose_2, 0))<< "Non-Kinematica Error: Could not compute FK from KDL";
  kdl_pose = kdl_pose_2.Inverse() * kdl_pose; //!< Get the actual position in terms of the tip of the root, which is what Kinematica is able to get us...
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_2, ROOT_FRAME, kinematica_pose)); //!< Specify that we want it w.r.t. the original root frame
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_2 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Segment 3
  ASSERT_GE(0, kdl_solver[SEGMENT_3]->JntToCart(joint_config_1_3_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_GE(0, kdl_solver[SEGMENT_3]->JntToCart(joint_config_1_3_, kdl_pose_2, 0))<< "Non-Kinematica Error: Could not compute FK from KDL";
  kdl_pose = kdl_pose_2.Inverse() * kdl_pose; //!< Get the actual position in terms of the tip of the root, which is what Kinematica is able to get us...
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_3, ROOT_FRAME, kinematica_pose)); //!< Specify that we want it w.r.t. the original root frame
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_3 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Segment 4
  ASSERT_GE(0, kdl_solver[SEGMENT_4]->JntToCart(joint_config_1_4_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_GE(0, kdl_solver[SEGMENT_4]->JntToCart(joint_config_1_4_, kdl_pose_2, 0))<< "Non-Kinematica Error: Could not compute FK from KDL";
  kdl_pose = kdl_pose_2.Inverse() * kdl_pose; //!< Get the actual position in terms of the tip of the root, which is what Kinematica is able to get us...
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_4, ROOT_FRAME, kinematica_pose)); //!< Specify that we want it w.r.t. the original root frame
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_4 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";
}

TEST_F(KinematicaAtlasTest, CheckingKinematicsAgainstKDLAlternateRootWithOffset)
{
  KDL::Frame kdl_pose, kdl_pose_2, kinematica_pose;
  double kdl_x, kdl_y, kdl_z, kdl_w, kin_x, kin_y, kin_z, kin_w;

  robot_solution_.root_segment = ROOT_ALTER;
  robot_solution_.root_seg_off.p = KDL::Vector(0.5, -0.4, 0.3);
  robot_solution_.root_seg_off.M.DoRotX(0.3);
  robot_solution_.root_seg_off.M.DoRotY(-0.2);
  robot_solution_.root_seg_off.M.DoRotZ(0.1);
  ASSERT_TRUE(robot_tree_.initKinematics(robot_urdf_, robot_solution_))<< "Cannot proceed with Test since could Not Initialise Robot";

  //!< First test with zeroed out joint angles
  ASSERT_TRUE(robot_tree_.updateConfiguration(Eigen::VectorXd::Zero(NJOINTS)))<< "Could not update kinematics";

  //!< Segment 1
  ASSERT_GE(0, kdl_solver[SEGMENT_1]->JntToCart(joint_config_0_9_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_GE(0, kdl_solver[SEGMENT_1]->JntToCart(joint_config_0_9_, kdl_pose_2, 0))<< "Non-Kinematica Error: Could not compute FK from KDL";
  kdl_pose = kdl_pose_2.Inverse() * kdl_pose; //!< Get the actual position in terms of the tip of the root, which is what Kinematica is able to get us...
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_1, ROOT_FRAME, kinematica_pose)); //!< Specify that we want it w.r.t. the original root frame
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_1 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Segment 2
  ASSERT_GE(0, kdl_solver[SEGMENT_2]->JntToCart(joint_config_0_9_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_GE(0, kdl_solver[SEGMENT_2]->JntToCart(joint_config_0_9_, kdl_pose_2, 0))<< "Non-Kinematica Error: Could not compute FK from KDL";
  kdl_pose = kdl_pose_2.Inverse() * kdl_pose; //!< Get the actual position in terms of the tip of the root, which is what Kinematica is able to get us...
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_2, ROOT_FRAME, kinematica_pose)); //!< Specify that we want it w.r.t. the original root frame
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_2 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Segment 3
  ASSERT_GE(0, kdl_solver[SEGMENT_3]->JntToCart(joint_config_0_6_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_GE(0, kdl_solver[SEGMENT_3]->JntToCart(joint_config_0_6_, kdl_pose_2, 0))<< "Non-Kinematica Error: Could not compute FK from KDL";
  kdl_pose = kdl_pose_2.Inverse() * kdl_pose; //!< Get the actual position in terms of the tip of the root, which is what Kinematica is able to get us...
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_3, ROOT_FRAME, kinematica_pose)); //!< Specify that we want it w.r.t. the original root frame
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_3 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Segment 4
  ASSERT_GE(0, kdl_solver[SEGMENT_4]->JntToCart(joint_config_0_6_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_GE(0, kdl_solver[SEGMENT_4]->JntToCart(joint_config_0_6_, kdl_pose_2, 0))<< "Non-Kinematica Error: Could not compute FK from KDL";
  kdl_pose = kdl_pose_2.Inverse() * kdl_pose; //!< Get the actual position in terms of the tip of the root, which is what Kinematica is able to get us...
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_4, ROOT_FRAME, kinematica_pose)); //!< Specify that we want it w.r.t. the original root frame
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_4 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Now test with random joint configuration
  ASSERT_TRUE(robot_tree_.updateConfiguration(joint_config_1_))<< "Could not update kinematics";

  //!< Segment 1
  ASSERT_GE(0, kdl_solver[SEGMENT_1]->JntToCart(joint_config_1_1_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_GE(0, kdl_solver[SEGMENT_1]->JntToCart(joint_config_1_1_, kdl_pose_2, 0))<< "Non-Kinematica Error: Could not compute FK from KDL";
  kdl_pose = kdl_pose_2.Inverse() * kdl_pose; //!< Get the actual position in terms of the tip of the root, which is what Kinematica is able to get us...
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_1, ROOT_FRAME, kinematica_pose)); //!< Specify that we want it w.r.t. the original root frame
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_1 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Segment 2
  ASSERT_GE(0, kdl_solver[SEGMENT_2]->JntToCart(joint_config_1_2_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_GE(0, kdl_solver[SEGMENT_2]->JntToCart(joint_config_1_2_, kdl_pose_2, 0))<< "Non-Kinematica Error: Could not compute FK from KDL";
  kdl_pose = kdl_pose_2.Inverse() * kdl_pose; //!< Get the actual position in terms of the tip of the root, which is what Kinematica is able to get us...
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_2, ROOT_FRAME, kinematica_pose)); //!< Specify that we want it w.r.t. the original root frame
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_2 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Segment 3
  ASSERT_GE(0, kdl_solver[SEGMENT_3]->JntToCart(joint_config_1_3_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_GE(0, kdl_solver[SEGMENT_3]->JntToCart(joint_config_1_3_, kdl_pose_2, 0))<< "Non-Kinematica Error: Could not compute FK from KDL";
  kdl_pose = kdl_pose_2.Inverse() * kdl_pose; //!< Get the actual position in terms of the tip of the root, which is what Kinematica is able to get us...
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_3, ROOT_FRAME, kinematica_pose)); //!< Specify that we want it w.r.t. the original root frame
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_3 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";

  //!< Segment 4
  ASSERT_GE(0, kdl_solver[SEGMENT_4]->JntToCart(joint_config_1_4_, kdl_pose))<< "Non-Kinematica Error: Could not compute FK from KDL";
  ASSERT_GE(0, kdl_solver[SEGMENT_4]->JntToCart(joint_config_1_4_, kdl_pose_2, 0))<< "Non-Kinematica Error: Could not compute FK from KDL";
  kdl_pose = kdl_pose_2.Inverse() * kdl_pose; //!< Get the actual position in terms of the tip of the root, which is what Kinematica is able to get us...
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_4, ROOT_FRAME, kinematica_pose)); //!< Specify that we want it w.r.t. the original root frame
  kdl_pose.M.GetQuaternion(kdl_x, kdl_y, kdl_z, kdl_w);
  kinematica_pose.M.GetQuaternion(kin_x, kin_y, kin_z, kin_w);
  EXPECT_TRUE(KDL::Equal(kdl_pose, kinematica_pose, 0.001))
      << "FK not correct for " << SEGMENT_4 << "! : x(" << std::setprecision(4)
      << kdl_pose.p.x() << "/" << kinematica_pose.p.x() << ") : y("
      << kdl_pose.p.y() << "/" << kinematica_pose.p.y() << ") : z("
      << kdl_pose.p.z() << "/" << kinematica_pose.p.z() << ") : q_x(" << kdl_x
      << "/" << kin_x << ") : q_y(" << kdl_y << "/" << kin_y << ") : q_z("
      << kdl_z << "/" << kin_z << ") : q_w(" << kdl_w << "/" << kin_w << ")";
}

TEST_F(KinematicaAtlasTest, TestingPhiComputation)
{
  //!< Temporaries
  KDL::Frame result_pose;
  Eigen::VectorXd individual = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd forward_map;

  ASSERT_TRUE(robot_tree_.initKinematics(robot_urdf_, robot_solution_))<< "Cannot proceed with Test since could Not Initialise Robot";

  //!< First test with zeroed out joint angles
  ASSERT_TRUE(robot_tree_.updateConfiguration(Eigen::VectorXd::Zero(NJOINTS)))<< "Could not update kinematics";

  //!< Check that they match up
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_1, result_pose));
  individual.segment(0, 3) = exotica::vectorKdlToEigen(result_pose.p);
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_2, result_pose));
  individual.segment(3, 3) = exotica::vectorKdlToEigen(result_pose.p);
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_3, result_pose));
  individual.segment(6, 3) = exotica::vectorKdlToEigen(result_pose.p);
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_4, result_pose));
  individual.segment(9, 3) = exotica::vectorKdlToEigen(result_pose.p);

  ASSERT_TRUE(robot_tree_.generateForwardMap(forward_map));
  EXPECT_TRUE(compareVectors(individual, forward_map, 0.0001))
      << "The values do not match up";

  //!< First test with zeroed out joint angles
  ASSERT_TRUE(robot_tree_.updateConfiguration(joint_config_1_))<< "Could not update kinematics";

  //!< Check that they match up
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_1, result_pose));
  individual.segment(0, 3) = exotica::vectorKdlToEigen(result_pose.p);
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_2, result_pose));
  individual.segment(3, 3) = exotica::vectorKdlToEigen(result_pose.p);
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_3, result_pose));
  individual.segment(6, 3) = exotica::vectorKdlToEigen(result_pose.p);
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_4, result_pose));
  individual.segment(9, 3) = exotica::vectorKdlToEigen(result_pose.p);

  ASSERT_TRUE(robot_tree_.generateForwardMap(forward_map));
  EXPECT_TRUE(compareVectors(individual, forward_map, 0.0001))
      << "The values do not match up";

  robot_solution_.root_segment = ROOT_ALTER;
  ASSERT_TRUE(robot_tree_.initKinematics(robot_urdf_, robot_solution_))<< "Cannot proceed with Test since could Not Initialise Robot";

  //!< First test with zeroed out joint angles
  ASSERT_TRUE(robot_tree_.updateConfiguration(Eigen::VectorXd::Zero(NJOINTS)))<< "Could not update kinematics";

  //!< Check that they match up
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_1, result_pose));
  individual.segment(0, 3) = exotica::vectorKdlToEigen(result_pose.p);
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_2, result_pose));
  individual.segment(3, 3) = exotica::vectorKdlToEigen(result_pose.p);
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_3, result_pose));
  individual.segment(6, 3) = exotica::vectorKdlToEigen(result_pose.p);
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_4, result_pose));
  individual.segment(9, 3) = exotica::vectorKdlToEigen(result_pose.p);

  ASSERT_TRUE(robot_tree_.generateForwardMap(forward_map));
  EXPECT_TRUE(compareVectors(individual, forward_map, 0.0001))
      << "The values do not match up";

  //!< First test with zeroed out joint angles
  ASSERT_TRUE(robot_tree_.updateConfiguration(joint_config_1_))<< "Could not update kinematics";

  //!< Check that they match up
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_1, result_pose));
  individual.segment(0, 3) = exotica::vectorKdlToEigen(result_pose.p);
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_2, result_pose));
  individual.segment(3, 3) = exotica::vectorKdlToEigen(result_pose.p);
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_3, result_pose));
  individual.segment(6, 3) = exotica::vectorKdlToEigen(result_pose.p);
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_4, result_pose));
  individual.segment(9, 3) = exotica::vectorKdlToEigen(result_pose.p);

  ASSERT_TRUE(robot_tree_.generateForwardMap(forward_map));
  EXPECT_TRUE(compareVectors(individual, forward_map, 0.0001))
      << "The values do not match up";

  //!< Now test that they work when you change the end-effectors
  robot_solution_.end_effector_segs =
  { SEGMENT_4, SEGMENT_2};
  robot_solution_.end_effector_offs.clear();
  ASSERT_TRUE(robot_tree_.updateEndEffectors(robot_solution_));
  individual = Eigen::VectorXd::Zero(6);  //!< Resize

  //!< Check that they match up
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_4, result_pose));
  individual.segment(0, 3) = exotica::vectorKdlToEigen(result_pose.p);
  ASSERT_TRUE(robot_tree_.getPose(SEGMENT_2, result_pose));
  individual.segment(3, 3) = exotica::vectorKdlToEigen(result_pose.p);

  ASSERT_TRUE(robot_tree_.generateForwardMap(forward_map));
  EXPECT_TRUE(compareVectors(individual, forward_map, 0.0001))
      << "The values do not match up";
}

TEST_F(KinematicaAtlasTest, TestingJacobianComputation)
{
  //!< Temporaries
  Eigen::VectorXd initial_pose;
  Eigen::VectorXd final_pose;
  Eigen::VectorXd diff_pose;
  Eigen::VectorXd config_var;
  Eigen::MatrixXd jacobian;

  //!< Start first with Normal root
  ASSERT_TRUE(robot_tree_.initKinematics(robot_urdf_, robot_solution_))<< "Cannot proceed with Test since could Not Initialise Robot";

  //!< Start
  config_var = Eigen::VectorXd::Zero(NJOINTS);
  ASSERT_TRUE(robot_tree_.updateConfiguration(config_var))<< "Could not update kinematics";
  ASSERT_TRUE(robot_tree_.generateForwardMap(initial_pose))<< "Could not generate phi";
  ASSERT_TRUE(robot_tree_.generateJacobian(jacobian))<< "Could not compute jacobian";

  for (int i = 0; i < NJOINTS; i++)
  {
    config_var = Eigen::VectorXd::Zero(NJOINTS);
    config_var(i) = EPSILON; //!< Perturb one joint

    ASSERT_TRUE(robot_tree_.updateConfiguration(config_var))<< "Could not update kinematics";
    ASSERT_TRUE(robot_tree_.generateForwardMap(final_pose))<< "Could not generate phi";
    diff_pose = (final_pose - initial_pose) / EPSILON;      //!< Find difference

    Eigen::VectorXd jac_column = jacobian.col(i); //!< Get the desired column (for the current joint)
    EXPECT_TRUE(compareVectors(diff_pose, jac_column, 0.0001))
        << "The Jacobian entry for the perturbed joint " << i << " is wrong\n"
        << jac_column - diff_pose << "\n";
  }

  //!< Start
  config_var = joint_config_1_;
  ASSERT_TRUE(robot_tree_.updateConfiguration(config_var))<< "Could not update kinematics";
  ASSERT_TRUE(robot_tree_.generateForwardMap(initial_pose))<< "Could not generate phi";
  ASSERT_TRUE(robot_tree_.generateJacobian(jacobian))<< "Could not compute jacobian";

  for (int i = 0; i < NJOINTS; i++)
  {
    config_var = joint_config_1_;
    config_var(i) += EPSILON; //!< Perturb one joint

    ASSERT_TRUE(robot_tree_.updateConfiguration(config_var))<< "Could not update kinematics";
    ASSERT_TRUE(robot_tree_.generateForwardMap(final_pose))<< "Could not generate phi";
    diff_pose = (final_pose - initial_pose) / EPSILON;      //!< Find difference

    Eigen::VectorXd jac_column = jacobian.col(i); //!< Get the desired column (for the current joint)
    EXPECT_TRUE(compareVectors(diff_pose, jac_column, 0.00001))
        << "The Jacobian entry for the perturbed joint " << i << " is wrong\n"
        << jac_column - diff_pose << "\n";
  }

  //!<========================================================================//

  //!< Repeat with alternative root
  robot_solution_.root_segment = ROOT_ALTER;
  ASSERT_TRUE(robot_tree_.initKinematics(robot_urdf_, robot_solution_))<< "Cannot proceed with Test since could Not Initialise Robot";

  //!< Start
  config_var = Eigen::VectorXd::Zero(NJOINTS);
  ASSERT_TRUE(robot_tree_.updateConfiguration(config_var))<< "Could not update kinematics";
  ASSERT_TRUE(robot_tree_.generateForwardMap(initial_pose))<< "Could not generate phi";
  ASSERT_TRUE(robot_tree_.generateJacobian(jacobian))<< "Could not compute jacobian";

  for (int i = 0; i < NJOINTS; i++)
  {
    config_var = Eigen::VectorXd::Zero(NJOINTS);
    config_var(i) = EPSILON; //!< Perturb one joint

    ASSERT_TRUE(robot_tree_.updateConfiguration(config_var))<< "Could not update kinematics";
    ASSERT_TRUE(robot_tree_.generateForwardMap(final_pose))<< "Could not generate phi";
    diff_pose = (final_pose - initial_pose) / EPSILON;      //!< Find difference

    Eigen::VectorXd jac_column = jacobian.col(i); //!< Get the desired column (for the current joint)
    EXPECT_TRUE(compareVectors(diff_pose, jac_column, 0.00001))
        << "====================\n" << std::setprecision(4) << jac_column
        << "\n==========\n" << diff_pose
        << "\nThe Jacobian entry for the perturbed joint " << i
        << " is wrong (Alternative Root/Zero Vector)\n"
        << jac_column - diff_pose << "\n\n";
  }

  //!< Start
  config_var = joint_config_1_;
  ASSERT_TRUE(robot_tree_.updateConfiguration(config_var))<< "Could not update kinematics";
  ASSERT_TRUE(robot_tree_.generateForwardMap(initial_pose))<< "Could not generate phi";
  ASSERT_TRUE(robot_tree_.generateJacobian(jacobian))<< "Could not compute jacobian";

  for (int i = 0; i < NJOINTS; i++)
  {
    config_var = joint_config_1_;
    config_var(i) += EPSILON; //!< Perturb one joint

    ASSERT_TRUE(robot_tree_.updateConfiguration(config_var))<< "Could not update kinematics";
    ASSERT_TRUE(robot_tree_.generateForwardMap(final_pose))<< "Could not generate phi";
    diff_pose = (final_pose - initial_pose) / EPSILON;      //!< Find difference

    Eigen::VectorXd jac_column = jacobian.col(i); //!< Get the desired column (for the current joint)
    EXPECT_TRUE(compareVectors(diff_pose, jac_column, 0.0001))
        << "The Jacobian entry for the perturbed joint " << i
        << " is wrong (Alternative Root/Random Vector)\n"
        << jac_column - diff_pose << "\n";
  }

  //!< Repeat with one of segments as root
  robot_solution_.root_segment = SEGMENT_1;
  ASSERT_TRUE(robot_tree_.initKinematics(robot_urdf_, robot_solution_))<< "Cannot proceed with Test since could Not Initialise Robot";

  //!< Start
  config_var = Eigen::VectorXd::Zero(NJOINTS);
  ASSERT_TRUE(robot_tree_.updateConfiguration(config_var))<< "Could not update kinematics";
  ASSERT_TRUE(robot_tree_.generateForwardMap(initial_pose))<< "Could not generate phi";
  ASSERT_TRUE(robot_tree_.generateJacobian(jacobian))<< "Could not compute jacobian";

  for (int i = 0; i < NJOINTS; i++)
  {
    config_var = Eigen::VectorXd::Zero(NJOINTS);
    config_var(i) = EPSILON; //!< Perturb one joint

    ASSERT_TRUE(robot_tree_.updateConfiguration(config_var))<< "Could not update kinematics";
    ASSERT_TRUE(robot_tree_.generateForwardMap(final_pose))<< "Could not generate phi";
    diff_pose = (final_pose - initial_pose) / EPSILON;      //!< Find difference

    Eigen::VectorXd jac_column = jacobian.col(i); //!< Get the desired column (for the current joint)
    EXPECT_TRUE(compareVectors(diff_pose, jac_column, 0.00001))
        << "====================\n" << std::setprecision(4) << jac_column
        << "\n==========\n" << diff_pose
        << "\nThe Jacobian entry for the perturbed joint " << i
        << " is wrong (Alternative Root/Zero Vector)\n"
        << jac_column - diff_pose << "\n\n";

    Eigen::VectorXd temp;
    temp = jac_column.segment(0, 3);
    EXPECT_TRUE(compareVectors(Eigen::VectorXd::Zero(3), temp, 0.00001))
        << " Jacobian Entry for " << SEGMENT_1
        << " with itself as root should be 0!";
    temp = diff_pose.segment(0, 3);
    EXPECT_TRUE(compareVectors(Eigen::VectorXd::Zero(3), temp, 0.00001))
        << " Forward Kinematics Difference Entry for " << SEGMENT_1
        << " with itself as root should be 0!";
  }

  //!< Start
  config_var = joint_config_1_;
  ASSERT_TRUE(robot_tree_.updateConfiguration(config_var))<< "Could not update kinematics";
  ASSERT_TRUE(robot_tree_.generateForwardMap(initial_pose))<< "Could not generate phi";
  ASSERT_TRUE(robot_tree_.generateJacobian(jacobian))<< "Could not compute jacobian";

  for (int i = 0; i < NJOINTS; i++)
  {
    config_var = joint_config_1_;
    config_var(i) += EPSILON; //!< Perturb one joint

    ASSERT_TRUE(robot_tree_.updateConfiguration(config_var))<< "Could not update kinematics";
    ASSERT_TRUE(robot_tree_.generateForwardMap(final_pose))<< "Could not generate phi";
    diff_pose = (final_pose - initial_pose) / EPSILON;      //!< Find difference

    Eigen::VectorXd jac_column = jacobian.col(i); //!< Get the desired column (for the current joint)
    EXPECT_TRUE(compareVectors(diff_pose, jac_column, 0.0001))
        << "The Jacobian entry for the perturbed joint " << i
        << " is wrong (Alternative Root/Random Vector)\n"
        << jac_column - diff_pose << "\n";

    Eigen::VectorXd temp;
    temp = jac_column.segment(0, 3);
    EXPECT_TRUE(compareVectors(Eigen::VectorXd::Zero(3), temp, 0.00001))
        << " Jacobian Entry for " << SEGMENT_1
        << " with itself as root should be 0!";
    temp = diff_pose.segment(0, 3);
    EXPECT_TRUE(compareVectors(Eigen::VectorXd::Zero(3), temp, 0.00001))
        << " Forward Kinematics Difference Entry for " << SEGMENT_1
        << " with itself as root should be 0!";
  }
}

TEST_F(KinematicaAtlasTest, TestingChangeEndEffector)
{
  //!< Start first with Normal root
  ASSERT_TRUE(robot_tree_.initKinematics(robot_urdf_, robot_solution_))<< "Cannot proceed with Test since could Not Initialise Robot";

  EXPECT_FALSE(robot_tree_.addEndEffector("sadasdasd",KDL::Frame::Identity())) << "Added non-existing end-effector";

  EXPECT_TRUE(robot_tree_.addEndEffector("l_clav",KDL::Frame::Identity())) << "Could not add end-effector";

  EXPECT_FALSE(robot_tree_.addEndEffector(SEGMENT_1,KDL::Frame::Identity())) << "Added same end-effector";

  EXPECT_FALSE(robot_tree_.removeEndEffector("sdfsdfsdf")) << "Removed non-existing end-effector";

  EXPECT_TRUE(robot_tree_.removeEndEffector(SEGMENT_1)) << "Could not remove end-effector";

  EXPECT_FALSE(robot_tree_.removeEndEffector("l_scap")) << "Removed segment which is not an end-effector";

  EXPECT_FALSE(robot_tree_.modifyEndEffector("sdfgsdfgdfg",KDL::Frame::Identity())) << "Modified non-existing end-effector";

  EXPECT_TRUE(robot_tree_.modifyEndEffector(SEGMENT_2,KDL::Frame::Identity())) << "Could not modify end-effector";

}

TEST_F(KinematicaAtlasTest, TestingChangeEndEffectorNotInit)
{
  //!< Start first with Normal root
  EXPECT_FALSE(robot_tree_.addEndEffector("sadasdasd",KDL::Frame::Identity()))
      << "Added non-existing end-effector";

  EXPECT_FALSE(robot_tree_.addEndEffector("l_clav",KDL::Frame::Identity()))
      << "Could not add end-effector";

  EXPECT_FALSE(robot_tree_.addEndEffector(SEGMENT_1,KDL::Frame::Identity()))
      << "Added same end-effector";

  EXPECT_FALSE(robot_tree_.removeEndEffector("sdfsdfsdf"))
      << "Removed non-existing end-effector";

  EXPECT_FALSE(robot_tree_.removeEndEffector(SEGMENT_1))
      << "Could not remove end-effector";

  EXPECT_FALSE(robot_tree_.removeEndEffector("l_scap"))
      << "Removed segment which is not an end-effector";

  EXPECT_FALSE(robot_tree_.modifyEndEffector("sdfgsdfgdfg",KDL::Frame::Identity()))
      << "Modified non-existing end-effector";

  EXPECT_FALSE(robot_tree_.modifyEndEffector(SEGMENT_2,KDL::Frame::Identity()))
      << "Could not modify end-effector";

}

TEST_F(KinematicaAtlasTest, CoMTest)
{
  ASSERT_TRUE(robot_tree_.initKinematics(robot_urdf_, robot_solution_))<< "Cannot proceed with Test since could Not Initialise Robot";
  EXPECT_TRUE(robot_tree_.generateCoM());
}
//TEST_F(KinematicaAtlasTest,TestAfterChangeEndEffector)
//{
//	ASSERT_TRUE(robot_tree_.initKinematics(robot_urdf_, robot_solution_))<< "Cannot proceed with Test since could Not Initialise Robot";
//	EXPECT_TRUE(robot_tree_.addEndEffector("l_clav",KDL::Frame::Identity())) << "Could not add end-effector";
//
//	Eigen::VectorXd config_var;
//	Eigen::MatrixXd jacobian;
//	config_var = Eigen::VectorXd::Zero(NJOINTS);
//	ASSERT_TRUE(robot_tree_.updateConfiguration(config_var)) << "Could not update kinematics";
//	ASSERT_TRUE(robot_tree_.generateForwardMap(initial_pose)) << "Could not generate phi";
//	ASSERT_TRUE(robot_tree_.generateJacobian(jacobian)) << "Could not compute jacobian";
//}

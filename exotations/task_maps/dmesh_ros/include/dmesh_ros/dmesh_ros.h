/*
 *      Author: Yiming Yang
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

#ifndef DMESH_ROS_H_
#define DMESH_ROS_H_

//EXOTica and SYSTEM packages
#include <exotica/Exotica.h>
#include <exotica/KinematicTree.h>
#include <boost/thread/mutex.hpp>
#include "GraphManager.h"
#include <tf/transform_listener.h>
#include <exotica/Problems/UnconstrainedEndPoseProblem.h>
#include <exotica/Definitions/TaskSqrError.h>
//ROS packages
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

#include <dmesh_ros/DMeshROSInitializer.h>

namespace exotica
{
  /**
   * \brief	Implementation of distance mesh task map with ROS.
   * Apart from dMesh task map, this task map use exotica and ROS node, msgs, etc.
   * The main improvement is now dMeshROS taking computation, visualisation, and robust into consideration
   * L(d_jl)=K*||p_j-p_l||, K(gain)={PoseGain(kp),ObstacleGain(ko),GoalGain(kg)}
   */
  class DMeshROS: public TaskMap, public Instantiable<DMeshROSInitializer>
  {
    public:
      /**
       * \brief	Default constructor
       */
      DMeshROS();

      virtual void Instantiate(DMeshROSInitializer& init);

      /**
       * \brief	Default destructor
       */
      ~DMeshROS();

      /**
       * \brief	Concrete implementation of update method
       * @param	x		Joint space configuration
       */
      virtual void update(Eigen::VectorXdRefConst x, const int t);

      /**
       * \brief	Get the task space dimension
       */
      virtual void taskSpaceDim(int & task_dim);

      /**
       * \brief	Get the goal laplace
       * @param	goal	Goal laplace
       */
      void getGoalLaplace(Eigen::VectorXd & goal, int t);

      void getLaplace(Eigen::VectorXd & lap);
      /**
       * \brief	Update external objects
       */
      void updateExternal(const exotica::MeshVertex & ext);
      void updateExternal(const exotica::MeshVertexArray & ext);

      void removeVertex(const std::string & name);

      bool hasActiveObstacle();

      //	Graph Manager
      GraphManager gManager_;

    private:
      /**
       * \brief	Compute Laplace
       */
      void computeLaplace(int t);

      /**
       * \brief	Compute Jacobian
       */
      void computeJacobian(int t);

      /**
       * \brief	Update the graph from kinematic scene
       */
      void updateGraphFromKS(int t);

      /**
       * \brief	Update the graph externally
       * @param	name		Vertex name
       * @param	pose		Vertex position
       */
      void updateGraphFromExternal(const std::string & name,
          const Eigen::Vector3d & pose);

      /**
       * \brief	Update the graph from real transform
       */
      void updateGraphFromTF();
      /**
       * \brief	Update the graph from given poses
       * @param	V		The given links' poses
       */
      void updateGraphFromPoses(const Eigen::Matrix3Xd & V);

      std::vector<std::string> links_;
      std::vector<bool> link_types_;
      Eigen::VectorXd radius_;

      //	If we want to get real joint state
      tf::TransformListener listener_;

      tf::StampedTransform transform_;

      //	Maximum graph size
      int size_;

      //	Robot links size
      int robot_size_;

      //	External objects size
      int ext_size_;

      //	Task space size
      int task_size_;

      //	Configuration size
      int q_size_;

      double kp_;
      double ko_;
      double kg_;

      //	Distance matrix
      Eigen::MatrixXd dist_;

      //	Initialisation flag
      bool initialised_;

      //	Scoped locker
      boost::mutex::scoped_lock lock_;

      //	True if the obstacle is close
      std::vector<bool> obs_close_;

      //	Interact Range
      double ir_;

      double wo_;
      double wg_;

      bool usePose_;


  };
  typedef boost::shared_ptr<DMeshROS> DMeshROS_Ptr;
} //namespace exotica

#endif /* DMESH_ROS_H_ */

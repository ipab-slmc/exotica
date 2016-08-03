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

#ifndef COM_H_
#define COM_H_

#include <exotica/TaskMap.h>
#include <task_map/CoMInitializer.h>
#include <exotica/KinematicTree.h>
#include <fstream>
#include <visualization_msgs/MarkerArray.h>
namespace exotica
{
  /**
   * @brief	Centre of mass Task Map.
   * 			Using a short-cut to get the jacobian from kinematica, not efficient.
   */
  class CoM: public TaskMap, public Instantiable<CoMInitializer>
  {
    public:
      /**
       * @brief	Constructor of CoMTaskMap
       */
      CoM();

      virtual void Instantiate(CoMInitializer& init) {};

      /**
       * @brief	Destructor of CoMTaskMap
       */
      virtual ~CoM();

      /**
       * @brief	Concrete implementation of the update method
       * @param	x	Input configuration
       * @return	Exotica return type
       */
      virtual void update(Eigen::VectorXdRefConst x, const int t);

      /**
       * @brief	Get the task space dimension
       */

      /**
       * \brief Concrete implementation of the task-space size
       */
      virtual void taskSpaceDim(int & task_dim);

      void setOffsetCallback(
          boost::function<void(CoM*, Eigen::VectorXdRefConst, int)> offset_callback);
      void setOffset(bool left, const KDL::Frame & offset);
      void checkGoal(const Eigen::Vector3d & goal);
      EParam<exotica::Vector> getBounds()
      {
        return bounds_;
      }

    protected:
      /**
       * @brief	Concrete implementation of the initialisation method
       * @param	handle	XML handle
       * @return	Exotica return type
       */
      virtual void initDerived(tinyxml2::XMLHandle & handle);

    private:
      /**
       * @brief	Compute the forward map (centre of mass position)
       * @return	True if succeeded, false otherwise
       */
      void computeForwardMap(int t);

      /**
       * @brief	Compute the jacobian
       * @return	True if succeeded, false otherwise
       */
      void computeJacobian(int t);

      /**
       * @brief	Change end-effectors offset to centre of mass
       * @return	True if succeeded, false otherwise
       */
      void changeEffToCoM();
      Eigen::VectorXd mass_;	//!< Mass of each link
      std::vector<KDL::Vector> cog_;	//!< Centre of gravity of each link
      std::vector<KDL::Frame> tip_pose_;	//!< Tip poses
      std::vector<KDL::Frame> base_pose_;	//!< Base poses
      bool initialised_;	//!< For Error checking
      boost::function<void(CoM*, Eigen::VectorXdRefConst, int)> offset_callback_;
      ros::Publisher com_pub_;
      ros::Publisher COM_pub_;
      ros::Publisher goal_pub_;
      visualization_msgs::Marker com_marker_;
      visualization_msgs::Marker COM_marker_;
      visualization_msgs::Marker goal_marker_;
      KDL::Frame base_offset_;
      KDL::Frame marker_offset_;
      std::ofstream com_file_;
      EParam<std_msgs::Bool> debug_;
      EParam<std_msgs::Bool> enable_z_;
      EParam<exotica::Vector> bounds_;
      int dim_;
  };
}

#endif /* COM_H_ */

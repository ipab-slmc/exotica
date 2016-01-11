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

#ifndef EXOTICA_EXOTATIONS_TASK_MAPS_TASK_MAP_INCLUDE_KINEMATIC_MAPS_DMESH_H_
#define EXOTICA_EXOTATIONS_TASK_MAPS_TASK_MAP_INCLUDE_KINEMATIC_MAPS_DMESH_H_
#include <exotica/EXOTica.hpp>
namespace exotica
{
  /**
   * \brief	Implementation of relative distance space representation, with mesh objects
   */
  class DMesh: public TaskMap
  {
    public:
      DMesh();
      ~DMesh();

      virtual EReturn update(Eigen::VectorXdRefConst x, const int t);
      virtual EReturn taskSpaceDim(int & task_dim);

      EReturn getGoalLaplace(Eigen::VectorXd & goal, int t = 0);
      EReturn addGoal(const std::string & name, std::string & toLink);
      EReturn addObstacle(const std::string & name);
    protected:
      virtual EReturn initDerived(tinyxml2::XMLHandle & handle);

    private:
      EReturn computeLaplace(int t = 0);
      EReturn computeJacobian(int t = 0);
      EReturn updateDistances();
      EReturn updatePositions();

      class Vertex
      {
        public:
          enum TYPE
          {
            LINK = 0,
            DUMMY_LINK = 5,
            GOAL = 10,
            OBSTACLE = 20,
            OBSTACLE_TO_ALL = 30,
            IGNORE = 40,
            NONE = 255
          };
          Vertex(const std::string & name, TYPE type, int index)
              : name_(name), type_(type), index_(index)
          {
            toVertices_.clear();
          }
          ~Vertex()
          {
          }

          void addNewRelation(const std::string & vertex, int index)
          {
            if (toVertices_.find(vertex) == toVertices_.end())
              toVertices_[vertex] = index;
            else
              toVertices_.at(vertex) = index;
          }
          std::string name_;
          TYPE type_;
          int index_;
          Eigen::Vector3d position_;
          std::map<std::string, int> toVertices_;

      };
      typedef boost::shared_ptr<Vertex> Vertex_Ptr;

      std::vector<Vertex_Ptr> vertices_;

      std::map<std::string, int> vertex_map_;

      //	Robot links used for dmesh
      EParam<exotica::StringList> links_;

      //	Link types
      EParam<exotica::BoolList> link_types_;

      //	Maximum graph size
      int graph_size_;

      int robot_size_;
      int ext_size_;

      int current_size_;

      //	Gain to keep robot pose
      EParam<std_msgs::Float64> kp_;

      //	Gain to avoid obstacle
      EParam<std_msgs::Float64> ko_;

      //	Gain to reach goal
      EParam<std_msgs::Float64> kg_;

      //	Gain in exp term
      EParam<std_msgs::Float64> kexp_;

      EParam<std_msgs::Float64> di_;

      //	Distance matrix
      Eigen::MatrixXd dist_;

      int task_size_;
      int q_size_;

  };

  typedef boost::shared_ptr<DMesh> DMesh_Ptr;
} //	namespace exotica

#endif /* EXOTICA_EXOTATIONS_TASK_MAPS_TASK_MAP_INCLUDE_KINEMATIC_MAPS_DMESH_H_ */

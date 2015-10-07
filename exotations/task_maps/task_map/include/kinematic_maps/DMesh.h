/*
 * DMesh.h
 *
 *  Created on: 15 Jun 2015
 *      Author: yiming
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

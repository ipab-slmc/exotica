/*
 * dynamic_reachability_map.cpp
 *
 *  Created on: 26 Aug 2015
 *      Author: yiming
 */

#include "dynamic_reachability_map/dynamic_reachability_map.h"
#include <eigen_conversions/eigen_msg.h>

double dRand(double dMin, double dMax)
{
  double d = (double) rand() / RAND_MAX;
  return dMin + d * (dMax - dMin);
}
std::vector<std::pair<Eigen::Vector3i, double> > getNeighborOffsets()
{
  std::vector<std::pair<Eigen::Vector3i, double> > ret;
  ret.clear();
  int depth = 1;
  for (int x = -1 * depth; x <= depth; x++)
    for (int y = -1 * depth; y <= depth; y++)
      for (int z = -1 * depth; z <= depth; z++)
      {
        if (!(x == 0 && y == 0 && z == 0))
        {
          std::pair<Eigen::Vector3i, double> tmp;
          tmp.first << x, y, z;
          tmp.second = sqrt(tmp.first.cwiseAbs().sum());
          ret.push_back(tmp);
        }
      }
  return ret;
}

namespace dynamic_reachability_map
{
  DynamicReachabilityMap::DynamicReachabilityMap()
      : space_size_(0), thread_id_(0)
  {
  }

  DynamicReachabilityMap::~DynamicReachabilityMap()
  {

  }

  bool DynamicReachabilityMap::createSpace(const Eigen::Vector2d & bx,
      const Eigen::Vector2d & by, const Eigen::Vector2d & bz, double cellsize)
  {
    bounds_.resize(3);
    bounds_[0] = bx;
    bounds_[1] = by;
    bounds_[2] = bz;
    int size_x = (bx(1) - bx(0)) / cellsize + 1;
    int size_y = (by(1) - by(0)) / cellsize + 1;
    int size_z = (bz(1) - bz(0)) / cellsize + 1;
    cell_cnts_(0) = size_x;
    cell_cnts_(1) = size_y;
    cell_cnts_(2) = size_z;

    space_size_ = size_x * size_y * size_z;
    cell_size_ = cellsize;
    space_.resize(space_size_);
    tcps_.resize(space_size_);
    density_.resize(space_size_);
    for (int i = 0; i < space_size_; i++)
    {
      space_[i].occupiedSmaples.clear();
      tcps_[i].index = i;
      tcps_[i].samples.clear();
    }
    int cnt = 0;

    double current_x = bx(0) + 0.5 * cellsize;
    double current_y = by(0) + 0.5 * cellsize;
    double current_z = bz(0) + 0.5 * cellsize;

    int int_x = 0;
    int int_y = 0;
    int int_z = 0;
    for (int z = 0; z < size_z; z++)
    {
      current_y = by(0) + 0.5 * cellsize;
      int_y = 0;
      for (int y = 0; y < size_y; y++)
      {
        current_x = bx(0) + 0.5 * cellsize;
        int_x = 0;
        for (int x = 0; x < size_x; x++)
        {
          space_[cnt].index = cnt;
          space_[cnt].centre.x = current_x;
          space_[cnt].centre.y = current_y;
          space_[cnt].centre.z = current_z;
          space_[cnt].space_index.resize(3);
          space_[cnt].space_index(0) = int_x;
          space_[cnt].space_index(1) = int_y;
          space_[cnt].space_index(2) = int_z;
          current_x += cellsize;
          int_x++;
          cnt++;
        }
        current_y += cellsize;
        int_y++;
      }
      current_z += cellsize;
      int_z++;
    }
//  ROS_INFO_STREAM(
//      "Indexed Space created with "<<space_size_<<" cells. cell size="<<cell_size_<<" xcnt="<<size_x<<" ycnt="<<size_y<<" zcnt="<<size_z);
    return true;
  }

  bool DynamicReachabilityMap::createRobotSampler(
      const robot_model::RobotModelConstPtr &model, const std::string & eff,
      const std::string & group_name)
  {
    if (!model->hasLinkModel(eff))
    {
      ROS_ERROR_STREAM("Robot "<<model->getName()<<" does not have link "<<eff);
      return false;
    }
    ps_.reset(new planning_scene::PlanningScene(model));
    root_link_ = ps_->getRobotModel()->getRootLinkName();
    eff_ = eff;
    group_ = model->getJointModelGroup(group_name);
    var_index_ = group_->getVariableIndexList();
    dimension_ = group_->getVariableCount();
    std::vector<std::string> joint_names =
        ps_->getCurrentState().getVariableNames();
    joint_names_.resize(var_index_.size());
    for (int i = 0; i < var_index_.size(); i++)
      joint_names_[i] = joint_names[var_index_[i]];
    return true;
  }

  bool DynamicReachabilityMap::isStateValid(const ompl::base::State *state)
  {
    Eigen::VectorXd eigen(dimension_);
    const ompl::base::RealVectorStateSpace::StateType *statetype =
        static_cast<const ompl::base::RealVectorStateSpace::StateType*>(state);
    memcpy(eigen.data(), statetype->values, sizeof(double) * dimension_);
    for (int i = 0; i < dimension_; i++)
      ps_->getCurrentStateNonConst().setVariablePosition(joint_names_[i],
          eigen(i));
    ps_->getCurrentStateNonConst().update(true);
    bool valid = ps_->isStateValid(ps_->getCurrentState());
    return valid;
  }

  void DynamicReachabilityMap::pathSimplifier(const Eigen::MatrixXd & path,
      Eigen::MatrixXd &simple)
  {

  }

  bool DynamicReachabilityMap::createSpacePlanningScene(
      moveit_msgs::PlanningSceneWorld &world,
      std::map<std::string, int> &cell_name_map)
  {
    for (int i = 0; i < space_size_; i++)
    {
      moveit_msgs::CollisionObject cell_obj;
      cell_obj.header.frame_id = root_link_;
      cell_obj.id = "cell_" + std::to_string(i);
      cell_name_map[cell_obj.id] = i;
      shape_msgs::SolidPrimitive cell;
      cell.type = cell.BOX;
      cell.dimensions.resize(3);
      cell.dimensions[0] = cell.dimensions[1] = cell.dimensions[2] = cell_size_;
      geometry_msgs::Pose pose;
      pose.orientation.w = 1;
      pose.position = space_[i].centre;
      cell_obj.primitives.push_back(cell);
      cell_obj.primitive_poses.push_back(pose);
      cell_obj.operation = cell_obj.APPEND;
      world.collision_objects.push_back(cell_obj);
    }
    return true;
  }

  bool DynamicReachabilityMap::setSpacePlanningScene(
      const moveit_msgs::PlanningSceneWorld world)
  {
    ROS_INFO_STREAM("Thread "<<thread_id_<<": Setting planning scene "<<&world);
    ps_->processPlanningSceneWorldMsg(world);
//  ps_->usePlanningSceneMsg(msg);
    ROS_INFO_STREAM(
        "Thread "<<thread_id_<<": Setting planning scene succeeded with "<<ps_->getWorld()->getObjectIds().size()<<" cell obstacles");
    return true;
  }
  bool DynamicReachabilityMap::indexingSpace(int sample_cnt,
      std::map<std::string, int> cell_name_map)
  {
    ros::Time start = ros::Time::now();
    ROS_INFO_STREAM("Thread "<<thread_id_<<": Start to index the space");
    sample_size_ = sample_cnt;
    samples_.resize(sample_size_);
    for (int i = 0; i < sample_size_; ++i)
    {
      if (sample_size_ < 1000
          || (thread_id_ == 0 && i % (sample_size_ / 1000) == 0))
      {
        std::cout << "Sampling process " << i << " / " << sample_size_ << "("
            << (double) i / (double) sample_size_ * 100.0 << "%).\r";
        std::cout.flush();
      }
      bool valid = false;
      collision_detection::CollisionRequest self_req;
      collision_detection::CollisionResult self_res;
      while (!valid)
      {
        ps_->getCurrentStateNonConst().setToRandomPositions(group_);
        ps_->getCurrentStateNonConst().update(true);
        Eigen::Affine3d effpose =
            ps_->getCurrentStateNonConst().getGlobalLinkTransform(eff_);
        int x = (effpose.translation()(0) - bounds_[0](0)) / cell_size_;
        int y = (effpose.translation()(1) - bounds_[1](0)) / cell_size_;
        int z = (effpose.translation()(2) - bounds_[2](0)) / cell_size_;
        int tcp_index = x + y * cell_cnts_[0]
            + z * cell_cnts_[0] * cell_cnts_[1];
        if (tcp_index != 1549)
        {
          continue;
        }
        ps_->checkSelfCollision(self_req, self_res);
        valid = !self_res.collision;
        self_res.clear();
      }
      ps_->getCurrentStateNonConst().update(true);
      Eigen::Affine3d effpose =
          ps_->getCurrentStateNonConst().getGlobalLinkTransform(eff_);
      if (effpose.translation()(0) < bounds_[0](0)
          || effpose.translation()(0) > bounds_[0](1)
          || effpose.translation()(1) < bounds_[1](0)
          || effpose.translation()(1) > bounds_[1](1)
          || effpose.translation()(2) < bounds_[2](0)
          || effpose.translation()(2) > bounds_[2](1))
      {
        --i;
        continue;
      }
      //	Find out which cell the tcp lies in
      int x = (effpose.translation()(0) - bounds_[0](0)) / cell_size_;
      int y = (effpose.translation()(1) - bounds_[1](0)) / cell_size_;
      int z = (effpose.translation()(2) - bounds_[2](0)) / cell_size_;
      int tcp_index = x + y * cell_cnts_[0] + z * cell_cnts_[0] * cell_cnts_[1];
      if (tcp_index < 0 || tcp_index >= space_size_)
      {
//				ROS_WARN_STREAM("TCP index "<<tcp_index<<" out of bounds "<<space_size_);
        --i;
        continue;
      }
      collision_detection::CollisionRequest request;
      request.contacts = true;
      request.max_contacts = space_size_;
      collision_detection::CollisionResult result;
      //  Find out cells that are occupied by this configuration pose
      ros::Time check_start = ros::Time::now();
      ps_->checkCollision(request, result);

      if (result.collision)
      {
        bool sample_ok = true;
        for (auto &it : result.contacts)
        {
          if (cell_name_map.find(it.first.first) == cell_name_map.end()
              && cell_name_map.find(it.first.second) == cell_name_map.end())
          {
            sample_ok = false;
            break;
          }
        }
        if (sample_ok)
        {
          std::map<int, bool> checked;
          for (auto &it : result.contacts)
          {
            int index = 0;
            if (cell_name_map.find(it.first.first) != cell_name_map.end())
            {
              index = cell_name_map.at(it.first.first);
            }
            else if (cell_name_map.find(it.first.second) != cell_name_map.end())
            {
              index = cell_name_map.at(it.first.second);
            }
            if (checked.find(index) == checked.end())
            {
              space_[index].occupiedSmaples.push_back(i);
              checked[index] = true;
            }
          }
        }
        else
        {
          --i;
        }
      }
      else
      {
        --i;
      }
      result.clear();
      tcps_[tcp_index].samples.push_back(i);
      samples_[i].valid = true;
      tf::poseEigenToMsg(effpose, samples_[i].tcp);
      samples_[i].tcpCell = tcp_index;

      samples_[i].q.resize(group_->getVariableCount());

      for (int var = 0; var < group_->getVariableCount(); var++)
      {
        samples_[i].q(var) = ps_->getCurrentState().getVariablePosition(
            group_->getVariableNames()[var]);
      }
    }
    ROS_INFO("Clearing world");
//  ps_->getWorldNonConst()->clearObjects();
//  ps_.reset(new planning_scene::PlanningScene(ps_->getRobotModel()));
    ROS_INFO_STREAM(
        "Thread "<<thread_id_<<": Indexing finished in "<<ros::Duration(ros::Time::now()-start).toSec()<<"sec, "<<sample_size_<<" states are sampled");
    return true;
  }

  bool DynamicReachabilityMap::update(const geometry_msgs::Point & point,
      int &index)
  {
    int x = (point.x - bounds_[0](0)) / cell_size_;
    int y = (point.y - bounds_[1](0)) / cell_size_;
    int z = (point.z - bounds_[2](0)) / cell_size_;
    index = x + y * cell_cnts_[0] + z * cell_cnts_[0] * cell_cnts_[1];
    if (index < 0 || index > space_size_) return true;
    if (space_[index].space_index(0) != x || space_[index].space_index(1) != y
        || space_[index].space_index(2) != z)
    {
//			ROS_ERROR_STREAM("Invalid point at ("<<point.x<<","<<point.y<<","<<point.z<<") "<<index);
      return false;
    }
    space_[index].isFree = false;
    int sample_cnt = space_[index].occupiedSmaples.size();
    for (int i = 0; i < sample_cnt; i++)
      samples_[space_[index].occupiedSmaples[i]].valid = false;
    return true;
  }

  bool DynamicReachabilityMap::updatePlanningScene(
      const geometry_msgs::Point & point, int &index)
  {
    int x = (point.x - bounds_[0](0)) / cell_size_;
    int y = (point.y - bounds_[1](0)) / cell_size_;
    int z = (point.z - bounds_[2](0)) / cell_size_;
    index = x + y * cell_cnts_[0] + z * cell_cnts_[0] * cell_cnts_[1];
    if (index < 0 || index > space_size_) return true;
    if (space_[index].space_index(0) != x || space_[index].space_index(1) != y
        || space_[index].space_index(2) != z)
    {
      //      ROS_ERROR_STREAM("Invalid point at ("<<point.x<<","<<point.y<<","<<point.z<<") "<<index);
      return false;
    }
    moveit_msgs::PlanningSceneWorld world;
    moveit_msgs::CollisionObject cell_obj;
    cell_obj.header.frame_id = root_link_;
    cell_obj.id = "cell_" + std::to_string(index);
    shape_msgs::SolidPrimitive cell;
    cell.type = cell.BOX;
    cell.dimensions.resize(3);
    cell.dimensions[0] = cell.dimensions[1] = cell.dimensions[2] = cell_size_;
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    pose.position = space_[index].centre;
    cell_obj.primitives.push_back(cell);
    cell_obj.primitive_poses.push_back(pose);
    cell_obj.operation = cell_obj.APPEND;
    world.collision_objects.push_back(cell_obj);
    ps_->processPlanningSceneWorldMsg(world);

    return true;
  }

  void DynamicReachabilityMap::getSpaceCentres(
      std::vector<geometry_msgs::Point> & centres)
  {
    centres.resize(space_size_);
    for (int i = 0; i < space_size_; i++)
      centres[i] = space_[i].centre;
  }
  void DynamicReachabilityMap::resetDensity()
  {
    for (int i = 0; i < space_size_; i++)
    {
      space_[i].isFree = true;
      density_[i] = 0;
    }
    for (int i = 0; i < sample_size_; i++)
      samples_[i].valid = true;
  }
  void DynamicReachabilityMap::computeDensity()
  {
    for (int i = 0; i < space_size_; i++)
    {
      for (int j = 0; j < tcps_[i].samples.size(); j++)
      {
        if (samples_[tcps_[i].samples[j]].valid)
        {
          density_[i]++;
        }
      }
    }
  }

  std::vector<int> &DynamicReachabilityMap::getDensity()
  {
    return density_;
  }

  bool DynamicReachabilityMap::solve(const Eigen::VectorXd & q0,
      const geometry_msgs::Pose & goal, Eigen::MatrixXd & solution,
      Eigen::MatrixXd & smooth_solution, std::vector<int> & sample_path)
  {
    ///	First find the current TCP cell
    if (q0.rows() != dimension_ || q0.rows() != group_->getVariableCount())
    {
      ROS_ERROR_STREAM(
          "Configuration space dim mismatch "<<q0.rows() <<" ! = "<<dimension_);
      return false;
    }
    ps_->getCurrentStateNonConst().setJointGroupPositions(group_, q0);
    ps_->getCurrentStateNonConst().update(true);
    Eigen::Affine3d effpose =
        ps_->getCurrentStateNonConst().getGlobalLinkTransform(eff_);
    if (effpose.translation()(0) < bounds_[0](0)
        || effpose.translation()(0) > bounds_[0](1)
        || effpose.translation()(1) < bounds_[1](0)
        || effpose.translation()(1) > bounds_[1](1)
        || effpose.translation()(2) < bounds_[2](0)
        || effpose.translation()(2) > bounds_[2](1))
    {
      ROS_ERROR_STREAM(
          "Start TCP "<<effpose.translation().transpose()<<" is not in the reachable space");
      return false;
    }
    //	Find out which cell the tcp lies in
    int x = (effpose.translation()(0) - bounds_[0](0)) / cell_size_;
    int y = (effpose.translation()(1) - bounds_[1](0)) / cell_size_;
    int z = (effpose.translation()(2) - bounds_[2](0)) / cell_size_;
    int tcp_index = x + y * cell_cnts_[0] + z * cell_cnts_[0] * cell_cnts_[1];

    if (tcp_index < 0 || tcp_index >= space_size_)
    {
      ROS_ERROR_STREAM(
          "Start TCP "<<effpose.translation().transpose()<<" index "<<tcp_index<<" out of bounds "<<space_size_);
      return false;
    }
    if (space_[tcp_index].space_index(0) != x
        || space_[tcp_index].space_index(1) != y
        || space_[tcp_index].space_index(2) != z)
    {
      ROS_ERROR_STREAM(
          "Error indexing start "<<tcp_index <<" ["<<x<<" "<<y<<" "<<z<<"] != ["<<space_[tcp_index].space_index(0)<<" "<<space_[tcp_index].space_index(1)<<" "<<space_[tcp_index].space_index(2)<<"]");
      return false;
    }

    if (density_[tcp_index] <= 0)
    {
      ROS_ERROR_STREAM(
          "Invalid start state, no sample reaches this TCP cell "<<tcp_index<<" ("<<effpose.translation()(0)<<","<<effpose.translation()(1)<<","<<effpose.translation()(2)<<")");
      return false;
    }
    //	Find out goal index
    x = (goal.position.x - bounds_[0](0)) / cell_size_;
    y = (goal.position.y - bounds_[1](0)) / cell_size_;
    z = (goal.position.z - bounds_[2](0)) / cell_size_;
    int goal_index = x + y * cell_cnts_[0] + z * cell_cnts_[0] * cell_cnts_[1];
    if (goal_index >= space_size_)
    {
      ROS_ERROR_STREAM(
          "Goal index "<<goal_index<<" out of bounds "<<space_size_);
      return false;
    }
    if (space_[goal_index].space_index(0) != x
        || space_[goal_index].space_index(1) != y
        || space_[goal_index].space_index(2) != z)
    {
      ROS_ERROR_STREAM(
          "Error indexing start "<<goal_index<<" ["<<x<<" "<<y<<" "<<z<<"ample] != ["<<space_[goal_index].space_index(0)<<" "<<space_[goal_index].space_index(1)<<" "<<space_[goal_index].space_index(2)<<"]");
      return false;
    }

    if (density_[goal_index] <= 0)
    {
      ROS_ERROR_STREAM("Invalid goal state, no sample reaches this goal cell");
      return false;
    }

    ///	Find closest sample in the TCP cell
    Eigen::VectorXd current = q0;
    int sample_size = tcps_[tcp_index].samples.size();
    double dist = INFINITY;
    int sample_index = -1;
    for (int i = 0; i < sample_size; i++)
    {
      if (samples_[tcps_[tcp_index].samples[i]].valid)
      {
        double tmp_dist =
            (samples_[tcps_[tcp_index].samples[i]].q - current).cwiseAbs().maxCoeff();
        if (tmp_dist < dist)
        {
          dist = tmp_dist;
          sample_index = tcps_[tcp_index].samples[i];
        }
      }
    }
    int cell_sample_cnt;
    std::map<int, double> neighbors = getNeighbors(tcp_index);
    for (std::map<int, double>::iterator it = neighbors.begin();
        it != neighbors.end(); it++)
    {
      cell_sample_cnt = tcps_[it->first].samples.size();
      for (int j = 0; j < cell_sample_cnt; j++)
      {
        if (samples_[tcps_[it->first].samples[j]].valid)
        {
          double tmp_dist =
              (samples_[tcps_[it->first].samples[j]].q - current).cwiseAbs().maxCoeff();
          if (tmp_dist < dist)
          {
            dist = tmp_dist;
            sample_index = tcps_[it->first].samples[j];
          }
        }
      }
    }

    if (sample_index == -1)
    {
      ROS_ERROR_STREAM("No close start sample is found");
      return false;
    }
    ///	Call A* planner
    std::vector<int> path;
    ros::Time start_time = ros::Time::now();
    bool succeed = AStarPathPlanning(tcp_index, q0, goal_index, path);
    if (succeed)
    {
      ROS_INFO_STREAM(
          "A* succeeded in "<<ros::Duration(ros::Time::now()-start_time).toSec()<<"sec");

      ///	Now we need to find configuration space trajectory
      solution.resize(path.size() + 1, dimension_);
      sample_path.resize(path.size() + 1);
      solution.row(0) = q0.transpose();
      sample_path[0] = sample_index;
      current = samples_[sample_index].q;
      for (int i = 0; i < path.size(); i++)
      {
        cell_sample_cnt = tcps_[path[i]].samples.size();
        dist = INFINITY;
        sample_index = -1;
        for (int j = 0; j < cell_sample_cnt; j++)
        {
          if (samples_[tcps_[path[i]].samples[j]].valid)
          {
            double tmp_dist =
                (samples_[tcps_[path[i]].samples[j]].q - current).cwiseAbs().maxCoeff();
            if (tmp_dist < dist)
            {
              dist = tmp_dist;
              sample_index = tcps_[path[i]].samples[j];
            }
          }
        }
        if (sample_index == -1)
        {
          ROS_ERROR_STREAM("No close sample is found at tcp path "<<path[i]);
          return false;
        }
        sample_path[i + 1] = sample_index;
        current = samples_[sample_index].q;
        solution.row(i + 1) = current.transpose();
      }
    }
    else
    {
      ROS_ERROR("A* failed");
      return false;
    }
    smooth_solution = solution;
//  pathSimplifier(solution, smooth_solution);
    return true;
  }

  double DynamicReachabilityMap::getHeuristicDist(int a, int b)
  {
    double weight = 1e3;
    double tmp =
        (space_[b].space_index - space_[a].space_index).cwiseAbs2().sum();
    return weight * sqrt(tmp);
  }

  std::map<int, double> DynamicReachabilityMap::getNeighbors(int index)
  {
    static std::vector<std::pair<Eigen::Vector3i, double> > offsets =
        getNeighborOffsets();
    std::map<int, double> Neighbors;
    for (int i = 0; i < offsets.size(); i++)
    {
      Eigen::Vector3i tmp_space_index = space_[index].space_index
          + offsets[i].first;
      int tmp_index = tmp_space_index(0) + tmp_space_index(1) * cell_cnts_(0)
          + tmp_space_index(2) * cell_cnts_(0) * cell_cnts_(1);
      if (tmp_index < 0 || tmp_index >= space_size_ || tmp_space_index(0) < 0
          || tmp_space_index(1) < 0 || tmp_space_index(2) < 0
          || tmp_space_index(0) >= cell_cnts_(0)
          || tmp_space_index(1) >= cell_cnts_(1)
          || tmp_space_index(2) >= cell_cnts_(2))
      {
//				ROS_ERROR_STREAM("Index out of bounds "<<tmp_index<<" >= "<<space_size_);
        continue;
      }

      if (space_[tmp_index].space_index != tmp_space_index)
      {
        ROS_ERROR_STREAM(
            "Error indexing "<<tmp_index <<", "<<space_[tmp_index].space_index.transpose()<<" != "<<tmp_space_index.transpose());
        continue;
      }
      if (density_[tmp_index] > 0)
      {
        Neighbors[tmp_index] = density_[tmp_index]
            * getHeuristicDist(index, tmp_index);
      }
    }
    return Neighbors;
  }

  bool DynamicReachabilityMap::AStarPathPlanning(int start,
      const Eigen::VectorXd &start_q, int goal, std::vector<int> &path)
  {
///	Lets do A* !
    std::map<int, double> closed;
    std::map<int, double> open;
    std::vector<AStarNode> aStar;
    aStar.resize(space_size_);
    aStar[start].g = 0;
    aStar[start].h = getHeuristicDist(start, goal);
    open[start] = aStar[start].f();

    while (open.size() > 0)
    {
      double min = INFINITY;
      std::map<int, double>::iterator current;
      for (std::map<int, double>::iterator it = open.begin(); it != open.end();
          it++)
      {
        if (it->second < min) current = it;
      }
      if (current->first == goal)
      {
        path.resize(0);
        int index = current->first;
        path.push_back(index);
        while (aStar[index].parent != -1)
        {
          index = aStar[index].parent;
          path.push_back(index);
        }
        std::reverse(path.begin(), path.end());
        return true;
      }

      int tmp_index = current->first;
      closed[current->first] = current->second;
      open.erase(current);
      current = closed.find(tmp_index);
      std::map<int, double> neighbors = getNeighbors(current->first);
      if (neighbors.size() == 0)
      {
        ROS_ERROR("No neighbors");
        return false;
      }
      for (std::map<int, double>::iterator it = neighbors.begin();
          it != neighbors.end(); it++)
      {
        if (closed.find(it->first) != closed.end())
          continue;
        else
        {
          double tentative_g = aStar[current->first].g + it->second;
          if (open.find(it->first) == open.end()
              || tentative_g < aStar[it->first].g)
          {
            aStar[it->first].parent = current->first;
            aStar[it->first].g = tentative_g;
            double h_dist = getHeuristicDist(it->first, goal);
            aStar[it->first].h = h_dist;
            if (open.find(it->first) == open.end())
            {
              open[it->first] = tentative_g + h_dist;
            }
          }
        }
      }
    }
    ROS_ERROR("A Star Failed !");
    return false;
  }

  dynamic_reachability_map::DRMResult DynamicReachabilityMap::getIKSolution(
      const dynamic_reachability_map::DRMGoalConstPtr &goal)
  {
    dynamic_reachability_map::DRMResult result;
    result.succeed = false;
    if (goal->goal_pose.position.x < bounds_[0](0)
        || goal->goal_pose.position.x > bounds_[0](1)
        || goal->goal_pose.position.y < bounds_[1](0)
        || goal->goal_pose.position.y > bounds_[1](1)
        || goal->goal_pose.position.z < bounds_[2](0)
        || goal->goal_pose.position.z > bounds_[2](1))
    {
      ROS_ERROR_STREAM(
          "Start TCP ("<<goal->goal_pose.position.x<<","<<goal->goal_pose.position.y<<","<<goal->goal_pose.position.z<<") is not in the reachable space");
      return result;
    }
    int x = (goal->goal_pose.position.x - bounds_[0](0)) / cell_size_;
    int y = (goal->goal_pose.position.y - bounds_[1](0)) / cell_size_;
    int z = (goal->goal_pose.position.z - bounds_[2](0)) / cell_size_;
    int tcp_index = x + y * cell_cnts_[0] + z * cell_cnts_[0] * cell_cnts_[1];
    if (tcp_index < 0 || tcp_index >= space_size_)
    {
      ROS_ERROR_STREAM(
          "Goal index "<<tcp_index<<" out of bounds "<<space_size_);
      return result;
    }
    if (space_[tcp_index].space_index(0) != x
        || space_[tcp_index].space_index(1) != y
        || space_[tcp_index].space_index(2) != z)
    {
      ROS_ERROR_STREAM(
          "Error indexing start "<<tcp_index<<" ["<<x<<" "<<y<<" "<<z<<"] != ["<<space_[tcp_index].space_index(0)<<" "<<space_[tcp_index].space_index(1)<<" "<<space_[tcp_index].space_index(2)<<"]");
      return result;
    }

    if (density_[tcp_index] <= 0)
    {
      ROS_ERROR_STREAM(
          "Cell "<<tcp_index<<" is not reachable in current environment");
      return result;
    }
    double dist = INFINITY;
    int tmp_index = -1;
    Eigen::VectorXd e1(4 + dimension_);
    e1(0) = goal->goal_pose.orientation.x;
    e1(1) = goal->goal_pose.orientation.y;
    e1(2) = goal->goal_pose.orientation.z;
    e1(3) = goal->goal_pose.orientation.w;
    for (int i = 0; i < dimension_; i++)
      e1(i + 4) = goal->q0.data[i];
    for (int i = 0; i < tcps_[tcp_index].samples.size(); i++)
    {
      if (samples_[tcps_[tcp_index].samples[i]].valid)
      {
        bool invalid = false;
        for (int j = 0; j < goal->invalid_samples.size(); j++)
          if (goal->invalid_samples[j] == tcps_[tcp_index].samples[i])
          {
            invalid = true;
            break;
          }
        if (!invalid)
        {
          Eigen::VectorXd e2(4 + dimension_);

          e2(0) = samples_[tcps_[tcp_index].samples[i]].tcp.orientation.x;
          e2(1) = samples_[tcps_[tcp_index].samples[i]].tcp.orientation.y;
          e2(2) = samples_[tcps_[tcp_index].samples[i]].tcp.orientation.z;
          e2(3) = samples_[tcps_[tcp_index].samples[i]].tcp.orientation.w;
          e2.segment(4, dimension_) = samples_[tcps_[tcp_index].samples[i]].q;
          double tmp_dist = (e1 - e2).norm();
          if (tmp_dist < dist)
          {
            dist = tmp_dist;
            tmp_index = tcps_[tcp_index].samples[i];
          }
        }
      }
    }
    if (tmp_index != -1)
    {
      result.sample_index = tmp_index;
      result.succeed = true;
      exotica::vectorEigenToExotica(samples_[tmp_index].q, result.q_out);
    }
    return result;
  }
  bool DynamicReachabilityMap::checkPath(const std::vector<int> &sample_path)
  {
    for (int i = 0; i < sample_path.size(); i++)
    {
      if (!samples_[sample_path[i]].valid) return false;
    }
    return true;
  }

  bool DynamicReachabilityMap::checkPathDetail(
      const std::vector<int> &sample_path, std::pair<int, int> &disconnect)
  {
    bool valid = true;
    bool isFirst = true;
    for (int i = 0; i < sample_path.size(); i++)
    {
      if (!samples_[sample_path[i]].valid)
      {
        valid = false;
        if (isFirst)
        {
          disconnect.first = i;
          isFirst = false;
        }
        else
          disconnect.second = i;
      }
    }
    return valid;
  }

  bool DynamicReachabilityMap::checkPathUsePlanningScene(
      const Eigen::MatrixXd &solution)
  {
    bool valid = true;
    for (int i = 0; i < solution.rows(); i++)
    {
      for (int j = 0; j < dimension_; j++)
        ps_->getCurrentStateNonConst().setVariablePosition(joint_names_[j],
            solution(i, j));
      ps_->getCurrentStateNonConst().update(true);
      if (!ps_->isStateValid(ps_->getCurrentState())) return false;
    }
    return valid;
  }

  int DynamicReachabilityMap::getIndex(const geometry_msgs::Point & point)
  {
    if (point.x < bounds_[0](0) || point.x > bounds_[0](1)
        || point.y < bounds_[1](0) || point.y > bounds_[1](1)
        || point.z < bounds_[2](0) || point.z > bounds_[2](1)) return -1;
    int x = (point.x - bounds_[0](0)) / cell_size_;
    int y = (point.y - bounds_[1](0)) / cell_size_;
    int z = (point.z - bounds_[2](0)) / cell_size_;
    int tcp_index = x + y * cell_cnts_[0] + z * cell_cnts_[0] * cell_cnts_[1];
    return tcp_index < space_size_ ? tcp_index : -1;
  }

  bool DynamicReachabilityMap::meanShiftClustering(double h)
  {
    ROS_WARN_STREAM(
        "max dist tol="<<h<<", sqrt dist tol="<<sqrt(dimension_ * (h * h)));
    sample_occupation_.resize(sample_size_);
    for (int i = 0; i < sample_size_; i++)
    {
      sample_occupation_[i].clear();
    }
    for (int i = 0; i < space_size_; i++)
      for (int j = 0; j < space_[i].occupiedSmaples.size(); j++)
      {
        sample_occupation_[space_[i].occupiedSmaples[j]].push_back(i);
      }
    std::vector<Cluster> clusters;
    meanShiftCell(1549, h, clusters);

    for (int i = 0; i < clusters.size(); i++)
    {
      std::map<int, int> tmp;
      for (int j = 0; j < clusters[i].samples.size(); j++)
      {
        for (int k = 0; k < sample_occupation_[clusters[i].samples[j]].size();
            k++)
          if (tmp.find(sample_occupation_[clusters[i].samples[j]][k])
              == tmp.end()) tmp[sample_occupation_[clusters[i].samples[j]][k]] =
              sample_occupation_[clusters[i].samples[j]][k];
      }
    }
    return true;
  }

  bool DynamicReachabilityMap::meanShiftCell(int index, double h,
      std::vector<Cluster> &clusters)
  {
    std::map<int, int> open_samples;
    int tmp_size = tcps_[index].samples.size();
    for (int i = 0; i < tmp_size; i++)
      open_samples[i] = tcps_[index].samples[i];
    double tol = 1e-3 * h;

    std::vector<Cluster> tmp_clusters;
    int sample_cnt = tmp_size;
    std::vector<std::vector<int>> cluster_votes;

    Eigen::VectorXd old_mean(dimension_);
    while (sample_cnt > 0)
    {
      ROS_INFO_STREAM("Remaining samples "<<sample_cnt);
      int tmp_index, start_index;
      while (true)
      {
        tmp_index = rand() % tmp_size;
        if (open_samples.find(tmp_index) == open_samples.end()) continue;
        start_index = open_samples.find(tmp_index)->second;
        break;
      }
      Eigen::VectorXd my_mean = samples_[start_index].q;
      std::vector<std::pair<int, int>> myMembers;
      std::pair<int, int> self(tmp_index, start_index);
      myMembers.push_back(self);
      std::vector<int> this_vote(tmp_size);
      int iteration = 0;
      while (iteration < 200)
      {
        std::vector<int> candidates;
        for (std::map<int, int>::iterator it = open_samples.begin();
            it != open_samples.end(); it++)
        {
          double max_dist =
              (my_mean - samples_[it->second].q).cwiseAbs().maxCoeff();
//        double sqr_dist = (my_mean - samples_[it->second].q).norm();
          if (max_dist < h)
          {
            candidates.push_back(it->second);
            bool found = false;
            for (int i = 0; i < myMembers.size(); i++)
              if (myMembers[i].second == it->second)
              {
                found = true;
                break;
              }
            if (!found)
            {
              std::pair<int, int> tmp_pair(it->first, it->second);
              myMembers.push_back(tmp_pair);
            }
          }
        }

        bool small_cluster = false;
        if (candidates.size() > 0)
        {
          old_mean = my_mean;
          my_mean.setZero();
          for (int i = 0; i < candidates.size(); i++)
            my_mean += samples_[candidates[i]].q;
          my_mean /= candidates.size();
        }
        else
        {
          small_cluster = true;
        }

        if (small_cluster || (my_mean - old_mean).norm() < tol)
        {
          for (int i = 0; i < myMembers.size(); i++)
            open_samples.erase(open_samples.at(myMembers[i].first));
          int merge_index = -1;
          for (int i = 0; i < tmp_clusters.size(); i++)
          {
            double dist_to_other = (my_mean - tmp_clusters[i].center).norm();
            if (dist_to_other < h / 2.0)
            {
              merge_index = i;
              break;
            }
          }

          if (merge_index != -1)
          {
            tmp_clusters[merge_index].center = (tmp_clusters[merge_index].center
                + my_mean) / 2;
            for (int i = 0; i < myMembers.size(); i++)
              tmp_clusters[merge_index].samples.push_back(myMembers[i].second);
          }
          else
          {
            Cluster new_cluster;
            new_cluster.center = my_mean;
            for (int i = 0; i < myMembers.size(); i++)
              new_cluster.samples.push_back(myMembers[i].second);
            tmp_clusters.push_back(new_cluster);
            std::pair<int, bool> tmp(myMembers.size(), false);
          }
          break;
        }
        iteration++;
      }

      sample_cnt = open_samples.size();
    }

    ROS_INFO_STREAM(
        "Remove small clusters from "<<tmp_clusters.size()<<" clusters");
    std::vector<int> big_index;
    for (int i = 0; i < tmp_clusters.size(); ++i)
      if (tmp_clusters[i].samples.size() >= 20) big_index.push_back(i);
    clusters.resize(big_index.size());

    sample_cnt = 0;
    for (int i = 0; i < clusters.size(); i++)
    {
      clusters[i] = tmp_clusters[big_index[i]];

      ROS_INFO_STREAM(
          "Cluster "<<i<<" has "<<clusters[i].samples.size()<<" samples");
      sample_cnt += clusters[i].samples.size();
    }
    ROS_INFO_STREAM(
        clusters.size()<<" clusters remained, with "<<sample_cnt<<" samples");
    return true;
  }

  bool DynamicReachabilityMap::saveSpace(const std::string & path,
      int multiThreads)
  {
    struct stat st = { 0 };
    std::string folderpath = path + "/" + ps_->getRobotModel()->getName() + "_"
        + std::to_string(samples_.size()) + "_" + std::to_string(cell_size_);
    if (stat(folderpath.c_str(), &st) == -1
        && mkdir(folderpath.c_str(), 0700) != 0)
    {
      ROS_ERROR("Cannot create directory");
      return false;
    }
    ROS_INFO("Saving samples");
    std::ofstream samples_file;
    samples_file.open(folderpath + "/samples.txt");
    for (int i = 0; i < samples_.size(); i++)
    {
      samples_file << samples_[i].tcp.position.x << " "
          << samples_[i].tcp.position.y << " " << samples_[i].tcp.position.z
          << " " << samples_[i].tcp.orientation.x << " "
          << samples_[i].tcp.orientation.y << " "
          << samples_[i].tcp.orientation.z << " "
          << samples_[i].tcp.orientation.w << " ";
      samples_file << samples_[i].tcpCell << " ";
      for (int j = 0; j < dimension_; j++)
        samples_file << samples_[i].q(j) << " ";
      samples_file << std::endl;
    }
    samples_file.close();

    ROS_INFO("Saving space");
    std::ofstream space_file;
    space_file.open(folderpath + "/space.txt");
    for (int i = 0; i < space_size_; i++)
    {
      space_file << space_[i].centre.x << " " << space_[i].centre.y << " "
          << space_[i].centre.z << " " << space_[i].space_index(0) << " "
          << space_[i].space_index(1) << " " << space_[i].space_index(2)
          << std::endl;
    }
    space_file.close();

    ROS_INFO("Saving space occupation");
    mkdir((folderpath + "/space_occupation").c_str(), 0700);
    int tmp_space_size = space_size_ / multiThreads;
    for (int t = 0; t < multiThreads; t++)
    {
      std::ofstream space_occupied_file;
      space_occupied_file.open(
          folderpath + "/space_occupation/space_occupation_" + std::to_string(t)
              + ".txt");
      for (int i = t * tmp_space_size; i < (t + 1) * tmp_space_size; i++)
      {
        ROS_INFO_STREAM_THROTTLE(2,
            "Space "<<i<<" / "<<space_size_<<"("<<(double)i/(double)space_size_*100.0<<"%).");
        int tmp_size = space_[i].occupiedSmaples.size();
        if (tmp_size < sample_size_)
        {
          for (int j = 0; j < tmp_size; j++)
            space_occupied_file << space_[i].occupiedSmaples[j] << " ";
          space_occupied_file << std::endl;
        }
        else
        {
          space_occupied_file << -1 << std::endl;
        }
      }
      space_occupied_file.close();
    }
    ROS_INFO("Saving TCP");
    std::ofstream tcp_file;
    tcp_file.open(folderpath + "/tcp.txt");
    for (int i = 0; i < space_size_; i++)
    {
      for (int j = 0; j < tcps_[i].samples.size(); j++)
        tcp_file << tcps_[i].samples[j] << " ";
      tcp_file << std::endl;
    }
    tcp_file.close();

    ROS_INFO("Saving information");
    std::ofstream info_file;
    info_file.open(folderpath + "/info.txt");
    for (int i = 0; i < bounds_.size(); i++)
      info_file << bounds_[i](0) << " " << bounds_[i](1) << " ";
    info_file << std::endl;
    info_file << space_size_ << std::endl;
    info_file << sample_size_ << std::endl;
    info_file << cell_size_ << std::endl;
    info_file << dimension_ << std::endl;
    info_file << group_->getName() << std::endl;
    for (int i = 0; i < joint_names_.size(); i++)
      info_file << joint_names_[i] << " ";
    info_file << std::endl << multiThreads << std::endl;
    info_file.close();

    ROS_INFO_STREAM("Save complete, all information is save to "<<folderpath);
    return true;
  }

  bool DynamicReachabilityMap::loadSpace(const std::string & path)
  {
    DIR *dp;
    struct dirent *dirp;
    if ((dp = opendir(path.c_str())) == NULL)
    {
      ROS_ERROR_STREAM("Result directory "<<path<<" does not exist");
      return false;
    }
    std::map<std::string, bool> files;
    files["samples.txt"] = false;
    files["space.txt"] = false;
    files["tcp.txt"] = false;
    files["info.txt"] = false;
    while ((dirp = readdir(dp)) != NULL)
    {
      if (files.find(dirp->d_name) != files.end()) files.at(dirp->d_name) =
          true;
    }
    for (auto &it : files)
      if (!it.second)
      {
        ROS_ERROR_STREAM("File "<<it.first<<" does not exist");
        return false;
      }

///	Loading the information
    std::ifstream info_file;
    info_file.open(path + "/info.txt");
    std::string line;

//	bounds
    getline(info_file, line);
    std::vector<std::string> bounds = getStringVector(line);
    bounds_.resize(3);
    bounds_[0](0) = stod(bounds[0]);
    bounds_[0](1) = stod(bounds[1]);
    bounds_[1](0) = stod(bounds[2]);
    bounds_[1](1) = stod(bounds[3]);
    bounds_[2](0) = stod(bounds[4]);
    bounds_[2](1) = stod(bounds[5]);

//	Space size
    getline(info_file, line);
    std::vector<std::string> space_size = getStringVector(line);
    space_size_ = stoi(space_size[0]);
    space_.resize(space_size_);
    density_.resize(space_size_);

//	Sample size
    getline(info_file, line);
    std::vector<std::string> sample_size = getStringVector(line);
    sample_size_ = stoi(sample_size[0]);
    samples_.resize(sample_size_);

//	Cell size
    getline(info_file, line);
    std::vector<std::string> cell_size = getStringVector(line);
    cell_size_ = stod(cell_size[0]);
    cell_cnts_(0) = (bounds_[0](1) - bounds_[0](0)) / cell_size_ + 1;
    cell_cnts_(1) = (bounds_[1](1) - bounds_[1](0)) / cell_size_ + 1;
    cell_cnts_(2) = (bounds_[2](1) - bounds_[2](0)) / cell_size_ + 1;

//	Dimension
    getline(info_file, line);
    std::vector<std::string> dim_size = getStringVector(line);
    dimension_ = stoi(dim_size[0]);

// Group name
    getline(info_file, line);
    std::vector<std::string> group_name = getStringVector(line);
    group_ = ps_->getRobotModel()->getJointModelGroup(group_name[0]);

//  Joint names
    getline(info_file, line);
    std::vector<std::string> joint_names = getStringVector(line);
    if (joint_names.size() != dimension_
        || joint_names.size() != group_->getVariableCount())
      ROS_ERROR_STREAM(
          "Invalid space information, group "<<group_->getName()<<" has "<<group_->getVariableCount()<<" variables, but only "<<joint_names.size()<<" are provided");
    joint_names_.resize(dimension_);
    joint_names_ = joint_names;

//  Threads
    getline(info_file, line);
    std::vector<std::string> threads = getStringVector(line);
    int thread_cnt = stoi(threads[0]);
    if (thread_cnt <= 0)
    {
      ROS_ERROR("Thread number does not match");
      return false;
    }
    std::map<std::string, bool> occup_files;
    for (int t = 0; t < thread_cnt; t++)
      occup_files["space_occupation_" + std::to_string(t) + ".txt"] = false;
    DIR *occup_dp;
    if ((occup_dp = opendir((path + "/space_occupation").c_str())) == NULL)
    {
      ROS_ERROR_STREAM(
          "Result directory "<<path<<"/space_occupation does not exist");
      return false;
    }
    while ((dirp = readdir(occup_dp)) != NULL)
    {
      if (occup_files.find(dirp->d_name) != occup_files.end())
        occup_files.at(dirp->d_name) = true;
    }
    for (auto &it : occup_files)
      if (!it.second)
      {
        ROS_ERROR_STREAM("File "<<it.first<<" does not exist");
        return false;
      }

    ROS_INFO_STREAM(
        "Space size = "<<space_size_<<", sample size = "<<sample_size_<<", cell size = "<<cell_size_<<", dimension = "<<dimension_);

    ROS_INFO("Loading samples");
///	Loading the samples
    std::ifstream samples_file;
    samples_file.open(path + "/samples.txt");
    for (int i = 0; i < sample_size_; i++)
    {
      samples_[i].valid = true;
      samples_[i].q.resize(dimension_);
      getline(samples_file, line);
      std::vector<std::string> sample = getStringVector(line);
      samples_[i].tcp.position.x = stod(sample[0]);
      samples_[i].tcp.position.y = stod(sample[1]);
      samples_[i].tcp.position.z = stod(sample[2]);
      samples_[i].tcp.orientation.x = stod(sample[3]);
      samples_[i].tcp.orientation.y = stod(sample[4]);
      samples_[i].tcp.orientation.z = stod(sample[5]);
      samples_[i].tcp.orientation.w = stod(sample[6]);
      samples_[i].tcpCell = stoi(sample[7]);
      for (int j = 0; j < dimension_; j++)
      {
        samples_[i].q(j) = stod(sample[j + 8]);
      }
    }

    ROS_INFO("Loading space");
///	Loading the space
    std::ifstream space_file;
    space_file.open(path + "/space.txt");
    for (int i = 0; i < space_size_; i++)
    {
      getline(space_file, line);
      std::vector<std::string> cell = getStringVector(line);
      space_[i].index = i;
      space_[i].centre.x = stod(cell[0]);
      space_[i].centre.y = stod(cell[1]);
      space_[i].centre.z = stod(cell[2]);
      space_[i].space_index(0) = stod(cell[3]);
      space_[i].space_index(1) = stod(cell[4]);
      space_[i].space_index(2) = stod(cell[5]);
    }

    ROS_INFO("Loading space occupation");
///	Loading occupation
    MultiThreadsSpaceOccupationLoader loader(path + "/space_occupation",
        thread_cnt, space_size_, sample_size_);
    loader.loadSpaceOccupation();
    int tmp_space_size = space_size_ / thread_cnt;
    for (int t = 0; t < thread_cnt; t++)
      for (int i = 0; i < tmp_space_size; i++)
        space_[t * tmp_space_size + i].occupiedSmaples =
            loader.space_occup_[t][i];
//  std::ifstream space_occupied_file;
//  space_occupied_file.open(path + "/space_occupation.txt");
//  for (int i = 0; i < space_size_; i++) {
//    ROS_INFO_STREAM_THROTTLE(
//        2,
//        "Loading space occupation "<<i<<" / "<<space_size_<<"("<<(double)i/(double)space_size_*100.0<<"%).");
//    getline(space_occupied_file, line);
//    std::vector<std::string> occup = getStringVector(line);
//    int mode = stoi(occup[0]);
//    if (mode == 1) {
//      space_[i].occupiedSmaples.resize(occup.size() - 1);
//      for (int j = 0; j < occup.size() - 1; j++)
//        space_[i].occupiedSmaples[j] = stoi(occup[j + 1]);
//    } else if (mode == -1) {
//      space_[i].occupiedSmaples.resize(sample_size_);
//      for (int s = 0; s < sample_size_; s++) {
//        space_[i].occupiedSmaples[s] = s;
//      }
//    } else {
//      ROS_ERROR("Unknown occupation mode");
//    }
//  }

    ROS_INFO("Loading TCP");
///	Loading TCP
    std::ifstream tcp_file;
    tcp_file.open(path + "/tcp.txt");
    tcps_.resize(space_size_);
    for (int i = 0; i < space_size_; i++)
    {
      tcps_[i].index = i;
      getline(tcp_file, line);
      std::vector<std::string> samples = getStringVector(line);
      tcps_[i].samples.resize(samples.size());
      for (int j = 0; j < samples.size(); j++)
        tcps_[i].samples[j] = stoi(samples[j]);
    }

    ROS_INFO_STREAM(
        "Loaded Indexed Space created with "<<space_size_<<" cells, "<<sample_size_<<" samples.");
    return true;
  }

  bool DynamicReachabilityMap::createCellURDF(const std::string & path)
  {
    struct stat st = { 0 };
    if (stat(path.c_str(), &st) == -1 && mkdir(path.c_str(), 0700) != 0)
    {
      ROS_ERROR("Cannot create directory");
      return false;
    }
    ROS_INFO("Creating DynamicReachabilityMapCell URDF");
    std::ofstream urdf;
    urdf.open(path + "/DynamicReachabilityMap.urdf");
    urdf << "<?xml version=\"1.0\" ?>" << std::endl;
    urdf << "<robot name=\"IndexedCell\">" << std::endl;
    urdf << "<material name=\"Black\">" << std::endl;
    urdf << "\t<color rgba=\"0 0 0 .5\"/>" << std::endl;
    urdf << "</material>" << std::endl;
    urdf << "<link name=\"base\">" << std::endl;
    urdf << "</link>" << std::endl;
    for (int i = 0; i < space_size_; i++)
    {
      urdf << "<link name=\"cell_" << i << "\">" << std::endl;
      urdf << "\t<visual>" << std::endl;
      urdf << "\t\t<origin rpy=\"0 0 0\" xyz=\"" << space_[i].centre.x << " "
          << space_[i].centre.y << " " << space_[i].centre.z << "\"/>"
          << std::endl;
      urdf << "\t\t<geometry>" << std::endl;
      urdf << "\t\t\t<box size=\"" << cell_size_ << " " << cell_size_ << " "
          << cell_size_ << "\"/>" << std::endl;
      urdf << "\t\t</geometry>" << std::endl;
      urdf << "\t\t<material name=\"Black\"/>" << std::endl;
      urdf << "\t</visual>" << std::endl;

      urdf << "\t<collision>" << std::endl;
      urdf << "\t\t<origin rpy=\"0 0 0\" xyz=\"" << space_[i].centre.x << " "
          << space_[i].centre.y << " " << space_[i].centre.z << "\"/>"
          << std::endl;
      urdf << "\t\t<geometry>" << std::endl;
      urdf << "\t\t\t<box size=\"" << cell_size_ << " " << cell_size_ << " "
          << cell_size_ << "\"/>" << std::endl;
      urdf << "\t\t</geometry>" << std::endl;
      urdf << "\t</collision>" << std::endl;
      urdf << "</link>" << std::endl;
    }
    for (int i = 0; i < space_size_; i++)
    {
      urdf << "<joint name=\"joint_" << i << "\" type=\"fixed\">" << std::endl;
      urdf << "\t<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>" << std::endl;
      urdf << "\t<parent link=\"base\"/>" << std::endl;
      urdf << "\t<child link=\"cell_" << i << "\"/>" << std::endl;
      urdf << "</joint>" << std::endl;
    }
    urdf << "</robot>" << std::endl;
    return true;
  }

  void MultiThreadsSpaceOccupationLoader::load(int id, int size)
  {
    std::ifstream space_occupied_file;
    space_occupied_file.open(
        path_ + "/space_occupation_" + std::to_string(id) + ".txt");
    std::stringstream buffer;
    std::string line;
    buffer << space_occupied_file.rdbuf();
    space_occup_[id].resize(space_size_);
    for (int i = 0; i < space_size_; i++)
    {
      if (id == 0 && i % (space_size_ / 1000) == 0)
      {
        std::cout << "Load space " << i << " / " << space_size_ << "("
            << (double) i / (double) sample_size_ * 1000.0 << "%).\r";
        std::cout.flush();
      }

      getline(buffer, line);
      std::vector<std::string> occup = getStringVector(line);
      if (occup.size() > 0)
      {
        if (occup[0] != "-1")
        {
          space_occup_[id][i].resize(occup.size() - 1);
          for (int j = 0; j < occup.size() - 1; j++)
            space_occup_[id][i][j] = stoi(occup[j + 1]);
        }
        else
        {
          space_occup_[id][i].resize(sample_size_);
          for (int s = 0; s < sample_size_; s++)
            space_occup_[id][i][s] = s;
        }
      }
    }
  }
}


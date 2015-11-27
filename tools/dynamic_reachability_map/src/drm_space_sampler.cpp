/*
 * drm_space_sampler.cpp
 *
 *  Created on: 27 Aug 2015
 *      Author: yiming
 */
#include "dynamic_reachability_map/DRMSpace.h"
#include "dynamic_reachability_map/DRMSampler.h"
#include "dynamic_reachability_map/DRMSpaceSaver.h"
#include "dynamic_reachability_map/DRMSpaceLoader.h"
#include "dynamic_reachability_map/DRMFullBodySampler.h"
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <ros/package.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");
  std::string eff, group_name;
  if (!nh.getParam("EndEffector", eff))
  {
    ROS_ERROR("EndEffector link not specified [rosparam: EndEffector]");
    return 0;
  }
  if (!nh.getParam("Group", group_name))
  {
    ROS_ERROR("Group name not specified [rosparam: Group]");
    return 0;
  }

  std::string bounds_string;
  if (!nh.getParam("SpaceBounds", bounds_string))
  {
    ROS_ERROR("Space bounds not specified [rosparam: SpaceBounds]");
    return 0;
  }
  std::istringstream stm(bounds_string);
  std::vector<Eigen::Vector2d> bounds(3);

  try
  {
    stm >> bounds[0](0) >> bounds[0](1) >> bounds[1](0) >> bounds[1](1)
        >> bounds[2](0) >> bounds[2](1);
  } catch (int e)
  {
    ROS_ERROR("Get space bounds failed");
    return 0;
  }

  double cell_size = 0;
  if (!nh.getParam("CellSize", cell_size))
  {
    ROS_ERROR("Space cell size not specified [rosparam: CellSize]");
    return 0;
  }

  int thread_cnt;
  if (!nh.getParam("ThreadsNumber", thread_cnt))
  {
    ROS_WARN("Threads number not specified [rosparam: ThreadsNumber]");
    ROS_WARN("Sampling time can be greatly reduced by using multi threads");
    thread_cnt = 1;
  }

  int sample_cnt;
  if (!nh.getParam("SampleSize", sample_cnt))
  {
    ROS_ERROR("Sample size not specified [rosparam: SampleSize]");
    return 0;
  }

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr model = robot_model_loader.getModel();
  if (!model)
  {
    ROS_ERROR("Robot model not loaded");
    return 0;
  }

  bool check_occupation = false;
  if (nh.hasParam("CheckOccupation"))
  {
    nh.getParam("CheckOccupation", check_occupation);
  }

  dynamic_reachability_map::SpaceBounds space_bounds;
  space_bounds.x_low = bounds[0](0);
  space_bounds.x_upper = bounds[0](1);
  space_bounds.y_low = bounds[1](0);
  space_bounds.y_upper = bounds[1](1);
  space_bounds.z_low = bounds[2](0);
  space_bounds.z_upper = bounds[2](1);
  dynamic_reachability_map::DRMSpace_ptr space;
  space.reset(new dynamic_reachability_map::DRMSpace());
  if (!space->createSpace(space_bounds, cell_size, model, eff, group_name))
  {
    ROS_ERROR("Can not create DRM space");
    return 0;
  }
  dynamic_reachability_map::DRMFullBodySampler drms(thread_cnt);
  if(!drms.initialise(
      "/home/yiming/devel/ipab-distro/drc/software/models/val_description/urdf/valkyrie_A_sim_drake.urdf"))
  {
    ROS_ERROR("DRMS initialization failed");
    return 0;
  }
//  int points_cnt = 4;
//  int dim = space->getDimension();
//  std::vector<std::string> var_names = space->getGroup()->getVariableNames();
//  std::vector<std::vector<double>> points(dim);
//  std::vector<int> current(dim);
//  std::vector<std::vector<double>> samples(std::pow(points_cnt, dim));
//
//  for (int i = 0; i < dim; i++)
//  {
//    double max = model->getVariableBounds(var_names[i]).max_position_;
//    double min = model->getVariableBounds(var_names[i]).min_position_;
//    double delta = (max - min) / (points_cnt + 1);
//    points[i].resize(points_cnt);
//    for (int j = 0; j < points_cnt; j++)
//      points[i][j] = min + (j + 1) * delta;
//    current[i] = 0;
//  }
//
//  unsigned long int cnt = 0;
//  for (cnt = 0; cnt < samples.size(); cnt++)
//  {
//    samples[cnt].resize(dim);
//    for (int i = 0; i < dim; i++)
//    {
//      samples[cnt][i] = points[i][current[i]];
//    }
//
//    current[0]++;
//    for (int i = 0; i < dim - 1; ++i)
//    {
//      if (current[i] == points_cnt)
//      {
//        current[i] = 0;
//        current[i + 1]++;
//      }
//    }
//  }
  std::vector<std::vector<double>> samples;
  ROS_INFO_STREAM("Sample size ="<<samples.size()<<" cnt="<<sample_cnt);

  drms.startSampling(space, sample_cnt, samples);
  std::ofstream density_file_;
  ros::Time time = ros::Time::now();
  std::string path = ros::package::getPath("dynamic_reachability_map")
      + "/result2";
  std::string savepath;
  dynamic_reachability_map::DRMSpaceSaver saver;
  saver.saveSpace(path, space, savepath);

  return 0;
  ROS_INFO("Load space");
  space.reset(new dynamic_reachability_map::DRMSpace());
  dynamic_reachability_map::DRMSpaceLoader drml;
  if (!drml.loadSpace(savepath, space, model))
  {
    ROS_ERROR("Load space failed");
    return 0;
  }
  else
    ROS_INFO_STREAM(
        "DRMSpace loaded! Space size "<<space->getSpaceSize()<<", samples size "<<space->getSampleSize());

  ros::Publisher drm_pub = nh.advertise<visualization_msgs::Marker>(
      "DynamicReachabilityMap", 10);
  ros::Publisher occup_pub = nh.advertise<visualization_msgs::Marker>(
      "OccupationMap", 10);
  visualization_msgs::Marker drm_mark_, occup_mark;
  drm_mark_.type = visualization_msgs::Marker::CUBE_LIST;
  drm_mark_.header.stamp = ros::Time::now();
  drm_mark_.header.frame_id = "world_frame";
  drm_mark_.scale.x = drm_mark_.scale.y = drm_mark_.scale.z =
      space->getResolution();
  drm_mark_.action = visualization_msgs::Marker::ADD;

  occup_mark = drm_mark_;

  std::vector<int> density(space->getSpaceSize());
  std::vector<int> occup(space->getSpaceSize());
  while (ros::ok())
  {
    drm_mark_.header.stamp = ros::Time::now();
    for (int i = 0; i < density.size(); i++)
    {
      density[i] = space->at(i).reach_samples.size();
    }
    int size = density.size();
    Eigen::VectorXd density_eigen(size);
    for (int i = 0; i < size; i++)
      density_eigen(i) = density[i];
    double max = density_eigen.maxCoeff();
    density_eigen = density_eigen / max;

    drm_mark_.colors.clear();
    drm_mark_.points.clear();
    for (int i = 0; i < size; i++)
    {
      if (density[i] > 0)
      {
        std_msgs::ColorRGBA c;
        c.a = density_eigen(i);
        c.r = 1 - density_eigen(i);
        c.g = 1;
        c.b = density_eigen(i);
        drm_mark_.colors.push_back(c);
        drm_mark_.points.push_back(space->at(i).center);
      }
    }
    if (drm_mark_.points.size() == 0)
    ROS_ERROR("Empty DRM!!!!");
    drm_pub.publish(drm_mark_);

    occup_mark.header.stamp = ros::Time::now();
    for (unsigned int i = 0; i < occup.size(); i++)
    {
      occup[i] = space->at(i).occup_samples.size();
    }
    Eigen::VectorXd occup_eigen(size);
    Eigen::VectorXd occup_tmp(size);
    for (int i = 0; i < size; i++)
      occup_tmp(i) = occup[i];
    max = occup_tmp.maxCoeff();
    std::vector<int> max_list;
    for (int i = 0; i < size; i++)
      if (occup_tmp(i) == max)
      {
        occup_tmp(i) = 0;
        max_list.push_back(i);
      }
    max = occup_tmp.maxCoeff();
    for (int i = 0; i < size; i++)
      occup_eigen(i) = occup[i];
    for (int i = 0; i < max_list.size(); i++)
      occup_eigen(max_list[i]) = max + 1;
    max = occup_eigen.maxCoeff();
    occup_eigen = occup_eigen / max;

    occup_mark.colors.clear();
    occup_mark.points.clear();
    for (int i = 0; i < size; i++)
    {
      if (occup[i] > 0)
      {
        std_msgs::ColorRGBA c;
        c.a = occup_eigen(i);
        c.r = 1;
        c.g = 0;
        c.b = 0;
        occup_mark.colors.push_back(c);
        occup_mark.points.push_back(space->at(i).center);
      }
    }
    occup_pub.publish(occup_mark);

    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

//  dynamic_reachability_map::DynamicReachabilityMap drm;
//
//  if (!drm.createSpace(bounds[0], bounds[1], bounds[2], cell_size))
//  {
//    ERROR("Create dynamic reachability map space failed");
//    return 0;
//  }
//
//  if (!drm.createRobotSampler(kinematic_model, eff, group_name))
//  {
//    ERROR("Create robot sample failed");
//    return 0;
//  }
//
//  dynamic_reachability_map::MultiThreadsIndexer *indexing_threads;
//  indexing_threads = new dynamic_reachability_map::MultiThreadsIndexer(drm, thread_cnt, sample_cnt);
//  indexing_threads->indexingSpace();
//  indexing_threads->combineSamples(drm);
//  delete indexing_threads;
//
//  std::ofstream density_file_;
//  ros::Time time = ros::Time::now();
//  std::string path = ros::package::getPath("dynamic_reachability_map") + "/result";
//  ROS_INFO_STREAM("Saving sampling result to "<<path);
  return 0;
}


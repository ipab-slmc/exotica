/*
 * GraphManager.cpp
 *
 *  Created on: 16 Aug 2014
 *      Author: yiming
 */

#include "GraphManager.h"

namespace exotica
{
  GraphManager::GraphManager()
      : initialised_(false), nh_("/MeshGraphManager"), graph_(
          new MeshGraph("MESH_GRAPH"))
  {
    //TODO
  }

  GraphManager::~GraphManager()
  {
    //TODO
  }

  bool GraphManager::initialisation(const exotica::StringListPtr & links,
      const exotica::BoolListPtr & link_types,
      const std::vector<double> & link_radius, const int size)
  {
    if (links->strings.size() == 0 || size < links->strings.size())
    {
      std::cout << "Mesh Graph Manager: wrong size(link size="
          << links->strings.size() << ", size=" << size << ")\n";
      INDICATE_FAILURE
      return false;
    }
    double i_range;
    if (!nh_.getParam("/MeshGraphManager/InteractRange", i_range))
    {
      INDICATE_FAILURE
      return false;
    }

    double eps;
    if (!nh_.getParam("/MeshGraphManager/SafetyThreshold", eps))
    {
      INDICATE_FAILURE
      return false;
    }
    bool dummy_table = false;
    nh_.getParam("/MeshGraphManager/DummyTable", dummy_table);
    std::vector<bool> tmp_types(links->strings.size());
    std::vector<std::string> tmp_links(links->strings.size());
    for (int i = 0; i < links->strings.size(); i++)
    {
      tmp_links[i] = links->strings[i];
      tmp_types[i] = link_types->data[i];
    }
    if (!graph_->initialisation(size, tmp_links, tmp_types, link_radius,
        i_range, eps, dummy_table))
    {
      INDICATE_FAILURE
      return false;
    }
    std::string topic;
    if (!nh_.getParam("/MeshGraphManager/ExternalTopic", topic))
    {
      INDICATE_FAILURE
      return false;
    }
//		ext_sub_ =
//				nh_.subscribe<exotica::MeshVertex>(topic, 10, boost::bind(&exotica::GraphManager::extCallback, this, _1));
    ext_arr_sub_ = nh_.subscribe<exotica::MeshVertexArray>(topic, 10,
        boost::bind(&exotica::GraphManager::extArrCallback, this, _1));
//		vel_timer_ =
//				nh_.createTimer(ros::Duration(0.05), boost::bind(&exotica::GraphManager::velTimeCallback, this, _1));

    ROS_INFO("Mesh Graph Manager has been initialised");
    initialised_ = true;
    return true;
  }

  MeshGraphPtr GraphManager::getGraph()
  {
    return graph_;
  }

  void GraphManager::extCallback(const exotica::MeshVertexConstPtr & ext)
  {
    LOCK(lock_);
    graph_->updateExternal(ext);
  }

  void GraphManager::extArrCallback(
      const exotica::MeshVertexArrayConstPtr & ext)
  {
    LOCK(lock_);
    for (int i = 0; i < ext->vertices.size(); i++)
      graph_->updateExternal(ext->vertices[i]);
  }

  void GraphManager::velTimeCallback(const ros::TimerEvent&)
  {
    graph_->computeVelocity();
  }
}	//namespace exotica

/*
 * GraphManager.h
 *
 *  Created on: 15 Aug 2014
 *      Author: yiming
 */

#ifndef GRAPHMANAGER_H_
#define GRAPHMANAGER_H_

#include <ros/ros.h>
#include <exotica/EXOTica.hpp>
#include "MeshGraph.h"
namespace exotica
{
  /**
   * \brief	The graph manager manages the graph (environment).
   * The manager handles external objects I/O,
   * i.e. receives objects information from outside, processes them and then send out to dmesh.
   */
  class GraphManager
  {
    public:
      /**
       * \brief	Default constructor
       */
      GraphManager();

      /**
       * \brief	Default destructor
       */
      ~GraphManager();

      /**
       * \brief	Initialise the manager
       * @param	links		All the robot links
       * @param	link_types	Link types, real or dummy
       * @param	size		Maximum graph size
       * @return	True if succeeded, false otherwise
       */
      bool initialisation(const exotica::StringListPtr & links,
          const exotica::BoolListPtr & link_types,
          const std::vector<double> & link_radius, const int size);

      /*
       * \brief	Get the graph pointer
       * @return	MeshGraph pointer
       */
      MeshGraphPtr getGraph();

    private:
      //	The mesh graph
      MeshGraphPtr graph_;

      //	Initialisation flag
      bool initialised_;

      //	Scoped locker
      boost::mutex::scoped_lock lock_;

      //	For ROS
      //	External object callback
      void extCallback(const exotica::MeshVertexConstPtr & ext);
      void extArrCallback(const exotica::MeshVertexArrayConstPtr & ext);

      ros::Timer vel_timer_;
      void velTimeCallback(const ros::TimerEvent&);

      //	Private node handle
      ros::NodeHandle nh_;

      //	External object subscriber
      ros::Subscriber ext_sub_;
      ros::Subscriber ext_arr_sub_;
  };
}	//namespace exotica

#endif /* GRAPHMANAGER_H_ */

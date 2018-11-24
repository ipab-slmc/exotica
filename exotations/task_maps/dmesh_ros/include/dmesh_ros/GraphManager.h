/*
 *      Author: Yiming Yang
 * 
 * Copyright (c) 2016, University of Edinburgh
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

#ifndef GRAPHMANAGER_H_
#define GRAPHMANAGER_H_

#include <dmesh_ros/DMeshVertexInitializer.h>
#include <dmesh_ros/MeshGraph.h>
#include <exotica/Exotica.h>
#include <ros/ros.h>

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
    void initialisation(std::vector<DMeshVertexInitializer>& verts, int size);

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

    //	For ROS
    //	External object callback
    void extCallback(const exotica::MeshVertexConstPtr& ext);
    void extArrCallback(const exotica::MeshVertexArrayConstPtr& ext);

    ros::Timer vel_timer_;
    void velTimeCallback(const ros::TimerEvent&);

    //	Private node handle
    ros::NodeHandle nh_;

    //	External object subscriber
    ros::Subscriber ext_sub_;
    ros::Subscriber ext_arr_sub_;
};
}  //namespace exotica

#endif /* GRAPHMANAGER_H_ */

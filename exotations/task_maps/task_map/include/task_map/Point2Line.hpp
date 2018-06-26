/*
 *      Author: Christian Rauch
 *
 * Copyright (c) 2018, University Of Edinburgh
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

#pragma once

#include <exotica/TaskMap.h>
#include <task_map/Point2LineInitializer.h>

namespace exotica
{
class Point2Line : public TaskMap, public Instantiable<Point2LineInitializer>
{
public:
    Point2Line() {}
    virtual ~Point2Line() {}
    virtual void Instantiate(Point2LineInitializer& init);

    virtual void update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi);

    virtual void update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J);

    virtual int taskSpaceDim();

private:
    Eigen::Vector3d line_start;  //<! start point of line in base frame
    Eigen::Vector3d line_end;    //<! end point of line in base frame
    Eigen::Vector3d line;        //<! vector from start to end point of line
    bool infinite;               //<! true: vector from start to end defines the direction of and infinite line
                                 //<! false: start and end define a line segment

    /**
     * @brief direction computes the vector from a point to its projection on a line
     * @param point point in base frame
     * @return 3D vector from #point to its projection on #line
     */
    Eigen::Vector3d direction(const Eigen::Vector3d& point);

    std::string link_name;  //<! frame of defined point
    std::string base_name;  //<! frame of defined line

    ros::Publisher pub_marker;        //<! publish marker for RViz
    ros::Publisher pub_marker_label;  //<! marker label

    bool visualise;
};

typedef std::shared_ptr<Point2Line> Point2Line_Ptr;

}  // namespace exotica

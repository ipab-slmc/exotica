/*
 *      Author: Christopher E. Mower
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

#ifndef EXOTICA_TASKMAP_POINT_TO_POINT_H
#define EXOTICA_TASKMAP_POINT_TO_POINT_H

#include <exotica/TaskMap.h>
#include <task_map/Point2PointInitializer.h>

namespace exotica
{
  class Point2Point : public TaskMap, public Instantiable<Point2PointInitializer>
  {
  public:
  
    Point2Point();
    virtual ~Point2Point() {}
    
    virtual void Instantiate(Point2PointInitializer& init);

    virtual void assignScene(Scene_ptr scene);

    virtual void update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi);

    virtual void update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J);

    virtual int taskSpaceDim();

    void setPoint(Eigen::Vector3d point);
    
    Eigen::Vector3d getPoint();

    void Initialize();
    
  private:

    Eigen::Vector3d point_;

    unsigned int N;

    Scene_ptr scene_;

    Eigen::Vector3d getEffPosition(int i);

    Eigen::Matrix3d getEffRotation(int i);
    Eigen::Matrix3d getEffRotation(double roll, double pitch, double yaw);

    Eigen::Vector3d getApproachVector(int i);
    Eigen::Vector3d getApproachVector(double roll, double pitch, double yaw);

    Eigen::Matrix3d getApproachVectorJacobian(double roll, double pitch, double yaw);


  };

  typedef std::shared_ptr<Point2Point> Point2Point_ptr;

}  // namespace exotica

#endif

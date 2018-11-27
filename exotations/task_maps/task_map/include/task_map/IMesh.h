/*
 *      Author: Vladimir Ivan
 *
 * Copyright (c) 2017, University of Edinburgh
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

#ifndef IMESH_H_
#define IMESH_H_

#include <exotica/KinematicTree.h>
#include <exotica/TaskMap.h>
#include <task_map/IMeshInitializer.h>
#include <visualization_msgs/Marker.h>

namespace exotica
{
class IMesh : public TaskMap, public Instantiable<IMeshInitializer>
{
public:
    IMesh();
    virtual ~IMesh();

    virtual void Instantiate(IMeshInitializer& init);

    virtual void assignScene(Scene_ptr scene);

    virtual void update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi);

    virtual void update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J);

    virtual int taskSpaceDim();

    void setWeight(int i, int j, double weight);
    void setWeights(const Eigen::MatrixXd& weights);
    Eigen::MatrixXd getWeights();

    static Eigen::VectorXd computeLaplace(Eigen::VectorXdRefConst EffPhi, Eigen::MatrixXdRefConst Weights, Eigen::MatrixXd* dist = nullptr, Eigen::VectorXd* wsum = nullptr);

    void computeGoalLaplace(const Eigen::VectorXd& x, Eigen::VectorXd& goal);

    static void computeGoalLaplace(const std::vector<KDL::Frame>& nodes, Eigen::VectorXd& goal, Eigen::MatrixXdRefConst Weights);

    virtual void debug(Eigen::VectorXdRefConst phi);
    void initDebug(std::string ref);
    void destroyDebug();

protected:
    Eigen::MatrixXd weights_;

    int eff_size_ = 0;
    bool Debug;

    ros::Publisher imesh_mark_pub_;
    visualization_msgs::Marker imesh_mark_;
};
typedef std::shared_ptr<IMesh> IMesh_Ptr;
}

#endif /* IMESH_H_ */

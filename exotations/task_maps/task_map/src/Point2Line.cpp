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

#include "task_map/Point2Line.hpp"

REGISTER_TASKMAP_TYPE("Point2Line", exotica::Point2Line);

namespace exotica
{
Eigen::Vector3d Point2Line::distance(const Eigen::Vector3d &point, const Eigen::Vector3d &line, const bool infinite, const bool dbg)
{
    // http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
    // let:
    //      s: start of line
    //      e: end of line
    //      p: point
    // then the point on vector v = e-s that is closest to p is vp = s + t*(e-s) with
    // t = ((s-p)*(e-s)) / (|e-s|^2)    (* denotes the dot product)

    // the line starts at the origin of BaseOffset, hence s=0, v='line', vp=t*'line'
    double t = (-point).dot(line) / line.norm();
    if (dbg) HIGHLIGHT_NAMED("P2L", "t " << t);
    if (!infinite)
    {
        // clip to to range [0,1]
        t = std::min(std::max(0.0, t), 1.0);
        if (dbg) HIGHLIGHT_NAMED("P2L", "|t| " << t);
    }

    // vector from point 'p' to point 'vp' on line
    const Eigen::Vector3d dv = (t * line) - point;
    if (dbg) HIGHLIGHT_NAMED("P2L", "dv " << dv.transpose());
    return dv;
}

Point2Line::Point2Line() {}
void Point2Line::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != Kinematics[0].Phi.rows()) throw_named("Wrong size of phi!");

    for (int i = 0; i < Kinematics[0].Phi.rows(); i++)
    {
        phi(i) = distance(Eigen::Map<const Eigen::Vector3d>(Kinematics[0].Phi(i).p.data), line, infinite, debug_).norm();
    }
}

void Point2Line::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    if (phi.rows() != Kinematics[0].Phi.rows()) throw_named("Wrong size of phi!");
    if (J.rows() != Kinematics[0].J.rows() || J.cols() != Kinematics[0].J(0).data.cols()) throw_named("Wrong size of J! " << Kinematics[0].J(0).data.cols());

    for (int i = 0; i < Kinematics[0].Phi.rows(); i++)
    {
        // direction from point to line
        const Eigen::Vector3d dv = distance(Eigen::Map<const Eigen::Vector3d>(Kinematics[0].Phi(i).p.data), line, infinite, debug_);
        phi(i) = dv.norm();
        for (int j = 0; j < J.cols(); j++)
        {
            J(i, j) = -dv.dot(Eigen::Map<const Eigen::Vector3d>(Kinematics[0].J[i].getColumn(j).vel.data)) / dv.norm();
        }
    }
}

void Point2Line::Instantiate(Point2LineInitializer &init)
{
    line = init.EndPoint.head<3>() - boost::any_cast<Eigen::VectorXd>(init.EndEffector[0].getProperty("BaseOffset")).head<3>();
    infinite = init.infinite;
}

int Point2Line::taskSpaceDim()
{
    return Kinematics[0].Phi.rows();
}
}  // namespace exotica

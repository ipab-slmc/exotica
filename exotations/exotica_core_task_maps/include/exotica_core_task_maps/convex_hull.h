//
// Copyright (c) 2018, University of Edinburgh
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#ifndef EXOTICA_CORE_TASK_MAPS_CONVEXHULL_H_
#define EXOTICA_CORE_TASK_MAPS_CONVEXHULL_H_

#include <Eigen/Dense>

#include <exotica_core/tools.h>

namespace exotica
{
/// \brief DetDiff2D Computes the 2D determinant (analogous to a 2D cross product) of a two vectors defined by P_1P_2 and P_1P.
double DetDiff2D(Eigen::VectorXdRefConst p1, Eigen::VectorXdRefConst p2, Eigen::VectorXdRefConst p)
{
    return (p(1) - p1(1)) * (p2(0) - p1(0)) - (p2(1) - p1(1)) * (p(0) - p1(0));
}

std::list<int> QuickHull(Eigen::MatrixXdRefConst points, std::list<int>& half_points, int p1, int p2)
{
    int ind = -1;
    double max_dist = 0;
    std::list<int> new_half_points;
    for (const int& i : half_points)
    {
        double d = DetDiff2D(points.row(p1).transpose(), points.row(p2).transpose(), points.row(i).transpose());
        if (d >= 0.0)
        {
            new_half_points.push_back(i);
        }
        if (d > max_dist)
        {
            ind = i;
            max_dist = d;
        }
    }

    std::list<int> hull;
    if (ind == -1)
    {
        hull.push_back(p2);
    }
    else
    {
        hull.splice(hull.begin(), QuickHull(points, new_half_points, p1, ind));
        hull.splice(hull.end(), QuickHull(points, new_half_points, ind, p2));
    }

    return hull;
}

std::list<int> ConvexHull2D(Eigen::MatrixXdRefConst points)
{
    if (points.cols() != 2) ThrowPretty("Input must contain 2D points!");

    int n = points.rows();

    std::list<int> hull;
    std::list<int> half_points;
    if (n < 3)
    {
        for (int i = 0; i < n; ++i) hull.push_back(i);
    }
    else
    {
        int min_x = 0, max_x = 0;
        half_points.push_back(0);
        for (int i = 1; i < n; ++i)
        {
            if (points(i, 0) < points(min_x, 0))
                min_x = i;
            if (points(i, 0) > points(max_x, 0))
                max_x = i;
            half_points.push_back(i);
        }
        hull.splice(hull.begin(), QuickHull(points, half_points, min_x, max_x));
        hull.splice(hull.end(), QuickHull(points, half_points, max_x, min_x));
    }

    return hull;
}
}

#endif  // EXOTICA_CORE_TASK_MAPS_CONVEXHULL_H_

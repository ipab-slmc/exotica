#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include "exotica/Tools.h"
#include "Eigen/Dense"

namespace exotica
{

double lineDist2D(Eigen::VectorXdRefConst p1, Eigen::VectorXdRefConst p2, Eigen::VectorXdRefConst p)
{
    return (p(1) - p1(1)) * (p2(0) - p1(0)) -
           (p2(1) - p1(1)) * (p(0) - p1(0));
}


std::list<int> quickHull(Eigen::MatrixXdRefConst points, std::list<int>& halfPoints, int p1, int p2)
{
    int ind = -1;
    double max_dist = 0;
    std::list<int> newHalfPoints;
    for (int i : halfPoints)
    {
        double d = lineDist2D(points.row(p1), points.row(p2), points.row(i));
        if (d>0.0)
        {
            newHalfPoints.push_back(i);
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
        hull.splice(hull.begin(), quickHull(points, newHalfPoints, p1, ind));
        hull.splice(hull.end(), quickHull(points, newHalfPoints, ind, p2));
    }
    return hull;
}

std::list<int> convexHull2D(Eigen::MatrixXdRefConst points)
{
    if (points.cols()!=2) throw_pretty("Input must contain 2D points!");

    int n = points.rows();

    std::list<int> hull;
    std::list<int> halfPoints;

    if (n < 3)
    {
        for(int i=0; i<n; i++) hull.push_back(i);
    }
    else
    {
        int min_x = 0, max_x = 0;
        halfPoints.push_back(0);
        for (int i=1; i<n; i++)
        {
            if (points(i,0) < points(min_x,0))
                min_x = i;
            if (points(i,0) > points(max_x,0))
                max_x = i;
            halfPoints.push_back(i);
        }
        hull.splice(hull.begin(), quickHull(points, halfPoints, min_x, max_x));
        hull.splice(hull.end(), quickHull(points, halfPoints, max_x, min_x));
    }
    return hull;
}



}

#endif // CONVEXHULL_H

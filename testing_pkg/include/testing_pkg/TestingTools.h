#ifndef TESTING_TESTING_TOOLS_H
#define TESTING_TESTING_TOOLS_H

#include <Eigen/Eigen>
#include <boost/filesystem.hpp>
#include <cmath>
#include <ros/package.h>

#define EPSILON     0.00001
#define TOLERANCE   0.0002
#define TOLERANCE_L 0.00001

#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __func__
#endif

#define CHECK_OK std::cout << "\nOK @ " << __PRETTY_FUNCTION__ << "===" << __FILE__ << "===" << __LINE__ << "\n" << std::endl
#define CHECK_OK_ std::cout << "\nOK @ " << __PRETTY_FUNCTION__ << "===" << __FILE__ << "===" << __LINE__

/**
 * \brief Function for comparing two Eigen::Vectors
 * @param vec_1 The first vector
 * @param vec_2 The second vector
 * @param prec  The precision at which we want to fail
 * @return      True if equal, false if something is wrong
 */
inline bool compareVectors(Eigen::VectorXdRefConst vec_1,
    Eigen::VectorXdRefConst vec_2, double prec = TOLERANCE)
{
  if (vec_1.size() != vec_2.size())
  {
    return false;
  }  //!< If not equal size then definitely not equal
  for (int i = 0; i < vec_1.size(); i++)
  {
    if (std::abs(vec_1(i) - vec_2(i)) > prec)
    {
      return false;
    }
  }
  return true;
}
;

inline bool compareMatrices(const Eigen::MatrixXd & mat_1,
    const Eigen::MatrixXd & mat_2, double prec = TOLERANCE)
{
  if (mat_1.rows() != mat_2.rows() or mat_1.cols() != mat_2.cols())
  {
    return false;
  }
  for (int i = 0; i < mat_1.rows(); i++)
  {
    for (int j = 0; j < mat_1.cols(); j++)
    {
      if (std::abs(mat_1(i, j) - mat_2(i, j)) > prec)
      {
        return false;
      }
    }
  }
  return true;
}
;

inline bool findPackagePath(const std::string & name, std::string & path)
{
  path = ros::package::getPath(name);
  return not (path.empty()); //!< Indicate if ok or not...
}
;
#endif

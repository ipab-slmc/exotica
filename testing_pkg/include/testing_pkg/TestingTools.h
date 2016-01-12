/*
 *      Author: Michael Camilleri
 * 
 * Copyright (c) 2016, University Of Edinburgh 
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

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

#ifndef EXOTICA_TOOLS_H
#define EXOTICA_TOOLS_H

#include <string>
#include <Eigen/Dense>
#include <kdl/tree.hpp>

#include <exotica/Tools/Exception.h>
#include <exotica/Tools/Uncopyable.h>
#include <exotica/Tools/Printable.h>
#include <exotica/Tools/Conversions.h>
#include <exotica/Version.h>
#include <std_msgs/ColorRGBA.h>

/**
 * \brief A convenience macro for the boost scoped lock
 */
#define LOCK(x) boost::mutex::scoped_lock(x)

/**
 * \brief A double-wrapper MACRO functionality for generating unique object names: The actual functionality is provided by EX_UNIQ (for 'exotica unique')
 */
#define EX_CONC(x, y) x ## y
#define EX_UNIQ(x, y) EX_CONC(x, y)

namespace exotica
{

   /**
   * @brief randomColor Generates random opaque color.
   * @return Random color
   */
  std_msgs::ColorRGBA randomColor();

  /**
   * @brief loadOBJ Loads mesh data from an OBJ file
   * @param file_name File name
   * @param tri Returned vertex indices of triangles
   * @param vert Vertex positions
   */
  void loadOBJ(std::string & data, Eigen::VectorXi& tri,
      Eigen::VectorXd& vert);

  void saveMatrix(std::string file_name,
      const Eigen::Ref<const Eigen::MatrixXd> mat);

  void getText(std::string& txt, KDL::Frame& ret);
}
#endif

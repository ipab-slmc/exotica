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
#include "tinyxml2/tinyxml2.h"
#include <Eigen/Dense>
#include <kdl/tree.hpp>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "rapidjson/document.h"
#include <exotica/MeshVertex.h>
#include <exotica/StringList.h>
#include <exotica/BoolList.h>
#include <exotica/Vector.h>
#include <exotica/Matrix.h>
#include <exotica/Tools/Exception.h>
#include <exotica/Tools/Uncopyable.h>

/**
 * \brief A set of debugging tools: basically these provide easy ways of checking code execution through std::cout prints
 */
#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __func__
#endif

#ifdef EXOTICA_DEBUG_MODE
#define CHECK_EXECUTION         std::cout << "\033[1;32m[EXOTica]:\033[0m Ok in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\n"; //!< With endline
#define INDICATE_FAILURE        std::cerr << "\033[1;32m[EXOTica]:\033[0m \033[1;31mFailed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\033[0m\n";//!< With endline
#define WARNING(x)				std::cout << "\033[1;32m[EXOTica]:\033[0m \033[33mWarning in " << __PRETTY_FUNCTION__ << ": " << x << "\033[0m\n";//!< With endline
#define ERROR(x)				std::cerr << "\033[1;32m[EXOTica]:\033[0m \033[1;31mFailed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\n" << x << "\033[0m\n";//!< With endline
#define INFO(x)					std::clog << "\033[1;32m[EXOTica]:\033[0m Info in " << __PRETTY_FUNCTION__ << ": " << x << "\n";//!< With endline
#else
#define CHECK_EXECUTION         // No operation
#define INDICATE_FAILURE        std::cerr << "\033[1;32m[EXOTica]:\033[0m \033[1;31mFailed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\033[0m\n";//!< With endline
#define WARNING(x)        std::cout << "\033[1;32m[EXOTica]:\033[0m \033[33mWarning in " << __PRETTY_FUNCTION__ << ": " << x << "\033[0m\n";//!< With endline
#define ERROR(x)                std::cerr << "\033[1;32m[EXOTica]:\033[0m \033[1;31mFailed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\n" << x << "\033[0m\n";//!< With endline
#define INFO(x)
#endif
#define HIGHLIGHT(x)			std::cout << "\033[1;32m[EXOTica]:\033[0m \033[36m" << x << "\033[0m\n";
#define HIGHLIGHT_NAMED(name, x)std::cout << "\033[1;32m[EXOTica]:\033[0m \033[35m[" << name <<"]\033[0m \033[36m" << x << "\033[0m\n";
#define WARNING_NAMED(name, x)	std::cout << "\033[1;32m[EXOTica]:\033[0m \033[35m[" << name <<"]\033[0m \033[33m" << x << "\033[0m\n";
#define INFO_NAMED(name, x)		std::cout << "\033[1;32m[EXOTica]:\033[0m \033[35m[" << name <<"]\033[0m " << x << "\n";
namespace Eigen
{

  /// \brief Convenience wrapper for storing references to sub-matrices/vectors
  template<typename Derived>
  class Ref_ptr: public boost::shared_ptr<Ref<Derived> >
  {
    public:
      inline Ref_ptr()
          : boost::shared_ptr<Ref<Derived> >()
      {

      }

      inline Ref_ptr(const Eigen::Block<Derived>& other)
      {
        this->reset(new Ref<Derived>(other));
      }

      inline Ref_ptr(Eigen::Block<Derived>& other)
      {
        this->reset(new Ref<Derived>(other));
      }

      inline Ref_ptr(const Eigen::VectorBlock<Derived>& other)
      {
        this->reset(new Ref<Derived>(other));
      }

      inline Ref_ptr(Eigen::VectorBlock<Derived>& other)
      {
        this->reset(new Ref<Derived>(other));
      }

      inline Ref_ptr(Derived& other)
      {
        this->reset(new Ref<Derived>(other));
      }

      inline Ref_ptr(const Derived& other)
      {
        this->reset(new Ref<Derived>(other));
      }
  };

  /// \brief Reference to sub-vector.
  typedef Ref_ptr<VectorXd> VectorXdRef_ptr;
  /// \brief Reference to sub-Matrix.
  typedef Ref_ptr<MatrixXd> MatrixXdRef_ptr;

  typedef Ref<VectorXd> VectorXdRef;
  typedef const Ref<const VectorXd>& VectorXdRefConst;
  typedef Ref<MatrixXd> MatrixXdRef;
  typedef const Ref<const MatrixXd>& MatrixXdRefConst;
}

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
   * \brief Enum for error reporting throughout the library
   */

  /**
   * \brief	Enum for termination criterion
   */
  enum ETerminate
  {
    TERMINATE = 0, CONTINUE = 1
  };


  /**
   * \brief Parses an XML element into an Eigen Matrix. The handle must point directly to the element with the matrix as its text child and must have no comments!
   * @param xml_matrix    The element for the XML matrix
   * @param eigen_matrix  Placeholder for storing the parsed matrix
   * @return              Indication of success: TODO
   */
  void getMatrix(const tinyxml2::XMLElement & xml_matrix,
      Eigen::MatrixXd & eigen_matrix);

  /**
   * \brief Parses an XML element into an Eigen Vector. The handle must point directly to the element with the vector as its text child and must have no comments!
   * @param xml_matrix    The element for the XML Vector
   * @param eigen_matrix  Placeholder for storing the parsed vector
   * @return              Indication of success: TODO
   */
  void getVector(const tinyxml2::XMLElement & xml_vector,
      Eigen::VectorXd & eigen_vector);
  void getStdVector(const tinyxml2::XMLElement & xml_vector,
      std::vector<double> & std_vector);
  void getBoolVector(const tinyxml2::XMLElement & xml_vector,
      std::vector<bool> & bool_vector);

  /**
   * \brief Get boolean
   */
  void getBool(const tinyxml2::XMLElement & xml_vector, bool & val);

  /**
   * \brief Parses an XML element into an float (for convenience)
   * @param xml_value    The element containing the numerical value as text child
   * @param value  Placeholder for storing the parsed double
   * @return              Indication of success: TODO
   */
  void getDouble(const tinyxml2::XMLElement & xml_value, double & value);

  /**
   * \brief Parses an XML element into an float (for convenience)
   * @param xml_value    The element containing the numerical value as text child
   * @param value  Placeholder for storing the parsed integer
   * @return              Indication of success: TODO
   */
  void getInt(const tinyxml2::XMLElement & xml_value, int & value);

  /**
   * \brief Parses an XML element into a vector of string
   * @param xml_value    The element containing the numerical value as text child
   * @param value  Placeholder for storing the parsed list
   * @return              Indication of success: TODO
   */
  void getList(const tinyxml2::XMLElement & xml_value,
      std::vector<std::string> & value);

  /**
   * \brief Removes all characters after the last forward slash (/): intended to be used to get a parent directory from a file-path
   * @param file_path[inout]  The complete file-path to be processed: the returned string contains no trailing forward slashes...
   */
  void resolveParent(std::string & file_path);

  /**
   * \brief Utility function for including files:
   *        Iterates through all children looking for include tags, recursively calling itself until leaves are reached
   * @param handle    Handle to the current element: will modify the document it is associated with...
   * @param directory The current working directory
   * @return          Indication of success TODO
   */
  void parseIncludes(tinyxml2::XMLHandle & handle, std::string directory);

  /**
   * \brief Utility function for copying a complete sub-tree from one document to another
   * @param parent[inout] The parent-to-be, where the sub-tree will be copied
   * @param child[in]     The next child to add to this parent...
   */
  void deepCopy(tinyxml2::XMLHandle & parent, tinyxml2::XMLHandle & child);

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

  void getJSON(const rapidjson::Value& a, Eigen::VectorXd& ret);
  void getJSON(const rapidjson::Value& a, double& ret);
  void getJSON(const rapidjson::Value& a, int& ret);
  void getJSON(const rapidjson::Value& a, bool& ret);
  void getJSON(const rapidjson::Value& a, std::string& ret);
  void getJSON(const rapidjson::Value& a, KDL::Frame& ret);
  void getJSON(const rapidjson::Value& a, std::vector<std::string>& ret);
  void getJSONFrameNdArray(const rapidjson::Value& a, KDL::Frame& ret);

  void getText(std::string& txt, KDL::Frame& ret);

  void vectorExoticaToEigen(const exotica::Vector & exotica,
      Eigen::VectorXd & eigen);
  void vectorEigenToExotica(Eigen::VectorXd eigen,
      exotica::Vector & exotica);
  void matrixExoticaToEigen(const exotica::Matrix & exotica,
      Eigen::MatrixXd & eigen);
  void matrixEigenToExotica(const Eigen::MatrixXd & eigen,
      exotica::Matrix & exotica);
}
#endif

/*
 *      Author: Yiming Yang
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

#ifndef IMESH_H_
#define IMESH_H_

#include <exotica/EXOTica.hpp>
#include <boost/thread/mutex.hpp>

namespace exotica
{
  /**
   * @brief	Implementation of Interaction Mesh Task Map
   */
  class IMesh: public TaskMap
  {
    public:
      /**
       * @brief	Default constructor
       */
      IMesh();

      /**
       * @brief	Destructor
       */
      virtual ~IMesh();

      /**
       * @brief	Concrete implementation of update method
       * @param	x	Joint space configuration
       */
      virtual EReturn update(Eigen::VectorXdRefConst x, const int t);

      virtual EReturn initialiseManual(std::string name, Server_ptr & server,
          const Scene_map & scene_ptr, boost::shared_ptr<PlanningProblem> prob,
          std::vector<std::pair<std::string,std::string> >& params);

      /**
       * @brief	Get the task space dimension
       * @return	Exotica return type, SUCCESS if succeeded
       */
      virtual EReturn taskSpaceDim(int & task_dim);

      /**
       * @brief	Set edge weight(s)
       * @param	i,j		Vertices i, j
       * @param	weight	Edge weight
       * @param	weights	Weight matrix
       * @return	Exotica return type
       */
      EReturn setWeight(int i, int j, double weight);
      EReturn setWeights(const Eigen::MatrixXd & weights);

      /**
       * @brief	Compute laplace coordinates of a vertices set
       * @param	V		3xN matrix of vertices positions
       * @param	wsum	Array of weight normalisers (out put)
       * @param	dist	Triangular matrix of distances between vertices (out put)
       * @return	3xN Laplace coordinates
       */
      EReturn computeLaplace(int t);

      EReturn computeGoalLaplace(const Eigen::VectorXd &x, Eigen::VectorXd &goal);

      virtual void debug();
      void initDebug(std::string ref);
      void destroyDebug();
    protected:
      /**
       * @brief	Concrete implementation of initialisation from xml
       * @param	handle	XML handler
       */
      virtual EReturn initDerived(tinyxml2::XMLHandle & handle);

      /** Member Functions **/

      /**
       * @brief	Compute Jacobian of the laplace coordinates with respect to the joint angle space
       * @param	q	Joint angles
       * @return	Jacobian matrix
       */
      exotica::EReturn computeIMesh(int t);

      /**
       * @brief	Update newest vertices status
       * @param	q	Robot joint configuration
       * @return	Exotica return type
       */
      EReturn updateVertices();
      /** Member Variables **/
      boost::mutex locker_;	//!<Thread locker
      bool initialised_;	//!< Initialisation flag

      /** Interaction Mesh Variables **/
      Eigen::MatrixXd weights_;	//!< Weighting matrix, currently set to ones

      Eigen::MatrixXd dist;
      Eigen::VectorXd wsum;

      int eff_size_;

      ros::Publisher imesh_mark_pub_;
      visualization_msgs::Marker imesh_mark_;
  };
  typedef boost::shared_ptr<IMesh> IMesh_Ptr;  //!< Task Map smart pointer
}

#endif /* IMESH_H_ */

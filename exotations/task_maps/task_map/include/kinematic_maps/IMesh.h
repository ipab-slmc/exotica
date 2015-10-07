/*
 * IMesh.h
 *
 *  Created on: 18 Mar 2014
 *      Author: yimingyang
 */

#ifndef IMESH_H_
#define IMESH_H_

#include <exotica/TaskMap.h>
#include <exotica/Factory.h>
#include <exotica/Test.h>
#include <tinyxml2/tinyxml2.h>
#include <kinematica/KinematicTree.h>
#include <Eigen/Eigen>
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
  };
}

#endif /* IMESH_H_ */

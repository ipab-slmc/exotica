/*
 * DMesh.h
 *
 *  Created on: 17 Jul 2014
 *      Author: yiming
 */

#ifndef DMESH_H_
#define DMESH_H_

#include <exotica/TaskMap.h>
#include <exotica/Factory.h>
#include <exotica/Test.h>
#include <exotica_msgs/MeshVertexArray.h>
#include <tinyxml2/tinyxml2.h>
#include <Eigen/Dense>
#include <boost/thread/mutex.hpp>

namespace exotica
{
	enum EXT_TYPE
	{
		OBSTACLE = 0, GOAL = 1, IGNORE = 255
	}
	;
	/**
	 * \brief	Implementation of distance based interaction mesh
	 */
	class DMesh: public TaskMap
	{
		public:
			/**
			 * \brief	Default constructor
			 */
			DMesh();

			/**
			 * \brief	Default destructor
			 */
			~DMesh();

			/**
			 * @brief	Concrete implementation of update method
			 * @param	x	Joint space configuration
			 */
            virtual EReturn update(const Eigen::VectorXd & x, const int t);

			/**
			 * @brief	Get the task space dimension
			 * @return	Exotica return type, SUCCESS if succeeded
			 */
			virtual EReturn taskSpaceDim(int & task_dim);

			/**
			 * @brief	Compute the laplace for goals
			 * @param	V		3xN matrix of vertices positions
			 */
			EReturn computeGoalLaplace(const Eigen::MatrixXd & V);
			EReturn computeGoalLaplace(const exotica_msgs::MeshVertexArray & V);

			/**
			 * @brief	Get Laplace coordinates
			 * @param	lap	Laplace Verctor
			 */
			EReturn getLaplace(Eigen::VectorXd & lap);

			/**
			 * @brief	Update external objects
			 * @param	ext	External objects
			 */
			void updateExternal(const exotica_msgs::MeshVertexArray & ext);
		protected:
			/**
			 * @brief	Concrete implementation of initialisation from xml
			 * @param	handle	XML handler
			 */
			virtual EReturn initDerived(tinyxml2::XMLHandle & handle);

		private:
			/**
			 * @brief	Compute laplace coordinates of a vertices set
			 * @param	V		3xN matrix of vertices positions
			 */
			EReturn computeLaplace(const Eigen::MatrixXd & V);
			/**
			 * @brief	Compute Jacobian of the laplace coordinates with respect to the joint angle space
			 * @param	q	Joint angles
			 * @return	Jacobian matrix
			 */
			EReturn computeJacobian(const Eigen::VectorXd & q);

			/**
			 * @brief	Update newest graph status
			 * @param	q	Robot joint configuration
			 * @return	Exotica return type
			 */
			EReturn updateGraph();

			boost::mutex locker_;	//!<Thread locker
			bool initialised_;	//!< Initialisation flag

			exotica_msgs::MeshVertexArray meshes_;
			Eigen::MatrixXd vertices_;	//!< Vertex positions
			Eigen::VectorXd laplace_;	//!< Laplace coordinates
			Eigen::MatrixXd dist_;
			Eigen::MatrixXd weights_;
			double gain_;
			double scale_;
			int eff_size_;
			int max_size_;
			int task_size_;
			double safe_range_;
			Eigen::MatrixXd J;
	};
} // namespace exotica

#endif /* DMESH_H_ */

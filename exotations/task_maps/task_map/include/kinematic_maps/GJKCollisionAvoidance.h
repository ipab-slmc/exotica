/*
 * GJKCollisionAvoidance.h
 *
 *  Created on: 7 Jul 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_TASK_MAPS_TASK_MAP_INCLUDE_KINEMATIC_MAPS_GJKCOLLISIONAVOIDANCE_H_
#define EXOTICA_EXOTATIONS_TASK_MAPS_TASK_MAP_INCLUDE_KINEMATIC_MAPS_GJKCOLLISIONAVOIDANCE_H_

#include "CollisionAvoidance.h"
namespace exotica
{
	/*
	 * \brief	This class uses GilbertJohnson-Keerthi algorithm(GJK) algorithms to calculate the
	 * 			distance information between two objects
	 * 			A fast procedure for computing the distance between complex objects in three-dimensional space
	 * 			http://graphics.stanford.edu/courses/cs448b-00-winter/papers/gilbert.pdf
	 */
	class GJKCollisionAvoidance: public CollisionAvoidance
	{
		public:
			GJKCollisionAvoidance();
			virtual ~GJKCollisionAvoidance();

			///	We only need to change the update function, other functions state the same as in collision avoidance
			virtual EReturn update(Eigen::VectorXdRefConst x, const int t);
	};
}



#endif /* EXOTICA_EXOTATIONS_TASK_MAPS_TASK_MAP_INCLUDE_KINEMATIC_MAPS_GJKCOLLISIONAVOIDANCE_H_ */

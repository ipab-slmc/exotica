/*
 * VertexGeometry.h
 *
 *  Created on: 10 Sep 2014
 *      Author: yiming
 */

#ifndef VERTEXGEOMETRY_H_
#define VERTEXGEOMETRY_H_

namespace exotica
{
  enum VERTEX_GEO_TYPE
  {
    UNKNOWN = 0, SPHERE = 10, CAPSULE = 20, PLANE = 30
  };
//	\brief	Implementation of graph vertex geometry properties
  class VertexGeometry
  {
    public:
      /**
       * \brief	Constructor
       */
      VertexGeometry();

      /**
       * \brief	Destructor
       */
      ~VertexGeometry();

      /**
       * \brief	Initialise to sphere
       * @param	r		Radius
       */
      bool initSphere(const double & r);

      /**
       * \brief	Initialise to capsule
       * @param	l		Height of the cylindrical part
       * @param	r		Radius
       */
      bool initCapsule(const double & l, const double & r);

      /**
       * \brief	Initialise to plane
       *
       */

  };
}

#endif /* VERTEXGEOMETRY_H_ */

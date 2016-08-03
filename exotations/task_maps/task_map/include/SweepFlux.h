/*
 *      Author: Vladimir Ivan
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

#ifndef SWEEPFLUX_H
#define SWEEPFLUX_H

#include <exotica/TaskMap.h>
#include <exotica/Factory.h>
#include <tinyxml2/tinyxml2.h>
#include <exotica/KinematicTree.h>
#include <Eigen/Eigen>
#include <boost/thread/mutex.hpp>
#include "SweepFlux.h"
#include <visualization_msgs/Marker.h>
#include <task_map/SweepFluxInitializer.h>

#define ID(x,a,b) {if(!(x>=a&&x<a+b)) ERROR("Out of bounds: "<<x <<" ("<<a<<" "<<b<<")");}

namespace exotica
{
  /**
   * @brief	Implementation of Flux Measure of swept area
   */
  class SweepFlux: public TaskMap, public Instantiable<SweepFluxInitializer>
  {
    public:
      /**
       * @brief	Default constructor
       */
      SweepFlux();

      virtual void Instantiate(SweepFluxInitializer& init);

      /**
       * @brief	Destructor
       */
      ~SweepFlux();

      /**
       * @brief	Concrete implementation of update method
       * @param	x	Joint space configuration
       */
      virtual void update(Eigen::VectorXdRefConst x, const int t);

      /**
       * @brief	Get the task space dimension
       */
      virtual void taskSpaceDim(int & task_dim);

      /**
       * @brief setTimeSteps Sets number of timesteps and allocates memory
       * @param T Number of time steps (this should be set by the planning problem)
       * @return Returns success.
       */
      virtual void setTimeSteps(const int T);

      /**
       * @brief initVis Initialises visualisation
       * @param topic Topic name
       */
      void initVis(ros::NodeHandle nh, std::string topic);

      /**
       * @brief doVis Publish the visualisation marker for displaying the meshes
       */
      void doVis();

      void transform(Eigen::Affine3d& val);

    protected:
      /**
       * @brief	Concrete implementation of initialisation from xml
       * @param	handle	XML handler
       */
      virtual void initDerived(tinyxml2::XMLHandle & handle);

    private:
      /** Member Variables **/
      boost::mutex locker_;	//!<Thread locker
      bool initialised_, init_int_, init_vis_;	//!< Initialisation flag
      int T_; //!< Number of time steps

      Eigen::VectorXi TrisQ_;
      Eigen::VectorXd VertsQ_orig_;
      Eigen::VectorXd VertsQ_;
      Eigen::VectorXd FluxQ_;
      Eigen::VectorXd Verts_;
      Eigen::VectorXi Tris_;
      Eigen::VectorXd TriFlux_;
      Eigen::MatrixXd VertJ_;
      int TrisStride_;

      std::string obj_file_;
      EParam<geometry_msgs::PoseStamped> obj_pose_;
      Eigen::Affine3d qTransform_;
      bool capTop_, capBottom_, capEnds_;

      ros::Publisher vis_pub_;
      ros::NodeHandle nh_;

      /**
       * @brief	Compute Jacobian of the time flux
       * @param	q	Joint angles
       * @return	Jacobian matrix
       */
      void computeFlux(const int t);

      void FluxTriangleTriangle(int* Tris, double* Verts, double* VertJ,
          int* TrisJ, int* TrisQ, double* VertsQ, double* Flux, double* FluxJ,
          int nTris, int nTrisJ, int n, int nTrisQ, double* FluxQ);

      inline double norm(double* a, double* b)
      {
        return sqrt(
            (a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1])
                + (a[2] - b[2]) * (a[2] - b[2]));
      }
      inline double norm(double* a)
      {
        return sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
      }
      inline double dot(double* a, double* b)
      {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
      }
      inline double dot(double* a, double* b, double* c)
      {
        return (a[0] - b[0]) * c[0] + (a[1] - b[1]) * c[1]
            + (a[2] - b[2]) * c[2];
      }
      inline double dot(double* a, double* b, double* c, double* d)
      {
        return (a[0] - b[0]) * (c[0] - d[0]) + (a[1] - b[1]) * (c[1] - d[1])
            + (a[2] - b[2]) * (c[2] - d[2]);
      }
      inline void cross(double* a, double* b, double* ret)
      {
        ret[0] = a[1] * b[2] - a[2] * b[1];
        ret[1] = a[2] * b[0] - a[0] * b[2];
        ret[2] = a[0] * b[1] - a[1] * b[0];
      }
      inline void cross(double* a, double* b, double* c, double* ret)
      {
        ret[0] = (a[1] - b[1]) * c[2] - (a[2] - b[2]) * c[1];
        ret[1] = (a[2] - b[2]) * c[0] - (a[0] - b[0]) * c[2];
        ret[2] = (a[0] - b[0]) * c[1] - (a[1] - b[1]) * c[0];
      }
      inline void cross(double* a, double* b, double* c, double* d, double* ret)
      {
        ret[0] = (a[1] - b[1]) * (c[2] - d[2]) - (a[2] - b[2]) * (c[1] - d[1]);
        ret[1] = (a[2] - b[2]) * (c[0] - d[0]) - (a[0] - b[0]) * (c[2] - d[2]);
        ret[2] = (a[0] - b[0]) * (c[1] - d[1]) - (a[1] - b[1]) * (c[0] - d[0]);
      }
      inline void cross1(double* a, double* c, double* d, double* ret)
      {
        ret[0] = (a[1]) * (c[2] - d[2]) - (a[2]) * (c[1] - d[1]);
        ret[1] = (a[2]) * (c[0] - d[0]) - (a[0]) * (c[2] - d[2]);
        ret[2] = (a[0]) * (c[1] - d[1]) - (a[1]) * (c[0] - d[0]);
      }

      inline void FluxPointTriangle(double* x, double* a, double* b, double* c,
          double* aJ, double* bJ, double* cJ, double* Flux, double* FluxJ,
          int n)
      {
        double J, K, _J, _K, nax, nbx, ncx, _nax, _nbx, _ncx, dab, dac, dcb,
            tmp[3], tmp1[3], tmp2[3], _a[3], _b[3], _c[3];
        nax = norm(x, a);
        nbx = norm(x, b);
        ncx = norm(x, c);
        dab = dot(x, a, x, b);
        dac = dot(x, a, x, c);
        dcb = dot(x, c, x, b);
        cross(x, a, x, b, tmp);
        J = dot(x, c, tmp);
        K = nax * nbx * ncx + dab * ncx + dac * nbx + dcb * nax;
        if (K * K < 1e-150 || nax < 1e-150 || nbx < 1e-150 || ncx < 1e-150)
        {
          *Flux = 0;
          memset(FluxJ, 0, sizeof(double) * n);
          return;
        }
        *Flux = 2.0 * atan2(J, K);
        for (int j = 0; j < n; j++)
        {
          _a[0] = -aJ[j * 3];
          _a[1] = -aJ[j * 3 + 1];
          _a[2] = -aJ[j * 3 + 2];
          _b[0] = -bJ[j * 3];
          _b[1] = -bJ[j * 3 + 1];
          _b[2] = -bJ[j * 3 + 2];
          _c[0] = -cJ[j * 3];
          _c[1] = -cJ[j * 3 + 1];
          _c[2] = -cJ[j * 3 + 2];
          _nax = dot(x, a, _a) / nax;
          _nbx = dot(x, b, _b) / nbx;
          _ncx = dot(x, c, _c) / ncx;
          cross1(_a, x, b, tmp1);
          cross(x, a, _b, tmp2);
          tmp1[0] += tmp2[0];
          tmp1[1] += tmp2[1];
          tmp1[2] += tmp2[2];
          _J = dot(x, c, tmp1) + dot(tmp, _c);
          _K = _nax * nbx * ncx + nax * _nbx * ncx + nax * nbx * _ncx
              + (dot(x, b, _a) + dot(x, a, _b)) * ncx + dab * _ncx
              + (dot(x, c, _a) + dot(x, a, _c)) * nbx + dac * _nbx
              + (dot(x, b, _c) + dot(x, c, _b)) * nax + dcb * _nax;
          if ((J * J + K * K) < 1e-50)
          {
            FluxJ[j] = 0.0;
            continue;
          }
          else
          {
            FluxJ[j] = 2.0 * (_J * K - J * _K) / (J * J + K * K);
          }
        }
      }
  };

  typedef boost::shared_ptr<exotica::SweepFlux> SweepFlux_ptr;
}
#endif // SWEEPFLUX_H

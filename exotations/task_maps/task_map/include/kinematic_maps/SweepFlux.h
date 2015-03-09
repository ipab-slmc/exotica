#ifndef SWEEPFLUX_H
#define SWEEPFLUX_H

#include <exotica/TaskMap.h>
#include <exotica/Factory.h>
#include <exotica/Test.h>
#include <tinyxml2/tinyxml2.h>
#include <kinematica/KinematicTree.h>
#include <Eigen/Eigen>
#include <boost/thread/mutex.hpp>
#include "kinematic_maps/SweepFlux.h"
#include <visualization_msgs/Marker.h>

#define ID(x,a,b) {if(!(x>=a&&x<a+b)) ERROR("Out of bounds: "<<x <<" ("<<a<<" "<<b<<")");}

namespace exotica
{
    /**
     * @brief	Implementation of Flux Measure of swept area
     */
    class SweepFlux : public TaskMap
    {
    public:
        /**
         * @brief	Default constructor
         */
        SweepFlux();

        /**
         * @brief	Destructor
         */
        ~SweepFlux();

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
         * @brief setTimeSteps Sets number of timesteps and allocates memory
         * @param T Number of time steps (this should be set by the planning problem)
         * @return Returns success.
         */
        virtual EReturn setTimeSteps(const int T);

        /**
         * @brief initVis Initialises visualisation
         * @param topic Topic name
         */
        void initVis(ros::NodeHandle nh ,std::string topic);

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
        virtual EReturn initDerived(tinyxml2::XMLHandle & handle);

    private:
        /** Member Variables **/
        boost::mutex locker_;	//!<Thread locker
        bool initialised_, init_int_, init_vis_;	//!< Initialisation flag
        int T_; //!< Number of time steps

        Eigen::MatrixXd J_;
        Eigen::VectorXd y_;
        Eigen::VectorXi TrisQ_;
        Eigen::VectorXd VertsQ_orig_;
        Eigen::VectorXd VertsQ_;
        Eigen::VectorXd FluxQ_;
        Eigen::VectorXd Verts_;
        Eigen::VectorXi Tris_;
        Eigen::VectorXd TriFlux_;
        Eigen::MatrixXd VertJ_;
        int TrisStride_;

        EParam<std::string> obj_file_;
        EParam<geometry_msgs::PoseStamped> obj_pose_;
        Eigen::Affine3d qTransform_;
        EParam<std_msgs::Bool> capTop_, capBottom_,capEnds_;

        ros::Publisher vis_pub_;
        ros::NodeHandle nh_;


        /**
         * @brief	Compute Jacobian of the time flux
         * @param	q	Joint angles
         * @return	Jacobian matrix
         */
        exotica::EReturn computeJacobian(const Eigen::VectorXd & q, const int t);
        //const Eigen::Ref<const Eigen::VectorXd> &
        //const Eigen::Ref<const Eigen::MatrixXd> &
        //Eigen::Ref<Eigen::MatrixXd>
        //Eigen::Ref<Eigen::VectorXd>

        /**
         * @brief FluxTriangleTriangle Computes flux between a set of field generating triangles (TisQ) and a set of flux measuring surface triangles (Tris)
         * @param Tris Index matrix of measurment triangles (Mx3)
         * @param Verts Positions of vertices of measurement triangles (3K)
         * @param VertJ Jacobians of vertex positions of measurement triangles (3KxN)
         * @param TrisQ Index matrix of field generating tringles (Jx3)
         * @param VertsQ Positions of vertices of field generating triangles (Ix3)
         * @param Flux Returned Flux value (1)
         * @param FluxJ Returned Flux Jacobian (1xN)
         */
        void FluxTriangleTriangle(const Eigen::Ref<const Eigen::VectorXi> & Tris,
                                  const Eigen::Ref<const Eigen::VectorXd> & Verts,
                                  const Eigen::Ref<const Eigen::MatrixXd> & VertJ,
                                  const Eigen::Ref<const Eigen::VectorXi> & TrisJ,
                                  const Eigen::Ref<const Eigen::VectorXi> & TrisQ,
                                  const Eigen::Ref<const Eigen::VectorXd> & VertsQ,
                                  Eigen::Ref<Eigen::VectorXd> Flux,
                                  Eigen::Ref<Eigen::MatrixXd> FluxJ);

        void FluxTriangleTriangle(int* Tris,
                                  double* Verts,
                                  double* VertJ,
                                  int* TrisJ,
                                  int* TrisQ,
                                  double* VertsQ,
                                  double* Flux,
                                  double* FluxJ,
                                  int nTris,
                                  int nTrisJ,
                                  int n,
                                  int nTrisQ,
                                  double* FluxQ);

        /**
         * @brief FluxPointTriangle Computes flux and its Jacobian between point x and triangle abc.
         * @param x Measurement point position
         * @param a Triangle vertex position
         * @param b Triangle vertex position
         * @param c Triangle vertex position
         * @param aJ Triangle vertex Jacobian
         * @param bJ Triangle vertex Jacobian
         * @param cJ Triangle vertex Jacobian
         * @param Flux Returned flux value
         * @param FluxJ Returned Flux Jacobian
         */
        void FluxPointTriangle(const Eigen::Ref<const Eigen::Vector3d> & x,
                               const Eigen::Ref<const Eigen::Vector3d> & a,
                               const Eigen::Ref<const Eigen::Vector3d> & b,
                               const Eigen::Ref<const Eigen::Vector3d> & c,
                               const Eigen::Ref<const Eigen::MatrixXd> & aJ,
                               const Eigen::Ref<const Eigen::MatrixXd> & bJ,
                               const Eigen::Ref<const Eigen::MatrixXd> & cJ,
                               double& Flux,
                               Eigen::Ref<Eigen::MatrixXd> FluxJ);

        inline double norm(double* a, double* b)
        {
            return sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1])+(a[2]-b[2])*(a[2]-b[2]));
        }
        inline double norm(double* a)
        {
            return sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
        }
        inline double dot(double* a, double* b)
        {
            return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
        }
        inline double dot(double* a, double* b, double* c)
        {
            return (a[0]-b[0])*c[0]+(a[1]-b[1])*c[1]+(a[2]-b[2])*c[2];
        }
        inline double dot(double* a, double* b, double* c, double* d)
        {
            return (a[0]-b[0])*(c[0]-d[0])+(a[1]-b[1])*(c[1]-d[1])+(a[2]-b[2])*(c[2]-d[2]);
        }
        inline void cross(double* a, double* b, double* ret)
        {
            ret[0] = a[1]*b[2]-a[2]*b[1];
            ret[1] = a[2]*b[0]-a[0]*b[2];
            ret[2] = a[0]*b[1]-a[1]*b[0];
        }
        inline void cross(double* a, double* b, double* c,double* ret)
        {
            ret[0] = (a[1]-b[1])*c[2]-(a[2]-b[2])*c[1];
            ret[1] = (a[2]-b[2])*c[0]-(a[0]-b[0])*c[2];
            ret[2] = (a[0]-b[0])*c[1]-(a[1]-b[1])*c[0];
        }
        inline void cross(double* a, double* b, double* c, double* d, double* ret)
        {
            ret[0] = (a[1]-b[1])*(c[2]-d[2])-(a[2]-b[2])*(c[1]-d[1]);
            ret[1] = (a[2]-b[2])*(c[0]-d[0])-(a[0]-b[0])*(c[2]-d[2]);
            ret[2] = (a[0]-b[0])*(c[1]-d[1])-(a[1]-b[1])*(c[0]-d[0]);
        }
        inline void cross1(double* a, double* c, double* d, double* ret)
        {
            ret[0] = (a[1])*(c[2]-d[2])-(a[2])*(c[1]-d[1]);
            ret[1] = (a[2])*(c[0]-d[0])-(a[0])*(c[2]-d[2]);
            ret[2] = (a[0])*(c[1]-d[1])-(a[1])*(c[0]-d[0]);
        }

        inline void FluxPointTriangle(double* x,
                               double* a,
                               double* b,
                               double* c,
                               double* aJ,
                               double* bJ,
                               double* cJ,
                               double* Flux,
                               double* FluxJ,
                               int n)
        {
            double J,K,_J,_K,nax,nbx,ncx,_nax,_nbx,_ncx,dab,dac,dcb,tmp[3], tmp1[3],tmp2[3], _a[3],_b[3],_c[3];
            nax=norm(x,a);
            nbx=norm(x,b);
            ncx=norm(x,c);
            dab=dot(x,a,x,b);
            dac=dot(x,a,x,c);
            dcb=dot(x,c,x,b);
            cross(x,a,x,b,tmp);
            J=dot(x,c,tmp);
            K=nax*nbx*ncx+dab*ncx+dac*nbx+dcb*nax;
            if(K*K<1e-150 || nax<1e-150 || nbx<1e-150 || ncx<1e-150)
            {
                *Flux=0;
                memset(FluxJ,0,sizeof(double)*n);
                return;
            }
            *Flux = 2.0*atan2(J,K);
            for (int j=0;j<n;j++)
            {
                _a[0]=-aJ[j*3];_a[1]=-aJ[j*3+1];_a[2]=-aJ[j*3+2];
                _b[0]=-bJ[j*3];_b[1]=-bJ[j*3+1];_b[2]=-bJ[j*3+2];
                _c[0]=-cJ[j*3];_c[1]=-cJ[j*3+1];_c[2]=-cJ[j*3+2];
                _nax=dot(x,a,_a)/nax;
                _nbx=dot(x,b,_b)/nbx;
                _ncx=dot(x,c,_c)/ncx;
                cross1(_a,x,b,tmp1);
                cross(x,a,_b,tmp2);
                tmp1[0]+=tmp2[0];
                tmp1[1]+=tmp2[1];
                tmp1[2]+=tmp2[2];
                _J=dot(x,c,tmp1)+dot(tmp,_c);
                _K=_nax*nbx*ncx+nax*_nbx*ncx+nax*nbx*_ncx
                        +(dot(x,b,_a)+dot(x,a,_b))*ncx+dab*_ncx
                        +(dot(x,c,_a)+dot(x,a,_c))*nbx+dac*_nbx
                        +(dot(x,b,_c)+dot(x,c,_b))*nax+dcb*_nax;
                if((J*J+K*K)<1e-50)
                {
                    FluxJ[j]=0.0;
                    continue;
                }
                else
                {
                    FluxJ[j]=2.0*(_J*K-J*_K)/(J*J+K*K);
                }
            }
        }
    };

    typedef boost::shared_ptr<exotica::SweepFlux> SweepFlux_ptr;
}
#endif // SWEEPFLUX_H

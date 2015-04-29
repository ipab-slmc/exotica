#include "kinematic_maps/Orientation.h"

#define P1 i*4*3
#define P2 i*4*3+3
#define P3 i*4*3+6
#define P4 i*4*3+9
#define X
#define Y +1
#define Z +2

REGISTER_TASKMAP_TYPE("Orientation", exotica::Orientation);

namespace exotica
{
    Orientation::Orientation()
    {
        //!< Empty constructor
    }

    EReturn Orientation::update(Eigen::VectorXdRefConst x, const int t)
    {
        if(!isRegistered(t)||!getEffReferences()) {INDICATE_FAILURE; return FAILURE;}
        JAC.setZero();
        for (int i = 0; i < PHI.rows(); i++)
        {
            PHI(i) = sqrt(
                      (EFFPHI(P2 X) - EFFPHI(P1 X) + EFFPHI(P4 X) - EFFPHI(P3 X))
                    * (EFFPHI(P2 X) - EFFPHI(P1 X) + EFFPHI(P4 X) - EFFPHI(P3 X))
                    + (EFFPHI(P2 Y) - EFFPHI(P1 Y) + EFFPHI(P4 Y) - EFFPHI(P3 Y))
                    * (EFFPHI(P2 Y) - EFFPHI(P1 Y) + EFFPHI(P4 Y) - EFFPHI(P3 Y))
                    + (EFFPHI(P2 Z) - EFFPHI(P1 Z) + EFFPHI(P4 Z) - EFFPHI(P3 Z))
                    * (EFFPHI(P2 Z) - EFFPHI(P1 Z) + EFFPHI(P4 Z) - EFFPHI(P3 Z)) );

            if (updateJacobian_ && PHI(i) > 1e-50)
            {
                for (int j = 0; j < JAC.cols(); j++)
                {
                    JAC(i, j) =
                            (  (EFFPHI(P2 X) - EFFPHI(P1 X) + EFFPHI(P4 X) - EFFPHI(P3 X))
                              * (EFFJAC(P2 X, j) - EFFJAC(P1 X, j) + EFFJAC(P4 X, j) - EFFJAC(P3 X, j))
                             + (EFFPHI(P2 Y) - EFFPHI(P1 Y) + EFFPHI(P4 Y) - EFFPHI(P3 Y))
                              * (EFFJAC(P2 Y, j) - EFFJAC(P1 Y, j) + EFFJAC(P4 Y, j) - EFFJAC(P3 Y, j))
                             + (EFFPHI(P2 Z) - EFFPHI(P1 Z) + EFFPHI(P4 Z) - EFFPHI(P3 Z))
                              * (EFFJAC(P2 Z, j) - EFFJAC(P1 Z, j) + EFFJAC(P4 Z, j) - EFFJAC(P3 Z, j))
                            ) / PHI(i);
                }
            }
        }

        return SUCCESS;
    }

    EReturn Orientation::initDerived(tinyxml2::XMLHandle & handle)
    {
        if (scene_->getMapSize(object_name_) % 4 != 0)
        {
            ERROR("Kinematic scene must have even number of end-effectors!");
            return FAILURE;
        }
        else
        {
            return SUCCESS;
        }
    }

    EReturn Orientation::taskSpaceDim(int & task_dim)
    {
        if (!scene_)
        {
            task_dim = -1;
            ERROR("Kinematic scene has not been initialized!");
            return MMB_NIN;
        }
        else
        {
            task_dim = scene_->getMapSize(object_name_) / 4;
        }
        return SUCCESS;
    }
}

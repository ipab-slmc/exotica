#include "kinematic_maps/Distance.h"

REGISTER_TASKMAP_TYPE("Distance", exotica::Distance);

namespace exotica
{
    Distance::Distance()
    {
        //!< Empty constructor
    }

    EReturn Distance::update(Eigen::VectorXdRefConst x, const int t)
    {
        if(!isRegistered(t)||!getEffReferences()) {INDICATE_FAILURE; return FAILURE;}
        JAC.setZero();
        for (int i = 0; i < PHI.rows(); i++)
        {
            PHI(i) = sqrt((EFFPHI(i * 2 * 3) - EFFPHI(i * 2 * 3 + 3))
                    * (EFFPHI(i * 2 * 3) - EFFPHI(i * 2 * 3 + 3))
                    + (EFFPHI(i * 2 * 3 + 1) - EFFPHI(i * 2 * 3 + 4))
                            * (EFFPHI(i * 2 * 3 + 1) - EFFPHI(i * 2 * 3 + 4))
                    + (EFFPHI(i * 2 * 3 + 2) - EFFPHI(i * 2 * 3 + 5))
                            * (EFFPHI(i * 2 * 3 + 2) - EFFPHI(i * 2 * 3 + 5)));

            if (updateJacobian_ && PHI(i) > 1e-50)
            {
                for (int j = 0; j < JAC.cols(); j++)
                {
                    JAC(i, j) = ((EFFPHI(i * 2 * 3) - EFFPHI(i * 2 * 3 + 3))
                            * (EFFJAC(i * 2 * 3, j) - EFFJAC(i * 2 * 3 + 3, j))
                            + (EFFPHI(i * 2 * 3 + 1) - EFFPHI(i * 2 * 3 + 4))
                                    * (EFFJAC(i * 2 * 3 + 1, j) - EFFJAC(i * 2 * 3 + 4, j))
                            + (EFFPHI(i * 2 * 3 + 2) - EFFPHI(i * 2 * 3 + 5))
                                    * (EFFJAC(i * 2 * 3 + 2, j) - EFFJAC(i * 2 * 3 + 5, j)))
                            / PHI(i);
                }
            }
        }


        return SUCCESS;
    }

    EReturn Distance::initDerived(tinyxml2::XMLHandle & handle)
    {
        if (scene_->getMapSize(object_name_) % 2 != 0)
        {
            ERROR("Kinematic scene must have even number of end-effectors!");
            return FAILURE;
        }
        else
        {
            return SUCCESS;
        }
    }

    EReturn Distance::taskSpaceDim(int & task_dim)
    {
        if (!scene_)
        {
            task_dim = -1;
            ERROR("Kinematic scene has not been initialized!");
            return MMB_NIN;
        }
        else
        {
            task_dim = scene_->getMapSize(object_name_) / 2;
        }
        return SUCCESS;
    }
}

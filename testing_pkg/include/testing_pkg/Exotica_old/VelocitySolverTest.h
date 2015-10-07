#ifndef TESTING_VELOCITY_SOLVER_TEST_H
#define TESTING_VELOCITY_SOLVER_TEST_H

#include <exotica/EXOTica.hpp>
#include <gtest/gtest.h>
#include "testing_pkg/TestingTools.h"
#include "testing_pkg/Exotica/VelocitySolverType_1.h"

/**
 * TO TEST
 *  > Initialisation
 *  > Inverse computation
 *  > Velocity makes error less
 */

class ExoticaVelSolverTest: public ::testing::Test
{
  public:

    virtual void SetUp()
    {
      EXPECT_TRUE(exotica::VelocitySolverCreator::Instance()->listImplementations(registered_types_))
          << "Could Not list the registered VelocitySolvers";
      params_.optimisation_window = 1;
      std::string package_path;
      ASSERT_TRUE(findPackagePath("testing_pkg", package_path)); //!< Removes dependency on ros
      resource_path_ = package_path.append("/resource/Exotica/");
    }
    ;

    /** Data Members **/
    std::string resource_path_;
    std::vector<std::string> registered_types_;
    exotica::OptimisationParameters_t params_;
    boost::shared_ptr<exotica::VelocitySolver> vel_solv_ptr_;
};

#endif

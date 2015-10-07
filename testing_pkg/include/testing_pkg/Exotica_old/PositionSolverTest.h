#ifndef TESTING_POSITION_SOLVER_TEST_H
#define TESTING_POSITION_SOLVER_TEST_H

#include <exotica/EXOTica.hpp>
#include <gtest/gtest.h>
#include "testing_pkg/TestingTools.h"
#include "testing_pkg/Exotica/PositionSolverType_1.h"
#include "testing_pkg/Exotica/VelocitySolverType_1.h"
#include "testing_pkg/Exotica/TaskType_1.h"
#include "testing_pkg/Exotica/TaskType_2.h"

/**
 * TO TEST
 *  > Initialisation
 */

class ExoticaPosSolverTest: public ::testing::Test
{
  public:

    virtual void SetUp()
    {
      EXPECT_TRUE(exotica::PositionSolverCreator::Instance()->listImplementations(registered_types_))
          << "Could Not list the registered PositionSolvers";
      std::string package_path;
      ASSERT_TRUE(findPackagePath("testing_pkg", package_path)); //!< Removes dependency on ros
      resource_path_ = package_path.append("/resource/Exotica/");
    }
    ;

    /** Data Members **/
    std::string resource_path_;
    std::vector<std::string> registered_types_;
    boost::shared_ptr<exotica::PositionSolver> pos_solv_ptr_;
};

#endif

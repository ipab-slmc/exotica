/*
 *      Author: Michael Camilleri
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

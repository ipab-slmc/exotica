//
// Copyright (c) 2018, University of Edinburgh
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#ifndef EXOTICA_CORE_H_
#define EXOTICA_CORE_H_

#include <exotica_core/dynamics_solver.h>
#include <exotica_core/feedback_motion_solver.h>
#include <exotica_core/loaders/xml_loader.h>
#include <exotica_core/motion_solver.h>
#include <exotica_core/planning_problem.h>
#include <exotica_core/problems/bounded_end_pose_problem.h>
#include <exotica_core/problems/bounded_time_indexed_problem.h>
#include <exotica_core/problems/dynamic_time_indexed_shooting_problem.h>
#include <exotica_core/problems/end_pose_problem.h>
#include <exotica_core/problems/sampling_problem.h>
#include <exotica_core/problems/time_indexed_problem.h>
#include <exotica_core/problems/time_indexed_sampling_problem.h>
#include <exotica_core/problems/unconstrained_end_pose_problem.h>
#include <exotica_core/problems/unconstrained_time_indexed_problem.h>
#include <exotica_core/server.h>
#include <exotica_core/setup.h>
#include <exotica_core/tools.h>
#include <exotica_core/version.h>

#endif  // EXOTICA_CORE_H_

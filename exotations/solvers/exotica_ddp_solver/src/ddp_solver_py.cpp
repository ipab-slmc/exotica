//
// Copyright (c) 2018-2020, University of Edinburgh, University of Oxford
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

#include <exotica_ddp_solver/analytic_ddp_solver.h>
#include <exotica_ddp_solver/control_limited_ddp_solver.h>
#include <exotica_ddp_solver/control_limited_feasibility_driven_ddp_solver.h>
#include <exotica_ddp_solver/feasibility_driven_ddp_solver.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace exotica;
namespace py = pybind11;

PYBIND11_MODULE(exotica_ddp_solver_py, module)
{
    module.doc() = "Exotica DDP Solver";

    py::module::import("pyexotica");

    py::class_<AbstractDDPSolver, std::shared_ptr<AbstractDDPSolver>, FeedbackMotionSolver>(module, "AbstractDDPSolver")
        .def_property_readonly("Vxx", &AbstractDDPSolver::get_Vxx)
        .def_property_readonly("Vx", &AbstractDDPSolver::get_Vx)
        .def_property_readonly("Qxx", &AbstractDDPSolver::get_Qxx)
        .def_property_readonly("Qux", &AbstractDDPSolver::get_Qux)
        .def_property_readonly("Quu", &AbstractDDPSolver::get_Quu)
        .def_property_readonly("Qx", &AbstractDDPSolver::get_Qx)
        .def_property_readonly("Qu", &AbstractDDPSolver::get_Qu)
        .def_property_readonly("K", &AbstractDDPSolver::get_K)
        .def_property_readonly("k", &AbstractDDPSolver::get_k)
        .def_property_readonly("X_try", &AbstractDDPSolver::get_X_try)
        .def_property_readonly("U_try", &AbstractDDPSolver::get_U_try)
        .def_property_readonly("X_ref", &AbstractDDPSolver::get_X_ref)
        .def_property_readonly("U_ref", &AbstractDDPSolver::get_U_ref)
        .def_property_readonly("Quu_inv", &AbstractDDPSolver::get_Quu_inv)
        .def_property_readonly("fx", &AbstractDDPSolver::get_fx)
        .def_property_readonly("fu", &AbstractDDPSolver::get_fu)
        .def_property_readonly("control_cost_evolution", &AbstractDDPSolver::get_control_cost_evolution)
        .def_property_readonly("steplength_evolution", &AbstractDDPSolver::get_steplength_evolution)
        .def_property_readonly("regularization_evolution", &AbstractDDPSolver::get_regularization_evolution);

    py::class_<AnalyticDDPSolver, std::shared_ptr<AnalyticDDPSolver>, AbstractDDPSolver> analytic_ddp_solver(module, "AnalyticDDPSolver");

    py::class_<ControlLimitedDDPSolver, std::shared_ptr<ControlLimitedDDPSolver>, AbstractDDPSolver> control_limited_ddp_solver(module, "ControlLimitedDDPSolver");

    py::class_<FeasibilityDrivenDDPSolver, std::shared_ptr<FeasibilityDrivenDDPSolver>, AbstractDDPSolver> feasibility_driven_ddp_solver(module, "FeasibilityDrivenDDPSolver");
    feasibility_driven_ddp_solver.def_property_readonly("fs", &FeasibilityDrivenDDPSolver::get_fs);
    feasibility_driven_ddp_solver.def_property_readonly("xs", &FeasibilityDrivenDDPSolver::get_xs);
    feasibility_driven_ddp_solver.def_property_readonly("us", &FeasibilityDrivenDDPSolver::get_us);

    py::class_<ControlLimitedFeasibilityDrivenDDPSolver, std::shared_ptr<ControlLimitedFeasibilityDrivenDDPSolver>, AbstractDDPSolver> control_limited_feasibility_driven_ddp_solver(module, "ControlLimitedFeasibilityDrivenDDPSolver");
}

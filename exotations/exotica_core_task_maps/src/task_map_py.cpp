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

#include <exotica_core/exotica_core.h>
#undef NDEBUG
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <exotica_core_task_maps/center_of_mass.h>
#include <exotica_core_task_maps/distance.h>
#include <exotica_core_task_maps/eff_axis_alignment.h>
#include <exotica_core_task_maps/eff_frame.h>
#include <exotica_core_task_maps/eff_orientation.h>
#include <exotica_core_task_maps/eff_position.h>
#include <exotica_core_task_maps/interaction_mesh.h>
#include <exotica_core_task_maps/joint_acceleration_backward_difference.h>
#include <exotica_core_task_maps/joint_jerk_backward_difference.h>
#include <exotica_core_task_maps/joint_limit.h>
#include <exotica_core_task_maps/joint_pose.h>
#include <exotica_core_task_maps/joint_velocity_backward_difference.h>
#include <exotica_core_task_maps/look_at.h>
#include <exotica_core_task_maps/point_to_line.h>
#include <exotica_core_task_maps/sphere_collision.h>

using namespace exotica;
namespace py = pybind11;

PYBIND11_MODULE(exotica_core_task_maps_py, module)
{
    module.doc() = "Exotica task map definitions";

    py::module::import("pyexotica");

    py::class_<EffFrame, std::shared_ptr<EffFrame>, TaskMap>(module, "EffFrame")
        .def_readonly("rotation_type", &EffFrame::rotation_type_);

    py::class_<EffPosition, std::shared_ptr<EffPosition>, TaskMap>(module, "EffPosition");

    py::class_<EffOrientation, std::shared_ptr<EffOrientation>, TaskMap>(module, "EffOrientation")
        .def_readonly("rotation_type", &EffOrientation::rotation_type_);

    py::class_<EffAxisAlignment, std::shared_ptr<EffAxisAlignment>, TaskMap>(module, "EffAxisAlignment")
        .def("get_axis", &EffAxisAlignment::GetAxis)
        .def("set_axis", &EffAxisAlignment::SetAxis)
        .def("get_direction", &EffAxisAlignment::GetDirection)
        .def("set_direction", &EffAxisAlignment::SetDirection);

    py::class_<LookAt, std::shared_ptr<LookAt>, TaskMap>(module, "LookAt")
        .def("get_look_at_target_in_world", &LookAt::get_look_at_target_in_world);

    py::class_<PointToLine, std::shared_ptr<PointToLine>, TaskMap>(module, "PointToLine")
        .def_property("end_point", &PointToLine::GetEndPoint, &PointToLine::SetEndPoint);

    py::class_<JointVelocityBackwardDifference, std::shared_ptr<JointVelocityBackwardDifference>, TaskMap>(module, "JointVelocityBackwardDifference")
        .def("set_previous_joint_state", &JointVelocityBackwardDifference::SetPreviousJointState);

    py::class_<JointAccelerationBackwardDifference, std::shared_ptr<JointAccelerationBackwardDifference>, TaskMap>(module, "JointAccelerationBackwardDifference")
        .def("set_previous_joint_state", &JointAccelerationBackwardDifference::SetPreviousJointState);

    py::class_<JointJerkBackwardDifference, std::shared_ptr<JointJerkBackwardDifference>, TaskMap>(module, "JointJerkBackwardDifference")
        .def("set_previous_joint_state", &JointJerkBackwardDifference::SetPreviousJointState);

    py::class_<CenterOfMass, std::shared_ptr<CenterOfMass>, TaskMap>(module, "CenterOfMass");

    py::class_<Distance, std::shared_ptr<Distance>, TaskMap>(module, "Distance");

    py::class_<JointPose, std::shared_ptr<JointPose>, TaskMap>(module, "JointPose")
        .def_readonly("N", &JointPose::N_)
        .def_readonly("joint_map", &JointPose::joint_map_)
        .def_readwrite("joint_ref", &JointPose::joint_ref_);

    py::class_<InteractionMesh, std::shared_ptr<InteractionMesh>, TaskMap>(module, "InteractionMesh")
        .def_property("W", &InteractionMesh::GetWeights, &InteractionMesh::SetWeights)
        .def("set_weight", &InteractionMesh::SetWeight)
        .def_static("compute_goal_laplace", [](const std::vector<KDL::Frame>& nodes, Eigen::MatrixXdRefConst weights) {
            Eigen::VectorXd goal;
            InteractionMesh::ComputeGoalLaplace(nodes, goal, weights);
            return goal;
        })
        .def_static("compute_laplace", [](Eigen::VectorXdRefConst eff_phi, Eigen::MatrixXdRefConst weights) {
            return InteractionMesh::ComputeLaplace(eff_phi, weights);
        });

    py::class_<JointLimit, std::shared_ptr<JointLimit>, TaskMap>(module, "JointLimit");

    py::class_<SphereCollision, std::shared_ptr<SphereCollision>, TaskMap>(module, "SphereCollision");
}

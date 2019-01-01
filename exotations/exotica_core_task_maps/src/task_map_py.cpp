#include <exotica_core/exotica_core.h>
#undef NDEBUG
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <exotica_core_task_maps/CoM.h>
#include <exotica_core_task_maps/distance.h>
#include <exotica_core_task_maps/eff_axis_alignment.h>
#include <exotica_core_task_maps/eff_frame.h>
#include <exotica_core_task_maps/eff_orientation.h>
#include <exotica_core_task_maps/eff_position.h>
#include <exotica_core_task_maps/identity.h>
#include <exotica_core_task_maps/interaction_mesh.h>
#include <exotica_core_task_maps/joint_acceleration_backward_difference.h>
#include <exotica_core_task_maps/joint_jerk_backward_difference.h>
#include <exotica_core_task_maps/joint_limit.h>
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
        .def("get_axis", &EffAxisAlignment::get_axis)
        .def("set_axis", &EffAxisAlignment::set_axis)
        .def("get_direction", &EffAxisAlignment::get_direction)
        .def("set_direction", &EffAxisAlignment::set_direction);

    py::class_<LookAt, std::shared_ptr<LookAt>, TaskMap>(module, "LookAt")
        .def("get_look_at_target_in_world", &LookAt::get_look_at_target_in_world);

    py::class_<Point2Line, std::shared_ptr<Point2Line>, TaskMap>(module, "Point2Line")
        .def_property("end_point", &Point2Line::getEndPoint, &Point2Line::setEndPoint);

    py::class_<JointVelocityBackwardDifference, std::shared_ptr<JointVelocityBackwardDifference>, TaskMap>(module, "JointVelocityBackwardDifference")
        .def("set_previous_joint_state", &JointVelocityBackwardDifference::set_previous_joint_state);

    py::class_<JointAccelerationBackwardDifference, std::shared_ptr<JointAccelerationBackwardDifference>, TaskMap>(module, "JointAccelerationBackwardDifference")
        .def("set_previous_joint_state", &JointAccelerationBackwardDifference::set_previous_joint_state);

    py::class_<JointJerkBackwardDifference, std::shared_ptr<JointJerkBackwardDifference>, TaskMap>(module, "JointJerkBackwardDifference")
        .def("set_previous_joint_state", &JointJerkBackwardDifference::set_previous_joint_state);

    py::class_<CoM, std::shared_ptr<CoM>, TaskMap>(module, "CoM");

    py::class_<Distance, std::shared_ptr<Distance>, TaskMap>(module, "Distance");

    py::class_<Identity, std::shared_ptr<Identity>, TaskMap>(module, "Identity")
        .def_readonly("N", &Identity::N_)
        .def_readonly("joint_map", &Identity::joint_map_)
        .def_readwrite("joint_ref", &Identity::joint_ref_);

    py::class_<IMesh, std::shared_ptr<IMesh>, TaskMap>(module, "IMesh")
        .def_property("W", &IMesh::get_weights, &IMesh::set_weights)
        .def("set_weight", &IMesh::set_weight)
        .def_static("compute_goal_laplace", [](const std::vector<KDL::Frame>& nodes, Eigen::MatrixXdRefConst weights) {
            Eigen::VectorXd goal;
            IMesh::compute_goal_laplace(nodes, goal, weights);
            return goal;
        })
        .def_static("compute_laplace", [](Eigen::VectorXdRefConst eff_phi, Eigen::MatrixXdRefConst weights) {
            return IMesh::compute_laplace(eff_phi, weights);
        });

    py::class_<JointLimit, std::shared_ptr<JointLimit>, TaskMap>(module, "JointLimit");

    py::class_<SphereCollision, std::shared_ptr<SphereCollision>, TaskMap>(module, "SphereCollision");
}

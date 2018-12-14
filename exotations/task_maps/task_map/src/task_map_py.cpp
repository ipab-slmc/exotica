#include <exotica/Exotica.h>
#undef NDEBUG
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <task_map/CoM.h>
#include <task_map/Distance.h>
#include <task_map/EffAxisAlignment.h>
#include <task_map/EffFrame.h>
#include <task_map/EffOrientation.h>
#include <task_map/EffPosition.h>
#include <task_map/IMesh.h>
#include <task_map/Identity.h>
#include <task_map/JointAccelerationBackwardDifference.h>
#include <task_map/JointJerkBackwardDifference.h>
#include <task_map/JointLimit.h>
#include <task_map/JointVelocityBackwardDifference.h>
#include <task_map/LookAt.h>
#include <task_map/Point2Line.h>
#include <task_map/SphereCollision.h>

using namespace exotica;
namespace py = pybind11;

PYBIND11_MODULE(task_map_py, module)
{
    module.doc() = "Exotica task map definitions";

    py::module::import("pyexotica");

    py::class_<EffFrame, std::shared_ptr<EffFrame>, TaskMap> effFrame(module, "EffFrame");
    effFrame.def_readonly("rotationType", &EffFrame::rotationType);

    py::class_<EffPosition, std::shared_ptr<EffPosition>, TaskMap> effPosition(module, "EffPosition");

    py::class_<EffOrientation, std::shared_ptr<EffOrientation>, TaskMap> effOrientation(module, "EffOrientation");
    effOrientation.def_readonly("rotationType", &EffOrientation::rotationType);

    py::class_<EffAxisAlignment, std::shared_ptr<EffAxisAlignment>, TaskMap> effAxisAlignment(module, "EffAxisAlignment");
    effAxisAlignment.def("getAxis", &EffAxisAlignment::getAxis);
    effAxisAlignment.def("setAxis", &EffAxisAlignment::setAxis);
    effAxisAlignment.def("getDirection", &EffAxisAlignment::getDirection);
    effAxisAlignment.def("setDirection", &EffAxisAlignment::setDirection);

    py::class_<LookAt, std::shared_ptr<LookAt>, TaskMap> lookAt(module, "LookAt");
    lookAt.def("getLookAtTarget", &LookAt::getLookAtTarget);

    py::class_<Point2Line, std::shared_ptr<Point2Line>, TaskMap> point2Line(module, "Point2Line");
    point2Line.def_property("endPoint", &Point2Line::getEndPoint, &Point2Line::setEndPoint);

    py::class_<JointVelocityBackwardDifference, std::shared_ptr<JointVelocityBackwardDifference>, TaskMap> jointVelocityBackwardDifference(module, "JointVelocityBackwardDifference");
    jointVelocityBackwardDifference.def("set_previous_joint_state", &JointVelocityBackwardDifference::SetPreviousJointState);

    py::class_<JointAccelerationBackwardDifference, std::shared_ptr<JointAccelerationBackwardDifference>, TaskMap> jointAccelerationBackwardDifference(module, "JointAccelerationBackwardDifference");
    jointAccelerationBackwardDifference.def("set_previous_joint_state", &JointAccelerationBackwardDifference::SetPreviousJointState);

    py::class_<JointJerkBackwardDifference, std::shared_ptr<JointJerkBackwardDifference>, TaskMap> jointJerkBackwardDifference(module, "JointJerkBackwardDifference");
    jointJerkBackwardDifference.def("set_previous_joint_state", &JointJerkBackwardDifference::SetPreviousJointState);

    py::class_<CoM, std::shared_ptr<CoM>, TaskMap> com(module, "CoM");

    py::class_<Distance, std::shared_ptr<Distance>, TaskMap> distance(module, "Distance");

    py::class_<Identity, std::shared_ptr<Identity>, TaskMap> identity(module, "Identity");
    identity.def_readonly("N", &Identity::N);
    identity.def_readonly("jointMap", &Identity::jointMap);
    identity.def_readwrite("jointRef", &Identity::jointRef);

    py::class_<IMesh, std::shared_ptr<IMesh>, TaskMap> iMesh(module, "IMesh");
    iMesh.def_property("W", &IMesh::getWeights, &IMesh::setWeights);
    iMesh.def("setWeight", &IMesh::setWeight);
    iMesh.def_static("computeGoalLaplace", [](const std::vector<KDL::Frame>& nodes, Eigen::MatrixXdRefConst Weights) {
        Eigen::VectorXd goal;
        IMesh::computeGoalLaplace(nodes, goal, Weights);
        return goal;
    });
    iMesh.def_static("computeLaplace", [](Eigen::VectorXdRefConst EffPhi, Eigen::MatrixXdRefConst Weights) { return IMesh::computeLaplace(EffPhi, Weights); });

    py::class_<JointLimit, std::shared_ptr<JointLimit>, TaskMap> jointLimit(module, "JointLimit");

    py::class_<SphereCollision, std::shared_ptr<SphereCollision>, TaskMap> sphereCollision(module, "SphereCollision");
}

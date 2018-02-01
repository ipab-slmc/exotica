#include <exotica/Exotica.h>
#undef NDEBUG
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <ompl_solver/ompl_native_solvers.h>

using namespace exotica;
namespace py = pybind11;

PYBIND11_MODULE(ompl_solver_py, module)
{
    module.doc() = "Exotica wrapper of OMPL solvers";

    py::module::import("pyexotica");

    py::class_<RRT, std::shared_ptr<RRT>, MotionSolver> rrt(module, "RRT");
    py::class_<RRTConnect, std::shared_ptr<RRTConnect>, MotionSolver> rrtcon(module, "RRTConnect");
    py::class_<PRM, std::shared_ptr<PRM>, MotionSolver> prm(module, "PRM");
    prm.def("growRoadmap", &PRM::growRoadmap);
    prm.def("expandRoadmap", &PRM::expandRoadmap);
    prm.def("clear", &PRM::clear);
    prm.def("clearQuery", &PRM::clearQuery);
    prm.def("setup", &PRM::setup);
    prm.def("edgeCount", &PRM::edgeCount);
    prm.def("milestoneCount", &PRM::milestoneCount);
    prm.def_property("multiQuery", &PRM::isMultiQuery, &PRM::setMultiQuery);

    py::class_<LazyPRM, std::shared_ptr<LazyPRM>, MotionSolver> lprm(module, "LazyPRM");
    lprm.def("clear", &LazyPRM::clear);
    lprm.def("clearQuery", &LazyPRM::clearQuery);
    lprm.def("setup", &LazyPRM::setup);
    lprm.def("edgeCount", &LazyPRM::edgeCount);
    lprm.def("milestoneCount", &LazyPRM::milestoneCount);
    lprm.def_property("multiQuery", &LazyPRM::isMultiQuery, &LazyPRM::setMultiQuery);
}

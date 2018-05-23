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

    py::class_<OMPLsolver<SamplingProblem>, std::shared_ptr<OMPLsolver<SamplingProblem>>, MotionSolver> omplSolver(module, "OMPLMotionSolver");
    omplSolver.def("getRandomSeed", &OMPLsolver<SamplingProblem>::getRandomSeed);

    // State-space properties exposed via OMPLsolver
    omplSolver.def("getMaximumExtent", &OMPLsolver<SamplingProblem>::getMaximumExtent);
    omplSolver.def("getLongestValidSegmentLength", &OMPLsolver<SamplingProblem>::getLongestValidSegmentLength);
    omplSolver.def("setLongestValidSegmentFraction", &OMPLsolver<SamplingProblem>::setLongestValidSegmentFraction);
    omplSolver.def("setValidSegmentCountFactor", &OMPLsolver<SamplingProblem>::setValidSegmentCountFactor);
    omplSolver.def("getValidSegmentCountFactor", &OMPLsolver<SamplingProblem>::getValidSegmentCountFactor);

    py::class_<RRT, std::shared_ptr<RRT>, OMPLsolver<SamplingProblem>> rrt(module, "RRT");
    py::class_<RRTConnect, std::shared_ptr<RRTConnect>, OMPLsolver<SamplingProblem>> rrtcon(module, "RRTConnect");
    rrtcon.def("getRange", &RRTConnect::getRange);
    rrtcon.def("setRange", &RRTConnect::setRange);
    py::class_<EST, std::shared_ptr<EST>, OMPLsolver<SamplingProblem>> est(module, "EST");
    py::class_<KPIECE, std::shared_ptr<KPIECE>, OMPLsolver<SamplingProblem>> kpiece(module, "KPIECE");
    py::class_<BKPIECE, std::shared_ptr<BKPIECE>, OMPLsolver<SamplingProblem>> bkpiece(module, "BKPIECE");
    py::class_<PRM, std::shared_ptr<PRM>, OMPLsolver<SamplingProblem>> prm(module, "PRM");
    prm.def("growRoadmap", &PRM::growRoadmap);
    prm.def("expandRoadmap", &PRM::expandRoadmap);
    prm.def("clear", &PRM::clear);
    prm.def("clearQuery", &PRM::clearQuery);
    prm.def("setup", &PRM::setup);
    prm.def("edgeCount", &PRM::edgeCount);
    prm.def("milestoneCount", &PRM::milestoneCount);
    prm.def_property("multiQuery", &PRM::isMultiQuery, &PRM::setMultiQuery);

    py::class_<LazyPRM, std::shared_ptr<LazyPRM>, OMPLsolver<SamplingProblem>> lprm(module, "LazyPRM");
    lprm.def("clear", &LazyPRM::clear);
    lprm.def("clearQuery", &LazyPRM::clearQuery);
    lprm.def("setup", &LazyPRM::setup);
    lprm.def("edgeCount", &LazyPRM::edgeCount);
    lprm.def("milestoneCount", &LazyPRM::milestoneCount);
    lprm.def_property("multiQuery", &LazyPRM::isMultiQuery, &LazyPRM::setMultiQuery);
}

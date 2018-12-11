#include <exotica/Exotica.h>  // TODO: be more selective
#undef NDEBUG
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <exotica_ompl_solver/ompl_native_solvers.h>

using namespace exotica;
namespace py = pybind11;

PYBIND11_MODULE(exotica_ompl_solver_py, module)
{
    module.doc() = "Exotica wrapper of OMPL solvers";

    py::module::import("pyexotica");

    py::class_<OMPLSolver<SamplingProblem>, std::shared_ptr<OMPLSolver<SamplingProblem>>, MotionSolver> ompl_solver(module, "OMPLMotionSolver");
    ompl_solver.def("GetRandomSeed", &OMPLSolver<SamplingProblem>::GetRandomSeed);

    // State-space properties exposed via OMPLSolver
    ompl_solver.def_property_readonly("maximum_extent", &OMPLSolver<SamplingProblem>::GetMaximumExtent);
    ompl_solver.def_property("longest_valid_segment_length", &OMPLSolver<SamplingProblem>::GetLongestValidSegmentLength, &OMPLSolver<SamplingProblem>::SetLongestValidSegmentFraction);
    ompl_solver.def_property("valid_segment_count_factor", &OMPLSolver<SamplingProblem>::GetValidSegmentCountFactor, &OMPLSolver<SamplingProblem>::SetValidSegmentCountFactor);

    py::class_<RRT, std::shared_ptr<RRT>, OMPLSolver<SamplingProblem>> rrt(module, "RRT");

    py::class_<RRTConnect, std::shared_ptr<RRTConnect>, OMPLSolver<SamplingProblem>> rrtcon(module, "RRTConnect");
    rrtcon.def_property("range", &RRTConnect::GetRange, &RRTConnect::SetRange);

    py::class_<EST, std::shared_ptr<EST>, OMPLSolver<SamplingProblem>> est(module, "EST");

    py::class_<KPIECE, std::shared_ptr<KPIECE>, OMPLSolver<SamplingProblem>> kpiece(module, "KPIECE");

    py::class_<BKPIECE, std::shared_ptr<BKPIECE>, OMPLSolver<SamplingProblem>> bkpiece(module, "BKPIECE");

    py::class_<PRM, std::shared_ptr<PRM>, OMPLSolver<SamplingProblem>> prm(module, "PRM");
    prm.def_property("multi_query", &PRM::IsMultiQuery, &PRM::SetMultiQuery);
    prm.def("grow_roadmap", &PRM::GrowRoadmap);
    prm.def("expand_roadmap", &PRM::ExpandRoadmap);
    prm.def("clear", &PRM::Clear);
    prm.def("clear_query", &PRM::ClearQuery);
    prm.def("setup", &PRM::Setup);
    prm.def("edge_count", &PRM::EdgeCount);
    prm.def("milestone_count", &PRM::MilestoneCount);

    py::class_<LazyPRM, std::shared_ptr<LazyPRM>, OMPLSolver<SamplingProblem>> lprm(module, "LazyPRM");
    lprm.def_property("multi_query", &LazyPRM::IsMultiQuery, &LazyPRM::SetMultiQuery);
    lprm.def("clear", &LazyPRM::Clear);
    lprm.def("clear_query", &LazyPRM::ClearQuery);
    lprm.def("setup", &LazyPRM::Setup);
    lprm.def("edge_count", &LazyPRM::EdgeCount);
    lprm.def("milestone_count", &LazyPRM::MilestoneCount);
}

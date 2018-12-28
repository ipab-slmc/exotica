#include <exotica_core/exotica_core.h>  // TODO: be more selective
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

    py::class_<RRTSolver, std::shared_ptr<RRTSolver>, OMPLSolver<SamplingProblem>> rrt(module, "RRTSolver");

    py::class_<RRTConnectSolver, std::shared_ptr<RRTConnectSolver>, OMPLSolver<SamplingProblem>> rrtcon(module, "RRTConnectSolver");
    rrtcon.def_property("range", &RRTConnectSolver::GetRange, &RRTConnectSolver::SetRange);

    py::class_<ESTSolver, std::shared_ptr<ESTSolver>, OMPLSolver<SamplingProblem>> est(module, "ESTSolver");

    py::class_<KPIECESolver, std::shared_ptr<KPIECESolver>, OMPLSolver<SamplingProblem>> kpiece(module, "KPIECESolver");

    py::class_<BKPIECESolver, std::shared_ptr<BKPIECESolver>, OMPLSolver<SamplingProblem>> bkpiece(module, "BKPIECESolver");

    py::class_<PRMSolver, std::shared_ptr<PRMSolver>, OMPLSolver<SamplingProblem>> prm(module, "PRMSolver");
    prm.def_property("multi_query", &PRMSolver::IsMultiQuery, &PRMSolver::SetMultiQuery);
    prm.def("grow_roadmap", &PRMSolver::GrowRoadmap);
    prm.def("expand_roadmap", &PRMSolver::ExpandRoadmap);
    prm.def("clear", &PRMSolver::Clear);
    prm.def("clear_query", &PRMSolver::ClearQuery);
    prm.def("setup", &PRMSolver::Setup);
    prm.def("edge_count", &PRMSolver::EdgeCount);
    prm.def("milestone_count", &PRMSolver::MilestoneCount);

    py::class_<LazyPRMSolver, std::shared_ptr<LazyPRMSolver>, OMPLSolver<SamplingProblem>> lprm(module, "LazyPRMSolver");
    lprm.def_property("multi_query", &LazyPRMSolver::IsMultiQuery, &LazyPRMSolver::SetMultiQuery);
    lprm.def("clear", &LazyPRMSolver::Clear);
    lprm.def("clear_query", &LazyPRMSolver::ClearQuery);
    lprm.def("setup", &LazyPRMSolver::Setup);
    lprm.def("edge_count", &LazyPRMSolver::EdgeCount);
    lprm.def("milestone_count", &LazyPRMSolver::MilestoneCount);
}

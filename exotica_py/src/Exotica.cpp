#include <exotica/Exotica.h>
#undef NDEBUG
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

using namespace exotica;
namespace py = pybind11;

std::map<std::string, Initializer> knownInitializers;

std::string version()
{
    return std::string(exotica::Version);
}

std::string branch()
{
    return std::string(exotica::Branch);
}

std::shared_ptr<exotica::MotionSolver> createSolver(const Initializer& init)
{
    return Setup::createSolver(init);
}

std::shared_ptr<exotica::TaskMap> createMap(const Initializer& init)
{
    return Setup::createMap(init);
}

std::shared_ptr<exotica::PlanningProblem> createProblem(const Initializer& init)
{
    return Setup::createProblem(init);
}

Initializer createInitializer(const Initializer& init)
{
    return Initializer(init);
}

std::pair<Initializer, Initializer> loadFromXML(std::string file_name, const std::string& solver_name = "", const std::string& problem_name = "", bool parsePathAsXML=false)
{
    Initializer solver, problem;
    XMLLoader::load(file_name, solver, problem, solver_name, problem_name, parsePathAsXML);
    return std::pair<Initializer, Initializer>(solver, problem);
}

void addInitializers(py::module& module)
{
    py::module inits = module.def_submodule("Initializers","Initializers for core EXOTica classes.");
    inits.def("Initializer", &createInitializer);
    std::vector<Initializer> initializers = Setup::getInitializers();
    for(Initializer& i : initializers)
    {

        std::string full_name = i.getName();
        std::string name = full_name.substr(8);
        knownInitializers[full_name] = createInitializer(i);
        inits.def((name+"Initializer").c_str(), [i](){return createInitializer(i);}, (name+"Initializer constructor.").c_str());
    }

    inits.def("loadXML", (Initializer (*)(std::string, bool)) &XMLLoader::load, "Loads initializer from XML", py::arg("xml"), py::arg("parseAsXMLString")=false);
    inits.def("loadXMLFull", &loadFromXML, "Loads initializer from XML", py::arg("xml"), py::arg("solver_name")=std::string(""), py::arg("problem_name")=std::string(""), py::arg("parseAsXMLString")=false);
}

Eigen::MatrixXd Solve(std::shared_ptr<MotionSolver> sol, Eigen::VectorXd q0)
{
    Eigen::MatrixXd ret;
    sol->Solve(q0, ret);
    return ret;
}

namespace pybind11 {
namespace detail {
    template <> struct type_caster<Initializer>
    {
    public:

        PYBIND11_TYPE_CASTER(Initializer, _("Initializer"));

        bool addPropertyFromDict(Property& target, PyObject* value_py)
        {
            if(target.getType()=="std::string")
            {
                target.set(std::string(PyString_AsString(value_py)));
                return true;
            }
            else if(target.getType()=="int")
            {
                if(PyString_Check(value_py) || PyUnicode_Check(value_py))
                {
                    target.set(parseInt(std::string(PyString_AsString(value_py))));
                    return true;
                }
                else if(PyInt_Check(value_py))
                {
                    target.set((int)PyInt_AsLong(value_py));
                    return true;
                }
            }
            else if(target.getType()=="long")
            {
                if(PyString_Check(value_py) || PyUnicode_Check(value_py))
                {
                    target.set((long)parseInt(std::string(PyString_AsString(value_py))));
                    return true;
                }
                else if(PyInt_Check(value_py))
                {
                    target.set(PyInt_AsLong(value_py));
                    return true;
                }
            }
            else if(target.getType()=="double")
            {
                if(PyString_Check(value_py) || PyUnicode_Check(value_py))
                {
                    target.set(parseDouble(std::string(PyString_AsString(value_py))));
                    return true;
                }
                else if(PyFloat_Check(value_py))
                {
                    target.set(PyFloat_AsDouble(value_py));
                    return true;
                }
            }
            else if(target.getType()=="Eigen::Matrix<double, -1, 1, 0, -1, 1>")
            {
                if(PyString_Check(value_py) || PyUnicode_Check(value_py))
                {
                    target.set(parseVector(std::string(PyString_AsString(value_py))));
                }
                else
                {
                    target.set(py::cast<Eigen::VectorXd>(value_py));
                }
                return true;
            }
            else if(target.getType()=="bool")
            {
                if(PyString_Check(value_py) || PyUnicode_Check(value_py))
                {
                    target.set(parseBool(std::string(PyString_AsString(value_py))));
                    return true;
                }
                else if(PyBool_Check(value_py))
                {
                    target.set(PyObject_IsTrue(value_py)==1);
                    return true;
                }
            }
            else if(target.getType()=="exotica::Initializer")
            {
                if(PyList_Check(value_py))
                {
                    Initializer tmp;
                    int n = PyList_Size(value_py);
                    if(n==1)
                    {
                        if(!PyToInit(PyList_GetItem(value_py, 0), tmp))
                        {
                            return false;
                        }
                    }
                    target.set(tmp);
                }
                else
                {
                    Initializer tmp;
                    if(!PyToInit(value_py, tmp))
                    {
                        return false;
                    }
                    target.set(tmp);
                }
                return true;
            }
            else if(target.isInitializerVectorType())
            {
                if(PyList_Check(value_py))
                {
                    int n = PyList_Size(value_py);
                    std::vector<Initializer> vec(n);
                    for(int i=0;i<n;i++)
                    {
                        if(!PyToInit(PyList_GetItem(value_py, i), vec[i]))
                        {
                            return false;
                        }
                    }
                    target.set(vec);
                    return true;
                }
            }
            else
            {
                HIGHLIGHT("Skipping unsupported type '"<<target.getType()<<"'");
            }

            return false;
        }

        bool PyToInit(PyObject *source, Initializer& ret)
        {
            if(!PyTuple_CheckExact(source)) return false;

            int sz = PyTuple_Size(source);

            if(sz<1 || sz>2) return false;

            PyObject* name_py = PyTuple_GetItem(source, 0);
            if(!PyString_Check(name_py)) return false;
            std::string name(PyString_AsString(name_py));

            ret = Initializer(knownInitializers.at(name));

            if(sz==2)
            {
                PyObject* dict = PyTuple_GetItem(source, 1);
                if(!PyDict_Check(dict)) return false;

                PyObject *key, *value_py;
                Py_ssize_t pos = 0;

                while (PyDict_Next(dict, &pos, &key, &value_py))
                {
                     std::string key_str(PyString_AsString(key));
                     if(ret.properties.find( key_str ) == ret.properties.end())
                     {
                         ret.addProperty(Property(key_str, false, boost::any(std::string(PyString_AsString(value_py)))));
                     }
                     else
                     {
                        if(!addPropertyFromDict(ret.properties.at(key_str), value_py))
                        {
                            HIGHLIGHT("Failed to add property '"<<key_str<<"'");
                            return false;
                        }
                     }
                }
            }
            return true;
        }

        bool load(handle src, bool)
        {
            return PyToInit(src.ptr(), value);
        }

        static PyObject* InitializerToTuple(const Initializer& src)
        {
            PyObject* dict = PyDict_New();
            for(auto& prop : src.properties)
            {
                addPropertyToDict(dict, prop.first, prop.second);
            }
            return PyTuple_Pack(2,PyString_FromString(src.getName().c_str()), dict);
        }

        static void addPropertyToDict(PyObject* dict, const std::string& name, const Property& prop)
        {
            if(prop.getType()=="std::string")
            {
                PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<std::string>(prop.get())).ptr());
            }
            else if(prop.getType()=="int")
            {
                PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<int>(prop.get())).ptr());
            }
            else if(prop.getType()=="long")
            {
                PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<long>(prop.get())).ptr());
            }
            else if(prop.getType()=="double")
            {
                PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<double>(prop.get())).ptr());
            }
            else if(prop.getType()=="Eigen::Matrix<double, -1, 1, 0, -1, 1>")
            {
                PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<Eigen::VectorXd>(prop.get())).ptr());
            }
            else if(prop.getType()=="bool")
            {
                PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<bool>(prop.get())).ptr());
            }
            else if(prop.getType()=="exotica::Initializer")
            {
                PyDict_SetItemString(dict, name.c_str(), InitializerToTuple(boost::any_cast<Initializer>(prop.get())));
            }
            else if(prop.isInitializerVectorType())
            {
                PyObject* vec = PyList_New(0);
                for(Initializer& i : boost::any_cast<std::vector<Initializer>>(prop.get()))
                {
                    PyList_Append(vec, InitializerToTuple(i));
                }
                PyDict_SetItemString(dict, name.c_str(), vec);
            }
            else
            {
                HIGHLIGHT("Skipping unsupported type '"<<prop.getType()<<"'");
            }
        }        

        static handle cast(Initializer src, return_value_policy /* policy */, handle /* parent */)
        {
            return handle(InitializerToTuple(src));
        }
    };
}}

PYBIND11_MODULE(exotica_py, module)
{
    //Setup::Instance();

    module.doc() = "Exotica Python binding";

    int argc = 0;
    ros::init(argc, nullptr, "ExoticaPyDummyNode");

    py::class_<Setup, std::unique_ptr<Setup, py::nodelete>> setup(module, "Setup");
    setup.def("__init__",[](Setup* instance){instance=Setup::Instance().get();});
    setup.def_static("getSolvers", &Setup::getSolvers);
    setup.def_static("getProblems",&Setup::getProblems);
    setup.def_static("getMaps",&Setup::getMaps);
    setup.def_static("createSolver", &createSolver);
    setup.def_static("createMap",&createMap);
    setup.def_static("createProblem",&createProblem);
    setup.def_static("printSupportedClasses",&Setup::printSupportedClasses);
    setup.def_static("getInitializers",&Setup::getInitializers);

    py::class_<Object, std::shared_ptr<Object>> object(module, "Object");
    object.def("getType", &Object::type, "Object type");
    object.def("getName", &Object::getObjectName, "Object name");
    object.def("__repr__", &Object::print, "String representation of the object", py::arg("prepend")=std::string(""));
    object.def_readwrite("namespace", &Object::ns_);
    object.def_readwrite("debug", &Object::debug_);

    py::class_<TaskMap, std::shared_ptr<TaskMap>, Object> taskMap(module, "TaskMap");
    taskMap.def_readonly("id", &TaskMap::Id);
    taskMap.def_readonly("start", &TaskMap::Start);
    taskMap.def_readonly("length", &TaskMap::Length);
    taskMap.def_readonly("startJ", &TaskMap::StartJ);
    taskMap.def_readonly("lengthJ", &TaskMap::LengthJ);
    taskMap.def("getFrames",&TaskMap::GetFrames);

    py::class_<TaskSpaceVector, std::shared_ptr<TaskSpaceVector>> taskSpaceVector(module, "TaskSpaceVector");
    taskSpaceVector.def("setZero", &TaskSpaceVector::setZero);
    taskSpaceVector.def_readwrite("data", &TaskSpaceVector::data);
    taskSpaceVector.def("__sub__", &TaskSpaceVector::operator-, py::is_operator());
    taskSpaceVector.def("__repr__", [](TaskSpaceVector* instance){return ((std::ostringstream&)(std::ostringstream("")<<"TaskSpaceVector ("<<instance->data.transpose()<<")")).str();});

    py::class_<MotionSolver, std::shared_ptr<MotionSolver>, Object> motionSolver(module, "MotionSolver");
    motionSolver.def("specifyProblem", &MotionSolver::specifyProblem, "Assign problem to the solver", py::arg("planningProblem"));
    motionSolver.def("solve", [](std::shared_ptr<MotionSolver> sol, Eigen::VectorXd q0){return Solve(sol, q0);}, "Solve the problem", py::arg("q0"));
    motionSolver.def("getProblem", &MotionSolver::getProblem);

    py::class_<PlanningProblem, std::shared_ptr<PlanningProblem>, Object> planningProblem(module, "PlanningProblem");
    planningProblem.def("getTasks", &PlanningProblem::getTasks);
    planningProblem.def("getScene", &PlanningProblem::getScene);
    planningProblem.def("__repr__", &PlanningProblem::print, "String representation of the object", py::arg("prepend")=std::string(""));

    // Problem types
    py::module prob = module.def_submodule("Problems","Problem types");
    py::class_<UnconstrainedTimeIndexedProblem, std::shared_ptr<UnconstrainedTimeIndexedProblem>, PlanningProblem> unconstrainedTimeIndexedProblem(prob, "UnconstrainedTimeIndexedProblem");
    unconstrainedTimeIndexedProblem.def("getDuration", &UnconstrainedTimeIndexedProblem::getDuration);
    unconstrainedTimeIndexedProblem.def("update", &UnconstrainedTimeIndexedProblem::Update);
    unconstrainedTimeIndexedProblem.def("setGoal", &UnconstrainedTimeIndexedProblem::setGoal);
    unconstrainedTimeIndexedProblem.def("setRho", &UnconstrainedTimeIndexedProblem::setRho);
    unconstrainedTimeIndexedProblem.def("getGoal", &UnconstrainedTimeIndexedProblem::getGoal);
    unconstrainedTimeIndexedProblem.def("getRho", &UnconstrainedTimeIndexedProblem::getRho);
    unconstrainedTimeIndexedProblem.def_readwrite("tau", &UnconstrainedTimeIndexedProblem::tau);
    unconstrainedTimeIndexedProblem.def_readwrite("Q_rate", &UnconstrainedTimeIndexedProblem::Q_rate);
    unconstrainedTimeIndexedProblem.def_readwrite("H_rate", &UnconstrainedTimeIndexedProblem::H_rate);
    unconstrainedTimeIndexedProblem.def_readwrite("W_rate", &UnconstrainedTimeIndexedProblem::W_rate);
    unconstrainedTimeIndexedProblem.def_readwrite("W", &UnconstrainedTimeIndexedProblem::W);
    unconstrainedTimeIndexedProblem.def_readwrite("H", &UnconstrainedTimeIndexedProblem::H);
    unconstrainedTimeIndexedProblem.def_readwrite("Q", &UnconstrainedTimeIndexedProblem::Q);
    unconstrainedTimeIndexedProblem.def_readonly("T", &UnconstrainedTimeIndexedProblem::T);
    unconstrainedTimeIndexedProblem.def_readonly("PhiN", &UnconstrainedTimeIndexedProblem::PhiN);
    unconstrainedTimeIndexedProblem.def_readonly("JN", &UnconstrainedTimeIndexedProblem::JN);
    unconstrainedTimeIndexedProblem.def_readonly("N", &UnconstrainedTimeIndexedProblem::N);
    unconstrainedTimeIndexedProblem.def_readonly("NumTasks", &UnconstrainedTimeIndexedProblem::NumTasks);
    unconstrainedTimeIndexedProblem.def_readonly("Rho", &UnconstrainedTimeIndexedProblem::Rho);
    unconstrainedTimeIndexedProblem.def_readonly("y", &UnconstrainedTimeIndexedProblem::y);
    unconstrainedTimeIndexedProblem.def_readonly("Phi", &UnconstrainedTimeIndexedProblem::Phi);
    unconstrainedTimeIndexedProblem.def_readonly("ydiff", &UnconstrainedTimeIndexedProblem::ydiff);
    unconstrainedTimeIndexedProblem.def_readonly("J", &UnconstrainedTimeIndexedProblem::J);
    py::class_<UnconstrainedEndPoseProblem, std::shared_ptr<UnconstrainedEndPoseProblem>, PlanningProblem> unconstrainedEndPoseProblem(prob, "UnconstrainedEndPoseProblem");
    unconstrainedEndPoseProblem.def("update", &UnconstrainedEndPoseProblem::Update);
    unconstrainedEndPoseProblem.def("setGoal", &UnconstrainedEndPoseProblem::setGoal);
    unconstrainedEndPoseProblem.def("setRho", &UnconstrainedEndPoseProblem::setRho);
    unconstrainedEndPoseProblem.def("getGoal", &UnconstrainedEndPoseProblem::getGoal);
    unconstrainedEndPoseProblem.def("getRho", &UnconstrainedEndPoseProblem::getRho);
    unconstrainedEndPoseProblem.def_readwrite("W", &UnconstrainedEndPoseProblem::W);
    unconstrainedEndPoseProblem.def_readonly("PhiN", &UnconstrainedEndPoseProblem::PhiN);
    unconstrainedEndPoseProblem.def_readonly("JN", &UnconstrainedEndPoseProblem::JN);
    unconstrainedEndPoseProblem.def_readonly("N", &UnconstrainedEndPoseProblem::N);
    unconstrainedEndPoseProblem.def_readonly("NumTasks", &UnconstrainedEndPoseProblem::NumTasks);
    unconstrainedEndPoseProblem.def_readonly("Rho", &UnconstrainedEndPoseProblem::Rho);
    unconstrainedEndPoseProblem.def_readonly("y", &UnconstrainedEndPoseProblem::y);
    unconstrainedEndPoseProblem.def_readonly("Phi", &UnconstrainedEndPoseProblem::Phi);
    unconstrainedEndPoseProblem.def_readonly("J", &UnconstrainedEndPoseProblem::J);
    unconstrainedEndPoseProblem.def_readonly("qNominal", &UnconstrainedEndPoseProblem::qNominal);
    py::class_<SamplingProblem, std::shared_ptr<SamplingProblem>, PlanningProblem> samplingProblem(prob, "SamplingProblem");
    samplingProblem.def("update", &SamplingProblem::Update);
    samplingProblem.def("setGoalState", &SamplingProblem::setGoalState);
    samplingProblem.def("getSpaceDim", &SamplingProblem::getSpaceDim);
    samplingProblem.def("getBounds", &SamplingProblem::getBounds);

    py::class_<Scene, std::shared_ptr<Scene>, Object> scene(module, "Scene");

    py::module kin = module.def_submodule("Kinematics","Kinematics submodule.");
    py::class_<KinematicTree, std::shared_ptr<KinematicTree>> kinematicTree(kin, "KinematicTree");

    py::class_<KinematicFrameRequest, std::shared_ptr<KinematicFrameRequest>> kinFrameReq(kin, "KinematicFrameRequest");
    kinFrameReq.def(py::init<std::string, KDL::Frame, std::string, KDL::Frame>());
    kinFrameReq.def(py::init<std::string, KDL::Frame, std::string>());
    kinFrameReq.def(py::init<std::string, KDL::Frame>());
    kinFrameReq.def(py::init<std::string>());
    kinFrameReq.def_readwrite("frameALinkName", &KinematicFrameRequest::FrameALinkName);
    kinFrameReq.def_readwrite("frameAOffset", &KinematicFrameRequest::FrameAOffset);
    kinFrameReq.def_readwrite("frameBLinkName", &KinematicFrameRequest::FrameBLinkName);
    kinFrameReq.def_readwrite("frameBOffset", &KinematicFrameRequest::FrameBOffset);

    module.attr("version") = version();
    module.attr("branch") = branch();

    addInitializers(module);

    auto cleanup_exotica = []()
    {
        Setup::Destroy();
    };
    module.add_object("_cleanup", py::capsule(cleanup_exotica));
}

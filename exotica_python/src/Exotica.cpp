#include <exotica/Exotica.h>
#undef NDEBUG
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <ros/package.h>

#if PY_MAJOR_VERSION<3
#define PY_OLDSTANDARD
#endif

#ifndef PY_OLDSTANDARD
bool PyInt_Check(PyObject* value_py) {return PyLong_Check(value_py);}
long PyInt_AsLong(PyObject* value_py) {return PyLong_AsLong(value_py);}
#endif

bool isPyString(PyObject* value_py)
{
#ifndef PY_OLDSTANDARD
    return PyUnicode_Check(value_py);
#else
    return PyString_Check(value_py) || PyUnicode_Check(value_py);
#endif
}

std::string pyAsString(PyObject* value_py)
{
#ifndef PY_OLDSTANDARD
    PyObject* tmp = PyUnicode_AsASCIIString(value_py);
    std::string ret = std::string(PyBytes_AsString(tmp));
    Py_DECREF(tmp);
    return ret;
#else
    return std::string(PyString_AsString(value_py));
#endif
}

PyObject* stringAsPy(std::string value)
{
#ifndef PY_OLDSTANDARD
    return PyUnicode_DecodeASCII(value.c_str(),value.size(), "");
#else
    return PyString_FromString(value.c_str());
#endif
}

using namespace exotica;
namespace py = pybind11;

std::map<std::string, Initializer> knownInitializers;

PyObject* createStringIOObject() {
#if PY_MAJOR_VERSION <= 2
  PyObject* module = PyImport_ImportModule("StringIO");
#else
  PyObject* module = PyImport_ImportModule("io");
#endif
  if (!module) throw_pretty("Can't load StringIO module.");
  PyObject* cls = PyObject_GetAttrString(module, "StringIO");
  if (!cls) throw_pretty("Can't load StringIO class.");
  PyObject* stringio = PyObject_CallObject(cls, NULL);
  if (!stringio) throw_pretty("Can't create StringIO object.");

  return stringio;

  // Do we need to run Py_DECREF ?
}

#define ROS_MESSAGE_WRAPPER(MessageType)                                      \
  namespace pybind11 {                                                        \
  namespace detail {                                                          \
  template <>                                                                 \
  struct type_caster<MessageType> {                                           \
   public:                                                                    \
    PYBIND11_TYPE_CASTER(MessageType, _("genpy.Message"));                    \
                                                                              \
    bool load(handle src, bool) {                                             \
      PyObject* source = src.ptr();                                           \
      PyObject* stringio = createStringIOObject();                            \
      if (!stringio) throw_pretty("Can't create StringIO instance.");         \
      PyObject* result =                                                      \
          PyObject_CallMethod(source, "serialize", "O", stringio);            \
      if (!result) throw_pretty("Can't serialize.");                          \
      result = PyObject_CallMethod(stringio, "getvalue", nullptr);            \
      if (!result) throw_pretty("Can't get buffer.");                         \
      char* data = PyByteArray_AsString(PyByteArray_FromObject(result));      \
      int len = PyByteArray_Size(result);                                     \
      unsigned char* udata = new unsigned char[len];                          \
      for (int i = 0; i < len; i++)                                           \
        udata[i] = static_cast<unsigned char>(data[i]);                       \
      ros::serialization::IStream stream(udata, len);                         \
      ros::serialization::deserialize<MessageType>(stream, value);            \
      delete[] udata;                                                         \
      return !PyErr_Occurred();                                               \
    }                                                                         \
                                                                              \
    static handle cast(MessageType src,                                       \
                       return_value_policy /* policy /, handle / parent */) { \
      ros::message_traits::DataType<MessageType::Type> type;                  \
      throw_pretty("Can't create python object from message of type '"        \
                   << type.value() << "'!");                                  \
    }                                                                         \
  };                                                                          \
  }                                                                           \
  }
#include <moveit_msgs/PlanningSceneWorld.h>
ROS_MESSAGE_WRAPPER(moveit_msgs::PlanningSceneWorld);

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

Eigen::MatrixXd Solve(std::shared_ptr<MotionSolver> sol)
{
    Eigen::MatrixXd ret;
    sol->Solve(ret);
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
                target.set(pyAsString(value_py));
                return true;
            }
            else if(target.getType()=="int")
            {
                if(isPyString(value_py))
                {
                    target.set(parseInt(pyAsString(value_py)));
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
                if(isPyString(value_py))
                {
                    target.set((long)parseInt(pyAsString(value_py)));
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
                if(isPyString(value_py))
                {
                    target.set(parseDouble(pyAsString(value_py)));
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
                if(isPyString(value_py))
                {
                    target.set(parseVector(pyAsString(value_py)));
                }
                else
                {
                    target.set(py::cast<Eigen::VectorXd>(value_py));
                }
                return true;
            }
            else if(target.getType()=="bool")
            {
                if(isPyString(value_py))
                {
                    target.set(parseBool(pyAsString(value_py)));
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
            if(!isPyString(name_py)) return false;
            std::string name = pyAsString(name_py);

            ret = Initializer(knownInitializers.at(name));

            if(sz==2)
            {
                PyObject* dict = PyTuple_GetItem(source, 1);
                if(!PyDict_Check(dict)) return false;

                PyObject *key, *value_py;
                Py_ssize_t pos = 0;

                while (PyDict_Next(dict, &pos, &key, &value_py))
                {
                     std::string key_str = pyAsString(key);
                     if(ret.properties.find( key_str ) == ret.properties.end())
                     {
                         ret.addProperty(Property(key_str, false, boost::any(pyAsString(value_py))));
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
            return PyTuple_Pack(2,stringAsPy(src.getName()), dict);
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


PYBIND11_MODULE(_pyexotica, module)
{
    //Setup::Instance();

    module.doc() = "Exotica Python wrapper";

    py::class_<Setup, std::unique_ptr<Setup, py::nodelete>> setup(module, "Setup");
    setup.def("__init__",[](Setup* instance){instance=Setup::Instance().get();});
    setup.def_static("getSolvers", &Setup::getSolvers);
    setup.def_static("getProblems",&Setup::getProblems);
    setup.def_static("getMaps",&Setup::getMaps);
    setup.def_static("createSolver", &createSolver, py::return_value_policy::take_ownership);
    setup.def_static("createMap",&createMap, py::return_value_policy::take_ownership);
    setup.def_static("createProblem",&createProblem, py::return_value_policy::take_ownership);
    setup.def_static("printSupportedClasses",&Setup::printSupportedClasses);
    setup.def_static("getInitializers",&Setup::getInitializers, py::return_value_policy::copy);
    setup.def_static("getPackagePath",&ros::package::getPath);
    setup.def_static("initRos",[](){int argc = 0; ros::init(argc, nullptr, "exotica_python_node"); Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~")));});
    setup.def_static("loadSolver", &XMLLoader::loadSolver);

    py::module tools = module.def_submodule("Tools");
    tools.def("parsePath", &parsePath);
    tools.def("parseBool", &parseBool);
    tools.def("parseDouble", &parseDouble);
    tools.def("parseVector", &parseVector);
    tools.def("parseList", &parseList);
    tools.def("parseInt", &parseInt);
    tools.def("parseIntList", &parseIntList);
    tools.def("loadOBJ", [](const std::string& path){ Eigen::VectorXi tri; Eigen::VectorXd vert; loadOBJ(loadFile(path), tri, vert); return py::make_tuple(tri, vert);});
    tools.def("getText", &getText);
    tools.def("saveMatrix", &saveMatrix);
    tools.def("VectorTransform", &Eigen::VectorTransform);
    tools.def("IdentityTransform", &Eigen::IdentityTransform);
    tools.def("loadFile", &loadFile);
    tools.def("pathExists", &pathExists);

    py::class_<Timer, std::shared_ptr<Timer>> timer(module, "Timer");
    timer.def(py::init());
    timer.def("reset", &Timer::reset);
    timer.def("getDuration", &Timer::getDuration);

    py::class_<Object, std::shared_ptr<Object>> object(module, "Object");
    object.def("getType", &Object::type, "Object type");
    object.def("getName", &Object::getObjectName, "Object name");
    object.def("__repr__", &Object::print, "String representation of the object", py::arg("prepend")=std::string(""));
    object.def_readwrite("namespace", &Object::ns_);
    object.def_readwrite("debugMode", &Object::debug_);

    py::enum_<RotationType>(module, "RotationType")
        .value("Quaternion", RotationType::QUATERNION)
        .value("RPY", RotationType::RPY)
        .value("ZYZ", RotationType::ZYZ)
        .value("ZYX", RotationType::ZYX)
        .value("AngleAxis", RotationType::ANGLE_AXIS)
        .value("Matrix", RotationType::MATRIX)
        .export_values();

    py::class_<TaskMap, std::shared_ptr<TaskMap>, Object> taskMap(module, "TaskMap");
    taskMap.def_readonly("id", &TaskMap::Id);
    taskMap.def_readonly("start", &TaskMap::Start);
    taskMap.def_readonly("length", &TaskMap::Length);
    taskMap.def_readonly("startJ", &TaskMap::StartJ);
    taskMap.def_readonly("lengthJ", &TaskMap::LengthJ);
    taskMap.def("taskSpaceDim", (int (TaskMap::*)()) &TaskMap::taskSpaceDim);
    taskMap.def("taskSpaceJacobianDim", &TaskMap::taskSpaceJacobianDim);
    taskMap.def("debug", &TaskMap::debug);

    py::class_<TaskSpaceVector, std::shared_ptr<TaskSpaceVector>> taskSpaceVector(module, "TaskSpaceVector");
    taskSpaceVector.def("setZero", &TaskSpaceVector::setZero);
    taskSpaceVector.def_readwrite("data", &TaskSpaceVector::data);
    taskSpaceVector.def("__sub__", &TaskSpaceVector::operator-, py::is_operator());
    taskSpaceVector.def("__repr__", [](TaskSpaceVector* instance){return ((std::ostringstream&)(std::ostringstream("")<<"TaskSpaceVector ("<<instance->data.transpose()<<")")).str();});

    py::class_<MotionSolver, std::shared_ptr<MotionSolver>, Object> motionSolver(module, "MotionSolver");
    motionSolver.def("specifyProblem", &MotionSolver::specifyProblem, "Assign problem to the solver", py::arg("planningProblem"));
    motionSolver.def("solve", [](std::shared_ptr<MotionSolver> sol){return Solve(sol);}, "Solve the problem");
    motionSolver.def("getProblem", &MotionSolver::getProblem, py::return_value_policy::reference_internal);

    py::class_<PlanningProblem, std::shared_ptr<PlanningProblem>, Object> planningProblem(module, "PlanningProblem");
    planningProblem.def("getTasks", &PlanningProblem::getTasks, py::return_value_policy::reference_internal);
    planningProblem.def("getScene", &PlanningProblem::getScene, py::return_value_policy::reference_internal);
    planningProblem.def("__repr__", &PlanningProblem::print, "String representation of the object", py::arg("prepend")=std::string(""));
    planningProblem.def_property("startState", &PlanningProblem::getStartState, &PlanningProblem::setStartState);

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
    scene.def("Update", &Scene::Update);
    scene.def("getJointNames", (std::vector<std::string> (Scene::*)()) &Scene::getJointNames);
    scene.def("getSolver", &Scene::getSolver, py::return_value_policy::reference_internal);
    scene.def("getModelJointNames", &Scene::getModelJointNames);
    scene.def("getModelState", &Scene::getModelState);
    scene.def("getModelStateMap", &Scene::getModelStateMap);
    scene.def("setModelState", (void (Scene::*)(Eigen::VectorXdRefConst)) &Scene::setModelState);
    scene.def("setModelStateMap", (void (Scene::*)(std::map<std::string, double>)) &Scene::setModelState);
    scene.def("publishScene", &Scene::publishScene);
    // CollisionScene-related functions exposed via Scene - NB: may change in future    
    scene.def("getClosestDistance", [](Scene* instance) {
        return instance->getCollisionScene()->getClosestDistance();
    });

    py::module kin = module.def_submodule("Kinematics","Kinematics submodule.");
    py::class_<KinematicTree, std::shared_ptr<KinematicTree>> kinematicTree(kin, "KinematicTree");
    kinematicTree.def("publishFrames", &KinematicTree::publishFrames);
    kinematicTree.def("getJointLimits", &KinematicTree::getJointLimits);
    kinematicTree.def("getRootName", &KinematicTree::getRootName);
    kinematicTree.def("getUsedJointLimits", &KinematicTree::getUsedJointLimits);

    py::class_<KDL::Frame> kdlFrame (module, "KDLFrame");
    kdlFrame.def(py::init());
    kdlFrame.def("__init__", [](KDL::Frame &me, Eigen::MatrixXd other) {me = getFrameFromMatrix(other);});
    kdlFrame.def("__init__", [](KDL::Frame &me, Eigen::VectorXd other) {me = getFrame(other);});
    kdlFrame.def("__repr__", [](KDL::Frame* me){return "KDL::Frame "+toString(*me);});
    kdlFrame.def("getRPY", [](KDL::Frame* me){return getFrameAsVector(*me, RotationType::RPY);});
    kdlFrame.def("getZYZ", [](KDL::Frame* me){return getFrameAsVector(*me, RotationType::ZYZ);});
    kdlFrame.def("getZYX", [](KDL::Frame* me){return getFrameAsVector(*me, RotationType::ZYX);});
    kdlFrame.def("getAngleAxis", [](KDL::Frame* me){return getFrameAsVector(*me, RotationType::ANGLE_AXIS);});
    kdlFrame.def("getQuaternion", [](KDL::Frame* me){return getFrameAsVector(*me, RotationType::QUATERNION);});
    kdlFrame.def("get", [](KDL::Frame* me){return getFrame(*me);});
    kdlFrame.def("inverse", (KDL::Frame (KDL::Frame::*)() const) &KDL::Frame::Inverse);
    kdlFrame.def("__mul__", [](const KDL::Frame& A, const KDL::Frame& B){return A*B;}, py::is_operator());
    py::implicitly_convertible<Eigen::MatrixXd, KDL::Frame>();
    py::implicitly_convertible<Eigen::VectorXd, KDL::Frame>();

    module.attr("version") = version();
    module.attr("branch") = branch();

    addInitializers(module);

    auto cleanup_exotica = []()
    {
        Setup::Destroy();
    };
    module.add_object("_cleanup", py::capsule(cleanup_exotica));
}

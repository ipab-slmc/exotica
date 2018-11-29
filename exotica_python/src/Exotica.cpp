#include <exotica/Exotica.h>
#undef NDEBUG
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <ros/package.h>

#if PY_MAJOR_VERSION < 3
#define PY_OLDSTANDARD
#endif

#ifndef PY_OLDSTANDARD
bool PyInt_Check(PyObject* value_py) { return PyLong_Check(value_py); }
long PyInt_AsLong(PyObject* value_py) { return PyLong_AsLong(value_py); }
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
    return PyUnicode_DecodeASCII(value.c_str(), value.size(), "");
#else
    return PyString_FromString(value.c_str());
#endif
}

using namespace exotica;
namespace py = pybind11;

std::map<std::string, Initializer> knownInitializers;

PyObject* createStringIOObject()
{
#if PY_MAJOR_VERSION <= 2
    PyObject* module = PyImport_ImportModule("StringIO");
    if (!module) throw_pretty("Can't load StringIO module.");
    PyObject* cls = PyObject_GetAttrString(module, "StringIO");
    if (!cls) throw_pretty("Can't load StringIO class.");
#else
    PyObject* module = PyImport_ImportModule("io");
    if (!module) throw_pretty("Can't load io module.");
    PyObject* cls = PyObject_GetAttrString(module, "BytesIO");
    if (!cls) throw_pretty("Can't load BytesIO class.");
#endif
    PyObject* stringio = PyObject_CallObject(cls, NULL);
    if (!stringio) throw_pretty("Can't create StringIO object.");
    Py_DECREF(module);
    Py_DECREF(cls);
    return stringio;
}

#define ROS_MESSAGE_WRAPPER(MessageType)                                        \
    namespace pybind11                                                          \
    {                                                                           \
    namespace detail                                                            \
    {                                                                           \
    template <>                                                                 \
    struct type_caster<MessageType>                                             \
    {                                                                           \
    public:                                                                     \
        PYBIND11_TYPE_CASTER(MessageType, _("genpy.Message"));                  \
                                                                                \
        bool load(handle src, bool)                                             \
        {                                                                       \
            PyObject* stringio = createStringIOObject();                        \
            if (!stringio) throw_pretty("Can't create StringIO instance.");     \
            PyObject* result =                                                  \
                PyObject_CallMethod(src.ptr(), "serialize", "O", stringio);     \
            if (!result) throw_pretty("Can't serialize.");                      \
            result = PyObject_CallMethod(stringio, "getvalue", nullptr);        \
            if (!result) throw_pretty("Can't get buffer.");                     \
            char* data = PyByteArray_AsString(PyByteArray_FromObject(result));  \
            int len = PyByteArray_Size(result);                                 \
            unsigned char* udata = new unsigned char[len];                      \
            for (int i = 0; i < len; i++)                                       \
                udata[i] = static_cast<unsigned char>(data[i]);                 \
            ros::serialization::IStream stream(udata, len);                     \
            ros::serialization::deserialize<MessageType>(stream, value);        \
            delete[] udata;                                                     \
            delete[] data;                                                      \
            Py_DECREF(stringio);                                                \
            Py_DECREF(result);                                                  \
            return !PyErr_Occurred();                                           \
        }                                                                       \
                                                                                \
        static handle cast(MessageType src,                                     \
                           return_value_policy /* policy /, handle / parent */) \
        {                                                                       \
            ros::message_traits::DataType<MessageType::Type> type;              \
            throw_pretty("Can't create python object from message of type '"    \
                         << type.value() << "'!");                              \
        }                                                                       \
    };                                                                          \
    }                                                                           \
    }
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PlanningSceneWorld.h>
ROS_MESSAGE_WRAPPER(moveit_msgs::PlanningScene);
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

std::pair<Initializer, Initializer> loadFromXML(std::string file_name, const std::string& solver_name = "", const std::string& problem_name = "", bool parsePathAsXML = false)
{
    Initializer solver, problem;
    XMLLoader::load(file_name, solver, problem, solver_name, problem_name, parsePathAsXML);
    return std::pair<Initializer, Initializer>(solver, problem);
}

void addInitializers(py::module& module)
{
    py::module inits = module.def_submodule("Initializers", "Initializers for core EXOTica classes.");
    inits.def("Initializer", &createInitializer);
    std::vector<Initializer> initializers = Setup::getInitializers();
    for (Initializer& i : initializers)
    {
        std::string full_name = i.getName();
        std::string name = full_name.substr(8);
        knownInitializers[full_name] = createInitializer(i);
        inits.def((name + "Initializer").c_str(), [i]() { return createInitializer(i); }, (name + "Initializer constructor.").c_str());
    }

    inits.def("loadXML", (Initializer(*)(std::string, bool)) & XMLLoader::load, "Loads initializer from XML", py::arg("xml"), py::arg("parseAsXMLString") = false);
    inits.def("loadXMLFull", &loadFromXML, "Loads initializer from XML", py::arg("xml"), py::arg("solver_name") = std::string(""), py::arg("problem_name") = std::string(""), py::arg("parseAsXMLString") = false);
}

Eigen::MatrixXd Solve(std::shared_ptr<MotionSolver> sol)
{
    Eigen::MatrixXd ret;
    sol->Solve(ret);
    return ret;
}

namespace pybind11
{
namespace detail
{
template <>
struct type_caster<Initializer>
{
public:
    PYBIND11_TYPE_CASTER(Initializer, _("Initializer"));

    bool addPropertyFromDict(Property& target, PyObject* value_py)
    {
        if (target.getType() == "std::string" || target.getType() == getTypeName(typeid(std::string)))
        {
            target.set(pyAsString(value_py));
            return true;
        }
        else if (target.getType() == "int")
        {
            if (isPyString(value_py))
            {
                target.set(parseInt(pyAsString(value_py)));
                return true;
            }
            else if (PyInt_Check(value_py))
            {
                target.set((int)PyInt_AsLong(value_py));
                return true;
            }
        }
        else if (target.getType() == "long")
        {
            if (isPyString(value_py))
            {
                target.set((long)parseInt(pyAsString(value_py)));
                return true;
            }
            else if (PyInt_Check(value_py))
            {
                target.set(PyInt_AsLong(value_py));
                return true;
            }
        }
        else if (target.getType() == "double")
        {
            if (isPyString(value_py))
            {
                target.set(parseDouble(pyAsString(value_py)));
                return true;
            }
            else if (PyFloat_Check(value_py))
            {
                target.set(PyFloat_AsDouble(value_py));
                return true;
            }
        }
        else if (target.getType() == "Eigen::Matrix<double, -1, 1, 0, -1, 1>")
        {
            if (isPyString(value_py))
            {
                target.set(parseVector<double, Eigen::Dynamic>(pyAsString(value_py)));
            }
            else
            {
                target.set(py::cast<Eigen::VectorXd>(value_py));
            }
            return true;
        }
        else if (target.getType() == "Eigen::Matrix<double, 3, 1, 0, 3, 1>")
        {
            if (isPyString(value_py))
            {
                target.set(parseVector<double, 3>(pyAsString(value_py)));
            }
            else
            {
                target.set(py::cast<Eigen::Vector3d>(value_py));
            }
            return true;
        }
        else if (target.getType() == getTypeName(typeid(std::vector<int>)))
        {
            if (isPyString(value_py))
            {
                target.set(parseIntList(pyAsString(value_py)));
            }
            else
            {
                target.set(py::cast<std::vector<int>>(value_py));
            }
            return true;
        }
        else if (target.getType() == getTypeName(typeid(std::vector<std::string>)))
        {
            if (isPyString(value_py))
            {
                target.set(parseList(pyAsString(value_py)));
            }
            else
            {
                target.set(py::cast<std::vector<std::string>>(value_py));
            }
            return true;
        }
        else if (target.getType() == "bool")
        {
            if (isPyString(value_py))
            {
                target.set(parseBool(pyAsString(value_py)));
                return true;
            }
            else if (PyBool_Check(value_py))
            {
                target.set(PyObject_IsTrue(value_py) == 1);
                return true;
            }
        }
        else if (target.getType() == "exotica::Initializer")
        {
            if (PyList_Check(value_py))
            {
                Initializer tmp;
                int n = PyList_Size(value_py);
                if (n == 1)
                {
                    if (!PyToInit(PyList_GetItem(value_py, 0), tmp))
                    {
                        return false;
                    }
                }
                target.set(tmp);
            }
            else
            {
                Initializer tmp;
                if (!PyToInit(value_py, tmp))
                {
                    return false;
                }
                target.set(tmp);
            }
            return true;
        }
        else if (target.isInitializerVectorType())
        {
            if (PyList_Check(value_py))
            {
                int n = PyList_Size(value_py);
                std::vector<Initializer> vec(n);
                for (int i = 0; i < n; i++)
                {
                    if (!PyToInit(PyList_GetItem(value_py, i), vec[i]))
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
            HIGHLIGHT("Skipping unsupported type '" << target.getType() << "'");
        }

        return false;
    }

    bool PyToInit(PyObject* source, Initializer& ret)
    {
        if (!PyTuple_CheckExact(source)) return false;

        int sz = PyTuple_Size(source);

        if (sz < 1 || sz > 2) return false;

        PyObject* name_py = PyTuple_GetItem(source, 0);
        if (!isPyString(name_py)) return false;
        std::string name = pyAsString(name_py);

        const auto& it = knownInitializers.find(name);
        if (it == knownInitializers.end())
        {
            HIGHLIGHT("Unknown initializer type '" << name << "'");
            return false;
        }
        ret = Initializer(it->second);

        if (sz == 2)
        {
            PyObject* dict = PyTuple_GetItem(source, 1);
            if (!PyDict_Check(dict)) return false;

            PyObject *key, *value_py;
            Py_ssize_t pos = 0;

            while (PyDict_Next(dict, &pos, &key, &value_py))
            {
                std::string key_str = pyAsString(key);
                if (ret.properties.find(key_str) == ret.properties.end())
                {
                    ret.addProperty(Property(key_str, false, boost::any(pyAsString(value_py))));
                }
                else
                {
                    if (!addPropertyFromDict(ret.properties.at(key_str), value_py))
                    {
                        HIGHLIGHT("Failed to add property '" << key_str << "'");
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
        for (auto& prop : src.properties)
        {
            addPropertyToDict(dict, prop.first, prop.second);
        }
        PyObject* name = stringAsPy(src.getName());
        PyObject* tup = PyTuple_Pack(2, name, dict);
        Py_DECREF(dict);
        Py_DECREF(name);
        return tup;
    }

    static void addPropertyToDict(PyObject* dict, const std::string& name, const Property& prop)
    {
        if (prop.getType() == "std::string" || prop.getType() == getTypeName(typeid(std::string)))
        {
            PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<std::string>(prop.get())).ptr());
        }
        else if (prop.getType() == "int")
        {
            PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<int>(prop.get())).ptr());
        }
        else if (prop.getType() == "long")
        {
            PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<long>(prop.get())).ptr());
        }
        else if (prop.getType() == "double")
        {
            PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<double>(prop.get())).ptr());
        }
        else if (prop.getType() == "Eigen::Matrix<double, -1, 1, 0, -1, 1>")
        {
            PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<Eigen::VectorXd>(prop.get())).ptr());
        }
        else if (prop.getType() == "Eigen::Matrix<double, 3, 1, 0, 3, 1>")
        {
            PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<Eigen::Vector3d>(prop.get())).ptr());
        }
        else if (prop.getType() == getTypeName(typeid(std::vector<int>)))
        {
            PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<std::vector<int>>(prop.get())).ptr());
        }
        else if (prop.getType() == getTypeName(typeid(std::vector<std::string>)))
        {
            PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<std::vector<std::string>>(prop.get())).ptr());
        }
        else if (prop.getType() == "bool")
        {
            PyDict_SetItemString(dict, name.c_str(), py::cast(boost::any_cast<bool>(prop.get())).ptr());
        }
        else if (prop.getType() == "exotica::Initializer")
        {
            PyObject* init = InitializerToTuple(boost::any_cast<Initializer>(prop.get()));
            PyDict_SetItemString(dict, name.c_str(), init);
            Py_DECREF(init);
        }
        else if (prop.isInitializerVectorType())
        {
            PyObject* vec = PyList_New(0);
            for (Initializer& i : boost::any_cast<std::vector<Initializer>>(prop.get()))
            {
                PyObject* init = InitializerToTuple(i);
                PyList_Append(vec, init);
                Py_DECREF(init);
            }
            PyDict_SetItemString(dict, name.c_str(), vec);
            Py_DECREF(vec);
        }
        else
        {
            HIGHLIGHT("Skipping unsupported type '" << prop.getType() << "'");
        }
    }

    static handle cast(Initializer src, return_value_policy /* policy */, handle /* parent */)
    {
        return handle(InitializerToTuple(src));
    }
};
}
}

PYBIND11_MODULE(_pyexotica, module)
{
    //Setup::Instance();
    module.doc() = "Exotica Python wrapper";

    py::class_<Setup, std::unique_ptr<Setup, py::nodelete>> setup(module, "Setup");
    setup.def(py::init([]() { return Setup::Instance().get(); }));
    setup.def_static("getSolvers", &Setup::getSolvers, "Returns a list of available solvers.");
    setup.def_static("getProblems", &Setup::getProblems, "Returns a list of available problems.");
    setup.def_static("getMaps", &Setup::getMaps, "Returns a list of available task maps.");
    setup.def_static("getCollisionScenes", &Setup::getCollisionScenes, "Returns a list of available collision scene plug-ins.");
    setup.def_static("createSolver", &createSolver, py::return_value_policy::take_ownership);    // "Creates an instance of the solver identified by name parameter.", py::arg("solverType"), py::arg("prependExoticaNamespace"));
    setup.def_static("createProblem", &createProblem, py::return_value_policy::take_ownership);  // "Creates an instance of the problem identified by name parameter.", py::arg("problemType"), py::arg("prependExoticaNamespace"));
    setup.def_static("createMap", &createMap, py::return_value_policy::take_ownership);          // "Creates an instance of the task map identified by name parameter.", py::arg("taskmapType"), py::arg("prependExoticaNamespace"));
    setup.def_static("printSupportedClasses", &Setup::printSupportedClasses, "Print a list of available plug-ins sorted by class.");
    setup.def_static("getInitializers", &Setup::getInitializers, py::return_value_policy::copy, "Returns a list of available initializers with all available parameters/arguments.");
    setup.def_static("getPackagePath", &ros::package::getPath, "ROS package path resolution.");
    setup.def_static("initRos",
                     [](const std::string& name, const bool& anonymous) {
                         int argc = 0;
                         if (anonymous)
                         {
                             ros::init(argc, nullptr, name, ros::init_options::AnonymousName);
                         }
                         else
                         {
                             ros::init(argc, nullptr, name);
                         }
                         Server::InitRos(std::make_shared<ros::NodeHandle>("~"));
                     },
                     "Initializes an internal ROS node for publishing debug information from Exotica (i.e., activates ROS features). Options are setting the name and whether to spawn an anonymous node.",
                     py::arg("name") = "exotica", py::arg("anonymous") = false);
    setup.def_static("loadSolver", &XMLLoader::loadSolver, "Instantiate solver and problem from an XML file containing both a solver and problem initializer.", py::arg("filepath"));
    setup.def_static("loadSolverStandalone", &XMLLoader::loadSolverStandalone, "Instantiate only a solver from an XML file containing solely a solver initializer.", py::arg("filepath"));
    setup.def_static("loadProblem", &XMLLoader::loadProblem, "Instantiate only a problem from an XML file containing solely a problem initializer.", py::arg("filepath"));

    py::module tools = module.def_submodule("Tools");
    tools.def("parsePath", &parsePath);
    tools.def("parseBool", &parseBool);
    tools.def("parseDouble", &parseDouble);
    tools.def("parseVector", &parseVector<double, Eigen::Dynamic>);
    tools.def("parseList", &parseList);
    tools.def("parseInt", &parseInt);
    tools.def("parseIntList", &parseIntList);
    tools.def("loadOBJ", [](const std::string& path) { Eigen::VectorXi tri; Eigen::VectorXd vert; loadOBJ(loadFile(path), tri, vert); return py::make_tuple(tri, vert); });
    tools.def("getText", &getText);
    tools.def("saveMatrix", &saveMatrix);
    tools.def("VectorTransform", &Eigen::VectorTransform);
    tools.def("IdentityTransform", &Eigen::IdentityTransform);
    tools.def("loadFile", &loadFile);
    tools.def("pathExists", &pathExists);
    tools.def("createCompositeTrajectory", [](Eigen::MatrixXdRefConst data, double radius) {
        return Trajectory(data, radius).toString();
    },
              py::arg("data"), py::arg("maxradius") = 1.0);

    py::class_<Timer, std::shared_ptr<Timer>> timer(module, "Timer");
    timer.def(py::init());
    timer.def("reset", &Timer::reset);
    timer.def("getDuration", &Timer::getDuration);

    py::class_<Object, std::shared_ptr<Object>> object(module, "Object");
    object.def("getType", &Object::type, "Object type");
    object.def("getName", &Object::getObjectName, "Object name");
    object.def("__repr__", &Object::print, "String representation of the object", py::arg("prepend") = std::string(""));
    object.def_readwrite("namespace", &Object::ns_);
    object.def_readwrite("debugMode", &Object::debug_);

    py::enum_<TerminationCriterion>(module, "TerminationCriterion")
        .value("NotStarted", TerminationCriterion::NotStarted)
        .value("IterationLimit", TerminationCriterion::IterationLimit)
        .value("BacktrackIterationLimit", TerminationCriterion::BacktrackIterationLimit)
        .value("StepTolerance", TerminationCriterion::StepTolerance)
        .value("FunctionTolerance", TerminationCriterion::FunctionTolerance)
        .value("GradientTolerance", TerminationCriterion::GradientTolerance)
        .value("Divergence", TerminationCriterion::Divergence)
        .value("UserDefined", TerminationCriterion::UserDefined)
        .export_values();

    py::enum_<RotationType>(module, "RotationType")
        .value("Quaternion", RotationType::QUATERNION)
        .value("RPY", RotationType::RPY)
        .value("ZYZ", RotationType::ZYZ)
        .value("ZYX", RotationType::ZYX)
        .value("AngleAxis", RotationType::ANGLE_AXIS)
        .value("Matrix", RotationType::MATRIX)
        .export_values();

    py::enum_<BASE_TYPE>(module, "BaseType")
        .value("Fixed", BASE_TYPE::FIXED)
        .value("Floating", BASE_TYPE::FLOATING)
        .value("Planar", BASE_TYPE::PLANAR)
        .export_values();

    py::class_<KDL::Frame> kdlFrame(module, "KDLFrame");
    kdlFrame.def(py::init());
    kdlFrame.def(py::init([](Eigen::MatrixXd other) { return getFrameFromMatrix(other); }));
    kdlFrame.def(py::init([](Eigen::VectorXd other) { return getFrame(other); }));
    kdlFrame.def(py::init([](const KDL::Frame& other) { return KDL::Frame(other); }));
    kdlFrame.def("__repr__", [](KDL::Frame* me) { return "KDL::Frame " + toString(*me); });
    kdlFrame.def("getRPY", [](KDL::Frame* me) { return getRotationAsVector(*me, RotationType::RPY); });
    kdlFrame.def("getZYZ", [](KDL::Frame* me) { return getRotationAsVector(*me, RotationType::ZYZ); });
    kdlFrame.def("getZYX", [](KDL::Frame* me) { return getRotationAsVector(*me, RotationType::ZYX); });
    kdlFrame.def("getAngleAxis", [](KDL::Frame* me) { return getRotationAsVector(*me, RotationType::ANGLE_AXIS); });
    kdlFrame.def("getQuaternion", [](KDL::Frame* me) { return getRotationAsVector(*me, RotationType::QUATERNION); });
    kdlFrame.def("getTranslation", [](KDL::Frame* me) { Eigen::Vector3d tmp; for (int i = 0; i < 3; i++) { tmp[i] = me->p.data[i]; } return tmp; });
    kdlFrame.def("getTranslationAndRPY", [](KDL::Frame* me) { return getFrameAsVector(*me, RotationType::RPY); });
    kdlFrame.def("getTranslationAndZYZ", [](KDL::Frame* me) { return getFrameAsVector(*me, RotationType::ZYZ); });
    kdlFrame.def("getTranslationAndZYX", [](KDL::Frame* me) { return getFrameAsVector(*me, RotationType::ZYX); });
    kdlFrame.def("getTranslationAndAngleAxis", [](KDL::Frame* me) { return getFrameAsVector(*me, RotationType::ANGLE_AXIS); });
    kdlFrame.def("getTranslationAndQuaternion", [](KDL::Frame* me) { return getFrameAsVector(*me, RotationType::QUATERNION); });
    kdlFrame.def("getFrame", [](KDL::Frame* me) { return getFrame(*me); });
    kdlFrame.def("inverse", (KDL::Frame (KDL::Frame::*)() const) & KDL::Frame::Inverse);
    kdlFrame.def("__mul__", [](const KDL::Frame& A, const KDL::Frame& B) { return A * B; }, py::is_operator());
    py::implicitly_convertible<Eigen::MatrixXd, KDL::Frame>();
    py::implicitly_convertible<Eigen::VectorXd, KDL::Frame>();

    py::class_<TaskMap, std::shared_ptr<TaskMap>, Object> taskMap(module, "TaskMap");
    taskMap.def_readonly("id", &TaskMap::Id);
    taskMap.def_readonly("start", &TaskMap::Start);
    taskMap.def_readonly("length", &TaskMap::Length);
    taskMap.def_readonly("startJ", &TaskMap::StartJ);
    taskMap.def_readonly("lengthJ", &TaskMap::LengthJ);
    taskMap.def("taskSpaceDim", (int (TaskMap::*)()) & TaskMap::taskSpaceDim);
    taskMap.def("taskSpaceJacobianDim", &TaskMap::taskSpaceJacobianDim);
    taskMap.def("debug", &TaskMap::debug);

    py::class_<TimeIndexedTask, std::shared_ptr<TimeIndexedTask>> timeIndexedTask(module, "TimeIndexedTask");
    timeIndexedTask.def_readonly("PhiN", &TimeIndexedTask::PhiN);
    timeIndexedTask.def_readonly("JN", &TimeIndexedTask::JN);
    timeIndexedTask.def_readonly("NumTasks", &TimeIndexedTask::NumTasks);
    timeIndexedTask.def_readonly("y", &TimeIndexedTask::y);
    timeIndexedTask.def_readonly("ydiff", &TimeIndexedTask::ydiff);
    timeIndexedTask.def_readonly("Phi", &TimeIndexedTask::Phi);
    // timeIndexedTask.def_readonly("H", &TimeIndexedTask::H);
    timeIndexedTask.def_readonly("J", &TimeIndexedTask::J);
    timeIndexedTask.def_readonly("S", &TimeIndexedTask::S);
    timeIndexedTask.def_readonly("T", &TimeIndexedTask::T);
    timeIndexedTask.def_readonly("Tasks", &TimeIndexedTask::Tasks);
    timeIndexedTask.def_readonly("TaskMaps", &TimeIndexedTask::TaskMaps);

    py::class_<EndPoseTask, std::shared_ptr<EndPoseTask>> endPoseTask(module, "EndPoseTask");
    endPoseTask.def_readonly("PhiN", &EndPoseTask::PhiN);
    endPoseTask.def_readonly("JN", &EndPoseTask::JN);
    endPoseTask.def_readonly("NumTasks", &EndPoseTask::NumTasks);
    endPoseTask.def_readonly("y", &EndPoseTask::y);
    endPoseTask.def_readonly("ydiff", &EndPoseTask::ydiff);
    endPoseTask.def_readonly("Phi", &EndPoseTask::Phi);
    // endPoseTask.def_readonly("H", &EndPoseTask::H);
    endPoseTask.def_readonly("J", &EndPoseTask::J);
    endPoseTask.def_readonly("S", &EndPoseTask::S);
    endPoseTask.def_readonly("Tasks", &EndPoseTask::Tasks);
    endPoseTask.def_readonly("TaskMaps", &EndPoseTask::TaskMaps);

    py::class_<SamplingTask, std::shared_ptr<SamplingTask>> samplingTask(module, "SamplingTask");
    samplingTask.def_readonly("PhiN", &SamplingTask::PhiN);
    samplingTask.def_readonly("JN", &SamplingTask::JN);
    samplingTask.def_readonly("NumTasks", &SamplingTask::NumTasks);
    samplingTask.def_readonly("y", &SamplingTask::y);
    samplingTask.def_readonly("ydiff", &SamplingTask::ydiff);
    samplingTask.def_readonly("Phi", &SamplingTask::Phi);
    samplingTask.def_readonly("S", &SamplingTask::S);
    samplingTask.def_readonly("Tasks", &SamplingTask::Tasks);
    samplingTask.def_readonly("TaskMaps", &SamplingTask::TaskMaps);

    py::class_<TaskSpaceVector, std::shared_ptr<TaskSpaceVector>> taskSpaceVector(module, "TaskSpaceVector");
    taskSpaceVector.def("setZero", &TaskSpaceVector::setZero);
    taskSpaceVector.def_readonly("data", &TaskSpaceVector::data);
    taskSpaceVector.def("__sub__", &TaskSpaceVector::operator-, py::is_operator());
    taskSpaceVector.def("__repr__", [](TaskSpaceVector* instance) { return ((std::ostringstream&)(std::ostringstream("") << "TaskSpaceVector (" << instance->data.transpose() << ")")).str(); });

    py::class_<MotionSolver, std::shared_ptr<MotionSolver>, Object> motionSolver(module, "MotionSolver");
    motionSolver.def_property("maxIterations", &MotionSolver::getNumberOfMaxIterations, &MotionSolver::setNumberOfMaxIterations);
    motionSolver.def("getPlanningTime", &MotionSolver::getPlanningTime);
    motionSolver.def("specifyProblem", &MotionSolver::specifyProblem, "Assign problem to the solver", py::arg("planningProblem"));
    motionSolver.def(
        "solve", [](std::shared_ptr<MotionSolver> sol) { return Solve(sol); },
        "Solve the problem");
    motionSolver.def("getProblem", &MotionSolver::getProblem, py::return_value_policy::reference_internal);

    py::class_<PlanningProblem, std::shared_ptr<PlanningProblem>, Object> planningProblem(module, "PlanningProblem");
    planningProblem.def("getTasks", &PlanningProblem::getTasks, py::return_value_policy::reference_internal);
    planningProblem.def("getTaskMaps", &PlanningProblem::getTaskMaps, py::return_value_policy::reference_internal);
    planningProblem.def("getScene", &PlanningProblem::getScene, py::return_value_policy::reference_internal);
    planningProblem.def("__repr__", &PlanningProblem::print, "String representation of the object", py::arg("prepend") = std::string(""));
    planningProblem.def_property("startState", &PlanningProblem::getStartState, &PlanningProblem::setStartState);
    planningProblem.def_property("startTime", &PlanningProblem::getStartTime, &PlanningProblem::setStartTime);
    planningProblem.def("getNumberOfProblemUpdates", &PlanningProblem::getNumberOfProblemUpdates);
    planningProblem.def("resetNumberOfProblemUpdates", &PlanningProblem::resetNumberOfProblemUpdates);
    planningProblem.def("getCostEvolution", (std::pair<std::vector<double>, std::vector<double>> (PlanningProblem::*)()) & PlanningProblem::getCostEvolution);
    planningProblem.def("isValid", &PlanningProblem::isValid);

    // Problem types
    py::module prob = module.def_submodule("Problems", "Problem types");

    py::class_<UnconstrainedTimeIndexedProblem, std::shared_ptr<UnconstrainedTimeIndexedProblem>, PlanningProblem> unconstrainedTimeIndexedProblem(prob, "UnconstrainedTimeIndexedProblem");
    unconstrainedTimeIndexedProblem.def("getDuration", &UnconstrainedTimeIndexedProblem::getDuration);
    unconstrainedTimeIndexedProblem.def("update", &UnconstrainedTimeIndexedProblem::Update);
    unconstrainedTimeIndexedProblem.def("setGoal", &UnconstrainedTimeIndexedProblem::setGoal);
    unconstrainedTimeIndexedProblem.def("setRho", &UnconstrainedTimeIndexedProblem::setRho);
    unconstrainedTimeIndexedProblem.def("getGoal", &UnconstrainedTimeIndexedProblem::getGoal);
    unconstrainedTimeIndexedProblem.def("getRho", &UnconstrainedTimeIndexedProblem::getRho);
    unconstrainedTimeIndexedProblem.def_property("tau",
                                                 &UnconstrainedTimeIndexedProblem::getTau,
                                                 &UnconstrainedTimeIndexedProblem::setTau);
    unconstrainedTimeIndexedProblem.def_readwrite("W", &UnconstrainedTimeIndexedProblem::W);
    unconstrainedTimeIndexedProblem.def_property(
        "InitialTrajectory",
        &UnconstrainedTimeIndexedProblem::getInitialTrajectory,
        &UnconstrainedTimeIndexedProblem::setInitialTrajectory);
    unconstrainedTimeIndexedProblem.def_property("T",
                                                 &UnconstrainedTimeIndexedProblem::getT,
                                                 &UnconstrainedTimeIndexedProblem::setT);
    unconstrainedTimeIndexedProblem.def_readonly("PhiN", &UnconstrainedTimeIndexedProblem::PhiN);
    unconstrainedTimeIndexedProblem.def_readonly("JN", &UnconstrainedTimeIndexedProblem::JN);
    unconstrainedTimeIndexedProblem.def_readonly("N", &UnconstrainedTimeIndexedProblem::N);
    unconstrainedTimeIndexedProblem.def_readonly("NumTasks", &UnconstrainedTimeIndexedProblem::NumTasks);
    unconstrainedTimeIndexedProblem.def_readonly("Phi", &UnconstrainedTimeIndexedProblem::Phi);
    unconstrainedTimeIndexedProblem.def_readonly("J", &UnconstrainedTimeIndexedProblem::J);
    unconstrainedTimeIndexedProblem.def("getScalarTaskCost", &UnconstrainedTimeIndexedProblem::getScalarTaskCost);
    unconstrainedTimeIndexedProblem.def("getScalarTaskJacobian", &UnconstrainedTimeIndexedProblem::getScalarTaskJacobian);
    unconstrainedTimeIndexedProblem.def("getScalarTransitionCost", &UnconstrainedTimeIndexedProblem::getScalarTransitionCost);
    unconstrainedTimeIndexedProblem.def("getScalarTransitionJacobian", &UnconstrainedTimeIndexedProblem::getScalarTransitionJacobian);
    unconstrainedTimeIndexedProblem.def_readonly("Cost", &UnconstrainedTimeIndexedProblem::Cost);
    unconstrainedTimeIndexedProblem.def_property_readonly("KinematicSolutions", &UnconstrainedTimeIndexedProblem::getKinematicSolutions);

    py::class_<TimeIndexedProblem, std::shared_ptr<TimeIndexedProblem>, PlanningProblem> timeIndexedProblem(prob, "TimeIndexedProblem");
    timeIndexedProblem.def("getDuration", &TimeIndexedProblem::getDuration);
    timeIndexedProblem.def("update", &TimeIndexedProblem::Update);
    timeIndexedProblem.def("setGoal", &TimeIndexedProblem::setGoal);
    timeIndexedProblem.def("setRho", &TimeIndexedProblem::setRho);
    timeIndexedProblem.def("getGoal", &TimeIndexedProblem::getGoal);
    timeIndexedProblem.def("getRho", &TimeIndexedProblem::getRho);
    timeIndexedProblem.def("setGoalEQ", &TimeIndexedProblem::setGoalEQ);
    timeIndexedProblem.def("setRhoEQ", &TimeIndexedProblem::setRhoEQ);
    timeIndexedProblem.def("getGoalEQ", &TimeIndexedProblem::getGoalEQ);
    timeIndexedProblem.def("getRhoEQ", &TimeIndexedProblem::getRhoEQ);
    timeIndexedProblem.def("setGoalNEQ", &TimeIndexedProblem::setGoalNEQ);
    timeIndexedProblem.def("setRhoNEQ", &TimeIndexedProblem::setRhoNEQ);
    timeIndexedProblem.def("getGoalNEQ", &TimeIndexedProblem::getGoalNEQ);
    timeIndexedProblem.def("getRhoNEQ", &TimeIndexedProblem::getRhoNEQ);
    timeIndexedProblem.def_property("tau",
                                    &TimeIndexedProblem::getTau,
                                    &TimeIndexedProblem::setTau);
    timeIndexedProblem.def_property("qDot_max", &TimeIndexedProblem::getJointVelocityLimit, &TimeIndexedProblem::setJointVelocityLimit);
    timeIndexedProblem.def_readwrite("W", &TimeIndexedProblem::W);
    timeIndexedProblem.def_property(
        "InitialTrajectory",
        &TimeIndexedProblem::getInitialTrajectory,
        &TimeIndexedProblem::setInitialTrajectory);
    timeIndexedProblem.def_property("T",
                                    &TimeIndexedProblem::getT,
                                    &TimeIndexedProblem::setT);
    timeIndexedProblem.def_readonly("PhiN", &TimeIndexedProblem::PhiN);
    timeIndexedProblem.def_readonly("JN", &TimeIndexedProblem::JN);
    timeIndexedProblem.def_readonly("N", &TimeIndexedProblem::N);
    timeIndexedProblem.def_readonly("NumTasks", &TimeIndexedProblem::NumTasks);
    timeIndexedProblem.def_readonly("Phi", &TimeIndexedProblem::Phi);
    timeIndexedProblem.def_readonly("J", &TimeIndexedProblem::J);
    timeIndexedProblem.def("getScalarTaskCost", &TimeIndexedProblem::getScalarTaskCost);
    timeIndexedProblem.def("getScalarTaskJacobian", &TimeIndexedProblem::getScalarTaskJacobian);
    timeIndexedProblem.def("getScalarTransitionCost", &TimeIndexedProblem::getScalarTransitionCost);
    timeIndexedProblem.def("getScalarTransitionJacobian", &TimeIndexedProblem::getScalarTransitionJacobian);
    timeIndexedProblem.def("getEquality", &TimeIndexedProblem::getEquality);
    timeIndexedProblem.def("getEqualityJacobian", &TimeIndexedProblem::getEqualityJacobian);
    timeIndexedProblem.def("getInequality", &TimeIndexedProblem::getInequality);
    timeIndexedProblem.def("getInequalityJacobian", &TimeIndexedProblem::getInequalityJacobian);
    timeIndexedProblem.def("getBounds", &TimeIndexedProblem::getBounds);
    timeIndexedProblem.def_readonly("Cost", &TimeIndexedProblem::Cost);
    timeIndexedProblem.def_readonly("Inequality", &TimeIndexedProblem::Inequality);
    timeIndexedProblem.def_readonly("Equality", &TimeIndexedProblem::Equality);

    py::class_<BoundedTimeIndexedProblem, std::shared_ptr<BoundedTimeIndexedProblem>, PlanningProblem> boundedTimeIndexedProblem(prob, "BoundedTimeIndexedProblem");
    boundedTimeIndexedProblem.def("getDuration", &BoundedTimeIndexedProblem::getDuration);
    boundedTimeIndexedProblem.def("update", &BoundedTimeIndexedProblem::Update);
    boundedTimeIndexedProblem.def("setGoal", &BoundedTimeIndexedProblem::setGoal);
    boundedTimeIndexedProblem.def("setRho", &BoundedTimeIndexedProblem::setRho);
    boundedTimeIndexedProblem.def("getGoal", &BoundedTimeIndexedProblem::getGoal);
    boundedTimeIndexedProblem.def("getRho", &BoundedTimeIndexedProblem::getRho);
    boundedTimeIndexedProblem.def_property("tau",
                                           &BoundedTimeIndexedProblem::getTau,
                                           &BoundedTimeIndexedProblem::setTau);
    boundedTimeIndexedProblem.def_readwrite("W", &BoundedTimeIndexedProblem::W);
    boundedTimeIndexedProblem.def_property(
        "InitialTrajectory",
        &BoundedTimeIndexedProblem::getInitialTrajectory,
        &BoundedTimeIndexedProblem::setInitialTrajectory);
    boundedTimeIndexedProblem.def_property("T",
                                           &BoundedTimeIndexedProblem::getT,
                                           &BoundedTimeIndexedProblem::setT);
    boundedTimeIndexedProblem.def_readonly("PhiN", &BoundedTimeIndexedProblem::PhiN);
    boundedTimeIndexedProblem.def_readonly("JN", &BoundedTimeIndexedProblem::JN);
    boundedTimeIndexedProblem.def_readonly("N", &BoundedTimeIndexedProblem::N);
    boundedTimeIndexedProblem.def_readonly("NumTasks", &BoundedTimeIndexedProblem::NumTasks);
    boundedTimeIndexedProblem.def_readonly("Phi", &BoundedTimeIndexedProblem::Phi);
    boundedTimeIndexedProblem.def_readonly("J", &BoundedTimeIndexedProblem::J);
    boundedTimeIndexedProblem.def("getScalarTaskCost", &BoundedTimeIndexedProblem::getScalarTaskCost);
    boundedTimeIndexedProblem.def("getScalarTaskJacobian", &BoundedTimeIndexedProblem::getScalarTaskJacobian);
    boundedTimeIndexedProblem.def("getScalarTransitionCost", &BoundedTimeIndexedProblem::getScalarTransitionCost);
    boundedTimeIndexedProblem.def("getScalarTransitionJacobian", &BoundedTimeIndexedProblem::getScalarTransitionJacobian);
    boundedTimeIndexedProblem.def("getBounds", &BoundedTimeIndexedProblem::getBounds);
    boundedTimeIndexedProblem.def_readonly("Cost", &BoundedTimeIndexedProblem::Cost);

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
    unconstrainedEndPoseProblem.def_readonly("Phi", &UnconstrainedEndPoseProblem::Phi);
    unconstrainedEndPoseProblem.def_readonly("J", &UnconstrainedEndPoseProblem::J);
    unconstrainedEndPoseProblem.def_property_readonly("ydiff", [](UnconstrainedEndPoseProblem* prob) { return prob->Cost.ydiff; });
    unconstrainedEndPoseProblem.def_property("qNominal", &UnconstrainedEndPoseProblem::getNominalPose, &UnconstrainedEndPoseProblem::setNominalPose);
    unconstrainedEndPoseProblem.def("getScalarCost", &UnconstrainedEndPoseProblem::getScalarCost);
    unconstrainedEndPoseProblem.def("getScalarJacobian", &UnconstrainedEndPoseProblem::getScalarJacobian);
    unconstrainedEndPoseProblem.def("getScalarTaskCost", &UnconstrainedEndPoseProblem::getScalarTaskCost);
    unconstrainedEndPoseProblem.def_readonly("Cost", &UnconstrainedEndPoseProblem::Cost);

    py::class_<EndPoseProblem, std::shared_ptr<EndPoseProblem>, PlanningProblem> endPoseProblem(prob, "EndPoseProblem");
    endPoseProblem.def("update", &EndPoseProblem::Update);
    endPoseProblem.def("setGoal", &EndPoseProblem::setGoal);
    endPoseProblem.def("setRho", &EndPoseProblem::setRho);
    endPoseProblem.def("getGoal", &EndPoseProblem::getGoal);
    endPoseProblem.def("getRho", &EndPoseProblem::getRho);
    endPoseProblem.def("setGoalEQ", &EndPoseProblem::setGoalEQ);
    endPoseProblem.def("setRhoEQ", &EndPoseProblem::setRhoEQ);
    endPoseProblem.def("getGoalEQ", &EndPoseProblem::getGoalEQ);
    endPoseProblem.def("getRhoEQ", &EndPoseProblem::getRhoEQ);
    endPoseProblem.def("setGoalNEQ", &EndPoseProblem::setGoalNEQ);
    endPoseProblem.def("setRhoNEQ", &EndPoseProblem::setRhoNEQ);
    endPoseProblem.def("getGoalNEQ", &EndPoseProblem::getGoalNEQ);
    endPoseProblem.def("getRhoNEQ", &EndPoseProblem::getRhoNEQ);
    endPoseProblem.def_readwrite("W", &EndPoseProblem::W);
    endPoseProblem.def_readonly("PhiN", &EndPoseProblem::PhiN);
    endPoseProblem.def_readonly("JN", &EndPoseProblem::JN);
    endPoseProblem.def_readonly("N", &EndPoseProblem::N);
    endPoseProblem.def_readonly("NumTasks", &EndPoseProblem::NumTasks);
    endPoseProblem.def_readonly("Phi", &EndPoseProblem::Phi);
    endPoseProblem.def_readonly("J", &EndPoseProblem::J);
    endPoseProblem.def("getScalarCost", &EndPoseProblem::getScalarCost);
    endPoseProblem.def("getScalarJacobian", &EndPoseProblem::getScalarJacobian);
    endPoseProblem.def("getScalarTaskCost", &EndPoseProblem::getScalarTaskCost);
    endPoseProblem.def("getEquality", &EndPoseProblem::getEquality);
    endPoseProblem.def("getEqualityJacobian", &EndPoseProblem::getEqualityJacobian);
    endPoseProblem.def("getInequality", &EndPoseProblem::getInequality);
    endPoseProblem.def("getInequalityJacobian", &EndPoseProblem::getInequalityJacobian);
    endPoseProblem.def("getBounds", &EndPoseProblem::getBounds);
    endPoseProblem.def_readonly("Cost", &EndPoseProblem::Cost);
    endPoseProblem.def_readonly("Inequality", &EndPoseProblem::Inequality);
    endPoseProblem.def_readonly("Equality", &EndPoseProblem::Equality);

    py::class_<BoundedEndPoseProblem, std::shared_ptr<BoundedEndPoseProblem>, PlanningProblem> boundedEndPoseProblem(prob, "BoundedEndPoseProblem");
    boundedEndPoseProblem.def("update", &BoundedEndPoseProblem::Update);
    boundedEndPoseProblem.def("setGoal", &BoundedEndPoseProblem::setGoal);
    boundedEndPoseProblem.def("setRho", &BoundedEndPoseProblem::setRho);
    boundedEndPoseProblem.def("getGoal", &BoundedEndPoseProblem::getGoal);
    boundedEndPoseProblem.def("getRho", &BoundedEndPoseProblem::getRho);
    boundedEndPoseProblem.def_readwrite("W", &BoundedEndPoseProblem::W);
    boundedEndPoseProblem.def_readonly("PhiN", &BoundedEndPoseProblem::PhiN);
    boundedEndPoseProblem.def_readonly("JN", &BoundedEndPoseProblem::JN);
    boundedEndPoseProblem.def_readonly("N", &BoundedEndPoseProblem::N);
    boundedEndPoseProblem.def_readonly("NumTasks", &BoundedEndPoseProblem::NumTasks);
    boundedEndPoseProblem.def_readonly("Phi", &BoundedEndPoseProblem::Phi);
    boundedEndPoseProblem.def_readonly("J", &BoundedEndPoseProblem::J);
    boundedEndPoseProblem.def("getScalarCost", &BoundedEndPoseProblem::getScalarCost);
    boundedEndPoseProblem.def("getScalarJacobian", &BoundedEndPoseProblem::getScalarJacobian);
    boundedEndPoseProblem.def("getScalarTaskCost", &BoundedEndPoseProblem::getScalarTaskCost);
    boundedEndPoseProblem.def("getBounds", &BoundedEndPoseProblem::getBounds);
    boundedEndPoseProblem.def_readonly("Cost", &BoundedEndPoseProblem::Cost);

    py::class_<SamplingProblem, std::shared_ptr<SamplingProblem>, PlanningProblem> samplingProblem(prob, "SamplingProblem");
    samplingProblem.def("update", &SamplingProblem::Update);
    samplingProblem.def_property("goalState", &SamplingProblem::getGoalState, &SamplingProblem::setGoalState);
    samplingProblem.def("getSpaceDim", &SamplingProblem::getSpaceDim);
    samplingProblem.def("getBounds", &SamplingProblem::getBounds);
    samplingProblem.def_readonly("N", &SamplingProblem::N);
    samplingProblem.def_readonly("NumTasks", &SamplingProblem::NumTasks);
    samplingProblem.def_readonly("Phi", &SamplingProblem::Phi);
    samplingProblem.def_readonly("Inequality", &SamplingProblem::Inequality);
    samplingProblem.def_readonly("Equality", &SamplingProblem::Equality);
    samplingProblem.def("setGoalEQ", &SamplingProblem::setGoalEQ);
    samplingProblem.def("setRhoEQ", &SamplingProblem::setRhoEQ);
    samplingProblem.def("getGoalEQ", &SamplingProblem::getGoalEQ);
    samplingProblem.def("getRhoEQ", &SamplingProblem::getRhoEQ);
    samplingProblem.def("setGoalNEQ", &SamplingProblem::setGoalNEQ);
    samplingProblem.def("setRhoNEQ", &SamplingProblem::setRhoNEQ);
    samplingProblem.def("getGoalNEQ", &SamplingProblem::getGoalNEQ);
    samplingProblem.def("getRhoNEQ", &SamplingProblem::getRhoNEQ);

    py::class_<TimeIndexedSamplingProblem, std::shared_ptr<TimeIndexedSamplingProblem>, PlanningProblem> timeIndexedSamplingProblem(prob, "TimeIndexedSamplingProblem");
    timeIndexedSamplingProblem.def("update", &TimeIndexedSamplingProblem::Update);
    timeIndexedSamplingProblem.def("getSpaceDim", &TimeIndexedSamplingProblem::getSpaceDim);
    timeIndexedSamplingProblem.def("getBounds", &TimeIndexedSamplingProblem::getBounds);
    timeIndexedSamplingProblem.def_property("goalState", &TimeIndexedSamplingProblem::getGoalState, &TimeIndexedSamplingProblem::setGoalState);
    timeIndexedSamplingProblem.def_property("goalTime", &TimeIndexedSamplingProblem::getGoalTime, &TimeIndexedSamplingProblem::setGoalTime);
    timeIndexedSamplingProblem.def_readonly("N", &TimeIndexedSamplingProblem::N);
    timeIndexedSamplingProblem.def_readonly("NumTasks", &TimeIndexedSamplingProblem::NumTasks);
    timeIndexedSamplingProblem.def_readonly("Phi", &TimeIndexedSamplingProblem::Phi);
    timeIndexedSamplingProblem.def_readonly("Inequality", &TimeIndexedSamplingProblem::Inequality);
    timeIndexedSamplingProblem.def_readonly("Equality", &TimeIndexedSamplingProblem::Equality);
    timeIndexedSamplingProblem.def("setGoalEQ", &TimeIndexedSamplingProblem::setGoalEQ);
    timeIndexedSamplingProblem.def("setRhoEQ", &TimeIndexedSamplingProblem::setRhoEQ);
    timeIndexedSamplingProblem.def("getGoalEQ", &TimeIndexedSamplingProblem::getGoalEQ);
    timeIndexedSamplingProblem.def("getRhoEQ", &TimeIndexedSamplingProblem::getRhoEQ);
    timeIndexedSamplingProblem.def("setGoalNEQ", &TimeIndexedSamplingProblem::setGoalNEQ);
    timeIndexedSamplingProblem.def("setRhoNEQ", &TimeIndexedSamplingProblem::setRhoNEQ);
    timeIndexedSamplingProblem.def("getGoalNEQ", &TimeIndexedSamplingProblem::getGoalNEQ);
    timeIndexedSamplingProblem.def("getRhoNEQ", &TimeIndexedSamplingProblem::getRhoNEQ);

    py::class_<CollisionProxy, std::shared_ptr<CollisionProxy>> proxy(module, "CollisionProxy");
    proxy.def(py::init());
    proxy.def_readonly("Contact1", &CollisionProxy::contact1);
    proxy.def_readonly("Contact2", &CollisionProxy::contact2);
    proxy.def_readonly("Normal1", &CollisionProxy::normal1);
    proxy.def_readonly("Normal2", &CollisionProxy::normal2);
    proxy.def_readonly("Distance", &CollisionProxy::distance);
    proxy.def_property_readonly("Object1", [](CollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e1->Segment.getName() : std::string(""); });
    proxy.def_property_readonly("Object2", [](CollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e2->Segment.getName() : std::string(""); });
    proxy.def_property_readonly("Transform1", [](CollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e1->Frame : KDL::Frame(); });
    proxy.def_property_readonly("Transform2", [](CollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e2->Frame : KDL::Frame(); });
    proxy.def("__repr__", &CollisionProxy::print);

    py::class_<ContinuousCollisionProxy, std::shared_ptr<ContinuousCollisionProxy>> continuous_proxy(module, "ContinuousCollisionProxy");
    continuous_proxy.def(py::init());
    continuous_proxy.def_readonly("ContactTransform1", &ContinuousCollisionProxy::contact_tf1);
    continuous_proxy.def_readonly("ContactTransform2", &ContinuousCollisionProxy::contact_tf2);
    continuous_proxy.def_readonly("InCollision", &ContinuousCollisionProxy::in_collision);
    continuous_proxy.def_readonly("TimeOfContact", &ContinuousCollisionProxy::time_of_contact);
    continuous_proxy.def_property_readonly("Object1", [](ContinuousCollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e1->Segment.getName() : std::string(""); });
    continuous_proxy.def_property_readonly("Object2", [](ContinuousCollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e2->Segment.getName() : std::string(""); });
    continuous_proxy.def_property_readonly("Transform1", [](ContinuousCollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e1->Frame : KDL::Frame(); });
    continuous_proxy.def_property_readonly("Transform2", [](ContinuousCollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e2->Frame : KDL::Frame(); });
    continuous_proxy.def("__repr__", &ContinuousCollisionProxy::print);

    py::class_<Scene, std::shared_ptr<Scene>, Object> scene(module, "Scene");
    scene.def("Update", &Scene::Update, py::arg("x"), py::arg("t") = 0.0);
    scene.def("getBaseType", &Scene::getBaseType);
    scene.def("getGroupName", &Scene::getGroupName);
    scene.def("getJointNames", (std::vector<std::string> (Scene::*)()) & Scene::getJointNames);
    scene.def("getControlledLinkNames", &Scene::getControlledLinkNames);
    scene.def("getModelLinkNames", &Scene::getModelLinkNames);
    scene.def("getSolver", &Scene::getSolver, py::return_value_policy::reference_internal);
    scene.def("getCollisionScene", &Scene::getCollisionScene, py::return_value_policy::reference_internal);
    scene.def("getModelJointNames", &Scene::getModelJointNames);
    scene.def("getModelState", &Scene::getModelState);
    scene.def("getModelStateMap", &Scene::getModelStateMap);
    scene.def("setModelState", (void (Scene::*)(Eigen::VectorXdRefConst, double, bool)) & Scene::setModelState, py::arg("x"), py::arg("t") = 0.0, py::arg("updateTrajectory") = false);
    scene.def("setModelStateMap", (void (Scene::*)(std::map<std::string, double>, double, bool)) & Scene::setModelState, py::arg("x"), py::arg("t") = 0.0, py::arg("updateTrajectory") = false);
    scene.def("getControlledState", &Scene::getControlledState);
    scene.def("publishScene", &Scene::publishScene);
    scene.def("publishProxies", &Scene::publishProxies);
    scene.def("setCollisionScene", &Scene::setCollisionScene);
    scene.def("loadScene",
              (void (Scene::*)(const std::string&, const KDL::Frame&, bool)) & Scene::loadScene,
              py::arg("sceneString"),
              py::arg("offsetTransform") = kdlFrame(),
              py::arg("updateCollisionScene") = true);
    scene.def("loadSceneFile",
              (void (Scene::*)(const std::string&, const KDL::Frame&, bool)) & Scene::loadSceneFile,
              py::arg("fileName"),
              py::arg("offsetTransform") = kdlFrame(),
              py::arg("updateCollisionScene") = true);
    scene.def("getScene", &Scene::getScene);
    scene.def("cleanScene", &Scene::cleanScene);
    scene.def("isStateValid", [](Scene* instance, bool self, double safe_distance) { return instance->getCollisionScene()->isStateValid(self, safe_distance); }, py::arg("self") = true, py::arg("SafeDistance") = 0.0);
    scene.def("isCollisionFree", [](Scene* instance, const std::string& o1, const std::string& o2, double safe_distance) { return instance->getCollisionScene()->isCollisionFree(o1, o2, safe_distance); }, py::arg("Object1"), py::arg("Object2"), py::arg("SafeDistance") = 0.0);
    scene.def("isAllowedToCollide", [](Scene* instance, const std::string& o1, const std::string& o2, bool self) { return instance->getCollisionScene()->isAllowedToCollide(o1, o2, self); }, py::arg("Object1"), py::arg("Object2"), py::arg("self") = true);
    scene.def("getCollisionDistance", [](Scene* instance, bool self) { return instance->getCollisionScene()->getCollisionDistance(self); }, py::arg("self") = true);
    scene.def("getCollisionDistance", [](Scene* instance, const std::string& o1, const std::string& o2) { return instance->getCollisionScene()->getCollisionDistance(o1, o2); }, py::arg("Object1"), py::arg("Object2"));
    scene.def("getCollisionDistance",
              [](Scene* instance, const std::string& o1, const bool& self) {
                  return instance->getCollisionScene()->getCollisionDistance(o1, self);
              },
              py::arg("Object1"), py::arg("self") = true);
    scene.def("getCollisionDistance",
              [](Scene* instance, const std::vector<std::string>& objects, const bool& self) {
                  return instance->getCollisionScene()->getCollisionDistance(objects, self);
              },
              py::arg("objects"), py::arg("self") = true);
    scene.def("updateWorld",
              [](Scene* instance, moveit_msgs::PlanningSceneWorld& world) {
                  moveit_msgs::PlanningSceneWorldConstPtr myPtr(
                      new moveit_msgs::PlanningSceneWorld(world));
                  instance->updateWorld(myPtr);
              });
    scene.def("updateCollisionObjects", &Scene::updateCollisionObjects);
    scene.def("getCollisionRobotLinks", [](Scene* instance) { return instance->getCollisionScene()->getCollisionRobotLinks(); });
    scene.def("getCollisionWorldLinks", [](Scene* instance) { return instance->getCollisionScene()->getCollisionWorldLinks(); });
    scene.def("getRootFrameName", &Scene::getRootFrameName);
    scene.def("getRootJointName", &Scene::getRootJointName);
    scene.def("attachObject", &Scene::attachObject);
    scene.def("attachObjectLocal", &Scene::attachObjectLocal);
    scene.def("detachObject", &Scene::detachObject);
    scene.def("hasAttachedObject", &Scene::hasAttachedObject);
    scene.def("fk", [](Scene* instance, const std::string& e1, const KDL::Frame& o1, const std::string& e2, const KDL::Frame& o2) { return instance->getSolver().FK(e1, o1, e2, o2); });
    scene.def("fk", [](Scene* instance, const std::string& e1, const std::string& e2) { return instance->getSolver().FK(e1, KDL::Frame(), e2, KDL::Frame()); });
    scene.def("fk", [](Scene* instance, const std::string& e1) { return instance->getSolver().FK(e1, KDL::Frame(), "", KDL::Frame()); });
    scene.def("jacobian", [](Scene* instance, const std::string& e1, const KDL::Frame& o1, const std::string& e2, const KDL::Frame& o2) { return instance->getSolver().Jacobian(e1, o1, e2, o2); });
    scene.def("jacobian", [](Scene* instance, const std::string& e1, const std::string& e2) { return instance->getSolver().Jacobian(e1, KDL::Frame(), e2, KDL::Frame()); });
    scene.def("jacobian", [](Scene* instance, const std::string& e1) { return instance->getSolver().Jacobian(e1, KDL::Frame(), "", KDL::Frame()); });
    scene.def("addTrajectoryFromFile", &Scene::addTrajectoryFromFile);
    scene.def("addTrajectory", (void (Scene::*)(const std::string&, const std::string&)) & Scene::addTrajectory);
    scene.def("getTrajectory", [](Scene* instance, const std::string& link) { return instance->getTrajectory(link)->toString(); });
    scene.def("removeTrajectory", &Scene::removeTrajectory);
    scene.def("updateSceneFrames", &Scene::updateSceneFrames);
    scene.def("addObject", [](Scene* instance, const std::string& name, const KDL::Frame& transform, const std::string& parent, const std::string& shapeResourcePath, Eigen::Vector3d scale, bool updateCollisionScene) { instance->addObject(name, transform, parent, shapeResourcePath, scale, KDL::RigidBodyInertia::Zero(), updateCollisionScene); },
              py::arg("name"),
              py::arg("transform") = KDL::Frame(),
              py::arg("parent") = "",
              py::arg("shapeResourcePath"),
              py::arg("scale") = Eigen::Vector3d::Ones(),
              py::arg("updateCollisionScene") = true);
    scene.def("removeObject", &Scene::removeObject);
    scene.def_property_readonly("modelLinkToCollisionLinkMap", &Scene::getModelLinkToCollisionLinkMap);
    scene.def_property_readonly("controlledLinkToCollisionLinkMap", &Scene::getControlledLinkToCollisionLinkMap);

    py::class_<CollisionScene, std::shared_ptr<CollisionScene>> collisionScene(module, "CollisionScene");
    // TODO: expose isStateValid, isCollisionFree, getCollisionDistance, getCollisionWorldLinks, getCollisionRobotLinks, getTranslation
    collisionScene.def_property("alwaysExternallyUpdatedCollisionScene", &CollisionScene::getAlwaysExternallyUpdatedCollisionScene, &CollisionScene::setAlwaysExternallyUpdatedCollisionScene);
    collisionScene.def_property("replacePrimitiveShapesWithMeshes", &CollisionScene::getReplacePrimitiveShapesWithMeshes, &CollisionScene::setReplacePrimitiveShapesWithMeshes);
    collisionScene.def_readwrite("replaceCylindersWithCapsules", &CollisionScene::replaceCylindersWithCapsules);
    collisionScene.def_property("robotLinkScale", &CollisionScene::getRobotLinkScale, &CollisionScene::setRobotLinkScale);
    collisionScene.def_property("worldLinkScale", &CollisionScene::getWorldLinkScale, &CollisionScene::setWorldLinkScale);
    collisionScene.def_property("robotLinkPadding", &CollisionScene::getRobotLinkPadding, &CollisionScene::setRobotLinkPadding);
    collisionScene.def_property("worldLinkPadding", &CollisionScene::getWorldLinkPadding, &CollisionScene::setWorldLinkPadding);
    collisionScene.def("updateCollisionObjectTransforms", &CollisionScene::updateCollisionObjectTransforms);
    collisionScene.def("continuousCollisionCheck", &CollisionScene::continuousCollisionCheck);

    py::module kin = module.def_submodule("Kinematics", "Kinematics submodule.");
    py::class_<KinematicTree, std::shared_ptr<KinematicTree>> kinematicTree(kin, "KinematicTree");
    kinematicTree.def_readwrite("debugMode", &KinematicTree::Debug);
    kinematicTree.def("publishFrames", &KinematicTree::publishFrames);
    kinematicTree.def("getRootFrameName", &KinematicTree::getRootFrameName);
    kinematicTree.def("getRootJointName", &KinematicTree::getRootJointName);
    kinematicTree.def("getModelBaseType", &KinematicTree::getModelBaseType);
    kinematicTree.def("getControlledBaseType", &KinematicTree::getControlledBaseType);
    kinematicTree.def("getControlledLinkMass", &KinematicTree::getControlledLinkMass);
    kinematicTree.def("getCollisionObjectTypes", &KinematicTree::getCollisionObjectTypes);
    kinematicTree.def("getRandomControlledState", &KinematicTree::getRandomControlledState);
    kinematicTree.def("getNumModelJoints", &KinematicTree::getNumModelJoints);
    kinematicTree.def("getNumControlledJoints", &KinematicTree::getNumControlledJoints);

    // Joint Limits
    kinematicTree.def("getJointLimits", &KinematicTree::getJointLimits);
    kinematicTree.def("resetJointLimits", &KinematicTree::resetJointLimits);
    kinematicTree.def("setJointLimitsLower", &KinematicTree::setJointLimitsLower);
    kinematicTree.def("setJointLimitsUpper", &KinematicTree::setJointLimitsUpper);
    kinematicTree.def("setFloatingBaseLimitsPosXYZEulerZYX", &KinematicTree::setFloatingBaseLimitsPosXYZEulerZYX);
    kinematicTree.def("getUsedJointLimits", &KinematicTree::getUsedJointLimits);

    // TODO: KinematicRequestFlags

    // TODO: KinematicFrame

    // KinematicResponse
    py::class_<KinematicResponse, std::shared_ptr<KinematicResponse>> kinematicResponse(kin, "KinematicResponse");
    kinematicResponse.def_property_readonly("Phi", [](KinematicResponse* instance) {
        std::vector<KDL::Frame> vec;
        for (unsigned int i = 0; i < instance->Phi.cols(); i++)
            vec.push_back(instance->Phi(i));
        return vec;
    });

    py::enum_<shapes::ShapeType>(module, "ShapeType")
        .value("UnknownShape", shapes::ShapeType::UNKNOWN_SHAPE)
        .value("Sphere", shapes::ShapeType::SPHERE)
        .value("Cylinder", shapes::ShapeType::CYLINDER)
        .value("Cone", shapes::ShapeType::CONE)
        .value("Box", shapes::ShapeType::BOX)
        .value("Plane", shapes::ShapeType::PLANE)
        .value("Mesh", shapes::ShapeType::MESH)
        .value("Octree", shapes::ShapeType::OCTREE)
        .export_values();

    module.attr("version") = version();
    module.attr("branch") = branch();

    addInitializers(module);

    auto cleanup_exotica = []() {
        Setup::Destroy();
    };
    module.add_object("_cleanup", py::capsule(cleanup_exotica));
}

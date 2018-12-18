#include <exotica/Exotica.h>
#include <exotica/Visualization.h>
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

bool IsPyString(PyObject* value_py)
{
#ifndef PY_OLDSTANDARD
    return PyUnicode_Check(value_py);
#else
    return PyString_Check(value_py) || PyUnicode_Check(value_py);
#endif
}

std::string PyAsStdString(PyObject* value_py)
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

PyObject* StdStringAsPy(std::string value)
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

PyObject* CreateStringIOObject()
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
            PyObject* stringio = CreateStringIOObject();                        \
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

Initializer CreateInitializer(const Initializer& init)
{
    return Initializer(init);
}

std::pair<Initializer, Initializer> LoadFromXML(std::string file_name, const std::string& solver_name = "", const std::string& problem_name = "", bool parsePathAsXML = false)
{
    Initializer solver, problem;
    XMLLoader::load(file_name, solver, problem, solver_name, problem_name, parsePathAsXML);
    return std::pair<Initializer, Initializer>(solver, problem);
}

void AddInitializers(py::module& module)
{
    py::module inits = module.def_submodule("Initializers", "Initializers for core EXOTica classes.");
    inits.def("Initializer", &CreateInitializer);
    std::vector<Initializer> initializers = Setup::getInitializers();
    for (Initializer& i : initializers)
    {
        std::string full_name = i.getName();
        std::string name = full_name.substr(8);
        knownInitializers[full_name] = CreateInitializer(i);
        inits.def((name + "Initializer").c_str(), [i]() { return CreateInitializer(i); }, (name + "Initializer constructor.").c_str());
    }

    inits.def("loadXML", (Initializer(*)(std::string, bool)) & XMLLoader::load, "Loads initializer from XML", py::arg("xml"), py::arg("parseAsXMLString") = false);
    inits.def("loadXMLFull", &LoadFromXML, "Loads initializer from XML", py::arg("xml"), py::arg("solver_name") = std::string(""), py::arg("problem_name") = std::string(""), py::arg("parseAsXMLString") = false);
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
            target.set(PyAsStdString(value_py));
            return true;
        }
        else if (target.getType() == "int")
        {
            if (IsPyString(value_py))
            {
                target.set(parseInt(PyAsStdString(value_py)));
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
            if (IsPyString(value_py))
            {
                target.set((long)parseInt(PyAsStdString(value_py)));
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
            if (IsPyString(value_py))
            {
                target.set(parseDouble(PyAsStdString(value_py)));
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
            if (IsPyString(value_py))
            {
                target.set(parseVector<double, Eigen::Dynamic>(PyAsStdString(value_py)));
            }
            else
            {
                target.set(py::cast<Eigen::VectorXd>(value_py));
            }
            return true;
        }
        else if (target.getType() == "Eigen::Matrix<double, 3, 1, 0, 3, 1>")
        {
            if (IsPyString(value_py))
            {
                target.set(parseVector<double, 3>(PyAsStdString(value_py)));
            }
            else
            {
                target.set(py::cast<Eigen::Vector3d>(value_py));
            }
            return true;
        }
        else if (target.getType() == getTypeName(typeid(std::vector<int>)))
        {
            if (IsPyString(value_py))
            {
                target.set(parseIntList(PyAsStdString(value_py)));
            }
            else
            {
                target.set(py::cast<std::vector<int>>(value_py));
            }
            return true;
        }
        else if (target.getType() == getTypeName(typeid(std::vector<std::string>)))
        {
            if (IsPyString(value_py))
            {
                target.set(parseList(PyAsStdString(value_py)));
            }
            else
            {
                target.set(py::cast<std::vector<std::string>>(value_py));
            }
            return true;
        }
        else if (target.getType() == "bool")
        {
            if (IsPyString(value_py))
            {
                target.set(parseBool(PyAsStdString(value_py)));
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
        if (!IsPyString(name_py)) return false;
        std::string name = PyAsStdString(name_py);

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
                std::string key_str = PyAsStdString(key);
                if (ret.properties.find(key_str) == ret.properties.end())
                {
                    ret.addProperty(Property(key_str, false, boost::any(PyAsStdString(value_py))));
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
        PyObject* name = StdStringAsPy(src.getName());
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
    setup.def_static("createSolver", [](const Initializer& init) { return Setup::createSolver(init); }, py::return_value_policy::take_ownership);    // "Creates an instance of the solver identified by name parameter.", py::arg("solverType"), py::arg("prependExoticaNamespace"));
    setup.def_static("createProblem", [](const Initializer& init) { return Setup::createProblem(init); }, py::return_value_policy::take_ownership);  // "Creates an instance of the problem identified by name parameter.", py::arg("problemType"), py::arg("prependExoticaNamespace"));
    setup.def_static("createMap", [](const Initializer& init) { return Setup::createMap(init); }, py::return_value_policy::take_ownership);          // "Creates an instance of the task map identified by name parameter.", py::arg("taskmapType"), py::arg("prependExoticaNamespace"));
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

    py::class_<KDL::Frame> kdl_frame(module, "KDLFrame");
    kdl_frame.def(py::init());
    kdl_frame.def(py::init([](Eigen::MatrixXd other) { return getFrameFromMatrix(other); }));
    kdl_frame.def(py::init([](Eigen::VectorXd other) { return getFrame(other); }));
    kdl_frame.def(py::init([](const KDL::Frame& other) { return KDL::Frame(other); }));
    kdl_frame.def("__repr__", [](KDL::Frame* me) { return "KDL::Frame " + toString(*me); });
    kdl_frame.def("getRPY", [](KDL::Frame* me) { return getRotationAsVector(*me, RotationType::RPY); });
    kdl_frame.def("getZYZ", [](KDL::Frame* me) { return getRotationAsVector(*me, RotationType::ZYZ); });
    kdl_frame.def("getZYX", [](KDL::Frame* me) { return getRotationAsVector(*me, RotationType::ZYX); });
    kdl_frame.def("getAngleAxis", [](KDL::Frame* me) { return getRotationAsVector(*me, RotationType::ANGLE_AXIS); });
    kdl_frame.def("getQuaternion", [](KDL::Frame* me) { return getRotationAsVector(*me, RotationType::QUATERNION); });
    kdl_frame.def("getTranslation", [](KDL::Frame* me) { Eigen::Vector3d tmp; for (int i = 0; i < 3; i++) { tmp[i] = me->p.data[i]; } return tmp; });
    kdl_frame.def("getTranslationAndRPY", [](KDL::Frame* me) { return getFrameAsVector(*me, RotationType::RPY); });
    kdl_frame.def("getTranslationAndZYZ", [](KDL::Frame* me) { return getFrameAsVector(*me, RotationType::ZYZ); });
    kdl_frame.def("getTranslationAndZYX", [](KDL::Frame* me) { return getFrameAsVector(*me, RotationType::ZYX); });
    kdl_frame.def("getTranslationAndAngleAxis", [](KDL::Frame* me) { return getFrameAsVector(*me, RotationType::ANGLE_AXIS); });
    kdl_frame.def("getTranslationAndQuaternion", [](KDL::Frame* me) { return getFrameAsVector(*me, RotationType::QUATERNION); });
    kdl_frame.def("getFrame", [](KDL::Frame* me) { return getFrame(*me); });
    kdl_frame.def("inverse", (KDL::Frame (KDL::Frame::*)() const) & KDL::Frame::Inverse);
    kdl_frame.def("__mul__", [](const KDL::Frame& A, const KDL::Frame& B) { return A * B; }, py::is_operator());
    kdl_frame.def_readwrite("p", &KDL::Frame::p);
    py::implicitly_convertible<Eigen::MatrixXd, KDL::Frame>();
    py::implicitly_convertible<Eigen::VectorXd, KDL::Frame>();

    py::class_<KDL::Vector>(module, "KDLVector")
        .def(py::init())
        .def(py::init<double, double, double>(),
             py::arg("x") = 0,
             py::arg("y") = 0,
             py::arg("z") = 0)
        .def("x", [](KDL::Vector& v) -> double& { return v[0]; })
        .def("y", [](KDL::Vector& v) -> double& { return v[1]; })
        .def("z", [](KDL::Vector& v) -> double& { return v[2]; })
        .def_static("Zero", &KDL::Vector::Zero);

    py::class_<KDL::RotationalInertia>(module, "KDLRotationalInertia")
        .def(py::init<double, double, double, double, double, double>(),
             py::arg("Ixx") = 0,
             py::arg("Iyy") = 0,
             py::arg("Izz") = 0,
             py::arg("Ixy") = 0,
             py::arg("Ixz") = 0,
             py::arg("Iyz") = 0)
        .def_static("Zero", &KDL::RotationalInertia::Zero);

    py::class_<KDL::RigidBodyInertia>(module, "KDLRigidBodyInertia")
        .def(py::init<double, KDL::Vector&, KDL::RotationalInertia&>(),
             py::arg("m") = 0,
             py::arg("oc") = KDL::Vector::Zero(),
             py::arg("Ic") = KDL::RotationalInertia::Zero())
        .def_static("Zero", &KDL::RigidBodyInertia::Zero);

    py::class_<TaskMap, std::shared_ptr<TaskMap>, Object> task_map(module, "TaskMap");
    task_map.def_readonly("id", &TaskMap::Id);
    task_map.def_readonly("start", &TaskMap::Start);
    task_map.def_readonly("length", &TaskMap::Length);
    task_map.def_readonly("startJ", &TaskMap::StartJ);
    task_map.def_readonly("lengthJ", &TaskMap::LengthJ);
    task_map.def("taskSpaceDim", (int (TaskMap::*)()) & TaskMap::taskSpaceDim);
    task_map.def("taskSpaceJacobianDim", &TaskMap::taskSpaceJacobianDim);
    task_map.def("debug", &TaskMap::debug);

    py::class_<TimeIndexedTask, std::shared_ptr<TimeIndexedTask>> time_indexed_task(module, "TimeIndexedTask");
    time_indexed_task.def_readonly("PhiN", &TimeIndexedTask::PhiN);
    time_indexed_task.def_readonly("JN", &TimeIndexedTask::JN);
    time_indexed_task.def_readonly("NumTasks", &TimeIndexedTask::NumTasks);
    time_indexed_task.def_readonly("y", &TimeIndexedTask::y);
    time_indexed_task.def_readonly("ydiff", &TimeIndexedTask::ydiff);
    time_indexed_task.def_readonly("Phi", &TimeIndexedTask::Phi);
    // time_indexed_task.def_readonly("H", &TimeIndexedTask::H);
    time_indexed_task.def_readonly("J", &TimeIndexedTask::J);
    time_indexed_task.def_readonly("S", &TimeIndexedTask::S);
    time_indexed_task.def_readonly("T", &TimeIndexedTask::T);
    time_indexed_task.def_readonly("Tasks", &TimeIndexedTask::Tasks);
    time_indexed_task.def_readonly("TaskMaps", &TimeIndexedTask::TaskMaps);

    py::class_<EndPoseTask, std::shared_ptr<EndPoseTask>> end_pose_task(module, "EndPoseTask");
    end_pose_task.def_readonly("PhiN", &EndPoseTask::PhiN);
    end_pose_task.def_readonly("JN", &EndPoseTask::JN);
    end_pose_task.def_readonly("NumTasks", &EndPoseTask::NumTasks);
    end_pose_task.def_readonly("y", &EndPoseTask::y);
    end_pose_task.def_readonly("ydiff", &EndPoseTask::ydiff);
    end_pose_task.def_readonly("Phi", &EndPoseTask::Phi);
    // end_pose_task.def_readonly("H", &EndPoseTask::H);
    end_pose_task.def_readonly("J", &EndPoseTask::J);
    end_pose_task.def_readonly("S", &EndPoseTask::S);
    end_pose_task.def_readonly("Tasks", &EndPoseTask::Tasks);
    end_pose_task.def_readonly("TaskMaps", &EndPoseTask::TaskMaps);

    py::class_<SamplingTask, std::shared_ptr<SamplingTask>> sampling_task(module, "SamplingTask");
    sampling_task.def_readonly("PhiN", &SamplingTask::PhiN);
    sampling_task.def_readonly("JN", &SamplingTask::JN);
    sampling_task.def_readonly("NumTasks", &SamplingTask::NumTasks);
    sampling_task.def_readonly("y", &SamplingTask::y);
    sampling_task.def_readonly("ydiff", &SamplingTask::ydiff);
    sampling_task.def_readonly("Phi", &SamplingTask::Phi);
    sampling_task.def_readonly("S", &SamplingTask::S);
    sampling_task.def_readonly("Tasks", &SamplingTask::Tasks);
    sampling_task.def_readonly("TaskMaps", &SamplingTask::TaskMaps);

    py::class_<TaskSpaceVector, std::shared_ptr<TaskSpaceVector>> task_space_vector(module, "TaskSpaceVector");
    task_space_vector.def("setZero", &TaskSpaceVector::setZero);
    task_space_vector.def_readonly("data", &TaskSpaceVector::data);
    task_space_vector.def("__sub__", &TaskSpaceVector::operator-, py::is_operator());
    task_space_vector.def("__repr__", [](TaskSpaceVector* instance) { return ((std::ostringstream&)(std::ostringstream("") << "TaskSpaceVector (" << instance->data.transpose() << ")")).str(); });

    py::class_<MotionSolver, std::shared_ptr<MotionSolver>, Object> motion_solver(module, "MotionSolver");
    motion_solver.def_property("maxIterations", &MotionSolver::getNumberOfMaxIterations, &MotionSolver::setNumberOfMaxIterations);
    motion_solver.def("getPlanningTime", &MotionSolver::getPlanningTime);
    motion_solver.def("specifyProblem", &MotionSolver::specifyProblem, "Assign problem to the solver", py::arg("planning_problem"));
    motion_solver.def(
        "solve", [](std::shared_ptr<MotionSolver> sol) {
            Eigen::MatrixXd ret;
            sol->Solve(ret);
            return ret;
        },
        "Solve the problem");
    motion_solver.def("getProblem", &MotionSolver::getProblem, py::return_value_policy::reference_internal);

    py::class_<PlanningProblem, std::shared_ptr<PlanningProblem>, Object> planning_problem(module, "PlanningProblem");
    planning_problem.def("getTasks", &PlanningProblem::getTasks, py::return_value_policy::reference_internal);
    planning_problem.def("getTaskMaps", &PlanningProblem::getTaskMaps, py::return_value_policy::reference_internal);
    planning_problem.def("getScene", &PlanningProblem::getScene, py::return_value_policy::reference_internal);
    planning_problem.def("__repr__", &PlanningProblem::print, "String representation of the object", py::arg("prepend") = std::string(""));
    planning_problem.def_property("startState", &PlanningProblem::getStartState, &PlanningProblem::setStartState);
    planning_problem.def_property("startTime", &PlanningProblem::getStartTime, &PlanningProblem::setStartTime);
    planning_problem.def("getNumberOfProblemUpdates", &PlanningProblem::getNumberOfProblemUpdates);
    planning_problem.def("resetNumberOfProblemUpdates", &PlanningProblem::resetNumberOfProblemUpdates);
    planning_problem.def("getCostEvolution", (std::pair<std::vector<double>, std::vector<double>> (PlanningProblem::*)()) & PlanningProblem::getCostEvolution);
    planning_problem.def("isValid", &PlanningProblem::isValid);

    // Problem types
    py::module prob = module.def_submodule("Problems", "Problem types");

    py::class_<UnconstrainedTimeIndexedProblem, std::shared_ptr<UnconstrainedTimeIndexedProblem>, PlanningProblem> unconstrained_time_indexed_problem(prob, "UnconstrainedTimeIndexedProblem");
    unconstrained_time_indexed_problem.def("getDuration", &UnconstrainedTimeIndexedProblem::getDuration);
    unconstrained_time_indexed_problem.def("update", &UnconstrainedTimeIndexedProblem::Update);
    unconstrained_time_indexed_problem.def("setGoal", &UnconstrainedTimeIndexedProblem::setGoal);
    unconstrained_time_indexed_problem.def("setRho", &UnconstrainedTimeIndexedProblem::setRho);
    unconstrained_time_indexed_problem.def("getGoal", &UnconstrainedTimeIndexedProblem::getGoal);
    unconstrained_time_indexed_problem.def("getRho", &UnconstrainedTimeIndexedProblem::getRho);
    unconstrained_time_indexed_problem.def_property("tau",
                                                    &UnconstrainedTimeIndexedProblem::getTau,
                                                    &UnconstrainedTimeIndexedProblem::setTau);
    unconstrained_time_indexed_problem.def_readwrite("W", &UnconstrainedTimeIndexedProblem::W);
    unconstrained_time_indexed_problem.def_property(
        "InitialTrajectory",
        &UnconstrainedTimeIndexedProblem::getInitialTrajectory,
        &UnconstrainedTimeIndexedProblem::setInitialTrajectory);
    unconstrained_time_indexed_problem.def_property("T",
                                                    &UnconstrainedTimeIndexedProblem::getT,
                                                    &UnconstrainedTimeIndexedProblem::setT);
    unconstrained_time_indexed_problem.def_readonly("PhiN", &UnconstrainedTimeIndexedProblem::PhiN);
    unconstrained_time_indexed_problem.def_readonly("JN", &UnconstrainedTimeIndexedProblem::JN);
    unconstrained_time_indexed_problem.def_readonly("N", &UnconstrainedTimeIndexedProblem::N);
    unconstrained_time_indexed_problem.def_readonly("NumTasks", &UnconstrainedTimeIndexedProblem::NumTasks);
    unconstrained_time_indexed_problem.def_readonly("Phi", &UnconstrainedTimeIndexedProblem::Phi);
    unconstrained_time_indexed_problem.def_readonly("J", &UnconstrainedTimeIndexedProblem::J);
    unconstrained_time_indexed_problem.def("getScalarTaskCost", &UnconstrainedTimeIndexedProblem::getScalarTaskCost);
    unconstrained_time_indexed_problem.def("getScalarTaskJacobian", &UnconstrainedTimeIndexedProblem::getScalarTaskJacobian);
    unconstrained_time_indexed_problem.def("getScalarTransitionCost", &UnconstrainedTimeIndexedProblem::getScalarTransitionCost);
    unconstrained_time_indexed_problem.def("getScalarTransitionJacobian", &UnconstrainedTimeIndexedProblem::getScalarTransitionJacobian);
    unconstrained_time_indexed_problem.def_readonly("Cost", &UnconstrainedTimeIndexedProblem::Cost);
    unconstrained_time_indexed_problem.def_property_readonly("KinematicSolutions", &UnconstrainedTimeIndexedProblem::getKinematicSolutions);

    py::class_<TimeIndexedProblem, std::shared_ptr<TimeIndexedProblem>, PlanningProblem> time_indexed_problem(prob, "TimeIndexedProblem");
    time_indexed_problem.def("getDuration", &TimeIndexedProblem::getDuration);
    time_indexed_problem.def("update", &TimeIndexedProblem::Update);
    time_indexed_problem.def("setGoal", &TimeIndexedProblem::setGoal);
    time_indexed_problem.def("setRho", &TimeIndexedProblem::setRho);
    time_indexed_problem.def("getGoal", &TimeIndexedProblem::getGoal);
    time_indexed_problem.def("getRho", &TimeIndexedProblem::getRho);
    time_indexed_problem.def("setGoalEQ", &TimeIndexedProblem::setGoalEQ);
    time_indexed_problem.def("setRhoEQ", &TimeIndexedProblem::setRhoEQ);
    time_indexed_problem.def("getGoalEQ", &TimeIndexedProblem::getGoalEQ);
    time_indexed_problem.def("getRhoEQ", &TimeIndexedProblem::getRhoEQ);
    time_indexed_problem.def("setGoalNEQ", &TimeIndexedProblem::setGoalNEQ);
    time_indexed_problem.def("setRhoNEQ", &TimeIndexedProblem::setRhoNEQ);
    time_indexed_problem.def("getGoalNEQ", &TimeIndexedProblem::getGoalNEQ);
    time_indexed_problem.def("getRhoNEQ", &TimeIndexedProblem::getRhoNEQ);
    time_indexed_problem.def_property("tau",
                                      &TimeIndexedProblem::getTau,
                                      &TimeIndexedProblem::setTau);
    time_indexed_problem.def_property("qDot_max", &TimeIndexedProblem::getJointVelocityLimit, &TimeIndexedProblem::setJointVelocityLimit);
    time_indexed_problem.def_readwrite("W", &TimeIndexedProblem::W);
    time_indexed_problem.def_property(
        "InitialTrajectory",
        &TimeIndexedProblem::getInitialTrajectory,
        &TimeIndexedProblem::setInitialTrajectory);
    time_indexed_problem.def_property("T",
                                      &TimeIndexedProblem::getT,
                                      &TimeIndexedProblem::setT);
    time_indexed_problem.def_readonly("PhiN", &TimeIndexedProblem::PhiN);
    time_indexed_problem.def_readonly("JN", &TimeIndexedProblem::JN);
    time_indexed_problem.def_readonly("N", &TimeIndexedProblem::N);
    time_indexed_problem.def_readonly("NumTasks", &TimeIndexedProblem::NumTasks);
    time_indexed_problem.def_readonly("Phi", &TimeIndexedProblem::Phi);
    time_indexed_problem.def_readonly("J", &TimeIndexedProblem::J);
    time_indexed_problem.def("getScalarTaskCost", &TimeIndexedProblem::getScalarTaskCost);
    time_indexed_problem.def("getScalarTaskJacobian", &TimeIndexedProblem::getScalarTaskJacobian);
    time_indexed_problem.def("getScalarTransitionCost", &TimeIndexedProblem::getScalarTransitionCost);
    time_indexed_problem.def("getScalarTransitionJacobian", &TimeIndexedProblem::getScalarTransitionJacobian);
    time_indexed_problem.def("getEquality", &TimeIndexedProblem::getEquality);
    time_indexed_problem.def("getEqualityJacobian", &TimeIndexedProblem::getEqualityJacobian);
    time_indexed_problem.def("getInequality", &TimeIndexedProblem::getInequality);
    time_indexed_problem.def("getInequalityJacobian", &TimeIndexedProblem::getInequalityJacobian);
    time_indexed_problem.def("getBounds", &TimeIndexedProblem::getBounds);
    time_indexed_problem.def_readonly("Cost", &TimeIndexedProblem::Cost);
    time_indexed_problem.def_readonly("Inequality", &TimeIndexedProblem::Inequality);
    time_indexed_problem.def_readonly("Equality", &TimeIndexedProblem::Equality);

    py::class_<BoundedTimeIndexedProblem, std::shared_ptr<BoundedTimeIndexedProblem>, PlanningProblem> bounded_time_indexed_problem(prob, "BoundedTimeIndexedProblem");
    bounded_time_indexed_problem.def("getDuration", &BoundedTimeIndexedProblem::getDuration);
    bounded_time_indexed_problem.def("update", &BoundedTimeIndexedProblem::Update);
    bounded_time_indexed_problem.def("setGoal", &BoundedTimeIndexedProblem::setGoal);
    bounded_time_indexed_problem.def("setRho", &BoundedTimeIndexedProblem::setRho);
    bounded_time_indexed_problem.def("getGoal", &BoundedTimeIndexedProblem::getGoal);
    bounded_time_indexed_problem.def("getRho", &BoundedTimeIndexedProblem::getRho);
    bounded_time_indexed_problem.def_property("tau",
                                              &BoundedTimeIndexedProblem::getTau,
                                              &BoundedTimeIndexedProblem::setTau);
    bounded_time_indexed_problem.def_readwrite("W", &BoundedTimeIndexedProblem::W);
    bounded_time_indexed_problem.def_property(
        "InitialTrajectory",
        &BoundedTimeIndexedProblem::getInitialTrajectory,
        &BoundedTimeIndexedProblem::setInitialTrajectory);
    bounded_time_indexed_problem.def_property("T",
                                              &BoundedTimeIndexedProblem::getT,
                                              &BoundedTimeIndexedProblem::setT);
    bounded_time_indexed_problem.def_readonly("PhiN", &BoundedTimeIndexedProblem::PhiN);
    bounded_time_indexed_problem.def_readonly("JN", &BoundedTimeIndexedProblem::JN);
    bounded_time_indexed_problem.def_readonly("N", &BoundedTimeIndexedProblem::N);
    bounded_time_indexed_problem.def_readonly("NumTasks", &BoundedTimeIndexedProblem::NumTasks);
    bounded_time_indexed_problem.def_readonly("Phi", &BoundedTimeIndexedProblem::Phi);
    bounded_time_indexed_problem.def_readonly("J", &BoundedTimeIndexedProblem::J);
    bounded_time_indexed_problem.def("getScalarTaskCost", &BoundedTimeIndexedProblem::getScalarTaskCost);
    bounded_time_indexed_problem.def("getScalarTaskJacobian", &BoundedTimeIndexedProblem::getScalarTaskJacobian);
    bounded_time_indexed_problem.def("getScalarTransitionCost", &BoundedTimeIndexedProblem::getScalarTransitionCost);
    bounded_time_indexed_problem.def("getScalarTransitionJacobian", &BoundedTimeIndexedProblem::getScalarTransitionJacobian);
    bounded_time_indexed_problem.def("getBounds", &BoundedTimeIndexedProblem::getBounds);
    bounded_time_indexed_problem.def_readonly("Cost", &BoundedTimeIndexedProblem::Cost);

    py::class_<UnconstrainedEndPoseProblem, std::shared_ptr<UnconstrainedEndPoseProblem>, PlanningProblem> unconstrained_end_pose_problem(prob, "UnconstrainedEndPoseProblem");
    unconstrained_end_pose_problem.def("update", &UnconstrainedEndPoseProblem::Update);
    unconstrained_end_pose_problem.def("setGoal", &UnconstrainedEndPoseProblem::setGoal);
    unconstrained_end_pose_problem.def("setRho", &UnconstrainedEndPoseProblem::setRho);
    unconstrained_end_pose_problem.def("getGoal", &UnconstrainedEndPoseProblem::getGoal);
    unconstrained_end_pose_problem.def("getRho", &UnconstrainedEndPoseProblem::getRho);
    unconstrained_end_pose_problem.def_readwrite("W", &UnconstrainedEndPoseProblem::W);
    unconstrained_end_pose_problem.def_readonly("PhiN", &UnconstrainedEndPoseProblem::PhiN);
    unconstrained_end_pose_problem.def_readonly("JN", &UnconstrainedEndPoseProblem::JN);
    unconstrained_end_pose_problem.def_readonly("N", &UnconstrainedEndPoseProblem::N);
    unconstrained_end_pose_problem.def_readonly("NumTasks", &UnconstrainedEndPoseProblem::NumTasks);
    unconstrained_end_pose_problem.def_readonly("Phi", &UnconstrainedEndPoseProblem::Phi);
    unconstrained_end_pose_problem.def_readonly("J", &UnconstrainedEndPoseProblem::J);
    unconstrained_end_pose_problem.def_property_readonly("ydiff", [](UnconstrainedEndPoseProblem* prob) { return prob->Cost.ydiff; });
    unconstrained_end_pose_problem.def_property("qNominal", &UnconstrainedEndPoseProblem::getNominalPose, &UnconstrainedEndPoseProblem::setNominalPose);
    unconstrained_end_pose_problem.def("getScalarCost", &UnconstrainedEndPoseProblem::getScalarCost);
    unconstrained_end_pose_problem.def("getScalarJacobian", &UnconstrainedEndPoseProblem::getScalarJacobian);
    unconstrained_end_pose_problem.def("getScalarTaskCost", &UnconstrainedEndPoseProblem::getScalarTaskCost);
    unconstrained_end_pose_problem.def_readonly("Cost", &UnconstrainedEndPoseProblem::Cost);

    py::class_<EndPoseProblem, std::shared_ptr<EndPoseProblem>, PlanningProblem> end_pose_problem(prob, "EndPoseProblem");
    end_pose_problem.def("update", &EndPoseProblem::Update);
    end_pose_problem.def("setGoal", &EndPoseProblem::setGoal);
    end_pose_problem.def("setRho", &EndPoseProblem::setRho);
    end_pose_problem.def("getGoal", &EndPoseProblem::getGoal);
    end_pose_problem.def("getRho", &EndPoseProblem::getRho);
    end_pose_problem.def("setGoalEQ", &EndPoseProblem::setGoalEQ);
    end_pose_problem.def("setRhoEQ", &EndPoseProblem::setRhoEQ);
    end_pose_problem.def("getGoalEQ", &EndPoseProblem::getGoalEQ);
    end_pose_problem.def("getRhoEQ", &EndPoseProblem::getRhoEQ);
    end_pose_problem.def("setGoalNEQ", &EndPoseProblem::setGoalNEQ);
    end_pose_problem.def("setRhoNEQ", &EndPoseProblem::setRhoNEQ);
    end_pose_problem.def("getGoalNEQ", &EndPoseProblem::getGoalNEQ);
    end_pose_problem.def("getRhoNEQ", &EndPoseProblem::getRhoNEQ);
    end_pose_problem.def_readwrite("W", &EndPoseProblem::W);
    end_pose_problem.def_readonly("PhiN", &EndPoseProblem::PhiN);
    end_pose_problem.def_readonly("JN", &EndPoseProblem::JN);
    end_pose_problem.def_readonly("N", &EndPoseProblem::N);
    end_pose_problem.def_readonly("NumTasks", &EndPoseProblem::NumTasks);
    end_pose_problem.def_readonly("Phi", &EndPoseProblem::Phi);
    end_pose_problem.def_readonly("J", &EndPoseProblem::J);
    end_pose_problem.def("getScalarCost", &EndPoseProblem::getScalarCost);
    end_pose_problem.def("getScalarJacobian", &EndPoseProblem::getScalarJacobian);
    end_pose_problem.def("getScalarTaskCost", &EndPoseProblem::getScalarTaskCost);
    end_pose_problem.def("getEquality", &EndPoseProblem::getEquality);
    end_pose_problem.def("getEqualityJacobian", &EndPoseProblem::getEqualityJacobian);
    end_pose_problem.def("getInequality", &EndPoseProblem::getInequality);
    end_pose_problem.def("getInequalityJacobian", &EndPoseProblem::getInequalityJacobian);
    end_pose_problem.def("getBounds", &EndPoseProblem::getBounds);
    end_pose_problem.def_readonly("Cost", &EndPoseProblem::Cost);
    end_pose_problem.def_readonly("Inequality", &EndPoseProblem::Inequality);
    end_pose_problem.def_readonly("Equality", &EndPoseProblem::Equality);

    py::class_<BoundedEndPoseProblem, std::shared_ptr<BoundedEndPoseProblem>, PlanningProblem> bounded_end_pose_problem(prob, "BoundedEndPoseProblem");
    bounded_end_pose_problem.def("update", &BoundedEndPoseProblem::Update);
    bounded_end_pose_problem.def("setGoal", &BoundedEndPoseProblem::setGoal);
    bounded_end_pose_problem.def("setRho", &BoundedEndPoseProblem::setRho);
    bounded_end_pose_problem.def("getGoal", &BoundedEndPoseProblem::getGoal);
    bounded_end_pose_problem.def("getRho", &BoundedEndPoseProblem::getRho);
    bounded_end_pose_problem.def_readwrite("W", &BoundedEndPoseProblem::W);
    bounded_end_pose_problem.def_readonly("PhiN", &BoundedEndPoseProblem::PhiN);
    bounded_end_pose_problem.def_readonly("JN", &BoundedEndPoseProblem::JN);
    bounded_end_pose_problem.def_readonly("N", &BoundedEndPoseProblem::N);
    bounded_end_pose_problem.def_readonly("NumTasks", &BoundedEndPoseProblem::NumTasks);
    bounded_end_pose_problem.def_readonly("Phi", &BoundedEndPoseProblem::Phi);
    bounded_end_pose_problem.def_readonly("J", &BoundedEndPoseProblem::J);
    bounded_end_pose_problem.def("getScalarCost", &BoundedEndPoseProblem::getScalarCost);
    bounded_end_pose_problem.def("getScalarJacobian", &BoundedEndPoseProblem::getScalarJacobian);
    bounded_end_pose_problem.def("getScalarTaskCost", &BoundedEndPoseProblem::getScalarTaskCost);
    bounded_end_pose_problem.def("getBounds", &BoundedEndPoseProblem::getBounds);
    bounded_end_pose_problem.def_readonly("Cost", &BoundedEndPoseProblem::Cost);

    py::class_<SamplingProblem, std::shared_ptr<SamplingProblem>, PlanningProblem> sampling_problem(prob, "SamplingProblem");
    sampling_problem.def("update", &SamplingProblem::Update);
    sampling_problem.def_property("goalState", &SamplingProblem::getGoalState, &SamplingProblem::setGoalState);
    sampling_problem.def("getSpaceDim", &SamplingProblem::getSpaceDim);
    sampling_problem.def("getBounds", &SamplingProblem::getBounds);
    sampling_problem.def_readonly("N", &SamplingProblem::N);
    sampling_problem.def_readonly("NumTasks", &SamplingProblem::NumTasks);
    sampling_problem.def_readonly("Phi", &SamplingProblem::Phi);
    sampling_problem.def_readonly("Inequality", &SamplingProblem::Inequality);
    sampling_problem.def_readonly("Equality", &SamplingProblem::Equality);
    sampling_problem.def("setGoalEQ", &SamplingProblem::setGoalEQ);
    sampling_problem.def("setRhoEQ", &SamplingProblem::setRhoEQ);
    sampling_problem.def("getGoalEQ", &SamplingProblem::getGoalEQ);
    sampling_problem.def("getRhoEQ", &SamplingProblem::getRhoEQ);
    sampling_problem.def("setGoalNEQ", &SamplingProblem::setGoalNEQ);
    sampling_problem.def("setRhoNEQ", &SamplingProblem::setRhoNEQ);
    sampling_problem.def("getGoalNEQ", &SamplingProblem::getGoalNEQ);
    sampling_problem.def("getRhoNEQ", &SamplingProblem::getRhoNEQ);

    py::class_<TimeIndexedSamplingProblem, std::shared_ptr<TimeIndexedSamplingProblem>, PlanningProblem> time_indexed_sampling_problem(prob, "TimeIndexedSamplingProblem");
    time_indexed_sampling_problem.def("update", &TimeIndexedSamplingProblem::Update);
    time_indexed_sampling_problem.def("getSpaceDim", &TimeIndexedSamplingProblem::getSpaceDim);
    time_indexed_sampling_problem.def("getBounds", &TimeIndexedSamplingProblem::getBounds);
    time_indexed_sampling_problem.def_property("goalState", &TimeIndexedSamplingProblem::getGoalState, &TimeIndexedSamplingProblem::setGoalState);
    time_indexed_sampling_problem.def_property("goalTime", &TimeIndexedSamplingProblem::getGoalTime, &TimeIndexedSamplingProblem::setGoalTime);
    time_indexed_sampling_problem.def_readonly("N", &TimeIndexedSamplingProblem::N);
    time_indexed_sampling_problem.def_readonly("NumTasks", &TimeIndexedSamplingProblem::NumTasks);
    time_indexed_sampling_problem.def_readonly("Phi", &TimeIndexedSamplingProblem::Phi);
    time_indexed_sampling_problem.def_readonly("Inequality", &TimeIndexedSamplingProblem::Inequality);
    time_indexed_sampling_problem.def_readonly("Equality", &TimeIndexedSamplingProblem::Equality);
    time_indexed_sampling_problem.def("setGoalEQ", &TimeIndexedSamplingProblem::setGoalEQ);
    time_indexed_sampling_problem.def("setRhoEQ", &TimeIndexedSamplingProblem::setRhoEQ);
    time_indexed_sampling_problem.def("getGoalEQ", &TimeIndexedSamplingProblem::getGoalEQ);
    time_indexed_sampling_problem.def("getRhoEQ", &TimeIndexedSamplingProblem::getRhoEQ);
    time_indexed_sampling_problem.def("setGoalNEQ", &TimeIndexedSamplingProblem::setGoalNEQ);
    time_indexed_sampling_problem.def("setRhoNEQ", &TimeIndexedSamplingProblem::setRhoNEQ);
    time_indexed_sampling_problem.def("getGoalNEQ", &TimeIndexedSamplingProblem::getGoalNEQ);
    time_indexed_sampling_problem.def("getRhoNEQ", &TimeIndexedSamplingProblem::getRhoNEQ);

    py::class_<CollisionProxy, std::shared_ptr<CollisionProxy>> collision_proxy(module, "CollisionProxy");
    collision_proxy.def(py::init());
    collision_proxy.def_readonly("Contact1", &CollisionProxy::contact1);
    collision_proxy.def_readonly("Contact2", &CollisionProxy::contact2);
    collision_proxy.def_readonly("Normal1", &CollisionProxy::normal1);
    collision_proxy.def_readonly("Normal2", &CollisionProxy::normal2);
    collision_proxy.def_readonly("Distance", &CollisionProxy::distance);
    collision_proxy.def_property_readonly("Object1", [](CollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e1->Segment.getName() : std::string(""); });
    collision_proxy.def_property_readonly("Object2", [](CollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e2->Segment.getName() : std::string(""); });
    collision_proxy.def_property_readonly("Transform1", [](CollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e1->Frame : KDL::Frame(); });
    collision_proxy.def_property_readonly("Transform2", [](CollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e2->Frame : KDL::Frame(); });
    collision_proxy.def("__repr__", &CollisionProxy::print);

    py::class_<ContinuousCollisionProxy, std::shared_ptr<ContinuousCollisionProxy>> continuous_collision_proxy(module, "ContinuousCollisionProxy");
    continuous_collision_proxy.def(py::init());
    continuous_collision_proxy.def_readonly("ContactTransform1", &ContinuousCollisionProxy::contact_tf1);
    continuous_collision_proxy.def_readonly("ContactTransform2", &ContinuousCollisionProxy::contact_tf2);
    continuous_collision_proxy.def_readonly("InCollision", &ContinuousCollisionProxy::in_collision);
    continuous_collision_proxy.def_readonly("TimeOfContact", &ContinuousCollisionProxy::time_of_contact);
    continuous_collision_proxy.def_property_readonly("Object1", [](ContinuousCollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e1->Segment.getName() : std::string(""); });
    continuous_collision_proxy.def_property_readonly("Object2", [](ContinuousCollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e2->Segment.getName() : std::string(""); });
    continuous_collision_proxy.def_property_readonly("Transform1", [](ContinuousCollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e1->Frame : KDL::Frame(); });
    continuous_collision_proxy.def_property_readonly("Transform2", [](ContinuousCollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e2->Frame : KDL::Frame(); });
    continuous_collision_proxy.def("__repr__", &ContinuousCollisionProxy::print);

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
              py::arg("offsetTransform") = kdl_frame(),
              py::arg("updateCollisionScene") = true);
    scene.def("loadSceneFile",
              (void (Scene::*)(const std::string&, const KDL::Frame&, bool)) & Scene::loadSceneFile,
              py::arg("fileName"),
              py::arg("offsetTransform") = kdl_frame(),
              py::arg("updateCollisionScene") = true);
    scene.def("getScene", &Scene::getScene);
    scene.def("cleanScene", &Scene::cleanScene);
    scene.def("isStateValid", [](Scene* instance, bool self, double safe_distance) { return instance->getCollisionScene()->isStateValid(self, safe_distance); }, py::arg("check_self_collision") = true, py::arg("SafeDistance") = 0.0);
    scene.def("isCollisionFree", [](Scene* instance, const std::string& o1, const std::string& o2, double safe_distance) { return instance->getCollisionScene()->isCollisionFree(o1, o2, safe_distance); }, py::arg("Object1"), py::arg("Object2"), py::arg("SafeDistance") = 0.0);
    scene.def("isAllowedToCollide", [](Scene* instance, const std::string& o1, const std::string& o2, bool self) { return instance->getCollisionScene()->isAllowedToCollide(o1, o2, self); }, py::arg("Object1"), py::arg("Object2"), py::arg("check_self_collision") = true);
    scene.def("getCollisionDistance", [](Scene* instance, bool self) { return instance->getCollisionScene()->getCollisionDistance(self); }, py::arg("check_self_collision") = true);
    scene.def("getCollisionDistance", [](Scene* instance, const std::string& o1, const std::string& o2) { return instance->getCollisionScene()->getCollisionDistance(o1, o2); }, py::arg("Object1"), py::arg("Object2"));
    scene.def("getCollisionDistance",
              [](Scene* instance, const std::string& o1, const bool& self) {
                  return instance->getCollisionScene()->getCollisionDistance(o1, self);
              },
              py::arg("Object1"), py::arg("check_self_collision") = true);
    scene.def("getCollisionDistance",
              [](Scene* instance, const std::vector<std::string>& objects, const bool& self) {
                  return instance->getCollisionScene()->getCollisionDistance(objects, self);
              },
              py::arg("objects"), py::arg("check_self_collision") = true);
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
    scene.def("addObject", (void (Scene::*)(const std::string&, const KDL::Frame&, const std::string&, shapes::ShapeConstPtr, const KDL::RigidBodyInertia&, bool)) & Scene::addObject,
              py::arg("name"),
              py::arg("transform") = KDL::Frame(),
              py::arg("parent") = std::string(),
              py::arg("shape"),
              py::arg("inertia") = KDL::RigidBodyInertia::Zero(),
              py::arg("updateCollisionScene") = true);
    scene.def("addObjectToEnvironment", &Scene::addObjectToEnvironment,
              py::arg("name"),
              py::arg("transform") = KDL::Frame(),
              py::arg("shape"),
              py::arg("updateCollisionScene") = true);
    scene.def("removeObject", &Scene::removeObject);
    scene.def_property_readonly("modelLinkToCollisionLinkMap", &Scene::getModelLinkToCollisionLinkMap);
    scene.def_property_readonly("controlledLinkToCollisionLinkMap", &Scene::getControlledLinkToCollisionLinkMap);

    py::class_<CollisionScene, std::shared_ptr<CollisionScene>> collision_scene(module, "CollisionScene");
    // TODO: expose isStateValid, isCollisionFree, getCollisionDistance, getCollisionWorldLinks, getCollisionRobotLinks, getTranslation
    collision_scene.def_property("alwaysExternallyUpdatedCollisionScene", &CollisionScene::getAlwaysExternallyUpdatedCollisionScene, &CollisionScene::setAlwaysExternallyUpdatedCollisionScene);
    collision_scene.def_property("replacePrimitiveShapesWithMeshes", &CollisionScene::getReplacePrimitiveShapesWithMeshes, &CollisionScene::setReplacePrimitiveShapesWithMeshes);
    collision_scene.def_readwrite("replaceCylindersWithCapsules", &CollisionScene::replaceCylindersWithCapsules);
    collision_scene.def_property("robotLinkScale", &CollisionScene::getRobotLinkScale, &CollisionScene::setRobotLinkScale);
    collision_scene.def_property("worldLinkScale", &CollisionScene::getWorldLinkScale, &CollisionScene::setWorldLinkScale);
    collision_scene.def_property("robotLinkPadding", &CollisionScene::getRobotLinkPadding, &CollisionScene::setRobotLinkPadding);
    collision_scene.def_property("worldLinkPadding", &CollisionScene::getWorldLinkPadding, &CollisionScene::setWorldLinkPadding);
    collision_scene.def("updateCollisionObjectTransforms", &CollisionScene::updateCollisionObjectTransforms);
    collision_scene.def("continuousCollisionCheck", &CollisionScene::continuousCollisionCheck);

    py::class_<Visualization> visualization(module, "Visualization");
    visualization.def(py::init<Scene_ptr>());
    visualization.def("displayTrajectory", &Visualization::displayTrajectory);

    py::module kin = module.def_submodule("Kinematics", "Kinematics submodule.");
    py::class_<KinematicTree, std::shared_ptr<KinematicTree>> kinematic_tree(kin, "KinematicTree");
    kinematic_tree.def_readwrite("debugMode", &KinematicTree::Debug);
    kinematic_tree.def("publishFrames", &KinematicTree::publishFrames);
    kinematic_tree.def("getRootFrameName", &KinematicTree::getRootFrameName);
    kinematic_tree.def("getRootJointName", &KinematicTree::getRootJointName);
    kinematic_tree.def("getModelBaseType", &KinematicTree::getModelBaseType);
    kinematic_tree.def("getControlledBaseType", &KinematicTree::getControlledBaseType);
    kinematic_tree.def("getControlledLinkMass", &KinematicTree::getControlledLinkMass);
    kinematic_tree.def("getCollisionObjectTypes", &KinematicTree::getCollisionObjectTypes);
    kinematic_tree.def("getRandomControlledState", &KinematicTree::getRandomControlledState);
    kinematic_tree.def("getNumModelJoints", &KinematicTree::getNumModelJoints);
    kinematic_tree.def("getNumControlledJoints", &KinematicTree::getNumControlledJoints);

    // Joint Limits
    kinematic_tree.def("getJointLimits", &KinematicTree::getJointLimits);
    kinematic_tree.def("resetJointLimits", &KinematicTree::resetJointLimits);
    kinematic_tree.def("setJointLimitsLower", &KinematicTree::setJointLimitsLower);
    kinematic_tree.def("setJointLimitsUpper", &KinematicTree::setJointLimitsUpper);
    kinematic_tree.def("setFloatingBaseLimitsPosXYZEulerZYX", &KinematicTree::setFloatingBaseLimitsPosXYZEulerZYX);
    kinematic_tree.def("setPlanarBaseLimitsPosXYEulerZ", &KinematicTree::setPlanarBaseLimitsPosXYEulerZ);
    kinematic_tree.def("getUsedJointLimits", &KinematicTree::getUsedJointLimits);

    // TODO: KinematicRequestFlags

    // TODO: KinematicFrame

    // KinematicResponse
    py::class_<KinematicResponse, std::shared_ptr<KinematicResponse>> kinematic_response(kin, "KinematicResponse");
    kinematic_response.def_property_readonly("Phi", [](KinematicResponse* instance) {
        std::vector<KDL::Frame> vec;
        for (unsigned int i = 0; i < instance->Phi.cols(); i++)
            vec.push_back(instance->Phi(i));
        return vec;
    });

    ////////////////////////////////////////////////////////////////////////////
    /// Shapes

    // shape base class
    py::class_<shapes::Shape, shapes::ShapePtr>(module, "Shape")
        .def("scale", &shapes::Shape::scale)
        .def("padd", &shapes::Shape::padd)
        .def("scaleAndPadd", &shapes::Shape::scaleAndPadd)
        .def("isFixed", &shapes::Shape::isFixed)
        .def_readonly("type", &shapes::Shape::type);

    py::class_<shapes::Sphere, shapes::Shape, std::shared_ptr<shapes::Sphere>>(module, "Sphere")
        .def(py::init())
        .def(py::init<double>())
        .def_readonly_static("name", &shapes::Sphere::STRING_NAME)
        .def("scaleAndPadd", &shapes::Sphere::scaleAndPadd)
        .def_readwrite("radius", &shapes::Sphere::radius);

    py::class_<shapes::Cylinder, shapes::Shape, std::shared_ptr<shapes::Cylinder>>(module, "Cylinder")
        .def(py::init())
        .def(py::init<double, double>())
        .def_readonly_static("name", &shapes::Cylinder::STRING_NAME)
        .def("scaleAndPadd", &shapes::Cylinder::scaleAndPadd)
        .def_readwrite("radius", &shapes::Cylinder::radius)
        .def_readwrite("length", &shapes::Cylinder::length);

    py::class_<shapes::Cone, shapes::Shape, std::shared_ptr<shapes::Cone>>(module, "Cone")
        .def(py::init())
        .def(py::init<double, double>())
        .def_readonly_static("name", &shapes::Cone::STRING_NAME)
        .def("scaleAndPadd", &shapes::Cone::scaleAndPadd)
        .def_readwrite("radius", &shapes::Cone::radius)
        .def_readwrite("length", &shapes::Cone::length);

    py::class_<shapes::Box, shapes::Shape, std::shared_ptr<shapes::Box>>(module, "Box")
        .def(py::init())
        .def(py::init<double, double, double>())
        .def_readonly_static("name", &shapes::Box::STRING_NAME)
        .def("scaleAndPadd", &shapes::Box::scaleAndPadd);

    py::class_<shapes::Plane, shapes::Shape, std::shared_ptr<shapes::Plane>>(module, "Plane")
        .def(py::init())
        .def(py::init<double, double, double, double>())
        .def_readonly_static("name", &shapes::Plane::STRING_NAME)
        .def("scaleAndPadd", &shapes::Plane::scaleAndPadd)
        .def("isFixed", &shapes::Plane::isFixed)
        .def_readwrite("a", &shapes::Plane::a)
        .def_readwrite("b", &shapes::Plane::b)
        .def_readwrite("c", &shapes::Plane::c)
        .def_readwrite("d", &shapes::Plane::d);

    py::enum_<shapes::ShapeType>(module, "ShapeType")
        .value("UNKNOWN_SHAPE", shapes::ShapeType::UNKNOWN_SHAPE)
        .value("SPHERE", shapes::ShapeType::SPHERE)
        .value("CYLINDER", shapes::ShapeType::CYLINDER)
        .value("CONE", shapes::ShapeType::CONE)
        .value("BOX", shapes::ShapeType::BOX)
        .value("PLANE", shapes::ShapeType::PLANE)
        .value("MESH", shapes::ShapeType::MESH)
        .value("OCTREE", shapes::ShapeType::OCTREE)
        .export_values();

    module.attr("version") = std::string(exotica::Version);
    module.attr("branch") = std::string(exotica::Branch);

    AddInitializers(module);

    auto cleanup_exotica = []() {
        Setup::Destroy();
    };
    module.add_object("_cleanup", py::capsule(cleanup_exotica));
}

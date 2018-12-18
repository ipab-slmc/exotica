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

    inits.def("load_xml", (Initializer(*)(std::string, bool)) & XMLLoader::load, "Loads initializer from XML", py::arg("xml"), py::arg("parseAsXMLString") = false);
    inits.def("load_xml_full", &LoadFromXML, "Loads initializer from XML", py::arg("xml"), py::arg("solver_name") = std::string(""), py::arg("problem_name") = std::string(""), py::arg("parseAsXMLString") = false);
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
    setup.def_static("get_solvers", &Setup::getSolvers, "Returns a list of available solvers.");
    setup.def_static("get_problems", &Setup::getProblems, "Returns a list of available problems.");
    setup.def_static("get_maps", &Setup::getMaps, "Returns a list of available task maps.");
    setup.def_static("get_collision_scenes", &Setup::getCollisionScenes, "Returns a list of available collision scene plug-ins.");
    setup.def_static("create_solver", [](const Initializer& init) { return Setup::createSolver(init); }, py::return_value_policy::take_ownership);    // "Creates an instance of the solver identified by name parameter.", py::arg("solverType"), py::arg("prependExoticaNamespace"));
    setup.def_static("create_problem", [](const Initializer& init) { return Setup::createProblem(init); }, py::return_value_policy::take_ownership);  // "Creates an instance of the problem identified by name parameter.", py::arg("problemType"), py::arg("prependExoticaNamespace"));
    setup.def_static("create_map", [](const Initializer& init) { return Setup::createMap(init); }, py::return_value_policy::take_ownership);          // "Creates an instance of the task map identified by name parameter.", py::arg("taskmapType"), py::arg("prependExoticaNamespace"));
    setup.def_static("print_supported_classes", &Setup::printSupportedClasses, "Print a list of available plug-ins sorted by class.");
    setup.def_static("get_initializers", &Setup::getInitializers, py::return_value_policy::copy, "Returns a list of available initializers with all available parameters/arguments.");
    setup.def_static("get_package_path", &ros::package::getPath, "ROS package path resolution.");
    setup.def_static("init_ros",
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
    setup.def_static("load_solver", &XMLLoader::loadSolver, "Instantiate solver and problem from an XML file containing both a solver and problem initializer.", py::arg("filepath"));
    setup.def_static("load_solver_standalone", &XMLLoader::loadSolverStandalone, "Instantiate only a solver from an XML file containing solely a solver initializer.", py::arg("filepath"));
    setup.def_static("load_problem", &XMLLoader::loadProblem, "Instantiate only a problem from an XML file containing solely a problem initializer.", py::arg("filepath"));

    py::module tools = module.def_submodule("Tools");
    tools.def("parse_path", &parsePath);
    tools.def("parse_bool", &parseBool);
    tools.def("parse_double", &parseDouble);
    tools.def("parse_vector", &parseVector<double, Eigen::Dynamic>);
    tools.def("parse_list", &parseList);
    tools.def("parse_int", &parseInt);
    tools.def("parse_int_list", &parseIntList);
    tools.def("load_obj", [](const std::string& path) { Eigen::VectorXi tri; Eigen::VectorXd vert; loadOBJ(loadFile(path), tri, vert); return py::make_tuple(tri, vert); });
    tools.def("get_text", &getText);
    tools.def("save_matrix", &saveMatrix);
    tools.def("VectorTransform", &Eigen::VectorTransform);
    tools.def("IdentityTransform", &Eigen::IdentityTransform);
    tools.def("load_file", &loadFile);
    tools.def("path_exists", &pathExists);
    tools.def("create_composite_trajectory", [](Eigen::MatrixXdRefConst data, double radius) {
        return Trajectory(data, radius).toString();
    },
              py::arg("data"), py::arg("max_radius") = 1.0);

    py::class_<Timer, std::shared_ptr<Timer>> timer(module, "Timer");
    timer.def(py::init());
    timer.def("reset", &Timer::reset);
    timer.def("get_duration", &Timer::getDuration);

    py::class_<Object, std::shared_ptr<Object>> object(module, "Object");
    object.def_property_readonly("type", &Object::type, "Object type");
    object.def_property_readonly("name", &Object::getObjectName, "Object name");
    object.def("__repr__", &Object::print, "String representation of the object", py::arg("prepend") = std::string(""));
    object.def_readwrite("namespace", &Object::ns_);
    object.def_readwrite("debug_mode", &Object::debug_);

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
    kdl_frame.def("get_rpy", [](KDL::Frame* me) { return getRotationAsVector(*me, RotationType::RPY); });
    kdl_frame.def("get_zyz", [](KDL::Frame* me) { return getRotationAsVector(*me, RotationType::ZYZ); });
    kdl_frame.def("get_zyx", [](KDL::Frame* me) { return getRotationAsVector(*me, RotationType::ZYX); });
    kdl_frame.def("get_angle_axis", [](KDL::Frame* me) { return getRotationAsVector(*me, RotationType::ANGLE_AXIS); });
    kdl_frame.def("get_quaternion", [](KDL::Frame* me) { return getRotationAsVector(*me, RotationType::QUATERNION); });
    kdl_frame.def("get_translation", [](KDL::Frame* me) { Eigen::Vector3d tmp; for (int i = 0; i < 3; i++) { tmp[i] = me->p.data[i]; } return tmp; });
    kdl_frame.def("get_translation_and_rpy", [](KDL::Frame* me) { return getFrameAsVector(*me, RotationType::RPY); });
    kdl_frame.def("get_translation_and_zyz", [](KDL::Frame* me) { return getFrameAsVector(*me, RotationType::ZYZ); });
    kdl_frame.def("get_translation_and_zyx", [](KDL::Frame* me) { return getFrameAsVector(*me, RotationType::ZYX); });
    kdl_frame.def("get_translation_and_angle_axis", [](KDL::Frame* me) { return getFrameAsVector(*me, RotationType::ANGLE_AXIS); });
    kdl_frame.def("get_translation_and_quaternion", [](KDL::Frame* me) { return getFrameAsVector(*me, RotationType::QUATERNION); });
    kdl_frame.def("get_frame", [](KDL::Frame* me) { return getFrame(*me); });
    kdl_frame.def("inverse", (KDL::Frame(KDL::Frame::*)() const) & KDL::Frame::Inverse);
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
    task_map.def("task_space_dim", (int (TaskMap::*)()) & TaskMap::taskSpaceDim);
    task_map.def("task_Space_jacobian_dim", &TaskMap::taskSpaceJacobianDim);

    py::class_<TimeIndexedTask, std::shared_ptr<TimeIndexedTask>> time_indexed_task(module, "TimeIndexedTask");
    time_indexed_task.def_readonly("PhiN", &TimeIndexedTask::PhiN);
    time_indexed_task.def_readonly("JN", &TimeIndexedTask::JN);
    time_indexed_task.def_readonly("num_tasks", &TimeIndexedTask::NumTasks);
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
    end_pose_task.def_readonly("num_tasks", &EndPoseTask::NumTasks);
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
    sampling_task.def_readonly("num_tasks", &SamplingTask::NumTasks);
    sampling_task.def_readonly("y", &SamplingTask::y);
    sampling_task.def_readonly("ydiff", &SamplingTask::ydiff);
    sampling_task.def_readonly("Phi", &SamplingTask::Phi);
    sampling_task.def_readonly("S", &SamplingTask::S);
    sampling_task.def_readonly("Tasks", &SamplingTask::Tasks);
    sampling_task.def_readonly("TaskMaps", &SamplingTask::TaskMaps);

    py::class_<TaskSpaceVector, std::shared_ptr<TaskSpaceVector>> task_space_vector(module, "TaskSpaceVector");
    task_space_vector.def("set_zero", &TaskSpaceVector::setZero);
    task_space_vector.def_readonly("data", &TaskSpaceVector::data);
    task_space_vector.def("__sub__", &TaskSpaceVector::operator-, py::is_operator());
    task_space_vector.def("__repr__", [](TaskSpaceVector* instance) { return ((std::ostringstream&)(std::ostringstream("") << "TaskSpaceVector (" << instance->data.transpose() << ")")).str(); });

    py::class_<MotionSolver, std::shared_ptr<MotionSolver>, Object> motion_solver(module, "MotionSolver");
    motion_solver.def_property("max_iterations", &MotionSolver::getNumberOfMaxIterations, &MotionSolver::setNumberOfMaxIterations);
    motion_solver.def("get_planning_time", &MotionSolver::getPlanningTime);
    motion_solver.def("specify_problem", &MotionSolver::specifyProblem, "Assign problem to the solver", py::arg("planning_problem"));
    motion_solver.def(
        "solve", [](std::shared_ptr<MotionSolver> sol) {
            Eigen::MatrixXd ret;
            sol->Solve(ret);
            return ret;
        },
        "Solve the problem");
    motion_solver.def("get_problem", &MotionSolver::getProblem, py::return_value_policy::reference_internal);

    py::class_<PlanningProblem, std::shared_ptr<PlanningProblem>, Object> planning_problem(module, "PlanningProblem");
    planning_problem.def("get_tasks", &PlanningProblem::getTasks, py::return_value_policy::reference_internal);
    planning_problem.def("get_task_maps", &PlanningProblem::getTaskMaps, py::return_value_policy::reference_internal);
    planning_problem.def("get_scene", &PlanningProblem::getScene, py::return_value_policy::reference_internal);
    planning_problem.def("__repr__", &PlanningProblem::print, "String representation of the object", py::arg("prepend") = std::string(""));
    planning_problem.def_property("start_state", &PlanningProblem::getStartState, &PlanningProblem::setStartState);
    planning_problem.def_property("start_time", &PlanningProblem::getStartTime, &PlanningProblem::setStartTime);
    planning_problem.def("get_number_of_problem_updates", &PlanningProblem::getNumberOfProblemUpdates);
    planning_problem.def("reset_number_of_problem_updates", &PlanningProblem::resetNumberOfProblemUpdates);
    planning_problem.def("get_cost_evolution", (std::pair<std::vector<double>, std::vector<double>>(PlanningProblem::*)()) & PlanningProblem::getCostEvolution);
    planning_problem.def("is_valid", &PlanningProblem::isValid);

    // Problem types
    py::module prob = module.def_submodule("Problems", "Problem types");

    py::class_<UnconstrainedTimeIndexedProblem, std::shared_ptr<UnconstrainedTimeIndexedProblem>, PlanningProblem> unconstrained_time_indexed_problem(prob, "UnconstrainedTimeIndexedProblem");
    unconstrained_time_indexed_problem.def("get_duration", &UnconstrainedTimeIndexedProblem::getDuration);
    unconstrained_time_indexed_problem.def("update", &UnconstrainedTimeIndexedProblem::Update);
    unconstrained_time_indexed_problem.def("set_goal", &UnconstrainedTimeIndexedProblem::setGoal);
    unconstrained_time_indexed_problem.def("set_rho", &UnconstrainedTimeIndexedProblem::setRho);
    unconstrained_time_indexed_problem.def("get_goal", &UnconstrainedTimeIndexedProblem::getGoal);
    unconstrained_time_indexed_problem.def("get_rho", &UnconstrainedTimeIndexedProblem::getRho);
    unconstrained_time_indexed_problem.def_property("tau", &UnconstrainedTimeIndexedProblem::getTau, &UnconstrainedTimeIndexedProblem::setTau);
    unconstrained_time_indexed_problem.def_readwrite("W", &UnconstrainedTimeIndexedProblem::W);
    unconstrained_time_indexed_problem.def_property("initial_trajectory", &UnconstrainedTimeIndexedProblem::getInitialTrajectory, &UnconstrainedTimeIndexedProblem::setInitialTrajectory);
    unconstrained_time_indexed_problem.def_property("T", &UnconstrainedTimeIndexedProblem::getT, &UnconstrainedTimeIndexedProblem::setT);
    unconstrained_time_indexed_problem.def_readonly("PhiN", &UnconstrainedTimeIndexedProblem::PhiN);
    unconstrained_time_indexed_problem.def_readonly("JN", &UnconstrainedTimeIndexedProblem::JN);
    unconstrained_time_indexed_problem.def_readonly("N", &UnconstrainedTimeIndexedProblem::N);
    unconstrained_time_indexed_problem.def_readonly("num_tasks", &UnconstrainedTimeIndexedProblem::NumTasks);
    unconstrained_time_indexed_problem.def_readonly("Phi", &UnconstrainedTimeIndexedProblem::Phi);
    unconstrained_time_indexed_problem.def_readonly("J", &UnconstrainedTimeIndexedProblem::J);
    unconstrained_time_indexed_problem.def("get_scalar_task_cost", &UnconstrainedTimeIndexedProblem::getScalarTaskCost);
    unconstrained_time_indexed_problem.def("get_scalar_task_jacobian", &UnconstrainedTimeIndexedProblem::getScalarTaskJacobian);
    unconstrained_time_indexed_problem.def("get_scalar_transition_cost", &UnconstrainedTimeIndexedProblem::getScalarTransitionCost);
    unconstrained_time_indexed_problem.def("get_scalar_transition_jacobian", &UnconstrainedTimeIndexedProblem::getScalarTransitionJacobian);
    unconstrained_time_indexed_problem.def_readonly("Cost", &UnconstrainedTimeIndexedProblem::Cost);
    unconstrained_time_indexed_problem.def_property_readonly("kinematic_solutions", &UnconstrainedTimeIndexedProblem::getKinematicSolutions);

    py::class_<TimeIndexedProblem, std::shared_ptr<TimeIndexedProblem>, PlanningProblem> time_indexed_problem(prob, "TimeIndexedProblem");
    time_indexed_problem.def("get_duration", &TimeIndexedProblem::getDuration);
    time_indexed_problem.def("update", &TimeIndexedProblem::Update);
    time_indexed_problem.def("set_goal", &TimeIndexedProblem::setGoal);
    time_indexed_problem.def("set_rho", &TimeIndexedProblem::setRho);
    time_indexed_problem.def("get_goal", &TimeIndexedProblem::getGoal);
    time_indexed_problem.def("get_rho", &TimeIndexedProblem::getRho);
    time_indexed_problem.def("set_goal_eq", &TimeIndexedProblem::setGoalEQ);
    time_indexed_problem.def("set_rho_eq", &TimeIndexedProblem::setRhoEQ);
    time_indexed_problem.def("get_goal_eq", &TimeIndexedProblem::getGoalEQ);
    time_indexed_problem.def("get_rho_eq", &TimeIndexedProblem::getRhoEQ);
    time_indexed_problem.def("set_goal_neq", &TimeIndexedProblem::setGoalNEQ);
    time_indexed_problem.def("set_rho_neq", &TimeIndexedProblem::setRhoNEQ);
    time_indexed_problem.def("get_goal_neq", &TimeIndexedProblem::getGoalNEQ);
    time_indexed_problem.def("get_rho_neq", &TimeIndexedProblem::getRhoNEQ);
    time_indexed_problem.def_property("tau", &TimeIndexedProblem::getTau, &TimeIndexedProblem::setTau);
    time_indexed_problem.def_property("q_dot_max", &TimeIndexedProblem::getJointVelocityLimit, &TimeIndexedProblem::setJointVelocityLimit);
    time_indexed_problem.def_readwrite("W", &TimeIndexedProblem::W);
    time_indexed_problem.def_property("initial_trajectory", &TimeIndexedProblem::getInitialTrajectory, &TimeIndexedProblem::setInitialTrajectory);
    time_indexed_problem.def_property("T", &TimeIndexedProblem::getT, &TimeIndexedProblem::setT);
    time_indexed_problem.def_readonly("PhiN", &TimeIndexedProblem::PhiN);
    time_indexed_problem.def_readonly("JN", &TimeIndexedProblem::JN);
    time_indexed_problem.def_readonly("N", &TimeIndexedProblem::N);
    time_indexed_problem.def_readonly("num_tasks", &TimeIndexedProblem::NumTasks);
    time_indexed_problem.def_readonly("Phi", &TimeIndexedProblem::Phi);
    time_indexed_problem.def_readonly("J", &TimeIndexedProblem::J);
    time_indexed_problem.def("get_scalar_task_cost", &TimeIndexedProblem::getScalarTaskCost);
    time_indexed_problem.def("get_scalar_task_jacobian", &TimeIndexedProblem::getScalarTaskJacobian);
    time_indexed_problem.def("get_scalar_transition_cost", &TimeIndexedProblem::getScalarTransitionCost);
    time_indexed_problem.def("get_scalar_transition_jacobian", &TimeIndexedProblem::getScalarTransitionJacobian);
    time_indexed_problem.def("get_equality", &TimeIndexedProblem::getEquality);
    time_indexed_problem.def("get_equality_jacobian", &TimeIndexedProblem::getEqualityJacobian);
    time_indexed_problem.def("get_inequality", &TimeIndexedProblem::getInequality);
    time_indexed_problem.def("get_inequality_jacobian", &TimeIndexedProblem::getInequalityJacobian);
    time_indexed_problem.def("get_bounds", &TimeIndexedProblem::getBounds);
    time_indexed_problem.def_readonly("Cost", &TimeIndexedProblem::Cost);
    time_indexed_problem.def_readonly("Inequality", &TimeIndexedProblem::Inequality);
    time_indexed_problem.def_readonly("Equality", &TimeIndexedProblem::Equality);

    py::class_<BoundedTimeIndexedProblem, std::shared_ptr<BoundedTimeIndexedProblem>, PlanningProblem> bounded_time_indexed_problem(prob, "BoundedTimeIndexedProblem");
    bounded_time_indexed_problem.def("get_duration", &BoundedTimeIndexedProblem::getDuration);
    bounded_time_indexed_problem.def("update", &BoundedTimeIndexedProblem::Update);
    bounded_time_indexed_problem.def("set_goal", &BoundedTimeIndexedProblem::setGoal);
    bounded_time_indexed_problem.def("set_rho", &BoundedTimeIndexedProblem::setRho);
    bounded_time_indexed_problem.def("get_goal", &BoundedTimeIndexedProblem::getGoal);
    bounded_time_indexed_problem.def("get_rho", &BoundedTimeIndexedProblem::getRho);
    bounded_time_indexed_problem.def_property("tau", &BoundedTimeIndexedProblem::getTau, &BoundedTimeIndexedProblem::setTau);
    bounded_time_indexed_problem.def_readwrite("W", &BoundedTimeIndexedProblem::W);
    bounded_time_indexed_problem.def_property("initial_trajectory", &BoundedTimeIndexedProblem::getInitialTrajectory, &BoundedTimeIndexedProblem::setInitialTrajectory);
    bounded_time_indexed_problem.def_property("T", &BoundedTimeIndexedProblem::getT, &BoundedTimeIndexedProblem::setT);
    bounded_time_indexed_problem.def_readonly("PhiN", &BoundedTimeIndexedProblem::PhiN);
    bounded_time_indexed_problem.def_readonly("JN", &BoundedTimeIndexedProblem::JN);
    bounded_time_indexed_problem.def_readonly("N", &BoundedTimeIndexedProblem::N);
    bounded_time_indexed_problem.def_readonly("num_tasks", &BoundedTimeIndexedProblem::NumTasks);
    bounded_time_indexed_problem.def_readonly("Phi", &BoundedTimeIndexedProblem::Phi);
    bounded_time_indexed_problem.def_readonly("J", &BoundedTimeIndexedProblem::J);
    bounded_time_indexed_problem.def("get_scalar_task_cost", &BoundedTimeIndexedProblem::getScalarTaskCost);
    bounded_time_indexed_problem.def("get_scalar_task_jacobian", &BoundedTimeIndexedProblem::getScalarTaskJacobian);
    bounded_time_indexed_problem.def("get_scalar_transition_cost", &BoundedTimeIndexedProblem::getScalarTransitionCost);
    bounded_time_indexed_problem.def("get_scalar_transition_jacobian", &BoundedTimeIndexedProblem::getScalarTransitionJacobian);
    bounded_time_indexed_problem.def("get_bounds", &BoundedTimeIndexedProblem::getBounds);
    bounded_time_indexed_problem.def_readonly("Cost", &BoundedTimeIndexedProblem::Cost);

    py::class_<UnconstrainedEndPoseProblem, std::shared_ptr<UnconstrainedEndPoseProblem>, PlanningProblem> unconstrained_end_pose_problem(prob, "UnconstrainedEndPoseProblem");
    unconstrained_end_pose_problem.def("update", &UnconstrainedEndPoseProblem::Update);
    unconstrained_end_pose_problem.def("set_goal", &UnconstrainedEndPoseProblem::setGoal);
    unconstrained_end_pose_problem.def("set_rho", &UnconstrainedEndPoseProblem::setRho);
    unconstrained_end_pose_problem.def("get_goal", &UnconstrainedEndPoseProblem::getGoal);
    unconstrained_end_pose_problem.def("get_rho", &UnconstrainedEndPoseProblem::getRho);
    unconstrained_end_pose_problem.def_readwrite("W", &UnconstrainedEndPoseProblem::W);
    unconstrained_end_pose_problem.def_readonly("PhiN", &UnconstrainedEndPoseProblem::PhiN);
    unconstrained_end_pose_problem.def_readonly("JN", &UnconstrainedEndPoseProblem::JN);
    unconstrained_end_pose_problem.def_readonly("N", &UnconstrainedEndPoseProblem::N);
    unconstrained_end_pose_problem.def_readonly("num_tasks", &UnconstrainedEndPoseProblem::NumTasks);
    unconstrained_end_pose_problem.def_readonly("Phi", &UnconstrainedEndPoseProblem::Phi);
    unconstrained_end_pose_problem.def_readonly("J", &UnconstrainedEndPoseProblem::J);
    unconstrained_end_pose_problem.def_property_readonly("ydiff", [](UnconstrainedEndPoseProblem* prob) { return prob->Cost.ydiff; });
    unconstrained_end_pose_problem.def_property("q_nominal", &UnconstrainedEndPoseProblem::getNominalPose, &UnconstrainedEndPoseProblem::setNominalPose);
    unconstrained_end_pose_problem.def("get_scalar_cost", &UnconstrainedEndPoseProblem::getScalarCost);
    unconstrained_end_pose_problem.def("get_scalar_jacobian", &UnconstrainedEndPoseProblem::getScalarJacobian);
    unconstrained_end_pose_problem.def("get_scalar_task_cost", &UnconstrainedEndPoseProblem::getScalarTaskCost);
    unconstrained_end_pose_problem.def_readonly("Cost", &UnconstrainedEndPoseProblem::Cost);

    py::class_<EndPoseProblem, std::shared_ptr<EndPoseProblem>, PlanningProblem> end_pose_problem(prob, "EndPoseProblem");
    end_pose_problem.def("update", &EndPoseProblem::Update);
    end_pose_problem.def("set_goal", &EndPoseProblem::setGoal);
    end_pose_problem.def("set_rho", &EndPoseProblem::setRho);
    end_pose_problem.def("get_goal", &EndPoseProblem::getGoal);
    end_pose_problem.def("get_rho", &EndPoseProblem::getRho);
    end_pose_problem.def("set_goal_eq", &EndPoseProblem::setGoalEQ);
    end_pose_problem.def("set_rho_eq", &EndPoseProblem::setRhoEQ);
    end_pose_problem.def("get_goal_eq", &EndPoseProblem::getGoalEQ);
    end_pose_problem.def("get_rho_eq", &EndPoseProblem::getRhoEQ);
    end_pose_problem.def("set_goal_neq", &EndPoseProblem::setGoalNEQ);
    end_pose_problem.def("set_rho_neq", &EndPoseProblem::setRhoNEQ);
    end_pose_problem.def("get_goal_neq", &EndPoseProblem::getGoalNEQ);
    end_pose_problem.def("get_rho_neq", &EndPoseProblem::getRhoNEQ);
    end_pose_problem.def_readwrite("W", &EndPoseProblem::W);
    end_pose_problem.def_readonly("PhiN", &EndPoseProblem::PhiN);
    end_pose_problem.def_readonly("JN", &EndPoseProblem::JN);
    end_pose_problem.def_readonly("N", &EndPoseProblem::N);
    end_pose_problem.def_readonly("num_tasks", &EndPoseProblem::NumTasks);
    end_pose_problem.def_readonly("Phi", &EndPoseProblem::Phi);
    end_pose_problem.def_readonly("J", &EndPoseProblem::J);
    end_pose_problem.def("get_scalar_cost", &EndPoseProblem::getScalarCost);
    end_pose_problem.def("get_scalar_jacobian", &EndPoseProblem::getScalarJacobian);
    end_pose_problem.def("get_scalar_task_cost", &EndPoseProblem::getScalarTaskCost);
    end_pose_problem.def("get_equality", &EndPoseProblem::getEquality);
    end_pose_problem.def("get_equality_jacobian", &EndPoseProblem::getEqualityJacobian);
    end_pose_problem.def("get_inequality", &EndPoseProblem::getInequality);
    end_pose_problem.def("get_inequality_jacobian", &EndPoseProblem::getInequalityJacobian);
    end_pose_problem.def("get_bounds", &EndPoseProblem::getBounds);
    end_pose_problem.def_readonly("Cost", &EndPoseProblem::Cost);
    end_pose_problem.def_readonly("Inequality", &EndPoseProblem::Inequality);
    end_pose_problem.def_readonly("Equality", &EndPoseProblem::Equality);

    py::class_<BoundedEndPoseProblem, std::shared_ptr<BoundedEndPoseProblem>, PlanningProblem> bounded_end_pose_problem(prob, "BoundedEndPoseProblem");
    bounded_end_pose_problem.def("update", &BoundedEndPoseProblem::Update);
    bounded_end_pose_problem.def("set_goal", &BoundedEndPoseProblem::setGoal);
    bounded_end_pose_problem.def("set_rho", &BoundedEndPoseProblem::setRho);
    bounded_end_pose_problem.def("get_goal", &BoundedEndPoseProblem::getGoal);
    bounded_end_pose_problem.def("get_rho", &BoundedEndPoseProblem::getRho);
    bounded_end_pose_problem.def_readwrite("W", &BoundedEndPoseProblem::W);
    bounded_end_pose_problem.def_readonly("PhiN", &BoundedEndPoseProblem::PhiN);
    bounded_end_pose_problem.def_readonly("JN", &BoundedEndPoseProblem::JN);
    bounded_end_pose_problem.def_readonly("N", &BoundedEndPoseProblem::N);
    bounded_end_pose_problem.def_readonly("num_tasks", &BoundedEndPoseProblem::NumTasks);
    bounded_end_pose_problem.def_readonly("Phi", &BoundedEndPoseProblem::Phi);
    bounded_end_pose_problem.def_readonly("J", &BoundedEndPoseProblem::J);
    bounded_end_pose_problem.def("get_scalar_cost", &BoundedEndPoseProblem::getScalarCost);
    bounded_end_pose_problem.def("get_scalar_jacobian", &BoundedEndPoseProblem::getScalarJacobian);
    bounded_end_pose_problem.def("get_scalar_task_cost", &BoundedEndPoseProblem::getScalarTaskCost);
    bounded_end_pose_problem.def("get_bounds", &BoundedEndPoseProblem::getBounds);
    bounded_end_pose_problem.def_readonly("Cost", &BoundedEndPoseProblem::Cost);

    py::class_<SamplingProblem, std::shared_ptr<SamplingProblem>, PlanningProblem> sampling_problem(prob, "SamplingProblem");
    sampling_problem.def("update", &SamplingProblem::Update);
    sampling_problem.def_property("goal_state", &SamplingProblem::getGoalState, &SamplingProblem::setGoalState);
    sampling_problem.def("get_space_dim", &SamplingProblem::getSpaceDim);
    sampling_problem.def("get_bounds", &SamplingProblem::getBounds);
    sampling_problem.def_readonly("N", &SamplingProblem::N);
    sampling_problem.def_readonly("num_tasks", &SamplingProblem::NumTasks);
    sampling_problem.def_readonly("Phi", &SamplingProblem::Phi);
    sampling_problem.def_readonly("Inequality", &SamplingProblem::Inequality);
    sampling_problem.def_readonly("Equality", &SamplingProblem::Equality);
    sampling_problem.def("set_goal_eq", &SamplingProblem::setGoalEQ);
    sampling_problem.def("set_rho_eq", &SamplingProblem::setRhoEQ);
    sampling_problem.def("get_goal_eq", &SamplingProblem::getGoalEQ);
    sampling_problem.def("get_rho_eq", &SamplingProblem::getRhoEQ);
    sampling_problem.def("set_goal_neq", &SamplingProblem::setGoalNEQ);
    sampling_problem.def("set_rho_neq", &SamplingProblem::setRhoNEQ);
    sampling_problem.def("get_goal_neq", &SamplingProblem::getGoalNEQ);
    sampling_problem.def("get_rho_neq", &SamplingProblem::getRhoNEQ);

    py::class_<TimeIndexedSamplingProblem, std::shared_ptr<TimeIndexedSamplingProblem>, PlanningProblem> time_indexed_sampling_problem(prob, "TimeIndexedSamplingProblem");
    time_indexed_sampling_problem.def("update", &TimeIndexedSamplingProblem::Update);
    time_indexed_sampling_problem.def("get_space_dim", &TimeIndexedSamplingProblem::getSpaceDim);
    time_indexed_sampling_problem.def("get_bounds", &TimeIndexedSamplingProblem::getBounds);
    time_indexed_sampling_problem.def_property("goal_state", &TimeIndexedSamplingProblem::getGoalState, &TimeIndexedSamplingProblem::setGoalState);
    time_indexed_sampling_problem.def_property("goal_time", &TimeIndexedSamplingProblem::getGoalTime, &TimeIndexedSamplingProblem::setGoalTime);
    time_indexed_sampling_problem.def_readonly("N", &TimeIndexedSamplingProblem::N);
    time_indexed_sampling_problem.def_readonly("num_tasks", &TimeIndexedSamplingProblem::NumTasks);
    time_indexed_sampling_problem.def_readonly("Phi", &TimeIndexedSamplingProblem::Phi);
    time_indexed_sampling_problem.def_readonly("Inequality", &TimeIndexedSamplingProblem::Inequality);
    time_indexed_sampling_problem.def_readonly("Equality", &TimeIndexedSamplingProblem::Equality);
    time_indexed_sampling_problem.def("set_goal_eq", &TimeIndexedSamplingProblem::setGoalEQ);
    time_indexed_sampling_problem.def("set_rho_eq", &TimeIndexedSamplingProblem::setRhoEQ);
    time_indexed_sampling_problem.def("get_goal_eq", &TimeIndexedSamplingProblem::getGoalEQ);
    time_indexed_sampling_problem.def("get_rho_eq", &TimeIndexedSamplingProblem::getRhoEQ);
    time_indexed_sampling_problem.def("set_goal_neq", &TimeIndexedSamplingProblem::setGoalNEQ);
    time_indexed_sampling_problem.def("set_rho_neq", &TimeIndexedSamplingProblem::setRhoNEQ);
    time_indexed_sampling_problem.def("get_goal_neq", &TimeIndexedSamplingProblem::getGoalNEQ);
    time_indexed_sampling_problem.def("get_rho_neq", &TimeIndexedSamplingProblem::getRhoNEQ);

    py::class_<CollisionProxy, std::shared_ptr<CollisionProxy>> collision_proxy(module, "CollisionProxy");
    collision_proxy.def(py::init());
    collision_proxy.def_readonly("contact_1", &CollisionProxy::contact1);
    collision_proxy.def_readonly("contact_2", &CollisionProxy::contact2);
    collision_proxy.def_readonly("normal_1", &CollisionProxy::normal1);
    collision_proxy.def_readonly("normal_2", &CollisionProxy::normal2);
    collision_proxy.def_readonly("distance", &CollisionProxy::distance);
    collision_proxy.def_property_readonly("object_1", [](CollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e1->Segment.getName() : std::string(""); });
    collision_proxy.def_property_readonly("object_2", [](CollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e2->Segment.getName() : std::string(""); });
    collision_proxy.def_property_readonly("transform_1", [](CollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e1->Frame : KDL::Frame(); });
    collision_proxy.def_property_readonly("transform_2", [](CollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e2->Frame : KDL::Frame(); });
    collision_proxy.def("__repr__", &CollisionProxy::print);

    py::class_<ContinuousCollisionProxy, std::shared_ptr<ContinuousCollisionProxy>> continuous_collision_proxy(module, "ContinuousCollisionProxy");
    continuous_collision_proxy.def(py::init());
    continuous_collision_proxy.def_readonly("contact_transform_1", &ContinuousCollisionProxy::contact_tf1);
    continuous_collision_proxy.def_readonly("contact_transform_2", &ContinuousCollisionProxy::contact_tf2);
    continuous_collision_proxy.def_readonly("in_collision", &ContinuousCollisionProxy::in_collision);
    continuous_collision_proxy.def_readonly("time_of_contact", &ContinuousCollisionProxy::time_of_contact);
    continuous_collision_proxy.def_property_readonly("object_1", [](ContinuousCollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e1->Segment.getName() : std::string(""); });
    continuous_collision_proxy.def_property_readonly("object_2", [](ContinuousCollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e2->Segment.getName() : std::string(""); });
    continuous_collision_proxy.def_property_readonly("transform_1", [](ContinuousCollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e1->Frame : KDL::Frame(); });
    continuous_collision_proxy.def_property_readonly("transform_2", [](ContinuousCollisionProxy* instance) { return (instance->e1 && instance->e2) ? instance->e2->Frame : KDL::Frame(); });
    continuous_collision_proxy.def("__repr__", &ContinuousCollisionProxy::print);

    py::class_<Scene, std::shared_ptr<Scene>, Object> scene(module, "Scene");
    scene.def("update", &Scene::Update, py::arg("x"), py::arg("t") = 0.0);
    scene.def("get_base_type", &Scene::getBaseType);
    scene.def("get_group_name", &Scene::getGroupName);
    scene.def("get_joint_names", (std::vector<std::string>(Scene::*)()) & Scene::getJointNames);
    scene.def("get_controlled_link_names", &Scene::getControlledLinkNames);
    scene.def("get_model_link_names", &Scene::getModelLinkNames);
    scene.def("get_kinematic_tree", &Scene::getKinematicTree, py::return_value_policy::reference_internal);
    scene.def("get_collision_scene", &Scene::getCollisionScene, py::return_value_policy::reference_internal);
    scene.def("get_model_joint_names", &Scene::getModelJointNames);
    scene.def("get_model_state", &Scene::getModelState);
    scene.def("get_model_state_map", &Scene::getModelStateMap);
    scene.def("set_model_state", (void (Scene::*)(Eigen::VectorXdRefConst, double, bool)) & Scene::setModelState, py::arg("x"), py::arg("t") = 0.0, py::arg("update_trajectory") = false);
    scene.def("set_model_state_map", (void (Scene::*)(std::map<std::string, double>, double, bool)) & Scene::setModelState, py::arg("x"), py::arg("t") = 0.0, py::arg("update_trajectory") = false);
    scene.def("get_controlled_state", &Scene::getControlledState);
    scene.def("publish_scene", &Scene::publishScene);
    scene.def("publish_proxies", &Scene::publishProxies);
    scene.def("set_collision_scene", &Scene::setCollisionScene);
    scene.def("load_scene",
              (void (Scene::*)(const std::string&, const KDL::Frame&, bool)) & Scene::loadScene,
              py::arg("scene_string"),
              py::arg("offset_transform") = kdl_frame(),
              py::arg("update_collision_scene") = true);
    scene.def("load_scene_file",
              (void (Scene::*)(const std::string&, const KDL::Frame&, bool)) & Scene::loadSceneFile,
              py::arg("file_name"),
              py::arg("offset_transform") = kdl_frame(),
              py::arg("update_collision_scene") = true);
    scene.def("get_scene", &Scene::getScene);
    scene.def("clean_scene", &Scene::cleanScene);
    scene.def("is_state_valid", [](Scene* instance, bool self, double safe_distance) { return instance->getCollisionScene()->isStateValid(self, safe_distance); }, py::arg("check_self_collision") = true, py::arg("safe_distance") = 0.0);
    scene.def("is_collision_free", [](Scene* instance, const std::string& o1, const std::string& o2, double safe_distance) { return instance->getCollisionScene()->isCollisionFree(o1, o2, safe_distance); }, py::arg("object_1"), py::arg("object_2"), py::arg("safe_distance") = 0.0);
    scene.def("is_allowed_to_collide", [](Scene* instance, const std::string& o1, const std::string& o2, bool self) { return instance->getCollisionScene()->isAllowedToCollide(o1, o2, self); }, py::arg("object_1"), py::arg("object_2"), py::arg("check_self_collision") = true);
    scene.def("get_collision_distance", [](Scene* instance, bool self) { return instance->getCollisionScene()->getCollisionDistance(self); }, py::arg("check_self_collision") = true);
    scene.def("get_collision_distance", [](Scene* instance, const std::string& o1, const std::string& o2) { return instance->getCollisionScene()->getCollisionDistance(o1, o2); }, py::arg("object_1"), py::arg("object_2"));
    scene.def("get_collision_distance",
              [](Scene* instance, const std::string& o1, const bool& self) {
                  return instance->getCollisionScene()->getCollisionDistance(o1, self);
              },
              py::arg("object_1"), py::arg("check_self_collision") = true);
    scene.def("get_collision_distance",
              [](Scene* instance, const std::vector<std::string>& objects, const bool& self) {
                  return instance->getCollisionScene()->getCollisionDistance(objects, self);
              },
              py::arg("objects"), py::arg("check_self_collision") = true);
    scene.def("update_planning_scene_world",
              [](Scene* instance, moveit_msgs::PlanningSceneWorld& world) {
                  moveit_msgs::PlanningSceneWorldConstPtr myPtr(
                      new moveit_msgs::PlanningSceneWorld(world));
                  instance->updateWorld(myPtr);
              });
    scene.def("update_collision_objects", &Scene::updateCollisionObjects);
    scene.def("get_collision_robot_links", [](Scene* instance) { return instance->getCollisionScene()->getCollisionRobotLinks(); });
    scene.def("get_collision_world_links", [](Scene* instance) { return instance->getCollisionScene()->getCollisionWorldLinks(); });
    scene.def("get_root_frame_name", &Scene::getRootFrameName);
    scene.def("get_root_joint_name", &Scene::getRootJointName);
    scene.def("attach_object", &Scene::attachObject);
    scene.def("attach_object_local", &Scene::attachObjectLocal);
    scene.def("detach_object", &Scene::detachObject);
    scene.def("has_attached_object", &Scene::hasAttachedObject);
    scene.def("fk", [](Scene* instance, const std::string& e1, const KDL::Frame& o1, const std::string& e2, const KDL::Frame& o2) { return instance->getKinematicTree().FK(e1, o1, e2, o2); });
    scene.def("fk", [](Scene* instance, const std::string& e1, const std::string& e2) { return instance->getKinematicTree().FK(e1, KDL::Frame(), e2, KDL::Frame()); });
    scene.def("fk", [](Scene* instance, const std::string& e1) { return instance->getKinematicTree().FK(e1, KDL::Frame(), "", KDL::Frame()); });
    scene.def("jacobian", [](Scene* instance, const std::string& e1, const KDL::Frame& o1, const std::string& e2, const KDL::Frame& o2) { return instance->getKinematicTree().Jacobian(e1, o1, e2, o2); });
    scene.def("jacobian", [](Scene* instance, const std::string& e1, const std::string& e2) { return instance->getKinematicTree().Jacobian(e1, KDL::Frame(), e2, KDL::Frame()); });
    scene.def("jacobian", [](Scene* instance, const std::string& e1) { return instance->getKinematicTree().Jacobian(e1, KDL::Frame(), "", KDL::Frame()); });
    scene.def("add_trajectory_from_file", &Scene::addTrajectoryFromFile);
    scene.def("add_trajectory", (void (Scene::*)(const std::string&, const std::string&)) & Scene::addTrajectory);
    scene.def("get_trajectory", [](Scene* instance, const std::string& link) { return instance->getTrajectory(link)->toString(); });
    scene.def("remove_trajectory", &Scene::removeTrajectory);
    scene.def("update_scene_frames", &Scene::updateSceneFrames);
    scene.def("add_object", [](Scene* instance, const std::string& name, const KDL::Frame& transform, const std::string& parent, const std::string& shapeResourcePath, Eigen::Vector3d scale, bool updateCollisionScene) { instance->addObject(name, transform, parent, shapeResourcePath, scale, KDL::RigidBodyInertia::Zero(), updateCollisionScene); },
              py::arg("name"),
              py::arg("transform") = KDL::Frame(),
              py::arg("parent") = "",
              py::arg("shape_resource_path"),
              py::arg("scale") = Eigen::Vector3d::Ones(),
              py::arg("update_collision_scene") = true);
    scene.def("add_object", (void (Scene::*)(const std::string&, const KDL::Frame&, const std::string&, shapes::ShapeConstPtr, const KDL::RigidBodyInertia&, bool)) & Scene::addObject,
              py::arg("name"),
              py::arg("transform") = KDL::Frame(),
              py::arg("parent") = std::string(),
              py::arg("shape"),
              py::arg("inertia") = KDL::RigidBodyInertia::Zero(),
              py::arg("update_collision_scene") = true);
    scene.def("add_object_to_environment", &Scene::addObjectToEnvironment,
              py::arg("name"),
              py::arg("transform") = KDL::Frame(),
              py::arg("shape"),
              py::arg("update_collision_scene") = true);
    scene.def("remove_object", &Scene::removeObject);
    scene.def_property_readonly("model_link_to_collision_link_map", &Scene::getModelLinkToCollisionLinkMap);
    scene.def_property_readonly("controlled_link_to_collision_link_map", &Scene::getControlledLinkToCollisionLinkMap);

    py::class_<CollisionScene, std::shared_ptr<CollisionScene>> collision_scene(module, "CollisionScene");
    // TODO: expose isStateValid, isCollisionFree, getCollisionDistance, getCollisionWorldLinks, getCollisionRobotLinks, getTranslation
    collision_scene.def_property("always_externally_updated_collision_scene", &CollisionScene::getAlwaysExternallyUpdatedCollisionScene, &CollisionScene::setAlwaysExternallyUpdatedCollisionScene);
    collision_scene.def_property("replace_primitive_shapes_with_meshes", &CollisionScene::getReplacePrimitiveShapesWithMeshes, &CollisionScene::setReplacePrimitiveShapesWithMeshes);
    collision_scene.def_readwrite("replace_cylinders_with_capsules", &CollisionScene::replaceCylindersWithCapsules);
    collision_scene.def_property("robot_link_scale", &CollisionScene::getRobotLinkScale, &CollisionScene::setRobotLinkScale);
    collision_scene.def_property("world_link_scale", &CollisionScene::getWorldLinkScale, &CollisionScene::setWorldLinkScale);
    collision_scene.def_property("robot_link_padding", &CollisionScene::getRobotLinkPadding, &CollisionScene::setRobotLinkPadding);
    collision_scene.def_property("world_link_padding", &CollisionScene::getWorldLinkPadding, &CollisionScene::setWorldLinkPadding);
    collision_scene.def("update_collision_object_transforms", &CollisionScene::updateCollisionObjectTransforms);
    collision_scene.def("continuous_collision_check", &CollisionScene::continuousCollisionCheck);

    py::class_<Visualization> visualization(module, "Visualization");
    visualization.def(py::init<Scene_ptr>());
    visualization.def("display_trajectory", &Visualization::displayTrajectory);

    py::module kin = module.def_submodule("Kinematics", "Kinematics submodule.");
    py::class_<KinematicTree, std::shared_ptr<KinematicTree>> kinematic_tree(kin, "KinematicTree");
    kinematic_tree.def_readwrite("debug_mode", &KinematicTree::Debug);
    kinematic_tree.def("publish_frames", &KinematicTree::publishFrames);
    kinematic_tree.def("get_root_frame_name", &KinematicTree::getRootFrameName);
    kinematic_tree.def("get_root_joint_name", &KinematicTree::getRootJointName);
    kinematic_tree.def("get_model_base_type", &KinematicTree::getModelBaseType);
    kinematic_tree.def("get_controlled_base_type", &KinematicTree::getControlledBaseType);
    kinematic_tree.def("get_controlled_link_mass", &KinematicTree::getControlledLinkMass);
    kinematic_tree.def("get_collision_object_types", &KinematicTree::getCollisionObjectTypes);
    kinematic_tree.def("get_random_controlled_state", &KinematicTree::getRandomControlledState);
    kinematic_tree.def("get_num_model_joints", &KinematicTree::getNumModelJoints);
    kinematic_tree.def("get_num_controlled_joints", &KinematicTree::getNumControlledJoints);

    // Joint Limits
    kinematic_tree.def("get_joint_limits", &KinematicTree::getJointLimits);
    kinematic_tree.def("reset_joint_limits", &KinematicTree::resetJointLimits);
    kinematic_tree.def("set_joint_limits_lower", &KinematicTree::setJointLimitsLower);
    kinematic_tree.def("set_joint_limits_upper", &KinematicTree::setJointLimitsUpper);
    kinematic_tree.def("set_floating_base_limits_pos_xyz_euler_zyx", &KinematicTree::setFloatingBaseLimitsPosXYZEulerZYX);
    kinematic_tree.def("set_planar_base_limits_pos_xy_euler_z", &KinematicTree::setPlanarBaseLimitsPosXYEulerZ);
    kinematic_tree.def("get_used_joint_limits", &KinematicTree::getUsedJointLimits);

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

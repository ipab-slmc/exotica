#include <exotica/Exotica.h>

using namespace exotica;

std::string urdf_string="<robot name=\"test_robot\"><link name=\"base\"><visual><geometry><cylinder length=\"0.3\" radius=\"0.2\"/></geometry><origin xyz=\"0 0 0.15\"/></visual></link><link name=\"link1\"> <inertial><mass value=\"0.2\"/><origin xyz=\"0 0 0.1\"/><inertia ixx=\"0.00381666666667\" ixy=\"0\" ixz=\"0\" iyy=\"0.0036\" iyz=\"0\" izz=\"0.00381666666667\"/></inertial><visual><geometry><cylinder length=\"0.15\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.075\"/></visual> </link><link name=\"link2\"><inertial><mass value=\"0.2\"/><origin xyz=\"0 0 0.1\"/><inertia ixx=\"0.00381666666667\" ixy=\"0\" ixz=\"0\" iyy=\"0.0036\" iyz=\"0\" izz=\"0.00381666666667\"/></inertial><visual><geometry><cylinder length=\"0.35\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.175\"/></visual> </link><link name=\"link3\"><inertial><mass value=\"0.2\"/><origin xyz=\"0 0 0.1\"/><inertia ixx=\"0.00381666666667\" ixy=\"0\" ixz=\"0\" iyy=\"0.0036\" iyz=\"0\" izz=\"0.00381666666667\"/></inertial><visual><geometry><cylinder length=\"0.45\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.225\"/></visual></link><link name=\"endeff\"><inertial><mass value=\"0.2\"/><origin xyz=\"0 0 0.1\"/><inertia ixx=\"0.00381666666667\" ixy=\"0\" ixz=\"0\" iyy=\"0.0036\" iyz=\"0\" izz=\"0.00381666666667\"/></inertial><visual><geometry><cylinder length=\"0.05\" radius=\"0.1\"/></geometry><origin xyz=\"0 0 -0.025\"/></visual></link><joint name=\"joint1\" type=\"revolute\"><parent link=\"base\"/><child link=\"link1\"/><origin xyz=\"0 0 0.3\" rpy=\"0 0 0\" /><axis xyz=\"0 0 1\" /><limit effort=\"200\"  velocity=\"1.0\" lower=\"-0.4\" upper=\"0.4\"/><safety_controller k_position=\"30\" k_velocity=\"30\" soft_lower_limit=\"-0.4\" soft_upper_limit=\"0.4\"/></joint><joint name=\"joint2\" type=\"revolute\"><parent link=\"link1\"/><child link=\"link2\"/><origin xyz=\"0 0 0.15\" rpy=\"0 0 0\" /><axis xyz=\"0 1 0\" /><limit effort=\"200\"  velocity=\"1.0\" lower=\"-0.4\" upper=\"0.4\"/><safety_controller k_position=\"30\" k_velocity=\"30\" soft_lower_limit=\"-0.4\" soft_upper_limit=\"0.4\"/></joint><joint name=\"joint3\" type=\"revolute\"><parent link=\"link2\"/><child link=\"link3\"/><origin xyz=\"0 0 0.35\" rpy=\"0 0 0\" /><axis xyz=\"0 1 0\" /><limit effort=\"200\"  velocity=\"1.0\" lower=\"-0.4\" upper=\"0.4\"/><safety_controller k_position=\"30\" k_velocity=\"30\" soft_lower_limit=\"-0.4\" soft_upper_limit=\"0.4\"/></joint><joint name=\"joint4\" type=\"fixed\"><parent link=\"link3\"/><child link=\"endeff\"/><origin xyz=\"0 0 0.45\" rpy=\"0 0 0\" /></joint></robot>";
std::string srdf_string="<robot name=\"test_robot\"><group name=\"arm\"><chain base_link=\"base\" tip_link=\"endeff\" /></group><virtual_joint name=\"world_joint\" type=\"fixed\" parent_frame=\"world_frame\" child_link=\"base\" /><group_state name=\"zero\" group=\"arm\"><joint name=\"joint1\" value=\"0\" /><joint name=\"joint2\" value=\"0.3\" /><joint name=\"joint3\" value=\"0.55\" /></group_state><disable_collisions link1=\"base\" link2=\"link1\" reason=\"Adjacent\" /><disable_collisions link1=\"endeff\" link2=\"link3\" reason=\"Adjacent\" /><disable_collisions link1=\"link1\" link2=\"link2\" reason=\"Adjacent\" /><disable_collisions link1=\"link2\" link2=\"link3\" reason=\"Adjacent\" /></robot>";
bool printInfo = false;

#define NUM_TRIALS 100

void testRandom(UnconstrainedEndPoseProblem_ptr problem)
{
    Eigen::VectorXd x(3);
    INFO_PLAIN("Testing random configurations:")
    for(int i=0;i<NUM_TRIALS;i++)
    {
        x.setRandom();
        problem->Update(x);
        if(printInfo)
        {
            INFO_PLAIN("x = "<<x.transpose());
            INFO_PLAIN("y = "<<problem->Phi.transpose());
            INFO_PLAIN("J = \n"<<problem->J);
        }
    }
    INFO_PLAIN("Test passed");
}

void testValues(Eigen::MatrixXdRefConst Xref, Eigen::MatrixXdRefConst Yref, Eigen::MatrixXdRefConst Jref, UnconstrainedEndPoseProblem_ptr problem, double eps = 1e-5)
{
    INFO_PLAIN("Testing set points:");
    int N = Xref.cols();
    int M = Yref.cols();
    int L = Xref.rows();
    for(int i=0;i<L;i++)
    {
        Eigen::VectorXd x = Xref.row(i);
        Eigen::VectorXd y = Yref.row(i);
        Eigen::MatrixXd J = Jref.middleRows(i*M,M);
        problem->Update(x);
        double errY = (y - problem->Phi).norm();
        double errJ = (J - problem->J).norm();
        if(errY > eps) throw_pretty("Task space error out of bounds: " << errY);
        if(errJ > eps)
        {
            HIGHLIGHT("x: "<<x.transpose());
            HIGHLIGHT("J*:\n"<<J);
            HIGHLIGHT("J:\n"<<problem->J);
            throw_pretty("Jacobian error out of bounds: " << errJ);
        }
    }

    INFO_PLAIN("Test passed");
}

void testJacobian(UnconstrainedEndPoseProblem_ptr problem, double eps = 1e-5)
{
    INFO_PLAIN("Testing Jacobian:");
    for(int j=0;j<NUM_TRIALS;j++)
    {
        Eigen::VectorXd x0(problem->N);
        x0.setRandom();
        problem->Update(x0);
        Eigen::VectorXd y0 = problem->Phi;
        Eigen::MatrixXd J0 = problem->J;
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(J0.rows(), J0.cols());
        for(int i=0;i<problem->N;i++)
        {
            Eigen::VectorXd x = x0;
            x(i)+=eps;
            problem->Update(x);
            J.col(i) = (problem->Phi - y0)/eps;
        }
        double errJ = (J-J0).norm();
        if(errJ > eps) throw_pretty("Jacobian error out of bounds: " << errJ);
    }
    INFO_PLAIN("Test passed");
}

UnconstrainedEndPoseProblem_ptr setupProbelm(Initializer& map)
{
    Initializer scene("Scene",{{"Name",std::string("MyScene")},{"JointGroup",std::string("arm")}});
    map.addProperty(Property("Scene", true, std::string("MyScene")));
    Eigen::VectorXd W(3);W << 3,2,1;
    Initializer problem("exotica/UnconstrainedEndPoseProblem",{
                            {"Name",std::string("MyProblem")},
                            {"PlanningScene",scene},
                            {"Maps",std::vector<Initializer>({map})},
                            {"W",W},
                        });
    Server::Instance()->getModel("robot_description",urdf_string,srdf_string);
    return boost::static_pointer_cast<UnconstrainedEndPoseProblem>(Setup::createProblem(problem));
}

void testEffPosition()
{
    HIGHLIGHT("End-effector position test");
    Initializer map("exotica/EffPosition",{
                        {"Name",std::string("Position")},
                        {"EndEffector",std::vector<Initializer>({
                             Initializer("Limb",{{"Segment",std::string("endeff")}})
                                        }) } });
    UnconstrainedEndPoseProblem_ptr problem = setupProbelm(map);
    testRandom(problem);

    int N = problem->N;
    int M = problem->PhiN;
    int L = 5;
    Eigen::MatrixXd X(L, N);
    Eigen::MatrixXd Y(L, M);
    Eigen::MatrixXd J(L*M, N);

    X << 0.680375, -0.211234, 0.566198, 0.59688, 0.823295, -0.604897, -0.329554, 0.536459, -0.444451, 0.10794, -0.0452059, 0.257742, -0.270431, 0.0268018, 0.904459;
    Y << 0.0645323, 0.0522249, 1.21417, 0.292945, 0.199075, 1.12724, 0.208378, -0.0712708, 1.19893, 0.0786457, 0.00852213, 1.23952, 0.356984, -0.0989639, 1.06844;
    J << -0.0522249, 0.594015, 0.327994
            , 0.0645323, 0.480726, 0.26544
            , 0, -0.0830172, -0.156401
            , -0.199075, 0.560144, 0.363351
            , 0.292945, 0.380655, 0.246921
            , 0, -0.354186, -0.0974994
            , 0.0712708, 0.708627, 0.423983
            , 0.208378, -0.24237, -0.145014
            , 0, -0.220229, -0.0413455
            ,-0.00852213, 0.784922, 0.437315
            , 0.0786457, 0.085055, 0.0473879
            , 0, -0.0791061, -0.0949228
            , 0.0989639, 0.595968, 0.258809
            , 0.356984, -0.165215, -0.0717477
            , 0, -0.370448, -0.361068;
    testValues(X ,Y ,J ,problem);

    testJacobian(problem);
}

void testDistance()
{
    HIGHLIGHT("Distance test");
    Initializer map("exotica/Distance",{
                        {"Name",std::string("Distance")},
                        {"EndEffector",std::vector<Initializer>({
                             Initializer("Limb",{{"Segment",std::string("endeff")}})
                                        }) } });
    UnconstrainedEndPoseProblem_ptr problem = setupProbelm(map);
    testRandom(problem);

    int N = problem->N;
    int M = problem->PhiN;
    int L = 5;
    Eigen::MatrixXd X(L, N);
    Eigen::MatrixXd Y(L, M);
    Eigen::MatrixXd J(L*M, N);

    X << 0.83239, 0.271423, 0.434594, -0.716795, 0.213938, -0.967399, -0.514226, -0.725537, 0.608354, -0.686642, -0.198111, -0.740419, -0.782382, 0.997849, -0.563486;
    Y << 1.19368, 1.14431, 1.19326, 1.14377, 1.1541;
    J << 0, -0.145441, -0.16562,
            0, 0.0918504, 0.234405,
            0, 0.107422, -0.0555943,
            0, 0.169924, 0.235715,
            0, -0.188516, -0.000946239;
    testValues(X ,Y ,J ,problem);

    testJacobian(problem);
}

void testJointLimit()
{
    HIGHLIGHT("Joint limit test");
    Initializer map("exotica/JointLimit",{
                        {"Name",std::string("JointLimit")},
                        {"SafePercentage", 0.0} });
    UnconstrainedEndPoseProblem_ptr problem = setupProbelm(map);
    testRandom(problem);

    int N = problem->N;
    int M = problem->PhiN;
    int L = 5;
    Eigen::MatrixXd X(L, N);
    Eigen::MatrixXd Y(L, M);
    Eigen::MatrixXd J(L*M, N);

    X << 0.0258648, 0.678224, 0.22528, -0.407937, 0.275105, 0.0485744, -0.012834, 0.94555, -0.414966, 0.542715, 0.05349, 0.539828, -0.199543, 0.783059, -0.433371;
    Y << 0, 0.278224, 0, -0.00793676, 0, 0, 0, 0.54555, -0.0149664, 0.142715, 0, 0.139828, 0, 0.383059, -0.0333705;
    J <<    1, 0, 0,
            0, 1, 0,
            0, 0, 1,
            1, 0, 0,
            0, 1, 0,
            0, 0, 1,
            1, 0, 0,
            0, 1, 0,
            0, 0, 1,
            1, 0, 0,
            0, 1, 0,
            0, 0, 1,
            1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    testValues(X ,Y ,J ,problem);
}

void testSphereCollision()
{
    HIGHLIGHT("Sphere collision test");
    Initializer map("exotica/SphereCollision",{
                        {"Name",std::string("SphereCollision")},
                        {"Precision",1e-2},
                        {"ReferenceFrame",std::string("base")},
                        {"EndEffector",std::vector<Initializer>({
                             Initializer("Limb",{{"Segment",std::string("base")}, {"Radius", 0.3}, {"Group", std::string("base")}}),
                             Initializer("Limb",{{"Segment",std::string("endeff")}, {"Radius", 0.3}, {"Group", std::string("eff")}})
                                        }) } });
    UnconstrainedEndPoseProblem_ptr problem = setupProbelm(map);
    testRandom(problem);

    int N = problem->N;
    int M = problem->PhiN;
    int L = 5;
    Eigen::MatrixXd X(L, N);
    Eigen::MatrixXd Y(L, M);
    Eigen::MatrixXd J(L*M, N);

    X << -0.590167, 1.2309, 1.67611, -1.72098, 1.79731, 0.103981, -1.65578, -1.23114, 0.652908, 1.56093, -0.604428, -1.74331, -1.91991, -0.169193, -1.74762;
    Y << 1, 5.71023e-44, 1.83279e-110, 6.87352e-16, 7.26371e-45;
    J <<    0, 0.431392, 0.449344,
            0, 0.431735, 0.26014,
            0, -0.234475, -0.0135658,
            0, -0.349195, -0.447214,
            0, -0.270171, -0.430172;
    testValues(X ,Y ,J ,problem);
}

void testIdentity()
{
    HIGHLIGHT("Identity test");
    Initializer map("exotica/Identity",{
                        {"Name",std::string("Identity")}
                    });
    UnconstrainedEndPoseProblem_ptr problem = setupProbelm(map);
    testRandom(problem);

    {
        int N = problem->N;
        int M = problem->PhiN;
        int L = 5;
        Eigen::MatrixXd X(L, N);
        Eigen::MatrixXd Y(L, M);
        Eigen::MatrixXd J(L*M, N);

        X << -0.52344, 0.941268, 0.804416, 0.70184, -0.466669, 0.0795207, -0.249586, 0.520497, 0.0250707, 0.335448, 0.0632129, -0.921439, -0.124725, 0.86367, 0.86162;
        Y << -0.52344, 0.941268, 0.804416, 0.70184, -0.466669, 0.0795207, -0.249586, 0.520497, 0.0250707, 0.335448, 0.0632129, -0.921439, -0.124725, 0.86367, 0.86162;
        J <<    1, 0, 0,
                0, 1, 0,
                0, 0, 1,
                1, 0, 0,
                0, 1, 0,
                0, 0, 1,
                1, 0, 0,
                0, 1, 0,
                0, 0, 1,
                1, 0, 0,
                0, 1, 0,
                0, 0, 1,
                1, 0, 0,
                0, 1, 0,
                0, 0, 1;
        testValues(X ,Y ,J ,problem);
    }
    testJacobian(problem);

    HIGHLIGHT("Identity test with reference");
    map = Initializer("exotica/Identity",{
                        {"Name",std::string("Identity")},
                        {"JointRef",std::string("0.5 0.5 0.5")}
                    });
    problem = setupProbelm(map);
    testRandom(problem);

    {
        int N = problem->N;
        int M = problem->PhiN;
        int L = 5;
        Eigen::MatrixXd X(L, N);
        Eigen::MatrixXd Y(L, M);
        Eigen::MatrixXd J(L*M, N);

        X << 0.441905, -0.431413, 0.477069, 0.279958, -0.291903, 0.375723, -0.668052, -0.119791, 0.76015, 0.658402, -0.339326, -0.542064, 0.786745, -0.29928, 0.37334;
        Y << -0.0580953, -0.931413, -0.0229314, -0.220042, -0.791903, -0.124277, -1.16805, -0.619791, 0.26015, 0.158402, -0.839326, -1.04206, 0.286745, -0.79928, -0.12666;
        J <<    1, 0, 0,
                0, 1, 0,
                0, 0, 1,
                1, 0, 0,
                0, 1, 0,
                0, 0, 1,
                1, 0, 0,
                0, 1, 0,
                0, 0, 1,
                1, 0, 0,
                0, 1, 0,
                0, 0, 1,
                1, 0, 0,
                0, 1, 0,
                0, 0, 1;
        testValues(X ,Y ,J ,problem);
    }
    testJacobian(problem);

    HIGHLIGHT("Identity test with subset of joints");
    map = Initializer("exotica/Identity",{
                        {"Name",std::string("Identity")},
                        {"JointMap",std::string("0")}
                    });
    problem = setupProbelm(map);
    testRandom(problem);

    {
        int N = problem->N;
        int M = problem->PhiN;
        int L = 5;
        Eigen::MatrixXd X(L, N);
        Eigen::MatrixXd Y(L, M);
        Eigen::MatrixXd J(L*M, N);

        X << 0.912937, 0.17728, 0.314608, 0.717353, -0.12088, 0.84794, -0.203127, 0.629534, 0.368437, 0.821944, -0.0350187, -0.56835, 0.900505, 0.840257, -0.70468;
        Y << 0.912937, 0.717353, -0.203127, 0.821944, 0.900505;
        J <<    1, 0, 0,
                1, 0, 0,
                1, 0, 0,
                1, 0, 0,
                1, 0, 0;
        testValues(X ,Y ,J ,problem);
    }
    testJacobian(problem);
}

void testCoM()
{
    HIGHLIGHT("CoM test");
    Initializer map("exotica/CoM",{
                        {"Name",std::string("CoM")},
                        {"EnableZ", true} });
    UnconstrainedEndPoseProblem_ptr problem = setupProbelm(map);
    testRandom(problem);

    {
        int N = problem->N;
        int M = problem->PhiN;
        int L = 5;
        Eigen::MatrixXd X(L, N);
        Eigen::MatrixXd Y(L, M);
        Eigen::MatrixXd J(L*M, N);

        X << 0.762124, 0.282161, -0.136093, 0.239193, -0.437881, 0.572004, -0.385084, -0.105933, -0.547787, -0.624934, -0.447531, 0.112888, -0.166997, -0.660786, 0.813608;
        Y << 0.0573909, 0.0547798, 0.790361, -0.0612788, -0.0149435, 0.779671, -0.111184, 0.0450653, 0.765376, -0.113474, 0.0818583, 0.771289, -0.096647, 0.0162915, 0.756008;
        J <<    -0.0547798, 0.255249, 0.116296,
                0.0573909, 0.243636, 0.111005,
                0, -0.0793381, -0.0236517,
                0.0149435, 0.332429, 0.156456,
                -0.0612788, 0.0810667, 0.0381535,
                0, 0.0630746, -0.0217297,
                -0.0450653, 0.303864, 0.11955,
                -0.111184, -0.123162, -0.048456,
                0, 0.11997, 0.0988233,
                -0.0818583, 0.270704, 0.124477,
                -0.113474, -0.195281, -0.0897958,
                0, 0.139918, 0.0533702,
                -0.0162915, 0.314077, 0.158372,
                -0.096647, -0.0529432, -0.0266963,
                0, 0.0980105, -0.024737;
        testValues(X ,Y ,J ,problem);
    }
    testJacobian(problem);

    HIGHLIGHT("CoM test with a subset of links");
    map = Initializer("exotica/CoM",{
                        {"Name",std::string("CoM")},
                        {"EnableZ", true},
                        {"EndEffector",std::vector<Initializer>({
                               Initializer("Limb",{{"Segment",std::string("link2")}}),
                               Initializer("Limb",{{"Segment",std::string("endeff")}})
                                          }) }});
    problem = setupProbelm(map);
    testRandom(problem);

    {
        int N = problem->N;
        int M = problem->PhiN;
        int L = 5;
        Eigen::MatrixXd X(L, N);
        Eigen::MatrixXd Y(L, M);
        Eigen::MatrixXd J(L*M, N);

        X << -0.793658, -0.747849, -0.00911187, 0.52095, 0.969503, 0.870008, 0.36889, -0.233623, 0.499542, -0.262673, -0.411679, -0.535477, 0.168977, -0.511175, -0.69522;
        Y << -0.239727, 0.243721, 0.814864, 0.390885, 0.224299, 0.504274, 0.0188228, 0.00727665, 0.934222, -0.302522, 0.0813437, 0.8168, -0.361786, -0.0617221, 0.744246;
        J <<    -0.243721, 0.255858, 0.140182,
                -0.239727, -0.26012, -0.142517,
                0, 0.341861, 0.188847,
                -0.224299, 0.0470744, -0.0633254,
                0.390885, 0.0270125, -0.0363377,
                0, -0.450667, -0.265131,
                -0.00727665, 0.451647, 0.247485,
                0.0188228, 0.174601, 0.0956743,
                0, -0.0201804, -0.0722688,
                -0.0813437, 0.354218, 0.15509,
                -0.302522, -0.0952442, -0.0417014,
                0, 0.313267, 0.223233,
                0.0617221, 0.290055, 0.0966114,
                -0.361786, 0.0494845, 0.0164823,
                0, 0.367013, 0.256943;
        testValues(X ,Y ,J ,problem);
    }
    testJacobian(problem);

    HIGHLIGHT("CoM test with projection on XY plane");
    map = Initializer("exotica/CoM",{
                        {"Name",std::string("CoM")},
                        {"EnableZ", false} });
    problem = setupProbelm(map);
    testRandom(problem);

    {
        int N = problem->N;
        int M = problem->PhiN;
        int L = 5;
        Eigen::MatrixXd X(L, N);
        Eigen::MatrixXd Y(L, M);
        Eigen::MatrixXd J(L*M, N);

        X << 0.464297, -0.74905, 0.586941, -0.671796, 0.490143, -0.85094, 0.900208, -0.894941, 0.0431268, -0.647579, -0.519875, 0.595596, 0.465309, 0.313127, 0.93481;
        Y << -0.145222, -0.0727291, 0.0287914, -0.0228944, -0.172956, -0.218045, -0.0694356, 0.0525205, 0.192777, 0.0967893;
        J <<    0.0727291, 0.274354, 0.143392,
                -0.145222, 0.1374, 0.0718125,
                0.0228944, 0.257111, 0.119001,
                0.0287914, -0.20445, -0.0946271,
                0.218045, 0.144262, 0.0665107,
                -0.172956, 0.18187, 0.0838498,
                -0.0525205, 0.267665, 0.12923,
                -0.0694356, -0.202459, -0.0977484,
                -0.0967893, 0.216122, 0.0460763,
                0.192777, 0.10851, 0.0231339;
        testValues(X ,Y ,J ,problem);
    }
    testJacobian(problem);
}

void testIMesh()
{
    HIGHLIGHT("Interaction mesh test");
    Initializer map("exotica/IMesh",{
                        {"Name",std::string("IMesh")},
                        {"ReferenceFrame",std::string("base")},
                        {"EndEffector",std::vector<Initializer>({
                               Initializer("Limb",{{"Segment",std::string("base")}}),
                               Initializer("Limb",{{"Segment",std::string("link2")}}),
                               Initializer("Limb",{{"Segment",std::string("endeff")}})
                                          }) }
                         });
    UnconstrainedEndPoseProblem_ptr problem = setupProbelm(map);
    testRandom(problem);

    int N = problem->N;
    int M = problem->PhiN;
    int L = 5;
    Eigen::MatrixXd X(L, N);
    Eigen::MatrixXd Y(L, M);
    Eigen::MatrixXd J(L*M, N);
    J.setZero();
    Y.setZero();
    X.setZero();

    X << 0.278917, 0.51947, -0.813039, -0.730195, 0.0404201, -0.843536, -0.860187, -0.59069, -0.0771591, 0.639355, 0.146637, 0.511162, -0.896122, -0.684386, 0.999987;
    Y << -0.0115151, -0.00329772, -0.652131, -0.01588, -0.00454774, 0.000489, 0.0418479, 0.0119845, 0.906934,
            0.0647007, -0.0579245, -0.635726, 0.0879002, -0.0786944, 0.0262201, -0.230697, 0.206536, 0.836692,
            0.0846586, -0.098373, -0.626482, 0.111269, -0.129294, 0.0559699, -0.308935, 0.358981, 0.824647,
            -0.0715067, -0.0531681, -0.641823, -0.0962226, -0.0715454, 0.0264905, 0.261816, 0.194671, 0.879061,
            0.014318, -0.0178999, -0.646355, 0.0198797, -0.024853, 0.00185134, -0.0509673, 0.0637179, 0.869616;
    J <<    0.00329772, -0.194435, -0.112919,
            -0.0115151, -0.0556828, -0.0323379,
            0, 0.00993552, -0.0177924,
            0.00454774, -0.267977, -0.155057,
            -0.01588, -0.0767438, -0.0444055,
            0, 0.0165184, 0.00951859,
            -0.0119845, 0.706188, 0.414101,
            0.0418479, 0.202239, 0.118591,
            0, -0.0420476, 0.139591,
            0.0579245, -0.143241, -0.0744983,
            0.0647007, 0.128239, 0.066696,
            0, -0.0728714, -0.0644042,
            0.0786944, -0.18799, -0.100693,
            0.0879002, 0.168302, 0.0901469,
            0, -0.11798, -0.0656211,
            -0.206536, 0.493387, 0.232834,
            -0.230697, -0.441714, -0.208449,
            0, 0.298475, 0.326197,
            0.098373, -0.124335, -0.0691047,
            0.0846586, 0.144477, 0.0802994,
            0, -0.110572, -0.0639689,
            0.129294, -0.151303, -0.0843603,
            0.111269, 0.175813, 0.0980263,
            0, -0.17058, -0.0955839,
            -0.358981, 0.420088, 0.230469,
            -0.308935, -0.488141, -0.267804,
            0, 0.457396, 0.270273,
            0.0531681, -0.159255, -0.085326,
            -0.0715067, -0.118412, -0.0634434,
            0, 0.0748349, 0.0556152,
            0.0715454, -0.207141, -0.112843,
            -0.0962226, -0.154018, -0.0839033,
            0, 0.119906, 0.0666997,
            -0.194671, 0.56362, 0.285766,
            0.261816, 0.419075, 0.212479,
            0, -0.315274, -0.273879,
            0.0178999, -0.122936, -0.0735487,
            0.014318, 0.153692, 0.0919485,
            0, -0.0190144, 0.0184454,
            0.024853, -0.170294, -0.100978,
            0.0198797, 0.212897, 0.12624,
            0, -0.0318257, -0.0186769,
            -0.0637179, 0.436598, 0.267206,
            -0.0509673, -0.545823, -0.334054,
            0, 0.0786625, -0.152426;
    testValues(X ,Y ,J ,problem);
    testJacobian(problem);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "EXOTica_test_maps");
    testEffPosition();
    testDistance();
    testJointLimit();
    testSphereCollision();
    testIdentity();
    testCoM();
    testIMesh();
    Setup::Destroy();
    return 0;
}

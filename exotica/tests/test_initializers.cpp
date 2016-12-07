#include <exotica/Exotica.h>
#include <ik_solver/IKsolverInitializer.h>
#include <task_map/EffPositionInitializer.h>
#include <exotica/TaskSqrErrorInitializer.h>
#include <exotica/IKProblemInitializer.h>

using namespace exotica;

std::string urdf_string="<robot name=\"test_robot\"><link name=\"base\"><visual><geometry><cylinder length=\"0.3\" radius=\"0.2\"/></geometry><origin xyz=\"0 0 0.15\"/></visual></link><link name=\"link1\"> <visual><geometry><cylinder length=\"0.15\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.075\"/></visual> </link><link name=\"link2\"><visual><geometry><cylinder length=\"0.35\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.175\"/></visual> </link><link name=\"link3\"><visual><geometry><cylinder length=\"0.45\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.225\"/></visual></link><link name=\"endeff\"><visual><geometry><cylinder length=\"0.05\" radius=\"0.1\"/></geometry><origin xyz=\"0 0 -0.025\"/></visual></link><joint name=\"joint1\" type=\"continuous\"><parent link=\"base\"/><child link=\"link1\"/><origin xyz=\"0 0 0.3\" rpy=\"0 0 0\" /><axis xyz=\"0 0 1\" /></joint><joint name=\"joint2\" type=\"continuous\"><parent link=\"link1\"/><child link=\"link2\"/><origin xyz=\"0 0 0.15\" rpy=\"0 0 0\" /><axis xyz=\"0 1 0\" /></joint><joint name=\"joint3\" type=\"continuous\"><parent link=\"link2\"/><child link=\"link3\"/><origin xyz=\"0 0 0.35\" rpy=\"0 0 0\" /><axis xyz=\"0 1 0\" /></joint><joint name=\"joint4\" type=\"fixed\"><parent link=\"link3\"/><child link=\"endeff\"/><origin xyz=\"0 0 0.45\" rpy=\"0 0 0\" /></joint></robot>";
std::string srdf_string="<robot name=\"test_robot\"><group name=\"arm\"><chain base_link=\"base\" tip_link=\"endeff\" /></group><virtual_joint name=\"world_joint\" type=\"fixed\" parent_frame=\"world_frame\" child_link=\"base\" /><group_state name=\"zero\" group=\"arm\"><joint name=\"joint1\" value=\"0\" /><joint name=\"joint2\" value=\"0.3\" /><joint name=\"joint3\" value=\"0.55\" /></group_state><disable_collisions link1=\"base\" link2=\"link1\" reason=\"Adjacent\" /><disable_collisions link1=\"endeff\" link2=\"link3\" reason=\"Adjacent\" /><disable_collisions link1=\"link1\" link2=\"link2\" reason=\"Adjacent\" /><disable_collisions link1=\"link2\" link2=\"link3\" reason=\"Adjacent\" /></robot>";

bool testCore()
{
    if(Setup::getSolvers().size()==0) {HIGHLIGHT_NAMED("EXOTica","Failed to find any solvers."); return false;}
    if(Setup::getProblems().size()==0) {HIGHLIGHT_NAMED("EXOTica","Failed to find any problems."); return false;}
    if(Setup::getMaps().size()==0) {HIGHLIGHT_NAMED("EXOTica","Failed to find any maps."); return false;}
    if(Setup::getTasks().size()==0) {HIGHLIGHT_NAMED("EXOTica","Failed to find any Tasks."); return false;}
    return true;
}

bool testManualInit()
{
    KinematicaInitializer kinematica(LimbInitializer("base"),{"joint1","joint2","joint3"});
    SceneInitializer scene("MyScene",kinematica);
    EffPositionInitializer map("Position","MyScene",false,{LimbInitializer("endeff"),LimbInitializer("endeff",Eigen::VectorTransform(0,0,0.5))});
    TaskSqrErrorInitializer task("MinimizeError","Position",1e2);
    Eigen::VectorXd W(3); W << 3,2,1;
    IKProblemInitializer problem("MyProblem",scene,{map},{task},1e-5,W);
    IKsolverInitializer solver("MySolver",false,1);
    Server::Instance()->getModel("robot_description",urdf_string,srdf_string);
    PlanningProblem_ptr any_problem = Setup::createProblem(problem);
    MotionSolver_ptr any_solver = Setup::createSolver(solver);
    return true;
}

bool testGenericInit()
{
    Initializer kinematica("Kinematica",std::map<std::string,boost::any>({
                    { "Root",Initializer("Limb",{ {"Segment",std::string("base")} }) },
                    { "Joints",std::string("joint1,joint2,joint3") }
                             }));
    Initializer scene("Scene",{{"Name",std::string("MyScene")},{"Solver",kinematica}});
    Initializer map("exotica/EffPosition",{
                        {"Name",std::string("Position")},
                        {"Scene",std::string("MyScene")},
                        {"EndEffector",std::vector<Initializer>({
                             Initializer("Limb",{{"Segment",std::string("endeff")}}),
                             Initializer("Limb",{{"Segment",std::string("endeff")},{"Frame",Eigen::VectorTransform(0,0,0.5)}})
                                        }) } });
    Initializer task("exotica/TaskSqrError",{
                         {"Name",std::string("MinimizeError")},
                         {"Map",std::string("Position")},
                         {"Rho",1e2}
                     });
    Eigen::VectorXd W(3);W << 3,2,1;
    Initializer problem("exotica/IKProblem",{
                            {"Name",std::string("MyProblem")},
                            {"PlanningScene",scene},
                            {"Maps",std::vector<Initializer>({map})},
                            {"Tasks",std::vector<Initializer>({task})},
                            {"W",W},
                            {"Tolerance",1e-5},
                        });
    Initializer solver("exotica/IKsolver",{
                            {"Name",std::string("MySolver")},
                            {"MaxIt",1},
                        });
    Server::Instance()->getModel("robot_description",urdf_string,srdf_string);
    PlanningProblem_ptr any_problem = Setup::createProblem(problem);
    MotionSolver_ptr any_solver = Setup::createSolver(solver);
    return true;
}

bool testXMLInit()
{
    std::string XMLstring = "<IKSolverDemoConfig><IKsolver Name=\"MySolver\"><MaxIt>1</MaxIt></IKsolver><IKProblem Name=\"MyProblem\"><PlanningScene><Scene Name=\"MyScene\"><PlanningMode>Optimization</PlanningMode><Solver><Kinematica><Root><Limb Segment=\"base\"/></Root><Joints>joint1,joint2,joint3</Joints></Kinematica></Solver></Scene></PlanningScene><Maps><EffPosition Name=\"Position\"><Scene>MyScene</Scene><EndEffector><Limb Segment=\"endeff\" /><Limb Segment=\"endeff\"><Frame>0.0 0.0 0.5 0.0 0.0 0.0 1.0</Frame></Limb></EndEffector></EffPosition></Maps><Tasks><TaskSqrError Name=\"MinimizeError\"><Map>Position</Map><Rho>1e2</Rho></TaskSqrError></Tasks><Tolerance>1e-5</Tolerance><W> 3 2 1 </W></IKProblem></IKSolverDemoConfig>";
    Initializer solver, problem;
    XMLLoader::load(XMLstring,solver, problem,"","",true);
    PlanningProblem_ptr any_problem = Setup::createProblem(problem);
    MotionSolver_ptr any_solver = Setup::createSolver(solver);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "EXOTica_test_initializers");
    if(!testCore()) exit(2); HIGHLIGHT_NAMED("EXOTica","Core test passed.");
    if(!testManualInit()) exit(2); HIGHLIGHT_NAMED("EXOTica","Manual initialization test passed.");
    if(!testGenericInit()) exit(2); HIGHLIGHT_NAMED("EXOTica","Generic initialization test passed.");
    if(!testXMLInit()) exit(2); HIGHLIGHT_NAMED("EXOTica","XML initialization test passed.");
    Setup::Destroy();
    return 0;
}

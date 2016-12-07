#include <exotica/Exotica.h>
#include "exotica/Initialiser.h"
#include <ik_solver/IKsolverInitializer.h>
#include <task_map/EffPositionInitializer.h>
#include <exotica/TaskSqrErrorInitializer.h>
#include <exotica/IKProblemInitializer.h>

using namespace exotica;

bool testCore()
{
    if(Initialiser::getSolvers().size()==0) {HIGHLIGHT_NAMED("EXOTica","Failed to find any solvers."); return false;}
    if(Initialiser::getProblems().size()==0) {HIGHLIGHT_NAMED("EXOTica","Failed to find any problems."); return false;}
    if(Initialiser::getMaps().size()==0) {HIGHLIGHT_NAMED("EXOTica","Failed to find any maps."); return false;}
    if(Initialiser::getTasks().size()==0) {HIGHLIGHT_NAMED("EXOTica","Failed to find any Tasks."); return false;}
    return true;
}

bool testManualInit()
{
    KinematicaInitializer kinematica(LimbInitializer("base"),
      {"lwr_arm_0_joint","lwr_arm_1_joint","lwr_arm_2_joint","lwr_arm_3_joint","lwr_arm_4_joint","lwr_arm_5_joint","lwr_arm_6_joint"});
    SceneInitializer scene("MyScene",kinematica);
    EffPositionInitializer map("Position","MyScene",false,
      {LimbInitializer("lwr_arm_6_link"),LimbInitializer("lwr_arm_6_link",Eigen::VectorTransform(0,0,0.5))});
    TaskSqrErrorInitializer task("MinimizeError","Position",1e2);
    Eigen::VectorXd W(7);
    W << 7,6,5,4,3,2,1;
    IKProblemInitializer problem("MyProblem",scene,{map},{task},1e-5,W);
    IKsolverInitializer solver("MySolver",false,1);
    PlanningProblem_ptr any_problem = Initialiser::createProblem(problem);
    MotionSolver_ptr any_solver = Initialiser::createSolver(solver);
    return true;
}

bool testGenericInit()
{

}

bool testXMLInit()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "EXOTica_test_initializers");
    if(!testCore()) exit(2);
    if(!testManualInit()) exit(2);
    HIGHLIGHT_NAMED("EXOTica","Core test passed.");
    return 0;
}

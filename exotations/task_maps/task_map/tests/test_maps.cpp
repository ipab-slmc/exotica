#include <exotica/Exotica.h>

using namespace exotica;

std::string urdf_string="<robot name=\"test_robot\"><link name=\"base\"><visual><geometry><cylinder length=\"0.3\" radius=\"0.2\"/></geometry><origin xyz=\"0 0 0.15\"/></visual></link><link name=\"link1\"> <visual><geometry><cylinder length=\"0.15\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.075\"/></visual> </link><link name=\"link2\"><visual><geometry><cylinder length=\"0.35\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.175\"/></visual> </link><link name=\"link3\"><visual><geometry><cylinder length=\"0.45\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.225\"/></visual></link><link name=\"endeff\"><visual><geometry><cylinder length=\"0.05\" radius=\"0.1\"/></geometry><origin xyz=\"0 0 -0.025\"/></visual></link><joint name=\"joint1\" type=\"continuous\"><parent link=\"base\"/><child link=\"link1\"/><origin xyz=\"0 0 0.3\" rpy=\"0 0 0\" /><axis xyz=\"0 0 1\" /><limit effort=\"200\"  velocity=\"1.0\" lower=\"-0.4\" upper=\"0.4\"/></joint><joint name=\"joint2\" type=\"continuous\"><parent link=\"link1\"/><child link=\"link2\"/><origin xyz=\"0 0 0.15\" rpy=\"0 0 0\" /><axis xyz=\"0 1 0\" /><limit effort=\"200\"  velocity=\"1.0\" lower=\"-0.4\" upper=\"0.4\"/></joint><joint name=\"joint3\" type=\"continuous\"><parent link=\"link2\"/><child link=\"link3\"/><origin xyz=\"0 0 0.35\" rpy=\"0 0 0\" /><axis xyz=\"0 1 0\" /><limit effort=\"200\"  velocity=\"1.0\" lower=\"-0.4\" upper=\"0.4\"/></joint><joint name=\"joint4\" type=\"fixed\"><parent link=\"link3\"/><child link=\"endeff\"/><origin xyz=\"0 0 0.45\" rpy=\"0 0 0\" /></joint></robot>";
std::string srdf_string="<robot name=\"test_robot\"><group name=\"arm\"><chain base_link=\"base\" tip_link=\"endeff\" /></group><virtual_joint name=\"world_joint\" type=\"fixed\" parent_frame=\"world_frame\" child_link=\"base\" /><group_state name=\"zero\" group=\"arm\"><joint name=\"joint1\" value=\"0\" /><joint name=\"joint2\" value=\"0.3\" /><joint name=\"joint3\" value=\"0.55\" /></group_state><disable_collisions link1=\"base\" link2=\"link1\" reason=\"Adjacent\" /><disable_collisions link1=\"endeff\" link2=\"link3\" reason=\"Adjacent\" /><disable_collisions link1=\"link1\" link2=\"link2\" reason=\"Adjacent\" /><disable_collisions link1=\"link2\" link2=\"link3\" reason=\"Adjacent\" /></robot>";

UnconstrainedEndPoseProblem_ptr setupProbelm(Initializer& map)
{
    Initializer kinematica("Kinematica",std::map<std::string,boost::any>({
                    { "Root",Initializer("Limb",{ {"Segment",std::string("base")} }) },
                    { "Joints",std::string("joint1,joint2,joint3") }
                             }));
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
    HIGHLIGHT("End Effector position test");
    Initializer map("exotica/EffPosition",{
                        {"Name",std::string("Position")},
                        {"EndEffector",std::vector<Initializer>({
                             Initializer("Limb",{{"Segment",std::string("endeff")}})
                                        }) } });
    UnconstrainedEndPoseProblem_ptr problem = setupProbelm(map);
    Eigen::VectorXd x(3);
    INFO("Testing random configurations:")
    for(int i=0;i<5;i++)
    {
        x.setRandom();
        problem->Update(x);
        INFO("x = "<<x.transpose());
        INFO(problem->Phi.transpose());
        INFO("J = \n"<<problem->J);
    }
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
    Eigen::VectorXd x(3);
    INFO("Testing random configurations:")
    for(int i=0;i<5;i++)
    {
        x.setRandom();
        problem->Update(x);
        INFO("x = "<<x.transpose());
        INFO(problem->Phi.transpose());
        INFO("J = \n"<<problem->J);
    }
}

void testJointLimit()
{
    HIGHLIGHT("Joint Limit test");
    Initializer map("exotica/JointLimit",{
                        {"Name",std::string("JointLimit")},
                        {"SafePercentage", 0.5} });
    UnconstrainedEndPoseProblem_ptr problem = setupProbelm(map);
    Eigen::VectorXd x(3);
    INFO("Testing random configurations:")
    for(int i=0;i<5;i++)
    {
        x.setRandom();
        problem->Update(x);
        INFO("x = "<<x.transpose());
        INFO(problem->Phi.transpose());
        INFO("J = \n"<<problem->J);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "EXOTica_test_maps");
    testEffPosition();
    testDistance();
    testJointLimit();
    Setup::Destroy();
    return 0;
}

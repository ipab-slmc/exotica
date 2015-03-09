/*
 * kinematic_scene.cpp
 *
 *  Created on: 13 Mar 2014
 *      Author: yiming
 */
#include "kinematic_scene/kinematic_scene.h"
//#define DEBUG_MODE

#ifdef DEBUG_MODE
#define CHECK_EXECUTION         std::cout << "Ok in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\n"; //!< With endline
#define INDICATE_FAILURE        std::cerr << "Failed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\n";//!< With endline
#define WARNING(x)							 std::clog << "Warning in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ": " << x << "\n";//!< With endline
#define ERROR(x)								 std::cerr << "Failed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\n" << x << "\n";//!< With endline
#else
#define CHECK_EXECUTION   //!< No operation
#define INDICATE_FAILURE  //!< No operation
#define WARNING(x)        //!< No operation
#define ERROR(x)
#endif

int kinematica::KinematicScene::getMapSize()
{
	if (initialised_)
	{
		return kinematic_solver_.getEffSize() + ext_ids_.size();
	}
	else
	{
		ERROR("Kinematic scene has not been initialised!");
		return 0;
	}
}

int kinematica::KinematicScene::getNumJoints()
{
    return kinematic_solver_.getNumJoints();
}

kinematica::KinematicScene::KinematicScene(bool useSpinner) :
				nh_("KinematicScene_node"),
				initialised_(false),
				useSpinner_(useSpinner),
				name_("KinematicScene")
{
	robot_model::RobotModelPtr model =
			robot_model_loader::RobotModelLoader("robot_description").getModel();
	if (!model)
	{
		ROS_ERROR_STREAM("Could not load robot model from 'robot_description' parameter!");
		return;
	}
	ps_.reset(new planning_scene::PlanningScene(model));
	initialised_ = true;
	scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1, true);
	isPublishing_ = false;

	if (useSpinner_)
	{
		ROS_WARN("Kinematic Scene is using separate spinner.");
		initSrv("/get_planning_scene");
		spinner_.reset(new ros::AsyncSpinner(1));
		spinner_->start();
		updateScene();
	}
}

std::string kinematica::KinematicScene::getRootName()
{
    return task_root_;
}

kinematica::KinematicScene::KinematicScene(const std::string & name, bool useSpinner) :
		nh_(name + "_node"), initialised_(false), useSpinner_(useSpinner), name_(name)
{
	robot_model::RobotModelPtr model =
			robot_model_loader::RobotModelLoader("robot_description").getModel();
	if (!model)
	{
		ROS_ERROR_STREAM("Could not load robot model from 'robot_description' parameter!");
		return;
	}
	ps_.reset(new planning_scene::PlanningScene(model));
	initialised_ = true;
	scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1, true);
	isPublishing_ = false;

	if (useSpinner_)
	{
		ROS_WARN("Kinematic Scene is using separate spinner.");
		initSrv("/get_planning_scene");
		spinner_.reset(new ros::AsyncSpinner(1));
		spinner_->start();
		updateScene();
	}
}

kinematica::KinematicScene::KinematicScene(const std::string & name,
		const std::string & robot_description, const std::string & scene_srv,
		const std::string & scene_name, bool useSpinner) :
		nh_(name + "_node"), initialised_(false), useSpinner_(useSpinner)
{
	robot_model::RobotModelPtr model =
			robot_model_loader::RobotModelLoader("robot_description").getModel();
	if (!model)
	{
		ROS_ERROR_STREAM("Could not load robot model from 'robot_description' parameter!");
		return;
	}
	ps_.reset(new planning_scene::PlanningScene(model));
	initialised_ = true;
	scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>(scene_name, 1, true);
	isPublishing_ = false;

	if (useSpinner_)
	{
		ROS_WARN("Kinematic Scene is using separate spinner.");
		initSrv(scene_srv);
		spinner_.reset(new ros::AsyncSpinner(1));
		spinner_->start();
		updateScene();
	}

}

/*kinematica::KinematicScene::KinematicScene(const KinematicScene & ks):
 spinner_(1)
 {
 nh_ = ks.nh_;

 spinner_.start();
 std::string topic = ks.scene_pub_.getTopic();
 scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>(topic, 1, true);
 //topic = const_cast<kinematica::KinematicScene>(ks).get_scene_client_.getService();
 initSrv("/get_planning_scene");
 //ps_->getRobotModel()->getURDF()->getName();
 ps_.reset(new planning_scene::PlanningSDefaultKinematicScenecene(robot_model_loader::RobotModelLoader("robot_description").getModel()));
 eff_world_map_ = ks.eff_world_map_;
 task_root_ = ks.task_root_;
 ext_ids_ = ks.ext_ids_;
 isPublishing_ = ks.isPublishing_;
 }

 kinematica::KinematicScene & kinematica::KinematicScene::operator=(
 const kinematica::KinematicScene & rhs)
 {
 nh_ = rhs.nh_;

 spinner_ = ros::AsyncSpinner(1);
 spinner_.start();
 std::string topic = rhs.scene_pub_.getTopic();
 scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>(topic, 1, true);
 //topic = const_cast<kinematica::KinematicScene>(rhs).get_scene_client_.getService();
 initSrv("/get_planning_scene");
 //ps_->getRobotModel()->getURDF()->getName();
 ps_.reset(new planning_scene::PlanningScene(robot_model_loader::RobotModelLoader("robot_description").getModel()));
 eff_world_map_ = rhs.eff_world_map_;
 task_root_ = rhs.task_root_;
 ext_ids_ = rhs.ext_ids_;
 isPublishing_ = rhs.isPublishing_;
 return *this;
 }
 */

void kinematica::KinematicScene::initSrv(const std::string & scene_srv)
{
	get_scene_client_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>(scene_srv);
	get_scene_srv_.request.components.components = get_scene_srv_.request.components.SCENE_SETTINGS
			| get_scene_srv_.request.components.ROBOT_STATE
			| get_scene_srv_.request.components.ROBOT_STATE_ATTACHED_OBJECTS
			| get_scene_srv_.request.components.WORLD_OBJECT_NAMES
			| get_scene_srv_.request.components.WORLD_OBJECT_GEOMETRY
			| get_scene_srv_.request.components.OCTOMAP
			| get_scene_srv_.request.components.TRANSFORMS
			| get_scene_srv_.request.components.ALLOWED_COLLISION_MATRIX
			| get_scene_srv_.request.components.LINK_PADDING_AND_SCALING
			| get_scene_srv_.request.components.OBJECT_COLORS;
}
bool kinematica::KinematicScene::initKinematicScene(const std::string & urdf_file,
		const kinematica::KinematicSceneForm_t & ks_form)
{
	/** We need to change the root offset with respect to the world frame */
	kinematica::SolutionForm_t tmp_opt = ks_form.optimisation;
	if (!ps_->knowsFrameTransform("/" + ks_form.optimisation.root_segment))
	{
#ifdef DEBUG_MODE
		KS_INFO("World frame: (" + ps_->getPlanningFrame() + ") to root frame ("
				+ ks_form.optimisation.root_segment + ") is not available.");
		INDICATE_FAILURE
		;
		return false;
#endif
	}

	tf::transformEigenToKDL(ps_->getFrameTransform("/" + ks_form.optimisation.root_segment), tmp_opt.root_seg_off);

	if (kinematic_solver_.initKinematics(urdf_file, tmp_opt))
	{
#ifdef DEBUG_MODE
		KS_INFO("Kinematica Initialised");
#endif
		initialised_ = true;
	}
	else
	{
		KS_INFO("Kinematica Initialisation Failed");
		initialised_ = false;
		INDICATE_FAILURE
		;
		return false;
	}
	task_root_ = ks_form.optimisation.root_segment;
	eff_world_map_ = ks_form.eff_world_map;
	ext_ids_ = ks_form.ext_ids;
	return true;
}

bool kinematica::KinematicScene::initKinematicScene(tinyxml2::XMLHandle & handle)
{
#ifdef DEBUG_MODE
	KS_INFO("Initialising from XML");
#endif
	if (!handle.FirstChildElement("Kinematica").ToElement())
	{
#ifdef DEBUG_MODE
		KS_INFO("Kinematica element not exist");
#endif
		INDICATE_FAILURE
		;
		return false;
	}
	tinyxml2::XMLHandle kinematica_handle(handle.FirstChildElement("Kinematica"));
	if (!kinematic_solver_.initKinematics(kinematica_handle))
	{
#ifdef DEBUG_MODE
		KS_INFO("Kinematica XML Initialisation Failed");
#endif
		INDICATE_FAILURE
		;
		return false;
	}
#ifdef DEBUG_MODE
	KS_INFO("Kinematica initialisation succeeded");
#endif
	tinyxml2::XMLHandle map_handle(handle.FirstChildElement("EffWorldMap").FirstChildElement("EndEffector"));
	eff_world_map_.clear();
	while (map_handle.ToElement())
	{
		std::string seg_name(map_handle.ToElement()->Attribute("segment")),
				obj_name(map_handle.ToElement()->Attribute("object"));
		if (seg_name.size() == 0 || obj_name.size() == 0)
		{
#ifdef DEBUG_MODE
			KS_INFO("Invalid EndEffector to World Map");
#endif
			INDICATE_FAILURE
			;
			return false;
		}
		eff_world_map_[seg_name] = obj_name;
		map_handle = map_handle.NextSiblingElement("EndEffector");
	}
	tinyxml2::XMLHandle ext_handle(handle.FirstChildElement("ExternalObjects").FirstChildElement("object"));
	ext_ids_.clear();
	while (ext_handle.ToElement())
	{
		std::string ext_name(ext_handle.ToElement()->Attribute("name"));
		if (ext_name.size() == 0)
		{
#ifdef DEBUG_MODE
			KS_INFO("Invaild External Object Name");
#endif
			INDICATE_FAILURE
			;
			return false;
		}
		ext_ids_.push_back(ext_name);
		ext_handle = ext_handle.NextSiblingElement("object");
	}
	task_root_ =
			handle.FirstChildElement("Kinematica").FirstChildElement("Root").ToElement()->Attribute("segment");
	if (task_root_.size() == 0)
	{
		INDICATE_FAILURE
		;
		return false;
	}
	return true;
}

std::string kinematica::KinematicScene::getName()
{
	return name_;
}

bool kinematica::KinematicScene::update(
        const Eigen::Ref<const Eigen::VectorXd> & joint_configuration, const int t)
{
	if (!kinematic_solver_.updateConfiguration(joint_configuration))
	{
#ifdef DEBUG_MODE
		KS_INFO("Kinematica Update Failed");
#endif
		return false;
	}
	if (!updateSceneToKinematica())
	{
#ifdef DEBUG_MODE
		KS_INFO("Update Scene to Kinematica Failed");
		return false;
#endif
	}
	if (!kinematic_solver_.generateForwardMap())
	{
#ifdef DEBUG_MODE
		KS_INFO("Generating Forward Mapping Failed");
#endif
		return false;
	}
	if (!kinematic_solver_.generateJacobian())
	{
#ifdef DEBUG_MODE
		KS_INFO("Generating Forward Mapping Failed");
#endif
		return false;
	}
	return true;
}

double kinematica::KinematicScene::distanceToCollision()
{
	if (!initialised_)
		return 0;
	boost::mutex::scoped_lock(locker_);
	return ps_.get()->distanceToCollision(ps_.get()->getCurrentState());
}

planning_scene::PlanningScenePtr kinematica::KinematicScene::getPlanningScene()
{
	boost::mutex::scoped_lock(locker_);
	return ps_;
}

kinematica::KinematicTree & kinematica::KinematicScene::getSolver()
{
	return kinematic_solver_;
}

bool kinematica::KinematicScene::isInitialised()
{
	return initialised_;
}

bool kinematica::KinematicScene::getInitialEff(std::vector<std::string> & segs,
		std::vector<KDL::Frame> & offsets)
{
	if (!initialised_)
		kinematic_solver_.getInitialEff(segs, offsets);
	return true;
}

bool kinematica::KinematicScene::getCoMProperties(std::vector<std::string> & segs,
		Eigen::VectorXd & mass, std::vector<KDL::Vector> & cog, std::vector<KDL::Frame> & tip_pose,
		std::vector<KDL::Frame> & base_pose)
{
	if (!kinematic_solver_.getCoMProperties(segs, mass, cog, tip_pose, base_pose))
		return false;
	return true;
}

bool kinematica::KinematicScene::updateEndEffectors(kinematica::SolutionForm_t & tmp_sol)
{
	if (!kinematic_solver_.updateEndEffectors(tmp_sol))
		return false;
	return true;
}

bool kinematica::KinematicScene::getForwardMap(Eigen::Ref<Eigen::VectorXd> phi,
		std::vector<std::string> & unknown_objects)
{
	boost::mutex::scoped_lock(locker_);
	unknown_objects.clear();
//	if (phi.rows() != getMapSize() * 3)
//	{
//		std::cout<<"Wrong size, mapsize="<<getMapSize()<<", phi size="<<phi.rows()<<std::endl;
//		return false;
//	}
	if (!kinematic_solver_.getPhi(phi.segment(0, kinematic_solver_.getEffSize() * 3)))
		return false;
	uint E = ext_ids_.size(), i;
	Eigen::VectorXd tmp_ext(3 * E);
	KDL::Frame root_w, ext_w, ext_root;
	if (!ps_->knowsFrameTransform("/" + task_root_))
	{
#ifdef DEBUG_MODE
		KS_INFO("Root transform [" + task_root_ + "] is unknown");
#endif
		return false;
	}
	tf::transformEigenToKDL(ps_->getFrameTransform("/" + task_root_), root_w);
    //kinematic_solver_.modifyRootOffset(root_w);
	for (i = 0; i < E; i++)
	{
		if (!ps_->knowsFrameTransform("/" + ext_ids_[i]))
		{
#ifdef DEBUG_MODE
			KS_INFO("Object transform [" + ext_ids_[i] + "] is unknown");
#endif
			unknown_objects.push_back("ext_ids_[i]");
			tmp_ext(3 * i) = INF;
			tmp_ext(3 * i + 1) = INF;
			tmp_ext(3 * i + 2) = INF;
			continue;
		}
		tf::transformEigenToKDL(ps_->getFrameTransform("/" + ext_ids_[i]), ext_w);
		ext_root = root_w.Inverse() * ext_w;
		tmp_ext(3 * i) = ext_root.p.x();
		tmp_ext(3 * i + 1) = ext_root.p.y();
		tmp_ext(3 * i + 2) = ext_root.p.z();
	}
	if (tmp_ext.rows() > 0)
		phi.segment(kinematic_solver_.getEffSize(), tmp_ext.rows()) = tmp_ext;
	return true;
}

bool kinematica::KinematicScene::getJacobian(Eigen::Ref<Eigen::MatrixXd> jac)
{
	boost::mutex::scoped_lock(locker_);
	if (!kinematic_solver_.getJacobian(jac))
	{
#ifdef DEBUG_MODE
		KS_INFO("Getting Forward Mapping Failed");
#endif
		return false;
	}
	return true;
}

bool kinematica::KinematicScene::updateSceneToKinematica()
{
	boost::mutex::scoped_lock(locker_);
	std::map<std::string, std::string>::iterator it;
	KDL::Frame eef_w, link_w, eef_link;
	kinematica::SolutionForm_t new_solution;
	for (it = eff_world_map_.begin(); it != eff_world_map_.end(); ++it)
	{
		if (!ps_->knowsFrameTransform("/" + it->first))
		{
#ifdef DEBUG_MODE
			KS_INFO("Frame Transform of [" + it->first + "] is unknown");
#endif
			return false;
		}
		if (!ps_->knowsFrameTransform("/" + it->second))
		{
#ifdef DEBUG_MODE
			KS_INFO("Frame Transform of [" + it->second + "] is unknown");
#endif
			//return false;
			//If the transform of the attached object is unknown, we just
			//use old one, rather than return a false
			continue;
		}

		tf::transformEigenToKDL(ps_->getFrameTransform("/" + it->first), link_w);
		tf::transformEigenToKDL(ps_->getFrameTransform("/" + it->second), eef_w);
		eef_link = link_w.Inverse() * eef_w;

		/** Now we update Kinematica offset */
		if (!kinematic_solver_.modifyEndEffector(it->first, eef_link))
		{
#ifdef DEBUG_MODE
			KS_INFO("Update end-effector [" + it->first + "] failed");
#endif
			return false;
		}
	}
	return true;
}

bool kinematica::KinematicScene::getPoses(const std::vector<std::string> names,
		std::vector<KDL::Frame> & poses)
{
	boost::mutex::scoped_lock(locker_);
	uint size = names.size(), i;
	poses.resize(size);
	for (i = 0; i < size; i++)
	{
		if (kinematic_solver_.getPose(names[i], poses[i]))
		{
			continue;
		}
		else if (ps_->knowsFrameTransform(names[i]))
		{

			tf::transformEigenToKDL(ps_->getFrameTransform(names[i]), poses[i]);
		}
		else
		{
#ifdef DEBUG_MODE
			KS_INFO("Frame Transform of [" + names[i] + "] is not available");
#endif
			poses.resize(0);
			return false;
		}
	}
	return true;
}

const collision_detection::AllowedCollisionMatrix& kinematica::KinematicScene::getACM() const
{
	return ps_->getAllowedCollisionMatrix();
}
bool kinematica::KinematicScene::addObject(const KS_Object object)
{
	//TODO
	//We might want to add objects into our own scene without affect the original scene
	//but right now, we can just add things through moveit scene
	return true;
}

void kinematica::KinematicScene::startPublishing()
{
#ifdef DEBUG_MODE
	KS_INFO("Start Publishing Robot State To The Scene");
#endif
	boost::mutex::scoped_lock(locker_);
	isPublishing_ = true;
}

void kinematica::KinematicScene::stopPublishing()
{
#ifdef DEBUG_MODE
	KS_INFO("Stop Publishing Robot State To The Scene");
#endif
	boost::mutex::scoped_lock(locker_);
	isPublishing_ = false;
}

void kinematica::KinematicScene::publishScene()
{
	boost::mutex::scoped_lock(locker_);
	if (isPublishing_)
	{
		moveit_msgs::PlanningScene planning_scene;
		ps_->getPlanningSceneMsg(planning_scene);
		scene_pub_.publish(planning_scene);
	}
}

bool kinematica::KinematicScene::updateScene()
{
	boost::mutex::scoped_lock(locker_);
	if (useSpinner_)
	{
		if (!get_scene_client_.call(get_scene_srv_))
		{
			KS_INFO("Can't get Planning Scene");
			return false;
		}

		ps_->usePlanningSceneMsg(get_scene_srv_.response.scene);
#ifdef DEBUG_MODE
		KS_INFO("Updating Planning Scene From Moveit.");
#endif
		return true;
	}
	else
		return false;
}

bool kinematica::KinematicScene::updateScene(const planning_scene::PlanningSceneConstPtr & scene)
{
	boost::mutex::scoped_lock(locker_);
	moveit_msgs::PlanningScene tmp;
	scene->getPlanningSceneMsg(tmp);
	ps_->usePlanningSceneMsg(tmp);
	KS_INFO("Updating " + name_ + " From Moveit.");
	return true;
}

void kinematica::KinematicScene::sceneCallback(const moveit_msgs::PlanningScene::ConstPtr & scene)
{
#ifdef DEBUG_MODE
	KS_INFO("Setting Planning Scene From Moveit (" + scene->name + ").");
#endif
	boost::mutex::scoped_lock(locker_);
	ps_->setPlanningSceneDiffMsg(*scene);
}

void kinematica::KinematicScene::KS_INFO(std::string info)
{
	std::cout << "Kinematic Scene: " << info << std::endl;
}

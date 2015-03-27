/*
 * exotica_collision.cpp
 *
 *  Created on: 8 Aug 2014
 *      Author: yiming
 */

#include "exotica_collision.h"

REGISTER_TASKMAP_TYPE("CollisionAvoidance", exotica::CollisionAvoidance);
#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) {INDICATE_FAILURE; return PAR_ERR;}}
#define XML_OK(x) if(!ok(x)){INDICATE_FAILURE; return PAR_ERR;}
namespace exotica
{
	CollisionAvoidance::CollisionAvoidance() :
			m_(0.05), initialised_(false), nh_("CollisionTask"), publishDebug_(false)
	{
		//TODO
		wall_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("wall_marker", 1);
		state_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("disp_state", 1);
		close_pub_ = nh_.advertise<visualization_msgs::Marker>("close_marker", 1);

	}

	CollisionAvoidance::~CollisionAvoidance()
	{
		//TODO
	}

	EReturn CollisionAvoidance::setPreUpdateCallback(
			boost::function<void(CollisionAvoidance*, const Eigen::VectorXd &, int)> pre_update_callback)
	{
		pre_update_callback_ = pre_update_callback;
	}

	EReturn CollisionAvoidance::update(const Eigen::VectorXd & x, const int t)
	{
		LOCK(lock_);
		invalidate();
		if (!initialised_)
		{
			return MMB_NIN;
		}
		if (!scene_->update(x, t) || !solver_->updateConfiguration(x))
		{
			return FAILURE;
		}
		if (pre_update_callback_)
			pre_update_callback_(this, x, t);
		dist_info_.resetDistance();
		if (!ok(computeDistace(x)))
			return FAILURE;
		if (!ok(setPhi(computeForwardMap(), t)))
			return FAILURE;
		if (!ok(setJacobian(computeJacobian(x.rows()), t)))
			return FAILURE;

		return SUCCESS;
	}

	EReturn CollisionAvoidance::initDerived(tinyxml2::XMLHandle & handle)
	{
		tinyxml2::XMLElement* xmltmp;
		if (handle.FirstChildElement("margin").ToElement())
		{
			XML_CHECK("margin");
			XML_OK(getDouble(*xmltmp, m_));
			std::cout << "Collision Detection: New margin = " << m_ << std::endl;
		}
		else
		{
			std::cout << "Collision Detection: Using default margin = " << m_ << "\n";
		}

		tinyxml2::XMLHandle tmp_handle = handle.FirstChildElement("DynamicFrame");
		server_->registerParam<std_msgs::Bool>(ns_, tmp_handle, dynamic_frame_);
		if (!handle.FirstChildElement("DynamicFrame").ToElement())
		{
			tmp_handle = handle.FirstChildElement("WorldFrame");
			server_->registerParam<std_msgs::String>(ns_, tmp_handle, world_frame_);
			tmp_handle = handle.FirstChildElement("ObjectFrame");
			server_->registerParam<std_msgs::String>(ns_, tmp_handle, obj_frame_);
		}
		else
		{
			dynamic_frame_->data = false;
		}
		tmp_handle = handle.FirstChildElement("LAAS");
		server_->registerParam<std_msgs::Bool>(ns_, tmp_handle, laas_);
		tmp_handle = handle.FirstChildElement("useAll");
		server_->registerParam<std_msgs::Bool>(ns_, tmp_handle, useAll_);

		if (scene_ == nullptr)
			return MMB_NIN;
		solver_.reset(new kinematica::KinematicTree(scene_->getSolver()));
		solver_->getInitialEff(initial_sol_.end_effector_segs, initial_sol_.end_effector_offs);
		std::vector<KDL::Segment> segs;
		solver_->getSegments(segs);
		if (!solver_->getControlledSegmentsAndJoints(links_, joints_))
			return MMB_NIN;

		if (useAll_->data)
		{
			std::map<std::string, bool> ignore_list_;
			//ignore_list_["HEAD_LINK1"] = true;
			//ignore_list_["LLEG_LINK0"] = true;
			//ignore_list_["RLEG_LINK0"] = true;
			ignore_list_["l_ankle"] = true;
			ignore_list_["r_ankle"] = true;
			ignore_list_["RHAND_LINK0"] = true;
			ignore_list_["RHAND_LINK1"] = true;
			ignore_list_["RHAND_LINK2"] = true;
			ignore_list_["RHAND_LINK3"] = true;
			ignore_list_["RHAND_LINK4"] = true;
			ignore_list_["LHAND_LINK0"] = true;
			ignore_list_["LHAND_LINK1"] = true;
			ignore_list_["LHAND_LINK2"] = true;
			ignore_list_["LHAND_LINK3"] = true;
			ignore_list_["LHAND_LINK4"] = true;
			links_.clear();
			for (KDL::Segment & it : segs)
			{
				if (ignore_list_.find(it.getName()) == ignore_list_.end())
				{
					links_.push_back(it.getName());
				}
			}
			initial_sol_.end_effector_segs = links_;
			initial_sol_.end_effector_offs = std::vector<KDL::Frame>(links_.size());
			if (!solver_->updateEndEffectors(initial_sol_))
			{
				INDICATE_FAILURE
				;
				return FAILURE;
			}
		}

		dist_info_.initialise(links_);
		for (auto & it : dist_info_.link_dist_map_)
		{
			for (int i = 0; i < links_.size(); i++)
			{
				if (it.first.compare(links_[i]) == 0)
				{
					links_map_[links_[i]] = i;
					break;
				}
			}
		}

		//	\Construct the allowed collision matrix
		if (laas_->data)
		{
			for (int i = 1; i <= 11; i++)
			{
				std::vector<std::string> tmp(0);
				switch (i)
				{
					case 1:
						tmp.push_back("RLEG_LINK2");
						tmp.push_back("RLEG_LINK3");
						tmp.push_back("LLEG_LINK2");
						tmp.push_back("LLEG_LINK3");
						break;
					case 11:
						tmp.push_back("r_ankle");
						tmp.push_back("l_ankle");
						tmp.push_back("LLEG_LINK0");
						tmp.push_back("RLEG_LINK0");
						break;
					case 2:
						tmp.push_back("RARM_LINK4");
						tmp.push_back("r_wrist");
						tmp.push_back("LARM_LINK4");
						tmp.push_back("l_wrist");
						break;
					case 3:
						tmp.push_back("RARM_LINK0");
						tmp.push_back("RARM_LINK2");
						tmp.push_back("RARM_LINK3");
						tmp.push_back("RARM_LINK4");
						tmp.push_back("r_wrist");
						tmp.push_back("LARM_LINK0");
						tmp.push_back("LARM_LINK2");
						tmp.push_back("LARM_LINK3");
						tmp.push_back("LARM_LINK4");
						tmp.push_back("l_wrist");

						tmp.push_back("BODY");
						tmp.push_back("torso");
						break;
					case 4:
						tmp.push_back("HEAD_LINK1");
						break;
					case 5:
						//tmp.push_back("HEAD_LINK1");
						break;
					case 6:
						//tmp.push_back("HEAD_LINK1");
						break;
					case 7:
						tmp.push_back("HEAD_LINK1");
						tmp.push_back("torso");
						break;
					case 8:
						tmp.push_back("HEAD_LINK1");
						break;
					case 9:
						tmp.push_back("RARM_LINK4");
						tmp.push_back("r_wrist");
						tmp.push_back("LARM_LINK4");
						tmp.push_back("l_wrist");
						break;
					case 10:
						tmp.push_back("RARM_LINK4");
						tmp.push_back("r_wrist");
						tmp.push_back("LARM_LINK4");
						tmp.push_back("l_wrist");
						tmp.push_back("BODY");
						tmp.push_back("torso");
						tmp.push_back("RLEG_LINK0");
						tmp.push_back("RLEG_LINK2");
						tmp.push_back("LLEG_LINK0");
						tmp.push_back("LLEG_LINK2");
						break;
					default:
						break;
				}
				acm_["wall_" + std::to_string(i)] = tmp;
			}

			{
				wall_marker_.markers.resize(12);
				for (int i = 0; i < 11; i++)
				{
					wall_marker_.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
					wall_marker_.markers[i].color.a = 0.0;
					wall_marker_.markers[i].color.r = 1.0;
					wall_marker_.markers[i].color.g = 1.0;
					wall_marker_.markers[i].color.b = 1.0;
					wall_marker_.markers[i].scale.x = wall_marker_.markers[i].scale.y =
							wall_marker_.markers[i].scale.z = 1.0;
					wall_marker_.markers[i].mesh_resource =
							"package://hrp2_14_description/urdf/wall-extended"
									+ std::to_string(i + 1) + ".obj";
					wall_marker_.markers[i].mesh_use_embedded_materials = true;
					wall_marker_.markers[i].header.frame_id = "/BODY";
					wall_marker_.markers[i].action = visualization_msgs::Marker::ADD;
					wall_marker_.markers[i].id = i + 1;
					wall_marker_.markers[i].ns = "Collision geometry";
				}

				{
					int i = 11;
					wall_marker_.markers[i].type = visualization_msgs::Marker::SPHERE_LIST;
					wall_marker_.markers[i].color.a = 1.0;
					wall_marker_.markers[i].color.r = 1.0;
					wall_marker_.markers[i].color.g = 1.0;
					wall_marker_.markers[i].color.b = 1.0;
					wall_marker_.markers[i].scale.x = wall_marker_.markers[i].scale.y =
							wall_marker_.markers[i].scale.z = 0.1;
					wall_marker_.markers[i].header.frame_id = "/BODY";
					wall_marker_.markers[i].action = visualization_msgs::Marker::ADD;
					wall_marker_.markers[i].id = i + 1;
					wall_marker_.markers[i].ns = "Collision geometry centres";
					wall_marker_.markers[i].points.resize(11 + links_.size());
				}

				close_.type = visualization_msgs::Marker::LINE_LIST;
				close_.action = visualization_msgs::Marker::ADD;
				close_.header.frame_id = "/BODY";
				close_.scale.x = 0.004;
				close_.color.g = 1;
				close_.color.a = 1;
				close_.id = 0;
				close_.ns = "Collision distances";
			}
		}


		initialised_ = true;
		return SUCCESS;
	}

	EReturn CollisionAvoidance::taskSpaceDim(int & task_dim)
	{
		task_dim = 1;
		return SUCCESS;
	}

	EReturn CollisionAvoidance::getMargin(double m)
	{
		if (!initialised_)
			return MMB_NIN;
		m = m_;
		return SUCCESS;
	}

	EReturn CollisionAvoidance::setMargin(double m)
	{
		if (!initialised_)
			return MMB_NIN;
		m_ = m;
		return SUCCESS;
	}

	Eigen::VectorXd CollisionAvoidance::computeForwardMap()
	{
		Eigen::VectorXd phi(1);
		phi(0) = 0.0;
		double d = 0.0;
		bool isCollision = false;
		if (!dist_info_.isInitialised())
			return phi;
		for (auto & it : dist_info_.link_dist_map_)
		{
			for (int i = 0; i < it.second.size(); i++)
			{
				if (it.second[i].d > m_)
					it.second[i].cost = 0;
				else if (it.second[i].d <= 0.005)
				{
					isCollision = true;
					Eigen::Vector3d tmpnorm = it.second[i].c2 - it.second[i].c1;
					it.second[i].cost = 1.0;			// + 1.0 / tmpnorm.norm();
				}
				else
					it.second[i].cost = 1.0 - it.second[i].d / m_;
				phi(0) += (double) it.second[i].cost * it.second[i].cost;
			}
		}
		if (publishDebug_ && isCollision)
			ROS_ERROR_STREAM("Collision detected");
		return phi;
	}

	Eigen::MatrixXd CollisionAvoidance::computeJacobian(const int size)
	{
		int M = useAll_->data ? links_map_.size() : scene_->getMapSize(), N = size, cnt = 0;
		std::vector<double> cost(0);
		Eigen::MatrixXd jac(1, N);
		jac.setZero();
		KDL::Frame tip_offset, cp_offset;
		Eigen::VectorXd phi(3 * M);
		if (!solver_->generateForwardMap(phi))
			INDICATE_FAILURE

		kinematica::SolutionForm_t tmp_sol;
		tmp_sol.end_effector_offs.clear();
		tmp_sol.end_effector_segs.clear();

		eff_map_.clear();
		for (auto & it : dist_info_.link_dist_map_)
		{
			for (int i = 0; i < it.second.size(); i++)
			{
				tip_offset = KDL::Frame(KDL::Vector(phi(3 * links_map_.at(it.first)), phi(3
						* links_map_.at(it.first) + 1), phi(3 * links_map_.at(it.first) + 2)));
				cp_offset =
						KDL::Frame(KDL::Vector(it.second[i].p1(0), it.second[i].p1(1), it.second[i].p1(2)));
				KDL::Frame eff_offset = tip_offset.Inverse() * cp_offset;
				tmp_sol.end_effector_segs.push_back(it.first);
				tmp_sol.end_effector_offs.push_back(eff_offset);
				cnt++;
			}
		}
		if (!solver_->updateEndEffectors(tmp_sol))
			INDICATE_FAILURE
		if (!solver_->generateForwardMap())
			INDICATE_FAILURE
		Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3 * cnt, N);
		if (!solver_->generateJacobian(J))
			INDICATE_FAILURE

		cnt = 0;
		for (auto & it : dist_info_.link_dist_map_)
		{

			for (int i = 0; i < it.second.size(); i++)
			{
				if (it.second[i].d <= m_)
				{
					if (it.second[i].d <= 0.005)
					{
						Eigen::Vector3d tmpnorm = it.second[i].c2 - it.second[i].c1;
						tmpnorm.normalize();
						jac += ((2.0 * (1.0 - 0.005)) / m_)
								* (tmpnorm.transpose() * J.block(3 * cnt, 0, 3, N));
					}
					else
					{
						Eigen::Vector3d tmpnorm = it.second[i].p2 - it.second[i].p1;
						tmpnorm.normalize();
						jac += ((2.0 * it.second[i].cost) / m_)
								* (tmpnorm.transpose() * J.block(3 * cnt, 0, 3, N));
					}
				}
				cnt++;
			}

		}
		if (!solver_->updateEndEffectors(initial_sol_))
			INDICATE_FAILURE
		return jac;
	}

	EReturn CollisionAvoidance::computeDistace(const Eigen::VectorXd & x)
	{
		LOCK(scene_lock_);
		if (scene_ == nullptr || !initialised_)
			return MMB_NIN;
		collision_detection::CollisionWorldEXOTica cworld_(scene_->getPlanningScene().get()->getWorldNonConst());
		robot_state::RobotState state(scene_->getPlanningScene().get()->getCurrentState());

		std::map<std::string, double> joint_state_map;
		for (int i = 0; i < joints_.size(); i++)
		{
			state.setJointPositions(joints_[i], &x(i));
		}
		state.update();
		collision_detection::CollisionRobotEXOTica crobot_(state.getRobotModel());
		cworld_.distanceRobot(crobot_, state);

		//Currently only mesh-mesh is supported, FCL library has problems with box, cylinder, etc...
		std::vector<boost::shared_ptr<fcl::CollisionObject> > world_objs, robot_objs;
		cworld_.getFCLObjects(world_objs);
		crobot_.getFCLObjects(state, robot_objs);

		//	If the frames are changing
		if (dynamic_frame_->data)
		{
			tf::TransformListener listener;
			tf::StampedTransform transform;
			listener.lookupTransform(obj_frame_->data, world_frame_->data, ros::Time(0), transform);
			fcl::Quaternion3f quat(transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());
			fcl::Vec3f vec(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
			fcl::Transform3f world_to_robot(quat, vec);
			for (int i = 0; i < world_objs.size(); i++)
			{
				fcl::Transform3f orig_transform = world_objs[i]->getTransform();
				world_objs[i]->setTransform(world_to_robot.inverseTimes(orig_transform));
			}
		}

		close_.header.stamp = ros::Time::now();
		if (laas_->data)
		{

			for (int i = 0; i < world_objs.size(); i++)
			{
				//fcl::Transform3f orig_transform = world_objs[i]->getTransform();
				world_objs[i]->setTransform(obs_in_base_tf_); // * orig_transform
			}
			if (publishDebug_)
			{
				moveit_msgs::DisplayRobotState msg;
				robot_state::robotStateToRobotStateMsg(state, msg.state);
				msg.state.joint_state.header.stamp = close_.header.stamp;
				state_pub_.publish(msg);
				geometry_msgs::Pose pa;
				pa.position.x = obs_in_base_tf_.getTranslation().data.vs[0];
				pa.position.y = obs_in_base_tf_.getTranslation().data.vs[1];
				pa.position.z = obs_in_base_tf_.getTranslation().data.vs[2];
				pa.orientation.w = obs_in_base_tf_.getQuatRotation().getW();
				pa.orientation.x = obs_in_base_tf_.getQuatRotation().getX();
				pa.orientation.y = obs_in_base_tf_.getQuatRotation().getY();
				pa.orientation.z = obs_in_base_tf_.getQuatRotation().getZ();
				for (int i = 0; i < 11; i++)
				{
					wall_marker_.markers[i].pose = pa;
					wall_marker_.markers[i].header.stamp = close_.header.stamp;
				}
			}
		}

		fcl::DistanceRequest req(true);
		fcl::DistanceResult res;
		fcl::Vec3f c1, c2;
		Eigen::Vector3d p1, p2;
		if (publishDebug_)
		{
			close_.points.clear();
			close_.colors.clear();
		}
		wall_marker_.markers[11].points.resize(robot_objs.size() + 11);
		for (int i = 0; i < robot_objs.size(); i++)
		{
			collision_detection::CollisionGeometryData* cd1 =
					static_cast<collision_detection::CollisionGeometryData*>(robot_objs[i]->collisionGeometry()->getUserData());
			for (int k = 0; k < links_.size(); k++)
			{
				{
					fcl::Vec3f aabb = robot_objs[i]->collisionGeometry()->aabb_center;
					c1 = robot_objs[i]->getTransform().transform(aabb);

					geometry_msgs::Point p1;
					p1.x = c1[0];
					p1.y = c1[1];
					p1.z = c1[2];
					wall_marker_.markers[11].points[11 + i] = p1;
				}
				if (dist_info_.hasLink(cd1->getID()))
				{
					for (int j = 0; j < world_objs.size(); j++)
					{
						{
							c2 =
									obs_in_base_tf_.transform(world_objs[j]->collisionGeometry()->aabb_center);
							geometry_msgs::Point p1;
							p1.x = c2[0];
							p1.y = c2[1];
							p1.z = c2[2];
							wall_marker_.markers[11].points[j] = p1;
						}
						res.clear();
						collision_detection::CollisionGeometryData* cd2 =
								static_cast<collision_detection::CollisionGeometryData*>(world_objs[j]->collisionGeometry()->getUserData());
						bool check = true;
						if (laas_->data)
						{
							check = false;
							std::map<std::string, std::vector<std::string>>::iterator it =
									acm_.find(cd2->getID());
							if (!check && it != acm_.end())
							{
								if (std::find(it->second.begin(), it->second.end(), cd1->getID())
										!= it->second.end())
								{
									check = true;
								}
							}
						}
						if (check)
						{
							double dist =
									fcl::distance(robot_objs[i].get(), world_objs[j].get(), req, res);
							DistancePair dist_pair;
							dist_pair.id1 = res.b1;
							dist_pair.id2 = res.b2;
							dist_pair.o1 = cd1->getID();
							dist_pair.o2 = cd2->getID();
							dist_pair.p1 =
									Eigen::Vector3d(res.nearest_points[0].data.vs[0], res.nearest_points[0].data.vs[1], res.nearest_points[0].data.vs[2]);
							dist_pair.p2 =
									Eigen::Vector3d(res.nearest_points[1].data.vs[0], res.nearest_points[1].data.vs[1], res.nearest_points[1].data.vs[2]);
							c2 =
									obs_in_base_tf_.transform(world_objs[j]->collisionGeometry()->aabb_center);
							dist_pair.c1 = Eigen::Vector3d(c1[0], c1[1], c1[2]);
							dist_pair.c2 = Eigen::Vector3d(c2[0], c2[1], c2[2]);
							dist_pair.norm1 = dist_pair.p1 - dist_pair.c1;
							dist_pair.norm1.normalize();
							dist_pair.norm2 = dist_pair.p2 - dist_pair.c2;
							dist_pair.norm2.normalize();
							dist_pair.d = res.min_distance;
							dist_info_.setDistance(dist_pair);

							if (publishDebug_ && dist < 2 * m_)
							{
								geometry_msgs::Point p1, p2;
								if (dist < 0.005)
								{
									p1.x = dist_pair.p1(0);
									p1.y = dist_pair.p1(1);
									p1.z = dist_pair.p1(2);
									p2.x = dist_pair.c2(0);
									p2.y = dist_pair.c2(1);
									p2.z = dist_pair.c2(2);
								}
								else
								{
									p1.x = dist_pair.p1(0);
									p1.y = dist_pair.p1(1);
									p1.z = dist_pair.p1(2);
									p2.x = dist_pair.p2(0);
									p2.y = dist_pair.p2(1);
									p2.z = dist_pair.p2(2);
								}
								close_.points.push_back(p1);
								close_.points.push_back(p2);
								std_msgs::ColorRGBA c1, c2;
								if (dist < 0.005)
								{
									c1.r = 1;
									c1.g = 0;
									c1.b = 0;
								}
								else if (dist < m_)
								{
									c1.r = dist / m_;
									c1.g = 1 - c1.r;
									c1.b = 0;
								}
								else
								{
									c1.r = 0;
									c1.g = 1;
									c1.b = 0;
								}
								c1.a = 1;
								c2 = c1;
								close_.colors.push_back(c1);
								close_.colors.push_back(c2);
							}
						}
					}
					break;
				}
			}
		}
		if (publishDebug_)
		{

			close_pub_.publish(close_);
			wall_pub_.publish(wall_marker_);
			ros::spinOnce();
		}

		//dist_info_.print();
		//scene_->getPlanningScene()->printKnownObjects(std::cerr);
		return SUCCESS;
	}
	EReturn CollisionAvoidance::setObsFrame(const KDL::Frame & tf)
	{
		if (!laas_->data)
		{
			ROS_ERROR("Not in LAAS experiment mode.");
			INDICATE_FAILURE
			return FAILURE;
		}
		std::vector<double> q(4);
		tf.M.GetQuaternion(q[0], q[1], q[2], q[3]);
		fcl::Quaternion3f quat(q[3], q[0], q[1], q[2]);
		fcl::Vec3f vec(tf.p.x(), tf.p.y(), tf.p.z() + 0.05);
		obs_in_base_tf_.setTransform(quat, vec);
		return SUCCESS;
	}
}


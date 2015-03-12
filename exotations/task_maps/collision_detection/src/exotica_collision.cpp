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
		wall_pub_ = nh_.advertise<visualization_msgs::Marker>("wall_marker", 1);
		state_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("disp_state", 1);
		close_pub_ = nh_.advertise<visualization_msgs::Marker>("close_marker", 1);
		wall_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
        wall_marker_.color.a = 0.0;
        wall_marker_.color.r = 1.0;
        wall_marker_.color.g = 1.0;
        wall_marker_.color.b = 1.0;
		wall_marker_.scale.x = wall_marker_.scale.y = wall_marker_.scale.z = 1.0;
        wall_marker_.mesh_resource = "package://hrp2_14_description/urdf/wall-extended.obj";
		wall_marker_.mesh_use_embedded_materials = true;

		close_.type = visualization_msgs::Marker::LINE_LIST;
		close_.header.frame_id = "/base_link";
		close_.scale.x = 0.004;
		close_.color.g = 1;
		close_.color.a = 1;
	}

	CollisionAvoidance::~CollisionAvoidance()
	{
		//TODO
	}

	EReturn CollisionAvoidance::setPreUpdateClaaback(
			boost::function<void(CollisionAvoidance*, int)> pre_update_callback)
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
			pre_update_callback_(this, t);
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
		if (!handle.FirstChildElement("margin").ToElement())
		{
			XML_CHECK("margin");
			XML_OK(getDouble(*xmltmp, m_));
			std::cout << "Collision Detection: New margin = " << m_ << std::endl;
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
			links_.clear();
			for (KDL::Segment & it : segs)
			{
				if(it.getName().compare("HEAD_LINK0")!=0 && it.getName().compare("HEAD_LINK1")!=0)
				links_.push_back(it.getName());
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
		acm_ = scene_->getPlanningScene()->getAllowedCollisionMatrixNonConst();

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

		if (!dist_info_.isInitialised())
			return phi;
		for (auto & it : dist_info_.link_dist_map_)
		{
			if (it.second.d > m_)
				d = 0.0;
			else if (it.second.d < 0)
			{
				std::cout << " Collision detected between [" << it.first << "] and [" << it.second.o2 << "]\n";
				d = 1.0;
			}
			else
				d = (1.0 - it.second.d / m_);
			phi(0) += (double) d * d;
		}
		return phi;
	}

	Eigen::MatrixXd CollisionAvoidance::computeJacobian(const int size)
	{
		int i = 0, M = useAll_->data ? links_map_.size() : scene_->getMapSize(), N = size;
		Eigen::VectorXd d(M);
		Eigen::MatrixXd jac(1, N);
		jac.setZero();
		KDL::Frame tip_offset, cp_offset, eff_offset;
		Eigen::VectorXd phi(3 * M);
		solver_->generateForwardMap(phi);

		for (auto & it : dist_info_.link_dist_map_)
		{
			if (it.second.d > m_)
				d(links_map_.at(it.first)) = 0.0;
			else if (it.second.d < 0)
			{
				d(links_map_.at(it.first)) = 1.0;
			}
			else
			{
				d(links_map_.at(it.first)) = (1.0 - it.second.d / m_);
			}
			tip_offset = KDL::Frame(KDL::Vector(phi(3 * links_map_.at(it.first)), phi(3
					* links_map_.at(it.first) + 1), phi(3 * links_map_.at(it.first) + 2)));
			cp_offset = KDL::Frame(KDL::Vector(it.second.p1(0), it.second.p1(1), it.second.p1(2)));
			eff_offset = tip_offset.Inverse() * cp_offset;
			solver_->modifyEndEffector(it.second.o1, eff_offset);
			i++;
		}

		Eigen::MatrixXd J(3 * M, N);
		solver_->generateJacobian(J);

		for (auto & it : dist_info_.link_dist_map_)
		{
			jac += ((2.0 * d(links_map_.at(it.first))) / m_)
					* (it.second.norm1.transpose() * J.block(3 * links_map_.at(it.first), 0, 3, N));
			i++;
		}
		solver_->updateEndEffectors(initial_sol_);
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

		if (laas_->data)
		{

			for (int i = 0; i < world_objs.size(); i++)
			{
				fcl::Transform3f orig_transform = world_objs[i]->getTransform();
				world_objs[i]->setTransform(obs_in_base_tf_); // * orig_transform
			}
            if(publishDebug_)
            {
                moveit_msgs::DisplayRobotState msg;
                robot_state::robotStateToRobotStateMsg(state, msg.state);
                state_pub_.publish(msg);
                geometry_msgs::Pose pa;
                pa.position.x = obs_in_base_tf_.getTranslation().data.vs[0];
                pa.position.y = obs_in_base_tf_.getTranslation().data.vs[1];
                pa.position.z = obs_in_base_tf_.getTranslation().data.vs[2];
                pa.orientation.w = obs_in_base_tf_.getQuatRotation().getW();
                pa.orientation.x = obs_in_base_tf_.getQuatRotation().getX();
                pa.orientation.y = obs_in_base_tf_.getQuatRotation().getY();
                pa.orientation.z = obs_in_base_tf_.getQuatRotation().getZ();
                wall_marker_.header.frame_id = "base_link";
                wall_marker_.pose = pa;
                wall_pub_.publish(wall_marker_);
            }
		}

		fcl::DistanceRequest req(true);
		fcl::DistanceResult res;
		Eigen::Vector3d p1, p2;
        if(publishDebug_)
        {
            close_.points.clear();
        }
		for (int i = 0; i < robot_objs.size(); i++)
		{
			collision_detection::CollisionGeometryData* cd1 =
					static_cast<collision_detection::CollisionGeometryData*>(robot_objs[i]->getCollisionGeometry()->getUserData());
			for (int k = 0; k < links_.size(); k++)
				if (dist_info_.hasLink(cd1->getID()))
				{
					for (int j = 0; j < world_objs.size(); j++)
					{
						res.clear();
						collision_detection::CollisionGeometryData* cd2 =
								static_cast<collision_detection::CollisionGeometryData*>(world_objs[j]->getCollisionGeometry()->getUserData());
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

						dist_pair.c1 =
								Eigen::Vector3d(robot_objs[i]->getTranslation().data.vs[0], robot_objs[i]->getTranslation().data.vs[1], robot_objs[i]->getTranslation().data.vs[2]);
						dist_pair.c2 =
								Eigen::Vector3d(world_objs[j]->getTranslation().data.vs[0], world_objs[j]->getTranslation().data.vs[1], world_objs[j]->getTranslation().data.vs[2]);
						dist_pair.norm1 = dist_pair.p1 - dist_pair.c1;
						dist_pair.norm1.normalize();
						dist_pair.norm2 = dist_pair.p2 - dist_pair.c2;
						dist_pair.norm2.normalize();
						dist_pair.d = dist;
						dist_info_.setDistance(dist_pair);
                        if(publishDebug_)
                        {
                            geometry_msgs::Point p1, p2;
                            p1.x = dist_pair.p1(0);
                            p1.y = dist_pair.p1(1);
                            p1.z = dist_pair.p1(2);
                            p2.x = dist_pair.p2(0);
                            p2.y = dist_pair.p2(1);
                            p2.z = dist_pair.p2(2);
                            close_.points.push_back(p1);
                            close_.points.push_back(p2);
                        }
					}
					break;
				}
		}
        if(publishDebug_)
        {
            close_pub_.publish(close_);
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
		fcl::Vec3f vec(tf.p.x(), tf.p.y(), tf.p.z()+0.05);
		obs_in_base_tf_.setTransform(quat, vec);
		return SUCCESS;
	}
}


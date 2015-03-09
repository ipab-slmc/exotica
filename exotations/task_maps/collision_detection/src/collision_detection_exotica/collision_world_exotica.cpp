/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include "collision_detection_exotica/collision_world_exotica.h"
#include <fcl/shape/geometric_shape_to_BVH_model.h>
#include <fcl/traversal/traversal_node_bvhs.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/collision_node.h>

collision_detection::CollisionWorldEXOTica::CollisionWorldEXOTica() :
		CollisionWorld()
{
	fcl::DynamicAABBTreeCollisionManager* m = new fcl::DynamicAABBTreeCollisionManager();
	// m->tree_init_level = 2;
	manager_.reset(m);

	// request notifications about changes to new world
	observer_handle_ =
			getWorld()->addObserver(boost::bind(&CollisionWorldEXOTica::notifyObjectChange, this, _1, _2));
}

collision_detection::CollisionWorldEXOTica::CollisionWorldEXOTica(const WorldPtr& world) :
		CollisionWorld(world)
{
	fcl::DynamicAABBTreeCollisionManager* m = new fcl::DynamicAABBTreeCollisionManager();
	// m->tree_init_level = 2;
	manager_.reset(m);

	// request notifications about changes to new world
	observer_handle_ =
			getWorld()->addObserver(boost::bind(&CollisionWorldEXOTica::notifyObjectChange, this, _1, _2));
	getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

collision_detection::CollisionWorldEXOTica::CollisionWorldEXOTica(
		const CollisionWorldEXOTica &other, const WorldPtr& world) :
		CollisionWorld(other, world)
{
	fcl::DynamicAABBTreeCollisionManager* m = new fcl::DynamicAABBTreeCollisionManager();
	// m->tree_init_level = 2;
	manager_.reset(m);

	fcl_objs_ = other.fcl_objs_;
	for (std::map<std::string, FCLObject>::iterator it = fcl_objs_.begin(); it != fcl_objs_.end();
			++it)
		it->second.registerTo(manager_.get());
	// manager_->update();

	// request notifications about changes to new world
	observer_handle_ =
			getWorld()->addObserver(boost::bind(&CollisionWorldEXOTica::notifyObjectChange, this, _1, _2));
}

collision_detection::CollisionWorldEXOTica::~CollisionWorldEXOTica()
{
	getWorld()->removeObserver(observer_handle_);
}

void collision_detection::CollisionWorldEXOTica::checkRobotCollision(const CollisionRequest &req,
		CollisionResult &res, const CollisionRobot &robot,
		const robot_state::RobotState &state) const
{
	checkRobotCollisionHelper(req, res, robot, state, NULL);
}

void collision_detection::CollisionWorldEXOTica::checkRobotCollision(const CollisionRequest &req,
		CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state,
		const AllowedCollisionMatrix &acm) const
{
	checkRobotCollisionHelper(req, res, robot, state, &acm);
}

void collision_detection::CollisionWorldEXOTica::checkRobotCollision(const CollisionRequest &req,
		CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state1,
		const robot_state::RobotState &state2) const
{
	logError("FCL continuous collision checking not yet implemented");
}

void collision_detection::CollisionWorldEXOTica::checkRobotCollision(const CollisionRequest &req,
		CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state1,
		const robot_state::RobotState &state2, const AllowedCollisionMatrix &acm) const
{
	logError("FCL continuous collision checking not yet implemented");
}

void collision_detection::CollisionWorldEXOTica::checkRobotCollisionHelper(
		const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot,
		const robot_state::RobotState &state, const AllowedCollisionMatrix *acm) const
{
	const CollisionRobotEXOTica &robot_fcl = dynamic_cast<const CollisionRobotEXOTica&>(robot);
	FCLObject fcl_obj;
	robot_fcl.constructFCLObject(state, fcl_obj);

	CollisionData cd(&req, &res, acm);
	cd.enableGroup(robot.getRobotModel());
	for (std::size_t i = 0; !cd.done_ && i < fcl_obj.collision_objects_.size(); ++i)
		manager_->collide(fcl_obj.collision_objects_[i].get(), &cd, &collisionEXOTicaCallback);

	if (req.distance)
		res.distance = distanceRobotHelper(robot, state, acm);
}

void collision_detection::CollisionWorldEXOTica::checkWorldCollision(const CollisionRequest &req,
		CollisionResult &res, const CollisionWorld &other_world) const
{
	checkWorldCollisionHelper(req, res, other_world, NULL);
}

void collision_detection::CollisionWorldEXOTica::checkWorldCollision(const CollisionRequest &req,
		CollisionResult &res, const CollisionWorld &other_world,
		const AllowedCollisionMatrix &acm) const
{
	checkWorldCollisionHelper(req, res, other_world, &acm);
}

void collision_detection::CollisionWorldEXOTica::checkWorldCollisionHelper(
		const CollisionRequest &req, CollisionResult &res, const CollisionWorld &other_world,
		const AllowedCollisionMatrix *acm) const
{
	const CollisionWorldEXOTica &other_fcl_world =
			dynamic_cast<const CollisionWorldEXOTica&>(other_world);
	CollisionData cd(&req, &res, acm);
	manager_->collide(other_fcl_world.manager_.get(), &cd, &collisionEXOTicaCallback);

	if (req.distance)
		res.distance = distanceWorldHelper(other_world, acm);
}

void collision_detection::CollisionWorldEXOTica::constructFCLObject(const World::Object *obj,
		FCLObject &fcl_obj) const
{
	for (std::size_t i = 0; i < obj->shapes_.size(); ++i)
	{
		FCLGeometryConstPtr g = createCollisionGeometry(obj->shapes_[i], obj);
		if (g)
		{
			fcl::CollisionObject *co =
					new fcl::CollisionObject(g->collision_geometry_, transform2fcl(obj->shape_poses_[i]));
			fcl_obj.collision_objects_.push_back(boost::shared_ptr<fcl::CollisionObject>(co));
			fcl_obj.collision_geometry_.push_back(g);
		}
	}
}

void collision_detection::CollisionWorldEXOTica::updateFCLObject(const std::string &id)
{
	// remove FCL objects that correspond to this object
	std::map<std::string, FCLObject>::iterator jt = fcl_objs_.find(id);
	if (jt != fcl_objs_.end())
	{
		jt->second.unregisterFrom(manager_.get());
		jt->second.clear();
	}

	// check to see if we have this object
	collision_detection::World::const_iterator it = getWorld()->find(id);
	if (it != getWorld()->end())
	{
		// construct FCL objects that correspond to this object
		if (jt != fcl_objs_.end())
		{
			constructFCLObject(it->second.get(), jt->second);
			jt->second.registerTo(manager_.get());
		}
		else
		{
			constructFCLObject(it->second.get(), fcl_objs_[id]);
			fcl_objs_[id].registerTo(manager_.get());
		}
	}
	else
	{
		if (jt != fcl_objs_.end())
			fcl_objs_.erase(jt);
	}

	// manager_->update();
}

void collision_detection::CollisionWorldEXOTica::setWorld(const WorldPtr& world)
{
	if (world == getWorld())
		return;

	// turn off notifications about old world
	getWorld()->removeObserver(observer_handle_);

	// clear out objects from old world
	manager_->clear();
	fcl_objs_.clear();
	cleanCollisionGeometryCache();

	CollisionWorld::setWorld(world);

	// request notifications about changes to new world
	observer_handle_ =
			getWorld()->addObserver(boost::bind(&CollisionWorldEXOTica::notifyObjectChange, this, _1, _2));

	// get notifications any objects already in the new world
	getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

void collision_detection::CollisionWorldEXOTica::notifyObjectChange(const ObjectConstPtr& obj,
		World::Action action)
{
	if (action == World::DESTROY)
	{
		std::map<std::string, FCLObject>::iterator it = fcl_objs_.find(obj->id_);
		if (it != fcl_objs_.end())
		{
			it->second.unregisterFrom(manager_.get());
			it->second.clear();
			fcl_objs_.erase(it);
		}
		cleanCollisionGeometryCache();
	}
	else
	{
		updateFCLObject(obj->id_);
		if (action & (World::DESTROY | World::REMOVE_SHAPE))
			cleanCollisionGeometryCache();
	}
}

double collision_detection::CollisionWorldEXOTica::distanceRobotHelper(const CollisionRobot &robot,
		const robot_state::RobotState &state, const AllowedCollisionMatrix *acm) const
{
	const CollisionRobotEXOTica& robot_fcl = dynamic_cast<const CollisionRobotEXOTica&>(robot);
	FCLObject fcl_obj;
	robot_fcl.constructFCLObject(state, fcl_obj);

	CollisionRequest req;
	CollisionResult res;
	CollisionData cd(&req, &res, acm);
	cd.enableGroup(robot.getRobotModel());
	for (std::size_t i = 0; !cd.done_ && i < fcl_obj.collision_objects_.size(); ++i)
		manager_->distance(fcl_obj.collision_objects_[i].get(), &cd, &distanceEXOTicaCallback);

	return res.distance;
}

double collision_detection::CollisionWorldEXOTica::distanceRobot(const CollisionRobot &robot,
		const robot_state::RobotState &state) const
{
	return distanceRobotHelper(robot, state, NULL);
}

double collision_detection::CollisionWorldEXOTica::distanceRobot(const CollisionRobot &robot,
		const robot_state::RobotState &state, const AllowedCollisionMatrix &acm) const
{
	return distanceRobotHelper(robot, state, &acm);
}

double collision_detection::CollisionWorldEXOTica::distanceWorld(const CollisionWorld &world) const
{
	return distanceWorldHelper(world, NULL);
}

double collision_detection::CollisionWorldEXOTica::distanceWorld(const CollisionWorld &world,
		const AllowedCollisionMatrix &acm) const
{
	return distanceWorldHelper(world, &acm);
}

double collision_detection::CollisionWorldEXOTica::distanceWorldHelper(
		const CollisionWorld &other_world, const AllowedCollisionMatrix *acm) const
{
	const CollisionWorldEXOTica& other_fcl_world =
			dynamic_cast<const CollisionWorldEXOTica&>(other_world);
	CollisionRequest req;
	CollisionResult res;
	CollisionData cd(&req, &res, acm);
	manager_->distance(other_fcl_world.manager_.get(), &cd, &distanceEXOTicaCallback);
	return res.distance;
}
void collision_detection::CollisionWorldEXOTica::getFCLObjects(
		std::vector<boost::shared_ptr<fcl::CollisionObject> > & obj)
{
	uint size = 0, i = 0;
	for (auto & it : fcl_objs_)
	{
		size += it.second.collision_objects_.size();
	}
	obj.resize(0);
	for (auto & it : fcl_objs_)
	{
		for (int j = 0; j < it.second.collision_objects_.size(); j++)
		{
			obj.push_back(it.second.collision_objects_[j]);
		}
	}
}
#include "collision_detection_exotica/collision_detector_allocator_exotica.h"
const std::string collision_detection::CollisionDetectorAllocatorEXOTica::NAME_("EXOTica");

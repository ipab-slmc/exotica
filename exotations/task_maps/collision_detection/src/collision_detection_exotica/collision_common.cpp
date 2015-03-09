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

/* Author: Ioan Sucan, Jia Pan */

#include "collision_detection_exotica/collision_common.h"
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/octree.h>
#include <boost/thread/mutex.hpp>

namespace collision_detection
{

	bool collisionEXOTicaCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void *data)
	{
		CollisionData *cdata = reinterpret_cast<CollisionData*>(data);
		if (cdata->done_)
			return true;
		const CollisionGeometryData *cd1 =
				static_cast<const CollisionGeometryData*>(o1->getCollisionGeometry()->getUserData());
		const CollisionGeometryData *cd2 =
				static_cast<const CollisionGeometryData*>(o2->getCollisionGeometry()->getUserData());

		// do not collision check geoms part of the same object / link / attached body
		if (cd1->sameObject(*cd2))
			return false;

		// If active components are specified
		if (cdata->active_components_only_)
		{
			const robot_model::LinkModel *l1 =
					cd1->type == collision_detection::BodyTypes::ROBOT_LINK ? cd1->ptr.link : (
							cd1->type == collision_detection::BodyTypes::ROBOT_ATTACHED ? cd1->ptr.ab->getAttachedLink() : NULL);
			const robot_model::LinkModel *l2 =
					cd2->type == collision_detection::BodyTypes::ROBOT_LINK ? cd2->ptr.link : (
							cd2->type == collision_detection::BodyTypes::ROBOT_ATTACHED ? cd2->ptr.ab->getAttachedLink() : NULL);

			// If neither of the involved components is active
			if ((!l1
					|| cdata->active_components_only_->find(l1)
							== cdata->active_components_only_->end())
					&& (!l2
							|| cdata->active_components_only_->find(l2)
									== cdata->active_components_only_->end()))
				return false;
		}

		// use the collision matrix (if any) to avoid certain collision checks
		collision_detection::DecideContactFn dcf;
		bool always_allow_collision = false;
		if (cdata->acm_)
		{
			collision_detection::AllowedCollision::Type type;
			bool found = cdata->acm_->getAllowedCollision(cd1->getID(), cd2->getID(), type);
			if (found)
			{
				// if we have an entry in the collision matrix, we read it
				if (type == collision_detection::AllowedCollision::ALWAYS)
				{
					always_allow_collision = true;
					if (cdata->req_->verbose)
						logDebug("Collision between '%s' (type '%s') and '%s' (type '%s') is always allowed. No contacts are computed.", cd1->getID().c_str(), cd1->getTypeString().c_str(), cd2->getID().c_str(), cd2->getTypeString().c_str());
				}
				else if (type == collision_detection::AllowedCollision::CONDITIONAL)
				{
					cdata->acm_->getAllowedCollision(cd1->getID(), cd2->getID(), dcf);
					if (cdata->req_->verbose)
						logDebug("Collision between '%s' and '%s' is conditionally allowed", cd1->getID().c_str(), cd2->getID().c_str());
				}
			}
		}

		// check if a link is touching an attached object
		if (cd1->type == collision_detection::BodyTypes::ROBOT_LINK
				&& cd2->type == collision_detection::BodyTypes::ROBOT_ATTACHED)
		{
			const std::set<std::string> &tl = cd2->ptr.ab->getTouchLinks();
			if (tl.find(cd1->getID()) != tl.end())
			{
				always_allow_collision = true;
				if (cdata->req_->verbose)
					logDebug("Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.", cd1->getID().c_str(), cd2->getID().c_str());
			}
		}
		else if (cd2->type == collision_detection::BodyTypes::ROBOT_LINK
				&& cd1->type == collision_detection::BodyTypes::ROBOT_ATTACHED)
		{
			const std::set<std::string> &tl = cd1->ptr.ab->getTouchLinks();
			if (tl.find(cd2->getID()) != tl.end())
			{
				always_allow_collision = true;
				if (cdata->req_->verbose)
					logDebug("Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.", cd2->getID().c_str(), cd1->getID().c_str());
			}
		}
		// bodies attached to the same link should not collide
		if (cd1->type == collision_detection::BodyTypes::ROBOT_ATTACHED
				&& cd2->type == collision_detection::BodyTypes::ROBOT_ATTACHED)
		{
			if (cd1->ptr.ab->getAttachedLink() == cd2->ptr.ab->getAttachedLink())
				always_allow_collision = true;
		}

		// if collisions are always allowed, we are done
		if (always_allow_collision)
			return false;

		if (cdata->req_->verbose)
			logDebug("Actually checking collisions between %s and %s", cd1->getID().c_str(), cd2->getID().c_str());

		// see if we need to compute a contact
		std::size_t want_contact_count = 0;
		if (cdata->req_->contacts)
			if (cdata->res_->contact_count < cdata->req_->max_contacts)
			{
				std::size_t have;
				if (cd1->getID() < cd2->getID())
				{
					std::pair<std::string, std::string> cp(cd1->getID(), cd2->getID());
					have =
							cdata->res_->contacts.find(cp) != cdata->res_->contacts.end() ? cdata->res_->contacts[cp].size() : 0;
				}
				else
				{
					std::pair<std::string, std::string> cp(cd2->getID(), cd1->getID());
					have =
							cdata->res_->contacts.find(cp) != cdata->res_->contacts.end() ? cdata->res_->contacts[cp].size() : 0;
				}
				if (have < cdata->req_->max_contacts_per_pair)
					want_contact_count =
							std::min(cdata->req_->max_contacts_per_pair - have, cdata->req_->max_contacts
									- cdata->res_->contact_count);
			}

		if (dcf)
		{
			// if we have a decider for allowed contacts, we need to look at all the contacts
			bool enable_cost = cdata->req_->cost;
			std::size_t num_max_cost_sources = cdata->req_->max_cost_sources;
			bool enable_contact = true;
			fcl::CollisionResult col_result;
			int num_contacts =
					fcl::collide(o1, o2, fcl::CollisionRequest(std::numeric_limits<size_t>::max(), enable_contact, num_max_cost_sources, enable_cost), col_result);
			if (num_contacts > 0)
			{
				if (cdata->req_->verbose)
					logInform("Found %d contacts between '%s' and '%s'. These contacts will be evaluated to check if they are accepted or not", num_contacts, cd1->getID().c_str(), cd2->getID().c_str());
				collision_detection::Contact c;
				const std::pair<std::string, std::string> &pc =
						cd1->getID() < cd2->getID() ? std::make_pair(cd1->getID(), cd2->getID()) : std::make_pair(cd2->getID(), cd1->getID());
				for (int i = 0; i < num_contacts; ++i)
				{
					fcl2contact(col_result.getContact(i), c);
					// if the contact is  not allowed, we have a collision
					if (dcf(c) == false)
					{
						// store the contact, if it is needed
						if (want_contact_count > 0)
						{
							--want_contact_count;
							cdata->res_->contacts[pc].push_back(c);
							cdata->res_->contact_count++;
							if (cdata->req_->verbose)
								logInform("Found unacceptable contact between '%s' and '%s'. Contact was stored.", cd1->getID().c_str(), cd2->getID().c_str());
						}
						else if (cdata->req_->verbose)
							logInform("Found unacceptable contact between '%s' (type '%s') and '%s' (type '%s'). Contact was stored.", cd1->getID().c_str(), cd1->getTypeString().c_str(), cd2->getID().c_str(), cd2->getTypeString().c_str());
						cdata->res_->collision = true;
						if (want_contact_count == 0)
							break;
					}
				}
			}

			if (enable_cost)
			{
				std::vector<fcl::CostSource> cost_sources;
				col_result.getCostSources(cost_sources);

				collision_detection::CostSource cs;
				for (std::size_t i = 0; i < cost_sources.size(); ++i)
				{
					fcl2costsource(cost_sources[i], cs);
					cdata->res_->cost_sources.insert(cs);
					while (cdata->res_->cost_sources.size() > cdata->req_->max_cost_sources)
						cdata->res_->cost_sources.erase(--cdata->res_->cost_sources.end());
				}
			}
		}
		else
		{
			if (want_contact_count > 0)
			{
				// otherwise, we need to compute more things
				bool enable_cost = cdata->req_->cost;
				std::size_t num_max_cost_sources = cdata->req_->max_cost_sources;
				bool enable_contact = true;

				fcl::CollisionResult col_result;
				int num_contacts =
						fcl::collide(o1, o2, fcl::CollisionRequest(want_contact_count, enable_contact, num_max_cost_sources, enable_cost), col_result);
				if (num_contacts > 0)
				{
					int num_contacts_initial = num_contacts;

					// make sure we don't get more contacts than we want
					if (want_contact_count >= (std::size_t) num_contacts)
						want_contact_count -= num_contacts;
					else
					{
						num_contacts = want_contact_count;
						want_contact_count = 0;
					}

					if (cdata->req_->verbose)
						logInform("Found %d contacts between '%s' (type '%s') and '%s' (type '%s'), which constitute a collision. %d contacts will be stored", num_contacts_initial, cd1->getID().c_str(), cd1->getTypeString().c_str(), cd2->getID().c_str(), cd2->getTypeString().c_str(), num_contacts);

					const std::pair<std::string, std::string> &pc =
							cd1->getID() < cd2->getID() ? std::make_pair(cd1->getID(), cd2->getID()) : std::make_pair(cd2->getID(), cd1->getID());
					cdata->res_->collision = true;
					for (int i = 0; i < num_contacts; ++i)
					{
						collision_detection::Contact c;
						fcl2contact(col_result.getContact(i), c);
						cdata->res_->contacts[pc].push_back(c);
						cdata->res_->contact_count++;
					}
				}

				if (enable_cost)
				{
					std::vector<fcl::CostSource> cost_sources;
					col_result.getCostSources(cost_sources);

					collision_detection::CostSource cs;
					for (std::size_t i = 0; i < cost_sources.size(); ++i)
					{
						fcl2costsource(cost_sources[i], cs);
						cdata->res_->cost_sources.insert(cs);
						while (cdata->res_->cost_sources.size() > cdata->req_->max_cost_sources)
							cdata->res_->cost_sources.erase(--cdata->res_->cost_sources.end());
					}
				}
			}
			else
			{
				bool enable_cost = cdata->req_->cost;
				std::size_t num_max_cost_sources = cdata->req_->max_cost_sources;
				bool enable_contact = false;
				fcl::CollisionResult col_result;
				int num_contacts =
						fcl::collide(o1, o2, fcl::CollisionRequest(1, enable_contact, num_max_cost_sources, enable_cost), col_result);
				if (num_contacts > 0)
				{
					cdata->res_->collision = true;
					if (cdata->req_->verbose)
						logInform("Found a contact between '%s' (type '%s') and '%s' (type '%s'), which constitutes a collision. Contact information is not stored.", cd1->getID().c_str(), cd1->getTypeString().c_str(), cd2->getID().c_str(), cd2->getTypeString().c_str());
				}

				if (enable_cost)
				{
					std::vector<fcl::CostSource> cost_sources;
					col_result.getCostSources(cost_sources);

					collision_detection::CostSource cs;
					for (std::size_t i = 0; i < cost_sources.size(); ++i)
					{
						fcl2costsource(cost_sources[i], cs);
						cdata->res_->cost_sources.insert(cs);
						while (cdata->res_->cost_sources.size() > cdata->req_->max_cost_sources)
							cdata->res_->cost_sources.erase(--cdata->res_->cost_sources.end());
					}
				}
			}
		}

		if (cdata->res_->collision)
			if (!cdata->req_->contacts || cdata->res_->contact_count >= cdata->req_->max_contacts)
			{
				if (!cdata->req_->cost)
					cdata->done_ = true;
				if (cdata->req_->verbose)
					logInform("Collision checking is considered complete (collision was found and %u contacts are stored)", (unsigned int )cdata->res_->contact_count);
			}

		if (!cdata->done_ && cdata->req_->is_done)
		{
			cdata->done_ = cdata->req_->is_done(*cdata->res_);
			if (cdata->done_ && cdata->req_->verbose)
				logInform("Collision checking is considered complete due to external callback. %s was found. %u contacts are stored.",
						cdata->res_->collision ? "Collision" : "No collision", (unsigned int )cdata->res_->contact_count);
		}

		return cdata->done_;
	}

	bool distanceEXOTicaCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* data,
			double& min_dist)
	{
		CollisionData* cdata = reinterpret_cast<CollisionData*>(data);

		const CollisionGeometryData* cd1 =
				static_cast<const CollisionGeometryData*>(o1->getCollisionGeometry()->getUserData());
		const CollisionGeometryData* cd2 =
				static_cast<const CollisionGeometryData*>(o2->getCollisionGeometry()->getUserData());

		// If active components are specified
		if (cdata->active_components_only_)
		{
			const robot_model::LinkModel *l1 =
					cd1->type == collision_detection::BodyTypes::ROBOT_LINK ? cd1->ptr.link : (
							cd1->type == collision_detection::BodyTypes::ROBOT_ATTACHED ? cd1->ptr.ab->getAttachedLink() : NULL);
			const robot_model::LinkModel *l2 =
					cd2->type == collision_detection::BodyTypes::ROBOT_LINK ? cd2->ptr.link : (
							cd2->type == collision_detection::BodyTypes::ROBOT_ATTACHED ? cd2->ptr.ab->getAttachedLink() : NULL);

			// If neither of the involved components is active
			if ((!l1
					|| cdata->active_components_only_->find(l1)
							== cdata->active_components_only_->end())
					&& (!l2
							|| cdata->active_components_only_->find(l2)
									== cdata->active_components_only_->end()))
			{
				min_dist = cdata->res_->distance;
				return cdata->done_;
			}
		}

		// use the collision matrix (if any) to avoid certain distance checks
		bool always_allow_collision = false;
		if (cdata->acm_)
		{
			collision_detection::AllowedCollision::Type type;

			bool found = cdata->acm_->getAllowedCollision(cd1->getID(), cd2->getID(), type);
			if (found)
			{
				// if we have an entry in the collision matrix, we read it
				if (type == collision_detection::AllowedCollision::ALWAYS)
				{
					always_allow_collision = true;
					if (cdata->req_->verbose)
						logDebug("Collision between '%s' and '%s' is always allowed. No contacts are computed.", cd1->getID().c_str(), cd2->getID().c_str());
				}
			}
		}

		// check if a link is touching an attached object
		if (cd1->type == collision_detection::BodyTypes::ROBOT_LINK
				&& cd2->type == collision_detection::BodyTypes::ROBOT_ATTACHED)
		{
			const std::set<std::string> &tl = cd2->ptr.ab->getTouchLinks();
			if (tl.find(cd1->getID()) != tl.end())
			{
				always_allow_collision = true;
				if (cdata->req_->verbose)
					logDebug("Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.", cd1->getID().c_str(), cd2->getID().c_str());
			}
		}
		else
		{
			if (cd2->type == collision_detection::BodyTypes::ROBOT_LINK
					&& cd1->type == collision_detection::BodyTypes::ROBOT_ATTACHED)
			{
				const std::set<std::string> &tl = cd1->ptr.ab->getTouchLinks();
				if (tl.find(cd2->getID()) != tl.end())
				{
					always_allow_collision = true;
					if (cdata->req_->verbose)
						logDebug("Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.", cd2->getID().c_str(), cd1->getID().c_str());
				}
			}
		}

		if (always_allow_collision)
		{
			min_dist = cdata->res_->distance;
			return cdata->done_;
		}

		if (cdata->req_->verbose)
			logDebug("Actually checking collisions between %s and %s", cd1->getID().c_str(), cd2->getID().c_str());

		fcl::DistanceResult dist_result;
		//dist_result.update(cdata->res_->distance, NULL, NULL, fcl::DistanceResult::NONE, fcl::DistanceResult::NONE); // can be faster
		double d = fcl::distance(o1, o2, fcl::DistanceRequest(true), dist_result);

		if (d < 0)
		{
			cdata->done_ = true;
			cdata->res_->distance = -1;
		}
		else
		{
			if (cdata->res_->distance > d)
				cdata->res_->distance = d;
		}

		min_dist = cdata->res_->distance;

		//Now lets fill in the distance information that would be used in EXOTica Collision Avoidance
//		exotica::DistancePair dist_pair;
//		dist_pair.id1 = dist_result.b2;
//		dist_pair.id2 = dist_result.b1;
//		dist_pair.o1 = cd2->getID();
//		dist_pair.o2 = cd1->getID();
//		dist_pair.p1 =
//				Eigen::Vector3d(dist_result.nearest_points[1].data.vs[0], dist_result.nearest_points[1].data.vs[1], dist_result.nearest_points[1].data.vs[2]);
//		dist_pair.p2 =
//				Eigen::Vector3d(dist_result.nearest_points[0].data.vs[0], dist_result.nearest_points[0].data.vs[1], dist_result.nearest_points[0].data.vs[2]);
//
//		dist_pair.c1 =
//				Eigen::Vector3d(o2->getTranslation().data.vs[0], o2->getTranslation().data.vs[1], o2->getTranslation().data.vs[2]);
//		dist_pair.c2 =
//				Eigen::Vector3d(o1->getTranslation().data.vs[0], o1->getTranslation().data.vs[1], o1->getTranslation().data.vs[2]);
//		dist_pair.norm1 = dist_pair.p1 - dist_pair.c1;
//		dist_pair.norm1.normalize();
//		dist_pair.norm2 = dist_pair.p2 - dist_pair.c2;
//		dist_pair.norm2.normalize();
//		dist_pair.id1 = dist_result.b2;
//		dist_pair.id2 = dist_result.b1;
//		dist_pair.d = (dist_pair.p1 - dist_pair.p2).norm();
//		if (dist_info_.isInitialised())
//		{
//			dist_info_.setDistance(dist_pair);
//		}
		return cdata->done_;
	}
}

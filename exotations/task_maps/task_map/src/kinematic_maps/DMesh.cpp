/*
 * DMesh.cpp
 *
 *  Created on: 15 Jun 2015
 *      Author: yiming
 */

#include "kinematic_maps/DMesh.h"
REGISTER_TASKMAP_TYPE("DMesh", exotica::DMesh);
namespace exotica
{
	DMesh::DMesh()
	{

	}

	DMesh::~DMesh()
	{

	}

	EReturn DMesh::initDerived(tinyxml2::XMLHandle & handle)
	{
		EParam<std_msgs::Int64> tmp;
		if (!ok(server_->getParam(server_->getName() + "/DMeshSize", tmp)))
		{
			INDICATE_FAILURE
			return FAILURE;
		}
		graph_size_ = tmp->data;

		tinyxml2::XMLHandle tmp_handle = handle.FirstChildElement("DMeshLinks");
		server_->registerParam<exotica::StringList>(ns_, tmp_handle, links_);

		tmp_handle = handle.FirstChildElement("DMeshLinkTypes");
		server_->registerParam<exotica::BoolList>(ns_, tmp_handle, link_types_);

		tmp_handle = handle.FirstChildElement("PoseGain");
		if (!ok(server_->registerParam<std_msgs::Float64>(ns_, tmp_handle, kp_)))
			kp_->data = 0;

		tmp_handle = handle.FirstChildElement("ObstacleGain");
		if (!ok(server_->registerParam<std_msgs::Float64>(ns_, tmp_handle, ko_)))
			ko_->data = 10;

		tmp_handle = handle.FirstChildElement("GoalGain");
		if (!ok(server_->registerParam<std_msgs::Float64>(ns_, tmp_handle, kg_)))
			kg_->data = 100;
		///	Initialise the mesh
		vertices_.resize(graph_size_);
		vertex_map_.clear();
		current_size_ = links_->strings.size();
		for (int i = 0; i < current_size_; i++)
		{
			vertices_[i].reset(new Vertex(links_->strings[i],
					link_types_->data[i] ? Vertex::LINK : Vertex::DUMMY_LINK, i));
			vertex_map_[links_->strings[i]] = i;
			if (kp_->data != 0)
				for (int j = i + 1; j < current_size_; j++)
					vertices_[i]->addNewRelation(links_->strings[i], i);
		}

		robot_size_ = current_size_;
		ext_size_ = graph_size_ - current_size_;
		task_size_ =
				(kp_->data == 0) ? current_size_ * (graph_size_ - current_size_) : (graph_size_
						* (graph_size_ - 1) / 2 - current_size_);
		dist_.setZero(graph_size_, graph_size_);
		HIGHLIGHT_NAMED("DMesh", "Distance Mesh has been initialised: Maximum Graph size="<<graph_size_<<", Robot link size="<<robot_size_<<", Unconnected external object size="<<ext_size_);
		return SUCCESS;
	}

	EReturn DMesh::taskSpaceDim(int & task_dim)
	{
		LOCK(lock_);
		task_dim = task_size_;
		return SUCCESS;
	}

	EReturn DMesh::update(Eigen::VectorXdRefConst x, const int t)
	{
		q_size_ = x.rows();
		if (!isRegistered(t) || !getEffReferences())
		{
			INDICATE_FAILURE
			return FAILURE;
		}
		if (!ok(updateDistances()))
		{
			INDICATE_FAILURE
			return FAILURE;
		}
		if (ok(computeLaplace(t)))
		{
			if (updateJacobian_)
			{
				if (ok(computeJacobian(t)))
				{
					return SUCCESS;
				}
				else
				{
					INDICATE_FAILURE
					return FAILURE;
				}
			}
			else
			{
				return SUCCESS;
			}
		}
		else
		{
			INDICATE_FAILURE
			return FAILURE;
		}
	}

	EReturn DMesh::computeLaplace(int t)
	{
		LOCK(lock_);
		PHI.setZero();
		uint j, l, cnt = 0;
		double b = 0, d = 0;
		for (j = 0; j < robot_size_; j++)
		{
			int tmp = robot_size_;
			if (usePose_->data)
				tmp = j + 2;
			for (l = tmp; l < graph_size_; l++)
			{
				b = d = 0;
				switch (gManager_.getGraph()->getVertex(l)->getType())
				{
					case VERTEX_TYPE::LINK:
						PHI(cnt) = kp_->data * dist_(j, l);
						break;
					case VERTEX_TYPE::OBSTACLE:
						if (gManager_.getGraph()->getVertex(l)->checkList(links_->strings[j]))
						{
							if (dist_(j, l) - gManager_.getGraph()->getVertex(j)->getRadius()
									- gManager_.getGraph()->getVertex(l)->getRadius() < 0.05)
								PHI(cnt) = ko_->data * (1 - exp(-wo_ * dist_(j, l)));
							else
								PHI(cnt) = ko_->data;

						}
						break;
					case VERTEX_TYPE::OBSTACLE_TO_ALL:
						if (dist_(j, l) - gManager_.getGraph()->getVertex(j)->getRadius()
								- gManager_.getGraph()->getVertex(l)->getRadius() < 0.05)
							PHI(cnt) = ko_->data * (1 - exp(-wo_ * dist_(j, l)));
						else
							PHI(cnt) = ko_->data;
						break;
					case VERTEX_TYPE::GOAL:
						PHI(cnt) = gManager_.getGraph()->getVertex(l)->w_ * dist_(j, l);
						break;
					default:
						break;
				}
				cnt++;
			}
		}
		return SUCCESS;
	}

	EReturn DMesh::updateDistances()
	{
		for (int j = 0; j < current_size_; j++)
		{
			for (auto & it : vertices_[j]->toVertices_)
				scene_->getCollisionScene()->getDistance(vertices_[j]->name_, it.first,
						(j < it.second) ? dist_(j, it.second) : dist_(it.second, j), di_->data);
		}
		return SUCCESS;
	}

	EReturn DMesh::addGoal(const std::string & name, std::string & toLink)
	{
		Vertex *tmp = new Vertex(name, Vertex::GOAL, current_size_);
		int eff_index = -1;
		for (int i = 0; i < links_->strings.size(); i++)
			if (links_->strings[i].compare(toLink) == 0)
			{
				tmp->addNewRelation(links_->strings[i], i);
			}
		if (tmp->toVertices_.size() == 0)
		{
			WARNING_NAMED("DMesh", "Add goal ["<<name<<"] failed, End-effector link ["<<toLink<<"] does not exist");
			delete tmp;
			return FAILURE;
		}
		vertices_[current_size_] = Vertex_Ptr(tmp);
		current_size_++;
		return SUCCESS;
	}

	EReturn DMesh::addObstacle(const std::string & name)
	{
		if (vertex_map_.find(name) != vertex_map_.end())
		{
			WARNING_NAMED("DMesh", "Add obstacle ["<<name<<"], already exists");
			return FAILURE;
		}
		Vertex *tmp = new Vertex(name, Vertex::OBSTACLE_TO_ALL, current_size_);
		for (int i = 0; i < links_->strings.size(); i++)
			tmp->addNewRelation(links_->strings[i], i);
		vertices_[current_size_] = Vertex_Ptr(tmp);
		current_size_++;
		return SUCCESS;
	}
} //	namespace exotica

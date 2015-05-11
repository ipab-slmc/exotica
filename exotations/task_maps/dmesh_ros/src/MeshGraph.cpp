/*
 * MeshGraph.cpp
 *
 *  Created on: 16 Aug 2014
 *      Author: yiming
 */

#include "MeshGraph.h"

namespace exotica
{
//	Functions of Vertex
	Vertex::Vertex(const std::string & name) :
			name_(name), type_(NONE), isActive_(false), radius_(0)
	{
		position_.setZero();
	}

	bool Vertex::setAsLink(const std::string & name, bool real_link,
			const Eigen::Vector3d & position, double r)
	{
		if (name.compare("") == 0)
		{
			INDICATE_FAILURE
			return false;
		}
		name_ = name;
		type_ = LINK;
		position_ = position;
		radius_ = r;
		isActive_ = true;
		w_ = 1;
		return true;
	}

	bool Vertex::setAsLink(const std::string & name, bool real_link,
			const geometry_msgs::Point & position, double r)
	{
		position_ << position.x, position.y, position.z;
		return setAsLink(name, real_link, position_, r);
	}

	bool Vertex::setAsGoal(const std::string & name, const Eigen::Vector3d & position, double r,
			const std::vector<std::string> & tolinks, double w)
	{
		if (name.compare("") == 0 || tolinks.size() == 0)
		{
			INDICATE_FAILURE
			return false;
		}
		name_ = name;
		type_ = GOAL;
		position_ = position;
		toLinks_ = tolinks;
		isActive_ = true;
		radius_ = r;
		w_ = w;
		return true;
	}

	bool Vertex::setAsGoal(const std::string & name, const geometry_msgs::Point & position,
			double r, const std::vector<std::string> & tolinks, double w)
	{
		position_ << position.x, position.y, position.z;
		return setAsGoal(name, position_, r, tolinks, w);
	}

	bool Vertex::setAsObstacle(const std::string & name, const Eigen::Vector3d & position, double r)
	{
		if (name.compare("") == 0)
		{
			INDICATE_FAILURE
			return false;
		}
		name_ = name;
		type_ = OBSTACLE_TO_ALL;
		position_ = position;
		radius_ = r;
		return true;
	}

	bool Vertex::setAsObstacle(const std::string & name, const geometry_msgs::Point & position,
			double r)
	{
		position_ << position.x, position.y, position.z;
		return setAsObstacle(name, position_, r);
	}

	bool Vertex::setAsObstacle(const std::string & name, const Eigen::Vector3d & position, double r,
			const std::vector<std::string> & tolinks)
	{
		if (name.compare("") == 0 || tolinks.size() == 0)
		{
			INDICATE_FAILURE
			return false;
		}
		name_ = name;
		type_ = OBSTACLE;
		position_ = position;
		toLinks_ = tolinks;
		radius_ = r;
		return true;
	}

	bool Vertex::setAsObstacle(const std::string & name, const geometry_msgs::Point & position,
			double r, const std::vector<std::string> & tolinks)
	{
		position_ << position.x, position.y, position.z;
		return setAsObstacle(name, position_, r, tolinks);
	}

	void Vertex::invalidate()
	{
		name_ = "UNKNOWN";
		position_.setZero();
		type_ = NONE;
		toLinks_.clear();
	}

	bool Vertex::activate()
	{
		if (type_ == NONE)
		{
			INDICATE_FAILURE
			return false;
		}

		return isActive_ = true;
	}

	bool Vertex::deactivate()
	{
		if (type_ == NONE)
		{
			WARNING("The vertex has not been initialised yet");
			return true;
		}
		isActive_ = false;
		return true;
	}

	bool Vertex::isActive()
	{
		return isActive_;
	}

	double Vertex::distance(const Eigen::Vector3d & other)
	{
		return (other - position_).norm();
	}

	double Vertex::distance(const Vertex & other)
	{
		return distance(other.position_);
	}

	double Vertex::distance(const VertexPtr & other)
	{
		return distance(other->position_);
	}

	bool Vertex::checkList(const std::string & link)
	{
		for (int i = 0; i < toLinks_.size(); i++)
		{
			if (toLinks_[i].compare(link) == 0)
				return true;
		}
		return false;
	}

	std::string Vertex::getName()
	{
		return name_;
	}

	double Vertex::getRadius()
	{
		return radius_;
	}
	VERTEX_TYPE Vertex::getType()
	{
		return type_;
	}

//	Functions of MeshGarph
	MeshGraph::MeshGraph(const std::string & name) :
					name_(name),
					size_(0),
					robot_size_(0),
					initialised_(false),
					interact_range_(1.0),
					eps_(0.05),
					nh_("/DMeshGraph")
	{
		//TODO
	}

	std::string MeshGraph::getName()
	{
		return name_;
	}

	bool MeshGraph::getGraphSize(int size)
	{
		if (!initialised_)
		{
			INDICATE_FAILURE
			size = -1;
			return false;
		}
		size = size_;
		return true;
	}

	int MeshGraph::getGraphSize()
	{
		if (!initialised_)
			return -1;
		return size_;
	}

	bool MeshGraph::getRobotSize(int robot_size)
	{
		if (!initialised_)
		{
			INDICATE_FAILURE
			robot_size = -1;
			return false;
		}
		robot_size = robot_size_;
		return true;
	}

	int MeshGraph::getRobotSize()
	{
		if (!initialised_)
			return -1;
		return robot_size_;
	}

	VertexPtr MeshGraph::getVertex(const int index)
	{
		if (index >= vertices_.size())
		{
			if (!initialised_)
				std::cout << "Graph not initialised" << std::endl;
			else
				std::cout << "You are querying the vertex with index " << index << ", but the graph current only contains " << vertices_.size() << " vertices" << std::endl;
			INDICATE_FAILURE
			return VertexPtr(new Vertex);
		}
		return vertices_[index];
	}

	bool MeshGraph::initialisation(const int size, const std::vector<std::string> & link_names,
			const std::vector<bool> link_type, const std::vector<double> & link_radius,
			const double i_range, const double eps, const bool dummy_table)
	{
		if (initialised_ || link_names.size() == 0 || size < link_names.size() || i_range < 0
				|| eps < 0)
		{
			INDICATE_FAILURE
			return false;
		}
		size_ = size;
		robot_size_ = link_names.size();
		link_names_ = link_names;
		interact_range_ = i_range;
		eps_ = eps;
		vertices_.resize(size_);
		for (int i = 0; i < robot_size_; i++)
		{
			if (vertex_map_.find(link_names[i]) != vertex_map_.end())
			{
				INDICATE_FAILURE
				return false;
			}
			vertices_[i].reset(new Vertex);
			vertices_[i]->setAsLink(link_names[i], true, Eigen::Vector3d::Zero(), link_radius[i]);
			if (!link_type[i])
				vertices_[i]->setToDummy();
			vertex_map_[link_names[i]] = i;
		}
		for (int j = robot_size_; j < size_; j++)
			vertices_[j].reset(new Vertex);

		dummy_table_ = dummy_table;
		if (dummy_table_)
		{
			//	Let's create a dummy table for end-effector
			vertex_map_["DummyTable_link6"] = robot_size_;
			std::vector<std::string> eff;
			vertices_[vertex_map_.at("DummyTable_link6")]->setAsObstacle("DummyTable_link6", Eigen::Vector3d::Zero(), 0.06, {
					"lwr_arm_6_link" });

//			vertex_map_["DummyTable_tip"] = robot_size_ + 1;
//			vertices_[vertex_map_.at("DummyTable_tip")]->setAsObstacle("DummyTable_tip", Eigen::Vector3d::Zero(), 0.06, {
//					"pen_tip_link" });
		}

		pubs_.resize(5);
		edges_.resize(5);
		//	0.	Desired mesh
		//	1.	Links in current mesh
		//	2.	Goal in current mesh
		//	3.	Active obstacle in current mesh
		//	4.	Non-active obstacle in current mesh
		for (int i = 0; i < 5; i++)
		{
			pubs_[i] =
					nh_.advertise<visualization_msgs::Marker>("DMeshEdges" + std::to_string(i), 100);
			edges_[i].type = visualization_msgs::Marker::LINE_LIST;
			edges_[i].header.frame_id = "/base";

		}

		edges_[0].scale.x = 0.002;
		edges_[1].scale.x = 0.002;
		edges_[2].scale.x = 0.006;
		edges_[3].scale.x = 0.002;
		edges_[4].scale.x = 0.002;


		edges_[0].color.r = 0;
		edges_[0].color.g = 1;
		edges_[0].color.b = 0.6;
		edges_[0].color.a = .6;

		edges_[1].color.r = 0.721;
		edges_[1].color.g = 0.698;
		edges_[1].color.b = 0.698;
		edges_[1].color.a = 1;

		edges_[2].color.r = 0.050;
		edges_[2].color.g = 0.686;
		edges_[2].color.b = 0.192;
		edges_[2].color.a = 1;

		edges_[3].color.r = 0;
		edges_[3].color.g = 0;
		edges_[3].color.b = 0;
		edges_[3].color.a = 1;

		edges_[4].color.r = 1;
		edges_[4].color.g = 0;
		edges_[4].color.b = 0;
		edges_[4].color.a = 1;

		initialised_ = true;
		return true;
	}

	bool MeshGraph::isInitialised()
	{
		return initialised_;
	}

    bool MeshGraph::updateLinksRef(Eigen::VectorXdRefConst link_poses)
	{
		//	Check size, assume they are in correct order
		if (robot_size_ != link_poses.rows() / 3 || link_poses.rows() % 3 != 0)
		{
			INDICATE_FAILURE
			return false;
		}

		for (int i = 0; i < robot_size_; i++)
			vertices_[i]->position_ = Eigen::Vector3d(link_poses.segment(3 * i, 3));

		if (dummy_table_)
		{
			//	When we update the links, dummy table should also be updated
			vertices_[vertex_map_.at("DummyTable_link6")]->position_ =
					vertices_[vertex_map_.at("lwr_arm_6_link")]->position_;
			vertices_[vertex_map_.at("DummyTable_link6")]->position_(2) = 0;
			checkActivation(vertices_[vertex_map_.at("DummyTable_link6")]);

//			vertices_[vertex_map_.at("DummyTable_tip")]->position_ =
//					vertices_[vertex_map_.at("pen_tip_link")]->position_;
//			vertices_[vertex_map_.at("DummyTable_tip")]->position_(2) = 0;
//			checkActivation(vertices_[vertex_map_.at("DummyTable_tip")]);
		}
		return true;
	}

	bool MeshGraph::updateLinks(const Eigen::MatrixX3d & link_poses)
	{
		//	Check size, assume they are in correct order
		if (robot_size_ != link_poses.cols())
		{
			INDICATE_FAILURE
			return false;
		}

		for (int i = 0; i < robot_size_; i++)
			vertices_[i]->position_ = Eigen::Vector3d(link_poses.row(i));
		return true;
	}

	bool MeshGraph::updateLink(const std::string & name, const Eigen::Vector3d & pose)
	{
		if (vertex_map_.find(name) == vertex_map_.end())
		{
			std::cout << "Link " << name << " does not exist" << std::endl;
			return false;
		}
		vertices_[vertex_map_.at(name)]->position_ = pose;
		return true;
	}

	bool MeshGraph::updateExternal(const exotica::MeshVertex & ext)
	{
		//	If it's already in the list, just change position
		if (vertex_map_.find(ext.name) != vertex_map_.end())
		{
			vertices_[vertex_map_.at(ext.name)]->position_(0) = ext.position.x;
			vertices_[vertex_map_.at(ext.name)]->position_(1) = ext.position.y;
			vertices_[vertex_map_.at(ext.name)]->position_(2) = ext.position.z;
			vertices_[vertex_map_.at(ext.name)]->radius_ = ext.radius;
			vertices_[vertex_map_.at(ext.name)]->w_ = ext.w;
			return true;
		}

		//	If not, then we need to add a new object
		bool success = false;
		VertexPtr newvertex(new Vertex);
		switch (ext.type)
		{
			case exotica::MeshVertex::GOAL:
				if (newvertex->setAsGoal(ext.name, ext.position, ext.radius, ext.toLinks, ext.w))
					success = true;
				break;
			case exotica::MeshVertex::OBSTACLE_TO_ALL:
				if (newvertex->setAsObstacle(ext.name, ext.position, ext.radius))
					success = true;
				break;
			case exotica::MeshVertex::OBSTACLE:
				if (newvertex->setAsObstacle(ext.name, ext.position, ext.radius, ext.toLinks))
					success = true;
				break;
			default:
				//	This function should not contain LINK, as it should be called from external subscriber callbacks
				break;
		}
		if (!success)
			return false;
		int tmpsize = vertex_map_.size();
		vertices_[tmpsize] = newvertex;
		vertex_map_[ext.name] = tmpsize;
		return true;
	}

	bool MeshGraph::updateExternal(const exotica::MeshVertexConstPtr & ext)
	{

		return updateExternal(*ext.get());
	}

	bool MeshGraph::removeVertex(const std::string & name)
	{
		std::map<std::string, int>::iterator it = vertex_map_.find(name);
		if (it == vertex_map_.end())
		{
			std::cout << "Link " << name << " does not exist" << std::endl;
			return false;
		}
		vertices_[it->second]->invalidate();
		vertex_map_.erase(it);
		return true;
	}
	void MeshGraph::checkActivation(const VertexPtr & ext)
	{
		double d = 999;
		bool found = false;
		switch (ext->getType())
		{
			case OBSTACLE:
				for (auto & it : vertex_map_)
				{
					if (ext->checkList(it.first))
					{
						d = ext->distance(vertices_[it.second]->position_);

						if (d < interact_range_) //if (d < interact_range_)
						{
							found = true;
							break;
						}
					}
				}
				break;
			case OBSTACLE_TO_ALL:
				for (auto & it : vertex_map_)
				{
					if (vertices_[it.second]->getType() == LINK)
					{
						d = ext->distance(vertices_[it.second]->position_);

						if (d < interact_range_) //if (d < interact_range_)
						{
							found = true;
							break;
						}
					}
				}
				break;
			case GOAL:
				for (auto & it : vertex_map_)
					if (ext->checkList(it.first))
					{
						found = true;
						break;
					}
				break;
			case LINK:
				found = true;
				break;
			case DUMMY_LINK:
				found = true;
				break;
			default:
				break;
		}
		if (found)
		{
			ext->activate();
		}
		else
		{
			ext->deactivate();
		}
	}

	bool MeshGraph::connect(const std::string & name)
	{
		if (!initialised_)
		{
			INDICATE_FAILURE
			return false;
		}
		if (vertex_map_.find(name) == vertex_map_.end())
		{
			ERROR("Vertex ["<<name<<"] does not exist, connection failed");
			return false;
		}

		return vertices_[vertex_map_.at(name)]->activate();
	}

	bool MeshGraph::disconnect(const std::string & name)
	{
		if (!initialised_)
		{
			INDICATE_FAILURE
			return false;
		}
		if (vertex_map_.find(name) == vertex_map_.end())
		{
			WARNING("Vertex ["<<name<<"] does not exist");
			return true;
		}

		return vertices_[vertex_map_.at(name)]->deactivate();
	}

	bool MeshGraph::hasVertex(const std::string & name)
	{
		if (!initialised_)
		{
			INDICATE_FAILURE
			return false;
		}
		if (vertex_map_.find(name) == vertex_map_.end())
			return false;
		return true;
	}

	bool MeshGraph::getGoalDistanceEigen(Eigen::MatrixXd & dist)
	{
		if (!initialised_ || vertex_map_.size() == 0 || vertices_.size() == 0)
		{
			INDICATE_FAILURE
			return false;
		}
		dist.setZero(size_, size_);
		double tmpd, tmpbound;
		for (int j = 0; j < robot_size_; j++)
		{
			checkActivation(vertices_[j]);
			for (int l = j + 1; l < size_; l++)
			{
				checkActivation(vertices_[l]);
				if (vertices_[l]->isActive() && vertices_[j]->isActive())
				{
					if (l < robot_size_)	//	Both are robot links
						dist(j, l) = vertices_[j]->distance(vertices_[l]->position_);
					else	//	Robot link and external object
					{
						//	Object l is6 an obstacle to all links
						if (vertices_[l]->getType() == OBSTACLE_TO_ALL)
						{

							//std::cout << "Link [" << vertices_[j]->getName() << "] is too close to [" << vertices_[l]->getName() << "] (" << tmpd << "), set desired distance to " << tmpbound << "\n";
							dist(j, l) = tmpbound;

						}
						//	Object l is an obstacle to some particular links
						else if (vertices_[l]->getType() == OBSTACLE)
						{
							//	Link j is in the interacting list
							if (vertices_[l]->checkList(vertices_[j]->getName()))
							{
								if (checkClose(j, l, tmpd, tmpbound))
								{
									//std::cout << "Link [" << vertices_[j]->getName() << "] is too close to [" << vertices_[l]->getName() << "] (" << tmpd << "), set desired distance to " << tmpbound << "\n";
									dist(j, l) = tmpbound;
								}
							}
						}
						//	Object l is a goal, or ignore
					}
					//	HERE: j and l both are greater then robot_size_.
					//	They both are external objects, which means the distance between them are not controllable,
					//	and we do not care (leave as zero)
				}
			}
		}
		return true;
	}

	bool MeshGraph::getAcutalDistanceEigen(Eigen::MatrixXd & dist)
	{
		if (!initialised_ || vertex_map_.size() == 0 || vertices_.size() == 0)
		{
			INDICATE_FAILURE
			return false;
		}

		dist.setZero(size_, size_);
		double d = 0.0;
		double tmpd, tmpbound;

		for (int j = 0; j < robot_size_; j++)
		{
			checkActivation(vertices_[j]);
			for (int l = 0; l < size_; l++)
			{
				checkActivation(vertices_[l]);
				if (vertices_[l]->isActive() && vertices_[j]->isActive())
				{
					if (l < robot_size_)	//	Both are robot links
						dist(j, l) = vertices_[j]->distance(vertices_[l]->position_);
					else	//	Robot link and external object
					{
						//	Object l is an obstacle to all links
						if (vertices_[l]->getType() == OBSTACLE_TO_ALL)
						{
							dist(j, l) = vertices_[j]->distance(vertices_[l]->position_);
						}
						//	Object l is an obstacle to some particular links
						else if (vertices_[l]->getType() == OBSTACLE)
						{
							//	Link j is in the interacting list
							if (vertices_[l]->checkList(vertices_[j]->getName()))
								dist(j, l) = vertices_[j]->distance(vertices_[l]->position_);
						}
						else if (vertices_[l]->getType() == GOAL)
						{
							if (vertices_[l]->checkList(vertices_[j]->getName()))
								dist(j, l) = vertices_[j]->distance(vertices_[l]->position_);
						}
						//	Object l is a goal, or ignore
					}
					//	HERE: j and l both are greater then robot_size_.
					//	They both are external objects, which means the distance between them are not controllable,
					//	and we do not care (leave as zero)
				}

			}
		}
//		dist_.resize(size_, size_);
		dist_ = dist;
		return true;
	}

	void MeshGraph::publishEdges()
	{
		for (int i = 0; i < 5; i++)
		{
			edges_[i].points.resize(0);
			edges_[i].colors.resize(0);
		}
		for (int j = 0; j < robot_size_; j++)
		{
			checkActivation(vertices_[j]);
			for (int l = 0; l < size_; l++)
			{
				checkActivation(vertices_[l]);
				if (vertices_[l]->isActive() && vertices_[j]->isActive())
				{
					//	Vision
					if (true
							|| vertices_[j]->getType() != VERTEX_TYPE::DUMMY_LINK
									&& vertices_[l]->getType() != VERTEX_TYPE::DUMMY_LINK
//							&& vertice6s_[j]->getName().compare("goal_ori") != 0
//							&& vertices_[l]->getName().compare("goal_ori") != 0
									&& vertices_[j]->getName().compare("DummyTable_link6") != 0
									&& vertices_[l]->getName().compare("DummyTable_link6") != 0)
					{
						geometry_msgs::Point tmpj, tmpl;
						tf::pointEigenToMsg(vertices_[j]->position_, tmpj);
						tf::pointEigenToMsg(vertices_[l]->position_, tmpl);
						if (vertices_[l]->getType() == VERTEX_TYPE::LINK)
						{
							edges_[1].points.push_back(tmpj);
							edges_[1].points.push_back(tmpl);
						}
						else if (vertices_[l]->getType() == VERTEX_TYPE::GOAL)
						{
							if (vertices_[l]->checkList(vertices_[j]->getName()))
							{
								edges_[2].points.push_back(tmpj);
								edges_[2].points.push_back(tmpl);
							}
							else
							{
								edges_[0].points.push_back(tmpj);
								edges_[0].points.push_back(tmpl);
							}
						}
						else if (vertices_[l]->getType() == VERTEX_TYPE::OBSTACLE_TO_ALL)
						{
							double bound = vertices_[j]->getRadius() + vertices_[l]->getRadius()
									+ eps_;
							if (dist_(j, l) <= bound)
							{
								edges_[3].points.push_back(tmpj);
								edges_[3].points.push_back(tmpl);
							}

							edges_[4].points.push_back(tmpj);
							edges_[4].points.push_back(tmpl);
						}
					}
				}
			}
		}

		for (int i = 0; i < 5; i++)
			pubs_[i].publish(edges_[i]);
	}
	double MeshGraph::getInteractRange()
	{
		if (!initialised_)
			return 0;
		return interact_range_;
	}

	double MeshGraph::getEps()
	{
		return eps_;
	}

	double MeshGraph::getCurrentSize()
	{
		if (!initialised_)
			return -1;
		return vertex_map_.size();
	}

	bool MeshGraph::checkClose(const int j, const int l, double &d, double &bound)
	{
		if (j >= vertices_.size() || l >= vertices_.size())
		{
			d = bound = 0;
			return false;
		}
		if (!vertices_[j]->isActive() || !vertices_[l]->isActive())
		{
			d = bound = 0.0;
			return false;
		}
		d = vertices_[j]->distance(vertices_[l]->position_);
		bound = vertices_[j]->getRadius() + vertices_[l]->getRadius() + eps_;
		if (d < bound)
			return true;
		return false;
	}

	void MeshGraph::printGraph()
	{
		std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
		if (initialised_)
		{
			std::cout << "Graph [" << name_ << "] has maximum size of " << size_ << ", currently has " << vertex_map_.size() << " activated vertices: \n";
			for (int i = 0; i < vertex_map_.size(); i++)
			{
				std::cout << "[" << vertices_[i]->getName() << "] is ";
				switch (vertices_[i]->getType())
				{
					case LINK:
						std::cout << "robot link [r=" << vertices_[i]->getRadius()
								<< "]\n Current position(" << vertices_[i]->position_.transpose()
								<< ")\n";
						break;
					case DUMMY_LINK:
						std::cout << "dummy robot link [r=" << vertices_[i]->getRadius()
								<< "]\n Current position(" << vertices_[i]->position_.transpose()
								<< ")\n";
						break;
					case OBSTACLE_TO_ALL:
						std::cout << "obstacle to all links [r=" << vertices_[i]->getRadius()
								<< "]\n Current position(" << vertices_[i]->position_.transpose()
								<< ")\n";
						break;
					case OBSTACLE:
						std::cout << "obstacle [r=" << vertices_[i]->getRadius() << "] to ";
						for (int j = 0; j < vertices_[i]->toLinks_.size(); j++)
							std::cout << "[" << vertices_[i]->toLinks_[j] << "]";
						std::cout << "\nCurrent position(" << vertices_[i]->position_.transpose()
								<< ")\n";
						break;
					case GOAL:
						std::cout << "goal to " << vertices_[i]->toLinks_[0]
								<< "\n Current position(" << vertices_[i]->position_.transpose()
								<< ")\n";
						break;
					default:
						std::cout << "currently deactivated\n";
						break;
				}
			}
		}
		else
		{
			std::cout << "This graph has not been initialised\n";
		}
		std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
	}

	bool MeshGraph::computeVelocity()
	{
		dist_ = groundTruthDistance();
		if (first_time_)
		{
			old_dist_ = dist_;
			vel_.setZero();
			first_time_ = false;
		}
		else
		{
			vel_ = 20 * (dist_ - old_dist_);
			old_dist_ = dist_;
		}
		//std::cout<<"velocity: \n"<<vel_<<std::endl;
		return true;
	}

	Eigen::MatrixXd MeshGraph::groundTruthDistance()
	{
		Eigen::MatrixXd dist;
		dist.resize(size_, size_);
		for (int j = 0; j < size_; j++)
		{
			for (int l = 0; l < size_; l++)
			{
				dist(j, l) = vertices_[j]->distance(vertices_[l]->position_);
			}
		}
		return dist;
	}

	Eigen::MatrixXd MeshGraph::getVelocity()
	{
		return vel_;
	}

	bool MeshGraph::hasActiveObstacle()
	{
		for (int i = 0; i < vertices_.size(); i++)
		{
			if (vertices_[i]->getType() == VERTEX_TYPE::OBSTACLE_TO_ALL
					|| vertices_[i]->getType() == VERTEX_TYPE::OBSTACLE)
			{
				if (vertices_[i]->isActive())
					return true;
			}
		}
		return false;
	}
}	//	namespace exotica


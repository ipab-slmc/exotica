/*
 * DMesh.cpp
 *
 *  Created on: 17 Jul 2014
 *      Author: yiming
 */

#include "kinematic_maps/DMesh.h"
#define DEBUG_MODE
REGISTER_TASKMAP_TYPE("DMesh", exotica::DMesh);
#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) {INDICATE_FAILURE; return PAR_ERR;}}
#define XML_OK(x) if(!ok(x)){INDICATE_FAILURE; return PAR_ERR;}
namespace exotica
{
	DMesh::DMesh() :
			eff_size_(0), safe_range_(0.05), max_size_(20), gain_(10), scale_(1.0)
	{
		initialised_ = false;
	}

	DMesh::~DMesh()
	{
		//TODO
	}

    EReturn DMesh::update(const Eigen::VectorXd & x, const int t)
	{
		LOCK(scene_lock_);
		invalidate();
		if (scene_ == nullptr)
		{
			INDICATE_FAILURE
			;
			return MMB_NIN;
		}

		if (!initialised_)
		{
			INDICATE_FAILURE
			;
			return MMB_NIN;
		}

		if (ok(computeJacobian(x)))
		{
            setPhi(laplace_,t);
			std::cout<<"Laplace size="<<laplace_.rows()<<std::endl;
            setJacobian(J,t);
		}
		else
		{
			INDICATE_FAILURE
			;
			return FAILURE;
		}

		//!< Phi will be updated in computeJacobian
		return SUCCESS;
	}

	EReturn DMesh::initDerived(tinyxml2::XMLHandle & handle)
	{
		tinyxml2::XMLElement* xmltmp;
		XML_CHECK("SafeRange");
		XML_OK(getDouble(*xmltmp, safe_range_));
		XML_CHECK("MaxGraph");
		XML_OK(getInt(*xmltmp, max_size_));
		XML_CHECK("Gain");
		XML_OK(getDouble(*xmltmp, gain_));
		vertices_.resize(max_size_, 3);

		eff_size_ = scene_->getMapSize();
		task_size_ = eff_size_ * (eff_size_ - 1) / 2 + eff_size_ * (max_size_ - eff_size_);
		std::cout << "Distance Mesh: Robot end-effect size=" << eff_size_ << " Maximum graph size=" << max_size_ << " Task space size=" << task_size_ << std::endl;
		laplace_.resize(task_size_);
		weights_.setOnes(max_size_, max_size_);
		meshes_.vertices.resize(max_size_);
		for (int i = 0; i < max_size_; i++)
		{
			meshes_.vertices[i].name = "unknown";
			if (i < eff_size_)
				meshes_.vertices[i].type = exotica_msgs::MeshVertex::LINK;
			else
				meshes_.vertices[i].type = exotica_msgs::MeshVertex::IGNORE;
			meshes_.vertices[i].radius = 0;

			if (i < eff_size_)
				meshes_.vertices[i].isActive = true;
			else
				meshes_.vertices[i].isActive = false;
		}

		std::cout << "Incremental Distance Interaction Mesh: Maximum graph size=" << max_size_ << ". Robot end-effector size=" << eff_size_ << ". Free virtual vertices: " << max_size_
				- eff_size_ << std::endl;
		initialised_ = true;
		return SUCCESS;
	}

	EReturn DMesh::taskSpaceDim(int & task_dim)
	{
		LOCK(locker_);
		if (!initialised_)
		{
			INDICATE_FAILURE
			;
			return MMB_NIN;
		}

		task_dim = task_size_;
		return SUCCESS;
	}

	EReturn DMesh::computeGoalLaplace(const exotica_msgs::MeshVertexArray & V)
	{
		int N = V.vertices.size();
		Eigen::MatrixXd vmat(N, 3);
		for (int i = 0; i < N; i++)
		{
			vmat(i, 0) = V.vertices[i].position.x;
			vmat(i, 1) = V.vertices[i].position.y;
			vmat(i, 2) = V.vertices[i].position.z;
		}
		return computeGoalLaplace(vmat);
	}

	EReturn DMesh::computeGoalLaplace(const Eigen::MatrixXd & V)
	{
		updateGraph();
		uint M = V.rows();
		uint j, l, cnt = 0;
		laplace_.setZero(task_size_);
		dist_.setZero(M, M);
		double tmpd = 0;

		for (j = 0; j < eff_size_; j++)
		{
			for (l = j + 1; l < M; l++)
			{

				if (l < eff_size_)
				{
					dist_(j, l) = (V.row(j) - V.row(l)).norm();
				}
				else
				{
					if (meshes_.vertices[l].type == exotica_msgs::MeshVertex::OBSTACLE)
					{
						for (int k = 0; k < meshes_.vertices[l].toLinks.size(); k++)
						{
							if (meshes_.vertices[l].toLinks[k] == j
									&& (V.row(j) - V.row(l)).norm()
											< meshes_.vertices[l].radius + safe_range_)
							{
								dist_(j, l) = meshes_.vertices[l].radius + safe_range_;
								break;
							}
						}
					}
					else if (meshes_.vertices[l].type == exotica_msgs::MeshVertex::GOAL)
					{
						for (int k = 0; k < meshes_.vertices[l].toLinks.size(); k++)
						{
							if (meshes_.vertices[l].toLinks[k] == j)
							{
								dist_(j, l) = meshes_.vertices[l].radius + safe_range_;
								break;
							}
						}
					}
				}
			}
		}

		for (j = 0; j < eff_size_; j++)
		{
			for (l = j + 1; l < M; l++)
			{

				if (l < eff_size_)
				{
					laplace_(cnt) = dist_(j, l);
				}
				else
				{
					if (meshes_.vertices[l].type == exotica_msgs::MeshVertex::GOAL)
					{
						laplace_(cnt) = dist_(j, l);
					}
					else if (meshes_.vertices[l].type == exotica_msgs::MeshVertex::OBSTACLE
							&& dist_(j, l) != 0)
					{
						laplace_(cnt) = (gain_
								* (meshes_.vertices[l].radius + safe_range_ - dist_(j, l)))
								* dist_(j, l);
					}
				}
				cnt++;
			}
		}
#ifdef DEBUG_MODE
		std::cout << "Goal Laplace: " << laplace_.transpose() << std::endl;
#endif
		return SUCCESS;
	}
	EReturn DMesh::computeLaplace(const Eigen::MatrixXd & V)
	{
		std::cout << "Compting lap\n" << V.transpose() << std::endl;
		updateGraph();
		uint M = V.rows();
		uint j, l, cnt = 0;
		laplace_.setZero(task_size_);
		dist_.setZero(M, M);
		/** Compute distance matrix (inverse proportional) */
		for (j = 0; j < eff_size_; j++)
		{
			for (l = j + 1; l < M; l++)
			{

				if (l < eff_size_)
				{
					dist_(j, l) = (V.row(j) - V.row(l)).norm();
				}
				else
				{
					if (meshes_.vertices[l].type == exotica_msgs::MeshVertex::OBSTACLE)
					{
						for (int k = 0; k < meshes_.vertices[l].toLinks.size(); k++)
						{
							if (meshes_.vertices[l].toLinks[k] == j
									&& (V.row(j) - V.row(l)).norm()
											< meshes_.vertices[l].radius + safe_range_)
							{
								dist_(j, l) = (V.row(j) - V.row(l)).norm();
								break;
							}
						}
					}
					else if (meshes_.vertices[l].type == exotica_msgs::MeshVertex::GOAL)
					{
						for (int k = 0; k < meshes_.vertices[l].toLinks.size(); k++)
						{
							if (meshes_.vertices[l].toLinks[k] == j)
							{
								dist_(j, l) = (V.row(j) - V.row(l)).norm();
								break;
							}
						}
					}
				}
			}
		}

		for (j = 0; j < eff_size_; j++)
		{
			for (l = j + 1; l < M; l++)
			{

				if (l < eff_size_)
				{
					laplace_(cnt) = dist_(j, l);
				}
				else
				{
					if (meshes_.vertices[l].type == exotica_msgs::MeshVertex::GOAL)
					{
						laplace_(cnt) = dist_(j, l);
					}
					else if (meshes_.vertices[l].type == exotica_msgs::MeshVertex::OBSTACLE
							&& dist_(j, l) != 0)
					{
						laplace_(cnt) = (gain_
								* (meshes_.vertices[l].radius + safe_range_ - dist_(j, l)))
								* dist_(j, l);
					}
				}
				cnt++;
			}
		}
#ifdef DEBUG_MODE
		std::cout << "Laplace: " << laplace_.transpose() << std::endl;
#endif
		return SUCCESS;
	}

	EReturn DMesh::getLaplace(Eigen::VectorXd & lap)
	{
		lap.resize(laplace_.rows());
		lap = laplace_;
		return SUCCESS;
	}
	EReturn DMesh::computeJacobian(const Eigen::VectorXd & q)
	{
		int N = q.size();
		int M = max_size_;
#ifdef DEBUG_MODE
		std::cout << "Computing Jacobian M = " << M << ", N =" << N << std::endl;
#endif
		computeLaplace(vertices_);
#ifdef DEBUG_MODE
		std::cout << "dist:\n" << dist_ << std::endl;
#endif
		Eigen::MatrixXd _p(3 * eff_size_, N);
		J.setZero(task_size_, N);
		scene_->getJacobian(_p);
#ifdef DEBUG_MODE
		std::cout << "_p: \n" << _p << std::endl;
#endif
		double _distance;
		uint i, j, k, n, l, cnt;
		for (i = 0; i < N; i++)
		{
			cnt = 0;
			for (j = 0; j < eff_size_; j++)
			{
				for (l = j + 1; l < M; l++)
				{
					_distance = 0;
					if (l < eff_size_)
					{
						if (dist_(j, l) > 0)
							_distance =
									((vertices_.row(j) - vertices_.row(l)).dot((Eigen::Vector3d(_p.block(3
											* j, i, 3, 1))
											- Eigen::Vector3d(_p.block(3 * l, i, 3, 1)))))
											/ dist_(j, l);
						J(cnt, i) = _distance;
					}
					else if (dist_(j, l) > 0)
					{
						_distance =
								((vertices_.row(j) - vertices_.row(l)).dot(Eigen::Vector3d(_p.block(3
										* j, i, 3, 1)))) / dist_(j, l);
						if (meshes_.vertices[l].type == exotica_msgs::MeshVertex::GOAL)
						{
							for (int k = 0; k < meshes_.vertices[l].toLinks.size(); k++)
							{
								if (meshes_.vertices[l].toLinks[k] == j)
								{
									J(cnt, i) = _distance;
									break;
								}
							}
						}
						else if (meshes_.vertices[l].type == exotica_msgs::MeshVertex::OBSTACLE)
						{
							for (int k = 0; k < meshes_.vertices[l].toLinks.size(); k++)
							{
								if (meshes_.vertices[l].toLinks[k] == j)
								{
									J(cnt, i) = (gain_ * (meshes_.vertices[l].radius + safe_range_))
											* _distance - 2 * gain_ * dist_(j, l) * _distance;
									break;
								}
							}
						}
						else
							J(cnt, i) = 0;
					}
					else
						J(cnt, i) = 0;
					cnt++;
				}
			}
		}
#ifdef DEBUG_MODE
		std::cout << "DMesh Jacobians:" << J.rows() << "x" << J.cols() << "\n" << J << std::endl;
#endif
		return SUCCESS;
	}
	EReturn DMesh::updateGraph()
	{
		Eigen::VectorXd tmp(eff_size_ * 3);
		std::vector<std::string> temp_vector;
		scene_->getForwardMap(tmp, temp_vector);
		vertices_.setZero(max_size_, 3);
		for (int i = 0; i < eff_size_; i++)
		{
			vertices_.row(i) = tmp.segment(i * 3, 3);
			meshes_.vertices[i].position.x = vertices_(i, 0);
			meshes_.vertices[i].position.y = vertices_(i, 1);
			meshes_.vertices[i].position.z = vertices_(i, 2);
		}
		for (int j = eff_size_; j < max_size_; j++)
		{
			vertices_(j, 0) = meshes_.vertices[j].position.x;
			vertices_(j, 1) = meshes_.vertices[j].position.y;
			vertices_(j, 2) = meshes_.vertices[j].position.z;
		}
		return SUCCESS;
	}

	void DMesh::updateExternal(const exotica_msgs::MeshVertexArray & ext)
	{
		if (ext.vertices.size() != max_size_ - eff_size_)
		{
			ERROR("Wrong size, expected external object size="<<max_size_-eff_size_<<", received size="<<ext.vertices.size());
		}
		else
		{
			for (int i = eff_size_; i < max_size_; i++)
				meshes_.vertices[i] = ext.vertices[i - eff_size_];
		}
	}
} // namespace exotica

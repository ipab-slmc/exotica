/*
 * IMesh.cpp
 *
 *  Created on: 18 Mar 2014
 *      Author: yimingyang
 */

#include "kinematic_maps/IMesh.h"
#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) {INDICATE_FAILURE; return PAR_ERR;}}
#define XML_OK(x) if(!ok(x)){INDICATE_FAILURE; return PAR_ERR;}
//#define DEBUG_MODE
REGISTER_TASKMAP_TYPE("IMesh", exotica::IMesh);
exotica::IMesh::IMesh() :
		eff_size_(0), max_size_(8), safe_range_(0.3)
{
	initialised_ = false;
}

exotica::IMesh::~IMesh()
{
	//TODO
}

exotica::EReturn exotica::IMesh::update(const Eigen::VectorXd & x, const int t)
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
		setPhi(getVectorLaplace(), t);
		setJacobian(J, t);
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

exotica::EReturn exotica::IMesh::initDerived(tinyxml2::XMLHandle & handle)
{
	tinyxml2::XMLElement* xmltmp;
	XML_CHECK("SafeRange");
	XML_OK(getDouble(*xmltmp, safe_range_));
	XML_CHECK("MaxGraph");
	XML_OK(getInt(*xmltmp, max_size_));
	vertices_.resize(max_size_, 3);
	vertices_old_.resize(max_size_, 3);
	original_v_.resize(max_size_, 3);
	laplace_.resize(max_size_, 3);
	weights_.setOnes(max_size_, max_size_);
	eff_size_ = scene_->getMapSize(object_name_);
	ext_.setOnes(3, max_size_ - eff_size_);
	ext_ = ext_ * 100;
	ext_old_ = ext_;

	std::cout << "Incremental Interaction Mesh: Maximum graph size=" << max_size_ << ". Robot end-effector size=" << eff_size_ << ". Free virtual vertices: " << max_size_
			- eff_size_ << std::endl;
	initialised_ = true;
	return SUCCESS;
}

exotica::EReturn exotica::IMesh::initManual(int n)
{
	vertices_.resize(n, 3);
	laplace_.resize(n, 3);
	weights_.setOnes(n, n);
	initialised_ = true;
	return SUCCESS;
}

exotica::EReturn exotica::IMesh::setVertices(const Eigen::VectorXd & v)
{
	if (v.rows() != vertices_.rows() * 3)
		return FAILURE;
	for (int i = 0; i < vertices_.rows(); i++)
		vertices_.row(i) = v.segment(i * 3, 3);
	return SUCCESS;
}

exotica::EReturn exotica::IMesh::taskSpaceDim(int & task_dim)
{
	LOCK(locker_);
	if (!initialised_)
	{
		INDICATE_FAILURE
		;
		return MMB_NIN;
	}
	if (max_size_ <= 0)
	{
		INDICATE_FAILURE
		;
		return FAILURE;
	}
	else
	{
		task_dim = 3 * max_size_;
		return SUCCESS;
	}
}
exotica::EReturn exotica::IMesh::computeLaplace(const Eigen::MatrixXd & V)
{
	uint N = V.rows();
	Eigen::MatrixXd dist(N, N);
	Eigen::VectorXd wsum(N);
	return computeLaplace(V, wsum, dist);
}

exotica::EReturn exotica::IMesh::computeLaplace(const Eigen::MatrixXd & V, Eigen::VectorXd & wsum,
		Eigen::MatrixXd & dist)
{
	updateVertices();
	ros::Time start = ros::Time::now();
	uint N = V.rows();
	if (N != max_size_)
	{
		ERROR("Size is wrong at computeLaplace! N="<<N<<" max size="<<max_size_);
		return FAILURE;
	}
	dist = Eigen::MatrixXd::Zero(N, N);
	wsum = Eigen::VectorXd::Zero(N);
	uint j, l;
	double w;
	/** Compute distance matrix (inverse proportional) */
	for (j = 0; j < N; j++)
	{
		for (l = j + 1; l < N; l++)
		{
			if (!(j >= eff_size_ && l >= eff_size_))
			{
				dist(j, l) = sqrt((V.row(j) - V.row(l)).dot((V.row(j) - V.row(l))));
				dist(l, j) = dist(j, l);
			}
		}
	}
	/** Computer weight normaliser */
	for (j = 0; j < N; j++)
	{
		for (l = 0; l < N; l++)
		{
			if (dist(j, l) > 0 && j != l)
			{
				wsum(j) += weights_(j, l) / dist(j, l);
			}
		}
	}
	/** Compute Laplace coordinates */
	for (j = 0; j < N; j++)
	{
		laplace_.row(j) = V.row(j);
		for (l = 0; l < N; l++)
		{
			if (j != l)
			{
				if (dist(j, l) > 0 && wsum(j) > 0)
				{
					w = weights_(j, l) / (dist(j, l) * wsum(j));
					laplace_.row(j) -= V.row(l) * w;
				}
			}
		}
	}
#ifdef DEBUG_MODE
	std::cout << "Laplace Coordinates:\n" << laplace_ << std::endl;
#endif
	ros::Duration d = ros::Time::now() - start;
	//std::cout << "Laplace computation time=" << d.toSec() << std::endl;
	return SUCCESS;
}

Eigen::MatrixXd exotica::IMesh::getLaplace()
{
	if (!initialised_)
	{
		INDICATE_FAILURE
		;
		return Eigen::MatrixXd::Zero(3, 0);
	}
	return laplace_;
}
Eigen::VectorXd exotica::IMesh::getVectorLaplace()
{
	if (!initialised_)
	{
		INDICATE_FAILURE
		;
		return Eigen::VectorXd::Zero(0);
	}
	Eigen::VectorXd lap_vec(3 * max_size_);
	for (int i = 0; i < max_size_; i++)
	{
		lap_vec(3 * i) = laplace_(i, 0);
		lap_vec(3 * i + 1) = laplace_(i, 1);
		lap_vec(3 * i + 2) = laplace_(i, 2);
	}
#ifdef DEBUG_MODE
	std::cout << "Vector Laplace " << lap_vec.transpose() << std::endl;
#endif

	return lap_vec;
}

exotica::EReturn exotica::IMesh::computeJacobian(const Eigen::VectorXd & q)
{
	int N = q.size();
	int M = vertices_.rows();
	if (M != max_size_)
	{
		ERROR("Size is wrong at computeJacobian! N="<<N<<" max size="<<max_size_);
		return FAILURE;
	}
#ifdef DEBUG_MODE
	std::cout << "Computing Jacobian M = " << M << ", N =" << N << std::endl;
#endif
	Eigen::MatrixXd dist(M, M);
	Eigen::VectorXd wsum(M);
	vertices_ = original_v_;
	computeLaplace(vertices_, wsum, dist);
	ros::Time start = ros::Time::now();
#ifdef DEBUG_MODE
	std::cout << "wsum:\n" << wsum << std::endl;
	std::cout << "dist:\n" << dist << std::endl;
	std::cout << "dist sum: " << dist.sum() << std::endl;
#endif

	Eigen::MatrixXd _p(3 * eff_size_, N);
	J.setZero(3 * M, N);
	scene_->getJacobian(object_name_, _p);
#ifdef DEBUG_MODE
	std::cout << "_p: \n" << _p << std::endl;
#endif
	double A, _A, Sk, Sl, w, _w;
	int i, j, k, l;
	Eigen::Vector3d distance, _distance = Eigen::Vector3d::Zero(3, 1);
	for (i = 0; i < N; i++)
	{
		for (j = 0; j < M; j++)
		{
			if (j < eff_size_)
				J.block(3 * j, i, 3, 1) = _p.block(3 * j, i, 3, 1);
			for (l = 0; l < M; l++)
			{
				if (j != l)
				{
					if (dist(j, l) > 0 && wsum(j) > 0 && weights_(j, l) > 0)
					{
						A = dist(j, l) * wsum(j);
						w = weights_(j, l) / A;

						_A = 0;
						distance = vertices_.row(j) - vertices_.row(l);
						if (j < eff_size_)
						{
							if (l < eff_size_)
								//Both j and l are points on the robot
								_distance = _p.block(3 * j, i, 3, 1) - _p.block(3 * l, i, 3, 1);
							else
								//l is not on the robot
								_distance = _p.block(3 * j, i, 3, 1);
						}
						else
						{
							if (l < eff_size_)
								//j is not on the robot
								_distance = -_p.block(3 * l, i, 3, 1);
						}

						Sl = distance.dot(_distance) / dist(j, l);
						for (k = 0; k < M; k++)
						{
							if (j != k && dist(j, k) > 0 && weights_(j, k) > 0)
							{
								distance = vertices_.row(j) - vertices_.row(k);
								if (j < eff_size_)
								{
									if (k < eff_size_)
										_distance = _p.block(3 * j, i, 3, 1)
												- _p.block(3 * k, i, 3, 1);
									else
										_distance = _p.block(3 * j, i, 3, 1);
								}
								else
								{
									if (k < eff_size_)
										_distance = -_p.block(3 * k, i, 3, 1);
								}
								Sk = distance.dot(_distance) / dist(j, k);
								_A += weights_(j, k) * (Sl * dist(j, k) - Sk * dist(j, l))
										/ (dist(j, k) * dist(j, k));
							}
						}
						_w = -weights_(j, l) * _A / (A * A);
					}
					else
					{
						_w = 0;
						w = 0;
					}
					if (l < eff_size_)
						J.block(3 * j, i, 3, 1) -= vertices_.row(l).transpose() * _w
								+ _p.block(3 * l, i, 3, 1) * w;
					else
						J.block(3 * j, i, 3, 1) -= vertices_.row(l).transpose() * _w;
				}
			}
		}
	}
#ifdef DEBUG_MODE
	std::cout << "IMesh Jacobians:" << J.rows() << "x" << J.cols() << "\n" << J << std::endl;
#endif
	ros::Duration d = ros::Time::now() - start;
	//std::cout << "Jacobian computation time=" << d.toSec() << std::endl;
	return SUCCESS;
}

exotica::EReturn exotica::IMesh::updateVertices()
{
	ros::Time start = ros::Time::now();
	Eigen::VectorXd tmp(eff_size_ * 3);
	scene_->getForwardMap(object_name_, tmp);
	int Size = eff_size_ + ext_.cols();
	vertices_.resize(Size, 3);
	for (int i = 0; i < eff_size_; i++)
		vertices_.row(i) = tmp.segment(i * 3, 3);
	for (int j = 0; j < ext_.cols(); j++)
		vertices_.row(j + eff_size_) = ext_.col(j);
	laplace_.resize(Size, 3);
	vertices_old_ = vertices_;
	//Now lets do magic to the vertices and weights!
	double d = 0, dsum = 0, dmax = 0, w;
	original_v_ = vertices_;
	weights_.setZero(Size, Size);
	visual_map_.setZero(Size, Size);
	for (int i = 0; i < eff_size_; i++)
	{
		for (int j = i + 1; j < eff_size_; j++)
		{
			d = (vertices_.row(i) - vertices_.row(j)).norm();
			weights_(i, j) = .01 * d;
			weights_(j, i) = .01 * d;
			visual_map_(i, j) = 1;
			visual_map_(j, i) = 1;
			dsum += d;
			if (dmax < d)
				dmax = d;
		}
	}
	for (int i = 0; i < ext_.cols(); i++)
	{
		Eigen::Vector3d tmp(3);
		tmp.setZero();
		int cnt = 0;
		for (int j = 0; j < eff_size_; j++)
		{
			d = (vertices_.row(j) - vertices_.row(i + eff_size_)).norm();
			if (d <= safe_range_)
			{
				w = eff_size_ * (safe_range_ - d);

				tmp += vertices_.row(j)
						+ (safe_range_ - d + 0.1)
								* (vertices_.row(j) - vertices_.row(i + eff_size_));
				cnt++;
				visual_map_(i + eff_size_, j) = 1;
				visual_map_(j, i + eff_size_) = 1;
			}
			else
			{
				w = 0;

			}
			weights_(i + eff_size_, j) = w;
			weights_(j, i + eff_size_) = w;
		}

		if (cnt != 0)
			vertices_.row(i + eff_size_) = 1 / cnt * tmp;
	}
	vertices_old_ = original_v_;
	ros::Duration dd = ros::Time::now() - start;
	//std::cout << "Update computation time=" << dd.toSec() << std::endl;
#ifdef DEBUG_MODE
	std::cout << "New weight = \n" << weights_ << std::endl;
#endif
	return SUCCESS;
}

exotica::EReturn exotica::IMesh::updateExternal(const Eigen::Matrix3Xd & ext)
{
	if (ext_.rows() != ext.rows() || ext_.cols() != ext.cols())
	{
		ERROR("External object size is wrong!");
		return FAILURE;
	}
	ext_old_ = ext_;
	ext_ = ext;
	return SUCCESS;
}

Eigen::MatrixXd exotica::IMesh::getVertices()
{
	return vertices_;
}
Eigen::MatrixXd exotica::IMesh::getOriginalVertices(Eigen::MatrixXd & v_map)
{
	v_map.resize(visual_map_.rows(), visual_map_.cols());
	v_map = visual_map_;
	return original_v_;
}

exotica::EReturn exotica::IMesh::setWeight(int i, int j, double weight)
{
	uint M = weights_.cols();
	if (i < 0 || i >= M || j < 0 || j >= M)
	{
		std::cout << "Invalid weight element (" << i << "," << j << "). Weight matrix " << M << "x" << M << std::endl;
		return FAILURE;
	}
	if (weight < 0)
	{
		std::cout << "Invalid weight: " << weight << std::endl;
		return FAILURE;
	}
	weights_(i, j) = weight;
	return SUCCESS;
}

exotica::EReturn exotica::IMesh::setWeights(const Eigen::MatrixXd & weights)
{
	uint M = weights_.cols();
	if (weights.rows() != M || weights.cols() != M)
	{
		std::cout << "Invalid weight matrix (" << weights.rows() << "X" << weights.cols() << "). Has to be" << M << "x" << M << std::endl;
		return FAILURE;
	}
	weights_ = weights;
	return SUCCESS;
}

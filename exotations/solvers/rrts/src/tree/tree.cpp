/*
 * tree.cpp
 *
 *  Created on: 18 Mar 2015
 *      Author: yiming
 */
#include "RRTs/tree/tree.h"

namespace exotica
{
	namespace RRTs
	{
		Node::Node() :
				isSet_(false), index_(-1), parent_(-1), p_cost_(0), r_cost_(0)
		{
			children_.clear();
			local_path_.clear();
			//	TODO
		}

		Node::~Node()
		{
			//	TODO
		}

		EReturn Node::setNode(const int index, const int parent,
				const Eigen::Ref<Eigen::VectorXd> & x, const double p_cost, const double r_cost)
		{
			LOCK(lock_);
			x_ = x;
			index_ = index;
			parent_ = parent;
			isSet_ = true;
			p_cost_ = p_cost;
			r_cost_ = r_cost;
			return FAILURE;
		}

		EReturn Node::resetParent(const int parent, const double p_cost)
		{
			LOCK(lock_);
			if (!isSet_)
			{
				INDICATE_FAILURE
				return FAILURE;
			}
			parent_ = parent;
			r_cost_ = r_cost_ - p_cost_ + p_cost;
			p_cost_ = p_cost;
			return SUCCESS;
		}

		EReturn Node::addChild(const int child)
		{
			LOCK(lock_);
			if (!isSet_)
			{
				INDICATE_FAILURE
				return FAILURE;
			}
			if (child < 0)
			{
				INDICATE_FAILURE
				return FAILURE;
			}
			children_.push_back(child);
			return SUCCESS;
		}

		double Node::cost2Parent()
		{
			LOCK(lock_);
			return p_cost_;
		}

		double Node::cost2Root()
		{
			LOCK(lock_);
			return r_cost_;
		}

		std::vector<Eigen::VectorXd> & Node::getLocalPath()
		{
			return local_path_;
		}

		Tree::Tree() :
				current_size_(0), maximum_size_(10000)
		{
			//	TODO
		}

		Tree::~Tree()
		{
			//	TODO
		}

		EReturn Tree::initialise(const Eigen::Ref<Eigen::VectorXd> & x_start, const int max)
		{
			maximum_size_ = max;
			nodes_.reserve(max);
			nodes_[0] = current_size_ = 0;
			return SUCCESS;
		}

		EReturn Tree::insertNode(const Eigen::Ref<Eigen::VectorXd> & x, const int parent,
				const double p_cost)
		{
			if (current_size_ >= maximum_size_ || parent > current_size_)
			{
				INDICATE_FAILURE
				return FAILURE;
			}
			nodes_[current_size_] = Node_ptr(new Node());
			nodes_[current_size_]->setNode(current_size_, parent, x, p_cost, nodes_[parent]->cost2Root());
			nodes_[parent]->addChild(current_size_);
			current_size_++;
			return SUCCESS;
		}

		EReturn Tree::rewire(const int index, const int parent, const double p_cost)
		{
			if (index > current_size_ || parent > current_size_)
			{
				INDICATE_FAILURE
				return FAILURE;
			}
			nodes_[index]->resetParent(parent, p_cost);
			return SUCCESS;
		}
	}
}


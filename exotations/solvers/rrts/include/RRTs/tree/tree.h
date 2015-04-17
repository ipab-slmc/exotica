/*
 * tree.h
 *
 *  Created on: 18 Mar 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_RRTS_INCLUDE_RRTS_TREE_TREE_H_
#define EXOTICA_EXOTATIONS_SOLVERS_RRTS_INCLUDE_RRTS_TREE_TREE_H_

#include <exotica/Tools.h>
namespace exotica
{
	namespace RRTs
	{
		//	Implementation of tree node
		class Node
		{
			public:
				/*
				 * \brief	Default constructor
				 */
				Node();

				/*
				 * \brief	Default destructor
				 */
				~Node();

				/*
				 * \brief	Set the node
				 * @param	index	Node index
				 * @param	parent	Parent node index
				 * @param	x		Configuration
				 * @param	p_cost	Cost to parent
				 * @param	r_cost	Cost to root
				 */
				EReturn setNode(const int index, const int parent,
						const Eigen::Ref<Eigen::VectorXd> & x, const double p_cost,
						const double r_cost);
				/*
				 * \brief	Reset parent
				 * @param	parent	New parent index
				 * @param	p_cost	Cost to new parent
				 */
				EReturn resetParent(const int parent, const double p_cost);

				/*
				 * \brief	Append new child index
				 * @param	child	Child index
				 */
				EReturn addChild(const int child);

				/*
				 * \brief	Insert local path that connects current node and its parent
				 * @param	path	Local path
				 */
				EReturn insertLocalPath(const std::vector<Eigen::VectorXd> & path);

				/*
				 * \brief	Cost to parent
				 * @return	Cost to parent
				 */
				double cost2Parent();

				/*
				 * \brief	Cost to root
				 * @return	Cost to root
				 */
				double cost2Root();

				/*
				 * \brief	Get local path
				 * @return	Local path
				 */
				std::vector<Eigen::VectorXd> & getLocalPath();
			private:
				//	Indicate if the node has been set
				bool isSet_;
				//	Configuration
				Eigen::VectorXd x_;
				//	Index
				int index_;
				//	Parent index
				int parent_;
				//	Children indices
				std::vector<int> children_;
				//	Cost to parent
				double p_cost_;
				//	Cost to root
				double r_cost_;
				//	Local path
				std::vector<Eigen::VectorXd> local_path_;
				//	Mutex locker
				boost::mutex lock_;
		};

		typedef boost::shared_ptr<Node> Node_ptr;

		//	Implementation of the tree
		class Tree
		{
			public:
				/*
				 * \brief	Default constructor
				 */
				Tree();

				/*
				 * \brief	Default destructor
				 */
				~Tree();

				/*
				 * \brief	Initialisation function
				 * @param	x_start	Start configuration
				 * @param	max		Maximum tree size
				 */
				EReturn initialise(const Eigen::Ref<Eigen::VectorXd> & x_start, const int max);

				/*
				 * \brief	Insert new node
				 * @param	x		Configuration
				 * @param	parent	Parent node index
				 * @param	p_cost	Cost to parent
				 */
				EReturn insertNode(const Eigen::Ref<Eigen::VectorXd> & x, const int parent,
						const double p_cost);

				/*
				 * \brief	Re-wire the node (change parent)
				 * @param	index	Node index
				 * @param	parent	New parent index
				 * @param	p_cost	Cost to new parent
				 */
				EReturn rewire(const int index, const int parent, const double p_cost);
			private:
				//	The nodes
				std::vector<Node_ptr> nodes_;
				//	Current size
				int current_size_;
				//	Maximum size
				int maximum_size_;
		};
		typedef boost::shared_ptr<Tree> Tree_ptr;
	}
}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_RRTS_INCLUDE_RRTS_TREE_TREE_H_ */

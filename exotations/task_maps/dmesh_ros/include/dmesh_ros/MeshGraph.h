/*
 * MeshGraph.h
 *
 *  Created on: 15 Aug 2014
 *      Author: yiming
 */

#ifndef MESHGRAPH_H_
#define MESHGRAPH_H_

#include <string>
#include <eigen_conversions/eigen_msg.h>
#include <boost/thread/mutex.hpp>
#include <exotica/Tools.h>
#include <exotica/MeshVertexArray.h>
#include <exotica/MeshVertex.h>
#include <exotica/StringList.h>
#include <exotica/BoolList.h>
#include <exotica/Vector.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace exotica {
enum VERTEX_TYPE {
	LINK = 0,
	DUMMY_LINK = 5,
	GOAL = 10,
	OBSTACLE = 20,
	OBSTACLE_TO_ALL = 30,
	IGNORE = 40,
	NONE = 255
};

//	\brief	The vertex
class Vertex {
public:
	/**
	 * \briref	Default constructor
	 * @param	name		The vertex name
	 */
	Vertex(const std::string & name = "UNKNOWN");

	/**
	 * \brief	Set this vertex as robot link
	 * @param	name		The vertex name
	 * @param	real_link	Whether this is a real link or dummy link
	 * @param	position	The position of this vertex
	 * @return	True is succeed, false if something goes wrong
	 */
	bool setAsLink(const std::string & name, bool real_link,
			const Eigen::Vector3d & position, double r);
	bool setAsLink(const std::string & name, bool real_link,
			const geometry_msgs::Point & position, double r);

	bool setToDummy() {
		type_ = VERTEX_TYPE::DUMMY_LINK;
		return true;
	}
	/**
	 * \brief	Set this vertex as goal
	 * @param	name		The vertex name
	 * @param	position	The position of this vertex
	 * @param	tolinks		The interacting links
	 * @return	True is succeed, false if something goes wrong
	 */
	bool setAsGoal(const std::string & name, const Eigen::Vector3d & position,
			double r, const std::vector<std::string> & tolinks, double w);
	bool setAsGoal(const std::string & name,
			const geometry_msgs::Point & position, double r,
			const std::vector<std::string> & tolinks, double w);

	/**
	 * \brief	Set this vertex as obstacle
	 * @param	name		The vertex name
	 * @param	position	The position of this vertex
	 * @param	tolinks		The interacting links
	 * @return	True is succeeded, false if something goes wrong
	 */
	bool setAsObstacle(const std::string & name,
			const Eigen::Vector3d & position, double r);
	bool setAsObstacle(const std::string & name,
			const geometry_msgs::Point & position, double r);
	bool setAsObstacle(const std::string & name,
			const Eigen::Vector3d & position, double r,
			const std::vector<std::string> & tolinks);
	bool setAsObstacle(const std::string & name,
			const geometry_msgs::Point & position, double r,
			const std::vector<std::string> & tolinks);

	/**
	 * \brief	Activate this vertex
	 * @return	True if succeeded, false otherwise
	 */
	bool activate();

	/**
	 * \brief	Deactivate this vertex
	 * @return	True if succeeded, false otherwise
	 */
	bool deactivate();

	/**
	 * \brief	Check if the vertex is activated
	 * @return	True if the vertex is activated, false otherwise
	 */
	bool isActive();

	/**
	 * \brief	Invalidate this vertex
	 */
	void invalidate();

	/**
	 * \brief	Get the distance to another vertex or position
	 * @param	other	Other vertex or position
	 * @return	Distance between two vertex, -1 if something wrong
	 */
	double distance(const Vertex & other);
	double distance(const boost::shared_ptr<Vertex> & other);
	double distance(const Eigen::Vector3d & other);

	/**
	 * \brief	Check whether another link is in the interaction list
	 * @param	link		The other link's name
	 * @return	True if in the list, false otherwise
	 */
	bool checkList(const std::string & link);

	/**
	 * \brief	Get the name of the vertex
	 * @return	The name of this vertex
	 */
	std::string getName();

	/**
	 * \brief	Get he radius of the vertex
	 * @return	The radius of this vertex
	 */
	double getRadius();

	/**
	 * \brief Get the type of the vertex
	 * @return The type of this vertex
	 */
	VERTEX_TYPE getType();

	//	The pose of the vertex (for now, just consider the position)
	Eigen::Vector3d position_;

	//	The list of which robot links this vertex is interacting with
	std::vector<std::string> toLinks_;

	//	The radius of this vertex
	double radius_;

	//	The weighting factor
	double w_;
private:
	//	The name of this vertex
	std::string name_;

	//	The type of the vertex
	VERTEX_TYPE type_;

	//	Flag indicates weather this vertex has been activated
	bool isActive_;

};

//	Vertex smart pointer
typedef boost::shared_ptr<Vertex> VertexPtr;

//	Graph initialiser
struct GraphProperties {
	//	The maximum size
	int size;

	//	List of all robot links in the graph
	std::vector<std::string> & link_names;

	//	Interactive range
	//	i.e. Object within this range will be considered to interact with
	double interact_range_;

	//	Safety threshold
	double eps;

};
/**
 * \brief	MeshGraph contains all the information of the vertices
 */
class MeshGraph {
public:
	/**
	 * \brief	Constructor
	 * @param	name		The name of the graph
	 */
	MeshGraph(const std::string & name = "MeshGraph");

	/**
	 * \brief	Check whether the graph was initialised
	 * @return	True if initialised, false otherwise
	 */
	bool isInitialised();

	/**
	 * \brief	Get the name of this graph
	 * @return	The name of this graph
	 */
	std::string getName();

	/**
	 * \brief	Get the maximum size of this graph
	 * @param	size		The maximum size of this graph
	 * @return	True if succeeded, false otherwise
	 */
	bool getGraphSize(int size);
	int getGraphSize();

	/**
	 * \brief	Get the robot link size of this graph
	 * @param	size		The robot link size of this graph
	 * @return	True if succeeded, false otherwise
	 */
	bool getRobotSize(int robot_size);
	int getRobotSize();

	/**
	 * \brief	Get vertex pointer
	 * @prarm	index	The vertex index
	 * @return	Vertex pointer
	 */
	VertexPtr getVertex(const int index);

	/**
	 * \brief	Initalise the graph
	 * @param	size		The maximum graph size
	 * @param	link_names	The names of robot links
	 * @param	link_type	The types of the link, real or dummy
	 * @param	i_range		Interacting range
	 * @param	eps			Safety threshold
	 * @param	dummy_table	Flag indicate if use the dummy table obstacle
	 * @return	True if succeeded, false otherwise
	 */
	bool initialisation(const int size,
			const std::vector<std::string> & link_names,
			const std::vector<bool> link_type, const std::vector<double> & link_radius, const double i_range,
			const double eps, const bool dummy_table);

	/**
	 * \brief	Update robot links
	 * @param	link_poses	Links' position that needs to be updated
	 * @return	True if succeeded, false otherwise
	 */
	bool updateLinks(const Eigen::VectorXd & link_poses);
	bool updateLinks(const Eigen::MatrixX3d & link_poses);

	/**
	 * \brief	Update individual link
	 * @param	name		Link name
	 * @param	pose		Link pose
	 * @return	True if succeeded, false otherwise
	 */
	bool updateLink(const std::string & name, const Eigen::Vector3d & pose);

	/**
	 * \brief	Update external vertices
	 * @param	ext			external objects
	 * @return	True if succeeded, false otherwise
	 */
	bool updateExternal(const exotica::MeshVertex & ext);
	bool updateExternal(const exotica::MeshVertexConstPtr & ext);

	/**
	 * \brief	Remove vertex from graph
	 * @param	name		Vertex name
	 * @return	True if succeeded, false otherwise
	 */
	bool removeVertex(const std::string & name);

	/**
	 * \brief	Connect the external vertex from the graph
	 * @param	name		The name of the external vertex
	 * @return	True if succeeded, false otherwise
	 */
	bool connect(const std::string & name);

	/**
	 * \brief	Disconnect the vertex from the graph
	 * @param	name		The name of the vertex
	 * @return	True if succeeded, false otherwise
	 */
	bool disconnect(const std::string & name);

	/**
	 * \brief	Check if the vertex is in the graph
	 * @param	name		The name of the vertex
	 * @return	True if the vertex exist, false otherwise
	 */
	bool hasVertex(const std::string & name);

	/**
	 * \brief	Get GOAL vertices and distances in Eigen format
	 * @param	dist		Distance matrix
	 * @return	True if succeeded, false otherwise
	 */
	bool getGoalDistanceEigen(Eigen::MatrixXd & dist);

	/**
	 * \brief	Get	ACTUAL vertices and distances in Eigen format
	 * @param	dist		Distance matrix
	 * @return	True if succeeded, false otherwise
	 */
	bool getAcutalDistanceEigen(Eigen::MatrixXd & dist);

	/**
	 * \brief	Get interact range
	 * @return	Interact range
	 */
	double getInteractRange();

	/**
	 * \brief	Get safety threshold
	 * @return	Safety threshold
	 */
	double getEps();

	/*
	 * \brief	Check whether another vertex is close
	 * @param	j,l		Vertex indexes
	 * @param	d		Distance between vertices j and l
	 * @param	bound	Bound
	 * @return	True if yes, false otherwise
	 */
	bool checkClose(const int j, const int l, double &d, double &bound);

	/**
	 * \brief	Get currently activated graph size
	 * @return	Current size, -1 if not initialised
	 */
	double getCurrentSize();

	/*
	 * \brief	Compute distance velocity. This is called by a ROS timer callback function
	 * @return	True if succeeded, false otherwise
	 */
	bool computeVelocity();

	Eigen::MatrixXd getVelocity();

	/**
	 * \brief	Check if currently there are any close obstacles
	 * return	True if yes, false otherwise
	 */
	bool hasActiveObstacle();

	//	Print the graph
	void printGraph();

	void publishEdges();
private:

	/*
	 * \brief	Check if a vertex needs to be activated
	 * @param	ext		The new external object
	 */
	void checkActivation(const VertexPtr & ext);

	Eigen::MatrixXd groundTruthDistance();

	//	Name of the graph, usually wont be used, but...why not
	std::string name_;

	//	Size of the graph, i.e. number of vertices
	int size_;

	//	Size of robot link size
	int robot_size_;

	//	Interactive range
	//	i.e. Object within this range will be considered to interact with
	double interact_range_;

	//	Safety threshold
	double eps_;

	//	Vertices
	std::vector<VertexPtr> vertices_;

	//	Vertex map
	std::map<std::string, int> vertex_map_;

	//	Link names
	std::vector<std::string> link_names_;

	//	Initialisation flag
	bool initialised_;

	//	True if use dummy table obstacle
	bool dummy_table_;

	ros::NodeHandle nh_;
	std::vector<ros::Publisher> pubs_;
	std::vector<visualization_msgs::Marker> edges_;

	bool first_time_;
	Eigen::MatrixXd old_dist_;
	Eigen::MatrixXd dist_;
	Eigen::MatrixXd vel_;
};

//	Graph smart pointer
typedef boost::shared_ptr<MeshGraph> MeshGraphPtr;
}	//namespace exotica

#endif /* MESHGRAPH_H_ */

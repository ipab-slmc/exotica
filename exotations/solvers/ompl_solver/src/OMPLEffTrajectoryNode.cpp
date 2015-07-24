/*
 * OMPLEffTrajectoryNode.cpp
 *
 *  Created on: 18 Jun 2015
 *      Author: yiming
 */

#include <visualization_msgs/Marker.h>
#include <exotica/EXOTica.hpp>
class EffPublisher
{
	public:

		EffPublisher(const std::vector<std::string> & links) :
				nh_("/OMPLEffTrajectory"), links_(links)
		{

			model_ = robot_model_loader::RobotModelLoader("robot_description").getModel();
			state_.reset(new robot_state::RobotState(model_));
			ok_ = true;
			for (int i = 0; i < links.size(); i++)
				if (!state_->getLinkModel(links[i]))
				{
					WARNING_NAMED("OMPLEffTrajectory", "End-effector "<<links[i]<<" does not exist.")
					ok_ = false;
					break;
				}

			if (ok_)
			{
				sub_ =
						nh_.subscribe<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, boost::bind(&EffPublisher::pathCallback, this, _1));
				HIGHLIGHT_NAMED("OMPLEffTrajectory", "Subscribing to path topic '/move_group/display_planned_path'");
				pubs_.resize(links.size());
				markers_.resize(links.size());
				for (int i = 0; i < links.size(); i++)
				{
					pubs_[i] = nh_.advertise<visualization_msgs::Marker>(links_[i], 1);
					HIGHLIGHT_NAMED("OMPLEffTrajectory", "Publishing trajectory of "<<links_[i]<<" to topic '/OMPLEffTrajectory/"<<links_[i]<<"'.");
					markers_[i].type = visualization_msgs::Marker::LINE_STRIP;
					markers_[i].scale.x = 0.01;
					markers_[i].color.r = 1 - (double) ((1.0 * i) / links.size());
					markers_[i].color.g = 0 + (double) ((1.0 * i) / links.size());
					markers_[i].color.a = 1;
					markers_[i].id = i;
				}
			}

			traj_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("ShiftedPath", 1);
		}

		~EffPublisher()
		{

		}
	private:
		void pathCallback(const moveit_msgs::DisplayTrajectoryConstPtr & path)
		{
			if (ok_)
			{
				INFO_NAMED("OMPLEffTrajectory", "Publishing new end-effector trajectories");
				for (int l = 0; l < links_.size(); l++)
				{
					markers_[l].points.clear();
					markers_[l].header.frame_id = "r_foot";
					markers_[l].header.stamp = ros::Time::now();
					int size = path->trajectory[0].joint_trajectory.points.size();

					for (int i = 0; i < size; i++)
					{
						for (int j = 0; j < path->trajectory[0].joint_trajectory.joint_names.size();
								j++)
							state_->setVariablePosition(path->trajectory[0].joint_trajectory.joint_names[j], path->trajectory[0].joint_trajectory.points[i].positions[j]);
						state_->update(true);
						Eigen::Vector3d pose =
								state_->getGlobalLinkTransform(links_[l]).translation();
						geometry_msgs::Point tmp;
						tmp.x = pose(0);
						tmp.y = pose(1);
						tmp.z = pose(2);
						markers_[l].points.push_back(tmp);
					}
					pubs_[l].publish(markers_[l]);
				}
				shift_traj_ = *path.get();

				int size = shift_traj_.trajectory[0].joint_trajectory.points.size();
				shift_traj_.trajectory[0].multi_dof_joint_trajectory.points.resize(size);
				shift_traj_.trajectory[0].multi_dof_joint_trajectory.joint_names =
				{	"world_joint"};
				shift_traj_.trajectory[0].multi_dof_joint_trajectory.header.frame_id =
						"/world_frame";
				for (int i = 0; i < size; i++)
				{
					shift_traj_.trajectory[0].multi_dof_joint_trajectory.points[i].time_from_start =
							shift_traj_.trajectory[0].joint_trajectory.points[i].time_from_start;
					shift_traj_.trajectory[0].multi_dof_joint_trajectory.points[i].transforms.resize(1);
					shift_traj_.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].translation.y =
							0.1 * i;
				}
				traj_pub_.publish(shift_traj_);
			}
		}
		ros::NodeHandle nh_;
		std::vector<ros::Publisher> pubs_;
		ros::Publisher traj_pub_;
		moveit_msgs::DisplayTrajectory shift_traj_;
		ros::Subscriber sub_;
		// Using vector or markers rather than marker array, because one may want to visualise them separately
		std::vector<visualization_msgs::Marker> markers_;
		robot_model::RobotModelPtr model_;
		robot_state::RobotStatePtr state_;
		std::vector<std::string> links_;
		bool ok_;
};
int main(int argc, char **argv)
{
	if (argc < 2)
	{
		HIGHLIGHT_NAMED("OMPLEffTrajectory", "Please at least specify one end-effector link name");
		return 0;
	}
	std::vector<std::string> links(argc - 1);
	for (int i = 0; i < argc - 1; i++)
		links[i] = argv[i + 1];
	ros::init(argc, argv, "OMPLEffTrajectoryPublisher");
	EffPublisher effp(links);
	ros::AsyncSpinner sp(2);
	sp.start();
	ros::waitForShutdown();
	return 0;
}


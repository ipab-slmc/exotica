/*
 * WorldFrameNode.cpp
 *
 *  Created on: 23 Jun 2015
 *      Author: yiming
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
class WorldFrame
{
	public:
		WorldFrame()
		{

		}
		~WorldFrame()
		{

		}

	private:
		void poseCallback(const geometry_msgs::TransformStampedConstPtr & pose)
		{
			boost::mutex::scoped_lock(lock_);
		}
		ros::Timer timer_;
		geometry_msgs::TransformStamped tf_;
		boost::mutex lock_;
};


/*
 * trajectories_node.cpp
 *
 *  Created on: December 10, 2015
 *      Author: Karen Bodie
 *
 *       Starts trajectory node to generate trajectory commands.
 */

#include <ros/ros.h>
#include <trajectories/Trajectories.hpp>


int main(int argc, char** argv)
{
	// Initialize node.
	ros::init(argc, argv, "trajectory_node");
	ros::NodeHandle nh("~");

	// Read params
	int loopFreq;
	if(!nh.getParam("freq", loopFreq)){
	  ROS_ERROR("[Trajectories] Could not retrieve update frequency. Exiting.");
	  return 1;
	}

	ros::Rate loop_rate(loopFreq);


	// Create instance of the trajectory generator object.
	boost::shared_ptr<trajectories::Trajectories> trajectoryGenerator =
	    boost::make_shared<trajectories::Trajectories>(nh, loopFreq);

	while (ros::ok())
	{
		if (trajectoryGenerator->isGoReceived())
		{
			trajectoryGenerator->sendTrajectory();
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

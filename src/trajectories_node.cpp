/*
 * trajectories_node.cpp
 *
 *  Created on: December 10, 2015
 *      Author: Karen Bodie
 *
 *       Starts trajectory node to generate trajectory commands.
 */

#include <ros/ros.h>
#include "Trajectories.hpp"


int main(int argc, char** argv)
{
	// Initialize node.
	ros::init(argc, argv, "trajectory_node");
	ros::NodeHandle n;
	int loop;

	// Read params
	n.getParam("trajectories/freq", loop);

	ros::Rate loop_rate(loop);

	// Create instance of the trajectory generator object.
	trajectories::Trajectories *trajectoryGenerator = new trajectories::Trajectories(n);

	while (ros::ok())
	{
		if (trajectoryGenerator->isGoReceived())
		{
			trajectoryGenerator->sendTrajectory();
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

}

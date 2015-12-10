/*
 * trajectories.hpp
 *
 *  Created on: December 9, 2015
 *      Author: Karen Bodie
 *
 */

#pragma once

// ros core
#include <ros/ros.h>
#include <ros/package.h>
#include <roscpp_nodewrap/NodeImpl.h>
#include <roscpp_nodewrap/Nodelet.h>

// ros msgs
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/WrenchStamped.h>
#include <visualization_msgs/MarkerArray.h>

// ros srvs
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

// eigen
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <Eigen/Geometry>

// std library
#include <cstring>
#include <vector>

#include <sensor_msgs/JointState.h>
#include <anypulator_msgs/StateCartesian.h>

namespace trajectories {

/*
 * Trajectories class: generate a circle or line trajectory and send to command topic.
 */

class Trajectories {
 public:
	Trajectories(ros::NodeHandle n);
  virtual ~Trajectories();

  bool sendTrajectory();
  bool isGoReceived();

 private:

  bool readParameters();

  void initializeSubscribers();
  void initializePublishers();
  void initializeServices();

  bool goCircleCommand(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool goLineCommand(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool goLineStart(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool stopCommand(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool generateCircleTrajectory();
  bool generateLineTrajectory();
  bool generateLineStart();

  bool fifthOrderPolynomial(const double &t, const double &Tf, const double &d, double &dpos, double &vel, double &accel);


  //Publishers
  ros::Publisher CommandPublisher_;
  ros::Publisher TrajectoryPosePublisher_;
  ros::Publisher TrajectoryTwistPublisher_;

  // Services
  ros::ServiceServer goCircle_;
  ros::ServiceServer goLine_;
  ros::ServiceServer startLine_;
  ros::ServiceServer stop_;

  ros::NodeHandle n_;

  bool isGo_;
  double pubFreq_;

  Eigen::Vector3d circleCenter_, circleNormal_;
  double circleRadius_;

  Eigen::Vector3d lineStart_, lineEnd_;
  double maxVel_, maxAccel_;

  std::vector<anypulator_msgs::StateCartesian> trajectory_;
  int index_;
};

} /* namespace trajectories */

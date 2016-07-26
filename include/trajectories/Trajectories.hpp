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

// ros msgs
#include <tf/transform_listener.h>
#include <huskanypulator_msgs/EEstate.h>

// actions
#include <actionlib/server/simple_action_server.h>
#include <mbzirc_mission2_msgs/MoveEEAction.h>

// ros srvs
#include <std_srvs/Empty.h>

// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// std library
#include <string>
#include <vector>



namespace trajectories {

/*
 * Trajectories class: generate a circle or line trajectory and send to command topic.
 */

class Trajectories {
 public:
	Trajectories(ros::NodeHandle n, int loopFreq);
  virtual ~Trajectories();

  bool sendTrajectory();
  bool isGoReceived() const {return isGo_;}

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
  bool generateLineTrajectory(
      Eigen::Vector3d startPoint, Eigen::Vector3d endPoint,
      Eigen::Quaternion<double> startQ, Eigen::Quaternion<double> endQ,
      std::string frameID);
  bool generateLineTrajectory() {return generateLineTrajectory(lineStart_,lineEnd_,orientationQ_,orientationQ_,frameID_);}
  bool generateLineStart();

  bool fifthOrderPolynomial(const double &t, const double &Tf, const double &d, double &dpos, double &vel, double &accel);

  void genTrajActionPreemptCB();
  void genTrajActionGoalCB();

  //Publishers
  ros::Publisher CommandPublisher_;
  ros::Publisher TrajectoryPosePublisher_;
  ros::Publisher TrajectoryTwistPublisher_;

  // Services
  ros::ServiceServer goCircle_;
  ros::ServiceServer goLine_;
  ros::ServiceServer startLine_;
  ros::ServiceServer stop_;
  boost::shared_ptr<actionlib::SimpleActionServer<mbzirc_mission2_msgs::MoveEEAction>> genTrajActionServer_;

  // Subscriber
  boost::shared_ptr<tf::TransformListener> tfListener_;

  ros::NodeHandle n_;

  bool isGo_; //if true, trajectory points get published
  bool doLoop_; //if true, trajectory gets played in a loop
  double pubFreq_;

  Eigen::Vector3d circleCenter_, circleNormal_;
  double circleRadius_;

  Eigen::Vector3d lineStart_, lineEnd_;
  double maxVel_, maxAccel_;

  Eigen::Quaternion<double> orientationQ_;

  std::string frameID_;
  std::string eeFrameID_;

  std::vector<huskanypulator_msgs::EEstate> trajectory_;
  int index_;
};

} /* namespace trajectories */

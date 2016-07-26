/*
 * Trajectories.cpp
 *
 *  Created on: December 9, 2015
 *      Author: Karen Bodie
 *
 */

#include <trajectories/Trajectories.hpp>

#define _USE_MATH_DEFINES

namespace trajectories {

Trajectories::Trajectories(ros::NodeHandle n, int loopFreq) :
  n_(n),
  isGo_(false),
  doLoop_(false),
  index_(0),
  pubFreq_(loopFreq){

	readParameters();
	initializeSubscribers();
	initializePublishers();
	initializeServices();

	trajectory_.clear();
}

Trajectories::~Trajectories(){}

bool Trajectories::sendTrajectory(){
  //this method gets called at the specified loop rate if isGo_ is true.

  //publish trajectory point at current index
  trajectory_.at(index_).header.stamp = ros::Time::now();//update stamp to allow plotting
	CommandPublisher_.publish(trajectory_.at(index_));

	if(index_+1 < trajectory_.size()){
	  index_++; //increment index as long as trajectory has further points
	}else if(doLoop_){
	  index_ = 0; //if end of trajectory, reset to zero if loop requested
	}else if(genTrajActionServer_->isActive()){
	  genTrajActionServer_->setSucceeded(); //if end of trajectory, arrived at goal
	}//otherwise do nothing, i.e., republish the last trajectory point again next time

	return true;
}

void Trajectories::initializeSubscribers() {
  ROS_INFO("[Trajectories::initializeSubscribers] initializing subscribers...");
  tfListener_ = boost::make_shared<tf::TransformListener>();
}

void Trajectories::initializePublishers() {
	ROS_INFO("[Trajectories::initializePublishers] initializing publishers...");
	CommandPublisher_ = n_.advertise<huskanypulator_msgs::EEstate>("/state_command", 100);
	TrajectoryPosePublisher_ = n_.advertise<geometry_msgs::PoseStamped>("/trajectory_position", 100);
	TrajectoryTwistPublisher_ = n_.advertise<geometry_msgs::TwistStamped>("/trajectory_velocity", 100);
}

void Trajectories::initializeServices() {
	ROS_INFO("[Trajectories::initializeServices] initializing services...");
	goCircle_ = n_.advertiseService("/trajectories/circle/go", &Trajectories::goCircleCommand, this);
	goLine_ = n_.advertiseService("/trajectories/line/go", &Trajectories::goLineCommand, this);
	startLine_ = n_.advertiseService("/trajectories/line/start", &Trajectories::goLineStart, this);
	stop_ = n_.advertiseService("/trajectories/stop", &Trajectories::stopCommand, this);

	genTrajActionServer_ = boost::make_shared<actionlib::SimpleActionServer<mbzirc_mission2_msgs::MoveEEAction>>(
	    n_,"genEETrajectory",false);
	genTrajActionServer_->registerGoalCallback(boost::bind(&Trajectories::genTrajActionGoalCB, this));
	genTrajActionServer_->registerPreemptCallback(boost::bind(&Trajectories::genTrajActionPreemptCB, this));
	genTrajActionServer_->start();
}

bool Trajectories::readParameters() {
	ROS_INFO("[Trajectories::readParameters] reading parameters...");

	/*
	 * Frame IDs for tf
	 */
	n_.param<std::string>("trajectories/frame_ID", frameID_, "map");
	n_.param<std::string>("trajectories/EE_frame_ID", eeFrameID_, "HAND");
	ROS_INFO_STREAM("[Trajectories::readParameters] frameID = " << frameID_);
  ROS_INFO_STREAM("[Trajectories::readParameters] eeFrameID_ = " << eeFrameID_);

	/*
	 * Circle Parameters
	 */
	circleCenter_ = Eigen::Vector3d::Zero();
	circleNormal_ = Eigen::Vector3d::Zero();
	n_.param<double>("trajectories/circle/center/x", circleCenter_(0), 0.3);
	n_.param<double>("trajectories/circle/center/y", circleCenter_(1), 0.0);
	n_.param<double>("trajectories/circle/center/z", circleCenter_(2), 0.3);
	n_.param<double>("trajectories/circle/radius", circleRadius_, 0.2);
	n_.param<double>("trajectories/circle/normal_vector/x", circleNormal_(0), 0.0);
	n_.param<double>("trajectories/circle/normal_vector/y", circleNormal_(1), 0.1);
	n_.param<double>("trajectories/circle/normal_vector/z", circleNormal_(2), 0.0);

	/*
	 * Line Parameters
	 */
	lineStart_ = Eigen::Vector3d::Zero();
	lineEnd_ = Eigen::Vector3d::Zero();
	n_.param<double>("trajectories/line/start/x", lineStart_(0), 0.3);
	n_.param<double>("trajectories/line/start/y", lineStart_(1), -0.2);
	n_.param<double>("trajectories/line/start/z", lineStart_(2), 0.3);
	n_.param<double>("trajectories/line/end/x", lineEnd_(0), 0.3);
	n_.param<double>("trajectories/line/end/y", lineEnd_(1), 0.2);
	n_.param<double>("trajectories/line/end/z", lineEnd_(2), 0.3);

	/*
	 * Velocity and Acceleration
	 */
	n_.param<double>("trajectories/max_velocity", maxVel_, 0.2);
	n_.param<double>("trajectories/max_acceleration", maxAccel_, 0.2);
	ROS_INFO_STREAM("[Trajectories::readParameters] Max velocity = " << maxVel_);
	ROS_INFO_STREAM("[Trajectories::readParameters] Max acceleration = " << maxAccel_);

	/*
	 * Orientation
	 */
	n_.param<double>("trajectories/orientation/w", orientationQ_.w(), 1.0);
	n_.param<double>("trajectories/orientation/x", orientationQ_.x(), 0.0);
	n_.param<double>("trajectories/orientation/y", orientationQ_.y(), 0.0);
	n_.param<double>("trajectories/orientation/z", orientationQ_.z(), 0.0);
	ROS_INFO_STREAM("[Trajectories::readParameters] Orientation = " << orientationQ_.vec().transpose());

	return true;
}

bool Trajectories::goCircleCommand(std_srvs::Empty::Request &req,
										               std_srvs::Empty::Response &res){
  if(genTrajActionServer_->isActive()){
    ROS_WARN("[Trajectories::goCircleCommand] Action in progress. Not accepting requests.");
    return true;
  }

	ROS_INFO("[Trajectories::goCircleCommand] Circle Trajectory Activated");
	generateCircleTrajectory();
	isGo_ = true;
	doLoop_ = true;

  return true;
}

bool Trajectories::goLineCommand(std_srvs::Empty::Request &req,
										             std_srvs::Empty::Response &res){
  if(genTrajActionServer_->isActive()){
    ROS_WARN("[Trajectories::goLineCommand] Action in progress. Not accepting requests.");
    return true;
  }
	ROS_INFO("[Trajectories::goLineCommand] Line Trajectory Activated");
	doLoop_ = true;
	generateLineTrajectory();
	isGo_ = true;

  return true;
}

bool Trajectories::goLineStart(std_srvs::Empty::Request &req,
										           std_srvs::Empty::Response &res){
  if(genTrajActionServer_->isActive()){
    ROS_WARN("[Trajectories::goLineStart] Action in progress. Not accepting requests.");
    return true;
  }
	ROS_INFO("[Trajectories::goLineStart] Line Start Activated");
	doLoop_ = false;
	generateLineStart();
	isGo_ = true;

  return true;
}

bool Trajectories::stopCommand(std_srvs::Empty::Request &req,
										 std_srvs::Empty::Response &res){
	ROS_INFO("[Trajectories::stopCommand] Stop Command Received");
	isGo_ = false;
	doLoop_ = false;
	trajectory_.clear();

  return true;
}

bool Trajectories::generateCircleTrajectory(){
	trajectory_.clear();
	index_ = 0;
	ROS_INFO("[Trajectories::generateCircleTrajectory] Not yet implemented.");

	return true;
}

bool Trajectories::generateLineTrajectory(
    Eigen::Vector3d startPoint, Eigen::Vector3d endPoint,
    Eigen::Quaternion<double> startQ, Eigen::Quaternion<double> endQ,
    std::string frameID){

	ROS_INFO("[Trajectories::generateLineTrajectory] Generating a line trajectory");

	/*
	 * Reset trajectory
	 */
	trajectory_.clear();
	index_ = 0;

	double dt = 1.0 / pubFreq_;
	double distance = (endPoint - startPoint).norm();
	double t = 0.0;
	Eigen::Vector3d unitDirection = (endPoint - startPoint).normalized();
	Eigen::Vector3d pos = startPoint;
	Eigen::Vector3d vel = Eigen::Vector3d::Zero();
	Eigen::Vector3d acc = Eigen::Vector3d::Zero();
	Eigen::Quaternion<double> rot = startQ;

	/*
	 * 5th order polynomial trajectory
	 */

	// max velocity time limit
	double T_vel = distance / maxVel_ * (15.0 / 8.0);

	// max acceleration limit
	double t_T = (6.0 - sqrt(12.0)) / 12.0;
	double T_acc = sqrt((distance * 60.0 * t_T / maxAccel_)*(1.0 - 3.0 * t_T + 2.0 * pow(t_T,2)));

	double tf;
	if (T_vel > T_acc){
		ROS_INFO("[Trajectories::generateLineTrajectory] limited by velocity");
		tf = T_vel;
	}else{
		ROS_INFO("[Trajectories::generateLineTrajectory] limited by acceleration");
		tf = T_acc;
	}
	double dposMagn, velMagn, accMagn;

	while(t < tf){

		fifthOrderPolynomial(t, tf, distance, dposMagn, velMagn, accMagn);

		pos = startPoint + unitDirection * dposMagn;
		vel = unitDirection * velMagn;
		acc = unitDirection * accMagn;
		rot = startQ.slerp(dposMagn, endQ);

		huskanypulator_msgs::EEstate state;

		state.header.frame_id = frameID;
		state.use_pose = true;
		state.use_twist = true;
		state.use_accel = false;
		state.use_wrench = false;
		state.mission_mode = huskanypulator_msgs::EEstate::MISSION_MODE_WRENCHAPPROACH;

		state.pose.position.x = pos(0);
		state.pose.position.y = pos(1);
		state.pose.position.z = pos(2);
		state.pose.orientation.w = rot.w();
		state.pose.orientation.x = rot.x();
		state.pose.orientation.y = rot.y();
		state.pose.orientation.z = rot.z();

		state.twist.linear.x = vel(0);
		state.twist.linear.y = vel(1);
		state.twist.linear.z = vel(2);
		state.twist.angular.x = 0.0;
		state.twist.angular.y = 0.0;
		state.twist.angular.z = 0.0;

		state.accel.linear.x = acc(0);
		state.accel.linear.y = acc(1);
		state.accel.linear.z = acc(2);
		state.accel.angular.x = 0.0;
		state.accel.angular.y = 0.0;
		state.accel.angular.z = 0.0;

		state.wrench.force.x = 0.0;
		state.wrench.force.y = 0.0;
		state.wrench.force.z = 0.0;
		state.wrench.torque.x = 0.0;
		state.wrench.torque.y = 0.0;
		state.wrench.torque.z = 0.0;

		trajectory_.push_back(state);

		t += dt;
	}

	if(doLoop_){
    /*
     * Add reverse trajectory to start point
     */
    int size = trajectory_.size();
    for (int i = 0; i < size; i++){
      huskanypulator_msgs::EEstate state;
      state = trajectory_.at(size - i - 1);
      //revert speed and acceleration
      state.twist.linear.x  *= (-1.0);
      state.twist.linear.y  *= (-1.0);
      state.twist.linear.z  *= (-1.0);
      state.twist.angular.x *= (-1.0);
      state.twist.angular.y *= (-1.0);
      state.twist.angular.z *= (-1.0);
      state.accel.linear.x  *= (-1.0);
      state.accel.linear.y  *= (-1.0);
      state.accel.linear.z  *= (-1.0);
      state.accel.angular.x *= (-1.0);
      state.accel.angular.y *= (-1.0);
      state.accel.angular.z *= (-1.0);

      trajectory_.push_back(state);
    }
	}

	return true;
}

bool Trajectories::fifthOrderPolynomial(const double &t, const double &Tf, const double &d, double &dpos, double &vel, double &accel){
		double a3 = 10.0 * d / pow(Tf,3);
		double a4 = -15.0 * d / pow(Tf,4);
		double a5 = 6 * d / pow(Tf,5);

		dpos = a3 * pow(t,3) + a4 * pow(t,4) + a5 * pow(t,5);
		vel = 3.0 * a3 * pow(t,2) + 4.0 * a4 * pow(t,3) + 5.0 * a5 * pow(t,4);
		accel = 6.0 * a3 * t + 12.0 * a4 * pow(t,2) + 20.0 * a5 * pow(t,3);

	return true;
}

bool Trajectories::generateLineStart(){
	ROS_INFO("[Trajectories::generateLineStart] Going to line start position");

	trajectory_.clear();
	index_ = 0;

	huskanypulator_msgs::EEstate state;

	state.header.frame_id = frameID_;
  state.use_pose = true;
  state.use_twist = false;
  state.use_accel = false;
  state.use_wrench = false;
  state.mission_mode = huskanypulator_msgs::EEstate::MISSION_MODE_WRENCHAPPROACH;

  state.pose.position.x = lineStart_(0);
  state.pose.position.y = lineStart_(1);
  state.pose.position.z = lineStart_(2);
  state.pose.orientation.w = orientationQ_.w();
  state.pose.orientation.x = orientationQ_.x();
  state.pose.orientation.y = orientationQ_.y();
  state.pose.orientation.z = orientationQ_.z();

  state.twist.linear.x = 0.0;
  state.twist.linear.y = 0.0;
  state.twist.linear.z = 0.0;
  state.twist.angular.x = 0.0;
  state.twist.angular.y = 0.0;
  state.twist.angular.z = 0.0;

  state.accel.linear.x = 0.0;
  state.accel.linear.y = 0.0;
  state.accel.linear.z = 0.0;
  state.accel.angular.x = 0.0;
  state.accel.angular.y = 0.0;
  state.accel.angular.z = 0.0;

  state.wrench.force.x = 0.0;
  state.wrench.force.y = 0.0;
  state.wrench.force.z = 0.0;
  state.wrench.torque.x = 0.0;
  state.wrench.torque.y = 0.0;
  state.wrench.torque.z = 0.0;

	trajectory_.push_back(state);

	return true;
}

void Trajectories::genTrajActionGoalCB(){
  huskanypulator_msgs::EEstate goal = genTrajActionServer_->acceptNewGoal()->goalPose;

  std::string cmdFrameID = goal.header.frame_id;

  //find start pose (current EE pose)
  tf::StampedTransform T_cmdFrame_ee_start;
  try {
    tfListener_->lookupTransform(cmdFrameID, eeFrameID_, ros::Time(0), T_cmdFrame_ee_start);
  }catch (tf::TransformException& ex) {
    ROS_WARN("[Trajectories::genTrajActionGoalCB] Unable to get the requested transform. %s", ex.what());
    genTrajActionServer_->setAborted();
    return;
  }

  Eigen::Vector3d startPoint(T_cmdFrame_ee_start.getOrigin().x(),
                             T_cmdFrame_ee_start.getOrigin().y(),
                             T_cmdFrame_ee_start.getOrigin().z());
  Eigen::Quaternion<double> startQ(T_cmdFrame_ee_start.getRotation().w(),
                                   T_cmdFrame_ee_start.getRotation().x(),
                                   T_cmdFrame_ee_start.getRotation().y(),
                                   T_cmdFrame_ee_start.getRotation().z());
  Eigen::Vector3d endPoint(goal.pose.position.x,
                           goal.pose.position.y,
                           goal.pose.position.z);
  Eigen::Quaternion<double> endQ(goal.pose.orientation.w,
                                 goal.pose.orientation.x,
                                 goal.pose.orientation.y,
                                 goal.pose.orientation.z);


  //generate trajectory
  doLoop_ = false;
  generateLineTrajectory(startPoint, endPoint, startQ, endQ, cmdFrameID);

  isGo_ = true;
}

void Trajectories::genTrajActionPreemptCB(){
  isGo_ = false;
  trajectory_.clear();
  genTrajActionServer_->setPreempted();
}

} /* namespace trajectories */

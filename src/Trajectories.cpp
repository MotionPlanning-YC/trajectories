/*
 * Trajectories.cpp
 *
 *  Created on: December 9, 2015
 *      Author: Karen Bodie
 *
 */

#include <Trajectories.hpp>

#define _USE_MATH_DEFINES

using namespace std;
using namespace ros;
using namespace tf;
using namespace Eigen;

namespace trajectories {

Trajectories::Trajectories(ros::NodeHandle n) {
	n_ = n;

	readParameters();
	initializePublishers();
	initializeServices();

	isGo_ = false;
	trajectory_.clear();
	index_ = 0;
}

Trajectories::~Trajectories(){}

bool Trajectories::isGoReceived(){
	return isGo_;
}

bool Trajectories::sendTrajectory(){
	CommandPublisher_.publish(trajectory_.at(index_));
	index_ ++;
	if (index_ >= trajectory_.size())
		index_ = 0;
	return true;
}

void Trajectories::initializePublishers() {
	ROS_INFO("[Trajectories::initializePublishers] initializing publishers...");
//	CommandPublisher_ = advertise<anypulator_msgs::StateCartesian>("robot_command", "/state_command", 100);
//	TrajectoryPosePublisher_ = advertise<geometry_msgs::PoseStamped>("trajectory_position", "/trajectory_position", 100);
//	TrajectoryTwistPublisher_ = advertise<geometry_msgs::TwistStamped>("trajectory_velocity", "/trajectory_velocity", 100);

	CommandPublisher_ = n_.advertise<anypulator_msgs::StateCartesian>("/state_command", 100);
	TrajectoryPosePublisher_ = n_.advertise<geometry_msgs::PoseStamped>("/trajectory_position", 100);
	TrajectoryTwistPublisher_ = n_.advertise<geometry_msgs::TwistStamped>("/trajectory_velocity", 100);
}

void Trajectories::initializeServices() {
	ROS_INFO("[Trajectories::initializeServices] initializing services...");
	goCircle_ = n_.advertiseService("/trajectories/circle/go", &Trajectories::goCircleCommand, this);
	goLine_ = n_.advertiseService("/trajectories/line/go", &Trajectories::goLineCommand, this);
	startLine_ = n_.advertiseService("/trajectories/line/start", &Trajectories::goLineStart, this);
	stop_ = n_.advertiseService("/trajectories/stop", &Trajectories::stopCommand, this);
}

bool Trajectories::readParameters() {
	ROS_INFO("[Trajectories::readParameters] reading parameters...");
	n_.param<double>("trajectories/freq", pubFreq_, 200);

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

	return true;
}

bool Trajectories::goCircleCommand(std_srvs::Empty::Request &req,
										 std_srvs::Empty::Response &res)
{
	ROS_INFO("[Trajectories::goCircleCommand] Circle Trajectory Activated");
	generateCircleTrajectory();
	isGo_ = true;

  return true;
}

bool Trajectories::goLineCommand(std_srvs::Empty::Request &req,
										 std_srvs::Empty::Response &res)
{
	ROS_INFO("[Trajectories::goLineCommand] Line Trajectory Activated");
	generateLineTrajectory();
	isGo_ = true;

  return true;
}

bool Trajectories::goLineStart(std_srvs::Empty::Request &req,
										 std_srvs::Empty::Response &res)
{
	ROS_INFO("[Trajectories::goLineStart] Line Start Activated");
	generateLineStart();
	isGo_ = true;

  return true;
}

bool Trajectories::stopCommand(std_srvs::Empty::Request &req,
										 std_srvs::Empty::Response &res)
{
	ROS_INFO("[Trajectories::stopCommand] Stop Command Received");
	isGo_ = false;
	trajectory_.clear();

  return true;
}

bool Trajectories::generateCircleTrajectory(){
	trajectory_.clear();
	index_ = 0;

	return true;
}

bool Trajectories::generateLineTrajectory(){
	ROS_INFO("[Trajectories::generateLineTrajectory] Generating a line trajectory");

	/*
	 * Reset trajectory
	 */
	trajectory_.clear();
	index_ = 0;

	double dt = 1.0 / pubFreq_;
	double distance = (lineEnd_ - lineStart_).norm();
	double t = 0.0;
	Eigen::Vector3d unitDirection = (lineEnd_ - lineStart_)/(lineEnd_ - lineStart_).norm();
	Eigen::Vector3d pos = lineStart_;
	Eigen::Vector3d vel = Eigen::Vector3d::Zero();
	Eigen::Vector3d acc = Eigen::Vector3d::Zero();

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

		pos = lineStart_ + unitDirection * dposMagn;
		vel = unitDirection * velMagn;
		acc = unitDirection * accMagn;

		anypulator_msgs::StateCartesian state;
		state.position.x = pos(0);
		state.position.y = pos(1);
		state.position.z = pos(2);

		state.velocity.x = vel(0);
		state.velocity.y = vel(1);
		state.velocity.z = vel(2);

		state.acceleration.x = acc(0);
		state.acceleration.y = acc(1);
		state.acceleration.z = acc(2);

		trajectory_.push_back(state);

		t += dt;
	}

	/*
	 * Add reverse trajectory to start point
	 */
	int size = trajectory_.size();
	for (int i = 0; i < size; i++){
		anypulator_msgs::StateCartesian state;
		state.position.x = trajectory_.at(size - i - 1).position.x;
		state.position.y = trajectory_.at(size - i - 1).position.y;
		state.position.z = trajectory_.at(size - i - 1).position.z;

		state.velocity.x = - trajectory_.at(size - i - 1).velocity.x;
		state.velocity.y = - trajectory_.at(size - i - 1).velocity.y;
		state.velocity.z = - trajectory_.at(size - i - 1).velocity.z;

		state.acceleration.x = - trajectory_.at(size - i - 1).acceleration.x;
		state.acceleration.y = - trajectory_.at(size - i - 1).acceleration.y;
		state.acceleration.z = - trajectory_.at(size - i - 1).acceleration.z;

		trajectory_.push_back(state);
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

	anypulator_msgs::StateCartesian state;
	state.position.x = lineStart_(0);
	state.position.y = lineStart_(1);
	state.position.z = lineStart_(2);

	state.velocity.x = 0.0;
	state.velocity.y = 0.0;
	state.velocity.z = 0.0;

	state.acceleration.x = 0.0;
	state.acceleration.y = 0.0;
	state.acceleration.z = 0.0;

	trajectory_.push_back(state);

	return true;
}

} /* namespace trajectories */

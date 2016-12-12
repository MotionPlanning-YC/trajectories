/*
 * Trajectories.cpp
 *
 *  Created on: December 9, 2015
 *      Author: Karen Bodie
 *
 */

#include <trajectories/Trajectories.hpp>
#include <tf_conversions/tf_eigen.h>

#define _USE_MATH_DEFINES

namespace trajectories {

Trajectories::Trajectories(const ros::NodeHandle& n, const int loopFreq) :
  n_(n),
  isGo_(false),
  doLoop_(false),
  index_(0),
  pubFreq_(loopFreq),
  missionMode_(huskanypulator_msgs::EEstate::MISSION_MODE_FREEMOTION),
  joy_stepLength(0.0),
  joy_velMin(0.0){

  init();
  initializeSubscribers();
  initializePublishers();
  initializeServices();
}

Trajectories::~Trajectories(){}

void Trajectories::init(){
  isGo_ = false;
  doLoop_ = false;
  index_ = 0;
  missionMode_ = huskanypulator_msgs::EEstate::MISSION_MODE_FREEMOTION;
  trajectory_.clear();
  target_msg_prev_.header.stamp = ros::Time(0.0);

  readParameters();

  ROS_INFO("[Trajectories] Init done.");
}


bool Trajectories::sendTrajectory(){
  //this method gets called at the specified loop rate if isGo_ is true.

  //publish trajectory point at current index
  trajectory_.at(index_).header.stamp = ros::Time::now();//update stamp to allow plotting
  publishTargetMsg(trajectory_.at(index_));

  if(index_+1 < trajectory_.size()){
    index_++; //increment index as long as trajectory has further points
  }else if(doLoop_){
    index_ = 0; //if end of trajectory, reset to zero if loop requested
  }else if(genTrajActionServer_->isActive()){
    prevFinalTrajPoint_ = trajectory_.at(index_);
    genTrajActionServer_->setSucceeded(); //if end of trajectory, arrived at goal
  }//otherwise do nothing, i.e., republish the last trajectory point again next time

  return true;
}

void Trajectories::initializeSubscribers() {
  ROS_INFO("[Trajectories::initializeSubscribers] initializing subscribers...");
  tfListener_.reset(new tf::TransformListener());
}

void Trajectories::initializePublishers() {
  ROS_INFO("[Trajectories::initializePublishers] initializing publishers...");
  CommandPublisher_ = n_.advertise<huskanypulator_msgs::EEstate>(commandPublisherTopic_, 10);
  ROS_INFO_STREAM("[Trajectories::initializePublishers] Publishing EE state commands on topic "
      << commandPublisherTopic_);
  TrajectoryPosePublisher_ = n_.advertise<geometry_msgs::PoseStamped>("/trajectory_position", 10);
  TrajectoryTwistPublisher_ = n_.advertise<geometry_msgs::TwistStamped>("/trajectory_velocity", 10);
}

void Trajectories::initializeServices() {
  ROS_INFO("[Trajectories::initializeServices] initializing services...");
//  goCircle_ = n_.advertiseService("circle/go", &Trajectories::goCircleCommand, this);
  goLine_ = n_.advertiseService("line/go", &Trajectories::goLineCommand, this);
  startLine_ = n_.advertiseService("line/start", &Trajectories::goLineStart, this);
  stop_ = n_.advertiseService("stop", &Trajectories::stopCommand, this);
  reset_ = n_.advertiseService("reset", &Trajectories::resetCommand, this);

  genTrajActionServer_ = boost::make_shared<actionlib::SimpleActionServer<mbzirc_mission2_msgs::MoveEEAction>>(
      n_,actionServerName_,false);
  genTrajActionServer_->registerGoalCallback(boost::bind(&Trajectories::genTrajActionGoalCB, this));
  genTrajActionServer_->registerPreemptCallback(boost::bind(&Trajectories::genTrajActionPreemptCB, this));
  genTrajActionServer_->start();
}

bool Trajectories::readParameters() {
  ROS_INFO("[Trajectories::readParameters] reading parameters...");

  /*
   * Frame IDs for tf
   */
  n_.param<std::string>("frame_ID", frameID_, "map");
  n_.param<std::string>("EE_frame_ID", eeFrameID_, "HAND");
  n_.param<std::string>("publish_frame_ID", publishFrameID_, "base_link");
  ROS_INFO_STREAM("[Trajectories::readParameters] frameID = " << frameID_);
  ROS_INFO_STREAM("[Trajectories::readParameters] eeFrameID_ = " << eeFrameID_);
  ROS_INFO_STREAM("[Trajectories::readParameters] publishFrameID_ = " << publishFrameID_);

  /*
   * Circle Parameters
   */
  circleCenter_ = Eigen::Vector3d::Zero();
  circleNormal_ = Eigen::Vector3d::Zero();
  n_.param<double>("circle/center/x", circleCenter_(0), 0.3);
  n_.param<double>("circle/center/y", circleCenter_(1), 0.0);
  n_.param<double>("circle/center/z", circleCenter_(2), 0.3);
  n_.param<double>("circle/radius", circleRadius_, 0.2);
  n_.param<double>("circle/normal_vector/x", circleNormal_(0), 0.0);
  n_.param<double>("circle/normal_vector/y", circleNormal_(1), 0.1);
  n_.param<double>("circle/normal_vector/z", circleNormal_(2), 0.0);

  /*
   * Line Parameters
   */
  lineStart_ = Eigen::Vector3d::Zero();
  lineEnd_ = Eigen::Vector3d::Zero();
  n_.param<double>("line/start/x", lineStart_(0), 0.3);
  n_.param<double>("line/start/y", lineStart_(1), -0.2);
  n_.param<double>("line/start/z", lineStart_(2), 0.3);
  n_.param<double>("line/end/x", lineEnd_(0), 0.3);
  n_.param<double>("line/end/y", lineEnd_(1), 0.2);
  n_.param<double>("line/end/z", lineEnd_(2), 0.3);

  /*
   * Velocity and Acceleration
   */
  n_.param<double>("max_velocity", maxVel_, 0.2);
  n_.param<double>("max_acceleration", maxAccel_, 0.2);
  ROS_INFO_STREAM("[Trajectories::readParameters] Max velocity = " << maxVel_);
  ROS_INFO_STREAM("[Trajectories::readParameters] Max acceleration = " << maxAccel_);

  /*
   * Orientation
   */
  n_.param<double>("orientation/w", orientationQ_.w(), 1.0);
  n_.param<double>("orientation/x", orientationQ_.x(), 0.0);
  n_.param<double>("orientation/y", orientationQ_.y(), 0.0);
  n_.param<double>("orientation/z", orientationQ_.z(), 0.0);
  ROS_INFO_STREAM("[Trajectories::readParameters] Orientation = " << orientationQ_.vec().transpose());

  /*
   * Topic names
   */
  n_.param<std::string>("commandPublisherTopic", commandPublisherTopic_, "/ee_state_command");
  n_.param<std::string>("actionServerName", actionServerName_, "/genEETrajectory");

  /*
   * msc
   */
  n_.param<double>("joystick/step_length_gain", joy_stepLength, 0.05);
  n_.param<double>("joystick/vel_min", joy_velMin, 0.0);

  return true;
}

bool Trajectories::goCircleCommand(std_srvs::Empty::Request &req,
                                   std_srvs::Empty::Response &res){
  if(genTrajActionServer_->isActive()){
    ROS_WARN("[Trajectories::goCircleCommand] Action in progress. Not accepting requests.");
    return true;
  }

  ROS_INFO("[Trajectories::goCircleCommand] Circle Trajectory Activated");
  doLoop_ = true;
  missionMode_ = huskanypulator_msgs::EEstate::MISSION_MODE_FREEMOTION;
  generateCircleTrajectory();
  isGo_ = true;

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
  missionMode_ = huskanypulator_msgs::EEstate::MISSION_MODE_FREEMOTION;
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
  missionMode_ = huskanypulator_msgs::EEstate::MISSION_MODE_FREEMOTION;
  generateLineStart();
  isGo_ = true;

  return true;
}

bool Trajectories::stopCommand(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &res){
  ROS_INFO("[Trajectories::stopCommand] Stop Command Received");
  isGo_ = false;
  doLoop_ = false;
  if(genTrajActionServer_->isActive()){
    genTrajActionServer_->setAborted();
  }
  trajectory_.clear();

  return true;
}

bool Trajectories::resetCommand(std_srvs::Empty::Request &req,
                                std_srvs::Empty::Response &res){
  if(genTrajActionServer_->isActive()){
    genTrajActionServer_->setAborted();
  }
  init();
  ROS_INFO("[Trajectories::resetCommand] Reset done.");
  return true;
}

bool Trajectories::generateCircleTrajectory(){
  trajectory_.clear();
  index_ = 0;
  ROS_INFO("[Trajectories::generateCircleTrajectory] Not yet implemented.");

  return true;
}

bool Trajectories::generateLineTrajectory(
    const Eigen::Vector3d& startPoint, const Eigen::Vector3d& endPoint,
    const Eigen::Quaternion<double>& startQ, const Eigen::Quaternion<double>& endQ,
    const std::string& frameID){

  ROS_INFO("[Trajectories::generateLineTrajectory] Generating a line trajectory");

  /*
   * Reset trajectory
   */
  trajectory_.clear();
  index_ = 0;

  const double dt = 1.0 / pubFreq_;
  const double distance = (endPoint - startPoint).norm();
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
    rot = startQ.slerp(dposMagn/distance, endQ);
    rot.normalize();

    huskanypulator_msgs::EEstate state;

    state.header.frame_id = frameID;
    state.use_pose = true;
    state.use_twist = true;
    state.use_accel = false;
    state.use_wrench = false;
    state.mission_mode = missionMode_;

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
     * Add reverse trajectory back to start point
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

  //find start pose (current EE pose)
  tf::StampedTransform T_cmdFrame_ee_start;
  ros::Time now = ros::Time::now();
  try {
    tfListener_->waitForTransform(frameID_, eeFrameID_, now, ros::Duration(0.2));
    tfListener_->lookupTransform(frameID_, eeFrameID_, now, T_cmdFrame_ee_start);
  }catch (tf::TransformException& ex) {
    ROS_WARN("[Trajectories::genTrajActionGoalCB] Unable to get the requested transform. %s", ex.what());
    return false;
  }

  const Eigen::Vector3d startPoint(T_cmdFrame_ee_start.getOrigin().x(),
                                   T_cmdFrame_ee_start.getOrigin().y(),
                                   T_cmdFrame_ee_start.getOrigin().z());
  const Eigen::Quaternion<double> startQ(T_cmdFrame_ee_start.getRotation().w(),
                                         T_cmdFrame_ee_start.getRotation().x(),
                                         T_cmdFrame_ee_start.getRotation().y(),
                                         T_cmdFrame_ee_start.getRotation().z());
  const Eigen::Vector3d endPoint(lineStart_(0),
                                 lineStart_(1),
                                 lineStart_(2));
  const Eigen::Quaternion<double> endQ(orientationQ_.w(),
                                       orientationQ_.x(),
                                       orientationQ_.y(),
                                       orientationQ_.z());


  //generate trajectory
  generateLineTrajectory(startPoint, endPoint, startQ, endQ, frameID_);
  return true;
}

void Trajectories::genTrajActionGoalCB(){
  ROS_INFO("[Trajectories::genTrajActionGoalCB] Received new goal.");
  if(genTrajActionServer_->isActive()){
    genTrajActionServer_->setAborted();
    ROS_INFO("[Trajectories::genTrajActionGoalCB] Old goal was not done.");
  }

  huskanypulator_msgs::EEstate goal = genTrajActionServer_->acceptNewGoal()->goalPose;

  //find start pose (current EE pose in publishFrame)
  tf::StampedTransform T_publishFrame_ee_start;
  ros::Time now = ros::Time::now();
  try {
    tfListener_->waitForTransform(publishFrameID_, eeFrameID_, now, ros::Duration(0.2));
    tfListener_->lookupTransform( publishFrameID_, eeFrameID_, now, T_publishFrame_ee_start);
  }catch (tf::TransformException& ex) {
    ROS_WARN("[Trajectories::genTrajActionGoalCB] Unable to get the requested transform. %s", ex.what());
    genTrajActionServer_->setAborted();
    return;
  }

  // starting points (current position according to TF)
  Eigen::Vector3d startPoint_in_pub(T_publishFrame_ee_start.getOrigin().x(),
                                    T_publishFrame_ee_start.getOrigin().y(),
                                    T_publishFrame_ee_start.getOrigin().z());
  Eigen::Quaternion<double> startQ_pub_ee(T_publishFrame_ee_start.getRotation().w(),
                                          T_publishFrame_ee_start.getRotation().x(),
                                          T_publishFrame_ee_start.getRotation().y(),
                                          T_publishFrame_ee_start.getRotation().z());

  //retrieve end point of previous trajectory and check how far it is from current pose
  const Eigen::Vector3d prevEndPoint_inPub(prevFinalTrajPoint_.pose.position.x,
                                           prevFinalTrajPoint_.pose.position.y,
                                           prevFinalTrajPoint_.pose.position.z);
  const Eigen::Quaternion<double> prevEndQ_pub_ee(prevFinalTrajPoint_.pose.orientation.w,
                                                  prevFinalTrajPoint_.pose.orientation.x,
                                                  prevFinalTrajPoint_.pose.orientation.y,
                                                  prevFinalTrajPoint_.pose.orientation.z);

  if((startPoint_in_pub-prevEndPoint_inPub).norm() < 0.05 and
      startQ_pub_ee.angularDistance(prevEndQ_pub_ee) < 0.1){
    startPoint_in_pub = prevEndPoint_inPub;
    startQ_pub_ee = prevEndQ_pub_ee;
    ROS_INFO("[Trajectories] using old goal as new starting point");
  }


  const Eigen::Vector3d endPoint_in_msg(goal.pose.position.x,
                                        goal.pose.position.y,
                                        goal.pose.position.z);
  Eigen::Quaternion<double> endQ_msg_ee(goal.pose.orientation.w,
                                        goal.pose.orientation.x,
                                        goal.pose.orientation.y,
                                        goal.pose.orientation.z);

  startQ_pub_ee.normalize();
  endQ_msg_ee.normalize();


  Eigen::Affine3d T_pub_msg;
  T_pub_msg.setIdentity(); //transform from msg header frame to publish frame

  if(goal.header.frame_id != publishFrameID_){
    tf::StampedTransform T_publishFrame_msgFrame;
    try {
      tfListener_->waitForTransform(publishFrameID_, goal.header.frame_id, now, ros::Duration(0.2));
      tfListener_->lookupTransform( publishFrameID_, goal.header.frame_id, now, T_publishFrame_msgFrame);
    }catch (tf::TransformException& ex) {
      ROS_WARN("[Trajectories::genTrajActionGoalCB] Unable to get the requested transform. %s", ex.what());
      genTrajActionServer_->setAborted();
      return;
    }

    tf::transformTFToEigen(T_publishFrame_msgFrame, T_pub_msg);
  }

  Eigen::Vector3d endPoint_in_pub = T_pub_msg * endPoint_in_msg;
  Eigen::Quaternion<double> endQ_pub_ee(T_pub_msg.rotation() * endQ_msg_ee);
  endQ_pub_ee.normalize();

  //generate trajectory
  doLoop_ = false;
  missionMode_ = goal.mission_mode;
  generateLineTrajectory(startPoint_in_pub, endPoint_in_pub, startQ_pub_ee, endQ_pub_ee, publishFrameID_);

  isGo_ = true;
}

void Trajectories::genTrajActionPreemptCB(){
  isGo_ = false;
  trajectory_.clear();
  genTrajActionServer_->setPreempted();
}

void Trajectories::publishTargetMsg(const huskanypulator_msgs::EEstate& msg) {
  CommandPublisher_.publish(msg);
  target_msg_prev_ = msg;
}

} /* namespace trajectories */

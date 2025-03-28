#include <iostream>
#include <CCM/ccmcontroller.h>
#include <std_msgs/Float64.h>
#include "mavros_msgs/AttitudeTarget.h"
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>

#include <CCM/ccmutils.h>
#include <CCM/Geodesic.h>
#include <CCM/Metric.h>

// Measured states
std::string current_mode;
Eigen::Vector4d mea_q;
Eigen::Matrix3d mea_R;
Eigen::Vector3d mea_wb;
Eigen::Vector3d mea_pos;
Eigen::Vector3d mea_vel;
Eigen::Vector3d vel_prev;
double vel_prev_t;
static Eigen::Matrix<double,3,3> Rz_T =
(Eigen::Matrix<double,3,3>() << 0.0, 1.0, 0.0,
                               -1.0, 0.0, 0.0,
                                0.0, 0.0, 1.0).finished();

// Thrust estimation MA
double FZ_EST_N;
double fz_est_raw;
double fz_est;
double fz_est_sum;

// Update variables
int pose_up;
int vel_up;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_mode = msg->mode;
}

void poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // By default, MAVROS gives local_position in ENU
  mea_pos(0) = msg->pose.position.y;
  mea_pos(1) = msg->pose.position.x;
  mea_pos(2) = -msg->pose.position.z;

  // ROS quaternion
  mea_q(0) = msg->pose.orientation.w;
  mea_q(1) = msg->pose.orientation.x;
  mea_q(2) = msg->pose.orientation.y;
  mea_q(3) = msg->pose.orientation.z;

  // Convert to Matrix
  ccmutils::quat2rotM(mea_q, mea_R);

  // Adjust by R_z
  mea_R = mea_R * Rz_T;

  // Extract ENU encoding of PX4 quat
  ccmutils::rotM2quat(mea_q, mea_R);

  // Convert to NED
  double q_w = mea_q(0);
  double q_x = mea_q(2);
  double q_y = mea_q(1);
  double q_z = -mea_q(3);

  mea_q << q_w, q_x, q_y, q_z;

  // Final conversion to rot matrix
  ccmutils::quat2rotM(mea_q, mea_R);

  pose_up = 1;

}

void velSubCB(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  // By default, MAVROS gives local_position/velocity in ENU

  vel_prev = mea_vel;

  double vel_dt = (msg->header.stamp.sec + msg->header.stamp.nsec*(1.0e-9))-
                   vel_prev_t;
  vel_prev_t = msg->header.stamp.sec + msg->header.stamp.nsec*(1.0e-9);

  mea_vel(0) = msg->twist.linear.y;
  mea_vel(1) = msg->twist.linear.x;
  mea_vel(2) = -msg->twist.linear.z;
  mea_wb(0) = msg->twist.angular.y;
  mea_wb(1) = msg->twist.angular.x;
  mea_wb(2) = -msg->twist.angular.z;

  //update accel estimate
  Eigen::Vector3d acc = (mea_vel - vel_prev)/vel_dt;
  acc(2) += -9.8066;
  fz_est_raw = -acc.dot(mea_R.col(2));

  // Moving average update
  fz_est_sum = fz_est_sum + fz_est_raw - (fz_est_sum/FZ_EST_N);
  fz_est = fz_est_sum/FZ_EST_N;

  vel_up = 1;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "asl_traj_ctrl");
  ros::NodeHandle nh;

  // Subscriptions
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
  ros::Subscriber poseSub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 2, poseSubCB);
  ros::Subscriber velSub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 2, velSubCB);

  pose_up = 0; vel_up = 0;

  // Actuator publisher
  ros::Publisher cmdPub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude",2);
  mavros_msgs::AttitudeTarget att_sp;


  // DEBUG PUBLICATIONS
  ros::Publisher debug_pub1 = nh.advertise<std_msgs::Float64>("/debug1", 1);
  ros::Publisher debug_pub2 = nh.advertise<std_msgs::Float64>("/debug2", 1);
  ros::Publisher debug_pub3 = nh.advertise<std_msgs::Float64>("/debug3", 1);
  ros::Publisher debug_pub4 = nh.advertise<std_msgs::Float64>("/debug4", 1);
  ros::Publisher debug_pub5 = nh.advertise<std_msgs::Float64>("/debug5", 1);
  ros::Publisher debug_pub6 = nh.advertise<std_msgs::Float64>("/debug6", 1);
  ros::Publisher debug_pub7 = nh.advertise<std_msgs::Float64>("/debug7", 1);
  ros::Publisher debug_pub8 = nh.advertise<std_msgs::Float64>("/debug8", 1);
  ros::Publisher debug_pub9 = nh.advertise<std_msgs::Float64>("/debug9", 1);
  ros::Publisher debug_pub10 = nh.advertise<std_msgs::Float64>("/debug10", 1);
  ros::Publisher debug_pub11 = nh.advertise<std_msgs::Float64>("/debug11", 1);
  ros::Publisher debug_pub12 = nh.advertise<std_msgs::Float64>("/debug12", 1);
  ros::Publisher debug_pub13 = nh.advertise<std_msgs::Float64>("/debug13", 1);
  ros::Publisher debug_pub14 = nh.advertise<std_msgs::Float64>("/debug14", 1);
  ros::Publisher debug_pub15 = nh.advertise<std_msgs::Float64>("/debug15", 1);
  ros::Publisher debug_pub16 = nh.advertise<std_msgs::Float64>("/debug16", 1);
  ros::Publisher debug_pub17 = nh.advertise<std_msgs::Float64>("/debug17", 1);
  ros::Publisher debug_pub18 = nh.advertise<std_msgs::Float64>("/debug18", 1);
  ros::Publisher debug_pub19 = nh.advertise<std_msgs::Float64>("/debug19", 1);
  std_msgs::Float64 debug_msg;

  // Define controller classes
  ros::param::get("~FZ_EST_N", FZ_EST_N);
  double FZ_CTRL_N = 1.0;
  ros::param::get("~FZ_CTRL_N", FZ_CTRL_N);
  CCMController ctrl(FZ_CTRL_N);


  // Define trajectory class
  std::string traj_type;
  ros::param::get("~TRAJ", traj_type);

  double start_delay;
  ros::param::get("~START_DELAY", start_delay);

  Trajectory* traj;
 // =================== Wybór wcześnej ustalonej trajektorii =================
  if (traj_type == "CIRCLE") {

    double circle_T;
    ros::param::get("~CIRCLE_T", circle_T);
    traj = new CircleTrajectory(1.0, 2.0*M_PI*(1.0/circle_T),start_delay);

  } else if (traj_type == "POLY") {

    double poly_scale;
    ros::param::get("~POLY_SCALE", poly_scale);
    traj = new PolyTrajectory(poly_scale, start_delay);

  } else if (traj_type == "FIG8") {

    double circle_T;
    double radius_x, radius_y = 1.0;
    ros::param::get("~CIRCLE_T", circle_T);
    ros::param::get("~RADIUS_X", radius_x);
    ros::param::get("~RADIUS_Y", radius_y);
    traj = new Fig8Trajectory(radius_x, radius_y, 2.0*M_PI*(1.0/circle_T), start_delay);

  } else  {

    traj = new HoverTrajectory();

  }

  // Takeoff params
  double TAKEOFF_HGT;
  ros::param::get("~TAKEOFF_HGT", TAKEOFF_HGT);

  double TAKEOFF_TIME;
  ros::param::get("~TAKEOFF_TIME", TAKEOFF_TIME);

  bool DO_TAKEOFF;
  ros::param::get("~DO_TAKEOFF", DO_TAKEOFF);

  // Controller frequency
  ros::Rate rate(250.0);

  // Tracking status variables
  bool traj_started = false;
  double time_prev = ros::Time::now().toSec();
  double time_traj = 0.0;
  double dt = 0.0;

  // Reference values
  double yaw_des = 0.0;
  double yaw_init = 0.0;
  ros::param::get("~YAW_INIT", yaw_init);

  double yaw_dot_des = 0.0;
  Eigen::Vector3d r_pos(0, 0, 0);
  Eigen::Vector3d r_vel(0, 0, 0);
  Eigen::Vector3d r_acc(0, 0, 0);
  Eigen::Vector3d r_jer(0, 0, 0);
  Eigen::Vector3d takeoff_loc(0, 0, 0);

  // Estimator values
  fz_est = 9.8066;
  fz_est_sum = fz_est * FZ_EST_N;
  Eigen::Vector3d euler;
  vel_prev_t = 0.0;
  mea_q.setZero();
  mea_R = Eigen::Matrix3d::Identity();
  mea_wb.setZero();
  mea_pos.setZero();
  mea_vel.setZero();
  vel_prev.setZero();

  // Command values
  Eigen::Vector3d ref_er(0,0,0);
  Eigen::Vector3d ref_om(0,0,0);
  double fz_cmd = 9.8066;

  while(ros::ok()) {
    // Check for state update
    ros::spinOnce();

    // Publish tracking results for debug
    debug_msg.data = r_pos(0);
    debug_pub1.publish(debug_msg);
    debug_msg.data = mea_pos(0);
    debug_pub2.publish(debug_msg);

    debug_msg.data = r_pos(1);
    debug_pub3.publish(debug_msg);
    debug_msg.data = mea_pos(1);
    debug_pub4.publish(debug_msg);

    debug_msg.data = r_pos(2);
    debug_pub5.publish(debug_msg);
    debug_msg.data = mea_pos(2);
    debug_pub6.publish(debug_msg);

    debug_msg.data = mea_vel(0);
    debug_pub7.publish(debug_msg);
    debug_msg.data = mea_vel(1);
    debug_pub8.publish(debug_msg);
    debug_msg.data = mea_vel(2);
    debug_pub9.publish(debug_msg);

    // Get commanded normalized thrust
    fz_cmd = ctrl.getfz();

    debug_msg.data = fz_cmd;
    debug_pub18.publish(debug_msg);
    debug_msg.data = fz_est;
    debug_pub19.publish(debug_msg);

    // Time loop calculations
    dt = ros::Time::now().toSec() - time_prev;
    time_prev = ros::Time::now().toSec();

    // ============================ dron startuje i miejsce startu to 0,0,0 ===========================
    if (current_mode == "OFFBOARD") {

      // update trajectory status variables
      if (!traj_started){

          traj_started = true;
          ctrl.setMode(traj_started);

          time_traj = 0.0;

          ROS_INFO("offboard started");

          // set takeoff location
          takeoff_loc << mea_pos(0), mea_pos(1), mea_pos(2);
          ROS_INFO("takeoff loc: (%.3f,%.3f,%.3f)",mea_pos(0),mea_pos(1),mea_pos(2));

          // initialize ref pos
          r_pos = takeoff_loc;

          // Give initial ref vel only if doing takeoff
          if (DO_TAKEOFF) {  r_vel << 0.0, 0.0, -(TAKEOFF_HGT/TAKEOFF_TIME);}

          // set start point for trajectory
          if (DO_TAKEOFF) {
            // if doing takeoff, start point is hover point above takeoff
            traj->set_start_pos(takeoff_loc + Eigen::Vector3d(0.0,0.0,-TAKEOFF_HGT));
          } else {
            // else start point is offboard init location
            traj->set_start_pos(takeoff_loc);
          }
      } else {
          time_traj += dt;
      }
    } else {
      //reset
      traj_started = false;
      time_traj = 0.0;
      ctrl.setMode(traj_started);
      r_vel.setZero(); r_acc.setZero(); r_jer.setZero();
    }

    // =================== Obliczna jest trajektoria względem punktu startowego -> dla zadanego czasu wyliczane są obecne wartości waypointa ======================
    // Compute nominal
    if (time_traj >= TAKEOFF_TIME) {
      // Once past takeoff time, start trajectory
      // ROS_INFO("error: %.3f", (r_pos-mea_pos).norm());
      traj->eval(time_traj-TAKEOFF_TIME+dt, r_pos, r_vel, r_acc, r_jer, yaw_des, yaw_dot_des);

      // add offset
      yaw_des += yaw_init;

    } else {
      if (DO_TAKEOFF) {
        // smooth takeoff
        r_pos(2) = takeoff_loc(2)-(time_traj/TAKEOFF_TIME)*TAKEOFF_HGT;

        yaw_des = yaw_init;

      }
    }
    // ======================== tutaj obliczenia kontrolera i publikowanie komend ===================
    // Update controller internal state
    ctrl.updateState(mea_pos, mea_R, mea_vel, mea_wb, fz_est, dt, pose_up, vel_up);

    // Compute feedback
    ctrl.calcCCM(yaw_des, yaw_dot_des, r_pos, r_vel, r_acc, r_jer);

    // get commands
    fz_cmd = ctrl.getfz();
    ref_er = ctrl.getEr();
    ref_om = ctrl.getOm();

    euler = ctrl.getEuler();

    att_sp.header.stamp = ros::Time::now();
    att_sp.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    att_sp.body_rate.x = ref_er(0);
    att_sp.body_rate.y = ref_er(1);
    att_sp.body_rate.z = ref_er(2);
    att_sp.thrust = std::min(1.0, std::max(0.0, 0.56 * (fz_cmd) / 9.8066));
    cmdPub.publish(att_sp);

    // Publish for debug
    debug_msg.data = euler(0);
    debug_pub10.publish(debug_msg);
    debug_msg.data = euler(1);
    debug_pub11.publish(debug_msg);
    debug_msg.data = euler(2);
    debug_pub12.publish(debug_msg);
    debug_msg.data = ctrl.getYawNom();
    debug_pub13.publish(debug_msg);

    debug_msg.data = ref_om(0);
    debug_pub14.publish(debug_msg);
    debug_msg.data = ref_om(1);
    debug_pub15.publish(debug_msg);
    debug_msg.data = ref_om(2);
    debug_pub16.publish(debug_msg);

    debug_msg.data = ctrl.getE();
    debug_pub17.publish(debug_msg);

    // Reset update
    pose_up = 0; vel_up = 0;
    // Check for state update
    ros::spinOnce();

	  rate.sleep();
  }

  return 0;
}

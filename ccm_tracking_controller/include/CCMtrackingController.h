/*
	FILE: CCMtrackingController.h
	-------------------------------
	function definition of px4 CCM controller
*/
#ifndef CCM_TRACKING_CONTROLLER_H
#define CCM_TRACKING_CONTROLLER_H
#include <ros/ros.h>
#include <Eigen/Dense>
#include <queue>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <iostream>
#include <std_msgs/Float64.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <ccm_tracking_controller/CCMTarget.h>
#include <CCM/ccmcontroller.h>
#include <CCM/ccmutils.h>
#include <CCM/Geodesic.h>
#include <CCM/Metric.h>

using std::cout; using std::endl;
namespace ccmtracking{
	class CCMtrackingController {
		private:
			ros::NodeHandle nh_;
			ros::Subscriber poseSub_; // Subscribe to pose
			ros::Subscriber velSub_; // Subscribe to velocity
			ros::Subscriber targetSub_; // subscriber for the tracking target states
			ros::Publisher cmdPub_; // command publisher
			ros::Publisher poseVisPub_; // current pose publisher
			ros::Publisher targetVisPub_; // target pose publisher
			ros::Publisher histTrajVisPub_; // history trajectory publisher
			ros::Publisher targetHistTrajVisPub_; // target trajectory publisher
			ros::Publisher velAndAccVisPub_; // velocity and acceleration visualization publisher
			ros::Timer cmdTimer_; // command timer
			ros::Timer visTimer_; // visualization timer

			// parameters
			double FZ_EST_N_;
			double FZ_CTRL_N_;
			bool verbose_;
			// CCM
			CCMController ctrl_;
			ccm_tracking_controller::CCMTarget target_;
			bool firstTargetReceived_ = false;
			bool targetReceived_ = false;
			bool poseReceived_ = false;
			bool velReceived_ = false;
			geometry_msgs::PoseStamped pose_;
			geometry_msgs::TwistStamped vel_;
			// Measured states
            Eigen::Vector4d mea_q_;
            Eigen::Matrix3d mea_R_;
            Eigen::Vector3d mea_wb_;
            Eigen::Vector3d mea_pos_;
            Eigen::Vector3d mea_vel_;
            Eigen::Vector3d vel_prev_;
			Eigen::Vector3d euler_;
            double vel_prev_t_;
            static Eigen::Matrix<double,3,3> Rz_T_;

            // Thrust estimation MA
            double fz_est_raw_;
            double fz_est_;
            double fz_est_sum_;

            // Update variables
            int pose_up_;
            int vel_up_;

			// visualization
			geometry_msgs::PoseStamped poseVis_;
			std::deque<geometry_msgs::PoseStamped> histTraj_;
			geometry_msgs::PoseStamped targetPoseVis_;
			std::deque<geometry_msgs::PoseStamped> targetHistTraj_;
			bool velFirstTime_ = true;
			Eigen::Vector3d prevVel_;
			ros::Time velPrevTime_;
		
		public:
			CCMtrackingController(const ros::NodeHandle& nh);
			void initParam();
			void registerPub();
			void registerCallback();

			// callback functions
			void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
			void velCB(const geometry_msgs::TwistStamped::ConstPtr& msg);
			void targetCB(const ccm_tracking_controller::TargetConstPtr& target);
			void cmdCB(const ros::TimerEvent&);
			void visCB(const ros::TimerEvent&);

			void publishCommand(const Eigen::Vector4d& cmd);

			// visualization
			void publishPoseVis();
			void publishHistTraj();
			void publishTargetVis();
			void publishTargetHistTraj();
			void publishVelAndAccVis();


	};
}

#endif
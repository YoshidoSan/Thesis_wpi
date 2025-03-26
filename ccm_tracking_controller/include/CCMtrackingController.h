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
#include <utils.h>
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
			double FZ_EST_N_; // rozmiar okna do estymacji siły ciągu (moving average)
			double FZ_CTRL_N_; // Stała czasowa filtru pochodnej ciągu
			bool verbose_;

			// CCM
			std::unique_ptr<CCMController> ctrl_;
			//CCMController ctrl_;

			//recevied data
			ccm_tracking_controller::CCMTarget target_;
			Eigen::Vector3d position_recevied_conv_;
			Eigen::Vector3d velocity_recevied_conv_;
			Eigen::Vector3d acceleration_recevied_conv_;
			Eigen::Vector3d jerk_recevied_conv;
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
			// matrix
            const Eigen::Matrix<double,3,3> Rz_T_= 
			(Eigen::Matrix<double,3,3>() 
			<< 0.0, 1.0, 0.0,
			-1.0, 0.0, 0.0,
			 0.0, 0.0, 1.0).finished();;
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
			void initParamModules();
			void registerPub();
			void registerCallback();

			// callback functions
			void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
			void velCB(const geometry_msgs::TwistStamped::ConstPtr& msg);
			void targetCB(const ccm_tracking_controller::CCMTargetConstPtr& target);
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
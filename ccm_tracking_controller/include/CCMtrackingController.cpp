/*
	FILE: CCMtrackingController.cpp
	-------------------------------
	function implementation of px4 tracking controller
*/

#include <CCMtrackingController.h>

namespace ccmtracking{
	CCMtrackingController::CCMtrackingController(const ros::NodeHandle& nh) : nh_(nh){
		this->initParamModules();
		//this->ctrl_ = CCMController(this->FZ_CTRL_N_);
		//this->ctrl_ = (new CCMController(this->FZ_CTRL_N_));
		this->registerPub();
		this->registerCallback();
	}


	void CCMtrackingController::initParamModules(){
		
		// Define controller classes
		if (not this->nh_.getParam("ccm_controller/FZ_EST_N", this->FZ_EST_N_)){
			this->FZ_EST_N_ = 1.0;
			cout << "[CCMtrackingController]: FZ_EST_N use default: 1.0." << endl;
		}
		else{
			cout << "[CCMtrackingController]: FZ_EST_N is set to: " << this->FZ_EST_N_  << endl;
		}

		if (not this->nh_.getParam("ccm_controller/FZ_CTRL_N", this->FZ_CTRL_N_)){
			this->FZ_CTRL_N_ = 1.0;
			cout << "[CCMtrackingController]: FZ_EST_N use default: 1.0." << endl;
		}
		else{
			cout << "[CCMtrackingController]: FZ_EST_N is set to: " << this->FZ_CTRL_N_  << endl;
		}
		//double N = this->FZ_CTRL_N_;
		//this->ctrl_ = (new CCMController (N));
		
		// ccmcontroller
		this->ctrl_ = std::make_unique<CCMController>(this->FZ_CTRL_N_);

		// Estimator values
		this->fz_est_ = 9.8066;
		this->fz_est_sum_ = this->fz_est_ * this->FZ_EST_N_;
		this->vel_prev_t_ = 0.0;
		this->mea_q_.setZero();
		this->mea_R_ = Eigen::Matrix3d::Identity();
		this->mea_wb_.setZero();
		this->mea_pos_.setZero();
		this->mea_vel_.setZero();
		this->vel_prev_.setZero();

		// Display message
		if (not this->nh_.getParam("ccm_controller/verbose", this->verbose_)){
			this->verbose_ = false;
			cout << "[CCMtrackingController]: No display message param. Use default: false." << endl;
		}
		else{
			cout << "[CCMtrackingController]: Display message is set to: " << this->verbose_  << endl;
		}
	}

	void CCMtrackingController::registerPub(){
		// command publisher
		this->cmdPub_ = this->nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 100);
		
		// current pose visualization publisher
		this->poseVisPub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/tracking_controller/robot_pose", 1);

		// trajectory history visualization publisher
		this->histTrajVisPub_ = this->nh_.advertise<nav_msgs::Path>("/tracking_controller/trajectory_history", 1);

		// target pose visualization publisher
		this->targetVisPub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/tracking_controller/target_pose", 1);
	
		// target trajectory history publisher
		this->targetHistTrajVisPub_ = this->nh_.advertise<nav_msgs::Path>("/tracking_controller/target_trajectory_history", 1); 

		// velocity and acceleration visualization publisher
		this->velAndAccVisPub_ = this->nh_.advertise<visualization_msgs::Marker>("/tracking_controller/vel_and_acc_info", 1);
	}

	void CCMtrackingController::registerCallback(){
		// pose subscriber
		this->poseSub_ = this->nh_.subscribe("mavros/local_position/pose", 1, &CCMtrackingController::poseCB, this);

		// vel subscriber
		this->velSub_ = this->nh_.subscribe("mavros/local_position/velocity", 1, &CCMtrackingController::velCB, this);

		// target setpoint subscriber
		this->targetSub_ = this->nh_.subscribe("/autonomous_flight/target_state", 1, &CCMtrackingController::targetCB, this);
	
		// controller publisher timer
		this->cmdTimer_ = this->nh_.createTimer(ros::Duration(0.01), &CCMtrackingController::cmdCB, this);

		// visualization timer
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.033), &CCMtrackingController::visCB, this);
	}


	void CCMtrackingController::poseCB(const geometry_msgs::PoseStamped::ConstPtr& pose) {
		this->pose_ = *pose;
		this->poseReceived_ = true;

		// By default, MAVROS gives local_position in ENU
		this->mea_pos_(0) = pose->pose.position.y;
		this->mea_pos_(1) = pose->pose.position.x;
		this->mea_pos_(2) = -pose->pose.position.z;

		// ROS quaternion
		this->mea_q_(0) = pose->pose.orientation.w;
		this->mea_q_(1) = pose->pose.orientation.x;
		this->mea_q_(2) = pose->pose.orientation.y;
		this->mea_q_(3) = pose->pose.orientation.z;
	  
		// Convert to Matrix
		ccmutils::quat2rotM(this->mea_q_, this->mea_R_);
	  
		// Adjust by R_z
		this->mea_R_ = this->mea_R_ * this->Rz_T_;
	  
		// Extract ENU encoding of PX4 quat
		ccmutils::rotM2quat(this->mea_q_, this->mea_R_);
	  
		// Convert to NED
		double temp_q_w = this->mea_q_(0);
		double temp_q_x = this->mea_q_(2);
		double temp_q_y = this->mea_q_(1);
		double temp_q_z = (-1)*this->mea_q_(3);
	  
		this->mea_q_ << temp_q_w, temp_q_x, temp_q_y, temp_q_z;
	  
		// Final conversion to rot matrix
		ccmutils::quat2rotM(this->mea_q_, this->mea_R_);
	  
		this->pose_up_ = 1;
	  }
	  
	void CCMtrackingController::velCB(const geometry_msgs::TwistStamped::ConstPtr& vel) {
		this->vel_ = *vel;
		this->velReceived_ = true;
		// By default, MAVROS gives local_position/velocity in ENU
		
		this->vel_prev_ = this->mea_vel_;
		
		double vel_dt = (vel->header.stamp.sec + vel->header.stamp.nsec*(1.0e-9))- this->vel_prev_t_;
		this->vel_prev_t_ = vel->header.stamp.sec + vel->header.stamp.nsec*(1.0e-9);
		
		this->mea_vel_(0) = vel->twist.linear.y;
		this->mea_vel_(1) = vel->twist.linear.x;
		this->mea_vel_(2) = -vel->twist.linear.z;
		this->mea_wb_(0) = vel->twist.angular.y;
		this->mea_wb_(1) = vel->twist.angular.x;
		this->mea_wb_(2) = -vel->twist.angular.z;
		
		//update accel estimate
		Eigen::Vector3d acc = (this->mea_vel_ - this->vel_prev_)/vel_dt;
		acc(2) += -9.8066;
		this->fz_est_raw_ = -acc.dot(this->mea_R_.col(2));
		
		// Moving average update
		this->fz_est_sum_ = this->fz_est_sum_ + this->fz_est_raw_ - (this->fz_est_sum_/this->FZ_EST_N_);
		this->fz_est_ = this->fz_est_sum_/this->FZ_EST_N_;
		
		this->vel_up_ = 1;
	}

	void CCMtrackingController::targetCB(const ccm_tracking_controller::CCMTargetConstPtr& target){
		this->target_ = *target;

		// convert to eigen
		this->position_recevied_conv_= Eigen::Vector3d(this->target_.position.x, this->target_.position.y, this->target_.position.z);;
		this->velocity_recevied_conv_= Eigen::Vector3d(this->target_.velocity.x, this->target_.velocity.y, this->target_.velocity.z);;
		this->acceleration_recevied_conv_= Eigen::Vector3d(this->target_.acceleration.x, this->target_.acceleration.y, this->target_.acceleration.z);;
		this->jerk_recevied_conv= Eigen::Vector3d(this->target_.jerk.x, this->target_.jerk.y, this->target_.jerk.z);;

		this->firstTargetReceived_ = true;
		this->targetReceived_ = true;
	}

	void CCMtrackingController::cmdCB(const ros::TimerEvent&){
		if (not (this->poseReceived_ and this->velReceived_) or not this->targetReceived_){return;}
		Eigen::Vector4d cmd;

		// update controller
		this->ctrl_->updateState(this->mea_pos_, this->mea_R_, this->mea_vel_, this->mea_wb_, this->fz_est_, this->target_.dt, this->pose_up_, this->vel_up_);

		// calculate ccm
		this->ctrl_->calcCCM(this->target_.yaw, this->target_.yaw_dot, this->position_recevied_conv_, this->velocity_recevied_conv_, this->acceleration_recevied_conv_, this->jerk_recevied_conv);

		// get commands
		Eigen::Vector3d ref_er(0,0,0);
		ref_er = this->ctrl_->getEr();

		cmd(0) = ref_er(0);
		cmd(1) = ref_er(1);
		cmd(2) = ref_er(2);
		cmd(3) = std::min(1.0, std::max(0.0, 0.56 * (this->ctrl_->getfz()) / 9.8066));

		// publish command
		this->publishCommand(cmd);

		// Reset update
		this->pose_up_ = 0; 
		this->vel_up_ = 0;
		this->targetReceived_ = false;
	}

	void CCMtrackingController::visCB(const ros::TimerEvent&){
		this->publishPoseVis();
		this->publishHistTraj();
		this->publishTargetVis();
		this->publishTargetHistTraj();
		this->publishVelAndAccVis();
	}

	void CCMtrackingController::publishCommand(const Eigen::Vector4d& cmd){
		mavros_msgs::AttitudeTarget cmdMsg;
		cmdMsg.header.stamp = ros::Time::now();
		cmdMsg.header.frame_id = "map";
		cmdMsg.body_rate.x = cmd(0);
		cmdMsg.body_rate.y = cmd(1);
		cmdMsg.body_rate.z = cmd(2);
		cmdMsg.thrust = cmd(3);
		cmdMsg.type_mask = cmdMsg.IGNORE_ATTITUDE;
		this->cmdPub_.publish(cmdMsg);
	}

	void CCMtrackingController::publishPoseVis(){
		if (not this->poseReceived_) return;
		geometry_msgs::PoseStamped ps;
		ps.header.frame_id = "map";
		ps.header.stamp = ros::Time::now();

		ps.pose.position.x = this->mea_pos_(0);
		ps.pose.position.y = this->mea_pos_(1);
		ps.pose.position.z = this->mea_pos_(2);
		
		ps.pose.orientation.w = this->mea_q_(0);
		ps.pose.orientation.x = this->mea_q_(1);
		ps.pose.orientation.y = this->mea_q_(2);
		ps.pose.orientation.z = this->mea_q_(3);

		if (this->histTraj_.size() <= 100){
			this->histTraj_.push_back(ps);
		}
		else{
			this->histTraj_.push_back(ps);
			this->histTraj_.pop_front();
		}
		this->poseVis_ = ps;
		this->poseVisPub_.publish(ps);
	}

	void CCMtrackingController::publishHistTraj(){
		if (not this->poseReceived_) return;
		nav_msgs::Path histTrajMsg;
		histTrajMsg.header.frame_id = "map";
		histTrajMsg.header.stamp = ros::Time::now();
		for (size_t i=0; i<this->histTraj_.size(); ++i){
			histTrajMsg.poses.push_back(this->histTraj_[i]);
		}
		
		this->histTrajVisPub_.publish(histTrajMsg);
	}

	void CCMtrackingController::publishTargetVis(){
		if (not this->firstTargetReceived_) return;
		geometry_msgs::PoseStamped ps;
		ps.header.frame_id = "map";
		ps.header.stamp = ros::Time::now();
		ps.pose.position.x = this->target_.position.x;
		ps.pose.position.y = this->target_.position.y;
		ps.pose.position.z = this->target_.position.z;
		ps.pose.orientation = ccmtracking::quaternion_from_rpy(0, 0, this->target_.yaw);
		if (this->targetHistTraj_.size() <= 100){
			this->targetHistTraj_.push_back(ps);
		}
		else{
			this->targetHistTraj_.push_back(ps);
			this->targetHistTraj_.pop_front();
		}

		this->targetPoseVis_ = ps;
		this->targetVisPub_.publish(ps);
		
	}

	void CCMtrackingController::publishTargetHistTraj(){
		if (not this->firstTargetReceived_) return;
		nav_msgs::Path targetHistTrajMsg;
		targetHistTrajMsg.header.frame_id = "map";
		targetHistTrajMsg.header.stamp = ros::Time::now();
		for (size_t i=0; i<this->targetHistTraj_.size(); ++i){
			targetHistTrajMsg.poses.push_back(this->targetHistTraj_[i]);
		}
		
		this->targetHistTrajVisPub_.publish(targetHistTrajMsg);
	}

	void CCMtrackingController::publishVelAndAccVis(){
		if (not this->poseReceived_ and not this->velReceived_) return;
		// current velocity
		Eigen::Vector3d currPos (this->mea_pos_(0), this->mea_pos_(1), this->mea_pos_(2));
		Eigen::Vector3d currVelBody (this->mea_vel_(0), this->mea_vel_(2), this->mea_vel_(2));
		Eigen::Vector4d currQuat (this->mea_q_(0), this->mea_q_(1), this->mea_q_(2), this->mea_q_(3));
		Eigen::Matrix3d currRot = this->mea_R_;
		Eigen::Vector3d currVel = currRot * currVelBody;	

		// current acceleration	
		Eigen::Vector3d currAcc;
		ros::Time currTime = ros::Time::now();
		if (this->velFirstTime_){
			this->velPrevTime_ = ros::Time::now();
			currAcc = Eigen::Vector3d (0.0, 0.0, 0.0);
			this->velFirstTime_ = false;
		}
		else{
			double dt = (currTime - this->velPrevTime_).toSec();
			currAcc = (currVel - this->prevVel_)/dt;
			// cout << "dt: " << dt << endl;
			// cout << "current velocity: " << currVel(0) << " " << currVel(1) << " " << currVel(2) << endl;
			// cout << "prev velocity: " << this->prevVel_(0) << " " << this->prevVel_(1) << " " << this->prevVel_(2) << endl;
		}
		this->prevVel_ = currVel;
		this->velPrevTime_ = currTime;

		// target velocity
		Eigen::Vector3d targetVel (this->target_.velocity.x, this->target_.velocity.y, this->target_.velocity.z);

		// target acceleration
		Eigen::Vector3d targetAcc (this->target_.acceleration.x, this->target_.acceleration.y, this->target_.acceleration.z);


		visualization_msgs::Marker velAndAccVisMsg;
        velAndAccVisMsg.header.frame_id = "map";
        velAndAccVisMsg.header.stamp = ros::Time::now();
        velAndAccVisMsg.ns = "tracking_controller";
        // velAndAccVisMsg.id = 0;
        velAndAccVisMsg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        velAndAccVisMsg.pose.position.x = this->mea_pos_(0);
        velAndAccVisMsg.pose.position.y = this->mea_pos_(1);
        velAndAccVisMsg.pose.position.z = this->mea_pos_(2) + 0.4;
        velAndAccVisMsg.scale.x = 0.15;
        velAndAccVisMsg.scale.y = 0.15;
        velAndAccVisMsg.scale.z = 0.15;
        velAndAccVisMsg.color.a = 1.0;
        velAndAccVisMsg.color.r = 1.0;
        velAndAccVisMsg.color.g = 1.0;
        velAndAccVisMsg.color.b = 1.0;
        velAndAccVisMsg.lifetime = ros::Duration(0.05);

        double vNorm = currVel.norm();
        double aNorm = currAcc.norm();
        double vNormTgt = targetVel.norm();
        double aNormTgt = targetAcc.norm();

        std::string velText = "|V|=" + std::to_string(vNorm) + ", |VT|=" + std::to_string(vNormTgt) + "\n|A|=" + std::to_string(aNorm) + ", |AT|=" + std::to_string(aNormTgt) ;
        velAndAccVisMsg.text = velText;
        this->velAndAccVisPub_.publish(velAndAccVisMsg);
	}
}
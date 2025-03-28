#ifndef CCMCONTROLLER_H
#define CCMCONTROLLER_H

#include "ros/ros.h"
#include "mavros_msgs/ActuatorControl.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <CCM/Geodesic.h>
#include <CCM/Metric.h>
#include <math.h>



class CCMController {
private:

  ros::NodeHandle nh;

  // Mode
  bool active;

  // measured states
  Eigen::Vector3d mea_vel;
  Eigen::Matrix3d mea_R;
  Eigen::Vector3d euler;
  Eigen::Vector3d mea_wb;
  Eigen::Vector3d mea_pos;

  double fz;

  // CCM specific variables
  Eigen::VectorXd _xc_nom;
  Eigen::Matrix3d _R_des;
  Eigen::Vector4d _uc_nom;
  Eigen::VectorXd _xc_mid;
  Eigen::VectorXd _xc;
  Eigen::VectorXd _xc_nom_dot;
  Eigen::VectorXd _xc_dot;
  Eigen::MatrixXd _W_nom;
  Eigen::MatrixXd _W_mid;
  Eigen::MatrixXd _W;
  Eigen::MatrixXd _M_nom;
  Eigen::MatrixXd _M_mid;
  Eigen::MatrixXd _M;
  Eigen::VectorXd _Xc_dot;
  double _M_yaw;

  // config matrix
  Eigen::Matrix4d A;

  // inertia
  Eigen::Matrix3d J;

  // CCM outputs
  double E;
  Eigen::Vector4d uc_fb;
  double fz_dot_sum;
  double filter_N;
  double fz_dot; //CCM computed thrust_dot
  Eigen::Vector3d euler_dot;
  Eigen::Vector3d r_wb;
  double fzCmd; // Thrust command

  // use for integration
  double dt;

  // constants
  double g;
  double lambda;
  Eigen::MatrixXd _B_ctrl;

  // CCM controller funcs
  void calc_CCM_dyn(const Eigen::VectorXd &xc,const Eigen::Vector4d &uc,
                    Eigen::VectorXd &dyn);
  void calc_xc_uc_nom(const Eigen::Vector3d &r_pos,
                      const Eigen::Vector3d &r_vel,
                      const Eigen::Vector3d &r_acc,
                      const Eigen::Vector3d &r_jer,
                      const double yaw_des,
                      const double yaw_dot_des); // compute xc, uc nom

  // Coordinate conversions
  void R2euler_123(void);
  void Euler2R_123(const double r, const double p, const double y);
  void R_om(const Eigen::Vector3d &q, const Eigen::Vector3d &q_dot);

public:
  CCMController(const double N=2.0);

  // copy in updated state into controller class
  void updateState(const Eigen::Vector3d &r, const Eigen::Matrix3d &R,
                   const Eigen::Vector3d &v, const Eigen::Vector3d &w,
                   const double _fz, const double _dt, const int pose_up, const int vel_up);

  // compute
  void calcCCM(
          const double yaw_des,
          const double yaw_dot_des,
          const Eigen::Vector3d &r_pos,
          const Eigen::Vector3d &r_vel,
          const Eigen::Vector3d &r_acc,
          const Eigen::Vector3d &r_jer);

  // Return functions
  double getE();
  double getfz();
  Eigen::Vector3d getEr();
  Eigen::Vector3d getOm();
  Eigen::Vector3d getEuler();
  double getYawNom();

  void setMode(const bool _m);
};




#endif

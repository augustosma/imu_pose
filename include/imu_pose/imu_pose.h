#ifndef IMU_POSE_H
#define IMU_POSE_H

#include <vector>

#include <Eigen/Dense>

#include <ros/time.h>

namespace imu_pose
{
	class IMUPose
	{
		public:
		IMUPose();
		~IMUPose(void);
		
		void update(const Eigen::Vector3d &accel,const Eigen::Vector3d &w,const double &duration);
		Eigen::Vector3d getPosition() const {return position_};
		Eigen::Vector3d getOrientation() const {return orientation_};
		//void getOrientationQuaternion();
		void getPose(Eigen::Vector3d &x) const {x=x_;}
				
		private:		
		//all in navigation frame
		Eigen::Vector3d accel_;
		Eigen::Vector3d delta_speed_;
		Eigen::Vector3d delta_position_;
		Eigen::Vector3d init_position_;
		//Eigen::Vector3d init_speed_;
		Eigen::Vector3d position_;
				
		Eigen::Vector3d ang_speed_;
		Eigen::Vector3d delta_orientation_;
		Eigen::Vector3d init_orientation_;
		Eigen::Vector3d orientation_;
		
		Eigen::Matrix<double,3,3> transf_mat;
		Eigen::Matrix<double,3,3> attit_trans_mat;
		
		//Eigen::Vector3d accel_bias_;
		//Eigen::Vector3d oscil_bias_;
		
		Eigen::Vector3d g_ << 0,0,9.8;
		
		void transf_update();		
	};
}
#endif

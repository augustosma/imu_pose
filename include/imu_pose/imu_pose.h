#ifndef IMU_POSE_H
#define IMU_POSE_H

#include <vector>
#include <math.h>
#include <Eigen/Dense>

#include <ros/time.h>

namespace imu_pose
{
	class IMUPose
	{
		public:
		IMUPose();
		~IMUPose();
		void initialize(const Eigen::Matrix<double,6,1> &imu_bias,const Eigen::Matrix<double,6,1> &imu_cov);
		
		//imu is acc, ang_vel
		//state is acc, vel, angular_vel
		void update(const Eigen::Matrix<double,6,1> &imu, const Eigen::Matrix<double,9,1> &state, Eigen::Matrix<double,9,1> &new_state,const double &dt);
		void calibrate(); //const Eigen::Vector3d &angles);
				
		private:		
		Eigen::Vector3d g_;
		void transform_update();	
		Eigen::Matrix<double,9,1> my_state;
		Eigen::Matrix<double,6,1> imu_bias;
		Eigen::Matrix<double,6,1> imu_cov;
	};
}
#endif

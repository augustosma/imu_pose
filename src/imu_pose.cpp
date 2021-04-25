#include <imu_pose/imu_pose.h>

namespace imu_pose
{	
	IMUPose::IMUPose(const Eigen::Vector3d &init_position,const Eigen::Vector3d &init_orientation)
	{
		position_ = init_position;
		orientation_ = init_orientation;
		IMUPose.calibrate();
	}
	
	IMUPose::~IMUPose(void)
	{
	}

	void IMUPose::calibrate(const Eigen::Vector3d &angles)
	{
		orientation_ << 0,0,0;
		position_ << 0,0,0;
	}
	
	void IMUPose::update(const Eigen::Vector3d &accel,const Eigen::Vector3d &w,const double dt)
	{
		this.transform_update();
		
		//TODO: remove bias
		
		//ANGULAR POSITION
		ang_speed_ = attit_trans_mat * w;
		delta_orientation = ang_speed_ * dt;
		orientation_ = orientation_ + delta_orientation;

		//LINEAR POSITION
		accel_ = transf_mat * accel - g_;	
		delta_speed_ = accel_*dt;
		delta_position_ = delta_speed_*dt;
		position_ = position_ + delta_position_;
	}
		
	void IMUPose::transform_update()
	{
		//imu frame
 		double phi = orientation_[0];//euler angle, rotation around x
		double theta = orientation_[1];// around y
 		double psi = orientation_[2];//around z

		/*
		Eigen::Matrix<double,3,3> c1,c2,c3;
		c1 << cos(psi), sin(psi), 0,
				-sin(psi), cos(psi), 0,
				0,0,1;
		c2 << cos(theta), 0, -sin(theta),
				0, 1, 0,
				sin(theta), 0 , cos(theta);
		c3 << 1, 0, 0,
				0, cos(phi), sin(phi),
				0, -sin(phi), cos(phi);
		
		transf_mat = c3*c2*c1;
		*/
		
		//ANGULAR POSITION
		attit_trans_mat << 1, cos(phi)*tan(theta), sin(phi)*tan(theta),
							0, cos(phi), -sin(phi),
							0, sin(phi)/cos(theta), cos(phi)/cos(theta); 
							
		/*		
		ang_speed_[0] = (w_y*sin(phi)+w_z*cos(phi))/cos(theta); 
		ang_speed_[1] = w_y*cos(phi) - w_z*sin(phi);
		ang_speed_[2] = (w_y*sin(phi)+w_z*cos(phi))*tan(theta)+w_x;
		*/
		transf_mat << cos(theta)*cos(psi), -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi), sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi),
					cos(theta)*sin(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*cos(psi),
					-sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta);
	}
	
}

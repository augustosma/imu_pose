#include <imu_pose/imu_pose.h>
#define PI 3.14159265359
namespace imu_pose
{	
	//IMUPose::IMUPose(const Eigen::Vector3d &init_position,const Eigen::Vector3d &init_orientation)
		
	IMUPose::IMUPose()
	{
		//position_ = init_position;
		//orientation_ = init_orientation;
		//this->calibrate();
		g_ << 0,0,9.8;

	}
	
	IMUPose::~IMUPose(void)
	{
	}
	
	void IMUPose::initialize(const Eigen::Matrix<double,6,1> &imu_bias,const Eigen::Matrix<double,6,1> &imu_cov)
	{
		this->imu_bias << 0,0,0,0,0,0;//imu_bias;
		this->imu_cov << 0,0,0,0,0,0;//imu_cov;
	}

	void IMUPose::calibrate()//const Eigen::Vector3d &angles)
	{
		//bias
		
		//cova
		
		//
		//phi = 0.0;//euler angle, rotation around x
		//theta = 0.0;// around y
		//psi = 0.0;//around z
	}
	
	void IMUPose::update(const Eigen::Matrix<double,6,1> &imu, const Eigen::Matrix<double,9,1> &last_state, Eigen::Matrix<double,9,1> &new_state, const double &dt)
	{
		double phi = last_state[6];
		double theta = last_state[7];
		double psi = last_state[8];

		//remove "constant" bias
		Eigen::Vector3d w_imu,acc_imu;
		acc_imu << imu[0]-imu_bias[0],imu[1]-imu_bias[1],imu[2]-imu_bias[2];
		w_imu << imu[3]-imu_bias[3],imu[4]-imu_bias[4],imu[5]-imu_bias[5];

		//attitute matrix		
		Eigen::Matrix<double,3,3> attit_transf_mat;
		attit_transf_mat << 1, cos(phi)*tan(theta), sin(phi)*tan(theta),
				0, cos(phi), -sin(phi),
				0, sin(phi)/cos(theta), cos(phi)/cos(theta); 
		//ANGULAR POSITION
		new_state.tail<3>() = last_state.tail<3>() + attit_transf_mat * w_imu * dt;
		//fix angles out of [0,2pi)
		for(int i =0;i<3;i++){
			if(new_state[i+6]>=2*PI)
				new_state[i+6] -= 2*PI;
			if(new_state[i+6]<0)
				new_state[i+6] += 2*PI;				
		}
		
		//cossine matrix with euler angles	
		Eigen::Matrix<double,3,3> transf_mat;
		transf_mat << cos(theta)*cos(psi), -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi), sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi),
					cos(theta)*sin(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*cos(psi),
					-sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta);
		//LINEAR POSITION
		Eigen::Matrix<double,3,1> accel = transf_mat * acc_imu - g_;	
		Eigen::Matrix<double,6,1> ac;
		ac << 0.0,0.0,0.0,accel[0],accel[1],accel[2];
		
		Eigen::Matrix<double,6,6> A = Eigen::MatrixXd::Identity(6,6);
		A.topRightCorner(3,3) = Eigen::MatrixXd::Identity(3,3) * dt;
		new_state.head<6>() = A * last_state.head<6>() + dt * ac;
		
		// measurement covariance matrix
		//Eigen::Matrix<double,3,3>
			
		
		
	}
	
}

#include <boost/assign.hpp>

#include <ros/node_handle.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>

#include <imu_pose/imu_pose.h>


class IMUPosePublisher
{
	public:
		IMUPosePublisher(ros::NodeHandle node);
		~IMUPosePublisher(void);
		void publish() const;//const Eigen::Vector3d &accel,const Eigen::Vector3d &w, const double &dt) const;
		//void subscribe(); 
			
	private:
		ros::NodeHandle node_;

		ros::Publisher imuPosePub_;
		ros::Publisher tfPub_;
		ros::Subscriber imuRawSub_;
		
		imu_pose::IMUPose imu_pose_;
		
		
						
		std::string imu_nav_frame_id_; // start frame
		std::string imu_body_frame_id_; // 
		
		ros::Time lastSamplingTime_;
		
		void imuCB(const sensor_msgs::Imu::ConstPtr &imu_msg);
		Eigen::Matrix<double,9,1> state_;
		Eigen::Matrix<double,9,1> last_state_;
		void initialState(const Eigen::Vector3d &position,const Eigen::Vector3d &speed,const Eigen::Vector3d &orientation);
};

IMUPosePublisher::IMUPosePublisher(ros::NodeHandle node)
	//:imu_pose_(imu_bias,imu_cov)
{
	Eigen::Matrix<double,6,1> imu_bias;
	Eigen::Matrix<double,6,1> imu_cov;
	imu_bias << 0,0,0,0,0,0;
	imu_cov << 0,0,0,0,0,0;

	imu_pose_.initialize(imu_bias,imu_cov);

	
	node_=node;
	
	imu_nav_frame_id_="imu_start";
	node_.param("imu_nav_frame_id",imu_nav_frame_id_,imu_nav_frame_id_);
		
	imu_body_frame_id_="imu_body_link";
	node_.param("imu_body_frame_id",imu_body_frame_id_,imu_body_frame_id_);
	
	imuPosePub_=node_.advertise<nav_msgs::Odometry>("imu_pose",1000);
	tfPub_=node_.advertise<tf::tfMessage>("/tf",1000);
	lastSamplingTime_.isZero();// = ros::Time::Time(0.0);// ros::Time::now();
	imuRawSub_ = node_.subscribe("imu",1000,&IMUPosePublisher::imuCB,this);
	
	Eigen::Vector3d pos,vel,ori;
	pos << 0,0,0;
	vel = pos;
	ori = pos;
	this->initialState(pos,vel,ori);
	//last_state_ << 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0; //Eigen::MatrixXd::Zero(9,1); // initial pose
}

void IMUPosePublisher::initialState(const Eigen::Vector3d &position,const Eigen::Vector3d &speed,const Eigen::Vector3d &orientation)
{
	last_state_.head<3>() << position;
	last_state_.segment<3>(3) << speed;
	last_state_.tail<3>() << orientation; 
}

IMUPosePublisher::~IMUPosePublisher(void)
{
	tfPub_.shutdown();
	imuPosePub_.shutdown();
	imuRawSub_.shutdown();
}

void IMUPosePublisher::imuCB(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
	//ros::Time time = ros::Time::now();
	ros::Time time = imu_msg->header.stamp;
	//ros::Duration duration = time - lastSamplingTime_;
	double dt = time.toSec() - lastSamplingTime_.toSec();
	if(lastSamplingTime_.toSec() == 0)//for 1st loop 
		dt = 0.0;
	lastSamplingTime_ = time;
	
	Eigen::Matrix<double,6,1> imu;
	imu.head<3>() << imu_msg->linear_acceleration.x,imu_msg->linear_acceleration.y,imu_msg->linear_acceleration.z;
	imu.tail<3>() << imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z;
	
	imu_pose_.update(imu, last_state_,state_,dt);
	last_state_ = state_;
	this->publish();
}

void IMUPosePublisher::publish() const//const Eigen::Vector3d &accel,const Eigen::Vector3d &w,const double &dt) const
{
	ros::Time time = ros::Time::now();
//////
	//ROS_INFO("\nphi=%f theta=%f psi=%f\n ", orientation[0],orientation[1],orientation[2]);
	//ROS_INFO("\nACCELERATION\nx=%f\ny=%f\nz=%f\n ", position[0],position[1],position[2]);
///
	nav_msgs::Odometry odomMsg;
	
	odomMsg.header.stamp = time;
	odomMsg.header.frame_id = imu_nav_frame_id_;
	odomMsg.child_frame_id = imu_body_frame_id_;

	odomMsg.pose.pose.position.x = state_[0]; //<< position;
	odomMsg.pose.pose.position.y = state_[1];
	odomMsg.pose.pose.position.z = state_[2];
	
	odomMsg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(state_[6],state_[7],state_[8]);
	/*	
	// Fake covariance
	double pose_cov[]={1e-6, 1e-6, 1e-16,1e-16,1e-16,1e-9};
	odomMsg.pose.covariance=boost::assign::list_of
		(pose_cov[0]) (0)  (0)  (0)  (0)  (0)
		(0)  (pose_cov[1]) (0)  (0)  (0)  (0)
		(0)  (0)  (pose_cov[2]) (0)  (0)  (0)
		(0)  (0)  (0)  (pose_cov[3]) (0)  (0)
		(0)  (0)  (0)  (0)  (pose_cov[4]) (0)
		(0)  (0)  (0)  (0)  (0)  (pose_cov[5]);


	odomMsg.twist.twist.linear.x=u[0]*cos(x[2]);
	odomMsg.twist.twist.linear.y=u[0]*sin(x[2]);
	odomMsg.twist.twist.linear.z=0;
	odomMsg.twist.twist.angular.x=0;
	odomMsg.twist.twist.angular.y=0;
	odomMsg.twist.twist.angular.z=u[1];
		
	//Fake covariance
	double twist_cov[]={1e-6,1e-6,1e-16,1e-16,1e-16,1e-9};
	odomMsg.twist.covariance=boost::assign::list_of
		(twist_cov[0]) (0)  (0)  (0)  (0)  (0)
		(0)  (twist_cov[1]) (0)  (0)  (0)  (0)
		(0)  (0)  (twist_cov[2]) (0)  (0)  (0)
		(0)  (0)  (0)  (twist_cov[3]) (0)  (0)
		(0)  (0)  (0)  (0)  (twist_cov[4]) (0)
		(0)  (0)  (0)  (0)  (0)  (twist_cov[5]);
	*/
	
	imuPosePub_.publish(odomMsg);

	tf::tfMessage tfMsg;        
    tfMsg.transforms.resize(1);
	//tfMsg.transforms[0].transform.translation.z=0.0;
	tfMsg.transforms[0].child_frame_id = imu_body_frame_id_;
	tfMsg.transforms[0].header.frame_id = imu_nav_frame_id_;
        
	geometry_msgs::TransformStamped &imu_frame = tfMsg.transforms[0];
	imu_frame.header.stamp = time;
	imu_frame.transform.translation.x = state_[0];
	imu_frame.transform.translation.y = state_[1];
	imu_frame.transform.translation.z = state_[2];
	imu_frame.transform.rotation=tf::createQuaternionMsgFromRollPitchYaw(state_[6],state_[7],state_[8]);
	
	tfPub_.publish(tfMsg);		
}

int main(int argc,char* argv[])
{
	ros::init(argc,argv,"imu_pose_publisher");
	ros::NodeHandle node;
	
	IMUPosePublisher IMUPosePublisher(node);
	
	int rate=1000;
	node.param("publish_rate",rate,rate);

	ros::Rate loop(rate);
	while(ros::ok())
	{	
		//IMUPosePublisher.publish();
		ros::spinOnce();
		loop.sleep();
	}
	return 0;
}


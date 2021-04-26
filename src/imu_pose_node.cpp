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
		void publish(void) const;
		//void subscribe(); 
			
	private:
		ros::NodeHandle node_;

		ros::Publisher imuPosePub_;
		ros::Publisher tfPub_;
		ros::Subscriber imuRawSub_;
		
		imu_pose::IMUPose imu_pose_;
		
		std::string imu_frame_id_;
		std::string base_frame_id_;
		
		ros::Time lastSamplingTime_;
		
		void imuCB(const sensor_msgs::Imu::ConstPtr &imu_msg);
};

IMUPosePublisher::IMUPosePublisher(ros::NodeHandle node)
{
	node_=node;
	
	imu_frame_id_="imu_start";
	node_.param("imu_frame_id",imu_frame_id_,imu_frame_id_);
		
	base_frame_id_="base_link";
	node_.param("base_frame_id",base_frame_id_,base_frame_id_);
	
	imuPosePub_=node_.advertise<nav_msgs::Odometry>("imu_pose",100);
	tfPub_=node_.advertise<tf::tfMessage>("/tf",100);
	lastSamplingTime_=ros::Time::now();
	imuRawSub_ = node_.subscribe("imu",100,&IMUPosePublisher::imuCB,this);
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
	
	Eigen::Vector3d ang_vel;
	ang_vel << imu_msg.angular_velocity.x,imu_msg.angular_velocity.y,imu_msg.angular_velocity.z;
	Eigen::Vector3d acc;
	acc << imu_msg.linear_acceleration.x,imu_msg.linear_acceleration.y,imu_msg.linear_acceleration.z;
	
	IMUPosePublisher::publish(acc, ang_vel, dt);
}

void IMUPosePublisher::publish(const Eigen::Vector3d &accel,const Eigen::Vector3d &w, ) const
{
	ros::Time time = ros::Time::now();
	double dt = time.toSec() - lastSamplingTime_.toSec();
	lastSamplingTime_ = time;
	
	imu_pose_.update(accel, w, dt);
	Eigen::Vector3d position = imu_pose_.getPosition();
	Eigen::Vector3d orientation = imu_pose_.getOrientation();
	
	//odom_.getPose(x);
	//odom_.getVelocity(u);

	nav_msgs::Odometry odomMsg;
	
	odomMsg.header.stamp = time;
	odomMsg.header.frame_id = imu_frame_id_;
	odomMsg.child_frame_id = base_frame_id_;

	odomMsg.pose.pose.position.x = position[0]; //<< position;
	odomMsg.pose.pose.position.y = position[1];
	odomMsg.pose.pose.position.z = position[2];
	
	odomMsg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(orientation[2],orientation[1],orientation[0]);//using euler, so inverted Roll and pitch
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
	tfMsg.transforms[0].child_frame_id = base_frame_id_;
	tfMsg.transforms[0].header.frame_id = imu_frame_id_;
        
	geometry_msgs::TransformStamped &odom_frame = tfMsg.transforms[0];
	odom_frame.header.stamp = time;
	odom_frame.transform.translation.x = position[0];
	odom_frame.transform.translation.y = position[1];
	odom_frame.transform.translation.z = position[2];
	odom_frame.transform.rotation=tf::createQuaternionMsgFromRollPitchYaw(orientation[2],orientation[1],orientation[0]);
	
	tfPub_.publish(tfMsg);
		

}

int main(int argc,char* argv[])
{
	ros::init(argc,argv,"imu_pose_publisher");
	ros::NodeHandle node;
	
	IMUPosePublisher IMUPosePublisher(node);
	
	int rate=100;
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


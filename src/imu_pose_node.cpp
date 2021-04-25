#include <boost/assign.hpp>

#include <ros/node_handle.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/IMU.h>
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
		ros::Subscriber rawIMUSubscriber_;
		
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

IMUPosePublisher::imuCB(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
	//ros::Time time = ros::Time::now();
	
	Eigen::Vector3d ang_vel << imu_msg.angular_velocity.x,imu_msg.angular_velocity.y,imu_msg.angular_velocity.z;
	Eigen::Vector3d acc << imu_msg.linear_acceleration.x,imu_msg.linear_acceleration.y,imu_msg.linear_acceleration.z;
	
	IMUPosePublisher::publish(acc, ang_vel, dt);
}

IMUPosePublisher::publish(const Eigen::Vector3d &accel,const Eigen::Vector3d &w, ) const
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

IMUPosePublisher::IMUPosePublisher(ros::NodeHandle node)//:
	//wheelRadius_(2),odom_(wheelBase_,wheelRadius_)
{
	node_=node;

	if(!node_.getParam("odometry_publisher/wheel_separation",wheelBase_))
	{
		ROS_ERROR("No 'wheel_separation' in node %s.",node_.getNamespace().c_str());
		return;
	}
		
	if(!node_.getParam("odometry_publisher/wheel_radius",wheelRadius_))
	{
		ROS_ERROR("No 'wheel_radius' in node %s.",node_.getNamespace().c_str());
		return;
	}
	
	odom_.setParams(wheelBase_,wheelRadius_);
	phi_.setZero();
	
	odom_frame_id_="odom";
	node_.param("odom_frame_id",odom_frame_id_,odom_frame_id_);
		
	base_frame_id_="base_link";
	node_.param("base_frame_id",base_frame_id_,base_frame_id_);

	odomPub_=node_.advertise<nav_msgs::Odometry>("odom",100);
	
	tfPub_=node_.advertise<tf::tfMessage>("/tf",100);
	
	lastSamplingTime_=ros::Time::now();
	
	jointStateSub_=node_.subscribe("joint_states",100,&OdometryPublisher::jointStateCB,this);
}

OdometryPublisher::~OdometryPublisher(void)
{
	tfPub_.shutdown();
	odomPub_.shutdown();
	jointStateSub_.shutdown();
}

void OdometryPublisher::jointStateCB(const sensor_msgs::JointState::ConstPtr &jointState)
{
	ros::Time time=jointState->header.stamp;
	
	// Incremental encoders sense angular displacement and
	// not velocity
	// phi[0] is the left wheel angular displacement
	// phi[1] is the right wheel angular displacement
	Eigen::Vector2d deltaPhi=-phi_;
	for(unsigned int i=0;i < jointState->position.size() && i < phi_.size();i++)
	{
		phi_[i]=jointState->position[i];
	}
	deltaPhi+=phi_;

	odom_.update(deltaPhi,time-lastSamplingTime_);
	lastSamplingTime_=time;
}

void OdometryPublisher::publish(void) const
{
	ros::Time time=ros::Time::now();
	
	Eigen::Vector3d x;
	Eigen::Vector2d u;
	odom_.getPose(x);
	odom_.getVelocity(u);

	nav_msgs::Odometry odomMsg;
	odomMsg.header.stamp=time;
	odomMsg.header.frame_id=odom_frame_id_;
	odomMsg.child_frame_id=base_frame_id_;
        	        	                	
	odomMsg.pose.pose.position.x=x[0];
	odomMsg.pose.pose.position.y=x[1];
	odomMsg.pose.pose.position.z=0;

	odomMsg.pose.pose.orientation=tf::createQuaternionMsgFromYaw(x[2]);
	
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


	odomPub_.publish(odomMsg);

	tf::tfMessage tfMsg;        
        tfMsg.transforms.resize(1);
	tfMsg.transforms[0].transform.translation.z=0.0;
	tfMsg.transforms[0].child_frame_id=base_frame_id_;
	tfMsg.transforms[0].header.frame_id=odom_frame_id_;
        
	geometry_msgs::TransformStamped &odom_frame=tfMsg.transforms[0];
	odom_frame.header.stamp=time;
	odom_frame.transform.translation.x=x[0];
	odom_frame.transform.translation.y=x[1];
	odom_frame.transform.rotation=tf::createQuaternionMsgFromYaw(x[2]);
	
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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <sstream>
#include <string>
#include <iostream>
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Vector3.h"
#include "eigen_conversions/eigen_msg.h"
#include <math.h>
//#include <eigen_conversions/tf_eigen.h>



using namespace std;
typedef Eigen::Matrix<double,6,1> Vector6d; 

//Class for 
class URForce{
    private:
		ros::NodeHandle n;
		
		ros::Publisher action_pub;
		ros::Subscriber force_sub;
		std::string object_map_frame_name;
		std::string ee_link_frame_;
		tf::TransformListener tl;
     
    public:
   
		URForce();
		virtual ~URForce();
		void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
		void publishTopics();
		void subscribeTopics();
		Eigen::Matrix3d getFTSFrame();
		double radiansToDegree(double value_in_radians);
		Eigen::Matrix3d getRotationMatrixX(double angle);
		Eigen::Matrix3d getRotationMatrixY(double angle);
		Eigen::Matrix3d getRotationMatrixZ(double angle);
};

//Constructor
URForce::URForce()
{
	object_map_frame_name ="base_link";
	ee_link_frame_="fts_link";

}

//destructor
URForce::~URForce()
{
}

void URForce::publishTopics(){
    action_pub = n.advertise<std_msgs::String>("/ur_driver/URScript", 1);
}

void URForce::subscribeTopics(){
    force_sub = n.subscribe("calibrated_fts_wrench", 1, &URForce::forceCallback, this);
}


void URForce::forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	Vector6d pos;
    Eigen::Vector3d v(0,0,0);
	string s;
	geometry_msgs::Vector3 force;
	Eigen::Matrix3d rot_matrix = getFTSFrame();
	force=msg->wrench.force;
	
    if(force.x > 10){v[0] = 1;  cout << "THe vector is " << v << endl;}
    else if(force.x < -10){v[0] = -1;}
    
    if(force.y > 10){ v[1] = 1;}
    else if(force.y < -10){v[1] = -1;}
    
    if(force.z > 10){ v[2] = 1;}
    else if(force.y < -10){v[2] = -1;}
    
    
    Eigen::Vector3d v_speed;
    v_speed = rot_matrix*v;
    s="speedl([" + std::to_string(v_speed[0]) +","+  std::to_string(v_speed[1]) +","+ std::to_string(v_speed[2]) + ",0,0,0],0.1, 0.1)";
    //cout << s << endl;
    std_msgs::String msg_s;
    msg_s.data = s;
    action_pub.publish(msg_s);
    
    
	//if ( force.x>10)
	//{
		//Eigen::Vector3d v(1,0,0);
		//Eigen::Vector3d v_speed;
		//v_speed = rot_matrix*v;
		//cout<<v_speed;
		////pos[1] = pos[1] + 0.0001;
		////s="movel(["+ std::to_string(pos[0]*1000) + "," + std::to_string(pos[1]*1000) + ","+ std::to_string(pos[2]*1000) + "," + std::to_string(pos[3]) + "," + std::to_string(pos[4]) + "," + std::to_string(pos[5]) + "],a=1.2 ,v=0.25)";
		//s="speedl([" + std::to_string(v_speed[0]) +","+  std::to_string(v_speed[1]) +","+ std::to_string(v_speed[2]) + ",0,0,0],0.1, 0.1)";
		//cout << s << endl;
		//std_msgs::String msg_s;
		//msg_s.data = s;
		//action_pub.publish(msg_s);
	//}
	//else if ( force.y>10)
	//{
		//Eigen::Vector3d v(0,1,0);
		//Eigen::Vector3d v_speed;
		//v_speed = rot_matrix*v;
		////pos[1] = pos[1] + 0.0001;
		//s="speedl([" + std::to_string(v_speed[0]) +","+  std::to_string(v_speed[1]) +","+ std::to_string(v_speed[2]) + ",0,0,0],0.1, 0.1)";
		//cout << s << endl;
		//std_msgs::String msg_s;
		//msg_s.data = s;
		//action_pub.publish(msg_s);
	//}
	//else if ( force.z>10)
	//{
		//Eigen::Vector3d v(0,0,1);
		//Eigen::Vector3d v_speed;
		//v_speed = rot_matrix*v;
		//pos[1] = pos[1] + 0.0001;
		//s="speedl([" + std::to_string(v_speed[0]) +","+  std::to_string(v_speed[1]) +","+ std::to_string(v_speed[2]) + ",0,0,0],0.1, 0.1)";
		//cout << s << endl;
		//std_msgs::String msg_s;
		//msg_s.data = s;
		//action_pub.publish(msg_s);
	//}
	//else if ( force.x<-10)
	//{
		//Eigen::Vector3d v(-1,0,0);
		//Eigen::Vector3d v_speed;
		//v_speed = rot_matrix*v;
		////pos[1] = pos[1] + 0.0001;
		////s="movel(["+ std::to_string(pos[0]*1000) + "," + std::to_string(pos[1]*1000) + ","+ std::to_string(pos[2]*1000) + "," + std::to_string(pos[3]) + "," + std::to_string(pos[4]) + "," + std::to_string(pos[5]) + "],a=1.2 ,v=0.25)";
		//s="speedl([" + std::to_string(v_speed[0]) +","+  std::to_string(v_speed[1]) +","+ std::to_string(v_speed[2]) + ",0,0,0],0.1, 0.1)";
		//cout << s << endl;
		//std_msgs::String msg_s;
		//msg_s.data = s;
		//action_pub.publish(msg_s);
	//}
	//else if ( force.y<-10)
	//{
		//Eigen::Vector3d v(0,-1,0);
		//Eigen::Vector3d v_speed;
		//v_speed = rot_matrix*v;
		////pos[1] = pos[1] + 0.0001;
		//s="speedl([" + std::to_string(v_speed[0]) +","+  std::to_string(v_speed[1]) +","+ std::to_string(v_speed[2]) + ",0,0,0],0.1, 0.1)";
		//cout << s << endl;
		//std_msgs::String msg_s;
		//msg_s.data = s;
		//action_pub.publish(msg_s);
	//}
	//else if ( force.z<-10)
	//{
		//Eigen::Vector3d v(0,0,-1);
		//Eigen::Vector3d v_speed;
		//v_speed = rot_matrix*v;
		////pos[1] = pos[1] + 0.0001;
		//s="speedl([" + std::to_string(v_speed[0]) +","+  std::to_string(v_speed[1]) +","+ std::to_string(v_speed[2]) + ",0,0,0],0.1, 0.1)";
		//cout << s << endl;
		//std_msgs::String msg_s;
		//msg_s.data = s;
		//action_pub.publish(msg_s);
	//}
}
Eigen::Matrix3d URForce::getFTSFrame()
{
	Eigen::Affine3d end2map = Eigen::Affine3d::Identity();
	tf::StampedTransform end_efect_frame_to_map;
	
	Vector6d current_pose;

	try {
			tl.waitForTransform(object_map_frame_name, ee_link_frame_, ros::Time(0), ros::Duration(10) );
			tl.lookupTransform(object_map_frame_name, ee_link_frame_, ros::Time(0), end_efect_frame_to_map);
			} catch (tf::TransformException ex) {
			ROS_ERROR("%s",ex.what());
			//return;
			}
	//tf::transformTFToEigen(end_efect_frame_to_map,end2map);
	geometry_msgs::Pose target_pose;
	tf::poseEigenToMsg(end2map, target_pose); 
	//current_pose[0] = target_pose.position.x;
	//current_pose[1] = target_pose.position.y;
	//current_pose[2] = target_pose.position.z;
	
	current_pose[0] = end_efect_frame_to_map.getOrigin().x();
	current_pose[1] = end_efect_frame_to_map.getOrigin().y();
	current_pose[2] = end_efect_frame_to_map.getOrigin().z();
	//getting the rotation (transformation is needed)
	tf::Quaternion q;
	q = end_efect_frame_to_map.getRotation();
	//tf::quaternionMsgToTF(target_pose.orientation,q);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	current_pose[3] = roll;
	current_pose[4] = pitch;
	current_pose[5] = yaw;
	
	//roll = radiansToDegree(roll);
	//pitch = radiansToDegree(pitch);
    //yaw = radiansToDegree(yaw);
   
	Eigen::Matrix3d rot_matrix_x = getRotationMatrixX(roll);
	Eigen::Matrix3d rot_matrix_y = getRotationMatrixY(pitch);
	Eigen::Matrix3d rot_matrix_z = getRotationMatrixZ(yaw);
	Eigen::Matrix3d rot_matrix = rot_matrix_z * rot_matrix_y *rot_matrix_x;
	return rot_matrix;
}

double URForce::radiansToDegree(double value_in_radians)
{
	double value_in_degrees = 0;
	value_in_degrees = value_in_radians * 180/3.1416;
	return value_in_degrees;
}

Eigen::Matrix3d URForce::getRotationMatrixX(double angle)
{
	Eigen::Matrix3d matrix;
	matrix.row(0) << 1, 0, 0;
    matrix.row(1) << 0, cos(angle), -sin(angle);
    matrix.row(2) << 0, sin(angle), cos(angle);
    
    return matrix;
}

Eigen::Matrix3d URForce::getRotationMatrixY(double angle)
{
	Eigen::Matrix3d matrix;
	matrix.row(0) << cos(angle), 0, sin(angle);
    matrix.row(1) << 0, 1, 0;
    matrix.row(2) << -sin(angle), 0, cos(angle);
    
    return matrix;
}

Eigen::Matrix3d URForce::getRotationMatrixZ(double angle)
{
	Eigen::Matrix3d matrix;
	matrix.row(0) << cos(angle), -sin(angle), 0;
    matrix.row(1) << sin(angle), cos(angle),0;
    matrix.row(2) << 0, 0 ,1 ;
    
    return matrix;
}

//Main function
int main(int argc, char **argv)
{
  //Initialize ROS
  ros::init(argc, argv, "fixed_joint_state_publisher");
  URForce myClass;
  myClass.publishTopics();
  myClass.subscribeTopics();
  ros::spin();
  return 0;
}

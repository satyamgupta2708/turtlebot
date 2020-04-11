#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/transform_datatypes.h"

 geometry_msgs::Vector3 rpy;
 geometry_msgs::Twist twist;

 

ros::Publisher chatter_pub;
ros::Subscriber quat_subscriber;

float goal_yaw = 3.5;

void MsgCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    // geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;


     twist.angular.z = +0.1;

      if (rpy.z <= 3.0 && rpy.z >=3.05)
      {
      	twist.angular.z=0;
      } 
       
      
      chatter_pub.publish(twist);

    // this Vector is then published:
    // rpy_publisher.publish(rpy);
    ROS_INFO("published rpy angles: roll=%f pitch=%f yaw=%f", rpy.x, rpy.y, rpy.z);

   

}





int main(int argc, char **argv)

{

ros::init(argc, argv, "talker");
// ros::Rate loop_rate(10);

ros::NodeHandle n;
// ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

chatter_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

quat_subscriber = n.subscribe("/mobile_base/sensors/imu_data", 1000, MsgCallback);
ROS_INFO("waiting for quaternion");
ros::spin();
     

    // loop_rate.sleep();



// twist.linear.x = -0.1; 

// twist.linear.y = 0.0;
// twist.linear.z = 0.0; 
// twist.angular.x = 0.0; 
// twist.angular.y = 0.0;



// ROS_INFO("published velocity_twist.linear.x=%f",twist.linear.x);
 
//  chatter_pub.publish(twist);

//  ros::spinOnce();
//  //ros::spin();
//  loop_rate.sleep();
    
	
    




return 0;
}
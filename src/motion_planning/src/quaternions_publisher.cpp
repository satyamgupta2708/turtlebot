#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "sensor_msgs/Imu.h"

ros::Publisher rpy_publisher;
ros::Subscriber quat_subscriber;

void MsgCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->orientation, quat);

    // double x  = quat.w();

    // ROS_INFO("got the quaternions: quat.w=%f", x);


    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    // this Vector is then published:
    rpy_publisher.publish(rpy);
    ROS_INFO("published rpy angles: roll=%f pitch=%f yaw=%f", rpy.x, rpy.y, rpy.z);
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    rpy_publisher = n.advertise<geometry_msgs::Vector3>("rpy_angles", 1000);
    quat_subscriber = n.subscribe("/mobile_base/sensors/imu_data", 1000, MsgCallback);

    // check for incoming quaternions untill ctrl+c is pressed
    ROS_INFO("waiting for quaternion");
    ros::spin();
    return 0;
}
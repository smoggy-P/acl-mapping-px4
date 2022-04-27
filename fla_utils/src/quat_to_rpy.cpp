#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "tf/transform_datatypes.h"

ros::Publisher pixhawk_publisher;
ros::Subscriber pixhawk_subscriber;

ros::Publisher setpoint_publisher;
ros::Subscriber setpoint_subscriber;

ros::Publisher samwise_publisher;
ros::Subscriber samwise_subscriber;

double rad_to_deg(double rad) {
    return rad * 180.0 / M_PI;
}

void SetpointQuatToRpy(geometry_msgs::Quaternion const& quat_msg, geometry_msgs::Vector3 & rpy_msg) {
    tf::Quaternion quat;
    tf::quaternionMsgToTF(quat_msg, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    rpy_msg.x = rad_to_deg(roll);
    rpy_msg.y = rad_to_deg(pitch);
    rpy_msg.z = rad_to_deg(yaw);
}

void EstimateQuatToRpy(geometry_msgs::Quaternion const& quat_msg, geometry_msgs::Vector3 & rpy_msg) {
    tf::Quaternion quat;
    tf::quaternionMsgToTF(quat_msg, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    rpy_msg.x = rad_to_deg(roll);
    rpy_msg.y = rad_to_deg(pitch);
    rpy_msg.z = rad_to_deg(yaw);
}

void PixhawkCallback(const geometry_msgs::PoseStamped pose_msg)
{
    geometry_msgs::Vector3 rpy;
    EstimateQuatToRpy(pose_msg.pose.orientation, rpy);
    pixhawk_publisher.publish(rpy);
}

void SamwiseCallback(const geometry_msgs::PoseStamped pose_msg)
{
    geometry_msgs::Vector3 rpy;
    EstimateQuatToRpy(pose_msg.pose.orientation, rpy);
    samwise_publisher.publish(rpy);
}

void SetpointCallback(const mavros_msgs::AttitudeTarget setpoint_msg)
{
    geometry_msgs::Vector3 rpy;
    SetpointQuatToRpy(setpoint_msg.orientation, rpy);
    setpoint_publisher.publish(rpy);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quat_to_rpy");
    ros::NodeHandle n;

    pixhawk_publisher = n.advertise<geometry_msgs::Vector3>("rpy_pixhawk", 10);
    pixhawk_subscriber = n.subscribe("pose_pixhawk", 10, PixhawkCallback);

    setpoint_publisher = n.advertise<geometry_msgs::Vector3>("rpy_setpoint", 10);
    setpoint_subscriber = n.subscribe("attitude_setpoint", 10, SetpointCallback);

    samwise_publisher = n.advertise<geometry_msgs::Vector3>("rpy_samwise", 10);
    samwise_subscriber = n.subscribe("pose_samwise", 10, SamwiseCallback);

    // check for incoming quaternions untill ctrl+c is pressed
    ROS_INFO("waiting for quaternion");
    ros::spin();

    return 0;
}

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// Publisher to command robot velocity
ros::Publisher motor_command_publisher;

// This callback executes whenever the service is called
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
                          ball_chaser::DriveToTarget::Response& res)
{
    ROS_INFO("DriveToTarget received - linear_x: %1.2f, angular_z: %1.2f",
             req.linear_x, req.angular_z);

    // Create motor command
    geometry_msgs::Twist motor_command;

    // Assign requested velocities
    motor_command.linear.x  = req.linear_x;
    motor_command.angular.z = req.angular_z;

    // Publish the velocities to the robot
    motor_command_publisher.publish(motor_command);

    // Return feedback message
    res.msg_feedback = "Set velocities - linear_x: " +
                       std::to_string(req.linear_x) +
                       " , angular_z: " +
                       std::to_string(req.angular_z);

    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a handle to the ROS system
    ros::NodeHandle n;

    // Publisher for /cmd_vel
    motor_command_publisher = 
        n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define service /ball_chaser/command_robot
    ros::ServiceServer service = 
        n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    ROS_INFO("Ready to send wheel commands through /ball_chaser/command_robot");

    // Handle incoming service requests
    ros::spin();

    return 0;
}

#include "ros/ros.h"

#include "ball_chaser/DriveToTarget.h"

#include <sensor_msgs/Image.h>



// Define a global client that can request services

ros::ServiceClient client;



// This function calls the command_robot service to drive the robot in the specified direction

void drive_robot(float lin_x, float ang_z)

{

    ROS_INFO_STREAM("Driving the robot - linear_x: " << lin_x << " angular_z: " << ang_z);



    // Create a service message

    ball_chaser::DriveToTarget srv;

    srv.request.linear_x = lin_x;

    srv.request.angular_z = ang_z;



    // Call the service and check for success

    if (!client.call(srv)) {

        ROS_ERROR("Failed to call service /ball_chaser/command_robot");

    }

}



// This callback function executes every time an image is received

void process_image_callback(const sensor_msgs::Image& img)
{
    int white = 255;
    int sum_x = 0;
    int count = 0;

    // Scan image for white pixels
    for (int i = 0; i < img.height * img.step; i += 3)
    {
        int r = img.data[i];
        int g = img.data[i + 1];
        int b = img.data[i + 2];

        if (r == white && g == white && b == white)
        {
            int x = (i / 3) % img.width;
            sum_x += x;
            count++;
        }
    }

    // No ball → search
    if (count == 0)
    {
        drive_robot(0.0, 0.3);
        return;
    }

    // Compute ball center
    int ball_x = sum_x / count;
    int mid = img.width / 2;
    int error = ball_x - mid;

    // Tuning parameters
    float max_ang = 0.4;
    float forward_speed = 0.4;
    int center_tolerance = 30;  // pixels

    float angular = 0.0;
    float linear  = 0.0;

    // Angular control (proportional)
    angular = -0.002 * error;

    // Clamp angular velocity
    if (angular >  max_ang) angular =  max_ang;
    if (angular < -max_ang) angular = -max_ang;

    // Forward motion logic
    if (abs(error) < center_tolerance)
    {
        // Ball roughly centered → move forward
        linear = forward_speed;
    }
    else
    {
        // Ball far off → rotate only
        linear = 0.0;
    }

    drive_robot(linear, angular);
}




int main(int argc, char** argv)

{

    // Initialize the process_image node and create a handle

    ros::init(argc, argv, "process_image");

    ros::NodeHandle n;



    // Client service for requesting robot movement

    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");



    // Subscribe to camera topic

    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);



    // Handle ROS communication

    ros::spin();



    return 0;

}



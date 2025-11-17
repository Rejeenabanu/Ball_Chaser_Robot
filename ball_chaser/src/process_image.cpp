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

void process_image_callback(const sensor_msgs::Image img)

{

    int white_pixel = 255;
    bool ball_found = false;

    int ball_pos = -1;



    // Loop through image data (3 bytes per pixel)

    for (int i = 0; i < img.height * img.step; i += 3) {



        int r = img.data[i];

        int g = img.data[i + 1];

        int b = img.data[i + 2];



        // Detect white pixel

        if (r == white_pixel && g == white_pixel && b == white_pixel) {
			ball_found = true;

            ball_pos = (i / 3) % img.width; // horizontal position

            break;

        }
		//Detect red pixel
		if (r == 255 && g == 0 && b == 0) {
			ball_found = true;
            ball_pos = (i / 3) % img.width; // horizontal position

            break;

        }
		//Detect green pixel
		 if (r == 0 && g == 255 && b == 0) {
			ball_found = true;
            ball_pos = (i / 3) % img.width; // horizontal position

            break;

        }
		//Detect blue pixel
		if (r == 0 && g == 0 && b == 255) {

            ball_pos = (i / 3) % img.width; // horizontal position

            break;

        }

    }
	
	int left_bound = img.width / 3;
    int right_bound = 2 * img.width / 3;



    // No ball detected → stop robot

    //if (ball_pos == -1) {

     //   drive_robot(0.0, 0.0);

        return;

   // }
   
   if (ball_found)
    {
        if (ball_pos < left_bound)
        {
            // Ball is on LEFT → turn left
            drive_robot(0.0, 0.5);
        }
        else if (ball_pos > right_bound)
        {
            // Ball is on RIGHT → turn right
            drive_robot(0.0, -0.5);
        }
        else
        {
            // Ball is CENTER → move forward
            drive_robot(0.5, 0.0);
        }
    }
    else
    {
        // No ball → rotate in place to search
        drive_robot(0.0, 0.5);  // slow spin left
    }



    // Divide width into left/middle/right

    //int section = img.width / 3;



    //if (ball_pos < section) {

        // Left side → turn left

     //   drive_robot(0.0, 0.5);

    //}

   // else if (ball_pos < 2 * section) {

        // Middle → move forward

      //  drive_robot(0.5, 0.0);

   // }

   // else {

        // Right → turn right

     //   drive_robot(0.0, -0.5);

   // }

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



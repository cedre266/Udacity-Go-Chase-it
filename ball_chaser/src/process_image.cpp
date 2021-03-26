#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <string>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    
    ROS_INFO_STREAM("Sending move command to the robot");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    
    if (!client.call(srv))
        ROS_ERROR("Failed to call service DriveToTarget");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    int col;
    int counter_left = 0;
    int counter_straight = 0;
    int counter_right = 0;
    float left_limit = img.width / 5;
    float right_limit = 4 * img.width / 5;
    int max_counter;
    std::string direction;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    
    for (int i = 0; i < img.height * img.step; i+=3) {
        
        if ((img.data[i] == white_pixel) && (img.data[i+1] == white_pixel) && (img.data[i+2] == white_pixel)) {
            
            col = i % img.width; // Find which column is the pixel belonging to
            
            // Count which part of the image the pixel is belonging to
            if (col < left_limit) counter_left++;
            else if ((col > left_limit) && (col < right_limit)) counter_straight++;
            else if (col > right_limit) counter_right++;
        }
    }
    
    if ((counter_left == 0) && (counter_straight == 0) && (counter_right == 0)) {
    	ROS_INFO_STREAM("White ball not found");
    	drive_robot(0,0);
    	}
    else { 
    	// Find maximum counter to decide direction of move
    	max_counter = counter_straight;
    	direction = "straight";
    	if (counter_left > max_counter) {
    	 direction = "left";
    	 max_counter = counter_left;
    	 }
    	if (counter_right > max_counter) {
    	 direction = "right";
    	 max_counter = counter_right;
    	 }
    	
    	// Send movement command
    	if (direction == "straight") {
	 ROS_INFO_STREAM("White ball straight ahead");
    	 drive_robot(0.5,0);
	}
    	if (direction == "left") {
    	 ROS_INFO_STREAM("White ball on the left");
    	 drive_robot(0,0.5);
    	}
    	if (direction == "right") {
	 ROS_INFO_STREAM("White ball on the right");
    	 drive_robot(0,-0.5);
    	}
    }

    //ROS_INFO("Pixel value: %d", img.data[0]);
     
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}

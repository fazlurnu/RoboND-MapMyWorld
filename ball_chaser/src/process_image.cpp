#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("Driving robot towards the ball");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service safe_move");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    // The idea is to find the center of mass (in step direction) of the ball and use it
    // to define whether the ball is on the left/center/right

    int white_pixel = 255;
    int image_height = img.height;
    int step = img.step;
    float x_center_of_image = step/2;
    
    float command_lin_x = 0;
    float command_ang_z = 0;

    int total_mass = 0;
    float sum_of_x = 0;

    float kp = 2; //gain for turning

    for(int i=0; i < image_height; i++){
        for(int j=0; j < step; j++){
            if(img.data[i*step + j] == white_pixel){
                sum_of_x += j;
                total_mass++;
            }
        }
    }

    float x_center_of_mass = sum_of_x/total_mass;

    if(total_mass > 0){
        command_lin_x = 0.1;
        command_ang_z = (x_center_of_image - x_center_of_mass)/x_center_of_image * kp; //normalized
    } else{
        command_lin_x = 0.0;
        command_ang_z = 0.0;
    }

    drive_robot(command_lin_x, command_ang_z);
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
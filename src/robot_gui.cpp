#include "robot_gui/robot_gui.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "robot_gui/GetDistance.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "std_srvs/Trigger.h"
#include <cmath>
#include <ros/ros.h>
#include <string>

RobotGUI::RobotGUI(ros::NodeHandle &nh) : nh_(nh) {
    robot_info_sub_ = nh_.subscribe("robot_info", 10, &RobotGUI::robotInfoCallBack, this);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    odom_sub_ = nh_.subscribe("odom", 10, &RobotGUI::odomInfoCallBack, this);

    service_client = nh_.serviceClient<std_srvs::Trigger>("get_distance");

    cv::namedWindow(WINDOW_NAME);
};

RobotGUI::~RobotGUI() {

};

void RobotGUI::runGraphicalInterface() {
    cvui::init(WINDOW_NAME);

    while (ros::ok()) {
        if (scaling_ != currentScaling_) {
            frame_ = cv::Mat(std::lround(scaling_ * 720), std::lround(scaling_ * 300), CV_8UC3);
            currentScaling_ = scaling_;
        };

		frame_ = cv::Scalar(49, 52, 49); // Fill the frame with a background color

        drawGeneralInfoArea();           // Genearl Info Area 
        drawTeloperationButtons();       // Teleoperation Buttons
        drawCurrentVelocities();         // Current Velocities 
        drawRobotPosition();             // Robot Position
        drawDistanceTravelledService();  // Distance travelled Service 

        twist_pub_.publish(twist_msg_);  // Publishing the current twist message
		
        cvui::update(); // Update cvui
		cv::imshow(WINDOW_NAME, frame_); // Show everything on the screen
		
        // Check if ESC key was pressed
		if (cv::waitKey(20) == 27) {
			break;
		};

        ros::spinOnce();
        cv::waitKey(30);
	}; 
};

void RobotGUI::robotInfoCallBack(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
    robotInfoString = msg->data_field_01 + "\n" + msg->data_field_02 + "\n" +
                      msg->data_field_03 + "\n" + msg->data_field_04 + "\n" +
                      msg->data_field_05 + "\n" + msg->data_field_06 + "\n" +
                      msg->data_field_07 + "\n" + msg->data_field_08 + "\n" +
                      msg->data_field_09 + "\n" + msg->data_field_10;
};

void RobotGUI::drawGeneralInfoArea() {
    // Display the robot information
    cvui::window(frame_, OFFSET_X, OFFSET_Y, WINDOW_WIDTH, WINDOW_HEIGHT, "Info");
    
    // Create an input string stream from robotInfoString
    std::istringstream stream(robotInfoString);

    int yOffset = 30; // y-axis of the first line of text

    // Populating the window line by line in a while loop  
    while (std::getline(stream, infoline)){ 
        cvui::text(frame_, 10, yOffset, infoline);
        yOffset += 20;
    };
};

bool RobotGUI::drawTeloperationButtons() {
    // Display the robot Buttons 

    // Move forward button and gray color
    if (cvui::button(frame_, 105, 210, BOTTON_LENGTH, BOTTON_WIDTH, "Forward", scaling_*cvui::DEFAULT_FONT_SCALE, 0xb1b8b4)) {
        linear_speed_ += 0.1;
        twist_msg_.linear.x = linear_speed_;
    };
    
    // Move backward button and gray color
    if (cvui::button(frame_, 105, 340, BOTTON_LENGTH, BOTTON_WIDTH, "Backwards", scaling_*cvui::DEFAULT_FONT_SCALE, 0xb1b8b4)){
        linear_speed_ -= 0.1;
        twist_msg_.linear.x = linear_speed_;
    };
    
    // Move right button and gray color
    if (cvui::button(frame_, 200, 275, BOTTON_LENGTH, BOTTON_WIDTH, "Right", scaling_*cvui::DEFAULT_FONT_SCALE, 0xb1b8b4)) {
        angular_speed_ -= 0.1;
        twist_msg_.angular.z = angular_speed_;
    };
    
    // Move left button and gray color
    if (cvui::button(frame_, 10, 275, BOTTON_LENGTH, BOTTON_WIDTH, "Left", scaling_*cvui::DEFAULT_FONT_SCALE, 0xb1b8b4)) {
        angular_speed_ += 0.1;
        twist_msg_.angular.z = angular_speed_;
    };

    // Stop button and orange color
    if (cvui::button(frame_, 105, 275, BOTTON_LENGTH, BOTTON_WIDTH, "Stop", scaling_*cvui::DEFAULT_FONT_SCALE, 0xffa950)) { 
        linear_speed_ = 0.0;
        angular_speed_ = 0.0;
        twist_msg_.linear.x = linear_speed_;
        twist_msg_.angular.z = angular_speed_;
    };

    return true;
};

void RobotGUI::drawCurrentVelocities() {
    // Display the current velocity 

    // Linear velocity window
    cvui::window(frame_, 10, 415, 140, 50, "Linear velocity: " );
    cvui::text(frame_, 15, 445, std::to_string(linear_speed_) + " m/sec", scaling_*cvui::DEFAULT_FONT_SCALE, 0xffa950);
    
    // Angular velocity window
    cvui::window(frame_, 150, 415, 140, 50, "Angular velocity: " );
    cvui::text(frame_, 155, 445, std::to_string(angular_speed_) + " rad/sec", scaling_*cvui::DEFAULT_FONT_SCALE, 0xffa950);
};

void RobotGUI::odomInfoCallBack(const nav_msgs::Odometry::ConstPtr &msg) {

    // Get the current position from the odometry message
    current_position_x_ = msg->pose.pose.position.x;
    current_position_y_ = msg->pose.pose.position.y;
    current_position_z_ = msg->pose.pose.position.z;
};

void RobotGUI::drawRobotPosition() {
    // Display the robot position
    cvui::text(frame_, 10, 480, "Estimated robot position based off odometry: ", 0.38);
    
    // X-axes window position 
    cvui::window(frame_, 10, 500, 90, 90,  "     X");
    cvui::printf(frame_, 22, 540, 0.9, 0xffffff, "%.1f", current_position_x_);

    // Y-axes window position 
    cvui::window(frame_, 105, 500, 90, 90, "     Y");
    cvui::printf(frame_, 117, 540, 0.9, 0xffffff, "%.1f", current_position_y_);

    // Z-axes window position 
    cvui::window(frame_, 200, 500, 90, 90, "     Z");
    cvui::printf(frame_, 212, 540, 0.9, 0xffffff, "%.1f", current_position_z_);

};

void RobotGUI::callDistanceService() {
    // Call the server to get the distance
    std_srvs::Trigger srv;

    if (service_client.call(srv)) {
        if (srv.response.success) {
            distance_travelled_ = std::stod(srv.response.message);    
        }  
        else {
            ROS_ERROR("Fail to call service get_distance");
        }    
    }
    else {
        ROS_ERROR("Fail to connect to server");
    }
};
  
void RobotGUI::drawDistanceTravelledService() {
    // Display distance window
    cvui::text(frame_, 10, 600, "Distance Travelled:");
    if (cvui::button(frame_, 10, 620, 90, 90, "Call")){
        callDistanceService();
    };
    cvui::window(frame_, 105, 620, 185, 90, "Distance in meters: ");
    cvui::printf(frame_, 130, 660, 1.0 , 0xffffff, "%.2f m", distance_travelled_);
};
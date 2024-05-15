#include "robot_gui/robot_gui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>

RobotGUI::RobotGUI(ros::NodeHandle &nh) : nh_(nh) {
    robot_info_sub_ = nh_.subscribe("robot_info", 10, &RobotGUI::robotInfoCallBack, this);
};

RobotGUI::~RobotGUI() {};

void RobotGUI::runGraphicalInterface() {
    cvui::init(WINDOW_NAME);

    while (ros::ok()) {
        if (scaling_ != currentScaling_) {
            frame_ = cv::Mat(std::lround(scaling_ * 700), std::lround(scaling_ * 300), CV_8UC3);
            currentScaling_ = scaling_;
        };

		// Fill the frame_ with a nice color
		frame_ = cv::Scalar(49, 52, 49);

		// Genearl Info Area 
        drawGeneralInfoArea();
        
		cvui::update();

		// Show everything on the screen
		cv::imshow(WINDOW_NAME, frame_);

		// Check if ESC key was pressed
		if (cv::waitKey(20) == 27) {
			break;
		};

        ros::spinOnce();
        count++;
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
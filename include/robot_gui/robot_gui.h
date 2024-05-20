#pragma once

#define CVUI_IMPLEMENTATION

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <robotinfo_msgs/RobotInfo10Fields.h>
#include <geometry_msgs/Twist.h>

#include "robot_gui/cvui.h"

#define WINDOW_NAME "Robot Info - GUI"

class RobotGUI {
public:
    RobotGUI(ros::NodeHandle &nh); // Constructor 
    ~RobotGUI();  // Destructor 
    
    void runGraphicalInterface(); // Function that runs the GUI

private:
    ros::NodeHandle nh_;
    ros::Subscriber robot_info_sub_; // Receives the robot information 
    ros::Publisher twist_pub_; // Publish the current message

    geometry_msgs::Twist twist_msg_;
    
    std::string robotInfoString; // Stores the data that will be display in the window
    std::string infoline; // Element of the data
    
    cv::Mat frame_;

    int count = 0;
    double trackbarValue {0.0};
    double scaling_ {1.0};
    double currentScaling_ {-1};

    // Genearl Info Area parameters //
    const int OFFSET_X {5};
    const int OFFSET_Y {5};
    const int WINDOW_WIDTH {290};
    const int WINDOW_HEIGHT {190};
    // End of Genearl Info Area parameters //

    // Teleoperation Buttons //
    double angular_speed_;
    double linear_speed_;
    const int BOTTON_LENGTH {90};
    const int BOTTON_WIDTH {60};
    // End of Teleoperation Buttons //

    void robotInfoCallBack(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg); // the Call Back this will resive the incoming messages
    void drawGeneralInfoArea(); // Function of the General Info Area
    bool drawTeloperationButtons(); // Function of the Teleopertion Buttons
};
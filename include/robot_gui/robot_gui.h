#pragma once

#include <string>
#include <vector>
#define CVUI_IMPLEMENTATION

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <robotinfo_msgs/RobotInfo10Fields.h>

#include "robot_gui/cvui.h"

#define WINDOW_NAME "Robot Info - GUI"

class RobotGUI {
public:
    RobotGUI(ros::NodeHandle &nh); // Constructor 
    ~RobotGUI();  // Destructor 
    
    void runGraphicalInterface(); // Function that runs the GUI

private:
    ros::NodeHandle nh_;
    ros::Subscriber robot_info_sub_;
    ros::Publisher robot_info_pub_;
    cv::Mat frame_;
	
    int count = 0;

    double trackbarValue {0.0};
    double scaling_ {1.0};
    double currentScaling_ {-1};
     
    // Genearl Info Area parameters //
    std::string robotInfoString; // Stores the data that will be display in the window
    std::string infoline; // Element of the data
    const int OFFSET_X {5};
    const int OFFSET_Y {5};
    const int WINDOW_WIDTH {290};
    const int WINDOW_HEIGHT {190};
    // End of Genearl Info Area parameters //

    void robotInfoCallBack(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg); // the Call Back this will resive the incoming messages
    void drawGeneralInfoArea(); // Function of the General Info Area
};
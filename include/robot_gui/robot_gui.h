#pragma once

#include "ros/service_client.h"
#define CVUI_IMPLEMENTATION

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <robotinfo_msgs/RobotInfo10Fields.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "robot_gui/GetDistance.h"

#include "robot_gui/cvui.h"

#define WINDOW_NAME "Robot Info - GUI"

class RobotGUI {
  public:
    RobotGUI(ros::NodeHandle &nh); 
    ~RobotGUI();
    
    void runGraphicalInterface();       // Function that runs the GUI

  private:
    ros::NodeHandle nh_;
    ros::Subscriber robot_info_sub_;    // Receives the robot information 
    ros::Publisher twist_pub_;          // Publish the current message
    ros::Subscriber odom_sub_;          // Recives the robot position 
    ros::ServiceClient service_client;  // Service client 

    geometry_msgs::Twist twist_msg_;
    nav_msgs::Odometry odom_msg_;
    
    std::string robotInfoString;        // Stores the data that will be display in the window
    std::string infoline;               // Element of the data

    cv::Mat frame_;

    double trackbarValue {0.0};
    double scaling_ {1.0};
    double currentScaling_ {-1};

    // Genearl Info Area parameters
    const int OFFSET_X {5};
    const int OFFSET_Y {5};
    const int WINDOW_WIDTH {290};
    const int WINDOW_HEIGHT {190};

    // Teleoperation Buttons parameter
    double angular_speed_ {0.0};
    double linear_speed_ {0.0};
    const int BOTTON_LENGTH {90};
    const int BOTTON_WIDTH {60};

    //Robot Position
    double distance_travelled_ {0.0};
    double current_position_x_ {0.0};
    double current_position_y_ {0.0};
    double current_position_z_ {0.0};

    void robotInfoCallBack(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);  // CallBack this will receive incoming messages
    void drawGeneralInfoArea();                                                      // Function of General Info Area
    bool drawTeloperationButtons();                                                  // Function of Teleopertion Buttons
    void drawCurrentVelocities();                                                    // Function of Current Velocities 
    void odomInfoCallBack(const nav_msgs::Odometry::ConstPtr &msg);                  // Updates the robot position 
    void drawRobotPosition();                                                        // Function of Robot Position 
    void callDistanceService();                                                      // Call the service to calculate the distance
    void drawDistanceTravelledService();                                             // Function of the Distance Travelled Service
};
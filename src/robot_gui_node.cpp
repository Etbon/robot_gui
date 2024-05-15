#include "robot_gui/robot_gui.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_gui_node");
    ROS_INFO("Node is online and running...");
    ros::NodeHandle nh;

    RobotGUI robotGUI(nh);
    robotGUI.runGraphicalInterface();

    return 0;
};
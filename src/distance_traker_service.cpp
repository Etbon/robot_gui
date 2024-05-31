#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "ros/subscriber.h"
#include "std_srvs/Trigger.h"


double total_distance_ {0.0};
double previous_position_x_ {0.0};
double previous_position_y_ {0.0};
bool first_reading = true;

// CallBack function for the odomerty data
void odmoDistanceCallBack(const nav_msgs::Odometry::ConstPtr &msg) {
    
    const double current_position_x_ = msg->pose.pose.position.x;
    const double current_position_y_ = msg->pose.pose.position.y;

    double dx = current_position_x_ - previous_position_x_;
    double dy = current_position_y_ - previous_position_y_;

    // Calculating the distance withe Euclidean formula
    total_distance_ = std::sqrt(std::pow(dx,2) + std::pow(dy,2));
    
    // Updating the last position 
    previous_position_x_ = current_position_x_;
    previous_position_y_ = current_position_y_;
};

bool getDistanceCallBack(std_srvs::Trigger::Request &req,
                         std_srvs::Trigger::Response &res) {
    res.success = true;
    res.message = std::to_string(total_distance_);
    return true;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "distance_traker_service");
    ros::NodeHandle nh;
    
    ROS_INFO("Ready to get distance...");

    ros::Subscriber sub = nh.subscribe("odom", 10, odmoDistanceCallBack);
    ros::ServiceServer service = nh.advertiseService("get_distance", &getDistanceCallBack);

    ros::spin();

    return 0;
};
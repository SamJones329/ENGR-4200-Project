#include "ros/ros.h"
#include "std_msgs/String.h"
#include "particle_filter.cpp"
#include "nav_msgs/Odometry.h"
#include <Eigen/Dense>

#include <sstream>

// DVL A50 WL-21035-2 (assuming standard) long term sensor accuracy +-1.01%
// https://yostlabs.com/product/3-space-nano/ - "sensor assist" AHRS on A50, specs on page 

class DoryLocalizationNode {

    DoryLoc::ParticleFilter pf;
    ros::Subscriber dvlSub;
    ros::Subscriber pixhawkSub;
    Eigen::Vector4d lastOdom;

    DoryLocalizationNode(ros::NodeHandle n) {
        dvlSub = n.subscribe<nav_msgs::Odometry>("DVL_ODOM", 10, dvlOdomCallback);
        pixhawkSub = n.subscribe<nav_msgs::Odometry>("ROV_ODOMETRY", 10, pixhawkOdomCallback);
    }

    void dvlOdomCallback(const nav_msgs::Odometry& odom) {
        vector<double> odomVec {
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
            odom.pose.pose.orientation.z
        };
        pf.weight(odomVec);
    }

    void pixhawkOdomCallback(const nav_msgs::Odometry& odom) {
        vector<double> odomVec {
            odom.pose.pose.position.x - lastOdom(0),
            odom.pose.pose.position.y - lastOdom(1),
            odom.pose.pose.position.z - lastOdom(2),
            odom.pose.pose.orientation.z - lastOdom(3)
        };
        pf.predict(odomVec);
        lastOdom(0) = odom.pose.pose.position.x;
        lastOdom(1) = odom.pose.pose.position.y;
        lastOdom(2) = odom.pose.pose.position.z;
        lastOdom(3) = odom.pose.pose.orientation.z; //yaw
    }

    void loop() {

    }

int main(int argc, char **argv) {

    ros::init(argc, argv, "dory_localization");
    ros::NodeHandle n;
    DoryLocalizationNode node(n);

    /*
        DVL_ODOM nav_msgs.Odometry {position: {x, y, z}, oritatation: {x (roll), y (pitch), z (yaw)} - Fused integration velocity and IMU
        ROV_ODOMETRY nav_msgs.Odometry {} - Pixhawk odom

        DVL_DOPPLER navg_msgs.Odometry {xvel, yvel, zvel} - Raw Velocities
        don't know raw depth message and raw IMU from DVL
    */

    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        node.loop();
    }
    return 0;
}
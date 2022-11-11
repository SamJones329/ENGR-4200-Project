#include "ros/ros.h"
#include "std_msgs/String.h"
#include "particle_filter.cpp"
#include "nav_msgs/Odometry.h"
#include <Eigen/Dense>
#include <random>


// DVL A50 WL-21035-2 (assuming standard) long term sensor accuracy +-1.01%
// https://yostlabs.com/product/3-space-nano/ - "sensor assist" AHRS on A50, specs on page 
namespace DoryLoc {
    class Node {
        public:

        DoryLoc::ParticleFilter pf;
        ros::Subscriber dvlSub;
        ros::Subscriber pixhawkSub;
        Eigen::Vector4d lastOdom;
        std::mt19937 mt;
        std::normal_distribution<double> pixhawkDist;
        std::normal_distribution<double> dvlDist;


        void dvlOdomCallback(const nav_msgs::Odometry& odom) {
            geometry_msgs::Point pos = odom.pose.pose.position;
            double noise = dvlDist(mt);
            double x = pos.x, y = pos.y, z = pos.z, yaw = odom.pose.pose.orientation.z;
            x += noise;
            y += noise;
            z += noise;
            yaw += noise;
            std::vector<double> odomVec {x, y, z, yaw};
            pf.weight(odomVec);
        }

        void pixhawkOdomCallback(const nav_msgs::Odometry& odom) {
            geometry_msgs::Point pos = odom.pose.pose.position;
            double noise = pixhawkDist(mt);
            double x = pos.x, y = pos.y, z = pos.z, yaw = odom.pose.pose.orientation.z;
            x += noise;
            y += noise;
            z += noise;
            yaw += noise;

            std::vector<double> odomVec {
                x - lastOdom(0),
                y - lastOdom(1),
                z - lastOdom(2),
                yaw - lastOdom(3)
            };
            pf.predict(odomVec);
            lastOdom(0) = x;
            lastOdom(1) = y;
            lastOdom(2) = z;
            lastOdom(3) = yaw; 
        }

        Node(std::mt19937 gen, std::normal_distribution<double> pixhawkDistribution, std::normal_distribution<double> dvlDistribution) {
            mt = gen;
            pixhawkDist = pixhawkDistribution;
            dvlDist = dvlDistribution;

            ros::NodeHandle n;

            // dvlSub = n.subscribe<nav_msgs::Odometry>("DVL_ODOM", 10, dvlOdomCallback); 
            dvlSub = n.subscribe<nav_msgs::Odometry>("odom", 1000, &Node::dvlOdomCallback); 
        
            // pixhawkSub = n.subscribe<nav_msgs::Odometry>("ROV_ODOMETRY", 10, pixhawkOdomCallback);
            pixhawkSub = n.subscribe<nav_msgs::Odometry>("odom", 1000, &Node::pixhawkOdomCallback);
        }


        void loop() {

        }
    };
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "dory_localization");
    std::random_device rd;
    std::mt19937 mt(rd());
    std::normal_distribution<double> pixhawkDist{0., 0.05};
    std::normal_distribution<double> dvlDist{0., 0.05};
    DoryLoc::Node node(mt, pixhawkDist, dvlDist);

    /*
        DVL_ODOM nav_msgs.Odometry {position: {x, y, z}, oritatation: {x (roll), y (pitch), z (yaw)} - Fused integration velocity and IMU
        ROV_ODOMETRY nav_msgs.Odometry {} - Pixhawk odom

        DVL_DOPPLER navg_msgs.Odometry {xvel, yvel, zvel} - Raw Velocities
        don't know raw depth message and raw IMU from DVL
        altimeter ping1d sensor on bluerobotics website
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
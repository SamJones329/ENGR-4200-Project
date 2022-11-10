#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "particle_filter.hpp"
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


        void dvlOdomCallback(const nav_msgs::Odometry& odom);

        void pixhawkOdomCallback(const nav_msgs::Odometry& odom);

        Node(std::mt19937 gen, std::normal_distribution<double> pixhawkDistribution, std::normal_distribution<double> dvlDistribution);


        void loop();
    };
}
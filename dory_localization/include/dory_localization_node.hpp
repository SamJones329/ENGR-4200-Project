#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "particle_filter.hpp"
#include <Eigen/Dense>
#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// DVL A50 WL-21035-2 (assuming standard) long term sensor accuracy +-1.01%
// https://yostlabs.com/product/3-space-nano/ - "sensor assist" AHRS on A50, specs on page 
namespace DoryLoc {
    class Node {
        public:

        DoryLoc::ParticleFilter pf;
        ros::NodeHandle n;
        ros::Subscriber dvlSub;
        ros::Subscriber pixhawkSub;
        ros::Publisher meanParticlePub;
        ros::Publisher allParticlePub;
        Eigen::Vector4d lastOdom;
        std::mt19937 mt;
        std::normal_distribution<double> pixhawkDist;
        std::normal_distribution<double> dvlDist;


        void dvlOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);

        void pixhawkOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);

        Node(std::mt19937 gen, std::normal_distribution<double> pixhawkDistribution, std::normal_distribution<double> dvlDistribution);

        void loop();
    };
}
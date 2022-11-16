#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "particle_filter.hpp"
#include <Eigen/Dense>
#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <unistd.h>

// DVL A50 WL-21035-2 (assuming standard) long term sensor accuracy +-1.01%
// https://yostlabs.com/product/3-space-nano/ - "sensor assist" AHRS on A50, specs on page 
namespace DoryLoc {
    class Node {
        public:

        // DoryLoc::Localizer *filter;
        DoryLoc::ParticleFilter *filter;
        ros::NodeHandle nh;
        ros::Subscriber dvlSub;
        ros::Subscriber pixhawkSub;
        ros::Subscriber testSub;
        ros::Publisher meanParticlePub;
        ros::Publisher allParticlePub;
        ros::Publisher allParticleMarkerPub;
        Eigen::Vector4d lastOdom;
        std::random_device rd;
        std::mt19937 mt;
        std::normal_distribution<double> pixhawkDist;
        std::normal_distribution<double> dvlDist;


        void dvlOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);

        void pixhawkOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);

        void testingCallback(const nav_msgs::Odometry::ConstPtr& odom);

        // Node(ros::NodeHandle &n, Localizer *filter, std::normal_distribution<double> pixhawkDistribution, std::normal_distribution<double> dvlDistribution);
        Node(ParticleFilter *filter, std::normal_distribution<double> pixhawkDistribution, std::normal_distribution<double> dvlDistribution, bool testingMode);

        void loop();
    };
}
#include "../include/dory_localization_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
/*

/home/active_stereo/catkin_ws/src/ENGR-4200-Project/dory_localization/src/dory_localization_node.cpp:47:82:   
required from here /usr/include/boost/function/function_template.hpp:231:11: 
error: no match for call to 
(boost::_mfi::mf1
    <void, DoryLoc::Node, const nav_msgs::Odometry_<std::allocator<void> >&>
) (const boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void> > >&)

*/

// DVL A50 WL-21035-2 (assuming standard) long term sensor accuracy +-1.01%
// https://yostlabs.com/product/3-space-nano/ - "sensor assist" AHRS on A50, specs on page 
void DoryLoc::Node::dvlOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    geometry_msgs::Point pos = odom->pose.pose.position;
    std::cout << "got pt from dvl " << pos.x << ", " << pos.y << ", " << pos.z << std::endl;
    // double noise = dvlDist(mt);
    // double x = pos.x, y = pos.y, z = pos.z, yaw = odom->pose.pose.orientation.z;
    // x += noise;
    // y += noise;
    // z += noise;
    // yaw += noise;
    // std::vector<double> odomVec {x, y, z, yaw};
    // this->pf.weight(odomVec);
}

void DoryLoc::Node::pixhawkOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    geometry_msgs::Point pos = odom->pose.pose.position;

    // double noise = pixhawkDist(mt);
    double deltax = pos.x - lastOdom(0);
    double deltay = pos.y - lastOdom(1); 
    double deltaz = pos.z - lastOdom(2);
    auto msgquat = odom->pose.pose.orientation; 
    tf2::Quaternion tfquat;
    tf2::convert(msgquat , tfquat);
    tf2::Matrix3x3 m(tfquat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // x += noise;
    // y += noise;
    // z += noise;
    // yaw += noise;
    double cosLastYaw = cos(lastOdom(3));
    double sinLastYaw = sin(lastOdom(3));
    std::vector<double> odomVec {
        deltax * cosLastYaw + deltay * sinLastYaw,
        deltay * cosLastYaw - deltax * sinLastYaw,
        0.,
        yaw - lastOdom(3)
    };
    this->pf.predict(odomVec);
    this->lastOdom(0) = pos.x;
    this->lastOdom(1) = pos.y;
    this->lastOdom(2) = pos.z;
    this->lastOdom(3) = yaw; 
}

DoryLoc::Node::Node(std::mt19937 gen, std::normal_distribution<double> pixhawkDistribution, std::normal_distribution<double> dvlDistribution)
    : mt(gen)
    , pixhawkDist(pixhawkDistribution)
    , dvlDist(dvlDistribution)
    , pf()
    , n()
{
    this->mt = gen;
    this->pixhawkDist = pixhawkDistribution;
    this->dvlDist = dvlDistribution;

    // dvlSub = n.subscribe<nav_msgs::Odometry>("DVL_ODOM", 10, dvlOdomCallback); 
    this->dvlSub = n.subscribe<nav_msgs::Odometry>("odom", 1000, &Node::dvlOdomCallback, this); 

    // pixhawkSub = n.subscribe<nav_msgs::Odometry>("ROV_ODOMETRY", 10, pixhawkOdomCallback);
    this->pixhawkSub = n.subscribe<nav_msgs::Odometry>("odom", 1000, &Node::pixhawkOdomCallback, this);

    this->meanParticlePub = n.advertise<geometry_msgs::PoseStamped>("mean_particle", 100);
}


void DoryLoc::Node::loop() {
    auto time = ros::Time::now();

    // [x, y, z, yaw]
    auto mean = this->pf.getMeanParticle();
    geometry_msgs::PoseStamped meanMsg;
    meanMsg.pose.position.x = mean.at(0);
    meanMsg.pose.position.y = mean.at(1);
    meanMsg.pose.position.z = mean.at(2);
    tf2::Quaternion meanQuat;
    meanQuat.setEuler(0, 0, mean.at(3));
    meanMsg.pose.orientation = tf2::toMsg(meanQuat);
    meanMsg.header.stamp = time; 
    meanMsg.header.frame_id = "odom";
    meanParticlePub.publish(meanMsg);

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
        node.loop();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
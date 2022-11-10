#include "../include/dory_localization_node.hpp"

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
    double noise = dvlDist(mt);
    double x = pos.x, y = pos.y, z = pos.z, yaw = odom->pose.pose.orientation.z;
    x += noise;
    y += noise;
    z += noise;
    yaw += noise;
    std::vector<double> odomVec {x, y, z, yaw};
    this->pf.weight(odomVec);
}

void DoryLoc::Node::pixhawkOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    geometry_msgs::Point pos = odom->pose.pose.position;
    double noise = pixhawkDist(mt);
    double x = pos.x, y = pos.y, z = pos.z, yaw = odom->pose.pose.orientation.z;
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
    this->pf.predict(odomVec);
    this->lastOdom(0) = x;
    this->lastOdom(1) = y;
    this->lastOdom(2) = z;
    this->lastOdom(3) = yaw; 
}

DoryLoc::Node::Node(std::mt19937 gen, std::normal_distribution<double> pixhawkDistribution, std::normal_distribution<double> dvlDistribution) {
    this->mt = gen;
    this->pixhawkDist = pixhawkDistribution;
    this->dvlDist = dvlDistribution;

    ros::NodeHandle n;

    // dvlSub = n.subscribe<nav_msgs::Odometry>("DVL_ODOM", 10, dvlOdomCallback); 
    this->dvlSub = n.subscribe<nav_msgs::Odometry>("odom", 1000, &Node::dvlOdomCallback, this); 

    // pixhawkSub = n.subscribe<nav_msgs::Odometry>("ROV_ODOMETRY", 10, pixhawkOdomCallback);
    this->pixhawkSub = n.subscribe<nav_msgs::Odometry>("odom", 1000, &Node::pixhawkOdomCallback, this);
}


void DoryLoc::Node::loop() {

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
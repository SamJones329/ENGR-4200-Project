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
    double x = pos.x, y = pos.y, z = pos.z; 
    auto msgquat = odom->pose.pose.orientation; 
    tf2::Quaternion tfquat;
    tf2::convert(msgquat , tfquat);
    tf2::Matrix3x3 m(tfquat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
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
    double deltax = pos.x - lastOdom(0);
    double deltay = pos.y - lastOdom(1); 
    double deltaz = pos.z - lastOdom(2);
    auto msgquat = odom->pose.pose.orientation; 
    tf2::Quaternion tfquat;
    tf2::convert(msgquat , tfquat);
    tf2::Matrix3x3 m(tfquat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    deltax += noise;
    deltay += noise;
    deltaz += noise;
    yaw += noise;
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
    this->allParticlePub = n.advertise<geometry_msgs::PoseArray>("particles", 100);
    this->allParticleMarkerPub = n.advertise<visualization_msgs::MarkerArray>("particle_markers", 100);
}


void DoryLoc::Node::loop() {

    if(pf.moving) {
        pf.resample();
    }

    auto time = ros::Time::now();

    // [x, y, z, yaw]
    auto mean = this->pf.getMeanParticle();
    geometry_msgs::PoseStamped meanMsg;
    meanMsg.pose.position.x = mean.at(0);
    meanMsg.pose.position.y = mean.at(1);
    meanMsg.pose.position.z = mean.at(2);
    tf2::Quaternion q;
    q.setEuler(0, 0, mean.at(3));
    meanMsg.pose.orientation = tf2::toMsg(q);
    meanMsg.header.stamp = time; 
    meanMsg.header.frame_id = "odom";
    meanParticlePub.publish(meanMsg);

    auto particles = pf.getParticles();
    geometry_msgs::PoseArray particlesMsg;
    visualization_msgs::MarkerArray particleMarkersMsg;
    particlesMsg.header.stamp = time;
    particlesMsg.header.frame_id = "odom";
    int idx = 0;
    for(auto particle : particles) {
        geometry_msgs::Pose p;
        p.position.x = particle[0];
        p.position.y = particle[1];
        p.position.z = particle[2];
        q.setEuler(0, 0, particle.at(3));
        p.orientation = tf2::toMsg(q);
        particlesMsg.poses.push_back(p);
        visualization_msgs::Marker m;
        m.pose = p;
        m.header.frame_id = "odom";
        m.header.stamp = time;
        m.id = idx++;
        m.ns = "weights";
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.color.a = 0.5;
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 1.0;
        double scale = 0.01 + 2. * particle[4];
        m.scale.x = scale;
        m.scale.y = scale;
        m.scale.z = 0.05;
        particleMarkersMsg.markers.push_back(m);
    }
    allParticlePub.publish(particlesMsg);
    allParticleMarkerPub.publish(particleMarkersMsg);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "dory_localization");
    std::random_device rd;
    std::mt19937 mt(rd());
    std::normal_distribution<double> pixhawkDist{0., 0.05};
    std::normal_distribution<double> dvlDist{0., 0.01};
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
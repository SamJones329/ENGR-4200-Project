#include "../include/dory_localization_node.hpp"

// DVL A50 WL-21035-2 (assuming standard) long term sensor accuracy +-1.01%
// https://yostlabs.com/product/3-space-nano/ - "sensor assist" AHRS on A50, specs on page 
void DoryLoc::Node::dvlOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    geometry_msgs::Point pos = odom->pose.pose.position;
    double x = pos.x, y = pos.y, z = pos.z, yaw = odom->pose.pose.orientation.z; 
    std::vector<double> odomVec {x, y, z, yaw};
    this->filter->update(odomVec);
}

void DoryLoc::Node::pixhawkOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    geometry_msgs::Point pos = odom->pose.pose.position;

    double deltax = pos.x - lastOdom(0);
    double deltay = pos.y - lastOdom(1); 
    double deltaz = pos.z - lastOdom(2);
    double yaw = odom->pose.pose.orientation.z;
    double cosLastYaw = cos(lastOdom(3));
    double sinLastYaw = sin(lastOdom(3));
    std::vector<double> odomVec {
        deltax * cosLastYaw + deltay * sinLastYaw,
        deltay * cosLastYaw - deltax * sinLastYaw,
        0.,
        yaw - lastOdom(3)
    };
    this->filter->predict(odomVec);
    this->lastOdom(0) = pos.x;
    this->lastOdom(1) = pos.y;
    this->lastOdom(2) = pos.z;
    this->lastOdom(3) = yaw; 
}

void DoryLoc::Node::testingCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    // simulate pixhawk callback
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
    this->filter->predict(odomVec);
    this->lastOdom(0) = pos.x;
    this->lastOdom(1) = pos.y;
    this->lastOdom(2) = pos.z;
    this->lastOdom(3) = yaw; 


    // simulate dvl callback
    pos = odom->pose.pose.position;
    noise = dvlDist(mt);
    double x = pos.x, y = pos.y, z = pos.z; 
    msgquat = odom->pose.pose.orientation; 
    tf2::convert(msgquat , tfquat);
    m = tf2::Matrix3x3(tfquat);
    m.getRPY(roll, pitch, yaw);
    x += noise;
    y += noise;
    z += noise;
    yaw += noise;
    odomVec = std::vector<double> {x, y, z, yaw};
    this->filter->update(odomVec);
}

DoryLoc::Node::Node(Localizer<4> *filter, std::normal_distribution<double> pixhawkDistribution, std::normal_distribution<double> dvlDistribution, bool testingMode=false)
    : nh()
    , rd()
    , mt(rd())
    , pixhawkDist(pixhawkDistribution)
    , dvlDist(dvlDistribution)
{
    this->filter = filter;

    this->pixhawkDist = pixhawkDistribution;
    this->dvlDist = dvlDistribution;

    if(testingMode) {
        this->testSub = nh.subscribe<nav_msgs::Odometry>("odom", 1000, &Node::testingCallback, this); 
    } else {

        // ROV_ODOMETRY we think is the filtered data combining DVL and Pixhawk IMU (at least on Nemo)
        // SCALED_IMU2 is the IMU mavlink msg
        // provided the DVL is configured correctly and the correct params are set, 
        // LOCAL_POSITION_NED on mavlink is the result of filtering SCALED_IMU2 and VISION_POSITION_DELTA
        // with an EKF  
        pixhawkSub = nh.subscribe<nav_msgs::Odometry>("ROV_ODOMETRY", 1000, &Node::pixhawkOdomCallback, this);
        
        // gotten directly from DVL via TCP
        dvlSub = nh.subscribe<nav_msgs::Odometry>("DVL_ODOM", 1000, &Node::dvlOdomCallback, this); 
    }


    this->meanParticlePub = nh.advertise<geometry_msgs::PoseStamped>("mean_particle", 100);
    this->allParticlePub = nh.advertise<geometry_msgs::PoseArray>("particles", 100);
    this->allParticleMarkerPub = nh.advertise<visualization_msgs::MarkerArray>("particle_markers", 100);
}


void DoryLoc::Node::loop() {
    auto time = ros::Time::now();

    // [x, y, z, yaw]
    auto mean = this->filter->getBelief();
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

    auto particles = filter->getParticles();
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
    ros::NodeHandle n("~");
    std::normal_distribution<double> pixhawkDist{0., 0.05};
    std::normal_distribution<double> dvlDist{0., 0.01};

    std::string mode;
    bool testing = false;
    n.getParam("mode", mode);
    if(!mode.compare("test")) {
        testing = true;
        std::cout << "Testing mode selected" << std::endl;
    }

    // std::string algo;
    // DoryLoc::Localizer<4>* filter;
    // n.getParam("algo", algo);
    // if(!algo.compare("particle_filter")) {
    //     std::cout << "Particle Filter selected" << std::endl;
    //     DoryLoc::ParticleFilter f = DoryLoc::ParticleFilter();
    //     filter = &f;
    // } else {
    //     ROS_ERROR("No localization algorithm selected, exiting...");
    //     exit(0);
    // }
    // DoryLoc::Node node(filter, pixhawkDist, dvlDist, testing);

    auto filter = DoryLoc::ParticleFilter();
    DoryLoc::Node node(&filter, pixhawkDist, dvlDist, testing);
    
    // auto testSub = n.subscribe<nav_msgs::Odometry>("odom", 1000, &DoryLoc::Node::testingCallback, &node); 

    /*
        DVL_ODOM nav_msgs.Odometry {position: {x, y, z}, oritatation: {x (roll), y (pitch), z (yaw)} - Fused integration velocity and IMU
        ROV_ODOMETRY nav_msgs.Odometry {} - Pixhawk odom

        DVL_DOPPLER navg_msgs.Odometry {xvel, yvel, zvel} - Raw Velocities
        don't know raw depth message and raw IMU from DVL
        altimeter ping1d sensor on bluerobotics website

        figure out if using the MavLink messages directly
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
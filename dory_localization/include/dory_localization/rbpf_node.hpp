#ifndef DORY_RBPF_NODE_H
#define DORY_RBPF_NODE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <dory_localization/rb_particle_filter.hpp>
#include <Eigen/Dense>
#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <unistd.h>
#include <iostream>

namespace DoryLoc {
    class RBPFNode {
    private:

        RBPF filter;
        ros::NodeHandle nh;
        ros::Subscriber imuSub;
        ros::Subscriber dvlSub;
        ros::Subscriber dvlVelSub;
        ros::Publisher dvlOdomPub;
        ros::Publisher dvlVisPub;
        ros::Publisher beliefPosePub;
        ros::Publisher beliefOdomPub;
        ros::Publisher particlesPub;
        ros::Publisher particleMarkersPub;
        ros::Publisher dvlAccelPub;
        ros::Publisher imuAccelPub;

        bool dvlInitSet = false;
        double dvlVelTimeSec;
        double dvlInitX;
        double dvlInitY;
        double dvlInitZ;
        double dvlLastVelX = 0;
        double dvlLastVelY = 0;
        double dvlLastVelZ = 0;

        uint32_t getRosTimeMs() {
            const ros::Time time = ros::Time::now();
            return time.sec * 1000 + time.nsec / 1000000;
        }

        void imuCallback(const nav_msgs::Odometry::ConstPtr& sensorReadings) {
            vector<double> u; // {xacc, yacc, zacc, xangvel, yangvel, zangvel} check that values
            std::cout << "callback w/ vals {" 
                << sensorReadings->pose.pose.position.x << ", "
                << sensorReadings->pose.pose.position.y << ", "
                << sensorReadings->pose.pose.position.z << ", "
                << sensorReadings->twist.twist.linear.x << ", "
                << sensorReadings->twist.twist.linear.y << ", "
                << sensorReadings->twist.twist.linear.z << "}"
                << std::endl;
            u.push_back(sensorReadings->pose.pose.position.x);
            u.push_back(sensorReadings->pose.pose.position.y);
            u.push_back(sensorReadings->pose.pose.position.z);
            u.push_back(sensorReadings->twist.twist.linear.x);
            u.push_back(sensorReadings->twist.twist.linear.y);
            u.push_back(sensorReadings->twist.twist.linear.z);
            // ideally would get timestamp like this sourced from SCALED_IMU2 mavlink message but is not set up to do that
            // uint32_t timestamp_ms = sensorReadings->header.stamp.sec * 1000 + sensorReadings->header.stamp.nsec / 1000000;

            nav_msgs::Odometry imuAccel;
            imuAccel.pose.pose.position.x = u[0] * mG_TO_mps2;
            imuAccel.pose.pose.position.y = u[1] * mG_TO_mps2;
            imuAccel.pose.pose.position.z = u[2] * mG_TO_mps2 + 9.81;
            imuAccel.header.stamp = ros::Time::now();
            imuAccelPub.publish(imuAccel);
            std::cout << "imu callback" << std::endl;
            
            filter.predict(u, getRosTimeMs());
        }

        void dvlCallback(const nav_msgs::Odometry::ConstPtr& odom) {
            // dvl fused odometry visualizaation
            geometry_msgs::PoseStamped odomVis;
            nav_msgs::Odometry odomFixed;
            odomFixed.header.frame_id = "world";
            odomVis.header.frame_id = "world";
            odomVis.pose.position = odom->pose.pose.position;
            auto eulerAngles = odom->pose.pose.orientation;
            tf2::Quaternion tfquat;
            tfquat.setEuler(eulerAngles.z, eulerAngles.y, eulerAngles.x);
            geometry_msgs::Quaternion odomQuat;
            tf2::convert(tfquat, odomQuat);
            odomVis.pose.orientation = odomQuat;
            if(!dvlInitSet) {
                dvlInitX = odom->pose.pose.position.x;
                dvlInitY = odom->pose.pose.position.y;
                dvlInitZ = odom->pose.pose.position.z;
                dvlInitSet = true;
            }
            odomVis.pose.position.x -= dvlInitX;
            odomVis.pose.position.y -= dvlInitY;
            odomVis.pose.position.z -= dvlInitZ;

            odomFixed.pose.pose = odomVis.pose;
            dvlVisPub.publish(odomVis);
            dvlOdomPub.publish(odomFixed);
            
            // pass dvl odom as measurement until can figure out how to use DVL measurements directly
        }

        void dvlVelCallback(const nav_msgs::Odometry::ConstPtr& odom) {
            nav_msgs::Odometry dvlAccel;
            double nextDvlTime = 0.001 * getRosTimeMs();
            double dTimeRecip = 1 / (nextDvlTime - dvlVelTimeSec);
            dvlVelTimeSec = nextDvlTime;
            dvlAccel.pose.pose.position.x = (odom->twist.twist.linear.x - dvlLastVelX) * dTimeRecip;
            dvlAccel.pose.pose.position.y = (odom->twist.twist.linear.y - dvlLastVelY) * dTimeRecip;
            dvlAccel.pose.pose.position.z = (odom->twist.twist.linear.z - dvlLastVelY) * dTimeRecip;
            dvlLastVelX = odom->twist.twist.linear.x;
            dvlLastVelY = odom->twist.twist.linear.y;
            dvlLastVelZ = odom->twist.twist.linear.z;
            std::cout << "last vels: {" << dvlLastVelX << ", " << dvlLastVelY << ", " << dvlLastVelZ << "}" << std::endl; 
            std::cout << "accels: {" 
                << dvlAccel.pose.pose.position.x << ", "
                << dvlAccel.pose.pose.position.y << ", "
                << dvlAccel.pose.pose.position.z << "}" << std::endl;
            dvlAccel.header.stamp = ros::Time::now();
            dvlAccelPub.publish(dvlAccel);
        }

    public:
        RBPFNode() 
        : nh()
        , filter(false) 
        , imuSub(nh.subscribe<nav_msgs::Odometry>("SCALED_IMU2", 1000, &RBPFNode::imuCallback, this))
        , dvlSub(nh.subscribe<nav_msgs::Odometry>("DVL_ODOM", 1000, &RBPFNode::dvlCallback, this))
        , dvlVelSub(nh.subscribe<nav_msgs::Odometry>("DVL_DOPPLER", 1000, &RBPFNode::dvlVelCallback, this))
        , dvlOdomPub(nh.advertise<nav_msgs::Odometry>("DVL_ODOM_QUAT", 100))
        , dvlVisPub(nh.advertise<geometry_msgs::PoseStamped>("DVL_ODOM_VIS", 100))
        , beliefPosePub(nh.advertise<geometry_msgs::PoseStamped>("mean_particle_pose", 100))
        , beliefOdomPub(nh.advertise<nav_msgs::Odometry>("mean_particle", 100))
        , particlesPub(nh.advertise<geometry_msgs::PoseArray>("particles", 100))
        , particleMarkersPub(nh.advertise<visualization_msgs::MarkerArray>("particle_markers", 100))
        , dvlAccelPub(nh.advertise<nav_msgs::Odometry>("DVL_ACCEL", 100))
        , imuAccelPub(nh.advertise<nav_msgs::Odometry>("IMU_ACCEL", 100))
        {
            std::cout << "Hello from RBPF Node" << std::endl;
            filter.initTime(getRosTimeMs());
        }

        void loop() {
            std::cout << "looping" << std::endl;
            auto time = ros::Time::now();

            auto mean = filter.getBelief();
            geometry_msgs::PoseStamped meanMsg;
            meanMsg.pose.position.x = mean(0);
            meanMsg.pose.position.y = mean(1);
            meanMsg.pose.position.z = mean(2);
            tf2::Quaternion q;
            q.setEuler(mean(4), mean(3), mean(5));
            meanMsg.pose.orientation = tf2::toMsg(q);
            meanMsg.header.stamp = time; 
            meanMsg.header.frame_id = "odom";
            beliefPosePub.publish(meanMsg);

            nav_msgs::Odometry odomBelief;
            odomBelief.header.frame_id = "odom";
            odomBelief.pose.pose = meanMsg.pose;
            beliefOdomPub.publish(odomBelief);

            // auto particles = filter.getParticles();
            // geometry_msgs::PoseArray particlesMsg;
            // visualization_msgs::MarkerArray particleMarkersMsg;
            // particlesMsg.header.stamp = time;
            // particlesMsg.header.frame_id = "odom";
            // int idx = 0;
            // for(int i = 0; i < particles.rows(); i++) {// } auto particle : particles) {
            //     auto particle = particles.row(i);
            //     geometry_msgs::Pose p;
            //     p.position.x = particle(0);
            //     p.position.y = particle(1);
            //     p.position.z = particle(2);
            //     q.setEuler(particle(3), particle(4), particle(5));
            //     p.orientation = tf2::toMsg(q);
            //     particlesMsg.poses.push_back(p);
            //     visualization_msgs::Marker m;
            //     m.pose = p;
            //     m.header.frame_id = "odom";
            //     m.header.stamp = time;
            //     m.id = idx++;
            //     m.ns = "weights";
            //     m.type = visualization_msgs::Marker::SPHERE;
            //     m.action = visualization_msgs::Marker::ADD;
            //     m.color.a = 0.5;
            //     m.color.r = 0.0;
            //     m.color.g = 1.0;
            //     m.color.b = 1.0;
            //     double scale = 0.01 + 2. * particle(6);
            //     m.scale.x = scale;
            //     m.scale.y = scale;
            //     m.scale.z = 0.05;
            //     particleMarkersMsg.markers.push_back(m);
            // }
            // particlesPub.publish(particlesMsg);
            // particleMarkersPub.publish(particleMarkersMsg);
        }
    };
}

#endif // DORY_RBPF_NODE
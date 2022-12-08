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
        ros::Publisher dvlVisPub;
        ros::Publisher beliefPub;
        ros::Publisher particlesPub;
        ros::Publisher particleMarkersPub;

        bool dvlInitSet = false;
        double dvlInitX;
        double dvlInitY;
        double dvlInitZ;

        void imuCallback(const nav_msgs::Odometry::ConstPtr& sensorReadings) {
            vector<uint16_t> u; // {xacc, yacc, zacc, xangvel, yangvel, zangvel} check that values
            u.push_back(sensorReadings->pose.pose.position.x);
            u.push_back(sensorReadings->pose.pose.position.y);
            u.push_back(sensorReadings->pose.pose.position.z);
            u.push_back(sensorReadings->pose.pose.orientation.x);
            u.push_back(sensorReadings->pose.pose.orientation.y);
            u.push_back(sensorReadings->pose.pose.orientation.z);
            uint32_t timestamp_ms = sensorReadings->header.stamp.sec * 1000;
            filter.predict(u, timestamp_ms);
        }

        void dvlCallback(const nav_msgs::Odometry::ConstPtr& odom) {
            geometry_msgs::PoseStamped odomVis;
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
            dvlVisPub.publish(odomVis);
            // dvlOdomPub.publish
        }

    public:
        RBPFNode() 
        : nh()
        , filter() 
        , imuSub(nh.subscribe<nav_msgs::Odometry>("SCALED_IMU2", 1000, &RBPFNode::imuCallback, this))
        , dvlSub(nh.subscribe<nav_msgs::Odometry>("DVL_ODOM", 1000, &RBPFNode::dvlCallback, this))
        , dvlVisPub(nh.advertise<geometry_msgs::PoseStamped>("DVL_ODOM_VIS", 100))
        , beliefPub(nh.advertise<geometry_msgs::PoseStamped>("mean_particle", 100))
        , particlesPub(nh.advertise<geometry_msgs::PoseArray>("particles", 100))
        , particleMarkersPub(nh.advertise<visualization_msgs::MarkerArray>("particle_markers", 100))
        {
            std::cout << "Hello from RBPF Node" << std::endl;
        }

        void loop() {

        }
    };
}

#endif // DORY_RBPF_NODE
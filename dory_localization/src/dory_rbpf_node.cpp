#include <dory_localization/rbpf_node.hpp>

int main(int argc, char **argv) {

    ros::init(argc, argv, "dory_rbpf_localization");
    DoryLoc::RBPFNode node;

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
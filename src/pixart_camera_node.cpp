#include <stdlib.h>
#include <string>
#include <iomanip>
#include <sstream>

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pixart/raw_point.h>
#include <pixart/world_point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "pixart_camera.h"

using namespace std;
using namespace Eigen;

int main (int argc, char** argv) {
    /* Initialize ROS */
    ros::init(argc, argv, "pixart_camera_node");
    ROS_INFO("Node pixart_camera_node connected to roscore");
    ros::NodeHandle node("~");

    pixart_camera cam(node);

    /* Initialize raw data subscriber */
    ros::Subscriber sub = node.subscribe("/pixart_raw", 100,
        &pixart_camera::raw_callback, &cam);

    /* Main loop */
    //ros::MultiThreadedSpinner spinner;
    //spinner.spin();
    ros::Rate rate(400);
    unsigned long n = 0;
    while (ros::ok()) {
        ros::spinOnce();
        if (!(n % 400))
            cam.sendframe();
        rate.sleep();
        n++;
    }
    ROS_INFO("Node pixart_camera_node terminated");
}
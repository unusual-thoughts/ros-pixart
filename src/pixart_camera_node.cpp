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

using namespace std;
using namespace Eigen;

class pixart_camera {
    public:
        pixart_camera(ros::NodeHandle &n) {
            double roll, pitch, yaw, f_u, f_v, u0, v0, gamma;
            /* Get camera intrinsic and extrinsic parameters */
            n.param("id", id, 0);
            n.param("robot_height", robot_height, 0.);
            n.param("x", x, 0.);
            n.param("y", y, 0.);
            n.param("z", z, 0.);
            n.param("roll", roll, 0.);
            n.param("pitch", pitch, 0.);
            n.param("yaw", yaw, 0.);
            n.param("f_u", f_u, 0.);
            n.param("f_v", f_v, 0.);
            n.param("u0", u0, 0.);
            n.param("v0", v0, 0.);
            n.param("gamma", gamma, 0.);

            /* Build rotation matrix */
            R = AngleAxisd(roll, Vector3d::UnitX())
                * AngleAxisd(pitch,  Vector3d::UnitY())
                * AngleAxisd(yaw, Vector3d::UnitZ());

            //cout << R << endl;

            /* Build camera matrix */
            Matrix3d K;
            K << f_u, gamma, u0, 0., f_v, v0, 0., 0., 1.;
            Kinv = K.inverse();

            //cout << Kinv << endl;

            /* Convert id to string */
            ostringstream str;
            str << std::setfill('0') << setw(2) << id;
            id_str = str.str();

            /* Initialize tf transform and broadcaster */
            //tf::TransformBroadcaster br;
            //tf::Transform frame;
            frame.setOrigin(tf::Vector3(x, y, z));
            tf::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            frame.setRotation(q);

            /* Initialize publisher */
            pub = n.advertise<pixart::world_point>(
                "/pixart_world", 1);

        };
        void unproject(const pixart::raw_point pt) {
            //cout << "hello" << endl;
            if (pt.camera_id == id) {
                pixart::world_point sol;
                Vector3d pp = Kinv * Vector3d(pt.x, pt.y, 1);
                double z_c = (robot_height - z) / (R.row(2) * pp);
                sol.pt.point.x = x + z_c * R.row(0) * pp;
                sol.pt.point.y = y + z_c * R.row(1) * pp;
                sol.pt.point.z = robot_height;   
                sol.pt.header.frame_id = "world";
                //sol.pt.point.header.seq =;
                sol.pt.header.stamp = pt.stamp;
                sol.camera_id = id;
                sol.point_id = pt.point_id;
                pub.publish(sol);
            }
        };
        void sendframe(void) {
            br.sendTransform(tf::StampedTransform(frame, ros::Time::now(),
                "world", "camera" + id_str));
        }
    private:
        int id;
        string id_str;
        double robot_height, x, y, z;
        Matrix3d R;
        Matrix3d Kinv;
        ros::Publisher pub;
        tf::TransformBroadcaster br;
        tf::Transform frame;
};

int main (int argc, char** argv) {
    /* Initialize ROS */
    ros::init(argc, argv, "pixart_camera_node");
    ROS_INFO("Node pixart_camera_node connected to roscore");
    ros::NodeHandle node("~");

    pixart_camera cam(node);

    /* Initialize raw data subscriber */
    ros::Subscriber sub = node.subscribe("/pixart_raw", 100,
        &pixart_camera::unproject, &cam);



    /* Main loop */

    //ros::MultiThreadedSpinner spinner;
    //spinner.spin();
    ros::Rate rate(100);
    while (ros::ok()) {
        //ROS_INFO("spin");
        ros::spinOnce();
        cam.sendframe();
        rate.sleep();
    }
    ROS_INFO("Node pixart_camera_node terminated");
}
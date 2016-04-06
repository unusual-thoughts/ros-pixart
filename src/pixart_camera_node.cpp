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


using namespace std;
using namespace Eigen;

class pixart_camera {
    public:
        pixart_camera(ros::NodeHandle &n) {
            double roll, pitch, yaw, f_u, f_v, u0, v0, gamma;
            /* Get camera intrinsic and extrinsic parameters */
            n.param("id", id, 0);
            n.param("/robot_height", robot_height, 0.);
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
            frame.setOrigin(tf::Vector3(x, y, z));
            tf::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            frame.setRotation(q);

            /* Draw outline of viewing area */
            visualization_msgs::Marker rect, origin;
            pixart::raw_point pt;
            rect.header.frame_id = "world";
            origin.header.frame_id = "world";
            rect.id = 0;
            origin.id = 1;
            rect.type = visualization_msgs::Marker::LINE_LIST;
            origin.type = visualization_msgs::Marker::SPHERE;


            geometry_msgs::Point camera_center, world_point;
            camera_center.x = x;
            camera_center.y = y;
            camera_center.z = z;
            
            pt.x = 0;
            pt.y = 0;
            world_point = unproject(pt).pt.point;
            rect.points.push_back(world_point);
            rect.points.push_back(camera_center);
            rect.points.push_back(world_point);
            pt.x = 1023;
            world_point = unproject(pt).pt.point;
            rect.points.push_back(world_point);
            rect.points.push_back(world_point);
            rect.points.push_back(camera_center);
            rect.points.push_back(world_point);
            pt.y = 767;
            world_point = unproject(pt).pt.point;
            rect.points.push_back(world_point);
            rect.points.push_back(world_point);
            rect.points.push_back(camera_center);
            rect.points.push_back(world_point);
            pt.x = 0;
            world_point = unproject(pt).pt.point;
            rect.points.push_back(world_point);
            rect.points.push_back(world_point);
            rect.points.push_back(camera_center);
            rect.points.push_back(world_point);
            rect.points.push_back(rect.points[0]);

            rect.pose.position.x = 0.;
            rect.pose.position.y = 0.;
            rect.pose.position.z = 0.;
            rect.pose.orientation.x = 0.0;
            rect.pose.orientation.y = 0.0;
            rect.pose.orientation.z = 0.0;
            rect.pose.orientation.w = 1.0;
            origin.pose.position.x = rect.points[0].x;
            origin.pose.position.y = rect.points[0].y;
            origin.pose.position.z = rect.points[0].z;
            origin.pose.orientation = rect.pose.orientation;
            rect.scale.x = 0.01;
            rect.scale.y = 0.01;
            rect.scale.z = 0.01;
            origin.scale.x = 0.1;
            origin.scale.y = 0.1;
            origin.scale.z = 0.1;
            rect.color.a = 1.0;
            rect.color.r = 0.0;
            rect.color.g = 1.0;
            rect.color.b = 0.0;
            origin.color = rect.color;
            origin.color.r = 1.0;

            outline.markers.push_back(rect);
            outline.markers.push_back(origin);
            outline_pub = n.advertise<visualization_msgs::MarkerArray>(
                "/camera_outline" + id_str, 1);

            /* Initialize publisher */
            pub = n.advertise<pixart::world_point>(
                "/pixart_world", 1);

        };
        pixart::world_point unproject(const pixart::raw_point pt) {
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
            
            return (sol);
            
        };
        void raw_callback(const pixart::raw_point pt) {
            if (pt.camera_id == id) {
                pub.publish(unproject(pt));
            }
        };
        void sendframe(void) {
            br.sendTransform(tf::StampedTransform(frame, ros::Time::now(),
                "world", "camera" + id_str));
            outline_pub.publish(outline);
        };
    private:
        int id;
        string id_str;
        double robot_height, x, y, z;
        Matrix3d R;
        Matrix3d Kinv;
        ros::Publisher pub, outline_pub;
        tf::TransformBroadcaster br;
        tf::Transform frame;
        visualization_msgs::MarkerArray outline;

};

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
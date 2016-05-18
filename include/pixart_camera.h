#ifndef PIXART_CAMERA_H
#define PIXART_CAMERA_H

using namespace std;
using namespace Eigen;

class pixart_camera {
    public:
        pixart_camera(ros::NodeHandle &n);
        pixart::world_point unproject(const pixart::raw_point pt);
        void raw_callback(const pixart::raw_point pt);
        void sendframe(void);
        void draw_outline(void);

    private:
        int id;
        string id_str;
        bool viz;
        double robot_height, x, y, z;
        Matrix3d R;
        Matrix3d Kinv;
        ros::Publisher pub, outline_pub, pt_pub;
        tf::TransformBroadcaster br;
        tf::Transform frame;
        visualization_msgs::MarkerArray outline;
};

#endif
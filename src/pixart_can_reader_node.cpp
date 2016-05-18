#include <stdlib.h>

#include "ros/ros.h"
#include <pixart/raw_point.h>

#include <net/if.h>
//#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

/* Each box is allocated 8 IDs (2 per camera) */
/* Root ID preferably a multiple of 64 */
#define PIXART_ROOT_CAN_ID 0x100
/* 11 bit mask, lower 6 bits free (room for 64 different IDs) */
#define PIXART_CAN_MASK    0x7C0

#define MAX_CAN_FRAME_STR  256
#define CAN_TIMEOUT        100000

using namespace std;

/* Commands to set up virtual CAN interface for testing:
	sudo modprobe vcan
	sudo ip link add dev vcan0 type vcan
	sudo ip link set up vcan0
*/

/* Send test frame:
	cansend vcan0 100#03ff02ff00000000
	-> Decoded as: Camera 00 from box 00, Point0(1023, 767) and Point1(0, 0)
*/

/* Opens CAN socket */
int can_init(const char *ifname) {
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_filter filter;
	struct timeval timeout;
	int sock;

	/* Create raw CAN socket */
	if ((sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		ROS_ERROR("Failed to open CAN socket");
		return (-1);
	}

	/* Attach it to specified CAN interface */
	strcpy(ifr.ifr_name, ifname);
	ioctl(sock, SIOCGIFINDEX, &ifr);

	/* Filter which CAN ids are received */
	filter.can_id   = PIXART_ROOT_CAN_ID;
	filter.can_mask = PIXART_CAN_MASK;

	if (setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
		ROS_ERROR("Failed to set CAN socket filter");
		return (-2);
	}

	/* Add read timeout to socket options */
	timeout.tv_sec = 0;
	timeout.tv_usec = CAN_TIMEOUT;

	if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
		ROS_ERROR("Failed to set CAN read timeout");
		return (-3);
	}

	/* Bind the socket to the address */
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		ROS_ERROR("Failed to bind CAN socket");
		return (-4);
	}
	return (sock);
}

/* Print CAN frame debug info */
char *show_frame(char *str, const struct can_frame *frame) {
	int i;

	sprintf(str, "%x #", frame->can_id);
	for (i = 0; i < frame->can_dlc; i++) {
		sprintf(&str[strlen(str)], " %02x", frame->data[i]);
	}
	return (str);
}

int main (int argc, char** argv) {
    /* Initialize ROS */
    ros::init(argc, argv, "pixart_can_reader_node");
    ROS_INFO("Node pixart_can_reader_node connected to roscore");
    ros::NodeHandle node("~");

    /* Get CAN interface name from parameter */
    string ifname;
    node.param("ifname", ifname, string("can0"));
    ROS_INFO("The CAN interface is set to %s", ifname.c_str());

    /* Get debug flag from parameter */
    bool debug;
    node.param("debug", debug, false);
    ROS_INFO("The debug flag is set to %d", debug);

    /* Initialize the CAN socket */
    int cansock;
    if ((cansock = can_init(ifname.c_str())) < 0) {
		return (-1);
	}

    /* Initialize pixart_raw publisher */
    ros::Publisher pub = node.advertise<pixart::raw_point>(
        "/pixart_raw", 10);

	ssize_t nbytes;
	struct can_frame frame;
	char str[MAX_CAN_FRAME_STR];

	/* Main loop */
	while (ros::ok()) {
		/* Attempt to read from CAN socket */
		nbytes = read(cansock, &frame, sizeof(frame));
		if (nbytes > 0) {
			if (debug)
				ROS_INFO("Received frame: %s", show_frame(str, &frame));
			if (frame.can_dlc == 8) {
				pixart::raw_point pt1, pt2;
				/* Add time stamp */
				pt1.stamp = ros::Time::now();
				pt2.stamp = pt1.stamp;
				/* Extract camera id from CAN ID */
				pt1.camera_id = ((frame.can_id - PIXART_ROOT_CAN_ID) & 0x3E) >> 1;
				pt2.camera_id = pt1.camera_id;
				/* Frame contains points 0 and 1 if CAN ID even, 2 and 3 if odd */
				pt1.point_id = (frame.can_id & 0x1) << 1;
				pt2.point_id = pt1.point_id | 0x1;
				/* Assemble point coordinates */
				pt1.x = (frame.data[0] << 8) | (frame.data[1]);
				pt1.y = (frame.data[2] << 8) | (frame.data[3]);
				pt2.x = (frame.data[4] << 8) | (frame.data[5]);
				pt2.y = (frame.data[6] << 8) | (frame.data[7]);
				
				if (debug)
					ROS_INFO("Decoded as: Camera %02u from box %02u, Point%u(%4u, %4u) and Point%u(%4u, %4u)",
						pt1.camera_id, pt1.camera_id >> 2,
						pt1.point_id, pt1.x, pt1.y, pt2.point_id, pt2.x, pt2.y);

				/* First point should never be empty if it was sent by microcontroller */
				pub.publish(pt1);
				/* Second point could still be empty */
				if (pt2.x != 1023 || pt2.y != 1023)
					pub.publish(pt2);
			} else {
				ROS_ERROR("Wrong frame size %d", frame.can_dlc);
			}
		}
	}

    ROS_INFO("Node pixart_can_reader_node terminated");
}

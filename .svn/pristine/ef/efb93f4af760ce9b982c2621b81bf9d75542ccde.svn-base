#include <cstdlib>
#include <stdlib.h>     /* atoi */

#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>

int main(int argc, char **argv)
{
    if(argc<1){
        printf("Usage: rosrun ros_erle_cpp_examples_rc rc yaw\n");
        return -1;
    }

    ros::init(argc, argv, "mavros_rc_override");
    ros::NodeHandle n;


    int yaw = atoi(argv[1]);

    int rate = 100;
    ros::Rate r(rate);

    ros::Publisher rc_override_pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);

    mavros_msgs::OverrideRCIn msg_override;

    while (n.ok()){

        msg_override.channels[0] = 1500;
        msg_override.channels[1] = 1500;
        msg_override.channels[2] = 1500;
        msg_override.channels[3] = 2000;
        msg_override.channels[4] = 65535;
        msg_override.channels[5] = 65535;
        msg_override.channels[6] = 65535;
        msg_override.channels[7] = 65535;

        rc_override_pub.publish(msg_override);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

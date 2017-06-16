
#include <cstdlib>
#include <string>

#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher pub;
ros::Subscriber scanSub;
ros::Publisher local_pos_pub;

bool turnLeft=false;
bool wait=false;
bool follow=false;
ros::Time lastTime;
int waitCounter;

void processLaserScan(const sensor_msgs::LaserScan scan){

	int size = scan.ranges.size();

	bool wallLeft=false;
	double midRange = scan.ranges[320];

	mavros_msgs::OverrideRCIn msg;


	bool turnRight= false;
	for( int i = 300; i < 420; i++){
		if(!isnan(scan.ranges[i]) &&scan.ranges[i]<2.5){
			turnRight = true;
			break;
		}
	}
	for( int i = 320; i < 384; i++){
		if(!isnan(scan.ranges[i])&& scan.ranges[i]<5.0){
			wallLeft = true;
		}
	}
	if(!wallLeft && follow){
		waitCounter=waitCounter+1;
		if(waitCounter >40){
			ROS_ERROR("Start turn");
			msg.channels[0] = 1500;     //Roll
			msg.channels[1] = 1500;    //Pitch
			msg.channels[2] = 1500;   //Throttle
			msg.channels[3] = 1300;        //Yaw
			msg.channels[4] = 0;
			msg.channels[5] = 0;
			msg.channels[6] = 0;
			msg.channels[7] = 0;

			pub.publish(msg); 
			return;			
		}

	}
		
	
	if(!turnRight){

		/*
            geometry_msgs::PoseStamped pose;
    	    pose.pose.position.x = 20;
            pose.pose.position.y = 20;
   	    pose.pose.position.z = 100;
            local_pos_pub.publish(pose);
	
	    */

    	    msg.channels[0] = 1500;     //Roll
	    msg.channels[1] = 1450;    //Pitch
	    msg.channels[2] = 1500;   //Throttle
	    msg.channels[3] = 1500;        //Yaw
	    msg.channels[4] = 0;
	    msg.channels[5] = 0;
	    msg.channels[6] = 0;
	    msg.channels[7] = 0;

	    pub.publish(msg);
	

	}else{
	    ROS_ERROR("Turn right%f",scan.ranges[0]);

	    waitCounter=0;	
	    follow=true;
 	    msg.channels[0] = 1500;     //Roll
	    msg.channels[1] = 1500;    //Pitch
	    msg.channels[2] = 1500;   //Throttle
	    msg.channels[3] = 1600;        //Yaw
	    msg.channels[4] = 0;
	    msg.channels[5] = 0;
	    msg.channels[6] = 0;
	    msg.channels[7] = 0;

	    pub.publish(msg); 
    	   	
	}
}

int main(int argc, char **argv)
{

    int rate = 10;

    ros::init(argc, argv, "mavros_explore");
    ros::NodeHandle n;

    ros::Rate r(rate);

    ////////////////////////////////////////////
    ///////////////////EXPLORE//////////////////////
    ////////////////////////////////////////////

    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "LOITER";
    if(cl.call(srv_setMode)){
        ROS_ERROR("setmode send ok %d value:", srv_setMode.response.success);
    }else{
        ROS_ERROR("Failed SetMode");
        return -1;
    }


    pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);

    local_pos_pub = n.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    scanSub =n.subscribe<sensor_msgs::LaserScan>("/scan",10,processLaserScan);
  
    ros::spin();
}


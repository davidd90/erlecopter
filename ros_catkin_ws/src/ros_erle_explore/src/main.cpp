
#include <cstdlib>
#include <string>

#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher pub;
ros::Subscriber scanSub;
ros::Subscriber odomSub;
ros::Publisher local_pos_pub;

enum flightMode { find, follow, leftTurn_wait , leftTurn_turn };
flightMode fMode=find;

bool freeFlight= false;
ros::Time lastTime;
int waitCounter;
bool odomReceived=false;
bool distanceReached=false;
double xStart;
double yStart;
int nearestScan;
double freeFlightDistance;
double lastReceivedDistanceToWall;


void storeOdomData(const nav_msgs::Odometry odom){
	if(!odomReceived){
		odomReceived= true;
		xStart= odom.pose.pose.position.x;
		yStart= odom.pose.pose.position.y;
	}else{
		double x = odom.pose.pose.position.x;
		double y = odom.pose.pose.position.y;	
		if((pow(xStart-x,2)+pow(yStart-y,2))>freeFlightDistance){
			odomSub.shutdown();
			odomReceived=false;
			distanceReached=true;
		}
	}
}

void flyStraight()
{
	mavros_msgs::OverrideRCIn msg;
	msg.channels[0] = 1500;     //Roll
	msg.channels[1] = 1450;    //Pitch
	msg.channels[2] = 1500;   //Throttle
	msg.channels[3] = 1500;        //Yaw
	msg.channels[4] = 0;
	msg.channels[5] = 0;
	msg.channels[6] = 0;
	msg.channels[7] = 0;
	pub.publish(msg);
}

void turnRight()
{
	mavros_msgs::OverrideRCIn msg;
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

void turnLeft()
{
	mavros_msgs::OverrideRCIn msg;
	msg.channels[0] = 1500;   //Roll
	msg.channels[1] = 1500;   //Pitch
	msg.channels[2] = 1500;   //Throttle
	msg.channels[3] = 1390;   //Yaw
	msg.channels[4] = 0;
	msg.channels[5] = 0;
	msg.channels[6] = 0;
	msg.channels[7] = 0;

	pub.publish(msg); 
}

void processLaserScan(const sensor_msgs::LaserScan scan){

	int size = scan.ranges.size();

	bool wallLeft=false;



	
	double droneSpaceHalfAngle= std::atan(0.5/2.0);
	double droneSpaceHalfRange = int(droneSpaceHalfAngle/scan.angle_increment);

	if(fMode==find){
		// check if the space in front of the drone is free
		double droneSpaceHalfAngle= std::atan(0.5/2.0);
		double droneSpaceHalfRange = int(droneSpaceHalfAngle/scan.angle_increment);
		
		for( int i = size/2-droneSpaceHalfRange; i < size/2+droneSpaceHalfRange; i++){
			//if space in front of drone is not free then turn
			if(!isnan(scan.ranges[i]) &&scan.ranges[i]<2){	
			    	turnRight();
		  	
				fMode=follow;
				break;
			}
		}
		flyStraight();	
	}
	//check if the drone sees a wall to its left
	for( int i = size/2; i < size; i++){
		if(!isnan(scan.ranges[i])){	
			if(wallLeft==false){
				wallLeft = true;
				lastReceivedDistanceToWall=scan.ranges[i];
				nearestScan=i;
				continue;
			}
			if(scan.ranges[i]<lastReceivedDistanceToWall){
				lastReceivedDistanceToWall=scan.ranges[i];
				nearestScan=i;
			}
		}
	}

	if (fMode==follow){
		
		if(!wallLeft){
			fMode=leftTurn_wait;
			double angle = (nearestScan-size/2)*scan.angle_increment;
			freeFlightDistance= std::cos(angle)*lastReceivedDistanceToWall+0.5;
 			flyStraight();
			ros::NodeHandle n;
			odomSub =n.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom",10,storeOdomData);	
			return;
			
		}else{			
			double realDistanceToWall=std::sin((nearestScan-size/2)*scan.angle_increment)*lastReceivedDistanceToWall;

			ROS_ERROR("Distance: %f",realDistanceToWall);
			if(realDistanceToWall<1){
				    turnRight();
			}else{
				    flyStraight();			
			}


		}
	}else if (fMode==leftTurn_wait ){
		
		if(distanceReached){
			if (wallLeft){
				distanceReached=false;
				fMode=follow;
			}else{
				turnLeft();
			}
		}
		return;				
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


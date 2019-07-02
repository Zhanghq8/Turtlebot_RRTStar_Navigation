#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"
#include <sensor_msgs/LaserScan.h>
#include "RRTStar.h"

using namespace std;

class Controller
{
private:
    // const double v_max = 2.0;
    // const double w_max = 2.0;


    // control input for linear velocity
    double v_normal;  //velocity for avoiding obstacle
    double v_ao;
    double w;

    // pid gain parameters
    double k_p;
    double k_d;
    double k_i;

    // error dynamics
    double e_P;
    double e_I;
    double e_D;

    // accumulated error
    double E_k;
    double e_k;
    // previous error
    double e_k_previous;

    //blending parameter
    double alpha;
    int waypoint_cnt;

    //obstacle pos
    double laserdis[5];
    double obstacle_pos[5][2];
    double lasergain[5] = {1, 1, 1, 1, 1};
    std::vector<RRT::Vec2i> path_;

    geometry_msgs::Pose2D currentpos, goalpos;
    geometry_msgs::Twist controlinput;

    ros::NodeHandle nh_;
    ros::Subscriber currentpos_sub_;
    ros::Subscriber stop_sub_;
    ros::Subscriber laserpos_sub_;
    ros::Publisher controlinput_pub_;

public:
    Controller(ros::NodeHandle* nodehandle);
    void initSub(); 
    void initPub();

    void setgoalpos(double x, double y);
    void setpidgains(double p=3.0, double i=0.0, double d=0.05);
    void setvelocity(double x=0.2, double y=0.05);
    double quatoeuler_yaw(const nav_msgs::Odometry& odom);
    

    void currentposCallback(const nav_msgs::Odometry& odom);
    void laserCallback(const sensor_msgs::LaserScan& scan);
    void eventCallback(const nav_msgs::Odometry& odom); 

};

#endif

#include "../include/turtlebot_rrtstar/Controller.h"

Controller::Controller(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ 
// constructor
    ROS_INFO("In class constructor of Controller");
    RRT::RRTStar plan;
    path_ = plan.path;
    
    waypoint_cnt = 1;
    w = 0;
    e_P = 0;
    e_I = 0;
    e_D= 0;

    // accumulated error
    E_k = 0;
    // previous error
    e_k_previous = 0;

    //blending parameter
    alpha = 0.90;


    initSub();
    initPub();
    setgoalpos(path_[1].x, path_[1].y);
    setvelocity();
    setpidgains();
}

void Controller::setpidgains(double p, double i, double d)
{
    k_p = p;
    k_i = i;
    k_d = d;
}

void Controller::setvelocity(double x, double y) 
{
    v_normal = x;
    v_ao = y;
}

void Controller::setgoalpos(double x, double y)
{   
    cout << "Setting goal point as " << "(" << x << "," << y << ")." << endl;
    goalpos.x = x;
    goalpos.y = y;
}

double Controller::quatoeuler_yaw(const nav_msgs::Odometry& odom)
{
    tf::Quaternion qua(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3 mat(qua);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return yaw;
}

void Controller::initSub()
{
    ROS_INFO("Initializing Subscribers");  
    laserpos_sub_ =  nh_.subscribe("/scan", 1, &Controller::laserCallback,this);
    currentpos_sub_ = nh_.subscribe("/odom", 1, &Controller::currentposCallback,this);
    stop_sub_ = nh_.subscribe("/odom", 1, &Controller::eventCallback,this);
}

//member helper function to set up publishers;
void Controller::initPub()
{
    ROS_INFO("Initializing Publishers");
    controlinput_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1, true); 
}


void Controller::laserCallback(const sensor_msgs::LaserScan& scan)
{   
    // laser_angle[0] = scan.angle_min;
    // laser_angle[4] = scan.angle_max;
    // laser_angle[1] = scan.angle_min+160.0/640*(scan.angle_max-scan.angle_min);
    // laser_angle[2] = scan.angle_min+320.0/640*(scan.angle_max-scan.angle_min);
    // laser_angle[3] = scan.angle_min+480.0/640*(scan.angle_max-scan.angle_min);
    double laser_angle[5] = {-0.521568, -0.260107, 0.00135422, 0.262815, 0.524276};

    int laser_index[5] = {0, 159, 319, 479, 639};
    for (int i=0; i<5; i++)
    {   
        if (scan.ranges[laser_index[i]] == scan.ranges[laser_index[i]] && scan.ranges[laser_index[i]] > 1.0)
        {
            laserdis[i] = scan.ranges[laser_index[i]];
        }

        else if (scan.ranges[laser_index[i]] - 1.0 <= 0)
        {
            laserdis[i] = scan.range_min;
        }
        else 
        {
            laserdis[i] = scan.range_max;
        }
        // laserdis[i] = scan.ranges[laser_index[i]] / 10.0;
        // cout << "Laser: " << i << ": " << laserdis[i] << " . ";
    }
    
    for (int i=0; i<5; i++)
    {   
        double x_or = laserdis[i]*cos(laser_angle[i]);
        double y_or = laserdis[i]*sin(laser_angle[i]);
        obstacle_pos[i][0] = currentpos.x + cos(currentpos.theta)*x_or - sin(currentpos.theta)*y_or;
        obstacle_pos[i][1] = currentpos.y + sin(currentpos.theta)*x_or + cos(currentpos.theta)*y_or;
    }
}


void Controller::currentposCallback(const nav_msgs::Odometry& odom) 
{   

    currentpos.x = odom.pose.pose.position.x;
    currentpos.y = odom.pose.pose.position.y;
    currentpos.theta = quatoeuler_yaw(odom);

    geometry_msgs::Pose2D u_gtg;
    u_gtg.x = 0;
    u_gtg.y = 0;

    // distance between goal and robot in x-direction
    u_gtg.x = goalpos.x - currentpos.x;
    // distance between goal and robot in y-direction
    u_gtg.y = goalpos.y - currentpos.y;
    // angle from robot to goal
    double theta_g = atan2(u_gtg.y, u_gtg.x);
    // error between the goal angle and robot's angle
    u_gtg.theta = theta_g - currentpos.theta;

    // cout << "theta_g: " << theta_g << " theta: " << currentpos.theta << endl;
    e_k = atan2(sin(u_gtg.theta),cos(u_gtg.theta));

    // error for the proportional term
    e_P = e_k;
    // error for the integral term. Approximate the integral using the accumulated error, obj.E_k, and the error for this time step
    e_I = e_k + E_k;

    // cout << e_I << endl;
    // error for the derivative term. Hint: Approximate the derivative using the previous error, and the error for this time step, e_k.
    e_D = e_k - e_k_previous; 
    // update errors
    E_k = e_I;
    e_k_previous = e_k;

    // control input 
    w = k_p*e_P + k_i*e_I + k_d*e_D;

    if (w > 0.8)
    {
        w = 0.8;
    }
    else if (w < -0.8)
    {
        w = -0.8;
    }

    controlinput.angular.z = w;
    controlinput.linear.x = v_normal;
    controlinput_pub_.publish(controlinput); //output the square of the received value;
    
}

void Controller::eventCallback(const nav_msgs::Odometry& odom) 
{   
    currentpos.x = odom.pose.pose.position.x;
    currentpos.y = odom.pose.pose.position.y;
    double theta = quatoeuler_yaw(odom);
    int pathsize = path_.size();
    // cout << "Xc pose: " << currentpos.x << " Xg pose: " << goalpos.x << " Yc pose: " << currentpos.y << " Yg pose: "<< goalpos.y <<endl;
    if ((abs(currentpos.x - goalpos.x)<0.1) && (abs(currentpos.y - goalpos.y)<0.1 ) 
        && (currentpos.x * goalpos.x > 0.01))
    {   
        if (waypoint_cnt == pathsize)
        {
            controlinput.angular.z = 0;
            controlinput.linear.x = 0;
            controlinput_pub_.publish(controlinput);
            ROS_INFO("Finished...");
            ros::shutdown();
        }
        else if ((abs(currentpos.x - path_[pathsize-1].x)<0.1) && (abs(currentpos.y - path_[pathsize-1].y)<0.1 ))
        {   
            controlinput.angular.z = 0;
            controlinput.linear.x = 0;
            controlinput_pub_.publish(controlinput);
            ROS_INFO("Finished...");
            ros::shutdown();
        }
        else
        {
            cout << "Waypoint " << waypoint_cnt << " reached. ";
            // cout << "currentpos: " << currentpos.x << " " << currentpos.y << endl;
            waypoint_cnt++;
            setgoalpos(path_[waypoint_cnt].x, path_[waypoint_cnt].y);
        }

    }
}

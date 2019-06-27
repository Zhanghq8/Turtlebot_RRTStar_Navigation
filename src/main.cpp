#include <stdio.h>
#include <iostream>
#include "../include/turtlebot_rrtstar/RRTStar.h"


int main()
{
	RRT::RRTStar temp;
	temp.getmap("aaa");
	// temp.setstepsize(3.0);
	// temp.setnearradius(3.5);
	// temp.setgoalbias(0.07);
	// temp.setrandompointsize(5.0);
	// temp.setgoalradius(1.0);
	// temp.setmaxiterations(10000);
	// RRT::Vec2i start, goal;
	// start.x = 10.0;
	// start.y = 10.0;
	// goal.x = 40.0;
	// goal.y = 25.0;
	// RRT::Rectobstacle obstacle1{5,20,15,20};
	// RRT::Rectobstacle obstacle2{5,30,30,0};
	// temp.addobstacle(obstacle1);
	// temp.addobstacle(obstacle2);
	// // std::cout << "obstacle: " << temp.Obstacleset[0].topleftx << " " << temp.Obstacleset[0].toplefty << std::endl;

	// temp.findPath(start, goal);
	return 0;
}
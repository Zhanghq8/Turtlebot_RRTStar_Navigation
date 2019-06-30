#include <stdio.h>
#include <iostream>
#include "../include/turtlebot_rrtstar/RRTStar.h"


int main()
{
	RRT::RRTStar temp;
	temp.readmapparameter();
	temp.getmap();
	temp.setstepsize();
	temp.setnearradius();
	temp.setgoalbias(0.07);
	temp.setrandompointsize();
	temp.setgoalradius(0.10);
	temp.setmaxiterations(10000);
	RRT::Vec2i start, goal;
	start.x = 0;
	start.y = 0;
	goal.x = 2;
	goal.y = 2;
	// RRT::Rectobstacle obstacle1{5,20,15,20};
	// RRT::Rectobstacle obstacle2{5,30,30,0};
	// temp.addobstacle(obstacle1);
	// temp.addobstacle(obstacle2);
	// // std::cout << "obstacle: " << temp.Obstacleset[0].topleftx << " " << temp.Obstacleset[0].toplefty << std::endl;

	temp.findPath(start, goal);
	return 0;
}
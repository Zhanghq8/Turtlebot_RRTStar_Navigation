#include "../include/turtlebot_rrtstar/RRTStar.h"

RRT::RRTStar::RRTStar()
{
	readmapparameter();
	getmap();
	setstepsize();
	setnearradius();
	setgoalbias(0.07);
	setrandompointsize();
	setgoalradius(0.10);
	setmaxiterations(10000);
	RRT::Vec2i start, goal;
	start.x = 0.0;
	start.y = 0.0;
	goal.x = 3.0;
	goal.y = -2.0;
	findPath(start, goal);
}

RRT::Vertex::Vertex(Vec2i coordinates_, Vertex *parent_, float cost_)
{
	coordinates = coordinates_;
	parent = parent_;
	cost = cost_;
}

void RRT::RRTStar::getmap(std::string file)
{	
	int rows =0, cols =0;
	std::ifstream infile(file, std::ios::binary);
	std::stringstream ss;	
	std::string inputLine ="";
	// https://cboard.cprogramming.com/cplusplus-programming/168760-reading-pgm-file-using-visual-cplusplus.html
	getline(infile,inputLine);// read the first line : P5
	// if(inputLine.compare("P5")!=0) std::cout <<"Version error"<< std::endl;
	// std::cout <<"Version : "<< inputLine << std::endl;

	getline(infile,inputLine);// read the second line : comment
	// std::cout <<"Comment : "<< inputLine << std::endl;

	ss << infile.rdbuf();//read the third line : width and height
	ss >> cols >> rows;
	// std::cout << cols <<" columns and "<< rows <<" rows"<< std::endl;

	int max_val;//maximum intensity value : 255
	ss >> max_val;
	// std::cout<<max_val;
	unsigned char pixel;
	unsigned int pixel_value[rows][cols];
	cv::Mat img(rows, cols, CV_8UC1, cv::Scalar(70));

	for(int i=0; i < rows; i++)
	{//record the pixel values
		for(int j=0; j < cols; j++){
			ss >> pixel;
			int temp = int(pixel);
			if (temp > 230) 
			{
				img.at<uchar> (i,j) = 255;
			}
			else
			{
				img.at<uchar> (i,j) = 0;
			}
			pixel_value[i][j]= pixel;
		}
	}

	infile.close(); 

	cv::Mat img_temp = img.clone();
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
	cv::morphologyEx(img_temp, img_temp, cv::MORPH_OPEN, element);
	cv::morphologyEx(img_temp, img_temp, cv::MORPH_CLOSE, element);

	int thresh = 30;
	cv::Mat canny_output;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::Canny(img_temp, canny_output, thresh, thresh * 3, 3);
	cv::findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	int area = 0;
	int index = -1;
	for (int i = 0; i< contours.size(); i++)
	{
		if (cv::contourArea(contours[i]) > area)
		{	
			// std::cout << "contous " << i << " " << cv::contourArea(contours[i]) << std::endl;
			area = cv::contourArea(contours[i]);
			index = i;
		}
	}
	cv::Rect r = cv::boundingRect(cv::Mat(contours[index]));

	// Set map size
	Vec2i topleft;
	Vec2i bottomright;
	topleft.x = r.tl().x;
	topleft.y = r.tl().y;
	bottomright.x = r.br().x;
	bottomright.y = r.br().y;
	// std::cout << "Topleft: " << topleft.x << " " << topleft.y << std::endl;
	// std::cout << "Bottomright: " << bottomright.x << " " << bottomright.y << std::endl;

	Vec2i pixelbottomleft = {topleft.x+ 2*2, bottomright.y- 2*2};
	// set mapbottomleft
	mapbottomleft = pixel2pos(pixelbottomleft, rows, cols);
	// std::cout << "Map bottomleft: " << mapbottomleft.x << " " << mapbottomleft.y << std::endl;

	Vec2i width_range;
	Vec2i height_range;
	width_range.x = topleft.x + 2*2; // include obstacle inflation
	width_range.y = bottomright.x - 2*2;

	height_range.x = topleft.y + 2*2;
	height_range.y = bottomright.y - 2*2;

	// set map size
	map_width = (width_range.y - width_range.x + 1) * resolution;
	map_height = (height_range.y - height_range.x + 1) * resolution;
	// std::cout << "Map size: " << map_width << " * " << map_height << std::endl;

	cv::Rect roi = cv::Rect(width_range.x, height_range.x, width_range.y - width_range.x, height_range.y - height_range.x);

	cv::Mat roiImg;
	roiImg = img(roi).clone();

	cv::Mat element_sub = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::morphologyEx(roiImg, roiImg, cv::MORPH_OPEN, element_sub);

	// get obstacle
	int thresh_sub = 30;
	cv::Mat canny_output_sub;
	std::vector<std::vector<cv::Point> > contours_sub;
	std::vector<cv::Vec4i> hierarchy_sub;
	cv::Canny(roiImg, canny_output_sub, thresh_sub, thresh_sub * 5, 5);
	cv::findContours(canny_output_sub, contours_sub, hierarchy_sub, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	// cv::Mat drawing = cv::Mat::zeros( canny_output_sub.size(), CV_8UC3 );
	for (int i = 0; i< contours_sub.size(); i++)
	{
		// std::cout << "contous " << i << " " << cv::contourArea(contours_sub[i]) << std::endl;
		Rectobstacle rect_obstacle;
		cv::Scalar color = cv::Scalar( 255, 0, 0 );
		cv::Rect r = cv::boundingRect(cv::Mat(contours_sub[i]));
		// std::cout << r.tl().x << " " << r.tl().y << " " << r.br().x << " " << r.br().y << std::endl;
		rect_obstacle.width = (r.br().x - r.tl().x + 1 + 2*4) * resolution;
		rect_obstacle.height = (r.br().y - r.tl().y + 1 + 2*4) * resolution;
		// Vec2i obstaclebl
		rect_obstacle.bottomleftx = pixel2pos({width_range.x + r.tl().x - 2*2, height_range.x + r.br().y + 2*2}, rows, cols).x;
		rect_obstacle.bottomlefty = pixel2pos({width_range.x + r.tl().x - 2*2, height_range.x + r.br().y + 2*2}, rows, cols).y;
		addobstacle(rect_obstacle); 
		// std::cout << "obstacle: " << i << "width " << Obstacleset[i].width << "height " << Obstacleset[i].height 
		// << "(" << Obstacleset[i].bottomleftx << "," << Obstacleset[i].bottomlefty << ")" << std::endl;
		// cv::rectangle(drawing, r, cv::Scalar(255), 1);
	}
	// std::cout << "obstacle size: " << Obstacleset.size() << std::endl;

	// cv::imshow("image", drawing);
	// cv::waitKey();	 

}

RRT::Vec2i RRT::RRTStar::pixel2pos(Vec2i pixel_, int rows_, int cols_)
{
	Vec2i pos;
	pos.x = pgmbottomleft.x + (pixel_.x + 1) * resolution;
	pos.y = pgmbottomleft.y + (rows_ - pixel_.y + 1) * resolution;
	return pos;
}

void RRT::RRTStar::readmapparameter(std::string file)
{
	YAML::Node lconf = YAML::LoadFile(file);
	resolution = lconf["resolution"].as<float>();
	// std::cout << "resolution  " << resolution << std::endl;
	std::vector<float> origin;
	origin = lconf["origin"].as<std::vector<float>>();
	pgmbottomleft.x = origin[0];
	pgmbottomleft.y = origin[1];
	std::cout << "origin: " << pgmbottomleft.x<< " " << pgmbottomleft.y << std::endl;
}

void RRT::RRTStar::setinflationradius(int inflation_radius_)
{
	inflation_radius = inflation_radius_;
}

void RRT::RRTStar::setgoalbias(float goal_bias_)
{
	goal_bias = goal_bias_;
}

void RRT::RRTStar::setstepsize(float step_size_)
{
	step_size = step_size_;
}

void RRT::RRTStar::setmaxiterations(int max_iterations_)
{
	max_iterations = max_iterations_;
}

void RRT::RRTStar::setgoalradius(float goal_radius_)
{
	goal_radius = goal_radius_;
}

void RRT::RRTStar::setnearradius(float near_radius_)
{
	near_radius = near_radius_;
}

void RRT::RRTStar::setrandompointsize(float randompoint_size_)
{
	randompoint_size = randompoint_size_;
}

void RRT::RRTStar::addobstacle(Rectobstacle obstacle_)
{
	Obstacleset.push_back(obstacle_);
}

bool RRT::RRTStar::isHit(Vec2i coordinates1_, Vec2i coordinates2_)
{	
	// std::cout << "Found " << Obstacleset.size() << " obstacle. " << std::endl;
	if (Obstacleset.size() == 0)
	{
		return false;
	}
	for (int i=0; i<Obstacleset.size(); i++)
	{	
		// std::cout << "Obstacle index: " << i << std::endl;	
		Vec2i bottomleft = {Obstacleset[i].bottomleftx, Obstacleset[i].bottomlefty};
		Vec2i bottomright = {Obstacleset[i].bottomleftx + Obstacleset[i].width, Obstacleset[i].bottomlefty};
		Vec2i topleft = {Obstacleset[i].bottomleftx, Obstacleset[i].bottomlefty + Obstacleset[i].height};
		Vec2i topright = {Obstacleset[i].bottomleftx + Obstacleset[i].width, Obstacleset[i].bottomlefty + Obstacleset[i].height};
		// std::cout << "point" << bottomleft.x << " " << bottomleft.y << " " << topleft.x << " " << topleft.y << std::endl;
		bool top = islineintersect(coordinates1_, coordinates2_, topleft, topright);
		bool bottom = islineintersect(coordinates1_, coordinates2_, bottomleft, bottomright);
		bool left = islineintersect(coordinates1_, coordinates2_, topleft, bottomleft);
		bool right = islineintersect(coordinates1_, coordinates2_, topright, bottomright);
		// std::cout << "line" << top << bottom << left << right << std::endl;
		if (top || bottom || left || right)
		{
			return true;
		}
	}
	return false;
}


bool RRT::RRTStar::islineintersect(Vec2i line1p1_, Vec2i line1p2_, Vec2i line2p1_, Vec2i line2p2_)
{
	// calculate the distance to intersection point
	float uA = ((line2p2_.x-line2p1_.x)*(line1p1_.y-line2p1_.y) - (line2p2_.y-line2p1_.y)*
		(line1p1_.x-line2p1_.x)) / ((line2p2_.y-line2p1_.y)*(line1p2_.x-line1p1_.x) - 
		(line2p2_.x-line2p1_.x)*(line1p2_.y-line1p1_.y));
	float uB = ((line1p2_.x-line1p1_.x)*(line1p1_.y-line2p1_.y) - (line1p2_.y-line1p1_.y)*
		(line1p1_.x-line2p1_.x)) / ((line2p2_.y-line2p1_.y)*(line1p2_.x-line1p1_.x) - 
		(line2p2_.x-line2p1_.x)*(line1p2_.y-line1p1_.y));

	// if uA and uB are between 0-1, lines are colliding
	if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1) 
	{
		return true;
  	}
  	return false;
}

// check if the coordinate is in all the rectangular obstacles
bool RRT::RRTStar::isInObstacle(Vec2i& coordinates_)
{
	if (Obstacleset.size() == 0)
	{
		return false;
	}
	for (int i=0; i<Obstacleset.size(); i++)
	{
		if (coordinates_.x >= Obstacleset[i].bottomleftx
			&& coordinates_.x <= Obstacleset[i].bottomleftx + Obstacleset[i].width
			&& coordinates_.y >= Obstacleset[i].bottomlefty 
			&& coordinates_.y <= Obstacleset[i].bottomlefty + Obstacleset[i].height)
		{
			return true;
		}
	}
	return false;
}

// check if the coordinate is at goal pos
bool RRT::RRTStar::isGoal(Vec2i source_, Vec2i goal_) 
{
	float distance = euclidean_dis(source_, goal_);
	if (distance <= goal_radius) 
	{
		return true;
	}
	return false;
}

// check if the coordinate is valid

bool RRT::RRTStar::isValid(Vec2i coordinates_, Vec2i closestvertex_) 
{
	if (coordinates_.x > mapbottomleft.x && coordinates_.y > mapbottomleft.y 
		&& coordinates_.x < mapbottomleft.x + map_width && coordinates_.y < mapbottomleft.y + map_height
		&& closestvertex_.x > mapbottomleft.x && closestvertex_.y > mapbottomleft.y 
		&& closestvertex_.x < mapbottomleft.x + map_width && closestvertex_.y < mapbottomleft.y + map_height 
		&& isInObstacle(coordinates_) == false && isInObstacle(closestvertex_) == false
		&& isHit(coordinates_, closestvertex_) == false)
	{
		return true;
	}
	return false;
}

// calculate the euclidean distance from current to goal
float RRT::RRTStar::euclidean_dis(Vec2i source_, Vec2i goal_) 
{
	float e_distance = sqrt(pow(source_.x - goal_.x, 2) + pow(source_.y - goal_.y, 2));
	return e_distance; 
}

//generate new randompoint with goal_bias% probability to pick goal point 
RRT::Vec2i RRT::RRTStar::GenerateRandomPoint(Vec2i goal_)
{
	Vec2i randompoint;
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> x(mapbottomleft.x, mapbottomleft.x + map_width);
	std::uniform_real_distribution<> y(mapbottomleft.y, mapbottomleft.y + map_height);

	randompoint.x = x(gen);
	randompoint.y = y(gen);
	// std::cout << "randomx: " << randompoint.x << std::endl;
	// std::cout << "randomy: " << randompoint.y << std::endl;
	bool setgoal = (rand() % 100 + 1) <= goal_bias;
	if (setgoal == true )
	{
		return goal_;
	}
	return randompoint;
}

RRT::Vertex* RRT::RRTStar::getClosestVertex(std::set<Vertex*>& Vertices_, Vec2i randompoint_)
{	
	Vertex* closestvertex = NULL;
	float min_distance = std::numeric_limits<float>::max();
	for (auto vertex:Vertices_) {
		if (euclidean_dis(vertex->coordinates, randompoint_) < min_distance) {
			min_distance = euclidean_dis(vertex->coordinates, randompoint_);
			closestvertex = vertex;
		}
	}
	return closestvertex;
}

//generate new point along the line contain closestvertex and randompoint
//check if the new point is valid 
bool RRT::RRTStar::extend(Vertex* closestvertex_, Vec2i randompoint_)
{
	float theta = atan2(randompoint_.y - closestvertex_->coordinates.y, randompoint_.x - closestvertex_->coordinates.x);
	// std::cout << "theta: " << theta << std::endl;
	Vec2i vertextemp;
	vertextemp.x = closestvertex_->coordinates.x + step_size * cos(theta);
	vertextemp.y = closestvertex_->coordinates.y + step_size * sin(theta);
	// std::cout << "vertextemp: " << vertextemp.x << " " << vertextemp.y << std::endl;
	// std::cout << "closestvertex_: " << closestvertex_->coordinates.x << " " << closestvertex_->coordinates.y << std::endl;
	// std::cout << "isvalid " << isValid(vertextemp, closestvertex_->coordinates) << std::endl;
	if (isValid(vertextemp, closestvertex_->coordinates) == true) 
	{	
		Vertex* newparent_ = rewire(VertexSet, vertextemp);
		// std::cout << " newparent_" << newparent_ << std::endl;
		if (isValid(vertextemp, newparent_->coordinates) == true)
		{
			Vertex* newvertex = new Vertex(vertextemp, newparent_, newparent_->cost + step_size);
			// std::cout << "cost: " << newvertex->cost << std::endl;
			current = newvertex;
			VertexSet.insert(newvertex);
			return true;
		}	
	}
	return false;
}

// Search in Vertices set to find all vertex which the distance to newvertex is less than near radius
// Find the vertex with minimum cost
RRT::Vertex* RRT::RRTStar::rewire(std::set<Vertex*>& Vertices_, Vec2i newvertex_)
{	
	std::set<Vertex*> Nearset;
	Vertex* newparent = NULL;
	for (auto vertex:Vertices_) 
	{
		if (euclidean_dis(vertex->coordinates, newvertex_) <= near_radius) {
			Nearset.insert(vertex);
		}
	}
	// std::cout << "Nearset size: " << Nearset.size() << std::endl;
	float mincost = std::numeric_limits<float>::max();
	for (auto nearvertex:Nearset) 
	{
		if (nearvertex->cost < mincost)
		{
			mincost = nearvertex->cost;
			newparent = nearvertex;
		}
	}
	// std::cout << "new parent: " << newparent << std::endl;
	// std::cout << "Minimum cost: " << mincost << std::endl;
	return newparent;
}

void RRT::RRTStar::findPath(Vec2i source_, Vec2i goal_)
{	
	bool done_flag = false;
	VertexSet.insert(new Vertex(source_));
	current = *VertexSet.begin();
	int current_iterations = 0;
	while (done_flag != true && current_iterations < max_iterations) 
	{	
		// std::cout << current_iterations << std::endl;
		Vec2i randompoint = GenerateRandomPoint(goal_);
		Vertex* closestv= getClosestVertex(VertexSet, randompoint);
		// std::cout << "Closestv: " << closestv << std::endl;
		if (extend(closestv, randompoint) == true)
		{	
			current_iterations++;
			// std::cout << "isgoal: " << isGoal(current->coordinates, goal_) << std::endl;
			if (isGoal(current->coordinates, goal_) == true)
			{
				done_flag = true;
				Vertex* goalvertex = new Vertex(goal_, current);
				current = goalvertex;
				std::cout << "Found a path ";
			}
		}
		if (current_iterations == max_iterations)
		{
			std::cout << "No path found." << std::endl;
			current = NULL;
			releaseVertices(VertexSet);
			return;
		}
	}
	
	while (current != NULL) 
	{
		path.push_back(current->coordinates);
		current = current->parent;
	}
	reverse(path.begin(), path.end());
	float final_cost = 0;
	if (!path.empty()) 
	{
		std::cout << "with " <<  path.size() << " vertices. " << std::endl;
		std::cout << "[" << path[0].x << "," << path[0].y << "] ";
		for (int i=1; i<path.size(); i++)
		{
			std::cout << "[" << path[i].x << "," << path[i].y << "] ";
			final_cost += euclidean_dis(path[i], path[i-1]); 
		}
		std::cout << "\n";
	}
	std::cout << "Final cost(without smooth): " << final_cost << std::endl;

	randomsmoothpath();
	float final_cost_s = 0;
	if (!smooth_path.empty()) 
	{
		std::cout << "After smooth, find " <<  smooth_path.size() << " vertices. " << std::endl;
		std::cout << "[" << smooth_path[0].x << "," << smooth_path[0].y << "] ";
		for (int i=1; i<smooth_path.size(); i++)
		{
			std::cout << "[" << smooth_path[i].x << "," << smooth_path[i].y << "] ";
			final_cost_s += euclidean_dis(smooth_path[i], smooth_path[i-1]); 
		}
		std::cout << "\n";
	}
	std::cout << "Final cost(after smooth): " << final_cost_s << std::endl;
	exportpath();
	releaseVertices(VertexSet);
}

void RRT::RRTStar::minsmoothpath(Vec2i goal_)
{	
	if (path.size() <= 2) {
		smooth_path = path;
		return;
	}
	smooth_path.push_back(path[0]);
	int index1 = 0;
	int index2 = 1;
	// std::cout << "path size: " << path.size() << std::endl;
	while (true)
	{
		// std::cout << "Index: " << index1 << " " << index2 << std::endl;
		if (isValid(path[index1], path[index2]) == true)
		{	
			// std::cout << index1 << " " << index2 << " " << std::endl;
			index2++;
		}
		else
		{	
			if (index2 - 1 != 0)
			{
				smooth_path.push_back(path[index2-1]);
			}
			index1 = index2 - 1;
			index2 = index1 + 1;
		}
		if (index1 + 1 ==  path.size() || index2 == path.size())
		{
			break;
		}
	}

	if (smooth_path.back().x != goal_.x && smooth_path.back().y != goal_.y)
	{
		smooth_path.push_back(path.back());
	}
}

void RRT::RRTStar::randomsmoothpath()
{	
	smooth_path = path;
	if (path.size() <= 2) 
	{
		return;
	}
	int iteration = 3;

	for (int i=0; i<iteration; i++)
	{	
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_int_distribution<int> x(0, smooth_path.size()-1);
		std::uniform_int_distribution<int> y(0, smooth_path.size()-1);

		auto index1 = x(gen);
		auto index2 = y(gen);
		// std::cout << "index " << index1 << " " << index2 << std::endl;
		if (isValid(smooth_path[index1], smooth_path[index2]))
		{	
			if (abs(index1 - index2) > 1)
			{
				smooth_path.erase(smooth_path.begin() + std::min(index1, index2) + 1,
				 smooth_path.begin() + std::max(index1, index2));				
			}
		}
	}
}

void RRT::RRTStar::releaseVertices(std::set<Vertex*>& Vertices_)
{	
	std::cout << "Visited vertices: " << Vertices_.size() << std::endl;
	for (auto it = Vertices_.begin(); it != Vertices_.end();) 
	{
		delete *it;
		it = Vertices_.erase(it);
	}
}

void RRT::RRTStar::exportpath()
{
	std::ofstream file_path;
	file_path.open("/home/han/catkin_ws/src/turtlebot_rrtstar/path.txt",std::ios::trunc);
	for (int i=0; i<path.size(); i++)
	{
		file_path << path[i].x << " "; 
	}
	file_path << "\n";
	for (int i=0; i<path.size(); i++)
	{
		file_path << path[i].y << " "; 
	}
	file_path << "\n";
	file_path.close();

	std::ofstream file_smoothpath;
	file_smoothpath.open("/home/han/catkin_ws/src/turtlebot_rrtstar/smoothpath.txt",std::ios::trunc);
	for (int i=0; i<smooth_path.size(); i++)
	{
		file_smoothpath << smooth_path[i].x << " "; 
	}
	file_smoothpath << "\n";
	for (int i=0; i<smooth_path.size(); i++)
	{
		file_smoothpath << smooth_path[i].y << " "; 
	}
	file_smoothpath << "\n";

	file_smoothpath.close();
}



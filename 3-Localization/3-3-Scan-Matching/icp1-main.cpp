using namespace std;

#include <string>
#include <sstream>
#include "helper.h"

#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc

Eigen::Matrix4d ICP(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations){

	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

	//TODO: complete the ICP function and return the corrected transform

	return transformation_matrix;

}

int main(){

	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (1.0);

	// create a room
	double lowerX = -5;
	double upperX = 5;
	double lowerY = -5;
	double upperY = 5;
	vector<LineSegment> room;
	LineSegment top(0, 1, upperY, lowerX, upperX);
	room.push_back(top);
	LineSegment bottom(0, 1, lowerY, lowerX, upperX);
	room.push_back(bottom);
	LineSegment right(1, 0, upperX, lowerY, upperY);
	room.push_back(right);
	LineSegment left(1, 0, lowerX, lowerY, upperY);
	room.push_back(left);

	// create lidar
	Lidar lidar(0, 0, 0, 100, 128);

	PointCloudT::Ptr poses (new PointCloudT); 	// ground truth
	PointCloudT::Ptr locator (new PointCloudT); // estimated locations

	// starting location
	poses->points.push_back(PointT(lidar.x, lidar.y, 0));
	locator->points.push_back(PointT(lidar.x, lidar.y, 0));

	// get map of room
	PointCloudT::Ptr map = lidar.scan(room);
	cout << "map captured " << map->points.size() << " points" << endl;

	// move around the room

	// Part 1. Localize from single step
	vector<Vect2> movement = {Vect2(0.5,pi/12)};

	// Part 2. TODO: localize after several steps
	if(false){ // Change to true
		movement.push_back(Vect2(0.8, pi/10));
		movement.push_back(Vect2(1.0, pi/6));
	}
	// Part 3. TODO: localize after randomly moving around the whole room
	if(false){ // Change to true
		srand(time(0));
		for(int i = 0; i < 10; i++){
			double mag = 0.5 * ((double) rand() / (RAND_MAX)) + 0.5;
			double angle = pi/8 * ((double) rand() / (RAND_MAX)) + pi/8;
			movement.push_back(Vect2(mag, angle));
		}
	}

	renderPointCloud(viewer, map, "map", Color(0,0,1)); // render map
	Pose location(Point(0,0), 0);
	PointCloudT::Ptr scan;
	int count = 0;
	for( Vect2 move : movement ){

		// execute move
		lidar.Move(move.mag, move.theta);
		poses->points.push_back(PointT(lidar.x, lidar.y, 0));

		// scan the room
		scan = lidar.scan(room);
		cout << "scan captured " << scan->points.size() << " points" << endl;
		renderPointCloud(viewer, scan, "scan_"+to_string(count), Color(1,0,0)); // render scan
		 
		// perform localization
		Eigen::Matrix4d transform = ICP(map, scan, location, 0); //TODO: make the iteration count greater than zero
		Pose estimate = getPose(transform);
		// TODO: save estimate location and use it as starting pose for ICP next time
		
		locator->points.push_back(PointT(estimate.position.x, estimate.position.y, 0));
		
		// view transformed scan
		// TODO: perform the transformation on the scan using transform from ICP
		// TODO: render the correct scan
		
		count++;
	}

	// display ground truth poses vs estimated pose
	renderPointCloud(viewer, poses, "poses", Color(0,1,0), 8);
	renderPath(viewer, poses, "posePath", Color(0,1,0) );
	renderPointCloud(viewer, locator, "locator", Color(0,0,1), 6);
	renderPath(viewer, locator, "locPath", Color(0,0,1) );

	while (!viewer->wasStopped ())
	{
		viewer->spinOnce ();
	}
		
	return 0;
}

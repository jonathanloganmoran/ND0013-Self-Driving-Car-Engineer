/* ----------------------------------------------------------------------------
 * Lesson "3.4: Utilizing Scan Matching"
 * Authors     : Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Main entry into the NDT scan matching programme.
 * 						 Here a simulated point cloud scan of the user-entered
 * 						 displacement (pose) of the vehicle is obtained. This
 * 						 point cloud is registered to the input scan of the
 * 						 vehicle's original ground-truth pose using the Point
 * 						 Cloud Library (PCL) Normal Distributions Transform
 * 						 (NDT) algorithm. The resulting transformation estimate
 * 						 is used to compute the displacement between the
 * 						 original and user-entered pose. The rotation and
 * 						 translation of the offset to the vehicle's original
 * 						 location is animated in the PCL Viewer.
 * 
 * Note : The objects defined here are intended to be used with the Point
 *        Cloud Library (PCL) in order to perform scan matching.
 * 
 * WARNING: This programme is currently a work-in-progress. There is no 
 * 			functionality at this time.
 * ----------------------------------------------------------------------------
 */


#include "helper.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>
#include <string>
#include <sstream>
#include <chrono> 
#include <ctime> 

/*** Defining the programme parameters ***/
// Define the user input state and name of registration algorithms available
enum Registration{ Off, Ndt};
// Set the starting user input state to 'Off' (i.e., no input offset entered yet)
Registration matching = Off;
// Set the base path relative to CWD where '.pcd' files are stored
const static std::string kBasePath = "../";
// Set the number of `.pcd` files to load from root directory
// Note: scans assumed to be numbered consecutively starting with 'scan1.pcd'
const static int kNumInputScansToLoad = 1;

/*** Defining the NDT hyperparameters ***/
// The maximum number of NDT iterations to perform before termination.
// Each iteration the NDT algorithm attempts to improve the accuracy of the
// transformation. Default: 150, Rule of thumb: start with default and
// decrease or increase depending on size and complexity of data set.
const static int kMaximumIterationsNDT = 50;
// The step size taken for each iteration of the NDT algorithm.
// Used to determine how much the transformation matrix is updated at
// each iteration. A larger step size will lead to faster convergence,
// but may lead to inaccurate results. Default: 0.1, Rule of thumb:
// decrease if NDT is coverging too quickly.
const static double kStepSizeNDT = 0.05;

/*** Setting voxel grid hyperparameters ***/
// Resolution of each 3D voxel ('box') used to downsample the point cloud
// Here we assume a cuboid, i.e., each of the sides (`lx`, `ly`, `lz`) have
// the same dimensions according to what is set here (in metres).
const static double kLeafSizeVoxelGrid = 0.5;


Pose pose(Point(0,0,0), Rotate(0,0,0));
Pose savedPose = pose;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer)
{
	if (event.getKeySym() == "Right" && event.keyDown()){
		matching = Off;
		pose.position.x += 0.1;
  	}
	else if (event.getKeySym() == "Left" && event.keyDown()){
		matching = Off;
		pose.position.x -= 0.1;
  	}
  	else if (event.getKeySym() == "Up" && event.keyDown()){
		matching = Off;
		pose.position.y += 0.1;
  	}
	else if (event.getKeySym() == "Down" && event.keyDown()){
		matching = Off;
		pose.position.y -= 0.1;
  	}
	else if (event.getKeySym() == "k" && event.keyDown()){
		matching = Off;
		pose.rotation.yaw += 0.1;
		while( pose.rotation.yaw > 2*pi)
			pose.rotation.yaw -= 2*pi; 
  	}
	else if (event.getKeySym() == "l" && event.keyDown()){
		matching = Off;
		pose.rotation.yaw -= 0.1;
		while( pose.rotation.yaw < 0)
			pose.rotation.yaw += 2*pi; 
  	}
	else if(event.getKeySym() == "n" && event.keyDown()){
		matching = Ndt;
	}
	else if(event.getKeySym() == "space" && event.keyDown()){
		matching = Off;
		pose = savedPose;
	}
	else if(event.getKeySym() == "x" && event.keyDown()){
		matching = Off;
		savedPose = pose;
	}

}


/* Implements the Normal Distributions Transform (NDT) algorithm.
 *
 * Returns the estimated transformation (i.e., rotation and translation)
 * between the `source` and `target` point clouds. The transformation
 * parameters are iteratively computed using the Point Cloud Library (PCL)
 * implementation of the NDT algorithm.
 * 
 * Here the `ndt` object is the pre-initialised NDT object which has been
 * set with the voxelised `target` point cloud. This is performed only once
 * to reduce setup costs for successive `source` alignments sharing the same
 * `target` pose to align with respect to. 
 *   
 * Note: the `source` point cloud is first transformed into the coordinate
 * frame of the given `startingPose` before it is registered to the `target`.
 * 
 * @param  ndt			 Pre-initialised NDT class instance from PCL. 
 * @param  source		 Starting point cloud to align to the `target`.
 * @param  startingPose  3D pose to transform the `source` point cloud by.
 * @param  iterations	 Maximum number of iterations to run NDT for.
 * @returns transformationMatrix
 */
Eigen::Matrix4d NDT(
		pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt, 
		PointCloudT::Ptr source, 
		Pose3D startingPose, 
		int iterations
) {
	// Initialising the output as the 4x4 identity matrix
  	Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
	// Construct the 2D transformation matrix from the `startingPose`
	Eigen::Matrix4d startingPoseTransform = transform3D(
		startingPose.rotation.yaw,
		startingPose.rotation.pitch,
		startingPose.rotation.roll,
		startingPose.position.x,
		startingPose.position.y,
		startingPose.position.z
	);
	// Transform the `source` point cloud by the `startingPose`
	PointCloudT::Ptr sourceTransformed(new PointCloudT);
	pcl::transformPointCloud(
		*source,
		*sourceTransformed,
		startingPoseTransform
	)
	/*** Compute the scan matching registration with the NDT algorithm ***/
	pcl::console::TicToc time;
	time.tic();
	// Perform the registration (alignment) with NDT
	PointCloudT::Ptr outputNDT(new PointCloudT);
	ndt.align(*outputNDT);
	std::cout << "Finished NDT alignment in " << time.toc() << " ms" << "\n";
	std::cout << "NDT converged: " << std::boolalpha << ndt.hasConverged();
	std::cout << ", Fitness score: " << ndt.getFitnessScore() << "\n";
	// Check if NDT algorithm has converged
	if (ndt.hasConverged()) {
		// Get the estimated transformation matrix
		transformationMatrix = ndt.getFinalTransformation().cast<double>();
		// Transform the estimated matrix into `startingPose` coordinate frame
		transformationMatrix *= startingPoseTransform;
		// Return estimated transformation matrix corrected by `startingPose`
		return transformationMatrix; 
	}
  	else {
		std::cout << "WARNING: NDT did not converge" << "\n";
		// Return the identity matrix (i.e., apply no transformation)
		return transformationMatrix;
	}

}

void drawCar(Pose pose, int num, Color color, double alpha, pcl::visualization::PCLVisualizer::Ptr& viewer){

	BoxQ box;
	box.bboxTransform = Eigen::Vector3f(pose.position.x, pose.position.y, 0);
    box.bboxQuaternion = getQuaternion(pose.rotation.yaw);
    box.cube_length = 4;
    box.cube_width = 2;
    box.cube_height = 2;
	renderBox(viewer, box, num, color, alpha);
}

struct Tester{

	Pose pose;
	bool init = true;
	int cycles = 0;
	pcl::console::TicToc timer;

	//thresholds
	double distThresh = 1e-3;
	double angleThresh = 1e-3;

	vector<double> distHistory;
	vector<double> angleHistory;

	void Reset(){
		cout << "Total time: " << timer.toc () << " ms, Total cycles: " << cycles << endl;
		init = true;
		cycles = 0;
		distHistory.clear();
		angleHistory.clear();
	}

	double angleMag( double angle){

		return abs(fmod(angle+pi, 2*pi) - pi);
	}

	bool Displacement( Pose p){

		if(init){
			timer.tic();
			pose = p;
			init = false;
			return true;
		}

		Pose movement = p - pose;
		double tdist = sqrt(movement.position.x * movement.position.x + movement.position.y * movement.position.y + movement.position.z * movement.position.z);
		double adist = max( max( angleMag(movement.rotation.yaw), angleMag(movement.rotation.pitch)), angleMag(movement.rotation.roll) );

		if(tdist > distThresh || adist > angleThresh){
			distHistory.push_back(tdist);
			angleHistory.push_back(adist);
			pose = p;

			cycles++;
			return true;
		}
		else
			return false;
	
	}

};

int main(){

	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
	viewer->setCameraPosition(pose.position.x, pose.position.y, 60, pose.position.x+1, pose.position.y+1, 0, 0, 0, 1);

	// Load map and display it
	PointCloudT::Ptr mapCloud(new PointCloudT);
  	pcl::io::loadPCDFile("map.pcd", *mapCloud);
  	cout << "Loaded " << mapCloud->points.size() << " data points from map.pcd" << endl;
	renderPointCloud(viewer, mapCloud, "map", Color(0,0,1)); 

	// True pose for the input scan
	vector<Pose> truePose ={Pose(Point(2.62296,0.0384164,0), Rotate(6.10189e-06,0,0)), Pose(Point(4.91308,0.0732088,0), Rotate(3.16001e-05,0,0))};
	drawCar(truePose[0], 0,  Color(1,0,0), 0.7, viewer);

	// Load input scan
	PointCloudT::Ptr scanCloud(new PointCloudT);
  	pcl::io::loadPCDFile("scan1.pcd", *scanCloud);

	typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);

	cloudFiltered = scanCloud; // TODO: remove this line
	//TODO: Create voxel filter for input scan and save to cloudFiltered
	// ......

	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	// Setting the NDT hyperparameters
	ndt.setMaxIterations(kMaximumIterationsNDT);
	ndt.setStepSize(kStepSizeNDT);
	//TODO: Set resolution and point cloud target (map) for ndt
	// ......

	PointCloudT::Ptr transformed_scan (new PointCloudT);
	Tester tester;

	while (!viewer->wasStopped())
  	{
		Eigen::Matrix4d transform = transform3D(pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll, pose.position.x, pose.position.y, pose.position.z);

		if( matching != Off){
			if( matching == Ndt)
				transform = NDT(ndt, cloudFiltered, pose, 0); //TODO: change the number of iterations to positive number
  			pose = getPose(transform);
			if( !tester.Displacement(pose) ){
				if(matching == Ndt)
					cout << " Done testing NDT" << endl;
				tester.Reset();
				double pose_error = sqrt( (truePose[0].position.x - pose.position.x) * (truePose[0].position.x - pose.position.x) + (truePose[0].position.y - pose.position.y) * (truePose[0].position.y - pose.position.y) );
				cout << "pose error: " << pose_error << endl;
				matching = Off;
			}
		}
		
  		pcl::transformPointCloud (*cloudFiltered, *transformed_scan, transform);
		viewer->removePointCloud("scan");
		renderPointCloud(viewer, transformed_scan, "scan", Color(1,0,0)	);

		viewer->removeShape("box1");
		viewer->removeShape("boxFill1");
		drawCar(pose, 1,  Color(0,1,0), 0.35, viewer);
		
  		viewer->spinOnce ();
  	}

	return 0;
}

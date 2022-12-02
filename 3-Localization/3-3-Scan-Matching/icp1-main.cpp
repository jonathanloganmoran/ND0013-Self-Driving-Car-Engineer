/* ----------------------------------------------------------------------------
 * Lesson "3.3: Scan Matching Algorithms"
 * Copyright (c) Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by: Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the 2-D Iterative Closest Points (ICP)
 * 						 algorithm, i.e., the scan matching algorithm used to
 * 						 localise a simulated robot in a 2D environment using
 * 						 laser scan data from a single simulated LiDAR sensor.
 * ----------------------------------------------------------------------------
 */

#include "helper.h"
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   			// TicToc
#include <string>
#include <sstream>


/*** Define the ICP hyperparameters ***/
// The maximum correspondence distance between `source` and `target`
// i.e., correspondences with higher distances will be ignored
const static double kMaxCorrespondenceDistanceICP = 1.0;  		// Metres (m)
// The maximum number of ICP iterations to perform before termination
const static int kMaximumIterationsICP = 60;
// The maximum epsilon threshold between previous transformation and current
// estimated transformation, i.e., difference must be smaller than this value
const static double kTransformationEpsilonICP = 1e-2;
// The maximum sum of Euclidean squared errors between two consecutive steps
// before algorithm is considered to be converged, i.e., threshold value
const static double kEuclideanFitnessEpsilonICP = 1e-3;
// The inlier distance threshold for the internal RANSAC outlier rejection loop
// Note: a point is considered an inlier if the distance between `target` and
// transformed `source` is smaller than inlier dist. threshold (default: 0.05m)
const static double kRANSACOutlierRejectionThresholdICP = 1.0;  // Metres (m)

/* Implements the 2D Iterative Closest Point (ICP) algorithm.
 * 
 * The ICP algorithm is provided in the Point Cloud Library (PCL) which uses
 * the Singular Value Decomposition (SVD) to estimate the transformation.
 * 
 * The rotation and translation between the `target` and `source` point clouds
 * are iteratively estimated. The ICP algorithm from the PCL library minimises
 * the sum of Euclidean squared errors between the coordinates of the matched
 * pairs. Here a pair is a set of point-to-point correspondences between the
 * `target` and `source` point cloud points.
 *  
 * After the ICP algorithm converges (or a maximum number of iterations have
 * been reached), the `transformation_matrix` is returned. This 4x4 matrix
 * contains the estimated translation and rotation components from the
 * `source` to `target` points.
 *
 * @param   target		  Next PCL instance from which to recover the transform.
 * @param   source		  Reference point cloud instance to align to `target`.
 * @param   startingPose  Initial robot pose at time $t=0$.
 * @param   iterations	  Number of maximum iterations to perform.
 * @returns transformation_matrix
 */
Eigen::Matrix4d ICP(
		PointCloudT::Ptr target,
		PointCloudT::Ptr source,
		Pose startingPose,
		int iterations
){
	// Initialise the output matrix as the identity matrix
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	// Construct the 2D transformation matrix from the `startingPose`
	Eigen::Matrix4d startingPoseTransform = transform2D(
		startingPose.theta,
		startingPose.position.x,
		startingPose.position.y
	);
	// Transform the `source` point cloud by `startingPose`
	PointCloudT::Ptr sourceTransformed(new PointCloudT);
	pcl::transformPointCloud(
		*source,
		*sourceTransformed,
		startingPoseTransform
	);
	// TODO: complete the ICP function and return the corrected transform
	// Start a `TicToc` time tracking instance to profile the ICP algorithm
	pcl::console::TicToc time;
	time.tic();
	// Instantiate the ICP class
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	// Set the input source and target point clouds
	icp.setInputSource(source);
	icp.setInputTarget(target);
	// Set the ICP hyperparameters
	icp.setMaxCorrespondenceDistance(kMaxCorrespondenceDistanceICP);
	icp.setMaximumIterations(kMaximumIterationsICP);
	icp.setTransformationEpsilon(kTransformationEpsilonICP);
	icp.setEuclideanFitnessEpsilon(kEuclideanFitnessEpsilonICP);
	icp.setRANSACOutlierRejectionThreshold(kRANSACOutlierRejectionThresholdICP);
	// Perform the alignment with ICP
	PointCloudT::Ptr outputICP(new PointCloudT);
	icp.align(*outputICP);
	std::cout << "Finished ICP alignment in " << time.toc() << " ms" << "\n";
	std::cout << "ICP converged: " << std::boolalpha << icp.hasConverged();
	std::cout << ", Fitness score: " << icp.getFitnessScore() << "\n";
	// Check if ICP algorithm converged
	if (icp.hasConverged()) {
		// Get the final transformation matrix and apply it to starting pose
		transformation_matrix = icp.getFinalTransformation().cast<double>();
		transformation_matrix = transformation_matrix * startingPoseTransform;
		return transformation_matrix;
	}
	else {
		std::cout << "WARNING: ICP did not converge" << "\n";
		return transformation_matrix;
	}
}


/* Entry point to the 2D robot localisation programme.
 * Using the Iterative Closest Point (ICP) algorithm to localise a simulated
 * robot in a 2D environment using simulated LiDAR scans (one per time-step).
 * We assume the estimated starting pose and the robot's ground-truth starting
 * pose are relatively close.
 */
int main() {
	/*** Initialising the sensor and environment variables ***/
	// Initialise a new PCL Viewer instance to render the environment / scans
	pcl::visualization::PCLVisualizer::Ptr viewer(
		new pcl::visualization::PCLVisualizer ("2D Viewer")
	);
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	// Create a 2D environment ("room")
	double lowerX = -5;
	double upperX = 5;
	double lowerY = -5;
	double upperY = 5;
	std::vector<LineSegment> room;
	LineSegment top(0, 1, upperY, lowerX, upperX);
	room.push_back(top);
	LineSegment bottom(0, 1, lowerY, lowerX, upperX);
	room.push_back(bottom);
	LineSegment right(1, 0, upperX, lowerY, upperY);
	room.push_back(right);
	LineSegment left(1, 0, lowerX, lowerY, upperY);
	room.push_back(left);
	// Initialise a new LiDAR sensor instance
	// Here we pass in the `(x, y, theta, range, res)` values
	Lidar lidar(0, 0, 0, 100, 128);
	// Initialise vectors for storing the pose object instances
	PointCloudT::Ptr poses(new PointCloudT); 		// Ground-truth poses
	PointCloudT::Ptr locator(new PointCloudT); 		// Estimated locations
	// Add the starting location to the vectors
	poses->points.push_back(PointT(lidar.x, lidar.y, 0));
	locator->points.push_back(PointT(lidar.x, lidar.y, 0));
	// Get the 2D scan map of room
	PointCloudT::Ptr map = lidar.scan(room);
	cout << "Map captured " << map->points.size() << " points" << "\n";
	/*** Moving the robot around the room ***/
	// Part 1. Localise from single step
	std::vector<Vect2> movement = {Vect2(0.5, pi / 12)};
	// Part 2. TODO: Localise after several steps
	bool runPart2 = false;						  // Change to true for Part 2
	if (runPart2) {
		movement.push_back(Vect2(0.8, pi / 10));
		movement.push_back(Vect2(1.0, pi / 6));
	}
	// Part 3. TODO: Localise after moving around entire room at random
	bool runPart3 = false;						  // Change to true for Part 3
	if (runPart3) {
		srand(time(0));
		for(int i = 0; i < 10; i++){
			double mag = 0.5 * ((double) rand() / (RAND_MAX)) + 0.5;
			double angle = pi / 8 * ((double) rand() / (RAND_MAX)) + pi / 8;
			movement.push_back(Vect2(mag, angle));
		}
	}
	// Render the map instance
	renderPointCloud(viewer, map, "map", Color(0, 0, 1));
	Pose location(Point(0, 0), 0);
	PointCloudT::Ptr scan;
	int count = 0;
	// Execute robot move, generate new LiDAR scans, and perform localisation
	for (Vect2 move : movement) {
		// Execute the robot move and add next pose to pose vector 
		lidar.Move(move.mag, move.theta);
		poses->points.push_back(PointT(lidar.x, lidar.y, 0));
		// Generate a new scan of the room with the LiDAR sensor
		scan = lidar.scan(room);
		cout << "Scan captured " << scan->points.size() << " points" << "\n";
		// Render the resulting LiDAR point cloud
		renderPointCloud(
			viewer, scan, "scan_" + std::to_string(count), Color(1, 0, 0)
		);
		// Perform the ICP localisation algorithm and obtain the transform
		// TODO: Set the iteration count to something greater than zero
		unsigned int num_iter = 0;
		Eigen::Matrix4d transform = ICP(map, scan, location, num_iter);
		// Get the pose estimate from the ICP transform
		Pose estimate = getPose(transform);
		// TODO: Save the estimate location and use it as starting pose
		// 		 for the ICP algorithm at the next time-step
		locator->points.push_back(
			PointT(estimate.position.x, estimate.position.y, 0)
		);
		// Render the transformed scan in the PCL Viewer
		// TODO: Perform the transformation on the scan with ICP result
		// TODO: Render the ground-truth pose
		count++;
	}
	// Display the ground-truth poses versus the estimated poses
	renderPointCloud(viewer, poses, "poses", Color(0, 1, 0), 8);
	renderPath(viewer, poses, "posePath", Color(0, 1, 0));
	renderPointCloud(viewer, locator, "locator", Color(0, 0, 1), 6);
	renderPath(viewer, locator, "locPath", Color(0, 0, 1));
	while (!viewer->wasStopped()) {
		viewer->spinOnce();
	}
	return 0;
}
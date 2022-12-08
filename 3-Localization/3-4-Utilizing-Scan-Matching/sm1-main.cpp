/* ----------------------------------------------------------------------------
 * Lesson "3.4: Utilizing Scan Matching"
 * Authors     : Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Main entry into the scan matching programme.
 * 						 Here a simulated point cloud scan of the user-entered
 * 						 displacement (pose) of the vehicle is obtained. This
 * 						 point cloud is registered to the input scan of the
 * 						 vehicle's original ground-truth pose using the Point
 * 						 Cloud Library (PCL) Iterative Closest Point (ICP)
 * 						 algorithm. The resulting transformation estimate is
 * 						 used to compute the displacement between the original
 * 						 and user-entered pose. The rotation / translation
 * 						 of the offset to the vehicle's original location is
 * 						 animated in the PCL Viewer.
 * 
 * Note : The objects defined here are intended to be used with the Point
 *        Cloud Library (PCL) in order to perform scan matching.
 * 
 * WARNING: This programme is currently a work-in-progress. There is no 
 * 			functionality at this time.
 * ----------------------------------------------------------------------------
 */

 
#include "helpers.h"
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include <sstream>
#include <chrono>
#include <ctime>

/*** Defining the programme parameters ***/
enum Registration{Off, Icp};
Registration matching = Off;

/*** Defining the initial state variables ***/
Pose3D pose(Point3D(0, 0, 0), Rotate(0, 0, 0));
Pose savedPose = pose;

/*** Define the ICP hyperparameters ***/
// The maximum correspondence distance between `source` and `target`
// i.e., correspondences with higher distances will be ignored
// Should be sufficiently large s.t. all points are considered.
// Rule of thumb: set to max distance between two points in the point clouds.
const static double kMaxCorrespondenceDistanceICP = 5;  		// Metres (m)
// The maximum number of ICP iterations to perform before termination.
// Should be large enough to ensure the algorithm has sufficient time to
// converge. Rule of thumb: set to twice the number of points in the PCL.
const static int kMaximumIterationsICP = 120;
// The maximum epsilon threshold between previous transformation and current
// estimated transformation. Rule of thumb: set between 1e-4 and 1e-8.
const static double kTransformationEpsilonICP = 1e-4;
// The maximum sum of Euclidean squared errors between two consecutive steps
// before algorithm is considered to be converged.
// Rule of thumb: set between 1 and 10.
const static double kEuclideanFitnessEpsilonICP = 2;
// The inlier distance threshold for the internal RANSAC outlier rejection loop
// Note: a point is considered an inlier if the distance between `target` and
// transformed `source` is smaller than inlier distance threshold.
// Default: 0.05m, Rule of thumb: set between 0.2 and 0.3 m. 
const static double kRANSACOutlierRejectionThresholdICP = 0.2;  // Metres (m)


/* Event handler that updates the PCL Viewer state and point cloud pose. 
 *
 * @param   event   `KeyboardEvent` containing the pressed key.
 * @param   viewer  PCL Viewer instance from which the event was created.
 */
void KeyboardEventOccurred(
        const pcl::visualization::KeyboardEvent &event,
        void* viewer
) {
  	// boost::shared_ptr<
    //     pcl::visualization::PCLVisualizer
    // > viewer = *static_cast<boost::shared_ptr<
    //         pcl::visualization::PCLVisualizer
    //     >*>(viewer_void
    // );
	if (event.getKeySym() == "Right" && event.keyDown()) {
		matching = Off;
		pose.position.x += 0.1;
  	}
	else if (event.getKeySym() == "Left" && event.keyDown()) {
		matching = Off;
		pose.position.x -= 0.1;
  	}
  	if (event.getKeySym() == "Up" && event.keyDown()) {
		matching = Off;
		pose.position.y += 0.1;
  	}
	else if (event.getKeySym() == "Down" && event.keyDown()) {
		matching = Off;
		pose.position.y -= 0.1;
  	}
	else if (event.getKeySym() == "k" && event.keyDown()) {
		matching = Off;
		pose.rotation.yaw += 0.1;
		while (upose.rotation.yaw > 2 * M_PI) {
			upose.rotation.yaw -= 2 * M_PI;
		}
  	}
	else if (event.getKeySym() == "l" && event.keyDown()) {
		update = true;
		pose.rotation.yaw -= 0.1;
		while (pose.rotation.yaw < 0) {
			pose.rotation.yaw += 2 * M_PI;
		} 
  	}
	else if (event.getKeySym() == "i" && event.keyDown()) {
		matching = Icp;
  	}
	else if (event.getKeySym() == "space" && event.keyDown()) {
		matching = Off;
		pose = savedPose;
  	}
	else if (event.getKeySym() == "x" && event.keyDown()) {
		matching = Off;
		savedPose = pose;
  	}
}


/* Implements the Iterative Closest Point (ICP) algorithm.
 *
 * Returns the estimated transformation (i.e., rotation and translation)
 * between the `source` and `target` point clouds. The transformation
 * parameters are iteratively computed using the Point Cloud Library (PCL)
 * ICP implementation.
 * 
 * Note: the `source` point cloud is first transformed into the coordinate
 * frame of the given `startingPose` before it is registered to the `target`. 
 *
 * @param    target			Reference point cloud to align the `source` to.
 * @param    source			Starting point cloud to align to the `target`.
 * @param    startingPose   3D pose to transform the `source` point cloud by.
 * param     iterations		Maximum number of iterations to run ICP for.
 * @returns  transformation_matrix
 */
Eigen::Matrix4d ICP(
		PointCloudT::Ptr target, 
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
		startingPose.position.xt,
		startingPose.position.yt,
		startingPose.position.zt
	)
	// Transform the `source` point cloud by the `startingPose`
	PointCloudT::Ptr sourceTransformed(new PointCloudT);
	pcl::transformPointCloud(
		*source,
		*sourceTransformed,
		startingPoseTransform
	)
	/*** Compute the scan matching registration with the ICP algorithm ***/
	// Start a `TicToc` time tracking instance to profile the ICP algorithm
	pcl::console::TicToc time;
	time.tic();
	// Instantiate the ICP class
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	// Set the input `source` and `target` point clouds
	icp.setInputSource(source);
	icp.setInputTarget(target);
	// Set the ICP hyperparameters
	icp.setMaxCorrespondenceDistance(kMaxCorrespondenceDistanceICP);
	icp.setMaximumIterations(kMaximumIterationsICP);
	icp.setTransformationEpsilon(kTransformationEpsilonICP);
	icp.setEuclideanFitnessEpsilon(kEuclideanFitnessEpsilonICP);
	icp.setRANSACOutlierRejectionThreshold(kRANSACOutlierRejectionThresholdICP);
	// Perform the registration (alignment) with ICP
	PointCloudT::Ptr outputICP(new PointCloudT);
	icp.align(*outputICP);
	std::cout << "Finished ICP alignment in " << time.toc() << " ms" << "\n";
	std::cout << "ICP converged: " << std::boolalpha << icp.hasConverged();
	std::cout << ", Fitness score: " << icp.getFitnessScore() << "\n";
	// Check if ICP algorithm converged
	if (icp.hasConverged()) {
		// Get estimated transformation matrix
		transformationMatrix = icp.getFinalTransformation().cast<double>();
		// Transform the estimated matrix into `startingPose` coordinate frame
		transformationMatrix *= startingPoseTransform;
		return transformationMatrix;
	}
	else {
		std::cout << "WARNING: ICP did not converge" << "\n";
		// Return the identity matrix (i.e., apply no transformation)
		return transformationMatrix
	}
}


/* Creates and renders a bounding box instance to represent the car dimensions.
 *
 * @param  pose		3D pose of the object to surround with bounding box.
 * @param  num		Unique integer `id` to assign the bounding box object.
 * @param  color	RGB-formatted `Color` instance to render the bbox with.
 * @param  alpha	Opacity value to render the bbox with, in range [0, 1].
 * @param  viewer	PCL Viewer instance to render the vehicle bbox onto.
 */
void drawCar(
		Pose3D pose, 
		int num, 
		Color color, 
		double alpha, 
		pcl::visualization::PCLVisualizer::Ptr& viewer
) {
	BoxQ box;
	// Store the 3D pose as 3x1 vector of Cartesian coordinates
	box.bboxTransform = Eigen::Vector3f(
		pose.position.x, 
		pose.position.y, 
		0
	);
	// Computing the quaternion form of the rotation given by yaw angle 
    box.bboxQuaternion = getQuaternion(
		pose.rotation.yaw
	);
    // Setting dimensions of the vehicle bounding box
	box.cube_length = 4;
    box.cube_width = 2;
    box.cube_height = 2;
	renderBox(
		viewer, 
		box, 
		num, 
		color, 
		alpha
	);
}

/* Stores the user-entered displacement over time.
 *
 * @struct	Tester		  "sm1-main.cpp"
 * @brief	Stores the displacement history of the vehicle entered by the user.  
 * @var		pose		  3D pose of the displacement.
 * @var		init		  Whether or not it is the first time-step.
 * @var		cycles		  Number of tracked displacements.
 * @var 	timer		  Tracks time to move object to 3D pose `p`.
 * @var		distThresh    Maxmium allowed distance of displacement.
 * @var		angleThresh   Maxmimum allowed orientation of displacement.
 * @var		distHistory   History of displacement distances.
 * @var		angleHistory  History of displacement orientation angles.
 */
struct Tester{
	// Initialise the starting displacement variables
	Pose3D pose;
	bool init = true;
	int cycles = 0;
	pcl::console::TicToc timer;
	// Set the displacement thresholds
	double distThresh = 1e-3;
	double angleThresh = 1e-3;
	// Initialise the displacement history vectors
	std::vector<double> distHistory;
	std::vector<double> angleHistory;

	// Resets the displacement history
	void Reset() {
		std::cout << "Total time: " << timer.toc();
		std::cout << " ms, Total cycles: " << cycles << "\n";
		init = true;
		cycles = 0;
		distHistory.clear();
		angleHistory.clear();
	}
	// Returns the magnitude of the input angle
	double angleMag(double angle) {
		return abs(fmod(angle + M_PI, 2 * M_PI) - M_PI);
	}
	// Updates the displacement history with the current Pose
	bool Displacement(Pose3D p) {
		if (init) {
			timer.tic();
			pose = p;
			init = false;
			return true;
		}
		Pose3D movement = p - pose;
		double tdist = sqrt(
			(movement.position.x * movement.position.x) 
			+ (movement.position.y * movement.position.y) 
			+ (movement.position.z * movement.position.z)
		);
		double adist = max(
			max(angleMag(movement.rotation.yaw),
				angleMag(movement.rotation.pitch)
			),
			angleMag(movement.rotation.roll)
		);
		if (tdist > distThresh || adist > angleThresh) {
			distHistory.push_back(tdist);
			angleHistory.push_back(adist);
			pose = p;
			cycles++;
			return true;
		}
		else {
			return false;
		}
	}
};


/* Runs and visualises the scan matching registration programme.
 * 
 * Here the point clouds are loaded from the local filesystem and visualised
 * onto a PCL Viewer instance. The true pose of the `input` scan is defined,
 * and the voxelised representation of the point cloud is computed.
 * 
 * Using the interactive PCL Viewer instance, the user is able to manually
 * shift and scale the ground-truth bounding box to an offset pose using the
 * `k`, `l`, left- and right-arrow keys (see `KeyboardEventOccurred` function).
 * This manual offset is "scanned" with the simulated LiDAR sensor, which is
 * used as the `target` point cloud to register the original `source` point
 * cloud to with respect to.
 * 
 * Once the transformation has been computed by the PCL ICP algorithm,
 * the PCL Viewer animates the estimated rotation and translation which results
 * in the user-offset bounding box returning to a pose close to its original
 * ground-truth location.
 */
int main() {
	pcl::visualization::PCLVisualizer::Ptr viewer(
		new pcl::visualization::PCLVisualizer("3D Viewer")
	);
  	viewer->setBackgroundColor(0, 0, 0);
	viewer->registerKeyboardCallback(
		keyboardEventOccurred, 
		(void*)&viewer
	);
	viewer->setCameraPosition(
		pose.position.x, 
		pose.position.y, 
		60, 
		pose.position.x + 1, 
		pose.position.y + 1, 
		0, 
		0, 
		0, 
		1
	);
	// Load the point cloud map and render it onto the PCL Viewer
	PointCloudT::Ptr mapCloud(new PointCloudT);
  	pcl::io::loadPCDFile(
		"map.pcd", 
		*mapCloud
	);
  	std::cout << "Loaded " << mapCloud->points.size();
	std::cout << " data points from 'map.pcd'" << "\n";
	renderPointCloud(
		viewer, 
		mapCloud, 
		"map", 
		Color(0, 0, 1)
	);
	// Define the ground-truth pose for the input scan
	std::vector<Pose3D> truePose = {
		Pose3D(Point3D(2.62296, 0.0384164, 0), 
			   Rotate(6.10189e-06, 0, 0)
		), 
		Pose3D(Point3D(4.91308, 0.0732088, 0),
			   Rotate(3.16001e-05, 0, 0)
		)
	};
	drawCar(
		truePose[0], 
		0, 
		Color(1, 0, 0), 
		0.7, 
		viewer
	);
	// Load `source` input scan
	PointCloudT::Ptr scanCloud(new PointCloudT);
  	pcl::io::loadPCDFile(
		"scan1.pcd", 
		*scanCloud
	);
	// Create a voxelised representation of the `source` point cloud
	typename pcl::PointCloud<PointT>::Ptr cloudFiltered(
		new pcl::PointCloud<PointT>
	);
	// TODO: Remove this line
	cloudFiltered = scanCloud;
	// TODO: Create voxel filter for input scan and save to `cloudFiltered`
	// ......
	PointCloudT::Ptr transformedScan(new PointCloudT);
	Tester tester;
	while (!viewer->wasStopped()) {
		Eigen::Matrix4d transform = transform3D(
			pose.rotation.yaw, 
			pose.rotation.pitch, 
			pose.rotation.roll, 
			pose.position.x, 
			pose.position.y, 
			pose.position.z
		);
		if (matching != Off) {
			if (matching == Icp) {
				 //TODO: Change the number of ICP iterations to positive number
				transform = ICP(mapCloud,
								cloudFiltered, 
								pose, 
								0
				);
			}
  			pose = getPose(transform);
			if (!tester.Displacement(pose)) {
				if (matching == Icp) {
					std::cout << " Done testing ICP" << "\n";
				}
				tester.Reset();
				double poseError = sqrt(
					(truePose[0].position.x - pose.position.x) 
					* (truePose[0].position.x - pose.position.x) 
					+ (truePose[0].position.y - pose.position.y) 
					* (truePose[0].position.y - pose.position.y)
				);
				std::cout << "Pose error: " << poseError << "\n";
				matching = Off;
			}
		}
  		pcl::transformPointCloud(
			*cloudFiltered, 
			*transformedScan, 
			transform
		);
		viewer->removePointCloud("scan");
		renderPointCloud(
			viewer, 
			transformedScan, 
			"scan", 
			Color(1, 0, 0)
		);
		viewer->removeShape("box1");
		viewer->removeShape("boxFill1");
		drawCar(
			pose, 
			1, 
			Color(0,1,0), 
			0.35, 
			viewer
		);
  		viewer->spinOnce();
  	}
	return 0;
}

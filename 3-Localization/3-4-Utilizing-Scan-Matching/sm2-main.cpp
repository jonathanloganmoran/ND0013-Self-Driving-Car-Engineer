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


#include "helpers.h"
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

/*** Setting the initial state variables ***/
Pose3D pose(Point3D(0, 0, 0), Rotate(0, 0, 0));
Pose3D savedPose = pose;

/*** Defining the NDT hyperparameters ***/
// The maximum number of NDT iterations to perform before termination.
// Each iteration the NDT algorithm attempts to improve the accuracy of the
// transformation. Default: 150, Rule of thumb: start with default and
// decrease or increase depending on size and complexity of data set.
const static int kMaximumIterationsNDT = 150;
// The step size taken for each iteration of the NDT algorithm.
// Used in the More-Thuente line search to determine how much the
// transformation matrix is updated at each iteration. A larger step size
//  will lead to faster convergence, but may lead to inaccurate results. 
// Default: 0.1, Rule of thumb: decrease if NDT is coverging too quickly.
const static double kStepSizeNDT = 1.0;
// The transformation epsilon threshold for the NDT algorithm.
// The maximum epsilon threshold between the previous and current estimated  
// transformation. Rule of thumb: set between 1e-4 and 1e-8.
const static double kTransformationEpsilonNDT = 1e-3;
// The resolution of the NDT `VoxelGridCovariance`
// i.e., the resolution side length of the 3D voxel to use for discretisation
// in the NDT algorithm. Here we assume a cubioid, i.e., each of the sides
// (`lx`, `ly`, `lz`) have the same dimensions according to what is set here.
const static double kVoxelGridCovarianceNDT = 1.0;

/*** Setting voxel grid hyperparameters ***/
// Resolution of each 3D voxel ('box') used to downsample the point cloud
// Here we assume a cuboid, i.e., each of the sides (`lx`, `ly`, `lz`) have
// the same dimensions according to what is set here (in metres).
const static double kLeafSizeVoxelGrid = 0.5;


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
		while (pose.rotation.yaw > 2 * M_PI) {
			pose.rotation.yaw -= 2 * M_PI;
		}
  	}
	else if (event.getKeySym() == "l" && event.keyDown()) {
		matching = Off;
		pose.rotation.yaw -= 0.1;
		while (pose.rotation.yaw < 0) {
			pose.rotation.yaw += 2 * M_PI;
		} 
  	}
	// IMPLEMENTED IN `sm1-main.cpp`:
	// else if (event.getKeySym() == "i" && event.keyDown()) {
	// 	matching = Icp;
  	// }
	else if (event.getKeySym() == "n" && event.keyDown()) {
		matching = Ndt;
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


/* Implements the Normal Distributions Transform (NDT) algorithm.
 *
 * The 3D Normal Distributions Transform (NDT) in Point Cloud Library (PCL)
 * implements the Magnussen, M. et al., 2009 paper titled
 * <b> The Three-Dimensional Normal-Distributions Transform — an Efficient
 * Representation for Registration, Surface Analysis, and Loop Detection.
 * PhD thesis, Örebro Studies in Technology, 36(1):1-201. Örebro universitet.
 * 2009. </b>
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
	// Construct the initial 3D transformation matrix estimate
	Eigen::Matrix4f initialGuess = transform3D(
		startingPose.rotation.yaw,
		startingPose.rotation.pitch,
		startingPose.rotation.roll,
		startingPose.position.x,
		startingPose.position.y,
		startingPose.position.z
	).cast<float>();
	/*** Compute the scan matching registration with the NDT algorithm ***/
	pcl::console::TicToc time;
	time.tic();
	// Setting the maximum iterations
	ndt.setMaximumIterations(iterations);
	// Set the input `source` point cloud
	ndt.setInputSource(source);
	// Perform the registration (alignment) with NDT
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputNDT(
		new pcl::PointCloud<pcl::PointXYZ>
	);
	ndt.align(*outputNDT, initialGuess);
	std::cout << "Finished NDT alignment in " << time.toc() << " ms" << "\n";
	std::cout << "NDT converged: " << std::boolalpha << ndt.hasConverged();
	std::cout << ", Fitness score: " << ndt.getFitnessScore() << "\n";
	// Check if NDT algorithm has converged
	if (ndt.hasConverged()) {
		// Get the estimated transformation matrix
		transformationMatrix = ndt.getFinalTransformation().cast<double>();
		// Return estimated transformation matrix
		return transformationMatrix; 
	}
  	else {
		std::cout << "WARNING: NDT did not converge" << "\n";
		// Return the identity matrix (i.e., apply no transformation)
		return transformationMatrix;
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
void DrawCar(
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
struct Tester {
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

	Tester()
		: pose(), init(), cycles(), timer(), distThresh(), 
		  angleThresh(), distHistory(), angleHistory() {}
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
		return std::abs(
			fmod(angle + M_PI, 2 * M_PI) - M_PI
		);
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
		double adist = std::max(
			std::max(angleMag(movement.rotation.yaw),
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


/* Loads the set of point cloud scans into the given vector.  
 *
 * Assumes that the scans exist in the current working directory
 * and have a uniform naming convention 'scan' postfixed with the
 * respective scan number, i.e., 'scan1'. All point cloud object
 * files are assumed to have the Point Cloud Library `pcd`
 * extension.
 * 
 * @param  scans	Address of vector to store the scans in.
 * @param  num		Number of scans to load from filesystem.
 */
void LoadScans(
		std::vector<PointCloudT::Ptr>& scans, 
		int num
) {
	for (int i = 0; i < num; i++) {
		// Get the next scan 'i' in the current working directory
		std::string inputFileName = "scan" + std::to_string(i + 1) + ".pcd";
		PointCloudT::Ptr inputPCD(new PointCloudT);
		int retVal;
		retVal = pcl::io::loadPCDFile(
			kBasePath + inputFileName, 
			*inputPCD
		);
		if (retVal != -1) {
			std::cout << "Successfully loaded '" << inputFileName <<"'" << "\n";
			scans.push_back(inputPCD);
		}
		else {
			std::cout << "Error loading '" << inputFileName << "'" << "\n";
		}
	}
}


/* Runs and visualises the NDT scan matching registration programme.
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
 * Once the transformation has been computed by the PCL NDT algorithm,
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
		KeyboardEventOccurred, 
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
	// Load the starting point cloud map and render it onto the PCL Viewer
	std::string inputFileName = "map.pcd"; 
	PointCloudT::Ptr mapCloud(new PointCloudT);
	int retVal;
  	retVal = pcl::io::loadPCDFile(
		kBasePath + inputFileName, 
		*mapCloud
	);
	if (retVal == -1) {
		std::cout << "Error loading '" << inputFileName << "'" << "\n";
		return retVal;
	}
  	std::cout << "Loaded " << mapCloud->points.size();
	std::cout << " data points from '" + inputFileName + "'" << "\n";
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
	DrawCar(
		truePose[0], 
		0, 
		Color(1, 0, 0), 
		0.7, 
		viewer
	);
	// Create list to store '*.pcd' files from local filesystem
	std::vector<PointCloudT::Ptr> scans;
	// Load the input '*.pcd' files (the `source` point cloud scans)
	LoadScans(
		scans,
		kNumInputScansToLoad
	);
	// Initialise and configure the NDT algorithm with static `target`
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	// Setting the NDT hyperparameters
	ndt.setStepSize(kStepSizeNDT);
	ndt.setTransformationEpsilon(kTransformationEpsilonNDT);
	ndt.setResolution(kVoxelGridCovarianceNDT);
	// Configuring the `target` point cloud only once
	ndt.setInputTarget(mapCloud);
	// Create a voxel filter to downsample the input point cloud scans
	pcl::VoxelGrid<PointT> voxelGrid;
	// Downsample the first input scan
	voxelGrid.setInputCloud(scans[0]);
	// Here we create 3D cuboid filter with equal dimensions (`lx`, `ly`, `lz`)
	voxelGrid.setLeafSize(
		kLeafSizeVoxelGrid, 
		kLeafSizeVoxelGrid, 
		kLeafSizeVoxelGrid
	);
	// Create a voxelised (downsampled) representation of `source` point cloud
	typename pcl::PointCloud<PointT>::Ptr cloudFiltered(
		new pcl::PointCloud<PointT>
	);
	voxelGrid.filter(*cloudFiltered);
	// Get the `target` point cloud from the user-entered offset
	PointCloudT::Ptr transformedScan(new PointCloudT);
	Tester tester;
	while (!viewer->wasStopped()) {
		// Compute the 3D transformation matrix of the starting pose
		Eigen::Matrix4d transform = transform3D(
			pose.rotation.yaw, 
			pose.rotation.pitch, 
			pose.rotation.roll, 
			pose.position.x, 
			pose.position.y, 
			pose.position.z
		);
		// If the user-entered transform is not aligned with original
		if (matching != Off) {
			// Using the NDT algorithm, align the offset `target` and `source`
			if (matching == Ndt) {
				// Here we set the number of NDT alignment steps to perform
				// as a positive number (`kMaxmimumIterationsNDT`) 
				transform = NDT(ndt,
								cloudFiltered, 
								pose, 
								kMaximumIterationsNDT
				);
			}
			// Compute the pose for this transformation 
  			pose = getPose3D::getPose(transform);
			// Run displacement animation 
			if (!tester.Displacement(pose)) {
				if (matching == Ndt) {
					// Print confirmation that NDT was performed
					std::cout << " Done testing NDT" << "\n";
				}
				// Reset the user-entered displacement tracker
				tester.Reset();
				// Compute the estimation error between the ground-truth pose
				// and the NDT estimated alignment from the user-entered pose
				double poseError = sqrt(
					(truePose[0].position.x - pose.position.x) 
					* (truePose[0].position.x - pose.position.x) 
					+ (truePose[0].position.y - pose.position.y) 
					* (truePose[0].position.y - pose.position.y)
				);
				std::cout << "Pose error: " << poseError << "\n";
				// Update the flag to allow for new user-entered pose
				matching = Off;
			}
		}
		// Compute the transformation between the `source` point cloud
		// and the `target` pose from the user-entered displacement
  		pcl::transformPointCloud(
			*cloudFiltered, 
			*transformedScan, 
			transform
		);
		// Update the PCL Viewer with the new transformation
		viewer->removePointCloud("scan");
		renderPointCloud(
			viewer, 
			transformedScan, 
			"scan", 
			Color(1, 0, 0)
		);
		// Remove the objects associated with the first scan (i.e., `scans[0]`)
		viewer->removeShape("box1");
		viewer->removeShape("boxFill1");
		DrawCar(
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
/* ----------------------------------------------------------------------------
 * Project "3.1: Scan Matching Localization"
 * Authors     : Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Main entry into the scan matching, mapping and
 * 						 localisation programme.
 *
 * Note : This programme is intended to be used with CARLA Simulator and the
 * 	      Point Cloud Library (PCL) in order to perform scan matching, mapping,
 * 		  and localisation.
 * ----------------------------------------------------------------------------
 */


#include "helpers.h"
#include <carla/client/Client.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Map.h>
#include <carla/geom/Location.h>
#include <carla/geom/Transform.h>
#include <carla/client/Sensor.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/client/Vehicle.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>
#include <string>
#include <sstream>
#include <thread>
#include <chrono> 
#include <ctime> 


/*** Defining the programme parameters ***/
// Define the scan matching / registration algorithm to use
// If `USE_ICP` is set to `false`, then the NDT algorithm will be used
const static bool USE_ICP = true;
// Set the base path relative to CWD where '.pcd' files are stored / saved
const static std::string kBasePathRel = "../";
// Minimum distance of the LiDAR scan points to preserve (in metres).
// A LiDAR return must be at least this far away from the sensor origin
// in order to be preserved. All LiDAR points with a distance less than
// this threshold will be clipped (ignored). This is to prevent LiDAR
// returns from including points reflected off the ego-vehicle.
const static double kMinimumDistanceLidarDetections = 8.0;
// Maximum number of map scan points to store in memory.
// All scans recieved after this number of points have been stored
// will be ignored.
const static int kMaximumScanPointsMap = 5000;

/*** Defining the ICP hyperparameters ***/
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


/*** Setting the intial state variables ***/
PointCloudT pclCloud;
carla::client::Vehicle::Control control;
std::chrono::time_point<std::chrono::system_clock> currentTime;
std::vector<ControlState> controlStateHistory;
bool refreshView = false;


/* Event handler that updates the PCL Viewer and vehicle state.
 *
 * @param   event   `KeyboardEvent` containing the pressed key.
 * @param   viewer  PCL Viewer instance from which the event was created.
 */
void KeyboardEventOccurred(
        const pcl::visualization::KeyboardEvent &event,
        void* viewer
) {
      //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
    if (event.getKeySym() == "Right" && event.keyDown()) {
        controlStateHistory.push_back(
            ControlState(0, -0.02, 0)
        );
      }
    else if (event.getKeySym() == "Left" && event.keyDown()) {
        controlStateHistory.push_back(
            ControlState(0, 0.02, 0)
        );
      }
      if (event.getKeySym() == "Up" && event.keyDown()) {
        controlStateHistory.push_back(
            ControlState(0.1, 0, 0)
        );
      }
    else if (event.getKeySym() == "Down" && event.keyDown()) {
        controlStateHistory.push_back(
            ControlState(-0.1, 0, 0)
        );
      }
    if(event.getKeySym() == "a" && event.keyDown()) {
        refreshView = true;
    }
}


/* Sets the simulated LiDAR sensor attributes used in the CARLA Simulator.
 *
 * Here the attribute values are pre-determined, but one can optionally
 * initialise them using the recommended values from CARLA by making use
 * of the `attribute.GetAttribute().GetRecommendedValues()` member function.
 *  
 * @param  lidarSensor	Initialised pointer to the LiDAR sensor instance.
 */
void SetLidarAttributes(
		auto& lidarSensor
) {
	// CANDO: Modify LiDAR parameter values to get different scan resolutions
    lidarSensor.SetAttribute(
        "upper_fov", "15"
    );
    lidarSensor.SetAttribute(
        "lower_fov", "-25"
    );
    lidarSensor.SetAttribute(
        "channels", "32"
    );
    lidarSensor.SetAttribute(
        "range", "30"
    );
    lidarSensor.SetAttribute(
        "rotation_frequency", "30"
    );
    lidarSensor.SetAttribute(
        "points_per_second", "500000"
    );
}

/* Checks if the given detection is within the vehicle bounds.
 *
 * This function returns `true` if a detection point has a sum of squared
 * distance less than or equal to the maximum threshold set by constant
 * `kMinimumDistanceLidarDetections`. In other words, a point with a distance
 * less than this threshold is assumed to have been generated from a reflection
 * off of the ego-vehicle. All points inside this detection area are ignored.
 *
 * @param  	 detection	LiDAR detection point to check.
 * @returns  bool		True if the point is within the limit.
 */
bool InEgoVehicleRange(
        auto& detection
) {
    return (((detection.point.x * detection.point.x)
             + (detection.point.y * detection.point.y)
             + (detection.point.z * detection.point.z)
            ) <= kMinimumDistanceLidarDetections
    );
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


/* Handles the vehicle state, i.e., simulates throttle, steering and brake.
 *
 * @param  response	 Current `ControlState` to update the vehicle state with.
 * @param  state	 Previous vehicle state to update.
 */
void Actuate(
        ControlState response,
        carla::client::Vehicle::Control& state
) {
    if (response.t > 0) {
        // Increase the throttle if vehicle is not in reverse
        if (!state.reverse) {
            state.throttle = std::min(
                state.throttle + response.t,
                1.0f
            );
        }
        else {
            // Set the vehicle throttle in forward direction
            state.reverse = false;
            state.throttle = std::min(
                response.t,
                1.0f
            );
        }
    }
    else if (response.t < 0) {
        // Decrease the throttle
        response.t = -response.t;
        if (state.reverse) {
            state.throttle = std::min(
                state.throttle + response.t,
                1.0f
            );
        }
        else {
            state.reverse = true;
            state.throttle = std::min(
                response.t,
                1.0f
            );
        }
    }
    // Set the steering wheel angle
    state.steer = std::min(
        std::max(state.steer + response.s, -1.0f),
        1.0f
    );
    // Set the brake state
    state.brake = response.b;
}


/* Fetches the vehicle pose and updates the PCL Viewer and control vector.
 *
 * A new pose is obtained from the `vehicle` which is rendered on the PCL
 * Viewer. The vehicle control history is updated and a new move is initiated.
 *
 * @param  viewer	 PCL Viewer instance to render the new pose in.
 * @param  vehicle	 CARLA `Vehicle` instance generating a new transform.
 * @param  truePose  Ground-truth pose to set with info from CARLA. 
 * @param  poseRef	 Last known 3D pose of the `vehicle`.
 * @param  control	 Current control state of the vehicle to update.
 * @param  controlStateHistory  Vector of previous `ControlState` instances.
 * */
void UpdatePoseAndControl(
        pcl::visualization::PCLVisualizer::Ptr& viewer,
        auto& vehicle,
        Pose3D& truePose,
        Pose3D& poseRef,
		auto& control,
        std::vector<ControlState>& controlStateHistory
) {
	// Update the PCL Viewer with the latest vehicle pose and control state
	viewer->removeShape("box0");
	viewer->removeShape("boxFill0");
    // Render the updated ground-truth vehicle pose in red
    truePose = Pose3D(
        Point3D(vehicle->GetTransform().location.x,
                vehicle->GetTransform().location.y,
                vehicle->GetTransform().location.z
        ),
        Rotate(vehicle->GetTransform().rotation.yaw * M_PI / 180,
               vehicle->GetTransform().rotation.pitch * M_PI / 180,
               vehicle->GetTransform().rotation.roll * M_PI / 180
        )
    ) - poseRef;
    // Render the updated pose
    DrawCar(
        truePose,
        0,
        Color(1, 0, 0),
        0.7,
        viewer
    );
    // Render the updated vehicle steering / heading angle in green
    double theta = truePose.rotation.yaw;
    double stheta = control.steer * M_PI / 4 + theta;
    viewer->removeShape("steer");
    renderRayT::renderRay(
        viewer,
        PointT(truePose.position.x + 2 * cos(theta),
               truePose.position.y + 2 * sin(theta),
               truePose.position.z

        ),
        PointT(truePose.position.x + 4 * cos(stheta),
               truePose.position.y + 4 * sin(stheta),
               truePose.position.z
        ),
        "steer",
        Color(0, 1, 0)
    );
	// Initialise new control state with brake `b` set to 1.0
	ControlState actuate(0, 0, 1);
	if(controlStateHistory.size() > 0){
		actuate = controlStateHistory.back();
		controlStateHistory.clear();
		Actuate(actuate, control);
		vehicle->ApplyControl(control);
	}
}


/* Computes the error between the estimated and ground-truth vehicle pose.
 *
 * @param  viewer		PCL Viewer instance to update with the current error.
 * @param  pose			Estimated vehicle pose calculated with scan matching.
 * @param  truePose		Ground-truth vehicle pose obtained from CARLA.
 * @param  maxError		Current maximum distance error.
 * 
 */
void UpdatePoseError(
	pcl::visualization::PCLVisualizer::Ptr& viewer,
	Pose3D& pose,
	Pose3D& truePose,
	double& maxError
	
) {
	double poseError = sqrt(
		(truePose.position.x - pose.position.x) 
		* (truePose.position.x - pose.position.x) 
		+ (truePose.position.y - pose.position.y) 
		* (truePose.position.y - pose.position.y)
	);
	if (poseError > maxError) {
		maxError = poseError;
	}
	double distDriven = sqrt(
		(truePose.position.x * truePose.position.x) 
		+ (truePose.position.y * truePose.position.y)
	);
	viewer->removeShape("maxE");
	viewer->addText(
		"Max Error: " + std::to_string(maxError) + " m", 
		200, 100, 32, 1.0, 1.0, 1.0, "maxE", 0
	);
	viewer->removeShape("derror");
	viewer->addText(
		"Pose error: " + std::to_string(poseError) + " m", 
		200, 150, 32, 1.0, 1.0, 1.0, "derror", 0
	);
	viewer->removeShape("dist");
	viewer->addText("Distance: " + std::to_string(distDriven) + " m", 
	200, 200, 32, 1.0, 1.0, 1.0, "dist", 0
	);
	if ((maxError > 1.2) || (distDriven >= 170.0)) {
		viewer->removeShape("eval");
	}
	if (maxError > 1.2) {
		viewer->addText(
			"Try Again", 
			200, 50, 32, 1.0, 0.0, 0.0, "eval", 0
		);
	}
	else {
		viewer->addText(
			"Passed!", 
			200, 50, 32, 0.0, 1.0, 0.0, "eval", 0
		);
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
 * @returns  transformationMatrix
 */
Eigen::Matrix4d ICP(
        PointCloudT::Ptr target,
        PointCloudT::Ptr source,
        Pose3D startingPose,
        int iterations
) {
    // Initialising the output as the 4x4 identity matrix
      Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
      // Construct the 3D transformation matrix from the `startingPose`
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
    );
    /*** Compute the scan matching registration with the ICP algorithm ***/
    // Start a `TicToc` time tracking instance to profile the ICP algorithm
    pcl::console::TicToc time;
    time.tic();
    // Instantiate the ICP class
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    // Set the input `source` and `target` point clouds
    icp.setInputSource(sourceTransformed);
    icp.setInputTarget(target);
    // Set the ICP hyperparameters
    icp.setMaxCorrespondenceDistance(kMaxCorrespondenceDistanceICP);
    icp.setMaximumIterations(iterations);
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
        return transformationMatrix;
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
 * Note: the `source` point cloud is first transformed into the coordinate
 * frame of the given `startingPose` before it is registered to the `target`.
 *
 * @param  target		 Reference point cloud to align the `source` to.
 * @param  source		 Starting point cloud to align to the `target`.
 * @param  startingPose  3D pose to transform the `source` point cloud by.
 * @param  iterations	 Maximum number of iterations to run NDT for.
 * @returns transformationMatrix
 */
Eigen::Matrix4d NDT(
        PointCloudT::Ptr target,
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
	// Initialise the NDT instance
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	// Setting the NDT hyperparameters
    ndt.setMaximumIterations(iterations);
	ndt.setStepSize(kStepSizeNDT);
	ndt.setTransformationEpsilon(kTransformationEpsilonNDT);
	ndt.setResolution(kVoxelGridCovarianceNDT);
    // Set the input `source` point cloud
    ndt.setInputSource(source);
	// Set the input `target` point cloud
	ndt.setInputTarget(target);
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


/*
 *
 */
int main() {
	/*** Initialise the CARLA Simulator ***/
    carla::client::Client client(
        "localhost",
        2000
    );
    client.SetTimeout(std::chrono::seconds(2));
	/*** Setting actors and blueprint ***/
	// Creating and spawning a vehicle actor and map from the blueprint
	// Load an empty world
	auto world = client.GetWorld();
	// Load the vehicle blueprint
	auto blueprintLibrary = world.GetBlueprintLibrary();
	auto vehicles = blueprintLibrary->Filter("vehicle");
	// Set the spawn point
	auto map = world.GetMap();
	auto spawnPointTransform = map->GetRecommendedSpawnPoints()[1];
	// Spawn the ego-vehicle at the spawn point
	auto egoActor = world.SpawnActor(
		(*vehicles)[12],
		spawnPointTransform
	);
	/*** Creating and configuring a simulated LiDAR sensor ***/
    // Create a new simulated LiDAR sensor
    auto lidarSensor = *(
		blueprintLibrary->Find("sensor.lidar.ray_cast")
	);
	// CANDO: Modify lidar values to get different scan resolutions
	// inside the `SetLidarAttributes` function
	SetLidarAttributes(lidarSensor);
	/*** Configuring the initial vehicle and sensor pose ***/
    auto userOffset = carla::geom::Location(0, 0, 0);
    auto lidarTransform = carla::geom::Transform(
        carla::geom::Location(-0.5, 0, 1.8) + userOffset
    );
    auto lidarActor = world.SpawnActor(
        lidarSensor,
        lidarTransform,
        egoActor.get()
    );
    auto lidar = boost::static_pointer_cast<
        carla::client::Sensor
    >(lidarActor);
    // Set the scan matching algorithm state to 'initialise'
    bool initScan = true;
	// Initialise the state variable to accept incoming LiDAR scans
    bool fetchNewScan = true;
	// Initialise new time tracking instances 
	std::chrono::time_point<
		std::chrono::system_clock
	> lastScanTime, startTime;
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer")
    );
    viewer->setBackgroundColor(0, 0, 0);
    viewer->registerKeyboardCallback(
        KeyboardEventOccurred,
        (void*)&viewer
    );
    auto vehicle = boost::static_pointer_cast<
        carla::client::Vehicle
    >(egoActor);
	// Initialise the estimated vehicle pose
	Pose3D pose(Point3D(0, 0, 0), Rotate(0, 0, 0));
	// Loading the map point cloud from local file system
	std::string inputFilename = "map.pcd";
	PointCloudT::Ptr mapCloud(new PointCloudT);
	int retVal;
	retVal = pcl::io::loadPCDFile(
		kBasePathRel + inputFilename,
		*mapCloud
	);
	if (retVal != -1) {
		std::cout << "Successfully loaded '" << inputFilename <<"'" << "\n";
		std::cout << "Map contains " << mapCloud->points.size();
		std::cout << " data points" << "\n";
	}
	else {
		std::cout << "Error loading '" << inputFilename << "'" << "\n";
	}
	// Render the map point cloud in blue onto the PCL Viewer 
	renderPointCloud(
		viewer, 
		mapCloud, 
		"map", 
		Color(0, 0, 1)); 
	// Initialising our downsampled point cloud map and incoming scan 
	typename pcl::PointCloud<PointT>::Ptr cloudFiltered(
		new pcl::PointCloud<PointT>
	);
	typename pcl::PointCloud<PointT>::Ptr scanCloud(
		new pcl::PointCloud<PointT>
	);
	// Listening for incoming LiDAR scans to process
	lidar->Listen(
		[&fetchNewScan, &lastScanTime, &scanCloud](auto data) 
	{
		if (fetchNewScan) {
			// Get a new LiDAR scan
			auto scan = boost::static_pointer_cast<
				carla::sensor::data::LidarMeasurement
			>(data);
			for (auto detection : *scan) {
				if (!InEgoVehicleRange(detection)) { 
					// Don't include points touching the ego-vehicle
					pclCloud.points.push_back(
						PointT(detection.point.x, 
							   detection.point.y, 
							   detection.point.z
						)
					);
				}
			}
			// CANDO: Modify this value to get different mapping resolutions
			if (pclCloud.points.size() > kMaximumScanPointsMap) {
				// Get the current time to stamp the input scan
				lastScanTime = std::chrono::system_clock::now();
				*scanCloud = pclCloud;
				// Stop processing any new scans after this one
				fetchNewScan = false;
			}
		}
	});
	// Get the reference pose of the vehicle from the simulator
	// i.e., the ground-truth position and orientation at this time-step
	Pose3D poseRef(
		Point3D(vehicle->GetTransform().location.x, 
				vehicle->GetTransform().location.y, 
				vehicle->GetTransform().location.z
		), 
		Rotate(vehicle->GetTransform().rotation.yaw * M_PI / 180, 
			   vehicle->GetTransform().rotation.pitch * M_PI / 180, 
			   vehicle->GetTransform().rotation.roll * M_PI / 180
		)
	);
	// Store maximum distance (error) between estimated and ground-truth pose
	double maxError = 0;
	while (!viewer->wasStopped()) {
		while (fetchNewScan) {
			std::this_thread::sleep_for(
				std::chrono::milliseconds(100)
			);
			world.Tick(
				std::chrono::seconds(1)
			);
		}
		if (refreshView) {
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
			refreshView = false;
		}
		// Update the PCL Viewer with the latest vehicle pose and control state
		Pose3D truePose;
		UpdatePoseAndControl(
			viewer,
			vehicle,
			truePose,
			poseRef,
			control,
			controlStateHistory
		);
  		viewer->spinOnce();
		// Compute the pose transformation using the scan matching algorithm
		if (!fetchNewScan) {
			if (initScan == true) {
				// Use the ground-truth vehicle pose to initialise the estimate 
				pose.position = truePose.position;
				pose.rotation = truePose.rotation;
			}
			// Done processing latest scan, set flag to fetch new scan
			fetchNewScan = true;
			// TODO: (Filter scan using voxel filter)
			pcl::VoxelGrid<PointT> voxelGrid;
			voxelGrid.setInputCloud(scanCloud);
			voxelGrid.setLeafSize(
				kLeafSizeVoxelGrid,
				kLeafSizeVoxelGrid,
				kLeafSizeVoxelGrid
			);
			typename pcl::PointCloud<PointT>::Ptr cloudFiltered(
				new pcl::PointCloud<PointT>
			);
			voxelGrid.filter(*cloudFiltered);
			// Compute the pose transform using ICP or NDT scan matching
			Eigen::Matrix4d transformEstimate = USE_ICP ? ICP(
				mapCloud,
				cloudFiltered,
				pose,
				kMaximumIterationsICP
			) : NDT(
				mapCloud,
				cloudFiltered,
				pose,
				kMaximumIterationsNDT
			);
			// Get the pose associated with this transformation estimate
			pose = getPose3D::getPose(transformEstimate);
			// TODO: Transform scan so it aligns with ego's actual pose and render that scan
			PointCloudT::Ptr correctedScan(new PointCloudT);
			pcl::transformPointCloud(
				*cloudFiltered,
				*correctedScan,
				transformEstimate
			);
			viewer->removePointCloud("scan");
			// TODO: Change `scanCloud` below to your transformed scan
			renderPointCloud(
				viewer, 
				correctedScan,
				"scan", 
				Color(1, 0, 0)
			);
			viewer->removeAllShapes();
			DrawCar(
				pose, 
				1, 
				Color(0, 1, 0), 
				0.35, 
				viewer
			);
			UpdatePoseError(
				viewer,
				pose,
				truePose,
				maxError
			);
			pclCloud.points.clear();
  		}
	}
	return 0;
}

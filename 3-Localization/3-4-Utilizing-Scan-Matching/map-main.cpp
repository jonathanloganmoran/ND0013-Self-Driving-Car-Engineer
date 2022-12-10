/* ----------------------------------------------------------------------------
 * Lesson "3.4: Utilizing Scan Matching"
 * Authors     : Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Main entry into the scan matching and mapping
 * 						 programme. 
 * 
 * Note : This programme is intended to be used with CARLA Simulator and the
 * 	      Point Cloud Library (PCL) in order to run scan matching / mapping.
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
#include <string>
#include <sstream>
#include <thread>
#include <chrono> 
#include <ctime> 

/*** Defining the programme parameters ***/
// Set the base path relative to CWD where '.pcd' files are stored / saved
const static std::string kBasePathRel = "../";
// Maximum number of points to store in memory.
// All scans recieved after this number of points have been stored
// will be ignored. 
const static int kMaximumScanPoints = 2000;
// Minimum elapsed time between scan intervals (in seconds).
// All new scans recieved before this many seconds has passed since
// the previous scan had been recieved will be discarded, i.e., the
// `scanPositions` vector will not be updated to include the new scan.
const static double kMinimumTimeBetweenScans = 1.0;
// Minimum distance of the LiDAR scan points to preserve (in metres).
// A LiDAR return must be at least this far away from the sensor origin
// in order to be preserved. All LiDAR points with a distance less than
// this threshold will be clipped (ignored). This is to prevent LiDAR
// returns from including points reflected off the ego-vehicle.
const static double kMinimumDistanceLidarDetections = 8.0;
// Minimum distance between the current pose and all existing scans
// A new pose must not have a distance to any of the existing scans
// less than this threshold.
const static double kMinimumDistanceBetweenScans = 5.0; 

/*** Setting the intial state variables ***/
PointCloudT pclCloud;
carla::client::Vehicle::Control control;
std::chrono::time_point<std::chrono::system_clock> currentTime;
std::vector<ControlState> controlStateHistory;
bool refresh_view = false;
bool save_map = false;


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
		refresh_view = true;
	}
	if(event.getKeySym() == "s" && event.keyDown()) {
		save_map = true;
	}
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
	return ((
			 (detection.point.x * detection.point.x)
			 + (detection.point.y * detection.point.y)
			 + (detection.point.z * detection.point.z)
			) <= kMinimumDistanceLidarDetections
	);
}


/* Fetches the vehicle pose and updates the PCL Viewer and control vector.
 * 
 * A new pose is obtained from the `vehicle` which is rendered on the PCL
 * Viewer. The vehicle control history is updated and a new move is initiated.
 *
 * @param  viewer	PCL Viewer instance to render the new pose in.	
 * @param  vehicle	CARLA `Vehicle` instance generating a new transform.
 * @param  poseRef	Last known 3D pose of the `vehicle`.
 * @param  controlStateHistory  Vector of previous `ControlState` instances.
 * */
void UpdatePoseAndControl(
		pcl::visualization::PCLVisualizer::Ptr& viewer, 
		boost::static_pointer_cast<carla::client::Vehicle>& vehicle,
		Pose3D& poseRef
		std::vector<ControlState>& controlStateHistory
) {
	// Clear the PCL Viewer canvas
	viewer->removeAllShapes();
	// Compute the next pose from the transform
	pose = Pose3D(
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
		pose,
		0,
		Color(1, 0, 0);
		0.7,
		viewer
	);
	// Update the steering angle
	double theta = pose.rotation.yaw;
	double stheta = control.steer * M_PI / 4 + theta;
	viewer->removeShape("steer");
	renderRay(
		viewer,
		Point3D(pose.position.x + 2 * cos(theta), 
				pose.position.y + 2 * sin(theta),
				pose.position.z

		),
		Point3D(pose.position.x + 4 * cos(stheta), 
				pose.position.y + 4 * sin(stheta),
				pose.position.z
		),
		"steer",
		Color(0, 1, 0)
	);
	// Update the control state
	ControlState actuate(0, 0, 1);
	if (!controlStateHistory.size()) {
		return;
	}
	else {
		actuate = controlStateHistory.back();
		controlStateHistory.clear();
		Actuate(actuate, control);
		vehicle->ApplyControl(control);
		return;
	}
}


/* Processes the pose computed from the LiDAR scan.
 * 
 * The given `pose` is saved if both conditions are true:
 *    (1) Distance between `pose` and previous scans is less than threhsold;
 *    (2) Elapsed time between the consecutive scans is greater than threshold.
 * If both are true, the position information of the `pose` is extracted
 * and added to the `scanPositions` vector. The corresponding state variable
 * `fetchNewScan` is reset to `true` so that a new scan can be received.
 * 
 * @param  fetchNewScan		    Indicates whether a new scan should be fetched.
 * @param  elapsedTimeLastScan  Time (s) between this scan and the previous.
 * @param  pose					3D pose computed from the new scan.
 * @param  scanPositions		State vector storing scan position history. 
 */
void ProcessScan(
	bool& fetchNewScan,
	std::chrono::duration<double>& elapsedTimeLastScan,
	Pose3D& pose,
	std::vector<Point3D>& scanPositions
) {
	if (fetchNewScan) {
		// No new LiDAR scan to process
		return;
	}
	// Get the 3D position information from this pose
	Point3D posePosition(
		pose.position.x,
		pose.position.y,
		pose.position.z
	);
	// Compute the minimum distance from this pose to all existing scans 
	double minPoseDistanceToScans = minDistance3D::minDistance(
		posePosition,
		scanPositions
	);
	if ((elapsedTimeLastScan.count() < kMinimumTimeBetweenScans)
		|| (minPoseDistanceToScans < kMinimumDistanceBetweenScans)
	) {
		// Do not update the scans with this pose
		return;
	}
	else {
		// Add the position of the `pose` to history
		scanPositions.push_back(posePosition);
		// Reset flag to accept new scans
		fetchNewScan = true;
		return;
	}
}

/* Saves the input point cloud to a `.pcd` file with the given filename. 
 * 
 * @param	 pclCloud		 Point cloud file to save as binary `.pcd`.
 * @param    outputFilename  Desired filename to save the PCD to.
 * @returns  retVal		     Return value from `pcl::io::savePCDFileASCII`,
 * 							 i.e., error saving if `retVal < 0`, otherwise `0`.
 */
int SavePointCloudToASCII(
		PointCloudT& pclCloud,
		std::string outputFilename
) {
	// Save the final point cloud map
	PointCloudT::Ptr scanCloud(new PointCloudT);
	*scanCloud = pclCloud;
  	scanCloud->width = scanCloud->points.size();
  	scanCloud->height = 1;
	// TODO: Downsample the map point cloud using a voxel filter
	int retVal = pcl::io::savePCDFileASCII(
		kBasePathRel + outputFilename, 
		*scanCloud
	);
	if (retVal < 0) {
		std::cout << "Error saving '" << outputFilename << "'" << "\n";
	}
	else {
		std::cout << "Saved '" << outputFilename << "' successfully" << "\n";
	}
	return retVal;
}


/* Runs the mapping programme using headless CARLA Simulator.
 * Assuming that a headless CARLA instance has been started, this programme
 * creates and initialises a new `Client`, `Vehicle` and `Lidar` sensor.
 * Here the LiDAR sensor attributes are configured and the results of the
 * mapping programme are visualised. Each new LiDAR scan is timestamped,
 * and, if a sufficient amount of time has elapsed between scans, it is
 * processed. All points within a distance assumed to be the ego-vehicle are
 * ignored. The pose information is extracted from each scan and the
 * ground-truth vehicle  
 *
 */
int main() {
	/*** Initialise the CARLA Simulator ***/
	auto client = carla::client::Client(
		"localhost", 
		2000
	);
	client.SetTimeout(5s);		// Seconds (s)
	auto world = client.GetWorld();
	auto blueprint_library = world.GetBlueprintLibrary();
	auto vehicles = blueprint_library->Filter("vehicle");
	auto map = world.GetMap();
	auto transform = map->GetRecommendedSpawnPoints()[1];
	auto ego_actor = world.SpawnActor(
		(*vehicles)[12], 
		transform
	);
	// Create a new simulated LiDAR sensor
	auto lidar_bp = *(blueprint_library->Find(
		"sensor.lidar.ray_cast"
	));
	// CANDO: Can modify LiDAR parameter values to get different scan resolutions
	lidar_bp.SetAttribute(
		"upper_fov", "15"
	);
    lidar_bp.SetAttribute(
		"lower_fov", "-25"
	);
    lidar_bp.SetAttribute(
		"channels", "32"
	);
    lidar_bp.SetAttribute(
		"range", "30"
	);
	lidar_bp.SetAttribute(
		"rotation_frequency", "30"
	);
	lidar_bp.SetAttribute(
		"points_per_second", "500000"
	);
	auto user_offset = carla::geom::Location(0, 0, 0);
	auto lidar_transform = carla::geom::Transform(
		carla::geom::Location(-0.5, 0, 1.8) + user_offset
	);
	auto lidar_actor = world.SpawnActor(
		lidar_bp, 
		lidar_transform, 
		ego_actor.get()
	);
	auto lidar = boost::static_pointer_cast<
		carla::client::Sensor
	>(lidar_actor);
	bool fetchNewScan = true;
	std::chrono::time_point<
		std::chrono::system_clock
	> lastScanTime;
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
	>(ego_actor);
	Pose3D pose(Point3D(0, 0, 0), Rotate(0, 0, 0));
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
	lidar->Listen([&fetchNewScan, &lastScanTime, &pose, &viewer](auto data) {
		if (fetchNewScan) {
			auto scan = boost::static_pointer_cast<
				carla::sensor::data::LidarMeasurement
			>(data);
			Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
			// TODO: Set transform to pose using transform3D()
			for (auto detection : *scan) {
				if (!InEgoVehicleRange(detection)) {
					// Don't include points touching ego
					Eigen::Vector4d local_point(
						detection.point.x, 
						detection.point.y, 
						detection.point.z, 
						1
					);
					// TODO: Multiply local_point by transform
					Eigen::Vector4d transform_point = local_point;
					pclCloud.points.push_back(
						PointT(transform_point[0], 
							   transform_point[1], 
							   transform_point[2]
						)
					);
				}
			}
			// CANDO: Can modify this value to get different scan resolutions
			if (pclCloud.points.size() > kMaximumScanPoints) {
				// Stop processing any new scans after this one
				fetchNewScan = false;
			}
			// Get the current time to stamp the input scan
			lastScanTime = std::chrono::system_clock::now();
			// Render the new scan onto the PCL Viewer 
			PointCloudT::Ptr scanCloud(new PointCloudT);
			*scanCloud = pclCloud;
			viewer->removeAllPointClouds();
			renderPointCloud(
				viewer, 
				scanCloud, 
				"map", 
				Color(0, 0, 1)
			); 
		}
	});
	// Save the position of the reference scan
	std::vector<Point3D> scanPositions = {
		Point3D(poseRef.position.x, 
				poseRef.position.y, 
				poseRef.position.z
		)
	};
	while (!viewer->wasStopped () && !save_map) {
		while (fetchNewScan) {
			std::this_thread::sleep_for(0.1s);	// Seconds (s)
			world.Tick(1s);						// Seconds (s)
		}
		if (refresh_view) {
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
			refresh_view = false;
		}
		UpdatePoseAndControl(
			viewer,
			vehicle,
			poseRef,
			controlStateHistory
		);
		viewer->spinOnce();
		currentTime = std::chrono::system_clock::now();
		// Compute the elapsed time since last scan
		std::chrono::duration<
			double
		> elapsedTimeLastScan = currentTime - lastScanTime;
		// Process the current scan w.r.t. elapsed time
		ProcessScan(
			fetchNewScan,
			elapsedTimeLastScan,
			pose,
			scanPositions
		);
  	}
	// Save and downsample the current point cloud map
	// TODO: Downsample the map point cloud using a voxel filter
	std::string pclOutputFilename = "my_map.pcd";
	SavePointCloudToASCII(
		pclCloud,
		pclOutputFilename
	);
	return 0;
}

#include <carla/client/Client.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Map.h>
#include <carla/geom/Location.h>
#include <carla/geom/Transform.h>
#include <carla/client/Sensor.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <thread>

#include <carla/client/Vehicle.h>

//pcl code
//#include "render/render.h"

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;

using namespace std;

#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include "helper.h"
#include <sstream>
#include <chrono> 
#include <ctime> 

PointCloudT pclCloud;
cc::Vehicle::Control control;
std::chrono::time_point<std::chrono::system_clock> currentTime;
vector<ControlState> cs;

bool refresh_view = false;
bool save_map = false;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer)
{

  	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
	if (event.getKeySym() == "Right" && event.keyDown()){
		cs.push_back(ControlState(0, -0.02, 0));
  	}
	else if (event.getKeySym() == "Left" && event.keyDown()){
		cs.push_back(ControlState(0, 0.02, 0)); 
  	}
  	if (event.getKeySym() == "Up" && event.keyDown()){
		cs.push_back(ControlState(0.1, 0, 0));
  	}
	else if (event.getKeySym() == "Down" && event.keyDown()){
		cs.push_back(ControlState(-0.1, 0, 0)); 
  	}

	if(event.getKeySym() == "a" && event.keyDown()){
		refresh_view = true;
	}
	if(event.getKeySym() == "s" && event.keyDown()){
		save_map = true;
	}
}

void Accuate(ControlState response, cc::Vehicle::Control& state){

	if(response.t > 0){
		if(!state.reverse){
			state.throttle = min(state.throttle+response.t, 1.0f);
		}
		else{
			state.reverse = false;
			state.throttle = min(response.t, 1.0f);
		}
	}
	else if(response.t < 0){
		response.t = -response.t;
		if(state.reverse){
			state.throttle = min(state.throttle+response.t, 1.0f);
		}
		else{
			state.reverse = true;
			state.throttle = min(response.t, 1.0f);

		}
	}
	state.steer = min( max(state.steer+response.s, -1.0f), 1.0f);
	state.brake = response.b;
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

int main(){

	
	auto client = cc::Client("localhost", 2000);
	client.SetTimeout(5s);
	auto world = client.GetWorld();

	auto blueprint_library = world.GetBlueprintLibrary();
	auto vehicles = blueprint_library->Filter("vehicle");

	auto map = world.GetMap();
	auto transform = map->GetRecommendedSpawnPoints()[1];
	auto ego_actor = world.SpawnActor((*vehicles)[12], transform);

	//Create lidar
	auto lidar_bp = *(blueprint_library->Find("sensor.lidar.ray_cast"));
	// CANDO: Can modify lidar values to get different scan resolutions
	lidar_bp.SetAttribute("upper_fov", "15");
    lidar_bp.SetAttribute("lower_fov", "-25");
    lidar_bp.SetAttribute("channels", "32");
    lidar_bp.SetAttribute("range", "30");
	lidar_bp.SetAttribute("rotation_frequency", "30");
	lidar_bp.SetAttribute("points_per_second", "500000");

	auto user_offset = cg::Location(0, 0, 0);
	auto lidar_transform = cg::Transform(cg::Location(-0.5, 0, 1.8) + user_offset);
	auto lidar_actor = world.SpawnActor(lidar_bp, lidar_transform, ego_actor.get());
	auto lidar = boost::static_pointer_cast<cc::Sensor>(lidar_actor);
	bool new_scan = true;
	std::chrono::time_point<std::chrono::system_clock> lastScanTime;

	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	auto vehicle = boost::static_pointer_cast<cc::Vehicle>(ego_actor);
	Pose pose(Point(0,0,0), Rotate(0,0,0));
	Pose poseRef(Point(vehicle->GetTransform().location.x, vehicle->GetTransform().location.y, vehicle->GetTransform().location.z), Rotate(vehicle->GetTransform().rotation.yaw * pi/180, vehicle->GetTransform().rotation.pitch * pi/180, vehicle->GetTransform().rotation.roll * pi/180));

	lidar->Listen([&new_scan, &lastScanTime, &pose, &viewer](auto data){

		if(new_scan){
			auto scan = boost::static_pointer_cast<csd::LidarMeasurement>(data);
	
			Eigen::Matrix4d transform= Eigen::Matrix4d::Identity ();
			// TODO: Set transform to pose using transform3D()
			for (auto detection : *scan){
				if((detection.point.x*detection.point.x + detection.point.y*detection.point.y + detection.point.z*detection.point.z) > 8.0){ // Don't include points touching ego
					Eigen::Vector4d local_point(detection.point.x, detection.point.y, detection.point.z, 1);
					Eigen::Vector4d transform_point = local_point; // TODO: Multiply local_point by transform
					pclCloud.points.push_back(PointT(transform_point[0], transform_point[1], transform_point[2]));
				}
	
			}
			if(pclCloud.points.size() > 5000) // CANDO: Can modify this value to get different scan resolutions
				new_scan = false;

			lastScanTime = std::chrono::system_clock::now();

			PointCloudT::Ptr scanCloud(new PointCloudT);
			*scanCloud = pclCloud;
			viewer->removeAllPointClouds();
			renderPointCloud(viewer, scanCloud, "map", Color(0,0,1)); 
			
		}

	});

	vector<Point> scanPoses = {Point(poseRef.position.x, poseRef.position.y, poseRef.position.z)};

	while (!viewer->wasStopped () && !save_map)
  	{
		while(new_scan){
			std::this_thread::sleep_for(0.1s);
			world.Tick(1s);
		}
		
		if(refresh_view){
			viewer->setCameraPosition(pose.position.x, pose.position.y, 60, pose.position.x+1, pose.position.y+1, 0, 0, 0, 1);
			refresh_view = false;
		}

		viewer->removeAllShapes();
		pose = Pose(Point(vehicle->GetTransform().location.x, vehicle->GetTransform().location.y, vehicle->GetTransform().location.z), Rotate(vehicle->GetTransform().rotation.yaw * pi/180, vehicle->GetTransform().rotation.pitch * pi/180, vehicle->GetTransform().rotation.roll * pi/180)) - poseRef;

		drawCar(pose, 0,  Color(1,0,0), 0.7, viewer);
		double theta = pose.rotation.yaw;
		double stheta = control.steer * pi/4 + theta;
		viewer->removeShape("steer");
		renderRay(viewer, Point(pose.position.x+2*cos(theta), pose.position.y+2*sin(theta),pose.position.z),  Point(pose.position.x+4*cos(stheta), pose.position.y+4*sin(stheta),pose.position.z), "steer", Color(0,1,0));
		
		ControlState accuate(0, 0, 1);
		if(cs.size() > 0){
			accuate = cs.back();
			cs.clear();

			Accuate(accuate, control);
			vehicle->ApplyControl(control);
		}

		viewer->spinOnce ();
		
      	double distanceRes = 5.0; // CANDO: Can modify this value
		double timeRes = 1.0; // CANDO: Can modify this value
		currentTime = std::chrono::system_clock::now();
		if(!new_scan){
			std::chrono::duration<double> last_scan_seconds = currentTime - lastScanTime; 
			if(last_scan_seconds.count() > timeRes && minDistance(Point(pose.position.x, pose.position.y, pose.position.z), scanPoses) > distanceRes){
				scanPoses.push_back(Point(pose.position.x, pose.position.y, pose.position.z));
				new_scan = true;

			}
		}
  	}

	// save the point cloud map
	PointCloudT::Ptr scanCloud(new PointCloudT);
	*scanCloud = pclCloud;
  	scanCloud->width = scanCloud->points.size();
  	scanCloud->height = 1;

	// TODO: Downsample the map point cloud using a voxel filter

	pcl::io::savePCDFileASCII ("my_map.pcd", *scanCloud);
	cout << "saved pcd map" << endl;

	return 0;
}

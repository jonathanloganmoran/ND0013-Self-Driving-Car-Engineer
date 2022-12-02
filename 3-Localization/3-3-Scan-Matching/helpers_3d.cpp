/* ----------------------------------------------------------------------------
 * Lesson "3.3: Scan Matching Algorithms"
 * Authors     : Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: 3D Helper functions used to perform simulated
 * 						 LiDAR scans and visualise the environment. Here the
 *                       `Point`, `Pose` and `Rotate` are defined in 3D. The
 *                       rotation matrices in 3D are also defined. All objects
 *                       are intended for use with Point Cloud Library (PCL)
 *                       and used to perform the Iterative Closest Point (ICP) 
 *                       algorithm.
 * ----------------------------------------------------------------------------
 */


#include "helper.h"

Eigen::Matrix4d transform2D(double theta, double xt, double yt){

	Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity ();

	matrix(0, 3) = xt;
	matrix(1, 3) = yt;

	matrix(0, 0) = cos(theta);
	matrix(0, 1) = -sin(theta);
	matrix(1, 0) = sin(theta);
	matrix(1, 1) = cos(theta);

	return matrix;
}

Eigen::Matrix4d transform3D(double yaw, double pitch, double roll, double xt, double yt, double zt){

	Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity ();

	matrix(0, 3) = xt;
	matrix(1, 3) = yt;
	matrix(2, 3) = zt;

	matrix(0, 0) = cos(yaw) * cos(pitch);
	matrix(0, 1) = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
	matrix(0, 2) = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll);
	matrix(1, 0) = sin(yaw) * cos(pitch);
	matrix(1, 1) = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
	matrix(1, 2) = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);
	matrix(2, 0) = -sin(pitch);
	matrix(2, 1) = cos(pitch) * sin(roll);
	matrix(2, 2) = cos(pitch) * cos(roll);

	return matrix;
}

Pose getPose(Eigen::Matrix4d matrix){

	Pose pose(Point(matrix(0,3), matrix(1,3), matrix(2,3)), Rotate(atan2(matrix(1, 0),matrix(0, 0)), atan2(-matrix(2,0), sqrt(matrix(2,1)*matrix(2,1) + matrix(2,2)*matrix(2,2))), atan2(matrix(2,1),matrix(2,2))));
	return pose;
}

void print4x4Matrix (const Eigen::Matrix4d & matrix){
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color, int renderSize){
    viewer->addPointCloud<pcl::PointXYZ> (cloud, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, renderSize, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
}

void renderRay(pcl::visualization::PCLVisualizer::Ptr& viewer, Point p1, Point p2, std::string name, Color color){
	viewer->addLine(PointT(p1.x, p1.y, 0), PointT(p2.x, p2.y, 0), color.r, color.g, color.b, name);
}
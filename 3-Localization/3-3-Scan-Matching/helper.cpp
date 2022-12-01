
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

Pose getPose(Eigen::Matrix4d matrix){

	Pose pose(Point(matrix(0,3), matrix(1,3)), atan2(matrix(1, 0),matrix(0, 0)));
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

void renderPath(pcl::visualization::PCLVisualizer::Ptr& viewer, const PointCloudT::Ptr& cloud, std::string name, Color color){

	int previous = 0;
	for(int index = previous+1; index < cloud->points.size(); index++){
		renderRay(viewer, Point(cloud->points[previous].x, cloud->points[previous].y), Point(cloud->points[index].x, cloud->points[index].y), name+to_string(previous), color);
		previous++;
	}

}


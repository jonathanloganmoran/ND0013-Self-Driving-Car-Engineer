
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

double getDistance(Point p1, Point p2){
	return sqrt( (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z) );
}

double minDistance(Point p1, vector<Point> points){
	if(points.size() > 0){
		double dist = getDistance(p1, points[0]);
		for(int index = 1; index < points.size(); index++){
			double newDist = getDistance(p1, points[index]);
			if( newDist < dist)
				dist = newDist;
		}
		return dist;
	}
	return -1;
}

void print4x4Matrix (const Eigen::Matrix4d & matrix){
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void print4x4Matrixf (const Eigen::Matrix4f & matrix){
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
		renderRay(viewer, Point(cloud->points[previous].x, cloud->points[previous].y, 0), Point(cloud->points[index].x, cloud->points[index].y, 0), name+to_string(previous), color);
		previous++;
	}

}

// angle around z axis
Eigen::Quaternionf getQuaternion(float theta)
{
	Eigen::Matrix3f rotation_mat;
	rotation_mat << 
	cos(theta), -sin(theta), 0,
	sin(theta),  cos(theta), 0,
	0, 			 0, 		 1;
	
	Eigen::Quaternionf q(rotation_mat);
	return q;
}

void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, BoxQ box, int id, Color color, float opacity)
{
    if(opacity > 1.0)
    	opacity = 1.0;
    if(opacity < 0.0)
        opacity = 0.0;
    std::string cube = "box"+std::to_string(id);
    viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

    std::string cubeFill = "boxFill"+std::to_string(id);
    viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
}



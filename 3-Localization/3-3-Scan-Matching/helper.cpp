/* ----------------------------------------------------------------------------
 * Lesson "3.3: Scan Matching Algorithms"
 * Copyright (c) Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by: Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Helper functions used to perform and visualise the 
 * 						 simulated 2D environment and LiDAR scans. Also defines
 * 						 the matrices needed to store and estimate the robot
 * 						 pose using Iterative Closest Point (ICP) algorithm.
 * ----------------------------------------------------------------------------
 */


#include "helper.h"


/* Returns a 4x4 transformation matrix given the 2D pose information.
 *
 * @param 	theta   angle of rotation about the x-axis (robot heading).
 * @param 	xt	 	position along the x-axis.
 * @param 	yt	 	position along the y-axis.
 * @returns matrix  4x4 transformation matrix.
 */
Eigen::Matrix4d transform2D(
		double theta,
		double xt,
		double yt
) {
	Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity ();
	matrix(0, 3) = xt;
	matrix(1, 3) = yt;
	matrix(0, 0) = cos(theta);
	matrix(0, 1) = -sin(theta);
	matrix(1, 0) = sin(theta);
	matrix(1, 1) = cos(theta);
	return matrix;
}


/* Returns the pose vector extracted from the input transformation matrix.
 *
 * @param 	matrix	4x4 transformation matrix.
 * @returns pose	3x1 pose vector.
 */
Pose getPose(
		Eigen::Matrix4d matrix
) {
	Pose pose(
		// Here we extract the `(xt, yt, theta)` values
		// Note: a transformation (azimuth correction) is needed to obtain
		// the angle $\theta$ from its 2D components along the y- and x-axes.
		Point(matrix(0, 3), matrix(1, 3)), atan2(matrix(1, 0), matrix(0, 0))
	);
	return pose;
}


/* Prints the 4x4 transformation matrix.
 *
 * @param	matrix	the 4x4 transformation matrix to print to the console.
 */
void print4x4Matrix(
		const Eigen::Matrix4d& matrix
) {
	printf("Rotation matrix :\n");
  	printf("    | %6.3f %6.3f %6.3f | \n", 
  			matrix (0, 0), matrix (0, 1), matrix (0, 2)
  	);
  	printf("R = | %6.3f %6.3f %6.3f | \n",
			matrix (1, 0), matrix (1, 1), matrix (1, 2)
	);
  	printf("    | %6.3f %6.3f %6.3f | \n",
			matrix (2, 0), matrix (2, 1), matrix (2, 2)
	);
  	printf("Translation vector :\n");
  	printf("t = < %6.3f, %6.3f, %6.3f >\n\n",
			matrix (0, 3), matrix (1, 3), matrix (2, 3)
	);
}


 /* Renders the point cloud instance onto the PCL Viewer canvas.
  *
  * @param	viewer		PCL Viewer instance to update.
  * @param	cloud		PCL object to render onto the `viewer`.
  * @param 	name		string `id` label to assign to the rendered point cloud.
  * @param 	color		RGB-valued `Color` instance used to render the PCL.
  * @param	renderSize	point size `value` to set.
  */
void renderPointCloud(
		pcl::visualization::PCLVisualizer::Ptr& viewer,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
		std::string name,
		Color color,
		int renderSize
) {
    viewer->addPointCloud<pcl::PointXYZ> (cloud, name);
    // Input arguments: `(property, value, id)`
	viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
			renderSize,
			name
	);
	// Input arguments: `(property, val1, val2, val3, id)`
    viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR,
			color.r,
			color.g,
			color.b,
			name
	);
}


/* Renders the simulated LiDAR point return as ray instance.
 *
 * @param 	viewer	PCL Viewer instance to update.
 * @param 	p1		starting 2D `Point` of the ray to draw.
 * @param 	p2		ending 2D `Point` of the ray to draw.
 * @param 	name	string `id` label to assign to the rendered line.
 * @param 	color	RGB-valued `Color` instance used to render the line.
 */
void renderRay(
		pcl::visualization::PCLVisualizer::Ptr& viewer,
		Point p1,
		Point p2,
		std::string name,
		Color color
) {
	// Draw a line segment from the start / end points and render onto Viewer 
	viewer->addLine(
			PointT(p1.x, p1.y, 0),
			PointT(p2.x, p2.y, 0),
			color.r,
			color.g,
			color.b,
			name
	);
}


/* Renders the robot 'path' from one pose to the next.
 *
 * @param	viewer		PCL Viewer instance to update.
 * @param	cloud		PCL object to render onto the `viewer`.
 * @param 	name		string `id` label to assign to the rendered line.
 * @param 	color		RGB-valued `Color` instance used to render the line.
 */
void renderPath(
		pcl::visualization::PCLVisualizer::Ptr& viewer,
		const PointCloudT::Ptr& cloud,
		std::string name,
		Color color
) {
	int previous = 0;
	for (int index = previous + 1; index < cloud->points.size(); index++) {
		renderRay(
				viewer,
				Point(cloud->points[previous].x,
					  cloud->points[previous].y
				),
				Point(cloud->points[index].x,
					  cloud->points[index].y
				),
				name + to_string(previous),
				color
		);
		previous++;
	}
}


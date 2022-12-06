/* ----------------------------------------------------------------------------
 * Lesson "3.3: Scan Matching Algorithms"
 * Authors     : Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 * 
 * Purpose of this file: Helper functions used to perform the simulated LiDAR
 * 						 scans and visualise the 2D or 3D environment.
 * 
 * Note : The objects defined here are intended to be used with the Point
 *        Cloud Library (PCL) in order to compute the Iterative Closest Point
 *        (ICP) algorithm.
 * ----------------------------------------------------------------------------
 */


#include "helpers.h"


/* Returns a 4x4 transformation matrix given the 3D pose information.
 *
 * @param 	yaw     Angle of rotation about the z-axis.
 * @param 	pitch   Angle of rotation about the y-axis.
 * @param 	roll    Angle of rotation about the x-axis (heading).
 * @param 	xt	 	Position along the x-axis.
 * @param 	yt	 	Position along the y-axis.
 * @param 	zt	 	Position along the z-axis.
 * @returns matrix  4x4 transformation matrix.
 */
Eigen::Matrix4d transform3D(
        double yaw, 
        double pitch, 
        double roll, 
        double xt, 
        double yt, 
        double zt
){
	Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
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


/* Returns a 4x4 transformation matrix given the 2D pose information.
 *
 * @param 	theta   Angle of rotation about the x-axis (robot heading).
 * @param 	xt	 	Position along the x-axis.
 * @param 	yt	 	Position along the y-axis.
 * @returns matrix  4x4 transformation matrix.
 */
Eigen::Matrix4d transform2D(
		double theta,
		double xt,
		double yt
) {
	Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
	matrix(0, 3) = xt;
	matrix(1, 3) = yt;
	matrix(0, 0) = cos(theta);
	matrix(0, 1) = -sin(theta);
	matrix(1, 0) = sin(theta);
	matrix(1, 1) = cos(theta);
	return matrix;
}


namespace getPose2D {
/* Returns the 2D pose vector extracted from the input transformation matrix.
 *
 * Example usage:
 * ```cpp
 *     getPose2D::getPose(matrix);
 * ```
 * @param 	matrix	4x4 transformation matrix.
 * @returns pose	3x1 pose vector as a `Pose2D` instance containing
 *                  the `Point2D` `position` and `theta` orientation.
 */
Pose2D getPose(
		Eigen::Matrix4d matrix
) {
	Pose2D pose(
		// Here we extract the `(xt, yt, theta)` values and return a 2D pose
		// Note: a transformation (azimuth correction) is needed to obtain
		// the angle $\theta$ from its 2D components along the y- and x-axes.
		Point2D(matrix(0, 3), matrix(1, 3)),
        atan2(matrix(1, 0), matrix(0, 0))
	);
	return pose;
}
}  // namespace getPose2D


namespace getPose3D {
/* Returns the 3D pose extracted from the input transformation matrix.
 *
 * Example usage:
 * ```cpp
 *     getPose3D::getPose(matrix);
 * ```
 * @param 	matrix	4x4 transformation matrix.
 * @returns pose	The `Pose3D` instance containing the `Point3D`
 *                  `position` and the `Rotation` matrix.
 */
Pose3D getPose(
        Eigen::Matrix4d matrix
) {
    Pose3D pose(
        // Get the 3D position
        Point3D(
            matrix(0, 3),
            matrix(1, 3),
            matrix(2, 3)
        ),
        // Compute the rotation angles using azimuth correction
        Rotate(
            atan2(
                matrix(1, 0), 
                matrix(0, 0)),
            atan2(
                -matrix(2, 0),
                sqrt(
                    matrix(2, 1) * matrix(2, 1) + matrix(2, 2) * matrix(2, 2))
                ),
            atan2(matrix(2, 1), matrix(2, 2))
        )
    );
	return pose;
}
}  // namespace getPose3D


/* Prints the 4x4 transformation matrix.
 *
 * @param	matrix	4x4 transformation matrix to print to the console.
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
  * @param 	name		String `id` label to assign to the rendered point cloud.
  * @param 	color		RGB-valued `Color` instance used to render the PCL.
  * @param	renderSize	Point size `value` to set.
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


/* Renders the point cloud with intensity values onto the PCL Viewer canvas.
 *
 * @param  viewer       PCL Viewer instance to update.
 * @param  cloud        PCL object to render onto the `viewer`.
 * @param  name         String `id` label to assign to the rendered point cloud.
 */
void renderPointCloudI(
    pcl::visualization::PCLVisualizer::Ptr& viewer, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
    std::string name
) {
	// Create colour palette based off LiDAR intensity values
	pcl::visualization::PointCloudColorHandlerGenericField<
        pcl::PointXYZI
    > intensity_distribution(cloud, "intensity");
    // Render the point cloud
  	viewer->addPointCloud<pcl::PointXYZI>(
        cloud,
        intensity_distribution, 
        name
    );
}


namespace renderRay2D {
/* Renders the simulated LiDAR point return as ray instance.
 *
 * @param 	viewer		PCL Viewer instance to update.
 * @param 	p1			Starting `Point2D` of the ray to draw.
 * @param 	p2			Ending `Point2D` of the ray to draw.
 * @param 	name		String `id` label to assign to the rendered line.
 * @param 	color		RGB-valued `Color` instance used to render the line.
 */
void renderRay(
		pcl::visualization::PCLVisualizer::Ptr& viewer,
		Point2D p1,
		Point2D p2,
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
}  // namespace renderRay2D


namespace renderRayT {
/* Renders the simulated LiDAR point return as ray instance.
 *
 * @param 	viewer		PCL Viewer instance to update.
 * @param 	p1			Starting `PointT` of the ray to draw.
 * @param 	p2			Ending `PointT` of the ray to draw.
 * @param 	name		String `id` label to assign to the rendered line.
 * @param 	color		RGB-valued `Color` instance used to render the line.
 */
void renderRay(
		pcl::visualization::PCLVisualizer::Ptr& viewer,
		PointT p1,
		PointT p2,
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
}  // namespace renderRayT

/* Renders the robot 'path' from one 2D pose to the next.
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
		renderRay2D::renderRay(
				viewer,
				Point2D(cloud->points[previous].x,
					  cloud->points[previous].y
				),
				Point2D(cloud->points[index].x,
					  cloud->points[index].y
				),
				name + std::to_string(previous),
				color
		);
		previous++;
	}
}


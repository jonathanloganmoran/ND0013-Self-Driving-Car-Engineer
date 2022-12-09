/* ----------------------------------------------------------------------------
 * Lesson "3.4: Utilizing Scan Matching"
 * Authors     : Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 * 
 * Purpose of this file: Helper functions used to perform the simulated LiDAR
 * 						 scans and visualise the 2D or 3D environment.
 * 
 * Note : The objects defined here are intended to be used with the Point
 *        Cloud Library (PCL) in order to perform scan matching.
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


namespace getDistance3D {
/* Returns the Euclidean distance between two 3D points. 
 *
 * @param    p1		First point to compute distance between. 
 * @param    p2		Second point to compute distance between.
 * @returns  dist	Distance between the two 3D points.
 */
double getDistance(
		Point3D p1, 
		Point3D p2
) {
	return sqrt(
		(p1.x - p2.x) * (p1.x - p2.x) 
		+ (p1.y - p2.y) * (p1.y - p2.y) 
		+ (p1.z - p2.z) * (p1.z - p2.z)
	);
}
}  // namespace getDistance3D


namespace minDistance3D {
/* Returns the shortest distance between a single point and all points in list.
 *  
 * @param	 p1		3D point to compute the distance from.
 * @param	 points	Set of 3D points to compute the shortest distance to.
 * @returns  dist	Distance from `p1` to closest point in `points` list.
 */
double minDistance(
		Point3D p1, 
		std::vector<Point3D> points
) {
	if (points.size() > 0) {
		double dist = getDistance3D::getDistance(p1, points[0]);
		for (uint index = 1; index < points.size(); index++) {
			double newDist = getDistance3D::getDistance(p1, points[index]);
			if (newDist < dist) {
				dist = newDist;
			}
		}
		return dist;
	}
	return -1;
}
}  // namespace minDistance3D


namespace print4x4Matrixd {
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
}  // namespace print4x4Matrix4d


namespace print4x4Matrix4f {
/* Prints the 4x4 transformation floating-point matrix.
 *
 * @param	matrix	4x4 transformation matrix to print to the console.
 */
void print4x4Matrix(
		const Eigen::Matrix4f& matrix
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
}  // namespace print4x4Matrix4f


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
	for (unsigned int idx = previous + 1; idx < cloud->points.size(); idx++) {
		renderRay2D::renderRay(
				viewer,
				Point2D(cloud->points[previous].x,
					  cloud->points[previous].y
				),
				Point2D(cloud->points[idx].x,
					  cloud->points[idx].y
				),
				name + std::to_string(previous),
				color
		);
		previous++;
	}
}


/* Computes the quaternion floating-point representation of angle theta.
 *  
 * Quaternions describe the rotation and orientation of 3D objects in a
 * compact, efficient and stable spherical interpolation representation.
 * They have the form: $w + (x * i) + (y * j) + (z * k)$.
 *
 * @param	 theta	Rotation angle to convert to quaternion form.
 * @returns  q		Floating-point quaternion as `Eigen::Quaternionf` object.
 */
Eigen::Quaternionf getQuaternion(
		float theta
) {
	Eigen::Matrix3f rotation_mat;
	rotation_mat << 
	cos(theta), -sin(theta), 0,
	sin(theta),  cos(theta), 0,
	0, 			 0, 		 1;
	
	Eigen::Quaternionf q(rotation_mat);
	return q;
}


/* Renders the bounding box onto the PCL Viewer.
 * 
 * @param  viewer	PCL Viewer instance to render the bounding box onto.
 * @param  box		Bounding box to render.
 * @param  id		Object integer `id`.
 * @param  color	RGB-formatted `Color` object to render the bbox with.
 * @oaran  opacity  Value of opacity to render the bbox with in range [0, 1]. 
 */
void renderBox(
		pcl::visualization::PCLVisualizer::Ptr& viewer, 
		BoxQ box, 
		int id, 
		Color color, 
		float opacity
) {
    if (opacity > 1.0) {
    	opacity = 1.0;
	}
    if (opacity < 0.0) {
        opacity = 0.0;
	}
	std::string cubeLabel = "box" + std::to_string(id);
    viewer->addCube(
		box.bboxTransform, 
		box.bboxQuaternion, 
		box.cube_length, 
		box.cube_width, 
		box.cube_height, 
		cubeLabel
	);
    viewer->setShapeRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 
		pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, 
		cubeLabel
	);
    viewer->setShapeRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_COLOR, 
		color.r, 
		color.g, 
		color.b, 
		cubeLabel
	);
    viewer->setShapeRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_OPACITY, 
		opacity, 
		cubeLabel
	);
    std::string cubeFillLabel = "boxFill" + std::to_string(id);
    viewer->addCube(
		box.bboxTransform, 
		box.bboxQuaternion, 
		box.cube_length, 
		box.cube_width, 
		box.cube_height, 
		cubeFillLabel
	);
    viewer->setShapeRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 
		pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, 
		cubeFillLabel
	);
    viewer->setShapeRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_COLOR, 
		color.r, 
		color.g, 
		color.b, 
		cubeFillLabel
	);
    viewer->setShapeRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_OPACITY, 
		opacity * 0.3, 
		cubeFillLabel
	);
}



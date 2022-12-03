/* ----------------------------------------------------------------------------
 * Lesson "3.3: Scan Matching Algorithms"
 * Authors     : Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the Helper functions used to perform
 * 						 the simulated LiDAR scans and visualise the 2D or 3D
 * 						 environment.
 * 
 * Note : The objects defined here are intended to be used with the Point
 *        Cloud Library (PCL) in order to compute the Iterative Closest Point
 *        (ICP) algorithm.
 * ----------------------------------------------------------------------------
 */


#ifndef HELPERS_H_
#define HELPERS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <math.h>
#include <vector>

// Using `PointT`, `PointCloudT` to abstract type for either 2D / 3D points
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


/* Abstract base class for `Point` object.
 * @class   Point
 * @brief   Base class to represent a 2D or 3D point.
 * @func    void    Print()
 * @brief   Prints the coordinate values to the console.
 */
class Point {
    virtual void Print();
};


/* Derived `Point` class storing 2D coordinates. 
 * @class   Point2D
 * @brief   Derived class to represent a 2D point.
 * @param   x   Position along the x-axis.
 * @param   y   Position along the y-axis.     
 */
class Point2D : public Point {
    double x;
    double y;
    
    Point()
        : x(0), y(0) {}
    Point(double setX, double setY)
        : x(setX), y(setY) {}
    virtual void Print() override {
        std::cout << "x: " << x << " y: " << y << "\n";
    }
};


/* Derived `Point` class storing 3D coordinates. 
 * @class   Point3D
 * @brief   Derived class to represent a 3D point.
 * @param   x   Position along the x-axis.
 * @param   y   Position along the y-axis.
 * @param   z   Position along the z-axis.
 */
class Point3D : public Point {
    double x;
    double y;
    double z;

    Point()
        : x(0), y(0), z(0) {}
    Point(double setX, double setY, double setZ)
        : x(setX), y(setY), z(setZ) {}
    virtual void Print() override {
        std::cout << "x : " << x << " y: " << y << " z : " << z << "\n";
    }
}


/* Base class defining the `Pose` object in either 2D or 3D.
 * 
 * Example usage:
 *  ```cpp
 *      // Initialise the 2D and 3D points derived from `Point` class
 *      Point2D p2d(1.0, 2.0);
 *      Point3D p3d(1.0, 2.0, 3.0);
 *      // Initialise the 2D and 3D `Pose` objects
 *      Pose pose1(&p2d, 0.0);
 *      Pose pose2(&p3d, 0.0);
 *  ```
 * 
 * @struct  Pose    "helpers.h"
 * @brief   Stores the derived `2DPoint` or `3DPoint` class instance
 *          along with the orientation angle `theta` as type `double`.
 * @var     position    Position in either 2D or 3D.
 * @var     theta       Orientation angle in radians.
 */
class Pose {
    Point* position;
};


/* Derived `Pose` class representing position and orientation in 3D.
 *
 * @class   Pose3D      "helpers.h"
 * @brief   Stores the pose (position and orientation) in 3D.
 * @param	position	Position in 3D as a `Point3D` object.
 * @param   rotation    Rotation in 3D as a `Rotation` object.
 * @func    operator-
 * @brief   Overloads subtract operator to subtract two `Pose3D` objects.
 */
class Pose3D : public Pose {
    Rotate rotation;

	Pose(Point* setPos, Rotate setRotation)
		: position(setPos), rotation(setRotation) {}
    Pose operator-(const Pose& p) {
        Pose result(
            Point(position.x - p.position.x,
                  position.y - p.position.y, position.z - p.position.z
            ),
            Rotate(rotation.yaw - p.rotation.yaw,
                   rotation.pitch - p.rotation.pitch,
                   rotation.roll - p.rotation.roll
            )
        );
        return result;    
};


/* Derived `Pose` class representing position and orientation in 2D.
 *
 * @class   Pose2D      "helpers.h"
 * @brief   Stores the pose (position and orientation) in 2D.
 * @param	position	Position in 2D as a `Point2D` object.
 * @param   rotation    Rotation in 2D as a `double` value.
 */
class Pose2D : public Pose {
    double theta;

    Pose(Point* setPos, double setTheta)
        : position(setPos), theta(setTheta) {}
};


/* Defines the 3D rotation angles.
 * 
 * Assumes 3D reference coordinate system is defined such that
 * the positive x-axis points in direction of motion relative to
 * object heading, positive y-axis points left of the object, and
 * positive z-axis points upward.
 * 
 * This coordinate frame is given by the right-hand rule.
 * 
 * @struct  Rotate
 * @brief   3D rotation angles `yaw`, `pitch`, `roll` stored as doubles.
 * @var     yaw     Angle of rotation about the z-axis.
 * @var     pitch   Angle of rotation about the y-axis.
 * @var     roll    Angle of rotation about the x-axis.
 */
struct Rotate{
	double yaw;
    double pitch;
    double roll;

	Rotate()
		: yaw(0), pitch(0), roll(0) {}

	Rotate(double setYaw, double setPitch, double setRoll)
		: yaw(setYaw), pitch(setPitch), roll(setRoll) {}

	void Print(){
		std::cout << "yaw: " << yaw;
                  << " pitch: " << pitch;
                  << " roll: " << roll << "\n";
	}
};


/* Defines the 2D `Vec2D` object. 
 * 
 * @struct	Vec2	"helpers.h"
 * @brief	Represents 2D vector with magnitude `mag` and angle `theta`.
 * @var		mag		Magnitude of the vector.
 * @var		theta	Angle of the vector.
 */
struct Vect2 {
	double mag;
	double theta;

	Vect2(double setMag, double setTheta)
		: mag(setMag), theta(setTheta) {}
};


/* Defines the RGB `Color` object.
 * 
 * @struct 	Color
 * @brief 	Represents a RGB color with normalised values in range [0, 1].
 * @var	 	r		Value for the 'red' channel.
 * @var		g		Value for the 'green' channel.
 * @var 	b		Value for the 'blue' channel.  
 */
struct Color {
    float r;
	float g;
	float b;

    Color(float setR, float setG, float setB)
        : r(setR), g(setG), b(setB) {}
};


/*** Forward declarations ***/
// The 4x4 3D translation matrix
Eigen::Matrix4d transform3D(
        double yaw,
        double pitch,
        double roll,
        double xt,
        double yt,
        double zt
);
// The 4x4 2D transformation matrix
Eigen::Matrix4d transform2D(
		double theta,
		double xt,
		double yt
);
// Function to extract the pose vector from the 4x4 transformation matrix
Pose getPose(
		Eigen::Matrix4d matrix
);
// Function to print the 4x4 transformation matrix values
void print4x4Matrix (
		const Eigen::Matrix4d& matrix
);
// Function to render the point cloud in a PCL Viewer instance
void renderPointCloud(
		pcl::visualization::PCLVisualizer::Ptr& viewer,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
		std::string name,
		Color color,
		int renderSize=4
);
// Function to draw a line in the PCL Viewer instance
void renderRay(
		pcl::visualization::PCLVisualizer::Ptr& viewer,
		Point p1,
		Point p2,
		std::string name,
		Color color
);
// Function to render the robot 'path' from a set of positions
void renderPath(
		pcl::visualization::PCLVisualizer::Ptr& viewer,
		const PointCloudT::Ptr& cloud,
		std::string name,
		Color color
);


/* Defines the `LineSegment` object.
 * 
 * @struct	LineSegment
 * @brief  	Stores the components of a line given in slope-intercept form.
 * @var 	my		Slope along the y-axis.
 * @var 	mx		Slope along the x-axis.
 * @var		b		Intercept of the line evaluated at `y(mx=1)`, `x(mx=0)`.
 * @var		min		TODO.
 * @var 	max		TODO.
 */
struct LineSegment {
	double my;
	double mx;
	double b;
	double min;		// (x{mx=1}/y{mx=0}) interval
	double max;

	LineSegment(float setMy,
				double setMx,
				double setB,
				double setMin,
				double setMax
	)
        	: my(setMy), mx(setMx), b(setB), min(setMin), max(setMax) {
		if (setMy == 0 and setMx == 0) {
			my = 0;
			mx = 1;
		}
    }
    LineSegment()
    		: my(1), mx(1), b(1), min(-1), max(0) {}

	// `Contains` returns true in the following case:
    //			p1
    //			o
    //		p2		  p3
   	// 		o---------o
    bool Contains(double p1, double p2, double p3) {
		// Returns true if `p1` is between `p2` and `p3`
    	return (p2 <= p1) && (p1 <= p3);
    }
    bool Intersect(LineSegment line, Point& intersection) {
    	if ((my / mx) == (line.my / line.mx)) {
    		// Lines do not intersect or are parallel
    		return false;
    	}
    	if (mx == 0) {
    		intersection.x = b;
			// Form the slope-intersect form of the line
    		intersection.y = (line.my / line.mx) * intersection.x + line.b;
    		return (Contains(intersection.y, min, max)
					&& Contains(intersection.x, line.min, line.max)
			);
    	}
    	else if (line.mx == 0) {
    		intersection.x = line.b;
    		intersection.y = (my / mx) * intersection.x + b;
    		return (Contains(intersection.y, line.min, line.max)
					&& Contains(intersection.x, min, max)
			);
    	}
    	else { 
    		intersection.x = (b - line.b) / ((line.my / line.mx) - (my / mx));
    		intersection.y = (my/mx) * intersection.x + b;
    		return (Contains(intersection.x, min, max)
					&& Contains(intersection.x, line.min, line.max
			));
    	}
    }
    void Print() {
    	std::cout << "my: " << my << " mx: " << mx << " b: " << b
				  << " min: " << min << " max: " << max << "\n";
    }

};


/* Defines the simulated `Lidar` sensor and its properties.  
 *
 * @struct 	Lidar
 * @brief	Simulated Lidar sensor with pose in reference to global frame.
 * @var		x		Position of the sensor along the x-axis in global frame.
 * @var		y		Position of the sensor along the y-axis in global frame.
 * @var		theta	Orientation of the sensor (radians) in global frame.
 * @var		range	Maximum distance (metres) to limit returns from Lidar scan.
 * @var		res		Angular resolution (radians) of the simulated Lidar sensor. 
 */
struct Lidar{
	double x;
	double y;
	double theta;
	double range;
	int res;

	Lidar(double setX,
		  double setY,
		  double setTheta,
		  double setRange,
		  int setRes
	)
		: x(setX), y(setY), theta(setTheta), range(setRange), res(setRes) {}

	// Perform a scan of the environment with known walls	
	pcl::PointCloud<pcl::PointXYZ>::Ptr scan(
			std::vector<LineSegment> walls
	) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new PointCloudT);
		double deltaTheta = (2 * M_PI) / double(res);
		double residue = .1 * deltaTheta;
		// Loop stopping condition
		double maxAngle = theta + 2 * M_PI - residue;
		for (double angle = theta; angle < maxAngle; angle += deltaTheta) {
			LineSegment ray;
			// UCOMMENT TO PRINT THE ANGLE
			// std::cout << "Angle " << angle << "\n";
			// Check if the angle is vertical
			if (ceil(tan(angle - M_PI / 2) * 1000) / 1000 == 0) {
				// UNCOMMENT TO PRINT TRUTH VALUE IF TRUE
				// std::cout << "Is vertical" << "\n";
				double yb = sin(angle) * range;
				double minb = std::min(y, yb);
				double maxb = std::max(y, yb);
				ray = LineSegment(1, 0, x, minb, maxb);
			}
			else {
				double m = ceil(tan(angle) * 1000) / 1000;
				double b = y - x * m;
				double xb = cos(angle) * range;
				double minb = std::min(x, xb);
				double maxb = std::max(x, xb);
				ray = LineSegment(m, 1, b, minb, maxb);
			}
			double closetDist = range;
			//Point cPoint;
            Point2D cpoint;
			for (LineSegment wall : walls) {
				//Point point;
                Point2D point;
				if (ray.Intersect(wall, point)) {
					// UNCOMMENT TO PRINT TRUTH VALUE
					// std::cout << "Collision" << "\n";
					// ray.Print();
					// wall.Print();
					double distance = sqrt(
							(x - point.x) * (x - point.x) 
							+ (y - point.y) * (y - point.y)
					);
					// UNCOMMENT TO PRINT DISTANCE BETWEEN POINTS 
					// std::cout << "Distance " << distance << "\n";
					// point.Print();
					if (distance < closetDist) {
						closetDist = distance;
						cPoint = point;
					}
				}
			}
			if (closetDist < range) {
				// UCOMMENT TO PRINT ANGLE
				// std::cout << "Angle " << angle << "\n";
				// std::cout << "Closest distance " << closetDist << "\n";
				
				// Transform coordinates from world to `Lidar` sensor frame
				double pointX = closetDist * cos(angle - theta);
				double pointY = closetDist * sin(angle - theta);
				cloud->points.push_back(pcl::PointXYZ(pointX, pointY, 0));
			}
		}
		return cloud;
	}
	// Transform the Lidar sensor coordinates
	void Move(double step,
			  double rotate
	){
		theta += rotate;
		x += step * cos(theta);
		y += step * sin(theta);
	}
};

#endif // HELPERS_H_
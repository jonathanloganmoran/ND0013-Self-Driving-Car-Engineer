/* ----------------------------------------------------------------------------
 * Lesson "3.3: Scan Matching Algorithms"
 * Copyright (c) Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by: Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the helper functions used to perform
 * 						 and visualise the simulated 2D environment and LiDAR
 * 						 scans. Here the `Point`, `Pose`, `Color`, `Lidar`,
 * 						 `Move` structs, among others, are defined and intended
 * 						 to be used with the Point Cloud Library (PCL) and the
 * 						 Iterative Closest Points (ICP) algorithm.
 * ----------------------------------------------------------------------------
 */


#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <math.h>
#include <vector>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


const double pi = M_PI;


/* Defines the 2D `Point` object.
 *
 * @struct  Point
 * @brief   2D `Point` object storing `x` and `y` coordinates as doubles.
 * @var		x	position along the x-axis.
 * @var		y	position along the y-axis.
 */
struct Point {
	double x;
	double y;

	Point():
		x(0), y(0){}
	Point(double setX, double setY)
		: x(setX), y(setY) {}
	void Print() {
		cout << "x: " << x << " y: " << y << endl;
	}
};


/* Defines the 2D `Pose` object.
 *
 * @struct  Pose
 * @brief   Stores the 2D position as a `Point` with coordinates in `x`, `y`
 * 			along with the orientation angle `theta` as a type `double`.
 * @var		x		position along the x-axis.
 * @var		y		position along the y-axis.
 * @var 	theta	orientation angle (radians).
 */
struct Pose {
	Point position;
	double theta;

	Pose(Point setPos, double setTheta)
		: position(setPos), theta(setTheta) {}
};


/* Defines the 2D `Vec2D` object. 
 * 
 * @struct	Vec2
 * @brief	Represents 2D vector object with magnitude `mag` and angle `theta`.
 * @var		mag		magnitude of the vector.
 * @var		theta	angle of the vector.
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
 * @var	 	r	desired value for the 'red' channel.
 * @var		g	desired value for the 'green' channel.
 * @var 	b	desired value for the 'blue' channel.  
 */
struct Color {
        float r;
		float g;
		float b;

        Color(float setR, float setG, float setB)
                : r(setR), g(setG), b(setB) {}
};

/*** Forward declarations ***/
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
 * @var 	my		slope along the y-axis.
 * @var 	mx		slope along the x-axis.
 * @var		b		intercept of the line evaluated at `y(mx=1)`, `x(mx=0)`.
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
			);
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
 * @var		x		position of the sensor along the x-axis in global frame.
 * @var		y		position of the sensor along the y-axis in global frame.
 * @var		theta	orientation of the sensor (radians) in global frame.
 * @var		range	maximum distance (metres) to limit returns from Lidar scan.
 * @var		res		angular resolution (radians) of the simulated Lidar sensor. 
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
			vector<LineSegment> walls
	) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new PointCloudT);
		double deltaTheta = (2 * pi) / double(res);
		double residue = .1 * deltaTheta;
		double maxAngle = theta + 2 * pi - residue;
		for (double angle = theta; angle < maxAngle; angle += deltaTheta) {
			LineSegment ray;
			// UCOMMENT TO PRINT THE ANGLE
			// std::cout << "Angle " << angle << "\n";
			// Check if the angle is vertical
			if (ceil(tan(angle - pi / 2) * 1000) / 1000 == 0) {
				// UNCOMMENT TO PRINT TRUTH VALUE IF TRUE
				// std::cout << "Is vertical" << endl;
				double yb = sin(angle) * range;
				double minb = min(y, yb);
				double maxb = max(y, yb);
				ray = LineSegment(1, 0, x, minb, maxb);
			}
			else {
				double m = ceil(tan(angle) * 1000) / 1000;
				double b = y - x * m;
				double xb = cos(angle) * range;
				double minb = min(x, xb);
				double maxb = max(x, xb);
				ray = LineSegment(m, 1, b, minb, maxb);
			}
			double closetDist = range;
			Point cPoint;
			for (LineSegment wall : walls) {
				Point point;
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
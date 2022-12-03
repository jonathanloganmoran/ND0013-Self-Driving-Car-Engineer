/* ----------------------------------------------------------------------------
 * Lesson "3.3: Scan Matching Algorithms"
 * Authors     : Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the 3D helper functions used to
 * 						 perform simulated LiDAR scans and visualise the
 *                       environment. Here the `Point`, `Pose` and `Rotate`
 *                       structs are defined in 3D. All objects here are
 *                       intended for use with the Point Cloud Library (PCL)
 *                       library to perform the Iterative Closest Point (ICP)
 *                       algorithm.
 * ----------------------------------------------------------------------------
 */


#ifndef HELPERS_3D_H
#define HELPERS_3D_H

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <math.h>
#include <vector>

// Using `PointT`, `PointCloudT` to abstract type for 2D / 3D implementation 
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


const double pi = M_PI;


/* Defines the 3D `Point` object.
 *
 * @struct  Point   "helpers_3d.h"
 * @brief   3D `Point` object storing `x`, `y` and `z` coordinates as doubles.
 * @var		x		Position along the x-axis.
 * @var		y		Position along the y-axis.
 * @var     z       Position along the z-axis.
 */
struct Point{
	double x;
    double y;
    double z;

	Point()
		: x(0), y(0), z(0) {}
	Point(double setX, double setY, double setZ)
		: x(setX), y(setY), z(setZ) {}
	void Print() {
		std::cout << "x: " << x << " y: " << y << " z: " << z << "\n";
	}
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

/* Defines the 3D `Pose` object.
 *
 * @struct  Pose    "helpers_3d.h"
 * @brief   Stores the 3D position as a `Point` with coordinates in `x`, `y`,
 * 			and `z` along with the orientation angle `theta` as type `double`.
 * @var		position.x		Position along the x-axis.
 * @var		position.y		Position along the y-axis.
 * @var     position.z      Position along the z-axis.      
 * @var 	theta	        Orientation angle (radians).
 */
struct Pose{
	Point position;
	Rotate rotation;

	Pose()
		: position(Point(0, 0, 0)), rotation(Rotate(0, 0, 0)) {}

	Pose(Point setPos, Rotate setRotation)
		: position(setPos), rotation(setRotation) {}

	Pose operator-(const Pose& p)
    {
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
    }
};

/* Defines the `Pair` of 3D point correspondences.
 * 
 * @struct  Pair
 * @brief   Stores the set of two 3D `Point` objects forming a correspondence
 *          given by their minimised Euclidean distance to each other.
 * @var     p1.x    Position of the first `Point` along the x-axis.
 * @var     p1.y    Position of the first `Point` along the y-axis.
 * @var     p1.z    Position of the first `Point` along the z-axis.
 * @var     p2.x    Position of the second `Point` along the x-axis.
 * @var     p2.y    Position of the second `Point` along the y-axis.
 * @var     p2.z    Position of the second `Point` along the z-axis.  
 */
struct Pair{
	Point p1;
	Point p2;

	Pair(Point setP1, Point setP2)
		: p1(setP1), p2(setP2) {}
};

/* Defines the 2D `Vec2D` object. 
 * 
 * @struct	Vec2    "helpers_3d.h"
 * @brief	Represents 2D vector object with magnitude `mag` and angle `theta`.
 * @var		mag		Magnitude of the vector.
 * @var		theta	Angle of the vector.
 */
struct Vect2{

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
struct Color
{

        float r, g, b;

        Color(float setR, float setG, float setB)
                : r(setR), g(setG), b(setB)
        {}
};

Eigen::Matrix4d transform2D(double theta, double xt, double yt);
Eigen::Matrix4d transform3D(double yaw, double pitch, double roll, double xt, double yt, double zt);
Pose getPose(Eigen::Matrix4d matrix);
void print4x4Matrix (const Eigen::Matrix4d & matrix);
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color, int renderSize = 4);
void renderRay(pcl::visualization::PCLVisualizer::Ptr& viewer, Point p1, Point p2, std::string name, Color color);

struct LineSegment{
	// slope of y component
	double my;
	// slope of x component
	double mx;
	//  (y{mx=1}/x{mx=0}) intercept
	double b;
	// (x{mx=1}/y{mx=0}) interval
	double min;
	double max;

	LineSegment(float setMy, double setMx, double setB, double setMin, double setMax)
                : my(setMy), mx(setMx), b(setB), min(setMin), max(setMax){
                if (setMy == 0 and setMx == 0){
                	my = 0;
                	mx = 1;
                }
    }
    LineSegment()
    			: my(1), mx(1), b(1), min(-1), max(0){}
    //			p1
    //			o
    //		p2		  p3
   	// 		o---------o

    bool Contains(double p1, double p2, double p3){
    	return  (p2 <= p1 ) && (p1 <= p3) ;
    }

    bool Intersect(LineSegment line, Point& intersection){
    	if ( (my/mx) == (line.my/line.mx) ){
    		// lines dont intersect or are on top of each other
    		return false;
    	}   
    	if(mx == 0){
    		intersection.x = b;
    		intersection.y = (line.my/line.mx) * intersection.x + line.b;
    		return Contains(intersection.y, min, max) && Contains(intersection.x, line.min, line.max);
    	}
    	else if(line.mx == 0){
    		intersection.x = line.b;
    		intersection.y = (my/mx) * intersection.x + b;
    		return Contains(intersection.y, line.min, line.max) && Contains(intersection.x, min, max);
    	}
    	// not dealing with any vertical or horizontal lines and lines are not the same slope
    	else{

    		intersection.x = (b - line.b)/( (line.my/line.mx) - (my/mx) ) ;
    		intersection.y = (my/mx) * intersection.x + b;
    		return Contains(intersection.x, min, max) && Contains(intersection.x, line.min, line.max);
    	}
    }

    void Print(){
    	cout << "my: " << my << " mx: " << mx << " b: " << b << " min: " << min << " max: " << max << endl;
    }

};

struct Lidar{

	double x;
	double y;
	double theta;
	double range;
	int res;

	Lidar(double setX, double setY, double setTheta, double setRange, int setRes)
		: x(setX), y(setY), theta(setTheta), range(setRange), res(setRes){}

	pcl::PointCloud<pcl::PointXYZ>::Ptr scan(vector<LineSegment> walls){

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new PointCloudT); 
		double deltaTheta = (2*pi)/double(res);
		double residue = .1*deltaTheta;
		for(double angle = theta; angle < theta + 2*pi - residue ; angle += deltaTheta ){

			LineSegment ray;

			//cout << "angle " << angle << endl;

			// check if angle is vertical
			if( ceil(tan(angle-pi/2)*1000)/1000 == 0){

				//cout << "vertical" << endl;

				double yb = sin(angle) * range;
				double minb = min(y,yb);
				double maxb = max(y,yb);

				ray = LineSegment(1, 0, x, minb, maxb);

			}
			else{

				double m = ceil(tan(angle)*1000)/1000;
				double b = y - x*m;

				double xb = cos(angle) * range;
				double minb = min(x,xb);
				double maxb = max(x,xb);

				ray = LineSegment(m, 1, b, minb, maxb);
			}

			double closetDist = range;
			Point cPoint;
			for(LineSegment wall : walls){
				Point point;
				if( ray.Intersect(wall, point) ){
					//cout << "collision" << endl;
					//ray.Print();
					//wall.Print();
					double distance = sqrt( (x - point.x)*(x - point.x) + (y - point.y)*(y - point.y) );
					//cout << "dis " << distance << endl;
					//point.Print();
					if( distance < closetDist){
						closetDist = distance; 
						cPoint = point;
					}
					
				}
			}
			if( closetDist < range ){
				// transform to lidars local coordinates
				//cout << "angle " << angle << endl;
				//cout << closetDist << endl;
				double pointX = closetDist * cos(angle-theta);
				double pointY = closetDist * sin(angle-theta);
				cloud->points.push_back(pcl::PointXYZ(pointX, pointY, 0));
			}
		}
		return cloud;
	}

	void Move(double step, double rotate){
		theta += rotate;
		x += step * cos(theta);
		y += step * sin(theta);
	}
};

#endif // HELPERS_3D_
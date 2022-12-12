/* ----------------------------------------------------------------------------
 * Project "3.1: Scan Matching Localization"
 * Authors     : Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the helper functions used to perform
 * 						 the simulated LiDAR scans, visualise the 2D or 3D 
 * 						 environment, and represent the simulated vehicle state
 * 						 in the CARLA Simulator.
 *
 * Note : The functions/classes/structs defined here are intended to be used
 *         with the Point Cloud Library (PCL) to perform scan matching.
 * ----------------------------------------------------------------------------
 */


#ifndef HELPERS_H_
#define HELPERS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <math.h>
#include <vector>
#include <Eigen/Geometry>

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
public:
    virtual ~Point() {}
    virtual void Print() {}
};


/* Derived `Point` class storing 2D coordinates.
 * @class   Point2D
 * @brief   Derived class to represent a 2D point.
 * @param   x   Position along the x-axis.
 * @param   y   Position along the y-axis.
 */
class Point2D : public Point {
public:
    double x;
    double y;

    ~Point2D() override {}
    Point2D()
        : x(0), y(0) {}
    Point2D(double setX, double setY)
        : x(setX), y(setY) {}
    void Print() override {
        std::cout << "x: " << x;
        std::cout << " y: " << y << "\n";
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
public:
    double x;
    double y;
    double z;

    ~Point3D() override {}
    Point3D()
        : x(0), y(0), z(0) {}
    Point3D(double setX, double setY, double setZ)
        : x(setX), y(setY), z(setZ) {}
    void Print() override {
        std::cout << "x : " << x;
        std::cout << " y: " << y;
        std::cout << " z : " << z << "\n";
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
struct Rotate {
    double yaw;
    double pitch;
    double roll;

    Rotate()
        : yaw(0), pitch(0), roll(0) {}

    Rotate(double setYaw, double setPitch, double setRoll)
        : yaw(setYaw), pitch(setPitch), roll(setRoll) {}

    void Print() {
        std::cout << "yaw: " << yaw;
        std::cout << " pitch: " << pitch;
        std::cout << " roll: " << roll << "\n";
    }
};


/* Base class defining the `Pose` object in either 2D or 3D.
 *
 * @class   Pose    "helpers.h"
 * @brief   Defines the abstract base class for the 2D or 3D pose instance.
 * @func    void    `Pose::Print()`
 * @brief   Prints the coordinates and rotation angle(s) to the console.
 */
class Pose {
public:
    virtual ~Pose() {}
    virtual void Print() {}
};


/* Derived `Pose` class representing position and orientation in 2D.
 *
 * Example usage:
 *  ```cpp
 *      // Initialise the 2D point derived from `Point` class
 *      Point2D p2d(1.0, 2.0);
 *      // Initialise the 2D `Pose` object
 *      Pose2D pose(p2d, 0.0);
 *  ```
 * @class   Pose2D      "helpers.h"
 * @brief   Stores the pose (position and orientation) in 2D.
 * @param	position	Position in 2D as a `Point2D` object.
 * @param   rotation    Rotation in 2D as a `double` value.
 */
class Pose2D : public Pose {
public:
    Point2D position;
    double theta;

    ~Pose2D() override {}
    Pose2D()
        : position(Point2D(0, 0)), theta(0.0) {}
    Pose2D(Point2D setPos, double setTheta)
        : position(setPos), theta(setTheta) {}
    Pose2D operator-(const Pose2D& p) {
        Pose2D result(
            Point2D(position.x - p.position.x,
                    position.y - p.position.y
            ),
            theta - p.theta
        );
        return result;
    }
    void Print() override {
        position.Print();
        std::cout << " theta: " << theta << "\n";
    }
};


/* Derived `Pose` class representing position and orientation in 3D.
 *
 * Example usage:
 *  ```cpp
 *      // Initialise the 3D point derived from `Point` class
 *      Point3D p3d(1.0, 2.0, 3.0);
 *      // Initialise the 3D `Pose` object
 *      Pose3D pose(p3d, Rotate(0.0, 1.0, 2.0));
 *  ```
 * @class   Pose3D      "helpers.h"
 * @brief   Stores the pose (position and orientation) in 3D.
 * @param	position	Position in 3D as a `Point3D` object.
 * @param   rotation    Rotation in 3D as a `Rotation` object.
 * @func    Pose3D      operator-
 * @brief   Overloads subtract operator to subtract two `Pose3D` objects.
 */
class Pose3D : public Pose {
public:
    Point3D position;
    Rotate rotation;

    ~Pose3D() override {}
    Pose3D()
        : position(Point3D(0, 0, 0)), rotation(Rotate(0, 0, 0)) {}
    Pose3D(Point3D setPos, Rotate setRotation)
        : position(setPos), rotation(setRotation) {}
    Pose3D operator-(const Pose3D& p) {
        Pose3D result(
            Point3D(position.x - p.position.x,
                    position.y - p.position.y,
                    position.z - p.position.z
            ),
            Rotate(rotation.yaw - p.rotation.yaw,
                   rotation.pitch - p.rotation.pitch,
                   rotation.roll - p.rotation.roll
            )
        );
        return result;
    }
    void Print() override {
        position.Print();
        rotation.Print();
    }
};


/* Defines the control state of the vehicle.
 *
 * @struct  ControlState 	"helpers.h"
 * @brief	Represents the vehicle control state.
 * @var		t	Throttle value of the vehicle.
 * @var		s	Steer angle of the vehicle.
 * @var		b	Brake state of the vehicle.
 */
struct ControlState {
    float t;
    float s;
    float b;

    ControlState(float setT, float setS, float setB)
        : t(setT), s(setS), b(setB) {}
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


/* Defines the bounding box object.
 *
 * @struct  BoxQ			"helpers.h"
 * @brief	Represents the 3D bounding box.
 * @var		bboxTransform 	Pose in (x, y, z).
 * @var		bboxQuaternion	Rotation quaternion values in (w, x, y, z).
 * @var		cube_length		Length of the bounding box.
 * @var		cube_width		Width of the bounding box.
 * @var		cube_height		Height of the bounding box.
 */
struct BoxQ
{
    Eigen::Vector3f bboxTransform;
    Eigen::Quaternionf bboxQuaternion;
    float cube_length;
    float cube_width;
    float cube_height;
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
namespace getPose3D {
// Function to extract the 3D pose from the 4x4 transformation matrix
Pose3D getPose(
        Eigen::Matrix4d matrix
);
}  // namespace getPose3D
namespace getPose2D {
// Function to extract the 2D pose vector from the 4x4 transformation matrix
Pose2D getPose(
        Eigen::Matrix4d matrix
);
}  // namespace getPose2D
// Function to render the point cloud in a PCL Viewer instance
void renderPointCloud(
        pcl::visualization::PCLVisualizer::Ptr& viewer,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        std::string name,
        Color color,
        int renderSize=4
);
// Function to render the point cloud with probabilities in PCL Viewer instance
void renderPointCloudI(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    std::string name
);
// Function to draw a line in the PCL Viewer instance
namespace renderRay2D {
void renderRay(
        pcl::visualization::PCLVisualizer::Ptr& viewer,
        Point2D p1,
        Point2D p2,
        std::string name,
        Color color
);
}  // namespace renderRay2D
namespace renderRayT {
void renderRay(
        pcl::visualization::PCLVisualizer::Ptr& viewer,
        PointT p1,
        PointT p2,
        std::string name,
        Color color
);
}  // namespace RenderRayT
// Function to render the robot 'path' from a set of positions
void renderPath(
        pcl::visualization::PCLVisualizer::Ptr& viewer,
        const PointCloudT::Ptr& cloud,
        std::string name,
        Color color
);
namespace getDistance3D {
// Function to return the distance between two 3D points
double getDistance(
        Point3D p1,
        Point3D p2
);
}  // namespace getDistance3D
namespace minDistance3D {
// Function to compute the minimum distance between one and a set of 3D points
double minDistance(
        Point3D p1,
        std::vector<Point3D> points
);
}  // namespace minDistance3D
namespace print4x4Matrix4d {
// Function to print the 4x4 transformation matrix decimal values
void print4x4Matrix (
        const Eigen::Matrix4d& matrix
);
}  // namespace print4x4Matrix4d
namespace print4x4Matrix4f {
// Function to print the 4x4 transformation matrix floating-point values
void print4x4Matrixf(
        const Eigen::Matrix4f & matrix
);
}  // namespace print4x4Matrix4f
Eigen::Quaternionf getQuaternion(float theta);
// Function to render a given bounding box onto the PCL Viewer
void renderBox(
        pcl::visualization::PCLVisualizer::Ptr& viewer,
        BoxQ box,
        int id,
        Color color,
        float opacity
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
            // Handle divide-by-zero
            my = 0;
            mx = 1;
        }
        if (setMy == 1 and setMx == 0) {
            // Handle possible divide-by-zero
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
    bool Intersect(LineSegment line, Point2D& intersection) {
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
        std::cout << "my: " << my << " mx: " << mx << " b: " << b;
        std::cout << " min: " << min << " max: " << max << "\n";
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
        // Resolution of each step
        double deltaTheta = (2 * M_PI) / double(res);
        // Residue prevents angle from being overlapped at 2*pi
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
            Point2D cPoint;
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

#endif  // HELPERS_H_
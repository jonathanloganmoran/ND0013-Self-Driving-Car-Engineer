#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <math.h>
#include <vector>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

const double pi = M_PI;

struct Point{
	double x, y;

	Point()
		: x(0), y(0){}

	Point(double setX, double setY)
		: x(setX), y(setY){}

	void Print(){
		cout << "x: " << x << " y: " << y << endl;
	}
};

struct Pose{

	Point position;
	double theta;

	Pose(Point setPos, double setTheta)
		: position(setPos), theta(setTheta) {}

};

struct Vect2{

	double mag;
	double theta;

	Vect2(double setMag, double setTheta)
		: mag(setMag), theta(setTheta) {}
};

struct Color
{

        float r, g, b;

        Color(float setR, float setG, float setB)
                : r(setR), g(setG), b(setB)
        {}
};

Eigen::Matrix4d transform2D(double theta, double xt, double yt);
Pose getPose(Eigen::Matrix4d matrix);
void print4x4Matrix (const Eigen::Matrix4d & matrix);
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color, int renderSize = 4);
void renderRay(pcl::visualization::PCLVisualizer::Ptr& viewer, Point p1, Point p2, std::string name, Color color);
void renderPath(pcl::visualization::PCLVisualizer::Ptr& viewer, const PointCloudT::Ptr& cloud, std::string name, Color color);

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
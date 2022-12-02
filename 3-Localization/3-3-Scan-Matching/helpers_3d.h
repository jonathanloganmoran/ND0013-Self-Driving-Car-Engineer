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


struct Point{
	double x, y, z;

	Point()
		: x(0), y(0), z(0){}

	Point(double setX, double setY, double setZ)
		: x(setX), y(setY), z(setZ){}

	void Print(){
		cout << "x: " << x << " y: " << y << " z: " << z << endl;
	}
};
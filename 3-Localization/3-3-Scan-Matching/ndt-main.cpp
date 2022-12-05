/* ----------------------------------------------------------------------------
 * Lesson "3.3: Scan Matching Algorithms"
 * Authors     : Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Main entry into the scan matching programme. Here the
 *                       Normal Distributions Transform (NDT) algorithm performs
 *                       the optimal association (registration) of the two point
 * 						 clouds. The NDT uses Newton's method to iteratively
 * 						 compute the roots of the differentiable piecewise loss
 * 						 function. The two point clouds are discretised s.t.
 * 						 each cell is defined by a mean and covariance. The
 * 						 probability density function evaluated at a location
 * 						 $x$ within the cell is given as a normal distribution.
 * 						 The Euclidean transformation matrix is formed by
 * 						 optimising the parameters of the transform which maps
 * 						 the `target` point cloud to the `first` using the
 * 						 gradient-based Newton's method.
 * 
 * Note : The objects defined here are intended to be used with the Point
 *        Cloud Library (PCL) and Eigen libraries in order to compute the
 *        Iterative Closest Point (ICP) algorithm.
 * ----------------------------------------------------------------------------
 */


#include "helper.h"
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   					// TicToc time-tracking

// Using `Derived` type to handle matrices
template<typename Derived>

/*** Define the starting variables as global variables ***/
Pose3D pose(Point3D(0, 0, 0), Rotate(0, 0, 0));
Pose3D upose = pose;
bool matching = false;
bool update = false;


/* Event handler that updates the PCL Viewer state and point cloud pose. 
 *
 * @param   event   `KeyboardEvent` containing the pressed key.
 * @param   viewer  PCL Viewer instance from which the event was created.
 */
void keyboardEventOccurred(
        const pcl::visualization::KeyboardEvent &event,
        void* viewer
) {
  	// boost::shared_ptr<
    //     pcl::visualization::PCLVisualizer
    // > viewer = *static_cast<boost::shared_ptr<
    //         pcl::visualization::PCLVisualizer
    //     >*>(viewer_void
    // );
	if (event.getKeySym() == "Right" && event.keyDown()){
		update = true;
		upose.position.x += 0.1;
  	}
	else if (event.getKeySym() == "Left" && event.keyDown()){
		update = true;
		upose.position.x -= 0.1;
  	}
  	if (event.getKeySym() == "Up" && event.keyDown()){
		update = true;
		upose.position.y += 0.1;
  	}
	else if (event.getKeySym() == "Down" && event.keyDown()){
		update = true;
		upose.position.y -= 0.1;
  	}
	else if (event.getKeySym() == "k" && event.keyDown()){
		update = true;
		upose.rotation.yaw += 0.1;
		while( upose.rotation.yaw > 2 * M_PI)
			upose.rotation.yaw -= 2 * M_PI;  
  	}
	else if (event.getKeySym() == "l" && event.keyDown()){
		update = true;
		upose.rotation.yaw -= 0.1;
		while( upose.rotation.yaw < 0)
			upose.rotation.yaw += 2 * M_PI; 
  	}
	else if (event.getKeySym() == "space" && event.keyDown()){
		matching = true;
		update = false;
  	}
	else if (event.getKeySym() == "n" && event.keyDown()){
		pose = upose;
		std::cout << "Set New Pose" << "\n";
  	}
	else if (event.getKeySym() == "b" && event.keyDown()){
		std::cout << "Do ICP With Best Associations" << "\n";
		matching = true;
		update = true;
  	}
}


/* Returns the probability of the given point w.r.t. its mean and stdev.
 *
 * @param    X		TODO.
 * @param    Q		TODO.
 * @param    S		TODO.
 * @returns  
 */
double Probability(Eigen::MatrixXd X, Eigen::MatrixXd Q, Eigen::MatrixXd S){
	// TODO: calculate the probibility of the point given mean and standard deviation
	return 0;
}


/* TODO. 
 *
 * @struct  Cell	"ndt-main.cpp"
 * @var		cloud	Point cloud to discretise.
 * @var		Q		TODO.
 * @var		S		TODO.
 */
struct Cell{
	PointCloudT::Ptr cloud;
	Eigen::MatrixXd Q;
	Eigen::MatrixXd S;

	Cell(){
		PointCloudT::Ptr input(new PointCloudT);
		cloud = input;
		Q = Eigen::MatrixXd::Zero(2,1);
		S = Eigen::MatrixXd::Zero(2,2);
	}
};


/* TODO.
 *
 * @struct   Grid		Represents a discretised region of the point cloud.
 * @var      res		Resolution of the grid cell.
 * @var	     width		The width of the discretised area in the grid.
 * @var      height		The height of the discretised area in the grid.
 * @var      grid		Vector of `Cell` instances in the grid.
 * @func     void		`Grid::addPoint(PointT point)`
 * @brief    Adds given `point` to the `grid` if within the boundary limits.
 * @func     void		`Grid::Build()`
 * @brief    Initialises the grid instance for NDT with a mean and variance. 
 * @func     Cell 		`Grid::getCell(PointT point)`
 * @brief    Returns the `Cell` of region mapped to by `point` in grid frame.
 * @func     double		`Grid::Value(PointT point)
 * @brief    Returns the probability value of the given `point` in grid frame.
 */
struct Grid{
	// each cell is a square res x res and total grid size is (2 x width) x (2 x height)
	double res;
	int width;
	int height;
	vector<vector<Cell>> grid;
	// Create a new `grid` instance from given dimensions
	Grid(double setRes,
		 int setWidth,
		 int setHeight
	) {
		res = setRes;
		width = setWidth;
		height = setHeight;
		for (int r = 0; r < height * 2; r++) {
			vector<Cell> row;
			for (int c = 0; c < width * 2; c++) {
				row.push_back(Cell());
			}
			grid.push_back(row);
		}
	}
	// Adds a given `point` to the `grid` if within the grid boundary limits
	void addPoint(PointT point) {
		//cout << point.x << "," << point.y << endl;
		int c = int((point.x + width * res  ) / res);
		int r = int((point.y + height * res ) / res);
		//cout << r << "," << c << endl;
		if ((c >= 0 && c < width * 2)
			 && (r >= 0 && r < height * 2)
		) {
			grid[r][c].cloud->points.push_back(point);
		}
	}
	// Initialise the `grid` mean and variance
	void Build() {
		for (int r = 0; r < height * 2; r++) {
			for (int c = 0; c < width * 2; c++) {
				PointCloudT::Ptr input = grid[r][c].cloud;
				if (input->points.size() > 2) {
					// Calculate the mean
					Eigen::MatrixXd Q(2, 1);
					Q << Eigen::MatrixXd::Zero(2, 1);
					for(PointT point : input->points){
						Q(0, 0) += point.x;
						Q(1, 0) += point.y;
					}
					Q(0, 0) = Q(0, 0) / input->points.size();
					Q(1, 0) = Q(1,0) / input->points.size();
					grid[r][c].Q = Q;
					// Calculate sigma
					Eigen::MatrixXd S(2, 2);
					S << Eigen::MatrixXd::Zero(2, 2);
					for (PointT point : input->points) {
						Eigen::MatrixXd X(2, 1);
						X(0, 0) = point.x;
						X(1, 0) = point.y;
						S += (X - Q) * (X - Q).transpose();
					}
					S(0, 0) = S(0, 0) / input->points.size();
					S(0, 1) = S(0, 1) / input->points.size();
					S(1, 0) = S(1, 0) / input->points.size();
					S(1, 1) = S(1, 1) / input->points.size();
					grid[r][c].S = S;
				}
			}
		}
	}
	// Returns the `Cell` given by `point` in grid frame
	Cell getCell(PointT point) {
		int c = int((point.x + width * res) / res);
		int r = int((point.y + height * res) / res);
		if ((c >= 0 && c < width * 2)
			 && (r >= 0 && r < height * 2)
		) {
			return grid[r][c];
		}
		return Cell();
	}
	// Returns the summed probability value in `Cell` given by `point` in grid
	double Value(PointT point) {
		Eigen::MatrixXd X(2,1);
		X(0,0) = point.x;
		X(1,0) = point.y;
		double value = 0.0;
		for (int r = 0; r < height * 2; r++) {
			for (int c = 0; c < width * 2; c++) {
				if (grid[r][c].cloud->points.size() > 2) {
					value += Probability(X,
										 grid[r][c].Q,
										 grid[r][c].S
					);
				}
			}
		}
		return value;
	}
};


/* TODO.
 *
 * @param	 input
 * @param	 res
 * @param  	 viewer
 * @returns  cell
 */
Cell PDF(
		PointCloudT::Ptr input, 
		int res, 
		pcl::visualization::PCLVisualizer::Ptr& viewer
) {
	// TODO: Calculate the 2 x 1 matrix Q, which is the mean of the input points
	Eigen::MatrixXd Q(2, 1);
	Q << Eigen::MatrixXd::Zero(2, 1);
	// TODO: Calculate the 2 x 2 matrix S, which is standard deviation of the input points
	Eigen::MatrixXd S(2, 2);
	S << Eigen::MatrixXd::Zero(2, 2);
	PointCloudTI::Ptr pdf(new PointCloudTI);
	for (double i = 0.0; i <= 10.0; i += 10.0 / double(res)) {
		for (double j = 0.0; j <= 10.0; j += 10.0 / double(res)) {
			Eigen::MatrixXd X(2, 1);
			X(0, 0) = i;
			X(1, 0) = j;
			PointTI point;
			point.x = i;
			point.y = j;
			point.z = Probability(X, Q, S);
			point.intensity = point.z;
			pdf->points.push_back(point);
		}
	}
	renderPointCloudI(
		viewer, 
		pdf, 
		"pdf"
	);
	Cell cell = Cell();
	cell.S = S;
	cell.Q = Q;
	cell.cloud = input;
	return cell;
}


/* TODO.
 *
 * @param  point
 * @param  theta
 * @param  cell
 * @param  g_previous
 * @param  H_previous
 */
void NewtonsMethod(
		PointT point, 
		double theta, 
		Cell cell, 
		Eigen::MatrixBase<Derived>& g_previous, 
		Eigen:: MatrixBase<Derived>& H_previous
) {
	// TODO: Get the Q and S matrices from cell, invert S matrix
	// TODO: make a 2 x 1 matrix from input point
	// TODO: calculate matrix q from X and Q
	// TODO: calculate the 3 2 x 1 partial derivative matrices
	// each with respect to x, y, and theta
	// TODO: calcualte the 1 x 1 exponential matrix which uses q, and S inverse
	// TODO: calculate the matrix g which uses q, exponential, S inverse, and partial derivatives
	Eigen::MatrixXd g(3,1);
	g << Eigen::MatrixXd::Zero(3,1);
    // TODO: calculate the 2 x 1 second order partial derivative matrix
	// TODO: calculate the matrix H which uses q, exponential, S inverse, partial derivatives, and second order partial derivative
	Eigen::MatrixXd H(3,3);
	H << Eigen::MatrixXd::Zero(3,3);
	H_previous += H;
	g_previous += g;

}


/* TODO.
 *
 * @param	 cloud
 * @param	 grid
 * @returns  score
 */
double Score(
		PointCloudT::Ptr cloud, 
		Grid grid
) {
	double score = 0;
	for(PointT point:cloud->points){
		Cell cell = grid.getCell(point);
		if(cell.cloud->points.size() > 2){
			score += grid.Value(point);
		}
	}
	return score;
}


/* TODO. 
 *
 * @param 	 alpha 
 * @param 	 T 
 * @param 	 source 
 * @param 	 pose 
 * @param 	 grid 
 * @returns  score
 */
double AdjustmentScore(
		double alpha, 
		Eigen::MatrixXd T, 
		PointCloudT::Ptr source, 
		Pose pose, 
		Grid grid
) {
	T *= alpha;
	pose.position.x += T(0, 0);
	pose.position.y += T(1, 0);
	pose.rotation.yaw += T(2, 0);
	while(pose.rotation.yaw > 2 * M_PI) {
		pose.rotation.yaw -= 2*M_PI;
	}
	Eigen::Matrix4d transform = transform3D(
		pose.rotation.yaw, 
		0, 
		0, 
		pose.position.x, 
		pose.position.y, 
		0
	);
	PointCloudT::Ptr transformedScan(new PointCloudT);
	pcl::transformPointCloud(
		*source, 
		*transformedScan, 
		transform
	);
	double score = Score(transformedScan, grid);
	// UNCOMMENT TO PRINT ADJUSTMENT SCORE
	// std::cout << "Score would be " << score << "\n";
	return score;
}


/* Computes the step size $\alpha$ for the transform update. 
 * 
 * @param   T 		
 * @param   source 
 * @param   pose 
 * @param   grid 
 * @param   currScore 
 * @returns 
 */ 
double computeStepLength(
		Eigen::MatrixXd T, 
		PointCloudT::Ptr source, 
		Pose pose, 
		Grid grid, 
		double currScore
) {
	double maxParam = max(max(T(0, 0), T(1, 0)), T(2, 0));
	double mlength = 1.0;
	if (maxParam > 0.2) {
		mlength =  0.1 / maxParam;
		T *= mlength;
	}
	double bestAlpha = 0;
	// CANDO: Try smaller steps
	double alpha = 1.0;
	for (int i = 0; i < 40; i++) {
		// std::cout << "Adjusting alpha smaller" << "\n";
		double adjScore = AdjustmentScore(
			alpha, 
			T, 
			source, 
			pose, 
			grid
		);
		if (adjScore > currScore) {
			bestAlpha = alpha;
			currScore = adjScore;
		}
		alpha *= .7;
	}
	if (bestAlpha == 0) {
		// CANDO: Try larger steps
		alpha = 2.0;
		for (int i = 0; i < 10; i++) {
			// std::cout << "Adjusting alpha bigger" << "\n";
			double adjScore = AdjustmentScore(
				alpha, 
				T, 
				source, 
				pose, 
				grid
			);
			if (adjScore > currScore) {
				bestAlpha = alpha;
				currScore = adjScore;
			}
			alpha *= 2;
		}
	}
	return bestAlpha * mlength;
}


/* Computes the positive definite form of the Hessian matrix.
 *
 * @param	 start		Initial value of the transform.
 * @param	 increment  Step size w.r.t. the change in $\Delta p$.
 * @param	 maxIt		TODO.
 * @returns  T			Transform updated w.r.t. the current iteration.
 */
double PosDef(
		Eigen::MatrixBase<Derived>& A, 
		double start, 
		double increment, 
		int maxIt
) {
	bool pass = false;
	int count = 0;
	A += start * Eigen::Matrix3d::Identity();
	while (!pass && count < maxIt) {
		// Compute the Cholesky decomposition of A
		Eigen::LLT<Eigen::MatrixXd> lltOfA(A);
    	if (lltOfA.info() == Eigen::NumericalIssue) {
			A += increment * Eigen::Matrix3d::Identity();
			count++;
    	}
		else {
			pass = true;
		}
	}
	return  start + increment * count;
}


/* Runs the NDT scan matching algorithm.
 *
 * 
 *
 */
int main() {
	// Set up the PCL Viewer instance
	pcl::visualization::PCLVisualizer::Ptr viewer(
		new pcl::visualization::PCLVisualizer("NDT Creation")
	);
  	viewer->setBackgroundColor(0, 0, 0);
  	viewer->addCoordinateSystem(1.0);
	viewer->registerKeyboardCallback(
		keyboardEventOccurred, (void*)&viewer
	);
	/*** Perform the NDT algorithm ***/
	// Part 1: Visualise the `Cell` computed from a PDF
	bool runPart1 = true;
	if (runPart1) {
		// Create `input` from set of 2D Cartesian coordinates (x, y)
		// Each point is in the range [0, 10]
		PointCloudT::Ptr input(new PointCloudT);
  		input->points.push_back(PointT(4, 4, 0));
		input->points.push_back(PointT(4, 9, 0));
		input->points.push_back(PointT(5, 5, 0));
		input->points.push_back(PointT(5, 2, 0));
		input->points.push_back(PointT(7, 5, 0));
		// Render the cell border
		renderRay(viewer, PointT(0,0,0), PointT(0,10,0), "left", Color(0,0,1));
		renderRay(viewer, PointT(0,10,0), PointT(10,10,0), "top", Color(0,0,1));
		renderRay(viewer, PointT(10,10,0), PointT(10,0,0), "right", Color(0,0,1));
		renderRay(viewer, PointT(0,0,0), PointT(10,0,0), "bottom", Color(0,0,1));
		// TODO: Finish writing the PDF function to visualise the 2D Guassian
		Cell cell = PDF(input, 200, viewer);
		// CANDO: Change the test `point` and observe the effect on convergece
		PointT point(1,2,1);
		input->points.push_back(
			PointT(point.x, point.y, 1.0)
		);
		// TODO: Increase the iteration count to get convergence
		int maxIter = 1;
		for (int iteration = 0; iteration < maxIter; iteration++) { 
			Eigen::MatrixXd g(3, 1);
			g << Eigen::MatrixXd::Zero(3, 1);
			Eigen::MatrixXd H(3, 3);
			H << Eigen::MatrixXd::Zero(3, 3);
			// TODO: Finish writing the `NewtonsMethod` function
			double theta = 0.0;
			NewtonsMethod(
				point,
				theta, 
				cell, 
				g, 
				H
			);
			 // TODO: Change the increment and max values to nonzero values
			double startValue = 0;
			double incrementValue = 0.0;
			int maxIterations = 0;
			PosDef(
				H,
				startValue, 
				incrementValue, 
				maxIterations
			);
			// TODO: Calculate the 3x1 matrix `T` by using inverse Hessian `H` and `g`
			// TODO: Calculate new point by transforming by the `T` matrix
			// `T` has the form: [x translation, y translation, theta rotation]
			// Values should be non-zero
			// pointT(new x, new y, 1)
			PointT pointT(0, 0, 1); 
			double magT = sqrt(pointT.x * pointT.x + pointT.y * pointT.y);
			// CANDO: Change the step size value `maxDelta`
			double maxDelta = 0.5;
			pointT.x *= maxDelta / magT;
			pointT.y *= maxDelta / magT;
			PointT point2(
				point.x + pointT.x,
				point.y + pointT.y,
				1
			);
			renderRay(
				viewer, 
				point, 
				point2, 
				"gradient_" + std::to_string(iteration), 
				Color(1, 1, 1)
			);
			input->points.push_back(
				PointT(point2.x, point2.y, 1.0)
			);
			point = point2;
		}
		renderPointCloud(
			viewer, 
			input, 
			"input", 
			Color(0,0,1)
		);
		while (!viewer->wasStopped()) {
  			viewer->spinOnce();
  		}
	}
	// Part 2: Create Multiple `Cell` instances with PDFs for `target`
	else {
		// Load `target` point cloud
		PointCloudT::Ptr target(new PointCloudT);
  		pcl::io::loadPCDFile("target.pcd", *target);
		// Render the `target` point cloud onto the PCL Viewer
		renderPointCloud(
			viewer, 
			target, 
			"target", 
			Color(0, 0, 1)
		);
		// Create a new `ndtGrid` instance
		// CANDO: Change dimensions of `grid`
		float gridResolution = 3.0;
		int gridWidth = 2;
		int gridHeight = 2;
		Grid ndtGrid(
			gridResolution,
			gridWidth,
			gridHeight
		);
		for (PointT point : target->points) {
			ndtGrid.addPoint(point);
		}
		// Initialise the grid with probability values
		ndtGrid.Build();
		// Draw the `grid` onto the PCL Viewer
		int rowc = 0;
		float gridR = ndtGrid.res;
		float gridH = ndtGrid.height;
		float gridW = ndtGrid.width;
		// TODO: Convert loop iterator into `int` units
		for (double y = -gridH * gridR; y <= gridH * gridR; y += gridR) {
			renderRay(
				viewer, 
				PointT(-gridW * gridR, y, 0), 
				PointT(ndtGrid.width * ndtGrid.res,y,0),
				"grid_row_" + std::to_string(rowc), 
				Color(0, 0, 1)
			);
			rowc++;
		}
		int colc = 0;
		// TODO: Convert loop iterator into `int` units
		for (double x = -gridW * gridR; x <= gridW * gridR; x += gridR) {
			renderRay(
				viewer, 
				PointT(x, -gridH * gridR, 0), 
				PointT(x, gridH * gridR, 0), 
				"grid_col_" + std::to_string(colc), 
				Color(0, 0, 1)
			);
			colc++;
		}
		// Draw the total probability distribution computed over all cells
		PointCloudTI::Ptr pdf(new PointCloudTI);
		int res = 10;
		// TODO: Convert loop iterator into `int` units
		for (double y = -gridH * gridRes; y <= gridH * gridR; y += gridR / double(res)) {
			for (double x = -gridW * gridR; x <= gridW * gridR; x += gridR / double(res)) {
				Eigen::MatrixXd X(2, 1);
				X(0, 0) = x;
				X(1, 0) = y;
				PointTI point;
				point.x = x;
				point.y = y;
				double value = ndtGrid.Value(PointT(x, y, 0));
				point.z = value;
				point.intensity = value;
				if (value > 0.01) {
					pdf->points.push_back(point);
				}
			}
		}
		// Render the resulting point cloud with probability values 
		renderPointCloudI(viewer, pdf, "pdf");
		// Load the `source` point cloud
		PointCloudT::Ptr source(new PointCloudT);
  		pcl::io::loadPCDFile("source.pcd", *source);
		// Render the `source` point cloud
		renderPointCloud(viewer, source, "source", Color(1,0,0));
		double sourceScore = Score(source, ndtGrid);
		viewer->addText(
			"Score: " + std::to_string(sourceScore), 
			200, 200, 32, 1.0, 1.0, 1.0, "score", 0
		);
		double currentScore = sourceScore;
		int iteration = 0;
  		while (!viewer->wasStopped ()) {
			if (matching) {
				viewer->removeShape("score");
				viewer->addText(
					"Score: " + std::to_string(currentScore),
					200, 200, 32, 1.0, 1.0, 1.0, "score", 0
				);
				Eigen::MatrixXd g(3, 1);
				g << Eigen::MatrixXd::Zero(3, 1);
				Eigen::MatrixXd H(3, 3);
				H << Eigen::MatrixXd::Zero(3, 3);
				for (PointT point:source->points) {
					Cell cell = ndtGrid.getCell(point);
					if (cell.cloud->points.size() > 2) {
						double theta = pose.rotation.yaw;
						double x = pose.position.x;
						double y = pose.position.y;
						// TODO: Compute the point transform using translation
						// matrix parameterised by `x`, `y`, `theta`
						// Note: values should be non-zero
						// pointTran(new x, new y, point.z)
						PointT pointTran(
							0, 
							0, 
							point.z
						);
						NewtonsMethod(
							pointTran, 
							theta, 
							cell, 
							g, 
							H
						);
					}
				}
				// TODO: Change the increment and max to non-zero values
				PosDef(H, 0, 0, 0);
				Eigen::MatrixXd T = -H.inverse() * g;
				 // CANDO: Modify the `computeStepLength`
				 // currently function is not optimised
				double alpha = computeStepLength(
					T, 
					source, 
					pose, 
					ndtGrid, 
					currentScore
				);
				T *= alpha;	
				pose.position.x += T(0, 0);
				pose.position.y += T(1, 0);
				pose.rotation.yaw += T(2, 0);
				while (pose.rotation.yaw > 2 * M_PI) {
					pose.rotation.yaw -= 2 * M_PI;
				}
				Eigen::Matrix4d transform = transform3D(
					pose.rotation.yaw, 
					0, 
					0, 
					pose.position.x, 
					pose.position.y, 
					0
				);
				PointCloudT::Ptr transformedScan(new PointCloudT);
				pcl::transformPointCloud(
					*source, 
					*transformedScan, 
					transform
				);
				// Compute the transform score (transformation error)
				double ndtScore = Score(transformedScan, ndtGrid);
				viewer->removeShape("nscore");
				viewer->addText(
					"NDT Score: " + std::to_string(ndtScore),
					200, 150, 32, 1.0, 1.0, 1.0, "nscore", 0
				);
				currentScore = ndtScore;
				iteration++;
				viewer->removePointCloud("ndt_scan");
				renderPointCloud(
					viewer, 
					transformedScan, 
					"ndt_scan", 
					Color(0,1,0)
				);
				matching = false;
			}
			else if (update) {
				Eigen::Matrix4d userTransform = transform3D(
					upose.rotation.yaw, 
					upose.rotation.pitch, 
					upose.rotation.roll, 
					upose.position.x, 
					upose.position.y, 
					upose.position.z
				);
				PointCloudT::Ptr transformedScan(new PointCloudT);
  				pcl::transformPointCloud(
					*source, 
					*transformedScan, 
					userTransform
				);
				viewer->removePointCloud("usource");
				renderPointCloud(
					viewer, 
					transformedScan, 
					"usource", 
					Color(0,1,1)
				);
				double score = Score(transformedScan, ndtGrid);
				viewer->removeShape("score");
				viewer->addText(
					"Score: " + std::to_string(score), 
					200, 200, 32, 1.0, 1.0, 1.0, "score", 0
				);
				update = false;
			}
  			viewer->spinOnce ();
  		}
  	}
	return 0;
}

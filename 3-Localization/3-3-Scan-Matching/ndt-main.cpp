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
 * 						 each cell follows a normal distribution and therefore
 * 						 is defined by a mean and covariance. The Euclidean
 * 						 transformation matrix is formed by optimising the
 * 						 parameters of the transform which maps the `target`
 * 						 point cloud to the `source` using the gradient-based
 * 						 Newton's method framed as a maximisation problem.
 * 
 * Note : The objects defined here are intended to be used with the Point
 *        Cloud Library (PCL) and Eigen libraries in order to compute the
 *        Normal Distributions Transform (NDT) algorithm.
 * ----------------------------------------------------------------------------
 */


#include "helpers.h"
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   					// TicToc time-tracking

/*** Define the starting variables as global variables ***/
Pose3D pose(Point3D(0, 0, 0), Rotate(0, 0, 0));
Pose3D upose = pose;
bool matching = false;
bool update = false;

// Setting the positive definite variables
double kStartValue = 0;
double kIncrementValue = 5.0;
int kMaxIterations = 100;

// CANDO: Increase the iteration count to get convergence
int kGradientIterations = 20;

// Part 1 : Visualise the probabilities of a single `grid` computed with PDF
// If False, Part 2 is run over all `grid` instances in point cloud
bool kRunPart1 = false;


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
	if (event.getKeySym() == "Right" && event.keyDown()) {
		update = true;
		upose.position.x += 0.1;
  	}
	else if (event.getKeySym() == "Left" && event.keyDown()) {
		update = true;
		upose.position.x -= 0.1;
  	}
  	if (event.getKeySym() == "Up" && event.keyDown()) {
		update = true;
		upose.position.y += 0.1;
  	}
	else if (event.getKeySym() == "Down" && event.keyDown()) {
		update = true;
		upose.position.y -= 0.1;
  	}
	else if (event.getKeySym() == "k" && event.keyDown()) {
		update = true;
		upose.rotation.yaw += 0.1;
		while( upose.rotation.yaw > 2 * M_PI)
			upose.rotation.yaw -= 2 * M_PI;  
  	}
	else if (event.getKeySym() == "l" && event.keyDown()) {
		update = true;
		upose.rotation.yaw -= 0.1;
		while( upose.rotation.yaw < 0)
			upose.rotation.yaw += 2 * M_PI; 
  	}
	else if (event.getKeySym() == "space" && event.keyDown()) {
		matching = true;
		update = false;
  	}
	else if (event.getKeySym() == "n" && event.keyDown()) {
		pose = upose;
		std::cout << "Set New Pose" << "\n";
  	}
	else if (event.getKeySym() == "b" && event.keyDown()) {
		std::cout << "Do ICP With Best Associations" << "\n";
		matching = true;
		update = true;
  	}
}


/* Returns the probability of the given point w.r.t. its mean and stdev.
 *
 * The probability of measuring a sample at 2D point `X` contained in a
 * respective `Cell` is given by evaluating the the normal distribution (ND)
 * 		$\mathcal{N} \sim \left(q, \Sigma\right)$,
 * at the given position `X`. The ND is parameterised by the mean `Q` and
 * covariance `S` matrices in two dimensions `x` and `y`.
 * 
 * @param    X		Position to obtain the probability of.
 * @param    Q		Mean value of coordinates in `x` and `y` within a `Cell`.
 * @param    S		Covariance matrix belonging to the ND of the`Cell`.
 * @returns  probability
 */
double Probability(
		Eigen::MatrixXd X, 
		Eigen::MatrixXd Q, 
		Eigen::MatrixXd S
) {
	return exp(-((X - Q).transpose() * S.inverse() * (X - Q))(0, 0) / 2);
}

/* Initialises the constant size `Cell` instance in a discretised grid. 
 *
 * @struct  Cell	"ndt-main.cpp"
 * @var		cloud	Cluster of points from the discretised point cloud.
 * @var		Q		Mean value of the cell, a 2x1 vector with average `x`, `y`.
 * @var		S		Covariance matrix, confidence that point belongs to `Cell`.
 */
struct Cell {
	PointCloudT::Ptr cloud;
	Eigen::MatrixXd Q;
	Eigen::MatrixXd S;

	Cell() {
		PointCloudT::Ptr input(new PointCloudT);
		cloud = input;
		// Initialise the mean and covariance matrices
		Q = Eigen::MatrixXd::Zero(2, 1);	// Mean
		S = Eigen::MatrixXd::Zero(2, 2);    // Covariance
	}
};


/* Implements the overlapping `Grid` of `Cells` (Biber, 2003).
 * 
 * Here a `Grid` is a unit of subdivided area within a discretised point cloud.
 * Each `Grid` is a set of overlapping `Cells` containing a subset of points
 * from the total point cloud. The `Grid` is implemented to minimise the effect
 * of discretisation s.t. each 2D point falls into the overlapping `Cells`
 * within the `Grid`. Each `Cell` is a square of dimensions `res` x `res`, and
 * each `Grid` is of size (2 * `width`) x (2 * `height`).
 * 
 * For example, a cell `res` of 3 and a grid `width` = `height` = 2 will result
 * in a set of 4 overlapping cells, each with dimensions 3 x 3 inside a grid of
 * size 4 x 4. The three overlapping cells are offset from the first in the
 * lower-left origin of the grid by a shift of floor(`res` / 2), i.e., 1 unit
 * to the right and/or 1 unit upwards. The probability density of a given point
 * inside the `Grid` is therefore the sum of the densities of all four cells.
 *
 * @struct   Grid		Represents a discretised region of the point cloud.
 * @var      res		Resolution of each `Cell` (its width and height).
 * @var	     width		The half-width of the overlapping `Grid`.
 * @var      height		The half-height of the overlapping `Grid`.
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
	double res;
	int width;
	int height;
	std::vector<std::vector<Cell>> grid;
	
	Grid(double setRes,
		 int setWidth,
		 int setHeight
	) {
		res = setRes;
		width = setWidth;
		height = setHeight;
		// Discretise the area of the `Grid` into
		// unit length `Cell` instances
		for (int r = 0; r < height * 2; r++) {
			std::vector<Cell> row;
			for (int c = 0; c < width * 2; c++) {
				// Create each `Cell` instance in the current `row`
				row.push_back(Cell());
			}
			// Append the discretised `row` to the `grid` list
			grid.push_back(row);
		}
	}
	// Adds a given `point` to the `grid` if within the grid boundary limits
	void addPoint(PointT point) {
		// UNCOMMENT TO PRINT COORDINATES OF POINT TO ADD
		// std::cout << point.x << "," << point.y << "\n";
		// Compute the coordinates of the `Cell` to store the `point`
		int c = int((point.x + width * res) / res);
		int r = int((point.y + height * res) / res);
		// UNCOMMENT TO PRINT ROW / COLUMN OF CELL
		// std::cout << r << "," << c << "\n";
		if ((c >= 0 && c < width * 2)
			 && (r >= 0 && r < height * 2)
		) {
			// Add the `point` to the `Cell` points cluster
			grid[r][c].cloud->points.push_back(point);
		}
	}
	// Initialise the `grid` mean and covariance
	void Build() {
		for (int r = 0; r < height * 2; r++) {
			for (int c = 0; c < width * 2; c++) {
				PointCloudT::Ptr input = grid[r][c].cloud;
				// 1. For each `Cell` that contains at least three points
				if (input->points.size() > 2) {
					// 2. Calculate the mean values of coordinates `x`, `y`
					Eigen::MatrixXd Q(2, 1);
					Q << Eigen::MatrixXd::Zero(2, 1);
					for(PointT point : input->points){
						Q(0, 0) += point.x;
						Q(1, 0) += point.y;
					}
					Q(0, 0) /= input->points.size();
					Q(1, 0) /= input->points.size();
					grid[r][c].Q = Q;
					// 3. Calculate the covariance matrix (sigma)
					Eigen::MatrixXd S(2, 2);
					S << Eigen::MatrixXd::Zero(2, 2);
					for (PointT point : input->points) {
						Eigen::MatrixXd X(2, 1);
						X(0, 0) = point.x;
						X(1, 0) = point.y;
						// Compute the summed standard deviation
						S += (X - Q) * (X - Q).transpose();
					}
					S(0, 0) /= input->points.size();
					S(0, 1) /= input->points.size();
					S(1, 0) /= input->points.size();
					S(1, 1) /= input->points.size();
					grid[r][c].S = S;
				}
			}
		}
	}
	// Returns the `Cell` in `grid` containing the `point`
	Cell getCell(PointT point) {
		int c = int((point.x + width * res) / res);
		int r = int((point.y + height * res) / res);
		if ((c >= 0 && c < width * 2)
			 && (r >= 0 && r < height * 2)
		) {
			return grid[r][c];
		}
		// Return empty `Cell` if `point` is out of bounds
		return Cell();
	}
	// Returns the summed probability of `point` in `grid`
	double Value(PointT point) {
		Eigen::MatrixXd X(2, 1);
		X(0, 0) = point.x;
		X(1, 0) = point.y;
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


/* Calculates the probability distribution function for an `input` point cloud.
 *
 * @param	 input		Cluster of points from the discretised point cloud.
 * @param	 res		Resolution of each `Cell` (its width and height).
 * @param  	 viewer		PCL Viewer to update with distribution probabilities.
 * @returns  cell		Cell updated with the distribution probabilities.
 */
Cell PDF(
		PointCloudT::Ptr input, 
		int res, 
		pcl::visualization::PCLVisualizer::Ptr& viewer
) {
	// Calculate the 2x1 matrix `Q`,
	// i.e., the mean of the input points
	Eigen::MatrixXd Q(2, 1);
	Q << Eigen::MatrixXd::Zero(2, 1);
	for (PointT point : input->points) {
		Q(0, 0) += point.x;
		Q(1, 0) += point.y;
	}
	Q /= input->points.size();
	//Q(0, 0) /= input->points.size();
	//Q(1, 0) /= input->points.size(); 
	// Calculate the 2x2 covariance matrix `S`,
	// i.e., standard deviation of input points
	Eigen::MatrixXd S(2, 2);
	S << Eigen::MatrixXd::Zero(2, 2);
	for (PointT point : input->points) {
		Eigen::MatrixXd X(2, 1);
		X(0, 0) = point.x;
		X(1, 0) = point.y;
		S += (X - Q) * (X - Q).transpose();
	}
	S /= input->points.size();
	PointCloudTI::Ptr pdf(new PointCloudTI);
	// TODO: Change loop iterator to `int` type
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


/* Implements the optimisation problem using Newton's algorithm.
 *
 * Determines the corresponding normal distributions for each mapped
 * input `point` parameterised by estimate `theta`. The Newton's algorithm
 * iteratively finds the parameters that minimise the function $f$ by solving
 * 		$H\Delta p = -g$,
 * s.t. $g$ is the transposed gradient of $f$ and $H$ is the Hessian of $f$.
 * The solution to this linear system is an increment $\Delta p$, which is
 * added to the previous estimates `g_previous`, `H_previous`.
 * 
 * @param  point		Point mapped into `target` point cloud.
 * @param  theta		Initial value of parameter to estimate.
 * @param  cell			`Cell` to compute the score of. 
 * @param  g_previous	Estimated gradient to update. 
 * @param  H_previous	Estimated Hessian to update.
 */
// Using `Derived` type to handle intermediate matrices
template<typename Derived>
void NewtonsMethod(
		PointT point, 
		double theta, 
		Cell cell, 
		Eigen::MatrixBase<Derived>& g_previous, 
		Eigen:: MatrixBase<Derived>& H_previous
) {
	// Get the mean and covariance matrices of the `cell`
	Eigen::MatrixXd Q = cell.Q;
	Eigen::MatrixXd S = cell.S;
	Eigen::MatrixXd S_inverse = S.inverse();
	// Construct a 2x1 matrix position from input `point`
	Eigen::MatrixXd X(2, 1);
	X(0, 0) = point.x;
	X(1, 0) = point.y;
	// Calculate matrix `q` from `X` and `Q` (Eq. 8)
	Eigen::MatrixXd q = X - Q;
	// Calculate the 1x1 exponential matrix `s` (Eq. 9)
	Eigen::MatrixXd s(1, 1);
	s << -exp((-q.transpose() * S_inverse * q)(0, 0) / 2);
	// Calculate the three 2x1 partial derivative matrices (Eqs. 10, 11)
	Eigen::MatrixXd q_p1(2, 1);		// Partial derivative w.r.t. `x`
	Eigen::MatrixXd q_p2(2, 1);		// Partial derivative w.r.t. `y`
	Eigen::MatrixXd q_p3(2, 1);		// Partial derivative w.r.t. `theta`
	q_p1(0, 0) = 1;
	q_p1(1, 0) = 0;
	q_p2(0, 0) = 0;
	q_p2(1, 0) = 1;
	q_p3(0, 0) = -X(0, 0) * sin(theta) - X(1, 0) * cos(theta);  
	q_p3(1, 0) = X(1, 0) * cos(theta) - X(1, 0) * sin(theta);
	// Calculate the 2x1 second-order partial derivatives matrix (Eq. 13)
	Eigen::MatrixXd q_pp(2, 1);
	q_pp(0, 0) = -X(0, 0) * cos(theta) + X(1, 0) * sin(theta);
	q_pp(1, 0) = -X(0, 0) * sin(theta) - X(1, 0) * cos(theta);
	// Calculate the gradient `g` (Eq. 10)
	Eigen::MatrixXd g(3, 1);
	g << Eigen::MatrixXd::Zero(3, 1);
	g(0, 0) = ((q.transpose() * S_inverse * q_p1) * s)(0, 0);
	g(1, 0) = ((q.transpose() * S_inverse * q_p2) * s)(0, 0);
	g(2, 0) = ((q.transpose() * S_inverse * q_p3) * s)(0, 0);
	// Calculate the Hessian matrix `H`
	Eigen::MatrixXd H(3, 3);
	H(0, 0) = (
		-s * ((-q.transpose() * S_inverse * q_p1)
		* (-q.transpose() * S_inverse * q_p1) 
	 	+ (-q_p1.transpose() * S_inverse * q_p1))
	)(0, 0);
	H(0, 1) = (
		-s * ((-q.transpose() * S_inverse * q_p1)
		* (-q.transpose() * S_inverse * q_p2) 
	 	+ (-q_p2.transpose() * S_inverse * q_p1))
	)(0, 0);
	H(0, 2) = (
		-s * ((-q.transpose() * S_inverse * q_p1)
		* (-q.transpose() * S_inverse * q_p3) 
	 	+ (-q_p3.transpose() * S_inverse * q_p1))
	)(0, 0);
	H(1, 0) = (
		-s * ((-q.transpose() * S_inverse * q_p2)
		* (-q.transpose() * S_inverse * q_p1) 
	 	+ (-q_p1.transpose() * S_inverse * q_p2))
	)(0, 0);
	H(1, 1) = (
		-s * ((-q.transpose() * S_inverse * q_p2)
		* (-q.transpose() * S_inverse * q_p2) 
	 	+ (-q_p2.transpose() * S_inverse * q_p2))
	)(0, 0);
	H(1, 2) = (
		-s * ((-q.transpose() * S_inverse * q_p2)
		* (-q.transpose() * S_inverse * q_p3) 
	 	+ (-q_p3.transpose() * S_inverse * q_p2))
	)(0, 0);
	H(2, 0) = (
		-s * ((-q.transpose() * S_inverse * q_p3)
		* (-q.transpose() * S_inverse * q_p1) 
	 	+ (-q_p1.transpose() * S_inverse * q_p3))
	)(0, 0);
	H(2, 1) = (
		-s * ((-q.transpose() * S_inverse * q_p3)
		* (-q.transpose() * S_inverse * q_p2) 
	 	+ (-q_p2.transpose() * S_inverse * q_p3))
	)(0, 0);
	H(2, 2) = (
		-s * ((-q.transpose() * S_inverse * q_p3)
		* (-q.transpose() * S_inverse * q_p3) 
	 	+ (-q.transpose() * S_inverse * q_pp)
		+ (-q_p3.transpose() * S_inverse * q_p3))
	)(0, 0);
	// Update the gradient and Hessian with this step increment
	H_previous += H;
	g_previous += g;
}


/* Computes the sum of the normal distributions in `grid`.
 *
 * @param	 cloud		Cluster of points from the discretised point cloud. 
 * @param	 grid		Set of `Cell` to evaluate.
 * @returns  score		The probability summed from each `Cell` PDF.  
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


/* Computes the score of the estimated transformation given by matrix `T`.
 *
 * @param 	 alpha 		The estimated step size to scale the transform `T` by.
 * @param 	 T 			Estimated transform between `source` and `target`.
 * @param 	 source 	Point cloud to transform by matrix `T`.
 * @param 	 pose 		Current pose to translate the matrix `T`. 
 * @param 	 grid 		Set of discretised `Cell` instances. 
 * @returns  score		Transform score, i.e., sum of `Cell` probabilities.
 */
double AdjustmentScore(
		double alpha, 
		Eigen::MatrixXd T, 
		PointCloudT::Ptr source, 
		Pose3D pose, 
		Grid grid
) {
	T *= alpha;
	pose.position.x += T(0, 0);
	pose.position.y += T(1, 0);
	pose.rotation.yaw += T(2, 0);
	while(pose.rotation.yaw > 2 * M_PI) {
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
	double score = Score(transformedScan, grid);
	// UNCOMMENT TO PRINT ADJUSTMENT SCORE
	// std::cout << "Score would be " << score << "\n";
	return score;
}


/* Computes the step size $\alpha$ for the transform update. 
 * 
 * @param   T 			Estimated transform between `source` and `target`.
 * @param   source 		Point cloud to transform by matrix `T`.
 * @param   pose 		Current pose to translate the matrix `T`.
 * @param   grid 		Set of discretised `Cell` instances.
 * @param   currScore 	Previous adjustment score to minimise.
 * @returns weighted `alpha` value minimised over number of iterations.
 */ 
double computeStepLength(
		Eigen::MatrixXd T, 
		PointCloudT::Ptr source, 
		Pose3D pose, 
		Grid grid, 
		double currScore
) {
	double maxParam = std::max(std::max(T(0, 0), T(1, 0)), T(2, 0));
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
 * Assumes that the `start` Hessian `H` is not positive definite 
 * s.t. $f(p)$ will initially decrease in the direction of $\Delta p$.
 * Therefore, we replace $H$ with $H^{\prime} = H + \lambda I$, with
 * $\lambda$ (`increment * count`) chosen such that $H^{\prime}$ is
 * safely positive definite.
 *
 * @param	 start		Initial value of the Hessian.
 * @param	 increment  Step size w.r.t. the change in $\Delta p$.
 * @param	 maxIt		Number of Hessian update steps to perform .
 * @returns  H_prime    Hessian assumed to be positive definite.
 */
// Using `Derived` type to handle intermediate matrices
template<typename Derived>
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
	return start + increment * count;
}


/* Performs and visualises the NDT scan matching algorithm (Biber et al., 2003).
 *
 * The `source.pcd` and `target.pcd` files are loaded from the local
 * file system and visualised with the Point Cloud Library (PCL).
 * The NDT algorithm is performed with an assumed `Cell` resolution of
 * `3.0` and a grid width and height of 2 * `2.0`. Therefore each `grid`
 * instance has dimensions (2 * 2.0)x(2 * 2.0) = 4x4 and is made up of
 * a set of four overlapping 3x3 cells in each `grid`. The NDT algorithm
 * assumes that the distribution of points contained in each `Cell` is
 * described by a normal distribution parameterised by a mean $q$ and
 * covariance $\Sigma$, in two dimensions `x` and `y`.
 * 
 * The two point clouds are registered (aligned) in an iterative process
 * performed by Newton's algorithm. The corresponding adjustment score at
 * each step is calculated as the sum of probabilities of a given point
 * belonging to a respective `Cell` within the `grid`. While in nature
 * this is a maximisation problem w.r.t. the adjustment score, we frame
 * the optimisation steps as a minimisation for consistency with literature.
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
	if (kRunPart1) {
		// Create `input` from set of 2D Cartesian coordinates (x, y)
		// Each point is in the range [0, 10]
		PointCloudT::Ptr input(new PointCloudT);
  		input->points.push_back(PointT(4, 4, 0));
		input->points.push_back(PointT(4, 9, 0));
		input->points.push_back(PointT(5, 5, 0));
		input->points.push_back(PointT(5, 2, 0));
		input->points.push_back(PointT(7, 5, 0));
		// Render the cell border
		renderRayT::renderRay(
			viewer, 
			PointT(0, 0, 0), 
			PointT(0, 10, 0), 
			"left", 
			Color(0, 0, 1)
		);
		renderRayT::renderRay(
			viewer, 
			PointT(0, 10, 0),
			PointT(10, 10, 0), 
			"top", 
			Color(0, 0, 1)
		);
		renderRayT::renderRay(
			viewer, 
			PointT(10, 10, 0), 
			PointT(10, 0, 0), 
			"right", 
			Color(0, 0, 1)
		);
		renderRayT::renderRay(
			viewer, 
			PointT(0,0,0), 
			PointT(10,0,0), 
			"bottom", 
			Color(0,0,1)
		);
		// Compute and visualise the probabilities from the PDF
		Cell cell = PDF(input, 200, viewer);
		// CANDO: Change the test `point` and observe the effect on convergece
		PointT point(1,2,1);
		input->points.push_back(
			PointT(point.x, point.y, 1.0)
		);
		for (int iteration = 0; iteration < kGradientIterations; iteration++) { 
			Eigen::MatrixXd g(3, 1);
			g << Eigen::MatrixXd::Zero(3, 1);
			Eigen::MatrixXd H(3, 3);
			H << Eigen::MatrixXd::Zero(3, 3);
			// Perform the `NewtonsMethod` optimisation function
			double theta = 0.0;
			NewtonsMethod(
				point,
				theta, 
				cell, 
				g, 
				H
			);
			 // CANDO: Change the increment and max values to nonzero values
			PosDef(
				H,
				kStartValue, 
				kIncrementValue, 
				kMaxIterations
			);
			// Calculate the 3x1 matrix `T` using positive definite `H`
			Eigen::MatrixXd T = -H.inverse() * g;
			// Calculate new point by transforming by the `T` matrix
			// `T` has the form: [x translation, y translation, theta rotation]
			double newX = (point.x * cos(T(2, 0)) - point.y * sin(T(2, 0)) 
						   + T(0, 0) - point.x
			);
			double newY = (point.x * sin(T(2, 0)) + point.y * cos(T(2, 0)) 
						    + T(1, 0) - point.y
			);
			PointT pointT(newX, newY, 1);
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
			renderRayT::renderRay(
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
  		pcl::io::loadPCDFile("../target.pcd", *target);
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
			renderRayT::renderRay(
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
			renderRayT::renderRay(
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
		for (double y = -gridH * gridR; y <= gridH * gridR; y += gridR / double(res)) {
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
  		pcl::io::loadPCDFile("../source.pcd", *source);
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
						// Compute the point transform using translation
						// matrix parameterised by `x`, `y`, `theta`
						double newX = (
							point.x * cos(theta) - point.y * sin(theta) + x
						);
						double newY = (
							point.x * sin(theta) + point.y * cos(theta) + y
						);
						double newZ = point.z;
						PointT pointTran(newX, newY, newZ);
						// Re-compute Newton's method using transformed point
						NewtonsMethod(
							pointTran, 
							theta, 
							cell, 
							g, 
							H
						);
					}
				}
				// CANDO: Change the increment and max values
				PosDef(
					H, 
					kStartValue, 
					kIncrementValue,
					kMaxIterations
				);
				// Compute the updated transformation
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

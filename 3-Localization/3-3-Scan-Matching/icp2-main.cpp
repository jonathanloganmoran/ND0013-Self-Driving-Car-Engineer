/* ----------------------------------------------------------------------------
 * Lesson "3.3: Scan Matching Algorithms"
 * Authors     : Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Main entry into the localisation programme. Here the
 *                       Iterative Closest Point (ICP) algorithm performs the
 *                       association using a KD Tree search on centred vectors.
 *                       The translation and rotation parameters between two
 *                       point clouds are recovered using the Singular Value
 *                       Decomposition (SVD). The resulting transformation
 *                       matrix is formed and visualised onto the PCL Viewer.
 * 
 * Note : The objects defined here are intended to be used with the Point
 *        Cloud Library (PCL) and Eigen libraries in order to compute the
 *        Iterative Closest Point (ICP) algorithm.
 * ----------------------------------------------------------------------------
 */


#include "helpers.h"                        // 3D Helper functions
#include <Eigen/Core>
#include <Eigen/SVD>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>               // TicToc
#include <string>
#include <sstream>


Pose3D pose(Point3D(0, 0, 0), Rotate(0, 0, 0));
Pose3D upose = pose;
std::vector<int> associations;
std::vector<int> bestAssociations = {
    5,6,7,8,9,10,11,12,13,14,15,16,16,17,18,19,20,21,22,23,24,25,26,26,27,28,
    29,30,31,32,33,34,35,36,37,38,39,40,42,43,44,45,46,47,48,49,50,51,52,53,54,
    55,56,57,58,59,60,62,63,64,65,66,67,68,69,70,71,72,74,75,76,77,78,79,80,81,
    82,83,84,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,
    106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,122,123,
    124,125,126,127,0,1,2,3,4,4
};
bool init = false;
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

/* Returns the error between the transformed `source` and `target` point cloud.
 * 
 * Here the sum of the distances between each point association is computed.
 * The smaller the overall `score`, the greater the total overlap is.
 * 
 * The `pairs` vector contains the indices of the closest matching points in
 * `target` for each of the given points from `source`.
 * 
 * @param   pairs       Set of matching `target` indices for each `source` `i`.
 * @param   target      Set of points in the `target` cloud. 
 * @param   source      Set of points in the `source` cloud.
 * @param   transform   Transformation matrix to apply to the `source` points.
 * @returns score       Error (summed distance) between the two point clouds.
 *
 */
double Score(
        std::vector<int> pairs,
        PointCloudT::Ptr target,
        PointCloudT::Ptr source,
        Eigen::Matrix4d transform
) {
	double score = 0;
	int index = 0;
	for(int i : pairs) {
		Eigen::MatrixXd p(4, 1);
		p(0, 0) = (*source)[index].x;
		p(1, 0) = (*source)[index].y;
		p(2, 0) = 0.0;
		p(3, 0) = 1.0;
	Eigen::MatrixXd p2 = transform * p;
		PointT association = (*target)[i];
		score += sqrt(
            ((p2(0, 0) - association.x) * (p2(0, 0) - association.x))
            + ((p2(1, 0) - association.y) * (p2(1, 0) - association.y))
        );
		index++;
	}
	return score;
}

/* Returns the nearest neighbour associations.
 * 
 * An association is a pair of indices from `target` and `source` which have
 * the closest distance to each other. The nearest neighbour association is
 * computed with a raidus search using the centred vectors stored in a KD Tree.
 * 
 * The radius search allows the association algorithm to search for points
 * within a restricted radius of the centred vector, making it faster and more
 * efficient since only a subset of the total search space is explored.
 * 
 * @param   target          Set of points in the target point cloud.
 * @param   source          Set of points in the source point cloud.
 * @param   initTransform   Rotation and translation of the starting pose. 
 * @param   dist            Maximum distance to consider for an association.
 * @returns associations    Set of the closest matching index in `target` for
 *                          each point given in the transformed `source`.
 */
std::vector<int> NN(
        PointCloudT::Ptr target,
        PointCloudT::Ptr source,
        Eigen::Matrix4d initTransform,
        double dist
){
	
	std::vector<int> associations;

	// TODO: complete this function which returns a vector of target indicies that correspond to each source index inorder.
	// E.G. source index 0 -> target index 32, source index 1 -> target index 5, source index 2 -> target index 17, ... 

	// TODO: create a KDtree with target as input

	// TODO: transform source by initTransform

	// TODO loop through each transformed source point and using the KDtree find the transformed source point's nearest target point. Append the nearest point to associaitons 

	return associations;
}


/* Returns the set of associations as pairs of indices.
 * 
 * Each pair is a set of two points, one from each cloud, which are determined
 * to be the closest in distance between all other points. For example, the
 * `target` point `p1` and the `source` point `p2` form a pair if their
 * distance is minimised to all other points in either cloud.
 *
 * @param   associations    Corresponding indices of `source` in `target`. 
 * @param   target          Set of points in the target point cloud.
 * @param   source          Set of points in the source point cloud.
 * @param   render          If True, the associations are displayed.
 * @returns viewer          PCL Viewer instance to display the associations.
 */
std::vector<Pair> PairPoints(
        std::vector<int> associations, 
        PointCloudT::Ptr target, 
        PointCloudT::Ptr source, 
        bool render, 
        pcl::visualization::PCLVisualizer::Ptr& viewer
) {
	std::vector<Pair> pairs;
    int idx_source = 0;
    for (PointT point : source->points) {
        // Get the corresponding point in `associations`
        int idx_target = associations[i];
        if (idx_target >= 0) {
            PointT association = (*target)[idx_target];
            if (render) {
                viewer->removeShape(std::to_string(idx_source));
                renderRay(viewer,
                          Point2D(point.x, point.y),
                          Point2D(association.x, association.y),
                          std::to_string(idx_source),
                          Color(0, 1, 0)
                );
            }
            pairs.push_back(Pair2D(
                Point2D(point.x, point.y),
                Point2D(association.x, association.y)
            ));
        }
        idx_source++;
    }
    return pairs;
}


/* Performs the Iterative Closest Point (ICP) algorithm.
 *
 * Recovers the transformation matrix between the points in the source and
 * the target point clouds using the SVD algorithm. The points are associated
 * using a minimised Euclidean distance metric computed with through a radius
 * search over the KD Tree of centred vectors formed by the points in each
 * cloud. The radius search reduces the number of candidate associations
 * to those only within a certain radius of the centred vectors.
 * 
 * @param   associations    Corresponding indices of `source` in `target`.
 * @param   target          Set of points in the target point cloud.
 * @param   source          Set of points in the source point cloud.
 * @param   startingPose    Initial pose with to transform `source` with.
 * @param   iterations      Maximum iterations to run the alignment for.
 * @returns transformationMatrix
 */
Eigen::Matrix4d ICP(
        std::vector<int> associations,
        PointCloudT::Ptr target,
        PointCloudT::Ptr source,
        Pose3D startingPose,
        int iterations,
        pcl::visualization::PCLVisualizer::Ptr& viewer
) {
  	Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
    // Transform the `source` point cloud by the `startingPose`
    Eigen::Matrix4d initTransform = transform3D(
        startingPose.rotation.yaw,
        startingPose.rotation.pitch,
        startingPose.rotation.roll,
        startingPose.position.x,
        startingPose.position.y,
        startingPose.position.z
    );
    PointCloudT::Ptr transformSource(new PointCloudT);
    pcl::transformPointCloud(*source, *transformSource, initTransform);
    // Compute the pairs (associations) between the two point clouds
    vector<Pair2D> pairs = PairPoints()
  	// Create 2x1 matrices `P`, `Q` which represent mean point of pairs 1, 2
    Eigen::MatrixXd P;
    Eigen::MatrixXd Q;
    P << Eigen::MatrixXd::Zeros(2, 1);
    Q << Eigen::MatrixXd::Zeros(2, 1);
  	for (int i = 0; i < associations.size(); i++) {
        P(0, 0) += transformSource->points[i].x;
        P(1, 0) += transformSource->points[i].y;
        Q(0, 0) += target->points[associations[i]].x;
        Q(0, 1) += target->points[associations[i]].y;
    }
    P /= associations.size();
    Q /= associations.size();
    // Create 2xn matrices `X`, `Y` where `n` is the number of pairs
    Eigen::MatrixXd X;
    Eigen::MatrixXd Y;
    for (int i = 0; i < associations.size(): i++) {
        X(0, i) = transformSource->points[i].x - P(0);
        X(1, i) = transformSource->points[i].y - P(1);
        Y(0, i) = target->points[associations[i]].x - Q(0);
        Y(1, i) = target->points[associations[i]].y - Q(1);
    }
  	// Create matrix `S` using Eq. 3 from the `svd_rot.pdf`
    // Here `W` is the identity matrix since all weights have value `1`
    Eigen::Matrix2d W = Eigen::Matrix2d::Identity();
    Eigen::MatrixXd S = Y * X.transpose() * W;
  	// Create matrix `R`, i.e., the optimal rotation
    // using Eq. 4 from `svd_rot.pdf` and taking the SVD of `S`
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        S, Eigen::ComputeThinU | Eigen::ComputeThinV
    );
    // Create vector `t`, i.e., the optimal translation,
    // using Eq. 5 from `svd_rot.pdf` 
    Eigen::Vector2d t = Q - R * P;
  	// Set the `transformationMatrix` based on recovered `R` and `t`
    transformationMatrix(0, 0) = R(0, 0);
    transformationMatrix(0, 1) = R(0, 1);
    transformationMatrix(1, 0) = R(1, 0);
    transformationMatrix(1, 1) = R(1, 1);
    transformationMatrix(0, 3) = t(0);
    transformationMatrix(1, 3) = t(1);
    // Return the estimated transformation matrix
  	return transformationMatrix;
}

/* Entry point into the localisation programme using ICP scan matching.
 * 
 * Here we use the optimised radius search algorithm to span a KD Tree and
 * compute the association pairs from each point cloud with the closest
 * distance. The associations are used to align the point clouds and recover a
 * transformation between the set of points that minimises the error.
 *   
 */
int main() {
	pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("ICP Creation")
    );
  	viewer->setBackgroundColor(0, 0, 0);
  	viewer->addCoordinateSystem(1.0);
	viewer->registerKeyboardCallback(
        keyboardEventOccurred,
        (void*)&viewer
    );
	// Load target
	PointCloudT::Ptr target(new PointCloudT);
  	pcl::io::loadPCDFile("target.pcd", *target);
	// Load source
	PointCloudT::Ptr source(new PointCloudT);
  	pcl::io::loadPCDFile("source.pcd", *source);
	renderPointCloud(viewer, target, "target", Color(0, 0, 1));
	renderPointCloud(viewer, source, "source", Color(1, 0, 0));
	viewer->addText("Score", 200, 200, 32, 1.0, 1.0, 1.0, "score", 0);

  	while (!viewer->wasStopped()) {
		if (matching) {
			init = true;
			viewer->removePointCloud("usource");
			Eigen::Matrix4d transformInit = transform3D(
                pose.rotation.yaw,
                pose.rotation.pitch, 
                pose.rotation.roll, 
                pose.position.x, 
                pose.position.y, 
                pose.position.z
            );
			PointCloudT::Ptr transformedScan(new PointCloudT);
  			pcl::transformPointCloud(
                *source,
                *transformedScan,
                transformInit
            );
			viewer->removePointCloud("source");
  			renderPointCloud(
                viewer, 
                transformedScan, 
                "source", 
                Color(1, 0, 0)
            );
			if (!update) {
				associations = NN(target, source, transformInit, 5);
            }
            else {
				associations = bestAssociations;
            }
			Eigen::Matrix4d transform = ICP(
                associations,
                target, 
                source, 
                pose, 
                1, 
                viewer
            );
			pose = getPose(transform);
  			pcl::transformPointCloud(
                *source, 
                *transformedScan, 
                transform
            );
			viewer->removePointCloud("icp_scan");
  			renderPointCloud(
                viewer,
                transformedScan, 
                "icp_scan", 
                Color(0,1,0)
            );
			double score = Score(
                associations, 
                target, 
                source, 
                transformInit
            );
			viewer->removeShape("score");
			viewer->addText(
                "Score: " + std::to_string(score),
                200, 200, 32, 1.0, 1.0, 1.0, "score", 0
            );
			double icpScore = Score(
                associations, 
                target, 
                source, 
                transform
            );
			viewer->removeShape("iscore");
			viewer->addText(
                "ICP Score: " + std::to_string(icpScore),
                200, 150, 32, 1.0, 1.0, 1.0, "iscore", 0
            );
			matching = false;
			update = false;
			upose = pose;
		}
		else if (update && init) {
			Eigen::Matrix4d userTransform = transform3D(
                upose.rotation.yaw, 
                upose.rotation.pitch, 
                upose.rotation.roll, 
                upose.position.x, 
                upose.position.y, 
                upose.position.z
            );
			PointCloudT::Ptr transformedScan(new PointCloudT);
  			pcl::transformPointCloud(*source, *transformedScan, userTransform);
			viewer->removePointCloud("usource");
			renderPointCloud(
                viewer, 
                transformedScan, 
                "usource", 
                Color(0, 1, 1)
            );
			std::vector<Pair3D> pairs = PairPoints(
                associations, 
                target, 
                transformedScan, 
                true, 
                viewer
            );
			double score = Score(
                associations, 
                target, 
                source, 
                userTransform
            );
			viewer->removeShape("score");
			viewer->addText(
                "Score: " + std::to_string(score), 
                200, 200, 32, 1.0, 1.0, 1.0, "score", 0
            );
			update = false;
		}
  		viewer->spinOnce();
  	}
	return 0;
}

/* ------------------------------------------------------------------------------
 * Lesson "4.2: Trajectory Generation"
 * Authors     : Sebastian Thrun, Emmanuel Boidot of Mercedes-Benz R&D.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the Hybrid A* search algorithm.
 * ----------------------------------------------------------------------------
 */

#ifndef HYBRID_BREADTH_FIRST_H_
#define HYBRID_BREADTH_FIRST_H_

#include <algorithm>
#include <math.h>
#include <iostream>
#include <vector>


/* The Hybrid A* with Breath-First Search.
 *
 * @class  HBF  "1_hybrid_breadth_first.h"
 * @brief  Implements Hybrid A* (Dolgov et al., 2008) with breadth-first search.
 */
class HBF {
public:
  // Constructor
  HBF();
  // Destructor
  virtual ~HBF();
  // State instance
  struct maze_s {
    int g;          // Node iterations (g-cost)
    double f;       // Heuristic evaluation cost (f-cost)
    double x;       
    double y;
    double theta;   // Continuous vehicle heading
  };
  // Path instance
  struct maze_path {
    std::vector<std::vector<std::vector<int>>> states_closed;
    std::vector<std::vector<std::vector<maze_s>>> states_came_from;
    maze_s final;
  };
  // Returns the corresponding 'stack' in 3D configuration space of the angle
  int theta_to_stack_number(
      double theta
  );
  // Computes the Manhattan distance heuristic
  double heuristic(
      double x,
      double y,
      std::vector<int>& goal
  );
  // Returns the corresponding grid index of the continuous position
  int idx(
      double float_num
  );
  // Expands the given state in the search space
  std::vector<maze_s> expand(
    maze_s& state,
    std::vector<int>& goal
  );
  // Returns the path of nodes to the `final` from `start`
  std::vector<maze_s> reconstruct_path(
      std::vector<std::vector<std::vector<maze_s>>>& states_came_from, 
      std::vector<double> &start, HBF::maze_s& final
  );
  // Implements the breadth-first search algorithm
  maze_path search(
      std::vector<std::vector<int>>& grid, 
      std::vector<double>& start, 
      std::vector<int>& goal
  );
private:
  // Define the number of partitions to divide the control space into
  // Here the control space refers to the direction of motion `theta`
  const int NUM_THETA_CELLS = 90;
  // Define the bicycle motion model parameters
  const double SPEED = 1.45;
  const double LENGTH = 0.5;
};


#endif  // HYBRID_BREADTH_FIRST_H_
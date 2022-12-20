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

#include <math.h>
#include <iostream>
#include <vector>


class HBF {
public:
  // Constructor
  HBF();
  // Destructor
  virtual ~HBF();
  // State instance
  struct maze_s {
    int g;  // iteration
    double x;
    double y;
    double theta;
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
  // Returns the corresponding grid index of the continuous position
  int idx(
      double float_num
  );
  // Expands the given state in the search space
  std::vector<maze_s> expand(
    maze_s& state
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
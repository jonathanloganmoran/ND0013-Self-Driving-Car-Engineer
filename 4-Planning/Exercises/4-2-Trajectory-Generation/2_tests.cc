/* ------------------------------------------------------------------------------
 * Lesson "4.2: Trajectory Generation"
 * Authors     : Sebastian Thrun, Emmanuel Boidot of Mercedes-Benz R&D.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Tests the Exercises 4.2.1 through 4.2.2.
 * ----------------------------------------------------------------------------
 */

#include "1_hybrid_breadth_first.h"
#include "2_jerk_minimising_trajectory.h"
#include <iostream>
#include <vector>


/* Tests the Hybrid A* algorithm implemented with breadth-first search.
 * 
 * Here the 2D grid environment is defined along with a 3D starting pose and
 * a 2D goal position. The Hybrid A* search finds the shortest path by
 * minimising the Manhattan distance cost from the current expansion node to
 * the goal position.
 * 
 * CANDO: Modify the maze environment (obstaces / free-space) or use different
 * cost / heuristic functions (e.g., holonomic-with-obstacles, Dolgov 2008).
 *
 */
void test_hybrid_breadth_first() {
  // Initialising the maze grid
  int X = 1;
  int _ = 0;
  // CANDO: Modify the grid maze to test different expansions.
  std::vector<std::vector<int>> GRID = {
    {_,X,X,_,_,_,_,_,_,_,X,X,_,_,_,_,},
    {_,X,X,_,_,_,_,_,_,X,X,_,_,_,_,_,},
    {_,X,X,_,_,_,_,_,X,X,_,_,_,_,_,_,},
    {_,X,X,_,_,_,_,X,X,_,_,_,X,X,X,_,},
    {_,X,X,_,_,_,X,X,_,_,_,X,X,X,_,_,},
    {_,X,X,_,_,X,X,_,_,_,X,X,X,_,_,_,},
    {_,X,X,_,X,X,_,_,_,X,X,X,_,_,_,_,},
    {_,X,X,X,X,_,_,_,X,X,X,_,_,_,_,_,},
    {_,X,X,X,_,_,_,X,X,X,_,_,_,_,_,_,},
    {_,X,X,_,_,_,X,X,X,_,_,X,X,X,X,X,},
    {_,X,_,_,_,X,X,X,_,_,X,X,X,X,X,X,},
    {_,_,_,_,X,X,X,_,_,X,X,X,X,X,X,X,},
    {_,_,_,X,X,X,_,_,X,X,X,X,X,X,X,X,},
    {_,_,X,X,X,_,_,X,X,X,X,X,X,X,X,X,},
    {_,X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,},
    {X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,}
  };
  std::vector<double> START = {0.0, 0.0, 0.0};
  std::vector<int> GOAL = {
    (int)GRID.size() - 1, 
    (int)GRID[0].size() - 1
  };
  std::cout << "Finding path through grid:" << "\n";
  // Creates an empty maze to test number of expansions
  for (int i = 0; i < GRID.size(); ++i) {
    std::cout << GRID[i][0];
    for (int j = 1; j < GRID[0].size(); ++j) {
      std::cout << "," << GRID[i][j];
    }
    std::cout << "\n";
  }
  HBF hbf = HBF();
  HBF::maze_path get_path = hbf.search(
    GRID,
    START,
    GOAL
  );
  std::vector<HBF::maze_s> show_path = hbf.reconstruct_path(
    get_path.states_came_from, 
    START, 
    get_path.final
  );
  std::cout << "Show path from start to finish" << "\n";
  for (int i = show_path.size()-1; i >= 0; --i) {
      HBF::maze_s step = show_path[i];
      std::cout << "##### step " << step.g << " #####" << "\n";
      std::cout << "x " << step.x << "\n";
      std::cout << "y " << step.y << "\n";
      std::cout << "theta " << step.theta << "\n";
  }
}


/* Tests the quintic polynomial solver using the test cases in `2_grader.h`.
 *
 * The `JMT` function in `2_jerk_minimising_trajectory.cc` returns the six
 * coefficient values corresponding to the quintic polynomial of the minimised
 * jerk trajectory. The matrix equation $Ax = b$ is solved using the inverse of
 * the matrix $A$ which is assumed to exist.
 * 
 * This helper function tests the returned coefficient values against the
 * expected values up to a deviation amount `epsilon` defined in `2_grader.h`.
 */
void test_jerk_minimising_trajectory() {
  // Create the test cases
  std::vector<test_case> tc = create_tests();
  bool total_correct = true;
  for (int i = 0; i < tc.size(); ++i) {
    std::vector<double> jmt = JMT(
        tc[i].start,
        tc[i].end,
        tc[i].T
    );
    bool correct = close_enough(jmt, answers[i]);
    total_correct &= correct;
  }
  if (!total_correct) {
    std::cout << "Try again!" << "\n";
  }
  else {
    std::cout << "Nice work!" << "\n";
  }
}
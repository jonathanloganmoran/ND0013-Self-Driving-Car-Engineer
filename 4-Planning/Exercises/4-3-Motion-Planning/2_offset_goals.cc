/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: October 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/*
In this exercise you will generate "_num_goals" goals, offset from the
center goal at a distance "_goal_offset". The offset goals will be aligned on a
perpendicular line to the heading of the main goal. To get a perpendicular
angle, just add 90 degrees (or π/2 rad) to the main goal heading (ϴ). After
that you will just need to calculate the x and y coordinates for each offset
goal using the equations presented in the lectures:
X_offset = X_center_goal +
goal_number * Offset_distance * cos(ϴ+π/2);
Y_offset = Y_center_goal +
goal_number * Offset_distance * sin(ϴ+π/2)

NOTE:
1) goal_number will go from: -3, -2, -1, 0, 1 ,2 ,3  (for 7 total goals)
2) When goal_number = 0, we will get the "center" goal.
*/

#include <math.h>

#include <cfloat>
#include <cmath>
#include <iostream>
#include <vector>

// In Meters
struct Location {
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
};

// In Radians
struct Rotation {
  float pitch = 0.0;
  float yaw = 0.0;
  float roll = 0.0;
};

struct State {
  Location location;
  Rotation rotation;
};

float _goal_offset = 1.0;
int _num_goals = 7;

std::vector<State> generate_offset_goals(const State& goal_state) {
  // Now we need to gernerate "_num_goals" goals offset from the center goal at
  // a distance "_goal_offset".
  std::vector<State> goals_offset;

  // The offset goals will be aligned on a perpendiclular line to the heading of
  // the main goal. To get a perpendicular angle, just add 90 degrees (or PI/2
  // rad) to the main goal heading.

  // TODO-Perpendicular direction: ADD pi/2 to the goal yaw
  // (goal_state.rotation.yaw)
  // yaw_plus_90 = goal_state.rotation.yaw + pi/2;
  auto yaw_plus_90 = ...;

  for (int i = 0; i < _num_goals; ++i) {
    auto goal_offset = goal_state;
    // This is the offset distance for goal "i"
    float offset = (i - (int)(_num_goals / 2)) * _goal_offset;

    // TODO-offset goal location: calculate the x and y coordinates of the
    // offset goals using "offset" (calculated above) and knowing that the goals
    // should be laid on a perpendicular line to the direction (yaw) of the main
    // goal. You calculated this direction above (yaw_plus_90).
    // HINT: use
    // std::cos(yaw_plus_90) and std::sin(yaw_plus_90)
    // X_offset = X_center_goal +
    // goal_number * Offset_distance * cos(ϴ+π/2);
    // Y_offset = Y_center_goal +
    // goal_number * Offset_distance * sin(ϴ+π/2)
    goal_offset.location.x = ...;
    goal_offset.location.y = ...;

    // std::cout << "x: " << goal_offset.location.x << std::endl;
    // std::cout << "y: " << goal_offset.location.y << std::endl;
    // std::cout << "yaw: " << goal_offset.rotation.yaw << std::endl;

    goals_offset.push_back(goal_offset);
  }
  return goals_offset;
}

// ******* MAIN FUNCTION ********
int main(int argc, const char* argv[]) {
  State goal_state;
  std::cout << "Test Case 1:" << std::endl;
  goal_state.location.x = 0.0;
  goal_state.location.y = 0.0;
  goal_state.rotation.yaw = 0.0;
  State offset_goal;
  std::vector<State> expected;

  offset_goal.location.x = -1.83697e-16;
  offset_goal.location.y = -3;
  offset_goal.rotation.yaw = 0;
  expected.push_back(offset_goal);

  offset_goal.location.x = -1.22465e-16;
  offset_goal.location.y = -2;
  offset_goal.rotation.yaw = 0;
  expected.push_back(offset_goal);

  offset_goal.location.x = -6.12323e-17;
  offset_goal.location.y = -1;
  offset_goal.rotation.yaw = 0;
  expected.push_back(offset_goal);

  offset_goal.location.x = 0;
  offset_goal.location.y = 0;
  offset_goal.rotation.yaw = 0;
  expected.push_back(offset_goal);

  offset_goal.location.x = 6.12323e-17;
  offset_goal.location.y = 1;
  offset_goal.rotation.yaw = 0;
  expected.push_back(offset_goal);

  offset_goal.location.x = 1.22465e-16;
  offset_goal.location.y = 2;
  offset_goal.rotation.yaw = 0;
  expected.push_back(offset_goal);

  offset_goal.location.x = 1.83697e-16;
  offset_goal.location.y = 3;
  offset_goal.rotation.yaw = 0;
  expected.push_back(offset_goal);

  auto result = generate_offset_goals(goal_state);
  for (size_t i = 0; i < _num_goals; ++i) {
    std::cout << (std::fabs(result[i].location.x - expected[i].location.x) <
                              FLT_EPSILON &&
                          std::fabs(result[i].location.y -
                                    expected[i].location.y) < FLT_EPSILON &&
                          std::fabs(result[i].rotation.yaw -
                                    expected[i].rotation.yaw) < FLT_EPSILON
                      ? "PASS"
                      : "FAIL")
              << std::endl;
  }

  return 0;
}
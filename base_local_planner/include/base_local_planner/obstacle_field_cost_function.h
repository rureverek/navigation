/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023
 *  All rights reserved.
 *
 * Author: Piotr Mikulowski @rureverek
 *********************************************************************/

#ifndef OBSTACLE_FIELD_COST_FUNCTION_H
#define OBSTACLE_FIELD_COST_FUNCTION_H

#include <base_local_planner/trajectory_cost_function.h>

#include <costmap_2d/costmap_2d.h>
#define SCALE_FACTOR 0.01

namespace base_local_planner {

class ObstacleFieldCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
  ObstacleFieldCostFunction(costmap_2d::Costmap2D* costmap);

  ~ObstacleFieldCostFunction() {}

  bool prepare() {return true;}

  double scoreTrajectory(Trajectory &traj);

  unsigned char getCellCosts(unsigned int cx, unsigned int cy);

private:

  costmap_2d::Costmap2D* costmap_;

};

} /* namespace base_local_planner */
#endif /* OBSTACLE_FIELD_COST_FUNCTION_H */

/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023
 *  All rights reserved.
 *
 * Author: Piotr Mikulowski @rureverek
 *********************************************************************/

#include <base_local_planner/obstacle_field_cost_function.h>
#include <ros/console.h>


namespace base_local_planner {

ObstacleFieldCostFunction::ObstacleFieldCostFunction(costmap_2d::Costmap2D* costmap) : 
    costmap_(costmap) {}

unsigned char ObstacleFieldCostFunction::getCellCosts(unsigned int px, unsigned int py) {

  unsigned char cell_cost = costmap_->getCost(px,py);
  return cell_cost;
}

double ObstacleFieldCostFunction::scoreTrajectory(Trajectory &traj) {
  double cost = 0.0;
  double px, py, pth;
  unsigned int cell_x, cell_y;
  unsigned char cell_cost;

  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
    traj.getPoint(i, px, py, pth);

    //we won't allow trajectories that go off the map... shouldn't happen that often anyways
    if ( ! costmap_->worldToMap(px, py, cell_x, cell_y)) {
      //we're off the map
      ROS_WARN("Off Map %f, %f", px, py);
      return -4.0;
    }

    cell_cost = getCellCosts(cell_x, cell_y);

    cost += cell_cost*SCALE_FACTOR; //sum_average

  }
  return cost;
}

} /* namespace base_local_planner */

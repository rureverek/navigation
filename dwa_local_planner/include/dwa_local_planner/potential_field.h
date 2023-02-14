#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/trajectory.h>

namespace potential_field {

class PotentialFieldPlanner
{
private:
    /* data */
public:
    /**
     * @brief Construct the Potential Field Planner object
     * 
     */
    PotentialFieldPlanner(costmap_2d::Costmap2D local_costmap, const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    /**
     * @brief Destroy the Potential Field Planner object
     * 
     */
    ~PotentialFieldPlanner();

    /**
     * @brief Create Repulsive Field Map based on Inflated Obstacle Layer / Sensor Data
     * 
     * @param local_costmap 
     * @return costmap_2d::Costmap2D 
     */
    costmap_2d::Costmap2D InitialiseCostMap(costmap_2d::Costmap2D _local_costmap);

    /**
     * @brief Create Attractive Field Map based on local goal
     * 
     * @param orig_global_plan 
     * @return costmap_2d::Costmap2D 
     */
    costmap_2d::Costmap2D SetGoalMap(const std::vector<geometry_msgs::PoseStamped>& _orig_global_plan);

    /**
     * @brief Combine Repulsive and Attractive Fields in PotentialField
     * 
     * @param attract_field 
     * @param repulsive_field 
     * @return costmap_2d::Costmap2D 
     */
    costmap_2d::Costmap2D SetPotentialFieldMap(costmap_2d::Costmap2D attract_field, costmap_2d::Costmap2D repulsive_field);

    /**
     * @brief Calculate Cost of trajectory
     * 
     * @param potential_field 
     * @param traj 
     * @return double 
     */
    double PotentialFieldCostFunction(costmap_2d::Costmap2D potential_field, base_local_planner::Trajectory traj);
};

PotentialFieldPlanner::PotentialFieldPlanner(costmap_2d::Costmap2D local_costmap, const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
}

PotentialFieldPlanner::~PotentialFieldPlanner()
{
}

}
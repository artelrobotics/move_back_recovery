#ifndef MOVE_BACK_RECOVERY_H_
#define MOVE_BACK_RECOVERY_H_
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <base_local_planner/costmap_model.h>
#include <string>
#include <costmap_2d/costmap_2d.h>

namespace move_back_recovery{
  /**
   * @class ClearCostmapRecovery
   * @brief A recovery behavior that reverts the navigation stack's costmaps to the static map outside of a user-specified region.
   */
  class MoveBackClearRecovery : public nav_core::RecoveryBehavior {
    public:
      /**
       * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
       * @param
       * @return
       */
      MoveBackClearRecovery();

      /**
       * @brief  Initialization function for the ClearCostmapRecovery recovery behavior
       * @param tf A pointer to a transform listener
       * @param global_costmap A pointer to the global_costmap used by the navigation stack
       * @param local_costmap A pointer to the local_costmap used by the navigation stack
       */
      void initialize(std::string name, tf2_ros::Buffer*,
          costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap);

      /**
       * @brief  Run the ClearCostmapRecovery recovery behavior. Reverts the
       * costmap to the static map outside of a user-specified window and
       * clears unknown space around the robot.
       */
      void runBehavior();

     /**
     * @brief  Destructor for the rotate recovery behavior
     */
     ~MoveBackClearRecovery();

    private:
      costmap_2d::Costmap2DROS* local_costmap_;
      std::string name_;
      ros::Publisher vel_pub;
      bool initialized_;
      base_local_planner::CostmapModel* world_model_;
      double distance_backwards_;
      double frequency_;
      costmap_2d::Costmap2D* costmap2d_;
  };
};
#endif
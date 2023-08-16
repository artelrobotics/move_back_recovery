#include <move_back_recovery/move_back_recovery.h>
#include <pluginlib/class_list_macros.hpp>
#include <boost/pointer_cast.hpp>
#include <vector>
#include <geometry_msgs/Twist.h>

PLUGINLIB_EXPORT_CLASS(move_back_recovery::MoveBackClearRecovery, nav_core::RecoveryBehavior)

namespace move_back_recovery
{
MoveBackClearRecovery::MoveBackClearRecovery(): local_costmap_(NULL), initialized_(false), world_model_(NULL)
{
}

void MoveBackClearRecovery::initialize(std::string name, tf2_ros::Buffer*,
    costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    local_costmap_ = local_costmap;

    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);
    private_nh.param("distance_backwards", distance_backwards_, 0.25);
    private_nh.param("frequency", frequency_, 20.0);
    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());
    costmap2d_ = local_costmap_->getCostmap();
    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}
MoveBackClearRecovery::~MoveBackClearRecovery()
{
    delete world_model_;
}

void MoveBackClearRecovery::runBehavior()
{
    if (!initialized_)
    {
        ROS_ERROR("This object must be initialized before runBehavior is called");
        return;
    }

    if (local_costmap_ == NULL)
    {
        ROS_ERROR("The costmap passed to the MoveBackClearRecovery object cannot be NULL. Doing nothing.");
        return;
    }
    ROS_WARN("MoveBackClearRecovery started.");
    ros::Rate r(frequency_);
    ros::NodeHandle nh;
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    geometry_msgs::PoseStamped global_pose;
    std::vector<geometry_msgs::Point> oriented_footprint;
    local_costmap_->getRobotPose(global_pose);
    double current_x = global_pose.pose.position.x;
    double current_y = global_pose.pose.position.y;
    double start_x = current_x;
    double start_y = current_y;
    bool got_dist = false;

    while (nh.ok() && !got_dist)
        {
            local_costmap_->getRobotPose(global_pose);
            current_x = global_pose.pose.position.x;
            current_y = global_pose.pose.position.y;
            double dx = current_x - start_x;
            double dy = current_y - start_y;
            double moved_dist = std::sqrt(dx * dx + dy * dy);
            ROS_WARN("MoveBackClearRecovery: moved_dist is %.2f", moved_dist);
            if (moved_dist >= distance_backwards_)
            {
                got_dist = true;
            }
            else
            {
                local_costmap_->getOrientedFootprint(oriented_footprint);
                unsigned int cell1_x, cell1_y, cell2_x, cell2_y;
                costmap2d_->worldToMap(oriented_footprint[0].x, oriented_footprint[0].y, cell1_x, cell1_y);
                costmap2d_->worldToMap(oriented_footprint[1].x, oriented_footprint[1].y, cell2_x, cell2_y);
                double line_cost = world_model_->lineCost(cell1_x, cell1_y, cell2_x, cell2_y);
                if (line_cost < 0)
                {
                    ROS_ERROR("MoveBackClearRecovery can't move back because there is a potential collision. Cost: %.2f", line_cost);
                    return;
                }
                else if (line_cost > 200)
                {
                    ROS_ERROR("MoveBackClearRecovery can't move back because there is a potential collision. Cost: %.2f", line_cost);
                    return;
                }

                else
                {
                    ROS_WARN("MoveBackClearRecovery: line cost is %.2f", line_cost);
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = -0.1;
                    cmd_vel.angular.z = 0.0;
                    vel_pub.publish(cmd_vel);
                    r.sleep();
                    local_costmap_->updateMap();
                }

            }
        }
}
};  // namespace move_back_recovery
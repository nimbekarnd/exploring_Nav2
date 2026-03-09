#pragma once

#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_custom_costmap_plugin
{

    class CustomInflationLayer : public nav2_costmap_2d::Layer
    {
    public:
        // Constructor - to initialize members to safe defaults
        CustomInflationLayer();

        // Destructor - Setting it to default tells compiler to 
        // genetare it automatially as its virtual in the base class
        ~CustomInflationLayer() override = default;

        // To read ROS params 
        void onInitialize() override;

        void reset() override {}

        // To send custom bounds to Nav2 
        void updateBounds(
            double robot_x, double robot_y, double robot_yaw,
            double *min_x, double *min_y,
            double *max_x, double *max_y) override;

        // To set custom cost for the grid
        void updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                         int min_i, int min_j, int max_i, int max_j) override;

        // custom check to verify if the inflation can be cleared
        bool isClearable() override { return false; }

    private:
        // Member variable to track the range of obstacle costs
        double inflation_radius_;

        // Member variable to track scalinf factor
        double cost_scaling_factor_;

        // Member variable to track if cost can be inflated into unmapped area
        bool inflate_unknown_;

        // class logger
        rclcpp::Logger logger_;
    };
} // namespace nav2_custom_costmap_plugin
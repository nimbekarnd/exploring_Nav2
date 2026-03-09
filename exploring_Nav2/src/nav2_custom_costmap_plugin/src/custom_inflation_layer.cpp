#include "nav2_custom_costmap_plugin/custom_inflation_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <cmath>
#include <algorithm>

PLUGINLIB_EXPORT_CLASS(
    nav2_custom_costmap_plugin::CustomInflationLayer,
    nav2_costmap_2d::Layer)

namespace nav2_custom_costmap_plugin
{
    CustomInflationLayer::CustomInflationLayer()
        : inflation_radius_(0.55), cost_scaling_factor_(10.0), inflate_unknown_(false), logger_(rclcpp::get_logger("CustomInflationLayer"))
    {
    }

    void CustomInflationLayer::onInitialize()
    {
        auto node = node_.lock();
        if (!node)
        {
            throw std::runtime_error("CustomInflationLayer: node handle expired");
        }

        declareParameter("inflation_radius", rclcpp::ParameterValue(0.55));
        declareParameter("cost_scaling_factor", rclcpp::ParameterValue(10.0));
        declareParameter("inflate_unknown", rclcpp::ParameterValue(false));

        node->get_parameter(name_ + ".inflation_radius", inflation_radius_);
        node->get_parameter(name_ + ".cost_scaling_factor", cost_scaling_factor_);
        node->get_parameter(name_ + ".inflate_unknown", inflate_unknown_);

        RCLCPP_INFO(
            logger_,
            "CustomInflationLayer - radius: %.2fm, scaling factor: %.1f",
            inflation_radius_, cost_scaling_factor_);

        current_ = true;
    }

    void CustomInflationLayer::updateBounds(
        double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
        double *min_x, double *min_y,
        double *max_x, double *max_y)
    {
        *min_x -= inflation_radius_;
        *min_y -= inflation_radius_;
        *max_x += inflation_radius_;
        *max_y += inflation_radius_;
    }

    void CustomInflationLayer::updateCosts(
        nav2_costmap_2d::Costmap2D &master_grid,
        int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabled_)
        {
            return;
        }

        const double resolution = master_grid.getResolution();
        const int cells = static_cast<int>(std::ceil(inflation_radius_ / resolution));

        for (int j = min_j; j < max_j; ++j)
        {
            for (int i = min_i; i < max_i; ++i)
            {

                if (master_grid.getCost(i, j) != nav2_costmap_2d::LETHAL_OBSTACLE)
                {
                    continue;
                }

                for (int dj = -cells; dj <= cells; ++dj)
                {
                    for (int di = -cells; di <= cells; ++di)
                    {
                        const int ni = i + di;
                        const int nj = j + dj;

                        if (ni < min_i || ni >= max_i || nj < min_j || nj >= max_j)
                        {
                            continue;
                        }

                        const unsigned char neighbour = master_grid.getCost(ni, nj);
                        if (neighbour == nav2_costmap_2d::LETHAL_OBSTACLE)
                        {
                            continue;
                        }
                        if (!inflate_unknown_ && neighbour == nav2_costmap_2d::NO_INFORMATION)
                        {
                            continue;
                        }

                        const double dist = std::hypot(di, dj) * resolution;
                        if (dist > inflation_radius_)
                        {
                            continue;
                        }

                        const double factor = 252.0 * std::exp(-cost_scaling_factor_ * dist);
                        const unsigned char new_cost = static_cast<unsigned char>(
                            std::clamp(static_cast<int>(factor), 0, 252));

                        if (new_cost > neighbour)
                        {
                            master_grid.setCost(ni, nj, new_cost);
                        }
                    }
                }
            }
        }
    }
}

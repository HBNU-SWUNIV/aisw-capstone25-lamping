#pragma once

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace virtual_layer
{

	class VirtualLayer : public nav2_costmap_2d::CostmapLayer
	{
	public:
		VirtualLayer();

		void onInitialize() override;

		void updateBounds(
			double robot_x, double robot_y, double robot_yaw,
			double *min_x, double *min_y, double *max_x, double *max_y) override;

		void updateCosts(
			nav2_costmap_2d::Costmap2D &master_grid,
			int min_i, int min_j, int max_i, int max_j) override;

		void reset() override; // 🔧 reset() 함수 구현 추가

		bool isClearable() override { return false; }

	private:
		void occupancyCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

		nav_msgs::msg::OccupancyGrid::SharedPtr latest_grid_;
		rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
	};

} // namespace virtual_layer

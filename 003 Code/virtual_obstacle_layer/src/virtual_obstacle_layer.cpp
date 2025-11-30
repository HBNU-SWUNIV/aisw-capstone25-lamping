#include "virtual_obstacle_layer/virtual_obstacle_layer.hpp"

namespace virtual_layer
{

	VirtualLayer::VirtualLayer() {}

	void VirtualLayer::onInitialize()
	{
		auto node = node_.lock();
		default_value_ = nav2_costmap_2d::FREE_SPACE;

		current_ = true;
		matchSize();

		// 🔧 transient_local QoS로 OccupancyGrid 구독
		grid_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
			"virtual_obstacles",
			rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
			std::bind(&VirtualLayer::occupancyCallback, this, std::placeholders::_1));

		RCLCPP_INFO(node->get_logger(), "🧱 VirtualObstacleLayer initialized.");
	}

	void VirtualLayer::occupancyCallback(
		const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
	{
		latest_grid_ = msg;
		RCLCPP_DEBUG(node_.lock()->get_logger(), "Received OccupancyGrid update");
	}

	void VirtualLayer::updateBounds(
		double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
		double *min_x, double *min_y, double *max_x, double *max_y)
	{
		if (!latest_grid_)
			return;

		*min_x = latest_grid_->info.origin.position.x;
		*min_y = latest_grid_->info.origin.position.y;
		*max_x = *min_x + latest_grid_->info.width * latest_grid_->info.resolution;
		*max_y = *min_y + latest_grid_->info.height * latest_grid_->info.resolution;

		current_ = true;
	}

	void VirtualLayer::updateCosts(
		nav2_costmap_2d::Costmap2D &master_grid,
		int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
	{
		if (!latest_grid_)
			return;

		unsigned int width = latest_grid_->info.width;
		unsigned int height = latest_grid_->info.height;
		double resolution = latest_grid_->info.resolution;
		double origin_x = latest_grid_->info.origin.position.x;
		double origin_y = latest_grid_->info.origin.position.y;

		RCLCPP_INFO(node_.lock()->get_logger(), "🧩 updateCosts called");

		for (unsigned int i = 0; i < width; i++)
		{
			for (unsigned int j = 0; j < height; j++)
			{
				int idx = j * width + i;
				if (latest_grid_->data[idx] > 0)
				{
					double wx = origin_x + i * resolution;
					double wy = origin_y + j * resolution;

					unsigned int mx, my;
					if (master_grid.worldToMap(wx, wy, mx, my))
					{
						RCLCPP_INFO(
							node_.lock()->get_logger(),
							"🧱 Setting virtual wall cost at (%.2f, %.2f) → costmap (%d, %d)",
							wx, wy, mx, my);

						master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
					}
					else
					{
						RCLCPP_WARN(
							node_.lock()->get_logger(),
							"⚠️ worldToMap failed for (%.2f, %.2f)", wx, wy);
					}
				}
			}
		}
	}

	void VirtualLayer::reset()
	{
		RCLCPP_INFO(node_.lock()->get_logger(), "🔄 VirtualObstacleLayer reset");
	}

} // namespace virtual_layer

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(virtual_layer::VirtualLayer, nav2_costmap_2d::Layer)

// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    pc_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "point_cloud", 10, std::bind(&MinimalSubscriber::point_cloud_callback, this, _1));

    map_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10, std::bind(&MinimalSubscriber::map_callback, this, _1));
  }

private:

  void topic_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }


  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received Point Cloud Data");
    // Process the point cloud data here
    // Iterating through the points and print their coordinates.
    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

    for (size_t i = 0; i < msg->width; ++i, ++iter_x, ++iter_y, ++iter_z) {
      RCLCPP_INFO(this->get_logger(), "Point %zu: x=%f, y=%f, z=%f", i, *iter_x, *iter_y, *iter_z);
    }
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received Occupancy Grid Map");
    // Process the occupancy grid map here
    // The map resolution and size
    RCLCPP_INFO(this->get_logger(), "Map Resolution: %f", msg->info.resolution);
    RCLCPP_INFO(this->get_logger(), "Map Size: %zu x %zu", msg->info.width, msg->info.height);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscriber;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <vector>
#include <random>

class MapPublisher : public rclcpp::Node {
public:
    MapPublisher() : Node("map_publisher") {
        map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
        timer = this->create_wall_timer(std::chrono::seconds(1), 
                                         std::bind(&MapPublisher::publishMap, this));
        generateMap();
    }

private:
    void generateMap() {
        map.header.frame_id = "map";
        map.info.resolution = 0.05;  // 5 cm per cell
        map.info.width = 100;        // 100x100 grid
        map.info.height = 100;
        map.info.origin.position.x = -2.5; // Origin at (-2.5, -2.5)
        map.info.origin.position.y = -2.5;
        map.info.origin.orientation.w = 1.0;

        // Initialize map with -1 (unknown), 0 (free), and 100 (occupied) randomly
        map.data.resize(map.info.width * map.info.height, -1);

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(0, 100);

        for (size_t i = 0; i < map.data.size(); ++i) {
            int rand_val = dis(gen);
            if (rand_val < 70)
                map.data[i] = 0;  // Free space
            else if (rand_val < 90)
                map.data[i] = 100;  // Occupied space
            else
                map.data[i] = -1; // Unknown space
        }
    }

    void publishMap() {
        map.header.stamp = this->now();
        map_publisher->publish(map);
        RCLCPP_INFO(this->get_logger(), "Published Occupancy Grid Map");
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;
    rclcpp::TimerBase::SharedPtr timer;
    nav_msgs::msg::OccupancyGrid map;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapPublisher>());
    rclcpp::shutdown();
    return 0;
}

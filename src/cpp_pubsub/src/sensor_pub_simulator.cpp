#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <random>

class PointCloudPublisher : public rclcpp::Node {
public:
    PointCloudPublisher() : Node("point_cloud_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), 
                                         std::bind(&PointCloudPublisher::publishPointCloud, this));
    }

private:
    void publishPointCloud() {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.height = 1;
        msg.width = 100000;  // Large dataset
        msg.is_dense = false;
        msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");

        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(0.0, 10.0);

        for (size_t i = 0; i < msg.width; ++i, ++iter_x, ++iter_y, ++iter_z) {
            *iter_x = dis(gen);
            *iter_y = dis(gen);
            *iter_z = dis(gen);
        }

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published Point Cloud Data");
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudPublisher>());
    rclcpp::shutdown();
    return 0;
}

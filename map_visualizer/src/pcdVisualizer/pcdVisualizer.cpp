#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

using namespace std::chrono_literals;

class PcdtoPointcloud2 : public rclcpp::Node
{
public:
  PcdtoPointcloud2()
      : Node("pcd_visualizer")
  {
    // Declare the parameter with default value
    this->declare_parameter("map_path", "/home/genis/Downloads/cloudSurf_none.pcd");

    // Get the parameter value
    std::string map_path = this->get_parameter("map_path").as_string();

    RCLCPP_INFO(this->get_logger(), "Loading PCD file from: %s", map_path.c_str());

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_path, *cloud_) == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file from: %s", map_path.c_str());
      return;
    }

    pcl::toROSMsg(*cloud_.get(), ros_pc2_);
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcd", 10);
    timer_ = this->create_wall_timer(5000ms, std::bind(&PcdtoPointcloud2::timer_callback, this));
    std::cout << "size: " << ros_pc2_.width * ros_pc2_.height << std::endl;
    ros_pc2_.header.frame_id = "map";
  }

private:
  void timer_callback()
  {
    if (publisher_->get_subscription_count() > 0 && m_first)
    {
      std::cout << "get_subscription_count:    " << publisher_->get_subscription_count() << std::endl;
      ros_pc2_.header.stamp = this->get_clock()->now();
      publisher_->publish(ros_pc2_);
      m_first = false;
    }
  }
  bool m_first{true};
  sensor_msgs::msg::PointCloud2 ros_pc2_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdtoPointcloud2>());
  rclcpp::shutdown();
  return 0;
}
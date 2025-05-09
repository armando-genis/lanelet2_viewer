
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// lanelet libraries
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_projection/LocalCartesian.h>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <polygon_msgs/msg/polygon2_d_stamped.hpp>
#include <polygon_msgs/msg/polygon2_d_collection.hpp>

#include <lanelet2_core/geometry/Point.h>

#include <lanelet2_core/primitives/Lanelet.h>

#include <lanelet2_traffic_rules/TrafficRules.h>
#include <boost/optional/optional_io.hpp>

// custon messages
// RoadElements
#include "traffic_information_msgs/msg/road_elements.hpp"
// for RoadElementsCollection
#include "traffic_information_msgs/msg/road_elements_collection.hpp"

// we want assert statements to work in release mode
#undef NDEBUG

using namespace std::chrono_literals;
using namespace lanelet;

class OsmVisualizer : public rclcpp::Node
{
public:
  OsmVisualizer();

private:
  void timer_callback();
  bool readParameters();
  void writeToFile(const std_msgs::msg::Float64MultiArray &multi_array);
  void fill_marker(lanelet::LaneletMapPtr &t_map);
  void fill_array(lanelet::LaneletMapPtr &t_map);
  void fill_array_with_left_right(lanelet::LaneletMapPtr &t_map);
  double getDistance(const lanelet::ConstLanelet &ll, size_t i);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr array_publisher_;
  rclcpp::Publisher<polygon_msgs::msg::Polygon2DCollection>::SharedPtr polygon_publisher_;
  rclcpp::Publisher<traffic_information_msgs::msg::RoadElementsCollection>::SharedPtr road_elements_publisher_;

  std_msgs::msg::Float64MultiArray m_array;
  visualization_msgs::msg::MarkerArray m_marker_array;

  // msgs for the crosswalks and road information
  polygon_msgs::msg::Polygon2DCollection crosswalk_polygons;
  traffic_information_msgs::msg::RoadElementsCollection road_elements;

  // colors for the terminal
  std::string green = "\033[1;32m";
  std::string red = "\033[1;31m";
  std::string blue = "\033[1;34m";
  std::string yellow = "\033[1;33m";
  std::string purple = "\033[1;35m";
  std::string reset = "\033[0m";

  bool m_first{true};
  bool m_second{true};
  bool m_third{true};

  // params
  std::string map_path_;
  bool enable_inc_path_points_;
  double interval_;

  // #########################
  // Stop signs and traffic lights
  // #########################

  void add_stop_signs(const std::vector<std::array<double, 3>> &positions, float scale_factor = 1.0);
  void add_traffic_lights(const std::vector<std::array<double, 3>> &positions, float scale_factor = 1.0);
  void create_circle(polygon_msgs::msg::Polygon2D &polygon, double center_x, double center_y, double radius, int num_segments);
  void add_speed_limit_signs(const std::vector<std::array<double, 3>> &positions, const std::vector<int> &speed_limits, float scale_factor = 1.0);

  // Publishers for stop signs and traffic lights
  rclcpp::Publisher<polygon_msgs::msg::Polygon2DCollection>::SharedPtr stop_sign_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr text_marker_publisher_;
  rclcpp::Publisher<polygon_msgs::msg::Polygon2DCollection>::SharedPtr traffic_light_publisher_;
  rclcpp::Publisher<polygon_msgs::msg::Polygon2DCollection>::SharedPtr speed_limit_publisher_;

  // Message collections
  polygon_msgs::msg::Polygon2DCollection stop_sign_polygons;
  polygon_msgs::msg::Polygon2DCollection traffic_light_polygons;
  visualization_msgs::msg::MarkerArray text_markers;
  polygon_msgs::msg::Polygon2DCollection speed_limit_polygons;

  // Flags for publishers
  bool m_stop_signs_published{false};
  bool m_traffic_lights_published{false};
  bool m_speed_limit_published{false};
};
// Based on the code made by Mücahit Ayhan but fully changed for needs and requirements to visualize the map. Also added some new features like the crosswalks, road elements, stop signs, traffic lights, and speed limit signs. Also add interpolation in the fill_array_with_left_right function for better data processing.
#include "../include/osmVisualizer/osmVisualizer.hpp"

OsmVisualizer::OsmVisualizer() : Node("OsmVisualizer")
{
  this->declare_parameter("map_path", "/home/atakan/Downloads/Town10.osm");
  this->declare_parameter("enable_inc_path_points", true);
  this->declare_parameter("interval", 2.0);
  if (!readParameters())
    rclcpp::shutdown();

  publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/hd_map", 10);
  array_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/array", 10);
  timer_ = this->create_wall_timer(200ms, std::bind(&OsmVisualizer::timer_callback, this));

  polygon_publisher_ = this->create_publisher<polygon_msgs::msg::Polygon2DCollection>("/crosswalk_polygons", 10);
  road_elements_publisher_ = this->create_publisher<traffic_information_msgs::msg::RoadElementsCollection>("/road_elements", 10);

  // Add publishers for stop signs and traffic lights
  stop_sign_publisher_ = this->create_publisher<polygon_msgs::msg::Polygon2DCollection>("/stop_sign_polygons", 10);
  traffic_light_publisher_ = this->create_publisher<polygon_msgs::msg::Polygon2DCollection>("/traffic_light_polygons", 10);
  text_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/sign_texts", 10);
  speed_limit_publisher_ = this->create_publisher<polygon_msgs::msg::Polygon2DCollection>("/speed_limit_polygons", 10);

  lanelet::Origin origin({49, 8.4});
  lanelet::projection::LocalCartesianProjector projector(origin);
  lanelet::LaneletMapPtr map = lanelet::load(map_path_, projector);
  // LaneletMapPtr map = load(map_path_, projection::UtmProjector(Origin({49, 8.4})));

  for (auto &point : map->pointLayer)
  {
    point.x() = point.attribute("local_x").asDouble().value();
    point.y() = point.attribute("local_y").asDouble().value();
  }

  fill_marker(map);
  fill_array_with_left_right(map);

  // Example positions for stop signs - adjust based on your map
  std::vector<std::array<double, 3>> stop_sign_positions = {
      {9.4, -5.2, 0.0}, // x, y, z
      {-5.0, 15.0, 0.5},
      {20.0, -8.0, 0.5},
      {-15.0, -12.0, 0.5}};

  // Example positions for traffic lights - adjust based on your map
  std::vector<std::array<double, 3>> traffic_light_positions = {
      {11.2, 1.5, 0.0}, // x, y, z
      {-10.0, 20.0, 1.0},
      {25.0, -12.0, 1.0},
      {-20.0, -15.0, 1.0}};

  // Speed limit signs with corresponding limits
  std::vector<std::array<double, 3>> speed_limit_positions = {
      {3.9, 1.4, 0.0},
      {-15.0, 5.0, 0.5},
      {10.0, -20.0, 0.5}};
  std::vector<int> speed_limits = {30, 50, 70};

  // Get scale parameters
  float stop_sign_scale = 50.0;
  float traffic_light_scale = 50.0;
  float speed_limit_scale = 50.0;

  // Calculate actual scale factors from percentage
  float stop_sign_scale_factor = 1.0 + (stop_sign_scale / 100.0);
  float traffic_light_scale_factor = 1.0 + (traffic_light_scale / 100.0);
  float speed_limit_scale_factor = 1.0 + (speed_limit_scale / 100.0);

  std::cout << yellow << "-----> Stop sign scale: " << stop_sign_scale << "% (factor: "
            << stop_sign_scale_factor << ")" << reset << std::endl;
  std::cout << yellow << "-----> Traffic light scale: " << traffic_light_scale << "% (factor: "
            << traffic_light_scale_factor << ")" << reset << std::endl;

  // Add stop signs and traffic lights to the map with scaling
  add_stop_signs(stop_sign_positions, stop_sign_scale_factor);
  add_traffic_lights(traffic_light_positions, traffic_light_scale_factor);
  add_speed_limit_signs(speed_limit_positions, speed_limits, speed_limit_scale_factor);

  RCLCPP_INFO(this->get_logger(), "\033[1;32m----> OsmVisualizer_node initialized.\033[0m");
}

bool OsmVisualizer::readParameters()
{
  if (!this->get_parameter("map_path", map_path_))
  {
    std::cout << "Failed to read parameter 'map_path' " << std::endl;
    return false;
  }
  if (!this->get_parameter("enable_inc_path_points", enable_inc_path_points_))
  {
    std::cout << "Failed to read parameter 'interval' to increase the path points" << std::endl;
    return false;
  }
  if (!this->get_parameter("interval", interval_))
  {
    std::cout << "Failed to read parameter 'interval' to increase the path points" << std::endl;
    return false;
  }
  return true;
}

void OsmVisualizer::timer_callback()
{
  // publish until one subcribre that is the rviz2 for vizualisation and publish only once
  if (publisher_->get_subscription_count() > 0 && m_first)
  {
    publisher_->publish(m_marker_array);
    m_first = false;
  }
  // publish until one subcribre that is the ocupanccy grid code and publish only once
  if (array_publisher_->get_subscription_count() > 0 && m_second)
  {
    array_publisher_->publish(m_array);
    m_second = false;
  }

  if (!crosswalk_polygons.polygons.empty())
  {
    // publish until one subcribre that is the rviz2 for vizualisation and publish only once
    if (polygon_publisher_->get_subscription_count() > 0 && m_third)
    {
      polygon_publisher_->publish(crosswalk_polygons);
      m_third = false;
    }
    // publish until one subcribre
    if (road_elements_publisher_->get_subscription_count() > 0)
    {
      road_elements_publisher_->publish(road_elements);
    }
  }

  // Publish stop signs
  if (!stop_sign_polygons.polygons.empty())
  {
    if (stop_sign_publisher_->get_subscription_count() > 0 && !m_stop_signs_published)
    {
      stop_sign_publisher_->publish(stop_sign_polygons);
      m_stop_signs_published = true;
    }
  }

  if (text_marker_publisher_->get_subscription_count() > 0 && !text_markers.markers.empty())
  {
    text_marker_publisher_->publish(text_markers);
  }

  // Publish traffic lights
  if (!traffic_light_polygons.polygons.empty())
  {
    if (traffic_light_publisher_->get_subscription_count() > 0 && !m_traffic_lights_published)
    {
      traffic_light_publisher_->publish(traffic_light_polygons);
      m_traffic_lights_published = true;
    }
  }

  // Publish speed limit signs
  if (!speed_limit_polygons.polygons.empty())
  {
    if (speed_limit_publisher_->get_subscription_count() > 0 && !m_speed_limit_published)
    {
      speed_limit_publisher_->publish(speed_limit_polygons);
      m_speed_limit_published = true;
    }
  }
}

void OsmVisualizer::fill_array(lanelet::LaneletMapPtr &t_map)
{
  m_array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  m_array.layout.dim[0].label = "rows";
  m_array.layout.dim[0].size = 100000;
  m_array.layout.dim[0].stride = 100000 * 2;
  m_array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  m_array.layout.dim[1].label = "cols";
  m_array.layout.dim[1].size = 2;
  m_array.layout.dim[1].stride = 2;

  for (const auto &ll : t_map->laneletLayer)
  {
    for (size_t i = 0; i < ll.centerline2d().size() - 1; i++)
    {
      if (getDistance(ll, i) > 2 && enable_inc_path_points_)
      {
        double dist = getDistance(ll, i);
        double interval = 1;
        int num_points = dist / interval;

        for (int k = 0; k < num_points; k++)
        {
          m_array.data.push_back(((ll.centerline2d()[i + 1].x() - ll.centerline2d()[i].x()) / num_points) * k + ll.centerline2d()[i].x());
          m_array.data.push_back(((ll.centerline2d()[i + 1].y() - ll.centerline2d()[i].y()) / num_points) * k + ll.centerline2d()[i].y());
        }
      }
      else
      {
        m_array.data.push_back(ll.centerline2d()[i].x());
        m_array.data.push_back(ll.centerline2d()[i].y());
      }
    }
  }
}

void OsmVisualizer::writeToFile(const std_msgs::msg::Float64MultiArray &multi_array)
{
  std::ofstream file("data.txt");
  if (file.is_open())
  {
    for (size_t i = 0; i < multi_array.data.size(); ++i)
    {
      file << multi_array.data[i] << ",";
      if ((i + 1) % (multi_array.layout.dim[0].size) == 0)
        file << "\n";
      if ((i + 1) % (multi_array.layout.dim[1].size) == 0)
        file << "\n";
    }
    file.close();
  }
}

void OsmVisualizer::fill_array_with_left_right(lanelet::LaneletMapPtr &t_map)
{
  m_array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  m_array.layout.dim[0].label = "rows";
  m_array.layout.dim[0].size = t_map->laneletLayer.size();
  m_array.layout.dim[0].stride = t_map->laneletLayer.size() * 4;
  m_array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  m_array.layout.dim[1].label = "cols";
  m_array.layout.dim[1].size = 4;
  m_array.layout.dim[1].stride = 4;

  // Define an interpolation interval (distance between each interpolated point)
  double interval = 0.5; // Adjust this value based on your map resolution and needs

  for (const auto &ll : t_map->laneletLayer)
  {
    std::vector<lanelet::ConstLineString3d> bounds;
    bounds.push_back(ll.leftBound());
    bounds.push_back(ll.rightBound());

    // Check if the lanelet has the subtype 'crosswalk' and skip it
    if (ll.hasAttribute(lanelet::AttributeName::Subtype) &&
        ll.attribute(lanelet::AttributeName::Subtype).value() != lanelet::AttributeValueString::Crosswalk)
    {
      // Check if the lanelet has exactly four points on each boundary (rectangular structure)
      if (bounds[0].size() == 2 && bounds[1].size() == 2)
      {

        // std::cout << blue << "Rects  lanelet ID: " << ll.id() << reset << std::endl;

        size_t left_size = bounds[0].size();
        size_t right_size = bounds[1].size();

        // Interpolate between each pair of consecutive points
        for (size_t segment = 0; segment < left_size - 1; ++segment)
        {
          // Get start and end points of the current segment for left and right boundaries
          auto left_start = bounds[0][segment];
          auto left_end = bounds[0][segment + 1];
          auto right_start = bounds[1][segment];
          auto right_end = bounds[1][segment + 1];

          // Calculate the distance for the left and right segments
          double left_dist = std::sqrt(std::pow(left_end.x() - left_start.x(), 2) +
                                       std::pow(left_end.y() - left_start.y(), 2));
          double right_dist = std::sqrt(std::pow(right_end.x() - right_start.x(), 2) +
                                        std::pow(right_end.y() - right_start.y(), 2));

          // Determine the number of interpolation points based on the shortest segment
          int num_points = static_cast<int>(std::min(left_dist, right_dist) / interval);

          for (int i = 0; i <= num_points; ++i)
          {
            // Interpolate for the left boundary
            double t = static_cast<double>(i) / num_points;
            double left_x = left_start.x() + t * (left_end.x() - left_start.x());
            double left_y = left_start.y() + t * (left_end.y() - left_start.y());

            // Interpolate for the right boundary
            double right_x = right_start.x() + t * (right_end.x() - right_start.x());
            double right_y = right_start.y() + t * (right_end.y() - right_start.y());

            // Add the interpolated points to m_array
            m_array.data.push_back(left_x);
            m_array.data.push_back(left_y);
            m_array.data.push_back(right_x);
            m_array.data.push_back(right_y);
          }
        }
      }
      else
      {
        // If there are more than four points, just add the existing points without interpolation
        // std::cout << green << "Not rect lanelet ID: " << ll.id() << reset << std::endl;

        size_t size = std::min(bounds[0].size(), bounds[1].size());
        for (size_t i = 0; i < size; i++)
        {
          m_array.data.push_back(bounds[0][i].x());
          m_array.data.push_back(bounds[0][i].y());
          m_array.data.push_back(bounds[1][i].x());
          m_array.data.push_back(bounds[1][i].y());
        }
      }
    }
  }
}

double OsmVisualizer::getDistance(const lanelet::ConstLanelet &ll, size_t i)
{
  return std::sqrt(std::pow(ll.centerline2d()[i].x() - ll.centerline2d()[i + 1].x(), 2) + std::pow(ll.centerline2d()[i].y() - ll.centerline2d()[i + 1].y(), 2));
}

void OsmVisualizer::fill_marker(lanelet::LaneletMapPtr &t_map)
{

  size_t i = 0;
  int crosswalk_count = 0;             // Counter for crosswalk subtype
  int road_element_count = 0;          // Counter for road elements
  crosswalk_polygons.polygons.clear(); // Clear the crosswalk polygons
  road_elements.polygons.clear();      // Clear the road elements

  crosswalk_polygons.header.stamp = rclcpp::Clock{}.now();
  crosswalk_polygons.header.frame_id = "map";

  road_elements.header.stamp = rclcpp::Clock{}.now();
  road_elements.header.frame_id = "map";

  std_msgs::msg::ColorRGBA color;
  color.r = 0.6; // Red color
  color.g = 0.6; // No green
  color.b = 0.6; // No blue
  color.a = 0.5; // Full opacity

  crosswalk_polygons.colors.clear();
  crosswalk_polygons.colors.push_back(color);

  double base_polygon_z_offset = 0.0;   // Z-offset for the base crosswalk polygon
  double stripe_polygon_z_offset = 0.1; // Z-offset for the zebra crossing stripes

  // Iterate over the lanelets in the map
  for (const auto &ll : t_map->laneletLayer)
  {
    std::vector<lanelet::ConstLineString3d> bounds;
    bounds.push_back(ll.leftBound());
    bounds.push_back(ll.rightBound());

    // Check if the lanelet has the subtype 'crosswalk'
    if (ll.hasAttribute(lanelet::AttributeName::Subtype) &&
        ll.attribute(lanelet::AttributeName::Subtype).value() == lanelet::AttributeValueString::Crosswalk)
    {

      // cout the id of the crosswalk
      std::cout << "Crosswalk id: " << ll.id() << std::endl;
      polygon_msgs::msg::Polygon2D base_polygon;
      traffic_information_msgs::msg::RoadElements crosswalks_element;

      crosswalk_count++;   // Increment the crosswalk counter
      int num_stripes = 5; // Number of stripes in the zebra crossing

      double max_z = std::numeric_limits<double>::lowest();
      for (const auto &point : ll.leftBound())
      {
        if (point.z() > max_z)
        {
          max_z = point.z();
        }
      }

      // =================================================================================================
      // base polygon for road_elements_publisher
      // For the left bound
      for (const auto &point : ll.leftBound())
      {
        polygon_msgs::msg::Point2D p;
        p.x = point.x(); // Convert from lanelet point to polygon_msgs Point2D
        p.y = point.y();
        crosswalks_element.points.push_back(p); // Add to the polygon's points
      }

      // For the right bound
      const auto &right_bound = ll.rightBound();
      for (int i = right_bound.size() - 1; i >= 0; --i)
      {
        polygon_msgs::msg::Point2D p;
        p.x = right_bound[i].x(); // Convert from lanelet point to polygon_msgs Point2D
        p.y = right_bound[i].y();
        crosswalks_element.points.push_back(p); // Add to the polygon's points
      }

      crosswalks_element.points.push_back(crosswalks_element.points[0]);

      crosswalks_element.id = ll.id(); // Set the ID for the crosswal

      crosswalks_element.type = ll.attribute(lanelet::AttributeName::Subtype).value();

      road_elements.polygons.push_back(crosswalks_element);

      // std::cout << "crosswal type: " << ll.attribute(lanelet::AttributeName::Subtype).value() << std::endl;

      // =================================================================================================
      // stripe polygons

      // Calculate the total length of the left and right bounds
      double left_bound_length = 0.0;
      for (size_t i = 0; i < ll.leftBound().size() - 1; ++i)
      {
        left_bound_length += std::sqrt(
            std::pow(ll.leftBound()[i + 1].x() - ll.leftBound()[i].x(), 2) +
            std::pow(ll.leftBound()[i + 1].y() - ll.leftBound()[i].y(), 2) +
            std::pow(ll.leftBound()[i + 1].z() - ll.leftBound()[i].z(), 2));
      }

      // std::cout << "----->left_bound_length: " << left_bound_length << std::endl;

      if (left_bound_length > 5.0)
      {
        num_stripes += static_cast<int>((left_bound_length - 5.0) / 1.0) * 1;
      }

      // std::cout << "----->num_stripes: " << num_stripes << std::endl;

      double stripe_length = left_bound_length / (2 * num_stripes);

      // Generate zebra stripes
      for (int stripe_idx = 0; stripe_idx < num_stripes; ++stripe_idx)
      {
        polygon_msgs::msg::Polygon2D stripe_polygon;
        stripe_polygon.z_offset = max_z;

        // Calculate the start and end points for the current stripe on the left and right bounds
        double start_dist = stripe_idx * 2 * stripe_length;
        double end_dist = start_dist + stripe_length;

        // Add points for the stripe from the left bound
        double accumulated_length = 0.0;
        polygon_msgs::msg::Point2D start_left, end_left;
        bool start_left_set = false, end_left_set = false;

        for (size_t i = 0; i < ll.leftBound().size() - 1; ++i)
        {
          double segment_length = std::sqrt(
              std::pow(ll.leftBound()[i + 1].x() - ll.leftBound()[i].x(), 2) +
              std::pow(ll.leftBound()[i + 1].y() - ll.leftBound()[i].y(), 2));

          if (accumulated_length + segment_length > start_dist && !start_left_set)
          {
            double ratio = (start_dist - accumulated_length) / segment_length;
            start_left.x = ll.leftBound()[i].x() + ratio * (ll.leftBound()[i + 1].x() - ll.leftBound()[i].x());
            start_left.y = ll.leftBound()[i].y() + ratio * (ll.leftBound()[i + 1].y() - ll.leftBound()[i].y());
            start_left_set = true;
          }

          if (accumulated_length + segment_length > end_dist && !end_left_set)
          {
            double ratio = (end_dist - accumulated_length) / segment_length;
            end_left.x = ll.leftBound()[i].x() + ratio * (ll.leftBound()[i + 1].x() - ll.leftBound()[i].x());
            end_left.y = ll.leftBound()[i].y() + ratio * (ll.leftBound()[i + 1].y() - ll.leftBound()[i].y());
            end_left_set = true;
            break;
          }

          accumulated_length += segment_length;
        }

        if (start_left_set && end_left_set)
        {
          stripe_polygon.points.push_back(start_left);
          stripe_polygon.points.push_back(end_left);
        }

        // Add points for the stripe from the right bound
        accumulated_length = 0.0;
        polygon_msgs::msg::Point2D start_right, end_right;
        bool start_right_set = false, end_right_set = false;

        for (size_t i = 0; i < ll.rightBound().size() - 1; ++i)
        {
          double segment_length = std::sqrt(
              std::pow(ll.rightBound()[i + 1].x() - ll.rightBound()[i].x(), 2) +
              std::pow(ll.rightBound()[i + 1].y() - ll.rightBound()[i].y(), 2));

          if (accumulated_length + segment_length > start_dist && !start_right_set)
          {
            double ratio = (start_dist - accumulated_length) / segment_length;
            start_right.x = ll.rightBound()[i].x() + ratio * (ll.rightBound()[i + 1].x() - ll.rightBound()[i].x());
            start_right.y = ll.rightBound()[i].y() + ratio * (ll.rightBound()[i + 1].y() - ll.rightBound()[i].y());
            start_right_set = true;
          }

          if (accumulated_length + segment_length > end_dist && !end_right_set)
          {
            double ratio = (end_dist - accumulated_length) / segment_length;
            end_right.x = ll.rightBound()[i].x() + ratio * (ll.rightBound()[i + 1].x() - ll.rightBound()[i].x());
            end_right.y = ll.rightBound()[i].y() + ratio * (ll.rightBound()[i + 1].y() - ll.rightBound()[i].y());
            end_right_set = true;
            break;
          }

          accumulated_length += segment_length;
        }

        if (start_right_set && end_right_set)
        {
          stripe_polygon.points.push_back(end_right);
          stripe_polygon.points.push_back(start_right);
        }

        // Close the polygon by adding the first point again
        if (stripe_polygon.points.size() >= 4)
        {

          // Set color for the stripe
          std_msgs::msg::ColorRGBA stripe_color;
          stripe_color.r = 0.6;
          stripe_color.g = 0.6;
          stripe_color.b = 0.6;
          stripe_color.a = 0.8;

          crosswalk_polygons.colors.push_back(stripe_color);

          stripe_polygon.points.push_back(stripe_polygon.points[0]);
          crosswalk_polygons.polygons.push_back(stripe_polygon);
        }
      }
    }

    else
    {
      road_element_count++; // Increment the road element counter
      // For each bound (left and right), create a marker
      for (const auto &bound : bounds)
      {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock{}.now();
        marker.ns = "lanelet";
        marker.id = i++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;

        // Add the points to the marker
        for (const auto &point : bound)
        {
          geometry_msgs::msg::Point p;
          p.x = point.x();
          p.y = point.y();
          p.z = point.z();
          marker.points.push_back(p);
        }

        // Add the marker to the array
        m_marker_array.markers.push_back(marker);
      }
    }
  }
  std::cout << blue << "----> Number of crosswalk lanelets: " << crosswalk_count << reset << std::endl;
  std::cout << blue << "----> Number of road elements: " << road_element_count << reset << std::endl;
}

// ############################################################################################################
// Add stop signs and traffic lights to the map
// ############################################################################################################
void OsmVisualizer::add_stop_signs(const std::vector<std::array<double, 3>> &positions, float scale_factor)
{
  stop_sign_polygons.polygons.clear();
  stop_sign_polygons.header.stamp = rclcpp::Clock{}.now();
  stop_sign_polygons.header.frame_id = "map";

  // Clear existing text markers
  text_markers.markers.clear();

  // Set color for stop signs (red)
  std_msgs::msg::ColorRGBA stop_sign_color;
  stop_sign_color.r = 1.0;
  stop_sign_color.g = 0.0;
  stop_sign_color.b = 0.0;
  stop_sign_color.a = 0.9;
  stop_sign_polygons.colors.push_back(stop_sign_color);

  // Set border color for stop signs (white)
  std_msgs::msg::ColorRGBA stop_sign_border_color;
  stop_sign_border_color.r = 1.0;
  stop_sign_border_color.g = 1.0;
  stop_sign_border_color.b = 1.0;
  stop_sign_border_color.a = 1.0;
  stop_sign_polygons.colors.push_back(stop_sign_border_color);

  int stop_sign_id = 1000;
  int marker_id = 0;

  for (const auto &position : positions)
  {
    // Create octagonal shape for stop sign
    polygon_msgs::msg::Polygon2D stop_sign;
    stop_sign.z_offset = position[2]; // Use provided z coordinate

    // Parameters for the stop sign
    double base_size = 0.8;                 // Base size of the stop sign
    double size = base_size * scale_factor; // Apply scaling
    double radius = size / 2.0;
    int num_sides = 8; // Octagon

    // Create the octagonal shape
    for (int i = 0; i < num_sides; ++i)
    {
      double angle = 2.0 * M_PI * i / num_sides + M_PI / 8.0; // Rotate to align flat edge at bottom
      polygon_msgs::msg::Point2D point;
      point.x = position[0] + radius * cos(angle);
      point.y = position[1] + radius * sin(angle);
      stop_sign.points.push_back(point);
    }

    // Close the polygon
    if (!stop_sign.points.empty())
    {
      stop_sign.points.push_back(stop_sign.points[0]);
    }

    stop_sign_polygons.polygons.push_back(stop_sign);

    // Add white background inner octagon
    polygon_msgs::msg::Polygon2D stop_text_bg;
    stop_text_bg.z_offset = position[2] + 0.01; // Slightly above the stop sign

    // Create a smaller inner octagon (for text background)
    double inner_radius = radius * 0.7;
    for (int i = 0; i < num_sides; ++i)
    {
      double angle = 2.0 * M_PI * i / num_sides + M_PI / 8.0;
      polygon_msgs::msg::Point2D point;
      point.x = position[0] + inner_radius * cos(angle);
      point.y = position[1] + inner_radius * sin(angle);
      stop_text_bg.points.push_back(point);
    }

    // Close the polygon
    if (!stop_text_bg.points.empty())
    {
      stop_text_bg.points.push_back(stop_text_bg.points[0]);
    }

    stop_sign_polygons.polygons.push_back(stop_text_bg);

    // Create "STOP" text marker
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = rclcpp::Clock{}.now();
    text_marker.ns = "stop_text";
    text_marker.id = marker_id++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;

    // Position the text in the center of the sign
    text_marker.pose.position.x = position[0];
    text_marker.pose.position.y = position[1];
    text_marker.pose.position.z = position[2] + 0.02;

    // Set orientation (identity quaternion for text_view_facing)
    text_marker.pose.orientation.w = 1.0;

    // Set the text
    text_marker.text = "STOP";

    // Set the scale
    double text_size = radius * 0.5;
    text_marker.scale.z = text_size;

    // Set color (black text)
    text_marker.color.r = 0.0;
    text_marker.color.g = 0.0;
    text_marker.color.b = 0.0;
    text_marker.color.a = 1.0;

    // Set lifetime (0 for forever)
    text_marker.lifetime = rclcpp::Duration(0, 0);

    text_markers.markers.push_back(text_marker);

    // Add the stop sign to road elements collection for semantic information
    traffic_information_msgs::msg::RoadElements stop_element;
    stop_element.id = stop_sign_id++; // Use ID for road elements
    stop_element.type = "stop_sign";

    // Copy the points from the stop sign polygon
    for (const auto &point : stop_sign.points)
    {
      polygon_msgs::msg::Point2D p;
      p.x = point.x;
      p.y = point.y;
      stop_element.points.push_back(p);
    }

    road_elements.polygons.push_back(stop_element);

    std::cout << blue << "----> Added stop sign with text at position ("
              << position[0] << ", " << position[1] << ", " << position[2]
              << ")" << reset << std::endl;
  }
}

void OsmVisualizer::add_traffic_lights(const std::vector<std::array<double, 3>> &positions, float scale_factor)
{
  traffic_light_polygons.polygons.clear();
  traffic_light_polygons.header.stamp = rclcpp::Clock{}.now();
  traffic_light_polygons.header.frame_id = "map";

  // Set base color for traffic light (dark gray)
  std_msgs::msg::ColorRGBA traffic_light_color;
  traffic_light_color.r = 0.3;
  traffic_light_color.g = 0.3;
  traffic_light_color.b = 0.3;
  traffic_light_color.a = 0.9;
  traffic_light_polygons.colors.push_back(traffic_light_color);

  // Red light color
  std_msgs::msg::ColorRGBA red_light_color;
  red_light_color.r = 1.0;
  red_light_color.g = 0.0;
  red_light_color.b = 0.0;
  red_light_color.a = 0.9;
  traffic_light_polygons.colors.push_back(red_light_color);

  // Yellow light color
  std_msgs::msg::ColorRGBA yellow_light_color;
  yellow_light_color.r = 1.0;
  yellow_light_color.g = 1.0;
  yellow_light_color.b = 0.0;
  yellow_light_color.a = 0.9;
  traffic_light_polygons.colors.push_back(yellow_light_color);

  // Green light color
  std_msgs::msg::ColorRGBA green_light_color;
  green_light_color.r = 0.0;
  green_light_color.g = 1.0;
  green_light_color.b = 0.0;
  green_light_color.a = 0.9;
  traffic_light_polygons.colors.push_back(green_light_color);

  int traffic_light_id = 2000; // Start IDs from 2000 to avoid conflicts

  for (const auto &position : positions)
  {
    // Create traffic light housing
    polygon_msgs::msg::Polygon2D traffic_light_housing;
    // Removed traffic_light_housing.id assignment
    traffic_light_housing.z_offset = position[2];

    // Parameters for the traffic light
    double base_width = 0.5;
    double base_height = 1.2;
    double width = base_width * scale_factor;
    double height = base_height * scale_factor;

    // Create the rectangular shape for the traffic light housing
    polygon_msgs::msg::Point2D p1, p2, p3, p4;
    p1.x = position[0] - width / 2;
    p1.y = position[1] - height / 2;

    p2.x = position[0] + width / 2;
    p2.y = position[1] - height / 2;

    p3.x = position[0] + width / 2;
    p3.y = position[1] + height / 2;

    p4.x = position[0] - width / 2;
    p4.y = position[1] + height / 2;

    traffic_light_housing.points.push_back(p1);
    traffic_light_housing.points.push_back(p2);
    traffic_light_housing.points.push_back(p3);
    traffic_light_housing.points.push_back(p4);
    traffic_light_housing.points.push_back(p1);

    traffic_light_polygons.polygons.push_back(traffic_light_housing);

    // Create the three light circles (red, yellow, green)
    double base_light_radius = 0.15;
    double base_light_spacing = 0.3;
    double light_radius = base_light_radius * scale_factor;
    double light_spacing = base_light_spacing * scale_factor;

    // Red light (top)
    polygon_msgs::msg::Polygon2D red_light;
    // Removed red_light.id assignment
    red_light.z_offset = position[2] + 0.01; // Slightly in front of housing
    create_circle(red_light, position[0], position[1] + light_spacing, light_radius, 16);
    traffic_light_polygons.polygons.push_back(red_light);

    // Yellow light (middle)
    polygon_msgs::msg::Polygon2D yellow_light;
    // Removed yellow_light.id assignment
    yellow_light.z_offset = position[2] + 0.01;
    create_circle(yellow_light, position[0], position[1], light_radius, 16);
    traffic_light_polygons.polygons.push_back(yellow_light);

    // Green light (bottom)
    polygon_msgs::msg::Polygon2D green_light;
    // Removed green_light.id assignment
    green_light.z_offset = position[2] + 0.01;
    create_circle(green_light, position[0], position[1] - light_spacing, light_radius, 16);
    traffic_light_polygons.polygons.push_back(green_light);

    // Add the traffic light to road elements collection for semantic information
    traffic_information_msgs::msg::RoadElements traffic_element;
    traffic_element.id = traffic_light_id++;
    traffic_element.type = "traffic_light";

    // Copy the points from the traffic light housing
    for (const auto &point : traffic_light_housing.points)
    {
      polygon_msgs::msg::Point2D p;
      p.x = point.x;
      p.y = point.y;
      traffic_element.points.push_back(p);
    }

    road_elements.polygons.push_back(traffic_element);

    std::cout << green << "----> Added traffic light at position ("
              << position[0] << ", " << position[1] << ", " << position[2]
              << ")" << reset << std::endl;
  }
}

// Helper function to create circle polygons
void OsmVisualizer::create_circle(polygon_msgs::msg::Polygon2D &polygon, double center_x, double center_y, double radius, int num_segments)
{
  polygon.points.clear();

  for (int i = 0; i < num_segments; ++i)
  {
    double angle = 2.0 * M_PI * i / num_segments;
    polygon_msgs::msg::Point2D point;
    point.x = center_x + radius * cos(angle);
    point.y = center_y + radius * sin(angle);
    polygon.points.push_back(point);
  }

  // Close the circle
  if (!polygon.points.empty())
  {
    polygon.points.push_back(polygon.points[0]);
  }
}

void OsmVisualizer::add_speed_limit_signs(const std::vector<std::array<double, 3>> &positions, const std::vector<int> &speed_limits, float scale_factor)
{
  speed_limit_polygons.polygons.clear();
  speed_limit_polygons.header.stamp = rclcpp::Clock{}.now();
  speed_limit_polygons.header.frame_id = "map";

  // Set color for speed limit signs (white background)
  std_msgs::msg::ColorRGBA white_bg_color;
  white_bg_color.r = 0.0;
  white_bg_color.g = 0.0;
  white_bg_color.b = 0.0;
  white_bg_color.a = 1.0;
  speed_limit_polygons.colors.push_back(white_bg_color);

  // Set color for the red circle border
  std_msgs::msg::ColorRGBA red_border_color;
  red_border_color.r = 1.0;
  red_border_color.g = 0.9;
  red_border_color.b = 0.0;
  red_border_color.a = 0.9;
  speed_limit_polygons.colors.push_back(red_border_color);

  int speed_limit_id = 5000; // Start IDs from 5000 to avoid conflicts

  for (size_t i = 0; i < positions.size(); ++i)
  {
    const auto &position = positions[i];
    int speed_limit = (i < speed_limits.size()) ? speed_limits[i] : 50; // Default to 50 if not specified

    // Create circular shape for speed limit sign
    polygon_msgs::msg::Polygon2D speed_limit_sign;
    speed_limit_sign.z_offset = position[2];

    // Parameters for the speed limit sign
    double base_size = 0.7;
    double size = base_size * scale_factor;
    double radius = size / 2.0;
    int num_segments = 24;

    // Create the circular shape
    create_circle(speed_limit_sign, position[0], position[1], radius, num_segments);
    speed_limit_polygons.polygons.push_back(speed_limit_sign);

    // Create red border circle
    polygon_msgs::msg::Polygon2D border_circle;
    border_circle.z_offset = position[2] + 0.01;

    // Create a slightly smaller white inner circle for the border
    polygon_msgs::msg::Polygon2D inner_circle;
    inner_circle.z_offset = position[2] + 0.02;

    // Border is 10% of the radius
    double inner_radius = radius * 0.9;
    create_circle(inner_circle, position[0], position[1], inner_radius, num_segments);
    speed_limit_polygons.polygons.push_back(inner_circle);

    // Add the speed limit sign to road elements collection for semantic information
    traffic_information_msgs::msg::RoadElements speed_limit_element;
    speed_limit_element.id = speed_limit_id++;
    speed_limit_element.type = "speed_limit";
    speed_limit_element.type = "speed_limit_" + std::to_string(speed_limit);

    // Copy the points from the sign polygon
    for (const auto &point : speed_limit_sign.points)
    {
      polygon_msgs::msg::Point2D p;
      p.x = point.x;
      p.y = point.y;
      speed_limit_element.points.push_back(p);
    }

    road_elements.polygons.push_back(speed_limit_element);

    // Add text marker for the speed limit
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = rclcpp::Clock{}.now();
    text_marker.ns = "speed_limit_text";
    text_marker.id = speed_limit_id;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;

    // Position the text in the center of the sign
    text_marker.pose.position.x = position[0];
    text_marker.pose.position.y = position[1];
    text_marker.pose.position.z = position[2] + 0.03;

    text_marker.pose.orientation.w = 1.0;

    // Set the text (speed limit value)
    text_marker.text = std::to_string(speed_limit);

    // Set the scale
    double text_size = radius * 0.8;
    text_marker.scale.z = text_size;

    // Set color (black text)
    text_marker.color.r = 0.0;
    text_marker.color.g = 0.0;
    text_marker.color.b = 0.0;
    text_marker.color.a = 1.0;

    // Set lifetime (0 for forever)
    text_marker.lifetime = rclcpp::Duration(0, 0);

    text_markers.markers.push_back(text_marker);

    std::cout << purple << "----> Added speed limit sign (" << speed_limit << " km/h) at position ("
              << position[0] << ", " << position[1] << ", " << position[2]
              << ")" << reset << std::endl;
  }
}
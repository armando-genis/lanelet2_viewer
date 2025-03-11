# lanelet2_viewer
OSMVisualizer is a ROS 2 node designed for rendering OpenStreetMap (OSM) data in RViz2. It contrains a modified lanelet2 package custom to supports elevation changes along the Z-axis, enabling more accurate terrain visualization. This changes are descripted at the bottom of this README. 

## → 🔄 Building Required Packages

```bash
git clone https://github.com/KIT-MRT/mrt_cmake_modules.git
```

Make sure you have the required dependencies installed:
```bash
sudo apt update
sudo apt-get install libeigen3-dev
sudo apt install libpcl-dev
sudo apt-get install libpcap-dev
sudo apt-get install libqt5serialport5-dev
sudo apt-get install libpugixml-dev
sudo apt-get install libgeographic-dev geographiclib-tools


#ros2 packages
sudo apt-get install ros-$ROS_DISTRO-pcl-ros
sudo apt install ros-$ROS_DISTRO-vision-msgs
sudo apt install ros-$ROS_DISTRO-perception-pcl
sudo apt install ros-$ROS_DISTRO-pcl-msgs
sudo apt install ros-$ROS_DISTRO-vision-opencv
sudo apt install ros-$ROS_DISTRO-diagnostic-updater
sudo apt install ros-$ROS_DISTRO-color-util
```

Then, build the required ROS2 packages:

```bash
colcon build --packages-select polygon_msgs
colcon build --packages-select mrt_cmake_modules
colcon build --packages-select traffic_information_msgs
source install/setup.bash
colcon build --packages-select map_visualizer
colcon build --packages-select lanelet2_core
colcon build --packages-select lanelet2_maps
colcon build --packages-select lanelet2_io
colcon build --packages-select lanelet2_projection
colcon build --packages-select lanelet2_traffic_rules
colcon build --packages-select lanelet2_routing
colcon build --packages-select lanelet2_validation
colcon build --packages-select lanelet2_matching
colcon build --packages-select lanelet2_projection
colcon build --packages-select waypoints_routing
source install/setup.bash
colcon build
```

## → 🛣️ Considerations for Creating HD Maps with Vector Map Builder

When creating a `Lanelet2Map` in the Vector Map Builder, follow these steps to configure the map projection:

1. Click on **Change Map Project Info**.
2. Select **Set MGRS from Lat/Lon** and input the following coordinates:
   - **Latitude:** `49`
   - **Longitude:** `8.4`
3. Click **Convert** to apply these settings.

> **Note:** When exporting the map, you may encounter an error indicating that the component `x` or `y` is negative. This error can be safely ignored, as it does not impact the map creation process. Proceed with creating the map even if these errors appear.

when finding black spaces in the rout is beacuse the lack of points you can add points with insert point with linestring. 

It's crucial to define a **centerline** for each lanelet. Here's how to create centerlines using the **TIER IV Vector Map Builder**:

1. **Select the Lanelet** ➝ Click on the lanelet for which you want to create a centerline.
2. **Access Actions Menu** ➝ In the top-right corner, click on **Action**.
3. **Create Centerline** ➝ From the dropdown, select **Create Centerline**.

## → 🛑 changes:

### - Lanelet changes
In the file called /lanelet2_projection/LocalCartesian.cpp I change the to this when using localcartesian map type in orden to get the ele attribute from the oms correctly and not modify. 

```bash
BasicPoint3d LocalCartesianProjector::forward(const GPSPoint& gps) const {
  BasicPoint3d local{0., 0., 0.};
  this->localCartesian_.Forward(gps.lat, gps.lon, gps.ele, local[0], local[1], local[2]);
  local[2] = gps.ele;
  return local;
}
```

also wacht that in the "lanelet2_io/io_handlers/OsmFile.cpp" this is in the code:

```bash
  static Nodes readNodes(const pugi::xml_node& osmNode) {
    Nodes nodes;
    for (auto node = osmNode.child(keyword::Node); node;  // NOLINT
         node = node.next_sibling(keyword::Node)) {
      if (isDeleted(node)) {
        continue;
      }
      const auto id = node.attribute(keyword::Id).as_llong(InvalId);
      const auto attributes = tags(node);
      const auto lat = node.attribute(keyword::Lat).as_double(0.);
      const auto lon = node.attribute(keyword::Lon).as_double(0.);
      // const auto ele = node.attribute(keyword::Elevation).as_double(0.);

      const auto ele = node.find_child_by_attribute(keyword::Tag, keyword::Key, keyword::Elevation)
                           .attribute(keyword::Value)
                           .as_double(0.);

      // std::cout << "ele: " << ele << std::endl;

      nodes[id] = Node{id, attributes, {lat, lon, ele}};
    }
    return nodes;
  }
```

### - Polygon ros changes: 

The Polygon2D structure has been modified to include a z offset, allowing for multiple polygons with different z offsets. The following files were updated to incorporate these changes:

- polygon_rviz_plugins/src/polygons_display.cpp
- polygon_rviz_plugins/src/polygon_parts.cpp
- polygon_rviz_plugins/include/polygon_rviz_plugins/polygon_base.hpp

These modifications enable the use of 3D polygons with varied z positions in the ROS environment.

Polygon2D.msg: 
```bash
# Vertices of a simple polygon. Adjacent points are connected, as are the first and last.
float64 z_offset
polygon_msgs/Point2D[] points
```

polygon_base.hpp:
```bash
  void updateProperties()
  {
    resetOutlines();
    if (mode_property_->shouldDrawOutlines())
    {
      Ogre::ColourValue outline_color = rviz_common::properties::qtToOgre(outline_color_property_->getColor());
      for (unsigned int i = 0; i < saved_outlines_.size(); ++i)
      {
        double z_offset = saved_outlines_[i].z_offset;  // Use the z_offset from each polygon
        outline_objects_[i]->setPolygon(saved_outlines_[i], outline_color, z_offset);
      }
    }

    if (!mode_property_->shouldDrawFiller() || saved_fillers_.empty())
    {
      resetFillers();
    }
    else
    {
      for (unsigned int i = 0; i < saved_fillers_.size(); ++i)
      {
        double z_offset = saved_fillers_[i].outer.z_offset;  // Use the z_offset from each complex polygon
        filler_objects_[i]->setPolygon(saved_fillers_[i], filler_colors_[i % filler_colors_.size()], z_offset);
      }
    }
  }
```



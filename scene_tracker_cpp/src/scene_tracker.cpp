#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "scene_msgs/msg/tracked_object.hpp"
#include "scene_msgs/msg/tracked_object_array.hpp"

using namespace std::chrono_literals;

struct SimObject
{
  std::string id;
  std::string type;
  std::string parent_frame;
  double px, py, pz;
  double qx, qy, qz, qw;
  double sx, sy, sz;
  double r, g, b, a;
};

class SceneTracker : public rclcpp::Node
{
public:
  SceneTracker()
  : Node("scene_tracker")
  {
    this->declare_parameter<std::string>("panda_frame", "panda_link0");
    this->declare_parameter<double>("publish_rate", 20.0);
    this->declare_parameter<std::string>("scene_topic", "/scene_objects");

    panda_frame_ = this->get_parameter("panda_frame").as_string();
    const double publish_rate = this->get_parameter("publish_rate").as_double();
    const std::string scene_topic = this->get_parameter("scene_topic").as_string();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("scene_markers", 10);

    scene_sub_ = this->create_subscription<scene_msgs::msg::TrackedObjectArray>(
      scene_topic,
      10,
      std::bind(&SceneTracker::scene_callback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&SceneTracker::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Scene tracker started. Listening on: %s", scene_topic.c_str());
  }

private:
  void scene_callback(const scene_msgs::msg::TrackedObjectArray::SharedPtr msg)
  {
    objects_.clear();
    objects_.reserve(msg->objects.size());

    for (const auto & tracked_obj : msg->objects) {
      SimObject obj;
      obj.id = tracked_obj.id;
      obj.type = tracked_obj.type;
      obj.parent_frame = panda_frame_;

      obj.px = tracked_obj.pose.position.x;
      obj.py = tracked_obj.pose.position.y;
      obj.pz = tracked_obj.pose.position.z;

      obj.qx = tracked_obj.pose.orientation.x;
      obj.qy = tracked_obj.pose.orientation.y;
      obj.qz = tracked_obj.pose.orientation.z;
      obj.qw = tracked_obj.pose.orientation.w;

      obj.sx = tracked_obj.bbox_size.x;
      obj.sy = tracked_obj.bbox_size.y;
      obj.sz = tracked_obj.bbox_size.z;

      set_color_from_type(obj);

      objects_.push_back(obj);
    }
  }

  void set_color_from_type(SimObject & obj)
  {
    if (obj.type == "cube") {
      obj.r = 0.8; obj.g = 0.2; obj.b = 0.2; obj.a = 1.0;
    } else if (obj.type == "box") {
      obj.r = 0.2; obj.g = 0.8; obj.b = 0.2; obj.a = 1.0;
    } else if (obj.type == "cylinder") {
      obj.r = 0.2; obj.g = 0.2; obj.b = 0.8; obj.a = 1.0;
    } else {
      obj.r = 0.7; obj.g = 0.7; obj.b = 0.7; obj.a = 1.0;
    }
  }

  void timer_callback()
  {
    publish_tf();
    publish_markers();
  }

  void publish_tf()
  {
    const auto now = this->get_clock()->now();

    for (const auto & obj : objects_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now;
      t.header.frame_id = obj.parent_frame;
      t.child_frame_id = obj.id;

      t.transform.translation.x = obj.px;
      t.transform.translation.y = obj.py;
      t.transform.translation.z = obj.pz;

      t.transform.rotation.x = obj.qx;
      t.transform.rotation.y = obj.qy;
      t.transform.rotation.z = obj.qz;
      t.transform.rotation.w = obj.qw;

      tf_broadcaster_->sendTransform(t);
    }
  }

  void publish_markers()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    const auto now = this->get_clock()->now();

    for (size_t i = 0; i < objects_.size(); ++i) {
      const auto & obj = objects_[i];

      visualization_msgs::msg::Marker marker;
      marker.header.stamp = now;
      marker.header.frame_id = obj.parent_frame;
      marker.ns = "scene_objects";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = obj.px;
      marker.pose.position.y = obj.py;
      marker.pose.position.z = obj.pz;

      marker.pose.orientation.x = obj.qx;
      marker.pose.orientation.y = obj.qy;
      marker.pose.orientation.z = obj.qz;
      marker.pose.orientation.w = obj.qw;

      marker.scale.x = obj.sx;
      marker.scale.y = obj.sy;
      marker.scale.z = obj.sz;

      marker.color.r = obj.r;
      marker.color.g = obj.g;
      marker.color.b = obj.b;
      marker.color.a = obj.a;

      marker.lifetime = rclcpp::Duration(0, 0);

      marker_array.markers.push_back(marker);
    }

    marker_pub_->publish(marker_array);
  }

  std::string panda_frame_;
  std::vector<SimObject> objects_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<scene_msgs::msg::TrackedObjectArray>::SharedPtr scene_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SceneTracker>());
  rclcpp::shutdown();
  return 0;
}
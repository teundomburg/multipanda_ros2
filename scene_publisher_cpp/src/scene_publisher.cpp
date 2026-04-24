#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "scene_msgs/msg/tracked_object.hpp"
#include "scene_msgs/msg/tracked_object_array.hpp"

using namespace std::chrono_literals;

class ScenePublisher : public rclcpp::Node
{
public:
  ScenePublisher() : Node("scene_publisher")
  {
    publisher_ = this->create_publisher<scene_msgs::msg::TrackedObjectArray>("/scene_objects", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&ScenePublisher::publish_scene, this));
    RCLCPP_INFO(this->get_logger(), "Scene publisher started.");
  }

private:
  void publish_scene()
  {
    scene_msgs::msg::TrackedObjectArray msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "panda_link0";

    msg.objects.push_back(make_box("cube1", "cube", 0.5, 0.0, 0.03, 0.06, 0.06, 0.06));
    msg.objects.push_back(make_box("cube2", "cube", 0.5, 0.0, 0.08, 0.04, 0.04, 0.04));
    msg.objects.push_back(make_box("cube3", "cube", 0.5, 0.0, 0.11, 0.02, 0.02, 0.02));

    publisher_->publish(msg);
  }

  scene_msgs::msg::TrackedObject make_box(
    const std::string & id,
    const std::string & type,
    double x, double y, double z,
    double sx, double sy, double sz)
  {
    scene_msgs::msg::TrackedObject obj;
    obj.id = id;
    obj.type = type;

    obj.pose.position.x = x;
    obj.pose.position.y = y;
    obj.pose.position.z = z;
    obj.pose.orientation.x = 0.0;
    obj.pose.orientation.y = 0.0;
    obj.pose.orientation.z = 0.0;
    obj.pose.orientation.w = 1.0;

    obj.bbox_size.x = sx;
    obj.bbox_size.y = sy;
    obj.bbox_size.z = sz;

    return obj;
  }

  rclcpp::Publisher<scene_msgs::msg::TrackedObjectArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScenePublisher>());
  rclcpp::shutdown();
  return 0;
}
#ifndef MUJOCO_SCENE_PUBLISHER__MUJOCO_SCENE_PUBLISHER_HPP_
#define MUJOCO_SCENE_PUBLISHER__MUJOCO_SCENE_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <mujoco/mujoco.h>

#include "mujoco_ros/ros_two/plugin_utils.hpp"
#include "scene_msgs/msg/tracked_object_array.hpp"

namespace mujoco_scene_publisher
{

class MujocoScenePublisher : public mujoco_ros::MujocoPlugin
{
public:
  ~MujocoScenePublisher() override = default;

  bool Load(const mjModel * m, mjData * d) override;
  void Reset() override {}
  void LastStageCallback(const mjModel * model, mjData * data) override;

private:
  struct ObjectSpec
  {
    std::string body_name;
    std::string id;
    std::string type;
    double sx, sy, sz;
  };

  rclcpp::Publisher<scene_msgs::msg::TrackedObjectArray>::SharedPtr pub_;
  std::vector<ObjectSpec> tracked_objects_;
};

}  // namespace mujoco_scene_publisher

#endif
#include "mujoco_scene_publisher/mujoco_scene_publisher.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace mujoco_scene_publisher
{

bool MujocoScenePublisher::Load(const mjModel * /*m*/, mjData * /*d*/)
{
  auto node = get_node();

  RCLCPP_INFO(node->get_logger(), "mujoco_scene_publisher: Load() called");

  // Use a relative topic name so namespace behavior is easier to inspect.
  pub_ = node->create_publisher<scene_msgs::msg::TrackedObjectArray>("scene_objects", 10);

  tracked_objects_ = {
    {"cube1", "cube1", "cube", 0.06, 0.06, 0.06},
    {"cube2", "cube2", "cube", 0.04, 0.04, 0.04},
    {"cube3", "cube3", "cube", 0.02, 0.02, 0.02},
  };

  RCLCPP_INFO(
    node->get_logger(),
    "mujoco_scene_publisher: publisher created on topic '%s' with %zu tracked objects",
    pub_->get_topic_name(),
    tracked_objects_.size());

  return true;
}

void MujocoScenePublisher::LastStageCallback(const mjModel * model, mjData * data)
{
  if (!pub_) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "mujoco_scene_publisher: publisher is null");
    return;
  }

  auto node = get_node();

  RCLCPP_INFO_THROTTLE(
    node->get_logger(),
    *node->get_clock(),
    2000,
    "mujoco_scene_publisher: LastStageCallback() running");

  scene_msgs::msg::TrackedObjectArray msg;
  msg.header.stamp = node->get_clock()->now();
  msg.header.frame_id = "panda_link0";

  for (const auto & spec : tracked_objects_) {
    const int body_id = mj_name2id(model, mjOBJ_BODY, spec.body_name.c_str());
    if (body_id < 0) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        2000,
        "mujoco_scene_publisher: body '%s' not found in MuJoCo model",
        spec.body_name.c_str());
      continue;
    }

    scene_msgs::msg::TrackedObject obj;
    obj.id = spec.id;
    obj.type = spec.type;

    obj.pose.position.x = data->xpos[3 * body_id + 0];
    obj.pose.position.y = data->xpos[3 * body_id + 1];
    obj.pose.position.z = data->xpos[3 * body_id + 2];

    const mjtNum * xmat = &data->xmat[9 * body_id];
    mjtNum quat[4];
    mju_mat2Quat(quat, xmat);

    // MuJoCo: [w, x, y, z]
    // ROS:    [x, y, z, w]
    obj.pose.orientation.x = quat[1];
    obj.pose.orientation.y = quat[2];
    obj.pose.orientation.z = quat[3];
    obj.pose.orientation.w = quat[0];

    obj.bbox_size.x = spec.sx;
    obj.bbox_size.y = spec.sy;
    obj.bbox_size.z = spec.sz;

    msg.objects.push_back(obj);
  }

  RCLCPP_INFO_THROTTLE(
    node->get_logger(),
    *node->get_clock(),
    2000,
    "mujoco_scene_publisher: publishing %zu objects on %s",
    msg.objects.size(),
    pub_->get_topic_name());

  pub_->publish(msg);
}

}  // namespace mujoco_scene_publisher

PLUGINLIB_EXPORT_CLASS(mujoco_scene_publisher::MujocoScenePublisher, mujoco_ros::MujocoPlugin)
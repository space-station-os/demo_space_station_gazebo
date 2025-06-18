#include "space_station_control/mux.hpp"

MuxNode::MuxNode() : Node("mux_node") {
  command_sub_ = this->create_subscription<space_station_control::msg::ThrustersCmd>(
    "/thruster_command", 10,
    std::bind(&MuxNode::command_callback, this, std::placeholders::_1)
  );

  std::vector<std::string> thruster_ids = {
    "bb_th", "bl_th", "br_th",
    "lth_bb", "lth_bf", "lth_tb", "lth_tf",
    "rth_bb", "rth_bf", "rth_tb", "rth_tf"
  };

  for (const auto &id : thruster_ids) {
    auto topic = "/spacestation/thrust/" + id;
    thruster_pubs_[id] = this->create_publisher<std_msgs::msg::Float64>(topic, 10);
  }

  RCLCPP_INFO(this->get_logger(), "Mux node initialized.");
}

void MuxNode::command_callback(const space_station_control::msg::ThrustersCmd::SharedPtr msg) {
  double thrust = msg->thrust;
  std::string dir = msg->direction;

  // 1. Always stop all thrusters before activating a new set
  for (const auto &pair : thruster_pubs_) {
    std_msgs::msg::Float64 zero_msg;
    zero_msg.data = 0.0;
    pair.second->publish(zero_msg);
  }

  std::vector<std::string> active_thrusters;

  // 2. Define motion-specific thruster groups
  if (dir == "forward") {
    active_thrusters = { "bl_th", "bb_th", "br_th" };
  } else if (dir == "backward") {
    active_thrusters = { "lth_bb", "rth_bb" };
  } else if (dir == "up") {
    active_thrusters = { "lth_tf", "rth_tf", "rth_bf", "lth_bf" };
  } else if (dir == "down") {
    active_thrusters = { "lth_tb", "rth_tb", "rth_tf", "lth_tf" };
  } else if (dir == "yaw_left") {
    active_thrusters = { "rth_tf", "rth_bf", "rth_tb", "rth_tf" };
  } else if (dir == "yaw_right") {
    active_thrusters = { "lth_tf", "lth_bb", "lth_bf", "lth_tb" };
  } else if (dir == "left") {
    active_thrusters = { "bl_th" };
  } else if (dir == "right") {
    active_thrusters = { "br_th" };
  } else if (dir == "halt") {
    // already zeroed out all thrusters above
    RCLCPP_INFO(this->get_logger(), "Halting all thrusters.");
    return;
  } else {
    RCLCPP_WARN(this->get_logger(), "Unknown direction command: %s", dir.c_str());
    return;
  }

  // 3. Publish thrust only to selected thrusters
  for (const auto &thruster : active_thrusters) {
    std_msgs::msg::Float64 msg_out;
    msg_out.data = thrust;
    thruster_pubs_[thruster]->publish(msg_out);
  }

  RCLCPP_INFO(this->get_logger(), "Direction: %s @ %f", dir.c_str(), thrust);
}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MuxNode>());
  rclcpp::shutdown();
  return 0;
}

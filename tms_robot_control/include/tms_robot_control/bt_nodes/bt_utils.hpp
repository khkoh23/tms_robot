#pragma once

#include <behaviortree_cpp/tree_node.h>
#include <atomic>

inline bool is_cancel_requested_from_blackboard(
  const BT::NodeConfig & config)
{
  if (!config.blackboard) {
    return false;
  }

  try {
    auto * flag =
      config.blackboard->get<std::atomic<bool> *>("cancel_requested");
    return flag && flag->load();
  } catch (...) {
    return false;
  }
}
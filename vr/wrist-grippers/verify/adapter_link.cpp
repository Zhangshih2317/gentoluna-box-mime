#include "wrist_adapter.hpp"

// Build-only check of the same constructor and destructor used by the owner.
// Loading this library does not create a node or connect to a controller.
std::unique_ptr<wrist::Adapter> make_wrist_adapter(rclcpp::Node &owner, bool motion) {
    return std::make_unique<wrist::Adapter>(owner, motion);
}

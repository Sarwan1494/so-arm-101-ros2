#include <memory>
#include <thread>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "startup_sequence",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("startup_sequence");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {
        executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "so_arm_101");

  std::vector<std::string> sequence = {"startup_move", "ready"};

  for (auto const &state_name : sequence)
  {
        RCLCPP_INFO(logger, "Planning move to '%s'", state_name.c_str());
        move_group_interface.setNamedTarget(state_name);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool const success = static_cast<bool>(move_group_interface.plan(plan));

        if (!success)
        {
            RCLCPP_ERROR(logger, "Planning to '%s' failed, aborting sequence", state_name.c_str());
            break;
        }

        RCLCPP_INFO(logger, "Executing move to '%s'", state_name.c_str());
        move_group_interface.execute(plan);
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}

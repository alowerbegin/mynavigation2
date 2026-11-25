// 规控协调器进程入口，初始化 ROS 和日志，创建生命周期节点交由多线程执行器处理回调，再在退出时释放资源。

#include <memory>

#include "nav2_regulated_modules/regulated_navigator.hpp"
#include "rclcpp/rclcpp.hpp"

// 规控协调器进程入口。创建生命周期节点并交给 ROS 执行器，业务激活由生命周期管理器触发。
// 返回码由调用环境用于区分成功／失败，进程启动本身不能作为业务功能验收。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  SpdlogWrapper::init("nav2_regulated_modules", "regulated_navigator");
  auto node = std::make_shared<nav2_regulated_modules::RegulatedNavigator>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  SpdlogWrapper::shutdown();
  return 0;
}

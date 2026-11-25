// 键盘只发布 ChassisControl 目标；加减速度和 jerk 由统一导航器处理。
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "byd_custom_msgs/msg/chassis_control.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

class TerminalMode
{
public:
  explicit TerminalMode(const std::string & input_device)
  {
    input_fd_ = open(input_device.c_str(), O_RDONLY | O_NOCTTY);
    if (input_fd_ == -1 && isatty(STDIN_FILENO))
    {
      input_fd_ = dup(STDIN_FILENO);
    }
    if (input_fd_ == -1)
    {
      throw std::runtime_error("failed to open input device '" + input_device + "' and stdin is not a terminal");
    }

    if (tcgetattr(input_fd_, &original_termios_) == -1)
    {
      close(input_fd_);
      input_fd_ = -1;
      throw std::runtime_error("failed to read terminal attributes");
    }

    original_flags_ = fcntl(input_fd_, F_GETFL, 0);
    if (original_flags_ == -1)
    {
      close(input_fd_);
      input_fd_ = -1;
      throw std::runtime_error("failed to read terminal flags");
    }

    termios raw = original_termios_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(input_fd_, TCSANOW, &raw) == -1)
    {
      close(input_fd_);
      input_fd_ = -1;
      throw std::runtime_error("failed to configure terminal attributes");
    }
    termios_changed_ = true;

    if (fcntl(input_fd_, F_SETFL, original_flags_ | O_NONBLOCK) == -1)
    {
      restore();
      close(input_fd_);
      input_fd_ = -1;
      throw std::runtime_error("failed to configure non-blocking terminal input");
    }
    flags_changed_ = true;
  }

  ~TerminalMode()
  {
    restore();
    if (input_fd_ != -1)
    {
      close(input_fd_);
      input_fd_ = -1;
    }
  }

  TerminalMode(const TerminalMode &) = delete;
  TerminalMode & operator=(const TerminalMode &) = delete;

  int inputFd() const
  {
    return input_fd_;
  }

private:
  void restore()
  {
    if (flags_changed_)
    {
      (void)fcntl(input_fd_, F_SETFL, original_flags_);
      flags_changed_ = false;
    }
    if (termios_changed_)
    {
      (void)tcsetattr(input_fd_, TCSANOW, &original_termios_);
      termios_changed_ = false;
    }
  }

  int input_fd_{-1};
  termios original_termios_{};
  int original_flags_{0};
  bool termios_changed_{false};
  bool flags_changed_{false};
};

}  // namespace

class MyAgvKeyboardControl : public rclcpp::Node
{
public:
  MyAgvKeyboardControl() : Node("myagv_keyboard_control")
  {
    const auto input_device = declare_parameter<std::string>("input_device", "/dev/tty");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/downstream/chassis_control");
    publish_rate_ = declare_parameter<double>("publish_rate", 100.0);
    linear_speed_ = declare_parameter<double>("linear_speed", 0.3);
    angular_speed_ = declare_parameter<double>("angular_speed", 0.3);
    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.3);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 0.3);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.5);
    validateParameters();
    terminal_mode_ = std::make_unique<TerminalMode>(input_device);
    publisher_ = create_publisher<Command>(output_topic, rclcpp::QoS(10).reliable().durability_volatile());
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_));
    timer_ = create_wall_timer(period, std::bind(&MyAgvKeyboardControl::timerCallback, this));
    RCLCPP_INFO(get_logger(), "键盘目标输出 %s（ChassisControl），频率 %.1f Hz；请先启动统一规控入口", output_topic.c_str(), publish_rate_);
    RCLCPP_INFO(get_logger(), "W/↑前进，S/↓后退，A/←左转，D/→右转，X/空格停车，Q发送停车请求后退出");
    RCLCPP_INFO(get_logger(), "有效线／角速度 %.3f m/s、%.3f rad/s，按键超时 %.3f s；加减速与 jerk 使用导航器配置", std::min(linear_speed_, max_linear_speed_), std::min(angular_speed_, max_angular_speed_), command_timeout_);
  }

private:
  using Command = byd_custom_msgs::msg::ChassisControl;

  void validateParameters() const
  {
    if (!std::isfinite(publish_rate_) || publish_rate_ <= 0.0 || !std::isfinite(max_linear_speed_) || max_linear_speed_ <= 0.0 || !std::isfinite(max_angular_speed_) || max_angular_speed_ <= 0.0)
    {
      throw std::invalid_argument("publish_rate and maximum speeds must be finite and positive");
    }
    if (!std::isfinite(linear_speed_) || linear_speed_ < 0.0 || !std::isfinite(angular_speed_) || angular_speed_ < 0.0 || !std::isfinite(command_timeout_) || command_timeout_ < 0.0)
    {
      throw std::invalid_argument("target speeds and command_timeout must be finite and non-negative");
    }
  }

  // 空闲时不发消息，以免持续占用遥控并阻止导航 Action。
  void timerCallback()
  {
    if (!quit_requested_)
    {
      readKeyboard();
    }
    if (motion_active_ && command_timeout_ > 0.0 && std::chrono::duration<double>(std::chrono::steady_clock::now() - last_direction_key_).count() > command_timeout_)
    {
      requestStop();
      RCLCPP_WARN(get_logger(), "方向键超时，发送零速度请求，下游按配置减速停车");
    }
    if (stop_frames_remaining_ > 0)
    {
      publishCommand(true);
      --stop_frames_remaining_;
      if (quit_requested_ && stop_frames_remaining_ == 0)
      {
        RCLCPP_INFO(get_logger(), "已发送三帧停车请求；实际停稳由下游反馈确认");
        rclcpp::shutdown();
      }
      return;
    }
    if (motion_active_)
    {
      publishCommand(false);
    }
  }

  void readKeyboard()
  {
    char key = 0;
    while (!quit_requested_)
    {
      const ssize_t bytes_read = read(terminal_mode_->inputFd(), &key, 1);
      if (bytes_read == 1)
      {
        processKey(key);
        continue;
      }
      if (bytes_read == -1 && errno == EINTR)
      {
        continue;
      }
      if (bytes_read == -1 && errno != EAGAIN && errno != EWOULDBLOCK)
      {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to read keyboard input: %s", strerror(errno));
      }
      break;
    }
  }

  void processKey(char key)
  {
    if (escape_state_ == 1)
    {
      escape_state_ = key == '[' ? 2 : 0;
      return;
    }
    if (escape_state_ == 2)
    {
      escape_state_ = 0;
      switch (key)
      {
        case 'A': commandMotion(Command::OP_FORWARD); return;
        case 'B': commandMotion(Command::OP_BACKWARD); return;
        case 'C': commandMotion(Command::OP_TURN_RIGHT); return;
        case 'D': commandMotion(Command::OP_TURN_LEFT); return;
        default: return;
      }
    }
    if (key == '\x1b')
    {
      escape_state_ = 1;
      return;
    }
    switch (std::tolower(static_cast<unsigned char>(key)))
    {
      case 'w': commandMotion(Command::OP_FORWARD); break;
      case 's': commandMotion(Command::OP_BACKWARD); break;
      case 'a': commandMotion(Command::OP_TURN_LEFT); break;
      case 'd': commandMotion(Command::OP_TURN_RIGHT); break;
      case 'x':
      case ' ': requestStop(); break;
      case 'q':
        requestStop();
        quit_requested_ = true;
        break;
      default: break;
    }
  }

  void commandMotion(uint8_t operation)
  {
    operation_ = operation;
    motion_active_ = true;
    stop_frames_remaining_ = 0;
    last_direction_key_ = std::chrono::steady_clock::now();
  }

  void requestStop()
  {
    motion_active_ = false;
    stop_frames_remaining_ = 3;
  }

  // 零速仍使用合法 op，两个速度幅值均填零；acceleration=0 采用下游默认加速度。
  void publishCommand(bool stop)
  {
    Command message;
    message.header.stamp = now();
    message.header.frame_id = "base_link";
    message.op = operation_;
    message.acceleration = 0.0F;
    if (!stop)
    {
      if (operation_ == Command::OP_FORWARD || operation_ == Command::OP_BACKWARD)
      {
        message.linear_velocity = static_cast<float>(std::min(linear_speed_, max_linear_speed_));
      }
      else
      {
        message.angular_velocity = static_cast<float>(std::min(angular_speed_, max_angular_speed_));
      }
    }
    publisher_->publish(message);
  }

  std::unique_ptr<TerminalMode> terminal_mode_;
  rclcpp::Publisher<Command>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double publish_rate_{100.0};
  double linear_speed_{0.3};
  double angular_speed_{0.3};
  double max_linear_speed_{0.3};
  double max_angular_speed_{0.3};
  double command_timeout_{0.5};
  uint8_t operation_{Command::OP_FORWARD};
  bool motion_active_{false};
  bool quit_requested_{false};
  int stop_frames_remaining_{0};
  int escape_state_{0};
  std::chrono::steady_clock::time_point last_direction_key_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try
  {
    auto node = std::make_shared<MyAgvKeyboardControl>();
    rclcpp::spin(node);
  }
  catch (const std::exception & error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("myagv_keyboard_control"), "%s", error.what());
    exit_code = 1;
  }
  // Ctrl+C／异常退出不绕过仲裁直接发底盘命令，由下游指令超时停车兜底。
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
  return exit_code;
}

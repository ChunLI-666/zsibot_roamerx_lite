#include <arpa/inet.h>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "cmd_vel_to_zsibot/cmd_vel_to_zsibot.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

class UdpCapture
{
public:
  UdpCapture()
  {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
      throw std::runtime_error("socket() failed");
    }

    int flags = fcntl(sock_, F_GETFL, 0);
    fcntl(sock_, F_SETFL, flags | O_NONBLOCK);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    if (bind(sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
      close(sock_);
      throw std::runtime_error("bind() failed");
    }

    socklen_t len = sizeof(addr);
    if (getsockname(sock_, reinterpret_cast<sockaddr *>(&addr), &len) != 0) {
      close(sock_);
      throw std::runtime_error("getsockname() failed");
    }
    port_ = ntohs(addr.sin_port);
  }

  ~UdpCapture()
  {
    if (sock_ >= 0) {
      close(sock_);
    }
  }

  int port() const { return port_; }

  std::optional<std::string> receive(std::chrono::milliseconds timeout)
  {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(sock_, &read_fds);

    timeval tv{};
    tv.tv_sec = static_cast<long>(timeout.count() / 1000);
    tv.tv_usec = static_cast<long>((timeout.count() % 1000) * 1000);

    int ready = select(sock_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (ready <= 0) {
      return std::nullopt;
    }

    char buf[512];
    ssize_t n = recv(sock_, buf, sizeof(buf), 0);
    if (n <= 0) {
      return std::nullopt;
    }
    return std::string(buf, static_cast<size_t>(n));
  }

private:
  int sock_{-1};
  int port_{0};
};

class NodeHarness
{
public:
  explicit NodeHarness(const std::vector<rclcpp::Parameter> & params)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(params);
    bridge_ = std::make_shared<cmd_vel_to_zsibot::CmdVelToZsibot>(options);
    test_node_ = std::make_shared<rclcpp::Node>("cmd_vel_to_zsibot_test");

    cmd_pub_ = test_node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_safe", 10);
    sent_sub_ = test_node_->create_subscription<std_msgs::msg::String>(
      "/cmd_vel_to_zsibot/sent_command", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        sent_commands_.push_back(msg->data);
      });
    debug_sub_ = test_node_->create_subscription<std_msgs::msg::String>(
      "/cmd_vel_to_zsibot/debug_command", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        debug_commands_.push_back(msg->data);
      });

    exec_.add_node(bridge_);
    exec_.add_node(test_node_);
    spin_thread_ = std::thread([this]() { exec_.spin(); });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  ~NodeHarness()
  {
    exec_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    exec_.remove_node(test_node_);
    exec_.remove_node(bridge_);
  }

  void publishCmd(double vx, double vy, double wz)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = vx;
    msg.linear.y = vy;
    msg.angular.z = wz;
    cmd_pub_->publish(msg);
  }

  std::optional<std::string> waitForSentCommand(std::chrono::milliseconds timeout)
  {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (!sent_commands_.empty()) {
        return sent_commands_.back();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return std::nullopt;
  }

  std::optional<std::string> waitForDebugCommand(std::chrono::milliseconds timeout)
  {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (!debug_commands_.empty()) {
        return debug_commands_.back();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return std::nullopt;
  }

  std::shared_ptr<cmd_vel_to_zsibot::CmdVelToZsibot> bridge() { return bridge_; }

private:
  rclcpp::executors::SingleThreadedExecutor exec_;
  std::shared_ptr<cmd_vel_to_zsibot::CmdVelToZsibot> bridge_;
  std::shared_ptr<rclcpp::Node> test_node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sent_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr debug_sub_;
  std::vector<std::string> sent_commands_;
  std::vector<std::string> debug_commands_;
  std::thread spin_thread_;
};

std::vector<rclcpp::Parameter> udpParams(int port)
{
  return {
    rclcpp::Parameter("output_mode", "udp"),
    rclcpp::Parameter("control_host", "127.0.0.1"),
    rclcpp::Parameter("control_port", port),
    rclcpp::Parameter("cmd_vel_topic", "cmd_vel_safe"),
    rclcpp::Parameter("command_check_rate", 50.0),
    rclcpp::Parameter("min_command_interval", 0.1),
    rclcpp::Parameter("command_epsilon", 0.001),
  };
}

}  // namespace

TEST(CmdVelToZsibotTest, DefaultsUseFastUdpRefreshAndNoAutoStandup)
{
  UdpCapture udp;
  NodeHarness harness({
    rclcpp::Parameter("control_port", udp.port()),
  });

  double interval = 0.0;
  bool auto_standup = true;
  std::string output_mode;
  ASSERT_TRUE(harness.bridge()->get_parameter("min_command_interval", interval));
  ASSERT_TRUE(harness.bridge()->get_parameter("auto_standup", auto_standup));
  ASSERT_TRUE(harness.bridge()->get_parameter("output_mode", output_mode));
  EXPECT_DOUBLE_EQ(interval, 0.1);
  EXPECT_FALSE(auto_standup);
  EXPECT_EQ(output_mode, "udp");
}

TEST(CmdVelToZsibotTest, PublishesActualUdpPayloadForNonzeroCmdVel)
{
  UdpCapture udp;
  NodeHarness harness(udpParams(udp.port()));

  harness.publishCmd(0.08, 0.0, 0.12);
  auto payload = udp.receive(std::chrono::milliseconds(1000));
  ASSERT_TRUE(payload.has_value());
  EXPECT_NE(payload->find("\"type\":\"twist\""), std::string::npos);
  EXPECT_NE(payload->find("\"vx\":0.08"), std::string::npos);
  EXPECT_NE(payload->find("\"vy\":0"), std::string::npos);
  EXPECT_NE(payload->find("\"wz\":0.12"), std::string::npos);

  auto debug = harness.waitForSentCommand(std::chrono::milliseconds(1000));
  ASSERT_TRUE(debug.has_value());
  EXPECT_EQ(*debug, *payload);

  auto debug_record = harness.waitForDebugCommand(std::chrono::milliseconds(1000));
  ASSERT_TRUE(debug_record.has_value());
  EXPECT_NE(debug_record->find("\"reason\":\"cmd_vel\""), std::string::npos);
  EXPECT_NE(debug_record->find("\"send_ok\":true"), std::string::npos);
  EXPECT_NE(debug_record->find("\\\"type\\\":\\\"twist\\\""), std::string::npos);
  EXPECT_NE(debug_record->find("\"input\":{\"vx\":0.08"), std::string::npos);
  EXPECT_NE(debug_record->find("\"normalized\":{\"vx\":0.08"), std::string::npos);
}

TEST(CmdVelToZsibotTest, TinyCmdVelBelowEpsilonSendsStopInsteadOfMinimumMotion)
{
  UdpCapture udp;
  NodeHarness harness(udpParams(udp.port()));

  harness.publishCmd(0.0005, 0.0, 0.0);
  auto payload = udp.receive(std::chrono::milliseconds(1000));
  ASSERT_TRUE(payload.has_value());
  EXPECT_NE(payload->find("\"vx\":0"), std::string::npos);
  EXPECT_NE(payload->find("\"vy\":0"), std::string::npos);
  EXPECT_NE(payload->find("\"wz\":0"), std::string::npos);
  EXPECT_EQ(payload->find("0.05"), std::string::npos);
}

TEST(CmdVelToZsibotTest, UnsupportedSdkModeDoesNotSendUdp)
{
  UdpCapture udp;
  NodeHarness harness({
    rclcpp::Parameter("output_mode", "sdk"),
    rclcpp::Parameter("control_host", "127.0.0.1"),
    rclcpp::Parameter("control_port", udp.port()),
    rclcpp::Parameter("cmd_vel_topic", "cmd_vel_safe"),
    rclcpp::Parameter("command_check_rate", 50.0),
  });

  harness.publishCmd(0.1, 0.0, 0.0);
  auto payload = udp.receive(std::chrono::milliseconds(300));
  EXPECT_FALSE(payload.has_value());
}

TEST(CmdVelToZsibotTest, FakeModePublishesDebugButDoesNotSendUdp)
{
  UdpCapture udp;
  NodeHarness harness({
    rclcpp::Parameter("output_mode", "fake"),
    rclcpp::Parameter("control_host", "127.0.0.1"),
    rclcpp::Parameter("control_port", udp.port()),
    rclcpp::Parameter("cmd_vel_topic", "cmd_vel_safe"),
    rclcpp::Parameter("command_check_rate", 50.0),
    rclcpp::Parameter("min_command_interval", 0.1),
  });

  harness.publishCmd(0.08, 0.0, 0.12);

  auto udp_payload = udp.receive(std::chrono::milliseconds(300));
  EXPECT_FALSE(udp_payload.has_value());

  auto sent = harness.waitForSentCommand(std::chrono::milliseconds(1000));
  ASSERT_TRUE(sent.has_value());
  EXPECT_NE(sent->find("\"type\":\"twist\""), std::string::npos);
  EXPECT_NE(sent->find("\"vx\":0.08"), std::string::npos);

  auto debug = harness.waitForDebugCommand(std::chrono::milliseconds(1000));
  ASSERT_TRUE(debug.has_value());
  EXPECT_NE(debug->find("\"fake\":true"), std::string::npos);
  EXPECT_NE(debug->find("\"send_ok\":true"), std::string::npos);
  EXPECT_NE(debug->find("\"reason\":\"cmd_vel\""), std::string::npos);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}

// Copyright (c) 2026 The project contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include "nav2_lifecycle_manager/lifecycle_manager.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
auto makeContext()
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  return context;
}

rclcpp::NodeOptions optionsFor(const rclcpp::Context::SharedPtr & context)
{
  rclcpp::NodeOptions options;
  options.context(context);
  options.parameter_overrides({
      rclcpp::Parameter("node_names", std::vector<std::string>{}),
      rclcpp::Parameter("autostart", false), rclcpp::Parameter("bond_timeout", 0.0)});
  return options;
}

class ManagerProbe : public nav2_lifecycle_manager::LifecycleManager
{
public:
  using LifecycleManager::LifecycleManager;
  size_t managedNameCount() const {return node_names_.size();}
};
}  // namespace

TEST(CallbackLifetime, RemovesRegistrationBeforeNodeDestruction)
{
  auto context = makeContext();
  const auto initial = context->get_pre_shutdown_callbacks().size();
  for (int iteration = 0; iteration < 5; ++iteration) {
    auto manager = std::make_shared<ManagerProbe>(optionsFor(context));
    EXPECT_EQ(initial + 1, context->get_pre_shutdown_callbacks().size());
    manager.reset();
    EXPECT_EQ(initial, context->get_pre_shutdown_callbacks().size());
  }
  // There must be no callback pointing at any of those destroyed managers.
  EXPECT_TRUE(context->shutdown("manager already destroyed"));
  context.reset();
}

TEST(CallbackLifetime, DefersManagerResourceCleanupUntilAfterContextShutdown)
{
  auto context = makeContext();
  auto options = optionsFor(context);
  options.parameter_overrides({
    rclcpp::Parameter("node_names", std::vector<std::string>{"never_started_fixture"}),
    rclcpp::Parameter("autostart", false), rclcpp::Parameter("bond_timeout", 0.0)});
  const auto initial = context->get_pre_shutdown_callbacks().size();
  auto manager = std::make_shared<ManagerProbe>(options);
  EXPECT_EQ(1u, manager->managedNameCount());
  EXPECT_TRUE(context->shutdown("manager still alive"));
  // Context must finish interrupting nested executors before any worker join
  // or resource cleanup. The live manager keeps resources until destruction.
  EXPECT_EQ(1u, manager->managedNameCount());
  manager.reset();
  EXPECT_EQ(initial, context->get_pre_shutdown_callbacks().size());
  context.reset();
}

TEST(CallbackLifetime, ConstructorFailureRemovesAlreadyRegisteredCallback)
{
  auto context = makeContext();
  const auto initial = context->get_pre_shutdown_callbacks().size();
  // Upstream registers before reading node_names. Missing this required value
  // throws after registration and must not leave a raw-this callback behind.
  rclcpp::NodeOptions options;
  options.context(context);
  EXPECT_THROW(std::make_shared<ManagerProbe>(options),
    rclcpp::exceptions::ParameterUninitializedException);
  EXPECT_EQ(initial, context->get_pre_shutdown_callbacks().size());
  EXPECT_TRUE(context->shutdown("construction failed"));
  context.reset();
}

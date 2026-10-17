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
// This probe deliberately does NOT link the lifecycle-manager library. Its only
// reference is a real class_loader component load, followed by actual unload.
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "class_loader/class_loader.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"
#include "std_srvs/srv/trigger.hpp"

bool mapped(const std::string & path)
{
  std::ifstream maps("/proc/self/maps");
  std::string line;
  while (std::getline(maps, line)) {
    if (line.find(path) != std::string::npos) {return true;}
  }
  return false;
}

int main(int argc, char ** argv)
{
  if (argc != 2 && argc != 3) {return 2;}
  const std::string library = argv[1];
  const bool populated = argc == 3 && std::string(argv[2]) == "populated";
  auto context = populated ? rclcpp::contexts::get_global_default_context() :
    std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  const auto initial = context->get_pre_shutdown_callbacks().size();
  auto fixture = std::make_shared<rclcpp::Node>(
    "component_client_fixture", rclcpp::NodeOptions().context(context));
  auto get_state = fixture->create_service<lifecycle_msgs::srv::GetState>(
    "component_target/get_state",
    [](const std::shared_ptr<lifecycle_msgs::srv::GetState::Request>,
    std::shared_ptr<lifecycle_msgs::srv::GetState::Response> response) {
      response->current_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
    });
  for (int iteration = 0; iteration < 3; ++iteration) {
    {
      class_loader::ClassLoader loader(library);
      const auto classes = loader.getAvailableClasses<rclcpp_components::NodeFactory>();
      const auto match = std::find_if(classes.begin(), classes.end(), [](const auto & value) {
            return value.find("nav2_lifecycle_manager::LifecycleManager") != std::string::npos;
          });
      if (match == classes.end()) {return 3;}
      {
        auto factory = loader.createInstance<rclcpp_components::NodeFactory>(*match);
        rclcpp::NodeOptions options;
        options.context(context).parameter_overrides({
          rclcpp::Parameter("node_names", populated ?
          std::vector<std::string>{"component_target"} : std::vector<std::string>{}),
          rclcpp::Parameter("autostart", false), rclcpp::Parameter("bond_timeout", 0.0)});
        {
          auto node = factory->create_node_instance(options);
          if (context->get_pre_shutdown_callbacks().size() != initial + 1) {return 4;}
          if (populated) {
            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(fixture);
            executor.add_node(node.get_node_base_interface());
            auto ready_client = fixture->create_client<std_srvs::srv::Trigger>(
              "lifecycle_manager/is_active");
            using namespace std::chrono_literals;
            if (!ready_client->wait_for_service(5s)) {return 8;}
            auto ready = ready_client->async_send_request(
              std::make_shared<std_srvs::srv::Trigger::Request>());
            if (executor.spin_until_future_complete(ready, 5s) !=
              rclcpp::FutureReturnCode::SUCCESS) {return 9;}
            executor.remove_node(node.get_node_base_interface());
            executor.remove_node(fixture);
            // Destroy this test executor while the component DSO is still loaded.
          }
        }
      }
      if (context->get_pre_shutdown_callbacks().size() != initial) {
        std::cerr << "dangling Context callback after component destruction\n";
        return 5;
      }
      loader.unloadLibrary();
    }
    if (mapped(library)) {
      std::cerr << "library did not actually unload; test cannot prove callback DSO lifetime\n";
      return 6;
    }
  }
  if (!context->shutdown("all components and libraries already gone")) {return 7;}
  get_state.reset();
  fixture.reset();
  context.reset();
  std::cout << "3 component loads/unloads; Context shutdown/destruct after dlclose: PASS\n";
  return 0;
}

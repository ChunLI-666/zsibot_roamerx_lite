// Copyright (c) 2026, The project contributors. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "bondcpp/bond.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{
struct Spinner
{
  explicit Spinner(rclcpp::Executor & value)
  : executor(value), thread([&value]() {value.spin();}) {}
  void finish()
  {
    if (thread.joinable()) {
      executor.cancel();
      thread.join();
    }
  }
  ~Spinner() {finish();}
  rclcpp::Executor & executor;
  std::thread thread;
};

void exercise(bool concurrent, int round)
{
  // The global Context stays valid so legacy global rclcpp::ok() cannot mask a
  // closed Node-specific Context. Bond must check its own Node's Context.
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context);
  auto left_node = std::make_shared<rclcpp::Node>("left_bond_" + std::to_string(round), options);
  auto right_node = std::make_shared<rclcpp::Node>("right_bond_" + std::to_string(round), options);
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(left_node);
  executor.add_node(right_node);
  auto left = std::make_shared<bond::Bond>("lifetime_bond", "shutdown", left_node);
  auto right = std::make_shared<bond::Bond>("lifetime_bond", "shutdown", right_node);
  Spinner spinning(executor);
  left->start();
  right->start();
  if (!left->waitUntilFormed(rclcpp::Duration(4s)) ||
    !right->waitUntilFormed(rclcpp::Duration(4s)))
  {
    throw std::runtime_error("real bond pair did not form");
  }
  spinning.finish();
  std::cout << "round " << round << ": real bond pair formed; " <<
    (concurrent ? "racing Context close and destructor" : "closing Context before destructor") <<
    std::endl;
  if (concurrent) {
    std::atomic<bool> go{false};
    std::thread closing([&]() {
        while (!go.load()) {std::this_thread::yield();}
        context->shutdown("concurrent bond destructor regression");
      });
    go.store(true);
    left.reset();
    closing.join();
  } else {
    context->shutdown("formed Bond outlives its Context");
    left.reset();
  }
  right.reset();
  executor.remove_node(left_node);
  executor.remove_node(right_node);
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {return 2;}
  const std::string mode = argv[1];
  if (mode != "formed" && mode != "concurrent") {return 2;}
  rclcpp::init(0, nullptr);
  try {
    const int rounds = mode == "concurrent" ? 3 : 1;
    for (int i = 0; i < rounds; ++i) {
      exercise(mode == "concurrent", i);
    }
    rclcpp::shutdown();
    std::cout << mode << ": PASS (formed Bond resource teardown)" << std::endl;
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}

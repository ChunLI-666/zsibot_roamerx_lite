// Copyright 2026 Zsibot navigation contributors. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "navigo_bt_navigator/navigators/navigate_to_pose.hpp"

class ExposedNavigator : public navigo_bt_navigator::NavigateToPoseNavigator
{
public:
  using NavigateToPoseNavigator::goalReceived;
};
class EpochTreeSelection : public ::testing::Test
{
protected:
  void SetUp() override {if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}}
  void TearDown() override {rclcpp::shutdown();}
  rclcpp_lifecycle::LifecycleNode::SharedPtr node(bool epoch, bool backup)
  {
    return std::make_shared<rclcpp_lifecycle::LifecycleNode>("tree_selection",
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("enable_epoch_contract", epoch),
        rclcpp::Parameter("enable_epoch_backup_recovery", backup),
        rclcpp::Parameter("default_nav_to_pose_bt_xml", "/tmp/unreviewed.xml")}));
  }
};
TEST_F(EpochTreeSelection, LegacyDefaultIsPreserved)
{
  ExposedNavigator nav; auto parent=node(false,false);
  EXPECT_EQ(nav.getDefaultBTFilepath(parent), "/tmp/unreviewed.xml");
}
TEST_F(EpochTreeSelection, EpochDefaultDoesNotEnableMotionRecovery)
{
  ExposedNavigator nav; auto parent=node(true,false);
  const auto path=nav.getDefaultBTFilepath(parent);
  EXPECT_NE(path.find("/navigate_to_pose_with_epoch.xml"), std::string::npos);
  EXPECT_EQ(path.find("backup"), std::string::npos);
}
TEST_F(EpochTreeSelection, ExplicitBackupSelectsOnlyPackagedTree)
{
  ExposedNavigator nav; auto parent=node(true,true);
  const auto path=nav.getDefaultBTFilepath(parent);
  EXPECT_NE(path.find("/navigate_to_pose_with_epoch_backup.xml"), std::string::npos);
  auto goal=std::make_shared<nav2_msgs::action::NavigateToPose::Goal>();
  goal->behavior_tree="/tmp/unreviewed.xml";
  EXPECT_FALSE(nav.goalReceived(goal));
}
TEST_F(EpochTreeSelection, BackupWithoutEpochFailsClosed)
{
  ExposedNavigator nav; auto parent=node(false,true);
  EXPECT_THROW(nav.getDefaultBTFilepath(parent), std::invalid_argument);
}

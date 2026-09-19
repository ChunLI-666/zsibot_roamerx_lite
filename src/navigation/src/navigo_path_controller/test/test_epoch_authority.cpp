#include <gtest/gtest.h>
#include "navigo_core/epoch_contract.hpp"

namespace epoch = navigo_core::epoch;
constexpr uint64_t start = 1000000000000ULL;

struct EpochAuthorityTest : testing::Test
{
  epoch::Authority authority;
  navigo_epoch_msgs::msg::LocalizationEpoch loc;
  navigo_epoch_msgs::msg::NavigationIntent intent;
  void SetUp() override
  {
    authority.boot = "boot";
    authority.updateGate("gate", start);
    loc.schema_version = 1; loc.heartbeat_sequence = 1;
    loc.lifecycle_enabled = loc.output_ready = loc.fusion_ready = true;
    loc.health = 1; loc.base_frame_id = "base_link";
    loc.identity.process_session_id = "localizer";
    loc.identity.map_loaded_instance = "loaded-map";
    loc.identity.epoch = loc.identity.commits = 1;
    loc.output_pose.header.frame_id = "map";
    loc.output_pose.header.stamp.sec = 10;
    loc.output_pose.pose.orientation.w = 1.;
    authority.updateLocalization(loc, start);
    intent.token.localization = loc.identity;
    intent.token.navigation_session_id = "navigator";
    intent.token.gate_session_id = "gate";
    intent.token.task_sequence = intent.token.plan_sequence = 1;
    intent.heartbeat_sequence = 1; intent.active = true;
    intent.boot_id = "boot"; intent.source_steady_time_ns = start;
    authority.updateIntent(intent, start);
  }
};

TEST_F(EpochAuthorityTest, RequiresExactInstalledContext)
{
  EXPECT_TRUE(authority.permits(intent.token, start));
  auto old = intent.token; ++old.plan_sequence;
  EXPECT_FALSE(authority.permits(old, start));
  old = intent.token; ++old.localization.epoch;
  EXPECT_FALSE(authority.permits(old, start));
}
TEST_F(EpochAuthorityTest, GateRestartRetiresOldChallenge)
{
  authority.updateGate("new-gate", start + 1);
  authority.updateGate("gate", start + 2);
  EXPECT_EQ(authority.gate_session, "new-gate");
  EXPECT_FALSE(authority.permits(intent.token, start + 2));
}
TEST_F(EpochAuthorityTest, DuplicateHeartbeatsDoNotRenewSensorFreshness)
{
  loc.heartbeat_sequence = 2;
  authority.updateLocalization(loc, start + 399000000);
  authority.updateGate("gate", start + 399000000);
  EXPECT_FALSE(authority.localizationReady(start + 401000000));
}
TEST_F(EpochAuthorityTest, OutputRegressionRequiresNewIdentity)
{
  loc.heartbeat_sequence = 2; loc.output_pose.header.stamp.sec = 9;
  authority.updateLocalization(loc, start + 1);
  loc.heartbeat_sequence = 3; loc.output_pose.header.stamp.sec = 11;
  authority.updateLocalization(loc, start + 2);
  EXPECT_FALSE(authority.localizationReady(start + 2));
  loc.heartbeat_sequence = 4; ++loc.identity.epoch; ++loc.identity.commits;
  authority.updateLocalization(loc, start + 3);
  EXPECT_TRUE(authority.localizationReady(start + 3));
  EXPECT_FALSE(authority.permits(intent.token, start + 3));
}
TEST_F(EpochAuthorityTest, LoadedMapChangeCannotReuseGrid)
{
  loc.heartbeat_sequence = 2; loc.identity.map_loaded_instance = "other-map";
  authority.updateLocalization(loc, start + 1);
  loc.heartbeat_sequence = 3; loc.identity.map_loaded_instance = "loaded-map";
  authority.updateLocalization(loc, start + 2);
  EXPECT_TRUE(authority.map_changed);
  EXPECT_FALSE(authority.localizationReady(start + 2));
}
TEST_F(EpochAuthorityTest, RetiredProducerAndRegressingPlanCannotReturn)
{
  loc.identity.process_session_id = "new-localizer";
  authority.updateLocalization(loc, start + 1);
  loc.identity.process_session_id = "localizer"; loc.heartbeat_sequence = 999;
  authority.updateLocalization(loc, start + 2);
  EXPECT_EQ(authority.localization.identity.process_session_id, "new-localizer");
  intent.token.localization = authority.localization.identity;
  intent.heartbeat_sequence = 2; intent.token.plan_sequence = 2;
  authority.updateIntent(intent, start + 3);
  intent.heartbeat_sequence = 3; intent.token.plan_sequence = 1;
  authority.updateIntent(intent, start + 4);
  EXPECT_EQ(authority.intent.token.plan_sequence, 2U);
}
TEST_F(EpochAuthorityTest, SourceClockCannotBeRenewedByRelay)
{
  navigo_epoch_msgs::msg::EpochCommand command;
  command.controller_session_id = "controller"; command.command_sequence = 1;
  command.boot_id = "boot"; command.source_steady_time_ns = start; command.max_age_ms = 300;
  EXPECT_TRUE(epoch::commandFresh(command, "boot", start, 300000000));
  command.relay_sequence = 200;
  EXPECT_FALSE(epoch::commandFresh(command, "boot", start + 300000001, 300000000));
  EXPECT_FALSE(epoch::commandFresh(command, "other-boot", start, 300000000));
  EXPECT_FALSE(epoch::commandFresh(command, "boot", start - 1, 300000000));
}

TEST_F(EpochAuthorityTest, ReloadRevocationAndZeroPoseKeepMaximumWatermarks)
{
  auto revoked = loc;
  revoked.heartbeat_sequence = 2; revoked.health = 3; revoked.output_ready = false;
  revoked.identity.map_loaded_instance.clear(); revoked.identity.epoch = revoked.identity.commits = 0;
  revoked.output_pose.header.stamp.sec = 0;
  authority.updateLocalization(revoked, start + 100000000);
  EXPECT_FALSE(authority.localization.output_ready);
  loc.heartbeat_sequence = 3;
  authority.updateLocalization(loc, start + 410000000);
  authority.updateGate("gate", start + 410000000);
  EXPECT_FALSE(authority.localizationReady(start + 410000000));
  loc.heartbeat_sequence = 4; loc.output_pose.header.stamp.sec = 11;
  authority.updateLocalization(loc, start + 420000000);
  EXPECT_TRUE(authority.localizationReady(start + 420000000));
  loc.heartbeat_sequence = 5; loc.identity.epoch = 0;
  authority.updateLocalization(loc, start + 430000000);
  EXPECT_EQ(authority.localization.identity.epoch, 1U);
}

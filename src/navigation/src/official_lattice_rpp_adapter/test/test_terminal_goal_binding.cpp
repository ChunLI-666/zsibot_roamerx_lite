// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <atomic>
#include <condition_variable>
#include <future>
#include <thread>
#include "official_lattice_rpp_adapter/terminal_goal_binding.hpp"
#include "official_lattice_rpp_adapter/terminal_stop_hold.hpp"
using namespace official_lattice_rpp_adapter;
static geometry_msgs::msg::PoseStamped at(double x,double y,double a) {
  geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.pose.position.x=x;p.pose.position.y=y;
  p.pose.orientation.z=std::sin(a/2.);p.pose.orientation.w=std::cos(a/2.);return p;
}
static nav_msgs::msg::Path path(std::initializer_list<geometry_msgs::msg::PoseStamped> poses) {
  nav_msgs::msg::Path p;p.header.frame_id="map";p.poses=poses;return p;
}
TEST(TerminalGoalBinding, RecordedNative144MicrometreGoalIsInsideOfficialCellBudget) {
  nav2_costmap_2d::Costmap2D map(2000,2000,.01,-9.18,-8.86,0);
  const auto s=at(.8207693804070251,1.1400428188075338,-1.503464284225);
  const auto g=at(.8206269664360291,1.1400660310173036,-3.0742606110200903);
  auto raw=path({s});raw.poses.front().pose.orientation=g.pose.orientation;
  const auto p=goal_region::proveGoalCell(raw,s,g,map);
  EXPECT_TRUE(p.valid);EXPECT_TRUE(p.native_singleton);
  EXPECT_NEAR(p.error,.0001442932632427916,1e-15);EXPECT_LT(p.error,p.budget);
}
TEST(TerminalGoalBinding, FullSearchFloatCellProofAcceptsQuantizedEndpointNotArbitraryNearbyXY) {
  nav2_costmap_2d::Costmap2D map(2000,2000,.01,-9.05,-8.9,0);
  const auto s=at(.94999,1.0949519390997173,0.),g=at(.95001,1.0949519390997173,0.);
  auto raw=path({s,at(.9599900436401363,1.0949521636962896,0.)});
  EXPECT_TRUE(goal_region::proveGoalCell(raw,s,g,map).valid);
  raw.poses.back().pose.position.x+=.0000001;
  EXPECT_FALSE(goal_region::proveGoalCell(raw,s,g,map).valid);
  raw.poses.back()=at(.9699900436401363,1.0949521636962896,0.);
  EXPECT_FALSE(goal_region::proveGoalCell(raw,s,g,map).valid);
}
TEST(TerminalGoalBinding, SplitsOnlyExplicitFinalGoalAndPreservesCompletePrefix) {
  auto p=path({at(0.,0.,0.),at(1.,0.,0.),at(1.,0.,M_PI_2),at(1.009,0.,M_PI_2)});
  const auto b=goal_region::splitBinding(p,.01,20.);
  ASSERT_EQ(b.executable.poses.size(),3u);EXPECT_EQ(b.rotation_begin,1u);EXPECT_EQ(b.suffix_begin,0u);
  EXPECT_DOUBLE_EQ(b.original_goal.pose.position.x,1.009);
  EXPECT_THROW(goal_region::splitBinding(path({at(0.,0.,0.),at(.02,0.,0.)}),.01,20.),std::runtime_error);
  EXPECT_THROW(goal_region::splitBinding(path({at(0.,0.,0.),at(.001,0.,1.)}),.01,20.),std::runtime_error);
}
TEST(TerminalGoalBinding, TerminalSuffixStartsAfterAllPriorTurnsAndCannotBeSelectedByLoopProximity) {
  auto p=path({at(0.,0.,0.),at(1.,0.,0.),at(1.,0.,M_PI_2),at(1.,1.,M_PI_2),
    at(1.,1.,M_PI),at(0.,1.,M_PI),at(0.,1.,-M_PI_2),at(0.,0.,-M_PI_2),at(0.,0.,0.),at(.001,0.,0.)});
  const auto b=goal_region::splitBinding(p,.01,20.);
  EXPECT_EQ(b.suffix_begin,6u);EXPECT_EQ(b.rotation_begin,7u);
  EXPECT_GT(b.suffix_begin,0u);
  auto fake=path({at(0.,0.,0.),at(.01,.01,.3),at(.02,.02,0.),at(.021,.02,0.)});
  EXPECT_THROW(goal_region::splitBinding(fake,.01,20.),std::runtime_error);
}
TEST(TerminalGoalBinding, OrderedHeadingCursorCannotSkipCompleteTurnAtMatchingFinalYaw) {
  std::vector<double> headings;for(int i=0;i<=64;++i)headings.push_back(i*2*M_PI/64);
  goal_region::HeadingCursor cursor;cursor.reset(headings);cursor.advance(0.,.08);
  EXPECT_FALSE(cursor.complete());EXPECT_LT(cursor.next(),3u);
  for(int i=1;i<=64;++i)cursor.advance(i*2*M_PI/64,.08);
  EXPECT_TRUE(cursor.complete());
  cursor.reset({0.,M_PI_2,M_PI,0.});cursor.advance(0.,.08);EXPECT_EQ(cursor.next(),1u);
}
TEST(TerminalGoalBinding, SameGoalNewRouteHasDifferentImmutableGeometricBinding) {
  const auto a=goal_region::splitBinding(path({at(0.,0.,0.),at(1.,0.,0.),at(1.001,0.,0.)}),.01,20.);
  const auto b=goal_region::splitBinding(path({at(.1,0.,0.),at(1.,0.,0.),at(1.001,0.,0.)}),.01,20.);
  EXPECT_NE(a.geometric_digest,b.geometric_digest);
  EXPECT_DOUBLE_EQ(a.original_goal.pose.position.x,b.original_goal.pose.position.x);
}
TEST(TerminalGoalBinding, NonterminalWindingAndReversalRemainAvailableToOrderedCheckpoints) {
  nav_msgs::msg::Path p;p.header.frame_id="map";
  for (int i=0;i<=64;++i)p.poses.push_back(at(0.,0.,i*2*M_PI/64));
  p.poses.push_back(at(.02,0.,0.));p.poses.push_back(at(.021,0.,0.));
  EXPECT_NO_THROW(goal_region::splitBinding(p,.01,20.));
  p=path({at(0.,0.,0.),at(0.,0.,M_PI_2),at(0.,0.,0.),at(.02,0.,0.),at(.021,0.,0.)});
  EXPECT_NO_THROW(goal_region::splitBinding(p,.01,20.));
}
TEST(TerminalGoalBinding, NonterminalHalfTurnAndNearbySignedRunsRemainSupported) {
  for (double sign:{-1.,1.}) {
    const auto p=path({at(0.,0.,0.),at(0.,0.,sign*M_PI),at(-.02,0.,sign*M_PI),at(-.021,0.,sign*M_PI)});
    EXPECT_NO_THROW(goal_region::splitBinding(p,.01,20.));
    const double h=sign*(M_PI-1e-4);
    const auto near=path({at(0.,0.,0.),at(0.,0.,h),at(.02*std::cos(h),.02*std::sin(h),h),
      at(.021*std::cos(h),.021*std::sin(h),h)});
    EXPECT_NO_THROW(goal_region::splitBinding(near,.01,20.));
  }
}
TEST(TerminalGoalBinding, WrappedNonterminalRotationRemainsMonotonic) {
  const double h=190.*M_PI/180.;
  const auto p=path({at(0.,0.,170.*M_PI/180.),at(0.,0.,179.*M_PI/180.),
    at(0.,0.,-179.*M_PI/180.),at(0.,0.,-170.*M_PI/180.),
    at(.02*std::cos(h),.02*std::sin(h),h),at(.021*std::cos(h),.021*std::sin(h),h)});
  EXPECT_NO_THROW(goal_region::splitBinding(p,.01,20.));
}
TEST(TerminalGoalBinding, ManySubEpsilonStepsCannotHideATranslatingLoopAsOneRotation) {
  nav_msgs::msg::Path p;p.header.frame_id="map";
  for (int i=0;i<=2048;++i) {
    const double a=i*2*M_PI/2048;
    p.poses.push_back(at(.01*(1.-std::cos(a)),.01*std::sin(a),a+M_PI_2));
  }
  p.poses.push_back(at(.001,0.,M_PI_2));
  EXPECT_THROW(goal_region::splitBinding(p,.01,20.),std::runtime_error);
}
static std::shared_ptr<const goal_region::Binding> bound(goal_region::CompletionGate & gate) {
  auto b=goal_region::splitBinding(path({at(0.,0.,0.),at(.001,0.,0.)}),.01,20.);
  b.generation=gate.bindPlan();auto immutable=std::make_shared<const goal_region::Binding>(std::move(b));
  gate.installBinding(immutable);return immutable;
}
TEST(CompletionGate, ServerResetAfterSetPlanPreservesNewImmutableGoalButRevokesCompletion) {
  goal_region::CompletionGate gate;const auto binding=bound(gate);
  gate.finishCycle(gate.beginCycle(binding->generation),true);const auto old=gate.snapshot();
  ASSERT_TRUE(gate.stillEligible(old));gate.invalidate();
  const auto reset=gate.snapshot();EXPECT_EQ(reset.binding,binding);EXPECT_FALSE(reset.ready);
  EXPECT_FALSE(gate.stillEligible(old));
  gate.finishCycle(gate.beginCycle(binding->generation),true);EXPECT_TRUE(gate.stillEligible(gate.snapshot()));
}
TEST(CompletionGate, SameGoalNewPlanAndFailedNewPlanCannotReusePreviousCompletion) {
  goal_region::CompletionGate gate;const auto first=bound(gate);
  gate.finishCycle(gate.beginCycle(first->generation),true);const auto old=gate.snapshot();
  const auto second=bound(gate);EXPECT_NE(first->generation,second->generation);
  EXPECT_DOUBLE_EQ(first->original_goal.pose.position.x,second->original_goal.pose.position.x);
  EXPECT_FALSE(gate.stillEligible(old));EXPECT_FALSE(gate.snapshot().ready);
  gate.finishCycle(gate.beginCycle(second->generation),true);const auto before_invalid=gate.snapshot();
  gate.bindPlan();EXPECT_FALSE(gate.snapshot().binding);EXPECT_FALSE(gate.stillEligible(before_invalid));
  EXPECT_THROW(gate.installBinding(second),std::runtime_error);
}
TEST(CompletionGate, CancelClearAndConcurrentCycleInvalidateAnInFlightGeometryCheck) {
  goal_region::CompletionGate gate;auto b=bound(gate);
  gate.finishCycle(gate.beginCycle(b->generation),true);auto token=gate.snapshot();
  gate.invalidate();EXPECT_FALSE(gate.stillEligible(token));
  const auto a=gate.beginCycle(b->generation),next=gate.beginCycle(b->generation);
  gate.finishCycle(a,true);EXPECT_FALSE(gate.snapshot().ready);
  gate.finishCycle(next,true);token=gate.snapshot();ASSERT_TRUE(gate.stillEligible(token));
  gate.finishCycle(gate.beginCycle(b->generation),true);EXPECT_FALSE(gate.stillEligible(token));
  token=gate.snapshot();gate.clearPlan();EXPECT_FALSE(gate.snapshot().binding);
  EXPECT_FALSE(gate.stillEligible(token));EXPECT_THROW(gate.beginCycle(b->generation),std::runtime_error);
}
TEST(CompletionGate, DefaultOffHasNoBindingAndRetainsOrderedCompletionPermission) {
  goal_region::CompletionGate gate;gate.bindPlan();
  EXPECT_FALSE(gate.snapshot().binding);EXPECT_FALSE(gate.snapshot().ready);
  gate.finishCycle(gate.beginCycle(),true);auto token=gate.snapshot();
  EXPECT_FALSE(token.binding);EXPECT_TRUE(gate.stillEligible(token));
  gate.invalidate();EXPECT_FALSE(gate.stillEligible(token));
}
TEST(CompletionGate, SamePlanCannotReplaceOrReinstallItsImmutableOriginalGoal) {
  goal_region::CompletionGate gate;const auto first=bound(gate);
  gate.finishCycle(gate.beginCycle(first->generation),true);const auto token=gate.snapshot();
  EXPECT_THROW(gate.installBinding(first),std::runtime_error);
  auto different=std::make_shared<goal_region::Binding>(*first);different->original_goal.pose.position.x=.002;
  EXPECT_THROW(gate.installBinding(different),std::runtime_error);
  EXPECT_EQ(gate.snapshot().binding,first);EXPECT_TRUE(gate.stillEligible(token));
  EXPECT_NO_THROW(bound(gate));EXPECT_FALSE(gate.stillEligible(token));
}

TEST(CompletionGate, OriginalSourceFutureAndExpiryBoundsCannotUseANewerClockSample) {
  goal_region::CompletionGate gate;const auto binding=bound(gate);
  gate.finishCycle(gate.beginCycle(binding->generation),true,
    std::chrono::steady_clock::now()+std::chrono::seconds(1),300,100);
  const auto token=gate.snapshot();ASSERT_TRUE(token.ready);
  EXPECT_FALSE(gate.stillEligible(token));
  EXPECT_FALSE(gate.stillEligible(token,[]{return 99;}));
  EXPECT_TRUE(gate.stillEligible(token,[]{return 100;}));
  EXPECT_TRUE(gate.stillEligible(token,[]{return 299;}));
  EXPECT_FALSE(gate.stillEligible(token,[]{return 300;}));
  gate.finishCycle(gate.beginCycle(binding->generation),true,
    std::chrono::steady_clock::now()-std::chrono::nanoseconds(1),300,100);
  EXPECT_FALSE(gate.snapshot().ready);
  EXPECT_FALSE(gate.stillEligible(token,[]{return 200;}));
}

TEST(CompletionGate, SourceClockIsSampledUnderTheFinalGateLock) {
  using namespace std::chrono_literals;
  goal_region::CompletionGate gate;const auto binding=bound(gate);
  gate.finishCycle(gate.beginCycle(binding->generation),true,
    std::chrono::steady_clock::now()+1s,300,100);
  const auto token=gate.snapshot();std::atomic<std::int64_t> source{200};
  std::promise<void> first_entered,release_first;
  auto release=release_first.get_future().share();
  auto first=std::async(std::launch::async,[&]{
    return gate.stillEligible(token,[&]{first_entered.set_value();release.wait();return source.load();});
  });
  const auto entered=first_entered.get_future().wait_for(100ms);
  if (entered!=std::future_status::ready) {
    release_first.set_value();(void)first.get();
    FAIL()<<"First clock provider did not acquire the gate within the bounded fixture wait";
    return;
  }
  std::atomic<bool> second_provider_called{false};
  auto second=std::async(std::launch::async,[&]{
    return gate.stillEligible(token,[&]{second_provider_called=true;return source.load();});
  });
  std::this_thread::sleep_for(5ms);
  const bool called_while_locked=second_provider_called.load();
  source=300;release_first.set_value();
  EXPECT_FALSE(called_while_locked);EXPECT_FALSE(first.get());EXPECT_FALSE(second.get());
  EXPECT_TRUE(second_provider_called.load());
}

TEST(CompletionGate, SteadyDeadlineIsCheckedAfterTheSourceClockProviderReturns) {
  using namespace std::chrono_literals;
  goal_region::CompletionGate gate;const auto binding=bound(gate);
  gate.finishCycle(gate.beginCycle(binding->generation),true,
    std::chrono::steady_clock::now()+15ms,300,100);
  const auto token=gate.snapshot();ASSERT_TRUE(token.ready);
  EXPECT_FALSE(gate.stillEligible(token,[]{std::this_thread::sleep_for(25ms);return 200;}));
}

TEST(P1StopHold, CallbackContinuityRevocationRetainsHoldAndRestartsBothOriginalDwellClocks) {
  using H=goal_region::TerminalStopHold;H hold;
  const auto sample=[&](std::int64_t source,std::int64_t steady){
    return hold.observe(1,true,true,true,true,source,steady);
  };
  EXPECT_EQ(sample(1000000000,2000000000),H::Action::HoldZero);
  EXPECT_EQ(sample(1050000000,2050000000),H::Action::HoldZero);
  EXPECT_EQ(sample(1350000000,2350000000),H::Action::Ready);
  hold.invalidateStopContinuity();ASSERT_TRUE(hold.holding());
  EXPECT_EQ(sample(1400000000,2400000000),H::Action::HoldZero);
  EXPECT_EQ(sample(1699999999,2700000000),H::Action::HoldZero);
  EXPECT_EQ(sample(1700000000,2700000000),H::Action::Ready);
}


TEST(P2StopHold, InnerAimIsOnlyAnEntryGateAndDoesNotReplaceOriginalToleranceOrAdvancingDwell) {
  using Hold=goal_region::TerminalStopHold;using A=Hold::Action;Hold h;
  EXPECT_EQ(h.observe(1,true,true,true,true,1000000000,2000000000,false),A::Track);
  EXPECT_FALSE(h.holding());
  EXPECT_EQ(h.observe(1,true,true,true,true,1050000000,2050000000,true),A::HoldZero);
  EXPECT_EQ(h.observe(1,true,true,true,true,1100000000,2100000000,false),A::HoldZero);
  EXPECT_EQ(h.observe(1,true,true,true,true,1399999999,2399999999,false),A::HoldZero);
  EXPECT_EQ(h.observe(1,true,true,true,true,1400000000,2400000000,false),A::Ready);
  EXPECT_EQ(h.observe(2,false,true,true,true,1450000000,2450000000,true),A::Track);
  EXPECT_FALSE(h.holding());
}

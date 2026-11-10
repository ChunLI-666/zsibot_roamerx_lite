#include <gtest/gtest.h>
#include <limits>
#include "matrix_recovery_bt/stop_barrier.hpp"
using namespace matrix_recovery_bt;
TEST(StopBarrier, FreshFiveHundredHzStationaryQualifies) {
  StopBarrier b;b.arm(1000000000,1000000000);
  for (int i=1;i<=151;++i) {b.sample(1000000000+i*2000000LL,1000000000+i*2000000LL,true,0,0,0);}
  EXPECT_TRUE(b.ready(1302000000));
  EXPECT_FALSE(b.ready(1604000000));
}
TEST(StopBarrier, MovingA12DoesNotQualifyAfterOneSecond) {
  StopBarrier b;b.arm(1000000000,1000000000);
  for (int i=1;i<=600;++i) {EXPECT_FALSE(b.sample(1000000000+i*2000000LL,1000000000+i*2000000LL,true,-.0717,.1007,.0125));}
}
TEST(StopBarrier, RepeatedAndBackwardAndGappedInputsReset) {
  for (auto delta:{0LL,-1LL,400000000LL}) {
    StopBarrier b;b.arm(1000000000,1000000000);
    b.sample(1100000000,1100000000,true,0,0,0);
    EXPECT_FALSE(b.sample(1100000000+delta,1200000000,true,0,0,0));
    EXPECT_FALSE(b.ready(1200000000));
  }
}
TEST(StopBarrier, SteadyAndSourceBothRequired) {
  StopBarrier b;b.arm(1000000000,1000000000);
  b.sample(1100000000,1100000000,true,0,0,0);
  EXPECT_FALSE(b.sample(1400000000,1100000001,true,0,0,0));
  EXPECT_FALSE(b.sample(1500000000,1500000000,false,0,0,0));
  EXPECT_FALSE(b.sample(1600000000,1600000000,true,std::numeric_limits<double>::quiet_NaN(),0,0));
}
TEST(ZeroEpoch, ReleaseAndLateCallbacksCannotZeroController) {
  ZeroEpoch s;auto first=s.acquire();EXPECT_TRUE(s.zeroAllowed(first));
  EXPECT_TRUE(s.release(first));EXPECT_FALSE(s.zeroAllowed(first));
  EXPECT_FALSE(s.close(first));EXPECT_TRUE(s.release(first));
  auto second=s.acquire();EXPECT_FALSE(s.release(first));EXPECT_FALSE(s.close(first));
  EXPECT_FALSE(s.zeroAllowed(first));EXPECT_TRUE(s.zeroAllowed(second));
  EXPECT_TRUE(s.close(second));EXPECT_FALSE(s.release(second));
}

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "stretch_guarded_executor/guard.hpp"
#include "stretch_guarded_executor/lift_sequence.hpp"

using namespace stretch_guarded_executor;

TEST(LiftSequenceTest, LimitsAndInvalidNumbers) {
  LiftSequence sequence;
  EXPECT_TRUE(sequence.start(.5, 0., 0., 1.0994316458886308, .01, .0025).ok);
  sequence.cancel();
  EXPECT_TRUE(
      sequence.start(.5, 1.0994316458886308, 0., 1.0994316458886308,
                     .01, .001)
          .ok);
  sequence.cancel();
  EXPECT_FALSE(sequence.start(.5, -.0001, 0., 1.1, .01, .001).ok);
  EXPECT_FALSE(sequence.start(.5, 1.1001, 0., 1.1, .01, .001).ok);
  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity(),
                     -std::numeric_limits<double>::infinity()})
    EXPECT_FALSE(sequence.start(.5, bad, 0., 1.1, .01, .001).ok);
  EXPECT_FALSE(sequence.start(.5, .6, 0., 1.1, .01, .0031).ok);
}

TEST(LiftSequenceTest, RequestedDistancesAndDirections) {
  for (const auto& item :
       std::vector<std::pair<double, size_t>>{{.001, 1},  {-.001, 1},
                                               {.01, 1},   {-.01, 1},
                                               {.08, 8},   {-.08, 8},
                                               {.023, 3},  {-.023, 3},
                                               {.030, 3},  {-.030, 3}}) {
    const auto targets =
        LiftSequence::nominalTargets(.5, .5 + item.first, .01, .0001);
    ASSERT_EQ(targets.size(), item.second) << item.first;
    EXPECT_NEAR(targets.back(), .5 + item.first, 1e-12);
    double previous = .5;
    for (double target : targets) {
      EXPECT_LE(std::abs(target - previous), .010000000001);
      EXPECT_EQ(target > previous, item.first > 0);
      previous = target;
    }
  }
}

TEST(LiftSequenceTest, AlreadyAtTarget) {
  LiftSequence sequence;
  auto result = sequence.start(.5, .5, 0., 1.1, .01, .001);
  EXPECT_TRUE(result.ok);
  EXPECT_FALSE(sequence.active());
  EXPECT_TRUE(sequence.satisfied(.5));
  EXPECT_EQ(sequence.totalSegments(), 0u);
}

TEST(LiftSequenceTest, FloatingPointEndpointIsExact) {
  const auto targets =
      LiftSequence::nominalTargets(.449, .472, .01, .0001);
  ASSERT_EQ(targets.size(), 3u);
  EXPECT_NEAR(targets[0], .459, 1e-15);
  EXPECT_NEAR(targets[1], .469, 1e-15);
  EXPECT_DOUBLE_EQ(targets[2], .472);
}

TEST(LiftSequenceTest, FreshMeasuredStateDrivesEveryNextSegment) {
  LiftSequence sequence;
  ASSERT_TRUE(sequence.start(.45, .53, 0., 1.1, .01, .001).ok);
  double measured = .45;
  std::vector<double> commands;
  while (sequence.active()) {
    auto target = sequence.next(measured);
    ASSERT_TRUE(target.has_value());
    commands.push_back(*target);
    // Mock controller reaches each requested absolute endpoint.
    measured = *target;
    sequence.segmentComplete(measured);
  }
  ASSERT_EQ(commands.size(), 8u);
  EXPECT_DOUBLE_EQ(commands.back(), .53);
  EXPECT_TRUE(sequence.satisfied(measured));

  ASSERT_TRUE(sequence.start(.53, .45, 0., 1.1, .01, .001).ok);
  measured = .53;
  commands.clear();
  while (sequence.active()) {
    auto target = sequence.next(measured);
    ASSERT_TRUE(target.has_value());
    commands.push_back(*target);
    measured = *target;
    sequence.segmentComplete(measured);
  }
  ASSERT_EQ(commands.size(), 8u);
  EXPECT_DOUBLE_EQ(commands.back(), .45);
}

TEST(LiftSequenceTest, EndpointBiasCannotAccumulateOrOvershoot) {
  LiftSequence sequence;
  ASSERT_TRUE(sequence.start(.449, .472, 0., 1.1, .01, .0001).ok);
  double measured = .449;
  int iterations = 0;
  while (sequence.active() && iterations++ < 10) {
    auto target = sequence.next(measured);
    ASSERT_TRUE(target.has_value());
    EXPECT_LE(std::abs(*target - measured), .010000000001);
    // Mock a repeatable 20 um undershoot. The final endpoint remains absolute.
    measured = *target - .00002;
    sequence.segmentComplete(measured);
  }
  EXPECT_FALSE(sequence.active());
  EXPECT_NEAR(measured, .472, .0001);
}

TEST(LiftSequenceTest, CancellationAndDoubleExecution) {
  LiftSequence sequence;
  ASSERT_TRUE(sequence.start(.45, .53, 0., 1.1, .01, .001).ok);
  EXPECT_FALSE(sequence.start(.45, .6, 0., 1.1, .01, .001).ok);
  sequence.cancel();
  EXPECT_FALSE(sequence.active());
  EXPECT_TRUE(sequence.canceled());
  EXPECT_FALSE(sequence.next(.45).has_value());
}

TEST(LiftSequenceTest, IntermediateControllerFailureStopsFollowingSegments) {
  LiftSequence sequence;
  ASSERT_TRUE(sequence.start(.45, .53, 0., 1.1, .01, .001).ok);
  auto first = sequence.next(.45);
  ASSERT_TRUE(first.has_value());
  sequence.segmentComplete(*first);
  ASSERT_TRUE(sequence.next(*first).has_value());
  sequence.fail("mock controller abort");
  EXPECT_FALSE(sequence.active());
  EXPECT_TRUE(sequence.failed());
  EXPECT_FALSE(sequence.next(*first).has_value());
}

TEST(LiftSequenceTest, EveryMockPhysicalTrajectoryIsLiftOnly) {
  LiftSequence sequence;
  ASSERT_TRUE(sequence.start(.45, .53, 0., 1.1, .01, .001).ok);
  double measured = .45;
  while (sequence.active()) {
    const double target = *sequence.next(measured);
    Plan plan;
    plan.start.setZero();
    plan.target.setZero();
    plan.start[0] = measured;
    plan.target[0] = target;
    plan.duration = 5.;
    const auto trajectory = Guard::trajectory(plan, true);
    EXPECT_EQ(trajectory.joint_names,
              std::vector<std::string>{"joint_lift"});
    measured = target;
    sequence.segmentComplete(measured);
  }
}

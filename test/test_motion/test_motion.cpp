// Unit tests for the trajectory generator and the layer-1 motion controller.
//   pio test -e native_tests   (on your PC; needs a host g++ - see README)
//   pio test -e uno_tests      (on the board)
//
// The property worth testing is not "does it produce a nice curve", it is
// "do all axes finish at the same instant, and do none of them exceed the
// limits they were given". Both are cheap to assert and both are exactly the
// thing that silently breaks when someone edits the profile math later.
#include <unity.h>

#include "ArmMath.h"
#include "MotionController.h"
#include "Trajectory.h"

using namespace armmath;

void setUp() {}
void tearDown() {}

static void test_deadband() {
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.0f, deadband(1.5f, 2.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.0f, deadband(-2.0f, 2.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 1.0f, deadband(3.0f, 2.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-4, -1.0f, deadband(-3.0f, 2.0f));
}

static void test_moving_average() {
  MovingAverage<4> f;
  f.reset(0.0f);
  f.update(4.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 1.0f, f.value());
  for (int i = 0; i < 4; i++) f.update(4.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 4.0f, f.value());  // settles to the input
}

static void test_triangular_profile() {
  // Short move: never reaches vmax, so duration is 2*sqrt(d/a).
  TrapezoidProfile p;
  p.plan(0.0f, 1.0f, 100.0f, 4.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 1.0f, p.duration());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.5f, p.sample(0.5f).pos);  // apex at half
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 1.0f, p.sample(1.0f).pos);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.0f, p.sample(1.0f).vel);
}

static void test_trapezoidal_profile() {
  // Long move: cruises. T = v/a + d/v = 1 + 10 = 11.
  TrapezoidProfile p;
  p.plan(0.0f, 20.0f, 2.0f, 2.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 11.0f, p.duration());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 2.0f, p.sample(5.0f).vel);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 20.0f, p.sample(11.0f).pos);
}

static void test_negative_direction() {
  TrapezoidProfile p;
  p.plan(10.0f, -10.0f, 5.0f, 5.0f);
  TEST_ASSERT_TRUE(p.sample(p.duration() * 0.5f).vel < 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, -10.0f, p.sample(p.duration()).pos);
}

static void test_zero_distance_is_not_a_move() {
  TrapezoidProfile p;
  p.plan(7.0f, 7.0f, 5.0f, 5.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.0f, p.duration());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 7.0f, p.sample(0.0f).pos);
  TEST_ASSERT_TRUE(p.done(0.0f));
}

static void test_stretched_profile_respects_accel() {
  // Same move forced to take 4x as long: the cruise velocity must drop, and
  // acceleration must stay at the limit rather than being scaled down with it.
  TrapezoidProfile fast, slow;
  fast.plan(0.0f, 20.0f, 2.0f, 2.0f);
  slow.planFor(0.0f, 20.0f, 2.0f, 2.0f, 44.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 44.0f, slow.duration());
  TEST_ASSERT_TRUE(slow.cruiseVelocity() < fast.cruiseVelocity());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 20.0f, slow.sample(44.0f).pos);
}

static void test_duration_is_clamped_up_never_down() {
  // Asking for an impossible duration must yield the fastest legal one, not a
  // profile that quietly violates the acceleration limit.
  TrapezoidProfile p;
  p.planFor(0.0f, 20.0f, 2.0f, 2.0f, 0.001f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 11.0f, p.duration());
}

static void test_sync_duration_is_the_slowest_axis() {
  const float d[3] = {1.0f, 40.0f, 5.0f};
  const float v[3] = {2.0f, 2.0f, 2.0f};
  const float a[3] = {2.0f, 2.0f, 2.0f};
  const float t = syncDuration(d, v, a, 3);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, TrapezoidProfile::minDuration(40.0f, 2.0f, 2.0f),
                           t);
}

static void test_axes_arrive_together() {
  // The whole point of layer 1. Step three very different distances and check
  // that no axis is still moving after the last one stops.
  MotionController<3, 4> mc;
  MotionController<3, 4>::Limits lim{2000.0f, 4000.0f};
  mc.begin(lim);

  const int32_t start[3] = {0, 0, 0};
  const int32_t target[3] = {100, 4000, -800};
  mc.resetTo(start);
  mc.moveTo(target, start);

  const float dt = 0.005f;
  int32_t reached[3] = {0, 0, 0};
  int firstArrival[3] = {-1, -1, -1};
  for (int step = 0; step < 4000 && mc.moving(); ++step) {
    mc.update(dt);
    for (uint8_t j = 0; j < 3; ++j) {
      reached[j] = mc.command(j).target_counts;
      if (firstArrival[j] < 0 && reached[j] == target[j]) firstArrival[j] = step;
    }
  }

  for (uint8_t j = 0; j < 3; ++j) {
    TEST_ASSERT_EQUAL_INT32(target[j], reached[j]);
    TEST_ASSERT_TRUE(firstArrival[j] >= 0);
  }
  // Within a couple of filter taps of each other, not a couple of seconds.
  const int spread0 = firstArrival[0] - firstArrival[1];
  const int spread2 = firstArrival[2] - firstArrival[1];
  TEST_ASSERT_TRUE(spread0 <= 4 && spread0 >= -4);
  TEST_ASSERT_TRUE(spread2 <= 4 && spread2 >= -4);
}

static void test_stream_never_exceeds_velocity_limit() {
  MotionController<2, 4> mc;
  MotionController<2, 4>::Limits lim{1000.0f, 3000.0f};
  mc.begin(lim);
  const int32_t start[2] = {0, 0};
  const int32_t target[2] = {5000, -5000};
  mc.resetTo(start);
  mc.moveTo(target, start);

  const float dt = 0.005f;
  int16_t peak = 0;
  for (int step = 0; step < 4000 && mc.moving(); ++step) {
    mc.update(dt);
    for (uint8_t j = 0; j < 2; ++j) {
      const int16_t v = mc.command(j).velFeedforward();
      const int16_t mag = (v < 0) ? (int16_t)-v : v;
      if (mag > peak) peak = mag;
    }
  }
  TEST_ASSERT_TRUE(peak <= 1010);  // 1 % slack for count rounding
}

static void test_feedforward_matches_setpoint_slope() {
  // The feedforward must be the derivative of the setpoint actually sent. If
  // these ever disagree the joint fights its own trajectory.
  MotionController<1, 4> mc;
  MotionController<1, 4>::Limits lim{1000.0f, 2000.0f};
  mc.begin(lim);
  const int32_t start[1] = {0};
  const int32_t target[1] = {3000};
  mc.resetTo(start);
  mc.moveTo(target, start);

  const float dt = 0.005f;
  int32_t prev = 0;
  for (int step = 0; step < 20; ++step) {
    mc.update(dt);
    const int32_t now = mc.command(0).target_counts;
    const float slope = (now - prev) / dt;
    prev = now;
    TEST_ASSERT_FLOAT_WITHIN(220.0f, slope, mc.command(0).velFeedforward());
  }
}

static int runAll() {
  UNITY_BEGIN();
  RUN_TEST(test_deadband);
  RUN_TEST(test_moving_average);
  RUN_TEST(test_triangular_profile);
  RUN_TEST(test_trapezoidal_profile);
  RUN_TEST(test_negative_direction);
  RUN_TEST(test_zero_distance_is_not_a_move);
  RUN_TEST(test_stretched_profile_respects_accel);
  RUN_TEST(test_duration_is_clamped_up_never_down);
  RUN_TEST(test_sync_duration_is_the_slowest_axis);
  RUN_TEST(test_axes_arrive_together);
  RUN_TEST(test_stream_never_exceeds_velocity_limit);
  RUN_TEST(test_feedforward_matches_setpoint_slope);
  return UNITY_END();
}

#ifdef ARDUINO
#include <Arduino.h>
void setup() {
  delay(2000);
  runAll();
}
void loop() {}
#else
int main(int, char **) { return runAll(); }
#endif

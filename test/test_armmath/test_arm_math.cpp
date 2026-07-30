// Unit tests for ArmMath.
//   pio test -e native_tests   (on your PC; needs a host g++ - see README)
//   pio test -e uno_tests      (on the board, once hardware arrives)
//
// These exist so you can develop and verify the tricky math (angle wrapping,
// multi-turn unwrapping) while the hardware is still in transit.
#include <unity.h>

#include "ArmMath.h"

using namespace armmath;

void setUp() {}
void tearDown() {}

static void test_wrap360() {
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 10.0f, wrap360(370.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 350.0f, wrap360(-10.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.0f, wrap360(720.0f));
}

static void test_wrap180() {
  TEST_ASSERT_FLOAT_WITHIN(1e-3, -10.0f, wrap180(350.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 90.0f, wrap180(90.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3, -180.0f, wrap180(180.0f));
}

static void test_shortest_delta() {
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 20.0f, shortestDelta(350.0f, 10.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3, -20.0f, shortestDelta(10.0f, 350.0f));
}

static void test_counts_conversion() {
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.0f, countsToDeg(0));
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 180.0f, countsToDeg(2048));
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 359.912f, countsToDeg(4095));
}

static void test_clamp_and_slew() {
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 5.0f, clampf(9.0f, -5.0f, 5.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 2.0f, slew(0.0f, 10.0f, 2.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 10.0f, slew(9.0f, 10.0f, 2.0f));
}

static void test_unwrap_forward_through_zero() {
  AngleUnwrapper u;
  u.reset(350.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 355.0f, u.update(355.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 365.0f, u.update(5.0f));
  TEST_ASSERT_EQUAL_INT32(1, u.turns());
}

static void test_unwrap_backward_through_zero() {
  AngleUnwrapper u;
  u.reset(5.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-2, -5.0f, u.update(355.0f));
  TEST_ASSERT_EQUAL_INT32(-1, u.turns());
}

static void test_unwrap_multi_turn_sweep() {
  AngleUnwrapper u;
  u.reset(0.0f);
  float last = 0.0f;
  for (int i = 1; i <= 100; i++) last = u.update(wrap360(i * 30.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-1, 3000.0f, last);  // 100 * 30 deg
}

static int runAll() {
  UNITY_BEGIN();
  RUN_TEST(test_wrap360);
  RUN_TEST(test_wrap180);
  RUN_TEST(test_shortest_delta);
  RUN_TEST(test_counts_conversion);
  RUN_TEST(test_clamp_and_slew);
  RUN_TEST(test_unwrap_forward_through_zero);
  RUN_TEST(test_unwrap_backward_through_zero);
  RUN_TEST(test_unwrap_multi_turn_sweep);
  return UNITY_END();
}

#ifdef ARDUINO
#include <Arduino.h>
void setup() {
  delay(2000);  // let the host open the serial port
  runAll();
}
void loop() {}
#else
int main(int, char **) { return runAll(); }
#endif

// Unit tests for the host-link framing.
//   pio test -e native_tests
//
// Framing bugs are the worst class of bug to debug on hardware: they present
// as "the arm twitched once and stopped", which looks like a control problem.
// Prove the layer here, on the host, where a failure prints a line number.
#include <string.h>
#include <unity.h>

#include "JointProtocol.h"
#include "PacketFraming.h"

void setUp() {}
void tearDown() {}

static void test_crc_detects_single_bit_flips() {
  uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  const uint8_t good = link::crc8(data, 8);
  for (uint8_t bit = 0; bit < 8; ++bit) {
    data[3] ^= (uint8_t)(1 << bit);
    TEST_ASSERT_NOT_EQUAL(good, link::crc8(data, 8));
    data[3] ^= (uint8_t)(1 << bit);
  }
  TEST_ASSERT_EQUAL_UINT8(good, link::crc8(data, 8));
}

// The host tool (tools/joint_link.py) and this firmware must produce byte-
// identical frames, and the only way to be sure is to check BOTH against the
// same third party rather than against each other. Round-trip tests cannot
// catch the two implementations drifting together. These vectors are the
// published CRC-8/ATM check value and the test vectors from Cheshire & Baker,
// "Consistent Overhead Byte Stuffing"; joint_link.py asserts the same ones.
static void test_crc_matches_published_check_value() {
  // CRC-8/ATM: poly 0x07, init 0x00, no reflection, no final xor.
  // CRC of the ASCII string "123456789" is 0xF4.
  const uint8_t msg[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  TEST_ASSERT_EQUAL_UINT8(0xF4, link::crc8(msg, 9));
}

static void test_cobs_matches_reference_vectors() {
  uint8_t enc[16];

  const uint8_t v1[1] = {0x00};
  const uint8_t e1[2] = {0x01, 0x01};
  TEST_ASSERT_EQUAL_UINT8(2, link::cobsEncode(v1, 1, enc));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(e1, enc, 2);

  const uint8_t v2[3] = {0x00, 0x11, 0x00};
  const uint8_t e2[4] = {0x01, 0x02, 0x11, 0x01};
  TEST_ASSERT_EQUAL_UINT8(4, link::cobsEncode(v2, 3, enc));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(e2, enc, 4);

  const uint8_t v3[4] = {0x11, 0x22, 0x00, 0x33};
  const uint8_t e3[5] = {0x03, 0x11, 0x22, 0x02, 0x33};
  TEST_ASSERT_EQUAL_UINT8(5, link::cobsEncode(v3, 4, enc));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(e3, enc, 5);

  const uint8_t v4[4] = {0x11, 0x22, 0x33, 0x44};
  const uint8_t e4[5] = {0x05, 0x11, 0x22, 0x33, 0x44};
  TEST_ASSERT_EQUAL_UINT8(5, link::cobsEncode(v4, 4, enc));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(e4, enc, 5);

  const uint8_t v5[4] = {0x11, 0x00, 0x00, 0x00};
  const uint8_t e5[5] = {0x02, 0x11, 0x01, 0x01, 0x01};
  TEST_ASSERT_EQUAL_UINT8(5, link::cobsEncode(v5, 4, enc));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(e5, enc, 5);
}

static void test_cobs_round_trip_with_zeros() {
  // Zeros are the whole reason COBS exists: a JointCommand for a joint near
  // its origin is mostly zero bytes.
  const uint8_t in[9] = {0, 0, 1, 0, 2, 0, 0, 3, 0};
  uint8_t enc[16], dec[16];
  const uint8_t n = link::cobsEncode(in, 9, enc);
  for (uint8_t i = 0; i < n; ++i) TEST_ASSERT_NOT_EQUAL(0, enc[i]);
  TEST_ASSERT_EQUAL_UINT8(9, link::cobsDecode(enc, n, dec, sizeof(dec)));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(in, dec, 9);
}

static void test_cobs_overhead_is_one_byte() {
  const uint8_t in[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  uint8_t enc[16];
  TEST_ASSERT_EQUAL_UINT8(9, link::cobsEncode(in, 8, enc));
}

static void test_frame_round_trip() {
  jointnode::JointCommand cmd{};
  cmd.target_counts = -12345;
  cmd.mode = jointnode::MODE_TRACK;
  cmd.flags = jointnode::CMD_ENABLE;
  cmd.setVelFeedforward(-777);

  uint8_t frame[link::kMaxFrame];
  const uint8_t n = link::buildFrame(link::PKT_CMD, 2, (const uint8_t *)&cmd,
                                     sizeof(cmd), frame);
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_EQUAL_UINT8(0, frame[n - 1]);

  link::PacketReader r;
  bool got = false;
  for (uint8_t i = 0; i < n; ++i) got = r.feed(frame[i]);
  TEST_ASSERT_TRUE(got);
  TEST_ASSERT_EQUAL_UINT8(link::PKT_CMD, r.type());
  TEST_ASSERT_EQUAL_UINT8(2, r.node());
  TEST_ASSERT_EQUAL_UINT8(sizeof(cmd), r.payloadLen());

  jointnode::JointCommand out{};
  memcpy(&out, r.payload(), sizeof(out));
  TEST_ASSERT_EQUAL_INT32(-12345, out.target_counts);
  TEST_ASSERT_EQUAL_INT16(-777, out.velFeedforward());
  TEST_ASSERT_EQUAL_UINT8(jointnode::MODE_TRACK, out.mode);
}

static void test_corrupted_frame_is_rejected() {
  uint8_t payload[8] = {9, 8, 7, 6, 5, 4, 3, 2};
  uint8_t frame[link::kMaxFrame];
  const uint8_t n =
      link::buildFrame(link::PKT_STATE, 1, payload, sizeof(payload), frame);
  frame[4] ^= 0x20;

  link::PacketReader r;
  bool got = false;
  for (uint8_t i = 0; i < n; ++i) got = r.feed(frame[i]) || got;
  TEST_ASSERT_FALSE(got);
  TEST_ASSERT_EQUAL_UINT16(1, r.crcErrors());
}

static void test_reader_resyncs_after_garbage() {
  // Drop a receiver into the middle of a byte stream and it must recover on
  // the next delimiter, not stay wedged. This is the property CSV lacks.
  uint8_t payload[8] = {1, 0, 3, 0, 5, 0, 7, 0};
  uint8_t frame[link::kMaxFrame];
  const uint8_t n =
      link::buildFrame(link::PKT_STATE, 0, payload, sizeof(payload), frame);

  link::PacketReader r;
  for (uint8_t i = 3; i < n; ++i) r.feed(frame[i]);  // truncated frame
  bool got = false;
  for (uint8_t i = 0; i < n; ++i) got = r.feed(frame[i]);
  TEST_ASSERT_TRUE(got);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, r.payload(), 8);
}

static void test_oversized_payload_is_refused() {
  uint8_t big[link::kMaxPayload + 1] = {};
  uint8_t frame[link::kMaxFrame];
  TEST_ASSERT_EQUAL_UINT8(
      0, link::buildFrame(link::PKT_LOG, 0, big, sizeof(big), frame));
}

static void test_status_bits_are_not_faults() {
  jointnode::JointState s{};
  s.fault = jointnode::STATUS_IN_POSITION;
  TEST_ASSERT_TRUE(s.healthy());
  TEST_ASSERT_TRUE(s.inPosition());
  s.fault |= jointnode::FAULT_ENCODER;
  TEST_ASSERT_FALSE(s.healthy());
}

static int runAll() {
  UNITY_BEGIN();
  RUN_TEST(test_crc_detects_single_bit_flips);
  RUN_TEST(test_crc_matches_published_check_value);
  RUN_TEST(test_cobs_matches_reference_vectors);
  RUN_TEST(test_cobs_round_trip_with_zeros);
  RUN_TEST(test_cobs_overhead_is_one_byte);
  RUN_TEST(test_frame_round_trip);
  RUN_TEST(test_corrupted_frame_is_rejected);
  RUN_TEST(test_reader_resyncs_after_garbage);
  RUN_TEST(test_oversized_payload_is_refused);
  RUN_TEST(test_status_bits_are_not_faults);
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

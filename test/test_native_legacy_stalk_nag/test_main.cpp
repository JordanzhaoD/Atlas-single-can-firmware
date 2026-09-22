#include <unity.h>

#include "can_helpers.h"
#include "dash_jitter_procedure.h"
#include "dash_legacy_stalk_nag.h"
#include "drivers/mock_driver.h"

static DashLegacyStalkNag nag;
static MockDriver driver;

// 0x045 integrity byte is CRC8-J1850 over bytes 0..6 (jitter_045 field
// format) — NOT the additive vehicle checksum used by 0x370/0x3EE.
static CanFrame neutral045(uint8_t profileByte = 0x42)
{
    CanFrame frame{.id = 0x045, .dlc = 8};
    frame.data[0] = 0x00;
    frame.data[1] = profileByte;
    frame.data[2] = 0x12;
    frame.data[3] = 0x34;
    frame.data[4] = 0x56;
    frame.data[5] = 0x78;
    frame.data[6] = 0x90;
    frame.data[7] = jitter_045::crc8J1850(frame.data, 7);
    return frame;
}

void setUp()
{
    nag = DashLegacyStalkNag{};
    driver.reset();
    nag.setEnabled(true);
}

void tearDown() {}

void test_only_captured_6_to_1_hands3_edge_arms_press()
{
    nag.onDasStatus(6, 1, 1000);
    nag.onDasStatus(1, 3, 1100);
    DashLegacyStalkNagDiag d = nag.diag();
    TEST_ASSERT_EQUAL_UINT32(1, d.warningEdges);
    TEST_ASSERT_TRUE(d.pressPending);

    CanFrame frame = neutral045();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DashLegacyStalkNagTxResult::Sent),
        static_cast<uint8_t>(nag.onStalkFrame(frame, driver, 1110)));
    TEST_ASSERT_EQUAL_UINT32(1, driver.sent.size());
    TEST_ASSERT_EQUAL_HEX8(2, driver.sent[0].data[0] & 0x3F);
    TEST_ASSERT_EQUAL_HEX8(frame.data[1], driver.sent[0].data[1]);
    TEST_ASSERT_EQUAL_HEX8(jitter_045::crc8J1850(driver.sent[0].data, 7), driver.sent[0].data[7]);
    // Regression pin: the additive vehicle checksum must NOT validate 0x045.
    // If this ever flips true again the fixture has drifted off the field
    // format (the v1.20.10 review found exactly this bug shipped green).
    TEST_ASSERT_FALSE(computeVehicleChecksum(driver.sent[0]) == driver.sent[0].data[7]);
    // The rolling counter is cloned from the source frame as-is.
    TEST_ASSERT_EQUAL_HEX8(frame.data[6], driver.sent[0].data[6]);
}

void test_press_is_followed_by_one_neutral_release()
{
    nag.onDasStatus(6, 1, 1000);
    nag.onDasStatus(1, 3, 1100);
    CanFrame frame = neutral045();
    nag.onStalkFrame(frame, driver, 1110);
    nag.onStalkFrame(frame, driver, 1120);
    nag.onStalkFrame(frame, driver, 1130);

    TEST_ASSERT_EQUAL_UINT32(2, driver.sent.size());
    TEST_ASSERT_EQUAL_HEX8(2, driver.sent[0].data[0] & 0x3F);
    TEST_ASSERT_EQUAL_HEX8(0, driver.sent[1].data[0] & 0x3F);
    DashLegacyStalkNagDiag d = nag.diag();
    TEST_ASSERT_EQUAL_UINT32(1, d.pressTxOk);
    TEST_ASSERT_EQUAL_UINT32(1, d.releaseTxOk);
    TEST_ASSERT_FALSE(d.pressPending);
    TEST_ASSERT_FALSE(d.releasePending);
}

void test_recovery_tail_does_not_retrigger()
{
    nag.onDasStatus(6, 1, 1000);
    nag.onDasStatus(1, 3, 1100);
    CanFrame frame = neutral045();
    nag.onStalkFrame(frame, driver, 1110);
    nag.onStalkFrame(frame, driver, 1120);

    nag.onDasStatus(2, 2, 1200);
    nag.onDasStatus(6, 1, 1300);
    nag.onStalkFrame(frame, driver, 1400);
    TEST_ASSERT_EQUAL_UINT32(2, driver.sent.size());
    TEST_ASSERT_EQUAL_UINT32(1, nag.diag().warningEdges);
}

void test_rearms_only_after_clear_debounce_and_cooldown()
{
    nag.onDasStatus(6, 1, 1000);
    nag.onDasStatus(1, 3, 1100);
    CanFrame frame = neutral045();
    nag.onStalkFrame(frame, driver, 1110);
    nag.onStalkFrame(frame, driver, 1120);

    nag.onDasStatus(2, 2, 1200);
    nag.onDasStatus(6, 1, 12000);
    nag.onDasStatus(6, 1, 12501);
    TEST_ASSERT_TRUE(nag.diag().armed);
    nag.onDasStatus(1, 3, 12600);
    nag.onStalkFrame(frame, driver, 12610);
    TEST_ASSERT_EQUAL_UINT32(3, driver.sent.size());
    TEST_ASSERT_EQUAL_UINT32(2, nag.diag().warningEdges);
}

void test_wrong_ap_edge_and_wrong_hands_do_not_trigger()
{
    nag.onDasStatus(4, 1, 1000);
    nag.onDasStatus(1, 3, 1100);
    nag.onDasStatus(6, 1, 1200);
    nag.onDasStatus(1, 2, 1300);
    CanFrame frame = neutral045();
    nag.onStalkFrame(frame, driver, 1310);
    TEST_ASSERT_TRUE(driver.sent.empty());
    TEST_ASSERT_EQUAL_UINT32(0, nag.diag().warningEdges);
}

void test_physical_stalk_and_bad_checksum_are_never_overwritten()
{
    nag.onDasStatus(6, 1, 1000);
    nag.onDasStatus(1, 3, 1100);

    CanFrame physical = neutral045();
    physical.data[0] = 1;
    physical.data[7] = jitter_045::crc8J1850(physical.data, 7);
    nag.onStalkFrame(physical, driver, 1110);
    TEST_ASSERT_TRUE(driver.sent.empty());
    TEST_ASSERT_TRUE(nag.diag().pressPending);

    CanFrame bad = neutral045();
    bad.data[7] ^= 0x55;
    nag.onStalkFrame(bad, driver, 1120);
    TEST_ASSERT_TRUE(driver.sent.empty());
    TEST_ASSERT_TRUE(nag.diag().pressPending);
}

void test_profile_byte_is_not_changed_by_press_or_release()
{
    nag.onDasStatus(6, 1, 1000);
    nag.onDasStatus(1, 3, 1100);
    CanFrame frame = neutral045(0xA6);
    nag.onStalkFrame(frame, driver, 1110);
    nag.onStalkFrame(frame, driver, 1120);
    TEST_ASSERT_EQUAL_HEX8(0xA6, driver.sent[0].data[1]);
    TEST_ASSERT_EQUAL_HEX8(0xA6, driver.sent[1].data[1]);
}

void test_outer_gate_blocks_and_drops_pending_action()
{
    nag.onDasStatus(6, 1, 1000);
    nag.onDasStatus(1, 3, 1100);
    TEST_ASSERT_TRUE(nag.hasPendingAction());

    CanFrame frame = neutral045();
    // Outer gates closed (vehicle OTA session active / abort guard
    // latched): the pending press is dropped, not deferred.
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DashLegacyStalkNagTxResult::None),
        static_cast<uint8_t>(nag.onStalkFrame(frame, driver, 1110, false)));
    TEST_ASSERT_TRUE(driver.sent.empty());
    DashLegacyStalkNagDiag d = nag.diag();
    TEST_ASSERT_EQUAL_UINT32(1, d.blockedOuterGate);
    TEST_ASSERT_FALSE(d.pressPending);
    TEST_ASSERT_FALSE(d.releasePending);
    TEST_ASSERT_EQUAL_STRING("outerGate", d.blockedReason);
    TEST_ASSERT_FALSE(nag.hasPendingAction());

    // Gate reopened: no stale press fires — the episode was dropped.
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DashLegacyStalkNagTxResult::None),
        static_cast<uint8_t>(nag.onStalkFrame(frame, driver, 1120, true)));
    TEST_ASSERT_TRUE(driver.sent.empty());
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_only_captured_6_to_1_hands3_edge_arms_press);
    RUN_TEST(test_press_is_followed_by_one_neutral_release);
    RUN_TEST(test_recovery_tail_does_not_retrigger);
    RUN_TEST(test_rearms_only_after_clear_debounce_and_cooldown);
    RUN_TEST(test_wrong_ap_edge_and_wrong_hands_do_not_trigger);
    RUN_TEST(test_physical_stalk_and_bad_checksum_are_never_overwritten);
    RUN_TEST(test_profile_byte_is_not_changed_by_press_or_release);
    RUN_TEST(test_outer_gate_blocks_and_drops_pending_action);
    return UNITY_END();
}

#include <unity.h>

#include "dash_legacy_045_bit27_clone.h"

// 0x045 bit27 clone DND — pinned to the reference-firmware parameters:
// window handsOnState {2..5}, DAS autopilotState > 3, 1500 ms clone interval,
// inject on CAN1 (X197 13/14, the DAS/CH line). The clone copies the latest
// native 0x45 frame, sets bit27 (StW_Cond_Flt, data[3] bit3), bumps the
// rolling counter and re-CRCs. Never touches the scroll fields, never
// injects against states 6+ (strike-out territory is fail-closed).

static const uint8_t kIdleC[8] = {0x40, 0x21, 0x00, 0x30, 0x00, 0x00, 0xC0, 0xE0};

static void arm(DashLegacy045Clone27 &c)
{
    c.configure(true);
    c.setRuntimeAllowed(true, DashLegacy045Clone27Block::None);
}

// Seeds native 0x45 + a DAS status inside the window (apState 5, hands 2).
static void seedInWindow(DashLegacy045Clone27 &c, uint32_t nowMs = 1000,
                         uint8_t handsState = 2)
{
    c.onNative045(kIdleC, 8, nowMs);
    c.onDasStatus(5, handsState, CAN_BUS_VEH, nowMs);
}

static void feedNative(DashLegacy045Clone27 &c, uint8_t counterValue, uint32_t nowMs)
{
    uint8_t native[8];
    memcpy(native, kIdleC, 8);
    native[6] = static_cast<uint8_t>(counterValue << 4);
    native[7] = DashLegacy045Clone27::crc8J1850(native, 7);
    c.onNative045(native, 8, nowMs);
}

static void feedPhysical(DashLegacy045Clone27 &c, uint32_t nowMs)
{
    uint8_t physical[8];
    memcpy(physical, kIdleC, 8);
    physical[4] = 0x40; // human scroll tick
    physical[7] = DashLegacy045Clone27::crc8J1850(physical, 7);
    c.onNative045(physical, 8, nowMs);
}

void test_clone_frame_sets_bit27_bumps_counter_and_recomputes_crc()
{
    DashLegacy045Clone27 c;
    uint8_t out[8] = {};
    arm(c);
    seedInWindow(c, 1000, 2);
    TEST_ASSERT_TRUE(c.nextFrame(1000, out));
    // Everything matches the native frame except data[3] bit3, counter, CRC.
    TEST_ASSERT_EQUAL_HEX8(0x40, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x21, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x38, out[3]); // 0x30 | bit27 mask 0x08
    TEST_ASSERT_EQUAL_HEX8(0x00, out[4]); // scroll fields untouched
    TEST_ASSERT_EQUAL_HEX8(0x00, out[5]);
    TEST_ASSERT_EQUAL_HEX8(0xD0, out[6]); // counter C -> D, low nibble kept
    uint8_t expect[8];
    memcpy(expect, out, 8);
    expect[7] = DashLegacy045Clone27::crc8J1850(expect, 7);
    TEST_ASSERT_EQUAL_HEX8(expect[7], out[7]);
    TEST_ASSERT_TRUE(DashLegacy045Clone27::validFrame(out));
}

void test_window_admits_2_through_5_and_fails_closed_at_6()
{
    for (uint8_t hands = 0; hands <= 10; ++hands)
    {
        DashLegacy045Clone27 c;
        uint8_t out[8] = {};
        arm(c);
        c.onNative045(kIdleC, 8, 1000);
        c.onDasStatus(5, hands, CAN_BUS_VEH, 1000);
        const bool shouldSend = hands >= 2 && hands <= 5;
        TEST_ASSERT_EQUAL(shouldSend, c.nextFrame(1000, out));
        if (!shouldSend)
            TEST_ASSERT_EQUAL(DashLegacy045Clone27State::WaitingWindow, c.diag.state);
    }
}

void test_das_gate_requires_autopilot_state_above_3()
{
    for (uint8_t ap = 0; ap <= 8; ++ap)
    {
        DashLegacy045Clone27 c;
        uint8_t out[8] = {};
        arm(c);
        c.onNative045(kIdleC, 8, 1000);
        c.onDasStatus(ap, 3, CAN_BUS_VEH, 1000);
        const bool shouldSend = ap > 3;
        TEST_ASSERT_EQUAL(shouldSend, c.nextFrame(1000, out));
        if (!shouldSend)
            TEST_ASSERT_EQUAL(DashLegacy045Clone27State::WaitingWindow, c.diag.state);
    }
}

void test_window_entry_counted_on_rising_edge_only()
{
    DashLegacy045Clone27 c;
    uint8_t out[8] = {};
    arm(c);
    seedInWindow(c, 1000, 1); // below window
    TEST_ASSERT_EQUAL_UINT32(0, c.diag.windowEntries);
    c.onDasStatus(5, 2, CAN_BUS_VEH, 1100); // entry edge
    TEST_ASSERT_EQUAL_UINT32(1, c.diag.windowEntries);
    c.onDasStatus(5, 3, CAN_BUS_VEH, 1200); // inside, no new entry
    TEST_ASSERT_EQUAL_UINT32(1, c.diag.windowEntries);
    c.onDasStatus(5, 1, CAN_BUS_VEH, 1300); // exit
    c.onDasStatus(5, 2, CAN_BUS_VEH, 1400); // re-entry
    TEST_ASSERT_EQUAL_UINT32(2, c.diag.windowEntries);
}

void test_first_clone_is_immediate_then_1500ms_cooldown()
{
    DashLegacy045Clone27 c;
    uint8_t out[8] = {};
    arm(c);
    seedInWindow(c, 1000, 2);
    TEST_ASSERT_TRUE(c.nextFrame(1000, out)); // first frame immediate
    c.recordTxResult(true, 1000);
    TEST_ASSERT_EQUAL_UINT32(1, c.diag.cloneTx);

    // Native + 0x399 keep flowing; no second clone before the interval.
    feedNative(c, 0x0D, 1200);
    c.onDasStatus(5, 2, CAN_BUS_VEH, 1200);
    feedNative(c, 0x0D, 1500);
    c.onDasStatus(5, 2, CAN_BUS_VEH, 1500);
    TEST_ASSERT_FALSE(c.nextFrame(1500, out));
    TEST_ASSERT_EQUAL(DashLegacy045Clone27State::Cooldown, c.diag.state);

    feedNative(c, 0x0D, 2490); // fresh native just before the interval opens
    c.onDasStatus(5, 2, CAN_BUS_VEH, 2490);
    TEST_ASSERT_FALSE(c.nextFrame(2499, out));
    feedNative(c, 0x0D, 2500);
    c.onDasStatus(5, 2, CAN_BUS_VEH, 2500);
    TEST_ASSERT_TRUE(c.nextFrame(2500, out)); // 1000 + 1500
    c.recordTxResult(true, 2500);
    TEST_ASSERT_EQUAL_UINT32(2, c.diag.cloneTx);
}

void test_native_staleness_blocks_cloning()
{
    DashLegacy045Clone27 c;
    uint8_t out[8] = {};
    arm(c);
    seedInWindow(c, 1000, 2);
    TEST_ASSERT_TRUE(c.nextFrame(1000, out));
    c.recordTxResult(true, 1000);

    // Native 0x45 goes silent past the 150 ms freshness window (0x399 fresh).
    c.onDasStatus(5, 2, CAN_BUS_VEH, 2999);
    TEST_ASSERT_FALSE(c.nextFrame(3000, out));
    TEST_ASSERT_EQUAL(DashLegacy045Clone27State::WaitingSource, c.diag.state);
    TEST_ASSERT_EQUAL(DashLegacy045Clone27Block::StaleNative, c.diag.blocked);
}

void test_context_staleness_blocks_cloning()
{
    DashLegacy045Clone27 c;
    uint8_t out[8] = {};
    arm(c);
    seedInWindow(c, 1000, 2);
    feedNative(c, 0x0D, 1900); // native stays fresh
    TEST_ASSERT_FALSE(c.nextFrame(2100, out)); // 0x399 silent > 1 s
    TEST_ASSERT_EQUAL(DashLegacy045Clone27Block::ContextStale, c.diag.blocked);
}

void test_own_echo_dropped_inside_window()
{
    DashLegacy045Clone27 c;
    uint8_t out[8] = {};
    arm(c);
    seedInWindow(c, 1000, 2);
    TEST_ASSERT_TRUE(c.nextFrame(1000, out));
    c.recordTxResult(true, 1000);
    c.onNative045(out, 8, 1010); // bus echoes our clone back within 30 ms
    TEST_ASSERT_EQUAL_UINT32(1, c.diag.ownEchoRx);
    TEST_ASSERT_EQUAL_UINT8(0x0C, c.diag.nativeCounter); // native cache untouched
}

void test_physical_scroll_skips_cloning_until_idle_returns()
{
    DashLegacy045Clone27 c;
    uint8_t out[8] = {};
    arm(c);
    seedInWindow(c, 1000, 2);
    TEST_ASSERT_TRUE(c.nextFrame(1000, out));
    c.recordTxResult(true, 1000);

    // Human scrolls: those frames must never become the clone base.
    feedPhysical(c, 1600);
    TEST_ASSERT_EQUAL_UINT32(1, c.diag.physicalSkip);
    TEST_ASSERT_EQUAL_UINT8(0x0C, c.diag.nativeCounter);

    // Idle native resumes -> cloning resumes at the next interval.
    feedNative(c, 0x0D, 2500);
    c.onDasStatus(5, 2, CAN_BUS_VEH, 2500);
    TEST_ASSERT_TRUE(c.nextFrame(2500, out));
    TEST_ASSERT_EQUAL_HEX8(0x38, out[3]); // bit27 still the only content change
    TEST_ASSERT_EQUAL_HEX8(0x00, out[4]); // human tick not copied
}

void test_invalid_native_never_used_as_base()
{
    DashLegacy045Clone27 c;
    uint8_t out[8] = {};
    arm(c);
    c.onDasStatus(5, 2, CAN_BUS_VEH, 1000);
    uint8_t bad[8] = {0x40, 0x21, 0x00, 0x30, 0x00, 0x00, 0xC0, 0x00}; // CRC broken
    c.onNative045(bad, 8, 1000);
    TEST_ASSERT_EQUAL_UINT32(1, c.diag.nativeInvalid);
    TEST_ASSERT_FALSE(c.nextFrame(1000, out));
    TEST_ASSERT_EQUAL(DashLegacy045Clone27Block::NoNative, c.diag.blocked);
}

void test_tx_failure_twice_auto_disarms_and_toggle_rearms()
{
    DashLegacy045Clone27 c;
    uint8_t out[8] = {};
    arm(c);
    seedInWindow(c, 1000, 2);
    TEST_ASSERT_TRUE(c.nextFrame(1000, out));
    c.recordTxResult(false, 1000);
    TEST_ASSERT_FALSE(c.diag.autoDisarmed);

    feedNative(c, 0x0D, 2500); // second interval, native fresh
    c.onDasStatus(5, 2, CAN_BUS_VEH, 2500);
    TEST_ASSERT_TRUE(c.nextFrame(2500, out));
    c.recordTxResult(false, 2500);
    TEST_ASSERT_TRUE(c.diag.autoDisarmed);
    TEST_ASSERT_EQUAL(DashLegacy045Clone27State::AutoDisarmed, c.diag.state);

    feedNative(c, 0x0D, 4000); // disarmed: silent even in window
    c.onDasStatus(5, 2, CAN_BUS_VEH, 4000);
    TEST_ASSERT_FALSE(c.nextFrame(4000, out));

    c.configure(false);
    c.configure(true);
    TEST_ASSERT_FALSE(c.diag.autoDisarmed);
    feedNative(c, 0x0D, 5000);
    c.onDasStatus(5, 2, CAN_BUS_VEH, 5000);
    TEST_ASSERT_TRUE(c.nextFrame(5000, out)); // re-armed, first frame immediate
}

void test_counter_wrap_uses_native_plus_one_modulo_16()
{
    DashLegacy045Clone27 c;
    uint8_t out[8] = {};
    arm(c);
    uint8_t native[8];
    memcpy(native, kIdleC, 8);
    native[6] = 0xF0; // counter 0xF
    native[7] = DashLegacy045Clone27::crc8J1850(native, 7);
    c.onNative045(native, 8, 1000);
    c.onDasStatus(5, 2, CAN_BUS_VEH, 1000);
    TEST_ASSERT_TRUE(c.nextFrame(1000, out));
    // Counter 0xF wraps to 0; low nibble (0xF0 & 0x0F = 0) is preserved.
    TEST_ASSERT_EQUAL_HEX8(0x00, out[6]);
    TEST_ASSERT_EQUAL_UINT8(0x0, DashLegacy045Clone27::counter(out));
    TEST_ASSERT_TRUE(DashLegacy045Clone27::validFrame(out));
}

void test_runtime_denied_blocks_cloning()
{
    DashLegacy045Clone27 c;
    uint8_t out[8] = {};
    arm(c);
    seedInWindow(c, 1000, 2);
    c.setRuntimeAllowed(false, DashLegacy045Clone27Block::OtaGuard);
    TEST_ASSERT_FALSE(c.nextFrame(1000, out));
    TEST_ASSERT_EQUAL(DashLegacy045Clone27Block::OtaGuard, c.diag.blocked);
    c.setRuntimeAllowed(true, DashLegacy045Clone27Block::None);
    feedNative(c, 0x0C, 1100);
    TEST_ASSERT_TRUE(c.nextFrame(1100, out));
}

void test_disabled_switch_stays_silent()
{
    DashLegacy045Clone27 c;
    uint8_t out[8] = {};
    c.setRuntimeAllowed(true, DashLegacy045Clone27Block::None);
    seedInWindow(c, 1000, 2);
    TEST_ASSERT_FALSE(c.nextFrame(1000, out));
    TEST_ASSERT_EQUAL(DashLegacy045Clone27Block::Disabled, c.diag.blocked);
}

void test_reset_runtime_clears_all_state()
{
    DashLegacy045Clone27 c;
    uint8_t out[8] = {};
    arm(c);
    seedInWindow(c, 1000, 2);
    TEST_ASSERT_TRUE(c.nextFrame(1000, out));
    c.recordTxResult(false, 1000);

    c.resetRuntime();
    TEST_ASSERT_FALSE(c.diag.autoDisarmed);
    TEST_ASSERT_EQUAL_UINT32(0, c.diag.cloneTx);
    TEST_ASSERT_EQUAL(DashLegacy045Clone27State::Idle, c.diag.state);

    seedInWindow(c, 3000, 2); // fresh cycle works
    TEST_ASSERT_TRUE(c.nextFrame(3000, out));
}

void test_clone_chain_tracks_latest_native_counter()
{
    DashLegacy045Clone27 c;
    uint8_t out[8] = {};
    arm(c);
    seedInWindow(c, 1000, 2);
    TEST_ASSERT_TRUE(c.nextFrame(1000, out)); // base C -> clone D
    c.recordTxResult(true, 1000);

    feedNative(c, 0x03, 2500); // native advanced well past our clone
    c.onDasStatus(5, 2, CAN_BUS_VEH, 2500);
    TEST_ASSERT_TRUE(c.nextFrame(2500, out)); // base 3 -> clone 4
    TEST_ASSERT_EQUAL_HEX8(0x40, out[6] & 0xF0);
    c.recordTxResult(true, 2500);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_clone_frame_sets_bit27_bumps_counter_and_recomputes_crc);
    RUN_TEST(test_window_admits_2_through_5_and_fails_closed_at_6);
    RUN_TEST(test_das_gate_requires_autopilot_state_above_3);
    RUN_TEST(test_window_entry_counted_on_rising_edge_only);
    RUN_TEST(test_first_clone_is_immediate_then_1500ms_cooldown);
    RUN_TEST(test_native_staleness_blocks_cloning);
    RUN_TEST(test_context_staleness_blocks_cloning);
    RUN_TEST(test_own_echo_dropped_inside_window);
    RUN_TEST(test_physical_scroll_skips_cloning_until_idle_returns);
    RUN_TEST(test_invalid_native_never_used_as_base);
    RUN_TEST(test_tx_failure_twice_auto_disarms_and_toggle_rearms);
    RUN_TEST(test_counter_wrap_uses_native_plus_one_modulo_16);
    RUN_TEST(test_runtime_denied_blocks_cloning);
    RUN_TEST(test_disabled_switch_stays_silent);
    RUN_TEST(test_reset_runtime_clears_all_state);
    RUN_TEST(test_clone_chain_tracks_latest_native_counter);
    return UNITY_END();
}

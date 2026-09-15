#include <unity.h>

#include "dash_jitter_procedure.h"

// v1.19 JITTER procedure native tests — the LittleGong-aligned replacement
// for the v1.18 coordinator suite. Every timestamp is caller-supplied so the
// whole machine is deterministic; helpers below drive the documented paths
// from the disassembly notes in dash_jitter_procedure.h.

// ─── helpers ──────────────────────────────────────────────────────────

// A plausible native 0x045 idle frame (gesture 0, counter 4 in byte6 high
// nibble, valid CRC over the first 7 bytes with our J1850 variant).
static uint8_t kNative045[8] = {0x40, 0x30, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00};

static void fixNative045Crc()
{
    kNative045[7] = jitter_045::crc8J1850(kNative045, 7);
}

static void arm(DashJitterProcedure &p)
{
    p.configure(true, true);
    p.setPermit(true);
}

// Idle -> Normal (any nonzero event) -> Arming (event 3).
static void reachArming(DashJitterProcedure &p, uint32_t t)
{
    p.observeDasStatus(1, t);
    p.observeDasStatus(3, t + 10);
}

// Arming -> Disengaging (event 6, first cycle) with the cancel pump armed.
static void reachDisengaging(DashJitterProcedure &p, uint32_t t)
{
    reachArming(p, t);
    p.observeDasStatus(6, t + 20);
}

// ─── constants (LittleGong disassembly pins) ──────────────────────────

void test_constants_pin_littlegong_values()
{
    TEST_ASSERT_EQUAL_UINT32(5000, DashJitterProcedure::kDeadlineMs);
    TEST_ASSERT_EQUAL_UINT32(47, DashJitterProcedure::kCarrierLockMs);
    TEST_ASSERT_EQUAL_UINT32(3, DashJitterProcedure::kPumpGapMs);
    TEST_ASSERT_EQUAL_UINT8(16, DashJitterProcedure::kPumpMaxFrames);
    TEST_ASSERT_EQUAL_FLOAT(45.0f, DashJitterProcedure::kSteerAbortDeg);
    TEST_ASSERT_EQUAL_FLOAT(90.0f, DashJitterProcedure::kSteerResetDeg);
    TEST_ASSERT_EQUAL_UINT32(5000, DashJitterProcedure::kNative3eeMaxAgeMs);
    // v1.19.3 widened Disengaging wait (ours, not LittleGong's; Arming
    // keeps kDeadlineMs=5000 above). v1.19.4: the 200 ms hold pacing
    // constant is deleted with the hold itself — window bit46 continuity
    // is the base path's live-frame rewrite now, not a clone cadence.
    TEST_ASSERT_EQUAL_UINT32(10000, DashJitterProcedure::kDisengWaitMs);
}

void test_crc8_j1850_golden()
{
    // Field-validated variant (init/xorout 0xFF, poly 0x1D) — keep OURS, do
    // not adopt LittleGong's decoded table variant. Golden values (computed
    // independently in Python) pin the exact byte form so a future "helpful"
    // change cannot drift silently.
    const uint8_t zeros[7] = {0, 0, 0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL_HEX8(0x0A, jitter_045::crc8J1850(zeros, 7));
    const uint8_t ones[7] = {1, 1, 1, 1, 1, 1, 1};
    TEST_ASSERT_EQUAL_HEX8(0x38, jitter_045::crc8J1850(ones, 7));
    const uint8_t seq[7] = {0x40, 0x30, 0x00, 0x00, 0x00, 0x00, 0x40};
    TEST_ASSERT_EQUAL_HEX8(0xFB, jitter_045::crc8J1850(seq, 7));
    fixNative045Crc();
    TEST_ASSERT_EQUAL_HEX8(0xFB, kNative045[7]);
}

void test_counter_of_high_nibble()
{
    uint8_t f[8] = {0, 0, 0, 0, 0, 0, 0x4A, 0};
    TEST_ASSERT_EQUAL_UINT8(4, jitter_045::counterOf(f));
    uint8_t g[8] = {0, 0, 0, 0, 0, 0, 0xF0, 0};
    TEST_ASSERT_EQUAL_UINT8(15, jitter_045::counterOf(g));
}

// ─── configure / arming gates ─────────────────────────────────────────

void test_default_is_inert_and_off()
{
    DashJitterProcedure p;
    TEST_ASSERT_FALSE(p.armed());
    TEST_ASSERT_FALSE(p.procedureActive());
    DashJitterDiag d = p.diag();
    TEST_ASSERT_FALSE(d.requested);
    TEST_ASSERT_FALSE(d.effective);
    TEST_ASSERT_EQUAL(JitterPhase::Inert, d.phase);
    TEST_ASSERT_EQUAL_STRING("off", d.reason);
    // An inert machine ignores everything: no phase change, no output.
    p.observeDasStatus(3, 100);
    p.observeDasStatus(6, 200);
    uint8_t out[8];
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(300, out));
    TEST_ASSERT_EQUAL(JitterPhase::Inert, p.diag().phase);
}

void test_gate_closed_resets_to_inert()
{
    DashJitterProcedure p;
    reachDisengaging(p, 100);
    p.configure(true, false);
    DashJitterDiag d = p.diag();
    TEST_ASSERT_EQUAL(JitterPhase::Inert, d.phase);
    TEST_ASSERT_EQUAL_STRING("apGateOff", d.reason);
    TEST_ASSERT_FALSE(p.procedureActive());
}

void test_disabled_switch_resets_to_inert()
{
    DashJitterProcedure p;
    reachDisengaging(p, 100);
    p.configure(false, true);
    DashJitterDiag d = p.diag();
    TEST_ASSERT_EQUAL(JitterPhase::Inert, d.phase);
    TEST_ASSERT_EQUAL_STRING("off", d.reason);
}

// ─── phase machine (0x399 low nibble events) ──────────────────────────

void test_state0_any_nonzero_event_enters_monitoring()
{
    DashJitterProcedure p;
    arm(p);
    TEST_ASSERT_EQUAL(JitterPhase::Idle, p.diag().phase);
    p.observeDasStatus(2, 100);
    TEST_ASSERT_EQUAL(JitterPhase::Normal, p.diag().phase);
    TEST_ASSERT_EQUAL_STRING("monitoring", p.diag().reason);
}

void test_event3_enters_arming_and_queues_one_shot()
{
    DashJitterProcedure p;
    arm(p);
    p.observeNative3eeMux0(kNative045, 50); // any bytes; freshness is what counts
    reachArming(p, 1000);
    DashJitterDiag d = p.diag();
    TEST_ASSERT_EQUAL(JitterPhase::Arming, d.phase);
    TEST_ASSERT_EQUAL_STRING("arming", d.reason);
    TEST_ASSERT_TRUE(p.procedureActive());
}

void test_event6_gates_on_one_cycle_per_period()
{
    DashJitterProcedure p;
    arm(p);
    p.observeNative045(kNative045, 30);
    reachDisengaging(p, 1000);
    TEST_ASSERT_EQUAL(JitterPhase::Disengaging, p.diag().phase);
    // The cancel consumes the trigger but not the cycle count — g_count only
    // moves on the event-2 re-request (LittleGong's counter placement).
    TEST_ASSERT_EQUAL_UINT8(0, p.diag().cycles);
    TEST_ASSERT_EQUAL_UINT32(1, p.diag().cancels);
    // A second event 6 while Disengaging does nothing (events 1/2 own it).
    p.observeDasStatus(6, 2000);
    TEST_ASSERT_EQUAL(JitterPhase::Disengaging, p.diag().phase);
    TEST_ASSERT_EQUAL_UINT8(0, p.diag().cycles);
    TEST_ASSERT_EQUAL_UINT32(1, p.diag().cancels);
}

void test_disengaging_event1_notes_cancel_seen_event2_re_requests()
{
    DashJitterProcedure p;
    arm(p);
    p.observeNative045(kNative045, 30);
    reachDisengaging(p, 1000);
    p.observeDasStatus(1, 1100);
    TEST_ASSERT_EQUAL_STRING("cancelSeen", p.diag().reason);
    p.observeDasStatus(2, 1200);
    DashJitterDiag d = p.diag();
    TEST_ASSERT_EQUAL_UINT8(1, d.cycles);
    TEST_ASSERT_EQUAL_UINT32(1, d.requests);
    TEST_ASSERT_EQUAL_STRING("reRequest", d.reason);
    TEST_ASSERT_EQUAL(JitterPhase::Disengaging, d.phase);
}

void test_events_8_9_10_full_reset_from_any_active_phase()
{
    DashJitterProcedure p;
    arm(p);
    reachArming(p, 1000);
    p.observeDasStatus(9, 1500);
    DashJitterDiag d = p.diag();
    TEST_ASSERT_EQUAL(JitterPhase::Idle, d.phase); // armed -> back to Idle
    TEST_ASSERT_EQUAL_STRING("apError", d.reason);
    TEST_ASSERT_EQUAL_UINT32(1, d.apErrorResets);
    TEST_ASSERT_FALSE(p.procedureActive());

    DashJitterProcedure q;
    arm(q);
    reachDisengaging(q, 1000);
    q.observeDasStatus(8, 1500);
    TEST_ASSERT_EQUAL(JitterPhase::Idle, q.diag().phase);
    TEST_ASSERT_EQUAL_UINT32(1, q.diag().apErrorResets);
}

void test_arming_timeout_full_reset()
{
    DashJitterProcedure p;
    arm(p);
    reachArming(p, 1000); // deadline = 1010 + 5000 = 6010
    uint8_t out[8];
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(6009, out));
    TEST_ASSERT_EQUAL(JitterPhase::Arming, p.diag().phase);   // 1 ms early: alive
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(6010, out)); // boundary: expired
    DashJitterDiag d = p.diag();
    TEST_ASSERT_EQUAL(JitterPhase::Idle, d.phase);
    TEST_ASSERT_EQUAL_STRING("timeout", d.reason);
    TEST_ASSERT_EQUAL_UINT32(1, d.timeoutResets);
    TEST_ASSERT_FALSE(p.procedureActive());
}

void test_permit_required_for_arming_entry()
{
    DashJitterProcedure p;
    p.configure(true, true);
    TEST_ASSERT_FALSE(p.diag().permit);
    p.observeDasStatus(1, 100);
    p.observeDasStatus(3, 200); // no permit -> stays Normal
    TEST_ASSERT_EQUAL(JitterPhase::Normal, p.diag().phase);
    p.setPermit(true);
    // v1.19.1 edge semantics: a repeated SAME state does not re-fire the
    // machine — leave state 3 first, then bring a fresh event-3 edge.
    p.observeDasStatus(2, 250);
    p.observeDasStatus(3, 300);
    TEST_ASSERT_EQUAL(JitterPhase::Arming, p.diag().phase);
}

void test_permit_loss_mid_procedure_resets()
{
    DashJitterProcedure p;
    arm(p);
    reachArming(p, 1000);
    p.setPermit(false);
    DashJitterDiag d = p.diag();
    TEST_ASSERT_EQUAL(JitterPhase::Idle, d.phase);
    TEST_ASSERT_EQUAL_STRING("permitLost", d.reason);
    TEST_ASSERT_FALSE(p.procedureActive());
}

// ─── bit46 one-shot ───────────────────────────────────────────────────

void test_one_shot_patches_native_template()
{
    DashJitterProcedure p;
    arm(p);
    uint8_t tmpl[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x00, 0x66, 0x77};
    p.observeNative3eeMux0(tmpl, 900);
    reachArming(p, 1000);
    uint8_t out[8];
    TEST_ASSERT_EQUAL(JitterAction::Bit46Shot, p.tick(1010, out));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(tmpl, out, 5); // bytes 0..4 ride the clone
    TEST_ASSERT_EQUAL_UINT8(tmpl[6], out[6]);
    TEST_ASSERT_EQUAL_UINT8(tmpl[7], out[7]);
    TEST_ASSERT_EQUAL_UINT8(0x43, out[5]); // byte5 |= 0x43, template bit5 was 0
    TEST_ASSERT_EQUAL_UINT32(1, p.diag().bit46Shots);
    // One-shot only: the next tick produces nothing.
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(1011, out));
    TEST_ASSERT_EQUAL_UINT32(1, p.diag().bit46Shots);
}

void test_one_shot_or_in_preserves_existing_bits()
{
    DashJitterProcedure p;
    arm(p);
    uint8_t tmpl[8] = {0, 0, 0, 0, 0, 0x41, 0, 0};
    p.observeNative3eeMux0(tmpl, 900);
    reachArming(p, 1000);
    uint8_t out[8];
    TEST_ASSERT_EQUAL(JitterAction::Bit46Shot, p.tick(1010, out));
    TEST_ASSERT_EQUAL_UINT8(0x43, out[5]); // 0x41 | 0x43 == 0x43
}

void test_one_shot_skipped_without_fresh_template()
{
    DashJitterProcedure p;
    arm(p);
    // No observeNative3eeMux0 at all.
    reachArming(p, 1000);
    uint8_t out[8];
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(1010, out));
    TEST_ASSERT_EQUAL_UINT32(0, p.diag().bit46Shots);
    TEST_ASSERT_EQUAL(JitterPhase::Arming, p.diag().phase); // cycle continues
    // A template older than 5000 ms is stale too. Reset the machine first —
    // Arming cannot re-enter without a reset (Disengaging never exits back).
    p.observeDasStatus(9, 1500);
    TEST_ASSERT_EQUAL(JitterPhase::Idle, p.diag().phase);
    uint8_t tmpl[8] = {0, 0, 0, 0, 0, 0x00, 0, 0};
    p.observeNative3eeMux0(tmpl, 15000);
    p.observeDasStatus(1, 20000);
    p.observeDasStatus(3, 20010);
    TEST_ASSERT_EQUAL(JitterPhase::Arming, p.diag().phase);
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(20020, out)); // age 5020 > 5000
    TEST_ASSERT_EQUAL_UINT32(0, p.diag().bit46Shots);
    TEST_ASSERT_EQUAL(JitterPhase::Arming, p.diag().phase); // skipped, not reset
}

// ─── carrier-locked 0x045 pump ────────────────────────────────────────

void test_pump_only_rides_fresh_carrier()
{
    DashJitterProcedure p;
    arm(p);
    reachDisengaging(p, 1000); // cancel pump started; no native 0x045 ever seen
    uint8_t out[8];
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(1010, out));
    // Native frame arrives -> carrier fresh for the next 47 ms.
    p.observeNative045(kNative045, 1020);
    TEST_ASSERT_EQUAL(JitterAction::Stalk045, p.tick(1025, out));
    // Carrier goes stale at 47 ms: no more frames even within the burst cap.
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(1070, out));
    TEST_ASSERT_EQUAL_UINT32(1, p.diag().pumpFrames);
    // A fresh native frame re-opens the window.
    p.observeNative045(kNative045, 1080);
    TEST_ASSERT_EQUAL(JitterAction::Stalk045, p.tick(1085, out));
}

void test_pump_respects_3ms_gap()
{
    DashJitterProcedure p;
    arm(p);
    p.observeNative045(kNative045, 1000);
    reachDisengaging(p, 1010);
    uint8_t out[8];
    TEST_ASSERT_EQUAL(JitterAction::Stalk045, p.tick(1020, out)); // tx @1020
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(1022, out));     // gap <3ms
    TEST_ASSERT_EQUAL(JitterAction::Stalk045, p.tick(1023, out)); // exactly 3ms
}

void test_pump_caps_at_16_frames()
{
    DashJitterProcedure p;
    arm(p);
    reachDisengaging(p, 1000);
    uint8_t out[8];
    uint32_t t = 1010;
    // A native carrier seen NOW keeps refreshing; the pump's own gap paces us.
    int frames = 0;
    for (int i = 0; i < 40; ++i)
    {
        p.observeNative045(kNative045, t);
        if (p.tick(t + 1, out) == JitterAction::Stalk045)
            ++frames;
        t += 4;
    }
    TEST_ASSERT_EQUAL_INT(16, frames);
    DashJitterDiag d = p.diag();
    TEST_ASSERT_EQUAL_UINT32(16, d.pumpFrames);
    TEST_ASSERT_EQUAL_UINT32(1, d.cancels);
}

void test_pump_first_counter_is_template_plus_one_then_increments()
{
    fixNative045Crc();
    DashJitterProcedure p;
    arm(p);
    p.observeNative045(kNative045, 1000); // counter = 4
    reachDisengaging(p, 1010);
    uint8_t out[8];
    TEST_ASSERT_EQUAL(JitterAction::Stalk045, p.tick(1020, out));
    TEST_ASSERT_EQUAL_UINT8(0x41, out[0]); // cancel gesture
    TEST_ASSERT_EQUAL_UINT8(5, jitter_045::counterOf(out));
    TEST_ASSERT_EQUAL_UINT8(kNative045[6] & 0x0F, out[6] & 0x0F); // low nibble rides
    TEST_ASSERT_EQUAL_UINT8(jitter_045::crc8J1850(out, 7), out[7]);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(kNative045 + 1, out + 1, 5); // bytes1..5 clone
    p.observeNative045(kNative045, 1030);
    TEST_ASSERT_EQUAL(JitterAction::Stalk045, p.tick(1032, out));
    TEST_ASSERT_EQUAL_UINT8(6, jitter_045::counterOf(out));
}

void test_pump_request_gesture_0x42()
{
    DashJitterProcedure p;
    arm(p);
    p.observeNative045(kNative045, 1000);
    reachDisengaging(p, 1010); // events at 1010/1020/1030
    p.observeDasStatus(1, 1040);
    p.observeDasStatus(2, 1050); // re-request burst
    uint8_t out[8];
    p.observeNative045(kNative045, 1060);
    TEST_ASSERT_EQUAL(JitterAction::Stalk045, p.tick(1065, out));
    TEST_ASSERT_EQUAL_UINT8(0x42, out[0]);
}

void test_tx_fail_silently_stops_pump()
{
    DashJitterProcedure p;
    arm(p);
    p.observeNative045(kNative045, 1000);
    reachDisengaging(p, 1010);
    uint8_t out[8];
    TEST_ASSERT_EQUAL(JitterAction::Stalk045, p.tick(1020, out));
    p.recordTxResult(false);
    p.observeNative045(kNative045, 1030);
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(1035, out));
    DashJitterDiag d = p.diag();
    TEST_ASSERT_EQUAL_UINT32(1, d.txFail);
    TEST_ASSERT_EQUAL_UINT32(0, d.txOk);
    // The phase machine keeps walking (LittleGong: no failure handling).
    TEST_ASSERT_EQUAL(JitterPhase::Disengaging, d.phase);
}

void test_tx_ok_counted()
{
    DashJitterProcedure p;
    arm(p);
    p.observeNative045(kNative045, 1000);
    reachDisengaging(p, 1010);
    uint8_t out[8];
    TEST_ASSERT_EQUAL(JitterAction::Stalk045, p.tick(1020, out));
    p.recordTxResult(true);
    TEST_ASSERT_EQUAL_UINT32(1, p.diag().txOk);
}

// ─── 0x488 steering abort ─────────────────────────────────────────────

void test_steer_over_45_aborts_burst_only()
{
    DashJitterProcedure p;
    arm(p);
    p.observeNative045(kNative045, 1000);
    reachDisengaging(p, 1010);
    p.observeSteerAngle(50.0f, true, 1020);
    DashJitterDiag d = p.diag();
    TEST_ASSERT_EQUAL_UINT32(1, d.steerAborts);
    TEST_ASSERT_EQUAL_UINT32(0, d.steerResets);
    TEST_ASSERT_EQUAL(JitterPhase::Disengaging, d.phase); // machine walks on
    TEST_ASSERT_EQUAL_STRING("steerAbort", d.reason);
    uint8_t out[8];
    p.observeNative045(kNative045, 1030);
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(1035, out)); // pump stopped
}

void test_steer_over_90_full_reset()
{
    DashJitterProcedure p;
    arm(p);
    reachArming(p, 1000);
    p.observeSteerAngle(-120.0f, true, 1020); // magnitude matters
    DashJitterDiag d = p.diag();
    TEST_ASSERT_EQUAL_UINT32(1, d.steerResets);
    TEST_ASSERT_EQUAL_UINT32(0, d.steerAborts);
    TEST_ASSERT_EQUAL(JitterPhase::Idle, d.phase);
    TEST_ASSERT_EQUAL_STRING("steerReset", d.reason);
}

void test_steer_ignored_without_procedure_or_invalid_type()
{
    DashJitterProcedure p;
    arm(p);
    p.observeSteerAngle(180.0f, true, 100); // no procedure running
    TEST_ASSERT_EQUAL_UINT32(0, p.diag().steerResets);
    reachDisengaging(p, 1000);
    p.observeSteerAngle(180.0f, false, 1020); // invalid steeringControlType
    TEST_ASSERT_EQUAL_UINT32(0, p.diag().steerResets);
    TEST_ASSERT_EQUAL_UINT32(0, p.diag().steerAborts);
    TEST_ASSERT_EQUAL(JitterPhase::Disengaging, p.diag().phase);
}

// ─── v1.19.4 window carve-out + clone extinction ─────────────────────

void test_procedure_active_only_in_arming_and_disengaging()
{
    DashJitterProcedure p;
    arm(p);
    TEST_ASSERT_FALSE(p.procedureActive()); // Idle
    p.observeDasStatus(1, 100);
    TEST_ASSERT_FALSE(p.procedureActive()); // Normal
    reachArming(p, 1000);
    TEST_ASSERT_TRUE(p.procedureActive());
    reachDisengaging(p, 2000);
    TEST_ASSERT_TRUE(p.procedureActive());
    p.observeDasStatus(9, 2100);
    TEST_ASSERT_FALSE(p.procedureActive()); // reset back to Idle
}

void test_in_disengaging_wait_window_scoped_accessor()
{
    // v1.19.4: the Legacy FSD gate keys its Disengaging carve-out on this
    // accessor — it must be window-scoped, NOT a procedureActive() alias:
    // Arming runs while the car is engaged (0x399 state 3..6) and passes
    // the plain APActive gate on its own, so granting it the carve-out
    // would only widen the bypass. The two predicates deliberately diverge
    // in Arming.
    DashJitterProcedure p;
    arm(p);
    TEST_ASSERT_FALSE(p.inDisengagingWait()); // Idle
    p.observeDasStatus(1, 100);
    TEST_ASSERT_FALSE(p.inDisengagingWait()); // Normal
    reachArming(p, 1000);
    TEST_ASSERT_FALSE(p.inDisengagingWait()); // Arming: car engaged 3..6
    TEST_ASSERT_TRUE(p.procedureActive());    // ...still procedure-active
    reachDisengaging(p, 2000);
    TEST_ASSERT_TRUE(p.inDisengagingWait()); // window opens at event 6
    p.observeDasStatus(1, 2100);
    TEST_ASSERT_TRUE(p.inDisengagingWait()); // the whole wait window
    p.observeDasStatus(6, 2200);             // landing...
    TEST_ASSERT_TRUE(p.inDisengagingWait()); // phase holds until reset
    p.observeDasStatus(9, 2300);             // apError -> full reset
    TEST_ASSERT_FALSE(p.inDisengagingWait());
    TEST_ASSERT_FALSE(p.procedureActive());
}

void test_disengaging_window_emits_no_bit46_clones()
{
    // v1.19.4 regression core (2026-09-16 beta16 field pass, two 120 s
    // CSVs): every synthetic 0x3EE clone beyond the arming one-shot is
    // RETIRED. Through the whole Disengaging window tick() must never
    // return Bit46Shot — in-window bit46 continuity is the base-path
    // live-frame rewrite's job (carrier-locked, field-coherent). The
    // retired v1.19.2 refresh + v1.19.3 hold clones carried a
    // self-contradictory shape (byte5 |= 0x43 offset-enable bits with no
    // offset written, byte3 washing the user offset EC->D0, stale
    // counter/checksum, 200 ms free-run off-carrier) and the car knocked
    // availability down 53-58x per 120 s synced to that cadence (beta15:
    // 9-11). Fresh templates throughout — a stale-template fail-closed
    // skip must not mask the emission.
    DashJitterProcedure p;
    arm(p);
    uint8_t tmpl[8] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x00, 0x36, 0x37};
    p.observeNative3eeMux0(tmpl, 900);
    p.observeNative045(kNative045, 1000);
    reachDisengaging(p, 1010);
    uint8_t out[8];
    TEST_ASSERT_EQUAL(JitterAction::Bit46Shot, p.tick(1015, out)); // arming one-shot (kept)
    TEST_ASSERT_EQUAL_UINT32(1, p.diag().bit46Shots);
    p.observeDasStatus(1, 1040);
    p.observeDasStatus(2, 1050); // re-request — no clone may precede it
    // No fresh 0x045 carrier: the pump stays silent, so every sample must
    // be a hard None. Samples cover the old 200 ms hold cadence and the
    // 1 s / 4 s window depths.
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(1055, out));
    for (uint32_t t = 1250; t <= 5050; t += 200)
    {
        p.observeNative3eeMux0(tmpl, t);
        TEST_ASSERT_EQUAL(JitterAction::None, p.tick(t, out));
    }
    TEST_ASSERT_EQUAL_UINT32(1, p.diag().bit46Shots); // no growth
    // Fresh 0x045 carriers open the 0x42 pump: Stalk045 alone may speak —
    // the burst rides carrier-locked, and bit46Shots stays pinned.
    for (int i = 0; i < 16; ++i)
    {
        uint32_t t = 6000 + i * 50;
        p.observeNative045(kNative045, t);
        TEST_ASSERT_EQUAL(JitterAction::Stalk045, p.tick(t + 1, out));
        TEST_ASSERT_EQUAL_UINT8(0x42, out[0]);
    }
    for (int i = 0; i < 4; ++i) // budget spent: silent, still clone-free
    {
        uint32_t t = 6800 + i * 50;
        p.observeNative045(kNative045, t);
        TEST_ASSERT_EQUAL(JitterAction::None, p.tick(t + 1, out));
    }
    TEST_ASSERT_EQUAL_UINT32(1, p.diag().bit46Shots);
    // Late landing stands everything down; post-landing stays clone-free
    // and the deadline (1050+10000) expires with success billing.
    p.observeDasStatus(6, 8100);
    TEST_ASSERT_EQUAL_STRING("reEngagedFsd", p.diag().reason);
    p.observeNative3eeMux0(tmpl, 8500);
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(8600, out));
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(11050, out));
    TEST_ASSERT_EQUAL_UINT8(0, p.diag().failedCycles);
    // Module reset contract (carried from the retired base-path-pause
    // test): the gate closing mid-procedure still resets to Inert.
    p.observeDasStatus(1, 12000);
    p.observeDasStatus(3, 12010);
    p.observeDasStatus(6, 12020);
    TEST_ASSERT_EQUAL(JitterPhase::Disengaging, p.diag().phase);
    p.configure(true, false);
    TEST_ASSERT_EQUAL(JitterPhase::Inert, p.diag().phase);
    TEST_ASSERT_EQUAL_STRING("apGateOff", p.diag().reason);
}

// ─── fullReset clears the working set but not the counters ────────────

void test_full_reset_clears_cycle_state_keeps_diag_counters()
{
    DashJitterProcedure p;
    arm(p);
    p.observeNative045(kNative045, 1000);
    reachDisengaging(p, 1010);
    TEST_ASSERT_TRUE(p.procedureActive());
    p.observeDasStatus(9, 1100); // apError reset
    DashJitterDiag d = p.diag();
    TEST_ASSERT_EQUAL_UINT8(0, d.cycles);
    TEST_ASSERT_EQUAL_UINT32(1, d.cancels); // lifetime counters survive
    TEST_ASSERT_EQUAL_UINT32(1, d.apErrorResets);
    // v1.19.3: the interrupted cycle has no re-engagement evidence, so it
    // bills the DIAGNOSTIC streak (failedCycles 1 — the consuming session
    // cap is deleted; re-entry is protected regardless). Park is the
    // physical session boundary and still clears the streak.
    TEST_ASSERT_EQUAL_UINT8(1, p.diag().failedCycles);
    p.setVehicleParked(true);
    p.setVehicleParked(false);
    TEST_ASSERT_EQUAL_UINT8(0, p.diag().failedCycles);
    // Re-arm works: state0 -> state1 -> state3 -> state6 runs again.
    p.observeDasStatus(1, 2000);
    p.observeDasStatus(3, 2010);
    p.observeNative3eeMux0(kNative045, 2020);
    uint8_t out[8];
    TEST_ASSERT_EQUAL(JitterAction::Bit46Shot, p.tick(2030, out));
    p.observeDasStatus(6, 2040);
    TEST_ASSERT_EQUAL(JitterPhase::Disengaging, p.diag().phase);
    TEST_ASSERT_EQUAL_UINT32(2, p.diag().cancels);
}

// ─── v1.19.1 field-incident regressions ───────────────────────────────

void test_v1191_incident_replay_level_feed_terminates()
{
    // The v1.19 field incident: FSD refused -> the car settles in state 2
    // and 0x399 keeps broadcasting it. v1.19 fired the event-2 branch per
    // frame (startPump reset the 16-frame cap, the branch refreshed the
    // deadline -> unbounded 0x42 bursts -> the EAP loop). v1.19.1 must
    // send exactly ONE re-request, let the deadline expire, and stand
    // down for the rest of the session.
    DashJitterProcedure p;
    arm(p);
    p.observeNative045(kNative045, 1000);
    reachDisengaging(p, 1010); // events 1,3,6; cancel burst started
    p.observeDasStatus(1, 1040);
    p.observeDasStatus(2, 1050); // the ONE re-request (deadline 6050)
    TEST_ASSERT_EQUAL_UINT32(1, p.diag().requests);
    // Level feed: 50 consecutive state-2 frames, each with a fresh native
    // carrier — the worst case, since v1.19 restarted the pump per frame.
    uint8_t out[8];
    uint32_t t = 1060;
    for (int i = 0; i < 50; ++i)
    {
        p.observeNative045(kNative045, t);
        p.observeDasStatus(2, t); // same state: ignored (edge semantics)
        p.tick(t + 1, out);
        t += 20;
    }
    TEST_ASSERT_EQUAL_UINT32(1, p.diag().requests);    // never re-fired
    TEST_ASSERT_EQUAL_UINT32(16, p.diag().pumpFrames); // one capped burst
    // Deadline set once at 1050+10000 (v1.19.3 widened window), never
    // refreshed: it expires.
    p.observeNative045(kNative045, 6100);
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(6101, out));
    TEST_ASSERT_EQUAL(JitterPhase::Disengaging, p.diag().phase); // 10 s alive
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(11049, out));
    TEST_ASSERT_EQUAL(JitterPhase::Disengaging, p.diag().phase);
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(11050, out)); // boundary
    TEST_ASSERT_EQUAL(JitterPhase::Idle, p.diag().phase);
    TEST_ASSERT_EQUAL_STRING("timeout", p.diag().reason);
    TEST_ASSERT_EQUAL_UINT8(1, p.diag().failedCycles); // refused = failed
    // v1.19.3: the session cap is DELETED — the next human FSD engage is
    // protected again immediately (the field cap left 3..4 captures bare).
    p.observeDasStatus(1, 12000); // Idle -> Normal
    p.observeNative3eeMux0(kNative045, 12100);
    p.observeDasStatus(3, 12200);
    TEST_ASSERT_EQUAL(JitterPhase::Arming, p.diag().phase);
    TEST_ASSERT_EQUAL_STRING("arming", p.diag().reason);
    TEST_ASSERT_EQUAL(JitterAction::Bit46Shot, p.tick(12210, out));
    TEST_ASSERT_EQUAL_UINT8(1, p.diag().failedCycles); // diag survives
}

void test_v1191_reengage_success_keeps_protection_alive()
{
    // The EAP-landing variant: the 0x42 re-request DOES engage (the car
    // lands in EAP, states 3/6). The cycle counts as success — no session
    // cap — and after the user stalk-cancels EAP the machine does NOT
    // react to state 2 (the v1.19 loop is dead) while staying ready for
    // every subsequent legitimate engagement.
    DashJitterProcedure p;
    arm(p);
    p.observeNative045(kNative045, 1000);
    reachDisengaging(p, 1010);
    p.observeDasStatus(1, 1040);
    p.observeDasStatus(2, 1050);                               // one 0x42 re-request
    p.observeDasStatus(3, 1200);                               // vehicle re-engaged (EAP)
    TEST_ASSERT_EQUAL_STRING("reEngagedEap", p.diag().reason); // v1.19.2 split
    TEST_ASSERT_EQUAL_UINT32(1, p.diag().reengagedEap);
    p.observeDasStatus(6, 1300);
    // Deadline from the event-2 refresh (1050+10000, v1.19.3) expires ->
    // success billing. No fresh 0x045 carrier is introduced here on
    // purpose: the re-request pump still has budget and would legally
    // speak inside the widened window.
    uint8_t out[8];
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(11050, out));
    TEST_ASSERT_EQUAL(JitterPhase::Idle, p.diag().phase);
    TEST_ASSERT_EQUAL_UINT8(0, p.diag().failedCycles);
    // User cancels EAP: 1 -> 2. Idle -> Normal on the 1; the 2 is inert
    // (Normal only reacts to event 3) — no further 0x42, ever.
    p.observeDasStatus(1, 6200);
    p.observeDasStatus(2, 6300);
    TEST_ASSERT_EQUAL_UINT32(1, p.diag().requests); // unchanged
    // A fresh legitimate FSD engagement re-arms normally: the protection
    // is per-engagement, not once-per-drive.
    p.observeDasStatus(3, 6400);
    TEST_ASSERT_EQUAL(JitterPhase::Arming, p.diag().phase);
}

void test_v1191_bounce_12_fires_one_request()
{
    // 1<->2 bounce: every edge here IS a state change (the edge trigger
    // alone would re-fire), so the cycles_ gate is what keeps it to one
    // 0x42 per period — and the blocked edges must not refresh the
    // deadline either.
    DashJitterProcedure p;
    arm(p);
    p.observeNative045(kNative045, 1000);
    reachDisengaging(p, 1010);
    for (int i = 0; i < 5; ++i)
    {
        p.observeDasStatus(1, 1100 + i * 10);
        p.observeDasStatus(2, 1105 + i * 10);
    }
    TEST_ASSERT_EQUAL_UINT32(1, p.diag().requests);
    TEST_ASSERT_EQUAL_UINT8(1, p.diag().cycles);
    // Deadline stands at the single re-request (1105+10000=11105, v1.19.3):
    // the blocked bounce edges at 1115..1145 did not push it out.
    uint8_t out[8];
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(11110, out));
    TEST_ASSERT_EQUAL(JitterPhase::Idle, p.diag().phase);
    TEST_ASSERT_EQUAL_STRING("timeout", p.diag().reason);
}

void test_v1191_park_midcycle_does_not_poison_next_session()
{
    // The field exit was Park while Disengaging. Parking must END the
    // cycle without billing it to the next drive session (accounting
    // runs, then the budget re-opens — order matters).
    DashJitterProcedure p;
    arm(p);
    p.observeNative045(kNative045, 1000);
    reachDisengaging(p, 1010); // cycle running, no re-engagement yet
    p.setVehicleParked(true);
    DashJitterDiag d = p.diag();
    TEST_ASSERT_EQUAL(JitterPhase::Idle, d.phase);
    TEST_ASSERT_EQUAL_STRING("parked", d.reason);
    TEST_ASSERT_EQUAL_UINT8(0, d.failedCycles); // billed, then re-opened
    p.setVehicleParked(false);
    p.observeDasStatus(1, 2000);
    p.observeDasStatus(3, 2010);
    TEST_ASSERT_EQUAL(JitterPhase::Arming, p.diag().phase);
}

// ─── v1.19.2 landing split (refresh RETIRED in v1.19.4) ──────────────

void test_v1192_landing_split_fsd_vs_eap()
{
    // v1.19.1 counted BOTH states as one "reEngaged" success — exactly why
    // the ~30% FSD rate was invisible in the field diag. The split keeps
    // the session accounting (both landings clear failedCycles) but counts
    // and reports them separately.
    DashJitterProcedure p;
    arm(p);
    p.observeNative045(kNative045, 1000);
    reachDisengaging(p, 1010);
    p.observeDasStatus(1, 1040);
    p.observeDasStatus(2, 1050);
    p.observeDasStatus(6, 1200); // FSD landing
    DashJitterDiag d = p.diag();
    TEST_ASSERT_EQUAL_UINT32(1, d.reengagedFsd);
    TEST_ASSERT_EQUAL_UINT32(0, d.reengagedEap);
    TEST_ASSERT_EQUAL_STRING("reEngagedFsd", d.reason);
    uint8_t out[8];
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(11050, out)); // timeout (10 s)
    TEST_ASSERT_EQUAL_UINT8(0, p.diag().failedCycles);         // success billing
    // An EAP landing on the next cycle counts separately.
    p.observeDasStatus(1, 6200); // Idle -> Normal
    p.observeNative3eeMux0(kNative045, 6250);
    p.observeDasStatus(3, 6300); // -> Arming
    p.observeDasStatus(6, 6400); // -> Disengaging (cancel #2)
    p.observeDasStatus(1, 6500);
    p.observeDasStatus(2, 6600); // re-request #2
    p.observeDasStatus(3, 6800); // EAP landing
    d = p.diag();
    TEST_ASSERT_EQUAL_UINT32(1, d.reengagedFsd);
    TEST_ASSERT_EQUAL_UINT32(1, d.reengagedEap);
    TEST_ASSERT_EQUAL_STRING("reEngagedEap", d.reason);
    TEST_ASSERT_EQUAL_UINT32(2, d.requests);
    TEST_ASSERT_EQUAL_UINT32(2, d.cancels);
}

void test_v1194_no_refresh_clone_before_re_request()
{
    // v1.19.2 queued ONE bit46 refresh at the event-2 edge, emitted by
    // tick() ahead of the pump's first 0x42 (2026-09-14 rationale: the
    // arming shot's unlock latch does not survive our own cancel — ~70% of
    // re-requests landed EAP). v1.19.4 RETIRES it: the beta16 field data
    // traced the availability knocks to exactly these clone frames (shape
    // self-contradictory, off-carrier), and LittleGong sends nothing
    // between its cancel and re-request. Fresh 3ee template + stale 0x045
    // carrier at the first post-edge tick: must be None, not Bit46Shot.
    DashJitterProcedure p;
    arm(p);
    uint8_t tmpl[8] = {0x21, 0x22, 0x23, 0x24, 0x25, 0x00, 0x26, 0x27};
    p.observeNative3eeMux0(tmpl, 900);
    p.observeNative045(kNative045, 1000);
    reachDisengaging(p, 1010);
    uint8_t out[8];
    TEST_ASSERT_EQUAL(JitterAction::Bit46Shot, p.tick(1015, out)); // arming one-shot (kept)
    TEST_ASSERT_EQUAL_UINT32(1, p.diag().bit46Shots);
    p.observeDasStatus(1, 1040);
    p.observeDasStatus(2, 1050); // re-request queued — v1.19.2 refreshed here
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(1055, out));
    TEST_ASSERT_EQUAL_UINT32(1, p.diag().bit46Shots); // no second clone
    // The first emission after the edge is the pump's 0x42 riding the next
    // fresh carrier — not a clone.
    p.observeNative045(kNative045, 1060);
    TEST_ASSERT_EQUAL(JitterAction::Stalk045, p.tick(1065, out));
    TEST_ASSERT_EQUAL_UINT8(0x42, out[0]);
    // A 1<->2 bounce after the consumed request stays clone-free too.
    p.observeDasStatus(1, 1080);
    p.observeDasStatus(2, 1090);
    p.observeNative045(kNative045, 1100);
    p.observeNative3eeMux0(tmpl, 1100);
    TEST_ASSERT_EQUAL(JitterAction::Stalk045, p.tick(1105, out));
    TEST_ASSERT_EQUAL_UINT8(0x42, out[0]);
    TEST_ASSERT_EQUAL_UINT32(1, p.diag().bit46Shots); // pinned all along
}

// ─── v1.19.3 widened wait + cap deletion (hold RETIRED in v1.19.4) ────

void test_v1193_disengaging_window_10s_arming_5s()
{
    // Split deadlines (v1.19.3): Arming keeps the LittleGong 5 s; the
    // Disengaging wait widens to 10 s — the measured vehicle response to
    // the re-request runs 2.8..7.1 s, so the old shared 5 s cut late
    // landings out of the accounting.
    DashJitterProcedure p;
    arm(p);
    reachArming(p, 1000); // deadline 6010
    uint8_t out[8];
    TEST_ASSERT_EQUAL(JitterPhase::Arming, p.diag().phase);
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(6010, out));
    TEST_ASSERT_EQUAL(JitterPhase::Idle, p.diag().phase); // 5 s arming kept
    TEST_ASSERT_EQUAL_STRING("timeout", p.diag().reason);

    DashJitterProcedure q;
    arm(q);
    q.observeNative045(kNative045, 1000);
    reachDisengaging(q, 1010); // deadline 1010+10000
    q.observeDasStatus(1, 1040);
    q.observeDasStatus(2, 1050);                              // deadline 1050+10000 = 11050
    TEST_ASSERT_EQUAL(JitterAction::None, q.tick(9050, out)); // +8 s: alive
    TEST_ASSERT_EQUAL(JitterPhase::Disengaging, q.diag().phase);
    TEST_ASSERT_EQUAL(JitterAction::None, q.tick(11050, out)); // boundary
    TEST_ASSERT_EQUAL(JitterPhase::Idle, q.diag().phase);
    TEST_ASSERT_EQUAL_UINT32(1, q.diag().timeoutResets);
}

void test_v1193_session_cap_deleted_rearms_after_failure()
{
    // v1.19.1 stood down after ONE refused cycle (kSessionFailCap=1) — the
    // 2026-09-15 field pass caught the machine spectating 3..4 unprotected
    // captures because of it. v1.19.3: failedCycles_ is diagnostic only;
    // every subsequent human engage is protected.
    DashJitterProcedure p;
    arm(p);
    p.observeNative045(kNative045, 1000);
    reachDisengaging(p, 1010);
    p.observeDasStatus(1, 1040);
    p.observeDasStatus(2, 1050); // the one re-request; the car refuses
    uint8_t out[8];
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(11050, out)); // 10 s expiry
    TEST_ASSERT_EQUAL_UINT8(1, p.diag().failedCycles);         // billed (diag)
    TEST_ASSERT_EQUAL(JitterPhase::Idle, p.diag().phase);
    // The human engages AGAIN: protection re-arms immediately.
    p.observeDasStatus(1, 12000); // Idle -> Normal
    p.observeDasStatus(3, 12010);
    TEST_ASSERT_EQUAL(JitterPhase::Arming, p.diag().phase);
    TEST_ASSERT_EQUAL_STRING("arming", p.diag().reason);
    // ...and after a SECOND refusal there is still no stand-down.
    p.observeDasStatus(6, 12020);
    TEST_ASSERT_EQUAL(JitterPhase::Disengaging, p.diag().phase);
    p.observeDasStatus(1, 12030);
    p.observeDasStatus(2, 12040);
    TEST_ASSERT_EQUAL_UINT32(2, p.diag().requests);
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(22050, out)); // second expiry
    TEST_ASSERT_EQUAL_UINT8(2, p.diag().failedCycles);
    p.observeDasStatus(1, 23000);
    p.observeDasStatus(3, 23010);
    TEST_ASSERT_EQUAL(JitterPhase::Arming, p.diag().phase); // protected again
}

void test_v1193_late_landing_counts_within_10s()
{
    // The measured response band runs to 7.1 s — beyond the old 5 s
    // deadline. A landing at +7 s lands INSIDE the window: it counts as
    // FSD success and the deadline expires with success billing (no
    // failed cycle). v1.19.4: the window is also clone-free — v1.19.3
    // pinned 34 hold frames across exactly this loop; tick() must return
    // nothing but silence now (bit46 continuity is the base path's
    // live-frame rewrite, fed by the ~1 Hz native mux0 carrier that
    // observe keeps fresh, matching production wiring).
    DashJitterProcedure p;
    arm(p);
    uint8_t tmpl[8] = {0, 0, 0, 0, 0, 0x01, 0, 0};
    p.observeNative3eeMux0(tmpl, 1000);
    p.observeNative045(kNative045, 1000);
    reachDisengaging(p, 1010);
    uint8_t out[8];
    TEST_ASSERT_EQUAL(JitterAction::Bit46Shot, p.tick(1015, out)); // arming shot
    p.observeDasStatus(1, 1040);
    p.observeDasStatus(2, 1050);
    uint32_t clones = 0;
    for (uint32_t t = 1100; t <= 8000; t += 100)
    {
        if ((t % 1000) == 0)
            p.observeNative3eeMux0(tmpl, t);
        if (p.tick(t, out) == JitterAction::Bit46Shot)
            ++clones;
    }
    TEST_ASSERT_EQUAL_UINT32(0, clones); // was 34 hold frames in v1.19.3
    // Late landing at +7.05 s (the field's manual-stalk timings).
    p.observeDasStatus(6, 8100);
    TEST_ASSERT_EQUAL_STRING("reEngagedFsd", p.diag().reason);
    TEST_ASSERT_EQUAL_UINT32(1, p.diag().reengagedFsd);
    // Nothing until the deadline — the window stands itself down silently.
    p.observeNative3eeMux0(tmpl, 8500);
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(8600, out));
    TEST_ASSERT_EQUAL_UINT32(1, p.diag().bit46Shots); // arming shot only
    // Deadline 1050+10000 expires with success billing.
    TEST_ASSERT_EQUAL(JitterAction::None, p.tick(11050, out));
    TEST_ASSERT_EQUAL(JitterPhase::Idle, p.diag().phase);
    TEST_ASSERT_EQUAL_UINT8(0, p.diag().failedCycles);
}

// ─── diag snapshot ────────────────────────────────────────────────────
void test_diag_reflects_armed_state()
{
    DashJitterProcedure p;
    arm(p);
    DashJitterDiag d = p.diag();
    TEST_ASSERT_TRUE(d.requested);
    TEST_ASSERT_TRUE(d.apGateOpen);
    TEST_ASSERT_TRUE(d.permit);
    TEST_ASSERT_TRUE(d.effective);
    TEST_ASSERT_EQUAL(JitterPhase::Idle, d.phase);
    p.observeDasStatus(6, 100); // event recorded even in Idle
    TEST_ASSERT_EQUAL_UINT8(6, p.diag().apState);
}

// ─── runner ───────────────────────────────────────────────────────────

int main()
{
    UNITY_BEGIN();

    // Constants
    RUN_TEST(test_constants_pin_littlegong_values);
    RUN_TEST(test_crc8_j1850_golden);
    RUN_TEST(test_counter_of_high_nibble);

    // Configure / gates
    RUN_TEST(test_default_is_inert_and_off);
    RUN_TEST(test_gate_closed_resets_to_inert);
    RUN_TEST(test_disabled_switch_resets_to_inert);

    // Phase machine
    RUN_TEST(test_state0_any_nonzero_event_enters_monitoring);
    RUN_TEST(test_event3_enters_arming_and_queues_one_shot);
    RUN_TEST(test_event6_gates_on_one_cycle_per_period);
    RUN_TEST(test_disengaging_event1_notes_cancel_seen_event2_re_requests);
    RUN_TEST(test_events_8_9_10_full_reset_from_any_active_phase);
    RUN_TEST(test_arming_timeout_full_reset);
    RUN_TEST(test_permit_required_for_arming_entry);
    RUN_TEST(test_permit_loss_mid_procedure_resets);

    // One-shot
    RUN_TEST(test_one_shot_patches_native_template);
    RUN_TEST(test_one_shot_or_in_preserves_existing_bits);
    RUN_TEST(test_one_shot_skipped_without_fresh_template);

    // Pump
    RUN_TEST(test_pump_only_rides_fresh_carrier);
    RUN_TEST(test_pump_respects_3ms_gap);
    RUN_TEST(test_pump_caps_at_16_frames);
    RUN_TEST(test_pump_first_counter_is_template_plus_one_then_increments);
    RUN_TEST(test_pump_request_gesture_0x42);
    RUN_TEST(test_tx_fail_silently_stops_pump);
    RUN_TEST(test_tx_ok_counted);

    // Steering abort
    RUN_TEST(test_steer_over_45_aborts_burst_only);
    RUN_TEST(test_steer_over_90_full_reset);
    RUN_TEST(test_steer_ignored_without_procedure_or_invalid_type);

    // v1.19.4 window carve-out + clone extinction
    RUN_TEST(test_procedure_active_only_in_arming_and_disengaging);
    RUN_TEST(test_in_disengaging_wait_window_scoped_accessor);
    RUN_TEST(test_disengaging_window_emits_no_bit46_clones);

    // Reset semantics
    RUN_TEST(test_full_reset_clears_cycle_state_keeps_diag_counters);

    // v1.19.1 field-incident regressions
    RUN_TEST(test_v1191_incident_replay_level_feed_terminates);
    RUN_TEST(test_v1191_reengage_success_keeps_protection_alive);
    RUN_TEST(test_v1191_bounce_12_fires_one_request);
    RUN_TEST(test_v1191_park_midcycle_does_not_poison_next_session);

    // v1.19.2 landing split + v1.19.4 refresh extinction
    RUN_TEST(test_v1192_landing_split_fsd_vs_eap);
    RUN_TEST(test_v1194_no_refresh_clone_before_re_request);

    // v1.19.3 widened wait + cap deletion (hold RETIRED in v1.19.4)
    RUN_TEST(test_v1193_disengaging_window_10s_arming_5s);
    RUN_TEST(test_v1193_session_cap_deleted_rearms_after_failure);
    RUN_TEST(test_v1193_late_landing_counts_within_10s);

    // Diag
    RUN_TEST(test_diag_reflects_armed_state);

    return UNITY_END();
}

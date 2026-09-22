#pragma once

#include <cstdint>

#include "can_frame_types.h"
#include "can_helpers.h"
#include "dash_jitter_procedure.h"
#include "drivers/can_driver.h"

// Legacy 0x045 one-shot stalk NAG.
//
// Vehicle evidence from the target car shows one warning episode as:
//   AP 6 -> 1, Hands=3; AP 1 -> 2, Hands=2; AP 2 -> 6, Hands=1.
// Only the first edge is actionable.  The remaining transitions clear and
// re-arm the latch; they must never synthesize additional stalk actions.
//
// 0x045 integrity byte: data[7] is CRC8-J1850 over bytes 0..6 (init 0xFF,
// poly 0x1D, final XOR 0xFF — jitter_045::crc8J1850, field-verified against
// 3202/3202 real frames).  It is NOT the additive vehicle checksum that
// 0x370/0x3EE use; the two algorithms are not interchangeable.
struct DashLegacyStalkNagDiag
{
    bool enabled{false};
    bool armed{true};
    bool pressPending{false};
    bool releasePending{false};
    bool templateValid{false};
    uint8_t previousApState{0};
    uint8_t apState{0};
    uint8_t handsOnState{0};
    uint32_t warningEdges{0};
    uint32_t pressTxOk{0};
    uint32_t pressTxFail{0};
    uint32_t releaseTxOk{0};
    uint32_t releaseTxFail{0};
    uint32_t blockedPhysicalStalk{0};
    uint32_t blockedBadTemplate{0};
    uint32_t blockedOuterGate{0};
    const char *blockedReason{"off"};
};

enum class DashLegacyStalkNagTxResult : uint8_t
{
    None = 0,
    Sent = 1,
    Failed = 2,
};

class DashLegacyStalkNag
{
public:
    static constexpr uint32_t kStalkCanId = 0x045;
    static constexpr uint32_t kDasStatusCanId = 0x399;
    static constexpr uint8_t kTriggerPreviousAp = 6;
    static constexpr uint8_t kTriggerAp = 1;
    static constexpr uint8_t kTriggerHands = 3;
    static constexpr uint8_t kResumeAction = 2;
    static constexpr uint8_t kNeutralAction = 0;
    static constexpr uint32_t kClearDebounceMs = 500;
    static constexpr uint32_t kCooldownMs = 10000;

    void setEnabled(bool enabled)
    {
        if (enabled_ == enabled)
            return;
        enabled_ = enabled;
        resetRuntime();
        blockedReason_ = enabled ? "waiting399" : "off";
    }

    void resetRuntime()
    {
        seenAp_ = false;
        armed_ = true;
        pressPending_ = false;
        releasePending_ = false;
        templateValid_ = false;
        previousApState_ = 0;
        apState_ = 0;
        handsOnState_ = 0;
        clearSinceMs_ = 0;
        blockedReason_ = enabled_ ? "waiting399" : "off";
    }

    void onDasStatus(uint8_t apState, uint8_t handsOnState, uint32_t nowMs)
    {
        if (!enabled_)
            return;

        const uint8_t previous = apState_;
        previousApState_ = previous;
        apState_ = apState;
        handsOnState_ = handsOnState;

        if (seenAp_ && armed_ && previous == kTriggerPreviousAp &&
            apState == kTriggerAp && handsOnState == kTriggerHands &&
            cooldownExpired(nowMs))
        {
            armed_ = false;
            pressPending_ = true;
            releasePending_ = false;
            clearSinceMs_ = 0;
            ++warningEdges_;
            blockedReason_ = "waiting045";
        }
        seenAp_ = true;

        // The captured recovery tail ends at AP=6, Hands=1.  Require it to
        // remain clear for 500 ms before allowing a future 6->1 event.
        if (!armed_ && apState == kTriggerPreviousAp && handsOnState <= 1)
        {
            if (clearSinceMs_ == 0)
                clearSinceMs_ = nowMs;
            else if (static_cast<uint32_t>(nowMs - clearSinceMs_) >= kClearDebounceMs)
            {
                armed_ = true;
                pressPending_ = false;
                releasePending_ = false;
                clearSinceMs_ = 0;
                blockedReason_ = "armed";
            }
        }
        else
        {
            clearSinceMs_ = 0;
        }
    }

    // Outer safety gates (vehicle OTA session active / abort guard latched)
    // are evaluated by the caller — same permit family as the JITTER tick in
    // main.cpp. The AP injection gate is deliberately NOT among them: the
    // resume fires on the AP 6->1 warning drop, an AP-inactive moment by
    // definition, so an AP-active requirement would veto every press.
    bool hasPendingAction() const { return pressPending_ || releasePending_; }

    DashLegacyStalkNagTxResult onStalkFrame(
        const CanFrame &frame,
        CanDriver &driver,
        uint32_t nowMs,
        bool outerGatesAllowed = true)
    {
        if (!enabled_ || frame.id != kStalkCanId)
            return DashLegacyStalkNagTxResult::None;

        if (!outerGatesAllowed)
        {
            // OTA session or abort latch owns the bus: drop any pending
            // action outright (the JITTER permit-loss -> fullReset precedent).
            // A resume fired at a stale moment once the gate reopens is worse
            // than a dropped assist — the driver can pull the stalk.
            pressPending_ = false;
            releasePending_ = false;
            ++blockedOuterGate_;
            blockedReason_ = "outerGate";
            return DashLegacyStalkNagTxResult::None;
        }

        templateValid_ = validTemplate(frame);
        if (!templateValid_)
        {
            ++blockedBadTemplate_;
            blockedReason_ = "bad045";
            return DashLegacyStalkNagTxResult::None;
        }

        const uint8_t physicalAction = static_cast<uint8_t>(frame.data[0] & 0x3FU);
        if (physicalAction != kNeutralAction || (frame.data[0] & 0x80U) != 0)
        {
            ++blockedPhysicalStalk_;
            blockedReason_ = "physicalStalk";
            return DashLegacyStalkNagTxResult::None;
        }

        // Complete the explicit release first.  One attempt only: ordinary
        // OEM neutral frames continue on the bus even if this duplicate send
        // fails, so an unbounded retry loop is neither necessary nor safe.
        if (releasePending_)
        {
            const CanFrame release = buildEcho(frame, kNeutralAction);
            releasePending_ = false;
            const bool ok = driver.send(release);
            if (ok)
            {
                ++releaseTxOk_;
                blockedReason_ = "latched";
                return DashLegacyStalkNagTxResult::Sent;
            }
            ++releaseTxFail_;
            blockedReason_ = "releaseTxFail";
            return DashLegacyStalkNagTxResult::Failed;
        }

        if (!pressPending_)
        {
            blockedReason_ = armed_ ? "armed" : "latched";
            return DashLegacyStalkNagTxResult::None;
        }

        pressPending_ = false;
        lastPulseMs_ = nowMs;
        const CanFrame press = buildEcho(frame, kResumeAction);
        const bool ok = driver.send(press);
        if (ok)
        {
            ++pressTxOk_;
            releasePending_ = true;
            blockedReason_ = "releasePending";
            return DashLegacyStalkNagTxResult::Sent;
        }

        ++pressTxFail_;
        blockedReason_ = "pressTxFail";
        return DashLegacyStalkNagTxResult::Failed;
    }

    DashLegacyStalkNagDiag diag() const
    {
        DashLegacyStalkNagDiag out;
        out.enabled = enabled_;
        out.armed = armed_;
        out.pressPending = pressPending_;
        out.releasePending = releasePending_;
        out.templateValid = templateValid_;
        out.previousApState = previousApState_;
        out.apState = apState_;
        out.handsOnState = handsOnState_;
        out.warningEdges = warningEdges_;
        out.pressTxOk = pressTxOk_;
        out.pressTxFail = pressTxFail_;
        out.releaseTxOk = releaseTxOk_;
        out.releaseTxFail = releaseTxFail_;
        out.blockedPhysicalStalk = blockedPhysicalStalk_;
        out.blockedBadTemplate = blockedBadTemplate_;
        out.blockedOuterGate = blockedOuterGate_;
        out.blockedReason = blockedReason_;
        return out;
    }

    static bool validTemplate(const CanFrame &frame)
    {
        // CRC8-J1850 over bytes 0..6 — see the header comment. The additive
        // vehicle checksum never matches a genuine 0x045, so using it here
        // would leave the module dead in "bad045" forever.
        return frame.id == kStalkCanId && frame.dlc >= 8 &&
               jitter_045::crc8J1850(frame.data, 7) == frame.data[7];
    }

    static CanFrame buildEcho(const CanFrame &source, uint8_t action)
    {
        CanFrame out = source;
        // data[0][5:0] SpdCtrlLvr_Stat: 0=neutral, 2=RWD/Resume.
        // Preserve VSL enable (bit 6), clear the invalid bit (bit 7).
        // The rolling counter (data[6] high nibble) is cloned from the
        // source frame as-is; only the gesture bits and the CRC change.
        out.data[0] = static_cast<uint8_t>((source.data[0] & 0x40U) |
                                           (action & 0x3FU));
        out.data[7] = jitter_045::crc8J1850(out.data, 7);
        return out;
    }

private:
    bool cooldownExpired(uint32_t nowMs) const
    {
        return lastPulseMs_ == 0 ||
               static_cast<uint32_t>(nowMs - lastPulseMs_) >= kCooldownMs;
    }

    bool enabled_{false};
    bool seenAp_{false};
    bool armed_{true};
    bool pressPending_{false};
    bool releasePending_{false};
    bool templateValid_{false};
    uint8_t previousApState_{0};
    uint8_t apState_{0};
    uint8_t handsOnState_{0};
    uint32_t clearSinceMs_{0};
    uint32_t lastPulseMs_{0};
    uint32_t warningEdges_{0};
    uint32_t pressTxOk_{0};
    uint32_t pressTxFail_{0};
    uint32_t releaseTxOk_{0};
    uint32_t releaseTxFail_{0};
    uint32_t blockedPhysicalStalk_{0};
    uint32_t blockedBadTemplate_{0};
    uint32_t blockedOuterGate_{0};
    const char *blockedReason_{"off"};
};

#pragma once

#include <cstdint>
#include <cstring>

#include "can_frame_types.h"
#include "dash_tx_evidence.h"

// Legacy 0x045 bit27 clone DND — a STATUS declaration, not an action spoof.
//
// The retired wheel/stalk gesture routes simulated a driver interaction
// (scroll tick / speed-lever nudge). The car counts those as one-shot
// attention evidence and, accumulated without real torque, inevitably
// strikes out (handsState 7: red wheel, forced takeover, 5 strikes = a week
// of Autosteer suspension).
//
// This module instead periodically re-declares the steering-wheel sensor
// condition itself: it clones the latest native 0x45 frame off the single
// CAN bus (the DAS/CH line — dual-CAN X197 13/14 equivalent), sets bit27 =
// StW_Cond_Flt (data[3] bit3), bumps
// the rolling counter and re-CRCs. DAS consumes the newest 0x45 value, so
// bit27=1 persists and the hands-on nag never arms — prevention, not
// clearance, hence no interaction debt and no strike-out.
//
// Ported verbatim from the dual-CAN 6.0.0-beta17 firmware (commit 7f24209,
// docs/LEGACY-0X045-BIT27-CLONE-DESIGN-20261118.md there): same engine, same
// parameters; only the injection line differs (one bus on this board).
//
// Reference-firmware parameters (field-verified, do not tune ad hoc):
//   - window: DAS_autopilotHandsOnState in {2..5} — engage from
//     REQD_NOT_DETECTED (pre-warning); HARD FAIL-CLOSED at 6+ so the module
//     never contests the strike-out domain.
//   - gate: DAS autopilotState > 3 (AP active domain).
//   - interval: 1500 ms per clone frame.
//
// Safety rails copied from the wheel-DND discipline: 150 ms native freshness,
// rolling-counter continuation, 30 ms own-echo fingerprint drop, physical
// scroll frames are never cloned as a base (never amplify the human), two
// consecutive TX failures auto-disarm until the switch is toggled.

enum class DashLegacy045Clone27State : uint8_t
{
    Idle = 0,
    WaitingWindow,
    WaitingSource,
    Cooldown,
    ClonePending,
    AutoDisarmed,
};

enum class DashLegacy045Clone27Block : uint8_t
{
    None = 0,
    Disabled,
    NotLegacy,
    ApInactive,
    OutsideWindow,
    WarningCleared,
    NoNative,
    StaleNative,
    InvalidNative,
    OtaGuard,
    TwaiUnavailable,
    ContextStale,
    PhysicalInput,
    TxFailed,
    AutoDisarmed,
    AbortGuard,
    StalkOwnedByJitter,
    Cooldown,
};

inline const char *dashLegacy045Clone27StateName(DashLegacy045Clone27State state)
{
    switch (state)
    {
    case DashLegacy045Clone27State::Idle:
        return "idle";
    case DashLegacy045Clone27State::WaitingWindow:
        return "waiting_window";
    case DashLegacy045Clone27State::WaitingSource:
        return "waiting_source";
    case DashLegacy045Clone27State::Cooldown:
        return "cooldown";
    case DashLegacy045Clone27State::ClonePending:
        return "clone_pending";
    case DashLegacy045Clone27State::AutoDisarmed:
        return "auto_disarmed";
    }
    return "unknown";
}

inline const char *dashLegacy045Clone27BlockName(DashLegacy045Clone27Block reason)
{
    switch (reason)
    {
    case DashLegacy045Clone27Block::None:
        return "none";
    case DashLegacy045Clone27Block::Disabled:
        return "disabled";
    case DashLegacy045Clone27Block::NotLegacy:
        return "not_legacy";
    case DashLegacy045Clone27Block::ApInactive:
        return "ap_inactive";
    case DashLegacy045Clone27Block::OutsideWindow:
        return "outside_window";
    case DashLegacy045Clone27Block::WarningCleared:
        return "warning_cleared";
    case DashLegacy045Clone27Block::NoNative:
        return "no_native";
    case DashLegacy045Clone27Block::StaleNative:
        return "stale_native";
    case DashLegacy045Clone27Block::InvalidNative:
        return "invalid_native";
    case DashLegacy045Clone27Block::OtaGuard:
        return "ota_guard";
    case DashLegacy045Clone27Block::TwaiUnavailable:
        return "twai_unavailable";
    case DashLegacy045Clone27Block::ContextStale:
        return "context_stale";
    case DashLegacy045Clone27Block::PhysicalInput:
        return "physical_input";
    case DashLegacy045Clone27Block::TxFailed:
        return "tx_failed";
    case DashLegacy045Clone27Block::AutoDisarmed:
        return "auto_disarmed";
    case DashLegacy045Clone27Block::AbortGuard:
        return "abort_guard";
    case DashLegacy045Clone27Block::StalkOwnedByJitter:
        return "stalk_owned_by_jitter";
    case DashLegacy045Clone27Block::Cooldown:
        return "cooldown";
    }
    return "unknown";
}

struct DashLegacy045Clone27Diag
{
    bool configured = false;
    bool runtimeArmed = false;
    bool autoDisarmed = false;
    bool apGateOk = false;
    bool inWindow = false;
    bool nativeSeen = false;
    bool nativeFresh = false;
    uint8_t apState = 0;
    uint8_t handsState = 0;
    uint8_t dasStatusBus = CAN_BUS_ANY;
    uint8_t nativeCounter = 0;
    uint8_t cloneCounter = 0;
    uint8_t cloneCrc = 0;
    uint8_t consecutiveTxFailures = 0;
    uint32_t dasStatusMs = 0;
    uint32_t nativeMs = 0;
    uint32_t nativeRx = 0;
    uint32_t nativeIdleRx = 0;
    uint32_t nativeInvalid = 0;
    uint32_t physicalSkip = 0;
    uint32_t windowEntries = 0;
    uint32_t cloneTx = 0;
    uint32_t cloneTxFailed = 0;
    uint32_t ownEchoRx = 0;
    uint32_t cooldownRemainingMs = 0;
    uint32_t lastCloneMs = 0;
    DashLegacy045Clone27State state = DashLegacy045Clone27State::Idle;
    DashLegacy045Clone27Block blocked = DashLegacy045Clone27Block::Disabled;
    DashTxEvidence tx{};
};

struct DashLegacy045Clone27
{
    static constexpr uint32_t kCanId = 0x045;
    static constexpr uint32_t kNativeMaxAgeMs = 150;
    static constexpr uint32_t kContextMaxAgeMs = 1000;
    // Reference-firmware parameters (see file header). kApGateState uses
    // "> 3" verbatim; keep as constants so a field finding only touches these.
    static constexpr uint32_t kCloneIntervalMs = 1500;
    static constexpr uint8_t kApGateStateAbove = 3;
    static constexpr uint8_t kWindowLo = 2;
    static constexpr uint8_t kWindowHi = 5;
    static constexpr uint8_t kBit27Mask = 0x08; // StW_Cond_Flt, data[3] bit3
    static constexpr uint32_t kOwnEchoWindowMs = 30;
    static constexpr uint8_t kMaxConsecutiveTxFailures = 2;
    static constexpr uint16_t kAnyWheelMask = 0x3FFF;

    DashLegacy045Clone27Diag diag{};

    static uint8_t crc8J1850(const uint8_t *data, uint8_t length)
    {
        uint8_t crc = 0xFF;
        for (uint8_t i = 0; i < length; ++i)
        {
            crc ^= data[i];
            for (uint8_t bit = 0; bit < 8; ++bit)
                crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x1D)
                                   : static_cast<uint8_t>(crc << 1);
        }
        return static_cast<uint8_t>(crc ^ 0xFF);
    }

    static bool validFrame(const uint8_t data[8])
    {
        return crc8J1850(data, 7) == data[7];
    }

    static uint8_t counter(const uint8_t data[8])
    {
        return static_cast<uint8_t>((data[6] >> 4) & 0x0F);
    }

    static bool isIdleWheelFrame(const uint8_t data[8])
    {
        const uint16_t sw = static_cast<uint16_t>(data[4]) |
                            static_cast<uint16_t>(static_cast<uint16_t>(data[5]) << 8);
        return (sw & kAnyWheelMask) == 0;
    }

    void configure(bool enabled)
    {
        if (diag.configured == enabled)
        {
            updateArmed();
            return;
        }
        diag.configured = enabled;
        if (!enabled)
        {
            pendingTx_ = false;
            autoDisarmed_ = false;
            diag.autoDisarmed = false;
            diag.consecutiveTxFailures = 0;
            consecutiveTxFailures_ = 0;
            setState(DashLegacy045Clone27State::Idle, DashLegacy045Clone27Block::Disabled);
        }
        updateArmed();
    }

    void setRuntimeAllowed(bool allowed, DashLegacy045Clone27Block deniedReason)
    {
        const DashLegacy045Clone27Block nextBlock =
            allowed ? DashLegacy045Clone27Block::None : deniedReason;
        if (runtimeAllowed_ == allowed && runtimeBlock_ == nextBlock)
        {
            updateArmed();
            return;
        }
        runtimeAllowed_ = allowed;
        runtimeBlock_ = nextBlock;
        if (!allowed)
            pendingTx_ = false;
        updateArmed();
    }

    void onDasStatus(uint8_t apState, uint8_t handsState, uint8_t sourceBus, uint32_t nowMs)
    {
        const bool wasInWindow = inWindow_;
        apGateOk_ = apState > kApGateStateAbove;
        inWindow_ = handsState >= kWindowLo && handsState <= kWindowHi;
        contextMs_ = nowMs;
        diag.apState = apState;
        diag.handsState = handsState;
        diag.dasStatusBus = sourceBus;
        diag.dasStatusMs = nowMs;
        diag.apGateOk = apGateOk_;
        diag.inWindow = inWindow_;
        if (inWindow_ && !wasInWindow)
        {
            ++diag.windowEntries;
            // A fresh window entry may clone immediately: the declaration
            // should be on the bus before the nag can escalate.
            if (!episodeActive_)
            {
                lastCloneMs_ = 0; // first frame immediate
                episodeActive_ = true;
            }
        }
        else if (!inWindow_)
        {
            episodeActive_ = false;
            pendingTx_ = false;
        }
    }

    void onNative045(const uint8_t data[8], uint8_t dlc, uint32_t nowMs)
    {
        ++diag.nativeRx;
        if (dlc != 8 || !validFrame(data))
        {
            ++diag.nativeInvalid;
            return;
        }
        if (hasTxFingerprint_ && nowMs - txFingerprintMs_ <= kOwnEchoWindowMs &&
            memcmp(data, txFingerprint_, sizeof(txFingerprint_)) == 0)
        {
            ++diag.ownEchoRx;
            return;
        }
        if (!isIdleWheelFrame(data))
        {
            // Never clone on top of a human scroll: the physical driver owns
            // the wheel fields while they are active.
            ++diag.physicalSkip;
            return;
        }
        memcpy(nativeData_, data, sizeof(nativeData_));
        nativeMs_ = nowMs;
        nativeSeen_ = true;
        diag.nativeSeen = true;
        diag.nativeFresh = true;
        diag.nativeMs = nowMs;
        diag.nativeCounter = counter(data);
        ++diag.nativeIdleRx;
    }

    bool nextFrame(uint32_t nowMs, uint8_t outData[8])
    {
        updateFresh(nowMs);
        updateArmed();
        if (!diag.runtimeArmed)
            return false; // updateArmed() already set the blocked reason

        if (!contextFresh(nowMs))
        {
            pendingTx_ = false;
            setState(DashLegacy045Clone27State::WaitingSource,
                     DashLegacy045Clone27Block::ContextStale);
            return false;
        }

        if (!apGateOk_ || !inWindow_)
        {
            pendingTx_ = false;
            setState(DashLegacy045Clone27State::WaitingWindow,
                     apGateOk_ ? DashLegacy045Clone27Block::OutsideWindow
                               : DashLegacy045Clone27Block::ApInactive);
            return false;
        }

        if (diag.state == DashLegacy045Clone27State::ClonePending && pendingTx_)
            return false; // one clone in flight

        if (!nativeSeen_)
        {
            setState(DashLegacy045Clone27State::WaitingSource,
                     DashLegacy045Clone27Block::NoNative);
            return false;
        }
        if (!nativeFresh(nowMs))
        {
            setState(DashLegacy045Clone27State::WaitingSource,
                     DashLegacy045Clone27Block::StaleNative);
            return false;
        }

        if (lastCloneMs_ != 0 && nowMs - lastCloneMs_ < kCloneIntervalMs)
        {
            diag.cooldownRemainingMs = lastCloneMs_ + kCloneIntervalMs - nowMs;
            setState(DashLegacy045Clone27State::Cooldown,
                     DashLegacy045Clone27Block::Cooldown);
            return false;
        }

        memcpy(outData, nativeData_, sizeof(nativeData_));
        outData[3] = static_cast<uint8_t>(outData[3] | kBit27Mask);
        const uint8_t cloneCounter =
            static_cast<uint8_t>((counter(nativeData_) + 1) & 0x0F);
        outData[6] =
            static_cast<uint8_t>((outData[6] & 0x0F) | (cloneCounter << 4));
        outData[7] = crc8J1850(outData, 7);
        memcpy(pendingData_, outData, sizeof(pendingData_));
        pendingTx_ = true;
        lastCloneMs_ = nowMs;
        diag.cloneCounter = cloneCounter;
        diag.cloneCrc = outData[7];
        diag.lastCloneMs = nowMs;
        diag.cooldownRemainingMs = kCloneIntervalMs;
        setState(DashLegacy045Clone27State::ClonePending, DashLegacy045Clone27Block::None);
        return true;
    }

    void recordTxResult(bool ok, uint32_t nowMs)
    {
        if (!pendingTx_)
            return;
        pendingTx_ = false;
        if (!ok)
        {
            ++diag.cloneTxFailed;
            if (consecutiveTxFailures_ < 0xFF)
                ++consecutiveTxFailures_;
            diag.consecutiveTxFailures = consecutiveTxFailures_;
            if (consecutiveTxFailures_ >= kMaxConsecutiveTxFailures)
            {
                autoDisarmed_ = true;
                diag.autoDisarmed = true;
                setState(DashLegacy045Clone27State::AutoDisarmed,
                         DashLegacy045Clone27Block::AutoDisarmed);
            }
            else
                setState(DashLegacy045Clone27State::Cooldown,
                         DashLegacy045Clone27Block::TxFailed);
            updateArmed();
            return;
        }

        consecutiveTxFailures_ = 0;
        diag.consecutiveTxFailures = 0;
        ++diag.cloneTx;
        memcpy(txFingerprint_, pendingData_, sizeof(txFingerprint_));
        txFingerprintMs_ = nowMs;
        hasTxFingerprint_ = true;
        setState(DashLegacy045Clone27State::Cooldown, DashLegacy045Clone27Block::None);
    }

    void resetRuntime()
    {
        const bool wasConfigured = diag.configured;
        pendingTx_ = false;
        nativeSeen_ = false;
        apGateOk_ = false;
        inWindow_ = false;
        episodeActive_ = false;
        autoDisarmed_ = false;
        consecutiveTxFailures_ = 0;
        lastCloneMs_ = 0;
        contextMs_ = 0;
        nativeMs_ = 0;
        hasTxFingerprint_ = false;
        diag = DashLegacy045Clone27Diag{};
        diag.configured = wasConfigured;
        updateArmed();
    }

private:
    uint8_t nativeData_[8] = {};
    uint8_t pendingData_[8] = {};
    uint8_t txFingerprint_[8] = {};
    uint32_t nativeMs_ = 0;
    uint32_t contextMs_ = 0;
    uint32_t txFingerprintMs_ = 0;
    uint32_t lastCloneMs_ = 0;
    uint8_t consecutiveTxFailures_ = 0;
    bool nativeSeen_ = false;
    bool runtimeAllowed_ = false;
    bool apGateOk_ = false;
    bool inWindow_ = false;
    bool episodeActive_ = false;
    bool autoDisarmed_ = false;
    bool pendingTx_ = false;
    bool hasTxFingerprint_ = false;
    DashLegacy045Clone27Block runtimeBlock_ = DashLegacy045Clone27Block::Disabled;

    bool nativeFresh(uint32_t nowMs) const
    {
        return nativeSeen_ && nowMs - nativeMs_ <= kNativeMaxAgeMs;
    }

    bool contextFresh(uint32_t nowMs) const
    {
        return contextMs_ != 0 && nowMs - contextMs_ <= kContextMaxAgeMs;
    }

    void updateFresh(uint32_t nowMs)
    {
        diag.nativeFresh = nativeFresh(nowMs);
    }

    void updateArmed()
    {
        diag.runtimeArmed = diag.configured && runtimeAllowed_ && !autoDisarmed_;
        if (!diag.configured)
        {
            diag.blocked = DashLegacy045Clone27Block::Disabled;
            if (diag.state != DashLegacy045Clone27State::Idle)
                diag.state = DashLegacy045Clone27State::Idle;
        }
        else if (!runtimeAllowed_)
            diag.blocked = runtimeBlock_;
        else if (autoDisarmed_)
            diag.blocked = DashLegacy045Clone27Block::AutoDisarmed;
        else
            diag.blocked = DashLegacy045Clone27Block::None;
    }

    void setState(DashLegacy045Clone27State state, DashLegacy045Clone27Block reason)
    {
        diag.state = state;
        diag.blocked = reason;
    }
};

#pragma once
// dash_power_mgmt.h — 功耗管理（Phase 1）
// 自动关机：5分钟无CAN数据 → deep sleep，DASH_WAKE_PIN/TWAI_RX_PIN 唤醒
// WiFi自动关闭：5分钟无Web请求 → 仅关STA保AP

#include <cstdint>
#include <esp_sleep.h>

// ── 配置参数 ─────────────────────────────────────────
#define CAN_TIMEOUT_MS 300000 // 5分钟
#define WEB_TIMEOUT_MS 300000 // 5分钟
#define NVS_KEY_AUTO_SHUTDOWN "auto_shutdown"
#define NVS_KEY_WIFI_AUTO_OFF "wifi_auto_off"

#ifndef DASH_WAKE_PIN
#ifdef TWAI_RX_PIN
#define DASH_WAKE_PIN TWAI_RX_PIN
#else
#define DASH_WAKE_PIN GPIO_NUM_4
#endif
#endif

// ── 全局状态 ─────────────────────────────────────────
static volatile bool autoShutdownEnabled = false;
static volatile bool wifiAutoOffEnabled = false;
static volatile uint32_t lastCanActivityMs = 0;
static volatile uint32_t lastWebActivityMs = 0;
static volatile bool wifiStaDisabled = false;       // STA已关闭标记
static volatile bool wifiStaRestorePending = false; // Web活动后待恢复STA

// Forward declarations for Arduino/ESP-IDF functions used
// These are available in the ESP-IDF/Arduino environment
extern unsigned long millis();

// WiFi mode constants - use WiFi object available globally
// WiFi.mode(WIFI_AP) = AP only, WiFi.mode(WIFI_AP_STA) = AP+STA

// ── 初始化 ───────────────────────────────────────────
inline void dashPowerMgmtConfigureWake()
{
    // CAN idle bus = recessive = transceiver RXD HIGH; dominant (traffic) = LOW.
    // Wake on level 0 so only real bus traffic wakes the device — level 1
    // would fire immediately on an idle bus and sleep would never hold.
    esp_sleep_enable_ext0_wakeup(DASH_WAKE_PIN, 0);
}

inline void dashPowerMgmtInit()
{
    if (autoShutdownEnabled)
    {
        dashPowerMgmtConfigureWake();
    }
    lastCanActivityMs = millis();
    lastWebActivityMs = millis();
}

// ── CAN活动更新 ──────────────────────────────────────
inline void dashPowerMgmtTouchCan()
{
    lastCanActivityMs = millis();
}

// ── Web活动更新 ──────────────────────────────────────
// Also requests STA restore if it was disabled by the idle timeout.
// The flag is left set until the main loop consumes it, so the STA
// suppression in dashCheckWifi stays armed until the restore path runs.
inline void dashPowerMgmtTouchWeb()
{
    lastWebActivityMs = millis();
    if (wifiStaDisabled)
        wifiStaRestorePending = true;
}

// Main loop calls this after the auto-off check: returns true exactly once
// per restore request so the caller can bring WIFI_AP_STA back and schedule
// a reconnect.
inline bool dashPowerMgmtConsumeStaRestore()
{
    if (!wifiStaRestorePending)
        return false;
    wifiStaRestorePending = false;
    wifiStaDisabled = false;
    return true;
}

// ── 主循环检查 ───────────────────────────────────────
// Returns true if device is about to enter deep sleep
inline bool dashPowerMgmtTick()
{
    uint32_t now = millis();

    // Auto-shutdown: deep sleep after CAN timeout
    if (autoShutdownEnabled && (now - lastCanActivityMs) > CAN_TIMEOUT_MS)
    {
        dashPowerMgmtConfigureWake();
        esp_deep_sleep_start();
        return true;
    }

    return false;
}

// ── Check if WiFi STA should be disabled (called from main loop) ──
inline bool dashPowerMgmtShouldDisableWifi()
{
    if (!wifiAutoOffEnabled || wifiStaDisabled)
        return false;
    return (millis() - lastWebActivityMs) > WEB_TIMEOUT_MS;
}

inline void dashPowerMgmtMarkWifiDisabled()
{
    wifiStaDisabled = true;
}

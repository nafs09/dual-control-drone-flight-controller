#pragma once
#include "../core/shared_state.hpp"
#include "cmsis_os.h"

namespace FC { namespace Safety {

/** Safety monitor — 100 Hz. Highest priority non-control task.
 *
 * BUG FIX #10: GPS fence used raw lat/lon degree deltas.
 *   1° latitude ≈ 111 km so the fence was meaningless.
 *   Fixed: uses EKF NED position (metres from home). See checkGeofence().
 *
 * BUG FIX #11: Original safety.c called while(1) on sensor fault.
 *   No reporting, no recovery, no watchdog feed. Replaced with
 *   proper triggerMotorKill() / triggerFailsafe() with IWDG feed. */
class SafetyMonitor {
public:
    void init();
    void update(uint32_t tick_ms);
    static void taskEntry(void* arg);

private:
    uint32_t prevDrdyCount_=0, drdyStallMs_=0;
    float    battVoltFilt_=12.6f;
    bool     originSet_=false;
    static constexpr float kBattAlpha=0.05f;

    void checkIMU(uint32_t tick_ms);
    void checkRC();
    void checkTilt();
    void checkBattery();
    void checkGeofence();
    void checkMotorLoss();
    void feedWatchdog();
    void triggerFailsafe(uint32_t errBit);
    void triggerMotorKill(uint32_t errBit);
};

}} // FC::Safety

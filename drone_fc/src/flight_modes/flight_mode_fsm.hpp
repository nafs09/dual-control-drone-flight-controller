
#pragma once
#include "../core/shared_state.hpp"
#include "../control/rate_controller.hpp"
#include "../control/attitude_controller.hpp"
#include "../control/position_controller.hpp"
#include "../control/velocity_controller.hpp"
#include "../actuation/mixer.hpp"
#include "../estimation/mahony_ahrs.hpp"
#include "../estimation/ekf.hpp"

namespace FC { namespace FlightModes {

using namespace Control;
using namespace Actuation;
using namespace Estimation;
using Math::Vector3f;
using Math::Quaternionf;

/**
 * @brief  Flight Mode State Machine
 *
 * FULL CAMERA-MODE CASCADE (all loops active):
 *
 *   GPS pos error [m]
 *       ↓  50 Hz  [PositionController]
 *   vel setpoint [m/s, NED]
 *       ↓  100 Hz [VelocityController]   ← NEW
 *   tilt demand [roll, pitch, rad]
 *       ↓  3840 Hz [AttitudeController]
 *   rate setpoint [rad/s]
 *       ↓  3840 Hz [RateController]
 *   torque [-1..+1]
 *       ↓  [Mixer]
 *   DSHOT[0..2047] × 4
 *
 * FPV MODE (all outer loops bypassed):
 *   RC sticks → rate setpoint → [RateController] → torque → [Mixer]
 *
 * CAMERA MODE (no GPS hold):
 *   RC sticks → vel setpoint → [VelocityController] → tilt
 *            → [AttitudeController] → rate setpoint → [RateController]
 *
 * CAMERA MODE (GPS hold active):
 *   pos error → [PositionController] → vel setpoint
 *            → [VelocityController] → tilt → [AttitudeController] → ...
 *
 * VELOCITY CONTROLLER DECIMATION:
 *   The control loop runs at ~3840 Hz (IMU-paced).
 *   The velocity controller runs at 100 Hz = every 38 IMU ticks.
 *   The last computed tilt from the velocity controller is held
 *   between its updates and used by the attitude controller every tick.
 *   This is correct because tilt changes are slow (<<100 Hz bandwidth).
 */
class FlightModeFSM {
public:
    FlightModeFSM();

    /**
     * @brief  Main update at IMU rate (~3840 Hz).
     * @param dt       True elapsed time since last call [s].
     * @param ahrs     Mahony AHRS.
     * @param ekf      EKF (position/velocity source).
     */
    void update(float dt, MahonyAHRS& ahrs, EKF& ekf);

    /**
     * @brief  Slower update at ~50 Hz for position loop.
     * @param dt   ~20 ms step.
     * @param ekf  EKF (position/velocity source).
     */
    void updatePositionLoop(float dt, const EKF& ekf);

    FlightMode getMode()  const { return g_state.mode; }
    ArmState   getArm()   const { return g_state.armState; }

private:
    /* ---- Sub-controllers ----------------------------------- */
    RateController     rateCtrl_;
    AttitudeController attCtrl_;
    PositionController posCtrl_;
    VelocityController velCtrl_;
    Mixer              mixer_;

    /* ---- Velocity controller state ------------------------- */
    /* Held between 100 Hz velocity updates, used every 3840 Hz tick */
    float   desiredRoll_rad_  = 0.f;
    float   desiredPitch_rad_ = 0.f;
    float   velThrustDelta_   = 0.f;

    /* Velocity setpoint — updated by pos controller or RC sticks */
    Vector3f velSetpoint_ned_ = {0.f, 0.f, 0.f};

    /* Decimation counters */
    uint16_t velDecim_ = 0;
    uint16_t posDecim_ = 0;
    static constexpr uint16_t kVelDecim = 38;   /* 3840/100 ≈ 38 → ~101 Hz */
    static constexpr uint16_t kPosDecim = 77;   /* 3840/50  ≈ 77 → ~49.9 Hz */

    /* ---- Mode helpers -------------------------------------- */
    void tryArm();
    void tryDisarm();
    void handleFailsafe();
    void enterFPV();
    void enterCamera();
    void enterLanding();

    /* ---- Per-mode update routines -------------------------- */
    void runFPV   (float dt, const Vector3f& gyro_rps);
    void runCamera(float dt, const Vector3f& gyro_rps,
                             const Quaternionf& attitude,
                             const Vector3f& vel_ned);

    static Vector3f rcToRateSetpointFPV(const RCInputs& rc);

    /* ---- State --------------------------------------------- */
    FlightMode prevMode_    = FlightMode::DISARMED;
    uint32_t   modeEntryMs_ = 0;
    float      landingThrottle_ = 0.3f;
    uint32_t   armHoldMs_  = 0;
    float      fsDescentRate_ = 0.f;

    static constexpr uint32_t kArmHoldMs = 500u;
    static constexpr float    kFsDescentRps = 0.5f;
};

}} // FC::FlightModes

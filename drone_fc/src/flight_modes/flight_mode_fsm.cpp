
#include "flight_mode_fsm.hpp"
#include "../../Inc/fc_config.h"
#include "cmsis_os.h"
#include <cmath>

namespace FC { namespace FlightModes {

FlightModeFSM::FlightModeFSM()
    : rateCtrl_(RateController::fpvPreset()),
      attCtrl_(AttitudeController::cameraGains()),
      posCtrl_(PositionController::cameraGains()),
      velCtrl_(VelocityController::cameraGains()),
      mixer_(Mixer::fpvConfig())
{}

/* ============================================================
 *  Main update — IMU rate (~3840 Hz)
 * ============================================================ */
void FlightModeFSM::update(float dt, InvEKF& ekf) {

    /* Read shared state atomically */
    RCInputs rc;
    osMutexAcquire(g_state.rcMtx, osWaitForever);
    rc = g_state.rc;
    osMutexRelease(g_state.rcMtx);

    osMutexAcquire(g_state.imuMtx, osWaitForever);
    Vector3f gyro  = g_state.gyro_rps;
    osMutexRelease(g_state.imuMtx);

    osMutexAcquire(g_state.ahMtx, osWaitForever);
    Quaternionf attitude = g_state.attitude;
    osMutexRelease(g_state.ahMtx);

    osMutexAcquire(g_state.navMtx, osWaitForever);
    Vector3f vel_ned = g_state.velocity_ned;
    osMutexRelease(g_state.navMtx);

    osMutexAcquire(g_state.safetyMtx, osWaitForever);
    bool killMotors = g_state.killMotors;
    osMutexRelease(g_state.safetyMtx);

    /* SafetyMonitor may enter FAILSAFE asynchronously (for battery/fence
     * faults). Initialise the autonomous descent on the transition itself. */
    if (g_state.mode == FlightMode::FAILSAFE && prevMode_ != FlightMode::FAILSAFE) {
        rateCtrl_.setPreset(RateController::cameraPreset());
        mixer_.setConfig(Mixer::cameraConfig());
        mixer_.enableMotors(true);
        attCtrl_.captureYaw(attitude.toEulerRPY().z);
        velCtrl_.reset();
        posCtrl_.reset();
        positionHoldActive_ = false;
        desiredRoll_rad_ = desiredPitch_rad_ = velThrustDelta_ = 0.0f;
        fsDescentRate_ = 0.0f;
        velSetpoint_ned_ = {0.0f, 0.0f, 0.0f};
    }

    /* GPS position hold is a 50 Hz outer loop. Run it before the 100 Hz
     * velocity loop so a coincident tick consumes the newest setpoint. */
    if (g_state.mode == FlightMode::CAMERA && ++posDecim_ >= kPosDecim) {
        posDecim_ = 0;
        updatePositionLoop(static_cast<float>(kPosDecim) / CTRL_LOOP_HZ, ekf);
    }

    /* Hard kill overrides everything */
    if (killMotors) {
        uint16_t z[4]={0,0,0,0};
        osMutexAcquire(g_state.motMtx, osWaitForever);
        for(int i=0;i<4;i++) g_state.motorCmd[i]=z[i];
        osMutexRelease(g_state.motMtx);
        g_state.mode=FlightMode::DISARMED;
        g_state.armState=ArmState::DISARMED;
        return;
    }

    /* ---- Velocity controller update (100 Hz) ----------------
     *
     * Runs every kVelDecim ticks.  Updates desiredRoll_rad_ and
     * desiredPitch_rad_ which are then used by the attitude
     * controller every tick at 3840 Hz.
     *
     * In camera mode the velocity setpoint comes from:
     *   - RC sticks (mapped to heading-relative velocity) if no GPS hold
     *   - posCtrl_ output (updated at 50 Hz) if GPS hold is active
     *
     * In FPV mode this block is skipped entirely.
     * --------------------------------------------------------- */
    if (g_state.mode == FlightMode::CAMERA ||
        g_state.mode == FlightMode::FAILSAFE) {

        if (++velDecim_ >= kVelDecim) {
            velDecim_ = 0;
            float dt_vel = static_cast<float>(kVelDecim) / CTRL_LOOP_HZ;

            Vector3f euler = attitude.toEulerRPY();
            float yaw = euler.z;

            osMutexAcquire(g_state.gpsMtx, osWaitForever);
            const bool gpsFixValid = g_state.gps.fixValid;
            osMutexRelease(g_state.gpsMtx);
            if (g_state.mode == FlightMode::FAILSAFE) {
                fsDescentRate_ = std::fmin(0.8f, fsDescentRate_ + kFsDescentRps * dt_vel);
                velSetpoint_ned_ = {0.f, 0.f, fsDescentRate_};
            } else if (!rc.altHold || !gpsFixValid) {
                /* No GPS hold: map sticks to NED velocity setpoint */
                velSetpoint_ned_ = VelocityController::sticksToVelSetpoint(
                    rc.pitch, rc.roll, yaw, CAM_MAX_VEL_HORIZ);
                /* Vertical: throttle stick → vertical velocity */
                velSetpoint_ned_.z = -(rc.throttle * 2.f - 1.f) * CAM_MAX_VEL_VERT;
            }
            /* else: velSetpoint_ned_ was set by updatePositionLoop() */

            auto out = velCtrl_.update(velSetpoint_ned_, vel_ned, yaw, dt_vel);
            desiredRoll_rad_  = out.roll_rad;
            desiredPitch_rad_ = out.pitch_rad;
            velThrustDelta_   = out.thrust_delta;
        }
    }

    /* ---- Main FSM ------------------------------------------ */
    switch (g_state.mode) {

    case FlightMode::DISARMED:
        { uint16_t k[4]={0,0,0,0};
          osMutexAcquire(g_state.motMtx, osWaitForever);
          for(int i=0;i<4;i++) g_state.motorCmd[i]=k[i];
          osMutexRelease(g_state.motMtx);
          mixer_.enableMotors(false); }
        tryArm();
        break;

    case FlightMode::ARMED:
        { uint16_t id[4];
          mixer_.idle(id);
          osMutexAcquire(g_state.motMtx, osWaitForever);
          for(int i=0;i<4;i++) g_state.motorCmd[i]=id[i];
          osMutexRelease(g_state.motMtx); }
        if (rc.valid && rc.throttle > 0.05f) {
            if (rc.modeSwitch) enterCamera(); else enterFPV();
        }
        tryDisarm();
        break;

    case FlightMode::FPV:
        if (!rc.valid)       { handleFailsafe(); break; }
        if (!rc.armSwitch)   { tryDisarm();      break; }
        if (rc.modeSwitch)   { enterCamera();    break; }
        runFPV(dt, gyro);
        break;

    case FlightMode::CAMERA:
        if (!rc.valid)       { handleFailsafe(); break; }
        if (!rc.armSwitch)   { tryDisarm();      break; }
        if (!rc.modeSwitch)  { enterFPV();       break; }
        runCamera(dt, gyro, attitude, vel_ned);
        break;

    case FlightMode::FAILSAFE:
        /* Velocity controller already updated above.
         * Run camera-style control for a gentle descent.            */
        runCamera(dt, gyro, attitude, vel_ned);
        { osMutexAcquire(g_state.navMtx, osWaitForever);
          float vd = g_state.velocity_ned.z;
          osMutexRelease(g_state.navMtx);
          osMutexAcquire(g_state.baroMtx, osWaitForever);
          const bool landed = g_state.baroValid && g_state.baro_alt_agl_m < 0.3f &&
                              std::fabs(vd) < 0.15f;
          osMutexRelease(g_state.baroMtx);
          if (landed) enterLanding(); }
        break;

    case FlightMode::LANDING:
        landingThrottle_ -= 0.2f * dt;
        if (landingThrottle_ < 0.f) {
            landingThrottle_ = 0.f;
            g_state.mode     = FlightMode::DISARMED;
            g_state.armState = ArmState::DISARMED;
            mixer_.enableMotors(false);
        } else {
            Vector3f torque = rateCtrl_.update({0,0,0}, gyro, dt);
            uint16_t cmds[4]; mixer_.mix(landingThrottle_, torque, cmds);
            osMutexAcquire(g_state.motMtx, osWaitForever);
            for(int i=0;i<4;i++) g_state.motorCmd[i]=cmds[i];
            osMutexRelease(g_state.motMtx);
        }
        break;

    default: break;
    }

    prevMode_ = g_state.mode;
}

/* ============================================================
 *  FPV — rate loop only, all outer loops bypassed
 * ============================================================ */
void FlightModeFSM::runFPV(float dt, const Vector3f& gyro_rps) {
    RCInputs rc;
    osMutexAcquire(g_state.rcMtx, osWaitForever); rc=g_state.rc;
    osMutexRelease(g_state.rcMtx);

    Vector3f rateSp = rcToRateSetpointFPV(rc);
    Vector3f torque = rateCtrl_.update(rateSp, gyro_rps, dt);
    uint16_t cmds[4];
    mixer_.mix(rc.throttle, torque, cmds);
    osMutexAcquire(g_state.motMtx, osWaitForever);
    for(int i=0;i<4;i++) g_state.motorCmd[i]=cmds[i];
    osMutexRelease(g_state.motMtx);
    g_state.rateSetpoint_rps=rateSp; g_state.torqueOutput=torque;
    g_state.thrustOutput=rc.throttle;
}

/* ============================================================
 *  Camera — full cascade: vel → attitude → rate
 *
 *  The velocity controller already ran at 100 Hz and stored
 *  desiredRoll_rad_ / desiredPitch_rad_.  Here we just pass
 *  those stored tilt angles to the attitude controller which
 *  runs every tick at 3840 Hz for tight attitude tracking.
 * ============================================================ */
void FlightModeFSM::runCamera(float dt, const Vector3f& gyro_rps,
                               const Quaternionf& attitude,
                               const Vector3f& /*vel_ned*/)
{
    RCInputs rc;
    osMutexAcquire(g_state.rcMtx, osWaitForever); rc=g_state.rc;
    osMutexRelease(g_state.rcMtx);

    /* Yaw: from stick directly, integrated into desired heading */
    float maxYawRps = CAM_MAX_YAW_RATE * (3.14159265f / 180.f);
    float yawRateDem = g_state.mode == FlightMode::FAILSAFE ? 0.f : rc.yaw * maxYawRps;

    /* Build desired quaternion from the velocity-controller tilt.
     * desiredRoll_rad_ / desiredPitch_rad_ are body-frame tilt angles
     * output by the velocity controller at 100 Hz (held between updates).
     * The attitude controller converts these to a quaternion and then
     * runs the proportional quaternion-error law at 3840 Hz.             */
    Quaternionf qDes = attCtrl_.buildDesiredAttitude(
        desiredRoll_rad_  / (CAM_MAX_ROLL_ANGLE  * 3.14159265f / 180.f),
        desiredPitch_rad_ / (CAM_MAX_PITCH_ANGLE * 3.14159265f / 180.f),
        yawRateDem,
        CAM_MAX_ROLL_ANGLE  * (3.14159265f / 180.f),
        CAM_MAX_PITCH_ANGLE * (3.14159265f / 180.f),
        dt);

    Vector3f rateSp = attCtrl_.update(qDes, attitude);
    Vector3f torque = rateCtrl_.update(rateSp, gyro_rps, dt);

    /* Thrust: base from velCtrl vertical output + small hover offset */
    float thrust = 0.5f + velThrustDelta_;
    thrust = thrust > 1.f ? 1.f : (thrust < 0.f ? 0.f : thrust);

    uint16_t cmds[4];
    mixer_.mix(thrust, torque, cmds);
    osMutexAcquire(g_state.motMtx, osWaitForever);
    for(int i=0;i<4;i++) g_state.motorCmd[i]=cmds[i];
    osMutexRelease(g_state.motMtx);
    g_state.rateSetpoint_rps=rateSp; g_state.torqueOutput=torque;
    g_state.thrustOutput=thrust;
}

/* ============================================================
 *  Position loop at ~50 Hz
 *  Updates velSetpoint_ned_ from GPS position error.
 * ============================================================ */
void FlightModeFSM::updatePositionLoop(float dt, const InvEKF& ekf) {
    if (g_state.mode != FlightMode::CAMERA) return;

    RCInputs rc;
    osMutexAcquire(g_state.rcMtx, osWaitForever); rc=g_state.rc;
    osMutexRelease(g_state.rcMtx);
    osMutexAcquire(g_state.gpsMtx, osWaitForever);
    const bool gpsFixValid = g_state.gps.fixValid;
    osMutexRelease(g_state.gpsMtx);
    if (!rc.altHold || !gpsFixValid) {
        positionHoldActive_ = false;
        return;
    }

    osMutexAcquire(g_state.navMtx, osWaitForever);
    const Vector3f pos = ekf.getPosition();
    const Vector3f vel = ekf.getVelocity();
    osMutexRelease(g_state.navMtx);
    if (!positionHoldActive_) {
        posCtrl_.captureSetpoint(pos);
        positionHoldActive_ = true;
    }
    velSetpoint_ned_ = posCtrl_.update(pos, vel, dt);
}

/* ============================================================
 *  Arming
 * ============================================================ */
void FlightModeFSM::tryArm() {
    RCInputs rc;
    osMutexAcquire(g_state.rcMtx, osWaitForever); rc=g_state.rc;
    osMutexRelease(g_state.rcMtx);

    osMutexAcquire(g_state.safetyMtx, osWaitForever);
    bool noFault = !g_state.killMotors &&
                   !(g_state.errorFlags & (FC_ERR_IMU | FC_ERR_TILT_LIMIT |
                                           FC_ERR_MOTOR_LOSS));
    osMutexRelease(g_state.safetyMtx);

    osMutexAcquire(g_state.imuMtx, osWaitForever);
    float gm = g_state.gyro_rps.norm();
    osMutexRelease(g_state.imuMtx);

    if (rc.valid && rc.armSwitch && rc.throttle < 0.05f && noFault && gm < 0.10f) {
        const uint32_t now = osKernelGetTickCount();
        if (!armHoldActive_) {
            armHoldStartTick_ = now;
            armHoldActive_ = true;
        }
        const uint32_t elapsedMs = static_cast<uint32_t>(
            (static_cast<uint64_t>(now - armHoldStartTick_) * 1000u) /
            osKernelGetTickFreq());
        if (elapsedMs >= kArmHoldMs) {
            g_state.armState=ArmState::ARMED; g_state.mode=FlightMode::ARMED;
            mixer_.enableMotors(true);
            rateCtrl_.reset(); attCtrl_.reset();
            velCtrl_.reset();  posCtrl_.reset();
            armHoldActive_=false; modeEntryMs_=osKernelGetTickCount();
        }
    } else { armHoldActive_=false; }
}

void FlightModeFSM::tryDisarm() {
    RCInputs rc;
    osMutexAcquire(g_state.rcMtx, osWaitForever); rc=g_state.rc;
    osMutexRelease(g_state.rcMtx);
    if (!rc.armSwitch || !rc.valid) {
        g_state.mode=FlightMode::DISARMED; g_state.armState=ArmState::DISARMED;
        mixer_.enableMotors(false);
        rateCtrl_.reset(); attCtrl_.reset(); velCtrl_.reset(); posCtrl_.reset();
    }
}

void FlightModeFSM::enterFPV() {
    rateCtrl_.setPreset(RateController::fpvPreset());
    mixer_.setConfig(Mixer::fpvConfig()); mixer_.enableMotors(true);
    /* Reset outer loops — not needed in FPV but clean state for next switch */
    velCtrl_.reset(); attCtrl_.reset();
    g_state.mode=FlightMode::FPV; modeEntryMs_=osKernelGetTickCount();
}

void FlightModeFSM::enterCamera() {
    rateCtrl_.setPreset(RateController::cameraPreset());
    mixer_.setConfig(Mixer::cameraConfig()); mixer_.enableMotors(true);
    /* Capture heading so there is no yaw snap */
    osMutexAcquire(g_state.ahMtx, osWaitForever);
    Math::Vector3f euler=g_state.attitude.toEulerRPY();
    osMutexRelease(g_state.ahMtx);
    attCtrl_.captureYaw(euler.z);
    /* Initialise velocity setpoint from current velocity so there
     * is no jump in the velocity controller's error on mode entry   */
    osMutexAcquire(g_state.navMtx, osWaitForever);
    velSetpoint_ned_=g_state.velocity_ned;
    osMutexRelease(g_state.navMtx);
    velCtrl_.reset(); posCtrl_.reset(); positionHoldActive_=false;
    desiredRoll_rad_=0.f; desiredPitch_rad_=0.f; velThrustDelta_=0.f;
    g_state.mode=FlightMode::CAMERA; modeEntryMs_=osKernelGetTickCount();
}

void FlightModeFSM::enterLanding() {
    landingThrottle_=g_state.thrustOutput;
    g_state.mode=FlightMode::LANDING;
}

void FlightModeFSM::handleFailsafe() {
    rateCtrl_.setPreset(RateController::cameraPreset());
    mixer_.setConfig(Mixer::cameraConfig()); mixer_.enableMotors(true);
    osMutexAcquire(g_state.ahMtx, osWaitForever);
    const float yaw = g_state.attitude.toEulerRPY().z;
    osMutexRelease(g_state.ahMtx);
    attCtrl_.captureYaw(yaw);
    velCtrl_.reset(); posCtrl_.reset(); positionHoldActive_=false;
    desiredRoll_rad_=desiredPitch_rad_=velThrustDelta_=0.0f;
    /* Zero velocity setpoint: hover in place and descend slowly */
    velSetpoint_ned_={0.f,0.f, 0.f};
    g_state.mode=FlightMode::FAILSAFE; fsDescentRate_=0.f;
    osMutexAcquire(g_state.safetyMtx, osWaitForever);
    g_state.errorFlags|=FC_ERR_RC_LOSS;
    osMutexRelease(g_state.safetyMtx);
}

Vector3f FlightModeFSM::rcToRateSetpointFPV(const RCInputs& rc) {
    constexpr float d2r=3.14159265f/180.f;
    return { rc.roll*FPV_MAX_ROLL_RATE*d2r,
             rc.pitch*FPV_MAX_PITCH_RATE*d2r,
             rc.yaw*FPV_MAX_YAW_RATE*d2r };
}

}} // FC::FlightModes

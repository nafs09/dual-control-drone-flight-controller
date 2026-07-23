#pragma once
#include "pid.hpp"
#include "../math/vector3.hpp"
#include <cmath>

namespace FC { namespace Control {

using Math::Vector3f;

/**
 * @brief  Velocity Controller  (second-outermost loop, 100 Hz)
 *
 * POSITION IN THE CASCADE:
 *
 *   [Position Controller 50 Hz]
 *          ↓ vel_setpoint (m/s, NED)
 *   [Velocity Controller 100 Hz]   ← THIS CLASS
 *          ↓ desired_tilt (roll, pitch in body frame, rad)
 *   [Attitude Controller ~3840 Hz]
 *          ↓ rate_setpoint (rad/s)
 *   [Rate Controller ~3840 Hz]
 *          ↓ torque [-1..+1]
 *   [Mixer] → motors
 *
 * BYPASS IN FPV MODE: this controller is completely skipped.
 * RC sticks → rate setpoint feeds directly to the rate controller.
 *
 * WHAT THE VELOCITY CONTROLLER DOES:
 *
 *   Given a desired NED velocity (from position controller or from
 *   RC sticks mapped to velocity) and a measured NED velocity (from
 *   the EKF), compute a desired horizontal tilt in body frame that
 *   will accelerate the drone to match the setpoint.
 *
 *   Physics:  a_horizontal = g * tan(tilt) ≈ g * tilt  (small angles)
 *   So:       tilt_desired = PID_output / g
 *
 * COORDINATE CONVERSION:
 *
 *   The PID outputs a desired acceleration in NED frame.
 *   This is converted to body-frame tilt using current yaw (ψ):
 *
 *     a_fwd   = a_N·cos(ψ) + a_E·sin(ψ)   (forward = body X direction)
 *     a_right = -a_N·sin(ψ) + a_E·cos(ψ)  (right   = body Y direction)
 *
 *     pitch_body = -a_fwd   / g   (negative: forward accel → nose down)
 *     roll_body  =  a_right / g   (positive: rightward accel → lean right)
 *
 *   Verification (drone facing North, ψ=0):
 *     Want a_N > 0 (go North): a_fwd = a_N → pitch = -a_N/g (nose down ✓)
 *     Want a_E > 0 (go East):  a_right = a_E → roll = a_E/g (lean right ✓)
 *
 *   Verification (drone facing East, ψ=π/2):
 *     Want a_N > 0 (go North): a_fwd = 0, a_right = -a_N
 *                               → roll = -a_N/g (lean left = lean North ✓)
 *
 * STICK → VELOCITY SETPOINT MAPPING (camera mode, no GPS hold):
 *
 *   The RC sticks map directly to a desired velocity in the body
 *   forward/right frame, then rotated to NED by yaw:
 *
 *     vel_body_fwd  = -pitch_stick × V_MAX   (push forward → +forward vel)
 *     vel_body_right =  roll_stick  × V_MAX   (push right   → +right vel)
 *     vel_sp_N = vel_body_fwd × cos(ψ) - vel_body_right × sin(ψ)
 *     vel_sp_E = vel_body_fwd × sin(ψ) + vel_body_right × cos(ψ)
 *
 *   This is "velocity mode": centred stick → drone stops and holds velocity.
 *   Far more cinematic than angle mode where the drone slides when you centre.
 *
 * WIND REJECTION:
 *
 *   A steady wind creates a constant velocity error (drone drifts downwind).
 *   The integral term builds up to hold a permanent tilt into the wind,
 *   exactly cancelling the aerodynamic drag. This is the primary mechanism
 *   for robust position hold. See process.md for a full derivation.
 *
 * VERTICAL VELOCITY:
 *
 *   A separate vertical PID maps vertical velocity error → throttle increment.
 *   Output is added to the base throttle in the FSM, not to tilt.
 *
 * TUNING ORDER (see process.md §6):
 *   1. Tune rate PIDs first.
 *   2. Tune attitude P gain.
 *   3. Tune velocity Kp until drone follows velocity setpoints without lag.
 *   4. Add Ki until wind drift is eliminated (watch integrator windup).
 *   5. Leave Kd = 0 unless EKF velocity estimate is very clean.
 */
class VelocityController {
public:
    struct Gains {
        /* Horizontal (N and E symmetric) */
        float kp     = 1.5f;    /**< (m/s²) per (m/s) velocity error          */
        float ki     = 0.3f;    /**< Wind rejection integral                   */
        float kd     = 0.0f;    /**< Usually 0 — EKF velocity is already smooth*/
        float i_max  = 5.0f;    /**< Integrator state clamp [m/s²]            */
        float out_max= 9.81f;   /**< Max desired accel = g → max 45° tilt     */
        float tau_d  = 0.02f;   /**< D-term filter time constant               */

        /* Vertical (throttle increment, not tilt) */
        float kp_v    = 2.5f;
        float ki_v    = 0.5f;
        float kd_v    = 0.5f;
        float i_max_v = 0.3f;
        float out_max_v = 0.4f; /**< Max throttle increment [0..1]            */
    };

    struct Output {
        float roll_rad;      /**< Desired body-frame roll  [rad]               */
        float pitch_rad;     /**< Desired body-frame pitch [rad]               */
        float thrust_delta;  /**< Vertical velocity → throttle increment       */
    };

    static Gains cameraGains() { return Gains{}; }

    explicit VelocityController(const Gains& g = cameraGains())
        : g_(g),
          northPID_({ g.kp, g.ki, g.kd, 0.f, g.tau_d, g.i_max, g.out_max }),
          eastPID_ ({ g.kp, g.ki, g.kd, 0.f, g.tau_d, g.i_max, g.out_max }),
          vertPID_ ({ g.kp_v, g.ki_v, g.kd_v, 0.f, 0.05f, g.i_max_v, g.out_max_v })
    {}

    void setGains(const Gains& g) {
        g_ = g;
        PID::Gains pg = { g.kp, g.ki, g.kd, 0.f, g.tau_d, g.i_max, g.out_max };
        northPID_.setGains(pg); eastPID_.setGains(pg);
        PID::Gains vg = { g.kp_v, g.ki_v, g.kd_v, 0.f, 0.05f, g.i_max_v, g.out_max_v };
        vertPID_.setGains(vg);
    }

    /**
     * @brief  Compute desired tilt from velocity error.
     *
     * @param vel_sp     Desired NED velocity [m/s].  z is positive-down (NED).
     * @param vel_meas   EKF NED velocity [m/s].
     * @param yaw_rad    Current heading, NED convention [rad].
     * @param dt         Time step [s].
     * @return           Desired body-frame tilt and throttle delta.
     */
    Output update(const Vector3f& vel_sp,
                  const Vector3f& vel_meas,
                  float yaw_rad,
                  float dt)
    {
        if (dt <= 0.f || dt > 0.5f) return {0.f, 0.f, 0.f};

        static constexpr float kG = 9.80665f;

        /* --- Horizontal PIDs in NED frame --- */
        float a_N = northPID_.update(vel_sp.x, vel_meas.x, dt);   /* desired North accel */
        float a_E = eastPID_ .update(vel_sp.y, vel_meas.y, dt);   /* desired East  accel */

        /* --- Convert NED acceleration demand to body-frame tilt ---
         * Forward component of (a_N, a_E) in body frame:
         *   a_fwd   =  a_N·cos(ψ) + a_E·sin(ψ)
         * Right component:
         *   a_right = -a_N·sin(ψ) + a_E·cos(ψ)
         *
         * Tilt angles (small-angle approximation a = g·θ):
         *   pitch_body = -a_fwd   / g  (negative: forward accel needs nose-down)
         *   roll_body  =  a_right / g                                          */
        float cy = cosf(yaw_rad);
        float sy = sinf(yaw_rad);

        float a_fwd   =  a_N * cy + a_E * sy;
        float a_right = -a_N * sy + a_E * cy;

        float pitch_rad = -a_fwd   / kG;
        float roll_rad  =  a_right / kG;

        /* Clamp tilt to safe limits (30° = 0.524 rad) */
        auto clamp = [](float v, float l){ return v>l?l:(v<-l?-l:v); };
        static constexpr float kMaxTilt = 0.524f;   /* 30 degrees */
        pitch_rad = clamp(pitch_rad, kMaxTilt);
        roll_rad  = clamp(roll_rad,  kMaxTilt);

        /* --- Vertical PID: vel_sp.z (NED down) → throttle delta ---
         * NED down is positive, so positive z error = sinking → need more thrust.
         * The vertical PID output sign: positive output = more thrust.          */
        float thrust_delta = vertPID_.update(vel_sp.z, vel_meas.z, dt);

        return { roll_rad, pitch_rad, thrust_delta };
    }

    /**
     * @brief  Build NED velocity setpoint from RC sticks (camera mode, no GPS hold).
     *
     * Converts stick deflection to velocity setpoint in NED frame using
     * the heading-relative mapping (push forward → go in direction drone faces).
     *
     * @param pitch_stick   Normalised pitch stick  [-1..+1]
     * @param roll_stick    Normalised roll stick   [-1..+1]
     * @param yaw_rad       Current heading [rad]
     * @param max_vel       Maximum horizontal speed [m/s]
     * @return              NED velocity setpoint (z = 0, use throttle for altitude)
     */
    static Vector3f sticksToVelSetpoint(float pitch_stick, float roll_stick,
                                         float yaw_rad, float max_vel)
    {
        /* Body-frame velocity intent (FRD):
         * Positive pitch_stick = push forward on TX → drone should go FORWARD
         * In our convention pitch is nose-up (+), so forward = -pitch_stick   */
        float v_fwd   = -pitch_stick * max_vel;
        float v_right =  roll_stick  * max_vel;

        /* Rotate from body to NED using current yaw */
        float cy = cosf(yaw_rad);
        float sy = sinf(yaw_rad);
        float vN = v_fwd * cy - v_right * sy;
        float vE = v_fwd * sy + v_right * cy;

        return { vN, vE, 0.f };
    }

    /** Reset all integrators. Call on mode transition or arm. */
    void reset() {
        northPID_.reset();
        eastPID_.reset();
        vertPID_.reset();
    }

    /* Access internal PIDs for logging/telemetry */
    PID& northPID() { return northPID_; }
    PID& eastPID()  { return eastPID_;  }
    PID& vertPID()  { return vertPID_;  }

private:
    Gains g_;
    PID   northPID_, eastPID_, vertPID_;
};

}} // FC::Control

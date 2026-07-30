
#pragma once
#include "../math/vector3.hpp"
#include "../core/shared_state.hpp"
#include <stdint.h>
#include <cmath>

namespace FC { namespace Actuation {

using Math::Vector3f;

/**
 * @brief  X-frame mixer with optional thrust-linearisation from eRPM.
 *
 * MOTOR LAYOUT (X-frame, top view):
 *   M1 (CW)  M2 (CCW)
 *      \     /
 *       \   /
 *        X-X
 *       /   \
 *      /     \
 *   M4 (CCW) M3 (CW)
 *
 * MIXING MATRIX:
 *   Motor:   M1    M2    M3    M4
 *   Thrust: +1    +1    +1    +1
 *   Roll:   -1    +1    +1    -1
 *   Pitch:  -1    -1    +1    +1
 *   Yaw:    +1    -1    +1    -1
 *
 * THRUST LINEARISATION (bidirectional DSHOT):
 * ────────────────────────────────────────────
 * DSHOT throttle is proportional to motor PWM duty, not to thrust.
 * Thrust ∝ RPM², so the relationship between throttle command and
 * actual force is quadratic and ESC-dependent.
 *
 * With eRPM feedback from bidirectional DSHOT we can instead command
 * the mixer in NEWTONS (or normalised force units) and solve for the
 * DSHOT value that achieves that force.
 *
 * Thrust model:  F_i = kF × mechRPM_i²
 * Inverse:       RPM_desired = sqrt(F_desired / kF)
 * DSHOT value:   cmd = RPM_to_DSHOT(RPM_desired)
 *                    = (RPM_desired / RPM_max) × 2047
 *
 * This requires a one-time test-bench calibration to find kF and RPM_max.
 * Until that is done, the mixer falls back to linear throttle mapping.
 *
 * DESATURATION:
 * ─────────────
 * 1. Compute raw motor values from thrust + torques.
 * 2. If any motor would exceed [idle, max], scale the torque vector
 *    down uniformly (preserving attitude control direction) until all
 *    motors fit.  Thrust priority: reduce torques before thrust.
 * 3. Hard-clamp remaining outliers.
 */
class Mixer {
public:
    struct Config {
        uint16_t dshotIdle  = 150u;
        uint16_t dshotMin   = 48u;
        uint16_t dshotMax   = 2047u;
        float    torqueScale= 0.25f;   /**< Maps torque[-1,1] → DSHOT increment  */
        bool     motorsEnabled = false;

        /* Thrust linearisation parameters */
        bool  thrustLinearise = false; /**< Enable only after test-bench cal      */
        float mechRPM_max     = 36000.f; /**< No-load RPM at DSHOT 2047 (measure) */
        /* kF is in DShot::kF_ (shared with telemetry decoder) */
    };

    static Config fpvConfig() {
        Config c; c.dshotIdle=150; c.torqueScale=0.28f; return c;
    }
    static Config cameraConfig() {
        Config c; c.dshotIdle=80; c.torqueScale=0.22f; return c;
    }

    explicit Mixer(const Config& cfg = fpvConfig()) : cfg_(cfg) {}

    void setConfig(const Config& c) { cfg_ = c; }
    void enableMotors(bool en)      { cfg_.motorsEnabled = en; }

    /**
     * @brief  Mix collective thrust and torques into motor DSHOT commands.
     *
     * @param thrust   Collective [0..1].
     * @param torque   Roll/pitch/yaw torque demands [-1..+1] per axis.
     * @param cmds     Output: 4 DSHOT values.
     */
    void mix(float thrust, const Vector3f& torque, uint16_t cmds[4]) const {
        if (!cfg_.motorsEnabled) {
            for (int i=0;i<4;i++) cmds[i]=0; return;
        }

        float base;
        if (cfg_.thrustLinearise) {
            /* Thrust linearisation: thrust [0..1] → desired force →
             * desired RPM → DSHOT value.
             *
             * F_desired = thrust × F_hover × 4  (total vehicle thrust)
             * For mixing we use normalised force per motor:
             *   RPM_des = sqrt(thrust) × RPM_max
             *   (because thrust ∝ RPM², sqrt linearises to RPM space)  */
            float rpm_des = sqrtf(fmaxf(thrust, 0.f)) * cfg_.mechRPM_max;
            base = (rpm_des / cfg_.mechRPM_max)
                   * static_cast<float>(cfg_.dshotMax - cfg_.dshotIdle)
                   + cfg_.dshotIdle;
        } else {
            /* Linear mapping: throttle [0..1] → DSHOT range */
            base = cfg_.dshotIdle
                 + thrust * static_cast<float>(cfg_.dshotMax - cfg_.dshotIdle);
        }

        float ts = cfg_.torqueScale * static_cast<float>(cfg_.dshotMax - cfg_.dshotIdle);

        /* X-frame mixing */
        float m[4];
        m[0] = base + (-torque.x - torque.y + torque.z) * ts;
        m[1] = base + ( torque.x - torque.y - torque.z) * ts;
        m[2] = base + ( torque.x + torque.y + torque.z) * ts;
        m[3] = base + (-torque.x + torque.y - torque.z) * ts;

        /* Desaturation: find how much we exceed the limits */
        float maxOver = 0.f, maxUnder = 0.f;
        for (int i=0;i<4;i++) {
            if (m[i] > cfg_.dshotMax)  maxOver  = fmaxf(maxOver,  m[i] - cfg_.dshotMax);
            if (m[i] < cfg_.dshotIdle) maxUnder = fmaxf(maxUnder, cfg_.dshotIdle - m[i]);
        }
        if (maxOver  > 0.f) for (int i=0;i<4;i++) m[i] -= maxOver;
        if (maxUnder > 0.f) for (int i=0;i<4;i++) m[i] += maxUnder;

        /* Hard clamp and convert */
        for (int i=0;i<4;i++) {
            float v = m[i];
            if (v > cfg_.dshotMax)  v = cfg_.dshotMax;
            if (v < cfg_.dshotMin)  v = cfg_.dshotMin;
            cmds[i] = static_cast<uint16_t>(v);
        }
    }

    void kill(uint16_t cmds[4]) const { for(int i=0;i<4;i++) cmds[i]=0; }
    void idle(uint16_t cmds[4]) const { for(int i=0;i<4;i++) cmds[i]=cfg_.dshotIdle; }

private:
    Config cfg_;
};

}} // FC::Actuation

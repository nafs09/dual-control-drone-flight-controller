
#pragma once
#include "../math/quaternion.hpp"
#include "../math/vector3.hpp"

namespace FC { namespace Estimation {

using Math::Vector3f;
using Math::Quaternionf;

/**
 * @brief  Mahony Complementary AHRS  —  Revised
 *
 * Improvements applied vs previous version:
 *
 *  #5  Complete attitude error dynamics
 *      Previously: δθ̇ = −δb_g  (bias coupling only)
 *      Now:        δθ̇ = −[ω]× δθ − δb_g − n_g
 *      The angular-velocity skew term −[ω]× δθ is added to the PI
 *      feedback path, coupling the current body rate into the
 *      observer.  This matters during rapid rotations where the
 *      attitude error is large and spinning quickly.
 *
 *  #8  Exponential-map quaternion integration
 *      Previously: first-order Euler  q += q_dot * dt
 *      Now:        q_{k+1} = q_k ⊗ exp(ω * dt / 2)
 *      Exact on the rotation manifold; no norm drift accumulation.
 *      Implemented inside Quaternionf::integrateBodyRate().
 *
 *  #9  Vector-based magnetometer update
 *      Previously: scalar yaw error only, discarding inclination
 *      Now:        r = m_meas × R^T * m_NED
 *      Uses the full 3D cross-product innovation, which:
 *        - Avoids atan2 / Euler singularities
 *        - Preserves roll/pitch information in the mag measurement
 *        - Handles high-inclination fields (e.g. northern latitudes)
 *      m_NED is set once from the first stationary mag reading and
 *      can be updated by the EKF at any time.
 *
 *  #10 Continuous accelerometer confidence weighting
 *      Previously: hard gate at [0.5g, 1.5g]  (discontinuous)
 *      Now:        w = exp(−(‖a‖ − g)² / σ²)
 *      The PI gains are scaled by w, giving a smooth transition
 *      from full trust at 1g to near-zero trust under high-g
 *      manoeuvres.  σ_a = 2.5 m/s² gives ~50% trust at 1.25g.
 *
 * Reference:
 *   Mahony et al., IEEE TAC 2008.
 *   Madgwick, "An efficient orientation filter", 2010.
 */
class MahonyAHRS {
public:
    struct Gains {
        float kp_acc  = 2.0f;    /**< Proportional gain – accel tilt        */
        float ki_acc  = 0.005f;  /**< Integral gain – accel bias path        */
        float kp_mag  = 1.0f;    /**< Proportional gain – mag yaw            */
        float ki_mag  = 0.002f;  /**< Integral gain – mag bias path          */
        float sigma_a = 2.5f;    /**< Accel confidence width [m/s²] (#10)    */
    };

    explicit MahonyAHRS(const Gains& g = Gains());

    /**
     * @brief  Update with one IMU sample.
     *
     * Implements improvements #5, #8, #10.
     *
     * @param gyro_rps    Gyro, body-frame FRD, rad/s.
     * @param accel_mps2  Accel, body-frame FRD, m/s².
     * @param dt          Sample interval (seconds).
     */
    void updateIMU(const Vector3f& gyro_rps,
                   const Vector3f& accel_mps2,
                   float dt);

    /**
     * @brief  Update with one magnetometer sample.
     *
     * Implements improvement #9 (full 3D vector innovation).
     *
     * @param mag_body  Magnetometer reading, body frame, any units.
     *                  Will be normalised internally.
     */
    void updateMag(const Vector3f& mag_body);

    /**
     * @brief  Set the reference Earth magnetic field vector (NED frame).
     *
     * Call once after the first stationary mag calibration, or whenever
     * the EKF produces an updated declination/inclination estimate.
     * The vector should be in the same units as mag_body but its
     * magnitude is normalised internally.
     *
     * If not set, the filter will set it automatically from the first
     * valid updateMag() call when the drone is stationary.
     *
     * @param mag_NED  Known Earth field in NED, arbitrary units.
     */
    void setMagReference(const Vector3f& mag_NED);

    Quaternionf getAttitude()  const { return q_; }
    Vector3f    getGyroBias()  const { return gyroBias_; }

    /** Override gyro bias from EKF state. */
    void setGyroBias(const Vector3f& b) { gyroBias_ = b; }

    void setGains(const Gains& g) { g_ = g; }

    void reset(const Quaternionf& q = Quaternionf::identity());

private:
    Quaternionf q_;
    Vector3f    gyroBias_;     /**< Integrated PI bias estimate, rad/s       */
    Vector3f    magRef_NED_;   /**< Earth mag vector in NED (unit)           */
    bool        magRefSet_;

    Gains       g_;

    /* Pending mag correction (accumulated between mag and IMU calls) */
    Vector3f    pendingMagErr_;
    bool        hasMagUpdate_;

    static constexpr float kGravity = 9.80665f;
};

}} // FC::Estimation

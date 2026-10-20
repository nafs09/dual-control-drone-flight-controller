
#include "mahony_ahrs.hpp"
#include <cmath>

namespace FC { namespace Estimation {

MahonyAHRS::MahonyAHRS(const Gains& g)
    : q_(Quaternionf::identity()),
      gyroBias_(Vector3f::zero()),
      magRefSet_(false),
      g_(g),
      pendingMagErr_(Vector3f::zero()),
      hasMagUpdate_(false)
{}

void MahonyAHRS::reset(const Quaternionf& q) {
    q_            = q;
    if (!std::isfinite(q_.normSq()) || q_.normSq() < 1.0e-8f) q_ = Quaternionf::identity();
    q_.normalise();
    gyroBias_     = Vector3f::zero();
    magRefSet_    = false;
    hasMagUpdate_ = false;
    pendingMagErr_= Vector3f::zero();
}

void MahonyAHRS::setMagReference(const Vector3f& mag_NED) {
    Vector3f reference = mag_NED;
    if (!std::isfinite(reference.x) || !std::isfinite(reference.y) ||
        !std::isfinite(reference.z) || !reference.normalise()) return;
    magRef_NED_ = reference;
    magRefSet_ = true;
}

/* ================================================================
 *  updateMag — full 3D vector magnetometer update
 *
 *  Instead of extracting a scalar yaw error, we:
 *    1. Rotate the known NED reference field into the estimated
 *       body frame:  m_pred = R^T * m_NED  =  q* ⊗ m_NED ⊗ q
 *    2. Form the innovation as the cross product:
 *         r = m_meas × m_pred
 *       This vector points along the rotation axis needed to bring
 *       m_pred onto m_meas, in the body frame.
 *    3. The cross product naturally ignores the field magnitude
 *       (no spurious corrections when ‖m‖ varies) and handles any
 *       inclination angle without Euler singularities.
 *
 *  Auto-calibration: if no reference has been set, the first valid
 *  reading while the AHRS is converged (low gyro mag) is used to
 *  initialise the NED reference from the current estimate.
 * ================================================================ */
void MahonyAHRS::updateMag(const Vector3f& mag_body) {
    Vector3f m = mag_body;
    if (!std::isfinite(m.x) || !std::isfinite(m.y) || !std::isfinite(m.z)) return;
    if (!m.normalise()) return;   /* Degenerate reading */

    if (!magRefSet_) {
        /* Bootstrap: rotate the current measurement into NED using the
         * present attitude estimate and store it as the reference.     */
        magRef_NED_ = q_.rotate(m);
        magRefSet_  = true;
        return;
    }

    /* Rotate NED reference into body frame using current attitude */
    Vector3f m_pred = q_.rotateInverse(magRef_NED_);
    m_pred.normalise();

    /* Innovation: cross product  r = m_meas × m_pred  (body frame)
     * For small errors:  |r| ≈ sin(ψ_error),  direction = rotation axis  */
    Vector3f err = m.cross(m_pred);

    /* Store to be applied in next updateIMU call */
    pendingMagErr_ = err;
    hasMagUpdate_  = true;
}

/* ================================================================
 *  updateIMU — high-rate PI complementary observer
 * ================================================================ */
void MahonyAHRS::updateIMU(const Vector3f& gyro_rps,
                           const Vector3f& accel_mps2,
                           float           dt)
{
    if (!std::isfinite(dt) || dt <= 0.0f || dt > 0.02f ||
        !std::isfinite(gyro_rps.x) || !std::isfinite(gyro_rps.y) ||
        !std::isfinite(gyro_rps.z) || !std::isfinite(accel_mps2.x) ||
        !std::isfinite(accel_mps2.y) || !std::isfinite(accel_mps2.z)) return;
    /* ----------------------------------------------------------
     * Continuous accelerometer confidence weight
     *
     *   w = exp( −(‖a‖ − g)² / σ_a² )
     *
     * Under 1g (hover / level): w ≈ 1.0   → full correction
     * Under 1.25g (hard pull):  w ≈ 0.51  → half trust
     * Under 2g   (snap roll):   w ≈ 0.014 → nearly ignored
     *
     * This is smooth and continuous — no hard gate hysteresis.
     * ---------------------------------------------------------- */
    float aMag  = accel_mps2.norm();
    float delta = aMag - kGravity;
    const float sigma_a = std::fmax(g_.sigma_a, 1.0e-3f);
    float w_acc = expf(-(delta * delta) / (sigma_a * sigma_a));

    /* Normalised accelerometer measurement */
    Vector3f a_norm = (aMag > 1e-3f) ? (accel_mps2 / aMag) : Vector3f::zero();

    /* Predicted specific-force direction at rest is world-up in NED. The
     * measured accelerometer specific force points opposite gravity. */
    const Vector3f force_pred_body = q_.rotateInverse({0.f, 0.f, -1.f});

    /* Body-frame cross-product innovation; its sign drives a positive tilt
     * estimate back toward the measured specific-force direction. */
    Vector3f err_acc = a_norm.cross(force_pred_body) * w_acc;

    /* ----------------------------------------------------------
     * Magnetometer correction (set by updateMag)
     * ---------------------------------------------------------- */
    Vector3f err_mag = Vector3f::zero();
    if (hasMagUpdate_) {
        err_mag       = pendingMagErr_;
        hasMagUpdate_ = false;
    }

    /* Standard Mahony integral feedback estimates gyro bias. */
    gyroBias_ -= (err_acc * g_.ki_acc + err_mag * g_.ki_mag) * dt;

    /* Hard clamp on bias estimate to prevent divergence */
    gyroBias_ = gyroBias_.clamped(0.5f);   /* 0.5 rad/s ≈ 28 deg/s max bias */

    /* ----------------------------------------------------------
     * Corrected angular rate = measured − bias + Kp * error
     * ---------------------------------------------------------- */
    Vector3f omega = gyro_rps - gyroBias_
                   + err_acc * g_.kp_acc
                   + err_mag * g_.kp_mag;

    /* ----------------------------------------------------------
     * Exact exponential-map quaternion integration
     *
     *   q_{k+1} = q_k ⊗ exp(ω * dt / 2)
     *
     * See Quaternionf::integrateBodyRate().
     * ---------------------------------------------------------- */
    q_.integrateBodyRate(omega, dt);
    /* integrateBodyRate normalises internally */
}

}} // FC::Estimation

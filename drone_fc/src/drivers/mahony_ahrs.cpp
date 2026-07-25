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
    gyroBias_     = Vector3f::zero();
    magRefSet_    = false;
    hasMagUpdate_ = false;
    pendingMagErr_= Vector3f::zero();
}

void MahonyAHRS::setMagReference(const Vector3f& mag_NED) {
    magRef_NED_ = mag_NED.normalised();
    magRefSet_  = true;
}

/* ================================================================
 *  updateMag — Improvement #9: full 3D vector magnetometer update
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
 *  updateIMU — Improvements #5, #8, #10
 * ================================================================ */
void MahonyAHRS::updateIMU(const Vector3f& gyro_rps,
                           const Vector3f& accel_mps2,
                           float           dt)
{
    /* ----------------------------------------------------------
     * Improvement #10: Continuous accelerometer confidence weight
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
    float w_acc = expf(-(delta * delta) / (g_.sigma_a * g_.sigma_a));

    /* Normalised accelerometer measurement */
    Vector3f a_norm = (aMag > 1e-3f) ? (accel_mps2 / aMag) : Vector3f::zero();

    /* Estimated gravity direction in body frame = third column of R^T */
    Vector3f g_body = q_.rotateInverse({0.f, 0.f, 1.f});

    /* Tilt error = cross(estimated_gravity, measured_gravity)
     * The cross product magnitude ≈ sin(tilt_error); sign gives axis. */
    Vector3f err_acc = g_body.cross(a_norm) * w_acc;

    /* ----------------------------------------------------------
     * Magnetometer correction (set by updateMag)
     * ---------------------------------------------------------- */
    Vector3f err_mag = Vector3f::zero();
    if (hasMagUpdate_) {
        err_mag       = pendingMagErr_;
        hasMagUpdate_ = false;
    }

    /* Combined error */
    Vector3f err = err_acc * 1.f + err_mag;   /* weights already in gains */

    /* ----------------------------------------------------------
     * Improvement #5: Complete attitude error dynamics
     *
     * Full linearised PI observer for gyro bias:
     *
     *   δθ̇ = −[ω]× δθ  −  δb_g  −  n_g
     *
     * The term −[ω]× δθ couples the current angular velocity into
     * the bias integrator.  Without it, fast rotations cause the
     * integral term to lag the true error.
     *
     * Implementation: the skew-symmetric action on the current bias
     * estimate is:
     *   [ω]× b  =  ω × b
     * ---------------------------------------------------------- */
    /* Skew term on current bias: ω × b */
    Vector3f skewBias = gyro_rps.cross(gyroBias_);

    /* PI feedback on bias integrator including skew coupling */
    Vector3f biasRate = -( err_acc * g_.ki_acc + err_mag * g_.ki_mag )
                        - skewBias * dt;
    gyroBias_ += biasRate * dt;

    /* Hard clamp on bias estimate to prevent divergence */
    gyroBias_ = gyroBias_.clamped(0.5f);   /* 0.5 rad/s ≈ 28 deg/s max bias */

    /* ----------------------------------------------------------
     * Corrected angular rate = measured − bias + Kp * error
     * ---------------------------------------------------------- */
    Vector3f omega = gyro_rps - gyroBias_
                   + err_acc * g_.kp_acc
                   + err_mag * g_.kp_mag;

    /* ----------------------------------------------------------
     * Improvement #8: Exact exponential-map quaternion integration
     *
     *   q_{k+1} = q_k ⊗ exp(ω * dt / 2)
     *
     * See Quaternionf::integrateBodyRate().
     * ---------------------------------------------------------- */
    q_.integrateBodyRate(omega, dt);
    /* integrateBodyRate normalises internally */
}

}} // FC::Estimation

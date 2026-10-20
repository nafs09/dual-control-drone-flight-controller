
#pragma once
#include "../math/quaternion.hpp"
#include "../math/vector3.hpp"

namespace FC { namespace Estimation {

using Math::Vector3f;
using Math::Quaternionf;

/**
 * @brief High-rate Mahony complementary attitude observer.
 *
 * The observer combines gyro integration with a confidence-weighted
 * accelerometer tilt innovation and an optional full-vector magnetic
 * innovation. It runs at the IMU rate; InvEKF consumes its quaternion
 * increments for navigation propagation.
 */
class MahonyAHRS {
public:
    struct Gains {
        float kp_acc  = 2.0f;    /**< Accelerometer tilt proportional gain */
        float ki_acc  = 0.005f;  /**< Accelerometer gyro-bias integral gain */
        float kp_mag  = 1.0f;    /**< Magnetometer proportional gain */
        float ki_mag  = 0.002f;  /**< Magnetometer gyro-bias integral gain */
        float sigma_a = 2.5f;    /**< Accel confidence width [m/s²] */
    };

    explicit MahonyAHRS(const Gains& g = Gains());

    /**
     * @brief  Update with one IMU sample.
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
     * @param mag_body  Magnetometer reading, body frame, any units.
     *                  Will be normalised internally.
     */
    void updateMag(const Vector3f& mag_body);

    /**
     * @brief  Set the reference Earth magnetic field vector (NED frame).
     *
     * Call after magnetic calibration using the local NED field reference,
     * and again if that reference changes.
     * The vector should be in the same units as mag_body but its
     * magnitude is normalised internally.
     *
     * If not set, the filter will set it automatically from the first
     * valid updateMag() call when the drone is stationary.
     *
     * @param mag_NED  Known Earth field in NED, arbitrary units.
     */
    void setMagReference(const Vector3f& mag_NED);

    Quaternionf getAttitude() const { return q_; }
    Vector3f getGyroBias() const { return gyroBias_; }

    /** Override gyro bias from the InvEKF state. */
    void setGyroBias(const Vector3f& b) { gyroBias_ = b.clamped(0.5f); }

    void setGains(const Gains& g) { g_ = g; }

    void reset(const Quaternionf& q = Quaternionf::identity());

private:
    Quaternionf q_;
    Vector3f    gyroBias_;     /**< Integrated PI bias estimate, rad/s */
    Vector3f    magRef_NED_;   /**< Earth mag vector in NED (unit)           */
    bool        magRefSet_;

    Gains       g_;

    /* Pending mag correction (accumulated between mag and IMU calls) */
    Vector3f    pendingMagErr_;
    bool        hasMagUpdate_;

    static constexpr float kGravity = 9.80665f;
};

}} // FC::Estimation

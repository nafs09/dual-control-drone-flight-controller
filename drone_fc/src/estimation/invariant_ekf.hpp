#pragma once

#include "../math/quaternion.hpp"
#include "../math/vector3.hpp"

namespace FC { namespace Estimation {

using Math::Quaternionf;
using Math::Vector3f;

/* Right-invariant navigation EKF on SE_2(3), with additive IMU biases.
 * The invariant error is expressed in the estimated body frame; nominal
 * position and velocity are NED. The class uses fixed storage only. */
class InvEKF {
public:
    static constexpr int kStateSize = 15;
    enum Index { ATT = 0, VEL = 3, POS = 6, BG = 9, BA = 12 };

    struct Config {
        float init_position_var = 25.0f;
        float init_velocity_var = 4.0f;
        float init_attitude_var = 0.12f * 0.12f;
        float init_gyro_bias_var = 0.02f * 0.02f;
        float init_accel_bias_var = 0.30f * 0.30f;
        float gyro_noise = 0.02f;
        float accel_noise = 0.25f;
        float gyro_bias_rw = 0.0005f;
        float accel_bias_rw = 0.005f;
        float gyro_bias_tau_s = 3600.0f;
        float accel_bias_tau_s = 3600.0f;
        float gps_position_floor_m2 = 0.25f;
        float gps_velocity_floor_m2ps2 = 0.04f;
        float baro_sigma_m = 0.5f;
        float gravity_sigma = 0.05f;
        float mag_sigma = 0.05f;
        float nis_1dof = 6.635f;
        float nis_3dof = 11.345f;
    };

    InvEKF();
    explicit InvEKF(const Config& config);

    void init(const Vector3f& position_ned = Vector3f::zero(),
              const Vector3f& velocity_ned = Vector3f::zero(),
              const Quaternionf& body_to_ned = Quaternionf::identity());
    bool isInitialised() const { return initialised_; }

    /* Called for every IMU sample (nominal state propagation). predict() is
     * called at the slower navigation rate to propagate covariance. */
    void accumulateIMU(const Vector3f& accel_mps2, const Vector3f& gyro_rps,
                       float dt);
    void predict(float dt_hint);

    void updateGPS(const Vector3f& position_ned, const Vector3f& velocity_ned,
                   float hAcc_m, float vAcc_m, float sAcc_mps, float hDOP,
                   int numSats);
    void updateBaro(float altitude_relative_m, float sigma_m);
    void updateMag(const Vector3f& mag_body, const Vector3f& mag_ned,
                   float sigma = -1.0f);
    void updateGravity(const Vector3f& accel_mps2);

    Vector3f getPosition() const { return position_ned_; }
    Vector3f getVelocity() const { return velocity_ned_; }
    Vector3f getGyroBias() const { return gyro_bias_; }
    Vector3f getAccelBias() const { return accel_bias_; }
    Quaternionf getAttitude() const { return body_to_ned_; }
    void getCovDiag(float diag[kStateSize]) const;

private:
    Config cfg_;
    bool initialised_ = false;
    Vector3f position_ned_, velocity_ned_, gyro_bias_, accel_bias_;
    Quaternionf body_to_ned_ = Quaternionf::identity();
    Vector3f force_body_sum_, omega_body_sum_;
    float covariance_dt_ = 0.0f;
    float P_[kStateSize][kStateSize] = {};
    float F_[kStateSize][kStateSize] = {};
    float Phi_[kStateSize][kStateSize] = {};
    float Qd_[kStateSize][kStateSize] = {};
    float scratch_a_[kStateSize][kStateSize] = {};
    float scratch_b_[kStateSize][kStateSize] = {};
    float scratch_c_[kStateSize][kStateSize] = {};
    unsigned gps_rejects_ = 0, baro_rejects_ = 0, mag_rejects_ = 0;

    static void zero(float* m, int rows, int cols);
    static void identity(float m[kStateSize][kStateSize]);
    static void multiply(const float a[kStateSize][kStateSize],
                         const float b[kStateSize][kStateSize],
                         float out[kStateSize][kStateSize]);
    static void skew(const Vector3f& v, float out[3][3]);
    static bool finite(float value);

    void buildF(float F[kStateSize][kStateSize], const Vector3f& force_body,
                const Vector3f& omega_body) const;
    void discretise(float dt);
    void inject(const float dx[kStateSize]);
    void resetCovariance(const float dx[kStateSize]);
    void stabiliseCovariance();

    template<int M>
    bool update(const float H[M][kStateSize], const float R[M],
                const float innovation[M], float nis_limit);
};

}} // namespace FC::Estimation

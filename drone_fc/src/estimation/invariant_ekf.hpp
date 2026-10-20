#pragma once

#include "../math/quaternion.hpp"
#include "../math/vector3.hpp"

namespace FC { namespace Estimation {

using Math::Quaternionf;
using Math::Vector3f;

/*
 * 24-state left-invariant navigation EKF on SE_2(3), augmented with IMU
 * biases, motor lag, magnetometer bias, barometer calibration, and wind.
 * The first nine error coordinates are left-invariant body-frame attitude,
 * velocity, and position errors. Gyro, accel, and mag biases are body-frame;
 * wind is NED; the remaining states are scalar physical parameters.
 * All storage is fixed-size for deterministic embedded use.
 */
class InvEKF {
public:
    static constexpr int kStateSize = 24;
    enum Index {
        ATT = 0, VEL = 3, POS = 6, BG = 9, BA = 12,
        TAU_M = 15, BMAG = 16, BBARO = 19, SBARO = 20, WIND = 21
    };

    struct Config {
        float init_position_var = 25.0f;
        float init_velocity_var = 4.0f;
        float init_attitude_var = 0.12f * 0.12f;
        float init_gyro_bias_var = 0.02f * 0.02f;
        float init_accel_bias_var = 0.30f * 0.30f;
        float init_motor_tau_var = 0.10f * 0.10f;
        float init_mag_bias_var = 4.0f;
        float init_baro_bias_var = 4.0f;
        float init_baro_scale_var = 0.01f * 0.01f;
        float init_wind_var = 9.0f;

        float motor_tau_nominal_s = 0.08f;
        float gyro_bias_tau_s = 3600.0f;
        float accel_bias_tau_s = 3600.0f;
        float motor_tau_tau_s = 1.0f;
        float mag_bias_tau_s = 3600.0f;
        float baro_bias_tau_s = 300.0f;
        float baro_scale_tau_s = 3600.0f;
        float wind_tau_s = 60.0f;
        float drag_per_s = 0.04f;

        float gyro_noise = 0.02f;
        float accel_noise = 0.25f;
        float gyro_bias_rw = 0.0005f;
        float accel_bias_rw = 0.005f;
        float motor_tau_rw = 0.01f;
        float mag_bias_rw = 0.02f;
        float baro_bias_rw = 0.02f;
        float baro_scale_rw = 0.0002f;
        float wind_rw_horizontal = 0.20f;
        float wind_rw_vertical = 0.50f;

        /* Floors are variances, not standard deviations. */
        float gps_position_floor_m2 = 0.25f;
        float gps_velocity_floor_m2ps2 = 0.04f;
        int gps_min_satellites = 6;
        float gps_max_position_dop = 5.0f;
        float baro_sigma_m = 0.5f;
        float mag_sigma = 1.0f;
        float mag_reference_strength_uT = 50.0f;
        float mag_variance_floor_uT2 = 1.0f;
        float nis_1dof = 6.635f;
        float nis_3dof = 11.345f;
        float motor_accel_gate_rad_s2 = 50.0f;
    };

    InvEKF();
    explicit InvEKF(const Config& config);

    void init(const Vector3f& position_ned = Vector3f::zero(),
              const Vector3f& velocity_ned = Vector3f::zero(),
              const Quaternionf& body_to_ned = Quaternionf::identity());
    bool isInitialised() const { return initialised_; }

    /* Mahony supplies its high-rate body-to-NED solution for every IMU
     * sample. InvEKF follows those attitude increments while propagating
     * navigation nominal states and the slower covariance. */
    void accumulateIMU(const Vector3f& accel_mps2, const Vector3f& gyro_rps,
                       const Quaternionf& mahony_body_to_ned, float dt);
    void predict(float dt_hint);

    void updateGPS(const Vector3f& position_ned, const Vector3f& velocity_ned,
                   float hAcc_m, float vAcc_m, float sAcc_mps, float positionDOP,
                   int numSats);
    bool gpsQualityAcceptable(float positionDOP, int numSats) const;
    void updateBaro(float altitude_relative_m, float sigma_m,
                    float vibration_rms = 0.0f);
    void updateGPSAltitude(float altitude_relative_m, float sigma_m);
    void updateMag(const Vector3f& mag_body, const Vector3f& mag_ned,
                   float sigma = -1.0f);
    void updateMotor(float omega_cmd_rad_s, float omega_meas_rad_s,
                     float omega_prev_rad_s, float dt);
    void setWindPrior(const Vector3f& wind_ned_mps, float sigma_mps);

    Vector3f getPosition() const { return position_ned_; }
    Vector3f getVelocity() const { return velocity_ned_; }
    Vector3f getGyroBias() const { return gyro_bias_; }
    Vector3f getAccelBias() const { return accel_bias_; }
    Vector3f getMagBias() const { return mag_bias_; }
    Vector3f getWind() const { return wind_ned_; }
    Quaternionf getAttitude() const { return body_to_ned_; }
    float getMotorTimeConstant() const { return motor_tau_s_; }
    float getBaroBias() const { return baro_bias_m_; }
    float getBaroScale() const { return baro_scale_; }
    void getCovDiag(float diag[kStateSize]) const;

private:
    Config cfg_;
    bool initialised_ = false;
    Vector3f position_ned_, velocity_ned_;
    Vector3f gyro_bias_, accel_bias_, mag_bias_, wind_ned_;
    Quaternionf body_to_ned_ = Quaternionf::identity();
    Quaternionf attitude_reference_ = Quaternionf::identity();
    float motor_tau_s_ = 0.08f;
    float baro_bias_m_ = 0.0f;
    float baro_scale_ = 0.0f;
    Vector3f force_body_sum_, omega_body_sum_;
    float covariance_dt_ = 0.0f;
    float P_[kStateSize][kStateSize] = {};
    float F_[kStateSize][kStateSize] = {};
    float Phi_[kStateSize][kStateSize] = {};
    float Qd_[kStateSize][kStateSize] = {};
    float scratch_a_[kStateSize][kStateSize] = {};
    float scratch_b_[kStateSize][kStateSize] = {};
    float scratch_c_[kStateSize][kStateSize] = {};
    unsigned gps_rejects_ = 0, baro_rejects_ = 0;
    unsigned mag_rejects_ = 0, motor_rejects_ = 0;

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

#pragma once

#include "../math/quaternion.hpp"
#include "../math/vector3.hpp"

namespace FC { namespace Estimation {

using Math::Quaternionf;
using Math::Vector3f;

/* 24-state error-state EKF. Position/velocity errors are NED, attitude and
 * IMU-bias errors are body-frame. Variances and noise values are SI units. */
constexpr int N = 24;
enum StateIndex {
    IDX_POS = 0, IDX_VEL = 3, IDX_ATT = 6, IDX_BG = 9, IDX_BA = 12,
    IDX_TAU_M = 15, IDX_BMAG = 16, IDX_BBARO = 19, IDX_SBARO = 20,
    IDX_WIND = 21
};

struct Config {
    float init_pos_var = 25.0f;
    float init_vel_var = 4.0f;
    float init_att_var = 0.12f * 0.12f;
    float init_bg_var = 0.02f * 0.02f;
    float init_ba_var = 0.30f * 0.30f;
    float init_tau_m_var = 0.10f * 0.10f;
    float init_bmag_var = 4.0f;
    float init_bbaro_var = 4.0f;
    float init_sbaro_var = 0.01f * 0.01f;
    float init_wind_var = 9.0f;

    float tau_m_nominal = 0.08f;
    float tau_gyro_bias = 3600.0f;
    float tau_accel_bias = 3600.0f;
    float tau_motor = 1.0f;
    float tau_mag = 3600.0f;
    float tau_baro = 300.0f;
    float tau_baro_scale = 3600.0f;
    float tau_wind = 60.0f;

    float k_drag = 0.04f;

    float sigma_accel = 0.25f;
    float sigma_gyro = 0.02f;
    float sigma_bg = 0.0005f;
    float sigma_ba = 0.005f;
    float sigma_tau_m = 0.01f;
    float sigma_bmag = 0.02f;
    float sigma_bbaro = 0.02f;
    float sigma_sbaro = 0.0002f;
    float sigma_wind_h = 0.20f;
    float sigma_wind_v = 0.50f;

    /* These are variance floors (m^2 and (m/s)^2), not standard deviations. */
    float gps_r_pos_floor = 0.25f;
    float gps_r_vel_floor = 0.04f;
    float nis_1dof = 6.635f;
    float nis_3dof = 11.345f;

    float mag_ref_strength = 50.0f; /* uT */
    float mag_r_base = 1.0f;        /* uT^2 */
    float motor_odot_thresh = 50.0f; /* rad/s^2 */
};

struct Preintegrated {
    Vector3f dp_ned;
    Vector3f dv_ned;
    Vector3f specific_force_body_sum;
    Vector3f gyro_body_sum;
    float dt = 0.0f;

    void reset() {
        dp_ned = Vector3f::zero();
        dv_ned = Vector3f::zero();
        specific_force_body_sum = Vector3f::zero();
        gyro_body_sum = Vector3f::zero();
        dt = 0.0f;
    }
};

class EKF {
public:
    EKF();
    explicit EKF(const Config& config);

    void init(const Vector3f& position_ned, const Vector3f& velocity_ned,
              const Quaternionf& body_to_ned);
    bool isInitialised() const { return initialised_; }

    void accumulateIMU(const Vector3f& accel_mps2, const Vector3f& gyro_rps,
                       const Quaternionf& body_to_ned, float dt);
    void predict(const Quaternionf& body_to_ned, float dt_hint);

    void updateGPS(const Vector3f& position_ned, const Vector3f& velocity_ned,
                   float hAcc_m, float vAcc_m, float sAcc_mps,
                   float hDOP, int numSats);
    void updateBaro(float altitude_m, float sigma_m, float vibration_rms);
    void updateGPSAltitude(float altitude_relative_m, float sigma_m);
    void updateMag(const Vector3f& mag_body_uT, const Vector3f& mag_ned_uT,
                   float sigma_uT);
    void updateMotor(float omega_cmd_rad_s, float omega_meas_rad_s,
                     float omega_prev_rad_s, float dt);
    void setWindPrior(const Vector3f& wind_ned_mps, float sigma_mps);

    Vector3f getPosition() const { return position_; }
    Vector3f getVelocity() const { return velocity_; }
    Vector3f getGyroBias() const { return gyro_bias_; }
    Vector3f getAccelBias() const { return accel_bias_; }
    Vector3f getMagBias() const { return mag_bias_; }
    Vector3f getWind() const { return wind_ned_; }
    Quaternionf getAttitude() const { return attitude_; }
    float getMotorTimeConstant() const { return tau_motor_s_; }
    float getBaroBias() const { return baro_bias_m_; }
    float getBaroScale() const { return baro_scale_; }
    void getCovDiag(float diag[N]) const;

private:
    Config cfg_;
    bool initialised_ = false;
    Vector3f position_, velocity_, gyro_bias_, accel_bias_, mag_bias_, wind_ned_;
    Quaternionf attitude_ = Quaternionf::identity();
    Quaternionf attitude_reference_ = Quaternionf::identity();
    float tau_motor_s_ = 0.08f;
    float baro_bias_m_ = 0.0f;
    float baro_scale_ = 0.0f;
    float P_[N][N] = {};
    /* Large matrix workspaces live with the filter rather than on the RTOS
     * task stack (the EKF runs from the latency-sensitive control task). */
    float F_[N][N] = {};
    float Phi_[N][N] = {};
    float Qd_[N][N] = {};
    mutable float scratch_a_[N][N] = {};
    mutable float scratch_b_[N][N] = {};
    float scratch_c_[N][N] = {};
    Preintegrated pre_;
    unsigned gps_rejects_ = 0, baro_rejects_ = 0;
    unsigned mag_rejects_ = 0, motor_rejects_ = 0;

    static void zero(float* matrix, int rows, int cols);
    static void identity(float matrix[N][N]);
    static void multiply(const float a[N][N], const float b[N][N],
                         float out[N][N]);
    static void skew(const Vector3f& v, float out[3][3]);
    static bool finite(float value);

    void buildF(float F[N][N], const Vector3f& force_body,
                const Vector3f& omega_body,
                const Quaternionf& body_to_ned) const;
    void discretise(const float F[N][N], float dt,
                    float Phi[N][N], float Qd[N][N]) const;
    void injectCorrection(const float dx[N]);
    void resetCovariance(const Vector3f& dtheta);
    void stabiliseCovariance();

    template<int M>
    bool kalmanUpdate(const float H[M][N], const float R_diag[M],
                      const float innovation[M], float nis_limit);
};

}} // namespace FC::Estimation

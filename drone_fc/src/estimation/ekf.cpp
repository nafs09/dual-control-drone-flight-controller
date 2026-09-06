#include "ekf.hpp"

#include <cmath>
#include <cstring>

namespace FC { namespace Estimation {
namespace {
constexpr float kGravity = 9.80665f;
constexpr float kMinDt = 1.0e-6f;
constexpr float kMaxPredictDt = 0.05f;
bool finiteVector(const Vector3f& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
bool finiteQuaternion(const Quaternionf& q) {
    const float normSq = q.normSq();
    return std::isfinite(q.w) && std::isfinite(q.x) &&
           std::isfinite(q.y) && std::isfinite(q.z) &&
           std::isfinite(normSq) && normSq > 1.0e-8f;
}
}

EKF::EKF() : cfg_(Config()) { pre_.reset(); }
EKF::EKF(const Config& config) : cfg_(config) { pre_.reset(); }

void EKF::zero(float* m, int rows, int cols) {
    std::memset(m, 0, sizeof(float) * static_cast<unsigned>(rows * cols));
}

void EKF::identity(float m[N][N]) {
    zero(&m[0][0], N, N);
    for (int i = 0; i < N; ++i) m[i][i] = 1.0f;
}

void EKF::multiply(const float a[N][N], const float b[N][N], float out[N][N]) {
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < N; ++k) sum += a[i][k] * b[k][j];
            out[i][j] = sum;
        }
    }
}

void EKF::skew(const Vector3f& v, float s[3][3]) {
    s[0][0] = 0.0f; s[0][1] = -v.z;  s[0][2] =  v.y;
    s[1][0] = v.z;  s[1][1] = 0.0f;  s[1][2] = -v.x;
    s[2][0] = -v.y; s[2][1] = v.x;   s[2][2] = 0.0f;
}

bool EKF::finite(float value) { return std::isfinite(value); }

void EKF::init(const Vector3f& p0, const Vector3f& v0, const Quaternionf& q0) {
    position_ = finiteVector(p0) ? p0 : Vector3f::zero();
    velocity_ = finiteVector(v0) ? v0 : Vector3f::zero();
    attitude_ = finiteQuaternion(q0) ? q0 : Quaternionf::identity();
    attitude_.normalise();
    attitude_reference_ = attitude_;
    gyro_bias_ = Vector3f::zero();
    accel_bias_ = Vector3f::zero();
    mag_bias_ = Vector3f::zero();
    wind_ned_ = Vector3f::zero();
    tau_motor_s_ = cfg_.tau_m_nominal;
    baro_bias_m_ = 0.0f;
    baro_scale_ = 0.0f;
    zero(&P_[0][0], N, N);
    for (int i = 0; i < 3; ++i) {
        P_[IDX_POS+i][IDX_POS+i] = cfg_.init_pos_var;
        P_[IDX_VEL+i][IDX_VEL+i] = cfg_.init_vel_var;
        P_[IDX_ATT+i][IDX_ATT+i] = cfg_.init_att_var;
        P_[IDX_BG+i][IDX_BG+i] = cfg_.init_bg_var;
        P_[IDX_BA+i][IDX_BA+i] = cfg_.init_ba_var;
        P_[IDX_BMAG+i][IDX_BMAG+i] = cfg_.init_bmag_var;
        P_[IDX_WIND+i][IDX_WIND+i] = cfg_.init_wind_var;
    }
    P_[IDX_TAU_M][IDX_TAU_M] = cfg_.init_tau_m_var;
    P_[IDX_BBARO][IDX_BBARO] = cfg_.init_bbaro_var;
    P_[IDX_SBARO][IDX_SBARO] = cfg_.init_sbaro_var;
    pre_.reset();
    gps_rejects_ = baro_rejects_ = mag_rejects_ = motor_rejects_ = 0;
    initialised_ = true;
}

void EKF::accumulateIMU(const Vector3f& accel, const Vector3f& gyro,
                        const Quaternionf& q, float dt) {
    if (!initialised_ || !finite(dt) || dt <= 0.0f || dt > 0.02f ||
        !finiteVector(accel) || !finiteVector(gyro) || !finiteQuaternion(q)) return;

    Quaternionf reference_now = q;
    reference_now.normalise();
    Quaternionf delta_reference = attitude_reference_.conjugate() * reference_now;
    if (delta_reference.w < 0.0f) {
        delta_reference.w = -delta_reference.w;
        delta_reference.x = -delta_reference.x;
        delta_reference.y = -delta_reference.y;
        delta_reference.z = -delta_reference.z;
    }
    attitude_ = attitude_ * delta_reference;
    attitude_.normalise();
    attitude_reference_ = reference_now;

    const Vector3f force_body = accel - accel_bias_;
    const Vector3f omega_body = gyro - gyro_bias_;
    const Vector3f specific_force_ned = attitude_.rotate(force_body);
    Vector3f acceleration_ned = specific_force_ned;
    acceleration_ned.z += kGravity;
    acceleration_ned -= (velocity_ - wind_ned_) * cfg_.k_drag;

    pre_.dp_ned += pre_.dv_ned * dt + acceleration_ned * (0.5f * dt * dt);
    pre_.dv_ned += acceleration_ned * dt;
    pre_.specific_force_body_sum += force_body * dt;
    pre_.gyro_body_sum += omega_body * dt;
    pre_.dt += dt;
}

void EKF::buildF(float F[N][N], const Vector3f& force_body,
                 const Vector3f& omega_body, const Quaternionf& q) const {
    zero(&F[0][0], N, N);
    float R[3][3], Sf[3][3], Sw[3][3];
    q.toRotationMatrix(R);
    skew(force_body, Sf);
    skew(omega_body, Sw);

    for (int i = 0; i < 3; ++i) {
        F[IDX_POS+i][IDX_VEL+i] = 1.0f;
        F[IDX_VEL+i][IDX_VEL+i] = -cfg_.k_drag;
        F[IDX_VEL+i][IDX_WIND+i] = cfg_.k_drag;
        for (int j = 0; j < 3; ++j) {
            float r_sf = 0.0f;
            for (int k = 0; k < 3; ++k) r_sf += R[i][k] * Sf[k][j];
            F[IDX_VEL+i][IDX_ATT+j] = -r_sf;
            F[IDX_VEL+i][IDX_BA+j] = -R[i][j];
            F[IDX_ATT+i][IDX_ATT+j] = -Sw[i][j];
        }
        F[IDX_ATT+i][IDX_BG+i] = -1.0f;
    }

    const auto gm = [this, &F](int index, float tau) {
        if (tau > 0.0f && std::isfinite(tau)) F[index][index] = -1.0f / tau;
    };
    for (int i = 0; i < 3; ++i) {
        gm(IDX_BG+i, cfg_.tau_gyro_bias);
        gm(IDX_BA+i, cfg_.tau_accel_bias);
        gm(IDX_BMAG+i, cfg_.tau_mag);
        gm(IDX_WIND+i, cfg_.tau_wind);
    }
    gm(IDX_TAU_M, cfg_.tau_motor);
    gm(IDX_BBARO, cfg_.tau_baro);
    gm(IDX_SBARO, cfg_.tau_baro_scale);
}

void EKF::discretise(const float F[N][N], float dt,
                     float Phi[N][N], float Qd[N][N]) const {
    /* Third-order state transition and two-node Gauss-Legendre integration
     * of Phi(t) Qc Phi(t)^T. The nodes avoid the old midpoint-only shortcut. */
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) scratch_a_[i][j] = F[i][j] * dt;
    multiply(scratch_a_, scratch_a_, scratch_b_);
    identity(Phi);
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            Phi[i][j] += scratch_a_[i][j] + 0.5f * scratch_b_[i][j];
            float f3 = 0.0f;
            for (int k = 0; k < N; ++k) f3 += scratch_b_[i][k] * scratch_a_[k][j];
            Phi[i][j] += f3 / 6.0f;
        }
    }

    float qdiag[N] = {};
    const auto sq = [](float x) { return x * x; };
    for (int i = 0; i < 3; ++i) {
        qdiag[IDX_VEL+i] = sq(cfg_.sigma_accel);
        qdiag[IDX_ATT+i] = sq(cfg_.sigma_gyro);
        qdiag[IDX_BG+i] = sq(cfg_.sigma_bg);
        qdiag[IDX_BA+i] = sq(cfg_.sigma_ba);
        qdiag[IDX_BMAG+i] = sq(cfg_.sigma_bmag);
        qdiag[IDX_WIND+i] = sq(i == 2 ? cfg_.sigma_wind_v : cfg_.sigma_wind_h);
    }
    qdiag[IDX_TAU_M] = sq(cfg_.sigma_tau_m);
    qdiag[IDX_BBARO] = sq(cfg_.sigma_bbaro);
    qdiag[IDX_SBARO] = sq(cfg_.sigma_sbaro);

    zero(&Qd[0][0], N, N);
    const float c = 0.2886751346f;
    const float nodes[2] = {dt * (0.5f - c), dt * (0.5f + c)};
    float phi_t[N][N];
    for (int sample = 0; sample < 2; ++sample) {
        const float t = nodes[sample];
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j) scratch_a_[i][j] = F[i][j] * t;
        multiply(scratch_a_, scratch_a_, scratch_b_);
        identity(phi_t);
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                phi_t[i][j] += scratch_a_[i][j] + 0.5f * scratch_b_[i][j];
                float f3 = 0.0f;
                for (int k = 0; k < N; ++k) f3 += scratch_b_[i][k] * scratch_a_[k][j];
                phi_t[i][j] += f3 / 6.0f;
            }
        }
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                float sum = 0.0f;
                for (int k = 0; k < N; ++k)
                    sum += phi_t[i][k] * qdiag[k] * phi_t[j][k];
                Qd[i][j] += 0.5f * dt * sum;
            }
        }
    }
    for (int i = 0; i < N; ++i)
        for (int j = i + 1; j < N; ++j)
            Qd[i][j] = Qd[j][i] = 0.5f * (Qd[i][j] + Qd[j][i]);
}

void EKF::predict(const Quaternionf& q_ned, float dt_hint) {
    if (!initialised_) return;
    /* Follow any AHRS increment not already consumed by accumulateIMU while
     * retaining EKF attitude corrections. */
    if (!finiteQuaternion(q_ned)) return;
    Quaternionf reference_now = q_ned;
    reference_now.normalise();
    Quaternionf delta_reference = attitude_reference_.conjugate() * reference_now;
    if (delta_reference.w < 0.0f) {
        delta_reference.w = -delta_reference.w;
        delta_reference.x = -delta_reference.x;
        delta_reference.y = -delta_reference.y;
        delta_reference.z = -delta_reference.z;
    }
    attitude_ = attitude_ * delta_reference;
    attitude_.normalise();
    attitude_reference_ = reference_now;

    const float dt = pre_.dt > kMinDt ? pre_.dt : dt_hint;
    if (!finite(dt) || dt <= 0.0f || dt > kMaxPredictDt) {
        pre_.reset();
        return;
    }
    position_ += pre_.dp_ned;
    velocity_ += pre_.dv_ned;
    const Vector3f force_body = pre_.dt > kMinDt
        ? pre_.specific_force_body_sum * (1.0f / pre_.dt) : Vector3f::zero();
    const Vector3f omega_body = pre_.dt > kMinDt
        ? pre_.gyro_body_sum * (1.0f / pre_.dt) : Vector3f::zero();
    pre_.reset();

    buildF(F_, force_body, omega_body, attitude_);
    discretise(F_, dt, Phi_, Qd_);
    multiply(Phi_, P_, scratch_a_);
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            float value = Qd_[i][j];
            for (int k = 0; k < N; ++k) value += scratch_a_[i][k] * Phi_[j][k];
            scratch_b_[i][j] = value;
        }
    }
    std::memcpy(P_, scratch_b_, sizeof(P_));
    stabiliseCovariance();
}

template<int M>
bool EKF::kalmanUpdate(const float H[M][N], const float Rdiag[M],
                       const float innovation[M], float nis_limit) {
    float HP[M][N], S[M][M], Sinv[M][M];
    for (int i = 0; i < M; ++i) {
        if (!finite(Rdiag[i]) || Rdiag[i] <= 0.0f || !finite(innovation[i])) return false;
        for (int j = 0; j < N; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < N; ++k) sum += H[i][k] * P_[k][j];
            HP[i][j] = sum;
        }
    }
    for (int i = 0; i < M; ++i) {
        for (int j = 0; j < M; ++j) {
            float sum = (i == j) ? Rdiag[i] : 0.0f;
            for (int k = 0; k < N; ++k) sum += HP[i][k] * H[j][k];
            S[i][j] = sum;
        }
    }

    float aug[M][2 * M] = {};
    for (int i = 0; i < M; ++i) {
        for (int j = 0; j < M; ++j) aug[i][j] = S[i][j];
        aug[i][M + i] = 1.0f;
    }
    for (int col = 0; col < M; ++col) {
        int pivot = col;
        for (int row = col + 1; row < M; ++row)
            if (std::fabs(aug[row][col]) > std::fabs(aug[pivot][col])) pivot = row;
        if (!finite(aug[pivot][col]) || std::fabs(aug[pivot][col]) < 1.0e-12f) return false;
        if (pivot != col)
            for (int j = 0; j < 2 * M; ++j) {
                const float t = aug[col][j]; aug[col][j] = aug[pivot][j]; aug[pivot][j] = t;
            }
        const float scale = 1.0f / aug[col][col];
        for (int j = 0; j < 2 * M; ++j) aug[col][j] *= scale;
        for (int row = 0; row < M; ++row) {
            if (row == col) continue;
            const float factor = aug[row][col];
            for (int j = 0; j < 2 * M; ++j) aug[row][j] -= factor * aug[col][j];
        }
    }
    for (int i = 0; i < M; ++i)
        for (int j = 0; j < M; ++j) Sinv[i][j] = aug[i][M + j];

    float nis = 0.0f;
    for (int i = 0; i < M; ++i)
        for (int j = 0; j < M; ++j) nis += innovation[i] * Sinv[i][j] * innovation[j];
    if (!finite(nis) || nis > nis_limit) return false;

    float PHt[N][M], K[N][M], dx[N] = {};
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < M; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < N; ++k) sum += P_[i][k] * H[j][k];
            PHt[i][j] = sum;
        }
    }
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < M; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < M; ++k) sum += PHt[i][k] * Sinv[k][j];
            K[i][j] = sum;
            dx[i] += sum * innovation[j];
        }
    }

    /* Joseph covariance update preserves symmetry and positive semidefiniteness. */
    identity(scratch_a_);
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
            for (int k = 0; k < M; ++k) scratch_a_[i][j] -= K[i][k] * H[k][j];
    multiply(scratch_a_, P_, scratch_b_);
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < N; ++k) sum += scratch_b_[i][k] * scratch_a_[j][k];
            for (int k = 0; k < M; ++k) sum += K[i][k] * Rdiag[k] * K[j][k];
            scratch_c_[i][j] = sum;
        }
    }
    std::memcpy(P_, scratch_c_, sizeof(P_));
    stabiliseCovariance();
    injectCorrection(dx);
    return true;
}

void EKF::injectCorrection(const float dx[N]) {
    position_ += Vector3f(dx[IDX_POS], dx[IDX_POS+1], dx[IDX_POS+2]);
    velocity_ += Vector3f(dx[IDX_VEL], dx[IDX_VEL+1], dx[IDX_VEL+2]);
    const Vector3f dtheta(dx[IDX_ATT], dx[IDX_ATT+1], dx[IDX_ATT+2]);
    gyro_bias_ += Vector3f(dx[IDX_BG], dx[IDX_BG+1], dx[IDX_BG+2]);
    accel_bias_ += Vector3f(dx[IDX_BA], dx[IDX_BA+1], dx[IDX_BA+2]);
    mag_bias_ += Vector3f(dx[IDX_BMAG], dx[IDX_BMAG+1], dx[IDX_BMAG+2]);
    tau_motor_s_ = std::fmax(0.001f, std::fmin(1.0f, tau_motor_s_ + dx[IDX_TAU_M]));
    baro_bias_m_ += dx[IDX_BBARO];
    baro_scale_ = std::fmax(-0.2f, std::fmin(0.2f, baro_scale_ + dx[IDX_SBARO]));
    wind_ned_ += Vector3f(dx[IDX_WIND], dx[IDX_WIND+1], dx[IDX_WIND+2]);

    attitude_ = attitude_ * Quaternionf::expMap(dtheta * 0.5f);
    attitude_.normalise();
    resetCovariance(dtheta);
}

void EKF::resetCovariance(const Vector3f& dtheta) {
    float G[3][3], S[3][3];
    skew(dtheta * 0.5f, S);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) G[i][j] = (i == j ? 1.0f : 0.0f) - S[i][j];

    std::memcpy(scratch_a_, P_, sizeof(P_));
    for (int i = 0; i < 3; ++i)
        for (int col = 0; col < N; ++col) {
            float value = 0.0f;
            for (int k = 0; k < 3; ++k) value += G[i][k] * P_[IDX_ATT+k][col];
            scratch_a_[IDX_ATT+i][col] = value;
        }
    std::memcpy(scratch_b_, scratch_a_, sizeof(P_));
    for (int row = 0; row < N; ++row)
        for (int j = 0; j < 3; ++j) {
            float value = 0.0f;
            for (int k = 0; k < 3; ++k) value += scratch_a_[row][IDX_ATT+k] * G[j][k];
            scratch_b_[row][IDX_ATT+j] = value;
        }
    std::memcpy(P_, scratch_b_, sizeof(P_));
    stabiliseCovariance();
}

void EKF::stabiliseCovariance() {
    for (int i = 0; i < N; ++i) {
        if (!finite(P_[i][i]) || P_[i][i] < 1.0e-12f) P_[i][i] = 1.0e-12f;
        for (int j = i + 1; j < N; ++j) {
            float value = 0.5f * (P_[i][j] + P_[j][i]);
            if (!finite(value)) value = 0.0f;
            P_[i][j] = P_[j][i] = value;
        }
    }
}

void EKF::updateGPS(const Vector3f& gps_pos, const Vector3f& gps_vel,
                    float hAcc, float vAcc, float sAcc, float hDOP, int sats) {
    if (!initialised_ || !finite(hAcc) || !finite(vAcc) || !finite(sAcc) ||
        !finite(hDOP) || hAcc < 0.0f || vAcc < 0.0f || sAcc < 0.0f ||
        !finite(gps_pos.x) || !finite(gps_pos.y) ||
        !finite(gps_pos.z) || !finite(gps_vel.x) || !finite(gps_vel.y) ||
        !finite(gps_vel.z)) return;
    /* NAV-PVT accuracy fields already include receiver geometry estimates;
     * multiplying by HDOP again overstates the reported variance. */
    (void)hDOP;
    (void)sats;
    const float r_pos_h = std::fmax(hAcc * hAcc, cfg_.gps_r_pos_floor);
    const float r_pos_v = std::fmax(vAcc * vAcc, cfg_.gps_r_pos_floor);
    const float r_vel_h = std::fmax(sAcc * sAcc, cfg_.gps_r_vel_floor);
    const float r_vel_v = std::fmax(sAcc * sAcc, cfg_.gps_r_vel_floor);

    float H[3][N] = {}, R[3], y[3];
    H[0][IDX_POS] = H[1][IDX_POS+1] = H[2][IDX_POS+2] = 1.0f;
    R[0] = R[1] = r_pos_h; R[2] = r_pos_v;
    y[0] = gps_pos.x - position_.x; y[1] = gps_pos.y - position_.y; y[2] = gps_pos.z - position_.z;
    if (!kalmanUpdate<3>(H, R, y, cfg_.nis_3dof)) ++gps_rejects_;

    zero(&H[0][0], 3, N);
    H[0][IDX_VEL] = H[1][IDX_VEL+1] = H[2][IDX_VEL+2] = 1.0f;
    R[0] = R[1] = r_vel_h; R[2] = r_vel_v;
    y[0] = gps_vel.x - velocity_.x; y[1] = gps_vel.y - velocity_.y; y[2] = gps_vel.z - velocity_.z;
    if (!kalmanUpdate<3>(H, R, y, cfg_.nis_3dof)) ++gps_rejects_;
}

void EKF::updateBaro(float altitude, float sigma, float vibration) {
    if (!initialised_ || !finite(altitude) || !finite(sigma) || sigma <= 0.0f ||
        !finite(vibration) || vibration < 0.0f) return;
    const float sigma_eff = std::fmax(0.1f, std::sqrt(sigma*sigma + 0.1f*vibration*vibration));
    const float h = -position_.z;
    const float prediction = h * (1.0f + baro_scale_) + baro_bias_m_;
    float H[1][N] = {}, R[1] = {sigma_eff * sigma_eff}, y[1] = {altitude - prediction};
    H[0][IDX_POS+2] = -(1.0f + baro_scale_);
    H[0][IDX_BBARO] = 1.0f;
    H[0][IDX_SBARO] = h;
    if (!kalmanUpdate<1>(H, R, y, cfg_.nis_1dof)) ++baro_rejects_;
}

void EKF::updateGPSAltitude(float altitude_relative, float sigma) {
    if (!initialised_ || !finite(altitude_relative) || !finite(sigma) || sigma <= 0.0f) return;
    float H[1][N] = {}, R[1] = {std::fmax(sigma*sigma, cfg_.gps_r_pos_floor)};
    float y[1] = {altitude_relative + position_.z};
    H[0][IDX_POS+2] = -1.0f;
    if (!kalmanUpdate<1>(H, R, y, cfg_.nis_1dof)) ++gps_rejects_;
}

void EKF::updateMag(const Vector3f& mag_body, const Vector3f& mag_ned, float sigma) {
    if (!initialised_ || !finite(sigma) || sigma <= 0.0f ||
        mag_body.norm() < 1.0e-3f || mag_ned.norm() < 1.0e-3f) return;
    const Vector3f predicted = attitude_.rotateInverse(mag_ned);
    const Vector3f innovation = mag_body - (predicted + mag_bias_);
    float Sm[3][3]; skew(predicted, Sm);
    const float magnitude_error = std::fabs(mag_body.norm() - cfg_.mag_ref_strength);
    const float variance = std::fmax(cfg_.mag_r_base, sigma*sigma) *
                           (1.0f + 4.0f * magnitude_error * magnitude_error /
                            std::fmax(cfg_.mag_ref_strength * cfg_.mag_ref_strength, 1.0f));
    float H[3][N] = {}, R[3] = {variance, variance, variance};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) H[i][IDX_ATT+j] = Sm[i][j];
        H[i][IDX_BMAG+i] = 1.0f;
    }
    const float y[3] = {innovation.x, innovation.y, innovation.z};
    if (!kalmanUpdate<3>(H, R, y, cfg_.nis_3dof)) ++mag_rejects_;
}

void EKF::updateMotor(float omega_cmd, float omega_meas, float omega_prev, float dt) {
    if (!initialised_ || !finite(omega_cmd) || !finite(omega_meas) ||
        !finite(omega_prev) || !finite(dt) || dt <= kMinDt) return;
    const float omega_dot = (omega_meas - omega_prev) / dt;
    if (std::fabs(omega_dot) < cfg_.motor_odot_thresh) return;
    const float tau_observed = (omega_cmd - omega_meas) / omega_dot;
    if (!finite(tau_observed) || tau_observed < 0.001f || tau_observed > 1.0f) {
        ++motor_rejects_;
        return;
    }
    float H[1][N] = {}, R[1] = {0.01f};
    float y[1] = {tau_observed - tau_motor_s_};
    H[0][IDX_TAU_M] = 1.0f;
    if (!kalmanUpdate<1>(H, R, y, cfg_.nis_1dof)) ++motor_rejects_;
}

void EKF::setWindPrior(const Vector3f& wind, float sigma) {
    if (!initialised_ || !finite(sigma) || sigma <= 0.0f) return;
    float H[3][N] = {}, R[3] = {sigma*sigma, sigma*sigma, 100.0f*sigma*sigma};
    const float y[3] = {wind.x-wind_ned_.x, wind.y-wind_ned_.y, wind.z-wind_ned_.z};
    H[0][IDX_WIND] = H[1][IDX_WIND+1] = H[2][IDX_WIND+2] = 1.0f;
    (void)kalmanUpdate<3>(H, R, y, cfg_.nis_3dof * 10.0f);
}

void EKF::getCovDiag(float diag[N]) const {
    if (!diag) return;
    for (int i = 0; i < N; ++i) diag[i] = P_[i][i];
}

template bool EKF::kalmanUpdate<1>(const float[1][N], const float[1], const float[1], float);
template bool EKF::kalmanUpdate<3>(const float[3][N], const float[3], const float[3], float);

}} // namespace FC::Estimation

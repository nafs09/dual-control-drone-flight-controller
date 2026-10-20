#include "invariant_ekf.hpp"

#include <cmath>
#include <cstring>

namespace FC { namespace Estimation {
namespace {
constexpr float kGravity = 9.80665f;
constexpr float kMinDt = 1.0e-6f;
constexpr float kMaxDt = 0.05f;
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

InvEKF::InvEKF() : cfg_(Config()) {}
InvEKF::InvEKF(const Config& config) : cfg_(config) {}

void InvEKF::zero(float* m, int rows, int cols) {
    std::memset(m, 0, sizeof(float) * static_cast<unsigned>(rows * cols));
}

void InvEKF::identity(float m[kStateSize][kStateSize]) {
    zero(&m[0][0], kStateSize, kStateSize);
    for (int i = 0; i < kStateSize; ++i) m[i][i] = 1.0f;
}

void InvEKF::multiply(const float a[kStateSize][kStateSize],
                      const float b[kStateSize][kStateSize],
                      float out[kStateSize][kStateSize]) {
    for (int i = 0; i < kStateSize; ++i)
        for (int j = 0; j < kStateSize; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < kStateSize; ++k) sum += a[i][k] * b[k][j];
            out[i][j] = sum;
        }
}

void InvEKF::skew(const Vector3f& v, float s[3][3]) {
    s[0][0]=0.0f; s[0][1]=-v.z;  s[0][2]=v.y;
    s[1][0]=v.z;  s[1][1]=0.0f;  s[1][2]=-v.x;
    s[2][0]=-v.y; s[2][1]=v.x;   s[2][2]=0.0f;
}

bool InvEKF::finite(float value) { return std::isfinite(value); }

bool InvEKF::gpsQualityAcceptable(float positionDOP, int numSats) const {
    return finite(positionDOP) && positionDOP > 0.0f &&
           positionDOP <= cfg_.gps_max_position_dop &&
           numSats >= cfg_.gps_min_satellites;
}

void InvEKF::init(const Vector3f& p, const Vector3f& v, const Quaternionf& q) {
    position_ned_ = finiteVector(p) ? p : Vector3f::zero();
    velocity_ned_ = finiteVector(v) ? v : Vector3f::zero();
    body_to_ned_ = finiteQuaternion(q) ? q : Quaternionf::identity();
    body_to_ned_.normalise();
    attitude_reference_ = body_to_ned_;
    gyro_bias_ = Vector3f::zero();
    accel_bias_ = Vector3f::zero();
    mag_bias_ = Vector3f::zero();
    wind_ned_ = Vector3f::zero();
    motor_tau_s_ = cfg_.motor_tau_nominal_s;
    baro_bias_m_ = 0.0f;
    baro_scale_ = 0.0f;
    force_body_sum_ = Vector3f::zero();
    omega_body_sum_ = Vector3f::zero();
    covariance_dt_ = 0.0f;
    zero(&P_[0][0], kStateSize, kStateSize);
    for (int i = 0; i < 3; ++i) {
        P_[ATT+i][ATT+i] = cfg_.init_attitude_var;
        P_[VEL+i][VEL+i] = cfg_.init_velocity_var;
        P_[POS+i][POS+i] = cfg_.init_position_var;
        P_[BG+i][BG+i] = cfg_.init_gyro_bias_var;
        P_[BA+i][BA+i] = cfg_.init_accel_bias_var;
        P_[BMAG+i][BMAG+i] = cfg_.init_mag_bias_var;
        P_[WIND+i][WIND+i] = cfg_.init_wind_var;
    }
    P_[TAU_M][TAU_M] = cfg_.init_motor_tau_var;
    P_[BBARO][BBARO] = cfg_.init_baro_bias_var;
    P_[SBARO][SBARO] = cfg_.init_baro_scale_var;
    gps_rejects_ = baro_rejects_ = mag_rejects_ = motor_rejects_ = 0;
    initialised_ = true;
}

void InvEKF::accumulateIMU(const Vector3f& accel, const Vector3f& gyro,
                           const Quaternionf& mahony_q, float dt) {
    if (!initialised_ || !finite(dt) || dt <= 0.0f || dt > 0.02f ||
        !finiteVector(accel) || !finiteVector(gyro) || !finiteQuaternion(mahony_q)) return;

    Quaternionf reference_now = mahony_q;
    reference_now.normalise();
    Quaternionf delta_reference = attitude_reference_.conjugate() * reference_now;
    if (delta_reference.w < 0.0f) {
        delta_reference.w = -delta_reference.w;
        delta_reference.x = -delta_reference.x;
        delta_reference.y = -delta_reference.y;
        delta_reference.z = -delta_reference.z;
    }
    const Quaternionf attitude_before = body_to_ned_;
    body_to_ned_ = body_to_ned_ * delta_reference;
    body_to_ned_.normalise();
    attitude_reference_ = reference_now;

    const Vector3f omega = gyro - gyro_bias_;
    const Vector3f force = accel - accel_bias_;

    const Quaternionf q_mid = Quaternionf::slerp(attitude_before, body_to_ned_, 0.5f);
    Vector3f acceleration_ned = q_mid.rotate(force) + Vector3f(0.0f, 0.0f, kGravity);
    acceleration_ned -= (velocity_ned_ - wind_ned_) * cfg_.drag_per_s;
    position_ned_ += velocity_ned_ * dt + acceleration_ned * (0.5f * dt * dt);
    velocity_ned_ += acceleration_ned * dt;

    force_body_sum_ += force * dt;
    omega_body_sum_ += omega * dt;
    covariance_dt_ += dt;
}

void InvEKF::buildF(float F[kStateSize][kStateSize], const Vector3f& force,
                    const Vector3f& omega) const {
    zero(&F[0][0], kStateSize, kStateSize);
    float Sw[3][3], Sf[3][3], Rned[3][3];
    skew(omega, Sw);
    skew(force, Sf);
    body_to_ned_.toRotationMatrix(Rned);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            F[ATT+i][ATT+j] = -Sw[i][j];
            F[VEL+i][VEL+j] = -Sw[i][j] - (i == j ? cfg_.drag_per_s : 0.0f);
            F[POS+i][POS+j] = -Sw[i][j];
            F[VEL+i][ATT+j] = -Sf[i][j];
            F[VEL+i][WIND+j] = cfg_.drag_per_s * Rned[j][i];
        }
        F[ATT+i][BG+i] = -1.0f;
        F[VEL+i][BA+i] = -1.0f;
        F[POS+i][VEL+i] = 1.0f;
        if (cfg_.gyro_bias_tau_s > 0.0f)
            F[BG+i][BG+i] = -1.0f / cfg_.gyro_bias_tau_s;
        if (cfg_.accel_bias_tau_s > 0.0f)
            F[BA+i][BA+i] = -1.0f / cfg_.accel_bias_tau_s;
        if (cfg_.mag_bias_tau_s > 0.0f)
            F[BMAG+i][BMAG+i] = -1.0f / cfg_.mag_bias_tau_s;
        if (cfg_.wind_tau_s > 0.0f)
            F[WIND+i][WIND+i] = -1.0f / cfg_.wind_tau_s;
    }
    if (cfg_.motor_tau_tau_s > 0.0f) F[TAU_M][TAU_M] = -1.0f / cfg_.motor_tau_tau_s;
    if (cfg_.baro_bias_tau_s > 0.0f) F[BBARO][BBARO] = -1.0f / cfg_.baro_bias_tau_s;
    if (cfg_.baro_scale_tau_s > 0.0f) F[SBARO][SBARO] = -1.0f / cfg_.baro_scale_tau_s;
}

void InvEKF::discretise(float dt) {
    const auto transitionAt = [this](float t, float out[kStateSize][kStateSize]) {
        for (int i = 0; i < kStateSize; ++i)
            for (int j = 0; j < kStateSize; ++j) scratch_a_[i][j] = F_[i][j] * t;
        multiply(scratch_a_, scratch_a_, scratch_b_);
        identity(out);
        for (int i = 0; i < kStateSize; ++i)
            for (int j = 0; j < kStateSize; ++j) {
                out[i][j] += scratch_a_[i][j] + 0.5f * scratch_b_[i][j];
                float f3 = 0.0f;
                for (int k = 0; k < kStateSize; ++k)
                    f3 += scratch_b_[i][k] * scratch_a_[k][j];
                out[i][j] += f3 / 6.0f;
            }
    };

    transitionAt(dt, Phi_);
    float qdiag[kStateSize] = {};
    for (int i = 0; i < 3; ++i) {
        qdiag[ATT+i] = cfg_.gyro_noise * cfg_.gyro_noise;
        qdiag[VEL+i] = cfg_.accel_noise * cfg_.accel_noise;
        qdiag[BG+i] = cfg_.gyro_bias_rw * cfg_.gyro_bias_rw;
        qdiag[BA+i] = cfg_.accel_bias_rw * cfg_.accel_bias_rw;
        qdiag[BMAG+i] = cfg_.mag_bias_rw * cfg_.mag_bias_rw;
        qdiag[WIND+i] = (i == 2 ? cfg_.wind_rw_vertical : cfg_.wind_rw_horizontal) *
                        (i == 2 ? cfg_.wind_rw_vertical : cfg_.wind_rw_horizontal);
    }
    qdiag[TAU_M] = cfg_.motor_tau_rw * cfg_.motor_tau_rw;
    qdiag[BBARO] = cfg_.baro_bias_rw * cfg_.baro_bias_rw;
    qdiag[SBARO] = cfg_.baro_scale_rw * cfg_.baro_scale_rw;
    zero(&Qd_[0][0], kStateSize, kStateSize);
    const float c = 0.2886751346f;
    const float nodes[2] = {dt * (0.5f - c), dt * (0.5f + c)};
    float phi_t[kStateSize][kStateSize];
    for (int n = 0; n < 2; ++n) {
        transitionAt(nodes[n], phi_t);
        for (int i = 0; i < kStateSize; ++i)
            for (int j = 0; j < kStateSize; ++j) {
                float sum = 0.0f;
                for (int k = 0; k < kStateSize; ++k)
                    sum += phi_t[i][k] * qdiag[k] * phi_t[j][k];
                Qd_[i][j] += 0.5f * dt * sum;
            }
    }
    for (int i = 0; i < kStateSize; ++i)
        for (int j = i + 1; j < kStateSize; ++j)
            Qd_[i][j] = Qd_[j][i] = 0.5f * (Qd_[i][j] + Qd_[j][i]);
}

void InvEKF::predict(float dt_hint) {
    if (!initialised_) return;
    const float dt = covariance_dt_ > kMinDt ? covariance_dt_ : dt_hint;
    if (!finite(dt) || dt <= 0.0f || dt > kMaxDt) {
        covariance_dt_ = 0.0f;
        force_body_sum_ = omega_body_sum_ = Vector3f::zero();
        return;
    }
    const Vector3f force = covariance_dt_ > kMinDt
        ? force_body_sum_ * (1.0f / covariance_dt_) : Vector3f::zero();
    const Vector3f omega = covariance_dt_ > kMinDt
        ? omega_body_sum_ * (1.0f / covariance_dt_) : Vector3f::zero();
    force_body_sum_ = omega_body_sum_ = Vector3f::zero();
    covariance_dt_ = 0.0f;

    /* Match the Gauss-Markov terms in F: slowly varying calibration states
     * relax toward zero, while the motor lag relaxes toward its configured
     * nominal value. */
    const auto decay = [dt](float tau) {
        return (std::isfinite(tau) && tau > 0.0f) ? std::exp(-dt / tau) : 1.0f;
    };
    gyro_bias_ *= decay(cfg_.gyro_bias_tau_s);
    accel_bias_ *= decay(cfg_.accel_bias_tau_s);
    mag_bias_ *= decay(cfg_.mag_bias_tau_s);
    wind_ned_ *= decay(cfg_.wind_tau_s);
    baro_bias_m_ *= decay(cfg_.baro_bias_tau_s);
    baro_scale_ *= decay(cfg_.baro_scale_tau_s);
    motor_tau_s_ = cfg_.motor_tau_nominal_s +
        (motor_tau_s_ - cfg_.motor_tau_nominal_s) * decay(cfg_.motor_tau_tau_s);

    buildF(F_, force, omega);
    discretise(dt);
    multiply(Phi_, P_, scratch_a_);
    for (int i = 0; i < kStateSize; ++i)
        for (int j = 0; j < kStateSize; ++j) {
            float sum = Qd_[i][j];
            for (int k = 0; k < kStateSize; ++k) sum += scratch_a_[i][k] * Phi_[j][k];
            scratch_b_[i][j] = sum;
        }
    std::memcpy(P_, scratch_b_, sizeof(P_));
    stabiliseCovariance();
}

template<int M>
bool InvEKF::update(const float H[M][kStateSize], const float R[M],
                    const float innovation[M], float nis_limit) {
    float HP[M][kStateSize], S[M][M], aug[M][2*M] = {};
    for (int i = 0; i < M; ++i) {
        if (!finite(R[i]) || R[i] <= 0.0f || !finite(innovation[i])) return false;
        for (int j = 0; j < kStateSize; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < kStateSize; ++k) sum += H[i][k] * P_[k][j];
            HP[i][j] = sum;
        }
    }
    for (int i = 0; i < M; ++i)
        for (int j = 0; j < M; ++j) {
            float sum = (i == j) ? R[i] : 0.0f;
            for (int k = 0; k < kStateSize; ++k) sum += HP[i][k] * H[j][k];
            S[i][j] = sum;
            aug[i][j] = sum;
        }
    for (int i = 0; i < M; ++i) aug[i][M+i] = 1.0f;
    for (int col = 0; col < M; ++col) {
        int pivot = col;
        for (int row = col + 1; row < M; ++row)
            if (std::fabs(aug[row][col]) > std::fabs(aug[pivot][col])) pivot = row;
        if (!finite(aug[pivot][col]) || std::fabs(aug[pivot][col]) < 1.0e-12f) return false;
        if (pivot != col)
            for (int j = 0; j < 2*M; ++j) {
                const float t = aug[col][j]; aug[col][j] = aug[pivot][j]; aug[pivot][j] = t;
            }
        const float scale = 1.0f / aug[col][col];
        for (int j = 0; j < 2*M; ++j) aug[col][j] *= scale;
        for (int row = 0; row < M; ++row) {
            if (row == col) continue;
            const float factor = aug[row][col];
            for (int j = 0; j < 2*M; ++j) aug[row][j] -= factor * aug[col][j];
        }
    }

    float nis = 0.0f;
    for (int i = 0; i < M; ++i)
        for (int j = 0; j < M; ++j) nis += innovation[i] * aug[i][M+j] * innovation[j];
    if (!finite(nis) || nis > nis_limit) return false;

    float K[kStateSize][M], dx[kStateSize] = {};
    for (int i = 0; i < kStateSize; ++i)
        for (int j = 0; j < M; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < kStateSize; ++k)
                for (int n = 0; n < M; ++n) sum += P_[i][k] * H[n][k] * aug[n][M+j];
            K[i][j] = sum;
            dx[i] += sum * innovation[j];
        }

    identity(scratch_a_);
    for (int i = 0; i < kStateSize; ++i)
        for (int j = 0; j < kStateSize; ++j)
            for (int k = 0; k < M; ++k) scratch_a_[i][j] -= K[i][k] * H[k][j];
    multiply(scratch_a_, P_, scratch_b_);
    for (int i = 0; i < kStateSize; ++i)
        for (int j = 0; j < kStateSize; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < kStateSize; ++k) sum += scratch_b_[i][k] * scratch_a_[j][k];
            for (int k = 0; k < M; ++k) sum += K[i][k] * R[k] * K[j][k];
            scratch_c_[i][j] = sum;
        }
    std::memcpy(P_, scratch_c_, sizeof(P_));
    stabiliseCovariance();
    inject(dx);
    return true;
}

void InvEKF::inject(const float dx[kStateSize]) {
    const Vector3f dtheta(dx[ATT], dx[ATT+1], dx[ATT+2]);
    const Vector3f dvel(dx[VEL], dx[VEL+1], dx[VEL+2]);
    const Vector3f dpos(dx[POS], dx[POS+1], dx[POS+2]);
    position_ned_ += body_to_ned_.rotate(dpos);
    velocity_ned_ += body_to_ned_.rotate(dvel);
    gyro_bias_ += Vector3f(dx[BG], dx[BG+1], dx[BG+2]);
    accel_bias_ += Vector3f(dx[BA], dx[BA+1], dx[BA+2]);
    mag_bias_ += Vector3f(dx[BMAG], dx[BMAG+1], dx[BMAG+2]);
    motor_tau_s_ = std::fmax(0.001f, std::fmin(1.0f, motor_tau_s_ + dx[TAU_M]));
    baro_bias_m_ += dx[BBARO];
    baro_scale_ = std::fmax(-0.2f, std::fmin(0.2f, baro_scale_ + dx[SBARO]));
    wind_ned_ += Vector3f(dx[WIND], dx[WIND+1], dx[WIND+2]);
    body_to_ned_ = body_to_ned_ * Quaternionf::expMap(dtheta * 0.5f);
    body_to_ned_.normalise();
    resetCovariance(dx);
}

void InvEKF::resetCovariance(const float dx[kStateSize]) {
    /* First-order BCH reset Jacobian G = I - 1/2 ad(dx) for the
     * left-invariant SE_2(3) error. Bias coordinates remain additive. */
    float G[kStateSize][kStateSize];
    identity(G);
    const Vector3f dtheta(dx[ATT], dx[ATT+1], dx[ATT+2]);
    const Vector3f dvel(dx[VEL], dx[VEL+1], dx[VEL+2]);
    const Vector3f dpos(dx[POS], dx[POS+1], dx[POS+2]);
    float St[3][3], Sv[3][3], Sp[3][3];
    skew(dtheta, St); skew(dvel, Sv); skew(dpos, Sp);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            G[ATT+i][ATT+j] -= 0.5f * St[i][j];
            G[VEL+i][ATT+j] -= 0.5f * Sv[i][j];
            G[VEL+i][VEL+j] -= 0.5f * St[i][j];
            G[POS+i][ATT+j] -= 0.5f * Sp[i][j];
            G[POS+i][POS+j] -= 0.5f * St[i][j];
        }
    multiply(G, P_, scratch_a_);
    for (int i = 0; i < kStateSize; ++i)
        for (int j = 0; j < kStateSize; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < kStateSize; ++k) sum += scratch_a_[i][k] * G[j][k];
            scratch_b_[i][j] = sum;
        }
    std::memcpy(P_, scratch_b_, sizeof(P_));
    stabiliseCovariance();
}

void InvEKF::stabiliseCovariance() {
    for (int i = 0; i < kStateSize; ++i) {
        if (!finite(P_[i][i]) || P_[i][i] < 1.0e-12f) P_[i][i] = 1.0e-12f;
        for (int j = i + 1; j < kStateSize; ++j) {
            float value = 0.5f * (P_[i][j] + P_[j][i]);
            if (!finite(value)) value = 0.0f;
            P_[i][j] = P_[j][i] = value;
        }
    }
}

void InvEKF::updateGPS(const Vector3f& gps_pos, const Vector3f& gps_vel,
                       float hAcc, float vAcc, float sAcc, float positionDOP, int sats) {
    if (!initialised_ || !finite(hAcc) || !finite(vAcc) || !finite(sAcc) ||
        !finite(positionDOP) || hAcc < 0.0f || vAcc < 0.0f || sAcc < 0.0f ||
        !finite(gps_pos.x) || !finite(gps_pos.y) || !finite(gps_pos.z) ||
        !finite(gps_vel.x) || !finite(gps_vel.y) || !finite(gps_vel.z)) return;
    if (!gpsQualityAcceptable(positionDOP, sats)) {
        ++gps_rejects_;
        return;
    }
    const float pos_var[3] = {
        std::fmax(hAcc*hAcc, cfg_.gps_position_floor_m2),
        std::fmax(hAcc*hAcc, cfg_.gps_position_floor_m2),
        std::fmax(vAcc*vAcc, cfg_.gps_position_floor_m2)};
    const float vel_var[3] = {
        std::fmax(sAcc*sAcc, cfg_.gps_velocity_floor_m2ps2),
        std::fmax(sAcc*sAcc, cfg_.gps_velocity_floor_m2ps2),
        std::fmax(sAcc*sAcc, cfg_.gps_velocity_floor_m2ps2)};
    float Hpos[3][kStateSize] = {};
    float Rpos[3][3];
    body_to_ned_.toRotationMatrix(Rpos);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) Hpos[i][POS+j] = Rpos[i][j];
    const float rp[3] = {gps_pos.x-position_ned_.x,
                         gps_pos.y-position_ned_.y,
                         gps_pos.z-position_ned_.z};
    if (!update<3>(Hpos, pos_var, rp, cfg_.nis_3dof)) ++gps_rejects_;

    float Hvel[3][kStateSize] = {};
    float Rvel[3][3];
    body_to_ned_.toRotationMatrix(Rvel);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) Hvel[i][VEL+j] = Rvel[i][j];
    const float rv[3] = {gps_vel.x-velocity_ned_.x,
                         gps_vel.y-velocity_ned_.y,
                         gps_vel.z-velocity_ned_.z};
    if (!update<3>(Hvel, vel_var, rv, cfg_.nis_3dof)) ++gps_rejects_;
}

void InvEKF::updateBaro(float altitude, float sigma, float vibration) {
    if (!initialised_ || !finite(altitude) || !finite(vibration) || vibration < 0.0f) return;
    if (!finite(sigma) || sigma <= 0.0f) sigma = cfg_.baro_sigma_m;
    if (!finite(sigma) || sigma <= 0.0f) return;
    const float sigma_eff = std::fmax(0.1f, std::sqrt(sigma*sigma + 0.1f*vibration*vibration));
    const float height = -position_ned_.z;
    const float prediction = height * (1.0f + baro_scale_) + baro_bias_m_;
    float Rned[3][3]; body_to_ned_.toRotationMatrix(Rned);
    float H[1][kStateSize] = {}, variance[1] = {sigma_eff*sigma_eff};
    for (int j = 0; j < 3; ++j) H[0][POS+j] = -(1.0f + baro_scale_) * Rned[2][j];
    H[0][BBARO] = 1.0f;
    H[0][SBARO] = height;
    const float y[1] = {altitude - prediction};
    if (!update<1>(H, variance, y, cfg_.nis_1dof)) ++baro_rejects_;
}

void InvEKF::updateMag(const Vector3f& mag_body, const Vector3f& mag_ned, float sigma) {
    if (!initialised_ || !finiteVector(mag_body) || !finiteVector(mag_ned) ||
        mag_body.norm() < 1.0e-6f || mag_ned.norm() < 1.0e-6f) return;
    const Vector3f predicted_field = body_to_ned_.rotateInverse(mag_ned);
    const Vector3f predicted = predicted_field + mag_bias_;
    const Vector3f innovation = mag_body - predicted;
    float Sm[3][3]; skew(predicted_field, Sm);
    float H[3][kStateSize] = {};
    const float sigma_eff = (finite(sigma) && sigma > 0.0f) ? sigma : cfg_.mag_sigma;
    const float magnitude_error = std::fabs(mag_body.norm() - cfg_.mag_reference_strength_uT);
    const float base_variance = std::fmax(cfg_.mag_variance_floor_uT2, sigma_eff*sigma_eff);
    const float variance = base_variance * (1.0f + 4.0f * magnitude_error * magnitude_error /
        std::fmax(cfg_.mag_reference_strength_uT * cfg_.mag_reference_strength_uT, 1.0f));
    float R[3] = {variance, variance, variance};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            H[i][ATT+j] = Sm[i][j];
            H[i][BMAG+j] = (i == j) ? 1.0f : 0.0f;
        }
    const float y[3] = {innovation.x, innovation.y, innovation.z};
    if (!update<3>(H, R, y, cfg_.nis_3dof)) ++mag_rejects_;
}

void InvEKF::updateGPSAltitude(float altitude, float sigma) {
    if (!initialised_ || !finite(altitude) || !finite(sigma) || sigma <= 0.0f) return;
    float Rned[3][3]; body_to_ned_.toRotationMatrix(Rned);
    float H[1][kStateSize] = {}, variance[1] = {std::fmax(sigma*sigma, cfg_.gps_position_floor_m2)};
    for (int j = 0; j < 3; ++j) H[0][POS+j] = -Rned[2][j];
    const float y[1] = {altitude + position_ned_.z};
    if (!update<1>(H, variance, y, cfg_.nis_1dof)) ++gps_rejects_;
}

void InvEKF::updateMotor(float omega_cmd, float omega_meas, float omega_prev, float dt) {
    if (!initialised_ || !finite(omega_cmd) || !finite(omega_meas) ||
        !finite(omega_prev) || !finite(dt) || dt <= kMinDt) return;
    const float omega_dot = (omega_meas - omega_prev) / dt;
    if (std::fabs(omega_dot) < cfg_.motor_accel_gate_rad_s2) return;
    const float tau_observed = (omega_cmd - omega_meas) / omega_dot;
    if (!finite(tau_observed) || tau_observed < 0.001f || tau_observed > 1.0f) {
        ++motor_rejects_;
        return;
    }
    float H[1][kStateSize] = {}, R[1] = {0.01f}, y[1] = {tau_observed - motor_tau_s_};
    H[0][TAU_M] = 1.0f;
    if (!update<1>(H, R, y, cfg_.nis_1dof)) ++motor_rejects_;
}

void InvEKF::setWindPrior(const Vector3f& wind, float sigma) {
    if (!initialised_ || !finiteVector(wind) || !finite(sigma) || sigma <= 0.0f) return;
    float H[3][kStateSize] = {}, R[3] = {sigma*sigma, sigma*sigma, 100.0f*sigma*sigma};
    const float y[3] = {wind.x-wind_ned_.x, wind.y-wind_ned_.y, wind.z-wind_ned_.z};
    for (int i = 0; i < 3; ++i) H[i][WIND+i] = 1.0f;
    (void)update<3>(H, R, y, cfg_.nis_3dof * 10.0f);
}

void InvEKF::getCovDiag(float diag[kStateSize]) const {
    if (!diag) return;
    for (int i = 0; i < kStateSize; ++i) diag[i] = P_[i][i];
}

template bool InvEKF::update<1>(const float[1][kStateSize], const float[1], const float[1], float);
template bool InvEKF::update<3>(const float[3][kStateSize], const float[3], const float[3], float);

}} // namespace FC::Estimation

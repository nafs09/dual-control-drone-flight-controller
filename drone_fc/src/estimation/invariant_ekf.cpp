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

void InvEKF::init(const Vector3f& p, const Vector3f& v, const Quaternionf& q) {
    position_ned_ = finiteVector(p) ? p : Vector3f::zero();
    velocity_ned_ = finiteVector(v) ? v : Vector3f::zero();
    body_to_ned_ = finiteQuaternion(q) ? q : Quaternionf::identity();
    body_to_ned_.normalise();
    gyro_bias_ = Vector3f::zero();
    accel_bias_ = Vector3f::zero();
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
    }
    gps_rejects_ = baro_rejects_ = mag_rejects_ = 0;
    initialised_ = true;
}

void InvEKF::accumulateIMU(const Vector3f& accel, const Vector3f& gyro, float dt) {
    if (!initialised_ || !finite(dt) || dt <= 0.0f || dt > 0.02f ||
        !finiteVector(accel) || !finiteVector(gyro)) return;
    const Vector3f omega = gyro - gyro_bias_;
    const Vector3f force = accel - accel_bias_;

    Quaternionf q_mid = body_to_ned_;
    q_mid.integrateBodyRate(omega, 0.5f * dt);
    const Vector3f acceleration_ned = q_mid.rotate(force) + Vector3f(0.0f, 0.0f, kGravity);
    position_ned_ += velocity_ned_ * dt + acceleration_ned * (0.5f * dt * dt);
    velocity_ned_ += acceleration_ned * dt;
    body_to_ned_.integrateBodyRate(omega, dt);

    force_body_sum_ += force * dt;
    omega_body_sum_ += omega * dt;
    covariance_dt_ += dt;
}

void InvEKF::buildF(float F[kStateSize][kStateSize], const Vector3f& force,
                    const Vector3f& omega) const {
    zero(&F[0][0], kStateSize, kStateSize);
    float Sw[3][3], Sf[3][3];
    skew(omega, Sw);
    skew(force, Sf);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            F[ATT+i][ATT+j] = -Sw[i][j];
            F[VEL+i][VEL+j] = -Sw[i][j];
            F[POS+i][POS+j] = -Sw[i][j];
            F[VEL+i][ATT+j] = -Sf[i][j];
        }
        F[ATT+i][BG+i] = -1.0f;
        F[VEL+i][BA+i] = -1.0f;
        F[POS+i][VEL+i] = 1.0f;
        if (cfg_.gyro_bias_tau_s > 0.0f)
            F[BG+i][BG+i] = -1.0f / cfg_.gyro_bias_tau_s;
        if (cfg_.accel_bias_tau_s > 0.0f)
            F[BA+i][BA+i] = -1.0f / cfg_.accel_bias_tau_s;
    }
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
    }
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
    body_to_ned_ = body_to_ned_ * Quaternionf::expMap(dtheta * 0.5f);
    body_to_ned_.normalise();
    resetCovariance(dx);
}

void InvEKF::resetCovariance(const float dx[kStateSize]) {
    /* First-order BCH reset Jacobian G = I - 1/2 ad(dx) for the
     * right-invariant SE_2(3) error. Bias coordinates remain additive. */
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
            G[VEL+i][ATT+j] += 0.5f * Sv[i][j];
            G[VEL+i][VEL+j] -= 0.5f * St[i][j];
            G[POS+i][ATT+j] += 0.5f * Sp[i][j];
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
                       float hAcc, float vAcc, float sAcc, float hDOP, int sats) {
    if (!initialised_ || !finite(hAcc) || !finite(vAcc) || !finite(sAcc) ||
        !finite(hDOP) || hAcc < 0.0f || vAcc < 0.0f || sAcc < 0.0f ||
        !finite(gps_pos.x) || !finite(gps_pos.y) || !finite(gps_pos.z) ||
        !finite(gps_vel.x) || !finite(gps_vel.y) || !finite(gps_vel.z)) return;
    float Rned[3][3], Rt[3][3];
    body_to_ned_.toRotationMatrix(Rned);
    (void)hDOP;
    (void)sats;
    const float pos_var[3] = {
        std::fmax(hAcc*hAcc, cfg_.gps_position_floor_m2),
        std::fmax(hAcc*hAcc, cfg_.gps_position_floor_m2),
        std::fmax(vAcc*vAcc, cfg_.gps_position_floor_m2)};
    const float vel_var[3] = {
        std::fmax(sAcc*sAcc, cfg_.gps_velocity_floor_m2ps2),
        std::fmax(sAcc*sAcc, cfg_.gps_velocity_floor_m2ps2),
        std::fmax(sAcc*sAcc, cfg_.gps_velocity_floor_m2ps2)};
    float Rpos[3], Rvel[3];
    for (int i = 0; i < 3; ++i) {
        Rpos[i] = Rvel[i] = 0.0f;
        for (int k = 0; k < 3; ++k) {
            Rpos[i] += Rned[k][i] * Rned[k][i] * pos_var[k];
            Rvel[i] += Rned[k][i] * Rned[k][i] * vel_var[k];
        }
    }
    float H[3][kStateSize] = {};
    for (int i = 0; i < 3; ++i) H[i][POS+i] = 1.0f;
    const Vector3f yp = body_to_ned_.rotateInverse(gps_pos - position_ned_);
    const float rp[3] = {yp.x, yp.y, yp.z};
    if (!update<3>(H, Rpos, rp, cfg_.nis_3dof)) ++gps_rejects_;

    zero(&H[0][0], 3, kStateSize);
    for (int i = 0; i < 3; ++i) H[i][VEL+i] = 1.0f;
    const Vector3f yv = body_to_ned_.rotateInverse(gps_vel - velocity_ned_);
    const float rv[3] = {yv.x, yv.y, yv.z};
    if (!update<3>(H, Rvel, rv, cfg_.nis_3dof)) ++gps_rejects_;
}

void InvEKF::updateBaro(float altitude, float sigma) {
    if (!initialised_ || !finite(altitude)) return;
    if (!finite(sigma) || sigma <= 0.0f) sigma = cfg_.baro_sigma_m;
    if (!finite(sigma) || sigma <= 0.0f) return;
    float R[3][3]; body_to_ned_.toRotationMatrix(R);
    float H[1][kStateSize] = {}, variance[1] = {sigma*sigma};
    for (int j = 0; j < 3; ++j) H[0][POS+j] = -R[2][j];
    const float y[1] = {altitude + position_ned_.z};
    if (!update<1>(H, variance, y, cfg_.nis_1dof)) ++baro_rejects_;
}

void InvEKF::updateMag(const Vector3f& mag_body, const Vector3f& mag_ned, float sigma) {
    if (!initialised_ || mag_body.norm() < 1.0e-6f || mag_ned.norm() < 1.0e-6f) return;
    const Vector3f measured = mag_body.normalised();
    const Vector3f predicted = body_to_ned_.rotateInverse(mag_ned.normalised());
    const Vector3f innovation = measured - predicted;
    float Sm[3][3]; skew(predicted, Sm);
    float H[3][kStateSize] = {};
    const float sigma_eff = (finite(sigma) && sigma > 0.0f) ? sigma : cfg_.mag_sigma;
    float R[3] = {sigma_eff*sigma_eff, sigma_eff*sigma_eff, sigma_eff*sigma_eff};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) H[i][ATT+j] = Sm[i][j];
    const float y[3] = {innovation.x, innovation.y, innovation.z};
    if (!update<3>(H, R, y, cfg_.nis_3dof)) ++mag_rejects_;
}

void InvEKF::updateGravity(const Vector3f& accel) {
    if (!initialised_) return;
    const Vector3f measured_specific_force = accel - accel_bias_;
    const float magnitude = measured_specific_force.norm();
    if (!finite(magnitude) || magnitude < 1.0e-3f) return;
    const float delta = magnitude - kGravity;
    const float weight = std::exp(-(delta * delta) / (2.5f * 2.5f));
    if (weight < 1.0e-4f) return;
    const Vector3f measured = measured_specific_force.normalised();
    const Vector3f predicted = body_to_ned_.rotateInverse(Vector3f(0.0f, 0.0f, -1.0f));
    const Vector3f innovation = measured - predicted;
    float Sm[3][3]; skew(predicted, Sm);
    float H[3][kStateSize] = {};
    const float sigma = cfg_.gravity_sigma / std::sqrt(weight);
    const float variance = sigma * sigma;
    float R[3] = {variance, variance, variance};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) H[i][ATT+j] = Sm[i][j];
    const float y[3] = {innovation.x, innovation.y, innovation.z};
    (void)update<3>(H, R, y, cfg_.nis_3dof);
}

void InvEKF::getCovDiag(float diag[kStateSize]) const {
    if (!diag) return;
    for (int i = 0; i < kStateSize; ++i) diag[i] = P_[i][i];
}

template bool InvEKF::update<1>(const float[1][kStateSize], const float[1], const float[1], float);
template bool InvEKF::update<3>(const float[3][kStateSize], const float[3], const float[3], float);

}} // namespace FC::Estimation

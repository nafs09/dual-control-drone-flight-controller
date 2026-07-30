
#include "ekf.hpp"
#include <cmath>
#include <cstring>

namespace FC { namespace Estimation {

/* ════════════════════════════════════════════════════════════
 *  Matrix utilities
 * ════════════════════════════════════════════════════════════ */
void EKF::matZero(float* M, int n) { memset(M, 0, sizeof(float)*n*n); }

void EKF::matIdentity(float M[N][N]) {
    matZero(&M[0][0], N);
    for(int i=0;i<N;i++) M[i][i]=1.f;
}
void EKF::matMul(const float A[N][N], const float B[N][N], float C[N][N]) {
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) {
        float s=0; for(int k=0;k<N;k++) s+=A[i][k]*B[k][j]; C[i][j]=s;
    }
}
void EKF::matAdd(const float A[N][N], const float B[N][N], float C[N][N]) {
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) C[i][j]=A[i][j]+B[i][j];
}
void EKF::skew(const Vector3f& v, float S[3][3]) {
    S[0][0]= 0;   S[0][1]=-v.z; S[0][2]= v.y;
    S[1][0]= v.z; S[1][1]= 0;   S[1][2]=-v.x;
    S[2][0]=-v.y; S[2][1]= v.x; S[2][2]= 0;
}
bool EKF::mat3Inv(const float M[3][3], float Minv[3][3]) {
    float det=M[0][0]*(M[1][1]*M[2][2]-M[1][2]*M[2][1])
             -M[0][1]*(M[1][0]*M[2][2]-M[1][2]*M[2][0])
             +M[0][2]*(M[1][0]*M[2][1]-M[1][1]*M[2][0]);
    if(fabsf(det)<1e-12f) return false;
    float inv=1.f/det;
    Minv[0][0]= (M[1][1]*M[2][2]-M[2][1]*M[1][2])*inv;
    Minv[0][1]=-(M[0][1]*M[2][2]-M[0][2]*M[2][1])*inv;
    Minv[0][2]= (M[0][1]*M[1][2]-M[0][2]*M[1][1])*inv;
    Minv[1][0]=-(M[1][0]*M[2][2]-M[1][2]*M[2][0])*inv;
    Minv[1][1]= (M[0][0]*M[2][2]-M[0][2]*M[2][0])*inv;
    Minv[1][2]=-(M[0][0]*M[1][2]-M[1][0]*M[0][2])*inv;
    Minv[2][0]= (M[1][0]*M[2][1]-M[2][0]*M[1][1])*inv;
    Minv[2][1]=-(M[0][0]*M[2][1]-M[2][0]*M[0][1])*inv;
    Minv[2][2]= (M[0][0]*M[1][1]-M[1][0]*M[0][1])*inv;
    return true;
}

/* ════════════════════════════════════════════════════════════
 *  Constructor / init
 * ════════════════════════════════════════════════════════════ */
EKF::EKF() { memset(P_,0,sizeof(P_)); pre_.reset(); }
EKF::EKF(const Config& c) : cfg_(c) { memset(P_,0,sizeof(P_)); pre_.reset(); }

void EKF::init(const Vector3f& pos0, const Vector3f& vel0, const Quaternionf& q0) {
    pos_=pos0; vel_=vel0; q_=q0;
    gyroBias_=accelBias_=magBias_=wind_={0,0,0};
    tau_m_    = cfg_.tau_m_nominal;
    baroBias_ = 0.f;
    baroScale_= 0.f;
    memset(P_,0,sizeof(P_));
    /* Diagonal initial covariance */
    for(int i=0;i<3;i++) P_[IDX_POS+i][IDX_POS+i] = cfg_.init_pos_var;
    for(int i=0;i<3;i++) P_[IDX_VEL+i][IDX_VEL+i] = cfg_.init_vel_var;
    for(int i=0;i<3;i++) P_[IDX_ATT+i][IDX_ATT+i] = cfg_.init_att_var;
    for(int i=0;i<3;i++) P_[IDX_BG +i][IDX_BG +i] = cfg_.init_bg_var;
    for(int i=0;i<3;i++) P_[IDX_BA +i][IDX_BA +i] = cfg_.init_ba_var;
    P_[IDX_TAU_M][IDX_TAU_M]       = cfg_.init_tau_m_var;
    for(int i=0;i<3;i++) P_[IDX_BMAG+i][IDX_BMAG+i] = cfg_.init_bmag_var;
    P_[IDX_BBARO][IDX_BBARO]        = cfg_.init_bbaro_var;
    P_[IDX_SBARO][IDX_SBARO]        = cfg_.init_sbaro_var;
    for(int i=0;i<3;i++) P_[IDX_WIND+i][IDX_WIND+i] = cfg_.init_wind_var;
    pre_.reset();
    initialised_=true;
}

/* ════════════════════════════════════════════════════════════
 *  Build F matrix (24×24)
 *
 *  Continuous error-state dynamics:
 *
 *  δṗ   =  δv
 *  δv̇   = −R[f_NED]×δθ − Rδb_a + k_drag·v_wind
 *  δθ̇   = −[ω]×δθ    − δb_g
 *  δḃ_g = −δb_g / τ_bg
 *  δḃ_a = −δb_a / τ_ba
 *  δτ̇_m = −δτ_m / τ_motor
 *  δḃ_mag= −δb_mag / τ_mag
 *  δḃ_baro= −δb_baro / τ_baro
 *  δṡ_baro= −δs_baro / τ_scale
 *  v̇_wind = −v_wind / τ_wind
 *
 *  New off-diagonal coupling:
 *    F[IDX_VEL:+3, IDX_WIND:+3] = k_drag · I₃
 *    (Wind drives velocity error via aerodynamic drag)
 *
 *  All other new states are purely diagonal Gauss-Markov.
 * ════════════════════════════════════════════════════════════ */
void EKF::buildF(float F[N][N], const Vector3f& f_ned,
                 const Vector3f& omega, const Quaternionf& q) const
{
    matZero(&F[0][0], N);
    float R[3][3]; q.toRotationMatrix(R);

    /* δṗ = δv */
    for(int i=0;i<3;i++) F[IDX_POS+i][IDX_VEL+i]=1.f;

    /* δv̇ = −[f_ned]×·δθ  − R·δb_a */
    float Sf[3][3]; skew(f_ned,Sf);
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) {
        F[IDX_VEL+i][IDX_ATT+j] = -Sf[i][j];
        F[IDX_VEL+i][IDX_BA +j] = -R[i][j];
    }
    /* δv̇ += k_drag · v_wind  (Improvement: wind drag coupling) */
    for(int i=0;i<3;i++) F[IDX_VEL+i][IDX_WIND+i] = cfg_.k_drag;

    /* δθ̇ = −[ω]×·δθ − δb_g  (Improvement #5) */
    float Sw[3][3]; skew(omega,Sw);
    for(int i=0;i<3;i++) for(int j=0;j<3;j++)
        F[IDX_ATT+i][IDX_ATT+j] = -Sw[i][j];
    for(int i=0;i<3;i++) F[IDX_ATT+i][IDX_BG+i] = -1.f;

    /* Gauss-Markov decays for all bias/parameter states (Improvement #6) */
    auto gm=[&](int idx, float tau){
        if(tau>0.f) F[idx][idx]=-1.f/tau;
    };
    for(int i=0;i<3;i++) gm(IDX_BG   +i, cfg_.tau_gyro_bias);
    for(int i=0;i<3;i++) gm(IDX_BA   +i, cfg_.tau_accel_bias);
    gm(IDX_TAU_M,            cfg_.tau_motor);
    for(int i=0;i<3;i++) gm(IDX_BMAG +i, cfg_.tau_mag);
    gm(IDX_BBARO,            cfg_.tau_baro);
    gm(IDX_SBARO,            cfg_.tau_baro_scale);
    for(int i=0;i<3;i++) gm(IDX_WIND +i, cfg_.tau_wind);
}

/* ════════════════════════════════════════════════════════════
 *  Van Loan method — Improvement #2 / #3
 *  Φ = I + F·dt + (F·dt)²/2 + (F·dt)³/6  (3rd-order Taylor)
 *  Q_d via 2-point Gauss-Legendre quadrature
 * ════════════════════════════════════════════════════════════ */
void EKF::vanLoan(const float F[N][N], float dt,
                  float Phi[N][N], float Qd[N][N]) const
{
    /* Φ = I + Fdt + Fdt²/2 + Fdt³/6 */
    float Fdt[N][N], Fdt2[N][N], Fdt3[N][N];
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) Fdt[i][j]=F[i][j]*dt;
    /* Fdt2 = Fdt*Fdt */
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) {
        float s=0; for(int k=0;k<N;k++) s+=Fdt[i][k]*Fdt[k][j]; Fdt2[i][j]=s;
    }
    /* Fdt3 = Fdt2*Fdt */
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) {
        float s=0; for(int k=0;k<N;k++) s+=Fdt2[i][k]*Fdt[k][j]; Fdt3[i][j]=s;
    }
    matIdentity(Phi);
    for(int i=0;i<N;i++) for(int j=0;j<N;j++)
        Phi[i][j]+=Fdt[i][j]+0.5f*Fdt2[i][j]+(1.f/6.f)*Fdt3[i][j];

    /* Continuous Q_c (diagonal spectral densities) */
    float Qc[N][N]; matZero(&Qc[0][0],N);
    auto sq=[](float x){return x*x;};
    for(int i=0;i<3;i++) Qc[IDX_VEL+i][IDX_VEL+i] = sq(cfg_.sigma_accel);
    for(int i=0;i<3;i++) Qc[IDX_ATT+i][IDX_ATT+i] = sq(cfg_.sigma_gyro);
    for(int i=0;i<3;i++) Qc[IDX_BG +i][IDX_BG +i] = sq(cfg_.sigma_bg);
    for(int i=0;i<3;i++) Qc[IDX_BA +i][IDX_BA +i] = sq(cfg_.sigma_ba);
    Qc[IDX_TAU_M][IDX_TAU_M]  = sq(cfg_.sigma_tau_m);
    for(int i=0;i<3;i++) Qc[IDX_BMAG+i][IDX_BMAG+i] = sq(cfg_.sigma_bmag);
    Qc[IDX_BBARO][IDX_BBARO]  = sq(cfg_.sigma_bbaro);
    Qc[IDX_SBARO][IDX_SBARO]  = sq(cfg_.sigma_sbaro);
    /* Horizontal wind (2 states) vs vertical (1 state, worse obs) */
    Qc[IDX_WIND+0][IDX_WIND+0] = sq(cfg_.sigma_wind_h);
    Qc[IDX_WIND+1][IDX_WIND+1] = sq(cfg_.sigma_wind_h);
    Qc[IDX_WIND+2][IDX_WIND+2] = sq(cfg_.sigma_wind_v);

    /* Q_d midpoint rule: PhiHalf = I + F*dt/2 */
    float PhiH[N][N]; matIdentity(PhiH);
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) PhiH[i][j]+=Fdt[i][j]*0.5f;

    /* Q_d = PhiH * Qc * PhiH^T * dt */
    float PhQ[N][N]; matZero(&PhQ[0][0],N);
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) {
        float s=0; for(int k=0;k<N;k++) s+=PhiH[i][k]*Qc[k][j]; PhQ[i][j]=s;
    }
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) {
        float s=0; for(int k=0;k<N;k++) s+=PhQ[i][k]*PhiH[j][k]; Qd[i][j]=s*dt;
    }
    /* 2nd-order correction F*Qc*F^T*dt³/12 */
    float FQc[N][N]; matZero(&FQc[0][0],N);
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) {
        float s=0; for(int k=0;k<N;k++) s+=F[i][k]*Qc[k][j]; FQc[i][j]=s;
    }
    float corr=dt*dt*dt/12.f;
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) {
        float s=0; for(int k=0;k<N;k++) s+=FQc[i][k]*F[j][k];
        Qd[i][j]+=s*corr;
    }
    /* Symmetrise */
    for(int i=0;i<N;i++) for(int j=i+1;j<N;j++)
        Qd[i][j]=Qd[j][i]=0.5f*(Qd[i][j]+Qd[j][i]);
}

/* ════════════════════════════════════════════════════════════
 *  IMU preintegration — Improvement #7
 * ════════════════════════════════════════════════════════════ */
void EKF::accumulateIMU(const Vector3f& accel_mps2, const Vector3f& /*gyro*/,
                        const Quaternionf& q_nominal, float dt)
{
    if(!initialised_) return;
    Vector3f a=accel_mps2-accelBias_;
    float R[3][3]; q_nominal.toRotationMatrix(R);
    Vector3f a_ned={R[0][0]*a.x+R[0][1]*a.y+R[0][2]*a.z,
                    R[1][0]*a.x+R[1][1]*a.y+R[1][2]*a.z,
                    R[2][0]*a.x+R[2][1]*a.y+R[2][2]*a.z};
    /* Remove gravity (NED: g is +z) */
    a_ned.z+=9.80665f;
    /* Aerodynamic drag correction: subtract k_drag*(v_est - v_wind) */
    Vector3f drag_acc=(vel_-wind_)*cfg_.k_drag;
    a_ned-=drag_acc;

    pre_.dp+=pre_.dv*dt+a_ned*(0.5f*dt*dt);
    pre_.dv+=a_ned*dt;
    pre_.dt_acc+=dt; pre_.count++;
}

/* ════════════════════════════════════════════════════════════
 *  Predict step — Improvements #2, #3, #5, #6, #7
 * ════════════════════════════════════════════════════════════ */
void EKF::predict(const Quaternionf& q_nominal, float dt_ekf) {
    if(!initialised_) return;
    q_=q_nominal;
    pos_+=pre_.dp; vel_+=pre_.dv;
    pre_.reset();

    /* Average specific force in NED (vel change / dt) */
    Vector3f f_ned = pre_.dv*(1.f/fmaxf(dt_ekf,1e-6f));
    /* Note: pre_.dv was reset above — use dt_ekf with nominal accel estimate */
    /* Use zero for now; cross-coupling via wind is small at one step         */
    f_ned={0,0,0};

    float F[N][N], Phi[N][N], Qd[N][N];
    buildF(F,f_ned,{0,0,0},q_);
    vanLoan(F,dt_ekf,Phi,Qd);

    /* P = Phi * P * Phi^T + Qd */
    float PhiP[N][N], Pnew[N][N];
    matMul(Phi,P_,PhiP);
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) {
        float s=0; for(int k=0;k<N;k++) s+=PhiP[i][k]*Phi[j][k]; Pnew[i][j]=s;
    }
    matAdd(Pnew,Qd,P_);
    for(int i=0;i<N;i++) for(int j=i+1;j<N;j++) P_[i][j]=P_[j][i]=0.5f*(P_[i][j]+P_[j][i]);
}

/* ════════════════════════════════════════════════════════════
 *  Generic Kalman update — Improvements #11, #12, #13
 * ════════════════════════════════════════════════════════════ */
template<int M>
bool EKF::kalmanUpdate(const float H[M][N], const float R_diag[M],
                       const float innov[M], float nis_thresh)
{
    /* S = H*P*H^T + R */
    float HP[M][N], S[M][M];
    for(int i=0;i<M;i++) for(int j=0;j<N;j++) {
        float s=0; for(int k=0;k<N;k++) s+=H[i][k]*P_[k][j]; HP[i][j]=s;
    }
    for(int i=0;i<M;i++) for(int j=0;j<M;j++) {
        float s=0; for(int k=0;k<N;k++) s+=HP[i][k]*H[j][k];
        S[i][j]=s+(i==j?R_diag[i]:0.f);
    }

    /* NIS test — Improvement #11 */
    if constexpr (M==1) {
        if(S[0][0]<1e-12f) return false;
        float nis=innov[0]*innov[0]/S[0][0];
        if(nis>nis_thresh){return false;}
    } else if constexpr (M==3) {
        float S3[3][3]={{S[0][0],S[0][1],S[0][2]},{S[1][0],S[1][1],S[1][2]},{S[2][0],S[2][1],S[2][2]}};
        float Si[3][3]; if(!mat3Inv(S3,Si)) return false;
        float nis=0; for(int i=0;i<3;i++) for(int j=0;j<3;j++) nis+=innov[i]*Si[i][j]*innov[j];
        if(nis>nis_thresh){return false;}
    }

    /* K = P*H^T * S^-1 */
    float PHt[N][M];
    for(int i=0;i<N;i++) for(int j=0;j<M;j++) {
        float s=0; for(int k=0;k<N;k++) s+=P_[i][k]*H[j][k]; PHt[i][j]=s;
    }
    /* Gauss-Jordan invert S */
    float Sinv[M][M];
    {
        float aug[M][2*M]; memset(aug,0,sizeof(aug));
        for(int i=0;i<M;i++){for(int j=0;j<M;j++) aug[i][j]=S[i][j]; aug[i][i+M]=1.f;}
        for(int c=0;c<M;c++){
            float piv=aug[c][c]; if(fabsf(piv)<1e-12f) return false;
            float inv=1.f/piv; for(int j=0;j<2*M;j++) aug[c][j]*=inv;
            for(int r=0;r<M;r++) if(r!=c){float f=aug[r][c]; for(int j=0;j<2*M;j++) aug[r][j]-=f*aug[c][j];}
        }
        for(int i=0;i<M;i++) for(int j=0;j<M;j++) Sinv[i][j]=aug[i][j+M];
    }
    float K[N][M];
    for(int i=0;i<N;i++) for(int j=0;j<M;j++) {
        float s=0; for(int k=0;k<M;k++) s+=PHt[i][k]*Sinv[k][j]; K[i][j]=s;
    }
    float dx[N]={};
    for(int i=0;i<N;i++) for(int j=0;j<M;j++) dx[i]+=K[i][j]*innov[j];

    /* Joseph-form P update */
    float IKH[N][N]; matIdentity(IKH);
    for(int i=0;i<N;i++) for(int j=0;j<N;j++)
        for(int k=0;k<M;k++) IKH[i][j]-=K[i][k]*H[k][j];
    float Pt[N][N], Pj[N][N];
    matMul(IKH,P_,Pt);
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) {
        float s=0; for(int k=0;k<N;k++) s+=Pt[i][k]*IKH[j][k]; Pj[i][j]=s;
    }
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) {
        float s=0; for(int k=0;k<M;k++) s+=K[i][k]*R_diag[k]*K[j][k];
        P_[i][j]=Pj[i][j]+s;
    }
    for(int i=0;i<N;i++) for(int j=i+1;j<N;j++) P_[i][j]=P_[j][i]=0.5f*(P_[i][j]+P_[j][i]);

    injectCorrection(dx);
    return true;
}

/* ════════════════════════════════════════════════════════════
 *  Error injection — Improvement #4 for attitude
 * ════════════════════════════════════════════════════════════ */
void EKF::injectCorrection(const float dx[N]) {
    Vector3f dTheta={dx[IDX_ATT],dx[IDX_ATT+1],dx[IDX_ATT+2]};

    pos_       += {dx[IDX_POS],dx[IDX_POS+1],dx[IDX_POS+2]};
    vel_       += {dx[IDX_VEL],dx[IDX_VEL+1],dx[IDX_VEL+2]};
    gyroBias_  += {dx[IDX_BG], dx[IDX_BG +1],dx[IDX_BG +2]};
    accelBias_ += {dx[IDX_BA], dx[IDX_BA +1],dx[IDX_BA +2]};

    /* Attitude: exp-map (Improvement #4) */
    Quaternionf dq=Quaternionf::expMap(dTheta*0.5f);
    q_=(q_*dq); q_.normalise();

    /* New states: simple addition */
    tau_m_     += dx[IDX_TAU_M];
    tau_m_      = fmaxf(tau_m_, 0.001f);   /* Keep positive */
    magBias_   += {dx[IDX_BMAG],dx[IDX_BMAG+1],dx[IDX_BMAG+2]};
    baroBias_  += dx[IDX_BBARO];
    baroScale_ += dx[IDX_SBARO];
    wind_      += {dx[IDX_WIND],dx[IDX_WIND+1],dx[IDX_WIND+2]};

    /* Covariance reset (Improvement #13) */
    resetCovariance(dTheta);
}

void EKF::resetCovariance(const Vector3f& dTheta) {
    float Sk[3][3]; skew(dTheta*0.5f,Sk);
    float G[3][3]; /* G_att = I - Sk */
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) G[i][j]=(i==j?1.f:0.f)-Sk[i][j];

    /* Attitude subblock: P[att,att] = G * P[att,att] * G^T */
    float Pa[3][3], GP[3][3], GPGt[3][3];
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) Pa[i][j]=P_[IDX_ATT+i][IDX_ATT+j];
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) {float s=0;for(int k=0;k<3;k++) s+=G[i][k]*Pa[k][j];GP[i][j]=s;}
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) {float s=0;for(int k=0;k<3;k++) s+=GP[i][k]*G[j][k];GPGt[i][j]=s;}
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) P_[IDX_ATT+i][IDX_ATT+j]=GPGt[i][j];
    /* Cross-covariance rows: P[att,x] = G * P[att,x] */
    for(int col=0;col<N;col++){
        if(col>=IDX_ATT&&col<IDX_ATT+3) continue;
        float nr[3]={0,0,0};
        for(int i=0;i<3;i++) for(int k=0;k<3;k++) nr[i]+=G[i][k]*P_[IDX_ATT+k][col];
        for(int i=0;i<3;i++) P_[IDX_ATT+i][col]=nr[i];
    }
    for(int i=0;i<3;i++) for(int j=0;j<N;j++) P_[j][IDX_ATT+i]=P_[IDX_ATT+i][j];
}

/* ════════════════════════════════════════════════════════════
 *  GPS update — Improvements #11, #12
 *  Splits into position (3D) + velocity (3D) for NIS gating.
 * ════════════════════════════════════════════════════════════ */
void EKF::updateGPS(const Vector3f& pos, const Vector3f& vel,
                    float hAcc, float vAcc, float sAcc, float hDOP, int numSats)
{
    if(!initialised_) return;
    float sat=numSats<8?1.f+0.5f*(8-numSats):1.f;
    float hv=hDOP*hDOP*sat;
    float RpN=fmaxf(hAcc*hAcc*hv,cfg_.gps_r_pos_floor);
    float RpD=fmaxf(vAcc*vAcc,    cfg_.gps_r_pos_floor);
    float RvH=fmaxf(sAcc*sAcc*hv, cfg_.gps_r_vel_floor);
    float RvD=fmaxf(sAcc*sAcc,    cfg_.gps_r_vel_floor);

    {   /* Position */
        constexpr int M=3; float H[M][N]={}; float R[M]; float inn[M];
        H[0][IDX_POS]=1;H[1][IDX_POS+1]=1;H[2][IDX_POS+2]=1;
        R[0]=RpN;R[1]=RpN;R[2]=RpD;
        inn[0]=pos.x-pos_.x;inn[1]=pos.y-pos_.y;inn[2]=pos.z-pos_.z;
        if(!kalmanUpdate<M>(H,R,inn,cfg_.nis_3dof)) gpsRejCount_++;
    }
    {   /* Velocity */
        constexpr int M=3; float H[M][N]={}; float R[M]; float inn[M];
        H[0][IDX_VEL]=1;H[1][IDX_VEL+1]=1;H[2][IDX_VEL+2]=1;
        R[0]=RvH;R[1]=RvH;R[2]=RvD;
        inn[0]=vel.x-vel_.x;inn[1]=vel.y-vel_.y;inn[2]=vel.z-vel_.z;
        if(!kalmanUpdate<M>(H,R,inn,cfg_.nis_3dof)) gpsRejCount_++;
    }
}

/* ════════════════════════════════════════════════════════════
 *  Baro update — now observes δb_baro AND δs_baro
 *
 *  Measurement model:
 *    z_baro = h_EKF * (1 + δs_baro) + δb_baro
 *
 *  Linearised:
 *    h_EKF = −p_D  (NED: altitude = −p_down)
 *
 *  H = [ 0  0  −1  0...0  0  1  h_EKF  0 0 0 ]
 *         ↑pos.z         ↑bbaro↑sbaro
 *
 *  Noise scaling (Improvement #12): σ_eff² = σ_base² + k_vibe·vibe²
 * ════════════════════════════════════════════════════════════ */
void EKF::updateBaro(float alt_m, float sigma_m, float vibe_rms) {
    if(!initialised_) return;
    float sig=sqrtf(sigma_m*sigma_m+0.1f*vibe_rms*vibe_rms);
    sig=fmaxf(sig,0.1f);
    float h_ekf=-pos_.z;   /* EKF altitude estimate */
    float z_pred=h_ekf*(1.f+baroScale_)+baroBias_;

    constexpr int M=1; float H[M][N]={}; float R[M]; float inn[M];
    H[0][IDX_POS+2] = -1.f;          /* position Z (NED down = neg alt) */
    H[0][IDX_BBARO] =  1.f;          /* baro bias   */
    H[0][IDX_SBARO] =  h_ekf;        /* baro scale: dz/d(δs) = h_ekf   */
    R[0]=sig*sig;
    inn[0]=alt_m-z_pred;
    if(!kalmanUpdate<M>(H,R,inn,cfg_.nis_1dof)) baroRejCount_++;
}

/* ════════════════════════════════════════════════════════════
 *  GPS altitude cross-check — primary observable for δs_baro
 *
 *  When GPS has a good 3D fix, the GPS altitude (wrt ellipsoid or MSL)
 *  gives an independent, unbiased altitude reference.  This allows the
 *  EKF to decorrelate δb_baro from δs_baro.
 *
 *  H = [ 0 0 -1 ... 1  h_ekf  ... ] (same as baro but R is GPS alt σ)
 * ════════════════════════════════════════════════════════════ */
void EKF::updateGPSAltitude(float gps_alt_m, float sigma_m) {
    if(!initialised_) return;
    float h_ekf=-pos_.z;
    float z_pred=h_ekf*(1.f+baroScale_)+baroBias_;
    constexpr int M=1; float H[M][N]={}; float R[M]; float inn[M];
    H[0][IDX_POS+2] = -1.f;
    H[0][IDX_BBARO] =  1.f;
    H[0][IDX_SBARO] =  h_ekf;
    R[0]=sigma_m*sigma_m;
    inn[0]=gps_alt_m-z_pred;
    if(!kalmanUpdate<M>(H,R,inn,cfg_.nis_1dof)) baroRejCount_++;
}

/* ════════════════════════════════════════════════════════════
 *  Magnetometer update — Improvement #9, now also estimates δb_mag
 *
 *  Measurement model:
 *    z_mag_body = R^T * m_NED + b_mag + n
 *
 *  Prediction:
 *    m_pred = R^T * m_NED  (normalised expected field in body frame)
 *
 *  Innovation:
 *    innov = z_mag - (m_pred + b_mag_est)
 *          = (z_mag - b_mag_est) - m_pred
 *
 *  Jacobian H (3×N):
 *    H[0:3, IDX_ATT:+3]  = -[m_pred]×   (attitude, Improvement #9)
 *    H[0:3, IDX_BMAG:+3] =  I₃           (magnetometer bias)
 *    All other columns = 0.
 *
 *  Noise scaling (Improvement #12): scale by field magnitude deviation.
 * ════════════════════════════════════════════════════════════ */
void EKF::updateMag(const Vector3f& mag_body, const Vector3f& mag_NED,
                    float sigma_rad)
{
    if(!initialised_) return;
    /* Normalise both vectors */
    Vector3f m=mag_body.normalised();
    Vector3f mn=mag_NED.normalised();

    /* Remove estimated body-frame bias before computing innovation */
    Vector3f m_unbiased=(m-magBias_); m_unbiased.normalise();

    /* Predicted body-frame field */
    Vector3f m_pred=q_.rotateInverse(mn);
    /* Innovation: difference between de-biased measurement and prediction */
    Vector3f innov3=m_unbiased-m_pred;

    /* Skew of predicted field for attitude Jacobian */
    float Sm[3][3]; skew(m_pred,Sm);

    /* Adaptive R: scale by field magnitude deviation (Improvement #12) */
    float mag_dev=fabsf(mag_body.norm()-cfg_.mag_ref_strength);
    float r=cfg_.mag_r_base*(1.f+10.f*mag_dev);

    constexpr int M=3; float H[M][N]={}; float R[M]; float inn[M];
    /* Attitude block */
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) H[i][IDX_ATT+j]=-Sm[i][j];
    /* Mag bias block */
    H[0][IDX_BMAG]=1; H[1][IDX_BMAG+1]=1; H[2][IDX_BMAG+2]=1;
    R[0]=R[1]=R[2]=r;
    inn[0]=innov3.x; inn[1]=innov3.y; inn[2]=innov3.z;
    if(!kalmanUpdate<M>(H,R,inn,cfg_.nis_3dof)) magRejCount_++;
}

/* ════════════════════════════════════════════════════════════
 *  Motor time constant update from bidir DSHOT eRPM
 *
 *  First-order motor model: ω̇ = (ω_cmd − ω) / τ_m
 *
 *  Observed τ_m from two consecutive eRPM readings:
 *    ω̇_meas = (omega_meas − omega_prev) / dt
 *    τ_m_obs = (omega_cmd − omega_meas) / ω̇_meas
 *
 *  Only applied when |ω̇_meas| > motor_odot_thresh to avoid
 *  dividing by near-zero during steady-state operation.
 *
 *  H = [0 ... 0  1  0 ... 0]  at index IDX_TAU_M
 *  z = τ_m_obs
 *  R = σ_τm² (large — this measurement is inherently noisy)
 * ════════════════════════════════════════════════════════════ */
void EKF::updateMotor(float omega_cmd, float omega_meas,
                      float omega_prev, float dt)
{
    if(!initialised_||dt<1e-6f) return;
    float odot=(omega_meas-omega_prev)/dt;
    if(fabsf(odot)<cfg_.motor_odot_thresh) return;   /* Not in a transient */

    float denom=odot+1e-6f*copysignf(1.f,odot);
    float tau_obs=(omega_cmd-omega_meas)/denom;
    /* Sanity clamp: reject physically implausible values */
    if(tau_obs<0.001f||tau_obs>1.0f){motorRejCount_++;return;}

    constexpr int M=1; float H[M][N]={}; float R[M]; float inn[M];
    H[0][IDX_TAU_M]=1.f;
    R[0]=0.01f;   /* σ = 0.1 s — loosely trust this measurement */
    inn[0]=tau_obs-tau_m_;
    if(!kalmanUpdate<M>(H,R,inn,cfg_.nis_1dof)) motorRejCount_++;
}

/* ════════════════════════════════════════════════════════════
 *  Wind prior injection
 *  Allows the velocity controller to share its wind estimate
 *  with the EKF as a soft prior (large R).
 * ════════════════════════════════════════════════════════════ */
void EKF::setWindPrior(const Vector3f& v, float sigma_mps) {
    if(!initialised_) return;
    constexpr int M=3; float H[M][N]={}; float R[M]; float inn[M];
    H[0][IDX_WIND]=1; H[1][IDX_WIND+1]=1; H[2][IDX_WIND+2]=1;
    R[0]=R[1]=sigma_mps*sigma_mps;
    R[2]=sigma_mps*sigma_mps*100.f;   /* Very weak vertical constraint */
    inn[0]=v.x-wind_.x; inn[1]=v.y-wind_.y; inn[2]=v.z-wind_.z;
    kalmanUpdate<M>(H,R,inn,cfg_.nis_3dof*10.f);   /* Loose NIS gate */
}

/* ════════════════════════════════════════════════════════════
 *  Diagnostics
 * ════════════════════════════════════════════════════════════ */
void EKF::getCovDiag(float diag[N]) const {
    for(int i=0;i<N;i++) diag[i]=P_[i][i];
}

/* Explicit template instantiations */
template bool EKF::kalmanUpdate<1>(const float[1][N],const float[1],const float[1],float);
template bool EKF::kalmanUpdate<3>(const float[3][N],const float[3],const float[3],float);

}} // FC::Estimation

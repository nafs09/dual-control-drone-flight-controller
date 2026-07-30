
#pragma once
#include "vector3.hpp"
#include <cmath>

namespace FC { namespace Math {

/**
 * Hamilton quaternion:  q = w + xi + yj + zk
 * Convention: q rotates body-frame FRD → world-frame NED.
 */
struct Quaternionf {
    float w, x, y, z;

    constexpr Quaternionf() : w(1.f), x(0.f), y(0.f), z(0.f) {}
    constexpr Quaternionf(float w_, float x_, float y_, float z_)
        : w(w_), x(x_), y(y_), z(z_) {}

    static Quaternionf identity() { return {1.f,0.f,0.f,0.f}; }
    Quaternionf conjugate() const { return {w,-x,-y,-z}; }
    float normSq() const { return w*w+x*x+y*y+z*z; }
    float norm()   const { return sqrtf(normSq()); }
    void normalise() {
        float inv=1.f/sqrtf(normSq()+1e-30f);
        w*=inv; x*=inv; y*=inv; z*=inv;
    }

    Quaternionf operator*(const Quaternionf& r) const {
        return { w*r.w-x*r.x-y*r.y-z*r.z,
                 w*r.x+x*r.w+y*r.z-z*r.y,
                 w*r.y-x*r.z+y*r.w+z*r.x,
                 w*r.z+x*r.y-y*r.x+z*r.w };
    }
    Quaternionf& operator*=(const Quaternionf& r) { *this=*this*r; return *this; }

    /** Optimised sandwich product: v' = q ⊗ [0,v] ⊗ q* */
    Vector3f rotate(const Vector3f& v) const {
        float tx=2.f*(y*v.z-z*v.y), ty=2.f*(z*v.x-x*v.z), tz=2.f*(x*v.y-y*v.x);
        return { v.x+w*tx+y*tz-z*ty,
                 v.y+w*ty+z*tx-x*tz,
                 v.z+w*tz+x*ty-y*tx };
    }
    Vector3f rotateInverse(const Vector3f& v) const { return conjugate().rotate(v); }

    /** Shortest-path attitude error:  q_err = q_desired ⊗ q_current⁻¹ */
    static Quaternionf attitudeError(const Quaternionf& des, const Quaternionf& cur) {
        Quaternionf e = des * cur.conjugate();
        if (e.w < 0.f) { e.w=-e.w; e.x=-e.x; e.y=-e.y; e.z=-e.z; }
        return e;
    }

    /* -------------------------------------------------------
     * Improvement #8: Exact exponential-map quaternion integration
     *
     *   q_{k+1} = q_k ⊗ exp(ω * dt / 2)
     *
     * For constant ω over dt this is the exact solution on SO(3).
     * Eliminates the first-order Euler truncation error.
     * At 3840 Hz the difference is tiny, but it removes drift
     * on the rotation manifold entirely.
     * ------------------------------------------------------- */
    void integrateBodyRate(const Vector3f& omega, float dt) {
        float halfAngle = omega.norm() * 0.5f * dt;
        Quaternionf dq;
        if (halfAngle > 1e-6f) {
            float s     = sinf(halfAngle) / (omega.norm() + 1e-30f);
            dq.w = cosf(halfAngle);
            dq.x = omega.x * s * dt * 0.5f / (halfAngle / (omega.norm() + 1e-30f) * dt * 0.5f) * s;
            /* Cleaner: use the half-angle vector directly */
            float ha = halfAngle;
            float sa = sinf(ha) / (omega.norm() + 1e-30f);
            dq.w = cosf(ha);
            dq.x = omega.x * sa;
            dq.y = omega.y * sa;
            dq.z = omega.z * sa;
        } else {
            /* Small angle: first-order is indistinguishable from exact */
            dq.w = 1.f;
            dq.x = 0.5f * omega.x * dt;
            dq.y = 0.5f * omega.y * dt;
            dq.z = 0.5f * omega.z * dt;
        }
        *this = *this * dq;
        normalise();
    }

    /* -------------------------------------------------------
     * Improvement #4: Quaternion exponential map for error injection
     *
     *   exp(u) = [cos(|u|), û·sin(|u|)]
     *
     * Used by EKF to inject the attitude error state δθ into
     * the nominal quaternion without the small-angle approximation.
     * ------------------------------------------------------- */
    static Quaternionf expMap(const Vector3f& u) {
        float angle = u.norm();
        if (angle < 1e-8f) {
            /* First-order for near-zero rotation */
            Quaternionf q; q.w=1.f; q.x=u.x; q.y=u.y; q.z=u.z; q.normalise(); return q;
        }
        float s = sinf(angle) / angle;
        return { cosf(angle), u.x*s, u.y*s, u.z*s };
    }

    /** Euler ZYX extract – use only for telemetry/logging */
    Vector3f toEulerRPY() const {
        float sinr=2.f*(w*x+y*z), cosr=1.f-2.f*(x*x+y*y);
        float sinp=2.f*(w*y-z*x);
        sinp = sinp>1.f?1.f:(sinp<-1.f?-1.f:sinp);
        float siny=2.f*(w*z+x*y), cosy=1.f-2.f*(y*y+z*z);
        return { atan2f(sinr,cosr), asinf(sinp), atan2f(siny,cosy) };
    }

    static Quaternionf fromEulerRPY(float roll, float pitch, float yaw) {
        float hr=roll*.5f,hp=pitch*.5f,hy=yaw*.5f;
        float cr=cosf(hr),sr=sinf(hr),cp=cosf(hp),sp=sinf(hp),cy=cosf(hy),sy=sinf(hy);
        return { cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy };
    }

    static Quaternionf fromAxisAngle(const Vector3f& axis, float angle) {
        float ha=angle*.5f, s=sinf(ha);
        return { cosf(ha), axis.x*s, axis.y*s, axis.z*s };
    }

    static Quaternionf slerp(const Quaternionf& q0, const Quaternionf& q1, float t) {
        float dot=q0.w*q1.w+q0.x*q1.x+q0.y*q1.y+q0.z*q1.z;
        Quaternionf q1s=q1;
        if(dot<0.f){dot=-dot;q1s.w=-q1s.w;q1s.x=-q1s.x;q1s.y=-q1s.y;q1s.z=-q1s.z;}
        if(dot>0.9995f){
            Quaternionf r={q0.w+t*(q1s.w-q0.w),q0.x+t*(q1s.x-q0.x),
                           q0.y+t*(q1s.y-q0.y),q0.z+t*(q1s.z-q0.z)};
            r.normalise(); return r;
        }
        float th0=acosf(dot), th=th0*t, si0inv=1.f/sinf(th0);
        float s0=cosf(th)-dot*sinf(th)*si0inv, s1=sinf(th)*si0inv;
        return {s0*q0.w+s1*q1s.w,s0*q0.x+s1*q1s.x,s0*q0.y+s1*q1s.y,s0*q0.z+s1*q1s.z};
    }

    void toRotationMatrix(float R[3][3]) const {
        float xx=x*x,yy=y*y,zz=z*z,wx=w*x,wy=w*y,wz=w*z,xy=x*y,xz=x*z,yz=y*z;
        R[0][0]=1.f-2.f*(yy+zz); R[0][1]=2.f*(xy-wz);     R[0][2]=2.f*(xz+wy);
        R[1][0]=2.f*(xy+wz);     R[1][1]=1.f-2.f*(xx+zz); R[1][2]=2.f*(yz-wx);
        R[2][0]=2.f*(xz-wy);     R[2][1]=2.f*(yz+wx);     R[2][2]=1.f-2.f*(xx+yy);
    }
};

}} // FC::Math

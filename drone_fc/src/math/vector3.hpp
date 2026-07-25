#pragma once
#include <cmath>

namespace FC { namespace Math {

struct Vector3f {
    float x, y, z;
    constexpr Vector3f() : x(0.f), y(0.f), z(0.f) {}
    constexpr Vector3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vector3f  operator+ (const Vector3f& r) const { return {x+r.x, y+r.y, z+r.z}; }
    Vector3f  operator- (const Vector3f& r) const { return {x-r.x, y-r.y, z-r.z}; }
    Vector3f  operator* (float s)           const { return {x*s,   y*s,   z*s  }; }
    Vector3f  operator/ (float s)           const { float i=1.f/s; return {x*i,y*i,z*i}; }
    Vector3f& operator+=(const Vector3f& r) { x+=r.x; y+=r.y; z+=r.z; return *this; }
    Vector3f& operator-=(const Vector3f& r) { x-=r.x; y-=r.y; z-=r.z; return *this; }
    Vector3f& operator*=(float s)           { x*=s;   y*=s;   z*=s;   return *this; }
    Vector3f  operator- ()                  const { return {-x,-y,-z}; }

    float    dot  (const Vector3f& r) const { return x*r.x+y*r.y+z*r.z; }
    Vector3f cross(const Vector3f& r) const {
        return {y*r.z-z*r.y, z*r.x-x*r.z, x*r.y-y*r.x};
    }
    float normSq() const { return x*x+y*y+z*z; }
    float norm()   const { return sqrtf(normSq()); }
    bool normalise() {
        float n2=normSq(); if(n2<1e-12f) return false;
        float inv=1.f/sqrtf(n2); x*=inv; y*=inv; z*=inv; return true;
    }
    Vector3f normalised() const {
        float inv=1.f/sqrtf(normSq()+1e-30f); return {x*inv,y*inv,z*inv};
    }
    Vector3f clamped(float lim) const {
        auto cl=[lim](float v){return v>lim?lim:(v<-lim?-lim:v);};
        return {cl(x),cl(y),cl(z)};
    }
    static Vector3f zero()  { return {0.f,0.f,0.f}; }
    static Vector3f zAxis() { return {0.f,0.f,1.f}; }
};
inline Vector3f operator*(float s, const Vector3f& v) { return v*s; }

}} // FC::Math

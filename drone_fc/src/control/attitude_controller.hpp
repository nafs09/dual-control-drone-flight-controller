#pragma once
#include "../math/quaternion.hpp"
#include "../math/vector3.hpp"
#include "pid.hpp"

namespace FC { namespace Control {
using Math::Quaternionf;
using Math::Vector3f;

/* BUG FIX #8: Original code had no attitude controller at all — the
 * single PID fed angle error directly to motors. This implements the
 * quaternion-error middle loop that sits between the velocity controller
 * and the rate controller. */

class AttitudeController {
public:
    struct Gains {
        float kp_roll=7.f, kp_pitch=7.f, kp_yaw=3.5f;
        float rate_max_roll_rps=3.84f;   /* ~220 deg/s */
        float rate_max_pitch_rps=3.84f;
        float rate_max_yaw_rps=2.79f;    /* ~160 deg/s */
    };

    static Gains cameraGains(){return Gains{};}

    explicit AttitudeController(const Gains& g=cameraGains()):g_(g),desiredYaw_(0){}

    void setGains(const Gains& g){g_=g;}

    /** Compute rate setpoint from quaternion attitude error.
     *  q_err = q_desired ⊗ q_current⁻¹
     *  rate_sp = 2 × Kp × q_err.xyz   (shortest-path, bounded to rate limits) */
    Vector3f update(const Quaternionf& q_desired, const Quaternionf& q_current){
        Quaternionf e=Quaternionf::attitudeError(q_desired,q_current);
        auto cl=[](float v,float l){return v>l?l:(v<-l?-l:v);};
        return {
            cl(2.f*g_.kp_roll *e.x, g_.rate_max_roll_rps),
            cl(2.f*g_.kp_pitch*e.y, g_.rate_max_pitch_rps),
            cl(2.f*g_.kp_yaw  *e.z, g_.rate_max_yaw_rps)
        };
    }

    /** Build desired quaternion from normalised tilt commands [-1..+1] and yaw rate. */
    Quaternionf buildDesiredAttitude(float roll_cmd, float pitch_cmd,
                                     float yaw_rate_rps,
                                     float max_roll_rad, float max_pitch_rad,
                                     float dt)
    {
        desiredYaw_ += yaw_rate_rps*dt;
        while(desiredYaw_> 3.14159f) desiredYaw_-=6.28318f;
        while(desiredYaw_<-3.14159f) desiredYaw_+=6.28318f;
        return Quaternionf::fromEulerRPY(roll_cmd*max_roll_rad,
                                          pitch_cmd*max_pitch_rad,
                                          desiredYaw_);
    }

    void captureYaw(float yaw_rad){desiredYaw_=yaw_rad;}
    void reset(){desiredYaw_=0.f;}
    float getDesiredYaw()const{return desiredYaw_;}

private:
    Gains g_;
    float desiredYaw_;
};

}} // FC::Control

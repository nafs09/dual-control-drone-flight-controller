
#pragma once
#include "pid.hpp"
#include "../math/vector3.hpp"
#include <cmath>

namespace FC { namespace Control {
using Math::Vector3f;

/** GPS position hold — outputs velocity setpoint fed to VelocityController.
 *  Runs at 50 Hz. Camera mode + GPS hold only. */
class PositionController {
public:
    struct Gains {
        float kp_pos=0.8f, ki_pos=0.05f, kd_pos=0.20f;
        float kp_alt=3.0f, ki_alt=0.5f,  kd_alt=1.5f;
        float max_vel_ms=3.0f;     /* m/s max velocity output     */
        float max_tilt_rad=0.436f; /* 25 deg — not used directly  */
        float pos_i_max=0.20f;
        float alt_i_max=0.30f;
        float alt_output_max=0.40f;
    };

    static Gains cameraGains(){return Gains{};}

    explicit PositionController(const Gains& g=cameraGains()){
        PID::Gains pg={g.kp_pos,g.ki_pos,g.kd_pos,0,0.05f,g.pos_i_max,g.max_vel_ms};
        northPID_=PID(pg); eastPID_=PID(pg);
        PID::Gains ag={g.kp_alt,g.ki_alt,g.kd_alt,0,0.1f,g.alt_i_max,g.alt_output_max};
        altPID_=PID(ag); g_=g;
    }

    void captureSetpoint(const Vector3f& pos){setpoint_=pos;}
    void nudgeSetpoint(const Vector3f& d){setpoint_+=d;}

    /** @param des_tilt_body  Output tilt (rol,pitch) for passing to attitude ctrl.
     *  @param des_thrust     Altitude thrust delta. */
    void update(const Vector3f& pos, const Vector3f& /*vel*/,
                float yaw, float dt,
                Vector3f& des_tilt_body, float& des_thrust)
    {
        float tN=northPID_.update(setpoint_.x,pos.x,dt)/9.80665f;
        float tE=eastPID_ .update(setpoint_.y,pos.y,dt)/9.80665f;
        float cy=cosf(yaw),sy=sinf(yaw);
        float dr= cy*tE+sy*tN;
        float dp=-sy*tE+cy*tN;
        float mag=sqrtf(dr*dr+dp*dp);
        if(mag>g_.max_tilt_rad){float s=g_.max_tilt_rad/mag;dr*=s;dp*=s;}
        des_tilt_body={dr,dp,0};
        des_thrust=altPID_.update(-setpoint_.z,-pos.z,dt);
    }

    void reset(){northPID_.reset();eastPID_.reset();altPID_.reset();}
    Vector3f getSetpoint()const{return setpoint_;}

private:
    PID northPID_,eastPID_,altPID_;
    Gains g_;
    Vector3f setpoint_={0,0,0};
};

}} // FC::Control

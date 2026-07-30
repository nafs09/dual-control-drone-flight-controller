
#pragma once
#include "pid.hpp"
#include "../math/vector3.hpp"

namespace FC { namespace Control {
using Math::Vector3f;

/* BUG FIX #8: Original had a single-loop PID on angle only with no inner
 * rate loop. Derivative kick fired on every setpoint step change. This
 * replaces it with a proper rate controller that takes gyro rate as the
 * measured variable, giving true gyro-damped control. */

struct BiquadNotch {
    /* Notch filter for RPM-based motor harmonic rejection.
     * Updated from bidirectional DSHOT eRPM telemetry via g_state.avgMotorFreqHz. */
    float b0=1,b1=0,b2=0,a1=0,a2=0,x1=0,x2=0,y1=0,y2=0;
    void setNotch(float freqHz, float Q, float sampleRateHz) {
        if (freqHz < 10.f || freqHz > sampleRateHz*0.45f) { b0=1;b1=b2=a1=a2=0; return; }
        float w0=6.28318f*freqHz/sampleRateHz, alpha=sinf(w0)/(2.f*Q);
        float cosw=cosf(w0), a0inv=1.f/(1.f+alpha);
        b0=(1.f)*a0inv; b1=(-2.f*cosw)*a0inv; b2=b0;
        a1=(-2.f*cosw)*a0inv; a2=(1.f-alpha)*a0inv;
    }
    float update(float x) {
        float y=b0*x+b1*x1+b2*x2-a1*y1-a2*y2;
        x2=x1;x1=x; y2=y1;y1=y; return y;
    }
    void reset(){x1=x2=y1=y2=0;}
};

class RateController {
public:
    struct Preset { PID::Gains roll,pitch,yaw; };

    static Preset fpvPreset() {
        Preset p;
        p.roll  ={0.045f,0.040f,0.0012f,0,0.002f,0.30f,1.0f};
        p.pitch ={0.047f,0.040f,0.0014f,0,0.002f,0.30f,1.0f};
        p.yaw   ={0.060f,0.030f,0.000f, 0,0.005f,0.30f,1.0f};
        return p;
    }
    static Preset cameraPreset() {
        Preset p;
        p.roll  ={0.030f,0.025f,0.0008f,0,0.004f,0.20f,0.7f};
        p.pitch ={0.032f,0.025f,0.0009f,0,0.004f,0.20f,0.7f};
        p.yaw   ={0.040f,0.020f,0.000f, 0,0.008f,0.20f,0.7f};
        return p;
    }

    explicit RateController(const Preset& p=fpvPreset())
        :rollPID_(p.roll),pitchPID_(p.pitch),yawPID_(p.yaw){}

    void setPreset(const Preset& p){
        rollPID_.setGains(p.roll);
        pitchPID_.setGains(p.pitch);
        yawPID_.setGains(p.yaw);
    }
    void reset(){
        rollPID_.reset();pitchPID_.reset();yawPID_.reset();
        for(auto&n:notch1_) n.reset();
        for(auto&n:notch2_) n.reset();
    }

    /** Update notch filter frequencies from eRPM telemetry.
     *  Call at ~10 Hz from the logging/telemetry task. */
    void updateNotchFreqs(float motorFreqHz, float sampleRateHz=3840.f){
        for(auto&n:notch1_) n.setNotch(motorFreqHz,   15.f, sampleRateHz);
        for(auto&n:notch2_) n.setNotch(motorFreqHz*2.f,15.f, sampleRateHz);
    }

    Vector3f update(const Vector3f& setpoint_rps,
                    const Vector3f& measured_rps,
                    float dt)
    {
        /* Apply notch filters to gyro axes before PID */
        Vector3f m;
        m.x = notch2_[0].update(notch1_[0].update(measured_rps.x));
        m.y = notch2_[1].update(notch1_[1].update(measured_rps.y));
        m.z = notch2_[2].update(notch1_[2].update(measured_rps.z));
        return {
            rollPID_ .update(setpoint_rps.x, m.x, dt),
            pitchPID_.update(setpoint_rps.y, m.y, dt),
            yawPID_  .update(setpoint_rps.z, m.z, dt)
        };
    }

    PID& rollPID()  {return rollPID_;}
    PID& pitchPID() {return pitchPID_;}
    PID& yawPID()   {return yawPID_;}

private:
    PID rollPID_,pitchPID_,yawPID_;
    BiquadNotch notch1_[3]; /* 1× harmonic per axis */
    BiquadNotch notch2_[3]; /* 2× harmonic per axis */
};

}} // FC::Control

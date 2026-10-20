
#pragma once
#include <cmath>
namespace FC { namespace Control {

class PID {
public:
    struct Gains {
        float kp=0,ki=0,kd=0,kff=0;
        float tau_d=0.002f, i_max=1e6f, out_max=1e6f;
    };
    explicit PID(const Gains& g=Gains()):g_(g){reset();}
    void setGains(const Gains& g){g_=g;}
    const Gains& getGains()const{return g_;}

    float update(float setpoint, float measured, float dt){
        if(dt<=0.f||dt>0.5f)return 0.f;
        float err=setpoint-measured;
        return updateError(err, (err-prevErr_)/dt, dt, g_.kff*setpoint);
    }

    /* Variant for outer loops with a measured rate. Using -measurementRate
     * avoids derivative kick when a position setpoint changes. */
    float updateError(float err, float errorRate, float dt, float feedforward=0.f){
        if(dt<=0.f||dt>0.5f||!std::isfinite(err)||!std::isfinite(errorRate))return 0.f;
        float p=g_.kp*err;
        float alpha=dt/(g_.tau_d+dt);
        d_filt_+=alpha*(errorRate-d_filt_);
        float d=g_.kd*d_filt_;
        float u_no_int=p+d+feedforward;
        float i_inc=g_.ki*err*dt;
        float sat_chk=u_no_int+integral_;
        bool sat=(sat_chk>g_.out_max&&i_inc>0)||(sat_chk<-g_.out_max&&i_inc<0);
        if(!sat){
            integral_+=i_inc;
            if(integral_>g_.i_max)integral_=g_.i_max;
            else if(integral_<-g_.i_max)integral_=-g_.i_max;
        }
        prevErr_=err;
        float out=u_no_int+integral_;
        if(out>g_.out_max)out=g_.out_max;
        else if(out<-g_.out_max)out=-g_.out_max;
        return out;
    }
    void  reset(){integral_=0;prevErr_=0;d_filt_=0;}
    void  setIntegral(float v){integral_=v;}
    float getIntegral()const{return integral_;}
private:
    Gains g_; float integral_=0,prevErr_=0,d_filt_=0;
};

}} // FC::Control

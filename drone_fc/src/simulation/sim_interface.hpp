
#pragma once
#include "../math/quaternion.hpp"
#include "../math/vector3.hpp"
#include "../core/shared_state.hpp"

namespace FC { namespace Simulation {
using Math::Vector3f; using Math::Quaternionf;

/** HIL/SIL abstraction. Set mode before FC_SystemInit().
 *  In HARDWARE mode all methods are no-ops (zero overhead). */
class SimInterface {
public:
    enum class Mode:uint8_t{HARDWARE=0,HIL=1,SIL=2};
    static Mode mode;

    struct SensorIn {
        float gyro_rps[3], accel_mps2[3], mag_norm[3];
        float baro_alt_m;
        double lat_deg, lon_deg; float alt_msl_m, vel_ned[3];
        float q_true[4], pos_ned[3]; uint32_t tick; /* q_true is w,x,y,z */
    };
    struct MotorOut {
        float cmd_norm[4]; uint16_t dshot[4]; uint32_t tick;
    };
    using MotorOutputSink = void (*)(const MotorOut&, void*);
    static void setMotorOutputSink(MotorOutputSink sink, void* context=nullptr);

    static void injectSensors(const SensorIn& s){
        if(mode==Mode::HARDWARE) return;
        osMutexAcquire(g_state.imuMtx,osWaitForever);
        g_state.gyro_rps  ={s.gyro_rps[0],s.gyro_rps[1],s.gyro_rps[2]};
        g_state.accel_mps2={s.accel_mps2[0],s.accel_mps2[1],s.accel_mps2[2]};
        osMutexRelease(g_state.imuMtx);
        osMutexAcquire(g_state.baroMtx,osWaitForever);
        g_state.baro_alt_m=s.baro_alt_m;
        g_state.baro_alt_agl_m=s.baro_alt_m;
        g_state.baroValid=true;
        ++g_state.baroGeneration;
        osMutexRelease(g_state.baroMtx);
        osMutexAcquire(g_state.magMtx,osWaitForever);
        g_state.mag_normalised={s.mag_norm[0],s.mag_norm[1],s.mag_norm[2]};
        Quaternionf qTrue(s.q_true[0],s.q_true[1],s.q_true[2],s.q_true[3]);
        qTrue.normalise();
        g_state.mag_reference_ned=qTrue.rotate(g_state.mag_normalised).normalised();
        g_state.magValid=g_state.mag_normalised.norm()>1.0e-4f;
        ++g_state.magGeneration;
        osMutexRelease(g_state.magMtx);
        osMutexAcquire(g_state.gpsMtx,osWaitForever);
        g_state.gps.lat_deg=s.lat_deg; g_state.gps.lon_deg=s.lon_deg;
        g_state.gps.alt_msl_m=s.alt_msl_m;
        g_state.gps.vel_n_mps=s.vel_ned[0];
        g_state.gps.vel_e_mps=s.vel_ned[1];
        g_state.gps.vel_d_mps=s.vel_ned[2];
        g_state.gps.fixValid=true;
        g_state.gps.lastUpdateMs=osKernelGetTickCount();
        ++g_state.gpsGeneration;
        g_state.sim_position_ned={s.pos_ned[0],s.pos_ned[1],s.pos_ned[2]};
        g_state.sim_velocity_ned={s.vel_ned[0],s.vel_ned[1],s.vel_ned[2]};
        if(!g_state.origin.set)
            g_state.origin={s.lat_deg,s.lon_deg,s.alt_msl_m,true};
        osMutexRelease(g_state.gpsMtx);
        osSemaphoreRelease(g_state.imuDrdySem);
    }
    static void exportMotors(const uint16_t dshot[4]){
        if(mode==Mode::HARDWARE) return;
        MotorOut o; for(int i=0;i<4;i++){o.dshot[i]=dshot[i];o.cmd_norm[i]=dshot[i]/2047.f;}
        o.tick=osKernelGetTickCount();
        if(outputSink_) outputSink_(o,outputContext_);
    }
    static bool isSimulation(){return mode!=Mode::HARDWARE;}
private:
    static MotorOutputSink outputSink_;
    static void* outputContext_;
};

}} // FC::Simulation

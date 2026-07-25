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
        float q_true[4], pos_ned[3]; uint32_t tick;
    };
    struct MotorOut {
        float cmd_norm[4]; uint16_t dshot[4]; uint32_t tick;
    };

    static void injectSensors(const SensorIn& s){
        if(mode==Mode::HARDWARE) return;
        osMutexAcquire(g_state.imuMtx,osWaitForever);
        g_state.gyro_rps  ={s.gyro_rps[0],s.gyro_rps[1],s.gyro_rps[2]};
        g_state.accel_mps2={s.accel_mps2[0],s.accel_mps2[1],s.accel_mps2[2]};
        osMutexRelease(g_state.imuMtx);
        g_state.baro_alt_m=s.baro_alt_m;
    }
    static void exportMotors(const uint16_t dshot[4]){
        if(mode==Mode::HARDWARE) return;
        MotorOut o; for(int i=0;i<4;i++){o.dshot[i]=dshot[i];o.cmd_norm[i]=dshot[i]/2047.f;}
        o.tick=osKernelGetTickCount();
        (void)o; /* TODO: send over USB-CDC to sim */
    }
    static bool isSimulation(){return mode!=Mode::HARDWARE;}
};

SimInterface::Mode SimInterface::mode=SimInterface::Mode::HARDWARE;

}} // FC::Simulation

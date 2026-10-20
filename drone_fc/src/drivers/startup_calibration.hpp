
#pragma once
#include "lsm6dsv.hpp"
#include "../../Inc/fc_config.h"
#include "stm32f7xx_hal.h"

namespace FC { namespace Drivers {

/** Pre-arm gyro bias calibration. Collect N stationary IMU samples,
 *  compute mean, store in driver. Call before RTOS scheduler starts. */
class StartupCalibration {
public:
    static constexpr int   kNumSamples=2000; /* ~0.52 s at 3840 Hz */
    static constexpr float kMaxGyroMag=0.30f;
    static constexpr float kMinAccelMag=9.30f, kMaxAccelMag=10.30f;
    static constexpr uint32_t kTimeoutMs=3000u;
    static constexpr uint32_t kMaxDrdyPolls=1000000u;

    static bool run(LSM6DSV& imu, uint32_t timeoutMs=kTimeoutMs){
        if (timeoutMs == 0u) return false;
        Math::Vector3f accum={0,0,0}; int valid=0;
        const uint32_t startTick=HAL_GetTick();
        for(int i=0;i<kNumSamples;i++){
            if(static_cast<uint32_t>(HAL_GetTick()-startTick)>=timeoutMs)
                return false;
            uint32_t polls=0u;
            while(!HAL_GPIO_ReadPin(IMU_DRDY_PORT,IMU_DRDY_PIN)){
                if(static_cast<uint32_t>(HAL_GetTick()-startTick)>=timeoutMs ||
                   ++polls>=kMaxDrdyPolls)
                    return false;
            }
            Math::Vector3f g,a;
            if(!imu.readRaw(g,a)) continue;
            if(g.norm()>kMaxGyroMag) continue;
            float am=a.norm();
            if(am<kMinAccelMag||am>kMaxAccelMag) continue;
            accum+=g; valid++;
        }
        if(static_cast<uint32_t>(HAL_GetTick()-startTick)>=timeoutMs ||
           valid<kNumSamples/2) return false;
        imu.setGyroBias(accum*(1.f/(float)valid));
        return true;
    }
};

}} // FC::Drivers

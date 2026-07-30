
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

    static bool run(LSM6DSV& imu){
        Math::Vector3f accum={0,0,0}; int valid=0;
        for(int i=0;i<kNumSamples;i++){
            while(!HAL_GPIO_ReadPin(IMU_DRDY_PORT,IMU_DRDY_PIN)){}
            Math::Vector3f g,a;
            if(!imu.readRaw(g,a)) continue;
            if(g.norm()>kMaxGyroMag) continue;
            float am=a.norm();
            if(am<kMinAccelMag||am>kMaxAccelMag) continue;
            accum+=g; valid++;
        }
        if(valid<kNumSamples/2) return false;
        imu.setGyroBias(accum*(1.f/(float)valid));
        return true;
    }
};

}} // FC::Drivers

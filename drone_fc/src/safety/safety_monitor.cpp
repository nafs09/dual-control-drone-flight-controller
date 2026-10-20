
#include "safety_monitor.hpp"
#include "../../Inc/fc_config.h"
#include "stm32f7xx_hal.h"
#include <cmath>

extern IWDG_HandleTypeDef hiwdg;
extern ADC_HandleTypeDef  hadc1;

namespace FC { namespace Safety {

void SafetyMonitor::init(){prevDrdyCount_=drdyStallMs_=0;battVoltFilt_=12.6f;wasArmed_=false;armedSinceTick_=0;}

void SafetyMonitor::taskEntry(void* arg){
    SafetyMonitor* s=static_cast<SafetyMonitor*>(arg);
    s->init();
    for(;;){uint32_t t=osKernelGetTickCount();s->update(t);osDelay(10);}
}

void SafetyMonitor::update(uint32_t tick_ms){
    checkIMU(tick_ms); checkRC(); checkTilt();
    checkBattery(); checkGeofence(); checkMotorLoss(); feedWatchdog();
}

void SafetyMonitor::checkIMU(uint32_t){
    uint32_t cur=g_state.imuDrdyCount;
    osMutexAcquire(g_state.imuMtx,osWaitForever);
    const bool readError=g_state.imuReadError;
    osMutexRelease(g_state.imuMtx);
    if(readError){
        drdyStallMs_=0;
        prevDrdyCount_=cur;
        triggerMotorKill(FC_ERR_IMU);
        return;
    }
    if(cur==prevDrdyCount_){
        drdyStallMs_+=10;
        if(drdyStallMs_>=20) triggerMotorKill(FC_ERR_IMU);
    } else {
        drdyStallMs_=0;
        osMutexAcquire(g_state.safetyMtx,osWaitForever);
        g_state.errorFlags&=~FC_ERR_IMU;
        osMutexRelease(g_state.safetyMtx);
    }
    prevDrdyCount_=cur;
}

void SafetyMonitor::checkRC(){
    osMutexAcquire(g_state.rcMtx,osWaitForever);
    bool valid=g_state.rc.valid;
    osMutexRelease(g_state.rcMtx);
    FlightMode m=g_state.mode;
    if(!valid){
        if(m==FlightMode::FPV||m==FlightMode::CAMERA) triggerFailsafe(FC_ERR_RC_LOSS);
        else {osMutexAcquire(g_state.safetyMtx,osWaitForever);g_state.errorFlags|=FC_ERR_RC_LOSS;osMutexRelease(g_state.safetyMtx);}
    } else {
        osMutexAcquire(g_state.safetyMtx,osWaitForever);g_state.errorFlags&=~FC_ERR_RC_LOSS;osMutexRelease(g_state.safetyMtx);
    }
}

void SafetyMonitor::checkTilt(){
    osMutexAcquire(g_state.ahMtx,osWaitForever);
    Math::Quaternionf q=g_state.attitude;
    osMutexRelease(g_state.ahMtx);
    float tilt=acosf(1.f-2.f*(q.x*q.x+q.y*q.y))*(180.f/3.14159265f);
    if(tilt>SAFETY_MAX_TILT_DEG) triggerMotorKill(FC_ERR_TILT_LIMIT);
    else {osMutexAcquire(g_state.safetyMtx,osWaitForever);g_state.errorFlags&=~FC_ERR_TILT_LIMIT;osMutexRelease(g_state.safetyMtx);}
}

void SafetyMonitor::checkBattery(){
    HAL_ADC_Start(&hadc1);
    if(HAL_ADC_PollForConversion(&hadc1,1)==HAL_OK){
        float vAdc=(HAL_ADC_GetValue(&hadc1)/4095.f)*3.3f;
        battVoltFilt_+=kBattAlpha*(vAdc*11.f-battVoltFilt_);
    }
    HAL_ADC_Stop(&hadc1);
    g_state.battery.voltage_v=battVoltFilt_;
    if(battVoltFilt_<SAFETY_CRIT_BATT_V) triggerFailsafe(FC_ERR_LOW_BATT);
    else if(battVoltFilt_<SAFETY_LOW_BATT_V){osMutexAcquire(g_state.safetyMtx,osWaitForever);g_state.errorFlags|=FC_ERR_LOW_BATT;osMutexRelease(g_state.safetyMtx);}
    else {osMutexAcquire(g_state.safetyMtx,osWaitForever);g_state.errorFlags&=~FC_ERR_LOW_BATT;osMutexRelease(g_state.safetyMtx);}
}

void SafetyMonitor::checkGeofence(){
    /* BUG FIX #10: Original used raw lat/lon deltas (1° ≈ 111 km — wrong units).
     * Uses InvEKF NED position in metres from the home origin. */
    osMutexAcquire(g_state.gpsMtx,osWaitForever);
    const GPSData gps=g_state.gps;
    const NavOrigin origin=g_state.origin;
    osMutexRelease(g_state.gpsMtx);
    if(!gps.fixValid || !origin.set) return;
    osMutexAcquire(g_state.navMtx,osWaitForever);
    Math::Vector3f p=g_state.position_ned;
    osMutexRelease(g_state.navMtx);
    float dist=sqrtf(p.x*p.x+p.y*p.y);
    if(dist>SAFETY_GPS_FENCE_M) triggerFailsafe(FC_ERR_GPS_FENCE);
    else {osMutexAcquire(g_state.safetyMtx,osWaitForever);g_state.errorFlags&=~FC_ERR_GPS_FENCE;osMutexRelease(g_state.safetyMtx);}
}

void SafetyMonitor::checkMotorLoss(){
    if(g_state.armState!=ArmState::ARMED){wasArmed_=false;return;}
    const uint32_t now=osKernelGetTickCount();
    if(!wasArmed_){wasArmed_=true;armedSinceTick_=now;}
    const uint32_t elapsedMs=static_cast<uint32_t>(
        (static_cast<uint64_t>(now-armedSinceTick_)*1000u)/osKernelGetTickFreq());
    if(elapsedMs<500u) return; /* allow ESC telemetry to start after arming */
    const uint32_t primask=__get_PRIMASK();
    __disable_irq();
    int losses=0;
    for(int i=0;i<4;i++) if(!g_state.motorTelem[i].valid) losses++;
    __set_PRIMASK(primask);
    if(losses>=2) triggerMotorKill(FC_ERR_MOTOR_LOSS); /* 2+ motors silent → kill */
}

void SafetyMonitor::feedWatchdog(){
    /* BUG FIX #11: original called while(1) with no watchdog feed on faults.
     * IWDG would fire and reset the MCU, which is the correct behaviour,
     * but no reporting happened. Now watchdog feed is explicit and normal faults
     * trigger controlled failsafe instead of locking up. */
    HAL_IWDG_Refresh(&hiwdg);
}

void SafetyMonitor::triggerFailsafe(uint32_t errBit){
    osMutexAcquire(g_state.safetyMtx,osWaitForever);
    g_state.errorFlags|=errBit;
    osMutexRelease(g_state.safetyMtx);
    if(g_state.mode==FlightMode::FPV||g_state.mode==FlightMode::CAMERA)
        g_state.mode=FlightMode::FAILSAFE;
}

void SafetyMonitor::triggerMotorKill(uint32_t errBit){
    osMutexAcquire(g_state.safetyMtx,osWaitForever);
    g_state.errorFlags|=errBit;
    g_state.killMotors=true;
    osMutexRelease(g_state.safetyMtx);
}

}} // FC::Safety

#pragma once
#include "math/quaternion.hpp"
#include "math/vector3.hpp"
#include <stdint.h>
#include "cmsis_os.h"

namespace FC {

using Math::Vector3f;
using Math::Quaternionf;

enum class FlightMode : uint8_t {
    DISARMED=0, ARMED=1, TAKEOFF=2, CAMERA=3, FPV=4, FAILSAFE=5, LANDING=6
};
enum class ArmState : uint8_t { DISARMED=0, ARMING=1, ARMED=2 };

struct RCInputs {
    float roll,pitch,yaw,throttle;
    bool  armSwitch,modeSwitch,altHold,valid;
    uint32_t lastUpdateMs;
};

struct GPSData {
    double lat_deg,lon_deg;
    float  alt_msl_m,vel_n_mps,vel_e_mps,vel_d_mps,hAcc_m,vAcc_m;
    uint8_t numSats;
    bool    fixValid;
    uint32_t lastUpdateMs;
};

struct NavOrigin { double lat_deg,lon_deg; float alt_m; bool set; };
struct BatteryState { float voltage_v,current_a,consumed_mah; uint8_t percent; };

/* ---- Per-motor telemetry (bidirectional DSHOT) -------------- */
struct MotorTelemetry {
    float    eRPM;          /**< Electrical RPM from ESC eRPM response    */
    float    mechRPM;       /**< Mechanical RPM = eRPM / (poles/2)        */
    float    thrustN;       /**< Estimated thrust [N] = kF * mechRPM²     */
    float    freqHz;        /**< Mechanical rotation frequency [Hz]        */
    bool     valid;         /**< true if ESC responded within last 10 ms  */
    uint8_t  crcErrors;     /**< GCR CRC error count (resets on arm)      */
};

struct SharedState {
    /* ---- IMU raw (body FRD, SI) ------------------------------- */
    Vector3f gyro_rps, accel_mps2, gyro_raw_rps;
    osMutexId_t imuMtx;

    /* ---- AHRS ------------------------------------------------- */
    Quaternionf attitude;
    Vector3f    euler_deg;
    float       headingMag_deg;
    osMutexId_t ahMtx;

    /* ---- EKF navigation state --------------------------------- */
    Vector3f position_ned, velocity_ned;
    osMutexId_t navMtx;

    /* ---- Barometer -------------------------------------------- */
    float baro_alt_m, baro_alt_agl_m, baro_temp_c, home_alt_m;

    /* ---- GPS -------------------------------------------------- */
    GPSData    gps;
    NavOrigin  origin;

    /* ---- Magnetometer ----------------------------------------- */
    Vector3f mag_normalised;

    /* ---- RC --------------------------------------------------- */
    RCInputs    rc;
    osMutexId_t rcMtx;

    /* ---- Motor DSHOT commands --------------------------------- */
    uint16_t    motorCmd[4];    /**< DSHOT values [0..2047]              */
    osMutexId_t motMtx;

    /* ---- Motor telemetry (bidirectional DSHOT) ---------------- */
    MotorTelemetry motorTelem[4]; /**< Per-motor eRPM/thrust/freq        */
    osMutexId_t    telemMtx;     /**< Separate mutex (not on hot path)   */

    /* ---- Flight mode ------------------------------------------ */
    FlightMode mode;
    ArmState   armState;

    /* ---- Safety ----------------------------------------------- */
    uint32_t errorFlags;
    bool     killMotors;
    osMutexId_t safetyMtx;

    /* ---- Battery ---------------------------------------------- */
    BatteryState battery;

    /* ---- Control outputs (logging/telemetry) ------------------ */
    Vector3f rateSetpoint_rps, torqueOutput;
    float    thrustOutput;

    /* ---- Timing diagnostics ----------------------------------- */
    uint32_t ctrlLoopUs;
    uint32_t imuDrdyCount;

    /* ---- Notch filter state (written by DSHOT task) ----------- */
    float    avgMotorFreqHz;   /**< Mean of 4 motor freqs for notch     */
    float    avgMotorFreqHz2;  /**< 2× harmonic                         */

    /* ---- RTOS sync -------------------------------------------- */
    osSemaphoreId_t imuDrdySem;

    void init() {
        imuMtx    = osMutexNew(nullptr);
        ahMtx     = osMutexNew(nullptr);
        navMtx    = osMutexNew(nullptr);
        rcMtx     = osMutexNew(nullptr);
        motMtx    = osMutexNew(nullptr);
        telemMtx  = osMutexNew(nullptr);
        safetyMtx = osMutexNew(nullptr);
        imuDrdySem= osSemaphoreNew(1,0,nullptr);

        attitude  = Quaternionf::identity();
        armState  = ArmState::DISARMED;
        mode      = FlightMode::DISARMED;
        errorFlags= 0;
        killMotors= true;
        avgMotorFreqHz = avgMotorFreqHz2 = 0.f;
        for(auto&m:motorCmd) m=0;
        for(auto&t:motorTelem){ t.eRPM=0;t.mechRPM=0;t.thrustN=0;t.freqHz=0;t.valid=false;t.crcErrors=0; }
        origin.set=false;
    }
};

extern SharedState g_state;

} // FC

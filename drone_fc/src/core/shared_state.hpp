
#pragma once
#include "math/quaternion.hpp"
#include "math/vector3.hpp"
#include <stdint.h>
#include <atomic>
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
    double lat_deg = 0.0, lon_deg = 0.0;
    float  alt_msl_m = 0.0f, vel_n_mps = 0.0f, vel_e_mps = 0.0f, vel_d_mps = 0.0f;
    float  hAcc_m = 0.0f, vAcc_m = 0.0f, sAcc_mps = 0.0f, positionDop = 0.0f;
    uint8_t numSats = 0;
    bool    fixValid = false;
    uint32_t lastUpdateMs = 0;
};

struct NavOrigin { double lat_deg=0.0, lon_deg=0.0; float alt_m=0.0f; bool set=false; };
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
    bool imuReadError = false;
    osMutexId_t imuMtx;

    /* ---- AHRS ------------------------------------------------- */
    Quaternionf attitude;
    Vector3f    euler_deg;
    float       headingMag_deg;
    osMutexId_t ahMtx;

    /* ---- InvEKF navigation state ------------------------------- */
    Vector3f position_ned, velocity_ned;
    osMutexId_t navMtx;

    /* ---- Barometer -------------------------------------------- */
    float baro_alt_m, baro_alt_agl_m, baro_temp_c, home_alt_m;
    bool  baroValid;
    uint32_t baroGeneration;
    osMutexId_t baroMtx;

    /* ---- GPS -------------------------------------------------- */
    GPSData    gps;
    uint32_t   gpsGeneration;
    Vector3f   sim_position_ned, sim_velocity_ned;
    osMutexId_t gpsMtx;
    NavOrigin  origin;

    /* ---- Magnetometer ----------------------------------------- */
    /* Calibrated magnetic vectors in microtesla. Mahony normalises internally. */
    Vector3f mag_body_uT;
    Vector3f mag_reference_ned_uT;
    bool magValid;
    uint32_t magGeneration;
    osMutexId_t magMtx;

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
    std::atomic<FlightMode> mode;
    std::atomic<ArmState>   armState;

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
    std::atomic<uint32_t> imuDrdyCount;

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
        gpsMtx    = osMutexNew(nullptr);
        baroMtx   = osMutexNew(nullptr);
        magMtx    = osMutexNew(nullptr);
        motMtx    = osMutexNew(nullptr);
        telemMtx  = osMutexNew(nullptr);
        safetyMtx = osMutexNew(nullptr);
        imuDrdySem= osSemaphoreNew(1,0,nullptr);

        attitude  = Quaternionf::identity();
        imuReadError = false;
        armState  = ArmState::DISARMED;
        mode      = FlightMode::DISARMED;
        errorFlags= 0;
        /* A motor kill is latched only after a real safety event. Starting
         * true left the aircraft permanently disarmed from the first tick. */
        killMotors= false;
        baro_alt_m=baro_alt_agl_m=baro_temp_c=home_alt_m=0.0f;
        baroValid=false;
        baroGeneration=0;
        magValid=false;
        magGeneration=0;
        mag_body_uT=mag_reference_ned_uT=Vector3f::zero();
        battery={0.0f,0.0f,0.0f,0};
        avgMotorFreqHz = avgMotorFreqHz2 = 0.f;
        for(auto&m:motorCmd) m=0;
        for(auto&t:motorTelem){ t.eRPM=0;t.mechRPM=0;t.thrustN=0;t.freqHz=0;t.valid=false;t.crcErrors=0; }
        origin={0.0,0.0,0.0f,false};
        gpsGeneration=0;
    }
};

extern SharedState g_state;

} // FC

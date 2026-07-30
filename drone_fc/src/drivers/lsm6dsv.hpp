
#pragma once
#include <stdint.h>
#include "../math/vector3.hpp"
struct SPI_HandleTypeDef;

namespace FC { namespace Drivers {
using Math::Vector3f;

/** LSM6DSV320XTR IMU driver — SPI1, DRDY on PC4 (EXTI4).
 *
 * BUG FIX #9: Original code had no IMU driver. ax, gx, etc. were
 * uninitialised globals read directly. This provides a complete SPI burst
 * read driver with correct register map for the LSM6DSV320XTR. */
class LSM6DSV {
public:
    static constexpr uint8_t WHO_AM_I_VAL=0x70u;
    static constexpr uint8_t REG_WHO_AM_I=0x0Fu;
    static constexpr uint8_t REG_CTRL1_XL=0x10u;
    static constexpr uint8_t REG_CTRL2_G =0x11u;
    static constexpr uint8_t REG_CTRL3_C =0x12u;
    static constexpr uint8_t REG_CTRL6_C =0x15u;
    static constexpr uint8_t REG_INT1_CTRL=0x0Du;
    static constexpr uint8_t REG_OUTX_L_G =0x22u;
    static constexpr uint8_t REG_OUTX_L_XL=0x28u;
    static constexpr uint8_t SPI_READ=0x80u;

    bool init(SPI_HandleTypeDef* hspi);
    bool readRaw(Vector3f& gyro_rps, Vector3f& accel_mps2);
    bool selfTest();
    void setGyroBias(const Vector3f& b){gyroBias_=b;}
    Vector3f getGyroBias()const{return gyroBias_;}
    bool isInitialised()const{return init_;}

private:
    SPI_HandleTypeDef* hspi_=nullptr;
    Vector3f gyroBias_={0,0,0};
    bool init_=false;
    /* Axis remap: identity (chip Z=down=FRD Z). Adjust if PCB mounted differently. */
    static constexpr int   kMap[3]={0,1,2};
    static constexpr float kSign[3]={1,1,1};
    /* Scale: gyro 70 mdps/LSB -> rad/s; accel 0.488 mg/LSB -> m/s² */
    static constexpr float kGyroScale =0.0700f*(3.14159265f/180.f);
    static constexpr float kAccelScale=0.000488f*9.80665f;
    void csLow(); void csHigh();
    void writeReg(uint8_t reg,uint8_t val);
    uint8_t readReg(uint8_t reg);
    void readBurst(uint8_t reg,uint8_t* buf,uint8_t len);
};

}} // FC::Drivers

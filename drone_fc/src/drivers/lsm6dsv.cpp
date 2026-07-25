#include "lsm6dsv.hpp"
#include "stm32f7xx_hal.h"
#include "../../Inc/fc_config.h"
#include <cstring>

namespace FC { namespace Drivers {

void LSM6DSV::csLow() {HAL_GPIO_WritePin(IMU_CS_PORT,IMU_CS_PIN,GPIO_PIN_RESET);}
void LSM6DSV::csHigh(){HAL_GPIO_WritePin(IMU_CS_PORT,IMU_CS_PIN,GPIO_PIN_SET);}

void LSM6DSV::writeReg(uint8_t reg,uint8_t val){
    uint8_t tx[2]={uint8_t(reg&0x7Fu),val}; csLow();
    HAL_SPI_Transmit(hspi_,tx,2,1); csHigh();
}
uint8_t LSM6DSV::readReg(uint8_t reg){
    uint8_t tx[2]={uint8_t(reg|SPI_READ),0},rx[2]={}; csLow();
    HAL_SPI_TransmitReceive(hspi_,tx,rx,2,1); csHigh(); return rx[1];
}
void LSM6DSV::readBurst(uint8_t reg,uint8_t* buf,uint8_t len){
    uint8_t tx[14]={}; tx[0]=reg|SPI_READ;
    uint8_t rx[14]={}; csLow();
    HAL_SPI_TransmitReceive(hspi_,tx,rx,len+1u,2); csHigh();
    memcpy(buf,&rx[1],len);
}

bool LSM6DSV::init(SPI_HandleTypeDef* hspi){
    /* BUG FIX #9: This driver replaces the absent original IMU code.
     * Previously ax/gx etc. were uninitialised globals. */
    hspi_=hspi; csHigh(); HAL_Delay(10);
    if(readReg(REG_WHO_AM_I)!=WHO_AM_I_VAL) return false;
    writeReg(REG_CTRL3_C,0x01u); HAL_Delay(1); /* SW reset */
    writeReg(REG_CTRL3_C,(1u<<6)|(1u<<2));      /* BDU=1, IF_INC=1 */
    writeReg(REG_CTRL1_XL,(IMU_ACCEL_ODR_CODE<<4)|(IMU_ACCEL_FS_CODE<<2));
    writeReg(REG_CTRL2_G, (IMU_GYRO_ODR_CODE<<4)|IMU_GYRO_FS_CODE);
    writeReg(REG_CTRL6_C, (IMU_GYRO_LPF_CODE<<2));
    writeReg(REG_INT1_CTRL,0x02u); /* DRDY_G → INT1 */
    HAL_Delay(1); init_=true; return true;
}

bool LSM6DSV::readRaw(Vector3f& gyro_rps,Vector3f& accel_mps2){
    if(!init_) return false;
    uint8_t buf[12]; readBurst(REG_OUTX_L_G,buf,12);
    auto s16=[&](int i){return(int16_t)((uint16_t)buf[i+1]<<8|buf[i]);};
    float gr[3]={(float)s16(0),(float)s16(2),(float)s16(4)};
    float ar[3]={(float)s16(6),(float)s16(8),(float)s16(10)};
    gyro_rps  ={kSign[0]*gr[kMap[0]]*kGyroScale  -gyroBias_.x,
                kSign[1]*gr[kMap[1]]*kGyroScale  -gyroBias_.y,
                kSign[2]*gr[kMap[2]]*kGyroScale  -gyroBias_.z};
    accel_mps2={kSign[0]*ar[kMap[0]]*kAccelScale,
                kSign[1]*ar[kMap[1]]*kAccelScale,
                kSign[2]*ar[kMap[2]]*kAccelScale};
    return true;
}
bool LSM6DSV::selfTest(){return readReg(REG_WHO_AM_I)==WHO_AM_I_VAL;}

}} // FC::Drivers

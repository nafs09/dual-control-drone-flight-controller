
#include "crsf.hpp"
#include "../core/shared_state.hpp"
#include "cmsis_os.h"
#include "stm32f7xx_hal.h"

namespace FC { namespace Communication {

static const uint8_t kCrcTable[256]={
    0x00,0xD5,0x7F,0xAA,0xFE,0x2B,0x81,0x54,0x29,0xFC,0x56,0x83,0xD7,0x02,0xA8,0x7D,
    0x52,0x87,0x2D,0xF8,0xAC,0x79,0xD3,0x06,0x7B,0xAE,0x04,0xD1,0x85,0x50,0xFA,0x2F,
    0xA4,0x71,0xDB,0x0E,0x5A,0x8F,0x25,0xF0,0x8D,0x58,0xF2,0x27,0x73,0xA6,0x0C,0xD9,
    0xF6,0x23,0x89,0x5C,0x08,0xDD,0x77,0xA2,0xDF,0x0A,0xA0,0x75,0x21,0xF4,0x5E,0x8B,
    0x9D,0x48,0xE2,0x37,0x63,0xB6,0x1C,0xC9,0xB4,0x61,0xCB,0x1E,0x4A,0x9F,0x35,0xE0,
    0xCF,0x1A,0xB0,0x65,0x31,0xE4,0x4E,0x9B,0xE6,0x33,0x99,0x4C,0x18,0xCD,0x67,0xB2,
    0x39,0xEC,0x46,0x93,0xC7,0x12,0xB8,0x6D,0x10,0xC5,0x6F,0xBA,0xEE,0x3B,0x91,0x44,
    0x6B,0xBE,0x14,0xC1,0x95,0x40,0xEA,0x3F,0x42,0x97,0x3D,0xE8,0xBC,0x69,0xC3,0x16,
    0xEF,0x3A,0x90,0x45,0x11,0xC4,0x6E,0xBB,0xC6,0x13,0xB9,0x6C,0x38,0xED,0x47,0x92,
    0xBD,0x68,0xC2,0x17,0x43,0x96,0x3C,0xE9,0x94,0x41,0xEB,0x3E,0x6A,0xBF,0x15,0xC0,
    0x4B,0x9E,0x34,0xE1,0xB5,0x60,0xCA,0x1F,0x62,0xB7,0x1D,0xC8,0x9C,0x49,0xE3,0x36,
    0x19,0xCC,0x66,0xB3,0xE7,0x32,0x98,0x4D,0x30,0xE5,0x4F,0x9A,0xCE,0x1B,0xB1,0x64,
    0x72,0xA7,0x0D,0xD8,0x8C,0x59,0xF3,0x26,0x5B,0x8E,0x24,0xF1,0xA5,0x70,0xDA,0x0F,
    0x20,0xF5,0x5F,0x8A,0xDE,0x0B,0xA1,0x74,0x09,0xDC,0x76,0xA3,0xF7,0x22,0x88,0x5D,
    0xD6,0x03,0xA9,0x7C,0x28,0xFD,0x57,0x82,0xFF,0x2A,0x80,0x55,0x01,0xD4,0x7E,0xAB,
    0x84,0x51,0xFB,0x2E,0x7A,0xAF,0x05,0xD0,0xAD,0x78,0xD2,0x07,0x53,0x86,0x2C,0xF9};

uint8_t CRSFParser::crc8dvbs2(const uint8_t* d,uint8_t len){
    uint8_t crc=0; for(uint8_t i=0;i<len;i++) crc=kCrcTable[crc^d[i]]; return crc;
}

void CRSFParser::reset(){
    parseState_=ParseState::WAIT_ADDR; bufIdx_=frameLen_=0;
    frameReady_=false; msSinceLastFrame_=0;
    for(auto&c:raw_) c=CH_MID;
}

void CRSFParser::feedByte(uint8_t b){
    switch(parseState_){
    case ParseState::WAIT_ADDR:
        if(b==CRSF_ADDR_FC){buf_[0]=b;bufIdx_=1;parseState_=ParseState::WAIT_LEN;}
        break;
    case ParseState::WAIT_LEN:
        if(b<4||b>CRSF_MAX_FRAME-2){parseState_=ParseState::WAIT_ADDR;break;}
        buf_[1]=b; frameLen_=b; bufIdx_=2; parseState_=ParseState::WAIT_DATA;
        break;
    case ParseState::WAIT_DATA:
        buf_[bufIdx_++]=b;
        if(bufIdx_>=frameLen_+2u){
            /* BUG FIX #2: validate CRC before accepting any frame */
            uint8_t calc=crc8dvbs2(&buf_[2],frameLen_-1u);
            uint8_t recv=buf_[frameLen_+1u];
            /* RC channels contain exactly 22 packed bytes. A short frame of
             * the same type must never reach unpackChannels(). */
            if(calc==recv && buf_[2]==CRSF_FRAMETYPE_RC && frameLen_==24u){
                unpackChannels(&buf_[3]);
                frameReady_=true;
                msSinceLastFrame_=0; /* BUG FIX #3: reset failsafe timer */
            }
            parseState_=ParseState::WAIT_ADDR; bufIdx_=0;
        }
        break;
    }
}

void CRSFParser::unpackChannels(const uint8_t* p){
    /* BUG FIX #1: original had `0.07FF` (float=0.07) as mask. All channels
     * beyond CH0 were garbage. Correct integer bitmask is 0x07FFu. */
    raw_[ 0]=((uint16_t)p[ 0]      |((uint16_t)p[ 1]<<8))&0x07FFu;
    raw_[ 1]=((uint16_t)p[ 1]>>3   |((uint16_t)p[ 2]<<5))&0x07FFu;
    raw_[ 2]=((uint16_t)p[ 2]>>6   |((uint16_t)p[ 3]<<2)|((uint16_t)p[4]<<10))&0x07FFu;
    raw_[ 3]=((uint16_t)p[ 4]>>1   |((uint16_t)p[ 5]<<7))&0x07FFu;
    raw_[ 4]=((uint16_t)p[ 5]>>4   |((uint16_t)p[ 6]<<4))&0x07FFu;
    raw_[ 5]=((uint16_t)p[ 6]>>7   |((uint16_t)p[ 7]<<1)|((uint16_t)p[8]<<9)) &0x07FFu;
    raw_[ 6]=((uint16_t)p[ 8]>>2   |((uint16_t)p[ 9]<<6))&0x07FFu;
    raw_[ 7]=((uint16_t)p[ 9]>>5   |((uint16_t)p[10]<<3))&0x07FFu;
    raw_[ 8]=((uint16_t)p[11]      |((uint16_t)p[12]<<8))&0x07FFu;
    raw_[ 9]=((uint16_t)p[12]>>3   |((uint16_t)p[13]<<5))&0x07FFu;
    raw_[10]=((uint16_t)p[13]>>6   |((uint16_t)p[14]<<2)|((uint16_t)p[15]<<10))&0x07FFu;
    raw_[11]=((uint16_t)p[15]>>1   |((uint16_t)p[16]<<7))&0x07FFu;
    raw_[12]=((uint16_t)p[16]>>4   |((uint16_t)p[17]<<4))&0x07FFu;
    raw_[13]=((uint16_t)p[17]>>7   |((uint16_t)p[18]<<1)|((uint16_t)p[19]<<9)) &0x07FFu;
    raw_[14]=((uint16_t)p[19]>>2   |((uint16_t)p[20]<<6))&0x07FFu;
    raw_[15]=((uint16_t)p[20]>>5   |((uint16_t)p[21]<<3))&0x07FFu;
}

void CRSFParser::processFrame(bool fpvMode){
    uint16_t channels[NUM_CHANNELS];
    const uint32_t primask=__get_PRIMASK();
    __disable_irq();
    const bool ready=frameReady_;
    if(ready){
        for(int i=0;i<NUM_CHANNELS;i++) channels[i]=raw_[i];
        frameReady_=false;
    }
    __set_PRIMASK(primask);
    if(!ready) return;
    /* BUG FIX #4: apply normalisation, deadband, expo — previously absent */
    float rExpo=fpvMode?FPV_ROLL_EXPO:CAM_ROLL_EXPO;
    float pExpo=fpvMode?FPV_PITCH_EXPO:CAM_PITCH_EXPO;
    float yExpo=fpvMode?FPV_YAW_EXPO:CAM_YAW_EXPO;
    float db=RC_DEADBAND;

    float roll    =applyDeadbandExpo(normaliseChannel(channels[RC_CH_ROLL]),   db,rExpo);
    float pitch   =applyDeadbandExpo(normaliseChannel(channels[RC_CH_PITCH]),  db,pExpo);
    float yaw     =applyDeadbandExpo(normaliseChannel(channels[RC_CH_YAW]),    db,yExpo);
    float throttle=normaliseThrottle(channels[RC_CH_THROTTLE]);
    bool  arm =(channels[RC_CH_ARM]     >SW_HIGH);
    bool  mode=(channels[RC_CH_MODE]    >SW_HIGH);
    bool  alt =(channels[RC_CH_ALT_HOLD]>SW_HIGH);

    osMutexAcquire(g_state.rcMtx,osWaitForever);
    g_state.rc={roll,pitch,yaw,throttle,arm,mode,alt,true,(uint32_t)osKernelGetTickCount()};
    osMutexRelease(g_state.rcMtx);
}

void CRSFParser::tick1ms(){
    const uint32_t primask=__get_PRIMASK();
    __disable_irq();
    if(msSinceLastFrame_<kFailsafeMs+1u) ++msSinceLastFrame_;
    const bool timedOut=msSinceLastFrame_>kFailsafeMs;
    __set_PRIMASK(primask);
    if(timedOut){
        osMutexAcquire(g_state.rcMtx,osWaitForever);
        g_state.rc.valid=false;
        osMutexRelease(g_state.rcMtx);
    }
}

}} // FC::Communication

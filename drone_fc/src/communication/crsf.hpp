#pragma once
#include <stdint.h>
#include "../../Inc/fc_config.h"

namespace FC { namespace Communication {

/** CRSF/ELRS RC Protocol Parser
 *
 * BUG FIX #1: `0.07FF` float literal used as bitmask — CH2-CH16 garbage. Fixed to 0x07FFu.
 * BUG FIX #2: No CRC validation. Fixed with CRC8/DVB-S2 on every frame.
 * BUG FIX #3: No failsafe timeout. Fixed: 300 ms triggers rc.valid=false.
 * BUG FIX #4: Raw [172-1811] fed to PID. Fixed: normalise + deadband + expo in processFrame().
 */
class CRSFParser {
public:
    static constexpr uint8_t  CRSF_ADDR_FC      = 0xC8u;
    static constexpr uint8_t  CRSF_FRAMETYPE_RC = 0x16u;
    static constexpr int      CRSF_MAX_FRAME    = 64;
    static constexpr int      NUM_CHANNELS      = 16;
    static constexpr uint16_t CH_MIN=172u, CH_MID=992u, CH_MAX=1811u;
    static constexpr uint16_t SW_HIGH=1600u;

    CRSFParser(){reset();}
    void feedByte(uint8_t b);
    void processFrame(bool fpvMode);
    void tick1ms();

    uint16_t getRawChannel(int ch)const{return(ch>=0&&ch<NUM_CHANNELS)?raw_[ch]:CH_MID;}
    bool isFrameReady()const{return frameReady_;}
    void clearFrameReady(){frameReady_=false;}

    static float normaliseChannel(uint16_t raw){
        float span=(raw>=CH_MID)?(float)(CH_MAX-CH_MID):(float)(CH_MID-CH_MIN);
        float n=((float)raw-(float)CH_MID)/span;
        return n>1.f?1.f:(n<-1.f?-1.f:n);
    }
    static float normaliseThrottle(uint16_t raw){
        float n=(float)(raw-CH_MIN)/(float)(CH_MAX-CH_MIN);
        return n>1.f?1.f:(n<0.f?0.f:n);
    }
    static float applyDeadbandExpo(float v,float db,float expo){
        if(v>db) v=(v-db)/(1.f-db);
        else if(v<-db) v=(v+db)/(1.f-db);
        else return 0.f;
        return (1.f-expo)*v+expo*v*v*v;
    }

private:
    enum class ParseState:uint8_t{WAIT_ADDR,WAIT_LEN,WAIT_DATA};
    uint8_t    buf_[CRSF_MAX_FRAME]; uint8_t bufIdx_=0,frameLen_=0;
    ParseState parseState_=ParseState::WAIT_ADDR;
    uint16_t   raw_[NUM_CHANNELS];
    bool       frameReady_=false;
    uint32_t   msSinceLastFrame_=0;
    static constexpr uint32_t kFailsafeMs=300u;
    void reset(); static uint8_t crc8dvbs2(const uint8_t*,uint8_t);
    void unpackChannels(const uint8_t*);
};

}} // FC::Communication

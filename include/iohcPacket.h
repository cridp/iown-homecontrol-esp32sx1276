#ifndef IOHC_PACKET_H
#define IOHC_PACKET_H

#include <cstring>
#include <vector>
#include <bitset>

#include <board-config.h>

#if defined(SX1276)
        #include <SX1276Helpers.h>
#elif defined(CC1101)
        #include <CC1101Helpers.h>
#endif

#define RESET_AFTER_LAST_MSG_US 15000
#define MAX_FRAME_LEN   32
#define IOHC_INBOUND_MAX_PACKETS        255     // Maximum Inbound packets buffer
#define IOHC_OUTBOUND_MAX_PACKETS       20      // Maximum Outbound packets
           
namespace IOHC {

typedef uint8_t address[3];

typedef struct  {
  uint8_t MsgLen : 5; //1
  uint8_t Protocol : 1; //2
  uint8_t StartFrame : 1; //3
  uint8_t EndFrame : 1; //4
} CB1;
typedef struct  {
  uint8_t Version : 2;
  uint8_t Prio : 1;
  uint8_t Unk2 : 1;
  uint8_t Unk3 : 1;
  uint8_t LPM : 1;
  uint8_t Routed : 1;
  uint8_t Beacon : 1;
} CB2;
union CtrlByte1Union {
    uint8_t asByte;
    CB1 asStruct;
};
union CtrlByte2Union {
    uint8_t asByte;
    CB2 asStruct;
};
// Maximum payload in IOHC is 32 bytes: 1 Length byte + 31 body bytes
    struct _header {
        CtrlByte1Union CtrlByte1; //1
        CtrlByte2Union CtrlByte2; //1
        address         target;   //3
        address         source;   //3
        uint8_t         cmd;      //1
    };

    struct Acei {
        uint8_t isvalid: 1;
        uint8_t extended:2;
        uint8_t service: 2;
        uint8_t level:   3;
    };
    union AceiUnion {
        uint8_t asByte;
        Acei asStruct;
    };

//61 0110 0001
//83 1000 0011
    inline void setAcei(AceiUnion &acei, uint8_t value) {
        acei.asByte = value;
        // acei.level = (value >> 5) & 0x07;
        // acei.service = (value >> 3) & 0x03;
        // acei.extended = (value >> 1) & 0x03;
        // acei.isvalid = value & 0x01;
    }

    struct _p0x00 {
        uint8_t         origin;
        AceiUnion       acei;
        uint8_t         main[2];
        uint8_t         fp1;
        uint8_t         fp2;
        uint8_t         data[2];    
        uint8_t         sequence[2];
        uint8_t         hmac[6];
    };
    struct _p0x00_all {
        uint8_t         origin;
        AceiUnion       acei;
        uint8_t         main[2];
        uint8_t         fp1;
        uint8_t         fp2;   
        uint8_t         sequence[2];
        uint8_t         hmac[6];
    };
    
    struct _p0x2b {
        uint8_t         actuator[2];
        address         backbone;
        uint8_t         manufacturer;
        uint8_t         info;
        uint8_t         tstamp[2];
    };
    struct _p0x2e {
        uint8_t         data;
        uint8_t         sequence[2];
        uint8_t         hmac[6];
    };
    struct _p0x30 {
        uint8_t         enc_key[16];
        uint8_t         man_id;
        uint8_t         data;
        uint8_t         sequence[2];
    };

    union _msg {
        _p0x00  p0x00;
        _p0x00_all  p0x00_all;
        _p0x2b  p0x29;
        _p0x2b  p0x2b;
        _p0x2e  p0x2e;
        _p0x30  p0x30;
        _p0x2e  p0x39;   // same format of 2e
    };

    struct _packet {
        _header header;
        _msg    msg;
    };

    typedef union {
        uint8_t     buffer[MAX_FRAME_LEN];
        _packet     packet;
    } Payload;

/* keep the size of variable lenght of data */
    typedef struct { 
        uint8_t memorizedCmd;
        std::vector<uint8_t> memorizedData;
    //    uint8_t* data;
    //    u_int8_t size;
    } Memorize;
 
    inline unsigned long packetStamp = 0L;
    inline unsigned long relStamp = 0L;
    inline size_t lastSendCmd = 0xFF;
    /*
    Class implementing the IOHC packet received/sent, including some additional members useful to track/control Radio parameters and timings
*/
    class iohcPacket {
        public:
            iohcPacket()  = default;
            ~iohcPacket() = default;

            Payload payload{};
            uint8_t buffer_length = 0;
            uint32_t frequency = CHANNEL2; // Both 1W & 2W
            unsigned long repeatTime = 0L;
            uint8_t repeat = 0;
            bool lock = false;
            unsigned long delayed = 0;

        double/*int32_t*/ afc{};    // AFC freq correction applied
        uint8_t snr{};    // in dB
        float/*uint8_t*/ rssi{};   // -RSSI*2 of last packet received
        uint8_t lna{};    // LNA attenuation in dB

//            inline static unsigned long stamp = 0L;
//            inline static unsigned long relStamp = 0L;
            
            void decode(bool verbosity = false);

        protected:
            uint8_t source_originator[3] = {0};

        private:

    };
}
#endif
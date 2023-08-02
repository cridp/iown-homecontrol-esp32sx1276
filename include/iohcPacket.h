#pragma once

#include <Arduino.h>
#include <SX1276Helpers.h>

#define RESET_AFTER_LAST_MSG_US 15000
#define MAX_FRAME_LEN   32

namespace IOHC
{
// To be checked
//
// Packet format
//
// b1 - b2: Frame length + Control 
// b3 - b5: Target nodeId
// b6 - b8: Source nodeId
// b9 - b11: ???
// b12: Command
// b13 - b16: ???
// b17: Sequence
// b18: ???

    typedef uint8_t address[3];


// Maximum payload in IOHC is 32 bytes: 1 Length byte + 31 body bytes
    struct _header
    {
        uint8_t         framelength:5;
        uint8_t         mode:1;
        uint8_t         first:1;
        uint8_t         last:1;
        uint8_t         prot_v:2;
        uint8_t         _unq1:1;
        uint8_t         _unq2:1;
        uint8_t         ack:1;
        uint8_t         low_p:1;
        uint8_t         routed:1;
        uint8_t         use_beacon:1;
        address         target;
        address         source;
        uint8_t         cmd;
    };

    struct _p0x00
    {
        uint8_t         origin;
        uint8_t         acei;
        uint8_t         main[2];
        uint8_t         fp1;
        uint8_t         fp2;
        uint8_t         sequence[2];
        uint8_t         hmac[6];
    };
    struct _p0x2b
    {
        uint8_t         actuator[2];
        address         backbone;
        uint8_t         manufacturer;
        uint8_t         info;
        uint8_t         tstamp[2];
    };
    struct _p0x2e
    {
        uint8_t         data;
        uint8_t         sequence[2];
        uint8_t         hmac[6];
    };
    struct _p0x30
    {
        uint8_t         enc_key[16];
        uint8_t         man_id;
        uint8_t         data;
        uint8_t         sequence[2];
    };

    union _msg
    {
        struct _p0x00  p0x00;
        struct _p0x2b  p0x2b;
        struct _p0x2e  p0x2e;
        struct _p0x30  p0x30;
        struct _p0x2e  p0x39;   // same format of 2e
    };

    struct _packet
    {
        _header header;
        _msg    msg;
    };

    typedef union
    {
        uint8_t     buffer[MAX_FRAME_LEN];
        _packet     packet;
    } Payload;

/*
    Class implementing the IOHC packet received/sent, including some additional members useful to track/control Radio parameters and timings
*/
    class iohcPacket
    {
        public:
            iohcPacket(void)
            {
            }


            ~iohcPacket()
            {
                ; // destroyer 
            }

            Payload payload;
            uint8_t buffer_length = 0;
            uint32_t frequency;
            unsigned long millis = 0;
            uint8_t repeat = 0;
            bool lock = false;
            float rssi = 0;

        protected:

        private:
    };
}
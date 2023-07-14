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


// Maximum payload in IOHC is 32 bytes: 1 Length byte + 31 body bytes
    struct _header
    {
        unsigned char   framelength:5;
        unsigned char   mode:1;
        unsigned char   first:1;
        unsigned char   last:1;
        unsigned char   prot_v:2;
        unsigned char   _unq1:1;
        unsigned char   _unq2:1;
        unsigned char   ack:1;
        unsigned char   low_p:1;
        unsigned char   routed:1;
        unsigned char   use_beacon:1;
        unsigned char   target[3];
        unsigned char   source[3];
        unsigned char   cmd;
    };

    struct _p0x00
    {
        unsigned char   origin;
        unsigned char   acei;
        unsigned char   main[2];
        unsigned char   fp1;
        unsigned char   fp2;
        unsigned char   sequence[2];
        unsigned char   hmac[6];
    };
    struct _p0x2e
    {
        unsigned char   data;
        unsigned char   sequence[2];
        unsigned char   hmac[6];
    };
    struct _p0x30
    {
        unsigned char   enc_key[16];
        unsigned char   man_id;
        unsigned char   data;
        unsigned char   sequence[2];
    };

    union _msg
    {
        struct _p0x00  p0x00;
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
        char            buffer[MAX_FRAME_LEN];
        _packet         packet;
    } Payload;




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
            float rssi = 0;

        protected:

        private:
    };
}
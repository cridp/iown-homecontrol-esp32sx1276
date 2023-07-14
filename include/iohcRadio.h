#pragma once

#include <Arduino.h>
#include <map>
#include <Delegate.h>
#include <SX1276Helpers.h>
#include <iohcPacket.h>
#include <TickerUs.h>

#define SCAN_INTERVAL_US                1500
#define SM_GRANULARITY_US               100
#define SM_CHECK_US                     500
//#define SM_PREAMBLE_RECOVERY_TIMEOUT_US 22000
#define SM_PREAMBLE_RECOVERY_TIMEOUT_US 12500
#define IOHC_INBOUND_MAX_PACKETS        199
#define IOHC_OUTBOUND_MAX_PACKETS       20


namespace IOHC
{
    using IohcPacketDelegate = Delegate<bool(IOHC::iohcPacket *iohc)>;

    class iohcRadio
    {
        public:
            static iohcRadio *getInstance();
            virtual ~iohcRadio() {};
            void start(uint8_t num_freqs, uint32_t *scan_freqs, uint32_t scanTimeUs, IohcPacketDelegate rxCallback, IohcPacketDelegate txCallback);
            void send(IOHC::iohcPacket *iohcTx[]);

        private:
            iohcRadio();
            bool receive(void);
            bool sent(IOHC::iohcPacket *packet);

            static iohcRadio *_iohcRadio;
            volatile static bool __g_preamble;
            volatile static bool __g_payload;
            volatile static bool __g_timeout;
            static uint8_t __flags[2];
            volatile static unsigned long __g_payload_millis;
            static bool f_lock;

            volatile uint32_t tickCounter = 0;
            volatile uint32_t preCounter = 0;
            volatile uint8_t txCounter = 0;

            bool preambleDetected = false;
            bool payloadOk = false;
            unsigned long payloadMillis = 0;

            uint8_t num_freqs = 0;
            uint32_t *scan_freqs;
            uint32_t scanTimeUs;
            uint8_t currentFreq = 0;
            bool fHopping = false;
            bool inTxMode = false;

            Timers::TickerUs TickTimer;
            Timers::TickerUs Sender;
//            Timers::TickerUs FreqScanner;

            IOHC::iohcPacket *iohc;
            IohcPacketDelegate rxCB = nullptr;
            IohcPacketDelegate txCB = nullptr;
            IOHC::iohcPacket *packets2send[IOHC_OUTBOUND_MAX_PACKETS];

        protected:
            static void IRAM_ATTR i_preamble(void);
            static void IRAM_ATTR i_payload(void);
            static void tickerCounter(iohcRadio *radio);
            static void packetSender(iohcRadio *radio);
    };
}
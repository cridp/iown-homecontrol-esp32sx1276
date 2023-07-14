#pragma once

#include <Arduino.h>
#include <map>
#include <Delegate.h>
#include <SX1276Helpers.h>
#include <iohcPacket.h>
#include <TickerUs.h>

#define SCAN_INTERVAL_US                3500
#define SM_GRANULARITY_US               100
#define SM_CHECK_US                     500
#define SM_PREAMBLE_RECOVERY_TIMEOUT_US 700000


namespace Radio
{
    using ReceivedPacketDelegate = Delegate<bool(IOHC::iohcPacket *iohc)>;

    class Receiver
    {
        public:
            Receiver(uint8_t num_freqs, uint32_t *scan_freqs, uint32_t scanTimeUs, ReceivedPacketDelegate callback = nullptr):
                num_freqs(num_freqs), scan_freqs(scan_freqs), scanTimeUs(scanTimeUs), packetCB(callback)
            {
                Init();
            }

            ~Receiver()
            {
                Scan(false);
                Stop();
            }

            void Start(void);
            void Stop(void);
            void Scan(bool enabled);

        protected:
            static void tickerCounter(Receiver *receiver);
            static void manageStates(Receiver *receiver);
            static void IRAM_ATTR i_fifo(void);
            static void IRAM_ATTR i_preamble(void);
            static void IRAM_ATTR i_payload(void);

        private:
            void Init(void);
            bool getPacket(void);

        private:
            Timers::TickerUs TickTimer;
            Timers::TickerUs StateMachine;
            Timers::TickerUs FreqScanner;

            uint8_t num_freqs = 0;
            uint32_t *scan_freqs;
            uint32_t scanTimeUs;
            ReceivedPacketDelegate packetCB;
            IOHC::iohcPacket iohc;

            uint32_t tickCounter = 0;
            uint32_t preCounter = 0;
            bool preambleDetected = false;
            bool crcReceivedOk = false;
            unsigned long crcReceivedMillis = 0;
            bool f_lock = false;

            uint8_t currentFreq = 0;
            bool fHopping = false;
    };

}
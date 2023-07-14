#pragma once

#include <Arduino.h>
#include <map>
#include <Delegate.h>
#include <SX1276Helpers.h>
#include <iohcPacket.h>
#include <TickerUs.h>

#define SM_GRANULARITY_US               100
#define SM_CHECK_US                     500


namespace Radio
{
    using TransmittedPacketDelegate = Delegate<bool(IOHC::iohcPacket *iohc)>;
    
    class Transmitter
    {
        public:
            Transmitter(TransmittedPacketDelegate callback = nullptr):
                packetCB(callback)
            {
                Init();
            }

            ~Transmitter()
            {
                Stop();
            }

            void Start(void);
            void Stop(void);
            void Send(IOHC::iohcPacket *iohcTx);

        protected:
            static void tickerCounter(Transmitter *transmitter);
            static void manageStates(Transmitter *transmitter);
            static void IRAM_ATTR i_payload(void);

        private:
            void Init(void);
            bool putPacket(void);

        private:
            Timers::TickerUs TickTimer;
            Timers::TickerUs StateMachine;

            IOHC::iohcPacket *iohc;
            TransmittedPacketDelegate packetCB;

            uint32_t tickCounter = 0;
            bool crcReceivedOk = false;
            unsigned long crcSentMillis = 0;
    };

}
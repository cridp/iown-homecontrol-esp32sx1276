#ifndef IOHC_RADIO_H
#define IOHC_RADIO_H

#include <memory>
#include <board-config.h>

#include <map>
#include <Delegate.h>
#include <iohcCryptoHelpers.h>
#if defined(SX1276)
        #include <SX1276Helpers.h>
#elif defined(CC1101)
        #include <CC1101Helpers.h>
#endif

#include <iohcPacket.h>
#if defined(ESP8266)
  #include <TickerUs.h>
#elif defined(HELTEC)  	
    #include <TickerUsESP32.h>
#endif

#define SM_GRANULARITY_US               100ULL  // Ticker function frequency in uS (100 min)
#define SM_GRANULARITY_MS               1       // Ticker function frequency in uS
#define SM_PREAMBLE_RECOVERY_TIMEOUT_US 12500   // SM_GRANULARITY_US * PREAMBLE_LSB //12500   // Maximum duration in uS of Preamble before reset of receiver
#define DEFAULT_SCAN_INTERVAL_US        13520   // Default uS between frequency changes

/*
    Singleton class to implement an IOHC Radio abstraction layer for controllers.
    Implements all needed functionalities to receive and send packets from/to the air, masking complexities related to frequency hopping
    IOHC timings, async sending and receiving through callbacks, ...
*/
namespace IOHC {
    using IohcPacketDelegate = Delegate<bool(iohcPacket *iohc)>;

    class iohcRadio  {
        public:
            static iohcRadio *getInstance();
            virtual ~iohcRadio() = default;
            void start(uint8_t num_freqs, uint32_t *scan_freqs, uint32_t scanTimeUs, IohcPacketDelegate rxCallback, IohcPacketDelegate txCallback);
            void send(std::vector<iohcPacket*>&iohcTx);

            // void send(std::array<iohcPacket*, 25>& iohcTx);
            // template <size_t N> //Just for the fun, waiting to use a vector for all
            // void send(std::array<iohcPacket*, N>& iohcTx) {
            //     uint8_t idx = 0;
            
            //     if (txMode)  return;
            //     do {
            //         packets2send[idx] = iohcTx[idx];
            //     } while (iohcTx[idx++]);
            //              // for (auto packet : iohcTx) {
            //              //     if (!packet) break;
            //              //     packets2send[idx++] = packet;
            //              // }
            //     txCounter = 0;
            //     Sender.attach_ms(packets2send[txCounter]->repeatTime, packetSender, this);
            // }

        // void send(std::vector<iohcPacket*>& iohcTx) {
        //         // uint8_t idx = 0;
        //         if (txMode)  return;

        //         packets2send = iohcTx; //std::move(iohcTx); //
        //         iohcTx.clear();

        //         txCounter = 0;
        //         Sender.attach_ms(packets2send[txCounter]->repeatTime, packetSender, this);
        //     }

        private:
            iohcRadio();
            bool IRAM_ATTR receive(bool stats);
            bool IRAM_ATTR sent(iohcPacket *packet);

            static iohcRadio *_iohcRadio;
            volatile static bool _g_preamble;
            volatile static bool _g_payload;
//            volatile static bool __g_timeout;
            static uint8_t _flags[2];
            volatile static unsigned long _g_payload_millis;
            
            volatile static bool f_lock;
            volatile static bool send_lock;
            volatile static bool txMode;

            volatile uint32_t tickCounter = 0;
            volatile uint32_t preCounter = 0;
            volatile uint8_t txCounter = 0;

            uint8_t num_freqs = 0;
            uint32_t *scan_freqs{};
            uint32_t scanTimeUs{};
            uint8_t currentFreqIdx = 0;


        #if defined(ESP8266)
            Timers::TickerUs TickTimer;
            Timers::TickerUs Sender;
//            Timers::TickerUs FreqScanner;
        #elif defined(HELTEC)
            TimersUS::TickerUsESP32 TickTimer;
            TimersUS::TickerUsESP32 Sender;
//            TickerUsESP32 TickTimer;
//            TickerUsESP32 Sender;
            //Timers::TickerUsESP32 FreqScanner;    
        #endif
            iohcPacket *iohc{};
            iohcPacket *delayed{};
            
            IohcPacketDelegate rxCB = nullptr;
            IohcPacketDelegate txCB = nullptr;
//            std::array<iohcPacket*, 25> packets2send{};
            std::vector<iohcPacket*> packets2send{};
        protected:
            static void IRAM_ATTR i_preamble();
            static void IRAM_ATTR i_payload();
            static void IRAM_ATTR tickerCounter(iohcRadio *radio);

            static void packetSender(iohcRadio *radio);

        #if defined(CC1101)
            uint8_t lenghtFrame=0;
        #endif
    };
}

//#include <iohcRadio.tpp>
#endif
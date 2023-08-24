#pragma once

#include <Arduino.h>
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
#elif defined(ESP32)  	
  #include <TickerUsESP32.h>
#endif

#define DEFAULT_SCAN_INTERVAL_US        1000    // Default uS between frequency changes
#define SM_GRANULARITY_US               100     // Ticker function frequency in uS
#define SM_GRANULARITY_MS               1       // Ticker function frequency in uS
#define SM_PREAMBLE_RECOVERY_TIMEOUT_US 12500   // Maximum duration in uS of Preamble before reset of receiver
#define IOHC_INBOUND_MAX_PACKETS        199     // Maximum Inbound packets buffer
#define IOHC_OUTBOUND_MAX_PACKETS       20      // Maximum Outbound packets


/*
    Singleton class to implement an IOHC Radio abstraction layer for controllers.
    Implements all needed functionalities to receive and send packets from/to the air, masking complexities related to frequency hopping
    IOHC timings, async sending and receiving through callbacks, ...
*/
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
//            volatile static bool __g_timeout;
            static uint8_t __flags[2];
            volatile static unsigned long __g_payload_millis;
            volatile static bool f_lock;
            volatile static bool send_lock;
            volatile static bool txMode;

            volatile uint32_t tickCounter = 0;
            volatile uint32_t preCounter = 0;
            volatile uint8_t txCounter = 0;

            uint8_t num_freqs = 0;
            uint32_t *scan_freqs;
            uint32_t scanTimeUs;
            uint8_t currentFreq = 0;


#if defined(ESP8266)
            Timers::TickerUs TickTimer;
            Timers::TickerUs Sender;
//            Timers::TickerUs FreqScanner;
#elif defined(ESP32)
            //Timers::TickerUsESP32 TickTimer;
            //Timers::TickerUsESP32 Sender;
            TickerUsESP32 TickTimer;
            TickerUsESP32 Sender;
//            Timers::TickerUsESP32 FreqScanner;    
#endif



            IOHC::iohcPacket *iohc;
            IohcPacketDelegate rxCB = nullptr;
            IohcPacketDelegate txCB = nullptr;
            IOHC::iohcPacket *packets2send[IOHC_OUTBOUND_MAX_PACKETS];

        protected:
//#if defined(SX1276)
            static void IRAM_ATTR i_preamble(void);
            static void IRAM_ATTR i_payload(void);
            static void tickerCounter(iohcRadio *radio);
            static void packetSender(iohcRadio *radio);
//#elif defined(CC1101)
            //static void IRAM_ATTR i_preamble(void);
            //static void IRAM_ATTR i_payload(void);
            //static void tickerCounter(iohcRadio *radio);
            //static void packetSender(iohcRadio *radio);
//#endif
#if defined(CC1101)
            uint8_t lenghtFrame=0;
#endif
    };
}
/*
   Copyright (c) 2024. CRIDP https://github.com/cridp

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

           http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */

#ifndef IOHC_RADIO_H
#define IOHC_RADIO_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <Delegate.h>
#include <cstdint>

#include <board-config.h>
#include <iohcCryptoHelpers.h>
#include <iohcPacket.h>
#include <queue>

#if defined(RADIO_SX127X)
        #include <SX1276Helpers.h>
#endif
// #if defined(RADIO_SX126X)
//         #include <SX126xHelpers.h>
// #endif
#if defined(ESP32)
    #include <TickerUsESP32.h>
#endif

#define SM_GRANULARITY_US               130ULL  // Ticker function frequency in uS (100 minimum) 4 x 26µs = 104
#define SM_GRANULARITY_MS               1       // Ticker function frequency in uS
#define SM_PREAMBLE_RECOVERY_TIMEOUT_US 1378 // 12500   // SM_GRANULARITY_US * PREAMBLE_LSB //12500   // Maximum duration in uS of Preamble before reset of receiver
// Frequency Hopping with ~2,75ms per Channel (Patent: 3ms)
#define DEFAULT_SCAN_INTERVAL_US        13520   // Default uS between frequency changes

struct TxPacketWrapper;
/*
    Singleton class to implement an IOHC Radio abstraction layer for controllers.
    Implements all needed functionalities to receive and send packets from/to the air, masking complexities related to frequency hopping
    IOHC timings, async sending and receiving through callbacks, ...
*/
namespace IOHC {
    using CallbackFunction = Delegate<bool(iohcPacket *iohc)>;

    class iohcRadio  {
        public:
            static iohcRadio *getInstance();
            virtual ~iohcRadio() {
                // Nettoyer la queue
                // clearTxQueue();
                if (txQueue_mutex) vSemaphoreDelete(txQueue_mutex);
                if (tx_mutex) vSemaphoreDelete(tx_mutex);
            };
            enum class RadioState : uint8_t {
                IDLE,        ///< Default state: nothing happening
                RX,          ///< Receiving mode
                TX,          ///< Transmitting mode
                PREAMBLE,    ///< Preamble detected
                PAYLOAD,     ///< Payload available
                LOCKED,      ///< Frequency locked
                ERROR        ///< Error or unknown state
            };
        // États de verrouillage FHSS
        enum class FHSSLockReason {
            NONE = 0,           // Pas de verrouillage, hopping libre
            PREAMBLE_DETECTED,  // Préambule détecté
            RECEIVING,          // Réception en cours
            TRANSMITTING,       // Transmission en cours
            WAITING_RESPONSE,   // En attente d'une réponse après TX
            MANUAL_LOCK         // Verrouillage manuel
        };

        FHSSLockReason fhssLockReason = FHSSLockReason::NONE;
        SemaphoreHandle_t fhss_state_mutex = nullptr;

        // Timeout pour déverrouiller automatiquement si pas d'activité
        uint32_t lastActivityTime = 0;
        static constexpr uint32_t FHSS_UNLOCK_TIMEOUT_MS = 500;  // 500ms sans activité = unlock

        // Flag pour indiquer qu'on attend une réponse après envoi
        bool expectingResponse = false;
        uint32_t responseTimeoutMs = 0;

            void start(uint8_t num_freqs, uint32_t *scan_freqs, uint32_t scanTimeUs, CallbackFunction rxCallback, CallbackFunction txCallback);

            bool send(std::vector<iohcPacket *> &TxPackets);

        static void setRadioState(RadioState newState);

            static void processNextPacketCallback(iohcRadio *radio);

            void startTransmission();

            void processNextPacket();

            void stopTransmission();

            void clearTxQueue();

            bool isTransmitting() const;

            void cancelTransmissions();

            static const char* radioStateToString(RadioState state);
        volatile static RadioState radioState;

            volatile static bool _g_preamble;
            // volatile static uint32_t preambleCounter;
            volatile static bool _g_payload;
            volatile static bool f_lock_hop;
            static void tickerCounter(iohcRadio *radio);

            bool sendSingle(iohcPacket *packet, bool copyPacket);

            bool sendPriority(iohcPacket *packet, bool copyPacket);

            static TaskHandle_t txTaskHandle; // TX Task handle
            static volatile bool txComplete;

                    volatile static bool txMode;
         uint8_t num_freqs; // = 0;
         uint32_t *scan_freqs; //{};
         uint32_t scanTimeUs; //{};
         uint8_t currentFreqIdx; // = 0;

        // File d'attente thread safe pour les paquets à envoyer
        std::queue<TxPacketWrapper*> txQueue;
        SemaphoreHandle_t txQueue_mutex = nullptr;
        // Paquet actuellement en cours de transmission
        TxPacketWrapper* currentTxPacket = nullptr;
        // Mutex pour protéger l'accès concurrent
        SemaphoreHandle_t tx_mutex = nullptr;
        bool isSending = false;

        private:
            iohcRadio();

            bool receive(bool stats) const;
            bool sent(iohcPacket *packet) const;

            static iohcRadio *_iohcRadio;
            static uint8_t _flags[2];
            volatile static unsigned long _g_payload_millis;
            
            volatile static bool send_lock;
//            volatile static bool txMode;

            volatile uint32_t tickCounter = 0;
            volatile uint8_t txCounter = 0;
            // static void txTaskLoop(void *pvParameters);
            // static void lightTxTask(void *pvParameters);
            //TaskHandle_t txTaskHandle = nullptr;
            // static void IRAM_ATTR onTxTicker(void *arg);


        #if defined(ESP32)
            TimersUS::TickerUsESP32 TickTimer;
            TimersUS::TickerUsESP32 Ticker;
        #endif
            iohcPacket *new_packet{};
            // iohcPacket *delayed{};
            
            CallbackFunction rxCB = nullptr;
            CallbackFunction txCB = nullptr;
            std::vector<iohcPacket*> packets2send{};


        protected:
            // static void i_preamble();
            // static void i_payload();
            static void packetSender(iohcRadio *radio);
     };
}

#endif
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

#include "iohcFHSS.hpp"
// #include "FHSSCalibration.hpp"

#if defined(RADIO_SX127X)
#include <SX1276Helpers.h>
#endif
// #if defined(RADIO_SX126X)
// #include <SX126xHelpers.h>
// #endif
#if defined(ESP32)
#include <TickerUsESP32.h>
#endif

#define SM_GRANULARITY_US               130ULL  // Ticker function frequency in uS (100 minimum) 4 x 26µs = 104
#define SM_GRANULARITY_MS               2       // Ticker function frequency in mS
#define SM_PREAMBLE_RECOVERY_TIMEOUT_US 1378 // 12500   // SM_GRANULARITY_US * PREAMBLE_LSB //12500   // Maximum duration in uS of Preamble before reset of receiver
// Frequency Hopping
#define DEFAULT_SCAN_INTERVAL_US        13520   // Default uS between frequency changes

struct TxPacketWrapper;

/*
    Singleton class to implement an IOHC Radio abstraction layer for controllers.
    Implements all needed functionalities to receive and send packets from/to the air, masking complexities related to frequency hopping
    IOHC timings, async sending and receiving through callbacks, ...
*/
namespace IOHC {

    using CallbackFunction = Delegate<bool(iohcPacket *iohc)>;

    // Déclaration de la fonction FHSSTimer (utilisée par AdaptiveFHSS)
    void FHSSTimer(iohcRadio *iohc_radio);

    class iohcRadio {
        friend class AdaptiveFHSS; // Permet à AdaptiveFHSS d'accéder aux membres privés
        friend class FHSSCalibration;
    public:
        static iohcRadio *getInstance();

        virtual ~iohcRadio() {
            // Nettoyer la queue
            clearTxQueue();
            // if (txQueue_mutex)
            //     vSemaphoreDelete(txQueue_mutex);
            if (tx_mutex)
                vSemaphoreDelete(tx_mutex);
            if (fhss_state_mutex)
                vSemaphoreDelete(fhss_state_mutex);
            if (txQueue_binary_sem)
                vSemaphoreDelete(txQueue_binary_sem);
        };

        enum class RadioState : uint8_t {
            IDLE, ///< Default state: nothing happening
            RX, ///< Receiving mode
            TX, ///< Transmitting mode
            PREAMBLE, ///< Preamble detected
            PAYLOAD, ///< Payload available
            LOCKED, ///< Frequency locked
            ERROR ///< Error or unknown state
        };

        static const char *radioStateToString(RadioState state) {
            switch (state) {
                case RadioState::IDLE: return "IDLE";
                case RadioState::RX: return "RX";
                case RadioState::TX: return "TX";
                case RadioState::PREAMBLE: return "PREAMBLE";
                case RadioState::PAYLOAD: return "PAYLOAD";
                case RadioState::LOCKED: return "LOCKED";
                case RadioState::ERROR: return "ERROR";
                default: return "UNKNOWN";
            }
        }

        // États de verrouillage FHSS
        enum class FHSSLockReason {
            NONE = 0, // Pas de verrouillage, hopping libre
            PREAMBLE_DETECTED, // Préambule détecté
            RECEIVING, // Réception en cours
            TRANSMITTING, // Transmission en cours
            WAITING_RESPONSE, // En attente d'une réponse après TX
            MANUAL_LOCK // Verrouillage manuel
        };

        static const char *fhssLockReasonToString(FHSSLockReason reason) {
            switch (reason) {
                case FHSSLockReason::NONE: return "NONE";
                case FHSSLockReason::PREAMBLE_DETECTED: return "PREAMBLE";
                case FHSSLockReason::RECEIVING: return "RECEIVING";
                case FHSSLockReason::TRANSMITTING: return "TRANSMITTING";
                case FHSSLockReason::WAITING_RESPONSE: return "WAITING_RESPONSE";
                case FHSSLockReason::MANUAL_LOCK: return "MANUAL";
                default: return "UNKNOWN";
            }
        }
        enum class FHSSState {
            SCANNING,      // En train de scanner les fréquences
            PREAMBLE_DETECTED, // Préambule détecté, verrouillé sur fréquence
            RECEIVING,     // En train de recevoir un payload
            PROCESSING,    // Traitement du paquet (ne bloque pas FHSS)
            TRANSMITTING   // En transmission
        };

        volatile FHSSState fhssState = FHSSState::SCANNING;
        // Méthodes pour gérer l'état FHSS
        void setFHSSState(FHSSState newState);
        FHSSState getFHSSState();

        const char *fhssStateToString(FHSSState state);

        void lockFHSS(FHSSLockReason reason);
        void unlockFHSS(FHSSLockReason reason);
        void forceUnlockFHSS();
        void checkFHSSTimeout();
        void updateFHSSActivity();

        // Utiliser atomic au lieu de sémaphore
        volatile FHSSLockReason fhssLockReason = FHSSLockReason::NONE;
        // Flag pour indiquer qu'on attend une réponse après envoi
        volatile bool expectingResponse = false;
        // Timeout pour déverrouiller automatiquement si pas d'activité
        volatile int64_t lastActivityTime = 0;
        volatile uint32_t responseTimeoutMs = 0;
        // Queue : utiliser une queue atomique simple (pas de sémaphore)
        volatile bool txQueue_busy = false;  // Simple spinlock

        SemaphoreHandle_t fhss_state_mutex = nullptr;

        static constexpr uint32_t FHSS_UNLOCK_TIMEOUT_MS = 6 * 47; // Sans activité = unlock


        void start(uint8_t num_freqs, uint32_t *scan_freqs, uint32_t scanTimeUs, CallbackFunction rxCallback, CallbackFunction txCallback);

        bool send(std::vector<iohcPacket *> &TxPackets);

        static void setRadioState(RadioState newState);

        static void processNextPacketCallback(iohcRadio *radio);

        void startTransmission();

        bool processNextPacket();
        void stopTransmission();
        void clearTxQueue();
        bool isTransmitting() const;
        void cancelTransmissions();

        // static const char *radioStateToString(RadioState state);

        volatile static RadioState radioState;

        // volatile static bool _g_preamble;
        // volatile static uint32_t preambleCounter;
        // volatile static bool _g_payload;
        volatile static bool f_lock_hop;

        static void tickerCounter(iohcRadio *radio);

        bool sendSingle(iohcPacket *packet);
        bool sendPriority(iohcPacket *packet);

        static TaskHandle_t txTaskHandle; // TX Task handle

        uint8_t num_freqs; // = 0;
        uint32_t *scan_freqs; //{};
        uint32_t scanTimeUs; //{};
        uint8_t currentFreqIdx; // = 0;

        // File d'attente thread safe pour les paquets à envoyer
        std::deque<TxPacketWrapper *> txQueue;
        // SemaphoreHandle_t txQueue_mutex = nullptr;
        SemaphoreHandle_t txQueue_binary_sem; // Au lieu de txQueue_busy
        // Paquet actuellement en cours de transmission
        TxPacketWrapper *currentTxPacket = nullptr;
        // Mutex pour protéger l'accès concurrent
        SemaphoreHandle_t tx_mutex = nullptr;
        bool isSending = false;

    //private:
        void setExpectingResponse(bool expecting, uint32_t timeoutMs);

        AdaptiveFHSS *adaptiveFHSS = nullptr;
        // FHSSCalibration *calibration = nullptr;

        iohcRadio();

        // bool receive_1(bool stats);

        void startPacketProcessor();

        static void packetProcessorTask(void *parameter);

        // Queue pour les paquets reçus
        static constexpr size_t PACKET_QUEUE_SIZE = 10;
        QueueHandle_t packetQueue;
        TaskHandle_t packetProcessorTaskHandle;

        // Buffer pour la lecture FIFO (toujours local à receive())
        iohcPacket tempRxPacket;

        // Mutex pour last1wPacket si nécessaire
        SemaphoreHandle_t lastPacketMutex;
        bool receive(bool stats);

        bool isResponsePacket(iohcPacket *packet);

        bool sent(iohcPacket *packet);

        static iohcRadio *_iohcRadio;
        static uint8_t _flags[2];
        volatile static unsigned long _g_payload_millis;

        volatile static bool send_lock;
        volatile static bool txMode;

        volatile uint32_t tickCounter = 0;
        volatile uint8_t txCounter = 0;


#if defined(ESP32)
        TimersUS::TickerUsESP32 TickTimer;
        TimersUS::TickerUsESP32 Ticker;
#endif
        iohcPacket *new_packet{};

        CallbackFunction rxCB = nullptr;
        CallbackFunction txCB = nullptr;
        std::vector<iohcPacket *> packets2send{};

        static void maybeCheckHeap(const char* tag) {
            if (!heap_caps_check_integrity_all(true)) {
                ets_printf("HEAP CORRUPTED at %s\n", tag);
            } else {
                ets_printf("heap ok at %s free:%u\n", tag, heap_caps_get_free_size(MALLOC_CAP_8BIT));
            }
        }
        void printStats() const {
            if (xSemaphoreTake(statsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                ets_printf("Radio Stats - Received: %lu, Processed: %lu, Dropped: %lu, Max Queue: %lu, Avg Delay: %luµs",
                        packetsReceived, packetsProcessed, packetsDropped,
                        maxQueueDepth, totalQueueDelay / (packetsProcessed ? packetsProcessed : 1));
                xSemaphoreGive(statsMutex);
            }
        }
    private:
        // static void i_preamble();
        // static void i_payload();
        static void packetSender(iohcRadio *radio);

        // Métriques de performance
        uint32_t packetsReceived = 0;
        uint32_t packetsProcessed = 0;
        uint32_t packetsDropped = 0;
        uint32_t maxQueueDepth = 0;
        uint32_t totalQueueDelay = 0;
        SemaphoreHandle_t statsMutex;

    };
}

#endif

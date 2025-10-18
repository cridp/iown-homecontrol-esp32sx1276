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

#include <esp32-hal-gpio.h>
#include <map>
#include "esp_log.h"

#include <iohcRadio.h>
#include <log_buffer.h>
#include <utility>

#include "iohcCozyDevice2W.h"

// trame typique FSK : preamble (variable) + sync (3 B) + header (2 B) + payload (32 B) + CRC (2 B).
// airtime (s) = ( (preamble_bytes + 3 + 2 + payload_bytes + 2) * 8 ) / bitrate
#define LONG_PREAMBLE_MS 64  // 21.458 ms 23~24 1W
#define SHORT_PREAMBLE_MS 16 // 11.458 ms

TaskHandle_t IOHC::iohcRadio::txTaskHandle = nullptr;

// Structure pour gérer un paquet en transmission
struct TxPacketWrapper {
    IOHC::iohcPacket *packet;
    uint8_t repeatsRemaining;
    uint32_t repeatTime;

    explicit TxPacketWrapper(IOHC::iohcPacket *pkt)
        : packet(pkt),
          repeatsRemaining(pkt ? pkt->repeat : 0),
          repeatTime(pkt ? pkt->repeatTime : 0)
          {}

    ~TxPacketWrapper() {
        if (packet) {
            delete packet;
            packet = nullptr;
        }
    }
};

namespace IOHC {
    iohcRadio *iohcRadio::_iohcRadio = nullptr;
    // volatile bool iohcRadio::_g_preamble = false;
    // volatile bool iohcRadio::_g_payload = false;
    volatile uint32_t iohcRadio::_g_payload_millis = 0L;
    uint8_t iohcRadio::_flags[2] = {0, 0};
    volatile bool iohcRadio::f_lock_hop = false;
    volatile bool iohcRadio::send_lock = false;
    volatile bool iohcRadio::txMode = false;
    volatile iohcRadio::RadioState iohcRadio::radioState = iohcRadio::RadioState::IDLE;

    static IOHC::iohcPacket last1wPacket;

    TaskHandle_t handle_interrupt;
    //
    struct RadioIrqEvent {
        uint8_t flags;         // bit0 = preamble, bit1 = payload, autres reserved
        uint32_t ts_ms;        // timestamp en ms (esp_timer_get_time()/1000)
    };

    static QueueHandle_t radioIrqQueue = nullptr;
    static constexpr size_t RADIO_IRQ_QUEUE_LEN = 128;

    /**
    * No semaphore / lockFHSS() / updateFHSSActivity() here.
    * esp_timer_get_time() is ISR-safe ; millis() never sure.
    */
    void IRAM_ATTR handle_interrupt_fromisr() {
        // Debounce
        static uint32_t lastIsrTime = 0;
        uint32_t now = esp_timer_get_time();
        if (now - lastIsrTime < 100) return; // 100µs debounce
        lastIsrTime = now;

        // Only GPIOS (opérations très courtes), pas d'appels bloquants
        bool preamble = digitalRead(RADIO_PREAMBLE_DETECTED);
        bool payload  = digitalRead(RADIO_PACKET_AVAIL);

        // Create an event
        RadioIrqEvent ev;
        ev.flags = 0;
        if (preamble) ev.flags |= 0x01;
        if (payload)  ev.flags |= 0x02;
        ev.ts_ms = static_cast<uint32_t>(now / 1000ULL);

        // Send it to queue (FromISR)
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (radioIrqQueue) {
            // Ignored if queue empty
            if (xQueueSendFromISR(radioIrqQueue, &ev, &xHigherPriorityTaskWoken) != pdTRUE) {
                // Optionnel Compteur de drops (volatile)
                // irq_drop_count++;
            }
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    /**
    * Sequential : Read queue — no concurrencies possible.
    * tickerCounter isn't called by the ISR.
    */
    void handle_interrupt_task(void *pvParameters) {
        IOHC::iohcRadio *radio = static_cast<IOHC::iohcRadio*>(pvParameters);
        RadioIrqEvent ev;
        const TickType_t xMaxBlock = pdMS_TO_TICKS(500); // ou portMAX_DELAY si on veux

        while (true) {
            if (xQueueReceive(radioIrqQueue, &ev, xMaxBlock) == pdTRUE) {
                // No reentrance possible, using sequential
                bool preamble = ev.flags & 0x01;
                bool payload  = ev.flags & 0x02;

                if (preamble) {
                    // timestamp and lock
                    radio->lockFHSS(iohcRadio::FHSSLockReason::PREAMBLE_DETECTED);
                    radio->updateFHSSActivity(); // Write atomic safe
                    radio->setRadioState(iohcRadio::RadioState::PREAMBLE);
                }
                if (payload) {
                    // timestamp - Reception and lock
                    radio->lockFHSS(iohcRadio::FHSSLockReason::RECEIVING);
                    radio->updateFHSSActivity();
                    radio->setRadioState(iohcRadio::RadioState::PAYLOAD);
                    // Call tickerCounter (read SPI, receive(), etc.)
                    // tickerCounter is in task — can use semaphores, malloc, etc. Not the ISR is able to do that
                    radio->tickerCounter(radio);
                }
            }
            // No busy-wait
        }
    }

    void FHSSTimer(iohcRadio *iohc_radio) {
        // Check timeout
        iohc_radio->checkFHSSTimeout();

        // Si toujours verrouillé, pas de hop
        if (iohc_radio->f_lock_hop) {
            return;
        }

        // Ne sauter que si on est vraiment en RX idle
        if (iohcRadio::radioState != iohcRadio::RadioState::RX) {
            return;
        }

        // Incrémenter l'index de fréquence
        iohc_radio->currentFreqIdx++;
        if (iohc_radio->currentFreqIdx >= iohc_radio->num_freqs) {
            iohc_radio->currentFreqIdx = 0;
        }

        // Changer la fréquence seulement si on est en RX
        Radio::setCarrier(Radio::Carrier::Frequency, iohc_radio->scan_freqs[iohc_radio->currentFreqIdx]);

        // ESP_LOGV("IOHC", "FHSS hop to freq[%d]: %d Hz",
        //  iohc_radio->currentFreqIdx,
        //  iohc_radio->scan_freqs[iohc_radio->currentFreqIdx]);
    }

    /**
    * Verrouille le FHSS avec une raison spécifique
    */
    void iohcRadio::lockFHSS(const FHSSLockReason reason) {
        if (xSemaphoreTake(fhss_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Garder la raison la plus "forte"
            if (reason > fhssLockReason) {
                fhssLockReason = reason;
                f_lock_hop = true;
                lastActivityTime = millis();

                // ets_printf("(D) FHSS locked: %s\n", fhssLockReasonToString(reason));
            }
            xSemaphoreGive(fhss_state_mutex);
        }
    }

    /**
     * Déverrouille le FHSS pour une raison spécifique
     */
    void iohcRadio::unlockFHSS(const FHSSLockReason reason) {
        if (xSemaphoreTake(fhss_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Déverrouiller seulement si c'est la même raison
            if (fhssLockReason == reason) {
                fhssLockReason = FHSSLockReason::NONE;
                f_lock_hop = false;

                // ets_printf("(D) FHSS unlocked: %s\n", fhssLockReasonToString(reason));
            }
            xSemaphoreGive(fhss_state_mutex);
        }
    }

    /**
        * Force le déverrouillage (pour reset manuel)
        */
    void iohcRadio::forceUnlockFHSS() {
        if (xSemaphoreTake(fhss_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            fhssLockReason = FHSSLockReason::NONE;
            f_lock_hop = false;
            expectingResponse = false;

            // ESP_LOGW("IOHC", "FHSS force unlocked");
            xSemaphoreGive(fhss_state_mutex);
        }
    }

    /**
     * FHSS Watchdog
     */
    void iohcRadio::checkFHSSTimeout() {
        if (xSemaphoreTake(fhss_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // todo
            if (f_lock_hop && fhssLockReason != FHSSLockReason::TRANSMITTING) {
                uint32_t elapsed = millis() - lastActivityTime;

                // Timeouts adaptatifs basés sur le débit
                // airtime (s) = ( (preamble_bytes + 3SYNC + 2HEAD + payload_bytes + 2CRC) * 8 ) / bitrate
                const uint32_t bitrate = 38400; // 38.4 kbps
                const uint32_t maxPacketBits = (SHORT_PREAMBLE_MS + 3 - 2 - MAX_FRAME_LEN + 2) * 8;
                const uint32_t minAirTimeMs = (maxPacketBits * 1000) / bitrate; // ~ms

                uint32_t timeout = 3 * minAirTimeMs; // 3x margin
                // uint32_t timeout = 48; // = expectingResponse ? responseTimeoutMs : FHSS_UNLOCK_TIMEOUT_MS / 3;
                // if (fhssLockReason == FHSSLockReason::PREAMBLE_DETECTED) timeout = 48; // lock_timeout (~2× airtime max).
                // if (fhssLockReason == FHSSLockReason::RECEIVING) timeout = 48;

                if (elapsed > timeout) {
                    // ets_printf("(D) FHSS timeout, forcing unlock (reason: %s, elapsed: %dms)\n", fhssLockReasonToString(fhssLockReason), elapsed);
                    fhssLockReason = FHSSLockReason::NONE;
                    f_lock_hop = false;
                    expectingResponse = false;
                }
            }
            xSemaphoreGive(fhss_state_mutex);
        }
    }

    /**
     * Mettre à jour l'horodatage d'activité
     * Pas besoin de mutex pour juste mettre à jour un uint32_t
     */
    // void iohcRadio::updateFHSSActivity() {
    //     if (xSemaphoreTake(fhss_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    //         lastActivityTime = millis();
    //         xSemaphoreGive(fhss_state_mutex);
    //     }
    // }
    inline void IRAM_ATTR iohcRadio::updateFHSSActivity() {
        lastActivityTime = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL); // micro→ms
    }
    /**
 * Indique qu'on attend une réponse après un envoi
 */
    void iohcRadio::setExpectingResponse(bool expecting, uint32_t timeoutMs = 1000) {
        if (xSemaphoreTake(fhss_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            expectingResponse = expecting;
            responseTimeoutMs = timeoutMs;
            if (expecting) {
                lockFHSS(FHSSLockReason::WAITING_RESPONSE);
            }
            xSemaphoreGive(fhss_state_mutex);
        }
    }


    iohcRadio::iohcRadio() {
        Radio::initHardware();
        Radio::calibrate();

        Radio::initRegisters(MAX_FRAME_LEN);
        Radio::setCarrier(Radio::Carrier::Deviation, 19200);
        Radio::setCarrier(Radio::Carrier::Bitrate, 38400);
        Radio::setCarrier(Radio::Carrier::Bandwidth, 250);
        Radio::setCarrier(Radio::Carrier::Modulation, Radio::Modulation::FSK);

        // Attach interrupts to Preamble detected and end of packet sent/received
        //        attachInterrupt(RADIO_PACKET_AVAIL, i_payload, CHANGE); //
        //        attachInterrupt(RADIO_PREAMBLE_DETECTED, i_preamble, CHANGE); //
        attachInterrupt(RADIO_DIO0_PIN, handle_interrupt_fromisr, RISING); //CHANGE); //
        //        attachInterrupt(RADIO_DIO1_PIN, handle_interrupt_fromisr, RISING); // CHANGE); //
        attachInterrupt(RADIO_DIO2_PIN, handle_interrupt_fromisr, RISING); //CHANGE); //

        // start state machine
        ets_printf("Starting Interrupt Handler...\n");
        // Créer les mutex
        tx_mutex = xSemaphoreCreateMutex(); xSemaphoreGive(tx_mutex);
        // txQueue_mutex = xSemaphoreCreateMutex(); xSemaphoreGive(txQueue_mutex);
        txQueue_binary_sem = xSemaphoreCreateBinary(); xSemaphoreGive(txQueue_binary_sem); // Initialement libre
        fhss_state_mutex = xSemaphoreCreateMutex(); xSemaphoreGive(fhss_state_mutex);

        // Créer la queue ISR -> task
        radioIrqQueue = xQueueCreate(RADIO_IRQ_QUEUE_LEN, sizeof(RadioIrqEvent));
        // if (!radioIrqQueue) {
        //     ets_printf("ERROR: radioIrqQueue creation failed\n");
        // }


        // Créer la queue de paquets
        packetQueue = xQueueCreate(PACKET_QUEUE_SIZE, sizeof(iohcPacket));
        if (!packetQueue) {
            ets_printf("Failed to create packet queue\n");
        }

        lastPacketMutex = xSemaphoreCreateMutex();

        // Démarrer la task de traitement des paquets
        startPacketProcessor();

        BaseType_t task_code = xTaskCreatePinnedToCore(handle_interrupt_task, "handle_interrupt_task", 8192,
                                                       this, 4,
                                                       &handle_interrupt, xPortGetCoreID());
        if (task_code != pdPASS) {
            ets_printf("ERROR STATEMACHINE Can't create task %d\n", task_code);
            return;
        }
    }

    /**
     * @brief Returns a pointer to a single instance of the `iohcRadio`
     * class, creating it if it doesn't already exist.
     *
     * @return An instance of the `iohcRadio` class.
     */
    iohcRadio *iohcRadio::getInstance() {
        if (!_iohcRadio)
            _iohcRadio = new iohcRadio();
        return _iohcRadio;
    }

    /**
     * Initializes the radio with specified parameters and sets it to receive mode.
     *
     * @param num_freqs Number of
     * frequencies to scan. It is of type `uint8_t`. This
     * parameter specifies how many frequencies the radio will scan during operation.
     * @param scan_freqs array of `uint32_t` values that represent the
     * frequencies to be scanned during the radio operation. The `start` function initializes the radio
     * with the provided frequencies for scanning.
     * @param scanTimeUs time interval in microseconds for scanning frequencies. If a specific value is
     * provided for `scanTimeUs`, it will be used as the scan interval. Otherwise, the default scan
     * interval defined as
     * @param rxCallback The `rxCallback` is a delegate or
     * function pointer that will be called when a packet is received by the radio. It is set to `nullptr`
     * by default if not provided during the function call.
     * @param txCallback The `txCallback` is a callback function that will be called when a packet is
     * transmitted by the radio. This callback function can be provided by the user
     */
    void iohcRadio::start(uint8_t num_freqs, uint32_t *scan_freqs, uint32_t scanTimeUs,
                          CallbackFunction rxCallback = nullptr, CallbackFunction txCallback = nullptr) {
        this->num_freqs = num_freqs;
        this->scan_freqs = scan_freqs;
        // this->scanTimeUs = scanTimeUs ? scanTimeUs : DEFAULT_SCAN_INTERVAL_US;
        this->rxCB = std::move(rxCallback);
        this->txCB = std::move(txCallback);
        this->currentFreqIdx = 0;
        this->expectingResponse = false;

        // CONFIGURATION PAR DÉFAUT RECOMMANDÉE
        if (scanTimeUs == 0) {
            // Démarrer en mode fast scan
            this->scanTimeUs = 15 * 1000; // 15ms par fréquence
            // ESP_LOGI("FHSS", "Using default adaptive FHSS: 15ms fast scan");
        } else {
            this->scanTimeUs = scanTimeUs;
        }

        Radio::clearBuffer();
        Radio::clearFlags();

        /* We always start at freq[0], the 1W/2W channel*/
        Radio::setCarrier(Radio::Carrier::Frequency, scan_freqs[0]); // CHANNEL2); // 868950000);

        if (this->num_freqs > 1) {
            this->currentFreqIdx = 0;

            // start adaptative
            adaptiveFHSS = new AdaptiveFHSS(this);

            // Start Frequency Hopping Timer
            ets_printf("Starting FHSSTimer Handler...\n");
            adaptiveFHSS->switchToFastScan();
        }
        Radio::setRx();
    }

    /**
     * Handles various radio operations based on different conditions
     * and configurations for SX127X
     *
     * @param radio Pointer to an
     * instance of the `iohcRadio` class. This pointer is used to access and modify the properties and
     * methods of the `iohcRadio` object within the function.
     *
     * Ne rien faire dans l'ISR qui puisse allouer, prendre mutex/sem, ou appeler des fonctions non-ISR-safe.
     * Faire en sorte que tickerCounter() ne fasse que le minimum : lire les IRQ, mettre à jour lastActivityTime, écrire un flag / envoyer une notification vers une task (via xTaskNotifyFromISR() ou xQueueSendFromISR()), et retourner.
     * Tout le traitement (lecture FIFO, allocation, vector insert) doit être fait dans une task FreeRTOS.
     */

    void IRAM_ATTR iohcRadio::tickerCounter(iohcRadio *radio) {
        Radio::readBytes(REG_IRQFLAGS1, _flags, sizeof(_flags));

        if (radioState == iohcRadio::RadioState::PREAMBLE) {
            // Préambule détecté, déjà verrouillé dans l'ISR
        } else if (radioState == iohcRadio::RadioState::PAYLOAD) {
            // if TX ready?
            if (_flags[0] & RF_IRQFLAGS1_TXREADY) {
                radio->sent(radio->new_packet);
                Radio::clearFlags();
                if (radioState != iohcRadio::RadioState::TX) {
                    Radio::setRx();
                    radio->setRadioState(iohcRadio::RadioState::RX);

                    // Only when no response expected
                    if (/*!radio->expectingResponse &&*/ !txMode) {
                        radio->unlockFHSS(FHSSLockReason::TRANSMITTING);
                    }
                }
                return;
            }
            // Réception de payload
            // maybeCheckHeap("BEFORE RECEIVE IN TICKERCOUNTER");
            radio->receive(false);
            // maybeCheckHeap("AFTER RECEIVE IN TICKERCOUNTER");
            Radio::clearFlags();

            // radio->tickCounter = 0;
            // Après réception, Only when no response expected

            // if (!radio->expectingResponse) {
            // Petit délai avant de déverrouiller pour laisser le temps de traiter
            // et éventuellement envoyer une réponse
            // vTaskDelay(pdMS_TO_TICKS(50));
            // radio->forceUnlockFHSS();
            // }
        }

    }
    /**
         * Crée un wrapper de paquet de manière SÉCURISÉE
         * Retourne nullptr si allocation échoue (au lieu de crasher)
         */
    static TxPacketWrapper* IRAM_ATTR createPacketWrapper(iohcPacket* sourcePacket) {
        if (!sourcePacket) {
            // ets_printf("createPacketWrapper: sourcePacket is null\n");
            return nullptr;
        }

        // Essayer d'allouer d'abord la copie du paquet
        iohcPacket* pktToSend = nullptr;

        pktToSend = new (std::nothrow) iohcPacket(*sourcePacket);
        if (!pktToSend) {
            // ets_printf("createPacketWrapper: Failed to allocate packet copy\n");
            return nullptr;  // Fuite zéro - rien n'a été alloué
        }

        // Ensuite allouer le wrapper
        auto wrapper = new (std::nothrow) TxPacketWrapper(pktToSend);
        if (!wrapper) {
            // ets_printf("createPacketWrapper: Failed to allocate wrapper\n");
            // Libérer le paquet
            delete pktToSend;
            return nullptr;  // Fuite zéro
        }

        return wrapper;
    }
    /**
 * Sends packets stored in a vector with a specified repeat time.
 *
 * @param TxPackets `Vector of pointers to `iohcPacket` objects.
*
 * Envoie un ou plusieurs paquets
 * Les paquets sont COPIÉS en interne, on garde la propriété des originaux
 */
    bool iohcRadio::send(std::vector<iohcPacket *> &TxPackets) {
        if (TxPackets.empty()) {
            ets_printf("Send called with empty packet list");
            return false;
        }
        if (xSemaphoreTake(txQueue_binary_sem, pdMS_TO_TICKS(100)) != pdTRUE) {
            return false;
        }

        txCounter = 0;
        // Ajouter chaque paquet à la queue (en faisant une COPIE)
        for (auto *pkt: TxPackets) {
            if (pkt) {
                // Utiliser la factory
                if (TxPacketWrapper* wrapper = createPacketWrapper(pkt)) {
                    txQueue.push_back(wrapper);
                    pkt = nullptr; // Transfer ownership
                }
            }
        }
        TxPackets.clear();

        startTransmission();
        xSemaphoreGive(txQueue_binary_sem);
        return true;

    }

    /**
     * Send a single packet, to help reveiced the answer in another frequency
     */
    bool iohcRadio::sendSingle(iohcPacket *packet) {
        if (!packet) return false;
        if (xSemaphoreTake(txQueue_binary_sem, pdMS_TO_TICKS(100)) != pdTRUE) {
            return false;
        }
        if (TxPacketWrapper* wrapper = createPacketWrapper(packet)) txQueue.push_back(wrapper);
        else {
            xSemaphoreGive(txQueue_binary_sem);
            // ets_printf("sendSingle: Failed to create packet wrapper");
            return false;
        }

        startTransmission();
        xSemaphoreGive(txQueue_binary_sem);
        return true;
    }

    /**
    * Insère un paquet en priorité (au début de la queue)
    * Pour les réponses urgentes
    */
    bool iohcRadio::sendPriority(iohcPacket *packet) {
        if (!packet) return false;
        if (xSemaphoreTake(txQueue_binary_sem, pdMS_TO_TICKS(100)) != pdTRUE) {
            return false;
        }

        // O(1)
        if (TxPacketWrapper* wrapper = createPacketWrapper(packet)) txQueue.push_front(wrapper);
        else {
            xSemaphoreGive(txQueue_binary_sem);
        return false;
        }

        // txQueue_busy = false;
        startTransmission();
        xSemaphoreGive(txQueue_binary_sem);
        return true;
    }

    /**
     * Handles the transmission of packets using radio
     * communication, including frequency setting, packet preparation, and handling of repeated
     * transmissions.
     *
     * @param radio Pointer to an object of type
     * `iohcRadio`. It is used to access and manipulate data and functions within the `iohcRadio` class.
     */
    void IRAM_ATTR iohcRadio::packetSender(iohcRadio *radio) {
        if (!radio || !radio->currentTxPacket) {
            ets_printf("packetSender called with null pointer");
            return;
        }

        digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);

        // Verrouiller pour transmission
        radio->lockFHSS(FHSSLockReason::TRANSMITTING);
        txMode = true;

        iohcPacket *pkt = radio->currentTxPacket->packet;

        // Use the frequency of the packet if specified. 1W is always set at CHANNEL2
        if (pkt->frequency == 0) pkt->frequency = radio->scan_freqs[radio->currentFreqIdx];;
        if (pkt->frequency != radio->scan_freqs[radio->currentFreqIdx]) {
            Radio::setCarrier(Radio::Carrier::Frequency, pkt->frequency);
        } else {
            pkt->frequency = radio->scan_freqs[radio->currentFreqIdx];
        }

        Radio::setStandby();
        Radio::clearFlags();
        // Not needed at start as already done at init
        // Radio::setPreambleLength(LONG_PREAMBLE_MS); //repeat?LONG_PREAMBLE_MS:SHORT_PREAMBLE_MS);
        Radio::writeBytes(REG_FIFO, pkt->payload.buffer, pkt->buffer_length);
        Radio::setTx();
        radio->setRadioState(iohcRadio::RadioState::TX);
        // There is no need to maintain radio locked between packets transmission unless clearly asked
        txMode = pkt->lock;

        packetStamp = esp_timer_get_time();

        pkt->decode(true);
        // Memorize last command sent
        IOHC::lastCmd = pkt->cmd();
        // Used for replies in main
        // iohcCozyDevice2W *cozyDevice2W = iohcCozyDevice2W::getInstance();
        // cozyDevice2W->memorizeSend.memorizedCmd = IOHC::lastCmd;
        // cozyDevice2W->memorizeSend.memorizedData = {pkt->payload.buffer+9, pkt->payload.buffer+radio->new_packet->buffer_length};
        // cozyDevice2W->memorizeSend.memorizedData.assign(pkt->payload.buffer+9, pkt->payload.buffer+pkt->buffer_length);
        // Vérifier si ce paquet attend une réponse
        // (vous pouvez ajouter un flag dans iohcPacket pour ça)
        // bool thisPacketExpectsResponse = pkt->expectsResponse;  // À ajouter dans iohcPacket

        if (pkt->repeat > 0) {
            // Only the first frame is LPM (1W)
            pkt->payload.packet.header.CtrlByte2.asStruct.LPM = 0;
            pkt->repeat--;
        }
        if (pkt->repeat == 0) {
            radio->Ticker.detach();

            // spinlock au lieu de sémaphore
            uint32_t timeout = esp_timer_get_time() + 50000;  // 50ms en microsecondes
            while (radio->txQueue_busy && esp_timer_get_time() < timeout) {
                // Spinlock court - OK dans ISR
            }

            // Vérifier s'il y a un délai avant le prochain paquet
            bool hasNextPacket = false;
            uint32_t nextDelay = 0;

            // VÉRIFICATION RAPIDE sans modifier la queue
            if (!radio->txQueue_busy && !radio->txQueue.empty()) {
                hasNextPacket = true;
                nextDelay = radio->txQueue.front()->repeatTime;
            }

            if (hasNextPacket && nextDelay > 0) {
                // Délai avant le prochain paquet
                radio->Ticker.delay_ms(nextDelay, processNextPacketCallback, radio);
            } else if (hasNextPacket) {
                // Pas de délai, traiter immédiatement
                radio->processNextPacket();
            } else {
                // Plus de paquets à envoyer
                txMode = false;

                radio->unlockFHSS(FHSSLockReason::TRANSMITTING);
                radio->stopTransmission();
            }
        }

        digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);
    }

    /**
     * Checks if a callback function `txCB` is set and calls
     * it with a packet as a parameter, returning the result.
     *
     * @param packet The `packet` parameter is a pointer to an object of type `iohcPacket`.
     *
     * @return  boolean , which is determined by the result of
     * calling the `txCB` function with the `packet` parameter. If `txCB` is not null, the return value
     * will be the result of calling `txCB(packet)`, otherwise it will be `false`.
     */
    bool IRAM_ATTR iohcRadio::sent(iohcPacket *packet) {
        bool ret = false;
        if (txCB) ret = txCB(packet);
        return ret;
    }

    void iohcRadio::startPacketProcessor() {
    BaseType_t result = xTaskCreatePinnedToCore(
        packetProcessorTask,   // Function
        "PacketProcessor",     // Name
        4096,                  // Stack size
        this,                  // Parameter
        5,                     // Priority (au-dessus de la task IRQ)
        &packetProcessorTaskHandle,
        xPortGetCoreID()
    );

    if (result != pdPASS) {
        ets_printf("Failed to create packet processor task");
    }
}

void iohcRadio::packetProcessorTask(void* parameter) {

    iohcRadio* radio = static_cast<iohcRadio*>(parameter);
    iohcPacket receivedPacket;

    ets_printf("Packet processor task started");

    while (true) {
        // Attendre un paquet (bloquant)
        if (xQueueReceive(radio->packetQueue, &receivedPacket, portMAX_DELAY) == pdTRUE) {
            // **VALIDATION SANS INTERRUPTION DU FLUX**
            bool shouldDecode = true;
            std::string rejectReason;

            // Critères de rejet (mais on maintient le flux)
            if (receivedPacket.buffer_length == 0) {
                shouldDecode = false;
                rejectReason = "Empty packet";
            }
            else if (receivedPacket.buffer_length < 9/*MIN_PACKET_LENGTH*/) {
                shouldDecode = false;
                rejectReason = "Too short";
            }

            // Vérifier l'intégrité du paquet
            // if (receivedPacket.buffer_length == 0 || receivedPacket.buffer_length > MAX_FRAME_LEN) {
            //     ets_printf("Invalid packet length: %d\n", receivedPacket.buffer_length);
            //     continue;
            // }

            // Vérifier les doublons
            bool isDuplicate = false;
            if (shouldDecode)
                if (xSemaphoreTake(radio->lastPacketMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    if (receivedPacket == last1wPacket && receivedPacket.is1W()) {
                        isDuplicate = true;
                        rejectReason = "Duplicate";
                        shouldDecode = false;
                    } else {
                        // Sauvegarder le nouveau paquet
                        last1wPacket = receivedPacket;
                    }
                    xSemaphoreGive(radio->lastPacketMutex);
                }

            // Traiter le paquet
            // **TOUJOURS logger, même les rejets**
            if (!shouldDecode) {
                ets_printf("Packet rejected: %s, len=%d\n",  rejectReason.c_str(), receivedPacket.buffer_length);
                // Option: callback pour paquets rejetés
                // if (radio->rxCB) {
                    // Peut-être un callback différent pour les erreurs?
                    // radio->rxErrorCB(&receivedPacket, rejectReason);
                // }
            } else {
            // Appeler le callback
                if (radio->rxCB) {
                    radio->rxCB(&receivedPacket);
                }

                // Décoder le paquet
                receivedPacket.decode(true);
                //ets_printf("Packet processed: len=%d, freq=%lu\n", receivedPacket.buffer_length, receivedPacket.frequency);
            }
            // **MAINTENIR L'ÉTAT FHSS**
            // S'assurer que le FHSS n'est pas bloqué
            radio->checkFHSSTimeout(); // Force un check après traitement
        }
    }
}

    bool IRAM_ATTR iohcRadio::receive(bool stats = false) {
        digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);

        // Utiliser un buffer TEMPORAIRE local
        tempRxPacket.reset();
        tempRxPacket.frequency = scan_freqs[currentFreqIdx];

        _g_payload_millis = esp_timer_get_time();
        packetStamp = _g_payload_millis;

        if (stats) {
            tempRxPacket.rssi = static_cast<float>(Radio::readByte(REG_RSSIVALUE)) / -2.0f;
            int16_t thres = Radio::readByte(REG_RSSITHRESH);
            tempRxPacket.snr = tempRxPacket.rssi > thres ? 0 : (thres - tempRxPacket.rssi);
            int16_t f = static_cast<uint16_t>(Radio::readByte(REG_AFCMSB));
            f = (f << 8) | static_cast<uint16_t>(Radio::readByte(REG_AFCLSB));
            tempRxPacket.afc = f * 61.0;
        }

        // Lire les données
        bool overflow = false;
        while (Radio::dataAvail()) {
            if (tempRxPacket.buffer_length < MAX_FRAME_LEN) {
                tempRxPacket.payload.buffer[tempRxPacket.buffer_length++] = Radio::readByte(REG_FIFO);
            } else {
                Radio::readByte(REG_FIFO);
                overflow = true;
            }
        }


        // **CRITIQUE: Toujours compléter le cycle FHSS même pour les paquets rejetés**
        // Marquer la fin de la réception pour le FHSS
        // bool shouldProcess = true;

        // Vérifier que nous avons un paquet valide
        // if (tempRxPacket.buffer_length == 0) {
            // digitalWrite(RX_LED, false);
            // shouldProcess = false;
        // }

        // **TOUJOURS envoyer à la queue, même les paquets "invalides"**
        // Les décisions de rejet se font dans le processor, pas ici
        // Envoyer une COPIE dans la queue (thread-safe)
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        BaseType_t queueResult = xQueueSendFromISR(packetQueue, &tempRxPacket, &xHigherPriorityTaskWoken);

        // if (queueResult == pdTRUE) {
        //     ets_printf("Packet queued: %d bytes\n", tempRxPacket.buffer_length);
        // } else {
        //     ets_printf("Packet queue full, dropped: %d bytes\n", tempRxPacket.buffer_length);
        // }

        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

        // **DÉVERROUILLAGE FHSS - TOUJOURS faire après réception**
        // Même pour les paquets vides/invalides, on doit déverrouiller
        // if (fhssLockReason == FHSSLockReason::PREAMBLE_DETECTED ||
        //     fhssLockReason == FHSSLockReason::RECEIVING) {
        //     unlockFHSS(fhssLockReason);
        //     }
        unlockFHSS(fhssLockReason);

        digitalWrite(RX_LED, false);
        return (queueResult == pdTRUE);
    }

    // HELPER : Détecter si un paquet est une réponse
    bool iohcRadio::isResponsePacket(iohcPacket *packet) {
        // À adapter
        // Vérifier si c'est une réponse au dernier cmd envoyé
        if (!packet) return false;

        // Exemple : si le paquet contient un ACK ou une réponse au dernier cmd
        return (packet->payload.packet.header.cmd == IOHC::lastCmd + 1) /*||
               (packet->payload.packet.header.CtrlByte1.asStruct.type == PACKET_TYPE_RESPONSE)*/;
    }

    /**
     * @deprecated The function `i_preamble` sets the value of `f_lock` based on the state of `_g_preamble`
     */
    // void IRAM_ATTR iohcRadio::i_preamble() {
    //     _g_preamble = digitalRead(RADIO_PREAMBLE_DETECTED);
    //     f_lock = _g_preamble;
    //     iohcRadio::setRadioState(_g_preamble ? iohcRadio::RadioState::PREAMBLE : iohcRadio::RadioState::RX);
    // }

    /**
     * @deprecated The function `iohcRadio::i_payload()` reads the value of a digital pin and stores it in a variable
     * `_g_payload`
     */
    // void IRAM_ATTR iohcRadio::i_payload() {
    //     _g_payload = digitalRead(RADIO_PACKET_AVAIL);
    //     iohcRadio::setRadioState(_g_payload ? iohcRadio::RadioState::PAYLOAD : iohcRadio::RadioState::RX);
    // }


    void IRAM_ATTR iohcRadio::setRadioState(const RadioState newState) {
        radioState = newState;
    }

    /**
* Callback statique pour le délai entre paquets
*/
    void IRAM_ATTR iohcRadio::processNextPacketCallback(iohcRadio *radio) {
        if (radio) {
            radio->processNextPacket();
        }
    }

    /**
* Démarre la transmission si pas déjà en cours
*/
    void iohcRadio::startTransmission() {
        if (xSemaphoreTake(tx_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            return;
        }

        if (!isSending) {
            isSending = true;
            xSemaphoreGive(tx_mutex);

            // Lancer le premier paquet
            processNextPacket();
        } else {
            xSemaphoreGive(tx_mutex);
        }
    }

    /**
     * Traite le prochain paquet de la queue
     */
    bool iohcRadio::processNextPacket() {
        // Nettoyer le paquet précédent
        if (currentTxPacket) {
            delete currentTxPacket;
            currentTxPacket = nullptr;
        }

        if (txQueue.empty()) {
            stopTransmission();
            return false;
        }

        currentTxPacket = txQueue.front();
        txQueue.pop_front();

        // Lancer le timer pour ce paquet
        Ticker.attach_ms(currentTxPacket->repeatTime, packetSender, this);

        return true;
    }

    /**
     * Arrête la transmission
     */
    void iohcRadio::stopTransmission() {
        if (xSemaphoreTake(tx_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            isSending = false;
            Ticker.detach();

            // Nettoyer le paquet courant
            if (currentTxPacket) {
                delete currentTxPacket;
                currentTxPacket = nullptr;
            }

            // Déverrouiller la radio
            if (xSemaphoreTake(fhss_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                txMode = false;
                f_lock_hop = false;
                xSemaphoreGive(fhss_state_mutex);
            }

            xSemaphoreGive(tx_mutex);
        }
    }

    /**
     * Vide la queue de transmission
     */
    void iohcRadio::clearTxQueue() {
        // uint32_t timeout = millis() + 100;
        // while (txQueue_busy && millis() < timeout) {
        //     vTaskDelay(pdMS_TO_TICKS(1));
        // }
        //
        // txQueue_busy = true;

            while (!txQueue.empty()) {
                const TxPacketWrapper *wrapper = txQueue.front();
                txQueue.pop_front();
                delete wrapper; // Le destructeur libère la mémoire
            }
        // txQueue_busy = false;
    }

    /**
         * Vérifie si une transmission est en cours
         */
    bool iohcRadio::isTransmitting() const {
        bool transmitting = false;
        if (xSemaphoreTake(tx_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            transmitting = isSending;
            xSemaphoreGive(tx_mutex);
        }
        return transmitting;
    }

    /**
     * Annule toutes les transmissions en cours
     */
    void iohcRadio::cancelTransmissions() {
        Ticker.detach();
        clearTxQueue();
        stopTransmission();
    }
}

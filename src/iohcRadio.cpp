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

// Timestamps pour mesures précises
volatile uint32_t timestamp_preamble_detected = 0;
volatile uint32_t timestamp_sync_detected = 0;
volatile uint32_t timestamp_payload_ready = 0;


// trame typique FSK : preamble (variable) + sync (3 B) + header (2 B) + payload (32 B) + CRC (2 B).
// airtime (s) = ( (preamble_bytes + 3 + 2 + payload_bytes + 2) * 8 ) / bitrate
#define LONG_PREAMBLE_MS 128  // 0x80 52-206 ms
#define SHORT_PREAMBLE_MS 16 // 11.458 ms
// Timeouts adaptatifs basés sur le débit
// airtime (s) = ( (preamble_bytes + 3SYNC + 2HEAD + payload_bytes + 2CRC) * 8 ) / bitrate
const uint32_t bitrate = 38400; // 38.4 kbps
const uint32_t maxPacketBits = (SHORT_PREAMBLE_MS + 3 + 2 + MAX_FRAME_LEN + 2) * 8;
const uint32_t minAirTimeMs = (maxPacketBits * 1000) / bitrate; // ~ms

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

    void FHSSTimer(iohcRadio *iohc_radio) {
        // Check timeout
        // iohc_radio->checkFHSSTimeout();

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
    }

    /**
    * Lock Fhss for a specific reason
    */
    void iohcRadio::lockFHSS(const FHSSLockReason reason) {
        if (xSemaphoreTake(fhss_state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            // Garder la raison la plus "forte"
            if (reason > fhssLockReason) {
                fhssLockReason = reason;
                f_lock_hop = true;
                // fhss.fhss_lock = true;
                lastActivityTime = esp_timer_get_time();
                // ets_printf("(D) FHSS locked: %s\n", fhssLockReasonToString(reason));
            }
            xSemaphoreGive(fhss_state_mutex);
        }
    }

    /**
     * Unlock FHSS for a specific reason
     */
    void iohcRadio::unlockFHSS(const FHSSLockReason reason) {
        if (xSemaphoreTake(fhss_state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            // Déverrouiller seulement si c'est la même raison
            if (fhssLockReason == reason) {
                fhssLockReason = FHSSLockReason::NONE;
                f_lock_hop = false;
                // fhss.fhss_lock = false;
                // ets_printf("(D) FHSS unlocked: %s\n", fhssLockReasonToString(reason));
            }
            xSemaphoreGive(fhss_state_mutex);
        }
    }

    /**
    * Force Unlock FHSS
    */
    void iohcRadio::forceUnlockFHSS() {
        if (xSemaphoreTake(fhss_state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            fhssLockReason = FHSSLockReason::NONE;
            f_lock_hop = false;
            expectingResponse = false;
            xSemaphoreGive(fhss_state_mutex);
        }
    }

    /**
     * Set/Get FHSS State
     */
    void iohcRadio::setFHSSState(FHSSState newState) {
        if (xSemaphoreTake(fhss_state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            FHSSState oldState = fhssState;
            fhssState = newState;

            // Actions spécifiques selon le nouvel état
            switch (newState) {
                case FHSSState::SCANNING:
                    f_lock_hop = false; // Autoriser le hopping
                    break;
                case FHSSState::PREAMBLE_DETECTED:
                case FHSSState::RECEIVING:
                case FHSSState::PROCESSING:
                case FHSSState::TRANSMITTING:
                    f_lock_hop = true; // Bloquer le hopping
                    break;
            }

            xSemaphoreGive(fhss_state_mutex);
        }
    }

    iohcRadio::FHSSState iohcRadio::getFHSSState() const {
        auto state = FHSSState::SCANNING;
        if (xSemaphoreTake(fhss_state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            state = fhssState;
            xSemaphoreGive(fhss_state_mutex);
        }
        return state;
    }

    const char* iohcRadio::fhssStateToString(FHSSState state) {
        switch (state) {
            case FHSSState::SCANNING: return "SCANNING";
            case FHSSState::PREAMBLE_DETECTED: return "PREAMBLE_DETECTED";
            case FHSSState::RECEIVING: return "RECEIVING";
            case FHSSState::PROCESSING: return "PROCESSING";
            case FHSSState::TRANSMITTING: return "TRANSMITTING";
            default: return "UNKNOWN";
        }
    }

    uint64_t iohcRadio::getAdaptiveTimeout(FHSSLockReason reason) const {
        switch (reason) {
            case FHSSLockReason::PREAMBLE_DETECTED:
                return 8750; // 42 bytes
            case FHSSLockReason::RECEIVING:
                return 10413;  // 50 bytes
            case FHSSLockReason::WAITING_RESPONSE:
                return responseTimeoutMs * 1000;
            default:
                return 100000; // 100ms par défaut
        }
    }
    void iohcRadio::handleFHSSTimeout() {
        FHSSLockReason oldReason = fhssLockReason;
        fhssLockReason = FHSSLockReason::NONE;
        f_lock_hop = false;
        expectingResponse = false;

        // Remettre en RX
        if (radioState != RadioState::TX) {
            Radio::setRx();
            setRadioState(RadioState::RX);
            unlockFHSS(oldReason);
            adaptiveFHSS->update();
        }

        ets_printf("FHSS Unlocked due to timeout (was: %s)\n", fhssLockReasonToString(oldReason));
    }/**
     * FHSS Watchdog
     */
    void iohcRadio::checkFHSSTimeout() {
        if (xSemaphoreTake(fhss_state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            if (f_lock_hop && fhssLockReason != FHSSLockReason::TRANSMITTING) {
                uint64_t elapsed = esp_timer_get_time() - lastActivityTime;
                uint64_t timeout = getAdaptiveTimeout(fhssLockReason);

                if (elapsed >= timeout) {
                        ets_printf("(D) FHSS timeout(%lld), forcing unlock (reason: %s, elapsed: %lldµs)\n", timeout, fhssLockReasonToString(fhssLockReason), elapsed);
                    handleFHSSTimeout();
                }
            }
            xSemaphoreGive(fhss_state_mutex);
        }
    }
    /**
     * Mettre à jour l'horodatage d'activité
     * Pas besoin de mutex pour juste mettre à jour un uint32_t
     */
    inline void IRAM_ATTR iohcRadio::updateFHSSActivity() {
        lastActivityTime = esp_timer_get_time();
    }
    /**
    * Indique qu'on attend une réponse après un envoi
    */
    void iohcRadio::setExpectingResponse(bool expecting, uint32_t timeoutMs = 1000) {
        if (xSemaphoreTake(fhss_state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            expectingResponse = expecting;
            responseTimeoutMs = timeoutMs;
            if (expecting) {
                lockFHSS(FHSSLockReason::WAITING_RESPONSE);
            }
            xSemaphoreGive(fhss_state_mutex);
        }
    }

    TaskHandle_t handle_interrupt;
    struct RadioIrqEvent {
        uint8_t source;  // 0 = DIO0 (payload), 1 = DIO2 (preamble)
        uint8_t edge;    // 0 = falling, 1 = rising
        uint32_t timestamp_us;
    };

    static QueueHandle_t radioIrqQueue = nullptr;
    static constexpr size_t RADIO_IRQ_QUEUE_LEN = 128;


    /**
    * No semaphore / lockFHSS() / updateFHSSActivity() here.
    * esp_timer_get_time() is ISR-safe ; millis() never sure.
    */
    void IRAM_ATTR handle_interrupt_fromDIO0(void* arg) {
        uint32_t now = esp_timer_get_time();

        // Only GPIOS short ops, non blocking
        RadioIrqEvent ev;
        ev.source = 0; // DIO0
        ev.edge = digitalRead(RADIO_PACKET_AVAIL) ? 1 : 0;
        ev.timestamp_us = now;

        // Send it to queue (FromISR)
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(radioIrqQueue, &ev, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    /**
    * No semaphore / lockFHSS() / updateFHSSActivity() here.
    * esp_timer_get_time() is ISR-safe ; millis() never sure.
    */
    void IRAM_ATTR handle_interrupt_fromDIO2(void* arg) {
        uint32_t now = esp_timer_get_time();

        // Only GPIOS short ops, non blocking
        RadioIrqEvent ev;
        ev.source = 1; // DIO2
        ev.edge = digitalRead(RADIO_PREAMBLE_DETECTED) ? 1 : 0;
        ev.timestamp_us = now;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(radioIrqQueue, &ev, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    void handlePreambleDetected(iohcRadio* radio, uint32_t timestamp) {
        timestamp_preamble_detected = timestamp;

        radio->lockFHSS(iohcRadio::FHSSLockReason::PREAMBLE_DETECTED);
        radio->updateFHSSActivity();
        radio->setRadioState(iohcRadio::RadioState::PREAMBLE);
        radio->setFHSSState(iohcRadio::FHSSState::PREAMBLE_DETECTED);
    }

    void handlePayloadReady(iohcRadio* radio, uint32_t timestamp) {
        timestamp_payload_ready = timestamp;
        if (iohcRadio::radioState == iohcRadio::RadioState::PREAMBLE || iohcRadio::radioState == iohcRadio::RadioState::RX) {
            radio->lockFHSS(iohcRadio::FHSSLockReason::RECEIVING);
            radio->updateFHSSActivity();
            radio->setRadioState(iohcRadio::RadioState::PAYLOAD);
            radio->tickerCounter(radio);
            }
    }

    void handlePreambleLost(iohcRadio* radio, uint32_t timestamp) {
        // Seulement si on était en train de recevoir un préambule
        if (radio->getFHSSState() == iohcRadio::FHSSState::PREAMBLE_DETECTED) {
            // C'est probablement une fausse détection ou un signal trop faible
            radio->unlockFHSS(iohcRadio::FHSSLockReason::PREAMBLE_DETECTED);
            radio->setFHSSState(iohcRadio::FHSSState::SCANNING);
            radio->setRadioState(iohcRadio::RadioState::RX);

            // Remettre la radio en état normal
            Radio::setRx();
        }
    }
    void handlePacketSent(iohcRadio* radio, uint32_t timestamp) {
        Radio::clearFlags();

        // Remettre la radio en mode RX
        Radio::setRx();
        radio->setRadioState(iohcRadio::RadioState::RX);
        radio->unlockFHSS(iohcRadio::FHSSLockReason::TRANSMITTING);
    }
    /**
    * Sequential : Read queue — no concurrencies possible.
    * tickerCounter isn't called by the ISR.
    */
    void handle_interrupt_task(void *pvParameters) {
        iohcRadio *radio = static_cast<iohcRadio*>(pvParameters);
        RadioIrqEvent evt;
        // RF_IRQFLAGS1_0xDB -> MODEREADY RXREADY PLLLOCK RSSI PREAMBLEDETECT SYNCADDRESSMATCH
        // RF_IRQFLAGS2_0x26 -> FIFOLEVEL PAYLOADREADY CRCOK
        // RF_IRQFLAGS1_0xB0 -> MODEREADY TXREADY PLLLOCK
        // RF_IRQFLAGS2_0x48 -> FIFOEMPTY PACKETSENT

        while (true) {
            if (xQueueReceive(radioIrqQueue, &evt, portMAX_DELAY) == pdTRUE) {
                switch (evt.source) {
                    case 1: // DIO2 - Preamble
                        if (evt.edge == 1) { // Rising edge = Preamble detected
                            handlePreambleDetected(radio, evt.timestamp_us);
                        } else {
                            handlePreambleLost(radio, evt.timestamp_us);
                        }
                        break;

                    case 0: // DIO0 - Payload
                        // Lire IRQFLAGS2 pour déterminer la cause
                        uint8_t irqflags2 = Radio::readByte(REG_IRQFLAGS2);
                        if (irqflags2 & RF_IRQFLAGS2_PACKETSENT) {
                            // End of transmission
                            handlePacketSent(radio, evt.timestamp_us);
                        } else if (irqflags2 & RF_IRQFLAGS2_PAYLOADREADY) {
                            // Reception Event
                            handlePayloadReady(radio, evt.timestamp_us);
                        }
                        break;
                }
            }
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

        // Configurer les interruptions avec pull-down pour éviter les faux déclenchements
        pinMode(RADIO_DIO0_PIN, INPUT); //_PULLDOWN);
        pinMode(RADIO_DIO2_PIN, INPUT); //_PULLDOWN);
        // Attacher les interruptions avec debounce matériel
        attachInterruptArg(RADIO_DIO0_PIN, handle_interrupt_fromDIO0, nullptr, RISING);
        attachInterruptArg(RADIO_DIO2_PIN, handle_interrupt_fromDIO2, nullptr, CHANGE);
        // #define GPIO_BIT_MASK  ((1ULL<<RADIO_DIO0_PIN) | (1ULL<<RADIO_DIO1) | (1ULL<<RADIO_DIO2_PIN))
        // gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        // gpio_config_t io_conf = {};
        // io_conf.intr_type = GPIO_INTR_POSEDGE;
        // io_conf.mode = GPIO_MODE_INPUT;
        // io_conf.pin_bit_mask = (1ULL<<RADIO_DIO0_PIN);
        // io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        // io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        // gpio_config(&io_conf);
        // gpio_isr_handler_add(GPIO_NUM_26 , handle_interrupt_fromDIO0, nullptr);
        // io_conf.pin_bit_mask = (1ULL<<RADIO_DIO2_PIN);
        // io_conf.intr_type = GPIO_INTR_ANYEDGE;
        // gpio_config(&io_conf);
        // gpio_isr_handler_add(GPIO_NUM_34 , handle_interrupt_fromDIO2, nullptr);

        // start state machine
        ets_printf("Starting Interrupt Handler...\n");
        // Create all mutex
        tx_mutex = xSemaphoreCreateMutex(); xSemaphoreGive(tx_mutex);
        txQueue_binary_sem = xSemaphoreCreateBinary(); xSemaphoreGive(txQueue_binary_sem); // Initialement libre
        fhss_state_mutex = xSemaphoreCreateMutex(); xSemaphoreGive(fhss_state_mutex);

        // Create ISR Queue
        radioIrqQueue = xQueueCreate(RADIO_IRQ_QUEUE_LEN, sizeof(RadioIrqEvent));

        // Create Packet Queue
        packetQueue = xQueueCreate(PACKET_QUEUE_SIZE, sizeof(iohcPacket));

        lastPacketMutex = xSemaphoreCreateMutex();

        // Start Packet processor
        startPacketProcessor();

        BaseType_t task_code = xTaskCreatePinnedToCore(handle_interrupt_task,
            "handle_interrupt_task", 8192,
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

        // Default
        if (scanTimeUs == 0) {
            this->scanTimeUs = 30 * 1000; // 15ms par fréquence
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
            adaptiveFHSS->switchToFastScan(18); // ...ms par fréquence
            ets_printf("FHSSTimer Handler Started...\n");
        }
        Radio::setRx();
    }

    /**
    */
    void IRAM_ATTR iohcRadio::tickerCounter(iohcRadio *radio) {
        Radio::readBytes(REG_IRQFLAGS1, _flags, sizeof(_flags));
        // uint32_t now = esp_timer_get_time();
        // Sync = preamble end
        if (_flags[0] & RF_IRQFLAGS1_SYNCADDRESSMATCH) {  }
        if (radioState == RadioState::PREAMBLE) {  }
        else if (radioState == RadioState::PAYLOAD) {
            // Payload ready
            radio->receive(false);
            Radio::clearFlags();
            radio->tickCounter = 0;
            if (!txMode) {
                radio->unlockFHSS(FHSSLockReason::RECEIVING); // ← Good one
            }
        }
    }

    /**
    */
    static TxPacketWrapper* IRAM_ATTR createPacketWrapper(iohcPacket* sourcePacket) {
        if (!sourcePacket) {
            return nullptr;
        }

        // Essayer d'allouer d'abord la copie du paquet
        iohcPacket* pktToSend = nullptr;

        pktToSend = new (std::nothrow) iohcPacket(*sourcePacket);
        if (!pktToSend) {
            return nullptr;  // Fuite zéro - rien n'a été alloué
        }

        // Ensuite allouer le wrapper
        auto wrapper = new (std::nothrow) TxPacketWrapper(pktToSend);
        if (!wrapper) {
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
            return false;
        }
        if (xSemaphoreTake(txQueue_binary_sem, pdMS_TO_TICKS(100)) != pdTRUE) {
            return false;
        }

        txCounter = 0;
        size_t packetsAdded = 0;
        // Ajouter chaque paquet à la queue (en faisant une COPIE)
        for (auto *pkt: TxPackets) {
            if (pkt) {
                // Utiliser la factory
                if (TxPacketWrapper* wrapper = createPacketWrapper(pkt)) {
                    txQueue.push_back(wrapper);
                    //pkt = nullptr; // Transfer ownership
                    packetsAdded++;
                }
            }
        }
        TxPackets.clear();
        xSemaphoreGive(txQueue_binary_sem);

        if (packetsAdded == 0) {
            ets_printf("send(): no packets added");
            return false;
        }

        startTransmission();
        return true;

    }

    /**
     * Send a single packet, to help reveiced the answer in another frequency
     */
    bool iohcRadio::sendSingle(iohcPacket *packet) {
        if (!packet) return false;
        if (TxPacketWrapper* wrapper = createPacketWrapper(packet))
            txQueue.push_back(wrapper);
        else {
            return false;
        }
        startTransmission();
        return true;
    }

    /**
    * Insère un paquet en priorité (au début de la queue)
    * Pour les réponses urgentes
    */
    bool iohcRadio::sendPriority(iohcPacket *packet) {
        if (!packet) return false;
        // O(1)
        if (TxPacketWrapper* wrapper = createPacketWrapper(packet)) txQueue.push_front(wrapper);
        else {
            return false;
        }
        startTransmission();
        return true;
    }

    /**

    */
    void IRAM_ATTR iohcRadio::packetSender(iohcRadio *radio) {
        if (!radio || !radio->currentTxPacket) {
            ets_printf("packetSender called with null pointer\n");
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

        Radio::writeBurst(REG_FIFO, pkt->payload.buffer, pkt->buffer_length);
        Radio::setTx();
        radio->setRadioState(iohcRadio::RadioState::TX);
        // There is no need to maintain radio locked between packets transmission unless clearly asked
        txMode = pkt->lock;

        packetStamp = esp_timer_get_time();

        pkt->decode(true);
        // Memorize last command sent
        IOHC::lastCmd = pkt->cmd();

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
                radio->Ticker.attach_ms(nextDelay, processNextPacketCallback, radio);
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
        ets_printf("Failed to create packet processor task\n");
    }
}

void iohcRadio::packetProcessorTask(void* parameter) {

    iohcRadio* radio = static_cast<iohcRadio*>(parameter);
    iohcPacket receivedPacket;

    ets_printf("Packet processor task started\n");

    while (true) {
        // Attendre un paquet (bloquant)
        if (xQueueReceive(radio->packetQueue, &receivedPacket, portMAX_DELAY) == pdTRUE) {
            // **CRITIQUE: Vérifier que le FHSS est dans l'état approprié**
            if (radio->getFHSSState() != FHSSState::PROCESSING && radio->getFHSSState() != FHSSState::RECEIVING) {
                // BUG 08:10:36.472 > Packet received in wrong FHSS state: SCANNING
                ets_printf("Packet received in wrong FHSS state: %s\n", radio->fhssStateToString(radio->getFHSSState()));
                }

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

            if (shouldDecode)
                if (xSemaphoreTake(radio->lastPacketMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    // Vérifier les doublons
                    if (receivedPacket == last1wPacket && receivedPacket.is1W()) {
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
                if (rejectReason != "Duplicate")
                ets_printf("Packet rejected: %s, len=%d\n",  rejectReason.c_str(), receivedPacket.buffer_length);
                radio->setFHSSState(FHSSState::SCANNING);
                radio->forceUnlockFHSS();
            } else {

            // Appeler le callback
                if (radio->rxCB) {
                    radio->rxCB(&receivedPacket);
                }

                // Décoder le paquet
                receivedPacket.decode(true);
            }
            // *KEEP FHSS state**
            radio->setFHSSState(FHSSState::SCANNING);

        }
    }
}

    bool IRAM_ATTR iohcRadio::receive(bool stats = false) {
        digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);
        // **CRITIQUE: Toujours commencer par mettre à jour l'état FHSS**
        // adaptiveFHSS->onPacketActivity();
        setFHSSState(FHSSState::RECEIVING);

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
        while (Radio::dataAvail()) {
            if (tempRxPacket.buffer_length < MAX_FRAME_LEN) {
                tempRxPacket.payload.buffer[tempRxPacket.buffer_length++] = Radio::readByte(REG_FIFO);
            } else {
                Radio::readByte(REG_FIFO);
            }
        }

        // **CRITIQUE: Toujours compléter le cycle FHSS même pour les paquets rejetés**
        // Marquer la fin de la réception pour le FHSS
        // **TOUJOURS envoyer à la queue, même les paquets "invalides"**
        // Les décisions de rejet se font dans le processor, pas ici
        // Envoyer une COPIE dans la queue (thread-safe)
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        BaseType_t queueResult = xQueueSendFromISR(packetQueue, &tempRxPacket, &xHigherPriorityTaskWoken);

        // DÉVERROUILLAGE IMMÉDIAT - le paquet est dans la queue
        // Ne pas attendre le traitement complet
        setFHSSState(FHSSState::PROCESSING);
        // Remettre en RX immédiatement
        Radio::setRx();
        setRadioState(RadioState::RX);

        // Déverrouiller le FHSS pour permettre le hopping
        unlockFHSS(FHSSLockReason::RECEIVING);
        adaptiveFHSS->update();

        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

        digitalWrite(RX_LED, false);
        return (queueResult == pdTRUE);
    }

    // HELPER : Détecter si un paquet est une réponse
    bool iohcRadio::isResponsePacket(iohcPacket *packet) {
        if (!packet) return false;
        return (packet->payload.packet.header.cmd == IOHC::lastCmd + 1);
    }

    /**
     * @deprecated Sets the value of `f_lock` based on the state of `_g_preamble`
     */
    // void IRAM_ATTR iohcRadio::i_preamble() {
    //     _g_preamble = digitalRead(RADIO_PREAMBLE_DETECTED);
    //     f_lock = _g_preamble;
    //     iohcRadio::setRadioState(_g_preamble ? iohcRadio::RadioState::PREAMBLE : iohcRadio::RadioState::RX);
    // }

    /**
     * @deprecated Reads the value of a digital pin and stores it in `_g_payload`
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
     * Process Next Packet in Queue
     */
    bool iohcRadio::processNextPacket() {
        // Clean current packet
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
        // Check validity
        if (!currentTxPacket || !currentTxPacket->packet) {
            ets_printf("processNextPacket(): Invalid packet!\n");
            stopTransmission();
            return false;
        }

        // Start packet timer
        uint32_t repeatTime = currentTxPacket->repeatTime;
        if (repeatTime == 0) repeatTime = 100;  // Défaut si non spécifié

        // Reattach the Ticker
        Ticker.attach_ms(repeatTime, packetSender, this);

        return true;
    }

    /**
     * Stop Transmission
     */
    void iohcRadio::stopTransmission() {
        if (xSemaphoreTake(tx_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            isSending = false;
            Ticker.detach();

            // Clean current packet
            if (currentTxPacket) {
                delete currentTxPacket;
                currentTxPacket = nullptr;
            }
            // Mark as not sending
            isSending = false;

            txMode = false;
            unlockFHSS(FHSSLockReason::TRANSMITTING);

            xSemaphoreGive(tx_mutex);
        }
    }

    /**
     * Clean TX Queue
     */
    void iohcRadio::clearTxQueue() {
        while (!txQueue.empty()) {
            const TxPacketWrapper *wrapper = txQueue.front();
            txQueue.pop_front();
            delete wrapper; // Le destructeur libère la mémoire
        }
    }

    /**
    * Check if currently transmitting
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
     * Clean TX Queue and stop transmission
     */
    void iohcRadio::cancelTransmissions() {
        Ticker.detach();
        clearTxQueue();
        stopTransmission();
    }
}

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

#define LONG_PREAMBLE_MS 206
#define SHORT_PREAMBLE_MS 52

TaskHandle_t IOHC::iohcRadio::txTaskHandle = nullptr;


// Structure pour gérer un paquet en transmission
struct TxPacketWrapper {
    IOHC::iohcPacket* packet;
    uint8_t repeatsRemaining;
    uint32_t repeatTime;
    bool ownsMemory; // Indique si on doit libérer la mémoire

    TxPacketWrapper(IOHC::iohcPacket* pkt, bool owns = true)
        : packet(pkt),
          repeatsRemaining(pkt ? pkt->repeat : 0),
          repeatTime(pkt ? pkt->repeatTime : 0),
          ownsMemory(owns) {}

    ~TxPacketWrapper() {
        if (ownsMemory && packet) {
            delete packet;
            packet = nullptr;
        }
    }
};


namespace IOHC {

    iohcRadio *iohcRadio::_iohcRadio = nullptr;
    volatile bool iohcRadio::_g_preamble = false;
    volatile bool iohcRadio::_g_payload = false;
    volatile uint32_t iohcRadio::_g_payload_millis = 0L;
    uint8_t iohcRadio::_flags[2] = {0, 0};
    volatile bool iohcRadio::f_lock_hop = false;
    volatile bool iohcRadio::send_lock = false;
    volatile bool iohcRadio::txMode = false;
    volatile iohcRadio::RadioState iohcRadio::radioState = iohcRadio::RadioState::IDLE;
    volatile bool iohcRadio::txComplete = false;

    static IOHC::iohcPacket last1wPacket;
    static bool last1wPacketValid = false;

    TaskHandle_t handle_interrupt;

    // Ajoutez un mutex/semaphore pour protéger l'accès concurrent
    static SemaphoreHandle_t fhss_mutex = nullptr;

    /**
     * TWaits for a notification and then calls the `tickerCounter`
     * function if certain conditions are met.
     *
     * @param pvParameters The `pvParameters` parameter in the `handle_interrupt_task` function is a void
     * pointer that can be used to pass any data or object to the task when it is created. In this specific
     * function, it is being cast to a pointer of type `iohcRadio` and then passed to the
     */
    void IRAM_ATTR handle_interrupt_task(void *pvParameters) {
        static uint32_t thread_notification;
        constexpr TickType_t xMaxBlockTime = pdMS_TO_TICKS(655 * 4); // 218.4 );
        while (true) {
            thread_notification = ulTaskNotifyTake(pdTRUE, xMaxBlockTime); // Wait
            if (thread_notification &&
                (iohcRadio::radioState == iohcRadio::RadioState::PAYLOAD ||
                 iohcRadio::radioState == iohcRadio::RadioState::PREAMBLE)) {
                iohcRadio::tickerCounter(static_cast<iohcRadio *>(pvParameters));
            }
        }
    }

    /**
     * Reads digital inputs and notifies a thread to wake up when
     * the interrupt service routine is complete.
     */
    void IRAM_ATTR handle_interrupt_fromisr() {
        bool preamble = digitalRead(RADIO_PREAMBLE_DETECTED);
        bool payload = digitalRead(RADIO_PACKET_AVAIL);
        if (preamble) {
            iohcRadio::_g_preamble = true;
            iohcRadio::setRadioState(iohcRadio::RadioState::PREAMBLE);
        }
        if (payload) {
            iohcRadio::_g_payload = true;
            iohcRadio::setRadioState(iohcRadio::RadioState::PAYLOAD);
        }
        // Set frequency lock/hop flag based on preamble detection
        iohcRadio::f_lock_hop = preamble;

        iohcRadio::txComplete = true;
        // Notify the thread so it will wake up when the ISR is complete
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(handle_interrupt/*_task*/, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    void FHSSTimer(iohcRadio *iohc_radio) {
        // Protection contre les accès concurrents
        if (xSemaphoreTake(fhss_mutex, 0) != pdTRUE) {
            return; // Ressayer au prochain tick
        }

        // Ne pas sauter de fréquence si :
        // 1. Le hopping est verrouillé
        // 2. La radio n'est pas en mode RX
        // 3. La radio est en train de traiter un paquet
        if (iohc_radio->f_lock_hop || iohcRadio::radioState != iohcRadio::RadioState::RX || iohcRadio::txMode) {
            xSemaphoreGive(fhss_mutex);
            return;
            }
        // Incrémenter l'index de fréquence
        iohc_radio->currentFreqIdx++;
        if (iohc_radio->currentFreqIdx >= iohc_radio->num_freqs) {
            iohc_radio->currentFreqIdx = 0;
        }

        // Changer la fréquence seulement si on est en RX
        Radio::setCarrier(Radio::Carrier::Frequency,  iohc_radio->scan_freqs[iohc_radio->currentFreqIdx]);

        xSemaphoreGive(fhss_mutex);
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
        BaseType_t task_code = xTaskCreatePinnedToCore(handle_interrupt_task, "handle_interrupt_task", 8192,
                                                       this, /*tskIDLE_PRIORITY*/4,
                                                       &handle_interrupt, /*tskNO_AFFINITY*/xPortGetCoreID());
        if (task_code != pdPASS) {
            ets_printf("ERROR STATEMACHINE Can't create task %d\n", task_code);
            // sx127x_destroy(device);
            return;
        }
        // Créer le mutex pour FHSS
        fhss_mutex = xSemaphoreCreateMutex();
        if (fhss_mutex == nullptr) {
            ESP_LOGE("IOHC", "Failed to create FHSS mutex");
        }
        // Créer les mutex
        txQueue_mutex = xSemaphoreCreateMutex();
        tx_mutex = xSemaphoreCreateMutex();

        if (!txQueue_mutex || !tx_mutex) {
            ESP_LOGE("IOHC", "Failed to create TX mutexes");
        }

    }

    /**
     * @brief The function `iohcRadio::getInstance()` returns a pointer to a single instance of the `iohcRadio`
     * class, creating it if it doesn't already exist.
     *
     * @return An instance of the `iohcRadio` class is being returned.
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
        this->scanTimeUs = scanTimeUs ? scanTimeUs : DEFAULT_SCAN_INTERVAL_US;
        this->rxCB = std::move(rxCallback);
        this->txCB = std::move(txCallback);

        Radio::clearBuffer();
        Radio::clearFlags();

        /* We always start at freq[0], the 1W/2W channel*/
        Radio::setCarrier(Radio::Carrier::Frequency, CHANNEL2); // scan_freqs[0]); //868950000);
        Radio::setRx();

        if (this->num_freqs > 1) {
            this->currentFreqIdx = 0;
            // this->tickCounter = 0;
            // Start Frequency Hopping Timer
            printf("Starting FHSSTimer Handler...\n");
            this->TickTimer.attach_us(this->scanTimeUs, FHSSTimer, this);
        }
    }


/**
 * Handles various radio operations based on different conditions
 * and configurations for SX127X
 * 
 * @param radio The `radio` parameter in the `iohcRadio::tickerCounter` function is a pointer to an
 * instance of the `iohcRadio` class. This pointer is used to access and modify the properties and
 * methods of the `iohcRadio` object within the function. The function uses this pointer
 */
    void IRAM_ATTR iohcRadio::tickerCounter(iohcRadio *radio) {
        Radio::readBytes(REG_IRQFLAGS1, _flags, sizeof(_flags));

        if (radioState == iohcRadio::RadioState::PREAMBLE) {
            // Verrouiller le hopping dès qu'un préambule est détecté
            if (xSemaphoreTake(fhss_mutex, 0) == pdTRUE) {
                f_lock_hop = true;
                xSemaphoreGive(fhss_mutex);
            }
            // radio->tickCounter = 0;
            // radio->preambleCounter = radio->preambleCounter + 1;

            // if (_flags[0] & RF_IRQFLAGS1_SYNCADDRESSMATCH) radio->preCounter = 0;
            // In case of Sync received resets the preamble duration
            // if ((preambleCounter * SM_GRANULARITY_US) >= SM_PREAMBLE_RECOVERY_TIMEOUT_US) {
            //     // Avoid hanging on a too long preamble detect
            //     Radio::clearFlags();
            //     // radio->preambleCounter = 0;
            // }
        } else
            if (radioState == iohcRadio::RadioState::PAYLOAD) {
                // if TX ready?
            if (_flags[0] & RF_IRQFLAGS1_TXREADY) {
                radio->sent(radio->new_packet);
                Radio::clearFlags();
                if (radioState != iohcRadio::RadioState::TX) {
                    Radio::setRx();
                    radio->setRadioState(iohcRadio::RadioState::RX);

                    // Déverrouiller seulement si pas en mode TX verrouillé
                    if (!txMode && xSemaphoreTake(fhss_mutex, 0) == pdTRUE) {
                        f_lock_hop = false;
                        xSemaphoreGive(fhss_mutex);
                    }
                }
                return;
            }
            // packet reçu
            radio->receive(false);
            Radio::clearFlags();
            radio->tickCounter = 0;
            return;
        }

        // radio->tickCounter = radio->tickCounter + 1;
        // if (radio->tickCounter * SM_GRANULARITY_US < radio->scanTimeUs) return;
        //
        // radio->tickCounter = 0;
    }

    /**
         * Envoie un seul paquet (plus simple pour les réponses)
         */
    bool iohcRadio::sendSingle(iohcPacket* packet, bool copyPacket = true) {
        if (!packet) return false;

        if (xSemaphoreTake(txQueue_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGE("IOHC", "Failed to acquire txQueue mutex");
            return false;
        }

        iohcPacket* pktToSend = copyPacket ? new iohcPacket(*packet) : packet;
        TxPacketWrapper* wrapper = new TxPacketWrapper(pktToSend, copyPacket);
        txQueue.push(wrapper);

        xSemaphoreGive(txQueue_mutex);

        startTransmission();
        return true;
    }

    /**
 * Insère un paquet en priorité (au début de la queue)
 * Utile pour les réponses urgentes
 */
    bool iohcRadio::sendPriority(iohcPacket* packet, bool copyPacket = true) {
        if (!packet) return false;

        if (xSemaphoreTake(txQueue_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGE("IOHC", "Failed to acquire txQueue mutex");
            return false;
        }

        // Créer une nouvelle queue temporaire
        std::queue<TxPacketWrapper*> tempQueue;

        // Ajouter le paquet prioritaire
        iohcPacket* pktToSend = copyPacket ? new iohcPacket(*packet) : packet;
        TxPacketWrapper* wrapper = new TxPacketWrapper(pktToSend, copyPacket);
        tempQueue.push(wrapper);

        // Copier le reste de la queue
        while (!txQueue.empty()) {
            tempQueue.push(txQueue.front());
            txQueue.pop();
        }

        // Remplacer la queue
        txQueue = tempQueue;

        xSemaphoreGive(txQueue_mutex);

        startTransmission();
        return true;
    }




    /**
     * Sends packets stored in a vector with a specified repeat time.
     *
     * @param TxPackets `iohcTx` is a reference to a vector of pointers to `iohcPacket` objects.
    *
     * Envoie un ou plusieurs paquets
     * Les paquets sont COPIÉS en interne, on garde la propriété des originaux
     */
    bool iohcRadio::send(std::vector<iohcPacket *> &TxPackets) {
        if (TxPackets.empty()) return false;
        // Prendre le mutex de la queue
        if (xSemaphoreTake(txQueue_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGE("IOHC", "Failed to acquire txQueue mutex");
            return false;
        }

        // packets2send = TxPackets; //std::move(iohcTx); // NO !!
        // TxPackets.clear();
        txCounter = 0;
        // Ajouter chaque paquet à la queue (en faisant une COPIE)
        for (auto* pkt : TxPackets) {
            if (pkt) {
                // Créer une COPIE du paquet
                iohcPacket* packetCopy = new iohcPacket(*pkt);
                TxPacketWrapper* wrapper = new TxPacketWrapper(packetCopy, true);
                txQueue.push(wrapper);
            }
        }
        xSemaphoreGive(txQueue_mutex);
        startTransmission();
        return true;

        // Lancer le timer pour ce paquet
//        Ticker.attach_ms(packets2send[txCounter]->repeatTime, packetSender, this);
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
            ESP_LOGE("IOHC", "packetSender called with null pointer");
            return;
        }

        digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);

        // Verrouiller le hopping pendant la transmission
        if (xSemaphoreTake(fhss_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            f_lock_hop = true; // Avoid Radio put in Rx mode at next packet sent/received
            txMode = true;
            xSemaphoreGive(fhss_mutex);
        }
        iohcPacket* pkt = radio->currentTxPacket->packet;

        // Using Delayed Packet radio->txCounter
        // if (radio->packets2send[radio->txCounter] == nullptr) {
        //     // Plus de Delayed Packet
        //     if (radio->delayed != nullptr)
        //         // Use Saved Delayed Packet
        //             radio->iohc = radio->delayed;
        // } else
//            pkt = radio->packets2send[radio->txCounter];

        // Use the frequency of the packet if specified. 1W is always set at CHANNEL2
        if (pkt->frequency != radio->scan_freqs[radio->currentFreqIdx]) {
            Radio::setCarrier(Radio::Carrier::Frequency, pkt->frequency);
        } else {
            pkt->frequency = radio->scan_freqs[radio->currentFreqIdx];
        }

        Radio::setStandby();
        Radio::clearFlags();
        // Radio::setPreambleLength(pkt->repeat?LONG_PREAMBLE_MS:SHORT_PREAMBLE_MS);
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

        if (pkt->repeat > 0) {
            // Only the first frame is LPM (1W)
            pkt->payload.packet.header.CtrlByte2.asStruct.LPM = 0;
            pkt->repeat--;
        }
        if (pkt->repeat == 0) {
            radio->Ticker.detach();
            // Vérifier s'il y a un délai avant le prochain paquet
            bool hasNextPacket = false;
            uint32_t nextDelay = 0;

            if (xSemaphoreTake(radio->txQueue_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (!radio->txQueue.empty()) {
                    hasNextPacket = true;
                    nextDelay = radio->txQueue.front()->repeatTime;
                }
                xSemaphoreGive(radio->txQueue_mutex);
            }
            if (hasNextPacket && nextDelay > 0) {
                // Délai avant le prochain paquet
                radio->Ticker.delay_ms(nextDelay,  processNextPacketCallback, radio);
            } else if (hasNextPacket) {
                // Pas de délai, traiter immédiatement
                radio->processNextPacket();
            } else {
                // Plus de paquets, arrêter
                radio->stopTransmission();
            }
            //
            // radio->txCounter = radio->txCounter + 1;
            // if (radio->txCounter < radio->packets2send.size() && radio->packets2send[radio->txCounter] != nullptr) {
            //
            //     // if (radio->packets2send[radio->txCounter]->delayed != 0) {
            //     //     radio->delayed = radio->packets2send[radio->txCounter];
            //     //     radio->packets2send[radio->txCounter] = nullptr;
            //          radio->Ticker.delay_ms(radio->packets2send[radio->txCounter]->repeatTime, packetSender, radio);
            //     // } else {
            //         // radio->Ticker.attach_ms(radio->packets2send[radio->txCounter]->repeatTime, packetSender, radio);
            //     // }
            // } else {
            //     // In any case, after last packet sent, unlock the radio
            //     // Déverrouiller après la dernière transmission
            //     if (xSemaphoreTake(fhss_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            //         txMode = false;
            //         f_lock_hop = false;
            //         xSemaphoreGive(fhss_mutex);
            //     }
            //     // Radio::setRx(); // Non ! Remet la radio en réception
            //     // FIX: Deallocate memory from previous packets before clearing the vector to prevent memory leaks.
            //     for (auto p : radio->packets2send) {
            //         delete p;
            //     }
            //     radio->packets2send.clear();
            // }
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
    bool IRAM_ATTR iohcRadio::sent(iohcPacket *packet) const {
        bool ret = false;
        if (txCB) ret = txCB(packet);
        return ret;
    }

    //    static uint8_t RF96lnaMap[] = { 0, 0, 6, 12, 24, 36, 48, 48 };
/**
 * The `iohcRadio::receive` function toggles an LED, reads radio data, processes it, and
 * triggers a callback function.
 * 
 * @param stats The `stats` boolean used to determine whether to gather additional statistics during the radio reception process. If
 * `stats` is set to `true`, the function will collect and process additional information such as RSSI
 * (Received Signal
 * 
 * @return The function `iohcRadio::receive` is returning a boolean value `true`.
 */
    bool IRAM_ATTR iohcRadio::receive(bool stats = false) const {
        digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);
        // Verrouiller le hopping pendant la réception
        if (xSemaphoreTake(fhss_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            f_lock_hop = true;
            xSemaphoreGive(fhss_mutex);
        }

        // Create a new packet for this reception. Use a pointer to pass to the callback.
        auto* currentPacket = new iohcPacket();

        currentPacket->buffer_length = 0;
        currentPacket->frequency = scan_freqs[currentFreqIdx];

        _g_payload_millis = esp_timer_get_time();
        packetStamp = _g_payload_millis;

        if (stats) {
            currentPacket->rssi = static_cast<float>(Radio::readByte(REG_RSSIVALUE)) / -2.0f;
            int16_t thres = Radio::readByte(REG_RSSITHRESH);
            currentPacket->snr = currentPacket->rssi > thres ? 0 : (thres - currentPacket->rssi);
            int16_t f = static_cast<uint16_t>(Radio::readByte(REG_AFCMSB));
            f = (f << 8) | static_cast<uint16_t>(Radio::readByte(REG_AFCLSB));
            currentPacket->afc = f * 61.0;
        }

        while (Radio::dataAvail()) {
            if (currentPacket->buffer_length < MAX_FRAME_LEN) {
                currentPacket->payload.buffer[currentPacket->buffer_length++] = Radio::readByte(REG_FIFO);
            } else {
                // Prevent buffer overflow, discard extra bytes.
                Radio::readByte(REG_FIFO);
            }
        }

        if ( *currentPacket == last1wPacket && currentPacket->is1W() /*&& last1wPacketValid*/) {
            // ets_printf("Same packet, skipping\n");
        } else {
            if (rxCB) rxCB(currentPacket);
            currentPacket->decode(true); //stats);
            // addLogMessage(String(currentPacket->decodeToString(true).c_str()));

            // Save the new packet's data for the next comparison
            last1wPacket = *currentPacket;
            /*last1wPacketValid = true;*/
        }

        delete currentPacket; // The packet is processed or duplicated, we can free it.

        // Déverrouiller le hopping après réception
        if (xSemaphoreTake(fhss_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            f_lock_hop = false;
            xSemaphoreGive(fhss_mutex);
        }

        digitalWrite(RX_LED, false);
        return true;
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

    const char* iohcRadio::radioStateToString(RadioState state) {
    switch (state) {
        case RadioState::IDLE:     return "IDLE";
        case RadioState::RX:       return "RX";
        case RadioState::TX:       return "TX";
        case RadioState::PREAMBLE: return "PREAMBLE";
        case RadioState::PAYLOAD:  return "PAYLOAD";
        case RadioState::LOCKED:   return "LOCKED";
        case RadioState::ERROR:    return "ERROR";
        default:                   return "UNKNOWN";
    }
    }


    void IRAM_ATTR iohcRadio::setRadioState(const RadioState newState) {
        radioState = newState;
    }

           /**
 * Callback statique pour le délai entre paquets
 */
           void IRAM_ATTR iohcRadio::processNextPacketCallback(iohcRadio* radio) {
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
    void iohcRadio::processNextPacket() {
        // Nettoyer le paquet précédent
        if (currentTxPacket) {
            delete currentTxPacket;
            currentTxPacket = nullptr;
        }

        // Récupérer le prochain paquet
        if (xSemaphoreTake(txQueue_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGE("IOHC", "Failed to acquire queue mutex in processNext");
            stopTransmission();
            return;
        }

        if (txQueue.empty()) {
            xSemaphoreGive(txQueue_mutex);
            stopTransmission();
            return;
        }

        currentTxPacket = txQueue.front();
        txQueue.pop();
        xSemaphoreGive(txQueue_mutex);

        // Lancer le timer pour ce paquet
        Ticker.attach_ms(currentTxPacket->repeatTime, packetSender, this);
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
            if (xSemaphoreTake(fhss_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                txMode = false;
                f_lock_hop = false;
                xSemaphoreGive(fhss_mutex);
            }

            xSemaphoreGive(tx_mutex);
        }
    }

        /**
         * Vide la queue de transmission
         */
        void iohcRadio::clearTxQueue() {
            if (xSemaphoreTake(txQueue_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                while (!txQueue.empty()) {
                    const TxPacketWrapper* wrapper = txQueue.front();
                    txQueue.pop();
                    delete wrapper; // Le destructeur libère la mémoire
                }
                xSemaphoreGive(txQueue_mutex);
            }
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

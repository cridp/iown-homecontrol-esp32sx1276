#include <Arduino.h>
#include <iohcRadio.h>
#include <utility>

namespace IOHC {
    iohcRadio* iohcRadio::_iohcRadio = nullptr;
    volatile bool iohcRadio::_g_preamble = false;
    volatile bool iohcRadio::_g_payload = false;
    volatile unsigned long iohcRadio::_g_payload_millis = 0L;
    uint8_t iohcRadio::_flags[2] = {0, 0};
    volatile bool iohcRadio::f_lock = false;
    volatile bool iohcRadio::send_lock = false;
    volatile bool iohcRadio::txMode = false;

    iohcRadio::iohcRadio() {
        Radio::initHardware();
        Radio::initRegisters(MAX_FRAME_LEN);
        Radio::setCarrier(Radio::Carrier::Deviation, 19200);
        Radio::setCarrier(Radio::Carrier::Bitrate, 38400);
        Radio::setCarrier(Radio::Carrier::Bandwidth, 250);
        Radio::setCarrier(Radio::Carrier::Modulation, Radio::Modulation::FSK);

        // Attach interrupts to Preamble detected and end of packet sent/received
#if defined(SX1276)
        attachInterrupt(RADIO_PREAMBLE_DETECTED, i_preamble, CHANGE);
        attachInterrupt(RADIO_PACKET_AVAIL, i_payload, CHANGE);
#elif defined(CC1101)
        attachInterrupt(RADIO_PREAMBLE_DETECTED, i_preamble, RISING);
#endif

        TickTimer.attach_us(SM_GRANULARITY_US/*SM_GRANULARITY_MS*/, tickerCounter, this);
    }

    iohcRadio* iohcRadio::getInstance() {
        if (!_iohcRadio)
            _iohcRadio = new iohcRadio();
        return _iohcRadio;
    }

    void iohcRadio::start(uint8_t num_freqs, uint32_t* scan_freqs, uint32_t scanTimeUs,
                          IohcPacketDelegate rxCallback = nullptr, IohcPacketDelegate txCallback = nullptr) {
        this->num_freqs = num_freqs;
        this->scan_freqs = scan_freqs;
        this->scanTimeUs = scanTimeUs ? scanTimeUs : DEFAULT_SCAN_INTERVAL_US;
        this->rxCB = std::move(rxCallback);
        this->txCB = std::move(txCallback);

        Radio::clearBuffer();
        Radio::clearFlags();
        Radio::setCarrier(Radio::Carrier::Frequency, scan_freqs[0]);
        // 868950000); // We always start at freq[0] the 1W/2W channel
        Radio::calibrate();
        Radio::setRx();
    }

    void IRAM_ATTR iohcRadio::tickerCounter(iohcRadio* radio) {
        // Not need to put in IRAM as we reuse task for µs instead ISR
#if defined(SX1276)
        Radio::readBytes(REG_IRQFLAGS1, _flags, sizeof(_flags));

        if (_g_payload) {
            // If Int of PayLoad
            if (_flags[0] & RF_IRQFLAGS1_TXREADY) {
                // if TX ready?
                // radio->sent(radio->iohc);
                Radio::clearFlags();
                if (!txMode) {
                    Radio::setRx();
                    f_lock = false;
                }
                radio->sent(radio->iohc); // Workaround to permit MQTT sending
                return;
            }
            else {
                // if in RX mode?
                radio->receive(false);
                radio->tickCounter = 0;
                radio->preCounter = 0;
                return;
            }
        }

        if (_g_preamble) {
            radio->tickCounter = 0;
            radio->preCounter += 1;
            if (_flags[0] & RF_IRQFLAGS1_SYNCADDRESSMATCH) radio->preCounter = 0;
            // In case of Sync received resets the preamble duration
            if ((radio->preCounter * SM_GRANULARITY_US) >= SM_PREAMBLE_RECOVERY_TIMEOUT_US) {
                // Avoid hanging on a too long preamble detect
                Radio::clearFlags();
                radio->preCounter = 0;
            }
        }

        if (f_lock) return;

        if ((++radio->tickCounter * SM_GRANULARITY_US) < radio->scanTimeUs) return;

        //        digitalWrite(SCAN_LED, false);
        radio->tickCounter = 0;

        if (radio->num_freqs == 1) return;

        radio->currentFreqIdx += 1;
        if (radio->currentFreqIdx >= radio->num_freqs)
            radio->currentFreqIdx = 0;

        Radio::setCarrier(Radio::Carrier::Frequency, radio->scan_freqs[radio->currentFreqIdx]);
        //        digitalWrite(SCAN_LED, true);

        // return;

#elif defined(CC1101)
        if (__g_preamble){
            radio->receive();
            radio->tickCounter = 0;
            radio->preCounter = 0;
            return;
        }

        if (f_lock)
            return;

        if ((++radio->tickCounter * SM_GRANULARITY_US) < radio->scanTimeUs)
            return;
#endif
    }

    //     template <size_t N>
    //     void iohcRadio::send(std::array<iohcPacket*, N>& iohcTx) {
    //         uint8_t idx = 0;

    //         if (txMode)  return;
    //         do {
    //             packets2send[idx] = iohcTx[idx];
    //         } while (iohcTx[idx++]);
    // //         for (auto packet : iohcTx) {
    // //             if (!packet) break;
    // //             packets2send[idx++] = packet;
    // //         }
    //         txCounter = 0;
    //         Sender.attach_ms(packets2send[txCounter]->repeatTime, packetSender, this);
    //     }

    void iohcRadio::packetSender(iohcRadio* radio) {
        digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);

        f_lock = true; // Stop frequency hopping
        txMode = true; // Avoid Radio put in Rx mode at next packet sent/received
        //        Serial.printf("Using Delayed Packet %u !\n", radio->txCounter);
        if (radio->packets2send[radio->txCounter] == nullptr) {
            //            Serial.printf("Plus de Delayed Packet !\n");
            if (radio->delayed != nullptr)
                //               Serial.printf("Use Saved Delayed Packet !\n");
                //           f_lock = false; txMode = false; return;
                radio->iohc = std::move(radio->delayed);
        }
        else
            radio->iohc = radio->packets2send[radio->txCounter];

        if (radio->iohc->frequency != 0)
            Radio::setCarrier(Radio::Carrier::Frequency, radio->iohc->frequency);
        else
            radio->iohc->frequency = radio->scan_freqs[radio->currentFreqIdx];

        Radio::setStandby();
        Radio::clearFlags();

#if defined(SX1276)
        Radio::writeBytes(REG_FIFO, radio->iohc->payload.buffer, radio->iohc->buffer_length);
        // Is valid for both SX1276 & CC1101?

#elif defined(CC1101)
        Radio::sendFrame(radio->iohc->payload.buffer, radio->iohc->buffer_length); // Prepare (encode, add crc, and so no) the packet for CC1101
#endif

        // Serial.printf("Size %u Counter %u", radio->packets2send.size(), radio->txCounter );

        /*radio->iohc->*/packetStamp = esp_timer_get_time(); //millis();
        radio->iohc->decode();
        IOHC::lastSendCmd = radio->iohc->payload.packet.header.cmd;

        Radio::setTx();
        // There is no need to maintain radio locked between packets transmission unless clearly asked
        txMode = radio->iohc->lock;

        if (radio->iohc->repeat)
            radio->iohc->repeat -= 1;
        if (radio->iohc->repeat == 0) {
            radio->Sender.detach();
            ++radio->txCounter;
            if (radio->txCounter < radio->packets2send.size() && radio->packets2send[radio->txCounter] != nullptr) {
                //if (radio->packets2send[++(radio->txCounter)]) {
                if (radio->packets2send[radio->txCounter]->delayed != 0) {
                    radio->delayed = radio->packets2send[radio->txCounter];
                    radio->packets2send[radio->txCounter] = nullptr;
                    radio->Sender.delay_ms(radio->delayed/*radio->packets2send[radio->txCounter]*/->delayed, packetSender, radio);
                }
                else {
                    radio->Sender.attach_ms(radio->packets2send[radio->txCounter]->repeatTime, packetSender, radio);
                }
            }
            else { 
                // In any case, after last packet sent, unlock the radio
                txMode = false; 
                radio->packets2send.clear();
                }
        }
        digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);
    }

    bool iohcRadio::sent(iohcPacket* packet) {
        bool ret = false;
        if (txCB) {
            ret = txCB(packet);
        }
        return ret;
    }

    //    static uint8_t RF96lnaMap[] = { 0, 0, 6, 12, 24, 36, 48, 48 };
    bool iohcRadio::receive(bool stats = false) {
        digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);
        bool frmErr = false;
        iohc = new iohcPacket; //(iohcPacket *)malloc(sizeof(iohcPacket));
        iohc->buffer_length = 0;
        iohc->frequency = scan_freqs[currentFreqIdx];
        /*iohc->*/
        packetStamp = _g_payload_millis;
#if defined(SX1276)
        if (stats) {
            iohc->rssi = static_cast<float>(Radio::readByte(REG_RSSIVALUE)) / -2.0f;
            int16_t thres = Radio::readByte(REG_RSSITHRESH);
            iohc->snr = iohc->rssi > thres ? 0 : (thres - iohc->rssi);
            //            iohc->lna = RF96lnaMap[ (Radio::readByte(REG_LNA) >> 5) & 0x7 ];
            int16_t f = (uint16_t)Radio::readByte(REG_AFCMSB);
            f = (f << 8) | (uint16_t)Radio::readByte(REG_AFCLSB);
            //            iohc->afc = f * (32000000.0 / 524288.0); // static_cast<float>(1 << 19));
            iohc->afc = /*(int32_t)*/f * 61.0;
            //            iohc->rssiAt = micros();
        }
#elif defined(CC1101)
        __g_preamble = false;

        uint8_t tmprssi=Radio::SPIgetRegValue(REG_RSSI);
        if (tmprssi>=128)
            iohc->rssi = (float)((tmprssi-256)/2)-74;
        else
            iohc->rssi = (float)(tmprssi/2)-74;

        uint8_t bytesInFIFO = Radio::SPIgetRegValue(REG_RXBYTES, 6, 0);
        size_t readBytes = 0;
        uint32_t lastPop = millis();
#endif

#if defined(SX1276)
        while (!Radio::dataAvail());

        while (Radio::dataAvail()) {
            iohc->payload.buffer[iohc->buffer_length++] = Radio::readByte(REG_FIFO);
        }

#elif defined(CC1101)
        uint8_t lenghtFrameCoded = 0xFF;
        uint8_t tmpBuffer[64]={0x00};
        while (readBytes < lenghtFrameCoded) {
            if ( (readBytes>=1) && (lenghtFrameCoded==0xFF) ){ // Obtain frame lenght
                lenghtFrame = (Radio::reverseByte( ((uint8_t)(tmpBuffer[0]<<4) | (uint8_t)(tmpBuffer[1]>>4)))) & 0b00011111;
                lenghtFrameCoded = ((lenghtFrame + 2 + 1)*8) + ((lenghtFrame + 2 + 1)*2);   // Calculate Num of bits of encoded frame (add 2 bit per byte)
                lenghtFrameCoded = ceil((float)lenghtFrameCoded/8);                         // divide by 8 bits per byte and round to up
                Radio::setPktLenght(lenghtFrameCoded);
                //Serial.printf("BytesReaded: %d\tlenghtFrame: 0x%d\t lenghtFrameCoded: 0x%d\n", readBytes, lenghtFrame,  lenghtFrameCoded);
            }

            if (bytesInFIFO == 0) {
                if (millis() - lastPop > 5) {
                    // readData was required to read a packet longer than the one received.
                    //Serial.println("No data for more than 5mS. Stop here.");
                    break;
                } else {
                    delay(1);
                    bytesInFIFO = Radio::SPIgetRegValue(REG_RXBYTES, 6, 0);
                    continue;
                }
            }

            // read the minimum between "remaining length" and bytesInFifo
            uint8_t bytesToRead = (((uint8_t)(lenghtFrameCoded - readBytes))<(bytesInFIFO)?((uint8_t)(lenghtFrameCoded - readBytes)):(bytesInFIFO));
            Radio::SPIreadRegisterBurst(REG_FIFO, bytesToRead, &(tmpBuffer[readBytes]));
            readBytes += bytesToRead;
            lastPop = millis();

            // Get how many bytes are left in FIFO.
            bytesInFIFO = Radio::SPIgetRegValue(REG_RXBYTES, 6, 0);
        }


        frmErr=true;
        if (lenghtFrameCoded<255){
            int8_t lenFuncDecodeFrame = Radio::decodeFrame(tmpBuffer, lenghtFrameCoded);
            if (lenFuncDecodeFrame>0 && lenFuncDecodeFrame<=MAX_FRAME_LEN){
                if (iohcUtils::radioPacketComputeCrc(tmpBuffer, lenFuncDecodeFrame) == 0 ){
                    iohc->buffer_length = lenFuncDecodeFrame;
                    memcpy(iohc->payload.buffer, tmpBuffer, lenFuncDecodeFrame);  // volcamos el resultado al array de origen
                    frmErr=false;
                }
            }
        }

        // Flush then standby according to RXOFF_MODE (default: RADIOLIB_CC1101_RXOFF_IDLE)
        if (Radio::SPIgetRegValue(REG_MCSM1, 3, 2) == RF_RXOFF_IDLE) {
            Radio::SPIsendCommand(CMD_IDLE);                    // set mode to standby
            Radio::SPIsendCommand(CMD_FLUSH_RX | CMD_READ);     // flush Rx FIFO
        }

        f_lock = false;
        Radio::SPIsendCommand(CMD_RX);

#endif

        //        Radio::clearFlags();
        iohc->decode();
        // IOHC::lastSendCmd = iohc->payload.packet.header.cmd;

        if (rxCB && !frmErr) rxCB(iohc);

        //        delete iohc; //free(iohc);

        digitalWrite(RX_LED, false); //digitalRead(RX_LED)^1);
        return true;
    }

    void IRAM_ATTR iohcRadio::i_preamble() {
#if defined(SX1276)
        _g_preamble = digitalRead(RADIO_PREAMBLE_DETECTED);
#elif defined(CC1101)
        __g_preamble = true;
#endif
        f_lock = _g_preamble;
    }

    void IRAM_ATTR iohcRadio::i_payload() {
#if defined(SX1276)
        _g_payload = digitalRead(RADIO_PACKET_AVAIL);
        if (_g_payload)
            _g_payload_millis = esp_timer_get_time(); // millis();
#endif
    }
}

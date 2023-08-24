#include <iohcRadio.h>


namespace IOHC
{
    iohcRadio *iohcRadio::_iohcRadio = nullptr;
    volatile bool iohcRadio::__g_preamble = false;
    volatile bool iohcRadio::__g_payload = false;
    volatile unsigned long iohcRadio::__g_payload_millis = 0;
    uint8_t iohcRadio::__flags[2] = {0, 0};
    volatile bool iohcRadio::f_lock = false;
    volatile bool iohcRadio::send_lock = false;
    volatile bool iohcRadio::txMode = false;


    iohcRadio::iohcRadio()
    {
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

#if defined(ESP8266)
        TickTimer.attach_us(SM_GRANULARITY_US, tickerCounter, this);
#elif defined(ESP32)
        // Quick solution, but less precise than 100uS of the ESP8266, which would require to implement the check in a timer interruption
        // Ticker lib on ESP32 relay on RTOS tick period. To change it its needed to recompile ESP-IDF, but this is already compiled
        // into the Arduino Framework
        // Good enought?
        TickTimer.attach_us(SM_GRANULARITY_MS, tickerCounter, this);
#endif
    }

    iohcRadio *iohcRadio::getInstance()
    {
        if (!_iohcRadio)
            _iohcRadio = new iohcRadio();
        return _iohcRadio;
    }

    void iohcRadio::start(uint8_t num_freqs, uint32_t *scan_freqs, uint32_t scanTimeUs, IohcPacketDelegate rxCallback = nullptr, IohcPacketDelegate txCallback = nullptr)
    {
        this->num_freqs = num_freqs;
        this->scan_freqs = scan_freqs;
        this->scanTimeUs = scanTimeUs?scanTimeUs:DEFAULT_SCAN_INTERVAL_US;
        this->rxCB = rxCallback;
        this->txCB = txCallback;

        Radio::clearBuffer();
        Radio::clearFlags();
        Radio::setCarrier(Radio::Carrier::Frequency, 868950000);
        Radio::calibrate();
        Radio::setRx();

    }

    void iohcRadio::tickerCounter(iohcRadio *radio)
    {
#if defined(SX1276)
        Radio::readBytes(REG_IRQFLAGS1, __flags, sizeof(__flags));

        if (__g_payload)                                                // If Int of PayLoad
        {
            if (__flags[0] & RF_IRQFLAGS1_TXREADY)                      // if TX ready?
            {
                radio->sent(radio->iohc);
                Radio::clearFlags();
                if (!txMode)
                {
                    Radio::setRx();
                    f_lock = false;
                }
                return;
            }
            else                                                        // if in RX mode?
            {
                radio->receive();
                radio->tickCounter = 0;
                radio->preCounter = 0;
                return;
            }
        }

        if (__g_preamble)
        {
            radio->tickCounter = 0;
            radio->preCounter += 1;
            if (__flags[0] & RF_IRQFLAGS1_SYNCADDRESSMATCH) radio->preCounter = 0;  // In case of Sync received resets the preamble duration
            if ((radio->preCounter * SM_GRANULARITY_US) >= SM_PREAMBLE_RECOVERY_TIMEOUT_US) // Avoid hanging on a too long preamble detect 
            {
                Radio::clearFlags();
                radio->preCounter = 0;
            }
        }

        if (f_lock)
            return;


        if ((++radio->tickCounter * SM_GRANULARITY_US) < radio->scanTimeUs)
            return;

        digitalWrite(SCAN_LED, false);
        radio->tickCounter = 0;
        radio->currentFreq += 1;
        if (radio->currentFreq >= radio->num_freqs)
            radio->currentFreq = 0;
        Radio::setCarrier(Radio::Carrier::Frequency, radio->scan_freqs[radio->currentFreq]);
        digitalWrite(SCAN_LED, true);

        return;

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

    void iohcRadio::send(IOHC::iohcPacket *iohcTx[])
    {
        uint8_t idx = 0;

        if (txMode)
            return;

        do
        {
            packets2send[idx] = iohcTx[idx];
        } while (iohcTx[idx++]);

        txCounter = 0;
        Sender.attach_ms(packets2send[txCounter]->millis, packetSender, this);
    }

    void iohcRadio::packetSender(iohcRadio *radio)
    {
Serial.println("packetSender");
        f_lock = true;  // Stop frequency hopping
        txMode = true;  // Avoid Radio put in Rx mode at next packet sent/received

        radio->iohc = radio->packets2send[radio->txCounter];
        if (radio->iohc->frequency != 0)
            Radio::setCarrier(Radio::Carrier::Frequency, radio->iohc->frequency);
        else
            radio->iohc->frequency = radio->scan_freqs[radio->currentFreq];

        Radio::setStandby();
        Radio::clearFlags();

        digitalWrite(RX_LED, digitalRead(RX_LED)^1);
#if defined(SX1276)
        for (uint8_t idx=0; idx < radio->iohc->buffer_length; ++idx)
        {
            Radio::writeByte(REG_FIFO, radio->iohc->payload.buffer[idx]);       // Is valid for both SX1276 & CC1101?
        }
#elif defined(CC1101)
        Radio::sendFrame(radio->iohc->payload.buffer, radio->iohc->buffer_length); // Prepare (encode, add crc, and so no) the packet for CC1101
#endif

        radio->iohc->millis = millis();
        Radio::setTx();
        digitalWrite(RX_LED, digitalRead(RX_LED)^1);
        txMode = radio->iohc->lock; // There is no need to maintain radio locked between packets transmission unless clearly asked

        if (radio->iohc->repeat)
            radio->iohc->repeat -= 1;
        if (radio->iohc->repeat == 0)
        {
            radio->Sender.detach();
            if (radio->packets2send[++(radio->txCounter)])
                radio->Sender.attach_ms(radio->packets2send[radio->txCounter]->millis, radio->packetSender, radio);
            else
            {
                txMode = false;     // In any case, after last packet sent, unlock the radio
            }
        }
    }

    bool iohcRadio::sent(IOHC::iohcPacket *packet)
    {
        bool ret = false;

        if (txCB)
            ret = txCB(packet);
        return ret;
    }

    bool iohcRadio::receive()
    {
        bool frmErr=false;
        iohc = (IOHC::iohcPacket *)malloc(sizeof(IOHC::iohcPacket));
        iohc->buffer_length = 0;
        iohc->frequency = scan_freqs[currentFreq];
        iohc->millis = __g_payload_millis;
#if defined(SX1276)
        iohc->rssi = (float)(Radio::readByte(REG_RSSIVALUE)/2)*-1;          //
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
        while (!Radio::dataAvail())
            ;
        digitalWrite(RX_LED, digitalRead(RX_LED)^1);
        do
        {
            iohc->payload.buffer[iohc->buffer_length] = Radio::readByte(REG_FIFO);
            iohc->buffer_length += 1;
        } while (Radio::dataAvail());
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
        if (rxCB && !frmErr) rxCB(iohc);
        free(iohc);
        digitalWrite(RX_LED, digitalRead(RX_LED)^1);
        return true;
    }

    void IRAM_ATTR iohcRadio::i_preamble() {
#if defined(SX1276)
        __g_preamble = digitalRead(RADIO_PREAMBLE_DETECTED) ? true:false;
#elif defined(CC1101)
        __g_preamble = true;
#endif
        f_lock = __g_preamble;
    }

    void IRAM_ATTR iohcRadio::i_payload() {
#if defined(SX1276)
            __g_payload = digitalRead(RADIO_PACKET_AVAIL) ? true:false;
            if (__g_payload)
                __g_payload_millis = millis();
#endif
    }

}
#include <iohcRadio.h>


namespace IOHC
{
    iohcRadio *iohcRadio::_iohcRadio = nullptr;
    volatile bool iohcRadio::__g_preamble = false;
    volatile bool iohcRadio::__g_payload = false;
    volatile bool iohcRadio::__g_timeout = false;
    uint8_t iohcRadio::__flags[2] = {0, 0};

    volatile unsigned long iohcRadio::__g_payload_millis = 0;
    bool iohcRadio::f_lock = false;


    iohcRadio::iohcRadio()
    {
        Radio::initHardware();
        Radio::initRegisters(MAX_FRAME_LEN);
        Radio::setCarrier(Radio::Carrier::Deviation, 19200);
        Radio::setCarrier(Radio::Carrier::Bitrate, 38400);
        Radio::setCarrier(Radio::Carrier::Bandwidth, 250);
        Radio::setCarrier(Radio::Carrier::Modulation, Radio::Modulation::FSK);

        // Attach interrupts to Preamble detected and end of packet sent/received
        attachInterrupt(RADIO_PREAMBLE_DETECTED, i_preamble, CHANGE);
        attachInterrupt(RADIO_PACKET_AVAIL, i_payload, CHANGE);

        TickTimer.attach_us(SM_GRANULARITY_US, tickerCounter, this);
//        StateMachine.attach_us(SM_CHECK_US, manageStates, this);
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
        this->scanTimeUs = scanTimeUs;
        this->rxCB = rxCallback;
        this->txCB = txCallback;

//        scan(true);
        Radio::clearBuffer();
        Radio::clearFlags();
        Radio::setCarrier(Radio::Carrier::Frequency, 868950000);
        Radio::calibrate();
        Radio::setRx();
    }

    void iohcRadio::tickerCounter(iohcRadio *radio)
    {
        Radio::readBytes(REG_IRQFLAGS1, __flags, sizeof(__flags));

        if (__g_payload)
        {
            if (__flags[0] & RF_IRQFLAGS1_TXREADY)
            {
                radio->sent(radio->iohc);

                Radio::clearFlags();
                Radio::setRx();
                f_lock = false;
                return;
            }
            else
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

        radio->tickCounter += 1;
        if (radio->tickCounter % 10)
            return;

        if (f_lock)
            return;

        digitalWrite(SCAN_LED, 0);
        radio->currentFreq += 1;
        if (radio->currentFreq >= radio->num_freqs)
            radio->currentFreq = 0;
        Radio::setCarrier(Radio::Carrier::Frequency, radio->scan_freqs[radio->currentFreq]);
        digitalWrite(SCAN_LED, 1);

        return;
    }

    void iohcRadio::send(IOHC::iohcPacket *iohcTx[])
    {
        uint8_t idx = 0;
        do
        {
            packets2send[idx] = iohcTx[idx];
        } while (iohcTx[idx++]);
        
        Sender.attach_ms(packets2send[txCounter]->millis, packetSender, this);

/*
        f_lock = true;  // Stop frequency hopping
        iohc = iohcTx[0];

        if (iohc->frequency == 0)
            iohc->frequency = scan_freqs[currentFreq];
        iohc->millis = millis();

        Radio::setStandby();
        if (iohc->frequency)
            Radio::setCarrier(Radio::Carrier::Frequency, iohc->frequency);
        Radio::clearFlags();

        digitalWrite(RX_LED, digitalRead(RX_LED)^1);
        Serial.printf("Sending ");
        for (uint8_t idx=0; idx < iohc->buffer_length; ++idx)
        {
            Serial.printf("%2.2x", iohc->payload.buffer[idx]);
            Radio::writeByte(REG_FIFO, iohc->payload.buffer[idx]);
        }
        Serial.printf(" on Carrier frequency: %6.3fM ... ", (float)(iohc->frequency)/1000000);  // If requested sets a specific frequency, otherwise uses last tuned
        Radio::setTx();
        digitalWrite(RX_LED, digitalRead(RX_LED)^1);
*/
    }

    void iohcRadio::packetSender(iohcRadio *radio)
    {
        f_lock = true;  // Stop frequency hopping
        radio->iohc = radio->packets2send[radio->txCounter];

        if (radio->iohc->frequency == 0)
            radio->iohc->frequency = radio->scan_freqs[radio->currentFreq];
        radio->iohc->millis = millis();

        Radio::setStandby();
        if (radio->iohc->frequency)
            Radio::setCarrier(Radio::Carrier::Frequency, radio->iohc->frequency);
        Radio::clearFlags();

        digitalWrite(RX_LED, digitalRead(RX_LED)^1);
//        Serial.printf("Sending ");
        for (uint8_t idx=0; idx < radio->iohc->buffer_length; ++idx)
        {
//            Serial.printf("%2.2x", radio->iohc->payload.buffer[idx]);
            Radio::writeByte(REG_FIFO, radio->iohc->payload.buffer[idx]);
        }
//        Serial.printf(" on Carrier frequency: %6.3fM ... ", (float)(radio->iohc->frequency)/1000000);  // If requested sets a specific frequency, otherwise uses last tuned
        Radio::setTx();
        digitalWrite(RX_LED, digitalRead(RX_LED)^1);
        if (radio->iohc->repeat)
            radio->iohc->repeat -= 1;
        if (radio->iohc->repeat == 0)
        {
            radio->Sender.detach();
            radio->f_lock = false;
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
        iohc = (IOHC::iohcPacket *)malloc(sizeof(IOHC::iohcPacket));
        iohc->buffer_length = 0;
        iohc->frequency = scan_freqs[currentFreq];
        iohc->millis = __g_payload_millis;
        iohc->rssi = (float)(Radio::readByte(REG_RSSIVALUE)/2)*-1;

        while (!Radio::dataAvail())
            ;
        digitalWrite(RX_LED, digitalRead(RX_LED)^1);

        do
        {
            iohc->payload.buffer[iohc->buffer_length] = Radio::readByte(REG_FIFO);
            iohc->buffer_length += 1;
        } while (Radio::dataAvail());

//        Radio::clearFlags();
        if (rxCB) rxCB(iohc);
        free(iohc);
        digitalWrite(RX_LED, digitalRead(RX_LED)^1);
        return true;
    }

    void IRAM_ATTR iohcRadio::i_preamble() {
        __g_preamble = digitalRead(RADIO_PREAMBLE_DETECTED) ? true:false;
        f_lock = __g_preamble;
    }

    void IRAM_ATTR iohcRadio::i_payload() {
            __g_payload = digitalRead(RADIO_PACKET_AVAIL) ? true:false;
            if (__g_payload)
                __g_payload_millis = millis();
    }

}
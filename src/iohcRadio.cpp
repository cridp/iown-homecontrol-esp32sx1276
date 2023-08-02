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
        attachInterrupt(RADIO_PREAMBLE_DETECTED, i_preamble, CHANGE);
        attachInterrupt(RADIO_PACKET_AVAIL, i_payload, CHANGE);

        TickTimer.attach_us(SM_GRANULARITY_US, tickerCounter, this);
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
        Radio::readBytes(REG_IRQFLAGS1, __flags, sizeof(__flags));

        if (__g_payload)
        {
            if (__flags[0] & RF_IRQFLAGS1_TXREADY)
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

        if (f_lock)
            return;

//        radio->tickCounter += 1;
//        if (radio->tickCounter % 10)
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
        for (uint8_t idx=0; idx < radio->iohc->buffer_length; ++idx)
        {
            Radio::writeByte(REG_FIFO, radio->iohc->payload.buffer[idx]);
        }

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
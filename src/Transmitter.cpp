#include <Transmitter.h>


namespace Radio
{
    bool __g_sent = false;
    unsigned long __g_sent_millis = 0;

    void Transmitter::tickerCounter(Transmitter *transmitter)
    {
       transmitter->tickCounter += 1;
       transmitter->crcReceivedOk = __g_sent;
       transmitter->crcSentMillis = __g_sent_millis;

       if (transmitter->crcReceivedOk)
       {
            transmitter->tickCounter = 0;
       }
    }

    void Transmitter::manageStates(Transmitter *transmitter)
    {
        if (transmitter->crcReceivedOk)
        {
            transmitter->putPacket();
        }
    }

    void Transmitter::Init()
    {
        Radio::setCarrier(Radio::Carrier::Deviation, 19200);
        Radio::setCarrier(Radio::Carrier::Bitrate, 38400);
        Radio::setCarrier(Radio::Carrier::Bandwidth, 250);
        Radio::setCarrier(Radio::Carrier::Modulation, Radio::Modulation::FSK);

        Radio::initTx();
    }

    bool Transmitter::putPacket()
    {
        bool ret = false;

        iohc->millis = crcSentMillis;
        if (packetCB)
            ret = packetCB(iohc);

        Radio::setStandby();
        return ret;
    }

    void Transmitter::Send(Radio::iohcPacket *iohcTx)
    {
        iohc = iohcTx;
        Radio::setCarrier(Radio::Carrier::Frequency, iohcTx->frequency);

        digitalWrite(RX_LED, digitalRead(RX_LED)^1);
        for (uint8_t idx=0; idx < iohcTx->buffer_length; ++idx)
        {
            writeByte(REG_FIFO, iohcTx->packet.buffer[idx]);
        }
        Radio::setTx();
        digitalWrite(RX_LED, digitalRead(RX_LED)^1);
    }

    void Transmitter::Start()
    {
        TickTimer.attach_us(SM_GRANULARITY_US, tickerCounter, this);
        StateMachine.attach_us(SM_CHECK_US, manageStates, this);

        // Attach interrupts to end of packet sent/received
        attachInterrupt(RADIO_PACKET_AVAIL, i_payload, CHANGE);

        Radio::clearBuffer();
        Radio::clearFlags();
    }

    void Transmitter::Stop()
    {
        Radio::setStandby();

        detachInterrupt(RADIO_PACKET_AVAIL);

        StateMachine.detach();
        TickTimer.detach();
    }

    void IRAM_ATTR Transmitter::i_payload() {
            __g_sent = digitalRead(RADIO_PACKET_AVAIL) ? true:false;
            if (__g_sent)
                __g_sent_millis = millis();
    }
}
#include <SX1276Helpers.h>
#include <map>
#include <TickerUs.h>

namespace Radio
{
    SPISettings SpiSettings(SPI_CLOCK_DIV2, MSBFIRST, SPI_MODE0);
    WorkingParams _params;

    uint8_t bufferIndex = 0;
    payload _payload;


    uint32_t freqs[MAX_FREQS] = FREQS2SCAN;
    uint8_t next_freq = 0;
    uint8_t scanCounter = 0;

    // Simplified bandwidth registries evaluation
    std::map<uint8_t, regBandWidth> __bw =
    {
        {25, {0x01, 0x04}},  // 25KHz
        {50, {0x01, 0x03}},
        {100, {0x01, 0x02}},
        {125, {0x00, 0x02}},
        {200, {0x01, 0x01}},
        {250, {0x00, 0x01}}  // 250KHz
    };

    void SPI_beginTransaction(void)
    {
        SPI.beginTransaction(Radio::SpiSettings);
        digitalWrite(RADIO_NSS, LOW);
    }

    void SPI_endTransaction(void)
    {
        digitalWrite(RADIO_NSS, HIGH);
        SPI.endTransaction();
    }

    void init(void)
    {
        Serial.println("SPI Init");
        // SPI pins configuration
        SPI.pins(RADIO_SCLK, RADIO_MISO, RADIO_MOSI, RADIO_NSS);
        pinMode(RADIO_RESET, INPUT);    // Connected to Reset; floating for POR

        // Check the availability of the Radio
        do {
            delayMicroseconds(1);
            wdt_reset();
        } while (!digitalRead(RADIO_RESET));
        delayMicroseconds(BOARD_READY_AFTER_POR);
        Serial.printf("Radio Chip is ready\n");

        // Initialize SPI bus
        SPI.begin();
        // Disable SPI device
        // Disable device NRESET pin
        pinMode(RADIO_NSS, OUTPUT);
        pinMode(RADIO_RESET, OUTPUT);
        digitalWrite(RADIO_RESET, HIGH);
        digitalWrite(RADIO_NSS, HIGH);
        delayMicroseconds(BOARD_READY_AFTER_POR);

        SPI.beginTransaction(Radio::SpiSettings);
        SPI.endTransaction();
        writeByte(REG_OPMODE, RF_OPMODE_STANDBY);       // Put Radio in Standby mode
        readBytes(REG_OPMODE, &_params.rfOpMode, 1);    // Collect rfOpMode register value
        readBytes(REG_SEQCONFIG1, _params.seqConf, 2);  // Collect seqConf register value

        pinMode(SCAN_LED, OUTPUT);
        digitalWrite(SCAN_LED, 1);
    }

    void calibrate(void)
    {
        // Start image and RSSI calibration
        writeByte(REG_IMAGECAL, (RF_IMAGECAL_AUTOIMAGECAL_MASK & RF_IMAGECAL_IMAGECAL_MASK) | RF_IMAGECAL_IMAGECAL_START);
        // Wait end of calibration
        do {} while (readByte(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING);
    }

    void initTx(void)
    {
        // Firstly put radio in StandBy mode as some parameters cannot be changed differently
        _params.rfOpMode &= RF_OPMODE_MASK;
        _params.rfOpMode |= RF_OPMODE_STANDBY;
        writeByte(REG_OPMODE, _params.rfOpMode);

        writeByte(REG_OSC, RF_OSC_RCCALSTART | RF_OSC_CLKOUT_OFF);  // RC Calibration and switch-off clockout


        // PA boost maximum power
        writeByte(REG_PACONFIG, RF_PACONFIG_PASELECT_MASK | RF_PACONFIG_PASELECT_PABOOST);
        // PA Ramp: No Shaping, Ramp up/down 15us
        writeByte(REG_PARAMP, RF_PARAMP_MODULATIONSHAPING_00 | RF_PARAMP_0015_US);


        // Variable packet lenght, generates working CRC !!!
        // Packet mode, IoHomeOn, IoHomePowerFrame to be add 0x10 ???
        writeByte(REG_PACKETCONFIG1, RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RF_PACKETCONFIG1_DCFREE_OFF | RF_PACKETCONFIG1_CRC_ON | RF_PACKETCONFIG1_CRCAUTOCLEAR_ON | RF_PACKETCONFIG1_ADDRSFILTERING_OFF | RF_PACKETCONFIG1_CRCWHITENINGTYPE_CCITT);
        writeByte(REG_PACKETCONFIG2, RF_PACKETCONFIG2_DATAMODE_PACKET | RF_PACKETCONFIG2_IOHOME_ON);// | RF_PACKETCONFIG2_IOHOME_POWERFRAME);   // Is IoHomePowerFrame useful ?

        // Setting Preamble Length
        writeByte(REG_PREAMBLEMSB, PREAMBLE_MSB);
        writeByte(REG_PREAMBLELSB, PREAMBLE_LSB);

        // Enabling Sync word - Size must be set to 2
        writeByte(REG_SYNCCONFIG, RF_SYNCCONFIG_PREAMBLEPOLARITY_AA | RF_SYNCCONFIG_SYNC_ON | RF_SYNCCONFIG_SYNCSIZE_2);
        writeByte(REG_SYNCVALUE1, SYNC_BYTE_1);
        writeByte(REG_SYNCVALUE2, SYNC_BYTE_2);


        // Sequencer off, Standby when idle, FromStart to Transmit, to LowPower, FromIdle to Transmit, fromTransmit to LowPower
        // 
        _params.seqConf[0] = RF_SEQCONFIG1_IDLEMODE_STANDBY | RF_SEQCONFIG1_FROMSTART_TOTX | RF_SEQCONFIG1_LPS_IDLE | RF_SEQCONFIG1_FROMIDLE_TOTX | RF_SEQCONFIG1_FROMTX_TOLPS;
        _params.seqConf[1] = 0x00;
        writeBytes(REG_SEQCONFIG1, _params.seqConf, 2);

        // Mapping of pins DIO0 to DIO3
        // DIO0: PayloadReady|PacketSent    DIO1: FIFO empty    DIO2: Sync   | DIO3: TxReady
        // Mapping of pins DIO4 and DIO5
        // DIO4: PreambleDetect  DIO5: Data
        writeByte(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00 | RF_DIOMAPPING1_DIO1_01 | RF_DIOMAPPING1_DIO2_11 | RF_DIOMAPPING1_DIO3_01);
        writeByte(REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_11 | RF_DIOMAPPING2_DIO5_10 | RF_DIOMAPPING2_MAP_PREAMBLEDETECT);
        // FIFO Threshold - currently useless
        writeByte(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTARTCONDITION_FIFONOTEMPTY);
        setCarrier(Radio::Carrier::Frequency, 869850000);

/*
        // Enable Tx
        _params.rfOpMode &= RF_OPMODE_MASK;
        _params.rfOpMode |= RF_OPMODE_TRANSMITTER;
        writeByte(REG_OPMODE, _params.rfOpMode);

        TxReady;
*/
    }

    void initRx(void)
    {
        // Firstly put radio in StandBy mode as some parameters cannot be changed differently
        _params.rfOpMode &= RF_OPMODE_MASK;
        _params.rfOpMode |= RF_OPMODE_STANDBY;
        writeByte(REG_OPMODE, _params.rfOpMode);

        writeByte(REG_OSC, RF_OSC_RCCALSTART | RF_OSC_CLKOUT_OFF);  // RC Calibration and switch-off clockout


        // Variable packet lenght, working with CRC
        // Packet mode, IoHomeOn, IoHomePowerFrame to be add 0x10 
        writeByte(REG_PACKETCONFIG1, RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RF_PACKETCONFIG1_DCFREE_OFF | RF_PACKETCONFIG1_CRC_ON | RF_PACKETCONFIG1_CRCAUTOCLEAR_ON | RF_PACKETCONFIG1_ADDRSFILTERING_OFF | RF_PACKETCONFIG1_CRCWHITENINGTYPE_CCITT);
        writeByte(REG_PACKETCONFIG2, RF_PACKETCONFIG2_DATAMODE_PACKET | RF_PACKETCONFIG2_IOHOME_ON);   // Is IoHomePowerFrame useful ?

        writeByte(REG_PAYLOADLENGTH, 0xff); // Disable lenght checking; do not delete !!!

        // Enabling Sync word - received word from remote is 2 bytes (0xff33), but has to be set to size 3 !!!
        writeByte(REG_SYNCCONFIG, RF_SYNCCONFIG_PREAMBLEPOLARITY_55 | RF_SYNCCONFIG_SYNC_ON | RF_SYNCCONFIG_SYNCSIZE_3);
        writeByte(REG_SYNCVALUE1, SYNC_BYTE_1);
        writeByte(REG_SYNCVALUE2, SYNC_BYTE_2);

        // Mapping of pins DIO0 to DIO3
        // DIO0: CRCok    DIO1: FIFO empty    DIO2: Sync   | DIO3: TxReady
        // Mapping of pins DIO4 and DIO5
        // DIO4: PreambleDetect  DIO5: Data
        writeByte(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 | RF_DIOMAPPING1_DIO1_01 | RF_DIOMAPPING1_DIO2_11 | RF_DIOMAPPING1_DIO3_01);
        writeByte(REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_11 | RF_DIOMAPPING2_DIO5_10 | RF_DIOMAPPING2_MAP_PREAMBLEDETECT);

        // Enable Fast Hoping (frequency change)
        writeByte(REG_PLLHOP, readByte(RF_PLLHOP_FASTHOP_ON) | RF_PLLHOP_FASTHOP_ON);

        // RSSI precision +-2dBm
        writeByte(REG_RSSICONFIG, RF_RSSICONFIG_SMOOTHING_32);

        writeByte(REG_RXCONFIG, RF_RXCONFIG_AFCAUTO_ON | RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT); // Activates Timeout interrupt on Preamble
        writeByte(REG_AFCBW, RF_AFCBW_MANTAFC_16 | RF_AFCBW_EXPAFC_1);  // 250KHz BW with AFC

        writeByte(REG_PREAMBLEDETECT, RF_PREAMBLEDETECT_DETECTOR_ON | RF_PREAMBLEDETECT_DETECTORSIZE_3 | RF_PREAMBLEDETECT_DETECTORTOL_10);
//        writeByte(REG_PREAMBLEDETECT, RF_PREAMBLEDETECT_DETECTOR_ON | RF_PREAMBLEDETECT_DETECTORSIZE_3 | RF_PREAMBLEDETECT_DETECTORTOL_10);

    /*
    Sequencer configuration: currently unused
        IdleMode:			0:	Standby mode
        FromStart:			00:	to LowPowerSelection
        LowPowerSelection:	1:	Idle
        FromIdle:			1:	to Receive state
        FromReceive:			011:	to PacketReceived state on CrcOk interrupt ... not in test
            FromReceive:		110:	to SequencerOff state on a PreambleDetect interrupt ... test phase
        FromRxTimeout:		10:	to LowPowerSelection
        FromPacketReceived:	010:	to LowPowerSelection
    */
        _params.seqConf[0] = RF_SEQCONFIG1_SEQUENCER_STOP | RF_SEQCONFIG1_IDLEMODE_STANDBY | RF_SEQCONFIG1_FROMSTART_TOLPS | RF_SEQCONFIG1_LPS_IDLE | RF_SEQCONFIG1_FROMIDLE_TORX;
        _params.seqConf[1] = RF_SEQCONFIG2_FROMRX_TOSEQUENCEROFF_ONPREAMBLE | RF_SEQCONFIG2_FROMRXTIMEOUT_TOLPS;
        writeBytes(REG_SEQCONFIG1, _params.seqConf, 2);

        writeByte(REG_TIMERRESOL, RF_TIMERRESOL_TIMER1RESOL_000064_US | RF_TIMERRESOL_TIMER2RESOL_000064_US);
        writeByte(REG_TIMER1COEF, 0x19);    // 1,6mS
        writeByte(REG_TIMER2COEF, 0x13);    // 1,2mS

/*
        _params.seqConf[0] |= RF_SEQCONFIG1_SEQUENCER_START;
        writeByte(REG_SEQCONFIG1, _params.seqConf[0]);
*/
/*
        _params.rfOpMode &= RF_OPMODE_MASK;
        _params.rfOpMode |= RF_OPMODE_RECEIVER;
        writeByte(REG_OPMODE, _params.rfOpMode);

        if (!(_params.seqConf[0] & RF_SEQCONFIG1_SEQUENCER_START))  // Check if Sequencer is not in use, then wait for Rx ready
            RxReady;    
*/
    }

    void setStandby(void)
    {
        _params.rfOpMode = (_params.rfOpMode & RF_OPMODE_MASK) | RF_OPMODE_STANDBY;
        writeByte(REG_OPMODE, _params.rfOpMode);
    }

    void setTx(void)
    {
        _params.rfOpMode &= RF_OPMODE_MASK;
        _params.rfOpMode |= RF_OPMODE_TRANSMITTER;
        writeByte(REG_OPMODE, _params.rfOpMode);

        TxReady;
    }

    void setRx(void)
    {
        _params.rfOpMode &= RF_OPMODE_MASK;
        _params.rfOpMode |= RF_OPMODE_RECEIVER;
        writeByte(REG_OPMODE, _params.rfOpMode);
    }

    void clearBuffer(void)
    {
        for (uint8_t idx=0; idx <= 64; ++idx)   // Clears FIFO at startup to avoid dirty reads
            readByte(REG_FIFO);
    }

    void clearFlags(void)
    {
        uint8_t out[2] = {0xff, 0xff};
        writeBytes(REG_IRQFLAGS1, out, 2);
    }

    bool preambleDetected(void)
    {
        if (readByte(REG_IRQFLAGS1) & RF_IRQFLAGS1_PREAMBLEDETECT)
            return true;

        return false;
    }

    bool syncAddress(void)
    {
        return (readByte(REG_IRQFLAGS1) & RF_IRQFLAGS1_SYNCADDRESSMATCH);
    }

    bool dataAvail(void)
    {
        return ((readByte(REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFOEMPTY)?false:true);
    }

    uint8_t readByte(uint8_t regAddr)
    {
        uint8_t getByte;
        readBytes(regAddr, &getByte, 1);

        return (getByte);
    }

    void readBytes(uint8_t regAddr, uint8_t *out, uint8_t len)
    {
        SPI_beginTransaction();
        SPI.transfer(regAddr);                  // Send Address
        for (uint8_t idx=0; idx < len; ++idx)
            out[idx] = SPI.transfer(regAddr);   // Get data
        SPI_endTransaction();

        return;
    }

    bool writeByte(uint8_t regAddr, uint8_t data, bool check)
    {
        return writeBytes(regAddr, &data, 1, check);
    }

    bool writeBytes(uint8_t regAddr, uint8_t *in, uint8_t len, bool check)
    {
        SPI_beginTransaction();
        SPI.write(regAddr | SPI_Write);      // Send Address with Write flag
        for (uint8_t idx=0; idx < len; ++idx)
            SPI.write(in[idx]);              // Send data
        SPI_endTransaction();

        if (check)
        {
            uint8_t getByte;

            SPI_beginTransaction();
            SPI.transfer(regAddr);                  // Send Address
            for (uint8_t idx=0; idx < len; ++idx)
            {
                getByte = SPI.transfer(regAddr);    // Get data
                if (in[idx] != getByte)
                {
                    SPI_endTransaction();
                    return false;
                }
            }
            SPI_endTransaction();
        }

        return true;
    }

    bool inStdbyOrSleep(void)
    {
        uint8_t data;
        
        data = readByte(REG_OPMODE);
        _params.rfOpMode = data;
        data &= ~RF_OPMODE_MASK;
        if ((data == RF_OPMODE_SLEEP) || (data == RF_OPMODE_STANDBY))
            return true;

        return false;
    }

    bool setCarrier(Carrier param, uint32_t value)
    {
        uint32_t tmpVal;
        uint8_t out[4];
        regBandWidth bw;

//  Change of Frequency can be done while the redaio is working thanks to Freq Hopping
        if (!inStdbyOrSleep())
            if (param != Carrier::Frequency)
                return false;

        switch (param)
        {
            case Carrier::Frequency:
                _params.carrierFrequency = value;
                tmpVal = (uint32_t)(((float_t)value/FXOSC)*(1<<19));
//                Serial.printf("Setting freq: %6x(%u)\n", tmpVal, tmpVal);
                out[0] = (tmpVal & 0x00ff0000) >> 16;
                out[1] = (tmpVal & 0x0000ff00) >> 8;
                out[2] = (tmpVal & 0x000000ff); // If Radio is active writing LSB triggers frequency change
                writeBytes(REG_FRFMSB, out, 3);
                break;
            case Carrier::Bandwidth:
                bw = bwRegs(value);
                writeByte(REG_RXBW, bw.Mant | bw.Exp);
                writeByte(REG_AFCBW, bw.Mant | bw.Exp);
                break;
            case Carrier::Deviation:
                _params.deviation = value;
                tmpVal = (uint32_t)(((float_t)value/FXOSC)*(1<<19));
                Serial.printf("Setting deviation: %4x(%u)\n", tmpVal, tmpVal);
                out[0] = (tmpVal & 0x0000ff00) >> 8;
                out[1] = (tmpVal & 0x000000ff);
                writeBytes(REG_FDEVMSB, out, 2);
                break;
            case Carrier::Modulation:
                switch (value)
                {
                    case Modulation::FSK:
                        _params.rfOpMode &= RF_OPMODE_LONGRANGEMODE_MASK;
                        _params.rfOpMode |= RF_OPMODE_LONGRANGEMODE_OFF;
                        _params.rfOpMode &= RF_OPMODE_MODULATIONTYPE_MASK;
                        _params.rfOpMode |= RF_OPMODE_MODULATIONTYPE_FSK;
                        _params.rfOpMode &= RF_OPMODE_MASK;
                        _params.rfOpMode |= RF_OPMODE_STANDBY;
                        if (_params.carrierFrequency > LOWER)
                            _params.rfOpMode &= ~0x08;
                        else
                            _params.rfOpMode |= 0x08;
                        writeByte(REG_OPMODE, _params.rfOpMode);
                        break;
                }
                break;
            case Carrier::Bitrate:
                _params.bitRate = value;
                tmpVal = FXOSC/value;
                Serial.printf("Setting bitrate: %4x(%u)\n", tmpVal, tmpVal);
                out[0] = (tmpVal & 0x0000ff00) >> 8;
                out[1] = (tmpVal & 0x000000ff);
                writeBytes(REG_BITRATEMSB, out, 2);
                break;
        }

        return true;
    }

    regBandWidth bwRegs(uint8_t bandwidth)
    {
        for (auto it = __bw.begin(); it != __bw.end(); it++)
            if (it->first == bandwidth)
                return it->second;

        return __bw.rbegin()->second;
    }


    void dump()
    {
        uint8_t idx = 1;
        do       
        {
            Serial.printf("*%2.2x=%2.2x\t", idx, readByte(idx)); idx += 1;
            Serial.printf("*%2.2x=%2.2x\t", idx, readByte(idx)); idx += 1;
            Serial.printf("*%2.2x=%2.2x\t", idx, readByte(idx)); idx += 1;
            Serial.printf("*%2.2x=%2.2x\t", idx, readByte(idx)); idx += 1;
            Serial.printf("*%2.2x=%2.2x\t", idx, readByte(idx)); idx += 1;
            Serial.printf("*%2.2x=%2.2x\t", idx, readByte(idx)); idx += 1;
            Serial.printf("*%2.2x=%2.2x\t", idx, readByte(idx)); idx += 1;
            Serial.printf("*%2.2x=%2.2x\t", idx, readByte(idx)); idx += 1;
            Serial.printf("\n");
        }
        while (idx < 0x70);
    }
}
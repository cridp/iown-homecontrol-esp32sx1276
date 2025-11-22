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

#include <Arduino.h>

#include <SX1276Helpers.h>
#include <board-config.h>

#if defined(RADIO_SX127X)
#include <map>

#if defined(ESP32)
#define CONFIG_DISABLE_HAL_LOCKS true
#include <SPI.h>
#include <TickerUsESP32.h>
#include <esp_task_wdt.h>
// #include <SPIeX.h>
#endif

namespace Radio {
    SPISettings SpiSettings(8000000, MSBFIRST, SPI_MODE0);

    // Simplified bandwidth registries evaluation
    std::map<uint8_t, regBandWidth> __bw =
    {
        {25, {0x01, 0x04}}, // 25KHz
        {50, {0x01, 0x03}},
        {100, {0x01, 0x02}},
        {125, {0x00, 0x02}},
        {200, {0x01, 0x01}},
        {250, {0x00, 0x01}} // 250KHz
    };

    // digitalWrite() prend entre 1.3 et 4.2 µs selon contention.
#define NSS_LOW  (GPIO.out_w1tc = (1<<RADIO_NSS))
#define NSS_HIGH (GPIO.out_w1ts = (1<<RADIO_NSS))

    // spinlock/mutex pour SPI (ISR-safe usage: on n'appelle pas le mutex en ISR)
    static portMUX_TYPE spiMux = portMUX_INITIALIZER_UNLOCKED;

    void IRAM_ATTR spi_select() {
        portENTER_CRITICAL(&spiMux);
        NSS_LOW; //GPIO.out_w1tc = (1 << RADIO_NSS);
    }

    void IRAM_ATTR spi_deselect() {
        NSS_HIGH; //GPIO.out_w1ts = (1 << RADIO_NSS);
        portEXIT_CRITICAL(&spiMux);
    }

/**
 * The function `SPI_beginTransaction` begins a SPI transaction and sets the RADIO_NSS pin to LOW.
 */
    // void IRAM_ATTR SPI_beginTransaction() {
    //     SPI.beginTransaction(Radio::SpiSettings);
    //     digitalWrite(RADIO_NSS, LOW);
    // }

/**
 * The function `SPI_endTransaction` ends the SPI transaction and sets the RADIO_NSS pin to HIGH.
 */
    // void IRAM_ATTR SPI_endTransaction() {
    //     digitalWrite(RADIO_NSS, HIGH);
    //     SPI.endTransaction();
    // }

/**
 * The function `initHardware` initializes the hardware for SPI communication with a radio chip, checks
 * the availability of the radio, configures SPI settings, and puts the radio chip in standby mode.
 */
    void initHardware() {
        printf("\nSPI Init");

        //gpio_pullup_en((gpio_num_t) RADIO_MISO);
        pinMode(RADIO_MISO, INPUT_PULLUP);

        // SPI pins configuration
        pinMode(RADIO_RESET, INPUT); // Connected to Reset; floating for POR

        // Check the availability of the Radio
        while (!digitalRead(RADIO_RESET)) {
#if defined(ESP32)
            esp_task_wdt_reset();
#endif
            ets_delay_us(1);
        }
        ets_delay_us(BOARD_READY_AFTER_POR);

        // Initialize SPI bus
        SPI.setHwCs(true); // Enable hardware CS Before SPI.begin() !!
        SPI.begin(RADIO_SCLK, RADIO_MISO, RADIO_MOSI, RADIO_NSS);

        // SPI.setFrequency(SPI_CLK_FRQ);
        // SPI.setDataMode(SPI_MODE0);
        // SPI.setBitOrder(MSBFIRST);

        // Disable SPI device
        // Disable device NRESET pin
        pinMode(RADIO_NSS, OUTPUT);
        pinMode(RADIO_RESET, OUTPUT);
        digitalWrite(RADIO_RESET, HIGH);
        digitalWrite(RADIO_NSS, HIGH);
        delayMicroseconds(BOARD_READY_AFTER_POR);

        // SPI.beginTransaction(Radio::SpiSettings);
        // SPI.endTransaction();

        writeByte(REG_OPMODE, RF_OPMODE_STANDBY); // Put Radio in Standby mode

        pinMode(SCAN_LED, OUTPUT);
        digitalWrite(SCAN_LED, 1);
        printf("\nRadio Chip is ready\n");
    }

void setPreambleLength(uint16_t preambleLen) {
    writeByte(REG_PREAMBLEMSB, (preambleLen >> 8) & 0xFF);
    writeByte(REG_PREAMBLELSB, preambleLen & 0xFF);
    // ets_printf("Radio: Preamble length set to %u symbols\n", preambleLen);
}

/**
 * The `initRegisters` function initializes various registers of a radio module for both transmission
 * and reception in a C++ program.
 * 
 * @param maxPayloadLength The `maxPayloadLength` parameter in the `initRegisters` function is used to
 * set the maximum payload length for the radio communication. In this function, it is set to a default
 * value of `0xff` (255 in decimal). This parameter is used to configure the radio module to handle
 * packets
 */
    void initRegisters(uint8_t maxPayloadLength = 0xff) {
        // Firstly put radio in StandBy mode as some parameters cannot be changed differently
        writeByte(REG_OPMODE, (readByte(REG_OPMODE) & RF_OPMODE_MASK) | RF_OPMODE_STANDBY);

        // ---------------- Common Register init section ----------------
        // Switch-off clockout
        writeByte(REG_OSC, RF_OSC_CLKOUT_OFF); // This only give power saveing maybe we can use it as ticker µs

        // Variable packet lenght, generates working CRC.
        // Packet mode, IoHomeOn, IoHomePowerFrame to be added (0x10) to avoid rx to newly detect the preamble during tx radio shutdown
        // Must CRCAUTOCLEAR_ON or do full clean FIFO !
        writeByte(
            REG_PACKETCONFIG1,
            RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RF_PACKETCONFIG1_DCFREE_OFF | RF_PACKETCONFIG1_CRC_ON |
            RF_PACKETCONFIG1_CRCAUTOCLEAR_ON | RF_PACKETCONFIG1_CRCWHITENINGTYPE_CCITT |
            RF_PACKETCONFIG1_ADDRSFILTERING_OFF);
        writeByte(
            REG_PACKETCONFIG2,
            RF_PACKETCONFIG2_DATAMODE_PACKET | RF_PACKETCONFIG2_IOHOME_ON | RF_PACKETCONFIG2_IOHOME_POWERFRAME); // | RF_PACKETCONFIG2_WMBUS_CRC_ENABLE);
            // Is IoHomePowerFrame useful ? What that wmbus CRC ? sx127x is able to do wmbus ??

        // Preamble shall be set to AA for packets to be received by appliances. Sync word shall be set with different values if Rx or Tx
        // NOT RF_SYNCCONFIG_AUTORESTARTRXMODE_WAITPLL_ON
        writeByte(
            REG_SYNCCONFIG,
            RF_SYNCCONFIG_AUTORESTARTRXMODE_WAITPLL_OFF | RF_SYNCCONFIG_PREAMBLEPOLARITY_AA | RF_SYNCCONFIG_SYNC_ON);
                //0x51); // 0x91); // TODOVERIFY 0x92

        // Set Sync word to 0xff33 both for rx and tx
        writeByte(REG_SYNCVALUE1, SYNC_BYTE_1);
        writeByte(REG_SYNCVALUE2, SYNC_BYTE_2);

        // Optimiser la configuration des registres
        writeByte(REG_DIOMAPPING1, 0x3D);
            // RF_DIOMAPPING1_DIO0_00 | // PayloadReady sur DIO0
            // RF_DIOMAPPING1_DIO2_11 | // PreambleDetect sur DIO2
            // RF_DIOMAPPING1_DIO1_01 |
            // RF_DIOMAPPING1_DIO3_01);

        writeByte(REG_DIOMAPPING2, 0xF1);
            // RF_DIOMAPPING2_MAP_PREAMBLEDETECT | RF_DIOMAPPING2_MAP_RSSI |
            // RF_DIOMAPPING2_DIO4_11 |
            // RF_DIOMAPPING2_DIO5_10);

        // Enable Fast Hoping (frequency change)
        if (MAX_FREQS != 1)
            writeByte(REG_PLLHOP, readByte(REG_PLLHOP) | RF_PLLHOP_FASTHOP_ON);

        // PA Ramp: No Shaping, Ramp up/down 15us
        writeByte(REG_PARAMP, RF_PARAMP_MODULATIONSHAPING_00 | RF_PARAMP_0015_US); //_0012_US); //_0031_US); //
        // Setting Preamble Length
        writeByte(REG_PREAMBLEMSB, PREAMBLE_MSB);
        writeByte(REG_PREAMBLELSB, 0x34); // PREAMBLE_LSB);

        // FIFO Threshold - currently useless
        writeByte(REG_FIFOTHRESH, 0x88); //0x08); // RF_FIFOTHRESH_TXSTARTCONDITION_FIFONOTEMPTY);

        // ---------------- RX Register init section ----------------
        // Set lenght checking if passed as parameter
        // The use of maxPayloadLength is not working. Prevents generating PayloadReady signal if set
        writeByte(REG_PAYLOADLENGTH, 0xff);
        // RSSI precision +-2dBm
        writeByte(REG_RSSICONFIG, RF_RSSICONFIG_SMOOTHING_4); // 8->0.512 ms // _128); // _32); //_256); //
        // Activates Timeout interrupt on Preamble
        //L'AFC (Automatic Frequency Control) tente de compenser les dérives de fréquence.
        //Pendant un saut FHSS, cela peut être contre-productif car le saut de fréquence
        //peut être interprété comme une dérive massive, amenant l'AFC à "lutter" contre le saut.
        //Désactiver (RF_RXCONFIG_AFCAUTO_OFF). Cela pourrait stabiliser la réception juste après un saut.
        writeByte(REG_RXCONFIG, // AGCAUTO_OFF not functional AFCAUTO_OFF change nothing
            RF_RXCONFIG_AFCAUTO_OFF | RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_RSSI_PREAMBLEDETECT | RF_RXCONFIG_RESTARTRXONCOLLISION_ON | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);

        // 250KHz BW with AFC
        writeByte(REG_AFCBW, 0x10); // RF_AFCBW_MANTAFC_16 | RF_AFCBW_EXPAFC_1); // 333333 Hz
        writeByte(REG_AFCFEI, 0x01); //

        // if AGC_AUTO_ON, RF_LNA_GAIN_XX do nothing
        writeByte(REG_LNA, 0xC3); //RF_LNA_BOOST_ON | RF_LNA_GAIN_G6); //

        // Enables Preamble Detect, 2 bytes
        writeByte(
            REG_PREAMBLEDETECT,
            RF_PREAMBLEDETECT_DETECTOR_ON | RF_PREAMBLEDETECT_DETECTORSIZE_2 | RF_PREAMBLEDETECT_DETECTORTOL_10);

        writeByte(REG_RSSITHRESH, RF_RSSITHRESH_THRESHOLD);

        // PA boost maximum power
        writeByte(REG_PACONFIG, RF_PACONFIG_PASELECT_MASK | RF_PACONFIG_PASELECT_PABOOST);
        writeByte(REG_OCP, RF_OCP_ON | RF_OCP_TRIM_240_MA); // 0x37); //200mA //0x3B 240mA
        writeByte(REG_PADAC, 0x87); //  RF_PADAC_20DBM_MASK | RF_PADAC_20DBM_ON); // turn 20dBm mode on

    }

/**
 * The `calibrate` function in C++ performs radio calibration by adjusting power levels and setting the
 * frequency band.
 */
    void calibrate() {
        // Save context
        uint8_t regPaConfigInitVal = readByte(REG_PACONFIG);

        // Cut the PA just in case, RFO output, power = -1 dBm
        writeByte(REG_PACONFIG, RF_PACONFIG_PASELECT_RFO);
        // RC Calibration (only call after setting correct frequency band)
        writeByte(REG_OSC, RF_OSC_RCCALSTART);
        // Start image and RSSI calibration
        writeByte(
            REG_IMAGECAL, (RF_IMAGECAL_AUTOIMAGECAL_MASK & RF_IMAGECAL_IMAGECAL_MASK) | RF_IMAGECAL_IMAGECAL_START);
        // Wait end of calibration
        while ((readByte(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING) {
        }
        // Set a Frequency in HF band
        Radio::setCarrier(Radio::Carrier::Frequency, CHANNEL2);
        // Start image and RSSI calibration
        writeByte(
            REG_IMAGECAL, (RF_IMAGECAL_AUTOIMAGECAL_MASK & RF_IMAGECAL_IMAGECAL_MASK) | RF_IMAGECAL_IMAGECAL_START);
        // Wait end of calibration
        while ((readByte(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING) {
        }

        // Restore context
        writeByte(REG_PACONFIG, regPaConfigInitVal);
    }

    void IRAM_ATTR setStandby() {
        writeByte(REG_OPMODE, (readByte(REG_OPMODE) & RF_OPMODE_MASK) | RF_OPMODE_STANDBY);
    }

    void IRAM_ATTR setTx() {
        // Enabling Sync word - Size must be set to SYNCSIZE_2
        writeByte(REG_SYNCCONFIG, (readByte(REG_SYNCCONFIG) & RF_SYNCCONFIG_SYNCSIZE_MASK) | RF_SYNCCONFIG_SYNCSIZE_2);
        writeByte(REG_OPMODE, (readByte(REG_OPMODE) & RF_OPMODE_MASK) | RF_OPMODE_TRANSMITTER);
        // TxReady;
    }

    void IRAM_ATTR setRx() {
        // Enabling Sync word - Size must be set to SYNCSIZE_3
        writeByte(REG_SYNCCONFIG, (readByte(REG_SYNCCONFIG) & RF_SYNCCONFIG_SYNCSIZE_MASK) | RF_SYNCCONFIG_SYNCSIZE_3);
        writeByte(REG_OPMODE, (readByte(REG_OPMODE) & RF_OPMODE_MASK) | RF_OPMODE_RECEIVER);
        // RxReady;
        /*
                // Start Sequencer
                writeByte(REG_OPMODE, (readByte(REG_OPMODE) & RF_OPMODE_MASK) | RF_OPMODE_RECEIVER);
                writeByte(REG_SEQCONFIG1, readByte(REG_SEQCONFIG1 | RF_SEQCONFIG1_SEQUENCER_START));
        */
    }


    void readBurst(uint8_t regAddr, uint8_t *buffer, uint8_t size) {
        for (uint8_t i = 0; i < size; ++i) {
            buffer[i] = readByte(regAddr + i);
        }
    }

    // Clears FIFO at startup to avoid dirty reads
    void clearBuffer() {
        // Taille du buffer FIFO du SX1276
        const uint8_t bufferSize = 64;

        // Lire le buffer par paquets de 32 octets
        for (uint8_t i = 0; i < bufferSize; i += 32) {
            uint8_t buffer[32]; // Tableau temporaire pour stocker les octets lus
            readBytes/*Burst*/(REG_FIFO, buffer, sizeof(buffer)); // Lire 32 octets à la fois
        }
    }

    // void clearFlags_A() {
    //   uint8_t flags = readByte(REG_IRQFLAGS1);
    //   flags &= ~0xFF; // Efface tous les drapeaux
    //   writeByte(REG_IRQFLAGS1, flags);
    // }
        void IRAM_ATTR clearFlags() {
            uint8_t out[2] = {0xff, 0xff};
            writeBurst(REG_IRQFLAGS1, out, 2);
        }
    // void IRAM_ATTR clearFlags() {
    //     uint16_t flags = readWord(REG_IRQFLAGS1);
    //     flags &= ~0xFFFF; // Efface tous les drapeaux
    //     writeWord(REG_IRQFLAGS1, flags);
    // }

    bool IRAM_ATTR preambleDetected() {
        return readByte(REG_IRQFLAGS1) & RF_IRQFLAGS1_PREAMBLEDETECT;
    }

    bool IRAM_ATTR syncedAddress() {
        return readByte(REG_IRQFLAGS1) & RF_IRQFLAGS1_SYNCADDRESSMATCH;
    }

    bool IRAM_ATTR dataAvail() {
        return (readByte(REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFOEMPTY) == 0; //?false:true;
    }

    uint8_t IRAM_ATTR readByte(uint8_t regAddr) {
        uint8_t getByte;
        readBytes(regAddr, &getByte, 1);
        return getByte;
    }

    void IRAM_ATTR readBytes(uint8_t regAddr, uint8_t *out, uint8_t len) {
        spi_select(); //NSS_LOW; //SPI_beginTransaction();
        SPI.transfer(regAddr); // Send Address
        for (uint8_t idx = 0; idx < len; ++idx) {
            out[idx] = SPI.transfer(regAddr); // Get data
        }
        spi_deselect(); //NSS_HIGH; //SPI_endTransaction();
    }

    void IRAM_ATTR writeByte(uint8_t regAddr, uint8_t data) {//, bool check) {
        // return writeBurst(regAddr, &data, 1);//, check);
        spi_select(); //NSS_LOW;
        SPI.write(regAddr | 0x80);
        // ets_delay_us(SPI_RW_DELAY);
        SPI.write/*transfer*/(data);
        spi_deselect(); //NSS_HIGH;
    }

    void IRAM_ATTR writeBurst(uint8_t addr, uint8_t *data, uint8_t len) {
        spi_select(); //NSS_LOW;
        SPI.write(addr | 0x80);
        SPI.transferBytes((uint8_t*)data, nullptr, len);
        spi_deselect(); //NSS_HIGH;
    }

    uint16_t IRAM_ATTR readWord(uint8_t regAddr) {
        uint8_t lowByte = readByte(regAddr);
        uint8_t highByte = readByte(regAddr + 1);
        return (highByte << 8) | lowByte;
    }

    void IRAM_ATTR writeWord(uint8_t regAddr, uint16_t value) {
        writeByte(regAddr, value >> 8);
        writeByte(regAddr + 1, value & 0xFF);
    }

    bool IRAM_ATTR inStdbyOrSleep() {
        uint8_t data = readByte(REG_OPMODE);
        data &= ~RF_OPMODE_MASK;
        if ((data == RF_OPMODE_SLEEP) || (data == RF_OPMODE_STANDBY))
            return true;

        return false;
    }
    uint8_t /*IRAM_ATTR*/ out[4] __attribute__((aligned(4))) = {0x00, 0x00, 0x00, 0x00};
    bool IRAM_ATTR setCarrier(Carrier param, uint32_t value) {
        uint32_t tmpVal;
        // uint8_t out[4];
        regBandWidth bw{};

        //  Change of Frequency can be done while the radio is working thanks to Freq Hopping
        if (!inStdbyOrSleep())
            if (param != Carrier::Frequency)
                return false;

        switch (param) {
            case Carrier::Frequency:
                /*uint32_t FRF = (newFreq * (uint32_t(1) << RADIOLIB_SX127X_DIV_EXPONENT)) / RADIOLIB_SX127X_CRYSTAL_FREQ;*/
                tmpVal = static_cast<uint32_t>((static_cast<float_t>(value) / FXOSC) * (1 << 19));
                out[0] = (tmpVal & 0x00ff0000) >> 16;
                out[1] = (tmpVal & 0x0000ff00) >> 8;
                out[2] = (tmpVal & 0x000000ff); // If Radio is active, writing LSB triggers frequency change
                writeBurst(REG_FRFMSB, out, 3);
                break;
            case Carrier::Bandwidth:
                bw = bwRegs(value);
                writeByte(REG_RXBW, bw.Mant | bw.Exp);
                writeByte(REG_AFCBW, bw.Mant | bw.Exp);
                break;
            case Carrier::Deviation:
                tmpVal = static_cast<uint32_t>((static_cast<float_t>(value) / FXOSC) * (1 << 19));
                out[0] = (tmpVal & 0x0000ff00) >> 8;
                out[1] = (tmpVal & 0x000000ff);
                writeBurst(REG_FDEVMSB, out, 2);
            //                writeByte(REG_BITRATEFRAC, 5); // Little more precision
                break;
            case Carrier::Modulation:
                switch (value) {
                    case Modulation::FSK: {
                        uint8_t rfOpMode = readByte(REG_OPMODE);
                        rfOpMode &= RF_OPMODE_LONGRANGEMODE_MASK;
                        rfOpMode |= RF_OPMODE_LONGRANGEMODE_OFF;
                        rfOpMode &= RF_OPMODE_MODULATIONTYPE_MASK;
                        rfOpMode |= RF_OPMODE_MODULATIONTYPE_FSK;
                        rfOpMode &= RF_OPMODE_MASK;
                        rfOpMode |= RF_OPMODE_STANDBY;
                        rfOpMode &= ~0x08;
                        writeByte(REG_OPMODE, rfOpMode);
                        break;
                    }
                    case Modulation::LoRa:
                    case Modulation::OOK:
                    default: break;
                }
                break;
            case Carrier::Bitrate:
                tmpVal = FXOSC / value;
                out[0] = (tmpVal & 0x0000ff00) >> 8;
                out[1] = (tmpVal & 0x000000ff);
                writeBurst(REG_BITRATEMSB, out, 2);
                break;
        }

        return true;
    }

    regBandWidth bwRegs(uint8_t bandwidth) {
        for (auto &it: __bw)
            if (it.first == bandwidth)
                return it.second;

        return __bw.rbegin()->second;
    }

    int32_t getFrequencyError() {
        uint16_t fei = (readByte(REG_FEIMSB) << 8) | readByte(REG_FEILSB);
        if (fei & 0x8000) { // Negative value
            // frequency error is negative
            fei |= (uint32_t) 0xFFF00000;
            fei = (~fei + 1); // Two's complement
            return -((static_cast<float_t>(fei) * FXOSC) / (1 << 19));
        } else {
            return ((static_cast<float_t>(fei) * FXOSC) / (1 << 19));
        }
    }
    int16_t getAFCError() {
        // get raw frequency error
        int16_t raw = (uint16_t) readByte(REG_AFCMSB) << 8;
        raw |= readByte(REG_AFCLSB);
        return raw * (FXOSC / (1 << 19));
    }
    void dump() {
        uint8_t idx = 0;

        Serial.printf("#Type\tRegister Name\tAddress[Hex]\tValue[Hex]\n");
        do {
            Serial.printf("REG\tname\t0x%2.2x\t0x%2.2x\n", idx, readByte(idx));
            idx += 1;
        } while (idx < 0x7f);
        Serial.printf("PKT\tFalse;False;255;0;\nXTAL\t32000000\n");
        // Serial.printf("\n");
        dumpReal();
    }

    void dumpReal() {
        uint8_t registers[0x80];
        registers[0] = 0x00;
        readBytes(0x01, registers + 1, 0x7F);
        // sx127x_dump_registers(registers, device);
        for (int idx = 0; idx < sizeof(registers); idx++) {
            if (idx != 0) {
                printf(",");
            }
            printf("0x%2.2x", registers[idx]);
        }
        printf("\n");

        dump_fsk_registers(registers);
    }
}
#endif

#pragma once

#include <sx1276Regs-Fsk.h>
#include <board-config.h>
#include <SPI.h>


#define KHz     *1000
#define MHz     KHz *1000
#define FXOSC   32000000
#define LOWER   525000000
#define HIGHER  779000000

#define SPI_Write   0x80
#define SPI_Read    0x00


#define TxReady  {do {/* Checks new Mode is ready */} while (!(readByte(REG_IRQFLAGS1) & RF_IRQFLAGS1_TXREADY));}   // Check for TxReady flag
#define RxReady  {do {/* Checks new Mode is ready */} while (!(readByte(REG_IRQFLAGS1) & RF_IRQFLAGS1_PLLLOCK));}   // Check for PllLock flag; do not use with sequencer

#define RF_PACKETCONFIG2_IOHOME_POWERFRAME  0x10    // Missing from SX1276 FSK modem registers and bits definitions



/*
    Helper functions to setup and manage SX1276 registry configuration, query status and SPI interaction
*/
namespace Radio
{
    enum class Carrier {
        Frequency,
        Deviation,
        Bandwidth,
        Bitrate,
        Modulation
    };

    enum Modulation:uint32_t {
        OOK = 0x00,
        FSK,
        LoRa
    };

    typedef struct  
    {
        uint32_t    carrierFrequency;
        uint8_t     rfOpMode;
        uint32_t    bitRate;
        uint32_t    deviation;
        uint8_t     seqConf[2];
    } WorkingParams;

    typedef struct
    {
        uint8_t     Mant;
        uint8_t     Exp;
    } regBandWidth;



    void initHardware(void);
    void initRegisters(uint8_t maxPayloadLength);
    void calibrate(void);
    void setStandby(void);
    void setTx(void);
    void setRx(void);
    void clearBuffer(void);
    void clearFlags(void);
    bool preambleDetected(void);
    bool syncedAddress(void);
    bool dataAvail(void);
    bool crcOk(void);
    uint8_t readByte(uint8_t regAddr);
    void readBytes(uint8_t regAddr, uint8_t *out, uint8_t len);
    bool writeByte(uint8_t regAddr, uint8_t data, bool check = NULL);
    bool writeBytes(uint8_t regAddr, uint8_t *in, uint8_t len, bool check = NULL);
    bool inStdbyOrSleep(void);
    bool setParams();
    bool setCarrier(Carrier param, uint32_t value);
    regBandWidth bwRegs(uint8_t bandwidth);
    void dump(void);
}
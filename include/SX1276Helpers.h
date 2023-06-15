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

// Maximum payload in IOHC is 32 bytes: 1 Length byte + 31 body bytes
#define MAX_FRAME_LEN   32

#define TxReady  {do {/* Checks new Mode is ready */} while (!(readByte(REG_IRQFLAGS1) & RF_IRQFLAGS1_TXREADY));}   // Check for TxReady flag
#define RxReady  {do {/* Checks new Mode is ready */} while (!(readByte(REG_IRQFLAGS1) & RF_IRQFLAGS1_PLLLOCK));}   // Check for PllLock flag; do not use with sequencer


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


// To be checked
    struct _control
    {
        uint16_t relationship:2;
        uint16_t __filler:1;
        uint16_t framelength:5;
        uint16_t byte2:8;
    };

    struct _nodeId
    {
        _control    __filler;
        uint32_t    source:24;
        uint32_t    target:24;
    };

    typedef union 
    {
        char        buffer[MAX_FRAME_LEN];
        _control    control;
        _nodeId     nodeid;
    } payload;
// To be checked
    

    extern bool dataAvail;
    extern bool packetEnd;
    extern bool iAmAReceiver;
    extern uint8_t bufferIndex;


    void init(void);
    void initTx(void);
    void initRx(void);
    void setStandby(void);
    void setTx(void);
    void setRx(void);
    void clearFlags(void);
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
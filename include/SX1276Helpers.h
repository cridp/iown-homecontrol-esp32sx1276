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


// Select a Timer Clock for ISR 
#define USING_TIM_DIV1                false           // for shortest and most accurate timer
#define USING_TIM_DIV16               false           // for medium time and medium accurate timer
#define USING_TIM_DIV256              true            // for longest timer but least accurate. Default
#define SCAN_INTERVAL_US                3500
#define SM_GRANULARITY_US               100


#define TxReady  {do {/* Checks new Mode is ready */} while (!(readByte(REG_IRQFLAGS1) & RF_IRQFLAGS1_TXREADY));}   // Check for TxReady flag
#define RxReady  {do {/* Checks new Mode is ready */} while (!(readByte(REG_IRQFLAGS1) & RF_IRQFLAGS1_PLLLOCK));}   // Check for PllLock flag; do not use with sequencer

#define RF_PACKETCONFIG2_IOHOME_POWERFRAME  0x10    // Missing from SX1276 FSK modem registers and bits definitions

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


    // State machine

        enum class States {
        Rst = 0,
        PrD = 1,
        SyD = 2,
        PaR = 3
    };

    typedef struct 
    {
        bool            scanFreqs;
        uint32_t        maxStayUs;
        bool            (*checkNextState)(void);
        States          nextStateOk;
        States          nextStateTimeout;
    } stateMachineTransitions;

    typedef struct
    {
        States          status;
        unsigned long   enteredTime;
    } stateMachineStatus;


// To be checked
//
// Packet format
//
// b1 - b2: Frame length + Control 
// b3 - b5: Target nodeId
// b6 - b8: Source nodeId
// b9 - b11: ???
// b12: Command
// b13 - b16: ???
// b17: Sequence
// b18: ???


    struct _header
    {
        unsigned char   framelength:5;
        unsigned char   mode:1;
        unsigned char   first:1;
        unsigned char   last:1;
        unsigned char   prot_v:2;
        unsigned char   _unq1:1;
        unsigned char   _unq2:1;
        unsigned char   ack:1;
        unsigned char   low_p:1;
        unsigned char   routed:1;
        unsigned char   use_beacon:1;
    };

    struct _nodeId
    {
        _header         __filler;
        unsigned char   target[3];
        unsigned char   source[3];
    };

    struct _message
    {
        _nodeId         __filler;
        unsigned char   cmd;
        unsigned char   _msg1[3];
        unsigned char   _msg2[4];
        unsigned char   sequence;
        unsigned char   msg[MAX_FRAME_LEN-8-8-1];
    };


    union _payload
    {
        char        buffer[MAX_FRAME_LEN];
        _header     control;
        _nodeId     nodeid;
        _message    message;
    };
// To be checked
    

    extern volatile bool dataAvail;
    extern volatile bool packetEnd;
    extern bool iAmAReceiver;
    extern uint8_t bufferIndex;
    extern union _payload payload;


    void init(void);
    void initTx(void);
    void initRx(void);
    void setStandby(void);
    void setTx(void);
    void setRx(void);
    void clearFlags(void);
    bool preambleDetected(void);
    bool syncAddress(void);
    bool crcOk(void);
    uint8_t readByte(uint8_t regAddr);
    void readBytes(uint8_t regAddr, uint8_t *out, uint8_t len);
    bool writeByte(uint8_t regAddr, uint8_t data, bool check = NULL);
    bool writeBytes(uint8_t regAddr, uint8_t *in, uint8_t len, bool check = NULL);
    bool inStdbyOrSleep(void);
    bool setParams();
    bool setCarrier(Carrier param, uint32_t value);
    void stateMachine(void);

    regBandWidth bwRegs(uint8_t bandwidth);
    void dump(void);
}
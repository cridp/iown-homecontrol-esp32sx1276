#pragma once

//#define SX1276
#define CC1101


/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */

#define BOARD_TCXO_WAKEUP_TIME                      0
#define BOARD_READY_AFTER_POR						10000


/*!
 * Board MCU pins definitions - NodeMCUv2
 */
#if defined(ESP8266)
#define RADIO_RESET                                 4   // NodeMCU D2

#define RADIO_MOSI                                  13  // NodeMCU D7
#define RADIO_MISO                                  12  // NodeMCU D6
#define RADIO_SCLK                                  14  // NodeMCU D5
#define RADIO_NSS                                   15  // NodeMCU D8
#elif defined(ESP32)
    #define RADIO_MOSI                              23  // Default VSPI
    #define RADIO_MISO                              19  // Default VSPI
    #define RADIO_SCLK                              18  // Default VSPI
    #if defined(SX1276)
        #define RADIO_RESET                         12
        #define RADIO_NSS                           25
    #elif defined(CC1101)
        #define RADIO_NSS                           32
    #endif
#endif


#if defined(SX1276)
    //#define RADIO_DIO_0                             5   // NodeMCU D1
    //#define RADIO_DIO_1                             2   // NodeMCU D4 // Not used - No wire
    //#define RADIO_DIO_2                             2   // NodeMCU D4 // Not used - No wire
    //#define RADIO_DIO_4                             2   // NodeMCU D4
    #define RADIO_DIO_0                             35
    //#define RADIO_DIO_1                             34      // Not used - No wire
    //#define RADIO_DIO_2                             34      // Not used - No wire
    #define RADIO_DIO_4                             34
#elif defined(CC1101)
    #define RADIO_DIO_0                             14
    #define RADIO_DIO_2                             33
#endif


#if defined(SX1276)
    #define RADIO_PACKET_AVAIL                      RADIO_DIO_0     // Packet Received / CRC ok from Radio
    #define RADIO_DATA_AVAIL                        RADIO_DIO_1     // FIFO empty from Radio
    #define RADIO_RXTIMEOUT                         RADIO_DIO_2     // Radio Rx Sequencer timeout (used to switch the receiver frequancy)
    #define RADIO_PREAMBLE_DETECTED                 RADIO_DIO_4     // Preamble detected from Radio (used instead of FIFO empty)
#elif defined(CC1101)
    #define RADIO_PREAMBLE_DETECTED                 RADIO_DIO_0     // Preamble detected from Radio (used instead of FIFO empty)
    //#define RADIO_RXCLK                             RADIO_DIO_0     // Radio Rx CLK for serial data sync
    // GDO Serial Clk (0x0B)
    // GD2 
#endif


#define SPI_CLK_FRQ                                 10000000


#define PREAMBLE_MSB                                0x00
#define PREAMBLE_LSB                                0x34    // 0x34: 12ms to have receiver up and running (52 0x55 bytes - 13,54mS)
#define SYNC_BYTE_1                                 0xff
#define SYNC_BYTE_2                                 0x33    // Sync word - Size must be set to 2; fist byte 0xff then 0x33 size-1 times
#define SYNC_BYTE_2_ENC                             0xB3    // Sync word Inverted + Enconded with start & stop bits

//#define MAX_FREQS                                   3       // Number of Frequencies to scan through Fast Hopping
//#define FREQS2SCAN                                  {868250000, 868950000, 869850000}
#define MAX_FREQS                                   1       // Number of Frequencies to scan through Fast Hopping
#define FREQS2SCAN                                  {868950000}

#if defined(ESP8266)
    #define SCAN_LED                                D0
#elif defined(ESP32)
    #define SCAN_LED                                22
    #define LED_BUILTIN                             21
#endif
#define RX_LED                                      LED_BUILTIN
#pragma once

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */

#define BOARD_TCXO_WAKEUP_TIME                      0
#define BOARD_READY_AFTER_POR						10000


/*!
 * Board MCU pins definitions - NodeMCUv2
 */
#define RADIO_RESET                                 4   // NodeMCU D2

#define RADIO_MOSI                                  13  // NodeMCU D7
#define RADIO_MISO                                  12  // NodeMCU D6
#define RADIO_SCLK                                  14  // NodeMCU D5
#define RADIO_NSS                                   15  // NodeMCU D8

#define RADIO_DIO_1                                 2   // NodeMCU D4
#define RADIO_DIO_0                                 5   // NodeMCU D1
#define RADIO_DATA_AVAIL                            RADIO_DIO_1
#define RADIO_PACKET_AVAIL                          RADIO_DIO_0
